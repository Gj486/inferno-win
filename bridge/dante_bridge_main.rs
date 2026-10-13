#![windows_subsystem = "windows"]
//! DanteBridge v0.3 - graphical version.
//!   Dante -> Windows  (receive from Dante, play to Windows output devices)
//!   Windows -> Dante  (record from Windows input devices, transmit to Dante)
//! Settings are edited in the window and saved to dante_bridge.toml.
//!
//! NOTE: the media clock here is the local PC clock (no PTP lock to Dante).
//! v0.4: optional ASIO mode - audio goes through the "Dante Bridge ASIO" driver
//! (4 in / 4 out) instead of VB-Matrix, via shared memory (shm.rs).
//!
//! v0.5: `usrvclock` now listens to PTPv1 Sync messages from the Dante clock
//! Leader (e.g. Yamaha QL5) on 224.0.1.129:319 and aligns Inferno's media
//! clock with the network clock. Without this, RX/TX meters stay at -90 dB.

mod shm;

use std::collections::{BTreeMap, VecDeque};
use std::net::Ipv4Addr;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, RwLock};
use std::time::{Duration, Instant};

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use eframe::egui;
use inferno_aoip::device_server::{
    AtomicSample, Clock, DeviceServer, ExternalBufferParameters, MediaClock, Sample, Settings,
};
use serde::{Deserialize, Serialize};

const RATE: u32 = 48000;
const RING: usize = 65536;
const TX_TOL: isize = 240;
const TX_RESYNC: isize = 4800;

/// How long to wait for the first PTP Sync before giving up.
/// Kept generous because a Gigabit switch may need ~10 s to flood the
/// multicast group after the application starts.
const MEDIA_CLOCK_TIMEOUT_S: u64 = 15;

/// Human-readable hint appended to every "media clock not available" error.
/// Appears in three places: initial `mc0` (shared start time), the WASAPI TX
/// per-device clock, and the ASIO TX clock. All three must see the PTP
/// overlay before the engine can start transmitting.
const MEDIA_CLOCK_HINT: &str =
    "media clock not available (no PTP Sync within 15 s). \
     Check: (1) UDP port 319 open in Windows Firewall, \
     (2) QL5 is Leader in Dante Controller, \
     (3) the PC is on the same subnet as QL5. \
     See dante_bridge.log for [usrvclock] lines.";

// ---------------------------------------------------------------- config

#[derive(Serialize, Deserialize, Clone)]
#[serde(default)]
struct Config {
    device_name: String,
    bind_ip: String,
    alt_port: u16,
    channels_per_device: usize,
    rx_devices: Vec<String>,
    tx_devices: Vec<String>,
    buffer_ms: usize,
    tx_latency_ms: u32,
    log_level: String,
    use_asio: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            device_name: "DanteBridge".into(),
            bind_ip: String::new(),
            alt_port: 14400,
            channels_per_device: 2,
            rx_devices: vec!["vbmatrix in 1".into(), "vbmatrix in 2".into()],
            tx_devices: vec![],
            buffer_ms: 100,
            tx_latency_ms: 40,
            log_level: "info".into(),
            use_asio: false,
        }
    }
}

fn exe_dir() -> PathBuf {
    std::env::current_exe()
        .ok()
        .and_then(|p| p.parent().map(|d| d.to_path_buf()))
        .unwrap_or_else(|| PathBuf::from("."))
}

fn config_path() -> PathBuf {
    exe_dir().join("dante_bridge.toml")
}

fn save_config(cfg: &Config) -> Result<(), String> {
    let text = toml::to_string_pretty(cfg).map_err(|e| e.to_string())?;
    std::fs::write(config_path(), text).map_err(|e| e.to_string())
}

// ---------------------------------------------------------------- shared state

struct DevStat {
    buf_frames: AtomicU64,
    under: AtomicU64,
    over: AtomicU64,
}

struct Stats {
    rx_frames: AtomicU64,
    tx_frames: AtomicU64,
    tx_corrections: AtomicU64,
    tx_resyncs: AtomicU64,
    rx_peaks: Vec<AtomicU32>,
    tx_peaks: Vec<AtomicU32>,
    rx_dev: Vec<DevStat>,
    asio_host: AtomicBool,
}

impl Stats {
    fn new(rx_ch: usize, tx_ch: usize, rx_devs: usize) -> Self {
        Self {
            rx_frames: AtomicU64::new(0),
            tx_frames: AtomicU64::new(0),
            tx_corrections: AtomicU64::new(0),
            tx_resyncs: AtomicU64::new(0),
            rx_peaks: (0..rx_ch).map(|_| AtomicU32::new(0)).collect(),
            tx_peaks: (0..tx_ch).map(|_| AtomicU32::new(0)).collect(),
            asio_host: AtomicBool::new(false),
            rx_dev: (0..rx_devs)
                .map(|_| DevStat {
                    buf_frames: AtomicU64::new(0),
                    under: AtomicU64::new(0),
                    over: AtomicU64::new(0),
                })
                .collect(),
        }
    }

    fn reset_counters(&self) {
        self.tx_corrections.store(0, Ordering::Relaxed);
        self.tx_resyncs.store(0, Ordering::Relaxed);
        for d in &self.rx_dev {
            d.under.store(0, Ordering::Relaxed);
            d.over.store(0, Ordering::Relaxed);
        }
    }
}

struct Shared {
    stats: Mutex<Option<Arc<Stats>>>,
    status: Mutex<String>,
    running: AtomicBool,
    stop: AtomicBool,
}

fn set_status(shared: &Shared, text: impl Into<String>) {
    *shared.status.lock().unwrap() = text.into();
}

struct Pair {
    q: VecDeque<f32>,
    ch: usize,
    target: usize,
    started: bool,
    frac: f64,
}

// ---------------------------------------------------------------- audio devices

fn find_device(host: &cpal::Host, needle: &str, input: bool) -> Option<cpal::Device> {
    let needle = needle.to_lowercase();
    let devs: Vec<cpal::Device> = if input {
        host.input_devices().ok()?.collect()
    } else {
        host.output_devices().ok()?.collect()
    };
    devs.into_iter()
        .find(|d| d.name().map(|n| n.to_lowercase().contains(&needle)).unwrap_or(false))
}

fn device_names(input: bool) -> Vec<String> {
    let host = cpal::default_host();
    let devs: Vec<cpal::Device> = if input {
        host.input_devices().map(|d| d.collect()).unwrap_or_default()
    } else {
        host.output_devices().map(|d| d.collect()).unwrap_or_default()
    };
    devs.iter().filter_map(|d| d.name().ok()).collect()
}

fn describe_configs<I: Iterator<Item = cpal::SupportedStreamConfigRange>>(cfgs: I) -> String {
    let mut msg = String::new();
    for c in cfgs {
        msg.push_str(&format!(
            "\n  {} ch, {}-{} Hz, {:?}",
            c.channels(),
            c.min_sample_rate().0,
            c.max_sample_rate().0,
            c.sample_format()
        ));
    }
    msg
}

fn max_channels_at_48k(dev: &cpal::Device, input: bool) -> usize {
    let cfgs: Vec<cpal::SupportedStreamConfigRange> = if input {
        dev.supported_input_configs()
            .map(|i| i.collect())
            .unwrap_or_default()
    } else {
        dev.supported_output_configs()
            .map(|i| i.collect())
            .unwrap_or_default()
    };
    let mut max = 0usize;
    for c in &cfgs {
        if c.sample_format() == cpal::SampleFormat::F32
            && c.min_sample_rate().0 <= RATE
            && c.max_sample_rate().0 >= RATE
        {
            max = max.max(c.channels() as usize);
        }
    }
    max
}

fn build_output(
    device: &cpal::Device,
    channels: usize,
    mut fill: impl FnMut(&mut [f32], usize) + Send + 'static,
) -> Result<cpal::Stream, String> {
    let mut chosen = None;
    for cfg in device.supported_output_configs().map_err(|e| e.to_string())? {
        if cfg.sample_format() == cpal::SampleFormat::F32
            && cfg.channels() as usize >= channels
            && cfg.min_sample_rate().0 <= RATE
            && cfg.max_sample_rate().0 >= RATE
        {
            chosen = Some(cfg.with_sample_rate(cpal::SampleRate(RATE)));
            break;
        }
    }
    let supported = match chosen {
        Some(s) => s,
        None => {
            let offers = device
                .supported_output_configs()
                .map(|c| describe_configs(c))
                .unwrap_or_default();
            return Err(format!("no usable 48 kHz output (need {channels} ch). Device offers:{offers}"));
        }
    };
    let out_ch = supported.channels() as usize;
    let config = supported.config();
    let stream = device
        .build_output_stream(
            &config,
            move |data: &mut [f32], _: &cpal::OutputCallbackInfo| fill(data, out_ch),
            |e| log::error!("audio output error: {e}"),
            None,
        )
        .map_err(|e| e.to_string())?;
    stream.play().map_err(|e| e.to_string())?;
    Ok(stream)
}

fn build_input(
    device: &cpal::Device,
    channels: usize,
    mut handle: impl FnMut(&[f32], usize) + Send + 'static,
) -> Result<cpal::Stream, String> {
    let mut chosen = None;
    for cfg in device.supported_input_configs().map_err(|e| e.to_string())? {
        if cfg.sample_format() == cpal::SampleFormat::F32
            && cfg.channels() as usize >= channels
            && cfg.min_sample_rate().0 <= RATE
            && cfg.max_sample_rate().0 >= RATE
        {
            chosen = Some(cfg.with_sample_rate(cpal::SampleRate(RATE)));
            break;
        }
    }
    let supported = match chosen {
        Some(s) => s,
        None => {
            let offers = device
                .supported_input_configs()
                .map(|c| describe_configs(c))
                .unwrap_or_default();
            return Err(format!("no usable 48 kHz input (need {channels} ch). Device offers:{offers}"));
        }
    };
    let in_ch = supported.channels() as usize;
    let config = supported.config();
    let stream = device
        .build_input_stream(
            &config,
            move |data: &[f32], _: &cpal::InputCallbackInfo| handle(data, in_ch),
            |e| log::error!("audio input error: {e}"),
            None,
        )
        .map_err(|e| e.to_string())?;
    stream.play().map_err(|e| e.to_string())?;
    Ok(stream)
}

fn fill_from_pair(shared: &Arc<M
