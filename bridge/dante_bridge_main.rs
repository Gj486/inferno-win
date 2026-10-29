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
use std::sync::atomic::{AtomicBool, AtomicI64, AtomicU32, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, RwLock};
use std::time::{Duration, Instant};

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use eframe::egui;
use inferno_aoip::device_server::{
    AtomicSample, Clock, DeviceServer, ExternalBufferParameters, MediaClock, RealTimeClockReceiver,
    Sample, Settings,
};
use serde::{Deserialize, Serialize};

const RATE: u32 = 48000;
const RING: usize = 65536;
const TX_TOL: isize = 240;
const TX_RESYNC: isize = 4800;
/// Smooth TX drift compensation: time constant of the error filter, time over which an error is
/// removed, and the largest speed change (1000 ppm = 0.1 %, about 1.7 cents).
const TX_EMA_SECONDS: f64 = 0.5;
const TX_CORRECTION_SECONDS: f64 = 2.0;
const TX_MAX_RATE_ADJ: f64 = 0.001;

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
    /// ask Windows for 1 ms timer ticks and a high priority class
    perf_boost: bool,
    /// "smooth" (resampling, default) or "legacy" (drop/repeat one sample) - Windows -> Dante direction
    tx_drift_mode: String,
    /// "smooth" or "legacy" - Dante -> ASIO inputs direction (applied by the ASIO driver)
    rx_drift_mode: String,
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
            perf_boost: true,
            tx_drift_mode: "smooth".into(),
            rx_drift_mode: "smooth".into(),
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
    /// milliseconds since the PTP clock overlay last changed (u64::MAX = never)
    ptp_age_ms: AtomicU64,
    tx_err_last: AtomicI64, // TX write position minus media clock (samples): >0 = ahead
    tx_err_peak: AtomicU64, // largest |error| since the last reset (samples)
    tx_rate_ppm: AtomicI64, // current TX speed adjustment (smooth mode)
    asio_in_lead: AtomicI64, // driver input queue lead vs target (frames)
    asio_in_underruns: AtomicU64,
    asio_in_ppm: AtomicI64,
    asio_driver_ver: AtomicU64,
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
            ptp_age_ms: AtomicU64::new(u64::MAX),
            tx_err_last: AtomicI64::new(0),
            tx_err_peak: AtomicU64::new(0),
            tx_rate_ppm: AtomicI64::new(0),
            asio_in_lead: AtomicI64::new(0),
            asio_in_underruns: AtomicU64::new(0),
            asio_in_ppm: AtomicI64::new(0),
            asio_driver_ver: AtomicU64::new(0),
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
        self.tx_err_peak.store(0, Ordering::Relaxed);
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

fn fill_from_pair(shared: &Arc<Mutex<Pair>>, dev: &DevStat, data: &mut [f32], out_ch: usize) {
    let mut st = shared.lock().unwrap();
    let n = st.ch;
    dev.buf_frames.store((st.q.len() / n) as u64, Ordering::Relaxed);
    if !st.started {
        if st.q.len() / n >= st.target {
            st.started = true;
            st.frac = 0.0;
        } else {
            data.fill(0.0);
            return;
        }
    }
    for frame in data.chunks_mut(out_ch) {
        let avail = st.q.len() / n;
        if avail < 2 {
            if st.started {
                dev.under.fetch_add(1, Ordering::Relaxed);
                st.started = false;
            }
            frame.fill(0.0);
            continue;
        }
        let err = (avail as f64 - st.target as f64) / st.target as f64;
        let step = (1.0 + err * 0.002).clamp(0.99, 1.01);
        let f = st.frac as f32;
        for c in 0..out_ch {
            frame[c] = if c < n {
                let a = st.q[c];
                let b = st.q[n + c];
                a + (b - a) * f
            } else {
                0.0
            };
        }
        st.frac += step;
        while st.frac >= 1.0 && st.q.len() >= n {
            for _ in 0..n {
                st.q.pop_front();
            }
            st.frac -= 1.0;
        }
    }
}

// ---------------------------------------------------------------- clock helper

/// Waits for the first PTP overlay. Returns a MediaClock plus the receiver that keeps
/// delivering newer overlays (the clock must be refreshed, because the Dante clock
/// drifts against the PC clock by tens of microseconds per second).
async fn make_media_clock(server: &DeviceServer) -> Option<(MediaClock, RealTimeClockReceiver)> {
    let mut rx = server.get_realtime_clock_receiver();
    let iterations = MEDIA_CLOCK_TIMEOUT_S * 10;
    for i in 0..iterations {
        rx.update();
        let mut found = None;
        if let Some(ovl) = rx.get() {
            found = Some(*ovl);
        }
        if let Some(o) = found {
            let mut mc = MediaClock::new(false);
            mc.update_overlay(o);
            return Some((mc, rx));
        }
        if i % 10 == 0 {
            log::info!(
                "waiting for PTP overlay... ({}/{})",
                i / 10,
                MEDIA_CLOCK_TIMEOUT_S
            );
        }
        tokio::time::sleep(Duration::from_millis(100)).await;
    }
    log::warn!(
        "gave up waiting for PTP overlay after {} s",
        MEDIA_CLOCK_TIMEOUT_S
    );
    None
}

/// One TX ring (RING samples of silence). Leaked on purpose: Inferno keeps raw pointers to it.
fn new_ring() -> &'static [AtomicSample] {
    let v: Vec<AtomicSample> = (0..RING).map(|_| AtomicSample::new(0)).collect();
    Box::leak(v.into_boxed_slice())
}

// ---------------------------------------------------------------- Dante transmit helper

struct TxWriter {
    bufs: Vec<&'static [AtomicSample]>,
    per: usize,
    ch_base: usize,
    w: usize,
    first: bool,
    start: Clock,
    media: MediaClock,
    clock_rx: RealTimeClockReceiver,
    st: Arc<Stats>,
    smooth: bool,
    ema: f64,       // filtered write-position error (samples)
    pos: f64,       // fractional read position in the input, relative to the next chunk
    prev: Vec<f32>, // last input sample of the previous chunk, per channel
}

impl TxWriter {
    fn push(&mut self, data: &[f32], in_ch: usize) {
        if self.smooth {
            self.push_smooth(data, in_ch);
        } else {
            self.push_legacy(data, in_ch);
        }
    }

    /// Smooth drift compensation: the write position error is low-pass filtered and removed by a
    /// tiny continuous speed change (linear interpolation) instead of dropping/repeating samples.
    fn push_smooth(&mut self, data: &[f32], in_ch: usize) {
        let n = data.len() / in_ch;
        if n == 0 {
            return;
        }
        self.clock_rx.update();
        if let Some(o) = self.clock_rx.get() {
            self.media.update_overlay(*o);
        }
        let now = match self.media.wrapping_now_in_timebase(RATE as u64) {
            Some(v) => v,
            None => return,
        };
        let m = now.wrapping_sub(self.start) as isize;
        if self.first {
            self.w = (m - n as isize).max(0) as usize;
            self.first = false;
        }
        let e = (self.w + n) as isize - m; // >0: we are ahead of the media clock
        self.st.tx_err_last.store(e as i64, Ordering::Relaxed);
        self.st.tx_err_peak.fetch_max(e.unsigned_abs() as u64, Ordering::Relaxed);
        if e.abs() > TX_RESYNC {
            self.w = (m - n as isize).max(0) as usize;
            self.pos = 0.0;
            self.ema = 0.0;
            self.st.tx_resyncs.fetch_add(1, Ordering::Relaxed);
        } else {
            let alpha = (n as f64 / (RATE as f64 * TX_EMA_SECONDS)).min(1.0);
            self.ema += (e as f64 - self.ema) * alpha;
        }
        // ahead of the clock -> consume input faster, i.e. write fewer samples
        let rho = (self.ema / (RATE as f64 * TX_CORRECTION_SECONDS)).clamp(-TX_MAX_RATE_ADJ, TX_MAX_RATE_ADJ);
        let step = 1.0 / (1.0 - rho);
        self.st.tx_rate_ppm.store((rho * 1e6).round() as i64, Ordering::Relaxed);

        let last = (n - 1) as f64;
        let mut written = 0usize;
        let mut peaks = [0f32; 16];
        while self.pos < last {
            let i0: isize = if self.pos < 0.0 { -1 } else { self.pos as isize };
            let frac = (self.pos - i0 as f64) as f32;
            for c in 0..self.per {
                let a = if i0 < 0 { self.prev[c] } else { data[i0 as usize * in_ch + c] };
                let b = data[(i0 + 1) as usize * in_ch + c];
                let s = a + (b - a) * frac;
                peaks[c] = peaks[c].max(s.abs());
                let v = (s.clamp(-1.0, 1.0) as f64 * 2147483647.0) as i32;
                self.bufs[c][(self.w + written) & (RING - 1)].store(v, Ordering::Relaxed);
            }
            written += 1;
            self.pos += step;
        }
        self.pos -= n as f64;
        for c in 0..self.per {
            self.prev[c] = data[(n - 1) * in_ch + c];
            self.st.tx_peaks[self.ch_base + c].fetch_max(peaks[c].to_bits(), Ordering::Relaxed);
        }
        self.w += written;
        self.st.tx_frames.fetch_add(n as u64, Ordering::Relaxed);
    }

    /// Original behaviour: drop or repeat one sample when the error leaves +-TX_TOL.
    fn push_legacy(&mut self, data: &[f32], in_ch: usize) {
        let n = data.len() / in_ch;
        if n == 0 {
            return;
        }
        // keep following the Dante clock (see make_media_clock)
        self.clock_rx.update();
        if let Some(o) = self.clock_rx.get() {
            self.media.update_overlay(*o);
        }
        let now = match self.media.wrapping_now_in_timebase(RATE as u64) {
            Some(v) => v,
            None => return,
        };
        let m = now.wrapping_sub(self.start) as isize;
        if self.first {
            self.w = (m - n as isize).max(0) as usize;
            self.first = false;
        }
        let e = (self.w + n) as isize - m;
        self.st.tx_err_last.store(e as i64, Ordering::Relaxed);
        self.st.tx_err_peak.fetch_max(e.unsigned_abs() as u64, Ordering::Relaxed);
        let mut n_out = n as isize;
        if e.abs() > TX_RESYNC {
            self.w = (m - n as isize).max(0) as usize;
            self.st.tx_resyncs.fetch_add(1, Ordering::Relaxed);
        } else if e > TX_TOL {
            n_out -= 1;
            self.st.tx_corrections.fetch_add(1, Ordering::Relaxed);
        } else if e < -TX_TOL {
            n_out += 1;
            self.st.tx_corrections.fetch_add(1, Ordering::Relaxed);
        }
        let n_out = n_out.max(1) as usize;
        for c in 0..self.per {
            let mut peak = 0f32;
            for i in 0..n_out {
                let src = i.min(n - 1);
                let s = data[src * in_ch + c];
                peak = peak.max(s.abs());
                let v = (s.clamp(-1.0, 1.0) as f64 * 2147483647.0) as i32;
                self.bufs[c][(self.w + i) & (RING - 1)].store(v, Ordering::Relaxed);
            }
            self.st.tx_peaks[self.ch_base + c].fetch_max(peak.to_bits(), Ordering::Relaxed);
        }
        self.w += n_out;
        self.st.tx_frames.fetch_add(n as u64, Ordering::Relaxed);
    }
}

fn asio_reader(mut writer: TxWriter, shm: Arc<shm::Shm>, shared: Arc<Shared>, st: Arc<Stats>) {
    let ch = shm::CH;
    let mut rd = shm.write_pos(shm::Dir::Out);
    let mut buf: Vec<f32> = Vec::new();
    let mut last_data = Instant::now();
    let mut idle: Option<(Instant, u64)> = None; // (start of the idle period, silence frames sent so far)
    let mut last_beat = shm.driver_beat();
    let mut last_beat_change = Instant::now();
    let mut last_report = Instant::now();
    let mut last_fixes = 0u64;
    while !shared.stop.load(Ordering::Relaxed) {
        std::thread::sleep(Duration::from_millis(2));
        shm.bump_bridge_beat();
        if last_report.elapsed() >= Duration::from_secs(10) {
            last_report = Instant::now();
            let fixes = st.tx_corrections.load(Ordering::Relaxed);
            log::info!(
                "[rx] driver_v{} queue_lead={:+.1} ms underruns={} rate={:+} ppm",
                st.asio_driver_ver.load(Ordering::Relaxed),
                st.asio_in_lead.load(Ordering::Relaxed) as f64 * 1000.0 / RATE as f64,
                st.asio_in_underruns.load(Ordering::Relaxed),
                st.asio_in_ppm.load(Ordering::Relaxed)
            );
            log::info!(
                "[tx] frames={} drift_fixes(+{} in 10 s) resyncs={} timing_error_last={:+.1} ms peak={:.1} ms rate={:+} ppm",
                st.tx_frames.load(Ordering::Relaxed),
                fixes - last_fixes,
                st.tx_resyncs.load(Ordering::Relaxed),
                st.tx_err_last.load(Ordering::Relaxed) as f64 * 1000.0 / RATE as f64,
                st.tx_err_peak.load(Ordering::Relaxed) as f64 * 1000.0 / RATE as f64,
                st.tx_rate_ppm.load(Ordering::Relaxed)
            );
            last_fixes = fixes;
        }
        let beat = shm.driver_beat();
        if beat != last_beat {
            last_beat = beat;
            last_beat_change = Instant::now();
        }
        st.asio_host
            .store(last_beat_change.elapsed() < Duration::from_millis(1000), Ordering::Relaxed);
        let (in_under, in_lead, in_ppm) = shm.rx_metrics();
        st.asio_in_underruns.store(in_under, Ordering::Relaxed);
        st.asio_in_lead.store(in_lead, Ordering::Relaxed);
        st.asio_in_ppm.store(in_ppm, Ordering::Relaxed);
        st.asio_driver_ver.store(shm.driver_version() as u64, Ordering::Relaxed);

        let wp = shm.write_pos(shm::Dir::Out);
        let mut avail = wp.wrapping_sub(rd) as i64;
        if avail < 0 || avail > (shm::FRAMES as i64) / 2 {
            rd = wp;
            avail = 0;
        }
        if avail > 0 {
            buf.resize(avail as usize * ch, 0.0);
            shm.read_frames(shm::Dir::Out, rd, &mut buf);
            rd = wp;
            writer.push(&buf, ch);
            last_data = Instant::now();
            idle = None;
        } else if last_data.elapsed() > Duration::from_millis(60) {
            // no ASIO host is playing: keep the Dante ring filled with silence at exactly the media rate
            // (counted against one fixed start time, so no fractional sample is ever lost)
            let (t0, sent) = idle.get_or_insert((Instant::now(), 0));
            let due = (t0.elapsed().as_secs_f64() * RATE as f64) as u64;
            if due > *sent {
                let n = (due - *sent) as usize;
                buf.clear();
                buf.resize(n * ch, 0.0);
                writer.push(&buf, ch);
                *sent = due;
            }
        }
    }
}

// ---------------------------------------------------------------- engine

fn engine_thread(cfg: Config, shared: Arc<Shared>, tone: bool) {
    let result = match tokio::runtime::Builder::new_current_thread().enable_all().build() {
        Ok(rt) => rt.block_on(run_engine(cfg, shared.clone(), tone)),
        Err(e) => Err(format!("cannot start runtime: {e}")),
    };
    *shared.stats.lock().unwrap() = None;
    match result {
        Ok(()) => set_status(&shared, "Stopped"),
        Err(e) => {
            log::error!("engine error: {e}");
            set_status(&shared, format!("ERROR: {e}"));
        }
    }
    shared.running.store(false, Ordering::SeqCst);
}

async fn run_engine(cfg: Config, shared: Arc<Shared>, tone: bool) -> Result<(), String> {
    let host = cpal::default_host();
    let mut per = cfg.channels_per_device.max(1);
    let configured_per = per;

    let mut rx_devs = Vec::new();
    for needle in &cfg.rx_devices {
        let d = find_device(&host, needle, false)
            .ok_or(format!("Output device '{needle}' not found"))?;
        rx_devs.push(d);
    }
    let mut tx_devs = Vec::new();
    for needle in &cfg.tx_devices {
        let d = find_device(&host, needle, true)
            .ok_or(format!("Input device '{needle}' not found"))?;
        tx_devs.push(d);
    }
    let use_asio = cfg.use_asio;
    let tx_smooth = cfg.tx_drift_mode.trim().to_lowercase() != "legacy";
    let rx_smooth = cfg.rx_drift_mode.trim().to_lowercase() != "legacy";
    log::info!(
        "[mode] perf_boost={} tx_drift_mode={} rx_drift_mode={} use_asio={}",
        cfg.perf_boost,
        if tx_smooth { "smooth" } else { "legacy" },
        if rx_smooth { "smooth" } else { "legacy" },
        use_asio
    );
    let tx_devs: Vec<cpal::Device> = if use_asio { Vec::new() } else { tx_devs };

    for d in &rx_devs {
        let m = max_channels_at_48k(d, false);
        if m == 0 {
            return Err(format!(
                "output device '{}' has no F32 48 kHz config at all",
                d.name().unwrap_or_default()
            ));
        }
        if m < per {
            log::warn!(
                "output device '{}' offers only {} ch at 48 kHz (configured {}); using {}",
                d.name().unwrap_or_default(),
                m,
                per,
                m
            );
            per = m;
        }
    }
    for d in &tx_devs {
        let m = max_channels_at_48k(d, true);
        if m == 0 {
            return Err(format!(
                "input device '{}' has no F32 48 kHz config at all",
                d.name().unwrap_or_default()
            ));
        }
        if m < per {
            log::warn!(
                "input device '{}' offers only {} ch at 48 kHz (configured {}); using {}",
                d.name().unwrap_or_default(),
                m,
                per,
                m
            );
            per = m;
        }
    }

    let rx_total = per * rx_devs.len();
    let tx_total = per * tx_devs.len();
    let rx_dante = if use_asio { rx_total.max(shm::CH) } else { rx_total };
    let tx_dante = if use_asio { shm::CH } else { tx_total };

    if tone {
        if rx_devs.is_empty() {
            return Err("no output devices selected".into());
        }
        let mut streams = Vec::new();
        for (di, dev) in rx_devs.iter().enumerate() {
            let base = di * per;
            let mut n: u64 = 0;
            streams.push(build_output(dev, per, move |data, out_ch| {
                for frame in data.chunks_mut(out_ch) {
                    for (c, s) in frame.iter_mut().enumerate() {
                        *s = if c < per {
                            let f = 1000.0 * ((base + c) as f64 + 1.0);
                            (0.1 * (2.0 * std::f64::consts::PI * f * n as f64 / RATE as f64).sin())
                                as f32
                        } else {
                            0.0
                        };
                    }
                    n = (n + 1) % RATE as u64;
                }
            })?);
        }
        set_status(&shared, "Test tone running (channel k = k kHz)");
        while !shared.stop.load(Ordering::Relaxed) {
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
        return Ok(());
    }

    let my_ip: Option<Ipv4Addr> = if cfg.bind_ip.trim().is_empty() {
        None
    } else {
        Some(
            cfg.bind_ip
                .trim()
                .parse()
                .map_err(|_| format!("'{}' is not a valid IPv4 address", cfg.bind_ip))?,
        )
    };

    let mut conf: BTreeMap<String, String> = BTreeMap::new();
    conf.insert("NAME".to_string(), cfg.device_name.clone());
    if cfg.alt_port != 0 {
        conf.insert("ALT_PORT".to_string(), cfg.alt_port.to_string());
    }
    conf.insert("TX_LATENCY_NS".to_string(), (cfg.tx_latency_ms as u64 * 1_000_000).to_string());
    let mut settings = Settings::new(&cfg.device_name, "DanteBridge", my_ip, &conf);
    settings.make_rx_channels(rx_dante);
    settings.make_tx_channels(tx_dante);

    let stats = Arc::new(Stats::new(rx_dante, tx_dante, rx_devs.len()));
    *shared.stats.lock().unwrap() = Some(stats.clone());

    let asio_shm: Option<Arc<shm::Shm>> = if use_asio {
        let m = shm::Shm::open().map_err(|e| format!("shared memory: {e}"))?;
        m.reset();
        m.init_layout(if rx_smooth { shm::FLAG_RX_SMOOTH } else { 0 });
        Some(Arc::new(m))
    } else {
        None
    };

    let target = cfg.buffer_ms.max(20) * (RATE as usize) / 1000;
    let pairs: Vec<Arc<Mutex<Pair>>> = (0..rx_devs.len())
        .map(|_| {
            Arc::new(Mutex::new(Pair {
                q: VecDeque::new(),
                ch: per,
                target,
                started: false,
                frac: 0.0,
            }))
        })
        .collect();

    let mut rx_streams = Vec::new();
    for (di, (dev, pair)) in rx_devs.iter().zip(pairs.iter()).enumerate() {
        let p = pair.clone();
        let st = stats.clone();
        rx_streams.push(build_output(dev, per, move |data, out_ch| {
            fill_from_pair(&p, &st.rx_dev[di], data, out_ch)
        })?);
    }

    // the PTP clock client (usrvclock) must join the multicast group on the Dante adapter
    if !cfg.bind_ip.trim().is_empty() {
        std::env::set_var("INFERNO_BIND_IP", cfg.bind_ip.trim());
    }
    let mut server = DeviceServer::start(settings).await;

    if rx_dante > 0 {
        let rx_pairs = pairs.clone();
        let st = stats.clone();
        let shm_rx = asio_shm.clone();
        let mut tmp: Vec<f32> = Vec::new();
        let rx_callback = move |samples_count: usize, chans: &Vec<Vec<Sample>>| {
            if chans.len() < rx_dante {
                return;
            }
            for (pi, pair) in rx_pairs.iter().enumerate() {
                let mut q = pair.lock().unwrap();
                let n = q.ch;
                for si in 0..samples_count {
                    for c in 0..n {
                        q.q.push_back((chans[pi * n + c][si] as f64 / 2147483648.0) as f32);
                    }
                }
                let frames = q.q.len() / n;
                if frames > q.target * 4 {
                    let drop = (frames - q.target) * n;
                    q.q.drain(..drop);
                    st.rx_dev[pi].over.fetch_add(1, Ordering::Relaxed);
                }
            }
            for c in 0..rx_dante {
                let peak = chans[c][..samples_count]
                    .iter()
                    .map(|s| (*s as i64).abs())
                    .max()
                    .unwrap_or(0);
                let pf = (peak as f64 / 2147483648.0) as f32;
                st.rx_peaks[c].fetch_max(pf.to_bits(), Ordering::Relaxed);
            }
            if let Some(m) = &shm_rx {
                tmp.clear();
                for si in 0..samples_count {
                    for c in 0..shm::CH {
                        tmp.push((chans[c][si] as f64 / 2147483648.0) as f32);
                    }
                }
                let wp = m.write_pos(shm::Dir::In);
                m.write_frames(shm::Dir::In, wp, &tmp);
                m.set_write_pos(shm::Dir::In, wp + samples_count as u64);
            }
            st.rx_frames.fetch_add(samples_count as u64, Ordering::Relaxed);
        };
        server.receive_with_callback(Box::new(rx_callback)).await;
    }

    let mut tx_streams = Vec::new();
    let mut asio_thread: Option<std::thread::JoinHandle<()>> = None;
    if tx_dante > 0 {
        let (mc0, _) = make_media_clock(&server)
            .await
            .ok_or_else(|| MEDIA_CLOCK_HINT.to_string())?;
        let start: Clock = mc0
            .wrapping_now_in_timebase(RATE as u64)
            .ok_or_else(|| {
                "media clock overlay present, but wrapping_now_in_timebase returned None \
                 (PTP timestamp looks invalid)".to_string()
            })?;
        let valid = Arc::new(RwLock::new(true));
        let mut params: Vec<ExternalBufferParameters<Sample>> = Vec::new();

        for (di, dev) in tx_devs.iter().enumerate() {
            let mut bufs: Vec<&'static [AtomicSample]> = Vec::new();
            for _ in 0..per {
                let b = new_ring();
                params.push(unsafe {
                    ExternalBufferParameters::<Sample>::new(b.as_ptr(), RING, 1, valid.clone(), None)
                });
                bufs.push(b);
            }
            let (media, clock_rx) = make_media_clock(&server)
                .await
                .ok_or_else(|| MEDIA_CLOCK_HINT.to_string())?;
            let mut writer = TxWriter {
                bufs,
                per,
                ch_base: di * per,
                w: 0,
                first: true,
                start,
                media,
                clock_rx,
                st: stats.clone(),
                smooth: tx_smooth,
                ema: 0.0,
                pos: 0.0,
                prev: vec![0.0; per],
            };
            tx_streams.push(build_input(dev, per, move |data, in_ch| writer.push(data, in_ch))?);
        }

        let mut asio_writer: Option<TxWriter> = None;
        if asio_shm.is_some() {
            let mut bufs: Vec<&'static [AtomicSample]> = Vec::new();
            for _ in 0..shm::CH {
                let b = new_ring();
                params.push(unsafe {
                    ExternalBufferParameters::<Sample>::new(b.as_ptr(), RING, 1, valid.clone(), None)
                });
                bufs.push(b);
            }
            let (media, clock_rx) = make_media_clock(&server)
                .await
                .ok_or_else(|| MEDIA_CLOCK_HINT.to_string())?;
            asio_writer = Some(TxWriter {
                bufs,
                per: shm::CH,
                ch_base: 0,
                w: 0,
                first: true,
                start,
                media,
                clock_rx,
                st: stats.clone(),
                smooth: tx_smooth,
                ema: 0.0,
                pos: 0.0,
                prev: vec![0.0; shm::CH],
            });
        }

        let (start_tx, start_rx) = tokio::sync::oneshot::channel::<Clock>();
        server
            .transmit_from_external_buffer(
                params,
                start_rx,
                Arc::new(AtomicUsize::new(usize::MAX)),
                None,
            )
            .await;
        let _ = start_tx.send(start);

        if let (Some(writer), Some(shm_t)) = (asio_writer, asio_shm.clone()) {
            let sh = shared.clone();
            let st = stats.clone();
            asio_thread = Some(std::thread::spawn(move || asio_reader(writer, shm_t, sh, st)));
        }
    }

    if per != configured_per {
        set_status(
            &shared,
            format!(
                "Running (channels_per_device clamped {} -> {}; check dante_bridge.log)",
                configured_per, per
            ),
        );
    } else {
        set_status(&shared, "Running");
    }
    // watch the PTP clock so the window can show whether we still follow the Dante clock
    let mut clk_rx = server.get_realtime_clock_receiver();
    let mut last_sync_seen: Option<i64> = None;
    let mut last_change = Instant::now();
    while !shared.stop.load(Ordering::Relaxed) {
        tokio::time::sleep(Duration::from_millis(100)).await;
        clk_rx.update();
        if let Some(o) = clk_rx.get() {
            if last_sync_seen != Some(o.last_sync) {
                last_sync_seen = Some(o.last_sync);
                last_change = Instant::now();
            }
        }
        stats.ptp_age_ms.store(last_change.elapsed().as_millis() as u64, Ordering::Relaxed);
    }

    drop(rx_streams);
    drop(tx_streams);
    if let Some(h) = asio_thread {
        let _ = h.join();
    }
    server.shutdown().await;
    Ok(())
}

// ---------------------------------------------------------------- GUI

fn list_nets() -> Vec<(String, String)> {
    let mut v = vec![("auto".to_string(), String::new())];
    if let Ok(list) = local_ip_address::list_afinet_netifas() {
        for (name, ip) in list {
            if let std::net::IpAddr::V4(v4) = ip {
                if !v4.is_loopback() {
                    v.push((name, v4.to_string()));
                }
            }
        }
    }
    v
}

fn meter_bar(ui: &mut egui::Ui, label: &str, peak: f32) {
    let db = if peak > 0.00003 { 20.0 * peak.log10() } else { -90.0 };
    let frac = ((db + 60.0) / 60.0).clamp(0.0, 1.0);
    let color = if db > -3.0 {
        egui::Color32::from_rgb(220, 60, 60)
    } else if db > -12.0 {
        egui::Color32::from_rgb(230, 200, 60)
    } else {
        egui::Color32::from_rgb(70, 200, 90)
    };
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new(label).monospace());
        let (rect, _) = ui.allocate_exact_size(egui::vec2(170.0, 14.0), egui::Sense::hover());
        let painter = ui.painter();
        painter.rect_filled(rect, 2.0, egui::Color32::from_gray(45));
        let fill = egui::Rect::from_min_size(rect.min, egui::vec2(rect.width() * frac, rect.height()));
        painter.rect_filled(fill, 2.0, color);
        ui.label(egui::RichText::new(format!("{db:6.1} dB")).monospace());
    });
}

enum Action {
    Start,
    Tone,
    Stop,
    Save,
    Refresh,
    ResetCounters,
}

struct App {
    cfg: Config,
    shared: Arc<Shared>,
    out_names: Vec<String>,
    in_names: Vec<String>,
    nets: Vec<(String, String)>,
    rx_disp: Vec<f32>,
    tx_disp: Vec<f32>,
    note: String,
}

impl App {
    fn new(cfg: Config, note: String) -> Self {
        Self {
            cfg,
            shared: Arc::new(Shared {
                stats: Mutex::new(None),
                status: Mutex::new("Stopped".to_string()),
                running: AtomicBool::new(false),
                stop: AtomicBool::new(false),
            }),
            out_names: device_names(false),
            in_names: device_names(true),
            nets: list_nets(),
            rx_disp: vec![],
            tx_disp: vec![],
            note,
        }
    }

    fn start(&mut self, tone: bool) {
        if self.shared.running.load(Ordering::SeqCst) {
            return;
        }
        if let Err(e) = save_config(&self.cfg) {
            self.note = format!("Could not save settings: {e}");
        }
        self.shared.stop.store(false, Ordering::SeqCst);
        self.shared.running.store(true, Ordering::SeqCst);
        set_status(&self.shared, "Starting...");
        let cfg = self.cfg.clone();
        let shared = self.shared.clone();
        std::thread::spawn(move || engine_thread(cfg, shared, tone));
    }
}

fn device_picker(
    ui: &mut egui::Ui,
    id: &str,
    list: &mut Vec<String>,
    names: &[String],
    add_label: &str,
) {
    let mut remove: Option<usize> = None;
    for i in 0..list.len() {
        ui.horizontal(|ui| {
            egui::ComboBox::from_id_source((id, i))
                .width(330.0)
                .selected_text(list[i].clone())
                .show_ui(ui, |ui| {
                    for n in names {
                        ui.selectable_value(&mut list[i], n.clone(), n);
                    }
                });
            if ui.button("Remove").clicked() {
                remove = Some(i);
            }
        });
    }
    if let Some(i) = remove {
        list.remove(i);
    }
    if ui.button(add_label).clicked() {
        list.push(names.first().cloned().unwrap_or_default());
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        ctx.request_repaint_after(Duration::from_millis(50));
        let running = self.shared.running.load(Ordering::SeqCst);
        let stats = self.shared.stats.lock().unwrap().clone();
        let status = self.shared.status.lock().unwrap().clone();

        if let Some(s) = &stats {
            if self.rx_disp.len() != s.rx_peaks.len() {
                self.rx_disp = vec![0.0; s.rx_peaks.len()];
            }
            if self.tx_disp.len() != s.tx_peaks.len() {
                self.tx_disp = vec![0.0; s.tx_peaks.len()];
            }
            for (i, p) in s.rx_peaks.iter().enumerate() {
                let v = f32::from_bits(p.swap(0, Ordering::Relaxed));
                self.rx_disp[i] = v.max(self.rx_disp[i] * 0.85);
            }
            for (i, p) in s.tx_peaks.iter().enumerate() {
                let v = f32::from_bits(p.swap(0, Ordering::Relaxed));
                self.tx_disp[i] = v.max(self.tx_disp[i] * 0.85);
            }
        } else {
            self.rx_disp.clear();
            self.tx_disp.clear();
        }

        let mut action: Option<Action> = None;
        let per = self.cfg.channels_per_device.max(1);

        egui::CentralPanel::default().show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                ui.heading("Dante Bridge");
                ui.label(
                    egui::RichText::new("media clock follows Dante PTP leader (QL5)")
                        .small()
                        .color(egui::Color32::GRAY),
                );
                ui.add_space(6.0);

                ui.horizontal(|ui| {
                    if ui.add_enabled(!running, egui::Button::new("Start")).clicked() {
                        action = Some(Action::Start);
                    }
                    if ui.add_enabled(running, egui::Button::new("Stop")).clicked() {
                        action = Some(Action::Stop);
                    }
                    if ui
                        .add_enabled(!running, egui::Button::new("Test tone (no Dante)"))
                        .clicked()
                    {
                        action = Some(Action::Tone);
                    }
                    if ui.button("Reset counters").clicked() {
                        action = Some(Action::ResetCounters);
                    }
                });
                let status_color = if status.starts_with("ERROR") {
                    egui::Color32::from_rgb(220, 60, 60)
                } else if status == "Running" {
                    egui::Color32::from_rgb(70, 200, 90)
                } else {
                    egui::Color32::GRAY
                };
                ui.label(egui::RichText::new(format!("Status: {status}")).color(status_color));
                if let Some(s) = &stats {
                    let age = s.ptp_age_ms.load(Ordering::Relaxed);
                    let (text, color) = if age == u64::MAX {
                        ("PTP clock: waiting for the first Sync...".to_string(), egui::Color32::YELLOW)
                    } else if age < 2000 {
                        ("PTP clock: following the Dante clock".to_string(), egui::Color32::from_rgb(70, 200, 90))
                    } else {
                        (
                            format!("PTP clock: NO Sync for {} s (check cable, firewall, clock Leader)", age / 1000),
                            egui::Color32::from_rgb(220, 60, 60),
                        )
                    };
                    ui.label(egui::RichText::new(text).color(color));
                }
                if !self.note.is_empty() {
                    ui.label(egui::RichText::new(self.note.clone()).color(egui::Color32::YELLOW));
                }
                ui.separator();

                if let Some(s) = &stats {
                    ui.columns(2, |cols| {
                        if self.cfg.use_asio {
                            let host_on = s.asio_host.load(Ordering::Relaxed);
                            cols[0].label(
                                egui::RichText::new(format!(
                                    "Dante -> ASIO inputs   frames: {}",
                                    s.rx_frames.load(Ordering::Relaxed)
                                ))
                                .strong(),
                            );
                            cols[0].label(
                                egui::RichText::new(if host_on {
                                    "ASIO host connected"
                                } else {
                                    "no ASIO host running"
                                })
                                .small(),
                            );
                            cols[0].label(
                                egui::RichText::new(format!(
                                    "input queue {:+.1} ms vs target   underruns {}   rate {:+} ppm",
                                    s.asio_in_lead.load(Ordering::Relaxed) as f64 * 1000.0 / RATE as f64,
                                    s.asio_in_underruns.load(Ordering::Relaxed),
                                    s.asio_in_ppm.load(Ordering::Relaxed)
                                ))
                                .small(),
                            );
                            let drv_ver = s.asio_driver_ver.load(Ordering::Relaxed);
                            if host_on && drv_ver != shm::DRIVER_VERSION as u64 {
                                cols[0].label(
                                    egui::RichText::new(format!(
                                        "ASIO driver build is v{drv_ver}, this app needs v{} - reinstall stageB",
                                        shm::DRIVER_VERSION
                                    ))
                                    .small()
                                    .color(egui::Color32::from_rgb(220, 60, 60)),
                                );
                            }
                            for gi in 0..shm::CH {
                                if gi < self.rx_disp.len() {
                                    meter_bar(&mut cols[0], &format!("RX{}", gi + 1), self.rx_disp[gi]);
                                }
                            }
                            cols[1].label(
                                egui::RichText::new(format!(
                                    "ASIO outputs -> Dante   frames: {}",
                                    s.tx_frames.load(Ordering::Relaxed)
                                ))
                                .strong(),
                            );
                            cols[1].label(
                                egui::RichText::new(format!(
                                    "drift fixes {}   resyncs {}   timing error {:+.1} ms (peak {:.1} ms)   rate {:+} ppm",
                                    s.tx_corrections.load(Ordering::Relaxed),
                                    s.tx_resyncs.load(Ordering::Relaxed),
                                    s.tx_err_last.load(Ordering::Relaxed) as f64 * 1000.0 / RATE as f64,
                                    s.tx_err_peak.load(Ordering::Relaxed) as f64 * 1000.0 / RATE as f64,
                                    s.tx_rate_ppm.load(Ordering::Relaxed)
                                ))
                                .small(),
                            );
                            for gi in 0..shm::CH {
                                if gi < self.tx_disp.len() {
                                    meter_bar(&mut cols[1], &format!("TX{}", gi + 1), self.tx_disp[gi]);
                                }
                            }
                            return;
                        }
                        cols[0].label(
                            egui::RichText::new(format!(
                                "Dante -> Windows   frames: {}",
                                s.rx_frames.load(Ordering::Relaxed)
                            ))
                            .strong(),
                        );
                        for (di, name) in self.cfg.rx_devices.iter().enumerate() {
                            if di >= s.rx_dev.len() {
                                break;
                            }
                            let d = &s.rx_dev[di];
                            let under = d.under.load(Ordering::Relaxed);
                            let over = d.over.load(Ordering::Relaxed);
                            let col = if under > 0 || over > 0 {
                                egui::Color32::from_rgb(220, 60, 60)
                            } else {
                                egui::Color32::from_rgb(70, 200, 90)
                            };
                            cols[0].label(egui::RichText::new(name.clone()).small());
                            cols[0].label(
                                egui::RichText::new(format!(
                                    "buffer {:.0} ms   underruns {}   overruns {}",
                                    d.buf_frames.load(Ordering::Relaxed) as f64 * 1000.0
                                        / RATE as f64,
                                    under,
                                    over
                                ))
                                .small()
                                .color(col),
                            );
                            for ch in 0..per {
                                let gi = di * per + ch;
                                if gi < self.rx_disp.len() {
                                    meter_bar(&mut cols[0], &format!("RX{}", gi + 1), self.rx_disp[gi]);
                                }
                            }
                        }

                        cols[1].label(
                            egui::RichText::new(format!(
                                "Windows -> Dante   frames: {}",
                                s.tx_frames.load(Ordering::Relaxed)
                            ))
                            .strong(),
                        );
                        cols[1].label(
                            egui::RichText::new(format!(
                                "drift fixes {}   resyncs {}   timing error {:+.1} ms (peak {:.1} ms)   rate {:+} ppm",
                                s.tx_corrections.load(Ordering::Relaxed),
                                s.tx_resyncs.load(Ordering::Relaxed),
                                s.tx_err_last.load(Ordering::Relaxed) as f64 * 1000.0 / RATE as f64,
                                s.tx_err_peak.load(Ordering::Relaxed) as f64 * 1000.0 / RATE as f64,
                                s.tx_rate_ppm.load(Ordering::Relaxed)
                            ))
                            .small(),
                        );
                        for (di, name) in self.cfg.tx_devices.iter().enumerate() {
                            cols[1].label(egui::RichText::new(name.clone()).small());
                            for ch in 0..per {
                                let gi = di * per + ch;
                                if gi < self.tx_disp.len() {
                                    meter_bar(&mut cols[1], &format!("TX{}", gi + 1), self.tx_disp[gi]);
                                }
                            }
                        }
                    });
                    ui.separator();
                }

                ui.add_enabled_ui(!running, |ui| {
                    ui.checkbox(
                        &mut self.cfg.use_asio,
                        "ASIO mode: use the \"Dante Bridge ASIO\" driver (4 in / 4 out) instead of VB-Matrix",
                    );
                    egui::CollapsingHeader::new("Settings")
                        .default_open(!running)
                        .show(ui, |ui| {
                            ui.horizontal(|ui| {
                                ui.label("Device name in Dante Controller:");
                                ui.text_edit_singleline(&mut self.cfg.device_name);
                            });
                            ui.horizontal(|ui| {
                                ui.label("Network adapter (Ethernet to Dante):");
                                let shown = if self.cfg.bind_ip.is_empty() {
                                    "auto".to_string()
                                } else {
                                    self.cfg.bind_ip.clone()
                                };
                                egui::ComboBox::from_id_source("bind_ip")
                                    .width(260.0)
                                    .selected_text(shown)
                                    .show_ui(ui, |ui| {
                                        for (name, ip) in &self.nets {
                                            ui.selectable_value(
                                                &mut self.cfg.bind_ip,
                                                ip.clone(),
                                                format!("{name}  {ip}"),
                                            );
                                        }
                                    });
                            });
                            ui.horizontal(|ui| {
                                ui.label("Alternative port (0 = standard):");
                                ui.add(egui::DragValue::new(&mut self.cfg.alt_port).clamp_range(0..=65535));
                            });
                            ui.horizontal(|ui| {
                                ui.label("Channels per Windows device:");
                                ui.add(
                                    egui::DragValue::new(&mut self.cfg.channels_per_device)
                                        .clamp_range(1..=8),
                                );
                            });
                            ui.horizontal(|ui| {
                                ui.label("Receive buffer (ms):");
                                ui.add(egui::DragValue::new(&mut self.cfg.buffer_ms).clamp_range(20..=1000));
                                ui.label("Transmit latency (ms):");
                                ui.add(
                                    egui::DragValue::new(&mut self.cfg.tx_latency_ms).clamp_range(5..=500),
                                );
                            });
                            ui.add_space(4.0);
                            ui.label(
                                egui::RichText::new("Dante -> Windows (Windows output devices)").strong(),
                            );
                            device_picker(ui, "rx", &mut self.cfg.rx_devices, &self.out_names, "+ Add output device");
                            ui.add_space(4.0);
                            ui.label(
                                egui::RichText::new("Windows -> Dante (Windows input devices)").strong(),
                            );
                            device_picker(ui, "tx", &mut self.cfg.tx_devices, &self.in_names, "+ Add input device");
                            ui.add_space(6.0);
                            ui.horizontal(|ui| {
                                if ui.button("Save settings").clicked() {
                                    action = Some(Action::Save);
                                }
                                if ui.button("Refresh device list").clicked() {
                                    action = Some(Action::Refresh);
                                }
                            });
                        });
                });
            });
        });

        match action {
            Some(Action::Start) => {
                self.note.clear();
                self.start(false);
            }
            Some(Action::Tone) => {
                self.note.clear();
                self.start(true);
            }
            Some(Action::Stop) => {
                self.shared.stop.store(true, Ordering::SeqCst);
                set_status(&self.shared, "Stopping...");
            }
            Some(Action::Save) => {
                self.note = match save_config(&self.cfg) {
                    Ok(_) => "Settings saved.".to_string(),
                    Err(e) => format!("Could not save settings: {e}"),
                };
            }
            Some(Action::Refresh) => {
                self.out_names = device_names(false);
                self.in_names = device_names(true);
                self.nets = list_nets();
            }
            Some(Action::ResetCounters) => {
                if let Some(s) = &stats {
                    s.reset_counters();
                }
            }
            None => {}
        }
    }

    fn on_exit(&mut self, _gl: Option<&eframe::glow::Context>) {
        self.shared.stop.store(true, Ordering::SeqCst);
        std::thread::sleep(Duration::from_millis(400));
    }
}

/// Windows defaults to a 15.6 ms timer tick, which makes every short sleep ~15 ms and
/// lets the real-time threads be descheduled. Ask for 1 ms ticks and a high priority class.
#[cfg(windows)]
mod winperf {
    #[link(name = "winmm")]
    extern "system" {
        fn timeBeginPeriod(period_ms: u32) -> u32;
    }
    #[link(name = "kernel32")]
    extern "system" {
        fn GetCurrentProcess() -> isize;
        fn SetPriorityClass(process: isize, class: u32) -> i32;
    }
    const HIGH_PRIORITY_CLASS: u32 = 0x0000_0080;

    pub fn boost() {
        // SAFETY: plain Win32 calls with valid arguments; GetCurrentProcess returns a pseudo-handle.
        unsafe {
            let timer = timeBeginPeriod(1);
            let prio = SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS);
            log::info!("[perf] timeBeginPeriod(1) -> {timer} (0 = ok), SetPriorityClass(HIGH) -> {prio} (non-zero = ok)");
        }
    }
}

fn main() -> eframe::Result<()> {
    let mut note = String::new();
    let cfg: Config = match std::fs::read_to_string(config_path()) {
        Ok(t) => match toml::from_str(&t) {
            Ok(c) => c,
            Err(e) => {
                note = format!("Error in dante_bridge.toml (using defaults): {e}");
                Config::default()
            }
        },
        Err(_) => Config::default(),
    };

    {
        let mut b = env_logger::Builder::new();
        // cmc_server logs 'unknown opcode 0x3010' (a QL5 status message) as ERROR every ~10 s
        b.parse_filters(&format!("{},inferno_aoip::device_server::cmc_server=off", cfg.log_level));
        if let Ok(f) = std::fs::File::create(exe_dir().join("dante_bridge.log")) {
            b.target(env_logger::Target::Pipe(Box::new(f)));
        }
        b.init();
    }
    std::panic::set_hook(Box::new(|info| log::error!("PANIC: {info}")));
    #[cfg(windows)]
    if cfg.perf_boost {
        winperf::boost();
    }

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([820.0, 720.0])
            .with_title("Dante Bridge"),
        ..Default::default()
    };
    eframe::run_native(
        "Dante Bridge",
        options,
        Box::new(move |_cc| Box::new(App::new(cfg, note))),
    )
}
