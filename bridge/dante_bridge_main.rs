#![windows_subsystem = "windows"]
//! DanteBridge v0.3 - graphical version.
//!   Dante -> Windows  (receive from Dante, play to Windows output devices)
//!   Windows -> Dante  (record from Windows input devices, transmit to Dante)
//! Settings are edited in the window and saved to dante_bridge.toml.
//!
//! NOTE: the media clock here is the local PC clock (no PTP lock to Dante).
//! v0.4: optional ASIO mode - audio goes through the "Dante Bridge ASIO" driver
//! (4 in / 4 out) instead of VB-Matrix, via shared memory (shm.rs).

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
const RING: usize = 65536; // samples per TX channel ring (power of two)
const TX_TOL: isize = 240; // 5 ms: drift tolerance before correcting
const TX_RESYNC: isize = 4800; // 100 ms: hard resync threshold

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
    rx_peaks: Vec<AtomicU32>, // f32 bits, peak since last UI read
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
    q: VecDeque<f32>, // interleaved frames
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
        // adaptive speed: follow the Dante clock by slightly changing read speed
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

async fn make_media_clock(server: &DeviceServer) -> Option<MediaClock> {
    let mut rx = server.get_realtime_clock_receiver();
    for _ in 0..50 {
        rx.update();
        if let Some(ovl) = rx.get() {
            let mut mc = MediaClock::new(false);
            mc.update_overlay(*ovl);
            return Some(mc);
        }
        tokio::time::sleep(Duration::from_millis(100)).await;
    }
    None
}

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
    st: Arc<Stats>,
}

impl TxWriter {
    /// `data` = interleaved f32 frames with `in_ch` channels per frame (first `per` are used).
    /// Call it when a chunk of audio has just been captured.
    fn push(&mut self, data: &[f32], in_ch: usize) {
        let n = data.len() / in_ch;
        if n == 0 {
            return;
        }
        let now = match self.media.wrapping_now_in_timebase(RATE as u64) {
            Some(v) => v,
            None => return,
        };
        let m = now.wrapping_sub(self.start) as isize; // media samples elapsed
        if self.first {
            self.w = (m - n as isize).max(0) as usize;
            self.first = false;
        }
        let e = (self.w + n) as isize - m; // >0: we are ahead of the media clock
        let mut n_out = n as isize;
        if e.abs() > TX_RESYNC {
            self.w = (m - n as isize).max(0) as usize;
            self.st.tx_resyncs.fetch_add(1, Ordering::Relaxed);
        } else if e > TX_TOL {
            n_out -= 1; // drop one sample
            self.st.tx_corrections.fetch_add(1, Ordering::Relaxed);
        } else if e < -TX_TOL {
            n_out += 1; // repeat one sample
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

/// Reads what the ASIO host plays (shared memory "Out" ring) and sends it to Dante.
/// When no host is playing, it keeps the Dante ring filled with silence.
fn asio_reader(mut writer: TxWriter, shm: Arc<shm::Shm>, shared: Arc<Shared>, st: Arc<Stats>) {
    let ch = shm::CH;
    let mut rd = shm.write_pos(shm::Dir::Out);
    let mut buf: Vec<f32> = Vec::new();
    let mut last_data = Instant::now();
    let mut last_push = Instant::now();
    let mut last_beat = shm.driver_beat();
    let mut last_beat_change = Instant::now();
    while !shared.stop.load(Ordering::Relaxed) {
        std::thread::sleep(Duration::from_millis(2));
        shm.bump_bridge_beat();
        let beat = shm.driver_beat();
        if beat != last_beat {
            last_beat = beat;
            last_beat_change = Instant::now();
        }
        st.asio_host
            .store(last_beat_change.elapsed() < Duration::from_millis(1000), Ordering::Relaxed);

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
            last_push = last_data;
        } else if last_data.elapsed() > Duration::from_millis(60) {
            let n = (last_push.elapsed().as_secs_f64() * RATE as f64) as usize;
            if n > 0 {
                buf.clear();
                buf.resize(n * ch, 0.0);
                writer.push(&buf, ch);
                last_push = Instant::now();
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
    let per = cfg.channels_per_device.max(1);

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
    let tx_devs: Vec<cpal::Device> = if use_asio { Vec::new() } else { tx_devs };
    let rx_total = per * rx_devs.len();
    let tx_total = per * tx_devs.len();
    let rx_dante = if use_asio { rx_total.max(shm::CH) } else { rx_total };
    let tx_dante = if use_asio { shm::CH } else { tx_total };

    // ---- tone test: 1 kHz, 2 kHz, ... on the rx devices, no Dante at all
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
        Some(Arc::new(m))
    } else {
        None
    };

    // ---- Dante receive -> Windows output devices
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

    // ---- transmit to Dante: from Windows input devices, or from the ASIO driver
    let mut tx_streams = Vec::new();
    let mut asio_thread: Option<std::thread::JoinHandle<()>> = None;
    if tx_dante > 0 {
        let mc0 = make_media_clock(&server)
            .await
            .ok_or("media clock not available".to_string())?;
        let start: Clock = mc0
            .wrapping_now_in_timebase(RATE as u64)
            .ok_or("media clock not ready".to_string())?;
        let valid = Arc::new(RwLock::new(true));
        let mut params: Vec<ExternalBufferParameters<Sample>> = Vec::new();

        for (di, dev) in tx_devs.iter().enumerate() {
            let mut bufs: Vec<&'static [AtomicSample]> = Vec::new();
            for _ in 0..per {
                let b = new_ring();
                // safety: b is leaked, so it stays valid for the whole program
                params.push(unsafe {
                    ExternalBufferParameters::<Sample>::new(b.as_ptr(), RING, 1, valid.clone(), None)
                });
                bufs.push(b);
            }
            let media = make_media_clock(&server)
                .await
                .ok_or("media clock not available".to_string())?;
            let mut writer = TxWriter {
                bufs,
                per,
                ch_base: di * per,
                w: 0,
                first: true,
                start,
                media,
                st: stats.clone(),
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
            let media = make_media_clock(&server)
                .await
                .ok_or("media clock not available".to_string())?;
            asio_writer = Some(TxWriter {
                bufs,
                per: shm::CH,
                ch_base: 0,
                w: 0,
                first: true,
                start,
                media,
                st: stats.clone(),
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

    set_status(&shared, "Running");
    while !shared.stop.load(Ordering::Relaxed) {
        tokio::time::sleep(Duration::from_millis(100)).await;
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

        // meters with decay
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
                    egui::RichText::new("local clock - not PTP locked")
                        .small()
                        .color(egui::Color32::GRAY),
                );
                ui.add_space(6.0);

                // ---- controls
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
                if !self.note.is_empty() {
                    ui.label(egui::RichText::new(self.note.clone()).color(egui::Color32::YELLOW));
                }
                ui.separator();

                // ---- meters
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
                                    "drift fixes {}   resyncs {}",
                                    s.tx_corrections.load(Ordering::Relaxed),
                                    s.tx_resyncs.load(Ordering::Relaxed)
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
                                "drift fixes {}   resyncs {}",
                                s.tx_corrections.load(Ordering::Relaxed),
                                s.tx_resyncs.load(Ordering::Relaxed)
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

                // ---- settings (locked while running)
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

    // log to a file next to the exe
    {
        let mut b = env_logger::Builder::new();
        b.parse_filters(&cfg.log_level);
        if let Ok(f) = std::fs::File::create(exe_dir().join("dante_bridge.log")) {
            b.target(env_logger::Target::Pipe(Box::new(f)));
        }
        b.init();
    }
    std::panic::set_hook(Box::new(|info| log::error!("PANIC: {info}")));

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
