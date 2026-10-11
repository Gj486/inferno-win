//! DanteBridge v0.2
//!   Dante -> Windows  (receive from Dante, play to Windows output devices)
//!   Windows -> Dante  (record from Windows input devices, transmit to Dante)
//! Colored text screen + dante_bridge.toml settings file.
//!
//! NOTE: the media clock here is the local PC clock (no PTP lock to Dante).

use std::collections::{BTreeMap, VecDeque};
use std::io::Write;
use std::net::Ipv4Addr;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, RwLock};
use std::time::{Duration, Instant};

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use crossterm::{
    cursor::{Hide, MoveTo, Show},
    event::{self, Event, KeyCode, KeyEventKind, KeyModifiers},
    execute, queue,
    style::{Color, Print, ResetColor, SetForegroundColor},
    terminal::{self, Clear, ClearType},
};
use inferno_aoip::device_server::{
    AtomicSample, Clock, DeviceServer, ExternalBufferParameters, MediaClock, Sample, Settings,
};
use serde::Deserialize;

const RATE: u32 = 48000;
const RING: usize = 65536; // samples per TX channel ring (power of two)
const TX_TOL: isize = 240; // 5 ms: drift tolerance before correcting
const TX_RESYNC: isize = 4800; // 100 ms: hard resync threshold

const DEFAULT_CONFIG: &str = include_str!("dante_bridge.toml");

// ---------------------------------------------------------------- config

#[derive(Deserialize, Clone)]
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
        }
    }
}

fn exe_dir() -> PathBuf {
    std::env::current_exe()
        .ok()
        .and_then(|p| p.parent().map(|d| d.to_path_buf()))
        .unwrap_or_else(|| PathBuf::from("."))
}

// ---------------------------------------------------------------- shared state

struct Pair {
    q: VecDeque<f32>, // interleaved frames
    ch: usize,
    target: usize,
    started: bool,
    frac: f64,
    underruns: u64,
    overruns: u64,
}

struct Stats {
    rx_frames: AtomicU64,
    tx_frames: AtomicU64,
    tx_corrections: AtomicU64,
    tx_resyncs: AtomicU64,
    rx_peaks: Vec<AtomicU32>, // f32 bits, peak since last UI read
    tx_peaks: Vec<AtomicU32>,
    quit: AtomicBool,
    reset: AtomicBool,
}

fn atomics(n: usize) -> Vec<AtomicU32> {
    (0..n).map(|_| AtomicU32::new(0)).collect()
}

// ---------------------------------------------------------------- audio devices

fn find_device(host: &cpal::Host, needle: &str, input: bool) -> Option<cpal::Device> {
    let needle = needle.to_lowercase();
    let devs: Vec<cpal::Device> = if input {
        host.input_devices().ok()?.collect()
    } else {
        host.output_devices().ok()?.collect()
    };
    devs.into_iter().find(|d| {
        d.name().map(|n| n.to_lowercase().contains(&needle)).unwrap_or(false)
    })
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
            return Err(format!("no usable 48 kHz F32 output. Device offers:{offers}"));
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
            return Err(format!("no usable 48 kHz F32 input. Device offers:{offers}"));
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

fn fill_from_pair(shared: &Arc<Mutex<Pair>>, data: &mut [f32], out_ch: usize) {
    let mut st = shared.lock().unwrap();
    let n = st.ch;
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
                st.underruns += 1;
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

// ---------------------------------------------------------------- text UI

struct UiInfo {
    name: String,
    ip: String,
    rx_names: Vec<String>,
    tx_names: Vec<String>,
    per: usize,
}

fn text_line<W: Write>(out: &mut W, color: Color, text: &str) {
    let _ = queue!(
        out,
        SetForegroundColor(color),
        Print(text),
        ResetColor,
        Clear(ClearType::UntilNewLine),
        Print("\r\n")
    );
}

fn meter<W: Write>(out: &mut W, label: &str, peak: f32) {
    let db = if peak > 0.00003 { 20.0 * peak.log10() } else { -90.0 };
    let filled = (((db + 60.0) / 60.0).clamp(0.0, 1.0) * 20.0).round() as usize;
    let color = if db > -3.0 {
        Color::Red
    } else if db > -12.0 {
        Color::Yellow
    } else {
        Color::Green
    };
    let _ = queue!(
        out,
        Print(format!("    {label:<6}[")),
        SetForegroundColor(color),
        Print("#".repeat(filled)),
        ResetColor,
        Print("-".repeat(20 - filled)),
        Print(format!("] {db:6.1} dB")),
        Clear(ClearType::UntilNewLine),
        Print("\r\n")
    );
}

fn ui_thread(info: UiInfo, stats: Arc<Stats>, pairs: Vec<Arc<Mutex<Pair>>>) {
    let mut out = std::io::stdout();
    let _ = terminal::enable_raw_mode();
    let _ = execute!(out, Hide, Clear(ClearType::All));
    let mut rx_disp = vec![0f32; stats.rx_peaks.len()];
    let mut tx_disp = vec![0f32; stats.tx_peaks.len()];
    let started = Instant::now();

    loop {
        for (i, p) in stats.rx_peaks.iter().enumerate() {
            let v = f32::from_bits(p.swap(0, Ordering::Relaxed));
            rx_disp[i] = v.max(rx_disp[i] * 0.85);
        }
        for (i, p) in stats.tx_peaks.iter().enumerate() {
            let v = f32::from_bits(p.swap(0, Ordering::Relaxed));
            tx_disp[i] = v.max(tx_disp[i] * 0.85);
        }

        let up = started.elapsed().as_secs();
        let _ = queue!(out, MoveTo(0, 0));
        text_line(&mut out, Color::Cyan, " DANTE BRIDGE v0.2  (local clock - not PTP locked)");
        text_line(
            &mut out,
            Color::White,
            &format!(
                " Name: {}   IP: {}   48 kHz   up {:02}:{:02}:{:02}",
                info.name,
                if info.ip.is_empty() { "auto" } else { &info.ip },
                up / 3600,
                (up / 60) % 60,
                up % 60
            ),
        );
        text_line(&mut out, Color::DarkGrey, "");

        text_line(
            &mut out,
            Color::Yellow,
            &format!(
                " Dante -> Windows   frames received: {}",
                stats.rx_frames.load(Ordering::Relaxed)
            ),
        );
        if info.rx_names.is_empty() {
            text_line(&mut out, Color::DarkGrey, "    (no rx_devices configured)");
        }
        for (di, name) in info.rx_names.iter().enumerate() {
            let (buf_ms, under, over) = {
                let st = pairs[di].lock().unwrap();
                ((st.q.len() / st.ch) as f64 * 1000.0 / RATE as f64, st.underruns, st.overruns)
            };
            let c = if under > 0 || over > 0 { Color::Red } else { Color::Green };
            text_line(
                &mut out,
                c,
                &format!("  {name}   buf {buf_ms:.0} ms   underruns {under}   overruns {over}"),
            );
            for ch in 0..info.per {
                let gi = di * info.per + ch;
                meter(&mut out, &format!("RX {}", gi + 1), rx_disp[gi]);
            }
        }
        text_line(&mut out, Color::DarkGrey, "");

        text_line(
            &mut out,
            Color::Yellow,
            &format!(
                " Windows -> Dante   frames sent: {}   drift fixes: {}   resyncs: {}",
                stats.tx_frames.load(Ordering::Relaxed),
                stats.tx_corrections.load(Ordering::Relaxed),
                stats.tx_resyncs.load(Ordering::Relaxed)
            ),
        );
        if info.tx_names.is_empty() {
            text_line(&mut out, Color::DarkGrey, "    (no tx_devices configured)");
        }
        for (di, name) in info.tx_names.iter().enumerate() {
            text_line(&mut out, Color::Green, &format!("  {name}"));
            for ch in 0..info.per {
                let gi = di * info.per + ch;
                meter(&mut out, &format!("TX {}", gi + 1), tx_disp[gi]);
            }
        }
        text_line(&mut out, Color::DarkGrey, "");
        text_line(&mut out, Color::DarkGrey, " q = quit    r = reset counters");
        let _ = queue!(out, Clear(ClearType::FromCursorDown));
        let _ = out.flush();

        if event::poll(Duration::from_millis(150)).unwrap_or(false) {
            if let Ok(Event::Key(k)) = event::read() {
                if k.kind == KeyEventKind::Press {
                    match k.code {
                        KeyCode::Char('q') | KeyCode::Char('Q') => {
                            stats.quit.store(true, Ordering::Relaxed)
                        }
                        KeyCode::Char('c') if k.modifiers.contains(KeyModifiers::CONTROL) => {
                            stats.quit.store(true, Ordering::Relaxed)
                        }
                        KeyCode::Char('r') | KeyCode::Char('R') => {
                            for p in &pairs {
                                let mut st = p.lock().unwrap();
                                st.underruns = 0;
                                st.overruns = 0;
                            }
                            stats.tx_corrections.store(0, Ordering::Relaxed);
                            stats.tx_resyncs.store(0, Ordering::Relaxed);
                            stats.reset.store(true, Ordering::Relaxed);
                        }
                        _ => {}
                    }
                }
            }
        }
        if stats.quit.load(Ordering::Relaxed) {
            break;
        }
    }
    let _ = execute!(out, Show, ResetColor);
    let _ = terminal::disable_raw_mode();
    println!();
}

// ---------------------------------------------------------------- main

#[tokio::main(flavor = "current_thread")]
async fn main() {
    let args: Vec<String> = std::env::args().collect();
    let host = cpal::default_host();

    if args.iter().any(|a| a == "--list") {
        println!("Output devices (use in rx_devices):");
        if let Ok(devs) = host.output_devices() {
            for d in devs {
                println!("  {}", d.name().unwrap_or_default());
            }
        }
        println!("Input devices (use in tx_devices):");
        if let Ok(devs) = host.input_devices() {
            for d in devs {
                println!("  {}", d.name().unwrap_or_default());
            }
        }
        return;
    }

    // ---- config file
    let cfg_path = exe_dir().join("dante_bridge.toml");
    if !cfg_path.exists() {
        match std::fs::write(&cfg_path, DEFAULT_CONFIG) {
            Ok(_) => println!(
                "Created settings file:\n  {}\nEdit it (set bind_ip), save, and run again.",
                cfg_path.display()
            ),
            Err(e) => eprintln!("Could not create {}: {e}", cfg_path.display()),
        }
        return;
    }
    let cfg: Config = match std::fs::read_to_string(&cfg_path)
        .map_err(|e| e.to_string())
        .and_then(|t| toml::from_str(&t).map_err(|e| e.to_string()))
    {
        Ok(c) => c,
        Err(e) => {
            eprintln!("Error in {}:\n{e}", cfg_path.display());
            return;
        }
    };

    // ---- log to file (keeps the screen clean)
    {
        let mut b = env_logger::Builder::new();
        b.parse_filters(&cfg.log_level);
        if let Ok(f) = std::fs::File::create(exe_dir().join("dante_bridge.log")) {
            b.target(env_logger::Target::Pipe(Box::new(f)));
        }
        b.init();
    }

    let per = cfg.channels_per_device.max(1);

    // ---- tone test mode: plays 1 kHz, 2 kHz, ... on the rx_devices
    if args.iter().any(|a| a == "--tone") {
        let mut streams = Vec::new();
        for (di, needle) in cfg.rx_devices.iter().enumerate() {
            let dev = match find_device(&host, needle, false) {
                Some(d) => d,
                None => {
                    eprintln!("Output device containing '{needle}' not found. Try --list.");
                    return;
                }
            };
            println!("Output device: {}", dev.name().unwrap_or_default());
            let base = di * per;
            let mut n: u64 = 0;
            match build_output(&dev, per, move |data, out_ch| {
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
            }) {
                Ok(s) => streams.push(s),
                Err(e) => {
                    eprintln!("{e}");
                    return;
                }
            }
        }
        println!("Test tone running: channel k = k kHz. Press Ctrl+C to stop.");
        let _ = tokio::signal::ctrl_c().await;
        return;
    }

    // ---- find devices
    let mut rx_devs = Vec::new();
    let mut rx_names = Vec::new();
    for needle in &cfg.rx_devices {
        match find_device(&host, needle, false) {
            Some(d) => {
                rx_names.push(d.name().unwrap_or_default());
                rx_devs.push(d);
            }
            None => {
                eprintln!("Output device containing '{needle}' not found. Run: dante_bridge.exe --list");
                return;
            }
        }
    }
    let mut tx_devs = Vec::new();
    let mut tx_names = Vec::new();
    for needle in &cfg.tx_devices {
        match find_device(&host, needle, true) {
            Some(d) => {
                tx_names.push(d.name().unwrap_or_default());
                tx_devs.push(d);
            }
            None => {
                eprintln!("Input device containing '{needle}' not found. Run: dante_bridge.exe --list");
                return;
            }
        }
    }
    let rx_total = per * rx_devs.len();
    let tx_total = per * tx_devs.len();

    let my_ip: Option<Ipv4Addr> = if cfg.bind_ip.trim().is_empty() {
        None
    } else {
        match cfg.bind_ip.trim().parse() {
            Ok(ip) => Some(ip),
            Err(_) => {
                eprintln!("bind_ip in settings is not a valid IPv4 address: {}", cfg.bind_ip);
                return;
            }
        }
    };

    let mut conf: BTreeMap<String, String> = BTreeMap::new();
    conf.insert("NAME".to_string(), cfg.device_name.clone());
    if cfg.alt_port != 0 {
        conf.insert("ALT_PORT".to_string(), cfg.alt_port.to_string());
    }
    conf.insert(
        "TX_LATENCY_NS".to_string(),
        (cfg.tx_latency_ms as u64 * 1_000_000).to_string(),
    );
    let mut settings = Settings::new(&cfg.device_name, "DanteBridge", my_ip, &conf);
    settings.make_rx_channels(rx_total);
    settings.make_tx_channels(tx_total);

    let stats = Arc::new(Stats {
        rx_frames: AtomicU64::new(0),
        tx_frames: AtomicU64::new(0),
        tx_corrections: AtomicU64::new(0),
        tx_resyncs: AtomicU64::new(0),
        rx_peaks: atomics(rx_total),
        tx_peaks: atomics(tx_total),
        quit: AtomicBool::new(false),
        reset: AtomicBool::new(false),
    });

    // ---- Dante receive -> Windows output devices
    let target = (cfg.buffer_ms.max(20)) * (RATE as usize) / 1000;
    let pairs: Vec<Arc<Mutex<Pair>>> = (0..rx_devs.len())
        .map(|_| {
            Arc::new(Mutex::new(Pair {
                q: VecDeque::new(),
                ch: per,
                target,
                started: false,
                frac: 0.0,
                underruns: 0,
                overruns: 0,
            }))
        })
        .collect();

    let mut rx_streams = Vec::new();
    for (dev, pair) in rx_devs.iter().zip(pairs.iter()) {
        let p = pair.clone();
        match build_output(dev, per, move |data, out_ch| fill_from_pair(&p, data, out_ch)) {
            Ok(s) => rx_streams.push(s),
            Err(e) => {
                eprintln!("Cannot open output device: {e}");
                return;
            }
        }
    }

    let mut server = DeviceServer::start(settings).await;

    if rx_total > 0 {
        let rx_pairs = pairs.clone();
        let st = stats.clone();
        let rx_callback = move |samples_count: usize, chans: &Vec<Vec<Sample>>| {
            if chans.len() < rx_total {
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
                    q.overruns += 1;
                }
                for c in 0..n {
                    let peak = chans[pi * n + c][..samples_count]
                        .iter()
                        .map(|s| (*s as i64).abs())
                        .max()
                        .unwrap_or(0);
                    let pf = (peak as f64 / 2147483648.0) as f32;
                    st.rx_peaks[pi * n + c].fetch_max(pf.to_bits(), Ordering::Relaxed);
                }
            }
            st.rx_frames.fetch_add(samples_count as u64, Ordering::Relaxed);
        };
        server.receive_with_callback(Box::new(rx_callback)).await;
    }

    // ---- Windows input devices -> Dante transmit
    let mut tx_streams = Vec::new();
    if tx_total > 0 {
        let mc0 = match make_media_clock(&server).await {
            Some(m) => m,
            None => {
                eprintln!("media clock not available");
                return;
            }
        };
        let start: Clock = match mc0.wrapping_now_in_timebase(RATE as u64) {
            Some(s) => s,
            None => {
                eprintln!("media clock not ready");
                return;
            }
        };
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
            let media = match make_media_clock(&server).await {
                Some(m) => m,
                None => {
                    eprintln!("media clock not available");
                    return;
                }
            };
            let st = stats.clone();
            let ch_base = di * per;
            let mut w: usize = 0;
            let mut first = true;
            let result = build_input(dev, per, move |data, in_ch| {
                let n = data.len() / in_ch;
                if n == 0 {
                    return;
                }
                let now = match media.wrapping_now_in_timebase(RATE as u64) {
                    Some(v) => v,
                    None => return,
                };
                let m = now.wrapping_sub(start) as isize; // media samples elapsed
                if first {
                    w = (m - n as isize).max(0) as usize;
                    first = false;
                }
                let e = (w + n) as isize - m; // >0: we are ahead of the media clock
                let mut n_out = n as isize;
                if e.abs() > TX_RESYNC {
                    w = (m - n as isize).max(0) as usize;
                    st.tx_resyncs.fetch_add(1, Ordering::Relaxed);
                } else if e > TX_TOL {
                    n_out -= 1; // drop one sample
                    st.tx_corrections.fetch_add(1, Ordering::Relaxed);
                } else if e < -TX_TOL {
                    n_out += 1; // repeat one sample
                    st.tx_corrections.fetch_add(1, Ordering::Relaxed);
                }
                let n_out = n_out.max(1) as usize;
                for c in 0..per {
                    let mut peak = 0f32;
                    for i in 0..n_out {
                        let src = i.min(n - 1);
                        let s = data[src * in_ch + c];
                        peak = peak.max(s.abs());
                        let v = (s.clamp(-1.0, 1.0) as f64 * 2147483647.0) as i32;
                        bufs[c][(w + i) & (RING - 1)].store(v, Ordering::Relaxed);
                    }
                    st.tx_peaks[ch_base + c].fetch_max(peak.to_bits(), Ordering::Relaxed);
                }
                w += n_out;
                st.tx_frames.fetch_add(n as u64, Ordering::Relaxed);
            });
            match result {
                Ok(s) => tx_streams.push(s),
                Err(e) => {
                    eprintln!("Cannot open input device: {e}");
                    return;
                }
            }
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
    }

    // ---- text screen
    let info = UiInfo {
        name: cfg.device_name.clone(),
        ip: cfg.bind_ip.clone(),
        rx_names,
        tx_names,
        per,
    };
    let ui_stats = stats.clone();
    let ui_pairs = pairs.clone();
    let ui = std::thread::spawn(move || ui_thread(info, ui_stats, ui_pairs));

    while !stats.quit.load(Ordering::Relaxed) {
        tokio::time::sleep(Duration::from_millis(100)).await;
    }
    let _ = ui.join();

    drop(rx_streams);
    drop(tx_streams);
    server.shutdown().await;
}
