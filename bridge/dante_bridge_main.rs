//! DanteBridge: receives Dante audio (via Inferno) and plays it to Windows
//! output devices (e.g. VB-Cable / VB-Matrix). Each device carries
//! --channels channels (default 2). Use --tone to test without any mixer.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use inferno_aoip::device_server::{DeviceServer, Sample, Settings};

const RATE: u32 = 48000;
const TARGET_FRAMES: usize = 4800; // 100 ms buffer target

struct Pair {
    q: VecDeque<f32>, // interleaved frames
    ch: usize,
    started: bool,
    frac: f64,
    underruns: u64,
    overruns: u64,
}

fn find_device(host: &cpal::Host, needle: &str) -> Option<cpal::Device> {
    let needle = needle.to_lowercase();
    for d in host.output_devices().ok()? {
        if let Ok(n) = d.name() {
            if n.to_lowercase().contains(&needle) {
                return Some(d);
            }
        }
    }
    None
}

fn build_stream(
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
            let mut msg = String::from("no usable 48 kHz F32 output. Device offers:");
            if let Ok(cfgs) = device.supported_output_configs() {
                for c in cfgs {
                    msg.push_str(&format!(
                        "\n  {} ch, {}-{} Hz, {:?}",
                        c.channels(),
                        c.min_sample_rate().0,
                        c.max_sample_rate().0,
                        c.sample_format()
                    ));
                }
            }
            return Err(msg);
        }
    };
    let out_ch = supported.channels() as usize;
    let config = supported.config();
    let stream = device
        .build_output_stream(
            &config,
            move |data: &mut [f32], _: &cpal::OutputCallbackInfo| fill(data, out_ch),
            |e| eprintln!("audio error: {e}"),
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
        if st.q.len() / n >= TARGET_FRAMES {
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
        let err = (avail as f64 - TARGET_FRAMES as f64) / TARGET_FRAMES as f64;
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

#[tokio::main(flavor = "current_thread")]
async fn main() {
    env_logger::init_from_env(env_logger::Env::default().default_filter_or("info"));
    let args: Vec<String> = std::env::args().collect();
    let host = cpal::default_host();

    if args.iter().any(|a| a == "--list") {
        println!("Output devices:");
        if let Ok(devs) = host.output_devices() {
            for d in devs {
                println!("  {}", d.name().unwrap_or_default());
            }
        }
        return;
    }

    let tone = args.iter().any(|a| a == "--tone");
    let list_arg = args
        .iter()
        .position(|a| a == "--devices" || a == "--device")
        .and_then(|i| args.get(i + 1))
        .cloned()
        .unwrap_or_else(|| "cable input".to_string());
    let per: usize = args
        .iter()
        .position(|a| a == "--channels")
        .and_then(|i| args.get(i + 1))
        .and_then(|v| v.parse().ok())
        .unwrap_or(2);

    let mut devices = Vec::new();
    for needle in list_arg.split(',').map(|s| s.trim()).filter(|s| !s.is_empty()) {
        match find_device(&host, needle) {
            Some(d) => {
                println!("Output device: {}", d.name().unwrap_or_default());
                devices.push(d);
            }
            None => {
                eprintln!("Output device containing '{needle}' not found. Run with --list.");
                return;
            }
        }
    }
    if devices.is_empty() {
        eprintln!("no devices given");
        return;
    }
    let total = per * devices.len();

    if tone {
        let mut streams = Vec::new();
        for (di, dev) in devices.iter().enumerate() {
            let base = di * per;
            let mut n: u64 = 0;
            let result = build_stream(dev, per, move |data, out_ch| {
                for frame in data.chunks_mut(out_ch) {
                    for (c, s) in frame.iter_mut().enumerate() {
                        *s = if c < per {
                            let f = 1000.0 * ((base + c) as f64 + 1.0);
                            (0.1 * (2.0 * std::f64::consts::PI * f * n as f64 / RATE as f64).sin()) as f32
                        } else {
                            0.0
                        };
                    }
                    n = (n + 1) % RATE as u64;
                }
            });
            match result {
                Ok(s) => streams.push(s),
                Err(e) => {
                    eprintln!("{e}");
                    return;
                }
            }
        }
        println!("Test tone: channel k = k kHz (CH1=1 kHz, CH2=2 kHz, ...). {total} channels. Ctrl+C to stop.");
        let _ = tokio::signal::ctrl_c().await;
        return;
    }

    // ---- Dante receive -> one ring buffer per device -> audio devices ----
    let pairs: Vec<Arc<Mutex<Pair>>> = (0..devices.len())
        .map(|_| {
            Arc::new(Mutex::new(Pair {
                q: VecDeque::new(),
                ch: per,
                started: false,
                frac: 0.0,
                underruns: 0,
                overruns: 0,
            }))
        })
        .collect();

    let mut streams = Vec::new();
    for (dev, pair) in devices.iter().zip(pairs.iter()) {
        let p = pair.clone();
        match build_stream(dev, per, move |data, out_ch| fill_from_pair(&p, data, out_ch)) {
            Ok(s) => streams.push(s),
            Err(e) => {
                eprintln!("{e}");
                return;
            }
        }
    }

    let rx_frames = Arc::new(AtomicU64::new(0));
    let stats_pairs = pairs.clone();
    let stats_rx = rx_frames.clone();
    std::thread::spawn(move || loop {
        std::thread::sleep(Duration::from_secs(1));
        let mut line = format!("rx frames {}", stats_rx.load(Ordering::Relaxed));
        for (i, p) in stats_pairs.iter().enumerate() {
            let st = p.lock().unwrap();
            line.push_str(&format!(
                " | dev{} buf {:.0} ms underruns {} overruns {}",
                i + 1,
                (st.q.len() / st.ch) as f64 * 1000.0 / RATE as f64,
                st.underruns,
                st.overruns
            ));
        }
        println!("{line}");
    });

    let mut settings = Settings::new("DanteBridge", "DanteBridge", None, &Default::default());
    settings.make_rx_channels(total);
    settings.make_tx_channels(0);

    let rx_pairs = pairs.clone();
    let rx_count = rx_frames.clone();
    let rx_callback = move |samples_count: usize, chans: &Vec<Vec<Sample>>| {
        if chans.len() < total {
            return;
        }
        for (pi, pair) in rx_pairs.iter().enumerate() {
            let mut st = pair.lock().unwrap();
            let n = st.ch;
            for si in 0..samples_count {
                for c in 0..n {
                    st.q.push_back((chans[pi * n + c][si] as f64 / 2147483648.0) as f32);
                }
            }
            let frames = st.q.len() / n;
            if frames > TARGET_FRAMES * 4 {
                let drop = (frames - TARGET_FRAMES) * n;
                st.q.drain(..drop);
                st.overruns += 1;
            }
        }
        rx_count.fetch_add(samples_count as u64, Ordering::Relaxed);
    };

    let mut server = DeviceServer::start(settings).await;
    server.receive_with_callback(Box::new(rx_callback)).await;

    let _ = tokio::signal::ctrl_c().await;
    server.shutdown().await;
    drop(streams);
}
