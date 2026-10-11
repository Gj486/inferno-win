//! DanteBridge: receives Dante audio (via Inferno) and plays it to a Windows
//! output device (e.g. VB-Cable). Use --tone to test without any mixer.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use inferno_aoip::device_server::{DeviceServer, Sample, Settings};

const RATE: u32 = 48000;
const TARGET_FRAMES: usize = 4800; // 100 ms buffer target

struct Shared {
    q: VecDeque<f32>, // interleaved frames
    ch: usize,
    started: bool,
    frac: f64,
    underruns: u64,
    overruns: u64,
    rx_frames: u64,
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
    let supported = chosen.ok_or("device has no 48 kHz output with enough channels (set it to 48000 Hz in Windows Sound settings)")?;
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
    let needle = args
        .iter()
        .position(|a| a == "--device")
        .and_then(|i| args.get(i + 1))
        .cloned()
        .unwrap_or_else(|| "cable input".to_string());
    let channels: usize = args
        .iter()
        .position(|a| a == "--channels")
        .and_then(|i| args.get(i + 1))
        .and_then(|v| v.parse().ok())
        .unwrap_or(2);

    let device = match find_device(&host, &needle) {
        Some(d) => d,
        None => {
            eprintln!("Output device containing '{needle}' not found. Run with --list.");
            return;
        }
    };
    println!("Output device: {}", device.name().unwrap_or_default());

    if tone {
        let mut n: u64 = 0;
        let _stream = match build_stream(&device, channels, move |data, out_ch| {
            for frame in data.chunks_mut(out_ch) {
                for (c, s) in frame.iter_mut().enumerate() {
                    *s = if c < channels {
                        let f = 1000.0 * (c as f64 + 1.0);
                        (0.1 * (2.0 * std::f64::consts::PI * f * n as f64 / RATE as f64).sin()) as f32
                    } else {
                        0.0
                    };
                }
                n = (n + 1) % RATE as u64;
            }
        }) {
            Ok(s) => s,
            Err(e) => {
                eprintln!("{e}");
                return;
            }
        };
        println!("Test tone: CH1 = 1 kHz, CH2 = 2 kHz (CH3 = 3 kHz ...). Press Ctrl+C to stop.");
        let _ = tokio::signal::ctrl_c().await;
        return;
    }

    // ---- Dante receive -> ring buffer -> audio device ----
    let shared = Arc::new(Mutex::new(Shared {
        q: VecDeque::new(),
        ch: channels,
        started: false,
        frac: 0.0,
        underruns: 0,
        overruns: 0,
        rx_frames: 0,
    }));

    let shared_out = shared.clone();
    let _stream = match build_stream(&device, channels, move |data, out_ch| {
        let mut st = shared_out.lock().unwrap();
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
    }) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("{e}");
            return;
        }
    };

    let shared_stats = shared.clone();
    std::thread::spawn(move || loop {
        std::thread::sleep(Duration::from_secs(1));
        let st = shared_stats.lock().unwrap();
        println!(
            "buffer {:.0} ms | rx frames {} | underruns {} | overruns {}",
            (st.q.len() / st.ch) as f64 * 1000.0 / RATE as f64,
            st.rx_frames,
            st.underruns,
            st.overruns
        );
    });

    let mut settings = Settings::new("DanteBridge", "DanteBridge", None, &Default::default());
    settings.make_rx_channels(channels);
    settings.make_tx_channels(0);

    let shared_rx = shared.clone();
    let rx_callback = move |samples_count: usize, chans: &Vec<Vec<Sample>>| {
        let mut st = shared_rx.lock().unwrap();
        let n = st.ch;
        if chans.len() < n {
            return;
        }
        for si in 0..samples_count {
            for c in 0..n {
                st.q.push_back((chans[c][si] as f64 / 2147483648.0) as f32);
            }
        }
        st.rx_frames += samples_count as u64;
        let frames = st.q.len() / n;
        if frames > TARGET_FRAMES * 4 {
            let drop = (frames - TARGET_FRAMES) * n;
            st.q.drain(..drop);
            st.overruns += 1;
        }
    };

    let mut server = DeviceServer::start(settings).await;
    server.receive_with_callback(Box::new(rx_callback)).await;

    let _ = tokio::signal::ctrl_c().await;
    server.shutdown().await;
}
