//! Windows replacement for usrvclock.
//! There is no PTP daemon here: the "media clock" is the local PC clock
//! (wall-clock time at startup + monotonic elapsed time). It is NOT locked
//! to the Dante clock, so long recordings will drift. For testing only.

use std::{path::PathBuf, sync::OnceLock, time::{Duration, Instant, SystemTime, UNIX_EPOCH}};
use custom_error::custom_error;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClockOverlay {
    pub clock_id: i64,
    pub last_sync: i64,
    pub shift: i64,
    pub freq_scale: f64,
}

custom_error! { pub OverlayReceiveError
    PacketTooShort = "packet too short",
    UnexpectedData = "unexpected data in packet",
    UnsupportedMajorVersion = "unsupported major version",
    InvalidFlags = "invalid flags (client too old?)",
}

pub const DEFAULT_SERVER_SOCKET_PATH: &str = "/tmp/ptp-usrvclock";

fn local_now_ns() -> i64 {
    static BASE: OnceLock<(Instant, i64)> = OnceLock::new();
    let (start, base_ns) = BASE.get_or_init(|| {
        let wall = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_nanos() as i64)
            .unwrap_or(0);
        (Instant::now(), wall)
    });
    base_ns.wrapping_add(start.elapsed().as_nanos() as i64)
}

impl ClockOverlay {
    pub fn underlying_to_overlay_ns(&self, timestamp: i64) -> i64 {
        let elapsed = timestamp.wrapping_sub(self.last_sync);
        let correction = ((elapsed as f64) * self.freq_scale).round() as i64;
        timestamp.wrapping_add(self.shift).wrapping_add(correction)
    }

    pub fn now_underlying_ns(&self) -> i64 {
        local_now_ns()
    }

    pub fn now_ns(&self) -> i64 {
        self.underlying_to_overlay_ns(self.now_underlying_ns())
    }

    pub fn freq_scale_including_hw(&self) -> f64 {
        self.freq_scale
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub struct SafeTimestamp {
    pub nanos: i64,
    pub estimated: bool,
}

impl SafeTimestamp {
    pub fn precise_ns(&self) -> Option<i64> {
        if !self.estimated { Some(self.nanos) } else { None }
    }
}

/// On Windows the underlying clock is monotonic, so no jump guard is needed.
pub struct SafeClock {}

impl SafeClock {
    pub fn new(_tolerance: f64, _timeout_ns: i64) -> Self {
        Self {}
    }

    pub fn now(&mut self, overlay: &ClockOverlay) -> SafeTimestamp {
        SafeTimestamp { nanos: overlay.now_ns(), estimated: false }
    }
}

#[cfg(feature = "tokio")]
pub struct AsyncClient {
    sender: tokio::sync::watch::Sender<Option<ClockOverlay>>,
}

#[cfg(feature = "tokio")]
impl AsyncClient {
    pub fn start(_path: PathBuf, _error_handler: Box<dyn FnMut(OverlayReceiveError) + Send>) -> Self {
        let overlay = ClockOverlay { clock_id: 0, last_sync: 0, shift: 0, freq_scale: 0.0 };
        let sender = tokio::sync::watch::Sender::new(Some(overlay));
        Self { sender }
    }

    pub async fn stop(self) -> Result<(), tokio::task::JoinError> {
        Ok(())
    }

    pub fn subscribe(&self) -> tokio::sync::watch::Receiver<Option<ClockOverlay>> {
        self.sender.subscribe()
    }
}

#[allow(dead_code)]
fn _unused(_: Duration) {}
