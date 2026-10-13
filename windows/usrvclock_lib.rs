//! Windows replacement for usrvclock.
//! Instead of a Unix-socket PTP daemon, this client listens to PTPv1 Sync
//! messages broadcast by the Dante clock Leader (e.g. Yamaha QL5) on the
//! multicast group 224.0.1.129:319, and uses them to align Inferno's media
//! clock with the Dante network clock. Without this alignment, received RTP
//! packets are timestamped against a clock that does not match Dante's, and
//! Inferno silently discards them (all meters stay at -90 dB).
//!
//! PTPv1 (IEEE 1588-2002) message layout used here:
//!   byte  0      : versionPTP << 4 | versionNetwork   (0x1X for PTPv1)
//!   bytes 2..17  : subdomain                          ("DEFAULT" for Dante)
//!   byte  18     : messageType                        (0x00 = Sync)
//!   bytes 34..37 : seconds                            (big-endian u32)
//!   bytes 38..41 : nanoseconds                        (big-endian u32)

use std::{path::PathBuf, sync::OnceLock, time::{Duration, Instant, SystemTime, UNIX_EPOCH}};
use custom_error::custom_error;
use log::{info, warn};

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
        let sender = tokio::sync::watch::Sender::new(None);
        let s = sender.clone();
        match tokio::runtime::Handle::try_current() {
            Ok(handle) => {
                handle.spawn(async move {
                    if let Err(e) = ptp_loop(s).await {
                        warn!("[usrvclock] PTP loop ended: {e}");
                    }
                });
            }
            Err(_) => {
                warn!("[usrvclock] no tokio runtime; PTP client not started");
            }
        }
        Self { sender }
    }

    pub async fn stop(self) -> Result<(), tokio::task::JoinError> {
        Ok(())
    }

    pub fn subscribe(&self) -> tokio::sync::watch::Receiver<Option<ClockOverlay>> {
        self.sender.subscribe()
    }
}

#[cfg(feature = "tokio")]
async fn ptp_loop(sender: tokio::sync::watch::Sender<Option<ClockOverlay>>) -> Result<(), String> {
    use std::net::{Ipv4Addr, SocketAddrV4};
    use tokio::net::UdpSocket;

    // PTPv1 event multicast group and port (IEEE 1588-2002).
    const PTP_EVENT_MCAST: Ipv4Addr = Ipv4Addr::new(224, 0, 1, 129);
    const PTP_EVENT_PORT: u16 = 319;

    // PTPv1 message header constants.
    const MSG_TYPE_OFFSET: usize = 18;
    const MSG_SYNC: u8 = 0x00;
    const TS_SEC_OFFSET: usize = 34;
    const MIN_PACKET_LEN: usize = 42;

    let bind_addr = SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, PTP_EVENT_PORT);
    let sock = UdpSocket::bind(bind_addr)
        .await
        .map_err(|e| format!("bind {bind_addr}: {e}"))?;
    sock.join_multicast_v4(&PTP_EVENT_MCAST, &Ipv4Addr::UNSPECIFIED)
        .map_err(|e| format!("join {PTP_EVENT_MCAST}: {e}"))?;
    info!("[usrvclock] PTP client listening on {PTP_EVENT_MCAST}:{PTP_EVENT_PORT}");

    let mut buf = [0u8; 4096];
    let mut last_logged_shift: Option<i64> = None;
    let mut sync_count: u64 = 0;

    loop {
        let (n, from) = match sock.recv_from(&mut buf).await {
            Ok(x) => x,
            Err(e) => {
                warn!("[usrvclock] PTP recv error: {e}");
                continue;
            }
        };
        if n < MIN_PACKET_LEN {
            continue;
        }
        // versionPTP is the upper nibble of byte 0 (0x1X for PTPv1).
        let version_ptp = buf[0] >> 4;
        if version_ptp != 1 {
            continue;
        }
        if buf[MSG_TYPE_OFFSET] != MSG_SYNC {
            continue;
        }

        let sec = u32::from_be_bytes([
            buf[TS_SEC_OFFSET],
            buf[TS_SEC_OFFSET + 1],
            buf[TS_SEC_OFFSET + 2],
            buf[TS_SEC_OFFSET + 3],
        ]);
        let nsec = u32::from_be_bytes([
            buf[TS_SEC_OFFSET + 4],
            buf[TS_SEC_OFFSET + 5],
            buf[TS_SEC_OFFSET + 6],
            buf[TS_SEC_OFFSET + 7],
        ]);
        if sec == 0 && nsec == 0 {
            continue;
        }

        let ptp_ns = (sec as i64) * 1_000_000_000i64 + (nsec as i64);
        let local_ns = local_now_ns();
        let shift = ptp_ns.wrapping_sub(local_ns);

        sync_count += 1;
        // Log first 5 syncs, then every 100 (≈12 s at 8 sync/s), plus on
        // any shift jump above 1 ms so real drift is always visible.
        let should_log = sync_count <= 5
            || sync_count % 100 == 0
            || last_logged_shift
                .map(|l| (l - shift).abs() > 1_000_000)
                .unwrap_or(true);
        if should_log {
            info!(
                "[usrvclock] PTP sync #{sync_count} from {from}: ptp={ptp_ns} local={local_ns} shift={}us",
                shift / 1000
            );
            last_logged_shift = Some(shift);
        }

        let overlay = ClockOverlay {
            clock_id: 0,
            last_sync: local_ns,
            shift,
            freq_scale: 0.0,
        };
        let _ = sender.send(Some(overlay));
    }
}

#[allow(dead_code)]
fn _unused(_: Duration) {}
