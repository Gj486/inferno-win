//! Windows replacement for usrvclock.
//! Instead of a Unix-socket PTP daemon, this client listens to the PTPv1 (IEEE 1588-2002)
//! Sync messages that the Dante clock Leader (e.g. Yamaha QL5) multicasts to 224.0.1.129
//! (UDP 319/320) and uses them to align Inferno's media clock with the Dante network clock.
//! The first packets received are written to the log in hex, to verify the message layout.

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

/// Parses a PTPv1 (IEEE 1588-2002) message header as used by Dante.
/// Layout (40-byte header, big-endian):
///   0..2   versionPTP (u16, == 1)      2..4 versionNetwork
///   4..20  subdomain (16 bytes)        20   messageType (1 = event, 2 = general)
///   22..28 sourceUuid                  30..32 sequenceId
///   32     control (0 Sync, 1 Delay_Req, 2 Follow_Up, 3 Delay_Resp, 4 Management)
///   40..44 seconds                     44..48 nanoseconds   (Sync: originTimestamp)
/// Returns (control, seconds, nanoseconds).
#[cfg(feature = "tokio")]
fn parse_ptp_v1(buf: &[u8]) -> Option<(u8, u32, u32)> {
    if buf.len() < 48 {
        return None;
    }
    if u16::from_be_bytes([buf[0], buf[1]]) != 1 {
        return None;
    }
    let control = buf[32];
    let sec = u32::from_be_bytes([buf[40], buf[41], buf[42], buf[43]]);
    let nsec = u32::from_be_bytes([buf[44], buf[45], buf[46], buf[47]]);
    Some((control, sec, nsec))
}

#[cfg(feature = "tokio")]
fn hex(buf: &[u8]) -> String {
    let n = buf.len().min(64);
    buf[..n].iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(" ")
}

/// Opens a UDP socket on `port`, with address reuse (so other PTP software
/// on the PC does not block us) and joined to the PTP multicast group on the
/// network interface used for Dante (INFERNO_BIND_IP), not on the default route.
#[cfg(feature = "tokio")]
fn open_ptp_socket(port: u16, iface: std::net::Ipv4Addr) -> Result<tokio::net::UdpSocket, String> {
    use socket2::{Domain, Protocol, SockAddr, Socket, Type};
    use std::net::{Ipv4Addr, SocketAddrV4};
    const PTP_MCAST: Ipv4Addr = Ipv4Addr::new(224, 0, 1, 129);

    let s = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP)).map_err(|e| e.to_string())?;
    s.set_reuse_address(true).map_err(|e| e.to_string())?;
    s.set_nonblocking(true).map_err(|e| e.to_string())?;
    let addr = SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, port);
    s.bind(&SockAddr::from(addr)).map_err(|e| format!("bind {addr}: {e}"))?;
    s.join_multicast_v4(&PTP_MCAST, &iface)
        .map_err(|e| format!("join {PTP_MCAST} on {iface}: {e}"))?;
    tokio::net::UdpSocket::from_std(s.into()).map_err(|e| e.to_string())
}

#[cfg(feature = "tokio")]
async fn ptp_loop(sender: tokio::sync::watch::Sender<Option<ClockOverlay>>) -> Result<(), String> {
    use std::collections::VecDeque;
    use std::net::Ipv4Addr;

    let iface: Ipv4Addr = std::env::var("INFERNO_BIND_IP")
        .ok()
        .and_then(|s| s.trim().parse().ok())
        .unwrap_or(Ipv4Addr::UNSPECIFIED);
    if iface.is_unspecified() {
        warn!("[usrvclock] INFERNO_BIND_IP not set: joining PTP multicast on the default interface (may be Wi-Fi!)");
    }

    let sock319 = open_ptp_socket(319, iface)?; // event messages (Sync, Delay_Req)
    let sock320 = open_ptp_socket(320, iface)?; // general messages (Follow_Up, ...)
    info!("[usrvclock] listening for PTPv1 on 224.0.1.129 ports 319/320, interface {iface}");

    let mut buf319 = [0u8; 2048];
    let mut buf320 = [0u8; 2048];
    let mut raw_logged_319 = 0u32;
    let mut raw_logged_320 = 0u32;
    let mut sync_count: u64 = 0;
    let mut window: VecDeque<i64> = VecDeque::new();
    let mut last_logged: Option<i64> = None;

    loop {
        tokio::select! {
            r = sock320.recv_from(&mut buf320) => {
                if let Ok((n, from)) = r {
                    if raw_logged_320 < 10 {
                        raw_logged_320 += 1;
                        info!("[usrvclock] port 320 packet from {from}, {n} bytes: {}", hex(&buf320[..n]));
                    }
                }
            }
            r = sock319.recv_from(&mut buf319) => {
                let (n, from) = match r {
                    Ok(x) => x,
                    Err(e) => { warn!("[usrvclock] PTP recv error: {e}"); continue; }
                };
                let local_ns = local_now_ns(); // taken as early as possible
                if raw_logged_319 < 10 {
                    raw_logged_319 += 1;
                    info!("[usrvclock] port 319 packet from {from}, {n} bytes: {}", hex(&buf319[..n]));
                }
                let (control, sec, nsec) = match parse_ptp_v1(&buf319[..n]) {
                    Some(x) => x,
                    None => continue,
                };
                if control != 0 || (sec == 0 && nsec == 0) {
                    continue; // not a Sync with a usable timestamp
                }
                let ptp_ns = (sec as i64) * 1_000_000_000i64 + (nsec as i64);
                let offset = ptp_ns.wrapping_sub(local_ns);

                // Packet delay only makes (ptp - local) smaller, so the largest offset in a
                // short window is the least-delayed (best) estimate.
                window.push_back(offset);
                if window.len() > 16 {
                    window.pop_front();
                }
                let shift = window.iter().copied().max().unwrap_or(offset);

                sync_count += 1;
                let log_it = sync_count <= 5
                    || sync_count % 60 == 0
                    || last_logged.map(|l| (l - shift).abs() > 500_000).unwrap_or(true);
                if log_it {
                    info!("[usrvclock] Sync #{sync_count} from {from}: ptp={ptp_ns} local={local_ns} shift={} us", shift / 1000);
                    last_logged = Some(shift);
                }
                let _ = sender.send(Some(ClockOverlay { clock_id: 0, last_sync: local_ns, shift, freq_scale: 0.0 }));
            }
        }
    }
}

#[allow(dead_code)]
fn _unused(_: Duration) {}
