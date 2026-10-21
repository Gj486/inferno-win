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

/// Parses a PTPv1 Follow_Up (control == 2). It carries the precise transmit time of an
/// earlier Sync (the Sync's own timestamp is taken before sending and is ~0.3 ms early).
/// Layout: associated Sync sequenceId u16 @42, seconds u32 @44, nanoseconds u32 @48.
/// Returns (associated sequenceId, seconds, nanoseconds).
#[cfg(feature = "tokio")]
fn parse_follow_up(buf: &[u8]) -> Option<(u16, u32, u32)> {
    if buf.len() < 52 || u16::from_be_bytes([buf[0], buf[1]]) != 1 || buf[32] != 2 {
        return None;
    }
    Some((
        u16::from_be_bytes([buf[42], buf[43]]),
        u32::from_be_bytes([buf[44], buf[45], buf[46], buf[47]]),
        u32::from_be_bytes([buf[48], buf[49], buf[50], buf[51]]),
    ))
}

/// sequenceId of any PTPv1 message (header offset 30).
#[cfg(feature = "tokio")]
fn sequence_id(buf: &[u8]) -> u16 {
    u16::from_be_bytes([buf[30], buf[31]])
}

/// Keeps the last offset samples (ptp_time - local_time) and returns the filtered one.
/// Packet delay only makes (ptp - local) smaller, so the largest value in a short window
/// is the least-delayed (best) estimate.
#[cfg(feature = "tokio")]
struct OffsetFilter {
    window: std::collections::VecDeque<i64>,
    samples: u64,
    last_logged: Option<i64>,
}

#[cfg(feature = "tokio")]
impl OffsetFilter {
    const WINDOW: usize = 16;

    fn new() -> Self {
        Self { window: std::collections::VecDeque::new(), samples: 0, last_logged: None }
    }

    /// Adds a sample and returns (filtered shift, whether it is worth logging).
    fn add(&mut self, offset: i64) -> (i64, bool) {
        self.window.push_back(offset);
        if self.window.len() > Self::WINDOW {
            self.window.pop_front();
        }
        let shift = self.window.iter().copied().max().unwrap_or(offset);
        self.samples += 1;
        let log_it = self.samples <= 5
            || self.samples % 60 == 0
            || self.last_logged.map(|l| (l - shift).abs() > 500_000).unwrap_or(true);
        if log_it {
            self.last_logged = Some(shift);
        }
        (shift, log_it)
    }
}

#[cfg(feature = "tokio")]
fn publish_offset(
    sender: &tokio::sync::watch::Sender<Option<ClockOverlay>>,
    filter: &mut OffsetFilter,
    source: &str,
    ptp_ns: i64,
    local_ns: i64,
) {
    let (shift, log_it) = filter.add(ptp_ns.wrapping_sub(local_ns));
    if log_it {
        info!(
            "[usrvclock] {source} #{}: ptp={ptp_ns} local={local_ns} shift={} us",
            filter.samples,
            shift / 1000
        );
    }
    let _ = sender.send(Some(ClockOverlay { clock_id: 0, last_sync: local_ns, shift, freq_scale: 0.0 }));
}

#[cfg(feature = "tokio")]
async fn ptp_loop(sender: tokio::sync::watch::Sender<Option<ClockOverlay>>) -> Result<(), String> {
    use std::collections::VecDeque;
    use std::net::Ipv4Addr;

    /// Sync messages waiting for their Follow_Up.
    const MAX_PENDING: usize = 8;
    /// Without any Follow_Up after this many Syncs, fall back to the Sync's own timestamp.
    const FALLBACK_AFTER_SYNCS: u64 = 3;

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
    let mut syncs: u64 = 0;
    let mut follow_ups: u64 = 0;
    // (sequenceId, local receive time of the Sync, Sync's own origin timestamp)
    let mut pending: VecDeque<(u16, i64, i64)> = VecDeque::new();
    let mut filter = OffsetFilter::new();

    loop {
        tokio::select! {
            r = sock320.recv_from(&mut buf320) => {
                let (n, from) = match r {
                    Ok(x) => x,
                    Err(e) => { warn!("[usrvclock] PTP recv error (320): {e}"); continue; }
                };
                if raw_logged_320 < 10 {
                    raw_logged_320 += 1;
                    info!("[usrvclock] port 320 packet from {from}, {n} bytes: {}", hex(&buf320[..n]));
                }
                if let Some((seq, sec, nsec)) = parse_follow_up(&buf320[..n]) {
                    if let Some(i) = pending.iter().position(|p| p.0 == seq) {
                        if let Some((_, local_ns, _)) = pending.remove(i) {
                            follow_ups += 1;
                            let precise_ns = (sec as i64) * 1_000_000_000i64 + (nsec as i64);
                            publish_offset(&sender, &mut filter, "Follow_Up", precise_ns, local_ns);
                        }
                    }
                }
            }
            r = sock319.recv_from(&mut buf319) => {
                let (n, from) = match r {
                    Ok(x) => x,
                    Err(e) => { warn!("[usrvclock] PTP recv error (319): {e}"); continue; }
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
                syncs += 1;
                let sync_ns = (sec as i64) * 1_000_000_000i64 + (nsec as i64);
                pending.push_back((sequence_id(&buf319[..n]), local_ns, sync_ns));
                if pending.len() > MAX_PENDING {
                    pending.pop_front();
                }
                if follow_ups == 0 && syncs >= FALLBACK_AFTER_SYNCS {
                    // this master sends no Follow_Up: use the Sync's own timestamp
                    publish_offset(&sender, &mut filter, "Sync (no Follow_Up seen)", sync_ns, local_ns);
                }
            }
        }
    }
}

#[allow(dead_code)]
fn _unused(_: Duration) {}

#[cfg(all(test, feature = "tokio"))]
mod tests {
    use super::*;

    /// First 64 bytes of a Sync captured from a Yamaha QL5 (sequence 0x25, 19078.243674519 s).
    const SYNC: [u8; 64] = [
        0x00, 0x01, 0x00, 0x01, 0x5f, 0x44, 0x46, 0x4c, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x1d, 0xc1, 0x12, 0x6d, 0x4e, 0x00, 0x01, 0x00, 0x25,
        0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4a, 0x86, 0x0e, 0x86, 0x2d, 0x97,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x1d, 0xc1, 0x12, 0x6d, 0x4e, 0x00, 0x00, 0x00, 0x25,
    ];

    /// The matching Follow_Up (precise transmit time 19078.243999592 s, 325 us after the Sync's own stamp).
    const FOLLOW_UP: [u8; 52] = [
        0x00, 0x01, 0x00, 0x01, 0x5f, 0x44, 0x46, 0x4c, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x02, 0x01, 0x00, 0x1d, 0xc1, 0x12, 0x6d, 0x4e, 0x00, 0x01, 0x00, 0x25,
        0x02, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25, 0x00, 0x00, 0x4a, 0x86,
        0x0e, 0x8b, 0x23, 0x68,
    ];

    #[test]
    fn parses_sync() {
        assert_eq!(parse_ptp_v1(&SYNC), Some((0, 19078, 243_674_519)));
        assert_eq!(sequence_id(&SYNC), 0x25);
    }

    #[test]
    fn parses_follow_up() {
        assert_eq!(parse_follow_up(&FOLLOW_UP), Some((0x25, 19078, 243_999_592)));
    }

    #[test]
    fn follow_up_is_not_a_sync_and_vice_versa() {
        assert_eq!(parse_ptp_v1(&FOLLOW_UP).map(|p| p.0), Some(2));
        assert!(parse_follow_up(&SYNC).is_none());
    }

    #[test]
    fn rejects_short_packets_and_wrong_version() {
        assert!(parse_ptp_v1(&SYNC[..40]).is_none());
        assert!(parse_follow_up(&FOLLOW_UP[..50]).is_none());
        let mut bad = SYNC;
        bad[1] = 2; // PTP version 2
        assert!(parse_ptp_v1(&bad).is_none());
    }

    #[test]
    fn filter_keeps_the_least_delayed_sample() {
        let mut f = OffsetFilter::new();
        f.add(-1000);
        f.add(-400);
        let (shift, _) = f.add(-900);
        assert_eq!(shift, -400);
    }

    #[test]
    fn filter_forgets_old_samples() {
        let mut f = OffsetFilter::new();
        f.add(5000);
        for _ in 0..OffsetFilter::WINDOW {
            f.add(100);
        }
        let (shift, _) = f.add(100);
        assert_eq!(shift, 100);
    }

    #[test]
    fn overlay_adds_shift() {
        let o = ClockOverlay { clock_id: 0, last_sync: 0, shift: 1_000, freq_scale: 0.0 };
        assert_eq!(o.underlying_to_overlay_ns(5_000), 6_000);
    }
}
