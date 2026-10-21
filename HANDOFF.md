# HANDOFF - read this first (for any engineer or AI assistant continuing the project)

## 1. Goal
Let a Windows 10 laptop act like "Dante Virtual Soundcard", **without Audinate software**:
a Yamaha **QL5 / CL5** (Dante card: Dante-MY16-AUD) <-> laptop over one Ethernet cable,
as a **4 in / 4 out ASIO device** that **Smaart** and **Open Sound Meter (OSM)** can use.
Built on **Inferno** (https://github.com/teodly/inferno), an open-source reverse-engineered Dante stack (Rust, Linux).
The owner has no programming experience and no build tools: **everything is built on GitHub Actions** (windows-latest);
the owner downloads artifacts, runs them, and pastes logs back. Owner speaks Arabic; keep instructions short, numbered, and Windows-10 specific.

## 2. Repository map (this repo = Gj486/inferno-win)
| Path | What |
|---|---|
| `.github/workflows/inferno.yml` | Clones Inferno into `inferno-src/`, applies our patches, builds `dante_bridge.exe` |
| `.github/workflows/asio.yml` | CMake build of the ASIO drivers (Stage A tones / Stage B real) |
| `windows/usrvclock_lib.rs`, `usrvclock_Cargo.toml` | Replaces Inferno's Linux-only `usrvclock-rs`: a **PTPv1 follower** (Dante clock) |
| `bridge/dante_bridge_main.rs` | The app: egui window + engine (RX, TX, ASIO bridge). Copied to `inferno-src/dante_bridge/src/main.rs` |
| `bridge/shm.rs` | Shared-memory ring buffers (also re-implemented in C++ in `asio/dante_asio.cpp`) |
| `bridge/dante_bridge.toml` | Settings file (also written by the GUI) |
| `asio/dante_asio.cpp` | **Stage B** ASIO driver (C++, COM, x64): reads/writes shared memory |
| `asio/stageA/dante_asio_stageA.cpp` | **Stage A** ASIO driver: 4 test tones on the INPUTS, no bridge needed |
| `asio/install.bat`, `uninstall.bat` | `regsvr32` register/unregister (run as Administrator) |

Workflow patches applied to the cloned Inferno (see `inferno.yml`): replace `usrvclock-rs/src/lib.rs`+`Cargo.toml`,
replace `inferno_aoip/src/util/os.rs` (thread priority on non-unix), add crate `dante_bridge` to the workspace,
remove `alsa_pcm_inferno` from the workspace (ALSA `links` conflict with `cpal`).

## 3. Architecture
```
 QL5/CL5 --Dante(UDP RTP, PTPv1)--> inferno_aoip --RX callback--> [A] Windows output devices (cpal, e.g. VB-Matrix In n)
                                                              \-> [B] shm "In" ring  -> dante_asio.dll -> Smaart/OSM (ASIO inputs)
 Smaart/OSM (ASIO outputs) -> dante_asio.dll -> shm "Out" ring -> asio_reader thread -> TxWriter -> inferno external TX ring -> Dante
 Windows input devices (cpal) -> TxWriter -> inferno external TX ring -> Dante            [alternative to ASIO]
```
Shared memory: memory-mapped file `%PUBLIC%\DanteBridge\asio_shm.bin`; header 128 B (in_write@16, out_write@24, bridge_beat@32,
driver_beat@40), then two rings of 32768 frames x 4 ch x f32. Writer publishes the write position with Release; readers use Acquire.

## 4. Facts learned about Inferno (verified in its source)
- Audio sample type `Sample = i32`, 24-bit left-justified (+-2^31). Convert: `s as f32 / 2147483648.0`.
- RX: `DeviceServer::receive_with_callback(Box<dyn FnMut(usize, &Vec<Vec<Sample>>)+Send>)`.
- TX: `transmit_from_external_buffer(Vec<ExternalBufferParameters<Sample>>, oneshot::Receiver<Clock>, Arc<AtomicUsize>, Option<..>)`.
  The transmitter reads ring index `(media_time - start_time - tx_latency)`; rings must be power-of-two sized; reads are
  **unconditional** (stale data is re-sent if nobody writes) -> always keep writing (silence when idle).
- Clock: Inferno needs a `usrvclock::ClockOverlay {clock_id,last_sync,shift,freq_scale}`; `now_ns = local + shift + ...`.
  Until the first overlay arrives, the media clock is "not ready". **If the clock does not match the Dante clock, RX packets are
  silently discarded (meters stay at -90 dB).**
- Settings keys (env prefix `INFERNO_`): `NAME`, `ALT_PORT`, `TX_LATENCY_NS`, `BIND_IP`...
- Ports: UDP 4440, 4455, 8700, 8800, 5353 (mDNS); PTPv1 multicast 224.0.1.129 UDP 319 (event) / 320 (general).
  Dante Virtual Soundcard's `conmon_cmm_service` may hold 8700/8800 -> we use `alt_port = 14400`.
- Windows Firewall: allow the **program** (any UDP port) - audio flows come from OS-assigned ports (`bridge/firewall.bat`).
- **mDNS on Windows**: a socket bound to the adapter's unicast IP never receives multicast replies; searchfire must bind `0.0.0.0`
  (patched in `inferno.yml`). Symptom of the bug: `resolving [...]` followed by `TimedOut`.
- **PTPv1 (verified with real QL5 packets)**: header 40 bytes; Sync (control 0) origin time at 40 (sec) / 44 (ns); Follow_Up (control 2, port 320)
  has the *precise* time at 44 (sec) / 48 (ns) and the associated Sync sequence at 42. The Sync's own stamp is ~325 us early -> use Follow_Up.
- The Dante clock drifts ~50 ppm against the PC clock, so every consumer of the clock (TX writer) must refresh the overlay continuously.
- Inferno derives the device identity from the bound IP (state dir `...\inferno_aoip\0000a9fe3a13` = 169.254.58.19): if the IP changes,
  Dante Controller sees a *new* device and subscriptions are lost -> give the laptop's Ethernet a fixed address.
- QL5 sends `cmc_server` opcode 0x3010 every ~10 s (status info). Inferno logs it as ERROR; we silence that module in the log filter.
- Sample rate of the console must be 48 kHz.

## 5. Current status (update this table in every change)
| Item | Status |
|---|---|
| Inferno builds on Windows (CI) | OK |
| GUI app, tone test, VB-Matrix paths (no mixer) | OK, tested in OSM |
| Device visible in Dante Controller, subscription | OK |
| PTP follower (Sync + Follow_Up), clock "following" indicator | OK in logs (offsets verified) |
| mDNS channel resolution (searchfire patch) | OK (`AdvertisedChannel` resolved) |
| **RX audio from QL5** | started ("receiving data"); **10-minute stability test still to do** |
| TX writer follows the live clock | implemented, untested with hardware |
| TX to Dante | untested with hardware |
| ASIO Stage A (tones) / Stage B | CI build fixed (export linkage); not yet installed/tested |
| Unit tests | PTP parsers/filter only (`cargo test -p usrvclock`, run in CI) |

## 6. Next steps (in order; do not skip ahead)
1. RX: cancel stale subscriptions in Dante Controller (old device `...2c5fa8`), subscribe 1 channel, run 10 min, watch underruns/overruns.
2. Give the laptop Ethernet a fixed IP; run `firewall.bat` as Administrator once.
3. TX test: tone from OSM -> QL5 channel (start with `tx_devices` via VB-Matrix, then ASIO).
4. ASIO Stage A in OSM (4 tones), then Stage B with the bridge, then Smaart.
5. Known small items: `unknown opcode 0x3010` (ignored), mutex in the RX audio callback (see CLEAN_CODE_REVIEW), no freq_scale estimate in the clock.
6. Clean-code refactor only after steps 1-4 pass.

## 7. Rules for contributors (humans and AIs)
- **Mixer safety**: speakers/PA muted, scene backed up, direct cable, QL5 not clock-follower of our app, start RX-only with ONE channel.
- Never claim something "works" unless the owner ran it; label code as untested. Cannot compile locally? Say so.
- **One change per commit**; never rewrite whole files to fix one bug; give diffs. Keep the status table above current.
- Do not mix refactoring with debugging. Do not add features before the previous step passes its acceptance test.
- Keep Windows x64 only, 48 kHz only, 4 channels, until the basics are stable.
- Legal: Inferno is GPL/AGPL; Dante is proprietary/patented (Audinate). Personal use only unless a lawyer says otherwise.
- Don't reverse-engineer Audinate binaries; analysing our own network traffic (Wireshark) is fine.

## 7b. Test protocol (copy results into the issue/PR)
Send: screenshot of the app window, `dante_bridge.log` (first 200 lines + any WARN/ERROR), Dante Controller screenshots
(Routing + Clock Status of both devices), and `ipconfig` output.
