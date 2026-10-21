[README.md](https://github.com/user-attachments/files/33770670/README.md)
# inferno-win

Windows 10 bridge between a Yamaha QL5/CL5 (Dante) and measurement software
(Smaart, Open Sound Meter): a free "Dante Virtual Soundcard"-like tool built on
[Inferno](https://github.com/teodly/inferno) (reverse-engineered Dante, GPL/AGPL).

**Status:** experimental. Receiving audio from the console works in tests; transmit and the ASIO
driver are not yet verified on hardware. Read `HANDOFF.md` first (architecture, facts, next steps).

## Build (GitHub Actions, windows-latest)
- `.github/workflows/inferno.yml` -> artifact **result**: `dante_bridge.exe`, `firewall.bat`, `tests.txt`
- `.github/workflows/asio.yml` -> artifact **asio**: `stageA/` (test tones) and `stageB/` (real driver) with `install.bat`

## Use
1. Unzip **result** to e.g. `C:\inferno`, run `firewall.bat` as Administrator (once).
2. Run `dante_bridge.exe`, pick the Ethernet adapter in *Settings*, press *Start*.
3. In Dante Controller subscribe DanteBridge RX channels to the console's TX channels.
4. Mixer safety: PA muted, scene backed up, direct cable, start with ONE channel.

## Layout
`bridge/` app (Rust, egui) | `windows/` Windows clock client (PTPv1 follower) | `asio/` ASIO driver (C++) | `docs/` reviews
