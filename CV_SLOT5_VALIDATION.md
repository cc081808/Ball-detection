# Slot 5 development checks — November 26, 2026

Implemented a separate `vex-cv` receiver, coordinate/tracking application, Cloudflare Tunnel phone camera page, QR pairing launcher, calibration workflow, and slot-specific installer. During that implementation, all pre-existing project files were compared with SHA-256 hashes recorded beforehand; none changed. Subsequent repository handoff work updates the README, collaboration instructions, and project descriptions without changing the original collector source.

## Passed

- 65 existing Python tests, 18 new slot 5 tracking/protocol/installer tests, and 7 phone-camera tests: **90 passing tests across the relevant runs**.
- Local HTTP integration: unauthorized access rejected, phone capture request issued, JPEG uploaded, local detector received it with the original timestamp, phone key could not retrieve frames.
- Stale and repeated uploads rejected; only one phone can supply frames; invalid/oversized images rejected.
- Generated QR code decoded back to the exact intended HTTPS URL and pairing fragment.
- Original supplied single-class ball model loaded and completed CPU inference.
- Python syntax checks and phone JavaScript syntax check.
- Current robot source compiled to an ARM object with the installed IQ2 SDK.
- Native C++ simulation source compiled with MSVC, but its executable could not run (see below). No simulation assertions are claimed as passed.

Installer unit tests use fake processes and ports. Their simulated “downloaded” messages do not represent a real Brain download.

## Blocked or not yet tested

- The Brain was unplugged. No program was downloaded or physically run; actual slot contents have not been inspected.
- The VEX GNU linker could not start in this session: Windows error 623. No final `.bin` was produced here. `6 - Install CV slot 5.cmd` rebuilds and verifies the result before any download; it does not use a stale binary.
- Windows Device Guard blocked the newly built native C++ test executable. The provided motion simulations have therefore not executed in this session.
- Windows would not launch the `winget` application alias from this session. Cloudflare was not installed, and no public tunnel was started. Run `10 - Setup phone camera.cmd`, then `11 - Pair phone camera.cmd` from the normal Windows session to perform setup and generate a live QR code.
- A real phone browser, Cloudflare round-trip latency, physical steering direction, encoder/heading behaviour, camera alignment, pickup distance and collection success still need testing.

The phone may be tested now, without the Brain: setup, pair, then run `12 - Preview phone camera.cmd`. The robot must remain unarmed until mount calibration is saved, the slot 5 receiver is running, and fresh coordinates are arriving. Start motion with a physical Touch LED press/release.
