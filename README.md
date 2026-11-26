# Ball detection

## Current work: CV collector in program slot 5

The separate **`vex-cv` / `cv-collector`** project uses camera coordinates to steer
and drive toward balls, then completes a measured pickup when the ball passes
into the intake blind spot. Motor rotation and heading feedback support movement
and area searching. The distance sensor is not used by this program.

The phone can send video through a **Cloudflare HTTPS page with QR pairing**;
Camo remains an alternative. Detection runs on the laptop, and the Brain still
connects to it by USB.

- [Slot 5 setup and architecture](CV_SLOT5_GUIDE.md)
- [Validation results and outstanding hardware tests](CV_SLOT5_VALIDATION.md)
- [Working together on this repository](CONTRIBUTING.md)

On a new Windows laptop, run `1 - Setup.cmd` first. To test phone video without
the robot, run `10 - Setup phone camera.cmd`, `11 - Pair phone camera.cmd`, then
`12 - Preview phone camera.cmd`. Scan the pairing QR on the phone and tap
**Start camera**. The pairing launcher must stay open.

Before robot movement, calibrate the camera with `9 - Calibrate CV camera.cmd`,
install with `6 - Install CV slot 5.cmd`, and run `7 - Start CV slot 5.cmd`.
The installer refuses to overwrite an unrelated program in slot 5. Motion needs
a physical Touch LED press/release. **This version still needs a full firmware
build and physical pickup testing; see the validation notes.**

| Component | Main files | Role |
| --- | --- | --- |
| Slot 5 laptop CV | `cv_autonomy.py`, `cv_tracking.py`, `cv_serial.py` | Detection, target association, coordinate messages |
| Slot 5 Brain | `vex-cv/src/main.cpp`, `vex-cv/include/cv_control.h` | Motor control, searching, pickup completion, stop controls |
| Phone camera | `phone-web/`, `phone_camera.py`, `start_phone_camera.py` | Phone capture, authenticated frame transfer, Cloudflare QR pairing |
| Slot 4 integration | `vex-receiver/`, `detect_ball.py`, `serial_bridge.py` | Original distance-sensor navigation with optional CV confirmation |
| Original collector | `vex-receiver/reference/collector_v4_original.iqcpp` | Preserved source reference |

## Existing integration in program slot 4

**VEX integration:** See [VEX_SERIAL_SETUP.md](VEX_SERIAL_SETUP.md). Program slot 4
now uses **SENSOR + CV V5.1**: the original distance-sensor collector controls the
route, target positions, turns and intake drive-through. The camera only approves
or rejects each sensor candidate before forward collection. It sends no steering
or speed commands. Slot 1 and the original V4 source are untouched.

Run `5 - Update slot 4.cmd`, then `3 - Start VEX link.cmd`. Verify
`BALL_SERIAL_V5 DRIVE`. Tap Touch LED, keep still for calibration, then press and
release Touch LED again at the start prompt. No controller is needed.
If the camera is missing, V5.1 stays on WAIT FOR CAMERA / Run PC script 3 instead
of returning to IDLE. Connecting alone never starts motion; press and release
Touch LED after the camera is ready. The Python serial script must stay running.
Brain Left selects the original sensor-only mode; the previous camera-driving
demo is no longer selectable.

The robot faces a sensor candidate and briefly checks the camera (up to 1.5 s).
Two distinct fresh observations of a plausible ball in the central confirmation
band approve the original collection move. An unconfirmed object is skipped.
Losing sight of an approved ball near the intake does not cancel its drive-through.
Touch LED, Brain Check and Q stop the mission.

The preview now shows the Brain's real stage and last check result. SCAN_360 with
No check yet means it is scanning and no camera decision has completed yet. CV rejected candidate
means it did nominate one but the visual check failed. The preview also explains
out-of-zone, shape and ambiguous-box rejections. The saved model is unchanged;
serial confidence is restored to 0.50 (the standalone preview still uses 0.25).

Keep the phone aligned with the intake as the robot turns. The preview's orange
band must cover the object the sensor-selected turn is aiming at. The supplied
model is unchanged; confirmation is an additional filter, not a guarantee that
chair legs cannot be misclassified. Test with both balls and chair legs.

Windows camera preview with a trained ping-pong ball detector. The included
`downloaded_model/weights.pt` runs locally with Ultralytics. Camo Studio is
selected by default when its camera is available.

If Camo Studio has video but the detector is black, restart the detector. It now
tries Media Foundation first for Camo, then DirectShow, matching the camera by
name because each backend assigns different indices. It requests 640 x 480 at
30 fps and checks for a nonblank image before opening the Brain connection.
The terminal reports which method works. To force one for troubleshooting, use
`--camera-backend msmf` or `--camera-backend dshow`. Blank images during a serial
session send STOP and end the session; restart after restoring Camo's feed.

## Setup in VS Code

1. Open `Ball-detection.code-workspace` in VS Code.
2. Run `1 - Setup.cmd`, or use **Terminal > Run Task > Ball detection: setup**.
   Setup installs dependencies into `.venv` and runs an inference check without
   opening a camera. Internet access is required for package installation.
3. Open Camo Studio and confirm its preview, or connect a webcam.
4. Press **F5** with **Ball detector: camera** selected, or run
   `2 - Start detector.cmd`. Choose a camera in the terminal.
5. Press **Q** or **Esc** in the preview to stop.

Python 3.12 is the preferred setup version. The setup script uses the current
Python 3 installation if 3.12 is unavailable. If package installation reports
an unsupported Python version, install 64-bit Python 3.12, rename `.venv` to
`.venv-old`, and rerun setup.

## Model check

From a terminal in this folder:

```bat
.venv\Scripts\python.exe detect_ball.py --check
```

Success prints `Model inference passed` and the model's class names. This checks
loading and inference on a blank image; it does not measure detection accuracy.

## Refresh the model (optional)

The existing weights are included, so downloading them again is unnecessary.
`download_model.py` targets Roboflow workspace `ct-c-fwa4v`, project
`ping-pong-ball-detection-kjmcq-5fze6`, version 1. To refresh them, install the
Roboflow SDK in a separate environment so its OpenCV dependencies cannot affect
the detector's GUI:

```bat
py -3 -m venv .venv-download
.venv-download\Scripts\python.exe -m pip install roboflow
.venv-download\Scripts\python.exe download_model.py
```

Enter the private API key at the hidden prompt. It can also be supplied through
`ROBOFLOW_API_KEY`; never place a key in committed source files.

## GitHub

The repository includes source, VS Code configuration, and the supplied model
weights. Virtual environments, secrets, logs, and remote-desktop files are
ignored. Use a private repository unless you intend to share the code and model.

Repository: https://github.com/cc081808/Ball-detection

Work on separate branches and open pull requests into `main`; see
[CONTRIBUTING.md](CONTRIBUTING.md). If the repository is private, each teammate
needs collaborator access before cloning or pushing. Keep runtime pairing keys,
QR session files, logs, environments and compiled output out of commits.
