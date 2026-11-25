# Ball detection

Windows camera preview with a trained ping-pong ball detector. The included
`downloaded_model/weights.pt` runs locally with Ultralytics. Camo Studio is
selected by default when its camera is available.

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

To publish from VS Code, open Source Control, choose **Publish Branch** or
**Publish to GitHub**, sign in if requested, and choose a private repository
named `Ball-detection`.
