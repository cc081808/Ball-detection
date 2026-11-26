# Working together

Repository: https://github.com/cc081808/Ball-detection

## Get the project

```bash
git clone https://github.com/cc081808/Ball-detection.git
cd Ball-detection
git switch -c feature/your-task
```

On Windows, run `1 - Setup.cmd` to install the laptop dependencies. The supplied
ball-model weights are included. Use VS Code with the VEX extension for the
Brain projects; open `vex-cv` for slot 5 or `vex-receiver` for slot 4.

For an existing checkout, start new work from an up-to-date, clean `main`:

```bash
git switch main
git pull --ff-only origin main
git switch -c feature/your-task
```

Commit or stash your own unfinished changes before switching branches. Do not
discard another person's work or force-push `main`.

## Divide the work

| Area | Files |
| --- | --- |
| CV accuracy and target tracking | `cv_autonomy.py`, `cv_tracking.py`, `tests/test_cv_slot5.py` |
| Robot movement and search | `vex-cv/include/cv_control.h`, `vex-cv/src/main.cpp`, `tests/cv_control_tests.cpp` |
| Phone camera and pairing | `phone-web/`, `phone_camera.py`, `start_phone_camera.py`, `tests/test_phone_camera.py` |
| Serial protocol | `cv_serial.py`, `vex-cv/include/cv_protocol.h` — coordinate changes on both sides |

The slot 5 project is separate from the original collector and slot 4. Preserve
that separation. Do not retarget downloads to another program slot. Coordinate
physical robot tests so only one laptop application owns the USB user port.

Calibration depends on the actual camera mount. The committed JSON settings
start uncalibrated. For a personal copy, copy `cv_slot5_phone.json` to
`.runtime/my-camera.json` and launch:

```bat
.venv\Scripts\python.exe cv_autonomy.py --camera phone --config .runtime\my-camera.json --calibrate
```

Do not commit pairing keys, QR sessions, API keys, `.runtime`, virtual
environments, build output or an untested claim that the robot works physically.

## Check and share changes

```bat
.venv\Scripts\python.exe -m unittest discover -s tests -v
.venv\Scripts\python.exe cv_autonomy.py --check
```

With the Visual Studio C++ build tools installed, `tests\run_cv_control.cmd`
builds and runs simulations of the actual robot controller. With the VEX SDK,
`vex-cv\Build CV.cmd` builds firmware without downloading it. Record any checks
that cannot run on your machine. Current limitations are in
[CV_SLOT5_VALIDATION.md](CV_SLOT5_VALIDATION.md).

Commit only files for your task, then push your branch:

```bash
git add path/to/changed-file
git commit -m "Describe the change"
git push -u origin feature/your-task
```

Open a GitHub pull request into `main`. Describe the behaviour change, software
tests, and any physical robot observations. After the PR is merged, teammates
can pull the updated `main` with `git pull --ff-only origin main`.
