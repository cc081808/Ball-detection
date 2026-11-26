# VEX serial integration test

The laptop detects balls and sends L (left), R (right), F (forward), or S (stop).
The receiver is a separate IQ2 C++ project in `vex-receiver` named `ball-serial`.
It targets **slot 4**, confirmed empty by the owner. Preserve every other slot,
including the latest ping-pong collector and manual-drive program.

## First test: no motor movement

1. Turn on the Brain and connect it directly to the laptop using a USB data cable.
2. Run `4 - Open VEX receiver.cmd`. This opens a separate VS Code window for
   `vex-receiver`. Keep the detector open in its original VS Code window.
3. In the receiver window, check that the selected project is `ball-serial` and
   the destination is **slot 4**. Build and download using the VEX extension.
   The extension must not download the old collector project by mistake.
   `Build receiver.cmd` is an alternative local build command; it never downloads.
4. Run `ball-serial` from slot 4 on the Brain. It displays TEST and Motors disabled.
5. Close/disconnect any VEX User Terminal or serial monitor so Python can use the
   user port. The receiver project disables the extension's user terminal by default.
6. Open Camo Studio with its live preview, then run `3 - Start VEX link.cmd` in
   the detector folder. Select the Camo camera. A unique VEX User Port is selected
   automatically; otherwise choose its number from the displayed COM port list.
7. Move a ball left, centre, and right. The preview and Brain screen should show
   L, F, and R. Remove the ball: S. Motors remain disabled throughout this test.
8. Press Q/Esc in the camera preview to stop. The Brain also stops motion commands
   after 300 ms without a valid received command. F-Up, the bumper, or the Touch LED
   ends the receiver program and requires restarting it.

The Brain was enumerated as COM7 (communications/download) and COM8 (user) during
setup. Port numbers can change. Python uses the **user** port, not communications.

## Commands in the detector terminal

```bat
.venv\Scripts\python.exe detect_ball.py --list-ports
.venv\Scripts\python.exe detect_ball.py --serial
.venv\Scripts\python.exe detect_ball.py --port COM8
```

The ordinary `2 - Start detector.cmd` remains camera-only. The existing model,
Roboflow downloader, collector source, and installed robot programs are preserved.

## After the display-only test passes

The compiled default is `ENABLE_MOTORS = false` in `vex-receiver/include/config.h`.
Changing it to true and rebuilding/downloading slot 4 enables a deliberately slow
approach test. Motion also requires a connected VEX controller with **E-Up held**.
Releasing E-Up stops drive and intake. F-Up, bumper, or Touch LED ends the program.

Ports and directions match the existing collector's configuration: left motor
12 normal, right motor 8 reversed, intake 11 reversed, bumper 5, Touch LED 4.
Turning uses 10%, forward uses 15%, and intake runs at 100% only while approaching.
These values are in config.h. This is an integration prototype, not the original
distance-sensor autonomous collector and not a complete collection strategy.

The camera must face forward on the robot. If the incoming video is mirrored,
use `--mirror-steering`; verify steering direction before a floor test.

## Protocol and failure handling

Python first sends `?`; only the new receiver answers `BALL_SERIAL_V1 TEST` or
`BALL_SERIAL_V1 DRIVE`. An unidentified receiver receives no movement commands.
Each command requires an `ACK L/R/F/S` reply before another frame is sent. ACK
confirms receipt, not physical motor movement. There is never a separate heartbeat
that keeps an old camera command alive. Low frame rate can therefore cause stops.

The receiver uses a serial-reading thread and a separate 20 ms control loop, so
waiting for a serial byte does not stop the watchdog or physical stop checks.
The laptop uses a 50% detection confidence threshold for serial mode and a centre
band of +/-15% of the image half-width. No detection or invalid coordinates send S.

Tests: `.venv\Scripts\python.exe -m unittest discover -s tests -v`.
Model-only check: `.venv\Scripts\python.exe detect_ball.py --check`.
