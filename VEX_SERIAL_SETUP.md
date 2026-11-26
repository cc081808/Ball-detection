# Distance-sensor collector with CV confirmation — slot 4

The original uploaded collector controls searching, target positions, turns,
forward motion and intake. The camera answers only: is there a plausible ping-pong
ball at the candidate the robot has aimed toward? It does not steer the robot.

## Install and run

1. Quit the detector with Q. In the normal Ball-detection cmd terminal, run:
   `cmd /c "5 - Update slot 4.cmd"`
2. Wait for build/download success. Only program slot 4 is targeted.
3. Run `cmd /c "3 - Start VEX link.cmd"`, select Camo, and verify
   **Connected: BALL_SERIAL_V5 DRIVE**. The new script rejects older steering firmware.
4. Use the original V4 start position: bottom-right corner facing along the area,
   rear about 150 mm from the bottom edge. Keep the robot still.
5. Tap Touch LED, wait for calibration, then at **SENSOR + CV V5.1 / Press TouchLED**
   press and release Touch LED again. No controller is needed.
6. Touch LED or Brain Check stops. Q/Esc/closing the detector sends mission stop.

Keep the Python serial script running; Camo Studio alone does not send observations
to the Brain. If it is missing, V5.1 waits on WAIT FOR CAMERA / Run PC script 3 with
motors stopped. It does not bounce back to IDLE or recalibrate. Once connected,
press and release Touch LED to start. A held button or connection alone cannot
start the robot. Check returns to the menu; Brain Left then selects the original
sensor-only program, which does not need the Python script.

The original area remains 6 m by 5 m, with five-minute/ten-ball limits. Allow
sufficient space and USB cable reach. The computer and phone supply confirmation;
the Brain runs the motion code. A count is an estimated collection based on a
completed drive-through, not an intake sensor proving a capture.

## Candidate check

| Stage | Behaviour |
|---|---|
| TO_STOP / SCAN_360 | Original lane search and distance-sensor width detection |
| AIM CANDIDATE | Original sensor-based turn toward the stored target |
| CHECK BALL (CV), purple LED | Wheels stopped; examine fresh camera observations |
| COLLECT BALL, blue LED | Approved: original drive-through and intake |
| CV SKIP: NO MATCH | Unconfirmed: exclude the spot for this mission, resume searching |
| CV unavailable | No fresh confirmation data at the check: stop and restart after restoring it |

A sensor candidate must pass before collection. The camera cannot create targets
or interrupt the route to chase something else.

The check starts after the sensor-based aiming turn has settled. It ignores
frames captured before the check's additional 150 ms settling interval, counts
distinct serial observations and requires two consecutive positives. Frames over
750 ms old cannot approve anything. Total checking time is capped at 1.5 seconds.
This permits normal CPU capture/inference latency while the wheels are stationary;
the post-aim capture requirement still rejects frames from the preceding turn.

Default camera filtering requires a model confidence of at least 0.50, a roughly
round detection box (width/height 0.6 to 1.6), and its centre inside the orange band
covering the central 15% of the image. Two eligible balls in that band are treated
as ambiguous. A high-confidence ball elsewhere in the image cannot approve a
central chair leg. The Brain counts the fresh consecutive observations; the
preview alone is not proof that the current candidate has passed.

The preview's Brain line is the actual reported mission stage, not inferred from
a camera box. Its Last check line keeps the most recent confirmation outcome
visible after scanning resumes. The CV line explains no box, low confidence,
outside check zone, rejected box shape, or multiple eligible balls. A 70–80%
detection already passes the confidence threshold; other checks still apply.

The weights match the saved repository model byte-for-byte. Confidence scores can
vary with the input image; changing the cutoff changes which boxes are shown,
not their scores. The serial cutoff is restored to the saved version's 0.50;
the standalone preview remains at 0.25.

After approval, the original encoder/gyro drive-through, 400 mm roll-over and
intake routine complete independently of visual detection. A lost box does not
cancel it. Original bumper, distance-to-obstacle, stall, time and physical-stop
checks still apply. There is no camera-controlled blind-forward timer.

Camera loss does not switch into unfiltered collection. Searching may continue,
but the next candidate cannot pass without fresh confirmation and stops the
mission if confirmation data is unavailable. Closing the detector normally sends
X immediately; unplugging USB may prevent that message, so use the physical stop
when needed. Brain Left deliberately selects the unfiltered original mode.

## Phone alignment and limits

Keep the phone at a consistent position and angle relative to the robot, turning
it with the robot. During CHECK BALL, the orange band should cover the particular
object the robot is facing. A rigid mount makes this repeatable. Holding the phone
independently can cause real balls to be rejected or the wrong object to be checked.

If the camera is offset, adjust the band centre without changing motor aiming:
`.venv\Scripts\python.exe detect_ball.py --serial --verify-centre 0.55`.
The default is 0.50. Band width can be adjusted with `--verify-width` (default
0.15); widening it makes unrelated-ball confirmation more likely.
`--confidence` adjusts the model threshold (serial default 0.50).

The supplied weights were not retrained, and no measured detection-accuracy gain
is claimed. A model can still mislabel a chair part as a ball. This visual check
also cannot distinguish objects at different depths on the same sight line
without camera calibration/depth information. A real ball outside the view or
missed by the detector will be skipped. Check the preview alignment using a real
ball before judging collection performance.

## Preserved code and controls

The original source remains exactly in
`vex-receiver/reference/collector_v4_original.iqcpp`, and the Downloads original
and slot 1 have not been changed. Lane/stop calculations, scan sweeps, sensor
segmentation, ball list, aiming, speed profile, drive-through and backoff match
the original after device-name and logging substitutions. Original numeric and
boolean tuning settings are retained. The original gyro test, settling and dwell
are restored. The added stationary candidate check is the intended new behaviour.

Shared hardware: left motor 12, right motor 8, intake 11, distance 7, bumper 5,
Touch LED 4, and built-in inertial sensor. Shared intake reversed=true/forward
preserves the original reversed=false/reverse direction.

| Input from IDLE | Mode |
|---|---|
| Touch LED, or optional controller E-Up | Original collector with CV confirmation |
| Brain Left, or optional controller L-Up | Original sensor-only collector |
| Optional controller E-Down | Manual drive |

Both autonomous modes calibrate and require a fresh Touch LED press to start.
The previous camera-driving demo is removed from the selectable controls.
Necessary integration changes also reset state for repeat runs, bound calibration,
reserve serial for acknowledgements and add physical/remote stops.

## Protocol and verification

V5 handshake: `BALL_SERIAL_V5 DRIVE` (TEST if motor enabling is disabled).
Each frame sends `B 1 120\n` for an eligible ball or `B 0 120\n` for no match;
120 is the measured capture/processing age in milliseconds. Complete valid
packets receive `ACK B <stage>|<last check>\n` in V5.1. Earlier V5's bare `ACK B\n`
is also accepted by the laptop. Only the reader thread writes replies so status
cannot interleave with acknowledgements. Receipt age is added on the Brain, and each packet gets
a distinct observation number. Partial packets never refresh observation state.
No motion values are transmitted. Old L/R/F/N/C steering bytes cannot approve
a target or control motors. X requests mission stop; S invalidates observations.

Python checks:
`.venv\Scripts\python.exe -m unittest discover -s tests -v`.
Firmware compile-time tests cover fresh post-aim observations, repeated-packet
rejection, consecutive positives, stale data and stop priority.
ARM source/object compilation succeeds in the agent session. The session's
Windows linker previously failed with error 623; updater 5 runs the full
build/download in the normal terminal. Physical chair-leg rejection and collection
still need a test on the robot.
