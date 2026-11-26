# Separate CV collector — program slot 5

This is a new `vex-cv` project named **cv-collector**. It does not import the old collector or distance sensor. The original slot 1 project and slot 4 integration remain separate. No program is downloaded merely by starting the camera.

## Phone camera with Cloudflare and a QR code

The phone camera page is reachable online through a temporary Cloudflare Tunnel. Scan a QR code to pair the phone. **YOLO detection still runs on the laptop**, using the same supplied ball model. The laptop sends coordinates over the Brain's USB user port. This is not a cloud-hosted inference service or a Cloudflare Workers deployment.

1. Run **10 - Setup phone camera.cmd** once. It installs Cloudflare's `cloudflared` utility with Windows Package Manager. No Cloudflare account or domain is required for this temporary tunnel.
2. Run **11 - Pair phone camera.cmd**. A local browser page opens with a QR code. Keep the pairing terminal open.
3. Scan the QR with the phone, open the HTTPS page, tap **Start camera**, and allow camera access. Use the rear camera. Keep the page visible and mount the phone rigidly facing forward/down. Closing/locking the phone or stopping this page ends fresh camera input.
4. Run **12 - Preview phone camera.cmd** to check detection without a Brain connection. Quit this preview before running the robot application.
5. Run **9 - Calibrate CV camera.cmd**. Place a ball aligned with the intake, still fully visible, about 10 cm before it would enter the camera blind spot. Click its centre. This gives the detector time for two sightings before the ball disappears. Use `+`/`-` to set the measured remaining travel from that position until the rollers take it in; Enter saves. The initial 200 mm is provisional, not a measured setting for your mount. Recalibrate after changing phone angle, zoom, lens, mirroring, or orientation.

The network must provide sufficiently recent frames. The software counts network and inference delay together; it stops movement rather than silently extending the freshness limit for a slow connection. A tunnel link changes whenever pairing is restarted. Existing Cloudflare tunnel configuration can interfere with Quick Tunnels; the connection log is `.runtime/cloudflare.log`.

The QR link contains a random pairing key. Keep it private. The camera page does not expose motor controls or the local frame-reading key. Frames are forwarded through Cloudflare and held in laptop memory; the app does not record them. Tokens/QR/logs are saved under the existing git-ignored `.runtime` folder.

## Install and run the robot program

1. Connect the powered IQ second-generation Brain to the laptop with its USB data cable.
2. Run **6 - Install CV slot 5.cmd**. It rebuilds from source, reads the Brain's program list, refuses to overwrite an unrelated program in slot 5, downloads only to slot 5, and checks other slot listings afterward. A failed build never downloads an older binary.
3. The Brain should show **CV SLOT 5 V1**. It calibrates the built-in inertial sensor once while stationary, then waits. Installing or connecting the laptop does not arm the motors.
4. With the phone video live and the mount calibrated, run **7 - Start CV slot 5.cmd**.
5. Start in a clear, measured **2 m × 2 m area** by default. Put the rear-axle midpoint 400 mm in from the near and right boundaries, facing forward into the area. Every new run assumes this same starting pose. Move the calibration ball farther ahead for the first approach test.
6. Press and release **Touch LED port 4** to start. Press it again, press **Brain Check**, press **Space** in the laptop preview, or quit the preview with **Q** to stop. A stopped robot requires a new physical press/release; reconnecting the camera never restarts it.

The Brain USB connection remains necessary. Cloudflare removes the phone-to-laptop camera cable, not the Brain cable. There is no controller requirement.

**8 - Open CV slot 5.cmd** opens the separate VS Code project. **13 - Start CV with Camo.cmd** keeps Camo/USB-camera capture available as an alternative; it has a separate mount calibration file.

## Behaviour

* Camera coordinates continuously control ball alignment and approach speed. The distance sensor does not choose, approve, or steer toward targets.
* The laptop requires repeat sightings to acquire a target and preserves its identity through brief gaps. It does not transmit remembered boxes as fresh observations. Visually identical or overlapping balls can still confuse association.
* Two distinct, confident observations in the calibrated, centred pickup zone commit a measured finish move. The encoder baseline comes from the first close observation. Lost boxes cannot reset or extend the distance. Heading feedback holds the final direction.
* A target lost earlier triggers a short local search around its last bearing. It does not cause an immediate unrelated full spin or a blind forward drive.
* With no target, the robot scans and follows a serpentine grid through the configured area. A ball interrupts this route immediately. After a pickup attempt it searches again without the original stationary dwell/check pause.
* Motor rotation estimates distance; the built-in inertial sensor supplies heading. This is **CV-directed navigation with movement feedback**, not strictly camera-only sensing. Position can drift with slipping. These estimates do not establish a reliable wall map.
* Bumper port 5 aborts the current pickup, backs off 150 mm, and turns before resuming search. A held bumper, repeated collisions, drive stall, motion timeout, estimated area boundary, 5-minute limit, 10 completed pickup attempts, or completed coverage stops the run. Touch/Check/PC stop and camera timeout interrupt every movement, including backoff and pickup.
* The count is completed **pickup attempts**, not verified captured balls. Newly acquired targets below the pickup line are excluded so a ball sitting in the rollers is not repeatedly chased. A camera view of actual capture would be needed to verify collection.

This first version assumes clear floor. A ping-pong detector does not detect every chair, wall, person, or table edge. The original distance-sensor wall detection is absent.

## Settings

Phone settings: `cv_slot5_phone.json`. Camo settings: `cv_slot5.json`.

| Setting | Default | Meaning |
| --- | --- | --- |
| `area_length_mm`, `area_width_mm` | 2000, 2000 | Measured clear search area, forward and left from the near-right corner |
| `aim_x` | 0.50 until calibrated | Image fraction aligned with the intake |
| `commit_y` | 0.78 until calibrated | Ball-centre row that begins the final pickup move |
| `finish_mm` | 200 until measured | Total advance from the first close observation; allowed 40–450 mm |
| `approach_mm_s` | 240 | Approach/search travel speed, maximum permitted 400 |
| `pickup_mm_s` | 120 | Final pickup speed, maximum permitted 240 |
| `steer_sign` | 1 | Use -1 only if image mirroring makes steering reversed |
| `acquire_confidence` / `retain_confidence` | 0.50 / 0.30 | Acquire a new ball / retain an existing target |

Motor ports/directions and geometry match the saved original collector: left 12 normal, right 8 reversed, intake 11 reversed with forward rotation (equivalent to the original normal/reverse setup), 225 mm track, 1.111 wheel mm per motor degree. Intake runs at 100% while active. Scan speed is 35°/s. The original 400 mm/s drive ceiling is retained; lower default speeds allow camera latency to be evaluated first.

## Protocol and testing

`?` identifies `CV_COORD_V1 DRIVE`. `C` supplies validated calibration while idle. `F` supplies sequence, target identity, visibility, normalized centre/size, confidence, and frame age. `X` stops. Replayed, malformed, and stale packets cannot refresh movement permission. Camera frames must be no more than 700 ms old in total.

For phone video, the laptop issues a numbered capture request before the phone captures a fresh browser video frame. The original request timestamp survives upload, camera buffering, inference, and serial transmission. This avoids relying on synchronized phone/laptop clocks. Only one outstanding capture request is valid, and each response is accepted once. Black/missing frames and detected frozen video stop the PC session.

Run Python checks with `.venv\Scripts\python.exe -m unittest discover -s tests -v`. `tests\run_cv_control.cmd` builds and runs the actual C++ controller against simulated wheel/heading feedback, including loss of view, stale frames, physical stops, bumper recovery, limits, and full-area search. `vex-cv\Build CV.cmd` performs the full VEX build without downloading.

Actual phone-to-Cloudflare performance, steering direction, pickup calibration, and collection success require physical testing. See `CV_SLOT5_VALIDATION.md` for the checks completed in the development session and outstanding steps.

Official setup references:
- https://developers.cloudflare.com/tunnel/get-started/quick-tunnels/
- https://developers.cloudflare.com/cloudflare-one/connections/connect-networks/downloads/
- https://developer.mozilla.org/en-US/docs/Web/API/MediaDevices/getUserMedia
