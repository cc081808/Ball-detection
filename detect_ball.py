from pathlib import Path
import argparse
import time

import cv2
import numpy as np
from cv2_enumerate_cameras import enumerate_cameras
from ultralytics import YOLO
from serial_bridge import BrainLink, choose_port
from camera_input import open_camera, has_image
from ball_confirmation import assess_confirmation, CV_FRAME_MAX_AGE_MS


def select_camera():
    cameras = list(enumerate_cameras(cv2.CAP_DSHOW))
    if not cameras:
        cameras = list(enumerate_cameras(cv2.CAP_MSMF))
    if not cameras:
        raise SystemExit("No cameras found. Open Camo Studio, confirm its preview, then try again.")
    for number, camera in enumerate(cameras, start=1):
        print(f"{number}. {camera.name}")
    default = next((i for i, c in enumerate(cameras, 1) if "camo" in c.name.lower()), 1)
    while True:
        choice = input(f"Select camera [Enter for {default}]: ").strip() or str(default)
        if choice.isdecimal() and 1 <= int(choice) <= len(cameras):
            return cameras[int(choice) - 1]


def main():
    parser = argparse.ArgumentParser(description="Ball detector with optional VEX IQ serial output")
    parser.add_argument("--check", action="store_true", help="Test model inference without a camera or robot")
    parser.add_argument("--serial", action="store_true", help="Choose the VEX user/console COM port")
    parser.add_argument("--port", help="VEX user/console port, for example COM5; enables serial output")
    parser.add_argument("--list-ports", action="store_true")
    parser.add_argument("--camera-backend", choices=("auto", "msmf", "dshow"), default="auto",
                        help="Windows capture method; auto tries another method if the selected camera is blank")
    parser.add_argument("--confidence", type=float, default=None, help="Detection threshold from 0 to 1; serial default 0.50")
    parser.add_argument("--verify-centre", type=float, default=.5,
                        help="Camera x fraction aligned with the aimed sensor candidate; default .5")
    parser.add_argument("--verify-width", type=float, default=.15,
                        help="Width fraction of the confirmation band; default .15")
    args = parser.parse_args()
    if args.confidence is not None and not 0 < args.confidence <= 1:
        parser.error("--confidence must be greater than 0 and at most 1")
    if not .1 <= args.verify_centre <= .9 or not .05 <= args.verify_width <= .3:
        parser.error("--verify-centre must be .1 to .9; --verify-width must be .05 to .3")
    if args.list_ports:
        from serial.tools import list_ports
        ports = list(list_ports.comports())
        for port in ports:
            print(f"{port.device}: {port.description}")
        if not ports:
            print("No COM ports found. Turn on the Brain and connect a USB data cable.")
        return
    weights = Path(__file__).parent / "downloaded_model" / "weights.pt"
    if not weights.is_file():
        raise SystemExit("Missing downloaded_model/weights.pt. Restore the supplied model or follow README.md to download it.")
    model = YOLO(str(weights), task="detect")
    if args.check:
        model.predict(np.zeros((480, 640, 3), dtype=np.uint8), device="cpu", verbose=False)
        print("Model inference passed. Classes:", model.names)
        return
    # Load inference kernels before opening a control session (first inference
    # can take seconds and would otherwise trip the Brain's freshness watchdog).
    model.predict(np.zeros((480, 640, 3), dtype=np.uint8), device="cpu", verbose=False)
    camera = select_camera()
    capture, first_frame = open_camera(camera, args.camera_backend)
    link = None
    try:
        if not capture.isOpened():
            raise SystemExit(f"Could not open {camera.name}. Check Camo Studio and Windows camera permissions.")
        if args.serial or args.port:
            link = BrainLink(args.port or choose_port())
            first_frame = None  # Read a current frame after the serial handshake.
            print(f"Connected: {link.mode}")
            if not link.supports_confirmation:
                raise RuntimeError("The Brain is running the older receiver. Close this window and run 5 - Update slot 4.cmd first.")
            if link.mode.endswith("TEST"):
                print("Receiver is in TEST mode: motors remain disabled.")
            else:
                print("Touch LED selects SENSOR + CV; keep still for calibration, then press/release LED when ready.")
                print("V5.1 waits for the camera instead of returning to IDLE. Connecting alone never starts the robot.")
                print("Touch LED or Brain Check stops the mission. No controller needed.")
                print("Distance sensing controls the original route, aiming and intake. CV only confirms candidates.")
                print("CHECK BALL: two fresh matching frames approve collection; no match skips the object.")
                print("Brain Left: original sensor-only mode. Q sends mission STOP.")
                print("Keep the phone aligned with the robot's forward direction, including while turning.")
        print("Press Q or Esc in the preview to quit.")
        title = "Ping-pong ball detector"
        confidence = args.confidence if args.confidence is not None else (0.50 if link else 0.25)
        while True:
            captured_at = time.monotonic()
            if first_frame is not None:
                ok, frame = True, first_frame
                first_frame = None
            else:
                ok, frame = capture.read()
            if not ok:
                raise SystemExit("Camera stopped delivering frames. Check the live preview in Camo Studio.")
            if not has_image(frame):
                if link:
                    link.send("S")
                raise RuntimeError("Camera image went black. " + ("Sent STOP. " if link else "") +
                                   "Check Camo Studio and restart the detector.")
            result = model.predict(frame, device="cpu", conf=confidence, verbose=False)[0]
            preview = result.plot()
            if link:
                boxes = result.boxes
                candidates = [] if boxes is None else list(zip(boxes.xyxy.tolist(), boxes.conf.tolist()))
                match, reason = assess_confirmation(candidates, frame.shape[1], frame.shape[0], confidence,
                                                   args.verify_centre, args.verify_width)
                age = min(5000, max(0, int((time.monotonic() - captured_at) * 1000)))
                link.send_confirmation(match is not None, age)
                label = "CV: " + reason
                if age > CV_FRAME_MAX_AGE_MS:
                    label = f"CV: FRAME TOO OLD ({age} ms)"
                cv2.putText(preview, label, (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.putText(preview, f"Brain: {link.brain_state}", (15, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
                cv2.putText(preview, f"Last check: {link.last_check}", (15, 85),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
                cv2.putText(preview, "Sensor drives | LED: SENSOR + CV | Check or Q: stop", (15, 110),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                if match:
                    x1, y1, x2, y2 = map(int, match[0])
                    cv2.rectangle(preview, (x1, y1), (x2, y2), (0, 255, 0), 2)
                h, w = frame.shape[:2]
                for edge in (args.verify_centre - args.verify_width / 2, args.verify_centre + args.verify_width / 2):
                    cv2.line(preview, (int(w * edge), 120), (int(w * edge), h), (0, 180, 255), 1)
            cv2.imshow(title, preview)
            if cv2.waitKey(1) & 0xFF in (27, ord("q"), ord("Q")):
                break
            if cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        if link:
            link.close()
        capture.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Stopped.")
    except (RuntimeError, OSError) as error:
        raise SystemExit(str(error)) from None
