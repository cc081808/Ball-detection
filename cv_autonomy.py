"""Separate slot 5 CV application. The existing detector/slot 4 are untouched."""
import argparse
import json
from pathlib import Path
import threading
import time
import zlib

import cv2
import numpy as np
from ultralytics import YOLO

from camera_input import open_camera, has_image
from detect_ball import select_camera
from serial_bridge import choose_port
from cv_serial import CoordinateLink, configuration_values
from cv_tracking import TargetLock

ROOT = Path(__file__).resolve().parent
TITLE = 'CV collector - SLOT 5'


class LatestCamera:
    """Drain capture continuously; inference receives only the newest frame."""
    def __init__(self, capture):
        self.capture = capture
        self.lock = threading.Lock()
        self.ended = threading.Event()
        self.latest = None
        self.error = None
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self):
        sequence, checksum, changed = 0, None, time.monotonic()
        try:
            while not self.ended.is_set():
                started = time.monotonic()
                if hasattr(self.capture, 'read_timed'):
                    ok, frame, started = self.capture.read_timed()
                else:
                    ok, frame = self.capture.read()
                if not ok or not has_image(frame):
                    raise RuntimeError('Camera stopped or returned a black image. Check the phone camera page or Camo.')
                new_checksum = zlib.crc32(frame.tobytes())
                if new_checksum != checksum:
                    checksum, changed = new_checksum, time.monotonic()
                sequence += 1
                with self.lock:
                    self.latest = (sequence, started, frame, changed)
        except Exception as error:
            with self.lock:
                self.error = error

    def get(self, previous=0, timeout=.65):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                if self.error:
                    raise self.error
                if self.latest is not None and self.latest[0] != previous:
                    return self.latest
            time.sleep(.005)
        raise RuntimeError('No new camera frame. Robot freshness timeout will stop movement.')

    def close(self):
        self.ended.set()
        self.capture.release()
        self.thread.join(timeout=1)


def overlay(frame, text, row, color=(0, 255, 255)):
    cv2.putText(frame, text, (12, 24 + row * 24), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 0), 4)
    cv2.putText(frame, text, (12, 24 + row * 24), cv2.FONT_HERSHEY_SIMPLEX, .55, color, 1)


def calibration(camera, config, path):
    print('\nCAMERA CALIBRATION (no serial connection or movement during this step).')
    print('Mount the phone rigidly, looking forward/down. Keep Camo zoom/mirroring fixed.')
    print('Place a ball aligned with the intake, still fully visible, about 10 cm before the camera blind spot.')
    print('Click its CENTRE in the preview. + / - adjust the remaining forward travel, in mm.')
    print('Measure from that ball position until the rollers fully take it in. Default 200 mm is provisional.')
    print('Enter saves. Q cancels. Area dimensions are in cv_slot5.json (default clear 2 x 2 metres).')
    selected = False
    shape = [480, 640]
    def click(event, x, y, flags, unused):
        nonlocal selected
        if event == cv2.EVENT_LBUTTONDOWN:
            aim, near = x / shape[1], y / shape[0]
            if .2 <= aim <= .8 and .4 <= near <= .95:
                config['aim_x'], config['commit_y'] = round(aim, 3), round(near, 3)
                selected = True
            else:
                print('Choose a point in the lower central view; adjust the phone angle if needed.')
    cv2.namedWindow(TITLE, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(TITLE, click)
    previous = 0
    try:
        while True:
            previous, stamp, frame, changed = camera.get(previous)
            shape[:] = frame.shape[:2]
            preview = frame.copy()
            h, w = preview.shape[:2]
            cv2.line(preview, (round(config['aim_x']*w), 0), (round(config['aim_x']*w), h), (0,255,0), 1)
            cv2.line(preview, (0, round(config['commit_y']*h)), (w, round(config['commit_y']*h)), (0,255,0), 1)
            overlay(preview, 'Click aligned ball about 10 cm before camera blind spot', 0)
            overlay(preview, f"Finish travel: {config['finish_mm']} mm  (+ / - to adjust)", 1)
            overlay(preview, 'ENTER: save calibration | Q: cancel', 2)
            cv2.imshow(TITLE, preview)
            key = cv2.waitKey(15) & 0xff
            if key in (27, ord('q'), ord('Q')) or cv2.getWindowProperty(TITLE, cv2.WND_PROP_VISIBLE) < 1:
                raise RuntimeError('Calibration cancelled. Nothing started on the robot.')
            if key in (ord('+'), ord('=')):
                config['finish_mm'] = min(450, config['finish_mm'] + 10)
            elif key == ord('-'):
                config['finish_mm'] = max(40, config['finish_mm'] - 10)
            elif key in (10, 13) and selected:
                config['calibrated'] = True
                config['image_width'], config['image_height'] = w, h
                configuration_values(config)
                path.write_text(json.dumps(config, indent=2) + '\n', encoding='utf-8')
                print('Camera calibration saved.')
                return
    finally:
        cv2.setMouseCallback(TITLE, lambda *args: None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preview', action='store_true', help='Camera/model preview only; never connects to a Brain')
    parser.add_argument('--calibrate', action='store_true', help='Redo mount/finish calibration before connecting')
    parser.add_argument('--port', help='VEX IQ user/console port')
    parser.add_argument('--camera-backend', choices=['auto', 'msmf', 'dshow'], default='auto')
    parser.add_argument('--camera', choices=['phone', 'camo'], default='phone')
    parser.add_argument('--config', type=Path)
    parser.add_argument('--check', action='store_true', help='Offline model/config check, no camera or motors')
    args = parser.parse_args()
    if args.config is None:
        args.config = ROOT / ('cv_slot5_phone.json' if args.camera == 'phone' else 'cv_slot5.json')
    config = json.loads(args.config.read_text(encoding='utf-8'))
    configuration_values(config)
    model = YOLO(str(ROOT / 'downloaded_model/weights.pt'), task='detect')
    # This project's supplied weights have one class: ping-pong balls.
    if len(model.names) != 1:
        raise RuntimeError('Expected the supplied single-class ping-pong model; check the weights file.')
    model.predict(np.zeros((480, 640, 3), dtype=np.uint8), device='cpu', conf=.3, verbose=False)
    if args.check:
        print('Slot 5 config and model inference passed. Classes:', model.names)
        return
    if args.camera == 'phone':
        from phone_camera import PhoneCapture
        capture = PhoneCapture()
        print('Waiting for phone camera. Scan the pairing QR and tap Start camera.', flush=True)
        deadline = time.monotonic() + 120
        while True:
            ok, _, _ = capture.read_timed()
            if ok:
                break
            if time.monotonic() > deadline:
                raise RuntimeError('No phone video. Run 11 - Pair phone camera.cmd first.')
    else:
        capture, _ = open_camera(select_camera(), args.camera_backend)
    camera = LatestCamera(capture)
    link = None
    try:
        if args.calibrate or (not args.preview and not config['calibrated']):
            calibration(camera, config, args.config)
        if not args.preview:
            link = CoordinateLink(args.port or choose_port(), config)
            print('Connected to CV SLOT 5. This connection does NOT start movement.')
            print(f"Area: {config['area_length_mm']} x {config['area_width_mm']} mm. Start 400 mm in from the near/right edges, facing forward.")
            print('Press/release Touch LED to start. LED, Brain Check, Space, or Q stops.')
            print('Keep the Brain USB connected and phone camera live. Use a clear floor area; bumper contact triggers backoff.')
        tracker = TargetLock(config['acquire_confidence'], config['retain_confidence'],
                             acquire_y_max=config['commit_y'])
        previous, previous_state = 0, 'IDLE'
        cv2.namedWindow(TITLE, cv2.WINDOW_AUTOSIZE)
        while True:
            previous, captured, frame, changed = camera.get(previous)
            if link and link.state not in ('IDLE', 'STOPPED', 'CALIBRATING') and time.monotonic() - changed > 1.5:
                raise RuntimeError('Camera frames are identical while moving; possible frozen Camo feed. Stopping.')
            result = model.predict(frame, device='cpu', conf=config['retain_confidence'], verbose=False)[0]
            boxes = result.boxes
            candidates = [] if boxes is None else list(zip(boxes.xyxy.tolist(), boxes.conf.tolist()))
            h, w = frame.shape[:2]
            if link and (w != config.get('image_width') or h != config.get('image_height')):
                raise RuntimeError('Camera resolution/orientation changed. Restart with --calibrate.')
            target = tracker.update(candidates, w, h, captured)
            age = min(5000, max(0, int((time.monotonic() - captured) * 1000)))
            if link:
                link.send_frame(target, tracker.identity, w, h, age)
                if link.state != previous_state:
                    print(f"Brain: {link.state} | {link.reason} | completed attempts: {link.attempts}", flush=True)
                    if previous_state == 'FINISH_PICKUP' or link.state in ('STOPPED', 'BACKOFF'):
                        tracker.reset()
                    previous_state = link.state
            preview = result.plot()
            cv2.line(preview, (round(config['aim_x']*w), 0), (round(config['aim_x']*w), h), (0,200,255), 1)
            cv2.line(preview, (0, round(config['commit_y']*h)), (w, round(config['commit_y']*h)), (0,200,255), 1)
            if target:
                a,b,c,d = map(int, target.box)
                cv2.rectangle(preview, (a,b), (c,d), (0,255,0), 3)
            overlay(preview, 'SLOT 5: ' + (link.state if link else 'PREVIEW ONLY'), 0)
            overlay(preview, f"Target: {tracker.identity if target else 'lost / searching'} | image age {age} ms", 1)
            overlay(preview, f"Attempts {link.attempts}/10 | {link.seconds}s | {link.reason}" if link else 'No robot connection', 2)
            overlay(preview, 'LED: start/stop | SPACE: stop | Q: quit', 3)
            cv2.imshow(TITLE, preview)
            key = cv2.waitKey(1) & 0xff
            if key in (27, ord('q'), ord('Q')) or cv2.getWindowProperty(TITLE, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key == ord(' ') and link:
                link.stop(wait=True)
                tracker.reset()
    finally:
        if link:
            link.close()
        camera.close()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Stopped.')
    except (RuntimeError, OSError, ValueError, KeyError) as error:
        raise SystemExit(str(error)) from None
