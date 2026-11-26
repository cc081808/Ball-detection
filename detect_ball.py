from pathlib import Path
import argparse

import cv2
import numpy as np
from cv2_enumerate_cameras import enumerate_cameras
from ultralytics import YOLO
from serial_bridge import BrainLink, choose_port, command_for_detection


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
    parser.add_argument("--mirror-steering", action="store_true", help="Reverse steering for a mirrored camera feed")
    args = parser.parse_args()
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
    camera = select_camera()
    capture = cv2.VideoCapture(camera.index, camera.backend)
    link = None
    try:
        if not capture.isOpened():
            raise SystemExit(f"Could not open {camera.name}. Check Camo Studio and Windows camera permissions.")
        if args.serial or args.port:
            link = BrainLink(args.port or choose_port())
            print(f"Connected: {link.mode}")
            if link.mode.endswith("TEST"):
                print("Receiver is in TEST mode: motors remain disabled.")
            else:
                print("Receiver is in DRIVE mode: hold controller E-Up to allow motion.")
        print("Press Q or Esc in the preview to quit.")
        title = "Ping-pong ball detector"
        while True:
            ok, frame = capture.read()
            if not ok:
                raise SystemExit("Camera stopped delivering frames. Check the live preview in Camo Studio.")
            result = model.predict(frame, device="cpu", conf=0.50 if link else 0.25, verbose=False)[0]
            preview = result.plot()
            if link:
                boxes = result.boxes
                bounds = None
                if boxes is not None and len(boxes):
                    best = int(boxes.conf.argmax().item())
                    bounds = boxes.xyxy[best].tolist()
                command = command_for_detection(bounds, frame.shape[1], mirror=args.mirror_steering)
                link.send(command)
                label = f"VEX: {command} | {link.mode.rsplit(' ', 1)[-1]}"
                cv2.putText(preview, label, (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
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
