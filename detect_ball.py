from pathlib import Path
import sys

import cv2
import numpy as np
from cv2_enumerate_cameras import enumerate_cameras
from ultralytics import YOLO


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
    weights = Path(__file__).parent / "downloaded_model" / "weights.pt"
    if not weights.is_file():
        raise SystemExit("Missing downloaded_model/weights.pt. Restore the supplied model or follow README.md to download it.")
    model = YOLO(str(weights), task="detect")
    if sys.argv[1:] == ["--check"]:
        model.predict(np.zeros((480, 640, 3), dtype=np.uint8), device="cpu", verbose=False)
        print("Model inference passed. Classes:", model.names)
        return
    camera = select_camera()
    capture = cv2.VideoCapture(camera.index, camera.backend)
    try:
        if not capture.isOpened():
            raise SystemExit(f"Could not open {camera.name}. Check Camo Studio and Windows camera permissions.")
        print("Press Q or Esc in the preview to quit.")
        title = "Ping-pong ball detector"
        while True:
            ok, frame = capture.read()
            if not ok:
                raise SystemExit("Camera stopped delivering frames. Check the live preview in Camo Studio.")
            result = model.predict(frame, device="cpu", conf=0.25, verbose=False)[0]
            cv2.imshow(title, result.plot())
            if cv2.waitKey(1) & 0xFF in (27, ord("q"), ord("Q")):
                break
            if cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        capture.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
