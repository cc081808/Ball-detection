"""Open the selected camera by name across Windows capture backends."""
import time

import cv2
import numpy as np
from cv2_enumerate_cameras import enumerate_cameras


def has_image(frame):
    """Reject missing or effectively black frames, not just a successful read flag."""
    return frame is not None and frame.size > 0 and bool(np.any(frame > 2))


def camera_candidates(selected, backend="auto"):
    methods = {"msmf": cv2.CAP_MSMF, "dshow": cv2.CAP_DSHOW}
    if backend != "auto":
        order = [methods[backend]]
    elif "camo" in selected.name.lower():
        order = [cv2.CAP_MSMF, cv2.CAP_DSHOW]
    else:
        order = [selected.backend, cv2.CAP_MSMF if selected.backend == cv2.CAP_DSHOW else cv2.CAP_DSHOW]
    for method in order:
        # Indices differ between backends. Never reuse an index or substitute a webcam.
        matches = [c for c in enumerate_cameras(method) if c.name.casefold() == selected.name.casefold()]
        if len(matches) == 1:
            yield matches[0]
        elif method == selected.backend:
            yield selected


def open_camera(selected, backend="auto", warmup_seconds=3.0):
    for camera in camera_candidates(selected, backend):
        method = "Media Foundation" if camera.backend == cv2.CAP_MSMF else "DirectShow"
        print(f"Opening {camera.name} using {method} (camera {camera.index})...", flush=True)
        capture = cv2.VideoCapture(camera.index, camera.backend)
        usable = False
        try:
            if not capture.isOpened():
                print(f"{method}: camera could not be opened.", flush=True)
                continue
            # A modest format reduces virtual-camera negotiation and inference load.
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            capture.set(cv2.CAP_PROP_FPS, 30)
            deadline = time.monotonic() + warmup_seconds
            while time.monotonic() < deadline:
                ok, frame = capture.read()
                if ok and has_image(frame):
                    usable = True
                    print(f"Camera image received: {frame.shape[1]} x {frame.shape[0]} via {method}.", flush=True)
                    return capture, frame
                time.sleep(0.03)
            print(f"{method}: no usable image (blank or missing frames).", flush=True)
        finally:
            if not usable:
                capture.release()
    raise RuntimeError(
        f"{selected.name} did not provide an image. Close other detector windows, keep Camo Studio "
        "showing live video, restart Camo Studio, then try again. No robot connection was started."
    )
