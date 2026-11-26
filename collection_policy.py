"""Camera-only decisions; never keep moving on a missing/failed camera frame."""
from serial_bridge import command_for_detection


class CollectionPolicy:
    def __init__(self, mirror=False, intake_line=0.65):
        self.mirror = mirror
        self.intake_line = intake_line
        self.error_milli = 0
        self.near_frames = 0
        self.last_frame_time = None
        self.finish_until = None

    def command(self, bounds, width, height, now):
        self.error_milli = 0
        # A slow/missing-frame interval cannot qualify a blind intake move.
        if self.last_frame_time is not None and now - self.last_frame_time > 0.30:
            self.near_frames = 0
            self.finish_until = None
        self.last_frame_time = now
        if bounds is not None:
            self.finish_until = None
            command = command_for_detection(bounds, width, deadband=0.10, mirror=self.mirror)
            if command == "S" or height <= 0 or bounds[3] > height:
                self.near_frames = 0
                return "S"
            self.error_milli = round(((bounds[0] + bounds[2]) / width - 1) * 1000 * (-1 if self.mirror else 1))
            # A ball low in the image and centred on two frames is approaching
            # the intake blind spot. This threshold assumes a forward-facing camera.
            near = command == "F" and bounds[3] >= self.intake_line * height
            self.near_frames = self.near_frames + 1 if near else 0
            return command
        if self.near_frames >= 2:
            self.finish_until = now + 0.60
        self.near_frames = 0
        if self.finish_until is not None and now < self.finish_until:
            return "C"
        self.finish_until = None
        return "N"  # Live image, no target: search. S still means explicit STOP.
