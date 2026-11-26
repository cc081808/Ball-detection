"""Require repeat sightings and retain a target instead of chasing score flicker."""
import math


class StableTarget:
    def __init__(self):
        self.box = None
        self.hits = 0
        self.last_seen = None
        self.confirmed = False

    def select(self, candidates, width, height, now):
        valid = [(list(box), score) for box, score in candidates
                 if len(box) == 4 and all(math.isfinite(v) for v in box) and
                 math.isfinite(score) and 0 <= box[0] < box[2] <= width and
                 0 <= box[1] < box[3] <= height]
        if self.last_seen is not None and now - self.last_seen > (1.2 if self.confirmed else .35):
            self.box = None
            self.hits = 0
            self.confirmed = False
        if not valid:
            if not self.confirmed:
                self.hits = 0
            return None  # Never transmit a remembered box as a new observation.
        if self.box is not None:
            cx = (self.box[0] + self.box[2]) / 2
            cy = (self.box[1] + self.box[3]) / 2
            matches = [(box, score) for box, score in valid
                       if abs((box[0] + box[2]) / 2 - cx) < width * .18 and
                       abs((box[1] + box[3]) / 2 - cy) < height * .18]
            if matches:
                box, _ = min(matches, key=lambda item:
                    ((item[0][0] + item[0][2]) / 2 - cx) ** 2 +
                    ((item[0][1] + item[0][3]) / 2 - cy) ** 2)
                self.box, self.hits, self.last_seen = box, self.hits + 1, now
                self.confirmed = self.hits >= 2
                return box if self.confirmed else None
            if self.confirmed:
                return None  # Let the robot reacquire this target before switching.
        # Prefer a central target, with confidence breaking ties.
        box, _ = min(valid, key=lambda item: (abs((item[0][0] + item[0][2]) / width - 1), -item[1]))
        self.box, self.hits, self.last_seen = box, 1, now
        return None
