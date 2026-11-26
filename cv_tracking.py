"""Slot 5 target association. Remembered boxes are never sent as fresh sightings."""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Target:
    identity: int
    box: tuple
    confidence: float


class TargetLock:
    def __init__(self, acquire_confidence=.5, retain_confidence=.3, hold_seconds=1.8, acquire_y_max=1.0):
        self.acquire_confidence = acquire_confidence
        self.retain_confidence = retain_confidence
        self.hold_seconds = hold_seconds
        self.acquire_y_max = acquire_y_max
        self.reset()
        self.next_id = 1

    def reset(self):
        self.target = None
        self.last_seen = -math.inf
        self.hits = 0

    @property
    def identity(self):
        return self.target.identity if self.target else 0

    def update(self, candidates, width, height, now):
        valid = []
        for box, confidence in candidates:
            if len(box) != 4 or not all(math.isfinite(v) for v in (*box, confidence)):
                continue
            x1, y1, x2, y2 = box
            if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
                continue
            if not .55 <= (x2 - x1) / (y2 - y1) <= 1.8:
                continue
            if confidence >= self.retain_confidence:
                valid.append((tuple(box), confidence))
        if now - self.last_seen > (self.hold_seconds if self.hits >= 2 else .4):
            self.reset()
        if self.target:
            old = self.target.box
            ox, oy = (old[0] + old[2]) / 2, (old[1] + old[3]) / 2
            old_size = max(old[2] - old[0], old[3] - old[1])
            matches = []
            for box, confidence in valid:
                cx, cy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
                size = max(box[2] - box[0], box[3] - box[1])
                dx, dy = (cx - ox) / width, (cy - oy) / height
                if abs(dx) <= .20 and abs(dy) <= .20 and .4 <= size / old_size <= 2.5:
                    matches.append((dx * dx + dy * dy, box, confidence))
            if not matches:
                if self.hits < 2:
                    self.reset()
                return None
            _, box, confidence = min(matches)
            if self.hits < 2 and confidence < self.acquire_confidence:
                self.reset()
                return None
            self.target = Target(self.target.identity, box, confidence)
            self.last_seen = now
            self.hits += 1
            return self.target if self.hits >= 2 else None
        choices = [(box, score) for box, score in valid if score >= self.acquire_confidence and
                   (box[1] + box[3]) / (2 * height) < self.acquire_y_max]
        if choices:
            # Prefer nearby, reasonably central balls; don't switch on score flicker.
            box, score = max(choices, key=lambda p:
                p[0][3] / height - .4 * abs((p[0][0] + p[0][2]) / width - 1))
            self.target = Target(self.next_id, box, score)
            self.next_id += 1
            self.last_seen, self.hits = now, 1
        return None
