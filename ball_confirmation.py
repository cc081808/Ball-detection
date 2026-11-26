"""Identify a plausible ball in the sensor candidate's camera region; never steer."""
import math

CV_FRAME_MAX_AGE_MS = 750


def assess_confirmation(candidates, width, height, confidence=.50, centre=.5, band=.15):
    if not (math.isfinite(width) and math.isfinite(height) and width > 0 and height > 0):
        return None, "INVALID FRAME"
    valid = []
    for box, score in candidates:
        if (len(box) != 4 or not all(math.isfinite(x) for x in box) or
                not math.isfinite(score) or not 0 <= score <= 1):
            continue
        x1, y1, x2, y2 = box
        if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
            continue
        valid.append((list(box), score))
    if not valid:
        return None, "NO BALL BOX"
    confident = [(box, score) for box, score in valid if score >= confidence]
    if not confident:
        return None, "BELOW CONFIDENCE LIMIT"
    centred = [(box, score) for box, score in confident
               if abs((box[0] + box[2]) / (2 * width) - centre) <= band / 2]
    if not centred:
        return None, "BALL OUTSIDE CHECK ZONE"
    # A tall leg or an elongated detection should not approve collection.
    matches = [(box, score) for box, score in centred if .6 <= (box[2] - box[0]) / (box[3] - box[1]) <= 1.6]
    if not matches:
        return None, "BOX SHAPE REJECTED"
    if len(matches) > 1:
        return None, "MULTIPLE BALLS IN ZONE"
    return matches[0], "BALL IN CHECK ZONE"


def select_confirmation(candidates, width, height, confidence=.50, centre=.5, band=.15):
    return assess_confirmation(candidates, width, height, confidence, centre, band)[0]
