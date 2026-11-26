"""Replay sightings through tracking, policy and the actual serial encoder."""
import unittest

from collection_policy import CollectionPolicy
from serial_bridge import BrainLink
from target_tracker import StableTarget
from test_serial_bridge import FakeSerial


class PipelineTests(unittest.TestCase):
    def replay(self, frames):
        port = FakeSerial([b"BALL_SERIAL_V4 DRIVE\n", b"ACK S\n"] + [b"ACK V\n"] * len(frames))
        link = BrainLink("FAKE", serial_factory=lambda *a, **k: port)
        tracker, policy = StableTarget(), CollectionPolicy()
        for now, candidates in frames:
            box = tracker.select(candidates, 640, 480, now)
            command = policy.command(box, 640, 480, now)
            link.send_vision(command, policy.error_milli, policy.near_frames >= 2)
        return port.writes[2:]

    def test_close_target_dropout_sends_finish_not_stop(self):
        ball = [([300, 300, 340, 360], .8)]
        self.assertEqual(self.replay([(0, ball), (.1, ball), (.2, ball), (.3, []), (.4, [])]),
                         [b"V N 0 0\n", b"V F 0 0\n", b"V F 0 1\n", b"V C 0 0\n", b"V C 0 0\n"])

    def test_brief_far_dropout_reacquires_same_target(self):
        ball = [([220, 100, 260, 140], .8)]
        other = [([480, 100, 520, 140], .99)]
        self.assertEqual(self.replay([(0, ball), (.1, ball), (.2, other), (.3, ball)]),
                         [b"V N 0 0\n", b"V L -250 0\n", b"V N 0 0\n", b"V L -250 0\n"])

    def test_processing_gap_cannot_request_blind_intake(self):
        ball = [([300, 300, 340, 360], .8)]
        packets = self.replay([(0, ball), (.1, ball), (.2, ball), (.6, [])])
        self.assertEqual(packets[-1], b"V N 0 0\n")
