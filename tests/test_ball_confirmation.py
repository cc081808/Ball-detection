import unittest

from ball_confirmation import select_confirmation, assess_confirmation
from serial_bridge import BrainLink
from test_serial_bridge import FakeSerial


class ConfirmationTests(unittest.TestCase):
    def select(self, candidates):
        return select_confirmation(candidates, 640, 480)

    def test_central_round_ball_matches(self):
        ball = ([300, 250, 340, 290], .85)
        self.assertEqual(self.select([ball]), ball)

    def test_ball_elsewhere_cannot_approve_central_chair_leg(self):
        chair_leg = ([300, 100, 340, 400], .95)
        ball_to_side = ([460, 250, 500, 290], .99)
        self.assertIsNone(self.select([chair_leg, ball_to_side]))

    def test_low_confidence_does_not_confirm(self):
        self.assertIsNone(self.select([([300, 250, 340, 290], .49)]))

    def test_seventy_percent_ball_passes_confidence_check(self):
        self.assertIsNotNone(self.select([([300, 250, 340, 290], .70)]))

    def test_diagnostics_explain_visible_box_rejection(self):
        match, reason = assess_confirmation([([460, 250, 500, 290], .9)], 640, 480)
        self.assertIsNone(match)
        self.assertEqual(reason, 'BALL OUTSIDE CHECK ZONE')
        match, reason = assess_confirmation([([300, 100, 340, 400], .9)], 640, 480)
        self.assertIsNone(match)
        self.assertEqual(reason, 'BOX SHAPE REJECTED')

    def test_brain_stage_and_last_result_survive_fragmented_reply(self):
        port = FakeSerial([b"BALL_SERIAL_V5 DRIVE\n", b"ACK S\n",
                           b"ACK B SCAN_", b"360|CV rejected candidate\n"])
        link = BrainLink('FAKE', serial_factory=lambda *a, **k: port)
        link.send_confirmation(True, 120)
        self.assertEqual(link.brain_state, 'SCAN_360')
        self.assertEqual(link.last_check, 'CV rejected candidate')

    def test_broken_status_ack_is_rejected(self):
        for response in (b"ACK B SCAN_360\n", b"ACK B |No check yet\n", b"ACK B IDLE|\n"):
            port = FakeSerial([b"BALL_SERIAL_V5 DRIVE\n", b"ACK S\n", response])
            link = BrainLink('FAKE', serial_factory=lambda *a, **k: port)
            with self.assertRaises(RuntimeError):
                link.send_confirmation(False, 120)

    def test_ambiguous_central_balls_are_rejected(self):
        self.assertIsNone(self.select([([300, 250, 340, 290], .85), ([305, 100, 335, 130], .8)]))

    def test_invalid_bounds_and_scores_rejected(self):
        for candidate in [([300, 250, 340, 490], .9), ([300, 250, 300, 290], .9),
                          ([float('nan'), 250, 340, 290], .9), ([300, 250, 340, 290], float('nan'))]:
            self.assertIsNone(self.select([candidate]))

    def test_alignment_can_be_calibrated(self):
        ball = ([380, 250, 420, 290], .9)
        self.assertIsNone(self.select([ball]))
        self.assertEqual(select_confirmation([ball], 640, 480, centre=.625), ball)

    def test_pipeline_sends_identification_only(self):
        port = FakeSerial([b"BALL_SERIAL_V5 DRIVE\n", b"ACK S\n", b"ACK B\n", b"ACK B\n"])
        link = BrainLink('FAKE', serial_factory=lambda *a, **k: port)
        for candidates in [[([300, 250, 340, 290], .9)], [([300, 100, 340, 400], .95)]]:
            link.send_confirmation(self.select(candidates) is not None, 120)
        self.assertEqual(port.writes, [b'?', b'S', b'B 1 120\n', b'B 0 120\n'])
        link.close()
        self.assertEqual(port.writes[-1], b'X')

    def test_old_steering_receiver_cannot_be_used(self):
        port = FakeSerial([b"BALL_SERIAL_V4 DRIVE\n", b"ACK S\n"])
        link = BrainLink('FAKE', serial_factory=lambda *a, **k: port)
        with self.assertRaisesRegex(RuntimeError, 'confirmation'):
            link.send_confirmation(True, 120)
        self.assertEqual(port.writes, [b'?', b'S'])

    def test_v5_refuses_steering(self):
        port = FakeSerial([b"BALL_SERIAL_V5 DRIVE\n", b"ACK S\n"])
        link = BrainLink('FAKE', serial_factory=lambda *a, **k: port)
        for command in ('L', 'R', 'F', 'N', 'C'):
            with self.assertRaisesRegex(ValueError, 'steering'):
                link.send(command)
        self.assertEqual(port.writes, [b'?', b'S'])

    def test_invalid_confirmation_is_not_transmitted(self):
        port = FakeSerial([b"BALL_SERIAL_V5 DRIVE\n", b"ACK S\n"])
        link = BrainLink('FAKE', serial_factory=lambda *a, **k: port)
        for ball, age in [(1, 100), (True, -1), (True, 5001), (True, float('nan'))]:
            with self.assertRaises(ValueError):
                link.send_confirmation(ball, age)
        self.assertEqual(port.writes, [b'?', b'S'])

    def test_wrong_confirmation_ack_fails(self):
        port = FakeSerial([b"BALL_SERIAL_V5 DRIVE\n", b"ACK S\n", b"ACK V\n"])
        link = BrainLink('FAKE', serial_factory=lambda *a, **k: port)
        with self.assertRaisesRegex(RuntimeError, 'acknowledgement'):
            link.send_confirmation(True, 100)
