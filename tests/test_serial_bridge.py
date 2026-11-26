import unittest
from unittest.mock import patch

from serial_bridge import BrainLink, command_for_detection


class FakeSerial:
    def __init__(self, chunks):
        self.chunks = list(chunks)
        self.writes = []
        self.closed = False

    def reset_input_buffer(self):
        pass

    def reset_output_buffer(self):
        pass

    def write(self, value):
        self.writes.append(value)
        return len(value)

    def read_until(self, *args):
        return self.chunks.pop(0) if self.chunks else b""

    def close(self):
        self.closed = True


class SteeringTests(unittest.TestCase):
    def test_missing_target_stops(self):
        self.assertEqual(command_for_detection(None, 640), "S")

    def test_left_center_right(self):
        for bounds, expected in [([0, 10, 100, 100], "L"), ([300, 10, 340, 100], "F"), ([540, 10, 640, 100], "R")]:
            with self.subTest(expected=expected):
                self.assertEqual(command_for_detection(bounds, 640), expected)

    def test_deadband_avoids_small_corrections(self):
        self.assertEqual(command_for_detection([280, 10, 320, 100], 640), "F")
        self.assertEqual(command_for_detection([320, 10, 360, 100], 640), "F")

    def test_mirror_changes_steering_not_forward(self):
        self.assertEqual(command_for_detection([0, 10, 100, 100], 640, mirror=True), "R")
        self.assertEqual(command_for_detection([540, 10, 640, 100], 640, mirror=True), "L")
        self.assertEqual(command_for_detection([300, 10, 340, 100], 640, mirror=True), "F")

    def test_invalid_boxes_stop(self):
        for bounds in ([float("nan"), 0, 20, 20], [10, 10, 5, 20], [-1, 0, 20, 20], [0, 0, 641, 20], [0, 0, 20, 0]):
            with self.subTest(bounds=bounds):
                self.assertEqual(command_for_detection(bounds, 640), "S")


class LinkTests(unittest.TestCase):
    def make_link(self, chunks):
        port = FakeSerial(chunks)
        link = BrainLink("FAKE", serial_factory=lambda *a, **k: port)
        return port, link

    def test_split_handshake_and_ack(self):
        port, link = self.make_link([b"noise\nBALL_SER", b"IAL_V1 TEST\n", b"ACK", b" S\n"])
        self.assertEqual(link.mode, "BALL_SERIAL_V1 TEST")
        self.assertEqual(port.writes, [b"?", b"S"])
        link.close()
        self.assertEqual(port.writes[-1], b"S")
        self.assertTrue(port.closed)

    def test_handshake_required_before_movement(self):
        port = FakeSerial([])
        with patch("serial_bridge.time.monotonic", side_effect=[0, 4]):
            with self.assertRaisesRegex(RuntimeError, "No receiver reply"):
                BrainLink("FAKE", serial_factory=lambda *a, **k: port)
        self.assertEqual(port.writes, [b"?"])
        self.assertTrue(port.closed)

    def test_unknown_mode_closes_port(self):
        port = FakeSerial([b"BALL_SERIAL_V1 OTHER\n"])
        with self.assertRaisesRegex(RuntimeError, "mode"):
            BrainLink("FAKE", serial_factory=lambda *a, **k: port)
        self.assertEqual(port.writes, [b"?"])
        self.assertTrue(port.closed)

    def test_commands_need_matching_ack(self):
        port, link = self.make_link([b"BALL_SERIAL_V1 TEST\n", b"ACK S\n", b"ACK R\n"])
        with self.assertRaisesRegex(RuntimeError, "acknowledgement"):
            link.send("L")
        link.close()
        self.assertEqual(port.writes[-1], b"S")

    def test_command_timeout(self):
        port, link = self.make_link([b"BALL_SERIAL_V1 TEST\n", b"ACK S\n"])
        with patch("serial_bridge.time.monotonic", side_effect=[0, 1]):
            with self.assertRaisesRegex(RuntimeError, "No receiver reply"):
                link.send("F")
        link.close()
        self.assertTrue(port.closed)

    def test_bad_command_not_transmitted(self):
        port, link = self.make_link([b"BALL_SERIAL_V1 TEST\n", b"ACK S\n"])
        before = len(port.writes)
        with self.assertRaises(ValueError):
            link.send("LF")
        self.assertEqual(len(port.writes), before)
        link.close()


if __name__ == "__main__":
    unittest.main()
