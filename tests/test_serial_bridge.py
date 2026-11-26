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
    def test_v4_continuous_steering_packet_and_stop(self):
        port, link = self.make_link([b"BALL_SERIAL_V4 DRIVE\n", b"ACK S\n", b"ACK V\n"])
        self.assertTrue(link.supports_vision_priority)
        link.send_vision("L", -350)
        self.assertEqual(port.writes[-1], b"V L -350 0\n")
        link.close()
        self.assertEqual(port.writes[-1], b"X")

    def test_v4_near_packet(self):
        port, link = self.make_link([b"BALL_SERIAL_V4 DRIVE\n", b"ACK S\n", b"ACK V\n"])
        link.send_vision("F", 50, True)
        self.assertEqual(port.writes[-1], b"V F 50 1\n")

    def test_v3_cannot_accept_new_packets(self):
        port, link = self.make_link([b"BALL_SERIAL_V3 DRIVE\n", b"ACK S\n"])
        self.assertFalse(link.supports_vision_priority)
        with self.assertRaisesRegex(RuntimeError, "vision-first"):
            link.send_vision("F")
        self.assertEqual(port.writes, [b"?", b"S"])

    def test_invalid_vision_packet_is_not_sent(self):
        port, link = self.make_link([b"BALL_SERIAL_V4 DRIVE\n", b"ACK S\n"])
        for command, error, near in [("F", 1001, False), ("L", -300, True),
                                     ("X", 0, False), ("F", float("nan"), False), ("N", 0, True)]:
            with self.assertRaises(ValueError):
                link.send_vision(command, error, near)
        self.assertEqual(port.writes, [b"?", b"S"])

    def test_vision_packet_needs_matching_ack(self):
        port, link = self.make_link([b"BALL_SERIAL_V4 DRIVE\n", b"ACK S\n", b"ACK F\n"])
        with self.assertRaisesRegex(RuntimeError, "acknowledgement"):
            link.send_vision("F")

    def test_v3_close_requests_mission_stop(self):
        port, link = self.make_link([b"BALL_SERIAL_V3 DRIVE\n", b"ACK S\n"])
        self.assertTrue(link.supports_area)
        link.close()
        self.assertEqual(port.writes[-1], b"X")

    def test_v3_stop_acknowledged(self):
        port, link = self.make_link([b"BALL_SERIAL_V3 DRIVE\n", b"ACK S\n", b"ACK X\n"])
        link.send("X")
        self.assertEqual(port.writes[-1], b"X")
    def test_v2_search_and_finish_need_matching_ack(self):
        port, link = self.make_link([b"BALL_SERIAL_V2 DRIVE\n", b"ACK S\n", b"ACK N\n", b"ACK C\n"])
        link.send("N")
        link.send("C")
        self.assertEqual(port.writes, [b"?", b"S", b"N", b"C"])

    def test_v1_never_receives_new_movement_commands(self):
        port, link = self.make_link([b"BALL_SERIAL_V1 DRIVE\n", b"ACK S\n"])
        with self.assertRaisesRegex(RuntimeError, "Update slot 4"):
            link.send("C")
        self.assertEqual(port.writes, [b"?", b"S"])

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
