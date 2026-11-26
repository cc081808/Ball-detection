"""Laptop side of the versioned ball-vision protocol; no actions on import."""
import math
import time


def command_for_detection(xyxy, frame_width, deadband=0.15, mirror=False):
    if xyxy is None:
        return "S"
    if not math.isfinite(frame_width) or frame_width <= 0:
        raise ValueError("Frame width must be positive")
    if len(xyxy) != 4 or not all(math.isfinite(v) for v in xyxy):
        return "S"
    x1, y1, x2, y2 = xyxy
    if not (0 <= x1 < x2 <= frame_width and 0 <= y1 < y2):
        return "S"
    error = (x1 + x2) / frame_width - 1
    if mirror:
        error = -error
    return "L" if error < -deadband else "R" if error > deadband else "F"


def choose_port():
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    if not ports:
        raise RuntimeError("No COM ports found. Turn on the Brain and connect a USB data cable.")
    user_ports = [p for p in ports if "vex" in p.description.lower() and "user" in p.description.lower()]
    if len(user_ports) == 1:
        print(f"Using {user_ports[0].device}: {user_ports[0].description}")
        return user_ports[0].device
    for number, port in enumerate(ports, 1):
        print(f"{number}. {port.device}: {port.description}")
    print("Choose the VEX user/console port, not the system/download port.")
    while True:
        choice = input("Port number: ").strip()
        if choice.isdecimal() and 1 <= int(choice) <= len(ports):
            return ports[int(choice) - 1].device


class BrainLink:
    def __init__(self, port, serial_factory=None):
        self.brain_state = "Update receiver for status"
        self.last_check = "Unavailable"
        if serial_factory is None:
            import serial
            serial_factory = serial.Serial
        self.connection = serial_factory(port, baudrate=115200, timeout=0.05, write_timeout=0.1)
        try:
            self.connection.reset_input_buffer()
            self.connection.reset_output_buffer()
            self.connection.write(b"?")
            self.mode = self._wait_for(b"BALL_SERIAL_V", 3.0).decode("ascii")
            if self.mode not in ("BALL_SERIAL_V1 TEST", "BALL_SERIAL_V1 DRIVE",
                                 "BALL_SERIAL_V2 TEST", "BALL_SERIAL_V2 DRIVE",
                                 "BALL_SERIAL_V3 TEST", "BALL_SERIAL_V3 DRIVE",
                                 "BALL_SERIAL_V4 TEST", "BALL_SERIAL_V4 DRIVE",
                                 "BALL_SERIAL_V5 TEST", "BALL_SERIAL_V5 DRIVE"):
                raise RuntimeError("Unexpected receiver mode")
            self.supports_vision_priority = self.mode.startswith("BALL_SERIAL_V4 ")
            self.supports_confirmation = self.mode.startswith("BALL_SERIAL_V5 ")
            self.supports_area = self.supports_confirmation or self.supports_vision_priority or self.mode.startswith("BALL_SERIAL_V3 ")
            self.supports_collection = self.supports_area or self.mode.startswith("BALL_SERIAL_V2 ")
            self.send("S")
        except Exception:
            # An unidentified port never receives a movement command.
            self.connection.close()
            raise

    def _wait_for(self, prefix, timeout):
        deadline = time.monotonic() + timeout
        pending = bytearray()
        while time.monotonic() < deadline:
            chunk = self.connection.read_until(b"\n", 128)
            pending.extend(chunk)
            while b"\n" in pending:
                line, _, rest = pending.partition(b"\n")
                pending = bytearray(rest)
                line = line.strip()
                if line.startswith(prefix):
                    return bytes(line)
            if len(pending) > 256:
                raise RuntimeError("Unexpected serial data; check the selected port")
        raise RuntimeError("No receiver reply. Run ball-serial in slot 4, select its user/console COM port, and close the VEX serial monitor.")

    def send(self, command):
        if command not in ("L", "R", "F", "S", "N", "C", "X"):
            raise ValueError("Unknown command")
        if self.supports_confirmation and command not in ("S", "X"):
            raise ValueError("Confirmation-only firmware does not accept steering commands")
        if command in ("N", "C") and not self.supports_collection:
            raise RuntimeError("Run 5 - Update slot 4.cmd to install search and intake-finish support.")
        if command == "X" and not self.supports_area:
            raise RuntimeError("This receiver does not support mission stop.")
        self.connection.write(command.encode("ascii"))
        reply = self._wait_for(b"ACK ", 0.5)
        if reply != f"ACK {command}".encode("ascii"):
            raise RuntimeError("Unexpected acknowledgement; stopping the serial session")

    def close(self):
        try:
            self.connection.write(b"X" if self.supports_area else b"S")
        except Exception:
            pass  # No stale data can approve another candidate in V5.
        finally:
            self.connection.close()

    def send_vision(self, command, error_milli=0, near=False):
        if not self.supports_vision_priority:
            raise RuntimeError("Run 5 - Update slot 4.cmd to install vision-first V4 firmware.")
        if (command not in ("L", "R", "F", "N", "C", "S") or
                type(error_milli) is not int or not -1000 <= error_milli <= 1000 or
                type(near) is not bool or (near and (command != "F" or abs(error_milli) > 150))):
            raise ValueError("Invalid vision observation")
        self.connection.write(f"V {command} {error_milli} {int(near)}\n".encode("ascii"))
        if self._wait_for(b"ACK ", .5) != b"ACK V":
            raise RuntimeError("Unexpected vision acknowledgement; stopping the serial session")

    def send_confirmation(self, ball, frame_age_ms):
        if not self.supports_confirmation:
            raise RuntimeError("Run 5 - Update slot 4.cmd to install sensor + CV confirmation firmware.")
        if type(ball) is not bool or type(frame_age_ms) is not int or not 0 <= frame_age_ms <= 5000:
            raise ValueError("Invalid confirmation observation")
        self.connection.write(f"B {int(ball)} {frame_age_ms}\n".encode("ascii"))
        reply = self._wait_for(b"ACK ", .5)
        if reply == b"ACK B":
            return  # Earlier V5 build: confirmation works but no status telemetry.
        if not reply.startswith(b"ACK B ") or reply[6:].count(b"|") != 1:
            raise RuntimeError("Unexpected confirmation acknowledgement; stopping the serial session")
        state, event = reply[6:].decode("ascii").split("|", 1)
        if not state or not event:
            raise RuntimeError("Incomplete Brain status; stopping the serial session")
        self.brain_state, self.last_check = state, event
