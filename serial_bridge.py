"""Laptop side of the BALL_SERIAL_V1 protocol; no motor commands on import."""
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
        if serial_factory is None:
            import serial
            serial_factory = serial.Serial
        self.connection = serial_factory(port, baudrate=115200, timeout=0.05, write_timeout=0.1)
        try:
            self.connection.reset_input_buffer()
            self.connection.reset_output_buffer()
            self.connection.write(b"?")
            self.mode = self._wait_for(b"BALL_SERIAL_V1 ", 3.0).decode("ascii")
            if self.mode not in ("BALL_SERIAL_V1 TEST", "BALL_SERIAL_V1 DRIVE"):
                raise RuntimeError("Unexpected receiver mode")
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
        if command not in ("L", "R", "F", "S"):
            raise ValueError("Unknown command")
        self.connection.write(command.encode("ascii"))
        reply = self._wait_for(b"ACK ", 0.5)
        if reply != f"ACK {command}".encode("ascii"):
            raise RuntimeError("Unexpected acknowledgement; stopping the serial session")

    def close(self):
        try:
            self.connection.write(b"S")
        except Exception:
            pass  # The Brain also stops after 300 ms without valid commands.
        finally:
            self.connection.close()
