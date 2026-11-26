"""Coordinate protocol for the separate slot 5 CV collector. Never arms motion."""
import time


def configuration_values(config):
    values = [round(config['aim_x'] * 1000), round(config['commit_y'] * 1000),
              config['finish_mm'], config['area_length_mm'], config['area_width_mm'],
              config['steer_sign'], config['approach_mm_s'], config['pickup_mm_s']]
    limits = [(200, 800), (400, 950), (40, 450), (1200, 6000), (1200, 5000),
              (-1, 1), (80, 400), (60, 240)]
    if any(type(v) is not int or not low <= v <= high for v, (low, high) in zip(values, limits)) or values[5] == 0:
        raise ValueError('Invalid slot 5 calibration or movement settings')
    if not 0 < config['retain_confidence'] <= config['acquire_confidence'] <= 1:
        raise ValueError('Invalid detection confidence thresholds')
    return values


class CoordinateLink:
    def __init__(self, port, config, serial_factory=None):
        values = configuration_values(config)
        if not config['calibrated']:
            raise ValueError('Calibrate the mounted camera before connecting for movement')
        if serial_factory is None:
            from serial import Serial
            serial_factory = Serial
        self.connection = serial_factory(port, baudrate=115200, timeout=.05, write_timeout=.1)
        self.sequence = 0
        self.state, self.reason, self.attempts, self.seconds = 'IDLE', 'START_WITH_LED', 0, 300
        try:
            self.connection.reset_input_buffer()
            self.connection.write(b'?')
            if self._reply(b'CV_COORD_', 3) != b'CV_COORD_V1 DRIVE':
                raise RuntimeError('Unexpected slot 5 receiver version')
            self.connection.write(('C ' + ' '.join(map(str, values)) + '\n').encode('ascii'))
            if self._reply(b'ACK ', .7) != b'ACK C':
                raise RuntimeError('Slot 5 refused configuration; stop the Brain program and restart it')
        except BaseException:
            self.close()
            raise

    def _reply(self, prefix, timeout):
        deadline = time.monotonic() + timeout
        pending = bytearray()
        while time.monotonic() < deadline:
            pending.extend(self.connection.read_until(b'\n', 192))
            while b'\n' in pending:
                line, _, remaining = pending.partition(b'\n')
                pending = bytearray(remaining)
                line = bytes(line.strip())
                if line.startswith(b'ERR'):
                    raise RuntimeError('Brain rejected packet: ' + line.decode('ascii', errors='replace'))
                if line.startswith(b'BALL_SERIAL_'):
                    raise RuntimeError('Slot 4 is running. Run cv-collector in PROGRAM SLOT 5 instead.')
                if line.startswith(prefix):
                    return line
            if len(pending) > 384:
                raise RuntimeError('Invalid serial response')
        raise RuntimeError('No slot 5 reply. Run cv-collector in slot 5 and close the VEX serial monitor.')

    def send_frame(self, target, identity, width, height, age_ms):
        if type(age_ms) is not int or not 0 <= age_ms <= 5000 or width <= 0 or height <= 0:
            raise ValueError('Invalid frame dimensions or age')
        self.sequence += 1
        coords = [0, 0, 0, 0, 0]
        if target:
            x1, y1, x2, y2 = target.box
            coords = [round((x1+x2)/2/width*1000), round((y1+y2)/2/height*1000),
                      round((x2-x1)/width*1000), round((y2-y1)/height*1000),
                      round(target.confidence*1000)]
            identity = target.identity
        fields = [self.sequence, identity, int(target is not None), *coords, age_ms]
        self.connection.write(('F ' + ' '.join(map(str, fields)) + '\n').encode('ascii'))
        reply = self._reply(b'ACK ', .7).decode('ascii').split()
        if len(reply) != 7 or reply[:3] != ['ACK', 'F', str(self.sequence)]:
            raise RuntimeError('Invalid coordinate acknowledgement')
        self.state, self.attempts, self.seconds, self.reason = reply[3], int(reply[4]), int(reply[5]), reply[6]

    def stop(self, wait=False):
        self.connection.write(b'X')
        if wait and self._reply(b'ACK ', .7) != b'ACK X':
            raise RuntimeError('Stop acknowledgement failed')

    def close(self):
        try:
            self.stop()
        except Exception:
            pass
        finally:
            self.connection.close()
