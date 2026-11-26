"""Authenticated phone-camera bridge for Cloudflare Tunnel; no robot commands."""
import argparse
import hmac
from http.client import HTTPConnection
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import secrets
import threading
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
SESSION_FILE = ROOT / '.runtime/phone-session.json'


class CameraState:
    def __init__(self):
        self.lock = threading.Lock()
        self.phone_key = secrets.token_urlsafe(32)
        self.consumer_key = secrets.token_urlsafe(32)
        self.client = None
        self.client_seen = 0
        self.sequence = 0
        self.pending = None
        self.latest = None

    def challenge(self, client, now):
        with self.lock:
            if self.client != client and now - self.client_seen < 2:
                return None
            self.client, self.client_seen = client, now
            self.sequence += 1
            self.pending = (self.sequence, now, client)
            return self.sequence

    def accept(self, client, sequence, jpeg, now):
        with self.lock:
            pending = self.pending
            if pending is None or pending[0] != sequence or pending[2] != client or now - pending[1] > 1:
                return False
            self.pending = None  # A request ID can be used only once.
        frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if frame is None or max(frame.shape[:2]) > 640:
            return False
        with self.lock:
            # Recheck ordering after decoding, since HTTP requests run in threads.
            if self.latest is None or sequence > self.latest[0]:
                self.latest = (sequence, pending[1], jpeg)
                self.client_seen = now
                return True
        return False


def server_for(state, port=8765):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'
        def setup(self):
            super().setup()
            self.connection.settimeout(5)

        def log_message(self, *args):
            pass  # Don't log session keys or frame requests.

        def authorized(self, key):
            return hmac.compare_digest(self.headers.get('Authorization', ''), 'Bearer ' + key)

        def reply(self, status, body=b'', kind='text/plain', headers=None):
            self.send_response(status)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Referrer-Policy', 'no-referrer')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Permissions-Policy', 'camera=(self), microphone=()')
            self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; connect-src 'self'; img-src 'self' data:; media-src 'self' blob:; frame-ancestors 'none'")
            for key, value in (headers or {}).items():
                self.send_header(key, str(value))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path in ('/', '/camera.js'):
                name = 'index.html' if self.path == '/' else 'camera.js'
                kind = 'text/html; charset=utf-8' if name.endswith('html') else 'text/javascript'
                return self.reply(200, (ROOT / 'phone-web' / name).read_bytes(), kind)
            if self.path == '/capture':
                if not self.authorized(state.phone_key):
                    return self.reply(401)
                client = self.headers.get('X-Phone-ID', '')
                if not 8 <= len(client) <= 64:
                    return self.reply(400)
                sequence = state.challenge(client, time.monotonic())
                if sequence is None:
                    return self.reply(409, b'Another phone is paired. Close its camera page first.')
                return self.reply(200, json.dumps({'id': sequence}).encode(), 'application/json')
            if self.path == '/frame':
                # The phone pairing key cannot read frames or access this endpoint.
                if not self.authorized(state.consumer_key):
                    return self.reply(401)
                with state.lock:
                    latest = state.latest
                if latest is None or time.monotonic() - latest[1] > .7:
                    return self.reply(503, b'No fresh phone frame')
                return self.reply(200, latest[2], 'image/jpeg',
                                  {'X-Frame-ID': latest[0], 'X-Capture-Started': latest[1]})
            return self.reply(404)

        def do_POST(self):
            if self.path != '/upload' or not self.authorized(state.phone_key):
                self.close_connection = True
                return self.reply(401)
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 8 <= size <= 1000000:
                    raise ValueError()
                payload = self.rfile.read(size)
                if len(payload) != size:
                    raise ValueError()
                sequence = int.from_bytes(payload[:4], 'big')
                client = self.headers.get('X-Phone-ID', '')
                valid = state.accept(client, sequence, payload[4:], time.monotonic())
                return self.reply(200 if valid else 409)
            except (ValueError, OSError):
                self.close_connection = True
                return self.reply(400)
    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    server.daemon_threads = True
    return server


class PhoneCapture:
    """OpenCV-style adapter retaining the original laptop request timestamp."""
    def __init__(self, session_file=SESSION_FILE):
        if not Path(session_file).is_file():
            raise RuntimeError('Run 11 - Pair phone camera.cmd before opening the detector.')
        session = json.loads(Path(session_file).read_text(encoding='utf-8'))
        self.port = int(session['port'])
        self.key = session['consumer_key']
        self.previous = 0
        self.closed = False

    def read_timed(self):
        deadline = time.monotonic() + .65
        while not self.closed and time.monotonic() < deadline:
            connection = HTTPConnection('127.0.0.1', self.port, timeout=.6)
            try:
                # This endpoint is always loopback HTTP. It needs no internet
                # proxy, HTTPS context, or phone/cloud credentials.
                connection.request('GET', '/frame', headers={'Authorization': 'Bearer ' + self.key})
                response = connection.getresponse()
                if response.status == 200:
                    sequence = int(response.getheader('X-Frame-ID'))
                    captured = float(response.getheader('X-Capture-Started'))
                    payload = response.read(1000001)
                    if sequence > self.previous and 0 <= time.monotonic() - captured <= .7:
                        frame = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
                        self.previous = sequence
                        return frame is not None, frame, captured
                elif response.status != 503:
                    raise RuntimeError('Phone pairing session changed. Restart the slot 5 camera launcher.')
            finally:
                connection.close()
            time.sleep(.015)
        return False, None, time.monotonic()

    def release(self):
        self.closed = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args()
    state = CameraState()
    server = server_for(state, args.port)
    SESSION_FILE.parent.mkdir(exist_ok=True)
    SESSION_FILE.write_text(json.dumps({'port': server.server_port, 'phone_key': state.phone_key,
                                        'consumer_key': state.consumer_key}), encoding='utf-8')
    print('Phone camera bridge listening on localhost. Frames stay in memory.', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
