"""Launch a temporary Cloudflare camera page and show a local pairing QR."""
import html
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
RUNTIME = ROOT / '.runtime'


def find_cloudflared():
    found = shutil.which('cloudflared')
    if found:
        return found
    candidates = [Path(os.environ.get('LOCALAPPDATA', '')) / 'Microsoft/WinGet/Links/cloudflared.exe',
                  Path('C:/Program Files (x86)/cloudflared/cloudflared.exe'),
                  Path('C:/Program Files/cloudflared/cloudflared.exe')]
    return next((str(p) for p in candidates if p.is_file()), None)


def pairing_page(url, key, runtime=RUNTIME):
    link = url.rstrip('/') + '/#key=' + key
    qr = cv2.QRCodeEncoder_create().encode(link)
    qr = np.pad(qr, 4, constant_values=255)
    qr = cv2.resize(qr, None, fx=6, fy=6, interpolation=cv2.INTER_NEAREST)
    if not cv2.imwrite(str(runtime / 'phone-qr.png'), qr):
        raise RuntimeError('Could not save pairing QR')
    page = runtime / 'phone-pairing.html'
    page.write_text('''<!doctype html><meta charset="utf-8"><title>Pair robot camera</title>
<style>body{font:18px system-ui;max-width:700px;margin:40px auto;padding:20px;background:#10202b;color:#efffff}img{max-width:90%;background:white;padding:12px;border-radius:12px}a{color:#74eac1;overflow-wrap:anywhere}p{line-height:1.6}</style>
<h1>Pair your phone camera</h1><p>Scan this QR code with your phone. Open the link, tap <b>Start camera</b>, and allow camera access.</p>
<img src="phone-qr.png" alt="Phone camera pairing QR code">
<p>Keep the camera page open. Mount the phone before calibration.</p>
<p>On the laptop, run <b>12 - Preview phone camera.cmd</b> to test video without the robot, or <b>7 - Start CV slot 5.cmd</b> when the Brain is ready.</p>
<p>This private pairing link changes when the bridge restarts. Keep the pairing terminal open.</p>
<p><a href="''' + html.escape(link, quote=True) + '">Open camera page</a></p>', encoding='utf-8')
    return page


def main():
    executable = find_cloudflared()
    if not executable:
        raise RuntimeError('Cloudflare Tunnel is not installed. Run 10 - Setup phone camera.cmd first.')
    RUNTIME.mkdir(exist_ok=True)
    children = []
    flags = subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0
    started = time.time()
    try:
        with (RUNTIME / 'phone-bridge.log').open('w') as server_log, (RUNTIME / 'cloudflare.log').open('w') as tunnel_log:
            server = subprocess.Popen([sys.executable, str(ROOT / 'phone_camera.py')], cwd=ROOT,
                                      stdout=server_log, stderr=subprocess.STDOUT, creationflags=flags)
            children.append(server)
            session_file = RUNTIME / 'phone-session.json'
            deadline = time.monotonic() + 12
            session = None
            while time.monotonic() < deadline:
                if server.poll() is not None:
                    raise RuntimeError('Camera bridge could not start. Close an earlier pairing session; see .runtime/phone-bridge.log.')
                if session_file.is_file() and session_file.stat().st_mtime >= started:
                    try:
                        session = json.loads(session_file.read_text())
                        break
                    except (ValueError, OSError):
                        pass
                time.sleep(.1)
            if session is None:
                raise RuntimeError('Camera bridge startup timed out')
            tunnel = subprocess.Popen([executable, '--no-autoupdate', 'tunnel', '--url',
                                       f"http://127.0.0.1:{session['port']}", '--protocol', 'http2'], cwd=ROOT,
                                      stdout=tunnel_log, stderr=subprocess.STDOUT, creationflags=flags)
            children.append(tunnel)
            print('Opening a temporary Cloudflare HTTPS tunnel...', flush=True)
            deadline = time.monotonic() + 60
            while time.monotonic() < deadline:
                if tunnel.poll() is not None:
                    raise RuntimeError('Cloudflare tunnel failed. See .runtime/cloudflare.log for the connection error.')
                log = (RUNTIME / 'cloudflare.log').read_text(errors='replace')
                match = re.search(r'https://[a-z0-9-]+\.trycloudflare\.com', log)
                if match:
                    page = pairing_page(match.group(), session['phone_key'])
                    os.startfile(str(page))
                    print('Pairing QR opened in your browser. Keep this terminal open. Ctrl+C stops the bridge.', flush=True)
                    break
                time.sleep(.2)
            else:
                raise RuntimeError('Cloudflare did not provide a URL. Check internet/firewall access; see .runtime/cloudflare.log.')
            while server.poll() is None and tunnel.poll() is None:
                time.sleep(.25)
            raise RuntimeError('The camera bridge or Cloudflare tunnel exited. Reopen the pairing launcher.')
    finally:
        for child in reversed(children):
            if child.poll() is None:
                child.terminate()
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    child.kill()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Phone camera bridge stopped.')
    except (RuntimeError, OSError) as error:
        raise SystemExit(str(error)) from None
