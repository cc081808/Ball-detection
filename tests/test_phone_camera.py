import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from contextlib import contextmanager
from http.client import HTTPConnection

import cv2
import numpy as np
from phone_camera import CameraState, PhoneCapture, server_for
from start_phone_camera import pairing_page


class PhoneCameraTests(unittest.TestCase):
    def setUp(self):
        self.state=CameraState()
        self.jpeg=cv2.imencode('.jpg',np.full((48,64,3),120,np.uint8))[1].tobytes()
    def test_frame_keeps_request_time_and_cannot_be_replayed(self):
        seq=self.state.challenge('phone-one',10)
        self.assertTrue(self.state.accept('phone-one',seq,self.jpeg,10.2))
        self.assertEqual(self.state.latest[1],10)
        self.assertFalse(self.state.accept('phone-one',seq,self.jpeg,10.3))
    def test_expired_or_wrong_phone_cannot_supply_fresh_frame(self):
        seq=self.state.challenge('phone-one',10)
        self.assertFalse(self.state.accept('phone-two',seq,self.jpeg,10.1))
        self.assertFalse(self.state.accept('phone-one',seq,self.jpeg,11.1))
        self.assertIsNone(self.state.latest)
    def test_only_one_phone_and_no_superseded_request(self):
        first=self.state.challenge('phone-one',10)
        self.assertIsNone(self.state.challenge('phone-two',10.1))
        second=self.state.challenge('phone-one',10.2)
        self.assertFalse(self.state.accept('phone-one',first,self.jpeg,10.3))
        self.assertTrue(self.state.accept('phone-one',second,self.jpeg,10.3))
    def test_invalid_image_not_published(self):
        seq=self.state.challenge('phone-one',10)
        self.assertFalse(self.state.accept('phone-one',seq,b'not-jpeg',10.1))
    def test_oversized_image_not_published(self):
        data=cv2.imencode('.jpg',np.zeros((700,700,3),np.uint8))[1].tobytes()
        seq=self.state.challenge('phone-one',10)
        self.assertFalse(self.state.accept('phone-one',seq,data,10.1))
    def test_pairing_qr_decodes_to_exact_https_link(self):
        with tempfile.TemporaryDirectory() as folder:
            runtime=Path(folder)
            pairing_page('https://camera-example.trycloudflare.com','test-pairing-key',runtime)
            result=cv2.QRCodeDetector().detectAndDecode(cv2.imread(str(runtime/'phone-qr.png')))[0]
            self.assertEqual(result,'https://camera-example.trycloudflare.com/#key=test-pairing-key')
            self.assertIn('phone-qr.png',(runtime/'phone-pairing.html').read_text())
    def test_http_pairing_upload_and_laptop_capture(self):
        server=server_for(self.state,0)
        thread=threading.Thread(target=server.serve_forever,daemon=True); thread.start()
        @contextmanager
        def request(path, headers=None, data=None):
            connection=HTTPConnection('127.0.0.1',server.server_port,timeout=2)
            try:
                connection.request('GET' if data is None else 'POST',path,body=data,headers=headers or {})
                yield connection.getresponse()
            finally:
                connection.close()
        try:
            with request('/') as response:
                self.assertIn(b'Start camera',response.read())
            with request('/capture') as response:
                self.assertEqual(response.status,401)
            headers={'Authorization':'Bearer '+self.state.phone_key,'X-Phone-ID':'phone-client-1'}
            with request('/capture',headers=headers) as response:
                seq=json.load(response)['id']
            sent=time.monotonic()
            with request('/upload',headers=headers,data=seq.to_bytes(4,'big')+self.jpeg) as response:
                self.assertEqual(response.status,200)
            with request('/frame',headers=headers) as response:
                self.assertEqual(response.status,401) # phone key cannot retrieve video
            with tempfile.TemporaryDirectory() as folder:
                session=Path(folder)/'session.json'
                session.write_text(json.dumps({'port':server.server_port,'consumer_key':self.state.consumer_key}))
                capture=PhoneCapture(session)
                ok,frame,stamp=capture.read_timed()
                self.assertTrue(ok); self.assertEqual(frame.shape,(48,64,3)); self.assertLessEqual(stamp,sent)
                capture.release()
                self.assertFalse(capture.read_timed()[0])
        finally:
            server.shutdown(); server.server_close(); thread.join(timeout=2)


if __name__=='__main__': unittest.main()
