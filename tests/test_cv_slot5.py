import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from cv_tracking import TargetLock, Target
from cv_serial import CoordinateLink, configuration_values
from update_cv_receiver import update, programs_from_json

ROOT = Path(__file__).resolve().parents[1]


class TrackerTests(unittest.TestCase):
    def test_requires_repeat_sighting(self):
        t = TargetLock()
        box = ([280, 280, 320, 320], .8)
        self.assertIsNone(t.update([box], 640, 480, 0))
        self.assertEqual(t.update([box], 640, 480, .1).identity, 1)

    def test_does_not_send_memory_as_observation_or_switch_to_other_ball(self):
        t = TargetLock()
        ball = ([100, 100, 140, 140], .8)
        t.update([ball], 640, 480, 0)
        chosen = t.update([ball], 640, 480, .1)
        self.assertIsNone(t.update([], 640, 480, .2))
        self.assertIsNone(t.update([([500,300,540,340], .99)], 640, 480, .3))
        self.assertEqual(t.identity, chosen.identity)
        self.assertEqual(t.update([ball], 640, 480, .4).identity, chosen.identity)

    def test_release_after_timeout_gets_new_identity(self):
        t = TargetLock()
        ball = ([100,100,140,140], .8)
        t.update([ball],640,480,0); t.update([ball],640,480,.1)
        self.assertIsNone(t.update([ball],640,480,2))
        self.assertEqual(t.update([ball],640,480,2.1).identity,2)

    def test_low_confidence_can_retain_but_cannot_acquire(self):
        t = TargetLock()
        box = [100,100,140,140]
        self.assertIsNone(t.update([(box,.35)],640,480,0))
        t.update([(box,.8)],640,480,.1)
        t.update([(box,.8)],640,480,.2)
        self.assertIsNotNone(t.update([(box,.35)],640,480,.3))

    def test_one_confident_frame_then_weak_frame_does_not_acquire(self):
        t=TargetLock()
        box=[100,100,140,140]
        self.assertIsNone(t.update([(box,.8)],640,480,0))
        self.assertIsNone(t.update([(box,.35)],640,480,.1))
        self.assertEqual(t.identity,0)

    def test_held_ball_below_commit_line_is_not_reacquired(self):
        t = TargetLock(acquire_y_max=.78)
        for i in range(10):
            self.assertIsNone(t.update([([300,400,340,440],.9)],640,480,i*.1))

    def test_existing_target_can_enter_pickup_zone(self):
        t = TargetLock(acquire_y_max=.78)
        for n,y in enumerate([300,330,360,390,400]):
            chosen=t.update([([300,y,340,y+40],.9)],640,480,n*.1)
        self.assertIsNotNone(chosen)

    def test_bad_boxes_do_not_acquire(self):
        t=TargetLock()
        for b in [[-1,10,20,30], [100,10,110,300], [float('nan'),0,30,30]]:
            self.assertIsNone(t.update([(b,.99)],640,480,0))


class FakeSerial:
    def __init__(self, *args, **kwargs):
        self.writes=[]; self.replies=[]; self.closed=False; self.mode=b'CV_COORD_V1 DRIVE\n'
    def reset_input_buffer(self): pass
    def write(self, data):
        self.writes.append(data)
        if data==b'?': self.replies.append(self.mode)
        elif data.startswith(b'C '): self.replies.append(b'ACK C\n')
        elif data.startswith(b'F '): self.replies.append(b'ACK F '+data.split()[1]+b' TRACK 0 299 RUNNING\n')
        elif data==b'X': self.replies.append(b'ACK X\n')
    def read_until(self, *args): return self.replies.pop(0) if self.replies else b''
    def close(self): self.closed=True


class SerialTests(unittest.TestCase):
    def setUp(self):
        self.config=json.loads((ROOT/'cv_slot5_phone.json').read_text())
        self.config['calibrated']=True
        self.serial=FakeSerial()
    def test_coordinates_and_freshness_sent_without_arm_command(self):
        link=CoordinateLink('TEST',self.config,lambda *a,**k:self.serial)
        link.send_frame(Target(7,(280,200,360,280),.8),7,640,480,180)
        self.assertEqual(self.serial.writes[-1],b'F 1 7 1 500 500 125 167 800 180\n')
        self.assertEqual(link.state,'TRACK')
        link.send_frame(None,7,640,480,200)
        self.assertEqual(self.serial.writes[-1],b'F 2 7 0 0 0 0 0 0 200\n')
        link.close(); self.assertEqual(self.serial.writes[-1],b'X'); self.assertTrue(self.serial.closed)
    def test_old_slot_four_receiver_is_rejected(self):
        self.serial.mode=b'BALL_SERIAL_V5 DRIVE\n'
        with self.assertRaisesRegex(RuntimeError,'Slot 4'):
            CoordinateLink('TEST',self.config,lambda *a,**k:self.serial)
        self.assertTrue(self.serial.closed)
        self.assertFalse(any(p.startswith(b'C ') for p in self.serial.writes))
    def test_uncalibrated_camera_never_opens_serial(self):
        self.config['calibrated']=False
        factory=Mock()
        with self.assertRaises(ValueError): CoordinateLink('TEST',self.config,factory)
        factory.assert_not_called()
    def test_limits_rejected(self):
        for field,value in [('finish_mm',10000),('steer_sign',0),('approach_mm_s',800),('aim_x',.99)]:
            c=dict(self.config); c[field]=value
            with self.assertRaises(ValueError): configuration_values(c)
    def test_space_stop_ack_does_not_poison_next_frame(self):
        link=CoordinateLink('TEST',self.config,lambda *a,**k:self.serial)
        link.stop(wait=True); link.send_frame(None,0,640,480,100)


class InstallTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name); self.project=self.root/'vex-cv'
        (self.project/'.vscode').mkdir(parents=True); (self.project/'build').mkdir()
        self.settings=self.project/'.vscode/vex_project_settings.json'
        self.settings.write_text(json.dumps({'project':{'slot':5,'name':'cv-collector','platform':'IQ2'}}))
        (self.project/'build/cv-collector.bin').write_bytes(b'CV SLOT 5 V1\0CV_COORD_V1 DRIVE'+b'\0'*100)
        self.ports=[SimpleNamespace(device='COM7',description='VEX IQ Communications Port')]
    def info(self, name=None):
        items=[{'slot':0,'file':'original','size':10,'type':'bin'}]
        if name: items.append({'slot':4,'file':name,'size':100,'type':'bin'})
        return SimpleNamespace(returncode=0,stdout=json.dumps({'iq2':{'programs':{'items':items}}}))
    def test_other_program_in_slot_five_is_never_overwritten(self):
        run=Mock(side_effect=[SimpleNamespace(returncode=0),self.info('other-project')])
        with self.assertRaisesRegex(RuntimeError,'not overwritten'): update(self.root,run,self.ports)
        self.assertEqual(run.call_count,2)
    def test_success_installs_only_slot_five_and_verifies_other_slots(self):
        run=Mock(side_effect=[SimpleNamespace(returncode=0),self.info(),SimpleNamespace(returncode=0),self.info('cv-collector')])
        update(self.root,run,self.ports)
        self.assertEqual(run.call_args_list[2].args[0][1:3],['--slot','5'])
        self.assertEqual(run.call_count,4)
    def test_build_failure_cannot_fall_back_to_old_binary(self):
        run=Mock(return_value=SimpleNamespace(returncode=1))
        with self.assertRaisesRegex(RuntimeError,'Build failed'): update(self.root,run,self.ports)
        self.assertEqual(run.call_count,1)
    def test_wrong_slot_blocks_even_build(self):
        self.settings.write_text(json.dumps({'project':{'slot':1,'name':'cv-collector','platform':'IQ2'}}))
        run=Mock()
        with self.assertRaises(RuntimeError): update(self.root,run,self.ports)
        run.assert_not_called()
    def test_unknown_slot_listing_blocks_download(self):
        with self.assertRaises(RuntimeError): programs_from_json('{"unexpected":true}')


if __name__=='__main__': unittest.main()
