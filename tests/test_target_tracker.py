import unittest
from target_tracker import StableTarget


class TrackingTests(unittest.TestCase):
    def test_single_frame_does_not_steer(self):
        t = StableTarget()
        self.assertIsNone(t.select([([280, 200, 330, 250], .8)], 640, 480, 0))
        self.assertIsNotNone(t.select([([283, 202, 333, 252], .8)], 640, 480, .1))

    def test_confidence_flicker_cannot_switch_target(self):
        t = StableTarget()
        a, b = [280, 200, 330, 250], [480, 200, 530, 250]
        t.select([(a, .8)], 640, 480, 0)
        self.assertEqual(t.select([(b, .99), (a, .51)], 640, 480, .1), a)

    def test_disappearance_never_resends_stale_bounds(self):
        t = StableTarget()
        a = [280, 200, 330, 250]
        t.select([(a, .8)], 640, 480, 0)
        t.select([(a, .8)], 640, 480, .1)
        self.assertIsNone(t.select([], 640, 480, .2))

    def test_lock_survives_other_target_during_recovery(self):
        t = StableTarget()
        a, b = [280, 200, 330, 250], [480, 200, 530, 250]
        t.select([(a, .8)], 640, 480, 0)
        t.select([(a, .8)], 640, 480, .1)
        self.assertIsNone(t.select([(b, .9)], 640, 480, .2))
        self.assertIsNone(t.select([(b, .9)], 640, 480, .3))
        self.assertEqual(t.select([(a, .7), (b, .99)], 640, 480, .4), a)

    def test_expired_lock_requires_confirmation_of_new_target(self):
        t = StableTarget()
        a, b = [280, 200, 330, 250], [480, 200, 530, 250]
        t.select([(a, .8)], 640, 480, 0)
        t.select([(a, .8)], 640, 480, .1)
        self.assertIsNone(t.select([(b, .9)], 640, 480, 1.4))
        self.assertEqual(t.select([(b, .9)], 640, 480, 1.5), b)

    def test_confirmed_ball_recovers_on_first_real_sighting(self):
        t = StableTarget()
        a = [280, 200, 330, 250]
        t.select([(a, .8)], 640, 480, 0)
        t.select([(a, .8)], 640, 480, .1)
        self.assertIsNone(t.select([], 640, 480, .2))
        self.assertEqual(t.select([(a, .8)], 640, 480, .3), a)

    def test_long_gap_resets_confirmation(self):
        t = StableTarget()
        a = [280, 200, 330, 250]
        t.select([(a, .8)], 640, 480, 0)
        self.assertIsNone(t.select([(a, .8)], 640, 480, .5))
