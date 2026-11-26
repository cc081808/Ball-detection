import unittest
from collection_policy import CollectionPolicy


class CollectionTests(unittest.TestCase):
    near = [290, 280, 350, 400]
    far = [290, 100, 350, 160]

    def test_proportional_error_and_mirror(self):
        p, mirrored = CollectionPolicy(), CollectionPolicy(mirror=True)
        self.assertEqual(self.send(p, [200, 100, 240, 160], 0), "L")
        self.assertEqual(self.send(mirrored, [200, 100, 240, 160], 0), "R")
        self.assertLess(p.error_milli, 0)
        self.assertEqual(mirrored.error_milli, -p.error_milli)
        self.send(p, None, .1)
        self.assertEqual(p.error_milli, 0)

    def test_close_flag_requires_two_centred_observations(self):
        p = CollectionPolicy()
        self.send(p, self.near, 0)
        self.assertLess(p.near_frames, 2)
        self.send(p, self.near, .1)
        self.assertGreaterEqual(p.near_frames, 2)
        self.send(p, [200, 280, 240, 400], .2)
        self.assertEqual(p.near_frames, 0)

    def send(self, policy, bounds, now):
        return policy.command(bounds, 640, 480, now)

    def test_live_no_target_searches(self):
        self.assertEqual(self.send(CollectionPolicy(), None, 0), "N")

    def test_centred_close_ball_finishes_then_searches(self):
        p = CollectionPolicy()
        self.assertEqual(self.send(p, self.near, 0), "F")
        self.assertEqual(self.send(p, self.near, .1), "F")
        self.assertEqual(self.send(p, None, .2), "C")
        for t in [.3, .5, .7]:
            self.assertEqual(self.send(p, None, t), "C")
        self.assertEqual(self.send(p, None, .81), "N")
        self.assertEqual(self.send(p, None, .9), "N")

    def test_far_or_offcentre_or_single_frame_loss_never_advances(self):
        for bounds, count in [(self.far, 2), ([10, 280, 70, 400], 2), (self.near, 1)]:
            p = CollectionPolicy()
            for i in range(count): self.send(p, bounds, i * .1)
            self.assertEqual(self.send(p, None, .2), "N")

    def test_long_gap_cancels_close_ball_memory(self):
        p = CollectionPolicy()
        self.send(p, self.near, 0)
        self.send(p, self.near, .1)
        self.assertEqual(self.send(p, None, .5), "N")

    def test_reacquisition_overrides_finish_immediately(self):
        p = CollectionPolicy()
        self.send(p, self.near, 0)
        self.send(p, self.near, .1)
        self.send(p, None, .2)
        self.assertEqual(self.send(p, [10, 280, 70, 400], .3), "L")
        self.assertEqual(self.send(p, None, .4), "N")

    def test_invalid_bounds_stop_and_clear_memory(self):
        p = CollectionPolicy()
        self.send(p, self.near, 0)
        self.send(p, self.near, .1)
        self.assertEqual(self.send(p, [290, 280, 350, 600], .2), "S")
        self.assertEqual(self.send(p, None, .3), "N")
