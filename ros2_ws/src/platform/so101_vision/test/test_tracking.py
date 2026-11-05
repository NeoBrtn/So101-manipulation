import numpy as np

from so101_vision.tracking import Tracker


def test_rejects_partly_hidden_blob():
    t = Tracker()
    assert t.update([0.2, 0.0, 0.025], area=100)
    assert not t.update([0.3, 0.0, 0.025], area=40)   # small blob: occluded
    assert np.allclose(t.estimate, [0.2, 0.0, 0.025])


def test_glitch_ignored_but_real_move_accepted():
    t = Tracker(confirm_frames=3)
    for _ in range(3):
        t.update([0.2, 0.0, 0.0], 100)
    assert not t.update([0.3, 0.0, 0.0], 100)          # one far detection: glitch?
    assert not t.update([0.3, 0.0, 0.0], 100)
    assert t.update([0.3, 0.0, 0.0], 100)              # 3 agreeing: it moved
    assert np.allclose(t.estimate, [0.3, 0.0, 0.0])


def test_median_filters_noise():
    t = Tracker()
    for x in (0.200, 0.201, 0.215, 0.199, 0.200):
        t.update([x, 0, 0], 100)
    assert abs(t.estimate[0] - 0.200) < 0.0015
