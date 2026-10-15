import math

import numpy as np
from synthetic import K, camera_pose, render

from sorting_cell_perception.geometry import BeltRegion, segment


def angle_diff_mod(a, b, period):
    d = (a - b) % period
    return min(d, period - d)


def test_single_box_pose_and_size():
    T = camera_pose()
    depth = render([(0.05, 0.62, 0.4, 0.06, 0.03, 0.025)], T)
    blobs, _, _ = segment(depth, K, T, BeltRegion())
    assert len(blobs) == 1
    b = blobs[0]
    assert np.linalg.norm(b.center[:2] - [0.05, 0.62]) < 0.003
    assert abs(b.height - 0.025) < 0.002
    assert abs(b.center[2] - (0.72 + 0.0125)) < 0.002
    assert abs(b.length - 0.06) < 0.004 and abs(b.width - 0.03) < 0.004
    assert angle_diff_mod(b.yaw, 0.4, math.pi) < math.radians(3)


def test_two_separate_parts_and_noise():
    T = camera_pose()
    depth = render([(-0.2, 0.58, 0.0, 0.04, 0.04, 0.04), (0.2, 0.64, 1.0, 0.04, 0.04, 0.04)], T)
    depth += np.random.default_rng(0).normal(0, 0.003, depth.shape).astype(np.float32)
    blobs, _, _ = segment(depth, K, T, BeltRegion())
    centres = sorted((round(b.center[0], 2), round(b.center[1], 2)) for b in blobs)
    assert len(blobs) == 2
    assert np.allclose(centres, [(-0.2, 0.58), (0.2, 0.64)], atol=0.01)


def test_parts_outside_belt_are_ignored():
    T = camera_pose()
    depth = render([(0.0, 0.9, 0.0, 0.05, 0.05, 0.03)], T)      # beyond the rail
    blobs, _, _ = segment(depth, K, T, BeltRegion())
    assert blobs == []
