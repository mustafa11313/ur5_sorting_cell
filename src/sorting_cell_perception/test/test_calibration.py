import math

import cv2
import numpy as np
from synthetic import K, camera_pose

from sorting_cell_perception.calibrate_camera import (average_poses, marker_corners_world,
                                                      solve_camera_pose)
from sorting_cell_perception.ros_utils import rpy_to_matrix


def test_pnp_recovers_a_bumped_camera():
    T_true = camera_pose(height=1.53, x=0.01, y=0.61)
    T_true[:3, :3] = T_true[:3, :3] @ rpy_to_matrix(0.02, -0.015, 0.03)
    world = np.vstack([marker_corners_world(-0.3, 0.835, 0.745, 0.0, 0.08),
                       marker_corners_world(0.3, 0.835, 0.745, 0.0, 0.08)])
    T_cw = np.linalg.inv(T_true)
    rvec, _ = cv2.Rodrigues(T_cw[:3, :3])
    img, _ = cv2.projectPoints(world, rvec, T_cw[:3, 3], K, None)
    img = img.reshape(-1, 2) + np.random.default_rng(1).normal(0, 0.2, (8, 2))
    T_est, err = solve_camera_pose(img, world, K)
    assert err < 1.0
    assert np.linalg.norm(T_est[:3, 3] - T_true[:3, 3]) < 0.01
    dR = T_est[:3, :3].T @ T_true[:3, :3]
    assert math.degrees(math.acos(min(1.0, (np.trace(dR) - 1) / 2))) < 0.5


def test_average_poses_is_a_rotation():
    a = camera_pose()
    b = camera_pose()
    b[:3, :3] = b[:3, :3] @ rpy_to_matrix(0, 0, 0.02)
    T = average_poses([a, b])
    assert np.allclose(T[:3, :3] @ T[:3, :3].T, np.eye(3), atol=1e-9)
