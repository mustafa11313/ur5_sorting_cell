"""Render synthetic overhead depth images of boxes on a belt (for tests)."""
import math

import numpy as np

W, H = 640, 480
FX = FY = (W / 2) / math.tan(0.5)
K = np.array([[FX, 0, W / 2], [0, FY, H / 2], [0, 0, 1.0]])


def camera_pose(height=1.55, x=0.0, y=0.6):
    """T_world_optical for a camera looking straight down, image right = +x, image up = +y."""
    T = np.eye(4)
    # optical axes: x_opt = +x_world, y_opt = -y_world, z_opt = -z_world
    T[:3, :3] = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=float)
    T[:3, 3] = [x, y, height]
    return T


def render(boxes, T, belt_z=0.72):
    """boxes: list of (x, y, yaw, length, width, height). Returns a depth image [m]."""
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    rays = np.stack(((u - K[0, 2]) / FX, (v - K[1, 2]) / FY, np.ones_like(u, dtype=float)), -1)
    R, t = T[:3, :3], T[:3, 3]
    d_world = rays @ R.T
    depth = (belt_z - t[2]) / d_world[..., 2]           # belt plane, z along optical axis
    for x, y, yaw, length, width, height in boxes:
        top = belt_z + height
        s = (top - t[2]) / d_world[..., 2]
        px = t[0] + s * d_world[..., 0] - x
        py = t[1] + s * d_world[..., 1] - y
        c, sn = math.cos(yaw), math.sin(yaw)
        lx, ly = c * px + sn * py, -sn * px + c * py
        inside = (np.abs(lx) <= length / 2) & (np.abs(ly) <= width / 2)
        depth = np.where(inside, np.minimum(depth, s), depth)
    return depth.astype(np.float32)
