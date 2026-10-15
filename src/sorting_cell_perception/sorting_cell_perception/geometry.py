"""Depth-based part segmentation and pose estimation (no ROS dependencies).

The overhead camera sees parts lying on a belt whose surface height is known.
Every depth pixel is back-projected into the world frame; pixels that stick out
of the belt surface by a few millimetres belong to parts. Each connected blob
is then measured in metric world coordinates (centre, footprint rectangle,
height), which gives the grasp pose directly and is independent of colour and
lighting.
"""
from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass
class BeltRegion:
    """Area of the belt (world frame) where parts are searched for."""

    surface_z: float = 0.72
    center_y: float = 0.6
    width: float = 0.3
    min_x: float = -1.6
    max_x: float = 1.6
    margin: float = 0.012          # ignore the outermost strip next to the rails
    min_height: float = 0.006      # pixel must stick out this much [m]
    max_height: float = 0.09       # ignore the robot arm when it is above the belt


@dataclass
class Blob:
    """One segmented part candidate."""

    mask: np.ndarray                  # bool image mask
    bbox: tuple                       # x, y, w, h in pixels
    center: np.ndarray                # world xyz of the part centre
    yaw: float                        # angle of the footprint's long axis [rad]
    length: float                     # long side of the footprint [m]
    width: float                      # short side (closing width for the gripper) [m]
    height: float                     # height above the belt [m]
    fill_ratio: float                 # footprint area / (length * width)
    area_px: int
    extras: dict = field(default_factory=dict)


def backproject(depth, K):
    """Depth image (metres) -> Nx3 camera-frame points as an HxWx3 array."""
    h, w = depth.shape
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    u, v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
    z = depth.astype(np.float32)
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return np.dstack((x, y, z))


def to_world(points_cam, T_world_cam):
    """Transform HxWx3 camera points with a 4x4 homogeneous matrix."""
    R = T_world_cam[:3, :3].astype(np.float32)
    t = T_world_cam[:3, 3].astype(np.float32)
    return points_cam @ R.T + t


def height_map(depth, K, T_world_cam, belt: BeltRegion):
    """Return (world_points HxWx3, height above belt HxW, valid mask HxW)."""
    depth = np.nan_to_num(depth.astype(np.float32), nan=0.0, posinf=0.0, neginf=0.0)
    depth = cv2.medianBlur(depth, 5)        # suppress per-pixel sensor noise
    pts = to_world(backproject(depth, K), T_world_cam)
    height = pts[..., 2] - belt.surface_z
    valid = depth > 0.05
    valid &= np.abs(pts[..., 1] - belt.center_y) < belt.width / 2 - belt.margin
    valid &= (pts[..., 0] > belt.min_x) & (pts[..., 0] < belt.max_x)
    return pts, height, valid


def part_mask(height, valid, belt: BeltRegion):
    mask = valid & (height > belt.min_height) & (height < belt.max_height)
    mask = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    return mask.astype(bool)


def measure(mask, pts, height, extras=None):
    """Measure one blob mask in world coordinates. Returns a Blob or None."""
    ys, xs = np.nonzero(mask)
    if len(xs) < 15:
        return None
    h_vals = height[ys, xs]
    # use the top surface only: side walls seen in perspective would bias the footprint
    top = h_vals > 0.6 * float(np.percentile(h_vals, 90))
    top_h = float(np.median(h_vals[top]))
    xy = pts[ys[top], xs[top], :2].astype(np.float32)
    if len(xy) < 10:
        return None
    (cx, cy), (w, h), angle = cv2.minAreaRect(xy * 1000.0)   # mm for numeric stability
    w, h = w / 1000.0, h / 1000.0
    yaw = np.deg2rad(angle)
    if w < h:                       # make yaw the direction of the long side
        w, h = h, w
        yaw += np.pi / 2
    yaw = float(np.arctan2(np.sin(yaw), np.cos(yaw)))
    hull = cv2.convexHull(xy * 1000.0)
    area = cv2.contourArea(hull) / 1e6
    fill = float(area / max(w * h, 1e-9))
    x, y, bw, bh = cv2.boundingRect(np.column_stack((xs, ys)).astype(np.int32))
    center = np.array([cx / 1000.0, cy / 1000.0, 0.0])
    return Blob(mask=mask, bbox=(x, y, bw, bh), center=center, yaw=yaw, length=float(w),
                width=float(h), height=top_h, fill_ratio=fill, area_px=int(len(xs)),
                extras=extras or {})


def split_touching(mask, height, max_single_area_px):
    """Split a blob that contains several touching parts (distance transform + watershed)."""
    if mask.sum() < 1.5 * max_single_area_px:
        return [mask]
    m8 = mask.astype(np.uint8)
    # height steps between parts of different type are strong separators
    hq = (np.clip(height, 0, 0.1) * 1000).astype(np.uint8)
    edges = cv2.Canny(hq, 6, 12) > 0
    inner = m8.copy()
    inner[edges] = 0
    dist = cv2.distanceTransform(inner, cv2.DIST_L2, 5)
    if dist.max() <= 0:
        return [mask]
    _, peaks = cv2.threshold(dist, 0.55 * dist.max(), 255, cv2.THRESH_BINARY)
    n, markers = cv2.connectedComponents(peaks.astype(np.uint8))
    if n <= 2:
        return [mask]
    markers = markers + 1
    markers[m8 == 0] = 0
    unknown = (m8 > 0) & (peaks == 0)
    markers[unknown] = 0
    vis = cv2.cvtColor(hq * 2, cv2.COLOR_GRAY2BGR)
    markers = cv2.watershed(vis, markers.astype(np.int32))
    out = []
    for label in range(2, n + 1):
        sub = (markers == label) & mask
        if sub.sum() > 30:
            out.append(sub)
    return out or [mask]


def segment(depth, K, T_world_cam, belt: BeltRegion, max_single_area_px=2500, border=3):
    """Full depth pipeline: returns (list[Blob], world points, height map).

    Blobs touching the image border are dropped: a part that is only partly
    visible cannot be measured (it will be seen whole a moment later).
    """
    pts, height, valid = height_map(depth, K, T_world_cam, belt)
    mask = part_mask(height, valid, belt)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
    blobs = []
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < 40:
            continue
        x, y, w, h = stats[i, :4]
        if x < border or y < border or x + w > depth.shape[1] - border or \
                y + h > depth.shape[0] - border:
            continue
        comp = labels == i
        for sub in split_touching(comp, height, max_single_area_px):
            b = measure(sub, pts, height)
            if b is not None:
                b.center[2] = belt.surface_z + b.height / 2.0
                blobs.append(b)
    return blobs, pts, height


def split_by_boxes(mask, boxes):
    """Assign the pixels of a merged blob to detector boxes (nearest box centre wins)."""
    ys, xs = np.nonzero(mask)
    centers = np.array([[(b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0] for b in boxes])
    inside = np.stack([(xs >= b[0]) & (xs <= b[2]) & (ys >= b[1]) & (ys <= b[3]) for b in boxes])
    d = ((xs[None, :] - centers[:, 0:1]) ** 2 + (ys[None, :] - centers[:, 1:2]) ** 2)
    d = np.where(inside, d, np.inf)
    owner = np.argmin(d, axis=0)
    owned = np.isfinite(d.min(axis=0))
    masks = []
    for k in range(len(boxes)):
        m = np.zeros_like(mask)
        sel = owned & (owner == k)
        m[ys[sel], xs[sel]] = True
        masks.append(m)
    return masks
