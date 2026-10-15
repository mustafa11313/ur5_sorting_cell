"""Grasp planning for a parallel-jaw gripper picking parts from a belt (no ROS dependencies).

A part is described by its footprint (centre, yaw of the long side, length,
width, height) as published by perception. The gripper approaches from above;
its fingers close along the tool y axis. With the TCP orientation
R = Rz(psi) * Rx(pi), tool y points along angle (psi - pi/2), so closing across
the footprint's short side means psi = yaw (mod pi).

Each part type has a rotational symmetry that yields several equivalent
grasp angles; among them the planner keeps the ones whose fingers do not
collide with neighbouring parts (touching parts) and picks the one that is
kindest to the wrist.
"""
import math
from dataclasses import dataclass

import numpy as np

SYMMETRY = {'cube': math.pi / 2, 'hex_nut': math.pi / 3, 'cylinder': math.pi / 12}

FINGER_THICKNESS = 0.012    # along the closing axis
FINGER_DEPTH = 0.020        # across the closing axis
OPEN_MARGIN = 0.012         # extra opening on each side while approaching
MAX_OPENING = 0.08
FINGERTIP_BELOW_TCP = 0.015  # fingertips extend this far beyond the TCP
MIN_TIP_CLEARANCE = 0.004    # fingertips must stay this far above the belt


@dataclass
class PartFootprint:
    x: float
    y: float
    yaw: float
    length: float
    width: float
    height: float
    ptype: str = 'cube'


@dataclass
class Grasp:
    x: float
    y: float
    z: float           # TCP height
    psi: float         # tool yaw
    opening: float     # gripper opening while approaching [m]
    close_to: float    # opening when closed on the part [m]
    clearance: float   # free space around the fingers [m]


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def wrap_half(a):
    """Wrap into [-pi/2, pi/2): the gripper is symmetric under a 180 deg turn."""
    a = wrap(a)
    if a >= math.pi / 2:
        a -= math.pi
    elif a < -math.pi / 2:
        a += math.pi
    return a


def candidate_angles(part):
    step = SYMMETRY.get(part.ptype, math.pi / 2)
    n = int(round(math.pi / step))
    return sorted({round(wrap_half(part.yaw + k * step), 6) for k in range(n)})


def finger_points(part, psi, opening, n=5):
    """Sample points on both finger pads (world xy) for an opening and tool yaw."""
    close_dir = np.array([math.cos(psi - math.pi / 2), math.sin(psi - math.pi / 2)])
    across = np.array([math.cos(psi), math.sin(psi)])
    pts = []
    for side in (-1.0, 1.0):
        centre = np.array([part.x, part.y]) + side * close_dir * (opening / 2 + FINGER_THICKNESS / 2)
        for a in np.linspace(-FINGER_DEPTH / 2, FINGER_DEPTH / 2, n):
            for b in np.linspace(-FINGER_THICKNESS / 2, FINGER_THICKNESS / 2, 3):
                pts.append(centre + a * across + b * close_dir)
    return np.array(pts)


def footprint_distance(points, other):
    """Distance from points to another part's footprint rectangle (<= 0 inside)."""
    c, s = math.cos(other.yaw), math.sin(other.yaw)
    rel = points - np.array([other.x, other.y])
    local = np.column_stack((rel @ np.array([c, s]), rel @ np.array([-s, c])))
    half = np.array([other.length / 2, other.width / 2])
    d = np.abs(local) - half
    outside = np.linalg.norm(np.maximum(d, 0.0), axis=1)
    inside = np.minimum(np.max(d, axis=1), 0.0)
    return float(np.min(outside + inside))


def plan_grasp(part, neighbours=(), belt_z=0.72, reference_psi=0.0, min_clearance=0.004):
    """Return the best Grasp, or None if every candidate collides with a neighbour.

    A wide approach opening is tried first; if a neighbour is in the way, a
    narrow one (fingers just clearing the part) is tried before giving up.
    """
    close_to = min(MAX_OPENING, part.width + 0.002)
    z = belt_z + max(part.height / 2.0, FINGERTIP_BELOW_TCP + MIN_TIP_CLEARANCE)
    wide = min(MAX_OPENING, part.width + 2 * OPEN_MARGIN)
    for opening in (wide, min(wide, close_to + 0.006)):
        best = None
        for psi in candidate_angles(part):
            pts = finger_points(part, psi, opening)
            clearance = min((footprint_distance(pts, o) for o in neighbours), default=1.0)
            if clearance < min_clearance:
                continue
            # prefer wrist-friendly angles, then more clearance
            cost = abs(wrap_half(psi - reference_psi)) - 2.0 * min(clearance, 0.02)
            if best is None or cost < best[0]:
                best = (cost, Grasp(part.x, part.y, z, psi, opening, close_to, clearance))
        if best is not None:
            return best[1]
    return None
