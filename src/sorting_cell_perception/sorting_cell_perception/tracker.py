"""Tiny multi-object tracker for parts moving on a conveyor.

Parts move (almost) rigidly along +x at the belt speed, so a constant-velocity
prediction plus nearest-neighbour association is enough. Class labels are
smoothed by confidence-weighted voting over the track's history.
"""
from collections import defaultdict
from dataclasses import dataclass, field

import numpy as np


@dataclass
class Track:
    track_id: int
    center: np.ndarray
    yaw: float
    length: float
    width: float
    height: float
    last_seen: float
    hits: int = 1
    votes: dict = field(default_factory=lambda: defaultdict(float))
    blob: object = None

    @property
    def label(self):
        if not self.votes:
            return 'unknown', 0.0
        name = max(self.votes, key=self.votes.get)
        total = sum(self.votes.values())
        return name, self.votes[name] / total


class BeltTracker:
    def __init__(self, gate=0.035, max_age=1.0, min_hits=2):
        self.gate = gate
        self.max_age = max_age
        self.min_hits = min_hits
        self.tracks = {}
        self._next_id = 0
        self._last_t = None

    def update(self, detections, t, belt_speed):
        """detections: list of (blob, class_name, confidence). Returns confirmed tracks."""
        dt = 0.0 if self._last_t is None else max(0.0, t - self._last_t)
        self._last_t = t
        for tr in self.tracks.values():
            tr.center = tr.center + np.array([belt_speed * dt, 0.0, 0.0])

        unmatched = set(self.tracks)
        for blob, name, conf in detections:
            best, best_d = None, self.gate
            for tid in unmatched:
                d = np.linalg.norm(self.tracks[tid].center[:2] - blob.center[:2])
                if d < best_d:
                    best, best_d = tid, d
            if best is None:
                tr = Track(self._next_id, blob.center.copy(), blob.yaw, blob.length, blob.width,
                           blob.height, t)
                self.tracks[self._next_id] = tr
                self._next_id += 1
            else:
                unmatched.discard(best)
                tr = self.tracks[best]
                tr.center = blob.center.copy()
                tr.yaw, tr.length, tr.width, tr.height = blob.yaw, blob.length, blob.width, blob.height
                tr.last_seen = t
                tr.hits += 1
            tr.votes[name] += conf
            tr.blob = blob

        for tid in list(self.tracks):
            if t - self.tracks[tid].last_seen > self.max_age:
                del self.tracks[tid]
        return [tr for tr in self.tracks.values()
                if tr.hits >= self.min_hits and tr.last_seen == t]
