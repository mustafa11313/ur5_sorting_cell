import numpy as np

from sorting_cell_perception.classes import gt_class_from_part_name, split_class
from sorting_cell_perception.geometry import Blob
from sorting_cell_perception.tracker import BeltTracker


def blob(x, y):
    return Blob(mask=np.zeros((2, 2), bool), bbox=(0, 0, 1, 1), center=np.array([x, y, 0.74]),
                yaw=0.0, length=0.04, width=0.04, height=0.04, fill_ratio=1.0, area_px=100)


def test_class_names():
    assert split_class('hex_nut_defect') == ('hex_nut', True)
    assert gt_class_from_part_name('part_0012__cylinder__defect') == 'cylinder_defect'
    assert gt_class_from_part_name('pedestal') is None


def test_tracker_follows_moving_part_and_votes():
    tr = BeltTracker(gate=0.03, min_hits=2)
    speed = 0.1
    out = []
    for k in range(5):
        t = 0.1 * k
        label = 'cube' if k != 2 else 'cylinder'      # one noisy frame
        out = tr.update([(blob(-0.5 + speed * t, 0.6), label, 0.9)], t, speed)
    assert len(out) == 1 and out[0].track_id == 0
    assert out[0].label[0] == 'cube'


def test_tracker_drops_stale_tracks():
    tr = BeltTracker(max_age=0.5)
    tr.update([(blob(0, 0.6), 'cube', 1.0)], 0.0, 0.0)
    tr.update([], 1.0, 0.0)
    assert tr.tracks == {}
