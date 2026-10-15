"""Cell geometry (shared config/cell.yaml) and the part -> bin sorting rules."""
import os

import yaml


def default_cell_path():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('sorting_cell_bringup'), 'config', 'cell.yaml')


def load_cell(path=None):
    with open(path or default_cell_path()) as f:
        return yaml.safe_load(f)['cell']


def bin_for_class(class_name, confidence=1.0, min_confidence=0.5):
    """Sorting rule: defective or uncertain parts go to the reject bin."""
    if class_name.endswith('_defect') or confidence < min_confidence or class_name == 'unknown':
        return 'bin_reject'
    return 'bin_' + class_name


def expected_bin_for_part(part_name):
    """Ground-truth bin for a Gazebo part name such as 'part_0007__cube__defect'."""
    try:
        _, ptype, quality = part_name.split('__')
    except ValueError:
        return None
    return 'bin_reject' if quality == 'defect' else 'bin_' + ptype
