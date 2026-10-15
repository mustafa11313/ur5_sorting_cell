"""Part classes shared by perception, task logic and evaluation."""

PART_TYPES = ('cube', 'cylinder', 'hex_nut')
CLASS_NAMES = ('cube', 'cube_defect', 'cylinder', 'cylinder_defect', 'hex_nut', 'hex_nut_defect')

# Nominal geometry [m]: footprint closing width and height (before scale jitter).
NOMINAL = {
    'cube': {'width': 0.040, 'height': 0.040},
    'cylinder': {'width': 0.042, 'height': 0.045},
    'hex_nut': {'width': 0.050, 'height': 0.025},
}


def split_class(name):
    """'hex_nut_defect' -> ('hex_nut', True)."""
    if name.endswith('_defect'):
        return name[:-len('_defect')], True
    return name, False


def gt_class_from_part_name(part_name):
    """Gazebo part names look like 'part_0007__cube__defect'."""
    try:
        _, ptype, quality = part_name.split('__')
    except ValueError:
        return None
    return ptype + ('_defect' if quality == 'defect' else '')
