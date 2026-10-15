"""Classical (non-learned) classifier: geometry from depth + colour cue for defects.

Used as a baseline against the YOLO detector and as a fallback when no trained
model is available.
"""
import cv2


def red_fraction(rgb_bgr, mask):
    hsv = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    red = ((h < 8) | (h > 172)) & (s > 110) & (v > 50)
    n = max(int(mask.sum()), 1)
    return float((red & mask).sum()) / n


def classify(blob, rgb_bgr, nut_max_height=0.031, cube_min_fill=0.9, defect_min_red=0.03):
    """Return (class_name, confidence) for a measured Blob."""
    if blob.height < nut_max_height:
        ptype = 'hex_nut'
        conf = min(1.0, 0.6 + (nut_max_height - blob.height) / 0.01 * 0.4)
    elif blob.fill_ratio >= cube_min_fill:
        ptype = 'cube'
        conf = min(1.0, 0.6 + (blob.fill_ratio - cube_min_fill) / 0.08 * 0.4)
    else:
        ptype = 'cylinder'
        conf = min(1.0, 0.6 + (cube_min_fill - blob.fill_ratio) / 0.08 * 0.4)
    rf = red_fraction(rgb_bgr, blob.mask)
    defect = rf > defect_min_red
    conf *= min(1.0, 0.7 + abs(rf - defect_min_red) / defect_min_red * 0.3)
    return (ptype + '_defect' if defect else ptype), float(conf)
