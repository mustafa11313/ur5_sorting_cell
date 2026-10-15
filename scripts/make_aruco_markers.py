#!/usr/bin/env python3
"""Regenerate the ArUco marker textures used in the Gazebo world (DICT_4X4_50, ids 0 and 1)."""
import os

import cv2
import numpy as np

OUT = os.path.join(os.path.dirname(__file__), '..', 'src', 'sorting_cell_gazebo', 'worlds', 'media')

aruco = cv2.aruco
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
draw = aruco.generateImageMarker if hasattr(aruco, 'generateImageMarker') else aruco.drawMarker
for marker_id in (0, 1):
    img = np.full((500, 500), 255, np.uint8)        # 80 % marker + white quiet zone
    img[50:450, 50:450] = draw(dictionary, marker_id, 400)
    cv2.imwrite(os.path.join(OUT, f'aruco_4x4_{marker_id}.png'), img)
print('markers written to', os.path.abspath(OUT))
