#!/usr/bin/env python3
"""Generate an ArUco marker image with a white border around it.

Usage: generate_marker.py OUTPUT_PNG [MARKER_ID]
"""
import sys

import cv2

MARKER_PIXELS = 400   # black square
BORDER_PIXELS = 50    # white frame on each side -> 500 px total

output = sys.argv[1] if len(sys.argv) > 1 else 'marker_0.png'
marker_id = int(sys.argv[2]) if len(sys.argv) > 2 else 0

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

if hasattr(cv2.aruco, 'generateImageMarker'):
    marker = cv2.aruco.generateImageMarker(dictionary, marker_id, MARKER_PIXELS)
else:
    marker = cv2.aruco.drawMarker(dictionary, marker_id, MARKER_PIXELS)

image = cv2.copyMakeBorder(
    marker, BORDER_PIXELS, BORDER_PIXELS, BORDER_PIXELS, BORDER_PIXELS,
    cv2.BORDER_CONSTANT, value=255)

cv2.imwrite(output, image)
print(f'Saved marker id {marker_id} to {output}')