#!/usr/bin/env python3
"""Generate an ArUco marker image with a white border around it."""
import sys

import cv2

MARKER_ID = 0
MARKER_PIXELS = 400   # black square
BORDER_PIXELS = 50    # white frame on each side -> 500 px total

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# The function name changed between OpenCV versions
if hasattr(cv2.aruco, 'generateImageMarker'):
    marker = cv2.aruco.generateImageMarker(dictionary, MARKER_ID, MARKER_PIXELS)
else:
    marker = cv2.aruco.drawMarker(dictionary, MARKER_ID, MARKER_PIXELS)

image = cv2.copyMakeBorder(
    marker, BORDER_PIXELS, BORDER_PIXELS, BORDER_PIXELS, BORDER_PIXELS,
    cv2.BORDER_CONSTANT, value=255)

output = sys.argv[1] if len(sys.argv) > 1 else 'marker_0.png'
cv2.imwrite(output, image)
print(f'Saved marker id {MARKER_ID} to {output}')