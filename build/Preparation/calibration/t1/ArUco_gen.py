import cv2
import cv2.aruco as aruco
import numpy as np
import os
from PIL import Image

OUTPUT_DIR = "aruco_markers"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# US Letter at 300 DPI
DPI = 300
MM_PER_INCH = 25.4
LETTER_W_IN, LETTER_H_IN = 8.5, 11.0
SHEET_W = int(LETTER_W_IN * DPI)
SHEET_H = int(LETTER_H_IN * DPI)

# Fixed marker size (mm) and count
MARKER_SIZE_MM = 50
MARKER_SIZE = int(MARKER_SIZE_MM / MM_PER_INCH * DPI)
MARKER_IDS = [0, 1, 2, 3, 4]

dict_aruco = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

pages = []
for marker_id in MARKER_IDS:
    # Create white US Letter sheet
    sheet = np.full((SHEET_H, SHEET_W), 255, dtype=np.uint8)

    # Center marker
    x = (SHEET_W - MARKER_SIZE) // 2
    y = (SHEET_H - MARKER_SIZE) // 2
    marker = aruco.generateImageMarker(dict_aruco, marker_id, MARKER_SIZE)
    sheet[y:y + MARKER_SIZE, x:x + MARKER_SIZE] = marker

    pages.append(Image.fromarray(sheet, mode="L"))

filepath = os.path.join(OUTPUT_DIR, "aruco_letter_1perpage.pdf")
pages[0].save(
    filepath,
    save_all=True,
    append_images=pages[1:],
    resolution=DPI,
)
print(f"Generated US Letter PDF: {filepath}")
