"""
Generate ArUco markers (DICT_6X6_250) at 75, 100, 125, 150, 200 mm.
Smart-packs onto US Letter (8.5 × 11 in) pages at 300 DPI.

Layout:
  Page 1:  75 mm (ID 0) + 100 mm (ID 1)  side-by-side on top row
           125 mm (ID 2)  centered below
  Page 2:  150 mm (ID 3)  centered
  Page 3:  200 mm (ID 4)  centered

Usage:  python aruco_gen2.py
"""

import os
import cv2
import cv2.aruco as aruco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

# ── Settings ────────────────────────────────────────────────────────────────
OUTPUT_DIR = "aruco_markers"
os.makedirs(OUTPUT_DIR, exist_ok=True)

DPI = 300
MM_PER_INCH = 25.4

# US Letter
LETTER_W_IN, LETTER_H_IN = 8.5, 11.0
SHEET_W = int(LETTER_W_IN * DPI)  # 2550 px
SHEET_H = int(LETTER_H_IN * DPI)  # 3300 px

MARGIN_MM = 12            # safe margin from page edge
GAP_MM = 10               # gap between markers
MARGIN_PX = int(MARGIN_MM / MM_PER_INCH * DPI)
GAP_PX = int(GAP_MM / MM_PER_INCH * DPI)

USABLE_W = SHEET_W - 2 * MARGIN_PX
USABLE_H = SHEET_H - 2 * MARGIN_PX

SIZES_MM = [75, 100, 125, 150, 200]
MARKER_IDS = [0, 1, 2, 3, 4]

dict_aruco = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)


def mm2px(mm):
    return int(mm / MM_PER_INCH * DPI)


def make_marker(marker_id, size_px):
    return aruco.generateImageMarker(dict_aruco, marker_id, size_px)


def add_label(sheet, text, cx, top_y):
    """Draw a size label centered at (cx, top_y) using OpenCV."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.9
    thickness = 2
    (tw, th), _ = cv2.getTextSize(text, font, scale, thickness)
    org = (cx - tw // 2, top_y + th + 4)
    cv2.putText(sheet, text, org, font, scale, 0, thickness, cv2.LINE_AA)


def new_sheet():
    return np.full((SHEET_H, SHEET_W), 255, dtype=np.uint8)


def place(sheet, marker_img, x, y, label):
    """Place marker on sheet and add label below it."""
    h, w = marker_img.shape
    sheet[y:y + h, x:x + w] = marker_img
    add_label(sheet, label, x + w // 2, y + h + 4)


# ── Layout planning ─────────────────────────────────────────────────────────
print("US Letter: {:.1f} × {:.1f} mm  ({} × {} px @ {} DPI)".format(
    LETTER_W_IN * MM_PER_INCH, LETTER_H_IN * MM_PER_INCH, SHEET_W, SHEET_H, DPI))
print(f"Usable area (margin {MARGIN_MM}mm): {USABLE_W}px × {USABLE_H}px "
      f"= {USABLE_W/DPI*MM_PER_INCH:.1f} × {USABLE_H/DPI*MM_PER_INCH:.1f} mm\n")

sizes_px = {mm: mm2px(mm) for mm in SIZES_MM}
for mm in SIZES_MM:
    print(f"  {mm:>3d} mm  →  {sizes_px[mm]:>4d} px  (ID {MARKER_IDS[SIZES_MM.index(mm)]})")

# Check: can all fit on one page?
# Row 1: 75 + 100 side by side  →  width = 75+gap+100 = 185 mm ✓ (usable ~192 mm)
#                                   height = max(75,100) = 100 mm
# Row 2: 125 centered            →  height += gap + 125 = 235 mm
# Row 3: 150?                    →  height += gap + 150 = 395 mm ✗ (usable ~254 mm)
#
# Conclusion: 75+100+125 fit on page 1;  150 and 200 need separate pages.

LABEL_SPACE = 40  # px reserved below each marker for the label

pages = []

# ── Page 1: 75 mm + 100 mm (top row), 125 mm (bottom row) ───────────────
sheet = new_sheet()

s75 = sizes_px[75]
s100 = sizes_px[100]
s125 = sizes_px[125]

# Top row: center the pair (75 + gap + 100) horizontally
row1_w = s75 + GAP_PX + s100
row1_h = max(s75, s100)
row1_x = MARGIN_PX + (USABLE_W - row1_w) // 2
row1_y = MARGIN_PX + 20  # small top padding

# Align both to bottom of row
m75 = make_marker(MARKER_IDS[0], s75)
m100 = make_marker(MARKER_IDS[1], s100)
place(sheet, m75,  row1_x, row1_y + (row1_h - s75), f"ID {MARKER_IDS[0]} | 75 mm")
place(sheet, m100, row1_x + s75 + GAP_PX, row1_y, f"ID {MARKER_IDS[1]} | 100 mm")

# Bottom row: 125 mm centered
row2_y = row1_y + row1_h + LABEL_SPACE + GAP_PX
row2_x = MARGIN_PX + (USABLE_W - s125) // 2
m125 = make_marker(MARKER_IDS[2], s125)
place(sheet, m125, row2_x, row2_y, f"ID {MARKER_IDS[2]} | 125 mm")

# Check fit
used_h = (row2_y + s125 + LABEL_SPACE) - MARGIN_PX
print(f"\nPage 1 (75+100+125): height used = {used_h/DPI*MM_PER_INCH:.1f} mm "
      f"/ {USABLE_H/DPI*MM_PER_INCH:.1f} mm usable  {'✓' if used_h <= USABLE_H else '✗'}")

pages.append(Image.fromarray(sheet, mode="L"))

# ── Page 2: 150 mm centered ─────────────────────────────────────────────────
sheet = new_sheet()
s150 = sizes_px[150]
m150 = make_marker(MARKER_IDS[3], s150)
cx = (SHEET_W - s150) // 2
cy = (SHEET_H - s150) // 2
place(sheet, m150, cx, cy, f"ID {MARKER_IDS[3]} | 150 mm")
print(f"Page 2 (150): {150} mm centered  ✓")
pages.append(Image.fromarray(sheet, mode="L"))

# ── Page 3: 200 mm centered ─────────────────────────────────────────────────
sheet = new_sheet()
s200 = sizes_px[200]
m200 = make_marker(MARKER_IDS[4], s200)
cx = (SHEET_W - s200) // 2
cy = (SHEET_H - s200) // 2
place(sheet, m200, cx, cy, f"ID {MARKER_IDS[4]} | 200 mm")
fit = s200 <= USABLE_W and s200 <= USABLE_H
print(f"Page 3 (200): {200} mm centered  {'✓' if fit else '✗ tight!'}")
pages.append(Image.fromarray(sheet, mode="L"))

# ── Save multi-page PDF ─────────────────────────────────────────────────────
pdf_path = os.path.join(OUTPUT_DIR, "aruco_5sizes.pdf")
pages[0].save(pdf_path, save_all=True, append_images=pages[1:], resolution=DPI)
print(f"\nSaved {len(pages)}-page PDF: {pdf_path}")
print("Sizes: " + ", ".join(f"{mm}mm (ID {mid})" for mm, mid in zip(SIZES_MM, MARKER_IDS)))
