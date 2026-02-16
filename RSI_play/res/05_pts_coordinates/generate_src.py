#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate minimal KRL SRC file from circle coordinates
"""

import sys
sys.path.append('res/05_pts_coordinates')

try:
    from circle_coordinates import INNER_CIRCLE
except ImportError:
    print("[ERROR] circle_coordinates.py not found!")
    print("Run 05_cir_cal_coordinates.py first to generate coordinates.")
    sys.exit(1)

# Generate SRC header
src_header = """&ACCESS RVP
&REL 1
&PARAM TEMPLATE = C:\\KRC\\Roboter\\Template\\vorgabe
&PARAM EDITMASK = *
DEF RSI_Circle_Motion ( )
    DECL INT ret
    DECL INT CONTID

    ;FOLD INI
    ;FOLD BASISTECH INI
    GLOBAL INTERRUPT DECL 3 WHEN $STOPMESS==TRUE DO IR_STOPM ( )
    INTERRUPT ON 3
    BAS (#INITMOV,0 )
    ;ENDFOLD (BASISTECH INI)
    ;ENDFOLD (INI)

    BAS(#TOOL, 3)
    BAS(#BASE, 0)
    BAS(#VEL_PTP, 10)

    PTP {X 750, Y 0, Z 650, A 0, B 90, C 0}

    ret = RSI_CREATE("RSI_test",CONTID,TRUE)
    IF (ret <> RSIOK) THEN
      HALT
    ENDIF

    ret = RSI_ON(#RELATIVE)
    IF (ret <> RSIOK) THEN
      HALT
    ENDIF

    ;FOLD Inner Circle Motion (48 points)
    WHILE TRUE
"""

# Generate LIN commands - minimal format like AUT file
lin_commands = ""
for i in range(48):
    pt = INNER_CIRCLE[i]
    x = pt['x']
    y = pt['y']
    z = pt['z']
    
    lin_commands += f"LIN {{X {x}, Y {y}, Z {z}, A 0, B 90, C 0}} C_DIS\n"
    lin_commands += "RSI_MOVECORR()\n\n"

# Generate SRC footer
src_footer = """    ENDWHILE
    ;ENDFOLD

    ret = RSI_OFF()

END
"""

# Combine all parts
src_content = src_header + lin_commands + src_footer

# Write to file
output_filename = 'RSI_Circle_Motion.src'
with open(output_filename, 'w') as f:
    f.write(src_content)

print("=" * 80)
print("[SRC FILE GENERATED]")
print(f"File: {output_filename}")
print(f"Points: 48")
print("Format: LIN {X, Y, Z, A, B, C} C_DIS (minimal)")
print("=" * 80)

# Show preview
print("\n[PREVIEW - First 10 points]")
preview_lines = src_content.split('\n')[35:55]
for line in preview_lines:
    print(line)

print("\n...")
print(f"\n[Ready to upload to KRC]")
