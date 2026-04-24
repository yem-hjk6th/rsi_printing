"""
src_header_replace.py
Extract motion commands (PTP cartesian / LIN / $VEL.CP) from PRC-generated .src,
replace header/footer with correct KRL structure for KRC.
"""

import re
import os

# ============================================================
# ▼▼▼  CONFIG  ▼▼▼
# ============================================================

# Input .src file path (PRC-generated)
INPUT_SRC = r"C:\Users\888y9\Desktop\Models\src_output\Ye_RSI_t6.src"

# Output path (None = auto-generate _fixed.src in same dir)
OUTPUT_SRC = None

# Program name (must match KRL DEF name and filename)
PROG_NAME = "Ye_RSI_t6"

# Extruder RPM
EXTRUDER_RPM = 400

# Print speed (m/s), LIN velocity after RSI_ON
PRINT_VEL_CP = 0.012

# ============================================================
# ▲▲▲  END CONFIG  ▲▲▲
# ============================================================


HEADER_TEMPLATE = """\
DEF {prog_name} ( )

;VARIABLE DECLARATIONS
DECL INT ret
DECL INT CONTID

;FOLD INI
BAS (#INITMOV,0 )
;BASE IS 0, TOOL IS 3, FIXED START POS
BAS(#TOOL, 3)
BAS(#BASE, 0)
BAS(#VEL_PTP, 10)
PDAT_ACT.ACC = 50
PDAT_ACT.APO_DIST = 50
$BWDSTART = TRUE
$VEL.CP=0.2
$ADVANCE=3
;ENDFOLD (INI)

PTP  {{A1 33,A2 -90,A3 100,A4 5,A5 -10,A6 -5,E1 0,E2 0,E3 0,E4 0}}

;EXTRUDER RPM
PelletExtruderRPM = {extruder_rpm}

;RSI INI
ret = RSI_CREATE("RSI_MIN.rsi", CONTID, TRUE)
IF (ret <> RSIOK) THEN
    HALT
ENDIF

ret = RSI_ON(#RELATIVE)
IF (ret <> RSIOK) THEN
    HALT
ENDIF
"""

FOOTER = """\

ret = RSI_OFF()

;RETURN TO HOME
PTP  {A1 33,A2 -90,A3 100,A4 5,A5 -10,A6 -5,E1 0,E2 0,E3 0,E4 0}

END
"""


def extract_moves(src_path):
    """Extract cartesian PTP, LIN, and $VEL.CP lines from PRC .src."""
    moves = []
    with open(src_path, "r", encoding="utf-8") as f:
        in_move_section = False
        for line in f:
            stripped = line.strip()

            # Skip RSI calls / joint PTP / END / fold comments
            if stripped.startswith("ret = RSI_"):
                continue
            if stripped.startswith("IF (ret"):
                continue
            if stripped == "HALT":
                continue
            if stripped == "ENDIF":
                continue
            if stripped.startswith(";"):
                continue
            if stripped == "END":
                continue

            # Cartesian PTP (contains X, not joint PTP)
            if re.match(r"^PTP\s*\{.*X\s", stripped):
                in_move_section = True
                moves.append(stripped)
                continue

            # LIN command
            if re.match(r"^LIN\s*\{", stripped):
                in_move_section = True
                moves.append(stripped)
                continue

            # $VEL.CP change within move section
            if in_move_section and re.match(r"^\$VEL\.CP\s*=", stripped):
                moves.append(stripped)
                continue

    return moves


def build_src(prog_name, extruder_rpm, moves):
    """Assemble complete .src file content."""
    header = HEADER_TEMPLATE.format(
        prog_name=prog_name,
        extruder_rpm=extruder_rpm,
    )

    body = "\n".join(moves)
    return header + "\n" + body + FOOTER


def main():
    src_path = INPUT_SRC
    if not os.path.isfile(src_path):
        print(f"[ERROR] Input file not found: {src_path}")
        return

    out_path = OUTPUT_SRC
    if out_path is None:
        base, ext = os.path.splitext(src_path)
        out_path = base + "_fixed" + ext

    print(f"[INPUT]  {src_path}")

    moves = extract_moves(src_path)
    print(f"[INFO]   Extracted {len(moves)} motion commands")

    if not moves:
        print("[ERROR] No motion commands found, check input file")
        return

    content = build_src(PROG_NAME, EXTRUDER_RPM, moves)

    with open(out_path, "w", encoding="utf-8") as f:
        f.write(content)

    total_lines = content.count("\n") + 1
    print(f"[OUTPUT] {out_path}")
    print(f"[INFO]   {total_lines} lines total")


if __name__ == "__main__":
    main()
