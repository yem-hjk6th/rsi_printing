"""
3_quarter_printbed_rsi_on_header_replacer.py
Extract motion commands from PRC-generated .src, replace header/footer with
KRL structure for KRC — RSI ON (closed-loop) version.

New bed reference:
  Bed top surface Z  = 0.0 mm  (KRC actual position)
  First layer height = 1.3 mm  → Layer 1 Z = 1.3 mm
  Layer step         = 1.0 mm  → Layer N Z = 1.3 + (N-1) * 1.0
"""

import re
import os

# ============================================================
# ▼▼▼  CONFIG  ▼▼▼
# ============================================================

INPUT_SRC  = r"C:\Users\888y9\Desktop\Models\src_output\Ye_RSI_t6.src"
OUTPUT_SRC = None          # None = auto _fixed.src in same dir
PROG_NAME  = "Ye_RSI_t6"
EXTRUDER_RPM = 400
PRINT_VEL_CP = 0.012       # m/s, used for $VEL.CP in MAIN section

# Z offset applied to all print motion lines (bed_top + first_layer_height)
# Z values above Z_SAFE_THRESHOLD are treated as approach heights and skipped
Z_OFFSET          = 1.3    # mm  (bed Z=0.0 + first layer 1.3mm)
Z_SAFE_THRESHOLD  = 50.0   # mm  above this = safe approach, no offset

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


def apply_z_offset(line, offset, safe_threshold):
    """Add offset to Z value in a KRL motion line, skip if Z > safe_threshold."""
    def replace_z(m):
        z_val = float(m.group(1))
        if abs(z_val) >= safe_threshold:
            return m.group(0)  # leave approach heights untouched
        return f", Z {round(z_val + offset, 4)},"
    return re.sub(r", Z ([+-]?\d+\.?\d*),", replace_z, line)


def extract_moves(src_path):
    moves = []
    with open(src_path, "r", encoding="utf-8") as f:
        in_move_section = False
        for line in f:
            stripped = line.strip()

            if stripped.startswith("ret = RSI_"):  continue
            if stripped.startswith("IF (ret"):     continue
            if stripped == "HALT":                 continue
            if stripped == "ENDIF":                continue
            if stripped.startswith(";"):           continue
            if stripped == "END":                  continue

            if re.match(r"^PTP\s*\{.*X\s", stripped):
                in_move_section = True
                moves.append(apply_z_offset(stripped, Z_OFFSET, Z_SAFE_THRESHOLD))
                continue

            if re.match(r"^LIN\s*\{", stripped):
                in_move_section = True
                moves.append(apply_z_offset(stripped, Z_OFFSET, Z_SAFE_THRESHOLD))
                continue

            if in_move_section and re.match(r"^\$VEL\.CP\s*=", stripped):
                moves.append(stripped)
                continue

    return moves


def build_src(prog_name, extruder_rpm, moves):
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
