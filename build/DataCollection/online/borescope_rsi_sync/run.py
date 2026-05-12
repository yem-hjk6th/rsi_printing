"""
CLI entry — start a borescope + RSI synchronized capture session.

Usage:
    python run.py                  # record continuously, headless
    python run.py --preview        # record + open live HUD window
    python run.py --wait-for-rsi   # hold borescope until first RSI packet

Output layout:
    rsi_printing/recorded_data/<session_ts>/
        bore_<session_ts>_001_<seg_start>.mp4
        bore_<session_ts>_002_<seg_start>.mp4   (every 15 min)
        ...
        borescope_frames_<session_ts>.csv
        session_meta.json
    rsi_printing/rsi_data/
        rsi_data_<session_ts>.csv

Stop with Ctrl+C, or by stopping the RSI/KRC side (auto-stops 2s
after the last RSI packet, but only if RSI was ever connected).
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pipeline import run


def main():
    parser = argparse.ArgumentParser(
        description="Borescope (NTG100) + RSI synchronized recorder"
    )
    parser.add_argument(
        "--wait-for-rsi", action="store_true",
        help="hold borescope recording until first RSI packet "
             "(default: start recording immediately)",
    )
    parser.add_argument(
        "--preview", action="store_true",
        help="open a live cv2 window with HUD; [q]/ESC stops the session",
    )
    args = parser.parse_args()
    run(wait_for_rsi=args.wait_for_rsi, preview=args.preview)


if __name__ == "__main__":
    main()
