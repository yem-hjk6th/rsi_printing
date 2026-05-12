"""
Constants for the borescope + RSI synchronized capture pipeline.

All settings derived from:
  - demo3_probe_params.py / demo4_stream_test.py findings on NTG100
  - RSI_EthernetConfig.xml (KRC RSI send/receive schema)
  - init_recorder_svo_csv.py (15-min chunking + power management precedent)
"""
import os
import cv2


CAM_INDEX        = 1
CAM_BACKEND      = cv2.CAP_DSHOW
RES_W, RES_H     = 1280, 720
VIDEO_FOURCC     = "mp4v"
FPS_NOMINAL      = 30.0
SEGMENT_MINUTES  = 15

SAFE_CONTROLS = {}

RSI_HOST                 = "0.0.0.0"
RSI_PORT                 = 59152
RSI_DISCONNECT_TIMEOUT_S = 2.0
CSV_FLUSH_INTERVAL       = 200

_THIS_DIR     = os.path.dirname(os.path.abspath(__file__))
RSI_PRINTING_ROOT = os.path.abspath(os.path.join(_THIS_DIR, "..", "..", "..", ".."))
VIDEO_OUTPUT_DIR  = os.path.join(RSI_PRINTING_ROOT, "recorded_data")
CSV_OUTPUT_DIR    = os.path.join(RSI_PRINTING_ROOT, "rsi_data")

WAIT_FOR_RSI_DEFAULT = False
STATUS_PRINT_INTERVAL_S = 5.0
