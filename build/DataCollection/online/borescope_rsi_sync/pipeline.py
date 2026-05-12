"""
Borescope + RSI synchronized capture orchestrator.

Default behavior:
  1. Spawn RSI listener thread (UDP 59152, replies zero RKorr)
  2. Open borescope, start continuous mp4 capture (15-min chunked)
  3. Print status every STATUS_PRINT_INTERVAL_S
  4. Stop on Ctrl+C, OR when RSI was once connected and then disconnects

`wait_for_rsi=True` defers borescope recording until first RSI packet
arrives (matches the older init_recorder_svo_csv.py pattern). The
default `wait_for_rsi=False` records globally — the user explicitly
chose this: easier offline trimming, no missed startup frames.
"""
import ctypes
import json
import os
import time
from datetime import datetime

import cv2

import config
from borescope import BoreScope
from rsi_listener import RsiListener


_PREVIEW_WINDOW = "Borescope+RSI Live"
_PREVIEW_SCREEN_W = 1920
_PREVIEW_SCREEN_H = 1020


_ES_CONTINUOUS       = 0x80000000
_ES_SYSTEM_REQUIRED  = 0x00000001
_ES_DISPLAY_REQUIRED = 0x00000002


def prevent_sleep():
    if os.name == "nt":
        ctypes.windll.kernel32.SetThreadExecutionState(
            _ES_CONTINUOUS | _ES_SYSTEM_REQUIRED | _ES_DISPLAY_REQUIRED
        )
        print("[PWR] sleep/display prevention ENABLED")


def allow_sleep():
    if os.name == "nt":
        ctypes.windll.kernel32.SetThreadExecutionState(_ES_CONTINUOUS)
        print("[PWR] sleep/display prevention disabled")


def make_session_paths():
    session_ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    session_video_dir = os.path.join(config.VIDEO_OUTPUT_DIR, session_ts)
    borescope_csv = os.path.join(session_video_dir,
                                 f"borescope_frames_{session_ts}.csv")
    rsi_csv = os.path.join(config.CSV_OUTPUT_DIR, f"rsi_data_{session_ts}.csv")
    return session_ts, session_video_dir, borescope_csv, rsi_csv


def write_session_meta(session_dir: str, session_ts: str, start_epoch: float,
                       wait_for_rsi: bool):
    meta = {
        "session_ts": session_ts,
        "start_epoch": start_epoch,
        "wait_for_rsi": wait_for_rsi,
        "borescope": {
            "camera_index": config.CAM_INDEX,
            "backend": "CAP_DSHOW",
            "resolution": [config.RES_W, config.RES_H],
            "fps_nominal": config.FPS_NOMINAL,
            "fourcc": config.VIDEO_FOURCC,
            "segment_minutes": config.SEGMENT_MINUTES,
        },
        "rsi": {
            "host": config.RSI_HOST,
            "port": config.RSI_PORT,
            "disconnect_timeout_s": config.RSI_DISCONNECT_TIMEOUT_S,
        },
    }
    os.makedirs(session_dir, exist_ok=True)
    with open(os.path.join(session_dir, "session_meta.json"), "w",
              encoding="utf-8") as f:
        json.dump(meta, f, indent=2)


def _draw_preview_hud(frame, borescope, rsi, fps_estimate):
    pkt = rsi.get_latest()
    h = frame.shape[0]
    overlay = frame.copy()
    cv2.rectangle(overlay, (10, 10), (560, 200), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

    def put(text, y, color):
        cv2.putText(frame, text, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 3)
        cv2.putText(frame, text, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 1)

    put(f"Frames {borescope.frame_count}   Seg {borescope.segment_idx}",
        40, (0, 210, 0))
    put(f"Capture fps ~ {fps_estimate:5.1f}", 66, (0, 210, 0))

    if pkt is None:
        put("RSI: waiting for packets ...", 100, (0, 200, 220))
    else:
        put(f"RSI pkts {rsi.packet_count}  ipoc {pkt.ipoc}", 100, (0, 210, 0))
        put(f"Pos X={pkt.rist[0]:8.1f} Y={pkt.rist[1]:8.1f} "
            f"Z={pkt.rist[2]:7.1f}", 126, (0, 210, 0))
        put(f"Override {pkt.override:>3} Vel {pkt.vel_act:>4} "
            f"RPM {pkt.rpm_ext:>4} Delay {pkt.delay}",
            152, (0, 210, 0))

    cv2.circle(frame, (frame.shape[1] - 40, 30), 12, (0, 0, 220), -1)
    cv2.putText(frame, "REC", (frame.shape[1] - 25, 38),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 220), 2)

    cv2.putText(frame, "[q] stop session",
                (20, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 160, 160), 1)
    return frame


def run(wait_for_rsi: bool = None, preview: bool = False):
    if wait_for_rsi is None:
        wait_for_rsi = config.WAIT_FOR_RSI_DEFAULT

    session_ts, video_dir, bore_csv, rsi_csv = make_session_paths()
    print(f"[SESSION] {session_ts}")
    print(f"  videos      -> {video_dir}")
    print(f"  borescope   -> {bore_csv}")
    print(f"  rsi         -> {rsi_csv}")
    print(f"  wait_for_rsi = {wait_for_rsi}")

    prevent_sleep()
    write_session_meta(video_dir, session_ts, time.time(), wait_for_rsi)

    rsi = RsiListener(csv_path=rsi_csv)
    rsi.start()

    borescope = BoreScope(session_ts=session_ts, video_dir=video_dir,
                          csv_path=bore_csv)
    borescope.open()

    if wait_for_rsi:
        print("[WAIT] holding borescope recording until RSI connects ...")
        while not rsi.is_connection_alive():
            time.sleep(0.2)
        print("[WAIT] RSI connected -> starting borescope.")

    borescope.start()

    if preview:
        cv2.namedWindow(_PREVIEW_WINDOW, cv2.WINDOW_NORMAL)
        scale = min(_PREVIEW_SCREEN_W / config.RES_W,
                    _PREVIEW_SCREEN_H / config.RES_H)
        cv2.resizeWindow(_PREVIEW_WINDOW,
                         int(config.RES_W * scale), int(config.RES_H * scale))

    last_status_t = 0.0
    last_loop_t = time.monotonic()
    last_frame_count = 0
    fps_estimate = 0.0
    rsi_was_connected = False

    try:
        while True:
            if preview:
                frame, _ = borescope.get_latest()
                if frame is not None:
                    _draw_preview_hud(frame, borescope, rsi, fps_estimate)
                    cv2.imshow(_PREVIEW_WINDOW, frame)
                key = cv2.waitKey(50) & 0xFF
                if key in (ord('q'), 27):
                    print("\n[STOP] preview quit key")
                    break
            else:
                time.sleep(0.5)

            now = time.monotonic()
            dt = now - last_loop_t
            if dt >= 1.0:
                fps_estimate = (borescope.frame_count - last_frame_count) / dt
                last_frame_count = borescope.frame_count
                last_loop_t = now

            if rsi.connected:
                rsi_was_connected = True

            if now - last_status_t >= config.STATUS_PRINT_INTERVAL_S:
                pkt = rsi.get_latest()
                pose_str = "no RSI packet yet"
                if pkt is not None:
                    pose_str = (
                        f"X={pkt.rist[0]:8.1f} Y={pkt.rist[1]:8.1f} "
                        f"Z={pkt.rist[2]:7.1f} ov={pkt.override} "
                        f"vel={pkt.vel_act} rpm={pkt.rpm_ext}"
                    )
                print(f"[STAT] frames={borescope.frame_count} "
                      f"seg={borescope.segment_idx} "
                      f"rsi_pkts={rsi.packet_count} | {pose_str}")
                last_status_t = now

            if rsi_was_connected and not rsi.is_connection_alive():
                print("[STOP] RSI disconnected, stopping session.")
                break

    except KeyboardInterrupt:
        print("\n[STOP] keyboard interrupt")

    finally:
        borescope.stop()
        rsi.stop()
        rsi.join(timeout=3.0)
        if preview:
            cv2.destroyAllWindows()
        allow_sleep()
        print(f"[DONE] session {session_ts}")
        print(f"       videos     : {video_dir}")
        print(f"       borescope  : {bore_csv}")
        print(f"       rsi        : {rsi_csv}")
