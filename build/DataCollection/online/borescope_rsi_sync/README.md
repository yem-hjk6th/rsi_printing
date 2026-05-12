# borescope_rsi_sync

Synchronized capture for a **TESLONG NTG100 borescope** (mounted on the
KUKA KR10 TCP) and the controller's **RSI Ethernet stream**. Produces a
chunked MP4 of the print site plus two CSVs (per-frame timestamps and
per-RSI-packet pose) that can be merged offline by PC timestamp.

This is the data-capture half of an eventual closed-loop printing
experiment. No control is performed here — every RSI packet is replied
to with zero `RKorr`, the controller-side path is unchanged.

## Architecture

```
run.py             CLI entry; sets sys.path; argparse
└─ pipeline.py     orchestrates threads, owns the main loop & preview HUD
   ├─ borescope.py BoreScope: cv2 capture thread, mp4 writer w/ 15 min
   │               chunking, per-frame CSV, get_latest() for preview
   └─ rsi_listener.py
                   RsiListener: UDP recv on 59152, parse RIst/RSol/Delay/
                   Override/Vel_Act/RPM_Ext/IPOC, reply zero RKorr,
                   per-packet CSV
config.py          all tunables in one file (paths, camera, RSI, timing)
```

### Why split this way

- `rsi_listener.py` is reusable for any other experiment that needs the
  KRC pose stream (ZED + RSI, microscope + RSI, etc). Don't copy-paste
  another monolith like `exp04_sync_capture.py`
- `borescope.py` owns the `cv2.VideoCapture` lifetime end-to-end —
  the DSHOW backend has an exclusive lock and reliability is best
  when a single object opens, reads, and closes it
- `config.py` is the only file an experimenter normally edits
- `pipeline.py` is just glue; replacing the borescope with another
  camera = swap one class

## Hardware findings baked in

Verified empirically in `Vision/bore_scope/demo3_probe_params.py` and
`demo4_stream_test.py`:

| Item                              | Value                                |
|-----------------------------------|--------------------------------------|
| Camera index on win-laptop        | `1` (`0` is the laptop webcam)       |
| OpenCV backend                    | **DSHOW only** — MSMF is broken      |
| Real resolutions                  | `640x480`, `1280x720` (1080p is fake)|
| Frame rate                        | Locked at ~30 fps in hardware        |
| Working distance / DOF            | ~10 mm / 0.4–0.6 inch (fixed focus)  |
| **`CAP_PROP_EXPOSURE`**           | **NEVER set — bricks until USB replug** |
| Safe-to-tune UVC controls         | BRIGHTNESS, CONTRAST, SATURATION, HUE, SHARPNESS, BACKLIGHT, WB_TEMPERATURE, AUTO_WB |
| Unsupported                       | FOCUS, AUTOFOCUS, ZOOM, GAIN(read-only) |

RSI XML schema (from `rsi_setup/RSI_set_ver/Mine/ver5_10mm_var/RSI_EthernetConfig.xml`):

- KRC sends: `RIst` (actual pose), `RSol` (commanded pose), `Delay`,
  `Override`, `Vel_Act`, `RPM_Ext`, plus `IPOC` (cycle counter)
- KRC expects back: `RKorr.X..C` correction vector. This module always
  sends zero — passive logging only

## Run

```powershell
# headless (recommended for actual runs)
python build/DataCollection/online/borescope_rsi_sync/run.py

# with live HUD window (use for setup, [q] or ESC to stop)
python build/DataCollection/online/borescope_rsi_sync/run.py --preview

# match the old "wait for KRC" pattern (not the default)
python build/DataCollection/online/borescope_rsi_sync/run.py --wait-for-rsi
```

**Suggested experiment start sequence** (matches the user's preference
for global recording with trim-the-beginning offline cleanup):

1. PC: start `run.py` — borescope begins recording immediately,
   RSI listener binds UDP 59152 and waits
2. KUKA: load `Roboter/src/Ye_RSI_bore_t1.src` (or any RSI-enabled
   `.src`), select run, press start
3. KRC begins streaming RSI packets; the PC `[RSI] connected ...`
   line prints
4. Robot runs the program
5. KRC program ends → no more RSI packets → after 2 s the PC side
   auto-stops; OR press Ctrl+C any time

## Output layout

```
rsi_printing/
├─ recorded_data/<session_ts>/
│   ├─ bore_<session_ts>_001_<seg_start_ts>.mp4   (15-min chunk #1)
│   ├─ bore_<session_ts>_002_<seg_start_ts>.mp4   (15-min chunk #2)
│   ├─ ...
│   ├─ borescope_frames_<session_ts>.csv          (per-frame log)
│   └─ session_meta.json                          (config snapshot)
└─ rsi_data/
    └─ rsi_data_<session_ts>.csv                  (per-packet log)
```

### CSV columns

`borescope_frames_<session_ts>.csv`:
`pc_ts_monotonic, pc_ts_wall, frame_idx, segment_idx, frame_in_segment, video_file`

`rsi_data_<session_ts>.csv` (19 cols):
`pc_ts_monotonic, pc_ts_wall, ipoc,`
`rist_x_mm..rist_c_deg (6), rsol_x_mm..rsol_c_deg (6),`
`delay, override, vel_act, rpm_ext`

**Offline merge**: align the two CSVs on `pc_ts_monotonic` with
`pandas.merge_asof(direction="nearest")`. RSI runs at 250 Hz, borescope
at 30 Hz — every video frame will get the closest-in-time pose,
typically within 4 ms.

## Tunable parameters (`config.py`)

| Section     | Knob                    | Default          | Notes                                       |
|-------------|-------------------------|------------------|---------------------------------------------|
| Borescope   | `CAM_INDEX`             | `1`              | webcam at 0, NTG100 at 1 on win-laptop      |
|             | `RES_W` × `RES_H`       | `1280 × 720`     | only `640×480` is the other real option     |
|             | `VIDEO_FOURCC`          | `"mp4v"`         | mp4v played fine in VLC; H264 also works    |
|             | `FPS_NOMINAL`           | `30.0`           | hardware locked, just the writer's nominal  |
|             | `SEGMENT_MINUTES`       | `15`             | mp4 chunk length                            |
|             | `SAFE_CONTROLS`         | `{}`             | empty by default — don't poke UVC controls  |
| RSI         | `RSI_HOST`              | `"0.0.0.0"`      | bind all interfaces                         |
|             | `RSI_PORT`              | `59152`          | matches `RSI_EthernetConfig.xml`            |
|             | `RSI_DISCONNECT_TIMEOUT_S`| `2.0`           | how long after last packet to auto-stop     |
|             | `CSV_FLUSH_INTERVAL`    | `200`            | rows between fsync                          |
| Paths       | `VIDEO_OUTPUT_DIR`      | `rsi_printing/recorded_data` | resolves relative to this file |
|             | `CSV_OUTPUT_DIR`        | `rsi_printing/rsi_data`      | same                            |
| Orchestration| `WAIT_FOR_RSI_DEFAULT` | `False`          | True = old init_recorder pattern            |
|             | `STATUS_PRINT_INTERVAL_S`| `5.0`           | terminal heartbeat cadence                  |

## Calibration tools (in `Vision/bore_scope/`)

- `demo1_init.py` — minimal find-camera-and-preview
- `demo2_capture.py` — preview + snapshot/record keys (no RSI)
- `demo3_probe_params.py` — UVC capability probe (used to discover the
  EXPOSURE bricking bug)
- `demo4_stream_test.py` — live HUD + interactive control tuning +
  built-in 30 s no-write / 30 s with-mp4 stability test. Run this
  to verify camera health and pick `BRIGHTNESS / CONTRAST / SHARPNESS`
  values before a long capture session

## See also

- `build/Preparation/test/init_recorder_svo_csv.py` — same chunked
  pattern but with ZED2i instead of borescope; reference for SVO2 path
  and `prevent_sleep` mechanism this module reuses
- `rsi_setup/RSI_logistics/exp04/exp04_sync_capture.py` — older
  monolithic ZED+RSI+ArUco recorder; this module is the modular
  refactor without the ArUco/marker side
