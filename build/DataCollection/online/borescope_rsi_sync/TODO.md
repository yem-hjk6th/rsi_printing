# TODO — borescope_rsi_sync

State at session end (2026-05-12): module skeleton built and dry-run
verified (Python-side starts, listens, opens camera, writes mp4 segment
+ both CSVs). Not yet exercised against a live KRC.

## Must do before first real experiment

- [ ] **Live end-to-end test with KRC**: start `run.py --preview` first,
      then start the KRC RSI program. Confirm:
      - `[RSI] connected to (10.100.1.x, 59152)` appears within 1 s
      - `[STAT]` lines show `rsi_pkts` growing (~250/s = one per 4 ms cycle)
      - `pkt.rist` numbers track actual TCP pose
      - Stopping KRC triggers `[STOP] RSI disconnected` within 2 s
- [ ] **Verify 15-min segment rotation**: run for ≥ 20 min, confirm
      `bore_..._002_...mp4` appears around the 15-min mark with no
      dropped frames at the boundary (check `borescope_frames_*.csv`
      for monotonic gap > 50 ms)
- [ ] **Long-haul stability**: 4-hour continuous run (the original
      failure mode that motivated chunking). Disk I/O, USB suspend,
      and `prevent_sleep` should all hold

## Should do soon

- [ ] **Offline merge script** (`build/DataCollection/offline/merge_bore_rsi.py`):
      `pandas.merge_asof` on `pc_ts_monotonic` to join `borescope_frames_*.csv`
      with `rsi_data_*.csv`. Output: one row per video frame with the
      RSI pose at that instant. This unlocks all four "what can you do
      with the data" applications discussed in the design phase
- [ ] **Sharpness thresholds for borescope**: the live HUD currently
      shows raw sharpness with no color coding. Once enough sessions
      exist, calibrate green/yellow/red thresholds (rough start:
      green ≥ 20, yellow ≥ 10 at 1280×720). Then port back into demo4
- [ ] **Snapshot key in preview**: `[s]` to write a PNG of current
      frame (analog to demo4). Useful as a marker during the run

## Hardware / setup

- [ ] **Borescope mounting position**: the nozzle was OOF in the demo4
      verification. Adjust the sensor-mount hole position so the
      target area sits at the ~10 mm working distance / DOF window.
      Hardware fix, not software
- [ ] **Plug NTG100 into a powered USB hub if dropping frames** under
      long sessions — UVC at 1280×720 MJPG is ~10 MB/s, comfortably
      within USB 2.0 but a busy hub can starve it

## Closed-loop (future)

- [ ] When ready to move from passive logging to active correction:
      replace `build_zero_reply()` in `rsi_listener.py` with a
      control law that consumes `(rsol - rist)` plus borescope-derived
      features. The CSV already records `rsol_*` columns for this
- [ ] Decide cycle budget: closed-loop must compute and respond
      within the 4 ms RSI period, or KRC will throw a delay/timeout
      fault. Borescope ML inference at 30 Hz is *not* the limiting
      factor — only the corrections need to be at 250 Hz

## Won't do (deliberate)

- ❌ **Expose `EXPOSURE` as a CLI / config knob** — `cap.set(CAP_PROP_EXPOSURE, ...)`
      bricks NTG100 until USB replug. Confirmed 2026-05-12. Use the
      hardware LED brightness wheel on the cable instead
- ❌ **MSMF backend fallback** — every `grab()` failed with
      `Error: -2147483638`. Module pins `cv2.CAP_DSHOW`
- ❌ **1920×1080 mode** — NTG100 silently falls back to 1280×720
      regardless of requested width/height
