# ZED 2i Stereo Demo Series

## demo1 — Turn On & Print Intrinsics
- Open ZED 2i, print left/right camera intrinsics (fx, fy, cx, cy, FOV, distortion, baseline)
- **Test**: confirm camera connection, verify calibration params at target resolution

## demo2 — Preview with Disparity ROI
- Live left/right view, red box marks pixels with valid disparity coverage
- Dead zone (no stereo match) shaded in red overlay
- **Test**: at your working distance, check how much of the FOV actually has depth — adjust `Z_min` with `+`/`-` to see ROI shrink/grow

## demo3 — Stereo Overlap Crop
- Crops left & right to show only the overlapping FOV region, side by side
- **Test**: verify that your ROI (nozzle/print area) falls within the overlap zone in both views; check for occlusion by extruder

## demo4 — Blend Overlap for Pixel Matching
- Composites left & right overlap into one image, three modes (`m` to cycle):
  - **BLEND**: 50/50 alpha — ghosting = disparity, sharp = matched background
  - **DIFF**: absolute difference heatmap — bright = mismatch, dark = well-matched
  - **ANAGLYPH**: red/cyan stereo — visual disparity inspection
- Green epipolar lines (`e` to toggle) — all matches should be on the same row
- **Test**: check rectification quality, visualize where disparity is large (close objects) vs small (far background); assess whether nozzle creates asymmetric occlusion in the overlap zone

## demo6 — 3-Frame Capture + FFS Depth
- Capture 3 rectified stereo pairs from ZED 2i (HD2K)
- Call Fast-FoundationStereo (ffs env) for disparity estimation
- Median-fuse 3 depth maps for noise reduction
- **Test**: single-viewpoint depth reconstruction, compare quality vs ZED built-in depth

## Common Controls
| Key | Action |
|-----|--------|
| `+`/`-` | Adjust Z_min (shifts d_max and overlap boundary) |
| `q` | Quit |
| `m` | Cycle blend mode (demo4) |
| `e` | Toggle epipolar lines (demo4) |
| `s` | Capture & run FFS (demo6) |

## Environment Convention
| Filename pattern | Conda env | Python |
|-----------------|-----------|--------|
| `*ffs*` in title | **ffs** | `C:\Users\888y9\miniconda3\envs\ffs\python.exe` |
| everything else | **zedenv** | `C:\Users\888y9\miniconda3\envs\zedenv\python.exe` |

Use `.vscode/launch.json` → "Python: FFS env" config for ffs files, "Python: zedenv" for others.
