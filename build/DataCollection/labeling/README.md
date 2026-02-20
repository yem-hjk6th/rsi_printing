# Labeling Demo

## What this demo does
- Uses two fixed pixels defined in the script header.
- Reads ZED depth/point cloud in real time.
- Converts pixel -> 3D camera points.
- Prints 3D Euclidean distance in mm.
- Optionally transforms to gripper frame using `T_cam2gripper` and prints the same distance for sanity check.

## Run
From workspace root:

```bash
python build/DataCollection/labeling/pixel_distance_demo.py
```

Press `q` to quit.

## Update pixel positions
Edit these fields in `pixel_distance_demo.py`:
- `PIXEL_1`
- `PIXEL_2`

## Notes
- If depth at one pixel is invalid, the script prints `invalid depth`.
- Keep points on the same physical object surface and avoid reflective areas for stable depth.
