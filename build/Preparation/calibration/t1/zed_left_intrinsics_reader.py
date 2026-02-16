import os
import numpy as np

INTRINSICS_PATH = os.path.join(
    os.path.dirname(__file__),
    "camera",
    "zed_left_intrinsics.npz",
)


def main() -> None:
    data = np.load(INTRINSICS_PATH)
    camera_matrix = data["camera_matrix"]
    dist_coeffs = data["dist_coeffs"]
    resolution = data.get("resolution", None)

    print("camera_matrix:")
    print(camera_matrix)
    print("dist_coeffs:")
    print(dist_coeffs)
    if resolution is not None:
        print("resolution:")
        print(resolution)


if __name__ == "__main__":
    main()
