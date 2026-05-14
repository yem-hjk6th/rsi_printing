"""Smoke test: GPU torch + pyzed + open3d + FFS weights in current env.

Run from any env:
    python Vision/artec_ffs/_smoke_test.py
"""
import os, sys, platform
from pathlib import Path

os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))  # Vision/

print(f"Python    {platform.python_version()}  exe={sys.executable}")

import zed_setup  # noqa: F401 — sets ZED SDK DLL paths
import pyzed.sl as sl
print(f"pyzed     SDK {sl.Camera.get_sdk_version()}")

import torch
print(f"torch     {torch.__version__}  cuda_built={torch.backends.cuda.is_built()}  "
      f"cuda_ver={torch.version.cuda}  available={torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"          gpu={torch.cuda.get_device_name(0)}  count={torch.cuda.device_count()}")
    x = torch.randn(2048, 2048, device="cuda")
    y = (x @ x.T).sum().item()
    print(f"          matmul OK  sum={y:.1f}")
else:
    print("          GPU not available")

import open3d as o3d, cv2, numpy as np
print(f"open3d    {o3d.__version__}    cv2 {cv2.__version__}    numpy {np.__version__}")

import config
ok = config.FFS_WEIGHTS_DIR.exists()
print(f"weights   exists={ok}  ->  {config.FFS_WEIGHTS_DIR}")

print("=> smoke test PASS" if (torch.cuda.is_available() and ok) else "=> smoke test FAIL")
