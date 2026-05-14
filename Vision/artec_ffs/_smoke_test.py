"""
_smoke_test.py — environment self-check for this package.

Run this FIRST on any new machine / conda env before capture or recon. It
verifies every dependency the pipeline needs and, when something is wrong,
prints the exact fix command — so a fresh-machine agent doesn't need access
to machine-local notes.

    python Vision/artec_ffs/_smoke_test.py

Exit code 0 = all good, 1 = at least one FAIL.

Each check is isolated: a missing dependency is reported with its fix, not a
crash, so the test still tells you about the *other* deps.
"""
import os, sys, platform
from pathlib import Path

os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))  # Vision/

_FAILS = []


def _ok(msg):    print(f"  [ OK ] {msg}")
def _warn(msg):  print(f"  [WARN] {msg}")
def _fail(msg, fix):
    print(f"  [FAIL] {msg}")
    print(f"         fix: {fix}")
    _FAILS.append(msg)


print(f"\nEnvironment self-check — artec_ffs")
print(f"  Python {platform.python_version()}  exe={sys.executable}\n")

# ── pyzed (ZED SDK python wrapper) ────────────────────────────────────────────
try:
    import zed_setup  # noqa: F401 — adds ZED SDK + CUDA DLL dirs (Vision/zed_setup.py)
    import pyzed.sl as sl
    _ok(f"pyzed       SDK {sl.Camera.get_sdk_version()}")
except ImportError as e:
    _fail(f"pyzed not importable ({e})",
          'run the ZED SDK python installer into this env:\n'
          '              python "C:\\Program Files (x86)\\ZED SDK\\get_python_api.py"')
except Exception as e:
    _fail(f"pyzed import raised {type(e).__name__}: {e}",
          "check Vision/zed_setup.py DLL paths + ZED SDK install")

# ── torch + CUDA ──────────────────────────────────────────────────────────────
# The #1 Windows gotcha: a plain `pip install torch` (or torch pulled in as a
# transitive dep) installs the CPU-only wheel from PyPI. CUDA wheels live ONLY
# on PyTorch's own index. Symptom: torch.__version__ ends in "+cpu".
try:
    import torch
    ver = torch.__version__
    if ver.endswith("+cpu") or not torch.backends.cuda.is_built():
        _fail(f"torch {ver} is CPU-only — --ffs inference needs CUDA",
              "pip install --index-url https://download.pytorch.org/whl/cu128 "
              "--upgrade torch torchvision")
    elif not torch.cuda.is_available():
        _fail(f"torch {ver} is a CUDA build but no GPU is visible",
              "check nvidia-smi / driver; ensure the GPU is not in use by another process")
    else:
        gpu = torch.cuda.get_device_name(0)
        x = torch.randn(2048, 2048, device="cuda")
        _ = (x @ x.T).sum().item()
        _ok(f"torch       {ver}  cuda={torch.version.cuda}  gpu={gpu}  matmul OK")
except ImportError:
    _fail("torch not installed",
          "pip install --index-url https://download.pytorch.org/whl/cu128 torch torchvision")

# ── triton (required for FFS torch.compile path on Windows) ───────────────────
# FFS calls model.forward(..., optimize_build_volume="pytorch1") which goes
# through torch.compile / inductor. Linux torch ships triton; Windows does not.
try:
    import triton  # noqa: F401
    _ok(f"triton      {getattr(triton, '__version__', '?')}  (FFS torch.compile path)")
except ImportError:
    if sys.platform == "win32":
        _fail("triton not installed — --ffs forward pass will raise TritonMissing",
              "pip install triton-windows")
    else:
        _warn("triton not installed — needed for the --ffs torch.compile path")

# ── open3d / cv2 / numpy ──────────────────────────────────────────────────────
try:
    import open3d as o3d, cv2, numpy as np
    _ok(f"open3d {o3d.__version__}  cv2 {cv2.__version__}  numpy {np.__version__}")
except ImportError as e:
    _fail(f"core dep missing ({e})", "pip install open3d opencv-contrib-python numpy")

# ── FFS weights (via config FFS_REPO_ROOT resolution) ─────────────────────────
try:
    import config
    if config.FFS_WEIGHTS_DIR.exists():
        _ok(f"FFS weights {config.FFS_WEIGHTS_DIR}")
    else:
        _fail(f"FFS weights not found: {config.FFS_WEIGHTS_DIR}",
              "set FFS_REPO_ROOT env var, or create Vision/artec_ffs/"
              "config_local.py with FFS_REPO_ROOT (see README)")
except Exception as e:
    _fail(f"config import failed: {type(e).__name__}: {e}",
          "check config.py / config_local.py")

# ── verdict ───────────────────────────────────────────────────────────────────
print()
if _FAILS:
    print(f"=> smoke test FAIL ({len(_FAILS)} issue(s)) — fix the above, then re-run.")
    print("   Full new-machine bootstrap chain is in README.md (环境说明).")
    sys.exit(1)
else:
    print("=> smoke test PASS — capture / recon are good to go.")
    sys.exit(0)
