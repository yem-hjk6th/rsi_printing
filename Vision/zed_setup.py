"""
zed_setup.py — ZED SDK DLL path setup for Windows.

Import this module BEFORE importing pyzed.sl to avoid DLL load errors.

Usage:
    import zed_setup          # adds DLL dirs + validates
    import pyzed.sl as sl     # now safe

Environment: conda activate zedenv (Python 3.10, pyzed 5.2)
"""

import os
import sys

# ── Paths (win-laptop: RTX 5070, ZED SDK 5.2, CUDA 12.8) ──
ZED_SDK_DIR = r"C:\Program Files (x86)\ZED SDK"
CUDA_DIR = r"C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.8"

_DLL_DIRS = [
    os.path.join(ZED_SDK_DIR, "bin"),
    os.path.join(ZED_SDK_DIR, "dependencies", "bin"),
    os.path.join(CUDA_DIR, "bin"),
]


def setup_dll_paths():
    """Add ZED SDK and CUDA DLL directories (Windows only)."""
    if os.name != "nt":
        return
    for p in _DLL_DIRS:
        if os.path.isdir(p):
            os.add_dll_directory(p)
            if p not in os.environ.get("PATH", ""):
                os.environ["PATH"] = p + os.pathsep + os.environ["PATH"]


def validate():
    """Quick validation: import sl, print SDK version."""
    try:
        import pyzed.sl as sl
        ver = sl.Camera.get_sdk_version()
        print(f"[zed_setup] pyzed OK — SDK {ver}, Python {sys.version.split()[0]}")
        return True
    except ImportError as e:
        print(f"[zed_setup] FAILED: {e}")
        print(f"[zed_setup] Fix: conda activate zedenv && "
              f'python "{ZED_SDK_DIR}\\get_python_api.py"')
        return False


# Auto-setup on import
setup_dll_paths()
