# ZED SDK DLL Path Fix — Standardized Solution

**Date**: 2026-04-16
**Issue**: `ImportError: DLL load failed while importing sl`

## Summary
Created `Vision/zed_setup.py` as a single-import solution for the recurring ZED SDK DLL path problem on Windows.

## What was done
1. **`Vision/zed_setup.py`** — helper module that calls `os.add_dll_directory()` for ZED SDK and CUDA paths on import. Any new ZED script just needs:
   ```python
   import sys, os
   sys.path.insert(0, os.path.join(os.path.dirname(__file__), "<path_to_Vision>"))
   import zed_setup
   import pyzed.sl as sl
   ```

2. **`Vision/.conda-env`** — contains `zedenv`, used by PowerShell profile for auto-activation.

3. **`.claude/zed_env_setup.md`** — full reference doc (paths, versions, troubleshooting).

## Environment snapshot
- ZED SDK 5.2.2 @ `C:\Program Files (x86)\ZED SDK`
- CUDA 12.8
- pyzed 5.2 (cp310-win_amd64), Python 3.10.20, conda env `zedenv`
- If pyzed breaks: `python "C:\Program Files (x86)\ZED SDK\get_python_api.py"`
