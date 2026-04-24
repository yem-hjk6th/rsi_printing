# Copilot Instructions — rsi_printing workspace

## Machine Configuration (两台机器)

This project runs on **two machines with different GPUs and CUDA versions**:

| Machine | GPU | CUDA Toolkit | ZED SDK |
|---------|-----|-------------|---------|
| Win Laptop | RTX 5070 | 12.8 | 5.2 |
| Desktop (this machine) | RTX 5080 | 13.0 | 5.2 (CUDA 13 build) |

## CUDA Path Policy — ALWAYS Auto-Detect, Never Hardcode

**All CUDA DLL path handling in this project is now auto-detecting.** Before suggesting any CUDA path edit, verify the current approach:

- **`Vision/zed_setup.py`** — canonical reference; uses `_find_cuda_dir()` with `glob.glob` to pick the highest installed `v*` under `NVIDIA GPU Computing Toolkit\CUDA\`.
- **All other files with ZED/CUDA DLL blocks** — use `glob` + `sorted(..., reverse=True)` to auto-pick the latest CUDA bin path at runtime.

**Do NOT hardcode `v12.8` or `v13.0`.** The pattern to use is:

```python
if os.name == "nt":
    import glob as _g
    _cuda_bin = next(iter(sorted(
        _g.glob(r"C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v*\bin"),
        reverse=True)), "")
    for p in [
        r"C:\Program Files (x86)\ZED SDK\bin",
        r"C:\Program Files (x86)\ZED SDK\dependencies\bin",
        _cuda_bin,
    ]:
        if p and os.path.isdir(p):
            os.add_dll_directory(p)
```

## GPU Detection (When Needed)

If you ever need to determine which machine you're on, run:

```powershell
nvidia-smi --query-gpu=name --format=csv,noheader
```

- Output contains `5070` → Win Laptop (CUDA 12.8 Toolkit installed)
- Output contains `5080` → Desktop (CUDA 13.0 Toolkit installed)

To confirm the actual installed CUDA Toolkit version:

```powershell
Get-ChildItem "C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\" | Select-Object Name
```

## Environments

| Conda Env | Python | Purpose |
|-----------|--------|---------|
| `zedenv` | 3.10 | ZED camera, SAM-2, pyzed 5.2, OpenGL |
| `ffs` | 3.12 | Fast-FoundationStereo, pybullet, depth estimation |

- Always `conda activate zedenv` before running scripts that import `pyzed.sl`.
- Always `conda activate ffs` before running `depth2_ffs.py` or any FFS depth model.

## Key Paths

- **ZED SDK**: `C:\Program Files (x86)\ZED SDK`
- **CUDA Toolkit root**: `C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\`
- **pyzed reinstall**: `conda run -n zedenv python "C:\Program Files (x86)\ZED SDK\get_python_api.py"`
