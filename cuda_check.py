#!/usr/bin/env python3
"""
Simple CUDA detection script — checks PyTorch, TensorFlow, and nvidia-smi.
Run: python cuda_check.py
"""
import subprocess
import sys


def check_torch():
    try:
        import torch
    except Exception as e:
        return False, f"torch import failed: {e}"
    try:
        ok = torch.cuda.is_available()
        details = []
        if ok:
            count = torch.cuda.device_count()
            details.append(f"CUDA available, device_count={count}")
            for i in range(count):
                details.append(f" - [{i}] {torch.cuda.get_device_name(i)}")
        else:
            details.append("CUDA not available according to torch")
        return ok, "\n".join(details)
    except Exception as e:
        return False, f"torch error: {e}"


def check_tf():
    try:
        import tensorflow as tf
    except Exception as e:
        return False, f"tensorflow import failed: {e}"
    try:
        gpus = tf.config.list_physical_devices('GPU')
        if gpus:
            names = [getattr(g, 'name', str(g)) for g in gpus]
            return True, f"TensorFlow sees GPUs: {names}"
        else:
            return False, "TensorFlow reports no GPUs"
    except Exception as e:
        return False, f"tensorflow error: {e}"


def check_nvidia_smi():
    try:
        p = subprocess.run(["nvidia-smi", "-L"], capture_output=True, text=True, timeout=5)
        if p.returncode == 0 and p.stdout.strip():
            return True, p.stdout.strip()
        else:
            return False, p.stderr.strip() or p.stdout.strip() or f"nvidia-smi returncode={p.returncode}"
    except FileNotFoundError:
        return False, "nvidia-smi not found (NVIDIA drivers may be missing)"
    except Exception as e:
        return False, f"nvidia-smi error: {e}"


def main():
    print("Checking CUDA via multiple methods...\n")
    ok, msg = check_torch()
    print("PyTorch:", "OK" if ok else "NO", "\n", msg, sep=" ")
    print("\n---\n")
    ok2, msg2 = check_tf()
    print("TensorFlow:", "OK" if ok2 else "NO", "\n", msg2, sep=" ")
    print("\n---\n")
    ok3, msg3 = check_nvidia_smi()
    print("nvidia-smi:", "OK" if ok3 else "NO", "\n", msg3, sep=" ")

    if not any([ok, ok2, ok3]):
        print("\nNo CUDA detected by any method.")
        sys.exit(1)
    else:
        print("\nCUDA detected (at least one method).")
        sys.exit(0)


if __name__ == "__main__":
    main()
