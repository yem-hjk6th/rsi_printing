import argparse
import json
import subprocess
from typing import Any, Dict, List

try:
    import pyzed.sl as sl
except Exception:  # pragma: no cover
    sl = None

PS_COMMAND = r"""
$controllers = Get-CimInstance Win32_USBController | Select-Object Name, PNPDeviceID
$devices = Get-PnpDevice -PresentOnly -Class USB | Select-Object FriendlyName, InstanceId
$images = Get-PnpDevice -PresentOnly -Class Image | Select-Object FriendlyName, InstanceId
$zed = Get-CimInstance Win32_PnPEntity -Filter "Name LIKE '%ZED%' OR Name LIKE '%Stereo%'" | Select-Object Name, DeviceID
[pscustomobject]@{ Controllers = $controllers; Devices = $devices; Images = $images; ZedPnP = $zed } | ConvertTo-Json -Depth 4
"""

CONNECTOR_TYPE_MAP = {
    0: "Unknown",
    1: "Type-A",
    2: "Type-B",
    3: "Type-C",
    4: "Optical",
    5: "Other",
}


def run_powershell_json(command: str) -> List[Dict[str, Any]]:
    creation_flags = 0x08000000  # CREATE_NO_WINDOW
    result = subprocess.run(
        ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", command],
        capture_output=True,
        text=True,
        timeout=20,
        creationflags=creation_flags,
    )
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or "PowerShell command failed.")
    if not result.stdout.strip():
        stderr = result.stderr.strip()
        if stderr:
            raise RuntimeError(stderr)
        return []
    data = json.loads(result.stdout)
    if isinstance(data, dict):
        return [data]
    return data


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pnp", action="store_true", help="Enable PnP/USB listing (may be slow)")
    args = parser.parse_args()

    if sl is not None:
        try:
            zed_devices = sl.Camera.get_device_list()
        except Exception:
            zed_devices = []
        if zed_devices:
            print("ZED SDK")
            print("-")
            for d in zed_devices:
                model = getattr(d, "camera_model", None)
                serial = getattr(d, "serial_number", None)
                name = str(model) if model is not None else "ZED"
                if serial is not None:
                    print(f"{name}  SN:{serial}")
                else:
                    print(name)
            print()
        else:
            print("ZED SDK: 未检测到设备")
    else:
        print("ZED SDK: 未安装或不可用")

    if not args.pnp:
        return

    data = run_powershell_json(PS_COMMAND)
    if not data:
        print("未找到USB设备")
        return

    payload = data[0] if isinstance(data, list) else data
    controllers = payload.get("Controllers") or []
    devices = payload.get("Devices") or []
    images = payload.get("Images") or []
    zed_pnp = payload.get("ZedPnP") or []

    print("USB Controllers")
    print("-")
    for c in controllers:
        print(c.get("Name") or "(no name)")
    print()

    print("USB Devices")
    print("-")
    for d in devices:
        print(d.get("FriendlyName") or d.get("InstanceId") or "(no name)")

    print()
    print("Image Devices")
    print("-")
    for d in images:
        print(d.get("FriendlyName") or d.get("InstanceId") or "(no name)")

    print()
    print("ZED PnP")
    print("-")
    for d in zed_pnp:
        print(d.get("Name") or d.get("DeviceID") or "(no name)")


if __name__ == "__main__":
    main()
