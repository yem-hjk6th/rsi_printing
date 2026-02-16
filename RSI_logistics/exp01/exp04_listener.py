#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import xml.etree.ElementTree as ET
import sys

RSI_PORT = 59152
# Requested travel range (what we try to trace); actual correction is clamped by RSI limits.
STEP_PER_IPOC = 0.05   # mm per IPOC (0.05mm per 4ms ≈ 12.5 mm/s until clamped)
X_TRAVEL = 100.0       # mm, target sweep range ±X_TRAVEL
X_RSI_LIMIT = 10.0     # mm, match your POSCORR single-step limit (±10mm)

ipoc = "0"
ipoc_start = 0
x_prev = 0.0
x_target = 0.0
direction = 1


def parse_xml(data):
    global ipoc
    try:
        root = ET.fromstring(data)

        ipoc_elem = root.find('IPOC')
        if ipoc_elem is not None:
            ipoc = ipoc_elem.text

        rist = root.find('RIst')
        z_ist = float(rist.get('Z', 0.0)) if rist is not None else 0.0

        ov_elem = root.find('Override')
        override = int(ov_elem.text or 0) if ov_elem is not None else 0

        vel_elem = root.find('Vel_Act')
        vel_act = int(vel_elem.text or 0) if vel_elem is not None else 0

        return True, z_ist, override, vel_act
    except Exception as exc:
        print(f"[ERROR] {exc}")
        return False, 0.0, 0, 0


def next_x_target():
    global x_target, direction
    # Update the nominal target within ±X_TRAVEL
    x_target += direction * STEP_PER_IPOC
    if x_target >= X_TRAVEL:
        x_target = X_TRAVEL
        direction = -1
    elif x_target <= -X_TRAVEL:
        x_target = -X_TRAVEL
        direction = 1

    # Clamp to RSI allowed range to avoid controller warnings
    clamped = False
    x_out = x_target
    if x_out > X_RSI_LIMIT:
        x_out = X_RSI_LIMIT
        clamped = True
    elif x_out < -X_RSI_LIMIT:
        x_out = -X_RSI_LIMIT
        clamped = True
    return x_out, clamped


def run_server():
    global ipoc_start, x_prev, x_target, direction

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', RSI_PORT))
    sock.settimeout(1.0)

    print(f"[RSI] Listening on 0.0.0.0:{RSI_PORT}")
    print("-" * 90)
    print(f"{'Sec':<8} | {'IPOC':<7} | {'OV':<3} | {'Vel':<5} | {'Z':<8} | {'X_Tgt':<8} | {'dX':<8} | {'Clamp':<5}")
    print("-" * 90)
    sys.stdout.flush()

    is_connected = False

    try:
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                success, z_ist, override, vel_act = parse_xml(data)

                if not is_connected:
                    print(f"[Connected] {addr}\n")
                    ipoc_start = int(ipoc)
                    x_prev = 0.0
                    x_target = 0.0
                    direction = 1
                    is_connected = True

                if success:
                    ipoc_curr = int(ipoc)
                    elapsed_cycles = ipoc_curr - ipoc_start
                    elapsed_sec = elapsed_cycles * 0.004

                    x_cmd, clamped = next_x_target()
                    delta_x = x_cmd - x_prev
                    x_prev = x_cmd

                    print(
                        f"T:{elapsed_sec:6.2f} | {ipoc_curr:<7} | {override:<3} | {vel_act:<5} | {z_ist:7.2f} | {x_cmd:7.2f} | {delta_x:7.4f} | {str(clamped):<5}",
                        end='\r',
                        flush=True,
                    )

                    reply_xml = (
                        f'<Sen Type="ImFree">'
                        f'<RKorr X="{delta_x:.4f}" Y="0.0000" Z="0.0000" A="0.0000" B="0.0000" C="0.0000" />'
                        f'<IPOC>{ipoc}</IPOC>'
                        f'</Sen>'
                    )
                    sock.sendto(reply_xml.encode(), addr)

            except socket.timeout:
                if is_connected:
                    print("\n[Timeout] Connection lost")
                    is_connected = False
                continue

    finally:
        sock.close()


if __name__ == "__main__":
    run_server()
