#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import xml.etree.ElementTree as ET
import time
import sys

RSI_PORT = 59152

ipoc = "0"
ipoc_start = 0
korr_x_prev = 0.0

def parse_xml(data):
    global ipoc
    try:
        root = ET.fromstring(data)
        
        ipoc_elem = root.find('IPOC')
        if ipoc_elem is not None:
            ipoc = ipoc_elem.text
        
        rist = root.find('RIst')
        if rist is not None:
            z_ist = float(rist.get('Z', 0))
        else:
            z_ist = 0.0
            
        override = 0
        ov_elem = root.find('Override')
        if ov_elem is not None:
            override = int(ov_elem.text or 0)
            
        vel_act = 0
        vel_elem = root.find('Vel_Act')
        if vel_elem is not None:
            vel_act = int(vel_elem.text or 0)

        return True, z_ist, override, vel_act
        
    except Exception as e:
        print(f"[ERROR] {e}")
        return False, 0.0, 0, 0

def calculate_correction(elapsed_cycles):
    # elapsed_cycles = (current_IPOC - start_IPOC) 
    # Each cycle = 4ms, so 250 cycles = 1 second
    elapsed_seconds = elapsed_cycles * 0.004
    
    if elapsed_seconds < 5:
        return 0.0
    elif elapsed_seconds < 10:
        # Ramp up from 0 to 8mm over 5 seconds (1.6mm/s)
        return (elapsed_seconds - 5) * 1.6
    elif elapsed_seconds < 15:
        # Continue ramp from 8mm to 10mm (from 10-15s, +0.4mm/s)
        return 8.0 + (elapsed_seconds - 10) * 0.4
    elif elapsed_seconds < 20:
        # Smooth ramp DOWN from 10mm back to 0 over 5 seconds
        return 10.0 - (elapsed_seconds - 15) * 2.0
    else:
        return 0.0

def run_server():
    global ipoc_start, korr_x_prev
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', RSI_PORT))
    sock.settimeout(1.0)
    
    print(f"[RSI] Listening on 0.0.0.0:{RSI_PORT}")
    print("-" * 80)
    print(f"{'Time':<8} | {'IPOC':<5} | {'OV':<3} | {'Vel':<5} | {'Z':<8} | {'X_Tgt':<7} | {'dX':<7}")
    print("-" * 80)
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
                    korr_x_prev = 0.0
                    is_connected = True
                
                if success:
                    ipoc_current = int(ipoc)
                    elapsed_cycles = ipoc_current - ipoc_start
                    elapsed_seconds = elapsed_cycles * 0.004
                    korr_x_target = calculate_correction(elapsed_cycles)
                    delta_x = korr_x_target - korr_x_prev
                    korr_x_prev = korr_x_target
                    
                    print(f"T:{elapsed_seconds:6.2f} | {ipoc:<5} | {override:<3} | {vel_act:<5} | {z_ist:7.2f} | {korr_x_target:6.2f} | {delta_x:6.4f}", end='\r', flush=True)

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
