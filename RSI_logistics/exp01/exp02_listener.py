#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import xml.etree.ElementTree as ET
import time
import threading
import sys
import os

RSI_PORT = 59152
STEP_SIZE = 5.0
CONTROL_FILE = "control.txt"

ipoc = "0"
start_time = None
korr_x_target = 0.0
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

# File monitor thread
def file_monitor_thread():
    global korr_x_target
    print(f"[File Monitor] Watching {CONTROL_FILE} for commands\n")
    while True:
        try:
            if os.path.exists(CONTROL_FILE):
                with open(CONTROL_FILE, 'r') as f:
                    cmd = f.read().strip().lower()
                if cmd == 'm':
                    korr_x_target = min(korr_x_target + STEP_SIZE, 10.0)
                    os.remove(CONTROL_FILE)
                elif cmd == 'n':
                    korr_x_target = 0.0
                    os.remove(CONTROL_FILE)
            time.sleep(0.1)
        except Exception as e:
            print(f"[Monitor Error] {e}")
            time.sleep(0.1)

def run_server():
    global start_time, korr_x_target, korr_x_prev
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', RSI_PORT))
    sock.settimeout(1.0)
    
    print(f"[RSI] Listening on 0.0.0.0:{RSI_PORT}")
    print("-" * 80)
    print(f"{'Time':<8} | {'IPOC':<5} | {'OV':<3} | {'Vel':<5} | {'Z':<8} | {'X_Tgt':<7} | {'dX':<7}")
    print("-" * 80)
    sys.stdout.flush()

    monitor_thread = threading.Thread(target=file_monitor_thread, daemon=True)
    monitor_thread.start()

    is_connected = False

    try:
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                
                if not is_connected:
                    print(f"[Connected] {addr}\n")
                    start_time = time.time()
                    korr_x_prev = 0.0
                    is_connected = True
                
                success, z_ist, override, vel_act = parse_xml(data)
                
                if success:
                    elapsed = time.time() - start_time
                    
                    delta_x = korr_x_target - korr_x_prev
                    korr_x_prev = korr_x_target
                    
                    print(f"T:{elapsed:6.2f} | {ipoc:<5} | {override:<3} | {vel_act:<5} | {z_ist:7.2f} | {korr_x_target:6.2f} | {delta_x:6.4f}", end='\r')

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
