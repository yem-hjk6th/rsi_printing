#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import xml.etree.ElementTree as ET
import time
import threading
import sys

RSI_PORT = 59152
CONTROL_PORT = 59153
STEP_SIZE = 5.0  # X-axis step size in mm

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

# User input thread
def user_input_thread():
    global korr_x_target
    print("[Input] Type 'm' to move X+5mm, 'n' to reset X, 'q' to quit\n")
    while True:
        try:
            cmd = input().strip().lower()
            if cmd == 'm':
                korr_x_target = min(korr_x_target + STEP_SIZE, 10.0)
            elif cmd == 'n':
                korr_x_target = 0.0
            elif cmd == 'q':
                break
        except:
            break

# Control command listener on separate port
def control_listener():
    global korr_x_target
    try:
        ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        ctrl_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        ctrl_sock.bind(('127.0.0.1', CONTROL_PORT))
        print(f"[Control] Listening on localhost:{CONTROL_PORT}\n")
        
        while True:
            try:
                data, _ = ctrl_sock.recvfrom(64)
                cmd = data.decode('utf-8').strip().lower()
                
                if cmd == 'm':
                    korr_x_target = min(korr_x_target + STEP_SIZE, 10.0)
                elif cmd == 'n':
                    korr_x_target = 0.0
            except Exception as e:
                print(f"[Control Error] {e}")
    except Exception as e:
        print(f"[Control Bind Error] Cannot bind to port {CONTROL_PORT}: {e}")

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

    input_thread = threading.Thread(target=user_input_thread, daemon=True)
    input_thread.start()

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
