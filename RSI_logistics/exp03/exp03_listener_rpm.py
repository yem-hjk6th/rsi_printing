#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import xml.etree.ElementTree as ET
import time
import threading
import sys
import os

RSI_PORT = 59152
CONTROL_PORT = 59153
STEP_SIZE = 5.0  # X-axis step size in mm

ipoc = "0"
start_time = None
korr_x_target = 0.0
korr_x_prev = 0.0

# Current robot state
robot_state = {
    'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
    'vel_act': 0,
    'override': 0,
    'rpm_ext': 0,
    'ipoc': '0',
    'timestamp': 0.0
}

def parse_xml(data):
    """Parse XML data from robot and extract all relevant parameters"""
    global ipoc, robot_state
    try:
        root = ET.fromstring(data)
        
        # Get IPOC
        ipoc_elem = root.find('IPOC')
        if ipoc_elem is not None:
            ipoc = ipoc_elem.text
            robot_state['ipoc'] = ipoc
        
        # Get Position (RIst element)
        rist = root.find('RIst')
        if rist is not None:
            x_ist = float(rist.get('X', 0))
            y_ist = float(rist.get('Y', 0))
            z_ist = float(rist.get('Z', 0))
            robot_state['position']['x'] = x_ist
            robot_state['position']['y'] = y_ist
            robot_state['position']['z'] = z_ist
        else:
            robot_state['position'] = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        # Get Override
        ov_elem = root.find('Override')
        if ov_elem is not None:
            robot_state['override'] = int(ov_elem.text or 0)
        else:
            robot_state['override'] = 0
        
        # Get Velocity Actual
        vel_elem = root.find('Vel_Act')
        if vel_elem is not None:
            robot_state['vel_act'] = int(vel_elem.text or 0)
        else:
            robot_state['vel_act'] = 0
        
        # Get RPM External
        rpm_elem = root.find('RPM_Ext')
        if rpm_elem is not None:
            robot_state['rpm_ext'] = int(rpm_elem.text or 0)
        else:
            robot_state['rpm_ext'] = 0
        
        robot_state['timestamp'] = time.time()
        
        return True
        
    except Exception as e:
        print(f"[ERROR] Parse error: {e}")
        return False

def print_robot_state():
    """Print current robot state in a formatted table"""
    state = robot_state
    print(f"T:{(state['timestamp'] - start_time):6.2f} | "
          f"{state['ipoc']:<5} | "
          f"{state['override']:<3} | "
          f"{state['vel_act']:<5} | "
          f"{state['position']['x']:7.2f} | "
          f"{state['position']['y']:7.2f} | "
          f"{state['position']['z']:7.2f} | "
          f"RPM:{state['rpm_ext']:<5}", end='\r')
    sys.stdout.flush()

def user_input_thread():
    """Handle user input commands"""
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

def control_listener():
    """Listen for external control commands on separate port"""
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
                    print(f"\n[Control] X target set to {korr_x_target}")
                elif cmd == 'n':
                    korr_x_target = 0.0
                    print(f"\n[Control] X target reset to 0.0")
            except Exception as e:
                print(f"[Control Error] {e}")
    except Exception as e:
        print(f"[Control Bind Error] Cannot bind to port {CONTROL_PORT}: {e}")

def run_server():
    """Main RSI server loop"""
    global start_time, korr_x_target, korr_x_prev
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', RSI_PORT))
    sock.settimeout(1.0)
    
    print(f"[RSI] Listening on 0.0.0.0:{RSI_PORT}")
    print("-" * 80)
    print(f"{'Time':<8} | {'IPOC':<5} | {'OV':<3} | {'Vel':<5} | {'X':<8} | {'Y':<8} | {'Z':<8} | {'RPM':<7}")
    print("-" * 80)
    sys.stdout.flush()

    # Start input thread
    input_thread = threading.Thread(target=user_input_thread, daemon=True)
    input_thread.start()

    is_connected = False
    last_print_time = time.time()

    try:
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                
                if not is_connected:
                    print(f"[Connected] {addr}\n")
                    start_time = time.time()
                    korr_x_prev = 0.0
                    is_connected = True
                    last_print_time = time.time()
                
                # Parse received data
                if parse_xml(data):
                    # Print state periodically (every 0.1 seconds to avoid too much output)
                    current_time = time.time()
                    if current_time - last_print_time >= 0.1:
                        print_robot_state()
                        last_print_time = current_time
                    
                    # Prepare correction response
                    delta_x = korr_x_target - korr_x_prev
                    korr_x_prev = korr_x_target
                    
                    reply_xml = (
                        f'<Sen Type="ImFree">'
                        f'<RKorr X="{delta_x:.4f}" Y="0.0000" Z="0.0000" A="0.0000" B="0.0000" C="0.0000" />'
                        f'<IPOC>{ipoc}</IPOC>'
                        f'</Sen>'
                    )
                    
                    sock.sendto(reply_xml.encode(), addr)
                
            except socket.timeout:
                if is_connected:
                    print("\n[Timeout] Connection lost - waiting for reconnection...")
                    is_connected = False
                continue

    except KeyboardInterrupt:
        print("\n[Server] Interrupted by user")
    finally:
        sock.close()
        print("[Server] Socket closed")

if __name__ == "__main__":
    print("=" * 100)
    print("RSI Real-Time Robot State Listener with RPM/Velocity Monitoring")
    print("=" * 100)
    print()
    run_server()
