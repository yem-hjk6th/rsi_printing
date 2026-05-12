#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RSI Async Keyboard Control - Version 2
Using msvcrt for non-blocking keyboard input
Gradual step-based X-axis motion control with ±10mm limit
"""

import socket
import xml.etree.ElementTree as ET
import time
import msvcrt

# === RSI Configuration ===
HOST = '0.0.0.0'
PORT = 59152
RSI_CYCLE_TIME = 0.004  # 4ms per cycle

# === Motion Parameters ===
STEP_SIZE = 3.0         # mm per keypress (target distance)
MOVE_SPEED = 1.0        # mm/s (gradual movement speed) - Reduced to avoid velocity limit
STEP_INCREMENT = MOVE_SPEED * RSI_CYCLE_TIME  # 0.004mm per cycle
MAX_POSITIVE = 10.0     # +X limit (mm)
MAX_NEGATIVE = -10.0    # -X limit (mm)

# === Global State ===
ipoc = "0"
start_time = None
korr_x_target = 0.0     # Target accumulated X correction
korr_x_current = 0.0    # Current accumulated X position
korr_x_prev = 0.0       # Previous sent value for delta calculation

def parse_xml(data):
    """Parse RSI XML packet and extract key information"""
    global ipoc
    try:
        root = ET.fromstring(data)
        
        ipoc_elem = root.find('IPOC')
        if ipoc_elem is not None:
            ipoc = ipoc_elem.text
        
        # Get real robot position
        rist = root.find('RIst')
        if rist is not None:
            x_ist = float(rist.get('X', 0))
            y_ist = float(rist.get('Y', 0))
            z_ist = float(rist.get('Z', 0))
        else:
            x_ist = y_ist = z_ist = 0.0
        
        # Get override and velocity
        override = 0
        ov_elem = root.find('Override')
        if ov_elem is not None:
            override = int(ov_elem.text or 0)
        
        vel_act = 0
        vel_elem = root.find('Vel_Act')
        if vel_elem is not None:
            vel_act = int(vel_elem.text or 0)
        
        return True, x_ist, y_ist, z_ist, override, vel_act
    
    except Exception as e:
        print(f"[XML_ERROR] {e}")
        return False, 0.0, 0.0, 0.0, 0, 0

def handle_keyboard_input():
    """Non-blocking keyboard check using msvcrt"""
    global korr_x_target
    
    if msvcrt.kbhit():
        try:
            key = msvcrt.getch().decode('utf-8').lower()
            
            if key == 'm':
                # Move positive X direction
                new_target = korr_x_target + STEP_SIZE
                if new_target <= MAX_POSITIVE:
                    korr_x_target = new_target
                    print(f"\n[CMD] +{STEP_SIZE}mm → X_Target: {korr_x_target:+.2f}mm")
                else:
                    print(f"\n[LIMIT] Cannot exceed +{MAX_POSITIVE}mm (current: {korr_x_target:+.2f}mm)")
                return True
            
            elif key == 'n':
                # Move negative X direction
                new_target = korr_x_target - STEP_SIZE
                if new_target >= MAX_NEGATIVE:
                    korr_x_target = new_target
                    print(f"\n[CMD] -{STEP_SIZE}mm → X_Target: {korr_x_target:+.2f}mm")
                else:
                    print(f"\n[LIMIT] Cannot exceed {MAX_NEGATIVE}mm (current: {korr_x_target:+.2f}mm)")
                return True
            
            elif key == 'r':
                # Reset to zero
                korr_x_target = 0.0
                print(f"\n[CMD] Reset → X_Target: 0.00mm")
                return True
            
            elif key == 'q':
                # Quit signal
                return 'quit'
            
        except Exception as e:
            print(f"[KEY_ERROR] {e}")
    
    return False

def run_async_server():
    """Main RSI server with async keyboard control"""
    global start_time, korr_x_prev, korr_x_current
    
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(1.0)  # 1s timeout for connection check
    
    try:
        sock.bind((HOST, PORT))
        print("=" * 80)
        print(f"RSI Async Keyboard Server V2 started on {HOST}:{PORT}")
        print(f"STEP_SIZE: {STEP_SIZE}mm | SPEED: {MOVE_SPEED}mm/s | X_LIMIT: [{MAX_NEGATIVE}, {MAX_POSITIVE}]mm")
        print("=" * 80)
        print("\nKeyboard Controls:")
        print("  [m] - Move X +3mm")
        print("  [n] - Move X -3mm")
        print("  [r] - Reset to X=0")
        print("  [q] - Quit program")
        print("\nWaiting for RSI connection...\n")
    except Exception as e:
        print(f"[BIND_ERROR] {e}")
        return
    
    is_connected = False
    packet_count = 0
    last_print_time = 0
    timeout_count = 0
    
    try:
        while True:
            try:
                # Receive RSI packet (blocking with 1s timeout)
                data, addr = sock.recvfrom(2048)
                packet_count += 1
                timeout_count = 0  # Reset timeout counter
                current_time = time.time()
                
                # First connection handling
                if not is_connected:
                    print(f"[CONNECTED] Robot controller at {addr}")
                    print("-" * 80)
                    print(f"{'Time':<8} | {'IPOC':<6} | {'OV':<3} | {'Vel':<5} | {'Real_X':<8} | {'Tgt/Cur':<18} | {'Delta_X':<8}")
                    print("-" * 80)
                    start_time = current_time
                    korr_x_prev = 0.0
                    korr_x_current = 0.0
                    # Don't reset korr_x_target - keep user's keyboard input
                    is_connected = True
                
                # Check keyboard input (non-blocking, after receiving data)
                kb_result = handle_keyboard_input()
                if kb_result == 'quit':
                    print("\n[EXIT] User requested quit")
                    break
                
                # Parse incoming data
                success, x_ist, y_ist, z_ist, override, vel_act = parse_xml(data)
                
                if success:
                    elapsed = current_time - start_time
                    
                    # Gradual movement: approach target step by step
                    diff = korr_x_target - korr_x_current
                    if abs(diff) > 0.001:  # Still moving towards target
                        if diff > 0:
                            korr_x_current = min(korr_x_current + STEP_INCREMENT, korr_x_target)
                        else:
                            korr_x_current = max(korr_x_current - STEP_INCREMENT, korr_x_target)
                    
                    # Calculate delta (differential for HOLDON=1 mode)
                    delta_x = korr_x_current - korr_x_prev
                    korr_x_prev = korr_x_current
                    
                    # Print status every 0.5s or when command received
                    if kb_result or (current_time - last_print_time) >= 0.5:
                        print(f"T:{elapsed:6.2f} | {ipoc:<6} | {override:<3} | {vel_act:<5} | "
                              f"{x_ist:7.2f} | Tgt:{korr_x_target:+6.2f} Cur:{korr_x_current:+6.2f} | {delta_x:+7.4f}",
                              end='\r' if not kb_result else '\n')
                        last_print_time = current_time
                    
                    # Build reply XML
                    reply_xml = (
                        f'<Sen Type="ImFree">'
                        f'<RKorr X="{delta_x:.4f}" Y="0.0000" Z="0.0000" '
                        f'A="0.0000" B="0.0000" C="0.0000" />'
                        f'<IPOC>{ipoc}</IPOC>'
                        f'</Sen>'
                    )
                    
                    # Send reply
                    sock.sendto(reply_xml.encode(), addr)
            
            except socket.timeout:
                timeout_count += 1
                if is_connected and timeout_count >= 3:
                    print("\n[TIMEOUT] Connection lost, waiting for reconnection...")
                    is_connected = False
                    timeout_count = 0
                continue
            
            except Exception as e:
                print(f"\n[COMM_ERROR] {e}")
                continue
    
    except KeyboardInterrupt:
        print("\n\n[STOPPED] Keyboard interrupt (Ctrl+C)")
    
    finally:
        print("=" * 80)
        print(f"[SUMMARY]")
        print(f"  Total Packets: {packet_count}")
        print(f"  Final X Target: {korr_x_target:+.2f}mm")
        if start_time:
            print(f"  Total Time: {time.time() - start_time:.2f}s")
        print("=" * 80)
        sock.close()
        print("[CLOSED] Socket closed")

if __name__ == "__main__":
    run_async_server()
