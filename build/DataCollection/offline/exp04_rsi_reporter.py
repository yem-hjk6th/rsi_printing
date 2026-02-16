#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RSI Simple Listener - Minimal Version
Only receive and display data, no motion correction
Keep RSI connection alive by sending zero corrections
"""

import socket
import xml.etree.ElementTree as ET
import time
import csv
import os
from datetime import datetime

HOST = '0.0.0.0'
PORT = 59152

ipoc = "0"
OUTPUT_DIR = "rsi_data"
FLUSH_INTERVAL = 250

def parse_xml(data):
    """Parse RSI XML packet and extract information"""
    global ipoc
    try:
        root = ET.fromstring(data)
        
        # Extract IPOC
        ipoc_elem = root.find('IPOC')
        if ipoc_elem is not None:
            ipoc = ipoc_elem.text
        
        # Extract robot position (RIst)
        rist = root.find('RIst')
        if rist is not None:
            x_ist = float(rist.get('X', 0))
            y_ist = float(rist.get('Y', 0))
            z_ist = float(rist.get('Z', 0))
            a_ist = float(rist.get('A', 0))
            b_ist = float(rist.get('B', 0))
            c_ist = float(rist.get('C', 0))
        else:
            x_ist = y_ist = z_ist = a_ist = b_ist = c_ist = 0.0
        
        # Extract Override
        override = 0
        ov_elem = root.find('Override')
        if ov_elem is not None:
            override = int(ov_elem.text or 0)
        
        # Extract Velocity
        vel_act = 0
        vel_elem = root.find('Vel_Act')
        if vel_elem is not None:
            vel_act = int(vel_elem.text or 0)
        
        # Extract RPM
        rpm_ext = 0
        rpm_elem = root.find('RPM_Ext')
        if rpm_elem is not None:
            rpm_ext = int(rpm_elem.text or 0)
        
        return True, x_ist, y_ist, z_ist, a_ist, b_ist, c_ist, override, vel_act, rpm_ext
    
    except Exception as e:
        print(f"[XML_ERROR] {e}")
        return False, 0, 0, 0, 0, 0, 0, 0, 0, 0

def run_listener():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(OUTPUT_DIR, f"rsi_data_{timestamp_str}.csv")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(1.0)
    
    try:
        sock.bind((HOST, PORT))
        print("=" * 100)
        print(f"RSI Simple Listener started on {HOST}:{PORT}")
        print(f"CSV output: {csv_path}")
        print("=" * 100)
        print("Waiting for RSI connection from robot controller...\n")
    except Exception as e:
        print(f"[BIND_ERROR] {e}")
        return
    
    is_connected = False
    packet_count = 0
    start_time = None
    last_print_time = 0
    timeout_count = 0
    csv_file = None
    csv_writer = None
    
    try:
        while True:
            try:
                data, addr = sock.recvfrom(2048)
                packet_count += 1
                timeout_count = 0
                current_time = time.time()
                
                if not is_connected:
                    print(f"[CONNECTED] Robot controller at {addr}\n")
                    print("-" * 100)
                    print(f"{'Time':<8} | {'IPOC':<8} | {'X':<9} | {'Y':<9} | {'Z':<9} | {'OV%':<5} | {'Vel':<6} | {'RPM':<6}")
                    print("-" * 100)
                    start_time = current_time
                    is_connected = True
                    
                    csv_file = open(csv_path, 'w', newline='')
                    csv_writer = csv.writer(csv_file)
                    csv_writer.writerow([f"start_time: {start_time}"])
                    csv_writer.writerow(["time_s", "ipoc", "x", "y", "z", "a", "b", "c", "override", "vel", "rpm"])
                    csv_file.flush()
                
                success, x_ist, y_ist, z_ist, a_ist, b_ist, c_ist, override, vel_act, rpm_ext = parse_xml(data)
                
                if success:
                    elapsed = current_time - start_time
                    
                    if (current_time - last_print_time) >= 0.1:
                        print(f"{elapsed:7.2f}s | {ipoc:<8} | {x_ist:8.2f} | {y_ist:8.2f} | {z_ist:8.2f} | "
                              f"{override:<5} | {vel_act:<6} | {rpm_ext:<6}", end='\r')
                        last_print_time = current_time
                    
                    csv_writer.writerow([f"{elapsed:.6f}", ipoc, f"{x_ist:.2f}", f"{y_ist:.2f}", f"{z_ist:.2f}", 
                                        f"{a_ist:.2f}", f"{b_ist:.2f}", f"{c_ist:.2f}", override, vel_act, rpm_ext])
                    if packet_count % FLUSH_INTERVAL == 0:
                        csv_file.flush()
                    
                    reply_xml = (
                        f'<Sen Type="ImFree">'
                        f'<RKorr X="0.0000" Y="0.0000" Z="0.0000" '
                        f'A="0.0000" B="0.0000" C="0.0000" />'
                        f'<IPOC>{ipoc}</IPOC>'
                        f'</Sen>'
                    )
                    
                    sock.sendto(reply_xml.encode(), addr)
            
            except socket.timeout:
                timeout_count += 1
                if is_connected and timeout_count >= 3:
                    print("\n\n[TIMEOUT] Connection lost, waiting for reconnection...")
                    is_connected = False
                    timeout_count = 0
                continue
            
            except Exception as e:
                print(f"\n[COMM_ERROR] {e}")
                continue
    
    except KeyboardInterrupt:
        print("\n\n[STOPPED] Keyboard interrupt (Ctrl+C)")
    
    finally:
        if csv_file:
            csv_file.flush()
            csv_file.close()
        print("\n" + "=" * 100)
        print(f"[SUMMARY]")
        print(f"  Total Packets Received: {packet_count}")
        if start_time:
            print(f"  Total Connection Time: {time.time() - start_time:.2f}s")
        print(f"  CSV saved: {csv_path}")
        print("=" * 100)
        sock.close()
        print("[CLOSED] Socket closed")

if __name__ == "__main__":
    run_listener()
