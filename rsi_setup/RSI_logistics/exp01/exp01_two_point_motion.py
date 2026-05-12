#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import xml.etree.ElementTree as ET
import time

HOST = '0.0.0.0'
PORT = 59152

ipoc = "0"
start_time = None

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

def run_server():
    global start_time
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, PORT))
    sock.settimeout(1.0)
    
    print(f"RSI Server listening on {HOST}:{PORT}")
    print("-" * 80)
    print(f"{'Time':<8} | {'IPOC':<5} | {'OV':<3} | {'Vel':<5} | {'Z':<8} | {'Send_Z':<8}")
    print("-" * 80)

    is_connected = False

    try:
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                
                if not is_connected:
                    print(f"[Connected] {addr}\n")
                    start_time = time.time()
                    is_connected = True
                
                success, z_ist, override, vel_act = parse_xml(data)
                
                if success:
                    elapsed = time.time() - start_time
                    korr_z = 0.0
                    
                    print(f"T:{elapsed:6.2f} | {ipoc:<5} | {override:<3} | {vel_act:<5} | {z_ist:7.2f} | {korr_z:7.4f}", end='\r')

                    reply_xml = (
                        f'<Sen Type="ImFree">'
                        f'<RKorr X="0.0000" Y="0.0000" Z="{korr_z:.4f}" A="0.0000" B="0.0000" C="0.0000" />'
                        f'<IPOC>{ipoc}</IPOC>'
                        f'</Sen>'
                    )
                    
                    sock.sendto(reply_xml.encode(), addr)
                
            except socket.timeout:
                if is_connected:
                    print("\n[Timeout] Connection lost")
                    is_connected = False
                continue

    except KeyboardInterrupt:
        print("\nServer stopped")
    finally:
        sock.close()

if __name__ == "__main__":
    run_server()
