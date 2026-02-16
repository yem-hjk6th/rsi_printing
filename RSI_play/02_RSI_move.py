import socket
import xml.etree.ElementTree as ET
import time

HOST = '0.0.0.0'
PORT = 59152
RSI_CYCLE_TIME = 0.004

MOVE_SPEED = 1 
TARGET_DISTANCE = 50 
STEP_INCREMENT = MOVE_SPEED * RSI_CYCLE_TIME

def run_controller():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        sock.bind((HOST, PORT))
        print(f"RSI Server started on port {PORT}")
    except Exception as e:
        print(f"Bind Error: {e}")
        return

    accumulated_dist = 0.0
    packet_count = 0
    
    try:
        while True:
            data, addr = sock.recvfrom(2048)
            packet_count += 1
            
            try:
                root = ET.fromstring(data)
                ipoc = root.find('IPOC').text
                rist = root.find('RIst')
                current_real_z = float(rist.get('Z'))
                
                delta_z = 0.0
                
                if packet_count < 100:
                    delta_z = 0.0
                elif accumulated_dist < TARGET_DISTANCE:
                    delta_z = STEP_INCREMENT
                    accumulated_dist += delta_z
                else:
                    delta_z = 0.0
                
                reply_xml = f'<Sen Type="ImFree"><RKorr X="0.0" Y="0.0" Z="{delta_z:.4f}" A="0.0" B="0.0" C="0.0" /><IPOC>{ipoc}</IPOC></Sen>'
                
                sock.sendto(reply_xml.encode(), addr)

                if packet_count % 50 == 0:
                    print(f"Real Z: {current_real_z:.2f} | Sending Delta Z: {delta_z:.4f} | Total Moved: {accumulated_dist:.4f}")

            except ET.ParseError:
                continue

    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        sock.close()

if __name__ == "__main__":
    run_controller()