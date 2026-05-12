import socket
import xml.etree.ElementTree as ET
import time
import math
import sys

# --- Configuration ---
HOST = '0.0.0.0'
PORT = 59152
AMPLITUDE = 9.5  # 目标振幅 10mm
FREQUENCY = 0.5   # 0.5 Hz (2秒一个周期)

# Global variables
ipoc = "0"
start_time = None
# [关键变量] 记录上一次计算出的理论位置，用于求差值
prev_target_z_pos = 0.0 

def parse_xml(data):
    """
    解析 XML，增加对 RPM_Ext 的支持
    """
    global ipoc
    try:
        root = ET.fromstring(data)
        
        # 1. IPOC
        ipoc_element = root.find('IPOC')
        if ipoc_element is not None:
            ipoc = ipoc_element.text
        
        # 2. RIst (实际位置 Z)
        z_pos = 0.0
        rist = root.find('RIst')
        if rist is not None:
            z_pos = float(rist.get('Z'))
            
        # 3. Override (倍率)
        override = 0
        ov_element = root.find('Override')
        if ov_element is not None:
            override = int(ov_element.text)
            
        # 4. Vel_Act (实际速度)
        vel_act = 0
        vel_element = root.find('Vel_Act')
        if vel_element is not None:
            vel_act = int(vel_element.text)

        # 5. RPM_Ext (新增: 你的 Config 中定义的 RPM)
        rpm = 0
        rpm_element = root.find('RPM_Ext')
        if rpm_element is not None:
            rpm = int(rpm_element.text)

        return True, z_pos, override, vel_act, rpm
        
    except Exception as e:
        print(f"[ERROR] Data Error: {e}")
        return False, 0.0, 0, 0, 0

def run_server():
    global start_time, prev_target_z_pos
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, PORT))
    sock.settimeout(1.0)
    print(f"RSI Server listening on {HOST}:{PORT}")
    print("Mode: Sine Wave (Differential) + Full Monitor")
    print("-" * 100)
    print(f"{'Time':<8} | {'OV':<4} | {'Vel':<6} | {'RPM':<6} | {'Real_Z':<8} | {'Tgt_Z':<8} | {'Send_dZ':<8}")
    print("-" * 100)

    is_connected = False

    try:
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                
                # 1. 连接初始化
                if not is_connected:
                    # 换行打印，避免冲掉表头
                    print(f"\n[INFO] Connection established with {addr}. Resetting Timer.")
                    start_time = time.time()
                    prev_target_z_pos = 0.0 
                    is_connected = True
                
                # 2. 计算
                elapsed = time.time() - start_time
                success, real_z_pos, override, vel_act, rpm = parse_xml(data)
                
                if success:
                    delta_z_to_send = 0.0
                    
                    if override > 0:
                        # --- 差分运动逻辑 ---
                        ramp_factor = min(elapsed / 2.0, 1.0)
                        current_target_z_pos = (AMPLITUDE * ramp_factor) * math.sin(2 * math.pi * FREQUENCY * elapsed)
                        delta_z_to_send = current_target_z_pos - prev_target_z_pos
                        prev_target_z_pos = current_target_z_pos
                    else:
                        # 倍率为0，暂停生成波形，重置时间防止跳变
                        delta_z_to_send = 0.0
                        prev_target_z_pos = 0.0 
                        start_time = time.time()

                    # 3. 实时打印 (使用 \r 覆盖当前行)
                    # 格式说明:
                    # OV: 倍率 %
                    # Vel: 机器人实际速度 (内部单位)
                    # RPM: 你的 Config 中映射的 RPM 值
                    # Tgt_Z: 我们的正弦波理论位置
                    # Send_dZ: 发送给机器人的微小增量
                    print(f"T:{elapsed:6.2f} | OV:{override:<3} | Vel:{vel_act:<5} | RPM:{rpm:<5} | Z:{real_z_pos:7.2f} | Tgt:{prev_target_z_pos:6.2f} | dZ:{delta_z_to_send:6.4f}  ", end='\r')

                    # 4. 发送回复
                    reply_xml = (
                        f'<Sen Type="ImFree">'
                        f'<RKorr X="0.0000" Y="0.0000" Z="{delta_z_to_send:.4f}" A="0.0000" B="0.0000" C="0.0000" />'
                        f'<IPOC>{ipoc}</IPOC>'
                        f'</Sen>'
                    )
                    
                    sock.sendto(reply_xml.encode(), addr)
                
            except socket.timeout:
                if is_connected:
                    print("\n[INFO] Connection lost. Waiting for robot...")
                    is_connected = False
                continue
            except Exception as e:
                print(f"\n[Error] {e}")
                break

    except KeyboardInterrupt:
        print("\nServer stopped.")
    finally:
        sock.close()

if __name__ == "__main__":
    run_server()