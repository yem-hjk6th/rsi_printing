import socket
import xml.etree.ElementTree as ET
import math
import time

# --- 配置区域 ---
HOST = '0.0.0.0'  # 监听所有网卡
PORT = 59152      # 必须与 XML 配置文件里的 <PORT> 一致
RSI_CYCLE_TIME = 0.004 # RSI 默认周期 4ms

# --- 初始化 UDP 服务器 ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((HOST, PORT))

print(f"RSI Server listening on {HOST}:{PORT}...")

# 变量存储上一帧的位置
prev_pos = None

try:
    while True:
        # 1. 接收数据 (阻塞等待)
        # data 是字节流, addr 是机器人的 IP
        data, addr = sock.recvfrom(1024)
        
        # 2. 解析 XML
        try:
            root = ET.fromstring(data)
            
            # 找到 <RIst> 标签 (实际位置)
            # 根据你的 xml 配置，标签名可能是 'RIst' 或其他，通常是 RIst
            rist = root.find('RIst')
            
            if rist is not None:
                # 提取 XYZ (注意 XML 里是字符串，要转 float)
                x = float(rist.get('X'))
                y = float(rist.get('Y'))
                z = float(rist.get('Z'))
                
                current_pos = (x, y, z)
                
                # 3. 计算速度
                if prev_pos is not None:
                    # 计算欧几里得距离 (3D空间移动距离)
                    dist = math.sqrt(
                        (x - prev_pos[0])**2 + 
                        (y - prev_pos[1])**2 + 
                        (z - prev_pos[2])**2
                    )
                    
                    # 速度 v = d / t
                    speed_mms = dist / RSI_CYCLE_TIME
                    
                    # 打印结果
                    print(f"Pos: [{x:.1f}, {y:.1f}, {z:.1f}] | Speed: {speed_mms:.2f} mm/s")
                else:
                    print(f"Pos: [{x:.1f}, {y:.1f}, {z:.1f}] | Initializing...")

                # 更新上一帧位置
                prev_pos = current_pos

                # 4. 【重要】必须给机器人回信！
                # RSI 是同步通讯，机器人发给你，你必须回发给它，否则机器人会报错 "Timeout"
                # 这里回发一个不做任何修正的空包 (RKorr)
                reply_xml = f'<Sen Type="ImFree"><RKorr X="0.0" Y="0.0" Z="0.0" A="0.0" B="0.0" C="0.0" /><IPOC>{root.find("IPOC").text}</IPOC></Sen>'
                sock.sendto(reply_xml.encode(), addr)

        except ET.ParseError:
            print("XML Parse Error")
            
except KeyboardInterrupt:
    print("\nServer Stopped.")
    sock.close()