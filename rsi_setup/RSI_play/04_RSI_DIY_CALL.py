"""
RSI 连接测试工具 - 建立 UDP 服务器接收机器人 RSI 数据，并显示 SEND 配置的值
"""

import socket
import xml.etree.ElementTree as ET
import time
import os


class RSIServer:
    """RSI 连接测试服务器"""
    
    def __init__(self, host='0.0.0.0', port=59152, config_path=None, timeout=None, verbose=False, save_xml=True):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock = None
        self.packet_count = 0
        self.start_time = None
        self.send_config = {}  # 存储 SEND 配置
        self.config_path = config_path or os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..",
            "RSI_set_ver", "Mine", "ver5_10mm_var", "RSI_EthernetConfig.xml")
        self.verbose = verbose  # 是否打印原始 XML
        self.save_xml = save_xml  # 是否保存 XML 到文件
        self.xml_file = None
        
        if self.save_xml:
            self.xml_file = open('rsi_packets.xml', 'w', encoding='utf-8')
            self.xml_file.write("<!-- RSI 原始数据包记录 -->\n")
        
        # 加载配置
        self.load_send_config()
        
    def load_send_config(self):
        """从 RSI_EthernetConfig.xml 加载 SEND 配置"""
        if not os.path.exists(self.config_path):
            print(f"⚠️  配置文件未找到: {self.config_path}")
            return
        
        try:
            tree = ET.parse(self.config_path)
            root = tree.getroot()
            
            send = root.find('SEND')
            if send is not None:
                for elem in send.findall('ELEMENT'):
                    tag = elem.get('TAG')
                    indx = elem.get('INDX')
                    elem_type = elem.get('TYPE')
                    
                    # 只保存有实际索引的元素（跳过 INTERNAL）
                    if indx and indx != 'INTERNAL':
                        self.send_config[int(indx)] = {
                            'TAG': tag,
                            'TYPE': elem_type,
                            'INDX': int(indx)
                        }
                
                # 按索引排序
                if self.send_config:
                    print(f"✅ 加载 SEND 配置成功，{len(self.send_config)} 个数据元素:")
                    for indx in sorted(self.send_config.keys()):
                        cfg = self.send_config[indx]
                        print(f"   Out{cfg['INDX']}: {cfg['TAG']:20} ({cfg['TYPE']})")
                    print()
        except Exception as e:
            print(f"⚠️  加载配置文件失败: {e}\n")
        
    def start(self):
        """启动 RSI 服务器并监听连接"""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            self.sock.bind((self.host, self.port))
            self.start_time = time.time()
            
            print(f"\n{'='*70}")
            print(f"RSI 连接测试服务器")
            print(f"{'='*70}")
            print(f"📡 监听地址: {self.host}:{self.port}")
            print(f"⏱️  等待机器人连接... (按 Ctrl+C 停止)\n")
            
            while True:
                try:
                    data, addr = self.sock.recvfrom(2048)
                    self.packet_count += 1
                    elapsed = time.time() - self.start_time
                    
                    # 解析 XML 数据
                    try:
                        root = ET.fromstring(data)
                        
                        # 保存原始 XML 到文件（前 10 个包）
                        if self.save_xml and self.packet_count <= 10:
                            xml_str = data.decode('utf-8', errors='ignore')
                            self.xml_file.write(f"\n<!-- 数据包 #{self.packet_count} -->\n{xml_str}\n")
                            self.xml_file.flush()
                        
                        # 如果启用 verbose，打印原始 XML（仅前 5 个包）
                        if self.verbose and self.packet_count <= 5:
                            print(f"\n【第 {self.packet_count} 个数据包 - 原始 XML】")
                            print(data.decode('utf-8', errors='ignore'))
                            print()
                        
                        ipoc = root.find('IPOC')
                        ipoc_val = ipoc.text if ipoc is not None else "N/A"
                        
                        rist = root.find('RIst')
                        
                        if rist is not None:
                            x = rist.get('X', 'N/A')
                            y = rist.get('Y', 'N/A')
                            z = rist.get('Z', 'N/A')
                            
                            # 解析 SEND 配置中的数据（直接子元素）
                            send_values = {}
                            for indx, cfg in self.send_config.items():
                                tag = cfg['TAG']
                                elem = root.find(tag)
                                if elem is not None and elem.text is not None:
                                    send_values[tag] = elem.text
                            
                            # 构建输出信息
                            info = f"[{self.packet_count:05d}] {elapsed:7.2f}s | 机器人 {addr[0]}:{addr[1]} | "
                            info += f"RIst(X:{float(x):8.2f} Y:{float(y):8.2f} Z:{float(z):8.2f})"
                            
                            # 添加 SEND 数据
                            if send_values:
                                send_str = " ".join([f"{k}:{v}" for k, v in send_values.items()])
                                info += f" | SEND: {send_str}"
                            
                            info += f" | IPOC:{ipoc_val}"
                            print(info)
                        
                        # 立即回复修正值（重要！必须回复）
                        reply_xml = f'<Sen Type="ImFree"><RKorr X="0.0" Y="0.0" Z="0.0" A="0.0" B="0.0" C="0.0" /><IPOC>{ipoc_val}</IPOC></Sen>'
                        self.sock.sendto(reply_xml.encode(), addr)
                        
                    except ET.ParseError as e:
                        print(f"[{self.packet_count:05d}] ❌ XML 解析错误: {e}")
                        
                except socket.timeout:
                    print(f"\n⚠️  没有接收到数据，等待中... ({elapsed:.1f}s)")
                    
        except KeyboardInterrupt:
            self.print_summary()
        except OSError as e:
            print(f"\n❌ 网络错误: {e}")
            print(f"   可能原因:")
            print(f"   - 端口被占用")
            print(f"   - 权限不足")
        except Exception as e:
            print(f"\n❌ 错误: {e}")
        finally:
            if self.xml_file:
                self.xml_file.close()
                print(f"\n✅ 原始 XML 数据已保存到: rsi_packets.xml")
            if self.sock:
                self.sock.close()
    
    def print_summary(self):
        """打印统计信息"""
        elapsed = time.time() - self.start_time if self.start_time else 0
        print(f"\n\n{'='*70}")
        print(f"📊 连接测试结果")
        print(f"{'='*70}")
        if self.packet_count > 0:
            print(f"✅ 连接成功！")
            print(f"   接收数据包: {self.packet_count} 个")
            print(f"   运行时间: {elapsed:.2f} 秒")
            print(f"   平均频率: {self.packet_count/elapsed:.1f} Hz (预期 250Hz)")
        else:
            print(f"❌ 未接收到任何数据")
            print(f"   检查项:")
            print(f"   - 机器人 IP 是否正确")
            print(f"   - 端口号是否为 59152")
            print(f"   - 防火墙是否允许 UDP")
            print(f"   - 机器人 RSI_EthernetConfig.xml 配置")


def main():
    """主程序"""
    import argparse
    
    parser = argparse.ArgumentParser(
        description='RSI 连接测试工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python 06_RSI_DIY_CALL.py                    # 默认 0.0.0.0:59152
  python 06_RSI_DIY_CALL.py -p 49152           # 使用端口 49152
  python 06_RSI_DIY_CALL.py -h 192.168.1.100  # 绑定特定 IP
        """
    )
    
    parser.add_argument('-p', '--port', type=int, default=59152, 
                        help='监听端口 (默认: 59152)')
    parser.add_argument('-i', '--ip', default='0.0.0.0', 
                        help='监听 IP (默认: 0.0.0.0 - 所有网卡)')
    parser.add_argument('-c', '--config', default=None,
                        help='RSI_EthernetConfig.xml 路径')
    parser.add_argument('-t', '--timeout', type=int, default=None,
                        help='超时时间（秒），为空表示无限等待')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='打印原始 XML 数据包（仅前 5 个）')
    parser.add_argument('--no-save', action='store_true',
                        help='不保存 XML 到文件')
    
    args = parser.parse_args()
    
    server = RSIServer(host=args.ip, port=args.port, config_path=args.config, 
                      timeout=args.timeout, verbose=args.verbose, save_xml=not args.no_save)
    server.start()


if __name__ == "__main__":
    main()
