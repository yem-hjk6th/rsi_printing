# KUKA RSI 实现示例代码集合

## 示例 1: 基础 KRL 实现（对应你的 RSI_PT_INT.src）

```krl
DEF RSI_PT_INT( )
    DECL INT ret
    DECL INT CONTID

    ;FOLD INI
      BAS (#INITMOV,0 )
    ;ENDFOLD (INI)

    BAS(#TOOL, 3)
    BAS(#BASE, 0)
    BAS(#VEL_PTP, 10)

    PTP $POS_ACT

    ; 创建 RSI 上下文 - 使用 RSI_CREATE 方式
    ret = RSI_CREATE("RSI_test.rsi", CONTID, TRUE)
    IF (ret <> RSIOK) THEN
      HALT
    ENDIF

    ; 启动 RSI - 相对模式用于修正
    ret = RSI_ON(#RELATIVE)
    IF (ret <> RSIOK) THEN
      HALT
    ENDIF

    ; 核心循环：持续接收和应用修正
    ; 这是 "执行预定路径同时接收RSI修正" 的关键
    WHILE TRUE
        RSI_MOVECORR()  ; 阻塞函数，每个4ms周期接收一个修正
    ENDWHILE

    ; 关闭 RSI
    ret = RSI_OFF()
END
```

---

## 示例 2: 增强版本 - 带日志和错误处理

```krl
DEF RSI_MotionWithCorrection()
    DECL INT ret
    DECL INT CONTID
    DECL INT correction_count = 0
    DECL INT max_corrections = 10000  ; 约40秒

    ; 初始化
    BAS(#INITMOV, 0)
    BAS(#TOOL, 3)
    BAS(#BASE, 0)
    BAS(#VEL_PTP, 20)

    ; 移动到起始位置
    PTP {A1 0, A2 -90, A3 90, A4 0, A5 90, A6 0}

    ; 创建RSI
    ret = RSI_CREATE("RSI_motion.rsi", CONTID, TRUE)
    IF (ret <> RSIOK) THEN
        ; 记录错误
        HALT
    ENDIF

    ; 启动RSI（相对修正模式）
    ret = RSI_ON(#RELATIVE)
    IF (ret <> RSIOK) THEN
        HALT
    ENDIF

    ; 修正循环 - 受限于最大迭代次数
    WHILE correction_count < max_corrections
        RSI_MOVECORR()
        INC correction_count
    ENDWHILE

    ; 关闭RSI
    ret = RSI_OFF()
    IF (ret <> RSIOK) THEN
        HALT
    ENDIF

    ; 返回安全位置
    PTP {A1 0, A2 -90, A3 90, A4 0, A5 90, A6 0}

END
```

---

## 示例 3: Python 侧 - 实时修正系统（改进版本）

```python
#!/usr/bin/env python3
"""
KUKA RSI 实时运动修正系统
支持轨迹插值和实时传感器修正
"""

import socket
import xml.etree.ElementTree as ET
import threading
import time
import math
from dataclasses import dataclass
from typing import Tuple, Optional

@dataclass
class RobotState:
    """机械臂当前状态"""
    ipoc: int
    axis_positions: dict  # {A1: 45.2, A2: -90.1, ...}
    timestamp: float

@dataclass
class Correction:
    """修正值"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    a: float = 0.0
    b: float = 0.0
    c: float = 0.0
    ipoc: int = -1


class KukaRSIController:
    """KUKA RSI 实时控制器"""
    
    def __init__(self, host: str = '0.0.0.0', port: int = 49152):
        self.host = host
        self.port = port
        self.socket = None
        self.running = False
        
        # 通信参数
        self.rsi_cycle_time = 0.004  # 4ms
        self.response_timeout = 0.010  # 10ms
        
        # 状态
        self.last_state: Optional[RobotState] = None
        self.last_ipoc = -1
        
        # 轨迹插值参数
        self.target_trajectory = []
        self.current_trajectory_index = 0
        self.trajectory_start_time = 0
        
    def initialize(self):
        """初始化UDP套接字"""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.settimeout(self.response_timeout)
        
        try:
            self.socket.bind((self.host, self.port))
            print(f"✓ RSI Server 启动在 {self.host}:{self.port}")
            self.running = True
        except Exception as e:
            print(f"✗ 绑定失败: {e}")
            return False
        
        return True
    
    def parse_robot_packet(self, data: bytes) -> Optional[RobotState]:
        """解析来自机械臂的XML数据包"""
        try:
            root = ET.fromstring(data)
            
            ipoc = int(root.find('IPOC').text)
            rist = root.find('RIst')
            
            # 提取所有轴位置
            axis_positions = {}
            for axis in ['A1', 'A2', 'A3', 'A4', 'A5', 'A6']:
                axis_positions[axis] = float(rist.get(axis))
            
            state = RobotState(
                ipoc=ipoc,
                axis_positions=axis_positions,
                timestamp=time.time()
            )
            
            return state
        
        except Exception as e:
            print(f"✗ 解析数据包失败: {e}")
            return None
    
    def create_correction_packet(self, correction: Correction) -> str:
        """创建修正XML数据包"""
        xml = f'''<Sen Type="ImFree"><RKorr X="{correction.x:.6f}" Y="{correction.y:.6f}" Z="{correction.z:.6f}" A="{correction.a:.6f}" B="{correction.b:.6f}" C="{correction.c:.6f}" /><IPOC>{correction.ipoc}</IPOC></Sen>'''
        return xml
    
    def plan_trajectory(self, start: dict, end: dict, 
                       duration: float, num_points: int = None) -> list:
        """
        规划光滑轨迹（线性插值）
        
        Args:
            start: 起始位置 {A1: val, A2: val, ...}
            end: 终止位置 {A1: val, A2: val, ...}
            duration: 执行时间（秒）
            num_points: 插值点数（如果为None，由RSI周期计算）
        
        Returns:
            轨迹点列表
        """
        if num_points is None:
            num_points = int(duration / self.rsi_cycle_time)
        
        trajectory = []
        axes = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6']
        
        for i in range(num_points):
            t = i / num_points  # 0.0 to 1.0
            point = {}
            
            for axis in axes:
                s = start[axis]
                e = end[axis]
                # 线性插值
                point[axis] = s + (e - s) * t
            
            trajectory.append(point)
        
        return trajectory
    
    def calculate_correction(self, state: RobotState) -> Correction:
        """
        计算当前的修正值
        
        这是关键的控制逻辑：
        1. 从轨迹中获取目标位置
        2. 计算误差
        3. 根据误差生成修正
        """
        if not self.target_trajectory:
            return Correction(ipoc=state.ipoc)
        
        # 获取当前目标
        idx = self.current_trajectory_index
        if idx >= len(self.target_trajectory):
            # 轨迹完成
            return Correction(ipoc=state.ipoc)
        
        target = self.target_trajectory[idx]
        current = state.axis_positions
        
        # 简单的比例修正（P控制）
        kp = 1.0  # 比例增益
        
        # 这里简化为只在Z方向修正
        # 实际应用中应对所有6轴进行修正
        error_z = target.get('A3', 0) - current.get('A3', 0)
        
        correction = Correction(
            z=error_z * kp,
            ipoc=state.ipoc
        )
        
        # 移动到下一个轨迹点
        self.current_trajectory_index += 1
        
        return correction
    
    def run(self):
        """主控制循环"""
        print("开始接收RSI数据包...")
        
        packet_count = 0
        max_packets = 10000  # 约40秒
        
        while self.running and packet_count < max_packets:
            try:
                # 接收机械臂数据包
                data, addr = self.socket.recvfrom(2048)
                
                # 解析状态
                state = self.parse_robot_packet(data)
                if not state:
                    continue
                
                # 检测IPOC增加（确认新包）
                if state.ipoc == self.last_ipoc:
                    continue
                
                self.last_ipoc = state.ipoc
                self.last_state = state
                
                # 计算修正值
                correction = self.calculate_correction(state)
                
                # 创建和发送修正数据包
                reply_xml = self.create_correction_packet(correction)
                self.socket.sendto(reply_xml.encode(), addr)
                
                # 日志
                if packet_count % 250 == 0:  # 每秒一次
                    print(f"[{packet_count}] IPOC={state.ipoc} "
                          f"Correction: Z={correction.z:.4f}")
                
                packet_count += 1
            
            except socket.timeout:
                print(f"✗ 接收超时，停止")
                self.running = False
                break
            
            except Exception as e:
                print(f"✗ 错误: {e}")
                continue
        
        print(f"完成: 处理了 {packet_count} 个数据包")
        self.shutdown()
    
    def shutdown(self):
        """关闭服务"""
        self.running = False
        if self.socket:
            self.socket.close()
        print("✓ RSI 控制器已关闭")


# 使用示例
if __name__ == "__main__":
    # 创建控制器
    controller = KukaRSIController(host='0.0.0.0', port=49152)
    
    if not controller.initialize():
        exit(1)
    
    # 规划轨迹：从当前位置移动到目标位置
    # （这只是示例，实际需要与机械臂起始位置匹配）
    start_pos = {
        'A1': 0,
        'A2': -90,
        'A3': 90,
        'A4': 0,
        'A5': 90,
        'A6': 0
    }
    
    target_pos = {
        'A1': 20,      # 移动20度
        'A2': -80,     # 移动10度
        'A3': 100,     # 移动10度
        'A4': 0,
        'A5': 90,
        'A6': 0
    }
    
    # 规划2秒的轨迹
    controller.target_trajectory = controller.plan_trajectory(
        start_pos, target_pos, duration=2.0
    )
    
    print(f"规划轨迹: {len(controller.target_trajectory)} 个点")
    print(f"预期时长: 2.0 秒")
    
    # 运行控制循环
    try:
        controller.run()
    except KeyboardInterrupt:
        print("\n用户中断")
        controller.shutdown()
```

---

## 示例 4: 高级 - 圆形轨迹插值（来自你的工作区）

```python
#!/usr/bin/env python3
"""
圆形轨迹执行with RSI修正
基于你工作区的 circle_coordinates.py
"""

import math
from kuka_rsi_controller import KukaRSIController

class CircularTrajectoryGenerator:
    """生成圆形轨迹"""
    
    def __init__(self, center: dict, radius: float, 
                 plane: str = 'XY', num_points: int = 100):
        """
        Args:
            center: 圆心 {x: ..., y: ..., z: ...}
            radius: 半径（mm）
            plane: 'XY', 'XZ', 或 'YZ'
            num_points: 轨迹点数
        """
        self.center = center
        self.radius = radius
        self.plane = plane
        self.num_points = num_points
    
    def generate(self) -> list:
        """生成圆形轨迹点"""
        trajectory = []
        
        for i in range(self.num_points):
            angle = 2 * math.pi * i / self.num_points
            
            x = self.center['x'] + self.radius * math.cos(angle)
            y = self.center['y'] + self.radius * math.sin(angle)
            z = self.center.get('z', 0)
            
            trajectory.append({
                'x': x,
                'y': y,
                'z': z,
                'angle': angle
            })
        
        return trajectory


# 使用示例
if __name__ == "__main__":
    # 创建RSI控制器
    controller = KukaRSIController()
    controller.initialize()
    
    # 生成圆形轨迹
    circle_gen = CircularTrajectoryGenerator(
        center={'x': 0, 'y': 100, 'z': 500},
        radius=50.0,
        num_points=250  # 1秒（250 * 4ms = 1000ms）
    )
    
    trajectory = circle_gen.generate()
    print(f"生成圆形轨迹: {len(trajectory)} 个点")
    
    # 转换为关节角度（简化示例）
    # 实际需要逆运动学求解
    controller.target_trajectory = trajectory
    
    # 运行
    controller.run()
```

---

## 示例 5: 完整的系统集成测试

```python
#!/usr/bin/env python3
"""
完整的RSI集成测试
"""

import time
import threading
from kuka_rsi_controller import KukaRSIController

def main():
    # 1. 初始化控制器
    print("=== KUKA RSI 系统集成测试 ===\n")
    
    controller = KukaRSIController(host='0.0.0.0', port=49152)
    
    if not controller.initialize():
        print("初始化失败！")
        return False
    
    # 2. 配置轨迹
    print("\n[配置阶段]")
    print("配置轨迹参数...")
    
    start_position = {
        'A1': 0,
        'A2': -90,
        'A3': 90,
        'A4': 0,
        'A5': 90,
        'A6': 0
    }
    
    target_position = {
        'A1': 30,
        'A2': -75,
        'A3': 105,
        'A4': 15,
        'A5': 85,
        'A6': 30
    }
    
    # 规划5秒的轨迹
    trajectory = controller.plan_trajectory(
        start_position, target_position, 
        duration=5.0
    )
    
    controller.target_trajectory = trajectory
    print(f"✓ 轨迹规划完成: {len(trajectory)} 个点")
    print(f"  预期执行时间: 5.0 秒")
    
    # 3. 启动控制循环
    print("\n[执行阶段]")
    print("等待机械臂连接...")
    print("(在KRL中启动 RSI_PT_INT 程序)\n")
    
    # 在线程中运行
    control_thread = threading.Thread(target=controller.run, daemon=True)
    control_thread.start()
    
    # 4. 监控执行
    try:
        while control_thread.is_alive():
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n\n用户中断")
    
    control_thread.join(timeout=5.0)
    
    # 5. 总结
    print("\n[结果总结]")
    if controller.last_state:
        print(f"最后位置:")
        for axis, value in controller.last_state.axis_positions.items():
            print(f"  {axis}: {value:.2f}°")
    
    print("\n✓ 测试完成")
    return True


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
```

---

## 关键要点

### KRL 实现
1. **使用 `RSI_MOVECORR()` 的 WHILE 循环** - 这是阻塞的实时循环
2. **相对模式 (`#RELATIVE`)** 用于修正（而不是绝对位置）
3. **准确的 IPOC 匹配** 确保数据对应

### Python 实现
1. **4ms 周期** - 不可协商的时间限制
2. **轨迹插值** 必须在外部系统中计算
3. **修正值计算** 是核心控制逻辑（P/PID控制）

### 系统集成
1. **网络要求** - UDP可靠性和延迟 < 10ms
2. **同步机制** - IPOC 计数器确保序列完整性
3. **错误处理** - 超时检测和优雅关闭

