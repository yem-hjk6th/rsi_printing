# KUKA Robot Sensor Interface (RSI) 实现教程和代码指南

## 📚 核心资源汇总

### 1. 官方资源和论文

#### 学术论文（Google Scholar）
| 资源 | 链接 | 描述 |
|------|------|------|
| **动态轨迹控制实现** | [Implementation of dynamic trajectory control for an industrial robot by external dSPACE real-time system using the KUKA RSI interface](https://opus4.kobv.de/opus4-haw/files/4998/I002130628Thesis.pdf) | 使用外部dSPACE系统通过RSI进行动态轨迹控制的完整实现 |
| **KVP vs RSI 对比** | [Comparison of KVP and RSI for controlling KUKA robots over ROS](https://www.sciencedirect.com/science/article/pii/S2405896320334509) | KUKA两种控制接口的详细比较，包括性能评估 |
| **ANFIS控制器实现** | [Development of ANFIS controller for trajectory tracking control using ROBO2L](https://ieeexplore.ieee.org/abstract/document/10548192/) | 使用ROBO2L MATLAB工具箱通过RSI进行轨迹跟踪控制 |
| **力/力矩传感器集成** | [Force-torque sensor integration in industrial robot control](https://ieeexplore.ieee.org/abstract/document/6920241/) | 通过RSI集成F/T传感器进行力控制 |
| **MATLAB工具箱开发** | [Development of a matlab toolbox for robot sensor interface (RSI) communication with KUKA KR6](https://ieeexplore.ieee.org/abstract/document/10735530/) | RSI通信的MATLAB工具箱（2024年最新） |
| **低成本同步技术** | [Low-Cost Synchronization Techniques for KUKA Robots and External Axes](https://www.scitepress.org/Papers/2023/122079/122079.pdf) | RSI、FSD和解释器的同步技术对比 |

### 2. GitHub 官方实现

#### ROS-Industrial RSI 硬件接口
- **主仓库**: [https://github.com/ros-industrial/kuka_experimental](https://github.com/ros-industrial/kuka_experimental)
- **RSI 硬件接口**: `kuka_rsi_hw_interface` 目录
- **KRL 代码示例**:
  - KR-C2 控制器: `kuka_rsi_hw_interface/krl/KR_C2/ros_rsi.src`
  - KR-C4 控制器: `kuka_rsi_hw_interface/krl/KR_C4/ros_rsi.src`

---

## 🔑 KRL 代码关键模式

### 模式1：RSI基础初始化和执行

#### 使用RSI_MOVECORR()的最简单方式（KR-C4）
```krl
DEF RSI_AxisCorr( )
  DECL INT ret
  DECL INT CONTID

  ; 移动到起始位置
  PTP {A1 0, A2 -90, A3 90, A4 0, A5 90, A6 0}

  ; 创建RSI上下文
  ret = RSI_CREATE("ros_rsi.rsi", CONTID, TRUE)
  IF (ret <> RSIOK) THEN
    HALT
  ENDIF

  ; 启动RSI执行 (绝对模式)
  ret = RSI_ON(#ABSOLUTE)
  IF (ret <> RSIOK) THEN
    HALT
  ENDIF

  ; 传感器引导运动 - 核心循环！
  ; RSI_MOVECORR()会循环等待外部系统的修正值
  ; 每个周期 (4ms) 接收一个XML数据包并在修正后继续
  RSI_MOVECORR()

  ; 关闭RSI
  ret = RSI_OFF()
  IF (ret <> RSIOK) THEN
    HALT
  ENDIF

  PTP {A1 0, A2 -90, A3 90, A4 0, A5 90, A6 0}

END
```

#### 使用ST_*函数的高级版本（KR-C2）
```krl
; ============ 配置和初始化 ============
numAxes = 6
HOME = {AXIS: A1 0, A2 -90, A3 90, A4 0, A5 90, A6 0}
configFileName[] = "ros_rsi_ethernet.xml"

; 设置各轴的修正范围（度数）
lowerBound[1] = -100    ; A1轴下限
upperBound[1] =  100    ; A1轴上限
; ... 对所有6轴设置类似的界限 ...

; 移动到起始位置
PTP $AXIS_ACT
PTP HOME

; ============ 创建RSI对象 ============
containerID = 0
err = ST_ETHERNET(hEthernet, containerID, configFileName[])
IF (err <> #RSIOK) THEN
  HALT
ENDIF

; 创建轴修正对象
err = ST_AXISCORR(hAxis, containerID)
IF (err <> #RSIOK) THEN
  HALT
ENDIF

; 链接从XML文件配置的轴修正对象
for i=1 TO numAxes
  err = ST_NEWLINK(hEthernet, i, hAxis, i)
  IF err<>#RSIOK THEN
    HALT
  ENDIF
ENDFOR

; 设置集成模式 (0=绝对, 1=相对)
err = ST_SETPARAM(hAxis, 1, 0)
IF err<>#RSIOK THEN
  HALT
ENDIF

; 设置下界和上界
for i=2 TO numAxes+1
  err = ST_SETPARAM(hAxis, i, lowerBound[i-1])
  IF err<>#RSIOK THEN
    HALT
  ENDIF
ENDFOR

for i=14 TO numAxes+13
  err = ST_SETPARAM(hAxis, i, upperBound[i-13])
  IF err<>#RSIOK THEN
    HALT
  ENDIF
ENDFOR

; ============ 启动RSI执行 ============
err = ST_ON()
IF (err <> #RSIOK) THEN
  HALT
ENDIF

; 运行RSI (直到RSI中断)
ST_SKIPSENS()

; 关闭RSI
err = ST_OFF()
IF (err <> #RSIOK) THEN
  HALT
ENDIF
```

---

### 模式2：同时执行预定路径和RSI修正

#### 方案A：在PTP/LIN中使用RSI（概念方案）

关键理解：
1. **RSI_MOVECORR()** 是**阻塞函数**，持续运行直到RSI关闭
2. 要在**运动的同时**接收实时修正，必须：
   - 让 `RSI_MOVECORR()` 运行（在PTP/LIN的某个时间点）
   - 外部系统（UDP）发送修正值
   - 机械臂在执行原始轨迹的同时累加修正

```krl
; 预定路径示例
DECL AXIS target_pos1, target_pos2, target_pos3

DEF PrePlannedPathWithRSI()
  DECL INT ret
  DECL INT CONTID

  ; 初始化RSI（同上）
  ret = RSI_CREATE("ros_rsi.rsi", CONTID, TRUE)
  IF (ret <> RSIOK) THEN
    HALT
  ENDIF

  ret = RSI_ON(#RELATIVE)  ; 相对模式（推荐用于修正）
  IF (ret <> RSIOK) THEN
    HALT
  ENDIF

  ; 关键：在WHILE循环中运行RSI_MOVECORR()
  ; 这允许外部系统在整个循环期间持续修正
  WHILE TRUE
    RSI_MOVECORR()  ; 在此期间接收和应用修正
  ENDWHILE

  ret = RSI_OFF()
  IF (ret <> RSIOK) THEN
    HALT
  ENDIF

END
```

#### 方案B：并行任务架构（高级）

理想的"执行预定路径同时接收RSI修正"需要两个**并行任务**：
- **任务1**：执行预定的PTP/LIN运动序列
- **任务2**：运行 `RSI_MOVECORR()` 持续监听修正

```krl
; 主程序
DEF MainWithParallelTasks()
  DECL INT ret
  DECL INT CONTID

  ; 初始化RSI
  ret = RSI_CREATE("ros_rsi.rsi", CONTID, TRUE)
  IF (ret <> RSIOK) THEN
    HALT
  ENDIF

  ret = RSI_ON(#RELATIVE)
  IF (ret <> RSIOK) THEN
    HALT
  ENDIF

  ; 启动并行任务执行轨迹
  ; (在实际KUKA系统中使用BG命令或并行处理)
  CALL ExecuteTrajectory()
  
  ; 同时处理RSI修正
  WHILE TRUE
    RSI_MOVECORR()
  ENDWHILE

  ret = RSI_OFF()
  IF (ret <> RSIOK) THEN
    HALT
  ENDIF
END

; 执行预定轨迹的独立程序
DEF ExecuteTrajectory()
  PTP target_pos1
  PTP target_pos2
  PTP target_pos3
  ; ... 执行完整轨迹
END
```

---

### 模式3：插值轨迹和修正

关键概念：
- RSI 工作在 **4ms 周期** (250Hz)
- 每个周期：
  1. 机械臂发送当前状态 (XML格式)
  2. 外部系统接收状态 + 发送修正值 (RKorr: X, Y, Z, A, B, C)
  3. 机械臂应用修正并继续运动

#### 实现轨迹插值修正的Python示例（来自你的工作区）

```python
# 这是你的 02_RSI_move.py 的核心模式

import socket
import xml.etree.ElementTree as ET

HOST = '0.0.0.0'
PORT = 59152
RSI_CYCLE_TIME = 0.004  # 4ms

def run_rsi_motion_corrector():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, PORT))
    
    accumulated_dist = 0.0
    TARGET_DISTANCE = 50  # mm
    MOVE_SPEED = 1        # mm per cycle
    
    while True:
        # 1. 接收来自KRL的状态信息
        data, addr = sock.recvfrom(2048)
        root = ET.fromstring(data)
        
        ipoc = root.find('IPOC').text
        rist = root.find('RIst')
        current_z = float(rist.get('Z'))
        
        # 2. 计算修正值（插值）
        delta_z = 0.0
        if accumulated_dist < TARGET_DISTANCE:
            delta_z = MOVE_SPEED * RSI_CYCLE_TIME
            accumulated_dist += delta_z
        
        # 3. 发送修正（RKorr）
        reply_xml = f'''<Sen Type="ImFree">
            <RKorr X="0.0" Y="0.0" Z="{delta_z:.4f}" 
                   A="0.0" B="0.0" C="0.0" />
            <IPOC>{ipoc}</IPOC>
        </Sen>'''
        
        sock.sendto(reply_xml.encode(), addr)
```

---

## 📋 RSI_MOVECORR() 的正确使用方式

### 基本特性

| 特性 | 说明 |
|------|------|
| **类型** | 阻塞函数（会持续运行） |
| **周期** | 4ms (250Hz) |
| **运行模式** | 绝对 (`#ABSOLUTE`) 或 相对 (`#RELATIVE`) |
| **数据格式** | XML (UDP) |
| **修正范围** | 在XML配置中定义的界限内 |

### 执行流程

```
RSI_MOVECORR() 执行流程：

1. KRL 调用 RSI_MOVECORR()
    ↓
2. 机械臂每 4ms 发送 XML 数据包：
   <Abb Type="RIst">
     <IPOC>123</IPOC>
     <RIst A1="45.2" A2="-90.1" ... />  （当前位置）
   </Abb>
    ↓
3. 等待外部系统响应 (最多 10ms)
    ↓
4. 接收外部系统的修正值 XML：
   <Sen Type="ImFree">
     <RKorr X="1.0" Y="0.5" Z="2.0" A="0.0" B="0.0" C="0.0" />
     <IPOC>123</IPOC>  （必须匹配）
   </Sen>
    ↓
5. 应用修正并继续运动
    ↓
6. 重复步骤 2-5 直到 RSI_OFF()
```

### 关键配置参数（XML）

从 `ros_rsi.rsi.xml` 中的关键参数：

```xml
<!-- 超时设置 -->
<Timeout>100</Timeout>  <!-- 100 个周期 = 400ms -->
<!-- 如果响应延迟超过 4ms 且计数达到 100，RSI 停止 -->

<!-- 轴修正限制 -->
<RSIObject NAME="AXISCORR">
  <LowerLimA1>-100</LowerLimA1>  <!-- A1轴下限 (度) -->
  <UpperLimA1>100</UpperLimA1>   <!-- A1轴上限 (度) -->
  <!-- 类似的定义用于其他轴 -->
</RSIObject>
```

---

## ⚙️ 网络配置关键点

### RSI 网络需求

| 方面 | 要求 |
|------|------|
| **协议** | UDP (Ethernet) |
| **机器人端口** | 通常 49152 |
| **通信周期** | 4ms (不可配置) |
| **最大响应时间** | ~10ms (否则计数超时) |
| **实时性要求** | 高 (建议使用 RT-Preempt 内核) |

### 网络诊断

```bash
# 测试机器人连接
ping 192.168.1.20  # 机器人 IP

# 检查网络配置（在KRL中）
# 运行 RSI-Network 工具查看配置
```

---

## 🚀 实现策略总结

### 策略 1: 简单的实时修正（推荐初级）
```
KRL: RSI_MOVECORR()
Python: 实时接收 → 计算修正 → 发送
特点: 简单、可靠、低延迟
```

### 策略 2: 轨迹插值修正（推荐中级）
```
KRL: RSI_MOVECORR() 在 WHILE 循环中
Python: 基于时间的插值器 + RSI 修正
特点: 平滑、可预测、支持复杂轨迹
```

### 策略 3: 完整的并行任务架构（高级）
```
Task 1: 预定轨迹执行 (PTP/LIN)
Task 2: RSI_MOVECORR() 在后台运行
Python: 高级控制系统
特点: 最大灵活性、最复杂
```

---

## 🔧 实际问题解决方案

### 问题 1: RSI 连接断开
**症状**: "SEN: RSI execution error - RSI stopped"

**原因**: 响应延迟超过 10ms

**解决方案**:
1. 编译安装 RT-Preempt 内核
2. 给 `kuka_rsi_hardware_interface` 分配实时优先级
3. 减少 XML 中的 Timeout 参数（增加检查频率）

### 问题 2: 修正不被应用
**症状**: 机械臂忽视 RKorr 值

**原因**: 
- IPOC 不匹配
- 修正超出范围
- 集成模式错误

**解决方案**:
```krl
; 确保IPOC匹配
; 确保修正值在界限内
; 检查集成模式 (0=绝对, 1=相对)
err = ST_SETPARAM(hAxis, 1, 1)  ; 设为相对模式
```

### 问题 3: 轨迹跳跃/不连续
**原因**: 没有正确同步修正和轨迹执行

**解决方案**:
- 确保修正值平滑变化
- 使用相对模式 (`#RELATIVE`) 而不是绝对模式
- 在 WHILE 循环中持续运行 `RSI_MOVECORR()`

---

## 📖 深度学习资源

### 推荐阅读顺序
1. **入门**: ROS-Industrial KR_C2/KR_C4 README
2. **中级**: 论文 "Comparison of KVP and RSI for controlling KUKA robots over ROS"
3. **高级**: "Implementation of dynamic trajectory control for an industrial robot by external dSPACE"

### KUKA 官方文档（需要自行获取）
- KUKA.RobotSensorInterface 编程手册
- KUKA.Ethernet RSI XML 参考
- KRL 参考手册 (函数: RSI_CREATE, RSI_ON, RSI_OFF, RSI_MOVECORR)

---

## 💡 关键洞察

### "执行预定路径同时接收RSI修正"的关键

**核心问题**: RSI_MOVECORR() 是阻塞函数

**解决方案**:
1. **让 RSI_MOVECORR() 在后台持续运行**
   - 这是你在工作区看到的 `WHILE TRUE` 模式
   
2. **修正是**增量式的**，不是绝对的**
   - 使用 `#RELATIVE` 模式
   - 每个 RKorr 是相对于当前运动的偏移
   
3. **轨迹插值必须在外部系统中**
   - KRL 中的 RSI_MOVECORR() 只是循环等待修正
   - Python/ROS 系统计算轨迹和修正值
   - KRL 应用这些修正继续运动

### 典型工作流程示例

```
1. Python 系统: 计算目标轨迹 (1000个点，100ms)
2. Python 系统: 计算每 4ms 的修正值
3. KRL 启动: RSI_MOVECORR()
4. KRL 循环: 
   - 接收来自Python的修正值
   - 应用修正
   - 继续运动（自动）
5. Python 继续: 计算下一个轨迹段的修正
6. 轨迹完成后: KRL 调用 RSI_OFF()
```

---

## 📌 快速参考

### KRL 函数速查

| 函数 | 参数 | 作用 |
|------|------|------|
| `RSI_CREATE()` | (filename, CONTID, TRUE) | 创建RSI上下文 |
| `RSI_ON()` | (#ABSOLUTE/#RELATIVE) | 启动RSI |
| `RSI_MOVECORR()` | (无) | 运行修正循环（阻塞） |
| `RSI_OFF()` | (无) | 关闭RSI |
| `ST_ETHERNET()` | (handle, containerID, configFile) | 创建以太网对象（KR-C2） |
| `ST_AXISCORR()` | (handle, containerID) | 创建轴修正对象 |
| `ST_SETPARAM()` | (handle, param_index, value) | 设置参数 |

### Python 数据格式速查

```xml
<!-- 机械臂发送 (Abb) -->
<Abb Type="RIst">
  <IPOC>123</IPOC>
  <RIst A1="45.0" A2="-90.0" A3="90.0" A4="0.0" A5="90.0" A6="0.0" />
</Abb>

<!-- 外部系统发送 (Sen) -->
<Sen Type="ImFree">
  <RKorr X="1.0" Y="0.5" Z="2.0" A="0.0" B="0.0" C="0.0" />
  <IPOC>123</IPOC>
</Sen>
```

---

## 结论

KUKA RSI 的核心是:
1. **阻塞式的 `RSI_MOVECORR()` 函数**在 KRL 中运行
2. **高频 UDP 通信** (4ms 周期) 用于修正值交换
3. **外部系统** (Python/ROS) 计算轨迹和修正
4. **修正应用于当前运动**，实现轨迹跟踪

通过组合这些要素，可以实现复杂的实时运动控制，包括：
- 传感器引导的运动
- 力控制
- 动态轨迹修正
- 与外部系统的实时同步

