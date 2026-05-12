# Experiment 01: Two Point Motion + RSI Correction Architecture

## 概述

这是方案A（LIN运动+RSI修正）的完整实验实现。

**核心目标**：验证src定义的往复运动可以被RSI平滑修正，而不会产生控制权争抢或机械臂急停。

---

## 文件说明

```
exp01/
├── exp01_two_point_motion.src           # KRL源代码
├── exp01_two_point_motion.rsi           # RSI配置文件（结构定义）
├── exp01_two_point_motion_Config.xml    # 网络配置文件（IP/端口）
├── exp01_two_point_motion.py            # Python服务器
└── README.md                             # 本文件
```

---

## 运行步骤

### 第1步：准备环境

1. **在KUKA机器人KOP中：**
   - 将 `exp01_two_point_motion.src` 复制到机器人的 `src/` 目录
   - 将 `exp01_two_point_motion.rsi` 复制到机器人的配置目录
   - 将 `exp01_two_point_motion_Config.xml` 放在同一配置目录

2. **修改网络配置：**
   - 编辑 `exp01_two_point_motion_Config.xml`
   - 将 `<IP>192.168.1.100</IP>` 改为**实际PC的IP地址**
   - 确保机器人能ping通该PC

### 第2步：启动Python服务器

```bash
cd c:\Users\dell\Desktop\RSI\RSI_logistics\exp01
python exp01_two_point_motion.py
```

输出应该显示：
```
============================================================
  RSI Experiment 01: Two Point Motion + Correction
============================================================
  Mode: PHASE_1
  Listening on 0.0.0.0:59152
  ...
```

### 第3步：在KUKA机器人中执行程序

在KOP中：
1. 打开程序编辑器
2. 加载 `exp01_two_point_motion.src`
3. 点击"启动"或按 `Start` 按钮
4. 机器人应该开始两点往复运动

### 第4步：观察输出

Python窗口会实时打印：
```
Time   | IPOC  | OV  | Vel_A | Z_Ist   | Z_Sol   | dZ_Sol  | Send_Z   | Frame
------+-------+-----+-------+---------+---------+---------+----------+------
T: 0.50 | 25    | 100 | 50    |   0.25  |  25.50  |  25.25  | 0.0000   | 125  
T: 1.50 | 50    | 100 | 50    |  50.50  |  50.00  |  -0.50  | 0.0000   | 375
```

---

## 实验阶段说明

### Phase 1: 零修正（默认）

```python
CORRECTION_MODE = "PHASE_1"
```

**目的**：验证src的基础运动是否正常

**预期现象**：
- 机器人执行两点往复运动（Z轴 +100mm → -100mm）
- 往复5次
- Python接收数据并实时打印，但不做任何修正（Send_Z = 0）

**检查项**：
- ✓ Z_Ist是否从0逐步增加到100
- ✓ Z_Sol是否显示期望轨迹
- ✓ 整个过程是否流畅（无急停告警）

---

### Phase 2: 微小持续修正

```python
CORRECTION_MODE = "PHASE_2"
```

**目的**：验证RSI的修正机制是否工作

**修正逻辑**：
```python
Send_Z = ±2mm * sin(2π * 0.5Hz * t)  # 频率0.5Hz，幅度±2mm
```

**预期现象**：
- 机器人Z轴轻微摆动（±2mm）
- 振荡频率 = 0.5Hz（2秒一个完整周期）
- 不应有任何告警

**检查项**：
- ✓ Send_Z是否在±2mm范围内变化
- ✓ 机器人是否跟随修正（Z_Ist有相应偏移）

---

### Phase 3: 平滑正弦修正

```python
CORRECTION_MODE = "PHASE_3"
```

**目的**：验证平滑修正不会触发扭矩超限

**修正逻辑**：
```python
ramp_factor = min(elapsed / 3.0, 1.0)      # 前3秒斜坡增幅
Send_Z = ±5mm * ramp_factor * sin(...)     # 逐步增加到±5mm
```

**预期现象**：
- 前3秒：修正值从0平滑增加到±5mm（无突变）
- 后续：持续±5mm正弦摆动
- 整个过程应无任何告警

**检查项**：
- ✓ Send_Z是否平滑变化（无跳跃）
- ✓ 是否有扭矩/电流告警
- ✓ 机器人反应是否自然

---

### Phase 4: 突发干预

```python
CORRECTION_MODE = "PHASE_4"
```

**目的**：演示"随时kick in"的能力

**修正逻辑**：
```python
if elapsed < 5.0:
    Send_Z = 0  # 前5秒无修正
else:
    Send_Z = ±3mm * sin(2π * 1.0Hz * ...)  # 5秒后突然加入修正
```

**预期现象**：
- 前5秒：正常往复（无修正）
- 第5秒开始：突然加入±3mm的快速振荡（频率1Hz）
- 机器人应平滑适应新的修正

**检查项**：
- ✓ 第5秒时是否有响应延迟
- ✓ 是否有尖刻的加速度变化
- ✓ 是否能干预原有的往复轨迹

---

## 安全注意事项

⚠️ **运行前必读**

1. **修正范围硬限制**：±10mm
   - Python代码中有安全检查：`korr_z = max(min(korr_z, 10.0), -10.0)`
   - 即使Python计算出>10mm，也会被截断

2. **网络超时**：100ms
   - 如果Python无法在100ms内响应，机器人自动安停
   - 确保PC网络连接稳定

3. **启动顺序**：
   - 必须先启动Python服务器（等待连接）
   - 再启动KUKA程序

4. **紧急停止**：
   - KUKA机器人上：按 `E-STOP` 按钮
   - Python：Ctrl+C 中断

---

## 故障排除

| 问题 | 原因 | 解决方案 |
|------|------|--------|
| Python显示"Timeout" | 机器人网络未连接 | 检查网络配置、IP地址 |
| KUKA显示"RSI_CREATE failed" | 配置文件路径错误 | 检查.rsi文件是否在正确目录 |
| 机器人执行时立即停止 | src语法错误 | 在KOP编辑器中验证.src文件 |
| 无法修改CORRECTION_MODE | Python已启动 | 先Ctrl+C停止，修改后重启 |

---

## 下一步

实验成功后，可以：

1. **exp02**：集成多个实验阶段，自动切换Mode
2. **exp03**：在Phase 3基础上，加入用户输入控制修正方向
3. **exp04**：对接真实的打印/焊接任务（基于坐标微调）

