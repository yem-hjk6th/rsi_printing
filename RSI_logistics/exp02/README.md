# RSI Async Keyboard Control - exp02

## 概述
单文件异步键盘控制实验，使用非阻塞输入实现实时运动控制。

## 架构特点
- **单进程单文件**：简化调试和部署
- **非阻塞输入**：使用 `msvcrt.kbhit()` 不影响RSI 4ms通信周期
- **步进控制**：按键触发固定增量运动
- **安全限位**：软件限制在±10mm范围内

## 配置参数

### 运动参数
```python
STEP_SIZE = 3.0         # 每次按键移动3mm
MAX_POSITIVE = 10.0     # X轴正向限位
MAX_NEGATIVE = -10.0    # X轴负向限位
```

### 通信参数
```python
HOST = '0.0.0.0'
PORT = 59152
RSI_CYCLE_TIME = 0.004  # 4ms周期
```

## 键盘控制

| 按键 | 功能 | 说明 |
|------|------|------|
| `m` | X +3mm | 正向移动，累积不超过+10mm |
| `n` | X -3mm | 负向移动，累积不超过-10mm |
| `r` | 重置归零 | 将目标位置重置为0 |
| `q` | 退出程序 | 安全关闭Socket |

## 使用方法

### 1. 启动服务器
```bash
cd RSI_logistics/exp02
python rsi_async_keyboard.py
```

### 2. 配合RSI文件
使用 `RSI_set_ver/Mine/ver5_10mm_var/` 中的配置文件：
- `RSI_MIN.rsi.xml` - 限位±10mm
- `RSI_EthernetConfig.xml` - HOLDON=1累积模式

### 3. 实时操作
程序运行后：
1. 等待RSI连接显示 `[CONNECTED]`
2. 按 `m` 增加X轴位置
3. 按 `n` 减少X轴位置
4. 观察 `Real_X` 和 `Target_X` 的变化
5. 按 `q` 或 `Ctrl+C` 退出

## 输出示例
```
RSI Async Keyboard Server started on 0.0.0.0:59152
STEP_SIZE: 3.0mm | X_LIMIT: [-10.0, 10.0]mm

[CONNECTED] Robot controller at ('10.100.1.1', 49152)
--------------------------------------------------------------------------------
Time     | IPOC   | OV  | Vel   | Real_X   | Target_X  | Delta_X
--------------------------------------------------------------------------------
T:  5.23 | 1308   | 100 | 0     |    0.00 |    +0.00 |  +0.0000

[CMD] +3.0mm → X_Target: +3.00mm
T:  6.45 | 1613   | 100 | 0     |    2.98 |    +3.00 |  +0.0000

[CMD] +3.0mm → X_Target: +6.00mm
T:  8.12 | 2030   | 100 | 0     |    5.97 |    +6.00 |  +0.0000
```

## 核心逻辑

### 差分计算（HOLDON=1模式）
```python
# 每个周期只发送增量差值
delta_x = korr_x_target - korr_x_prev
korr_x_prev = korr_x_target

# RSI会累积所有delta：实际位置 = Σ(delta)
```

### 限位保护
```python
if new_target <= MAX_POSITIVE:
    korr_x_target = new_target
else:
    print("Cannot exceed limit")
```

## 技术细节

### 非阻塞架构
```
主循环 (4ms周期)
├─ msvcrt.kbhit() 检查键盘 (微秒级)
├─ sock.recvfrom() 接收RSI (超时10ms)
├─ 计算delta并发送
└─ 无任何阻塞等待
```

### 与03_async的区别
| 特性 | exp02 | 03_async |
|------|-------|----------|
| 线程数 | 1 (主线程) | 3 (通信/触发/输入) |
| 触发方式 | 键盘即时 | 随机定时 |
| 锁机制 | 无需 (GIL) | threading.Lock |
| 复杂度 | ⭐ 简单 | ⭐⭐⭐ 复杂 |
| 适用场景 | 手动测试 | 自动化测试 |

## 注意事项

1. **Windows专用**：使用 `msvcrt` 仅支持Windows系统
2. **终端窗口激活**：按键时需要终端窗口保持焦点
3. **限位检查**：累积值超过±10mm时拒绝移动并提示
4. **连接断开**：RSI超时后自动等待重连，无需重启程序

## 扩展建议

### 增加功能
- 添加 Y/Z 轴控制（'i'/'k' for Z轴）
- 可变步长（按住Shift增加到5mm）
- 记录位置历史到CSV

### 远程控制
如需网络远程控制，可在主循环增加UDP监听线程：
```python
# 监听49152端口接收网络命令
ctrl_sock.bind(('127.0.0.1', 49152))
data, _ = ctrl_sock.recvfrom(64)
```

## 故障排查

### 问题1：按键无响应
- 检查终端窗口是否激活
- 确认没有其他程序占用按键

### 问题2：超过限位
- 检查 `korr_x_target` 是否正确重置
- 验证 RSI 配置中的 `UpperLimX/LowerLimX`

### 问题3：连接超时
- 确认机器人控制器IP配置
- 检查防火墙是否阻止59152端口
