# KUKA RSI 快速参考卡片

## 🎯 核心概念速览

### RSI_MOVECORR() 的关键特性
```
┌─────────────────────────────────────────────────────────────┐
│ RSI_MOVECORR() 的工作原理                                    │
├─────────────────────────────────────────────────────────────┤
│ 1. 每 4ms 发送机械臂状态 (XML格式)                           │
│ 2. 等待外部系统的修正值 (最多 10ms)                          │
│ 3. 应用修正并继续运动                                        │
│ 4. 重复，直到 RSI_OFF()                                      │
│                                                              │
│ 特点: 阻塞函数 + 实时性 + 高频率 (250Hz)                    │
└─────────────────────────────────────────────────────────────┘
```

---

## 📊 时序图

```
KRL 端                          Python 端
─────────────────────────────────────────────────
RSI_MOVECORR()
  │
  ├─> 发送数据包 ─────────────────────> 接收
  │   (XML) 
  │   IPOC: 123                     解析状态
  │                                  │
  │                            计算修正值
  │                            Correction: 
  │                            x=1.0, y=0.5
  │                                  │
  │   接收 <─────────────────── 发送修正
  │   (XML)                    (XML)
  │   <RKorr/>                 IPOC: 123
  │   应用修正
  │   继续运动
  │
  ├─> 发送数据包 ─────────────────────> 接收
  │   (XML)
  │   IPOC: 124
  │   [循环继续...]
  │
```

---

## 🔧 快速故障排除

| 问题 | 症状 | 原因 | 解决方案 |
|------|------|------|---------|
| **连接失败** | 机械臂无法连接 | IP/端口错误 | 检查 `ros_rsi_ethernet.xml` |
| **RSI 停止** | "RSI execution error - RSI stopped" | 响应延迟 > 10ms | 使用 RT-Preempt 内核 |
| **修正不生效** | 机械臂忽视修正值 | IPOC不匹配 | 确保IPOC一致 |
| **轨迹跳跃** | 运动不连续 | 修正值太大 | 减小修正增益或范围 |

---

## 📝 配置检查清单

### KRL 端
- [ ] 创建 RSI 上下文 (`RSI_CREATE`)
- [ ] 启动 RSI (`RSI_ON` with `#RELATIVE`)
- [ ] 使用 `WHILE TRUE` 循环运行 `RSI_MOVECORR()`
- [ ] 配置文件在正确位置 (`ros_rsi.rsi`)
- [ ] 轴修正范围合理 (±100度左右)

### Python 端
- [ ] UDP 套接字绑定正确的 IP:PORT
- [ ] 能成功解析 XML 数据包
- [ ] IPOC 在回复中正确匹配
- [ ] 修正值在界限内
- [ ] 响应时间 < 10ms

### 网络配置
- [ ] 机械臂和PC在同一子网
- [ ] Ping 连接正常
- [ ] 防火墙允许 UDP 端口 49152
- [ ] 不使用 192.0.1.x 地址范围

---

## 💻 代码片段集锦

### KRL: 最小化示例
```krl
DEF rsi_min()
    RSI_CREATE("config.rsi", CONTID, TRUE)
    RSI_ON(#RELATIVE)
    WHILE TRUE
        RSI_MOVECORR()
    ENDWHILE
    RSI_OFF()
END
```

### Python: 最小化示例
```python
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 49152))

while True:
    data, addr = sock.recvfrom(2048)
    root = ET.fromstring(data)
    ipoc = root.find('IPOC').text
    
    reply = f'<Sen Type="ImFree"><RKorr X="1.0" Y="0.0" Z="0.0" A="0.0" B="0.0" C="0.0" /><IPOC>{ipoc}</IPOC></Sen>'
    sock.sendto(reply.encode(), addr)
```

---

## 📈 性能优化

### 减少延迟
```python
# 使用非阻塞 I/O
sock.setblocking(False)

# 预分配缓冲区
buffer = bytearray(2048)

# 使用线程池
from concurrent.futures import ThreadPoolExecutor
```

### 改进实时性
```bash
# Linux: 使用 RT-Preempt 内核
$ uname -r
# 应该包含 "rt"

# 提升进程优先级
$ chrt -f 50 python rsi_controller.py
```

### 优化轨迹计算
```python
# 预先计算轨迹而不是实时计算
trajectory = pre_compute_trajectory(...)

# 使用查表而不是复杂计算
lookup_table = precompute_corrections(...)
```

---

## 🔗 关键文件位置

### 机械臂端
```
C:\KRC\ROBOTER\KRC\R1\Program
  └─ ros_rsi.src          (主程序)

C:\KRC\ROBOTER\Init
  └─ ros_rsi_ethernet.xml (网络配置)

C:\KRC\ROBOTER\Config\User\Common\SensorInterface
  └─ ros_rsi.rsi          (RSI配置)
  └─ ros_rsi.rsi.xml      (RSI参数)
```

### PC 端 (ROS)
```
kuka_rsi_hw_interface/
  ├─ config/
  │   ├─ controller_joint_names.yaml
  │   └─ hardware_controllers.yaml
  ├─ krl/
  │   ├─ KR_C2/ros_rsi.src
  │   └─ KR_C4/ros_rsi.src
  └─ src/
      └─ kuka_rsi_hardware_interface.cpp
```

---

## 🚀 执行步骤

### 1. 准备阶段
```
1. 配置机械臂网络和RSI文件
2. 启动 ROS 节点 (kuka_rsi_hw_interface)
3. 创建并启动 Python 控制脚本
```

### 2. 启动阶段
```
1. 在教示器中选择 T1 模式
2. 加载 ros_rsi.src 程序
3. 按下启动按钮 - 机械臂移动到起始位置
4. 再次按下 - 开始 RSI 修正
5. 终端应显示 "Got connection from robot"
```

### 3. 监控阶段
```
1. 观察轨迹执行
2. 检查延迟和准确性
3. 监控错误和警告
```

### 4. 停止阶段
```
1. 发送 RSI_OFF() 信号
2. Python 脚本自动退出
3. 机械臂返回安全位置
```

---

## 📚 推荐学习路线

### 初级 (1-2天)
```
1. 读: README (KR_C2 or KR_C4)
2. 理解: RSI_MOVECORR() 的工作原理
3. 运行: Python 最小示例
4. 测试: 基本连接和修正
```

### 中级 (2-5天)
```
1. 学习: 论文 "Comparison of KVP and RSI"
2. 实现: 轨迹插值系统
3. 测试: 复杂运动（圆形、多段）
4. 优化: 减少延迟，提高精度
```

### 高级 (1-2周)
```
1. 研究: "Dynamic trajectory control" 论文
2. 实现: 力控制/传感器融合
3. 集成: 与 MoveIt! 或其他规划器
4. 验证: 性能测试和安全评估
```

---

## 🎓 常见问题

### Q1: 为什么要用 WHILE TRUE 循环？
**A**: RSI_MOVECORR() 是阻塞函数，持续等待修正。WHILE 循环确保连续的实时修正。

### Q2: 能否同时执行 PTP 和 RSI_MOVECORR()？
**A**: 不能直接。需要：
- 方案A: 在 RSI_MOVECORR() 中通过修正值实现轨迹
- 方案B: 使用并行任务（高级）

### Q3: 修正值的范围是多少？
**A**: 在 XML 中定义，通常 ±100 度（关节空间）或 ±100mm（笛卡尔空间）。

### Q4: 如何调整响应灵敏度？
**A**: 修改控制增益（Python中的 kp 参数）或在 XML 中调整界限。

### Q5: 最大可实现的精度是多少？
**A**: 理论上取决于：
- 机械臂精度（±0.1mm）
- 传感器精度
- 网络延迟 (< 4ms)
- 实时操作系统支持

通常可达 ±1-5mm 的实时修正精度。

---

## 📞 技术支持资源

| 资源 | URL | 备注 |
|------|-----|------|
| ROS-Industrial KUKA | https://github.com/ros-industrial/kuka_experimental | 官方代码 |
| KUKA 官方支持 | https://www.kuka.com | 技术文档 |
| Google Scholar | https://scholar.google.com | 学术论文 |
| ResearchGate | https://www.researchgate.net | 研究社区 |

---

## 🔐 安全提示

⚠️ **使用 RSI 时的安全注意事项**:

1. **始终在 T1 模式下测试** - 限制速度
2. **确保紧急停止按钮可达** - 准备好中断
3. **验证修正范围** - 防止异常运动
4. **使用超时保护** - 网络故障时停止
5. **监控机械臂温度** - 连续运行可能过热
6. **定期检查网络连接** - 防止通信中断
7. **备份所有配置** - USB 驱动器保存

---

## 📊 性能基准

### 典型数值

| 参数 | 值 | 单位 |
|------|-----|------|
| RSI 周期 | 4 | ms |
| RSI 频率 | 250 | Hz |
| 最大响应时间 | 10 | ms |
| 最大延迟 (缓冲) | 100 | 周期 |
| 最大超时 | 400 | ms |
| 网络带宽 | ~1 | Mbps |
| 数据包大小 | ~200 | 字节 |

### 性能优化目标

```
优先级 1: 网络延迟 < 4ms (最关键)
优先级 2: 修正计算 < 2ms
优先级 3: 轨迹平滑性 (加速度 < 1000°/s²)
优先级 4: 内存使用 (< 100MB)
```

---

## 版本信息

- **KUKA RSI 版本**: 3.0+
- **KRL 版本**: RSI_CREATE/RSI_ON/RSI_MOVECORR
- **Python 版本**: 3.6+
- **ROS 版本**: Kinetic+ (melodic-devel 分支)

---

*最后更新: 2024年 | 基于 ROS-Industrial KUKA 实验包*

