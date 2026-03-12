# 2026-03-12 Bead Width 测量流水线开发记录

## 今日产出文件评估

| 文件 | 重要性 | 说明 |
|---|---|---|
| **sam2_bead_224.py** | ⭐⭐⭐ 核心 | SAM2 离线 bead 测宽 (v4), 含 GUM 不确定度, NPZ mask 导出. 是所有下游校准的 ground truth 来源 |
| **spatial_bead_width.py** | ⭐⭐⭐ 核心 | 按机械臂行进距离 1mm 间隔采样, Sobel Y 测宽 + 邻帧连续性验证. 最终生产用脚本的原型 |
| **calibrate_fast_methods.py** | ⭐⭐ 中间件 | Meijering / Sobel 网格搜索校准. 已完成使命 (产出最优参数), 后续不需要反复跑 |
| **eval_fast_methods.py** | ⭐ 参考 | 100 帧独立评估. 暴露了 Meijering 的不稳定问题, 结论已得到, 可归档 |

## 会话主线思路

### 1. SAM2 → GUM 不确定度 (替代假置信度)

之前 `compute_confidence` 是没有理论依据的启发式公式。本次替换为 GUM 标准:
- **CI95** = 1.96 × σ / √n (统计置信区间)
- **u_w** = w_mm × √((U_Z/Z)² + (U_PX/w_px)² + (U_FX/fx)²) (误差传播)
- 常量: U_Z=0.002m, U_PX=0.5px, U_FX=1.0px

### 2. SAM2 做离线 Ground Truth 生成

- 50 帧随机采样 (seed=42, range 800–4800)
- 每帧导出 `f{idx}_bead_mask.npz` (mask + roi_origin)
- **结果: 1.83 ± 0.25 mm** (50 帧)

### 3. 快速方法校准 (Meijering vs Sobel Y)

用 SAM2 GT 做参考, 网格搜索最优参数:

| 方法 | 最优参数 | MAE (px) | 速度 | 20Hz? |
|---|---|---|---|---|
| **Meijering** | sigmas=[1,2,3,4,5], pct=95 | 1.47 | 102.7ms | NO |
| **Sobel Y** | ksize=5, threshold=20 | 1.54 | 1.4ms | YES |

### 4. 100 帧独立评估 → 发现 Meijering 不可靠

| | Meijering | Sobel Y |
|---|---|---|
| 均值 | 1.68 ± 0.55 mm | 2.08 ± 0.29 mm |
| 相关系数 | 0.125 (极低) | — |

**关键发现**: 31/32 个争议帧 (|Δ| ≥ 0.7mm) 都是 Sobel >> Meijering. Meijering 在后段 (f3500+) 系统性偏低至 0.70mm, 因为 ridge detection 的 "最高 aspect ratio 连通域" 策略选中了错误结构.

### 5. Ground Truth 对照 (f4145)

用户实测 GT = 1.27mm:

| 方法 | 测量值 | 误差 |
|---|---|---|
| SAM2 | 1.73mm | +36% |
| Meijering | 0.70mm | −45% |
| Sobel Y | 2.79mm | +120% |

**三个方法都不准**, SAM2 最接近但仍偏高. 说明 SAM2 做的是 pseudo label 而非 absolute GT.

### 6. 按行进距离空间采样 (最终方案)

放弃随机帧思路, 转为按机械臂 RSI 轨迹的累积行进距离采样:
- RSI CSV → 累积距离 → 每 1mm 取对应 SVO 帧号
- Sobel Y 测宽 (ksize=5, threshold=20)
- 前后各 2 帧连续性比对 → flag: OK / JUMP / MISS

**结果 (1936 采样点, 全程 1935.6mm)**:
- Width: **2.14 ± 0.39 mm** (range 1.11–3.75)
- OK: 1199 (62%), JUMP: 737 (38%), MISS: 0
- 速度: 111ms/帧 (瓶颈在 SVO seek+解码, Sobel 本身 <2ms)

JUMP 比例高 (38%) 说明 0.5mm 门槛对 Sobel 的噪声来说太严.

## 关键输出路径

```
labeling/post_labeling/
├── t1/20260312_161035/          # SAM2 50帧 GT (CSV + 50 NPZ + 250 PNG)
├── t1/20260312_164025/          # SAM2 单帧 f4145
├── calibration/20260312_162116/ # Meijering/Sobel 校准 (4 CSV)
├── eval/20260312_163209/        # 100帧独立评估 (2 CSV)
└── spatial/20260312_170351/     # 空间采样 1936点 (1 CSV)
```

## 待解决问题

1. **Sobel 绝对精度不足**: 需要 bias correction (用少量实测 GT 线性校准)
2. **JUMP 门槛**: 0.5mm 太严, 需根据 Sobel 噪声水平调整 (建议 0.8–1.0mm)
3. **SVO 回放速度**: 111ms/帧主要是 seek 开销, 实时部署时直接读 live 流不会有此问题
4. **SAM2 也偏高 36%**: SAM2 的 mask 边界倾向于比实际 bead 更宽, 本质上只能做 pseudo label
