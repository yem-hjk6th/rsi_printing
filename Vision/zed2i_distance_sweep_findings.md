# ZED2i 点云质量 vs 采集距离 实验记录

## 实验目的
用白纸平面作为参考目标，系统测定 ZED2i 在不同工作距离下的点云精度，
为后续重建任务选定最优工作区间。

## 使用脚本
`Vision/zed_prin_exp_demo/demo9_distance_sweep.py`

## 核心设置
```python
DISTANCES_MM  = [500, 800, 1200, 1500]   # 名义距离 mm
N_FRAMES      = 15                        # 每个距离采 15 帧取均值
LOCK_WINDOW   = 20                        # 滑窗帧数（自动锁定用）
LOCK_TOL_MM   = 40                        # 锁定容差 ±40mm
LOCK_STD_MM   = 15                        # 锁定稳定性阈值 std<15mm
DEPTH_MODE    = ULTRA                     # ZED SDK 深度模式
RESOLUTION    = HD2K (2208×1242)
```

## 实验结果（2026-05-05，白纸背景，有效ROI）
结果目录：`Vision/vision_demo_test_res/distance_sweep_20260505_131352/`

| Nominal Z | Actual Z | valid_ratio | depth_noise | pts/cm² | plane_RMSE |
|-----------|----------|-------------|-------------|---------|------------|
| 500 mm    | 489 mm   | 99.7%       | 78.7 mm     | 1514    | **46.1 mm** ❌ |
| 800 mm    | 783 mm   | 100.0%      | 82.1 mm     | 594     | **2.3 mm** ✅ |
| 1200 mm   | 1213 mm  | 100.0%      | 107.9 mm    | 247     | **6.7 mm** ✅ |
| 1500 mm   | 1488 mm  | 100.0%      | 141.9 mm    | 164     | **7.9 mm** ✅ |

## 关键 Takeaways

1. **最优工作距离：800mm**（plane_RMSE=2.3mm，点云密度合理）
2. **可用区间：800–1200mm**（RMSE均<10mm）
3. **500mm 不适合精度重建**（RMSE=46mm，near-range disparity严重压缩）
4. `depth_noise_mm` 不等于传感器噪声——反映 ROI 内深度梯度与外点，应用 `plane_RMSE` 作为精度代理
5. 点云密度遵循 $\rho \propto 1/z^2$：500mm→1514 pts/cm²，1500mm→164 pts/cm²
6. `valid_ratio` 在 800mm 以上接近 100%（白纸平面 + 充足 stereo overlap）

## 实验设计注意事项（控制变量）
- **必须设 ROI**：框选平面参考目标，否则 depth_noise 和 plane_RMSE 反映的是场景杂乱度
- **固定目标**：白纸贴墙/放平，每次对准同一平面
- **actual_z 应在 ±40mm 内**：否则数据点应作废
- `depth_noise_mm` 当前算法仍是全 ROI std，待改为 MAD（robust sigma）才能真正反映测量噪声

## 相机参数（SN 37529394）
- fx = 1907.9 px，baseline = 120.1 mm（HD2K）
- `d_max = fx * B / Z`：Z=500mm 时 d_max=457px（占图宽21%），Z=800mm 时 d_max=286px（13%）

## 后续 TODO
见 `Vision/reconstruction_todo.md`：
- ICP 多视角配准融合
- TSDF Fusion（利用手眼标定位姿）
- NeRF / 3DGS（需独立环境）
- `depth_noise_mm` 改为 MAD-based robust sigma
