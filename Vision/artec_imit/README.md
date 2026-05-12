# artec_imit — 使用说明

## 环境准备

```bash
cd C:\Users\888y9\Desktop\rsi_printing
conda activate zedenv
```

---

## Step 1 — 采集（需要 ZED 相机在线）

```bash
# 手动模式：SPACE 触发快门，每帧弹出 ROI 框选对话框
python Vision/artec_imit/pipeline.py capture

# 自动模式：运动量超阈值自动触发，自动深度平面估计 ROI
python Vision/artec_imit/pipeline.py capture --mode auto

# 指定输出目录（默认自动生成时间戳目录）
python Vision/artec_imit/pipeline.py capture --out Vision/vision_demo_test_res/my_session
```

**手动模式操作键：**
| 键 | 功能 |
|----|------|
| `SPACE` | 采集当前帧 |
| `u` | 撤销上一帧 |
| `q` | 结束采集 |

采集完成后，输出路径打印在终端，格式如：
```
vision_demo_test_res/imit_20260506_153042/
```

---

## Step 2 — 离线重建（不需要相机）

```bash
# Pose Graph 方式（默认，较快）
python Vision/artec_imit/pipeline.py recon "Vision/vision_demo_test_res/imit_20260506_153042"

# Frame-to-model 方式（更准，但慢）
python Vision/artec_imit/pipeline.py recon "Vision/vision_demo_test_res/imit_20260506_153042" --ftm
```

输出写入 `<data_dir>/artec_recon/`：
- `mesh.ply` — 后处理网格，用 MeshLab / CloudCompare 打开
- `pcd.ply` — 点云
- `recon_log.txt` — 每帧配准 fitness & RMSE

---

## 关键参数（在 config.py 中修改）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `DEPTH_MAX_M` | 1.2 m | 工作距离 ~800mm，超出截掉 |
| `AUTO_TRANS_M` | 40 mm | 自动触发平移阈值 |
| `AUTO_ROT_DEG` | 8° | 自动触发旋转阈值 |
| `FTM_WARMUP_FRAMES` | 3 | FTM 热身帧数 |
| `TSDF_VOXEL` | 1 mm | TSDF 体素分辨率 |
