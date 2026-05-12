# artec_ffs — 使用说明

在 `artec_imit` 基础上增加 **Fast-FoundationStereo (FFS)** 深度优化，解决 ZED SGM 在低纹理/反光区域的离群点问题。

---

## 文件结构

```
artec_ffs/
  config.py        参数集中（含 FFS 权重路径）
  capture.py       采集（同时保存左图 + 右图，供 FFS 使用）
  ffs_depth.py     FFS 深度推理，替换 ZED depth.npy
  register.py      配准（Pose Graph / Frame-to-model）
  fuse.py          TSDF 积分
  mesh.py          Marching Cubes + 后处理
  pipeline.py      顶层入口
  ffs_core/        FFS 源码（core/ + Utils.py，从 Fast-FoundationStereo 拷入）
```

**权重不在此目录**，在 `config.py` 的 `FFS_WEIGHTS_DIR` 指向原始路径：
```
C:\Users\888y9\Desktop\Repo\Fast-FoundationStereo\weights\23-36-37\
```

---

## 环境说明

| 步骤 | 所需环境 |
|------|---------|
| 采集 | `ffs` |
| 重建（无 FFS）| `ffs` |
| 重建（`--ffs`）| `ffs`（需要 GPU） |

---

## 数据目录结构

```
Vision/vision_demo_test_res/ffs_YYYYMMDD_HHMMSS/   ← capture 输出 / recon 输入
  capture_meta.json          相机内参 + 分辨率 + 元信息
  view_000_color.png         左目彩图（配准 / TSDF 用）
  view_000_right.png         右目彩图（FFS 深度推理用）
  view_000_depth.npy         ZED 深度，float32，单位米，NaN=无效
  view_000.ply               ZED 点云（由深度 + 内参生成）
  view_001_...               ← 每次按 SPACE 保存一帧
  ...
  artec_recon/               ← recon 输出（自动创建）
    mesh.ply                 重建网格（Marching Cubes + 后处理）
    pcd.ply                  TSDF 点云
    recon_log.txt            每帧配准 fitness / RMSE

  （--ffs 运行后新增）
  view_000_depth_zed.npy     原始 ZED 深度备份
  view_000_depth.npy         已被 FFS 深度覆盖
  view_000.ply               已被 FFS 深度重新生成
```

> `view_NNN_right.png` 只有 `artec_ffs/capture.py` 采集的数据才有；
> 用 `artec_imit/capture.py` 采的数据**没有右图**，无法跑 `--ffs`。

---

## 工作目录

```bash
cd C:\Users\888y9\Desktop\rsi_printing
```

---

## Step 1 — 采集（ffs env，相机在线）

```bash
conda activate ffs

# 手动模式
python Vision/artec_ffs/pipeline.py capture

# 自动模式
python Vision/artec_ffs/pipeline.py capture --mode auto
```

**手动模式操作键：**`SPACE`=触发一帧  `u`=撤销上一帧  `q`=结束

**Auto 模式注意：**
- 启动后先看 preview 确认相机对准物体（z 值合理），再按 **SPACE 保存第一帧**
- 第一帧保存后自动触发：平移 > 40mm 或旋转 > 8° 触发，缓慢绕物体移动即可
- `SPACE` 在 auto 模式下是**强制触发**，`u`=撤销，`q`=结束

采集完后输出目录如 `Vision/vision_demo_test_res/ffs_20260506_XXXXXX/`，每帧包含：
- `view_NNN_color.png` 左图
- `view_NNN_right.png` 右图（供 FFS 使用）
- `view_NNN_depth.npy` ZED ULTRA 深度（初始）
- `view_NNN.ply`       点云

---

## Step 2 — 重建

### 不用 FFS（仅 ZED 深度）

```bash
conda activate ffs
python Vision/artec_ffs/pipeline.py recon "Vision/vision_demo_test_res/ffs_20260506_XXXXXX"
python Vision/artec_ffs/pipeline.py recon "Vision/vision_demo_test_res/ffs_20260506_XXXXXX" --ftm
```

### 使用 FFS 深度（需要 GPU）

```bash
conda activate ffs
python Vision/artec_ffs/pipeline.py recon "Vision/vision_demo_test_res/ffs_20260506_XXXXXX" --ffs
python Vision/artec_ffs/pipeline.py recon "Vision/vision_demo_test_res/ffs_20260506_XXXXXX" --ffs --ftm
```

`--ffs` 在注册之前：
1. 对每帧左/右图对运行 FFS 推理
2. 将 `view_NNN_depth.npy` 替换为 FFS 深度（原始 ZED 深度备份为 `view_NNN_depth_zed.npy`）
3. 重新生成 `view_NNN.ply`

### 指定输出目录

```bash
python Vision/artec_ffs/pipeline.py recon <data_dir> --ffs --ftm --out <out_dir>
```

输出：`<out_dir>/mesh.ply` + `pcd.ply` + `recon_log.txt`

---

## 单独跑 FFS 深度替换

不进行重建，只替换深度：

```bash
conda activate ffs
python Vision/artec_ffs/ffs_depth.py "Vision/vision_demo_test_res/ffs_20260506_XXXXXX"
```

> 若不需要 GPU（仅验证配准），可以跳过此步，直接 `recon` 不加 `--ffs`。

---

## 关键参数（config.py）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `FFS_VALID_ITERS` | 8 | FFS 迭代次数，↓4=快，↑16=最准 |
| `FFS_SCALE` | 1.0 | 推理前缩放比，0.5=快2×但略损精度 |
| `DEPTH_MAX_M` | 1.2 m | 深度截止距离 |
| `FTM_WARMUP_FRAMES` | 3 | FTM 热身帧数 |
| `TSDF_VOXEL` | 1 mm | TSDF 体素分辨率 |
