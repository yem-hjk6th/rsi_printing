# ffs_poisson_detail

`artec_ffs` 的细节增强版。技术路线 = **Screened Poisson 表面重建**，核心改动 = 把
TSDF Marching Cubes 换成对融合点云做 Poisson 隐式拟合，配一个**录完即查的数据质量门**。

为什么单独开包：`artec_ffs` 的 3mm TSDF 体素 + Marching Cubes 对人脸这种 ~1-2mm
起伏会平均成"blob"。这里把链路的**融合/网格化两段**换掉，采集/配准/FFS 三段直接复用。

---

## 与 artec_ffs 的关系

| 模块 | 来源 | 改动 |
|------|------|------|
| `capture.py` `register.py` `ffs_depth.py` `fuse.py` `ffs_core/` | 从 artec_ffs 直接复制 | 无 |
| `config.py` | 基于 artec_ffs | 加细节恢复阶梯 + QC 阈值段 |
| `mesh.py` | 重写 | 双后端：`marching_cubes`(原行为) / `poisson`(默认) |
| `pipeline.py` | 基于 artec_ffs | 接 `MESH_BACKEND`、新增 `qc` 子命令 |
| `quality_check.py` | **新增** | 录完即查的数据质量门 |
| `config_local.py` | 复制（gitignored） | 本机 `FFS_REPO_ROOT` |

`FFS_REPO_ROOT` 解析逻辑同 artec_ffs：环境变量 → `config_local.py` → 占位符。

---

## 细节恢复阶梯（易 → 难，都在 `config.py` 里）

| 档 | 参数 | 作用 |
|----|------|------|
| **Rung 1 易** | `FUSE_VOXEL` 0.003→0.0015、`FUSE_TRUNC` 0.012→0.006 | TSDF 体素细化，等效平滑半径减半 |
| **Rung 2 中** | `MESH_BACKEND = "poisson"` | Screened Poisson 替代 Marching Cubes，把表面钉在输入采样上而不是平均掉 |
| **Rung 3 中** | `POISSON_DEPTH` 9、`POISSON_DENSITY_QUANTILE` 0.1 | octree 深度（+1 翻倍分辨率）、低密度顶点裁剪（去 Poisson 的"气球"伪面） |

> **更难的档（神经场 NeuS/Neuralangelo、高斯泼溅 2DGS/SuGaR）不在这个包里** ——
> 那是另一个技术流派，会单独开 `ffs_neural_*` / `ffs_gs_*` 包。参考 `doc/refs/`。

---

## 工作流

```bash
conda activate ffs
cd <path-to-rsi_printing>
```

### 1. 采集（默认 auto 模式）

```bash
python Vision/ffs_poisson_detail/pipeline.py capture --mode auto
```

确认首帧后慢速绕物体移动，相机以 trans>40mm 或 rot>8° 自动触发。
`SPACE`=强制触发首帧/补帧  `u`=撤销  `q`=结束。

### 2. 质量门（录完立刻跑，~2-15s）

```bash
python Vision/ffs_poisson_detail/pipeline.py qc "Vision/vision_demo_test_res/ffs_XXXXXX"
```

**不跑全重建就知道数据好不好。** 输出：

- 逐帧：有效点数 / ROI 填充率 / 深度区间 / 右图是否存在
- 逐相邻帧：快速配准探针（FPFH+RANSAC+point-to-plane ICP），给 fitness / 帧间平移旋转
  —— 直接预测哪个 pair 会在全重建里崩或被 drop
- 总判定 `PASS / WARN / FAIL` + 该重录哪几帧的清单
- 写 `quality_report.json` 到采集目录

退出码：`FAIL` → 1，否则 0（可串脚本）。`--no-pairs` 只跑逐帧、跳过配准探针。

### 3. 重建（默认 FTM + Poisson）

```bash
# ZED 深度
python Vision/ffs_poisson_detail/pipeline.py recon "Vision/vision_demo_test_res/ffs_XXXXXX"

# FFS 深度（需 GPU）
python Vision/ffs_poisson_detail/pipeline.py recon "Vision/vision_demo_test_res/ffs_XXXXXX" --ffs

# 回退到 artec_ffs 行为
python Vision/ffs_poisson_detail/pipeline.py recon <dir> --no-ftm --mesh marching_cubes
```

输出到 `<data_dir>/poisson_recon/`：`mesh.ply` + `pcd.ply` + `recon_log.txt`。

---

## 实测（7 视角人像，YE-SERVER / RTX 5080，2026-05-14）

| 配置 | 顶点 / 三角形 | 总耗时 |
|------|--------------|--------|
| artec_ffs：FFS + Marching Cubes | 67,536 / 128,263 | ~14s |
| 本包：ZED + Poisson | 494,370 / 977,541 | 22s |
| 本包：FFS + Poisson | 580,842 / 1,149,784 | 27s |

网格密度 ~7-9×。**细节是否真的回来需肉眼比对** `mesh.ply` —— Poisson 给的是更稠密、
更贴采样的表面，但当前这批数据本身帧间跳变大（QC 已标 WARN），最终上限受数据限制。
建议先按 auto 模式重录一组、QC 过 PASS，再比较两套网格。

---

## 已知边界

- Poisson 依赖法向：用的是 TSDF 点云自带的 SDF 梯度法向（一致性好）；若缺失则回退到估计+定向。
- Poisson 会在未扫描区域"吹气球"，靠 `POISSON_DENSITY_QUANTILE` 裁低密度顶点缓解，激进数据可调高。
- QC 的逐帧探针用 point-to-plane ICP（比 recon 里的 colored ICP 宽松），所以它标 WARN 的 pair
  在全重建里可能真的被 drop —— WARN 就该认真看。
