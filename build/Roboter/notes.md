# Roboter Build Notes

## Printbed 坐标参考（3/4 quarter printbed，2026-04）

| 参数 | 值 |
|------|----|
| Bed 顶面 Z（KRC actual position） | 0.0 mm |
| 初始层高（first layer height） | 1.3 mm |
| 层步进（layer step） | 1.0 mm |
| Layer N Z | `1.3 + (N-1) × 1.0` mm |

> 旧 printbed 顶面约 -11.3mm，Layer 1 约 -10mm，层步进 1.5mm。

---

## 文件夹结构

- `ori/` — PRC 直接生成的原始 .src（不可直接上传 KRC）
- `src/` — 经 replacer 处理后的 KRC 可用 .src
- `repalcer/` — header 替换脚本

---

## 安全高度

- 接近点安全高度：Z ≥ 50mm 视为 approach，不做偏移处理
- 典型接近点：`Z 60`（PTP C_PTP 下降至打印区域上方）
