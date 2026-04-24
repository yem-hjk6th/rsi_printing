# Printbed 设置变更记录

## 当前配置（3/4 quarter printbed，2026-04）

- Bed 顶面 Z = **0.0 mm**（KRC actual position）
- 初始层高 = 1.3 mm → Layer 1 Z = **1.3 mm**
- 层步进 = 1.0 mm → Layer N Z = `1.3 + (N-1) × 1.0`
- `$VEL.CP` 打印速度：0.012 m/s
- 挤出机 RPM：400

## 旧配置（参考）

- Bed 顶面约 -11.3 mm
- Layer 1 Z ≈ -10.0 mm
- 层步进 1.5 mm

## src 文件中需注意的位置

| 位置 | 说明 |
|------|------|
| 接近行（PTP C_PTP） | Z ≥ 50 为安全高度，不做偏移 |
| Layer 1 Z | 最关键，= bed_top + first_layer_height |
| 各层步进 Z | 逐层递增，replacer 脚本通过 Z_OFFSET 自动处理 |
| HOME PTP（关节空间） | 无 X/Y/Z，replacer 不处理 |
| 最后一层封口点 | 注意 X 坐标是否回到起点（部分文件末尾 X 与其他层不同，属有意设计） |
