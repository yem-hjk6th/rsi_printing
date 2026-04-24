# Header Replacer Scripts — Notes

## 脚本说明

| 脚本 | 用途 |
|------|------|
| `3_quarter_printbed_rsi_off_header_replacer.py` | 开环运行，无 RSI，新 printbed |
| `3_quarter_printbed_rsi_on_header_replacer.py` | 闭环运行，RSI ON，新 printbed |
| `src_header_replace_with_rsi.py` | 旧 printbed 版本（legacy，勿直接用） |

## CONFIG 区关键参数

```python
INPUT_SRC         # 输入：PRC 生成的原始 .src
OUTPUT_SRC        # 输出路径，None = 自动生成 _fixed.src
PROG_NAME         # 必须与 KRL DEF 名和文件名一致
EXTRUDER_RPM      # 挤出机转速
Z_OFFSET          # 打印行 Z 偏移 = bed_top + first_layer_height
                  #   当前值：1.3mm（bed=0.0 + 初始层1.3）
Z_SAFE_THRESHOLD  # 超过此值的 Z 视为安全高度，不做偏移
                  #   当前值：50.0mm
```

## extract_moves 过滤规则

跳过：`ret = RSI_*` / `IF (ret` / `HALT` / `ENDIF` / `;注释` / `END` / `DECL` / `PelletExtruder`

保留：
- 笛卡尔 PTP（含 `X`）
- LIN
- `$VEL.CP=` （仅在运动段内）

## Z offset 逻辑

`apply_z_offset()` 用正则替换行内 `, Z <value>,`，对 `abs(z) >= Z_SAFE_THRESHOLD` 的行跳过。
多层打印时 PRC 原始 Z 值各层不同，offset 正确叠加在每层上。

## 已知陷阱

- PRC 生成的单层文件 Z 通常全为 0，运行脚本后自动偏移至正确高度
- `PROG_NAME` 必须手动改，否则 KRC DEF 名与文件名不匹配
- RSI ON 版本 header 包含 `DECL INT ret/CONTID`；OFF 版本不含，注意勿混用
