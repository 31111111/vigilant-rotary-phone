# 三版本 IK 数据导出与可视化（500 组随机点回归）

本目录直接复用固件真实源码（`#include` 对应版本的 `robot_kinematics.c`），导出
**原版 / patch1 / patch2** 三个版本在同一组 500 个随机目标点上的逐点误差与候选分布，
并生成可复现的对比图表。所有数字均来自真实编译运行，可重新生成。

## 数据结论

| 版本 | 有解（正确） | 误选伪根（位置错） | 判定无解 | 最大位置误差 |
|---|---|---|---|---|
| 原版固件 | 438 | **43** | 19 | **354.47 mm** |
| patch1（位置残差校验） | 438 | 0 | 62（正确拒绝） | 0.01 mm |
| patch2（θ1 双分支） | **500** | 0 | 0 | **0.012 mm** |

进一步诊断：62 个失败点在 patch1 中“限位内的候选全是伪根、真解都超限位”；
在 patch2 中枚举 θ1 另一符号根后，**62/62 全部由遗漏的反号根救活**。

## 文件

| 文件 | 说明 |
|---|---|
| `dump_data.c` | 三版本共用的数据导出器（直接 `#include` 固件源码，不复刻算法） |
| `plot_ik.py` | 读取 CSV，用 matplotlib 生成 3 张图（中文字体 DroidSansFallback） |
| `selected_<ver>.csv` | 运行生成：`i,ret,sel_err_mm`（ret: 0=有解 / -1=无解） |
| `cand_<ver>.csv` | 运行生成：`i,k,residual_mm,lim_ok,theta1_deg` |
| `fig*.png` | 运行生成：3 张对比图（二进制，不入库） |

## 复现步骤

```bash
# 在仓库根目录，依次对三个版本编译并导出（-I 指向各自版本目录）
cd analysis
gcc -std=gnu11 -O2 -I../stm32_firmware/pc_ik_test dump_data.c -lm -o dump_orig   && ./dump_orig orig
gcc -std=gnu11 -O2 -I../stm32_firmware/patch       dump_data.c -lm -o dump_patch1 && ./dump_patch1 patch1
gcc -std=gnu11 -O2 -I../stm32_firmware/patch2      dump_data.c -lm -o dump_patch2 && ./dump_patch2 patch2

# 画图（需要 numpy / matplotlib）
python3 plot_ik.py
```

生成 3 张图（图片为二进制，未随仓库分发，运行脚本即生成）：

- `fig1_overview.png`：三版本“正确/误选/无解”堆叠柱 + 逐点位置误差（对数轴）
- `fig2_candidates.png`：62 个失败点的候选分类（残差 × 限位），patch1 vs patch2
- `fig3_theta1_roots.png`：被误选 θ1 与正确 θ1 全部落在 `y=360°−x` 反号参考线上

## 采样口径（与固件 PC 验证一致）

- `srand(42)`；关节限位（度）：J1[0,360] J2[90,180] J3[-90,90] J4[-90,90] J5[0,90] J6[0,360]
- 收窄采样避免边界：lo=[5,95,-85,-85,5,5]，hi=[355,175,85,85,85,355]
- 500 个目标点在三个版本中完全一致（点序号可对齐）
- 位置残差：用 MDH 正运动学反推候选解末端，与目标平移列的欧氏距离（mm）
- 限位判定：复刻固件 `radians_to_degrees_0_360` + `joint_angle_map`（±360 映射后判限位）
