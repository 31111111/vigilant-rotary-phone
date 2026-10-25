# STM32 6DOF 机械臂固件 · 交叉编译 + 逆运动学验证

对开源项目 **zero-robotic-arm（零一造物，主控 STM32F407VETx）** 的固件做两件事：

1. **命令行交叉编译**：原工程是 STM32CubeIDE 工程（无 Makefile），这里补一份 GNU Makefile，用免安装的 `arm-none-eabi-gcc` 直接编译出可烧录固件 `.bin/.hex/.elf`。
2. **PC 端运行固件核心算法**：把固件里**逐字节未改动**的逆运动学源码 `robot_kinematics.c` 抽出来，用普通 gcc 在 x86 上编译运行，验证解析解正确性，并发现/定位了一处选解缺陷。

> 固件源码（HAL、FreeRTOS、CAN 步进电机驱动等）来自原项目，不在此重复；本目录只放可复现的构建脚本与验证代码。
> 原项目：https://gitee.com/cursorai/zero-robotic-arm （路径 `2. Software/robot/`）

---

## 一、交叉编译固件

### 1. 准备免安装 ARM 工具链（无需 root）

```bash
# 下载 xPack GNU Arm Embedded GCC 并解压到本地（示例为 13.2.1）
curl -L -o arm-gcc.tar.gz \
  https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v13.2.1-1.1/xpack-arm-none-eabi-gcc-13.2.1-1.1-linux-x64.tar.gz
tar xzf arm-gcc.tar.gz
```

### 2. 编译

把 `Makefile` 放到原工程根目录（与 `Core/`、`Drivers/`、`Middlewares/` 同级），
按工具链实际解压位置修改 `Makefile` 顶部的 `GCC_DIR`，然后：

```bash
make          # 生成 build/robot.elf / robot.bin / robot.hex
make clean
```

### 3. 构建结果

| 项 | 数值 |
|---|---|
| 芯片 | STM32F407VETx（Cortex-M4F，512 KB Flash / 128 KB SRAM） |
| 代码段 text | 51,308 B |
| 已初始化数据 data | 96 B（占 Flash） |
| BSS（未初始化/堆栈） | 23,104 B |
| **Flash 占用** | **约 50 KB（~10%）** |
| SRAM 占用 | 约 23 KB（~18%） |
| 入口 | `Reset_Handler @ 0x0800775c`（Flash 0x08000000 段）✓ |

`robot.bin` 即可用 ST-Link / J-Link 或串口 ISP 烧录（本环境无硬件，仅验证到编译产物）。

---

## 二、PC 端运行并验证固件逆运动学（无需硬件）

固件的逆运动学是纯 C 数学代码（基于 DH 参数的 6 轴解析解，用 tan(θ/2) 半角法，一次输出 4 组解再选最优）。
`pc_ik_test/` 中 `robot_kinematics.c/.h` 与固件**完全一致**，仅用 `robot.h`（精简数据结构）和 `usart.h`（把 `LOG` 重定向到 printf）替代硬件头文件。

```bash
cd pc_ik_test
# [1] 复位位姿反算 + 可达点测试
gcc -std=gnu11 -O2 main.c robot_kinematics.c -lm -o ik_test && ./ik_test
# [2] FK→IK 闭环往返（200 组随机姿态）
gcc -std=gnu11 -O2 roundtrip.c robot_kinematics.c -lm -o roundtrip && ./roundtrip
# [3] 选解缺陷验证（修复前 vs 修复后）
gcc -std=gnu11 -O2 verify_fix.c -lm -o verify_fix && ./verify_fix
```

### 验证结果

**(1) 复位位姿精确还原**：喂入固件标定的复位位姿 `T_0_6_reset`（末端位置 (0,-47.63,15.5) mm），逆解输出

```
[90, 90, -90, 0, 90, 0]°  ==  固件复位关节角（误差 0.000°）
```

**(2) FK→IK 闭环**：200 组随机关节角 → 正运动学生成位姿 → 固件 IK 反算 → 再正运动学还原：
- 末端姿态误差 < 0.04°（旋转解非常精确）
- 但约 9% 的点末端位置误差很大（最大 ~354 mm）

**(3) 缺陷定位（数据驱动）**：对误差大的点逐组检查内部 4 组解，发现**其中 3 组位置误差精确为 0 mm，只有 1 组是误差上百 mm 的伪根**，而固件的最优解选择（`robot_kinematics_get_optimal_result`）**只按“与当前关节角加权距离”挑选、不校验末端位置**，于是约 9% 情况下选到了伪根。

- 伪根来源：`theta2` 用半角法（`u=tan(θ/2)`）平方求解时引入的额外根；
- 修复验证：在选解中增加一次“正运动学末端位置残差 < 1 mm”的过滤，500 组随机点：

```
固件原始（仅按关节角距离选解）：位置误差 <1mm 占 91.1%，最大 354 mm
加入位置残差校验后：           位置误差 <1mm 占 100%，最大 0.01 mm
```

### 缺陷修复（已落地为 patch，见 `patch/`）

上面的改进建议**已实现并交叉编译验证通过**，见 [`patch/`](patch/)：

- 在逆解计算完成后（弧度域、角度制转换之前），用 MDH 正运动学反推每组候选解的末端位置，残差 > 1 mm 判为伪根并置无效；
- 不改动原解析算法，仅在选解前多一道几何校验；
- 回归验证（完整 `robot_kinematics_inverse`，500 组）：**凡被接受的目标位置误差 100% < 1 mm（最大 0.0088 mm）**；约 12% 目标因「唯一限位内解是伪根、真解超限位」被正确判定为不可达而拒绝执行（宁可不做、不做错）；
- 交叉编译修复版固件零错误，`text` 仅 +640 B（Flash 仍 ~10%）。

已生成的修复版可烧录固件：`zero_robotic_arm_stm32f407_fix.bin/.elf`（见交付文件）。

### 仍有 62 组“真解被判无解” → patch2（θ1 双分支）+ 数值 IK

把 patch1 放到完整 500 组回归后发现：位置校验能保证**被接受的解都正确**，但仍有 **62 组目标明明可达，`robot_kinematics_inverse` 却返回 -1**。诊断结论：

- 这 62 个点的臂部构型（θ2/θ3）解析值与真解完全一致，**唯一差别在 θ1**；
- θ1 用半角法 `u=tan(θ/2)` 求根后，原代码用 `if (fabs(py-eq2)>ERR) u=-u;` 做一次性二选一。该判据在边界情况下选错符号，且只保留一个根，错选即漏解；
- 62 个点的正确 θ1 **全部**是固件给出值的另一个符号根（`−θ1`），`θ1±π` 分支贡献为 0。

**修复（patch2，[`patch2/`](patch2/)）**：对每个 (θ2,θ3) 显式枚举 θ1 的两个几何根（`u=+√` 与 `u=−√`，即 θ1 与 −θ1），各自重算腕部，候选解由 4 增至 8，统一由 patch1 的位置校验 + 限位 + 关节距离选解。补丁为最小侵入，仅改 `RESULT_NUM`、`calc_theta2`、`calc_theta1` 三处。

```
patch1（4 候选）：438/500 成功，62 组无解
patch2（8 候选，枚举 θ1 两支）：500/500 成功，位置误差 100% <1mm（最大 0.0119 mm，平均 0.00021 mm）
```

**数值 IK 第二保险（[`numeric_ik/`](numeric_ik/)）**：与解析法完全独立的阻尼最小二乘（DLS）6 轴数值逆解，可在解析 8 候选全部无效时兜底。同一批 62 个失败点上：

```
数值 IK 真解种子自检：500/500（最大位置误差 0.0000 mm）
解析失败点数值兜底：62/62（位置 max 0.988 mm / 平均 0.254 mm，姿态 max 0.108° / 平均 0.0095°）
综合成功率：500/500 = 100%
```

patch2 与 numeric_ik 相互独立、任一条都可单独达成 100%；建议默认走 patch2 解析、numeric_ik 作为可选兜底。

patch2 已交叉编译零错误，可烧录固件：`zero_robotic_arm_stm32f407_patch2.bin/.elf`（51,880 B；text 51,772 B / data 96 B / BSS 23,200 B，`g_robot_kinematics` 结构体大小 0x104=260 B，即由 4 候选 140 B 扩为 8 候选 260 B）。

> 数值 IK 关键点：误差定义为 `e = x_target − x_fk(q)`，其差分为 `−J`，故数值雅可比必须取负 `J[:,i] = −(e(q+δ)−e(q−δ))/(2δ)`，否则 DLS 反向发散。

---

## 文件说明

```
stm32_firmware/
├── Makefile                 # 为 CubeIDE 工程补的交叉编译 Makefile
├── pc_ik_test/
│   ├── robot_kinematics.c   # 固件逆运动学源码（原样，未改动）
│   ├── robot_kinematics.h   # 原样
│   ├── robot.h              # PC 精简数据结构（去硬件依赖）
│   ├── usart.h              # LOG -> printf 桩
│   ├── main.c               # 复位位姿反算 + 可达性
│   ├── roundtrip.c          # FK→IK 闭环往返验证
│   └── verify_fix.c         # 选解缺陷定位 + 位置校验修复验证（口径：手动位置校验）
├── patch/                   # patch1：IK 选解位置残差校验（已落地、可审阅、已验证）
│   ├── ik_select_fix.patch  # diff -u 统一格式补丁
│   ├── robot_kinematics.c   # 打补丁后的固件源码
│   ├── verify_patched.c     # 回归验证（口径：完整 robot_kinematics_inverse）
│   ├── diag_patch.c         # 无解点诊断（伪根 vs 真解超限位）
│   └── README.md
├── patch2/                  # patch2：θ1 双分支（候选 4→8），修复 62 组可达却无解
│   ├── ik_theta1_dualbranch.patch  # patch1 ↔ patch2 统一格式 diff（最小侵入，约 100 行）
│   ├── robot_kinematics.c/.h       # 打补丁后的固件源码（RESULT_NUM=8）
│   ├── verify_patch2.c / Makefile  # 500 组回归（make && ./verify_patch2）
│   └── README.md
├── numeric_ik/              # 数值 IK 兜底（DLS 阻尼最小二乘，独立第二保险）
│   ├── numeric_ik.c/.h      # FK + 数值雅可比 + DLS + 多起点
│   ├── verify_numeric_ik.c / Makefile  # 500 组验证（make && ./verify_numeric_ik）
│   └── README.md
├── zero_robotic_arm_stm32f407.bin/.elf        # 已交叉编译固件（原版）
├── zero_robotic_arm_stm32f407_fix.bin/.elf    # patch1（位置校验）
└── zero_robotic_arm_stm32f407_patch2.bin/.elf # patch2（θ1 双分支，推荐）
```
