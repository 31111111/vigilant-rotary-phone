# IK 选解缺陷修复 patch（`zero-robotic-arm` 固件）

针对 zero-robotic-arm 固件逆运动学**选解逻辑缺陷**的修复补丁，可审阅、可复现、已验证。

## 缺陷现象

原固件 `robot_kinematics_get_optimal_result` 只按「候选解关节角与当前关节角的加权距离」挑选最优解，**不校验该解能否真正到达目标末端位置**。而解析逆解由半角法 `tan(θ/2)` 平方引入额外根（伪根），当伪根的关节角恰好落在限位内时会被选中，导致机械臂被送到**位置误差可达数百 mm** 的错误坐标。

500 组随机 FK→IK 回归：
- 原固件选解：**91.1%** 位置误差 <1mm，最大误差 **354 mm**（约 9% 选中伪根）
- 修复后选解：**100%** 位置误差 <1mm（凡被接受的目标，最大 0.0088mm）

## 修复原理（不改动原解析算法）

逆解计算完成后（此时关节角仍为弧度、且各候选解相互独立），用 **MDH 正运动学**反推每组候选解的末端位置，与目标位姿平移列比较：

- 末端位置残差 **> 1 mm** → 判定为伪根，置为无效（`result_invalid_mask`），跳过；
- 残差 ≤ 1 mm → 保留，进入原有的限位映射与「按关节角距离选解」。

由于位置校验在弧度域、在角度制转换**之前**执行，避免了「把角度当弧度」的量纲错误。

## 文件说明

| 文件 | 说明 |
|---|---|
| `ik_select_fix.patch` | `diff -u` 生成的统一格式补丁（原版 ↔ 修复版） |
| `robot_kinematics.c` | 打补丁后的固件源码（可直接替换原文件） |
| `robot_kinematics.h` | 对应头文件 |
| `robot.h` / `usart.h` | PC 编译桩头文件（去 FreeRTOS/硬件依赖，LOG→printf） |
| `verify_patched.c` | 回归验证：500 组 FK→IK 走完整 `robot_kinematics_inverse` |
| `diag_patch.c` | 诊断：对「返回无解」的目标点打印 4 组解的残差与限位状态 |
| `Makefile` | x86 构建：`make && ./verify_patched`（同时构建 `diag_patch`） |

## 关于「无解」（重要，修复的正确行为）

修复后约 **62/500（12%）** 目标点返回无解（`-1`）。诊断显示这类点**所有真解都超出关节限位、唯一在限位内的解是伪根**：

```
目标 qd=[16.7 121.4 32.4 -13.2 21.5 92.5]°，平移=[-275.7 -82.8 221.6]mm
  组0: 残差=165.6 mm  限位有效=1   ← 伪根（原固件会选中它 → 末端错 165mm！）
  组1: 残差=0.000 mm  限位有效=0   ← 真解，但超限位
  组2: 残差=0.000 mm  限位有效=0
  组3: 残差=0.000 mm  限位有效=0
```

修复前固件会返回**组0 伪根**（位置错 165mm）；修复后正确判定「该位姿在当前限位构型下不可达」并**拒绝执行**。工程上这是期望行为：**宁可不做，不做错**。

> 注：另有约 62 组真解超限位，属于「IK 解析解分支在限位构型下覆盖不全」的独立问题，不在本 patch 范围，见 README「后续改进」。

## 交叉编译验证（固件本体）

用 xPack `arm-none-eabi-gcc 13.2.1` 对修复后源码交叉编译，**零错误**：

```
   text    data     bss     dec
  51948      96   23104   75148      ← text 比原版 +640B（约 0.6KB）
```

Flash 占用 ≈ 50 KB / 512 KB（10%），入口不变。

## 复现命令

```bash
# 1) 交叉编译修复版固件（工具链见上级 README；此处为在仓库外独立复现的示意）
#    在 zero-robotic-arm/2. Software/robot/ 下替换 robot_kinematics.c 后 make

# 2) PC 端回归验证（需 gcc，推荐直接 make）
make && ./verify_patched
make diag_patch && ./diag_patch
# 等价手动命令：
gcc -std=gnu11 -O2 verify_patched.c -lm -o verify_patched && ./verify_patched
gcc -std=gnu11 -O2 diag_patch.c -lm -o diag_patch && ./diag_patch
```

## 与上级验证的关系

`../pc_ik_test/verify_fix.c` 报告「修复后 100%、0 无解」，口径为**手动位置校验（不计限位）**；本目录 `verify_patched.c` 走**完整 `robot_kinematics_inverse`（含限位校验）**，故出现 62 组「正确拒绝的无解」。两者共同证明：修复剔除了伪根，被接受的目标位置全部精确。
