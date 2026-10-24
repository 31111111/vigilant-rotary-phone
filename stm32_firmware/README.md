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

### 给固件的改进建议（不改动原算法，仅在选解处）

在 `robot_kinematics_get_optimal_result()` 遍历候选解时，先用正运动学（或直接代入位置方程）算末端位置残差，
把残差超过阈值（如 1 mm）的解判为无效，再在剩余解中按关节角距离选最近者。这样可消除半角法伪根，把逆解位置可靠性提升到接近 100%。

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
│   └── verify_fix.c         # 选解缺陷定位 + 位置校验修复验证
└── zero_robotic_arm_stm32f407.bin/.elf   # 已交叉编译的固件（二进制，见交付文件）
```
