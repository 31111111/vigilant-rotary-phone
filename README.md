# 机械臂 · 仿真合集 + STM32 固件编译（KUKA iiwa / STM32 6DOF）

基于 [PyBullet](https://github.com/bulletphysics/bullet3) 的机械臂仿真，以及 STM32 机械臂固件的交叉编译与算法验证，共四部分：

1. **KUKA iiwa · 逆运动学（IK）画圆轨迹**：演示正/逆运动学与位置控制
2. **KUKA iiwa · 真实抓取-搬运任务**：机械臂从桌面拾起方块，搬运到目标点精确放下
3. **STM32 6DOF 机械臂 · 逆运动学画圆**：零一造物机械臂（STM32F407 主控）的运动学控制复现
4. **STM32 固件交叉编译 + IK 算法验证**：用 `arm-none-eabi-gcc` 编译出可烧录固件，并在 PC 上验证、定位固件逆运动学

## 效果演示

| 视频 | 内容 |
|---|---|
| `kuka_iiwa_ik.mp4` | KUKA iiwa 末端画圆（半径 0.30 m），侧上方视角 |
| `kuka_iiwa_grasp.mp4` | 抓取-搬运：拾起绿色方块 → 抬升 → 搬运 → 精确放下（落点偏差 0.001 m） |
| `stm32_6dof_arm.mp4` | STM32 6DOF 机械臂末端在 YZ 平面画圆 |

## 运行环境

- Python ≥ 3.8
- 依赖：`pybullet`、`numpy`、`pillow`、`ffmpeg`（仅合成视频需要）

```bash
pip install pybullet numpy pillow
```

## 运行方法

```bash
# demo 1：KUKA 逆运动学画圆
python3 kuka_iiwa_ik_demo.py

# demo 2：KUKA 抓取-搬运
python3 kuka_iiwa_grasp_demo.py

# demo 3：STM32 6DOF 逆运动学画圆
python3 stm32_6dof_arm_demo.py
```

> demo 3 需要 `stm32_arm/meshes/*.STL` 模型文件（见下方“模型获取”）。

合成视频（可选）：

```bash
ffmpeg -framerate 20 -i frames_kuka/frame_%04d.png -c:v libx264 -pix_fmt yuv420p kuka_iiwa_ik.mp4
ffmpeg -framerate 16 -i frames_kuka_grasp/frame_%04d.png -c:v libx264 -pix_fmt yuv420p kuka_iiwa_grasp.mp4
ffmpeg -framerate 20 -i frames_6dof/frame_%04d.png -c:v libx264 -pix_fmt yuv420p stm32_6dof_arm.mp4
```

---

## Demo 1 · KUKA iiwa 逆运动学画圆

| 概念 | 说明 |
|---|---|
| 正运动学 FK | 给定关节角 → 计算末端执行器位置/姿态 |
| 逆运动学 IK | 给定末端位置/姿态 → 反解关节角（`calculateInverseKinematics`） |
| 位置控制 | 让关节角平滑跟踪 IK 解（`setJointMotorControl2`，POSITION_CONTROL） |

半径 0.30 m 圆轨迹，720 物理步 ≈ 3 s，60 帧 @ 20 fps，最大末端误差 0.324 m（位置控制跟踪滞后）。

## Demo 2 · KUKA iiwa 抓取-搬运任务

状态机：**接近 → 下移夹取 → 抬升 → 搬运 → 下放 → 松开收回**

抓取模拟用 `createConstraint(p.JOINT_FIXED)` 把方块固定到末端 link6 下方；到达目标点后 `removeConstraint` 松开放下。落点偏差 **0.001 m**（通过提高位置控制增益并按位移分配步数保证收敛）。

> 经验：抓取-搬运对位置控制收敛精度要求高。搬运位移大时步数太少会导致末端滞后、中途放下。可提高 `positionGain`(0.10)、`force`(1000) 并按位移分配步数。

## Demo 3 · STM32 6DOF 机械臂逆运动学画圆

模型来自开源项目 **zero-robotic-arm（零一造物，STM32F407 主控）**。STM32 固件需在芯片上运行、其官方仿真依赖 MATLAB，故用官方 SolidWorks 导出的 URDF + STL 模型，在 PyBullet 中复现固件的运动学控制逻辑（IK 求解 6 关节角）。

要点：
- 6 个旋转关节（基座 + 肩肘腕）+ 末端 `ee_link`
- 该机械臂工作空间集中在 x≈0 的 YZ 平面，故在 YZ 平面画圆（半径 0.12 m，圆心 (0,0,0.35)）
- 连续关节限位（`lower>upper`）给 ±π；用上一步关节角作 restPose 抑制 IK 解跳变；画圆前 200 步热身消除起始跳变
- 最大末端误差 0.132 m（含圆上个别点 IK 固有误差）

### 模型获取

`stm32_arm/meshes/` 下的 `.STL` 网格为二进制文件，未纳入本仓库。请从原项目获取并放入对应目录：

```bash
# 原项目：https://gitee.com/cursorai/zero-robotic-arm
# 所需 STL（放入 stm32_arm/meshes/）：
#   base_link.STL, link1..5.STL, ee_link.STL
```

## Demo 4 · STM32 固件交叉编译 + 逆运动学验证（`stm32_firmware/`）

针对 zero-robotic-arm 的 STM32CubeIDE 固件工程（原工程无 Makefile）：

- **交叉编译**：补了一份 GNU Makefile，用免安装的 xPack `arm-none-eabi-gcc` 13.2.1 编译 46 个 C 源文件（HAL + FreeRTOS + CAN 步进电机驱动），零错误生成 `robot.bin/.hex/.elf`。Flash 占用约 **50 KB / 512 KB（10%）**，入口 `Reset_Handler @ 0x0800775c`。
- **PC 端验证固件 IK**：把固件**未改动**的逆运动学源码 `robot_kinematics.c` 用普通 gcc 在 x86 上编译运行：
  - 复位位姿反算 = `[90,90,-90,0,90,0]°`，与固件复位关节角**误差 0**；
  - 200 组 FK→IK 闭环，姿态误差 < 0.04°；
  - **定位到一处真实缺陷**：4 组解析解中含半角法 `tan(θ/2)` 引入的伪根，而原选解只按关节角距离挑选、不校验末端位置，约 9% 情况选到伪根（位置误差可达 354 mm）；在选解中加入“末端位置残差 < 1 mm”校验后，500 组随机点位置误差 <1 mm 的比例从 **91.1% 提升到 100%（最大 0.01 mm）**。

详见 [`stm32_firmware/README.md`](stm32_firmware/README.md)。

## 目录结构

```
.
├── kuka_iiwa_ik_demo.py      # demo 1：KUKA 逆运动学画圆
├── kuka_iiwa_grasp_demo.py   # demo 2：KUKA 抓取-搬运
├── stm32_6dof_arm_demo.py    # demo 3：STM32 6DOF 逆运动学画圆
├── stm32_arm/urdf/arm.urdf   # demo 3：6DOF 机械臂 URDF（修正 mesh 路径）
├── stm32_arm/meshes/*.STL    # demo 3：6DOF 机械臂网格（需自取，见“模型获取”）
├── stm32_firmware/           # demo 4：STM32 固件交叉编译 Makefile + PC 端 IK 验证
├── README.md
├── kuka_iiwa_ik.mp4          # demo 1 视频
├── kuka_iiwa_grasp.mp4       # demo 2 视频
├── stm32_6dof_arm.mp4        # demo 3 视频
└── frames_kuka/ frames_kuka_grasp/ frames_6dof/   # 逐帧渲染图片（运行后生成）
```

## 参考

- PyBullet 官方示例：`bullet3/examples/pybullet/examples/inverse_kinematics.py`
- 模型：`pybullet_data` 中的 `kuka_iiwa/model.urdf`
- STM32 6DOF 机械臂：`zero-robotic-arm`（https://gitee.com/cursorai/zero-robotic-arm）
