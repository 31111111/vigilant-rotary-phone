# 机械臂 · 仿真合集 + STM32 固件编译（KUKA iiwa / STM32 6DOF）

基于 [PyBullet](https://github.com/bulletphysics/bullet3) 的机械臂仿真，以及 STM32 机械臂固件的交叉编译与逆运动学算法验证。

**最推荐先跑（clone 即可跑，无需任何模型文件）**：`stm32_6dof_arm_demo_lite.py`（画圆）、`stm32_6dof_arm_grasp_demo.py`（抓取-搬运）。

## 内容总览

| # | 脚本 | 内容 | 依赖 |
|---|---|---|---|
| 1 | `kuka_iiwa_ik_demo.py` | KUKA iiwa 末端画圆（正/逆运动学 + 位置控制） | pybullet 自带模型 |
| 2 | `kuka_iiwa_grasp_demo.py` | KUKA 抓取-搬运：拾起方块 → 搬运 → 精确放下（偏差 0.001 m） | pybullet 自带模型 |
| 3 | `stm32_6dof_arm_demo_lite.py` | **STM32 6DOF 画圆（简化几何，无 STL，clone 即跑）**，渲染最大误差 1.6 mm | 仅 pybullet |
| 4 | `stm32_6dof_arm_grasp_demo.py` | **STM32 6DOF 抓取-搬运**：把绿方块从左台搬到右台，水平偏差 2.9 mm | 仅 pybullet |
| 5 | `stm32_6dof_arm_demo.py` | STM32 6DOF 画圆（官方 mesh 版，需 18MB STL） | 需 STL 网格 |
| 6 | `stm32_firmware/` | 固件交叉编译 + IK 算法验证 + patch1/patch2 + 数值 IK | gcc / arm-gcc |

## 效果演示

| 视频 | 内容 |
|---|---|
| `stm32_6dof_circle_lite.mp4` | **STM32 6DOF 末端在 YZ 平面画圆（简化几何，无 STL）** |
| `stm32_6dof_grasp.mp4` | **STM32 6DOF 抓取-搬运：绿方块从近台搬到远台** |
| `kuka_iiwa_ik.mp4` | KUKA iiwa 末端画圆（半径 0.30 m） |
| `kuka_iiwa_grasp.mp4` | KUKA 抓取-搬运，落点偏差 0.001 m |
| `stm32_6dof_arm.mp4` | STM32 6DOF 画圆（官方 mesh 版） |

## 运行环境

- Python ≥ 3.8，依赖 `pybullet`、`numpy`、`pillow`；`ffmpeg` 仅合成视频需要。

```bash
pip install pybullet numpy pillow
```

## 运行方法（无模型依赖，clone 即跑）

```bash
python3 stm32_6dof_arm_demo_lite.py    # STM32 6DOF 画圆（简化几何）
python3 stm32_6dof_arm_grasp_demo.py   # STM32 6DOF 抓取-搬运
python3 verify_simple_ik.py            # 校验 arm_simple.urdf 的 FK→IK 自洽（机器精度 3.5e-16）

python3 kuka_iiwa_ik_demo.py           # KUKA 画圆
python3 kuka_iiwa_grasp_demo.py        # KUKA 抓取-搬运
```

合成视频：

```bash
ffmpeg -y -framerate 24 -i frames_lite/frame_%04d.png -c:v libx264 -pix_fmt yuv420p -crf 20 stm32_6dof_circle_lite.mp4
ffmpeg -y -framerate 24 -i frames_6dof_grasp/frame_%04d.png -c:v libx264 -pix_fmt yuv420p -crf 20 stm32_6dof_grasp.mp4
```

---

## Demo 3 · STM32 6DOF 画圆（简化几何，clone 即跑）

模型来自开源项目 **zero-robotic-arm（零一造物，STM32F407 主控）**。为去掉对 18MB 二进制 STL 的依赖，用 **URDF 基础 box/cylinder 几何体**重建了 `stm32_arm/urdf/arm_simple.urdf`：关节运动学与官方 `arm.urdf` 逐字一致，各 link 保留官方真实质量/质心/惯量，仅用基础几何体做视觉（无 collision、无 mesh）。

精度（150 帧）：
- `verify_simple_ik.py`：FK→IK 自洽误差 **3.55e-16**（机器精度）；
- 离线画圆轨迹最大误差 5.62 mm，**实际渲染最大误差仅 1.6 mm**。

## Demo 4 · STM32 6DOF 抓取-搬运（简化几何）

该臂在抓取点末端天然为**水平姿态**（沿 ±Y，全姿态约束下无法竖直向下），故采用：
**水平对准 → 水平伸出 → 竖直抬起 → 搬运 → 竖直放下 → 松开**。

- 运动学臂每帧 `resetJointState` 置位后 `stepSimulation` 推方块；方块为真实动力学刚体；
- 用 `createConstraint(JOINT_FIXED)` 固定抓取，`removeConstraint` 松开；
- 7 个关键点 IK 误差均 ≤ 0.55 mm；方块最终水平偏离目标中心 **2.9 mm**，z 向正好落稳在远台板顶（226 帧）。

### STM32 6DOF 仿真的关键工程经验（务必遵守）

1. **`ee_link` 的 inertial origin 必须为 `(0,0,0)`**：PyBullet 的 IK 目标施加在 link **惯性系原点（质心）**，而 `getLinkState(ee)[0]` 返回的是 **link 坐标系原点**。官方 ee_link 惯性原点有偏移（官方为 `(0,~0,-0.0081)`），不置零会产生等于质心偏移的常量位置误差（曾精确出现 0.05 m / 0.1564 m 误差）。
2. **用纯位置 IK**（不传 `targetOrientation`）：该臂在 yz 平面用单位四元数做全姿态约束会过约束。
3. **必须收紧残差阈值**：`calculateInverseKinematics(..., maxNumIterations=200, residualThreshold=1e-8)`；默认 `residualThreshold=1e-4` 会让 IK 在约 10 mm 误差处提前停止。
4. **多起点 + 两阶段选解**：6 个种子求 IK，先在误差 ≤ 8 mm 的候选里选关节变化最小者，否则选误差最小者，保证轨迹连续且无跳变。
5. **纯运动学动画**：`resetJointState` 置位后**不要** `stepSimulation`（默认电机会把关节带离）；抓取演示则相反，需要 step 推动方块。

## Demo 5 · STM32 画圆（官方 mesh 版，需 STL）

`stm32_6dof_arm_demo.py` + `stm32_arm/urdf/arm.urdf`（mesh 路径 `../meshes/*.STL`）。`.STL` 为二进制未纳入仓库，可从原项目 https://gitee.com/cursorai/zero-robotic-arm 获取后放入 `stm32_arm/meshes/`。

---

## Demo 6 · STM32 固件交叉编译 + 逆运动学验证（`stm32_firmware/`）

针对 zero-robotic-arm 的 STM32CubeIDE 固件工程（原工程无 Makefile）：

- **交叉编译**：补了 GNU Makefile，用 xPack `arm-none-eabi-gcc` 13.2.1 编译 HAL + FreeRTOS + CAN 步进电机驱动，零错误生成 `.bin/.elf/.hex`。
- **PC 端验证固件 IK**：把固件逆运动学源码用普通 gcc 在 x86 编译，500 组随机 FK→IK 回归，定位并修复了两层真实缺陷：

| 层级 | 问题 | 修复 | 验证 |
|---|---|---|---|
| **patch1** `patch/` | 半角法 `tan(θ/2)` 引入伪根，原选解只按关节距离、不校验末端位置，约 9% 选到伪根（位置误差可达 354 mm） | 选解前用 MDH 正运动学校验末端位置残差，>1mm 判伪根 | 被接受目标 100% <1mm（最大 0.0088 mm） |
| **patch2** `patch2/` | patch1 后仍有 **62 组可达却返回无解**：θ1 半角法 `py` 二选一选错符号、丢掉另一个几何根 | 对每个 (θ2,θ3) 显式枚举 θ1 两个几何根（±u），候选 4→8，统一由位置校验选解（最小侵入，仅改 3 处） | **438/500 → 500/500**（最大 0.012 mm），已交叉编译 |
| **数值兜底** `numeric_ik/` | 解析几何未覆盖的奇异/边界目标 | 阻尼最小二乘（DLS）数值 IK，独立第二保险 | 62 个解析失败点 **62/62 兜底**（位置 max 0.988 mm、姿态 max 0.108°），综合 100% |

可烧录固件：`zero_robotic_arm_stm32f407_patch2.bin`（patch2，51880 B）；`..._fix.bin`（patch1）、`..._407.bin`（原版）。

> 数值 IK 关键点：误差定义为 `e = x_target − x_fk(q)`，其差分为 `−J`，故数值雅可比必须取负 `J[:,i] = −(e(q+δ)−e(q−δ))/(2δ)`，否则 DLS 反向发散。

详见 [`stm32_firmware/README.md`](stm32_firmware/README.md)、[`stm32_firmware/patch2/README.md`](stm32_firmware/patch2/README.md)、[`stm32_firmware/numeric_ik/README.md`](stm32_firmware/numeric_ik/README.md)。

## 目录结构

```
.
├── stm32_6dof_arm_demo_lite.py    # 推荐：6DOF 画圆（无 STL，clone 即跑）
├── stm32_6dof_arm_grasp_demo.py   # 推荐：6DOF 抓取-搬运（无 STL）
├── verify_simple_ik.py            # arm_simple.urdf 的 FK→IK 自洽校验
├── stm32_arm/urdf/arm_simple.urdf  # box/cylinder 简化几何（无 STL，clone 即跑）
├── stm32_arm/urdf/arm.urdf         # 官方 mesh 版（需 meshes/*.STL）
├── kuka_iiwa_ik_demo.py / kuka_iiwa_grasp_demo.py
├── stm32_6dof_arm_demo.py          # 旧：官方 mesh 版画圆（需 STL）
├── stm32_firmware/                 # 固件交叉编译 + IK 验证 + patch/ + patch2/ + numeric_ik/
├── *.mp4                            # 演示视频
└── frames_lite/ frames_6dof_grasp/  # 逐帧渲染（运行后生成）
```

## 参考

- PyBullet 官方示例：`bullet3/examples/pybullet/examples/inverse_kinematics.py`
- 模型：`pybullet_data` 中的 `kuka_iiwa/model.urdf`
- STM32 6DOF 机械臂：`zero-robotic-arm`（https://gitee.com/cursorai/zero-robotic-arm）
