# 机械臂 · 仿真合集 + STM32 固件编译 + OpenCV 视觉引导（KUKA iiwa / STM32 6DOF / eye-to-hand）

基于 [PyBullet](https://github.com/bulletphysics/bullet3) 的机械臂仿真，STM32 机械臂固件的交叉编译与逆运动学算法验证，以及基于 OpenCV 的 eye-to-hand 视觉引导抓取闭环。

**最推荐先跑（clone 即可跑，无需任何模型文件）**：`stm32_6dof_arm_demo_lite.py`（画圆）、`stm32_6dof_arm_grasp_demo.py`（抓取-搬运）。纯 Python、不依赖 pybullet、不依赖硬件的一体化视觉项目见 [`vision_arm/`](vision_arm/README.md)，`python main.py` 一条命令即可出闭环视频。

## 内容总览

| # | 脚本 / 目录 | 内容 | 依赖 |
|---|---|---|---|
| 1 | `kuka_iiwa_ik_demo.py` | KUKA iiwa 末端画圆（正/逆运动学 + 位置控制） | pybullet 自带模型 |
| 2 | `kuka_iiwa_grasp_demo.py` | KUKA 抓取-搬运：拾起方块 → 搬运 → 精确放下（偏差 0.001 m） | pybullet 自带模型 |
| 3 | `stm32_6dof_arm_demo_lite.py` | **STM32 6DOF 画圆（简化几何，无 STL，clone 即跑）**，渲染最大误差 1.6 mm | 仅 pybullet |
| 4 | `stm32_6dof_arm_grasp_demo.py` | **STM32 6DOF 抓取-搬运**：把绿方块从左台搬到右台，水平偏差 2.9 mm | 仅 pybullet |
| 5 | `stm32_6dof_arm_demo.py` | STM32 6DOF 画圆（官方 mesh 版，需 18MB STL） | 需 STL 网格 |
| 6 | `stm32_firmware/` | 固件交叉编译 + IK 算法验证 + patch1/patch2 + 数值 IK | gcc / arm-gcc |
| 7 | `vision_arm/` | **OpenCV 视觉引导抓取（eye-to-hand）**：HSV 分割 → 手眼标定 → 平面两连杆 IK → 抓取-搬运闭环 + 100 次蒙特卡洛误差量化 | numpy / opencv / pillow / imageio |

## 效果演示

> **说明**：演示视频为二进制文件，为控制仓库体积未纳入版本库（文本推送通道无法传二进制）。
> 请运行对应脚本自行生成（`vision_arm/main.py` 直接产出 mp4），或在本仓库 Releases 页下载。

| 视频 | 内容 | 生成方式 |
|---|---|---|
| `stm32_6dof_circle_lite.mp4` | **STM32 6DOF 末端在 YZ 平面画圆（简化几何，无 STL）** | 运行 `stm32_6dof_arm_demo_lite.py` |
| `stm32_6dof_grasp.mp4` | **STM32 6DOF 抓取-搬运：绿方块从近台搬到远台** | 运行 `stm32_6dof_arm_grasp_demo.py` |
| `kuka_iiwa_ik.mp4` | KUKA iiwa 末端画圆（半径 0.30 m） | 运行 `kuka_iiwa_ik_demo.py` |
| `kuka_iiwa_grasp.mp4` | KUKA 抓取-搬运，落点偏差 0.001 m | 运行 `kuka_iiwa_grasp_demo.py` |
| `stm32_6dof_arm.mp4` | STM32 6DOF 画圆（官方 mesh 版） | 运行 `stm32_6dof_arm_demo.py` |
| `vision_arm/vision_guided_pickplace.mp4` | **视觉引导抓取-搬运闭环（138 帧，视觉定位误差 0.67 mm）** | `cd vision_arm && python main.py` |

## 运行环境

### 仿真 / 固件部分（`stm32_6dof_*`、`kuka_iiwa_*`）

- Python ≥ 3.8，依赖 `pybullet`、`numpy`、`pillow`；`ffmpeg` 仅合成视频需要。

```bash
pip install pybullet numpy pillow
```

### 视觉部分（`vision_arm/`，**不依赖 pybullet**）

- Python ≥ 3.10，依赖 `numpy`、`opencv-python`、`pillow`、`imageio`、`imageio-ffmpeg`、`matplotlib`。

```bash
cd vision_arm && pip install -r requirements.txt
```

## 运行方法（无模型依赖，clone 即跑）

```bash
python3 stm32_6dof_arm_demo_lite.py    # STM32 6DOF 画圆（简化几何）
python3 stm32_6dof_arm_grasp_demo.py   # STM32 6DOF 抓取-搬运
python3 verify_simple_ik.py            # 校验 arm_simple.urdf 的 FK→IK 自洽（机器精度 3.5e-16）

python3 kuka_iiwa_ik_demo.py           # KUKA 画圆
python3 kuka_iiwa_grasp_demo.py        # KUKA 抓取-搬运

cd vision_arm && python main.py        # 视觉引导抓取闭环（输出 mp4 + 指标）
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

---

## Demo 7 · OpenCV 视觉引导抓取闭环（`vision_arm/`）

**eye-to-hand 视觉伺服**：相机固定在机械臂外、俯视工作台，用 OpenCV 识别目标，手眼标定把像素坐标转为机械臂基坐标，平面两连杆 IK 解算关节角，完成“对准 → 下探 → 夹取 → 搬运 → 释放”九阶段闭环。

**不依赖 pybullet、不依赖真实硬件**：相机与场景用 numpy + OpenCV 自行渲染，算法可先在合成环境跑通、量化误差，再迁移到真实摄像头。

链路：

```
scene.render()  俯视相机（尺度/主点/旋转真值 + 传感器噪声）
      ↓ BGR
vision.detect() HSV 分割 → 形态学开闭 → 轮廓 → 图像矩 / minAreaRect
      ↓ 像素中心
calibration     121 点标定板 → 像素→世界 变换（相似 / 仿射 / 单应）
      ↓ 目标世界坐标 (m)
arm_kinematics  底座方位角 + 平面两连杆 IK（余弦定理，含可达性/肘分支）
      ↓ 关节角
main            9 阶段动作序列 + 夹爪开合 → 138 帧视频 + 指标
```

**结果（均可复现）**：

- 手眼标定（121 点）：尺度估计 **749.72** vs 真值 750 px/m，旋转 **−8.000°** 精确命中，主点精确；重投影误差平均 **0.43 mm**。
- 闭环抓取（138 帧）：视觉定位 / 抓取对准误差 **0.67 mm**，放置落点误差 **0**。
- 蒙特卡洛 100 个随机目标：识别像素均值 **0.75 px**、视觉定位均值 **0.95 mm**（最大 1.49 mm）；误差在工作区内无空间偏向，说明主导误差来自随机噪声而非系统畸变。

分步运行：

```bash
cd vision_arm
python scene.py           # 场景渲染      → output/01_scene_truth.png
python vision.py          # 目标识别      → output/02_detection.png
python calibration.py     # 手眼标定      → 内参估计与重投影误差
python main.py            # 闭环抓取      → vision_guided_pickplace.mp4
python error_analysis.py  # 蒙特卡洛误差  → output/05_error_analysis.png
```

原理、复试高频问答、向真实摄像头 / 机械臂的迁移路径见 [`vision_arm/README.md`](vision_arm/README.md)。

---

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
├── vision_arm/                     # OpenCV 视觉引导抓取（不依赖 pybullet）
│   ├── scene.py                    # 俯视相机模型与场景渲染
│   ├── vision.py                   # HSV 分割 / 形态学 / 轮廓 / 矩
│   ├── calibration.py              # 手眼标定（相似 / 仿射 / 单应）
│   ├── arm_kinematics.py           # 平面两连杆 IK / FK
│   ├── main.py                     # 九阶段抓取闭环
│   └── error_analysis.py           # 100 次蒙特卡洛误差分析
├── *.mp4                            # 演示视频（运行后生成，不入库）
└── frames_lite/ frames_6dof_grasp/  # 逐帧渲染（运行后生成）
```

## 三块内容之间的关系

| 模块 | 回答的问题 | 对应能力 |
|---|---|---|
| `stm32_firmware/` | 固件逆运动学为什么会选错解 / 无解？怎么修？ | 嵌入式 + 数值算法 + 调试归因 |
| 仿真 demo | 机械臂真的能按 IK 动起来吗？ | PyBullet 仿真 + 轨迹验证 |
| `vision_arm/` | 看到的东西怎么变成机械臂能执行的坐标？ | 机器视觉 + 手眼标定 + 闭环控制 |

三者组成一条完整链路：**算得对（固件 IK）→ 动得对（仿真）→ 看得见（视觉引导）**。

## 参考

- PyBullet 官方示例：`bullet3/examples/pybullet/examples/inverse_kinematics.py`
- 模型：`pybullet_data` 中的 `kuka_iiwa/model.urdf`
- STM32 6DOF 机械臂：`zero-robotic-arm`（https://gitee.com/cursorai/zero-robotic-arm）
