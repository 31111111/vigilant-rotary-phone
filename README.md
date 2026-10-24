# KUKA iiwa 机械臂 · PyBullet 仿真（逆运动学 + 抓取-搬运）

基于 [PyBullet](https://github.com/bulletphysics/bullet3)（GitHub 官方 `bullet3` 仓库）中的 KUKA iiwa 模型，包含两个仿真 demo：

1. **逆运动学（IK）画圆轨迹**：演示正/逆运动学与位置控制
2. **真实抓取-搬运任务**：机械臂从桌面拾起方块，搬运到目标点精确放下

## 效果演示

| 视频 | 内容 |
|---|---|
| `kuka_iiwa_ik.mp4` | KUKA iiwa 末端在三维空间画圆（半径 0.30 m），侧上方视角 |
| `kuka_iiwa_grasp.mp4` | 抓取-搬运：拾起绿色方块 → 抬升 → 搬运 → 精确放下 |

## 运行环境

- Python ≥ 3.8
- 依赖：`pybullet`、`numpy`、`pillow`、`ffmpeg`（仅合成视频需要）

```bash
pip install pybullet numpy pillow
```

## 运行方法

```bash
# demo 1：逆运动学画圆
python3 kuka_iiwa_ik_demo.py

# demo 2：抓取-搬运任务
python3 kuka_iiwa_grasp_demo.py
```

运行后分别生成 `frames_kuka/`、`frames_kuka_grasp/` 帧目录，并在终端输出关键数据。

合成视频（可选）：

```bash
ffmpeg -framerate 20 -i frames_kuka/frame_%04d.png -c:v libx264 -pix_fmt yuv420p kuka_iiwa_ik.mp4
ffmpeg -framerate 16 -i frames_kuka_grasp/frame_%04d.png -c:v libx264 -pix_fmt yuv420p kuka_iiwa_grasp.mp4
```

---

## Demo 1 · 逆运动学（IK）画圆

| 概念 | 说明 |
|---|---|
| 正运动学 FK | 给定关节角 → 计算末端执行器位置/姿态 |
| 逆运动学 IK | 给定末端位置/姿态 → 反解关节角（`calculateInverseKinematics`） |
| 位置控制 | 让关节角平滑跟踪 IK 解（`setJointMotorControl2`，POSITION_CONTROL） |

IK 求解时使用关节上下限（`ll/ul`）、范围（`jr`）与休息位（`rp`）做零空间约束，避免奇异解。

**关键数据**：半径 0.30 m 圆轨迹，720 物理步 ≈ 3 s，60 帧 @ 20 fps，最大末端误差 0.324 m（位置控制跟踪滞后，非 IK 求解误差）。

## Demo 2 · 抓取-搬运任务

状态机流程：**接近 → 下移夹取 → 抬升 → 搬运 → 下放 → 松开收回**

抓取模拟：用 `createConstraint(p.JOINT_FIXED)` 把方块刚性固定到末端 link6 下方，等价于"夹住并抓走"；到达目标点后 `removeConstraint` 松开放下。

**关键数据**：

| 项 | 值 |
|---|---|
| 方块 | 0.06 m 绿色立方体，质量 0.2 kg |
| 拾取点 | (0.45, 0.15) |
| 放置点 | (0.20, -0.45) |
| 落点偏差 | **0.001 m**（位置控制已按阶段分配步数并提高增益收敛到位） |

> 经验：抓取-搬运对**位置控制收敛精度**要求高。搬运位移大时，若每阶段步数太少，末端会滞后、中途就放下方块。可通过提高 `positionGain`（0.10）、加大 `force`（1000），并按位移量分配各阶段步数来保证到位。

## 目录结构

```
.
├── kuka_iiwa_ik_demo.py      # demo 1：逆运动学画圆
├── kuka_iiwa_grasp_demo.py   # demo 2：抓取-搬运任务
├── README.md
├── kuka_iiwa_ik.mp4          # demo 1 仿真视频
├── kuka_iiwa_grasp.mp4       # demo 2 仿真视频
└── frames_kuka/ frames_kuka_grasp/   # 逐帧渲染图片（运行后生成）
```

## 参考

- PyBullet 官方示例：`bullet3/examples/pybullet/examples/inverse_kinematics.py`
- 模型：`pybullet_data` 中的 `kuka_iiwa/model.urdf`
