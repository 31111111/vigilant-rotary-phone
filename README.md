# KUKA iiwa 机械臂 · 逆运动学（IK）画圆轨迹仿真

基于 [PyBullet](https://github.com/bulletphysics/bullet3)（GitHub 官方 `bullet3` 仓库）中的 KUKA iiwa 模型，演示**逆运动学（Inverse Kinematics）**的核心原理：给定末端目标位置与姿态，反解出 7 个关节角，并用位置控制让机械臂平滑跟踪一段圆形轨迹，最终逐帧渲染出仿真视频。

## 效果演示

`kuka_iiwa_ik.mp4`：KUKA iiwa 末端在三维空间画圆（半径 0.30 m，圆心 `(-0.35, 0.00, 0.50)` m），侧上方相机视角，20 fps。

## 运行环境

- Python ≥ 3.8
- 依赖：`pybullet`、`numpy`、`pillow`、`ffmpeg`（仅合成视频需要）

```bash
pip install pybullet numpy pillow
```

## 运行方法

```bash
python3 kuka_iiwa_ik_demo.py
```

运行后：
- 在当前目录生成 `frames_kuka/` 目录，存放逐帧 PNG（60 帧）
- 终端输出最大末端误差与帧数

合成视频（可选）：

```bash
ffmpeg -framerate 20 -i frames_kuka/frame_%04d.png -c:v libx264 -pix_fmt yuv420p kuka_iiwa_ik.mp4
```

## 核心原理

| 概念 | 说明 |
|---|---|
| 正运动学 FK | 给定关节角 → 计算末端执行器位置/姿态 |
| 逆运动学 IK | 给定末端位置/姿态 → 反解关节角（`calculateInverseKinematics`） |
| 位置控制 | 让关节角平滑跟踪 IK 解（`setJointMotorControl2`，POSITION_CONTROL） |

机械臂：**KUKA iiwa**，7 自由度，固定基座。IK 求解时使用关节上下限（`ll/ul`）、范围（`jr`）与休息位（`rp`）做零空间约束，避免奇异解。

## 关键数据

| 项 | 值 |
|---|---|
| 机械臂 | KUKA iiwa，7 自由度 |
| 轨迹 | 半径 0.30 m 的圆，圆心 (-0.35, 0.00, 0.50) m |
| 仿真时长 | 720 物理步（240 Hz）≈ 3 s |
| 渲染帧 | 60 帧 @ 20 fps |
| 最大末端误差 | 0.324 m（位置控制跟踪滞后，非 IK 求解误差） |

> 说明：0.324 m 的误差来自位置控制对快速画圆的跟踪滞后。想验证 IK 求解本身的精度，可将控制模式改为直接 `resetJointState` 到 IK 解（不经过动力学），误差会降到毫米级。

## 目录结构

```
.
├── kuka_iiwa_ik_demo.py   # 主仿真脚本
├── README.md
├── kuka_iiwa_ik.mp4       # 仿真视频
└── frames_kuka/           # 逐帧渲染图片（运行后生成）
```

## 参考

- PyBullet 官方 IK 示例：`bullet3/examples/pybullet/examples/inverse_kinematics.py`
- 模型：`pybullet_data` 中的 `kuka_iiwa/model.urdf`
