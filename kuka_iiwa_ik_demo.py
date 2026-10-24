# -*- coding: utf-8 -*-
"""
KUKA iiwa 机械臂 逆运动学(IK)圆轨迹仿真
基于官方 bullet3 仓库的 kuka_iiwa/model.urdf 模型与 PyBullet IK 接口。

流程：
  1) 加载 KUKA iiwa（7 自由度串联机械臂，固定基座）
  2) 给定一个末端目标位置（沿圆形轨迹连续变化）与姿态
  3) 调用 calculateInverseKinematics 求解 7 个关节角
  4) 用位置控制让关节平滑跟踪，逐帧渲染并保存图片
  5) 统计 IK 求解精度（末端目标 vs 实际位置误差）
"""
import os
import math
import numpy as np
import pybullet as p
import pybullet_data
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
FRAMES = os.path.join(HERE, "frames_kuka")
os.makedirs(FRAMES, exist_ok=True)

# ---------------------------------------------------------------
# 0. 初始化：DIRECT 无界面模式（稳定、不依赖显示服务器）
# ---------------------------------------------------------------
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# ---------------------------------------------------------------
# 1. 加载地面 + KUKA iiwa（固定基座）
# ---------------------------------------------------------------
p.loadURDF("plane.urdf", [0, 0, -0.05])
kuka = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
p.resetBasePositionAndOrientation(kuka, [0, 0, 0], [0, 0, 0, 1])

EE = 6                      # 末端执行器(link)索引
NJ = p.getNumJoints(kuka)   # 关节数，KUKA iiwa = 7
assert NJ == 7, "期望 7 自由度机械臂"

# 关节上下限、范围、休息位（来自官方 inverse_kinematics.py）
ll = [-0.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]
ul = [0.967,  2,  2.96, 2.29,  2.96,  2.09,  3.05]
jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]

for i in range(NJ):
    p.resetJointState(kuka, i, rp[i])
p.setGravity(0, 0, 0)

# ---------------------------------------------------------------
# 2. 相机：从侧上方观察机械臂
# ---------------------------------------------------------------
W, H = 720, 720
view = p.computeViewMatrix(cameraEyePosition=[-0.9, -1.3, 1.15],
                           cameraTargetPosition=[0, 0, 0.55],
                           cameraUpVector=[0, 0, 1])
proj = p.computeProjectionMatrixFOV(fov=45, aspect=W / H, nearVal=0.05, farVal=10)

# ---------------------------------------------------------------
# 3. 主循环：末端沿圆形轨迹运动，IK 求解 + 位置控制 + 渲染
# ---------------------------------------------------------------
TOTAL = 720          # 总物理步数（约 3 秒，240Hz）
RENDER_EVERY = 12    # 每 12 步渲染一帧 -> 视频 20fps
radius = 0.30
center = [-0.35, 0.0, 0.50]   # 圆轨迹圆心

frame_no = 0
max_err = 0.0
print("目标   : 末端在 XY 平面画圆（半径 %.2f m，圆心 %.2f,%.2f,%.2f m）" % (radius, *center))

for s in range(TOTAL):
    t = s / TOTAL * 2 * math.pi
    # 目标末端位置（沿圆轨迹）
    target = [center[0],
              center[1] + radius * math.cos(t),
              center[2] + radius * math.sin(t)]
    # 期望姿态：末端朝下
    orn = p.getQuaternionFromEuler([0, -math.pi, 0])

    # 逆运动学：由末端位姿 -> 7 个关节角
    jointPoses = p.calculateInverseKinematics(kuka, EE, target, orn,
                                              ll, ul, jr, rp)

    # 位置控制跟踪目标关节角
    for i in range(NJ):
        p.setJointMotorControl2(kuka, i, p.POSITION_CONTROL,
                                targetPosition=jointPoses[i],
                                force=500, positionGain=0.03, velocityGain=1)
    p.stepSimulation()

    # 每帧：读取末端实际位置 + 渲染保存
    if s % RENDER_EVERY == 0:
        actual = p.getLinkState(kuka, EE)[0]
        err = math.dist(target, actual)
        max_err = max(max_err, err)

        _, _, rgba, _, _ = p.getCameraImage(W, H, view, proj,
                                            renderer=p.ER_TINY_RENDERER)
        if isinstance(rgba, (bytes, tuple)):  # DIRECT 下返回展平 bytes/tuple
            rgba = np.array(rgba, dtype=np.uint8).reshape(H, W, 4)
        img = Image.fromarray(rgba[:, :, :3], "RGB")
        img.save(os.path.join(FRAMES, "frame_%04d.png" % frame_no))
        frame_no += 1

p.disconnect()
print("渲染帧数 : %d 张" % frame_no)
print("最大末端误差 : %.3f m" % max_err)
print("帧已保存至 : %s" % FRAMES)
