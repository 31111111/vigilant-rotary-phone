# -*- coding: utf-8 -*-
"""
STM32 6自由度机械臂 · 逆运动学(IK)画圆轨迹仿真
模型：zero-robotic-arm（零一造物，STM32F407 主控）的 SolidWorks 导出 URDF + STL。
说明：STM32 固件本身需在芯片上运行；这里用其官方 URDF 模型在 PyBullet 中
复现固件里的运动学控制逻辑——逆运动学求解 6 关节角，位置控制驱动末端画圆。

工作空间：该机械臂末端主要在 x≈0 的 YZ 平面运动，故在 YZ 平面画圆。
"""
import os
import math
import numpy as np
import pybullet as p
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
FRAMES = os.path.join(HERE, "frames_6dof")
os.makedirs(FRAMES, exist_ok=True)

# ---------------------------------------------------------------
# 0. 加载机械臂 URDF（已修正 mesh 相对路径）
# ---------------------------------------------------------------
p.connect(p.DIRECT)
URDF = os.path.join(HERE, "stm32_arm", "urdf", "arm.urdf")
arm = p.loadURDF(URDF, [0, 0, 0], useFixedBase=True)

NJ = p.getNumJoints(arm)
EE = NJ - 1                     # 末端 ee_link = link(NJ-1)
assert NJ == 6, "期望 6 自由度机械臂"

# 关节限位：continuous 关节(lower>upper) 给 ±π
ll, ul, jr = [], [], []
for i in range(NJ):
    info = p.getJointInfo(arm, i)
    lo, hi = info[8], info[9]
    if lo > hi:
        lo, hi = -math.pi, math.pi
    ll.append(lo); ul.append(hi); jr.append(hi - lo)

# 初始姿态：各关节归零
for i in range(NJ):
    p.resetJointState(arm, i, 0.0)

# 关闭重力：位置控制可稳定收敛到 IK 姿态（零重力下不塌落）
p.setGravity(0, 0, 0)

# ---------------------------------------------------------------
# 相机：侧前方观察机械臂
# ---------------------------------------------------------------
W, H = 720, 720
view = p.computeViewMatrix(cameraEyePosition=[-0.7, -0.7, 0.65],
                           cameraTargetPosition=[0, 0, 0.32],
                           cameraUpVector=[0, 0, 1])
proj = p.computeProjectionMatrixFOV(fov=45, aspect=W / H, nearVal=0.05, farVal=10)

# ---------------------------------------------------------------
# 主循环：末端在 YZ 平面画圆
# ---------------------------------------------------------------
TOTAL = 720
RENDER_EVERY = 12
cx, cy, cz, r = 0.0, 0.0, 0.35, 0.12      # 圆心 (0,0,0.35)，半径 0.12
saved = 0
max_err = 0.0

# 热身：从初始姿态平滑移动到圆起点，消除起始跳变
prev_jp = [0.0] * NJ
warmup_target = [cx, cy + r, cz]
for _ in range(200):
    jointPoses = p.calculateInverseKinematics(arm, EE, warmup_target,
                                              [0, 0, 0, 1], ll, ul, jr, prev_jp)
    prev_jp = jointPoses
    for i in range(NJ):
        p.setJointMotorControl2(arm, i, p.POSITION_CONTROL,
                                targetPosition=jointPoses[i],
                                force=500, positionGain=0.10, velocityGain=1)
    p.stepSimulation()

for s in range(TOTAL):
    ang = s / TOTAL * 2 * math.pi
    target = [cx, cy + r * math.cos(ang), cz + r * math.sin(ang)]

    # 用上一步关节角作 restPose，抑制 IK 解跳变，保证运动连续平滑
    jointPoses = p.calculateInverseKinematics(arm, EE, target,
                                              [0, 0, 0, 1], ll, ul, jr, prev_jp)
    prev_jp = jointPoses
    for i in range(NJ):
        p.setJointMotorControl2(arm, i, p.POSITION_CONTROL,
                                targetPosition=jointPoses[i],
                                force=500, positionGain=0.10, velocityGain=1)
    p.stepSimulation()

    if s % RENDER_EVERY == 0:
        actual = p.getLinkState(arm, EE)[0]
        err = math.dist(target, actual)
        max_err = max(max_err, err)
        _, _, rgba, _, _ = p.getCameraImage(W, H, view, proj,
                                            renderer=p.ER_TINY_RENDERER)
        if isinstance(rgba, (bytes, tuple)):
            rgba = np.array(rgba, dtype=np.uint8).reshape(H, W, 4)
        Image.fromarray(rgba[:, :, :3], "RGB").save(
            os.path.join(FRAMES, "frame_%04d.png" % saved))
        saved += 1

p.disconnect()
print("== 6DOF 机械臂 IK 画圆完成 ==")
print("轨迹: YZ 平面, 圆心 (0, 0, 0.35), 半径 0.12 m")
print("渲染帧数: %d | 最大末端误差: %.3f m" % (saved, max_err))
print("帧目录: %s" % FRAMES)
