# -*- coding: utf-8 -*-
"""
KUKA iiwa 机械臂 · 真实抓取-搬运任务仿真
基于官方 bullet3 仓库的 kuka_iiwa/model.urdf 模型与 PyBullet IK/约束接口。

任务流程（状态机）：
  1) approach : 末端运动到方块正上方
  2) grasp    : 末端下移到方块上方夹取位
  3) [创建固定约束，模拟夹爪夹紧]
  4) lift     : 抬升方块
  5) carry    : 搬运到目标点上方
  6) lower    : 下降到目标夹取位
  7) [移除约束，方块落下 = 松开]
  8) retract  : 末端收回

抓取模拟：用 createConstraint(p.JOINT_FIXED) 把方块刚性固定到末端 link6 下方，
视觉上等价于"夹住并抓走"。
"""
import os
import math
import numpy as np
import pybullet as p
import pybullet_data
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
FRAMES = os.path.join(HERE, "frames_kuka_grasp")
os.makedirs(FRAMES, exist_ok=True)

# ---------------------------------------------------------------
# 0. 初始化 + 加载场景
# ---------------------------------------------------------------
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", [0, 0, 0])            # 桌面
kuka = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
p.resetBasePositionAndOrientation(kuka, [0, 0, 0], [0, 0, 0, 1])

EE = 6                                   # 末端 link 索引
NJ = p.getNumJoints(kuka)                 # KUKA iiwa = 7 关节
assert NJ == 7, "期望 7 自由度机械臂"

# 关节上下限 / 范围 / 休息位（来自官方 inverse_kinematics.py）
ll = [-0.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]
ul = [0.967,  2,  2.96, 2.29,  2.96,  2.09,  3.05]
jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]
for i in range(NJ):
    p.resetJointState(kuka, i, rp[i])

# 重力：真实（抓取/放下才有意义）
p.setGravity(0, 0, -9.81)

# ---------------------------------------------------------------
# 方块：边长 0.06 m 的绿色小立方体，放在桌面上（区别于机械臂的橙色关节）
# ---------------------------------------------------------------
HALF = 0.03
PICK = np.array([0.45, 0.15, HALF])     # 拾取点（方块中心）
DROP = np.array([0.20, -0.45, HALF])    # 放置点（目标）
col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[HALF] * 3)
vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[HALF] * 3,
                          rgbaColor=[0.10, 0.85, 0.30, 1])
cube = p.createMultiBody(0.2, col, vis, basePosition=PICK)

# ---------------------------------------------------------------
# 相机：侧前方观察抓取区
# ---------------------------------------------------------------
W, H = 720, 720
view = p.computeViewMatrix(cameraEyePosition=[-0.45, -1.05, 1.15],
                           cameraTargetPosition=[0.33, -0.05, 0.28],
                           cameraUpVector=[0, 0, 1])
proj = p.computeProjectionMatrixFOV(fov=45, aspect=W / H, nearVal=0.05, farVal=10)

# ---------------------------------------------------------------
# 轨迹状态机
# ---------------------------------------------------------------
GRASP_Z = HALF + 0.035          # 夹取时末端高度（方块中心上方 0.035）
HIGH = 0.38                     # 抬升/搬运高度

# (阶段名, 终点坐标, 动作, 步数)  步数按位移量分配，保证位置控制收敛到位
phases = [
    ("approach", np.array([PICK[0], PICK[1], HIGH]), None, 240),
    ("grasp",    np.array([PICK[0], PICK[1], GRASP_Z]), "CREATE_CONSTRAINT", 180),
    ("lift",     np.array([PICK[0], PICK[1], HIGH]), None, 180),
    ("carry",    np.array([DROP[0], DROP[1], HIGH]), None, 360),
    ("lower",    np.array([DROP[0], DROP[1], GRASP_Z]), "REMOVE_CONSTRAINT", 240),
    ("retract",  np.array([DROP[0], DROP[1], HIGH]), None, 180),
]

RENDER_EVERY = 15               # 每 15 步渲染一帧
orn_down = p.getQuaternionFromEuler([0, -math.pi, 0])   # 末端朝下

constraint = None
saved = 0
start = None

def render_frame():
    """渲染一帧并保存（连续编号）"""
    global saved
    _, _, rgba, _, _ = p.getCameraImage(W, H, view, proj,
                                        renderer=p.ER_TINY_RENDERER)
    if isinstance(rgba, (bytes, tuple)):
        rgba = np.array(rgba, dtype=np.uint8).reshape(H, W, 4)
    Image.fromarray(rgba[:, :, :3], "RGB").save(
        os.path.join(FRAMES, "frame_%04d.png" % saved))
    saved += 1

print("== 抓取-搬运任务开始 ==")
print("拾取点 PICK: (%.2f, %.2f, %.3f) | 放置点 DROP: (%.2f, %.2f, %.3f)"
      % (*PICK, *DROP))

for name, end, action, steps in phases:
    start = np.array(p.getLinkState(kuka, EE)[0]) if start is None else start
    print("[阶段] %-8s -> 目标 (%.2f, %.2f, %.3f)  (%d 步)" % (name, *end, steps))
    for s in range(steps):
        frac = (s + 1) / steps
        target = start + (end - start) * frac
        jointPoses = p.calculateInverseKinematics(
            kuka, EE, list(target), orn_down, ll, ul, jr, rp)
        for i in range(NJ):
            p.setJointMotorControl2(kuka, i, p.POSITION_CONTROL,
                                    targetPosition=jointPoses[i],
                                    force=1000, positionGain=0.10, velocityGain=1)
        p.stepSimulation()
        if s % RENDER_EVERY == 0:
            render_frame()
    if action == "CREATE_CONSTRAINT":
        # 把方块中心固定到末端 link6 下方 0.035。
        # 注：link6 本地 z 轴朝下（orn=[0,-pi,0]），故本地 +0.035 -> 世界下方。
        constraint = p.createConstraint(kuka, EE, cube, -1, p.JOINT_FIXED,
                                        [0, 0, 0], [0, 0, 0.035], [0, 0, 0])
        print("   -> 夹紧：创建固定约束，方块已固定在末端")
    elif action == "REMOVE_CONSTRAINT":
        p.removeConstraint(constraint)
        constraint = None
        print("   -> 松开：移除约束，方块放下")
    start = end

# ---------------------------------------------------------------
# 结果
# ---------------------------------------------------------------
cube_pos = np.array(p.getBasePositionAndOrientation(cube)[0])
print("== 任务完成 ==")
print("方块最终位置: (%.3f, %.3f, %.3f) | 目标: (%.3f, %.3f, %.3f)"
      % (*cube_pos, *DROP))
print("落点偏差: %.3f m" % np.linalg.norm(cube_pos - DROP))
print("渲染帧数: %d | 帧目录: %s" % (saved, FRAMES))
p.disconnect()
