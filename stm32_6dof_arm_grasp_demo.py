# -*- coding: utf-8 -*-
"""
STM32 6自由度机械臂 · 抓取-搬运（简化几何版，无 STL 依赖）
================================================================
在 stm32_6dof_arm_demo_lite.py 同一套简化模型与多起点纯位置 IK 基础上，
让机械臂从左侧平台取起一个小方块，搬运到右侧平台放下。

实现要点：
  * 机械臂为运动学驱动：每帧 resetJointState 到 IK 轨迹关节角，
    再 stepSimulation 推动方块与约束、随后复位关节（纠正接触漂移）；
  * 方块是真实动力学刚体（受重力、与平台碰撞），抓取用
    JOINT_FIXED 固定约束“抓住”，松开 removeConstraint 后方块自然下落；
  * arm_simple.urdf 不含碰撞，方块不会与臂身相撞，抓取完全由约束完成，
    运动稳定；
  * 该臂工作空间末端为水平姿态（沿 ±Y），因此采用“水平对准→水平伸出
    夹住→竖直抬起→搬运→竖直放下→松开”的抓取流程。

运行：
    pip install pybullet numpy pillow
    python stm32_6dof_arm_grasp_demo.py
    PB_GUI=1 python stm32_6dof_arm_grasp_demo.py
"""
import os
import math
import subprocess
import numpy as np
import pybullet as p
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
FRAMES = os.path.join(HERE, "frames_6dof_grasp")
os.makedirs(FRAMES, exist_ok=True)

# ---------------------------------------------------------------
# 场景
# ---------------------------------------------------------------
p.connect(p.GUI if os.environ.get("PB_GUI") else p.DIRECT)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.setGravity(0, 0, -9.81)
p.createMultiBody(0, p.createCollisionShape(p.GEOM_PLANE))

arm = p.loadURDF(os.path.join(HERE, "stm32_arm", "urdf", "arm_simple.urdf"),
                 [0, 0, 0], useFixedBase=True)
NJ = p.getNumJoints(arm); EE = NJ - 1
ll, ul, jr = [], [], []
for i in range(NJ):
    info = p.getJointInfo(arm, i)
    lo, hi = info[8], info[9]
    if lo > hi:
        lo, hi = -math.pi, math.pi
    ll.append(lo); ul.append(hi); jr.append(hi - lo)
for i in range(NJ):
    p.resetJointState(arm, i, 0.0)

# 左右两个固定平台：柱 + 桌面，顶面高度 = 0.16
def make_table(x, y, color):
    top = 0.16
    leg = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.012, 0.012, top/2])
    plate = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.006])
    v_leg = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.012, 0.012, top/2],
                                rgbaColor=color)
    v_plate = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.006],
                                  rgbaColor=color)
    body = p.createMultiBody(
        baseMass=0, baseCollisionShapeIndex=leg,
        baseVisualShapeIndex=v_leg, basePosition=[x, y, top/2])
    p.createMultiBody(
        baseMass=0, baseCollisionShapeIndex=plate,
        baseVisualShapeIndex=v_plate, basePosition=[x, y, top + 0.006])
    return top
GREY = [0.35, 0.36, 0.39, 1]
GREY2 = [0.45, 0.46, 0.49, 1]
TOP_A = make_table(0.0, -0.20, GREY)
TOP_B = make_table(0.0,  0.15, GREY2)

# 方块（左平台顶面 + 半边长 0.015）
S = 0.03
cube_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[S/2]*3)
cube_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[S/2]*3,
                               rgbaColor=[0.10, 0.70, 0.25, 1])
GRASP = (0.0, -0.20, TOP_A + S/2)
PLACE = (0.0,  0.15, TOP_B + S/2)
cube = p.createMultiBody(
    baseMass=0.02, baseCollisionShapeIndex=cube_col,
    baseVisualShapeIndex=cube_vis, basePosition=list(GRASP))

# ---------------------------------------------------------------
# 相机
# ---------------------------------------------------------------
W, H = 720, 720
view = p.computeViewMatrix(cameraEyePosition=[-0.65, -0.85, 0.55],
                           cameraTargetPosition=[0, 0, 0.28],
                           cameraUpVector=[0, 0, 1])
proj = p.computeProjectionMatrixFOV(45, W / H, 0.05, 10)
def render():
    _, _, rgba, _, _ = p.getCameraImage(W, H, view, proj,
                                       renderer=p.ER_TINY_RENDERER)
    if isinstance(rgba, (bytes, tuple)):
        rgba = np.array(rgba, dtype=np.uint8).reshape(H, W, 4)
    return rgba[:, :, :3]

# ---------------------------------------------------------------
# 多起点纯位置 IK（与画圆 demo 同一套）
# ---------------------------------------------------------------
def RAD(x): return math.radians(x)
SEEDS = [
    [0, 0, 0, 0, 0, 0],
    [0, RAD(60), RAD(80), 0, RAD(-40), 0],
    [0, RAD(-30), RAD(60), 0, RAD(-20), 0],
    [0, RAD(60), RAD(140), 0, RAD(-70), 0],
    [0, RAD(-60), RAD(90), 0, RAD(-30), 0],
    [0, RAD(20), RAD(120), 0, RAD(-60), 0],
]
POS_TOL = 0.008

def set_q(q):
    for i in range(NJ):
        p.resetJointState(arm, i, float(q[i]))

def fk(q):
    set_q(q)
    return p.getLinkState(arm, EE)

def solve_ik(target, cur):
    cur = list(cur); cands = []
    for seed in [cur] + SEEDS:
        jp = list(p.calculateInverseKinematics(
            arm, EE, target, ll, ul, jr, list(seed),
            maxNumIterations=200, residualThreshold=1e-8))
        ls = fk(jp)
        cands.append((math.dist(target, ls[0]), jp))
    good = [c for c in cands if c[0] <= POS_TOL]
    if good:
        return min(good, key=lambda c: sum((a-b)**2 for a, b in
                                           zip(c[1], cur)))[1]
    return min(cands, key=lambda c: c[0])[1]

def ang_lerp(a, b, t):
    d = (b - a + math.pi) % (2*math.pi) - math.pi
    return a + d * t

def q_lerp(q1, q2, t):
    return [ang_lerp(q1[i], q2[i], t) for i in range(NJ)]

# ---------------------------------------------------------------
# 关键点位姿（末端空间点）
# ---------------------------------------------------------------
pre_A   = (0.0, -0.12, GRASP[2])          # 左平台水平对准（缩回）
lift_A  = (0.0, -0.20, 0.30)              # 左平台竖直抬起
over_B  = (0.0,  0.15, 0.30)              # 右平台上方
post_B  = (0.0,  0.07, PLACE[2])          # 右平台水平缩回
home    = (0.0,  0.0, 0.34)

q0 = [0.0] * NJ
q_home = solve_ik(home, q0)
q_preA = solve_ik(pre_A, q_home)
q_grasp = solve_ik(GRASP, q_preA)
q_liftA = solve_ik(lift_A, q_grasp)
q_overB = solve_ik(over_B, q_liftA)
q_place = solve_ik(PLACE, q_overB)
q_postB = solve_ik(post_B, q_place)

# 校验关键点精度
for nm, qq, pt in [("home", q_home, home), ("pre_A", q_preA, pre_A),
                   ("grasp", q_grasp, GRASP), ("lift_A", q_liftA, lift_A),
                   ("over_B", q_overB, over_B), ("place", q_place, PLACE),
                   ("post_B", q_postB, post_B)]:
    fk(qq)
    err = math.dist(pt, p.getLinkState(arm, EE)[0])
    print("  关键点 %-7s IK 误差 %.2f mm" % (nm, err*1000))

# ---------------------------------------------------------------
# 抓取约束
# ---------------------------------------------------------------
grip = None
def attach():
    global grip
    # 先把方块精确对齐到末端（中心 = ee 原点，姿态 = ee 姿态）
    ls = p.getLinkState(arm, EE)
    p.resetBasePositionAndOrientation(cube, ls[0], ls[1])
    for _ in range(5):
        p.stepSimulation()
    grip = p.createConstraint(
        arm, EE, cube, -1, p.JOINT_FIXED, [0, 0, 0],
        [0, 0, 0], [0, 0, 0], [0, 0, 0, 1], [0, 0, 0, 1])
    p.changeConstraint(grip, maxForce=200)

def detach():
    global grip
    if grip is not None:
        p.removeConstraint(grip); grip = None
    # 让方块拥有稳定的零速度
    p.resetBaseVelocity(cube, [0, 0, 0], [0, 0, 0])

frames = []
def step_and_grab(q, settle=4):
    set_q(q)
    for _ in range(settle):
        p.stepSimulation()
    set_q(q)              # 复位关节，保证运动学位置精确
    _, _, px, _, _ = p.getCameraImage(W, H, view, proj,
                                      renderer=p.ER_TINY_RENDERER)
    if isinstance(px, (bytes, tuple)):
        px = np.array(px, dtype=np.uint8).reshape(H, W, 4)
    frames.append(px[:, :, :3])

def move_to(q1, q2, n):
    for k in range(n):
        t = (k + 1) / n
        t = 0.5 - 0.5 * math.cos(math.pi * t)
        step_and_grab(q_lerp(q1, q2, t))

# 让方块先在左平台静止
for _ in range(60):
    p.stepSimulation()

# 起始零位先画几帧
for _ in range(10):
    step_and_grab(q0, settle=1)
# 零位 -> 高位 home
move_to(q0, q_home, 25)
# 水平对准左平台
move_to(q_home, q_preA, 25)
# 水平伸出，夹住方块
move_to(q_preA, q_grasp, 18)
attach()
# 竖直抬起
move_to(q_grasp, q_liftA, 22)
# 搬运到右平台上方
move_to(q_liftA, q_overB, 38)
# 竖直放下
move_to(q_overB, q_place, 20)
detach()
# 停顿几帧让方块落定
for _ in range(20):
    step_and_grab(q_place, settle=4)
# 水平缩回
move_to(q_place, q_postB, 20)
# 回到高位
move_to(q_postB, q_home, 28)

# 最终落定
for _ in range(40):
    p.stepSimulation()
final_pos = p.getBasePositionAndOrientation(cube)[0]
print("方块最终位置: (%.3f, %.3f, %.3f)" % final_pos)
print("目标平台中心: (0, 0.15, ~%.3f)" % (TOP_B + S/2))
dist_to_goal = math.dist(final_pos[:2], (0, 0.15))
print("水平偏离目标中心: %.1f mm" % (dist_to_goal*1000))

p.disconnect()

# 保存帧 + ffmpeg
for i, fr in enumerate(frames):
    Image.fromarray(fr, "RGB").save(
        os.path.join(FRAMES, "frame_%04d.png" % i))
mp4 = os.path.join(HERE, "stm32_6dof_grasp.mp4")
cmd = ["ffmpeg", "-y", "-framerate", "24",
       "-i", os.path.join(FRAMES, "frame_%04d.png"),
       "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", mp4]
try:
    subprocess.run(cmd, check=True, capture_output=True)
    print("视频:", mp4)
except Exception as e:
    print("ffmpeg 合成失败：", e)
print("总帧数:", len(frames))
