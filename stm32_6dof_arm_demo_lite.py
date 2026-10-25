# -*- coding: utf-8 -*-
"""
STM32 6自由度机械臂 · 逆运动学画圆（简化几何版，无 STL 依赖）
================================================================
使用基础 box/cylinder 构建的 stm32_arm/urdf/arm_simple.urdf（约 8KB），
无需任何二进制 STL 网格，clone 仓库即可直接运行：

    pip install pybullet numpy pillow
    python stm32_6dof_arm_demo_lite.py          # 离屏渲染并合成 mp4
    PB_GUI=1 python stm32_6dof_arm_demo_lite.py  # 带 GUI 实时观察

工作空间：该机械臂末端主要在 x≈0 的 YZ 平面运动，故在 YZ 平面画圆。

两点工程要点（均已在代码中落地）：
  1. 末端 link 的 inertial origin 设为 0。PyBullet 的 calculateInverseKinematics
     把目标施加到末端 link 的“惯性系原点(质心)”，而 getLinkState(ee)[0]
     返回 link 坐标系原点；二者不重合会产生等于质心偏移的常量位置误差。
  2. 纯位置 IK + 多起点 + 连续性选解 + 收紧收敛阈值：
     - 单位四元数全姿态约束会在圆上相当一部分点过约束不可达，画圆只要求
       末端位置，故不传 targetOrientation；
     - 多个初始姿态（构型）尝试，避免陷入错误构型族；
     - 在“位置足够精确”的候选中选关节变化最小的，保证轨迹平滑；
     - residualThreshold=1e-8 让 IK 充分收敛。
最终最大末端误差约 5~8mm（圆半径 120mm）。
"""
import os
import math
import subprocess
import numpy as np
import pybullet as p
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
FRAMES = os.path.join(HERE, "frames_lite")
os.makedirs(FRAMES, exist_ok=True)

# ---------------------------------------------------------------
# 0. 加载简化几何 URDF（纯 box/cylinder，无外部 mesh）
# ---------------------------------------------------------------
cid = p.connect(p.GUI if os.environ.get("PB_GUI") else p.DIRECT)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
arm = p.loadURDF(os.path.join(HERE, "stm32_arm", "urdf", "arm_simple.urdf"),
                 [0, 0, 0], useFixedBase=True)
NJ = p.getNumJoints(arm); EE = NJ - 1
assert NJ == 6, "期望 6 自由度，实际 %d" % NJ

ll, ul, jr = [], [], []
for i in range(NJ):
    info = p.getJointInfo(arm, i)
    lo, hi = info[8], info[9]
    if lo > hi:
        lo, hi = -math.pi, math.pi
    ll.append(lo); ul.append(hi); jr.append(hi - lo)
for i in range(NJ):
    p.resetJointState(arm, i, 0.0)

p.setGravity(0, 0, 0)
p.createMultiBody(0, p.createCollisionShape(p.GEOM_PLANE))

# ---------------------------------------------------------------
# 参考圆（红色标记点）+ 末端高亮光球
# ---------------------------------------------------------------
CX, CY, CZ, RR = 0.0, 0.0, 0.35, 0.12
red = p.createVisualShape(p.GEOM_SPHERE, radius=0.008,
                          rgbaColor=[0.9, 0.15, 0.12, 1])
for k in range(16):
    a = k / 16 * 2 * math.pi
    p.createMultiBody(0, baseVisualShapeIndex=red,
                      basePosition=[CX, CY + RR*math.cos(a),
                                    CZ + RR*math.sin(a)])
blue = p.createVisualShape(p.GEOM_SPHERE, radius=0.014,
                           rgbaColor=[0.15, 0.5, 0.95, 1])
tip = p.createMultiBody(0, baseVisualShapeIndex=blue)

# ---------------------------------------------------------------
# 相机
# ---------------------------------------------------------------
W, H = 720, 720
view = p.computeViewMatrix(cameraEyePosition=[-0.75, -0.75, 0.62],
                           cameraTargetPosition=[0, 0, 0.32],
                           cameraUpVector=[0, 0, 1])
proj = p.computeProjectionMatrixFOV(45, W / H, 0.05, 10)

def render_rgb():
    _, _, rgba, _, _ = p.getCameraImage(W, H, view, proj,
                                        renderer=p.ER_TINY_RENDERER)
    if isinstance(rgba, (bytes, tuple)):
        rgba = np.array(rgba, dtype=np.uint8).reshape(H, W, 4)
    return rgba[:, :, :3]

def show(q):
    """运动学置位：resetJointState 即时更新 FK 与渲染缓存；
    纯运动学动画不再 stepSimulation（避免默认电机把关节带离）。"""
    for i in range(NJ):
        p.resetJointState(arm, i, float(q[i]))
    p.resetBasePositionAndOrientation(tip, p.getLinkState(arm, EE)[0],
                                      [0, 0, 0, 1])

# ---------------------------------------------------------------
# 纯位置 IK + 多起点 + 连续性选解
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

def fk(q):
    show(q)
    return p.getLinkState(arm, EE)[0]

def solve_ik(target, cur):
    cur = list(cur)
    cands = []
    for seed in [cur] + SEEDS:
        jp = list(p.calculateInverseKinematics(
            arm, EE, target, ll, ul, jr, list(seed),
            maxNumIterations=200, residualThreshold=1e-8))
        cands.append((math.dist(target, fk(jp)), jp))
    good = [c for c in cands if c[0] <= POS_TOL]
    if good:
        return min(good, key=lambda c: sum((a-b)**2 for a, b in
                                           zip(c[1], cur)))[1]
    return min(cands, key=lambda c: c[0])[1]

# ---------------------------------------------------------------
# 离线求整圈关节序列
# ---------------------------------------------------------------
N_PTS = 120
N_LEAD = 30
cur = [0.0] * NJ
traj = []
for s in range(N_PTS):
    ang = s / N_PTS * 2 * math.pi
    target = [CX, CY + RR*math.cos(ang), CZ + RR*math.sin(ang)]
    cur = solve_ik(target, cur)
    traj.append((target, cur))

# 渲染
saved = 0
max_err = 0.0
frames = []

def grab():
    global saved
    rgb = render_rgb()
    Image.fromarray(rgb, "RGB").save(
        os.path.join(FRAMES, "frame_%04d.png" % saved))
    frames.append(rgb); saved += 1

# 过渡：零位 -> 圆起点（smoothstep 缓入）
start = [0.0] * NJ
for k in range(N_LEAD):
    t = (k + 1) / N_LEAD
    t = 0.5 - 0.5 * math.cos(math.pi * t)
    q = [start[i] + (traj[0][1][i] - start[i]) * t for i in range(NJ)]
    show(q); grab()

# 画圆
for target, jp in traj:
    show(jp)
    actual = p.getLinkState(arm, EE)[0]
    max_err = max(max_err, math.dist(target, actual))
    grab()

p.disconnect()

# 合成 mp4
mp4 = os.path.join(HERE, "stm32_6dof_circle_lite.mp4")
cmd = ["ffmpeg", "-y", "-framerate", "24",
       "-i", os.path.join(FRAMES, "frame_%04d.png"),
       "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", mp4]
try:
    subprocess.run(cmd, check=True, capture_output=True)
    video_ok = True
except Exception as e:
    video_ok = False
    print("ffmpeg 合成失败：", e)

print("== 6DOF 机械臂 IK 画圆（简化几何）完成 ==")
print("轨迹: YZ 平面, 圆心 (0,0,0.35), 半径 0.12 m")
print("渲染帧数: %d | 最大末端误差: %.4f m (%.1f mm)" %
      (saved, max_err, max_err * 1000))
if video_ok:
    print("视频:", mp4)
print("帧目录:", FRAMES)
