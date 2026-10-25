# -*- coding: utf-8 -*-
"""验证 arm_simple.urdf：
  1) 随机限位构型 FK->IK 自洽（检验末端 inertial origin 是否与测量点重合）
  2) 多起点纯位置 IK 画圆的最大位置误差与最大关节跳变
"""
import math, os
import numpy as np
import pybullet as p

HERE = os.path.dirname(os.path.abspath(__file__))
p.connect(p.DIRECT)
arm = p.loadURDF(os.path.join(HERE, "stm32_arm", "urdf", "arm_simple.urdf"),
                 [0, 0, 0], useFixedBase=True)
NJ = p.getNumJoints(arm); EE = NJ - 1
assert NJ == 6, NJ
ll, ul, jr = [], [], []
for i in range(NJ):
    info = p.getJointInfo(arm, i)
    lo, hi = info[8], info[9]
    if lo > hi:
        lo, hi = -math.pi, math.pi
    ll.append(lo); ul.append(hi); jr.append(hi - lo)

def fk(q):
    for i in range(NJ):
        p.resetJointState(arm, i, q[i])
    return p.getLinkState(arm, EE)[0]

# ---- 1) 随机构型 FK->IK 自洽 ----
rng = np.random.RandomState(0)
worst_self = 0.0
for _ in range(200):
    q = [float(rng.uniform(ll[i], ul[i])) for i in range(NJ)]
    pos = fk(q)
    qq = list(p.calculateInverseKinematics(arm, EE, pos, ll, ul, jr, q))
    back = fk(qq)
    worst_self = max(worst_self, math.dist(pos, back))
print("FK->IK 自洽最大误差: %.2e m" % worst_self)

# ---- 2) 多起点纯位置 IK 画圆 ----
def R(x): return math.radians(x)
SEEDS = [
    [0, 0, 0, 0, 0, 0],
    [0, R(60), R(80), 0, R(-40), 0],
    [0, R(-30), R(60), 0, R(-20), 0],
    [0, R(60), R(140), 0, R(-70), 0],
    [0, R(-60), R(90), 0, R(-30), 0],
    [0, R(20), R(120), 0, R(-60), 0],
]
TOL = 0.008

def solve(target, cur):
    cur = list(cur)
    cands = []
    for sd in [cur] + SEEDS:
        jp = list(p.calculateInverseKinematics(arm, EE, target,
                                               ll, ul, jr, list(sd),
                                               maxNumIterations=200,
                                               residualThreshold=1e-8))
        cands.append((math.dist(target, fk(jp)), jp))
    good = [c for c in cands if c[0] <= TOL]
    if good:
        return min(good, key=lambda c: sum((a-b)**2 for a, b in
                                           zip(c[1], cur)))[1]
    return min(cands, key=lambda c: c[0])[1]

N = 120; cx, cy, cz, r = 0.0, 0.0, 0.35, 0.12
cur = [0.0] * NJ
worst = 0.0; maxjump = 0.0; over = []
prev = None
for s in range(N):
    a = s / N * 2 * math.pi
    tgt = [cx, cy + r*math.cos(a), cz + r*math.sin(a)]
    cur = solve(tgt, cur)
    e = math.dist(tgt, fk(cur))
    worst = max(worst, e)
    if e > 0.01:
        over.append((s, round(e, 4)))
    if prev is not None:
        maxjump = max(maxjump, max(abs(cur[i]-prev[i]) for i in range(NJ)))
    prev = cur
print("画圆最大位置误差: %.5f m (%.2f mm)" % (worst, worst*1000))
print("最大单步关节跳变: %.3f°" % math.degrees(maxjump))
print("超 1cm 的点:", over)
p.disconnect()
