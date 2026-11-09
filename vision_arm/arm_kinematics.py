# -*- coding: utf-8 -*-
"""
arm_kinematics.py —— 平面两连杆机械臂（圆柱坐标抓取）
=====================================================
俯视桌面抓取的经典结构：
  q1 底座旋转 φ（对准目标方位）
  q2 大臂肩角、q3 小臂肘角（在底座旋转后的竖直平面内，控制水平伸展距离 r）
  夹爪升降 z 与开合单独按动作序列控制

平面两连杆逆运动学（机器人学必考）：目标在底座旋转后的局部 x 轴上、距离 r，
用余弦定理求肘角，再求肩角；含肘上/肘下两个分支与可达性判断。
"""
import numpy as np

L1 = 0.20    # 大臂长度（m）
L2 = 0.18    # 小臂长度（m）
R_MIN = abs(L1 - L2)
R_MAX = L1 + L2


def ik(x, y, elbow="up"):
    """世界目标 (x,y) -> 关节角。返回 dict（角度均为弧度）。"""
    r = float(np.hypot(x, y))
    phi = float(np.arctan2(y, x))
    if r < R_MIN or r > R_MAX:
        return {"reachable": False, "phi": phi, "r": r,
                "theta1": 0.0, "theta2": 0.0}

    c2 = (r * r - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    c2 = float(np.clip(c2, -1.0, 1.0))
    theta2 = np.arccos(c2)          # 肘内角
    if elbow == "down":
        theta2 = -theta2
    # 目标在局部 x 轴上（y=0）
    theta1 = -np.arctan2(L2 * np.sin(theta2), L1 + L2 * np.cos(theta2))
    return {"reachable": True, "phi": phi, "r": r,
            "theta1": float(theta1), "theta2": float(theta2)}


def fk_points(sol):
    """由关节解返回俯视绘制点：[基座, 肘, 腕]（世界坐标 xy）。"""
    phi, t1, t2 = sol["phi"], sol["theta1"], sol["theta2"]
    base = np.array([0.0, 0.0])
    elbow = L1 * np.array([np.cos(phi + t1), np.sin(phi + t1)])
    wrist = elbow + L2 * np.array([np.cos(phi + t1 + t2),
                                  np.sin(phi + t1 + t2)])
    return np.vstack([base, elbow, wrist])


def interpolate(sol_a, sol_b, t):
    """两个关节解之间线性插值（用于动画平滑）。"""
    def ang(a, b):
        d = b - a
        d = (d + np.pi) % (2 * np.pi) - np.pi   # 走最短角路径
        return a + d * t
    return {
        "reachable": True,
        "phi": ang(sol_a["phi"], sol_b["phi"]),
        "theta1": sol_a["theta1"] + (sol_b["theta1"] - sol_a["theta1"]) * t,
        "theta2": sol_a["theta2"] + (sol_b["theta2"] - sol_a["theta2"]) * t,
        "r": sol_a.get("r", 0) + (sol_b.get("r", 0) - sol_a.get("r", 0)) * t,
    }


if __name__ == "__main__":
    # 自洽验证：随机目标 IK 后 FK 末端应回到目标
    rng = np.random.default_rng(0)
    errs = []
    for _ in range(200):
        r = rng.uniform(R_MIN + 0.01, R_MAX - 0.01)
        a = rng.uniform(-np.pi, np.pi)
        x, y = r * np.cos(a), r * np.sin(a)
        sol = ik(x, y)
        wrist = fk_points(sol)[-1]
        errs.append(np.hypot(wrist[0] - x, wrist[1] - y))
    print("两连杆 IK->FK 自洽最大误差: %.2e m" % np.max(errs))
    print("工作半径范围: %.2f ~ %.2f m" % (R_MIN, R_MAX))
