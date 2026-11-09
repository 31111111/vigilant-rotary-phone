# -*- coding: utf-8 -*-
"""
main.py —— 视觉引导机械臂抓取闭环（eye-to-hand）
================================================
完整链路：
  渲染俯视场景 -> OpenCV 识别目标像素中心 -> 标定矩阵转机械臂基坐标 ->
  平面两连杆 IK 解算 -> 逐帧动作（对准/下探/夹取/抬起/搬运/下放/松开/退回）
  -> 合成 mp4，并统计全链路误差

关键：机械臂去的是"视觉估计"的位置（不是直接知道真值），因此最终误差
真实反映了 识别 + 标定 + 运动学 整条链路的精度。
"""
import os
import json
import numpy as np
import cv2

import scene
import vision
import calibration
import arm_kinematics as arm

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "output")
os.makedirs(OUT, exist_ok=True)

TARGET_NAME = "red"
PLACE_XY = np.array([-0.06, -0.22])
GRIP_OPEN, GRIP_CLOSE = 0.012, 0.002


# ---------------- 1. 视觉感知 ----------------
def perceive():
    img = scene.render(seed=2)
    dets = vision.detect(img)
    cal = calibration.calibrate(verbose=False)
    M = cal["M_sim"]

    found = None
    for d in dets:
        w = calibration.pixel_to_world(M, d["center_px"])
        if d["name"] == TARGET_NAME:
            found = (d, w)
    assert found is not None, "未找到目标 %s" % TARGET_NAME
    d, world_est = found
    truth = np.array(next(t["xy"] for t in scene.TARGETS if t["name"] == TARGET_NAME))
    perception_err = float(np.linalg.norm(world_est - truth))
    return world_est, truth, perception_err


# ---------------- 2. 绘制 ----------------
def draw_gripper(img, wrist, direction, grip):
    d = direction / (np.linalg.norm(direction) + 1e-9)
    n = np.array([-d[1], d[0]])
    for s in (-1, 1):
        base = wrist + n * grip * s
        tip = base + d * 0.02
        p1 = scene.world_to_pixel([base])[0].astype(int)
        p2 = scene.world_to_pixel([tip])[0].astype(int)
        cv2.line(img, tuple(p1), tuple(p2), (20, 20, 20), 3)


def render_frame(sol, grip, stage, zlabel, target_state, extra):
    img = scene.render(draw_targets=False, add_noise=True, seed=3)

    # 画所有目标
    for t in scene.TARGETS:
        name = t["name"]
        if name in target_state:
            xy = target_state[name]
            if xy is not None:
                scene.draw_block(img, tuple(xy), t["size"], t["ang_deg"], t["bgr"])

    # 机械臂
    pts = arm.fk_points(sol)
    ppx = scene.world_to_pixel(pts).astype(int)
    cv2.polylines(img, [ppx], False, (60, 70, 90), 8)
    for p in ppx:
        cv2.circle(img, tuple(p), 5, (200, 200, 200), -1)

    # 夹爪（被抓目标画在腕点）
    wrist, elbow = pts[2], pts[1]
    draw_gripper(img, wrist, wrist - elbow, grip)

    # 状态文字（用英文避免 OpenCV 中文乱码）
    cv2.rectangle(img, (0, 0), (330, 86), (255, 255, 255), -1)
    cv2.putText(img, "STAGE: %s" % stage, (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
    cv2.putText(img, "Z: %s   GRIP: %s" %
                (zlabel, "CLOSE" if grip < 0.006 else "OPEN"),
                (8, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 2)
    cv2.putText(img, extra, (8, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 200), 1)
    return img


# ---------------- 3. 动作序列 ----------------
def run():
    world_est, truth, perr = perceive()

    sol_home = arm.ik(0.0, 0.06)
    sol_target = arm.ik(world_est[0], world_est[1])
    sol_place = arm.ik(PLACE_XY[0], PLACE_XY[1])
    for s in (sol_home, sol_target, sol_place):
        assert s["reachable"], "存在不可达关键点"

    # (关节解, 夹爪, 阶段, z, 帧数, 是否抓取/释放事件)
    seq = [
        (sol_home,   GRIP_OPEN,  "HOME",     "HIGH", 10, None),
        (sol_target, GRIP_OPEN,  "APPROACH", "HIGH", 22, None),
        (sol_target, GRIP_OPEN,  "DESCEND",  "LOW",  14, None),
        (sol_target, GRIP_CLOSE, "GRASP",    "LOW",  10, "grab"),
        (sol_target, GRIP_CLOSE, "LIFT",     "HIGH", 14, None),
        (sol_place,  GRIP_CLOSE, "TRANSFER", "HIGH", 24, None),
        (sol_place,  GRIP_CLOSE, "PLACE",    "LOW",  14, None),
        (sol_place,  GRIP_OPEN,  "RELEASE",  "LOW",  10, "release"),
        (sol_home,   GRIP_OPEN,  "RETRACT",  "HIGH", 20, None),
    ]

    target_state = {t["name"]: np.array(t["xy"]) for t in scene.TARGETS}
    frames = []
    prev_sol, prev_grip = seq[0][0], seq[0][1]
    extra = "vis err %.1f mm" % (perr * 1000)

    for sol, grip, stage, zlabel, n, event in seq:
        for k in range(n):
            t = (k + 1) / n
            cur_sol = arm.interpolate(prev_sol, sol, t)
            cur_grip = prev_grip + (grip - prev_grip) * t

            # 被抓目标跟随腕点
            if TARGET_NAME in target_state and stage in \
                    ("GRASP", "LIFT", "TRANSFER", "PLACE"):
                target_state[TARGET_NAME] = arm.fk_points(cur_sol)[2]

            fr = render_frame(cur_sol, cur_grip, stage, zlabel,
                              target_state, extra)
            frames.append(fr)
        if event == "grab":
            target_state[TARGET_NAME] = arm.fk_points(sol)[2]
        if event == "release":
            target_state[TARGET_NAME] = PLACE_XY
        prev_sol, prev_grip = sol, grip

    # 放置落点误差
    place_err = float(np.linalg.norm(target_state[TARGET_NAME] - PLACE_XY))
    # 抓取对准误差：视觉估计 vs 真值
    grasp_align = float(np.linalg.norm(world_est - truth))

    metrics = {
        "perception_position_err_mm": perr * 1000,
        "grasp_alignment_err_mm": grasp_align * 1000,
        "place_landing_err_mm": place_err * 1000,
        "frames": len(frames),
    }

    # 保存
    cv2.imwrite(os.path.join(OUT, "04_vision_guided_final.png"), frames[-1])
    cv2.imwrite(os.path.join(OUT, "04_vision_guided_grasp.png"),
                frames[len(frames) // 3])
    with open(os.path.join(OUT, "metrics.json"), "w", encoding="utf-8") as f:
        json.dump(metrics, f, ensure_ascii=False, indent=2)

    import imageio
    mp4 = os.path.join(HERE, "vision_guided_pickplace.mp4")
    w = imageio.get_writer(mp4, fps=20, codec="libx264",
                           quality=8, macro_block_size=16)
    for fr in frames:
        w.append_data(cv2.cvtColor(fr, cv2.COLOR_BGR2RGB))
    w.close()
    return metrics, mp4


if __name__ == "__main__":
    metrics, mp4 = run()
    print("== 视觉引导抓取完成 ==")
    for k, v in metrics.items():
        print("  %s: %s" % (k, v))
    print("视频:", mp4)
