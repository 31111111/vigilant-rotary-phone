# -*- coding: utf-8 -*-
"""
error_analysis.py —— 视觉链路蒙特卡洛误差分析
============================================
在机械臂可达工作区内随机生成 N 个目标位置，每个位置完整走一遍：
  渲染单目标图 -> OpenCV 识别 -> 标定矩阵转世界坐标
记录三类误差并统计、画图：
  1) 识别中心像素误差 (px)
  2) 标定世界误差 (mm，标定板重投影)
  3) 视觉定位世界误差 (mm，最终用于 IK 的目标位置误差)
"""
import os
import json
import numpy as np
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import scene
import vision
import calibration
import arm_kinematics as arm

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
plt.rcParams["axes.unicode_minus"] = False

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "output")
N = 100


def render_single(xy, size, seed):
    img = scene.render(draw_targets=False, add_noise=True, seed=seed)
    scene.draw_block(img, tuple(xy), size, 0, (40, 40, 200))  # 红色目标
    return img


def main():
    rng = np.random.default_rng(7)
    px_err, world_err, calib_err, rs, positions = [], [], [], [], []

    for i in range(N):
        r = rng.uniform(0.06, 0.34)
        a = rng.uniform(-np.pi, np.pi)
        truth = np.array([r * np.cos(a), r * np.sin(a)])

        img = render_single(truth, 0.05, seed=100 + i)
        dets = vision.detect(img)
        red = next((d for d in dets if d["name"] == "red"), None)
        if red is None:
            continue

        cal = calibration.calibrate(verbose=False)
        M = cal["M_sim"]
        truth_px = scene.world_to_pixel([truth])[0]
        px_e = float(np.linalg.norm(np.array(red["center_px"]) - truth_px))
        est = calibration.pixel_to_world(M, red["center_px"])
        w_e = float(np.linalg.norm(est - truth) * 1000)

        px_err.append(px_e)
        world_err.append(w_e)
        calib_err.append(cal["err_sim_mm"])
        rs.append(r)
        positions.append(truth)

    px_err = np.array(px_err)
    world_err = np.array(world_err)
    calib_err = np.array(calib_err)
    positions = np.array(positions)

    def summ(x):
        return {"mean": float(np.mean(x)), "std": float(np.std(x)),
                "max": float(np.max(x)),
                "p95": float(np.percentile(x, 95))}

    summary = {"detection_px": summ(px_err),
               "world_position_mm": summ(world_err),
               "calibration_mm": summ(calib_err),
               "samples": len(px_err)}
    with open(os.path.join(OUT, "error_summary.json"), "w",
              encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    # ---------------- 画图 ----------------
    fig = plt.figure(figsize=(14, 5))
    ax1 = fig.add_subplot(1, 3, 1)
    ax1.hist(px_err, bins=20, color="#3b6fc0", alpha=0.85)
    ax1.set_title("识别中心像素误差 (px)")
    ax1.set_xlabel("像素"); ax1.set_ylabel("频次")

    ax2 = fig.add_subplot(1, 3, 2)
    ax2.hist(world_err, bins=20, color="#2e9e5b", alpha=0.85)
    ax2.set_title("视觉定位世界误差 (mm)")
    ax2.set_xlabel("mm")

    ax3 = fig.add_subplot(1, 3, 3)
    sc = ax3.scatter(positions[:, 0], positions[:, 1], c=world_err,
                     cmap="RdYlGn_r", s=42, edgecolor="k", linewidth=0.3)
    circ = plt.Circle((0, 0), arm.R_MAX, fill=False, ls="--", color="gray")
    ax3.add_patch(circ)
    ax3.set_aspect("equal")
    ax3.set_title("工作区误差空间分布 (mm)")
    ax3.set_xlabel("x (m)"); ax3.set_ylabel("y (m)")
    fig.colorbar(sc, ax=ax3, fraction=0.046)

    plt.tight_layout()
    fig.savefig(os.path.join(OUT, "05_error_analysis.png"), dpi=150)
    plt.close()

    print("样本数: %d" % len(px_err))
    print("识别像素误差  均值 %.3f | P95 %.3f | 最大 %.3f px"
          % (px_err.mean(), np.percentile(px_err, 95), px_err.max()))
    print("视觉定位误差  均值 %.3f | P95 %.3f | 最大 %.3f mm"
          % (world_err.mean(), np.percentile(world_err, 95), world_err.max()))
    print("标定重投影    均值 %.3f mm" % calib_err.mean())


if __name__ == "__main__":
    main()
