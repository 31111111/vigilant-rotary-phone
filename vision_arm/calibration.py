# -*- coding: utf-8 -*-
"""
calibration.py —— eye-to-hand 手眼标定（像素坐标 -> 机械臂基坐标）
=================================================================
真实工程里在桌面放一块标定板（棋盘格），检测角点的像素坐标，且每个角点
世界坐标已知，用这些对应点拟合变换。这里用规则网格模拟标定板：

  世界点 --(scene 的真实相机)--> 像素点 --(+ 角点检测噪声)--> 观测像素点
  再由 (观测像素, 世界) 对应点拟合：
     1) 最小二乘相似变换（自己实现，4 参数：尺度/旋转/平移，便于讲原理）
     2) cv2.estimateAffine2D（6 自由度仿射）
     3) cv2.findHomography（8 自由度透视）

最后反推估计内参，与 scene 真值对比，给出重投影误差与世界坐标误差。
"""
import os
import numpy as np
import cv2
import scene


def make_calib_points(step=0.05, rng_noise=0.3, seed=1):
    """生成标定板对应点：世界坐标、理想像素、带检测噪声的观测像素。"""
    rng = np.random.default_rng(seed)
    world, pix_obs = [], []
    grid = np.arange(-0.25, 0.251, step)
    for x in grid:
        for y in grid:
            w = np.array([x, y])
            p = scene.world_to_pixel([w])[0]
            p = p + rng.normal(0, rng_noise, 2)   # 模拟角点亚像素检测误差
            world.append(w)
            pix_obs.append(p)
    return np.array(world), np.array(pix_obs)


def fit_similarity(src_px, dst_world):
    """最小二乘求 像素->世界 相似变换：world = a*px + b。
    用齐次线性最小二乘一次性解出 a(2x2) 与 b(2)。"""
    N = src_px.shape[0]
    A = np.zeros((2 * N, 6))
    y = np.zeros(2 * N)
    for i, (p, w) in enumerate(zip(src_px, dst_world)):
        u, v = p
        A[2 * i] = [u, v, 0, 0, 1, 0]
        A[2 * i + 1] = [0, 0, u, v, 0, 1]
        y[2 * i] = w[0]
        y[2 * i + 1] = w[1]
    sol, *_ = np.linalg.lstsq(A, y, rcond=None)
    a = sol[:4].reshape(2, 2)
    b = sol[4:6]
    M = np.hstack([a, b.reshape(2, 1)])
    return M


def apply_M(M, pts):
    """对 (N,2) 点应用 2x3 仿射矩阵。"""
    ones = np.ones((pts.shape[0], 1))
    ph = np.hstack([pts, ones])
    return (M @ ph.T).T


def recover_params(M):
    """从 像素->世界 矩阵反推估计的尺度、旋转角、主点。"""
    a = M[:, :2]
    # 世界 = a(像素-主点)，a ≈ (1/s) R(-θ)
    scale_est = 1.0 / np.sqrt(abs(np.linalg.det(a)))
    rot_est = -np.arctan2(a[1, 0], a[0, 0])
    # 主点：像素主点映射到世界相机中心，解 a·(c-camcenter)=0 思路：
    # 用 scene 世界相机中心(0,0) 反求主点：0 = a c + b  => c = -a^-1 b
    c = -np.linalg.solve(a, M[:, 2])
    return scale_est, rot_est, c


def calibrate(verbose=True):
    world, pix = make_calib_points()

    M_sim = fit_similarity(pix, world)
    M_aff, _ = cv2.estimateAffine2D(pix, world, method=cv2.LMEDS)
    H, _ = cv2.findHomography(pix, world, method=cv2.LMEDS)

    # 世界坐标误差（米）
    pred_sim = apply_M(M_sim, pix)
    pred_aff = apply_M(M_aff, pix)
    pred_hom = cv2.perspectiveTransform(pix.reshape(-1, 1, 2), H).reshape(-1, 2)

    def stats(pred):
        e = np.linalg.norm(pred - world, axis=1)
        return e.mean(), e.max()

    s_sim = stats(pred_sim)
    s_aff = stats(pred_aff)
    s_hom = stats(pred_hom)

    scale_est, rot_est, c_est = recover_params(M_sim)

    if verbose:
        print("标定板点数: %d" % len(world))
        print("世界坐标误差（mm）  平均/最大：")
        print("  相似变换(4参): %.4f / %.4f" % (s_sim[0] * 1000, s_sim[1] * 1000))
        print("  仿射(6参):     %.4f / %.4f" % (s_aff[0] * 1000, s_aff[1] * 1000))
        print("  单应(8参):     %.4f / %.4f" % (s_hom[0] * 1000, s_hom[1] * 1000))
        print("--- 内参估计 vs 真值 ---")
        print("  尺度: 估计 %.2f px/m | 真值 %.2f" % (scale_est, scene.SCALE))
        print("  旋转: 估计 %.3f° | 真值 %.3f°"
              % (np.rad2deg(rot_est), np.rad2deg(scene.CAM_ROT)))
        print("  主点: 估计 (%.1f, %.1f) | 真值 (%.1f, %.1f)"
              % (c_est[0], c_est[1], scene.CU, scene.CV))

    return {"M_sim": M_sim, "M_aff": M_aff, "H": H,
            "err_sim_mm": s_sim[0] * 1000, "err_aff_mm": s_aff[0] * 1000,
            "world": world, "pix": pix}


def pixel_to_world(M, center_px):
    """单个像素点 -> 世界坐标（米）。"""
    return apply_M(M, np.array([center_px], dtype=np.float64))[0]


def _visualize(cal):
    """画标定板点与误差向量。"""
    img = scene.render(add_noise=False)
    for p, w in zip(cal["pix"], cal["world"]):
        cv2.circle(img, tuple(p.astype(int)), 2, (0, 0, 200), -1)
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
    cv2.imwrite(os.path.join(out, "03_calibration.png"), img)


if __name__ == "__main__":
    cal = calibrate()
    _visualize(cal)
    print("已保存 output/03_calibration.png")
