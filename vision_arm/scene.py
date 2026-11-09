# -*- coding: utf-8 -*-
"""
scene.py —— 俯视相机场景渲染器（eye-to-hand 数字孪生）
=====================================================
模拟一台固定在机械臂上方、俯视桌面的相机：
  * 世界坐标系：机械臂基座在原点 (0,0)，桌面为 XY 平面，单位【米】
  * 相机带真实内参（尺度 px/m、主点、绕光轴旋转角），世界->像素为带旋转的相似变换
  * 渲染桌面、基座、若干彩色目标方块（方块有世界坐标/尺寸/朝向），并可加噪声

这些相机参数是"地面真值"，标定模块 calibration.py 并不知道它们，
只能通过标定点估计，最后再与真值对比、量化标定误差。
"""
import numpy as np
import cv2

# ---------------- 画面与相机地面真值参数 ----------------
IMG_W, IMG_H = 640, 640
SCALE = 750.0          # 像素/米
CU, CV = 322.0, 318.0   # 主点（像素），故意偏离画面正中心
CAM_CENTER = (0.0, 0.0) # 主点对应的世界坐标（米）
CAM_ROT = np.deg2rad(-8.0)  # 相机绕光轴旋转（弧度），故意没完全对齐

# ---------------- 目标方块（世界坐标，米）----------------
# 每个目标：名称、中心(x,y)、边长、朝向角(度)、BGR
TARGETS = [
    {"name": "red",   "xy": (0.18, 0.12),  "size": 0.05, "ang_deg": 15,  "bgr": (40, 40, 200)},
    {"name": "green", "xy": (-0.16, 0.15), "size": 0.045, "ang_deg": -30, "bgr": (60, 180, 75)},
    {"name": "blue",  "xy": (0.10, -0.18), "size": 0.05, "ang_deg": 45,  "bgr": (200, 90, 30)},
    {"name": "yellow","xy": (-0.20, -0.12),"size": 0.04, "ang_deg": 0,   "bgr": (0, 210, 230)},
]


def world_to_pixel(pts_world):
    """世界点 (N,2) 米 -> 像素 (N,2)。带旋转的相似变换（相机地面真值）。"""
    pts = np.asarray(pts_world, dtype=np.float64)
    dx = pts[:, 0] - CAM_CENTER[0]
    dy = pts[:, 1] - CAM_CENTER[1]
    c, s = np.cos(CAM_ROT), np.sin(CAM_ROT)
    rx = c * dx - s * dy
    ry = s * dx + c * dy
    u = CU + SCALE * rx
    v = CV + SCALE * ry
    return np.stack([u, v], axis=1)


def _square_corners(t):
    """方块在世界系的 4 个角点。"""
    h = t["size"] / 2.0
    local = np.array([[-h, -h], [h, -h], [h, h], [-h, h]])
    a = np.deg2rad(t["ang_deg"])
    c, s = np.cos(a), np.sin(a)
    R = np.array([[c, -s], [s, c]])
    rot = local @ R.T
    return rot + np.array(t["xy"])


def draw_block(img, xy, size, ang_deg, bgr):
    """在指定世界位置画一个方块（投影四角后填充）。"""
    t = {"xy": xy, "size": size, "ang_deg": ang_deg}
    corners_px = world_to_pixel(_square_corners(t)).astype(np.int32)
    cv2.fillPoly(img, [corners_px], bgr)
    cv2.polylines(img, [corners_px], True, (30, 30, 30), 1)


def render(add_noise=True, seed=0, show_base=True, draw_targets=True):
    """渲染一张 BGR 俯视图。"""
    img = np.full((IMG_H, IMG_W, 3), 235, dtype=np.uint8)  # 浅灰桌面

    # 工作区网格（每 0.1m 一条），让画面更像真实桌面
    for g in np.arange(-0.3, 0.31, 0.1):
        p1 = world_to_pixel([[g, -0.32]])[0]
        p2 = world_to_pixel([[g, 0.32]])[0]
        cv2.line(img, tuple(p1.astype(int)), tuple(p2.astype(int)), (220, 220, 220), 1)
        p3 = world_to_pixel([[-0.32, g]])[0]
        p4 = world_to_pixel([[0.32, g]])[0]
        cv2.line(img, tuple(p3.astype(int)), tuple(p4.astype(int)), (220, 220, 220), 1)

    # 机械臂基座（中心圆 + 内圈）
    if show_base:
        c0 = world_to_pixel([[0.0, 0.0]])[0].astype(int)
        r_out = int(0.06 * SCALE)
        cv2.circle(img, tuple(c0), r_out, (90, 90, 90), -1)
        cv2.circle(img, tuple(c0), int(r_out * 0.45), (160, 160, 160), -1)

    # 目标方块：投影四角后填充，并画深色边
    if draw_targets:
        for t in TARGETS:
            draw_block(img, t["xy"], t["size"], t["ang_deg"], t["bgr"])

    if add_noise:
        rng = np.random.default_rng(seed)
        # 高斯传感器噪声
        noise = rng.normal(0, 6, img.shape).astype(np.int16)
        img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        # 轻微亮度不均（模拟光照）
        yy, xx = np.mgrid[0:IMG_H, 0:IMG_W]
        shade = 1.0 + 0.06 * ((xx - IMG_W / 2) / IMG_W)
        img = np.clip(img.astype(np.float64) * shade[..., None], 0, 255).astype(np.uint8)

    return img


if __name__ == "__main__":
    import os
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
    os.makedirs(out, exist_ok=True)
    img = render()
    # 标注地面真值中心，便于核对
    for t in TARGETS:
        p = world_to_pixel([t["xy"]])[0].astype(int)
        cv2.drawMarker(img, tuple(p), (0, 0, 0), cv2.MARKER_CROSS, 12, 1)
    cv2.imwrite(os.path.join(out, "01_scene_truth.png"), img)
    print("已保存场景图 output/01_scene_truth.png，目标数 =", len(TARGETS))
