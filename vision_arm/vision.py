# -*- coding: utf-8 -*-
"""
vision.py —— OpenCV 目标识别模块
================================
流程（复试高频考点全部覆盖）：
  BGR -> HSV 颜色空间 -> 颜色阈值分割 -> 形态学开/闭运算 ->
  轮廓检测 -> 面积过滤 -> 图像矩求中心 -> 最小外接旋转矩形求尺寸/朝向

输入 scene.py 渲染的 BGR 俯视图，输出每个目标的像素中心、面积、朝向等。
"""
import numpy as np
import cv2

# HSV 阈值（OpenCV: H 0-179, S/V 0-255）。红色跨 0/180 用两段。
COLOR_RANGES = {
    "red": [
        (np.array([0, 100, 80]),   np.array([10, 255, 255])),
        (np.array([170, 100, 80]), np.array([179, 255, 255])),
    ],
    "green":  [(np.array([40, 60, 60]),  np.array([85, 255, 255]))],
    "blue":   [(np.array([95, 60, 60]),  np.array([130, 255, 255]))],
    "yellow": [(np.array([18, 80, 120]), np.array([32, 255, 255]))],
}

MIN_AREA = 250          # 小于该像素面积的轮廓视为噪声
KERNEL = np.ones((5, 5), np.uint8)


def _mask_for(hsv, name):
    mask = None
    for lo, hi in COLOR_RANGES[name]:
        m = cv2.inRange(hsv, lo, hi)
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    # 开运算去椒盐噪点，闭运算填内部小孔
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL, iterations=2)
    return mask


def detect(bgr):
    """返回检测目标列表：name, center_px, area, rect(中心,wh,角度), 轮廓。"""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    detections = []
    for name in COLOR_RANGES:
        mask = _mask_for(hsv, name)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                      cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_AREA:
                continue
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            u = M["m10"] / M["m00"]
            v = M["m01"] / M["m00"]
            rect = cv2.minAreaRect(cnt)  # ((cx,cy),(w,h),angle)
            detections.append({
                "name": name,
                "center_px": (float(u), float(v)),
                "area": float(area),
                "rect": rect,
                "contour": cnt,
            })
    return detections


def draw_detections(bgr, dets):
    """把识别结果画到图上：轮廓、中心、名称、朝向。"""
    out = bgr.copy()
    for d in dets:
        (cx, cy), (w, h), ang = d["rect"]
        cv2.drawContours(out, [d["contour"]], -1, (255, 255, 255), 2)
        cv2.circle(out, (int(cx), int(cy)), 4, (0, 0, 0), -1)
        cv2.putText(out, "%s  %.1fdeg" % (d["name"], ang),
                    (int(cx) + 8, int(cy) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    return out


if __name__ == "__main__":
    import os
    import scene
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
    os.makedirs(out, exist_ok=True)

    img = scene.render()
    dets = detect(img)
    print("检测到 %d 个目标：" % len(dets))
    for d in dets:
        print("  %-6s 中心=(%.1f, %.1f) 面积=%.0f 朝向=%.1f°"
              % (d["name"], d["center_px"][0], d["center_px"][1],
                 d["area"], d["rect"][2]))

    vis = draw_detections(img, dets)
    cv2.imwrite(os.path.join(out, "02_detection.png"), vis)
    # 同时保存分割 mask，便于文档展示
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    masks = [_mask_for(hsv, n) for n in COLOR_RANGES]
    cv2.imwrite(os.path.join(out, "02_masks.png"),
                np.vstack([cv2.cvtColor(m, cv2.COLOR_GRAY2BGR) for m in masks]))
    print("已保存 output/02_detection.png 和 02_masks.png")
