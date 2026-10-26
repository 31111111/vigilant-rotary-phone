#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
plot_ik.py —— 用 dump_data.c 生成的真实数据绘制 3 张图（面试/博客用）
  fig1_overview.png    三版本总体对比：正确/误选/无解 堆叠柱 + 逐点误差
  fig2_candidates.png  62 个失败点的候选分类：伪根 / 超限真解 / 双分支救活
  fig3_theta1_roots.png 根因机理：被误选 θ1 与正确 θ1 互为反号（y=360-x）
"""
import csv, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm

for fp in ["/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
           "/usr/share/fonts/truetype/SourceHanSans-VF.ttf",
           "/usr/share/fonts/truetype/SimHei.ttf"]:
    if os.path.exists(fp):
        try:
            fm.fontManager.addfont(fp)
            plt.rcParams["font.family"] = fm.FontProperties(fname=fp).get_name()
            break
        except Exception:
            continue
plt.rcParams["axes.unicode_minus"] = False

HERE = os.path.dirname(os.path.abspath(__file__))
def load_sel(n):
    return {int(r["i"]): (int(r["ret"]), float(r["sel_err_mm"]))
            for r in csv.DictReader(open(os.path.join(HERE, f"selected_{n}.csv")))}
def load_cand(n):
    d = {}
    for r in csv.DictReader(open(os.path.join(HERE, f"cand_{n}.csv"))):
        d.setdefault(int(r["i"]), []).append(
            (int(r["k"]), float(r["residual_mm"]), int(r["lim_ok"]), float(r["theta1_deg"])))
    return d

sel = {n: load_sel(n) for n in ["orig", "patch1", "patch2"]}
cand = {n: load_cand(n) for n in ["patch1", "patch2"]}
labels = ["原版固件", "patch1\n位置残差校验", "patch2\nθ1双分支"]
names = ["orig", "patch1", "patch2"]
C_OK, C_BAD, C_NO = "#2e9e5b", "#d64545", "#9aa0a6"

# ---------- 图1：总体对比 ----------
fig, (axL, axR) = plt.subplots(1, 2, figsize=(13, 5.2), gridspec_kw={"width_ratios": [1, 1.35]})
n_ok, n_bad, n_no = [], [], []
for n in names:
    rows = sel[n].values()
    ok = sum(1 for ret, e in rows if ret == 0 and e < 1)
    bad = sum(1 for ret, e in rows if ret == 0 and e >= 1)
    no = sum(1 for ret, e in rows if ret == -1)
    n_ok.append(ok); n_bad.append(bad); n_no.append(no)
x = np.arange(3)
axL.bar(x, n_ok, color=C_OK, label="正确（位置误差<1mm）")
axL.bar(x, n_bad, bottom=n_ok, color=C_BAD, label="误选伪根（位置错，仍执行）")
axL.bar(x, n_no, bottom=np.array(n_ok) + np.array(n_bad), color=C_NO, label="判定无解（拒绝执行）")
for i in range(3):
    if n_ok[i]: axL.text(i, n_ok[i] / 2, str(n_ok[i]), ha="center", va="center", color="white", fontweight="bold")
    if n_bad[i]: axL.text(i, n_ok[i] + n_bad[i] / 2, str(n_bad[i]), ha="center", va="center", color="white", fontweight="bold")
    if n_no[i]: axL.text(i, n_ok[i] + n_bad[i] + n_no[i] / 2, str(n_no[i]), ha="center", va="center", color="#333", fontweight="bold")
axL.set_xticks(x); axL.set_xticklabels(labels); axL.set_ylabel("目标点数（共 500）")
axL.set_ylim(0, 560)
axL.legend(loc="upper center", bbox_to_anchor=(0.5, -0.13), fontsize=9, framealpha=0.95)
axL.set_title("① 三版本选解结果：原版 43 点误选伪根\npatch1 正确拒绝但丢 62 点；patch2 全部 500 点正确", fontsize=11)

# 右：逐点误差（有解点按误差升序，无解点画在底部）
colors = {"orig": "#c98a00", "patch1": "#3b6fc0", "patch2": C_OK}
for ni, n in enumerate(names):
    pts = []
    for ret, e in sel[n].values():
        pts.append(e if ret == 0 else np.nan)
    noidx = [i for i, (ret, e) in enumerate(sel[n].values()) if ret == -1]
    sol = sorted([e for ret, e in sel[n].values() if ret == 0])
    xx = np.arange(len(sol))
    axR.plot(xx, sol, "o", ms=2.6, color=colors[n], label=["原版", "patch1", "patch2"][ni])
    if noidx:
        axR.plot(np.arange(len(sol), len(sol) + len(noidx)), np.full(len(noidx), 1.2e-4),
                 "x", ms=5, color=C_NO)
axR.axhline(1.0, color=C_BAD, ls="--", lw=1.2)
axR.text(505, 1.15, "1mm 伪根阈值", color=C_BAD, fontsize=9, ha="right")
axR.set_yscale("log"); axR.set_ylim(1e-4, 600)
axR.set_xlabel("目标点（按位置误差升序排列，× = 无解）"); axR.set_ylabel("选中解末端位置误差 (mm，对数)")
axR.set_title("② 逐点位置误差：原版最大 354mm，patch2 全部 ≤0.012mm", fontsize=11)
axR.legend(loc="upper left", fontsize=10); axR.grid(alpha=0.3, which="both")
plt.tight_layout()
plt.savefig(os.path.join(HERE, "fig1_overview.png"), dpi=150)
plt.close()

# ---------- 图2：62 失败点候选分类 ----------
fails = [i for i, (ret, e) in sel["patch1"].items() if ret == -1]
rng = np.random.default_rng(0)
def category(res, lim):
    # key 按 (是否限位内, 是否残差≈0) 判定
    inlim, good = (lim == 1), (res < 1)
    if inlim and good: return C_OK            # 绿：限位内真解（可执行）
    if inlim and not good: return C_BAD       # 红：限位内伪根（原版会误选）
    if (not inlim) and good: return "#3b6fc0" # 蓝：真解但超限位
    return "#bdbdbd"                          # 灰：超限且残差大
fig, axes = plt.subplots(2, 1, figsize=(11.5, 7.2), sharex=True)
for ax, n, title in zip(axes, ["patch1", "patch2"],
                        ["patch1（4 候选 / 点）：限位内的候选全是伪根（红），真解都超限位（蓝）",
                         "patch2（8 候选 / 点）：枚举 θ1 另一符号根后，新增限位内真解（绿），62 点全部救活"]):
    for i in fails:
        for k, res, lim, t1 in cand[n][i]:
            xv = max(res, 1e-3)
            yv = 1 if lim == 1 else 0
            yv += rng.uniform(-0.16, 0.16)
            ax.plot(xv, yv, "o", ms=5, color=category(res, lim), alpha=0.6, mew=0)
    ax.axvline(1.0, color=C_BAD, ls="--", lw=1.2)
    ax.set_yticks([0, 1]); ax.set_yticklabels(["限位外\n(超限)", "限位内\n(可执行)"])
    ax.set_ylim(-0.5, 1.5); ax.set_ylabel("候选解状态")
    ax.set_title(title, fontsize=11)
    ax.grid(alpha=0.3, axis="x")
axes[1].set_xscale("log")
axes[1].set_xlim(5e-4, 800)
axes[1].set_xlabel("候选解末端位置残差 (mm，对数；左端 1e-3 代表残差≈0)")
from matplotlib.lines import Line2D
handles = [
    Line2D([0],[0],marker="o",color="w",mfc=C_BAD,ms=10,label="限位内但残差大＝伪根（原版会误选）"),
    Line2D([0],[0],marker="o",color="w",mfc=C_OK,ms=10,label="限位内且残差≈0＝可执行真解"),
    Line2D([0],[0],marker="o",color="w",mfc="#3b6fc0",ms=10,label="残差≈0 但超限位（不可达）"),
    Line2D([0],[0],marker="o",color="w",mfc="#bdbdbd",ms=10,label="超限且残差大"),
]
axes[0].legend(handles=handles, loc="upper right", fontsize=9, framealpha=0.9)
plt.tight_layout()
plt.savefig(os.path.join(HERE, "fig2_candidates.png"), dpi=150)
plt.close()

# ---------- 图3：θ1 双符号根 ----------
bad_t1, good_t1 = [], []
for i in fails:
    in_bad = [t1 for k, res, lim, t1 in cand["patch1"][i] if lim == 1 and res >= 1]
    in_good = [t1 for k, res, lim, t1 in cand["patch2"][i] if lim == 1 and res < 1]
    if in_bad and in_good:
        bad_t1.append(in_bad[0]); good_t1.append(in_good[0])
fig, ax = plt.subplots(figsize=(6.8, 6.4))
ax.plot([0, 360], [360, 0], color="#3b6fc0", ls="--", lw=1.5, label="正确 θ1 = 360° − 被误选 θ1（即互为反号）")
ax.plot(bad_t1, good_t1, "o", color=C_BAD, ms=8, alpha=0.8,
        label=f"原版误选的 {len(bad_t1)} 个点（均唯一落在反号线上）")
ax.annotate("典型点：被误选 343.29°（=−16.71°）\n正确为 +16.71°",
            xy=(343.29, 16.71), xytext=(205, 118), fontsize=10,
            arrowprops=dict(arrowstyle="->", color="#333"),
            bbox=dict(boxstyle="round", fc="#fff7e6", ec="#c98a00"))
ax.set_xlabel("固件（patch1）限位内伪根给出的 θ1（°）")
ax.set_ylabel("patch2 救活真解的 θ1（°）")
ax.set_xlim(0, 360); ax.set_ylim(0, 360); ax.set_aspect("equal")
ax.set_title("根因：半角法漏算 θ1 另一符号根——\n原版误选点的正确 θ1 全部落在反号参考线上", fontsize=11)
ax.legend(loc="upper right", fontsize=9); ax.grid(alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(HERE, "fig3_theta1_roots.png"), dpi=150)
plt.close()
print("已生成 fig1_overview.png / fig2_candidates.png / fig3_theta1_roots.png")
