"""Draw the Modified DH frames of both arms from dh_params.json.

    python dh_plot.py              -> figures/dh_frames.png (LA | RA), figures/dh_frames_<side>.png
    python dh_plot.py --q home     same at the model's rest pose (q4 = +-90 deg)

Frames are recomputed from the DH table itself (T_base_0, then
RotX(alpha) TransX(a) RotZ(q + theta_off) TransZ(d) per row), so the picture is
also a check of the table: it shows what the numbers say, not the model.
Only x (common normal, red) and z (joint axis, blue) are drawn - y follows from
them - because the axes of one joint group share an origin (shoulder {1,2},
elbow {3,4}, wrist {5,6,7}) and a full triad per frame would pile up.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
OUT = HERE / "figures"

INK, MUTED, GRID = "#1f2430", "#6b7280", "#d7dbe2"
X_COL, Z_COL, LINK = "#d1453b", "#2f6fd6", "#9aa3b2"
HOME = {"RA": [0, 0, 0, 90, 0, 0, 0], "LA": [0, 0, 0, -90, 0, 0, 0]}


def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1.]])


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.]])


def D(x=0.0, z=0.0):
    t = np.eye(4)
    t[0, 3], t[2, 3] = x, z
    return t


def frames(arm, q_deg):
    T0 = np.eye(4)
    T0[:3, :3] = np.array(arm["T_base_0"]["R"])
    T0[:3, 3] = arm["T_base_0"]["p_mm"]
    out, T = [("0", T0)], T0
    for i, r in enumerate(arm["rows"]):
        q = np.radians(q_deg[i]) if i < 7 else 0.0
        T = T @ Rx(np.radians(r["alpha_deg"])) @ D(x=r["a_mm"]) @ Rz(q + np.radians(r["theta_off_deg"])) @ D(z=r["d_mm"])
        out.append((str(r["i"]) if r["i"] != "tool" else "T", T))
    return out


def style(ax, center, half, labels=True):
    ax.set_xlim(center[0] - half, center[0] + half)
    ax.set_ylim(center[1] - half, center[1] + half)
    ax.set_zlim(center[2] - half, center[2] + half)
    ax.set_box_aspect((1, 1, 1))
    if labels:
        ax.set_xlabel("x 앞", color=MUTED, fontsize=7, labelpad=-8)
        ax.set_ylabel("y 왼쪽", color=MUTED, fontsize=7, labelpad=-8)
        ax.set_zlabel("z 위", color=MUTED, fontsize=7, labelpad=-8)
    ax.tick_params(colors=MUTED, labelsize=6, pad=-3)
    for a in (ax.xaxis, ax.yaxis, ax.zaxis):
        a.pane.set_facecolor((1, 1, 1, 0))
        a.pane.set_edgecolor(GRID)
        a._axinfo["grid"]["color"] = GRID
    ax.view_init(elev=20, azim=-58)


def overview(ax, F):
    org = np.array([f[1][:3, 3] for f in F])
    ax.plot(*np.vstack([[0, 0, 0], org]).T, color=LINK, lw=3, solid_capstyle="round")
    ax.scatter(*org.T, s=18, color=INK, depthshade=False)
    ax.scatter(0, 0, 0, s=18, color=MUTED, depthshade=False)
    ax.text(0, 0, -25, "base", color=MUTED, fontsize=8, ha="center", va="top")
    for key, names in groups(F).items():
        ax.text(key[0], key[1] + 40, key[2], "{" + ",".join(names) + "}", color=INK, fontsize=8, fontweight="bold")
    pts = np.vstack([org, [[0, 0, 0]]])
    style(ax, (pts.max(0) + pts.min(0)) / 2, (pts.max(0) - pts.min(0)).max() / 2 + 40)


def groups(F):
    g = {}
    for name, T in F:
        g.setdefault(tuple(np.round(T[:3, 3], 1)), []).append(name)
    return g


def zoom(ax, F, keep, L=55.0):
    """x and z axes of the frames in `keep`, around their common region."""
    sel = [(n, T) for n, T in F if n in keep]
    org = np.array([T[:3, 3] for _, T in sel])
    c = (org.max(0) + org.min(0)) / 2
    half = max((org.max(0) - org.min(0)).max() / 2, 0) + L * 1.35
    allo = np.array([T[:3, 3] for _, T in F])
    for a_, b_ in zip(allo[:-1], allo[1:]):          # link segments, clipped to the panel box
        seg = clip(a_, b_, c - half, c + half)
        if seg is not None:
            ax.plot(*np.array(seg).T, color=LINK, lw=3, solid_capstyle="round", alpha=0.6)
    ax.scatter(*org.T, s=24, color=INK, depthshade=False, zorder=3)
    tips = []
    for name, T in sel:
        p = T[:3, 3]
        lab = "t" if name == "T" else name
        for k, col, a in ((0, X_COL, "x"), (2, Z_COL, "z")):
            if name == "T" and k == 2:
                continue
            v = T[:3, k] * L
            ax.quiver(*p, *v, color=col, lw=1.8, arrow_length_ratio=0.15)
            tip = p + v * 1.18
            # nudge labels that land on top of an earlier one
            for t in tips:
                if np.linalg.norm(tip - t) < 12:
                    tip = tip + np.array([0, 14.0, 6.0])
            tips.append(tip)
            ax.text(*tip, f"{a}{lab}", color=col, fontsize=9, fontweight="bold", ha="center", va="center")
    for key, names in groups(sel).items():
        txt = "{" + ",".join("tool" if n == "T" else n for n in names) + "}"
        ax.text(key[0], key[1], key[2] - 16, txt, color=INK, fontsize=9, ha="center", va="top")
    style(ax, c, half)


def clip(a, b, lo, hi):
    """Liang-Barsky: the part of segment a-b inside the box [lo, hi], or None."""
    t0, t1, d = 0.0, 1.0, b - a
    for k in range(3):
        for p_, q_ in ((-d[k], a[k] - lo[k]), (d[k], hi[k] - a[k])):
            if abs(p_) < 1e-12:
                if q_ < 0:
                    return None
                continue
            t = q_ / p_
            if p_ < 0:
                t0 = max(t0, t)
            else:
                t1 = min(t1, t)
    return None if t0 > t1 else (a + t0 * d, a + t1 * d)


def table(ax, arm, side):
    ax.axis("off")
    rows = [[str(r["i"]), f"{r['alpha_deg']:g}", f"{r['a_mm']:g}", f"{r['d_mm']:g}", f"{r['theta_off_deg']:g}"]
            for r in arm["rows"]]
    t = ax.table(cellText=rows, colLabels=["i", "α(i-1) [°]", "a(i-1) [mm]", "d(i) [mm]", "θ_off(i) [°]"],
                 loc="upper center", cellLoc="center", colLoc="center")
    t.auto_set_font_size(False)
    t.set_fontsize(9)
    t.scale(1, 1.25)
    for (r, c), cell in t.get_celld().items():
        cell.set_edgecolor(GRID)
        cell.get_text().set_color(INK)
        if r == 0:
            cell.set_facecolor("#eef1f6")
            cell.get_text().set_fontweight("bold")


def main() -> int:
    for fam in ("Malgun Gothic", "AppleGothic", "NanumGothic"):
        if any(fam in f.name for f in matplotlib.font_manager.fontManager.ttflist):
            plt.rcParams["font.family"] = fam
            break
    plt.rcParams["axes.unicode_minus"] = False
    home = "--q" in sys.argv and sys.argv[sys.argv.index("--q") + 1] == "home"
    dh = json.loads((HERE / "dh_params.json").read_text(encoding="utf-8"))
    OUT.mkdir(exist_ok=True)
    tag = "home" if home else "q0"
    pose_txt = "초기자세 (q4 = ±90°)" if home else "q = 0 (조립자세)"

    fig = plt.figure(figsize=(17, 13.2), facecolor="white")
    gs = fig.add_gridspec(4, 5, height_ratios=[1, 0.62, 1, 0.62], width_ratios=[1.15, 1, 1, 1, 0.05],
                          hspace=0.12, wspace=0.02)
    parts = [("어깨", ["0", "1", "2"]), ("팔꿈치", ["3", "4"]), ("손목 · 툴", ["5", "6", "7", "T"])]
    for r, side in enumerate(("LA", "RA")):
        q = HOME[side] if home else [0] * 7
        F = frames(dh[side], q)
        name = "좌완 LA" if side == "LA" else "우완 RA"
        ax = fig.add_subplot(gs[2 * r, 0], projection="3d")
        overview(ax, F)
        ax.set_title(f"{name} — 전체, {pose_txt}", color=INK, fontsize=12, fontweight="bold", pad=0)
        for c, (pname, keep) in enumerate(parts):
            ax = fig.add_subplot(gs[2 * r, c + 1], projection="3d")
            zoom(ax, F, keep)
            ax.set_title(f"{name} {pname} " + "{" + ",".join("tool" if k == "T" else k for k in keep) + "}",
                         color=INK, fontsize=10, pad=0)
        table(fig.add_subplot(gs[2 * r + 1, :4]), dh[side], side)
    fig.text(0.5, 0.012,
             "x_i: 공통수선 (빨강), z_i: 관절축 (파랑), y_i = z_i × x_i (생략).  "
             "T = RotX(α) · TransX(a) · RotZ(q + θ_off) · TransZ(d).  프레임은 DH 표에서 다시 계산.  "
             "출처: HumanoidUpperBody.rmd (2026-10-06)",
             ha="center", va="bottom", color=MUTED, fontsize=9)
    fig.subplots_adjust(left=0.01, right=0.99, top=0.97, bottom=0.045)
    path = OUT / f"dh_frames_{tag}.png"
    fig.savefig(path, dpi=130)
    print("wrote", path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
