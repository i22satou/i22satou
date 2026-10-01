#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
卒業論文の第1章・第2章で使う概念図(原理の説明図)を生成する。

実測データ(センサーログCSV)は使わない。粒子の位置や軌跡は、乱数(シード固定)や
決めた値で置いた説明用のもので、実験結果ではない。実測データから作る図は
thesis_figures.py の担当。図1.2だけは、建物の二値地図と、本体(pdr_pf_improved.py)
の経路帯の作り方をそのまま使う(図5.1と同じ自動抽出)。

図2.1は本体の地図座標に合わせ、xは右向き、yは画像と同じく下向き、方位θはx軸の
正の向きから測る(北向きが-90度になる。図8.2と同じ)。

出力先は figures/ 直下。実行:
    python3 concept_figures.py
"""
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import japanize_matplotlib  # noqa: F401  (日本語フォント登録)
import matplotlib.pyplot as plt
from matplotlib.patches import Arc, FancyArrowPatch, Rectangle

HERE = Path(__file__).resolve().parent

C_POINT = "#1f4e79"   # 推定位置・生き残った粒子
C_STEP = "#1f77b4"    # 1歩の変位・移動後の粒子
C_ZERO = "#d62728"    # 重み0の粒子
C_OLD = "#9e9e9e"     # 過去の位置・移動前の粒子
C_WALL = "#6d6d6d"    # 壁
C_ANGLE = "#b5460a"   # 方位


def save(fig, stem):
    out = HERE / f"{stem}.png"
    fig.savefig(out, dpi=200, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"保存: {out.name}")


def arrow(ax, p, q, color, lw=1.8, ms=14, z=3):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle="-|>", mutation_scale=ms,
                                 color=color, lw=lw, zorder=z, shrinkA=0, shrinkB=0))


# ---------------------------------------------------------------------------
# 図2.1 PDRによる位置更新(式(2.1))
# ---------------------------------------------------------------------------
def fig_pdr_update():
    # 説明用の歩幅[m]と方位[度]。最後の1歩を歩kとして詳しく描く。
    lengths = [1.0, 0.95, 1.05, 1.6]
    headings = [-15.0, -5.0, 10.0, 35.0]
    pts = [np.array([0.0, 0.0])]
    for l, h in zip(lengths, headings):
        r = np.deg2rad(h)
        pts.append(pts[-1] + l * np.array([np.cos(r), np.sin(r)]))

    fig, ax = plt.subplots(figsize=(8.2, 4.2))
    ax.set_aspect("equal")

    # 歩k-1までの軌跡
    for a, b in zip(pts[:-2], pts[1:-1]):
        arrow(ax, a, b, C_OLD, lw=1.4, ms=11)
    for p in pts[:-2]:
        ax.plot(*p, "o", color=C_OLD, ms=6, zorder=4)
    ax.annotate("$(x_0,\\,y_0)$\n既知の出発点", pts[0], textcoords="offset points",
                xytext=(0, 10), ha="center", va="bottom", fontsize=10, color="#555555")

    # 歩k
    p0, p1 = pts[-2], pts[-1]
    th_deg = headings[-1]
    arrow(ax, p0, p1, C_STEP, lw=2.4, ms=16, z=5)
    ax.plot(*p0, "o", color=C_POINT, ms=8, zorder=6)
    ax.plot(*p1, "o", color=C_POINT, ms=8, zorder=6)
    ax.annotate("$(x_{k-1},\\,y_{k-1})$", p0, textcoords="offset points",
                xytext=(-8, -10), ha="right", va="top", fontsize=11, color=C_POINT)
    ax.annotate("$(x_k,\\,y_k)$", p1, textcoords="offset points",
                xytext=(10, 0), ha="left", va="center", fontsize=11, color=C_POINT)
    mid = (p0 + p1) / 2
    ax.annotate("歩幅 $l_k$", mid, textcoords="offset points", xytext=(-10, -4),
                ha="right", va="top", fontsize=11, color=C_STEP)

    # x軸の正の向きの基準線と方位θk(yは下向きなので、表示上は時計回りに測る)
    ref_end = p0 + np.array([2.0, 0.0])
    ax.plot([p0[0], ref_end[0]], [p0[1], p0[1]], "--", color="#555555", lw=1.0, zorder=2)
    ax.text(ref_end[0] + 0.05, p0[1], "x軸の正の向き", fontsize=9, va="center",
            color="#555555")
    ax.add_patch(Arc(p0, 1.0, 1.0, theta1=0, theta2=th_deg, color=C_ANGLE, lw=1.6))
    a = np.deg2rad(th_deg / 2)
    ax.text(p0[0] + 0.62 * np.cos(a), p0[1] + 0.62 * np.sin(a), "$\\theta_k$",
            fontsize=12, color=C_ANGLE, ha="left", va="center")

    # 成分 l_k cosθ_k と l_k sinθ_k
    corner = np.array([p1[0], p0[1]])
    ax.plot([corner[0], p1[0]], [corner[1], p1[1]], ":", color="#555555", lw=1.3)
    ax.text(p1[0] + 0.06, (p0[1] + p1[1]) / 2, "$l_k \\sin\\theta_k$",
            fontsize=10, va="center", color="#555555")
    ax.text((p0[0] + corner[0]) / 2, p0[1] - 0.07, "$l_k \\cos\\theta_k$",
            fontsize=10, ha="center", va="bottom", color="#555555")

    # 座標軸の向き(yは下向き)
    o = np.array([-0.2, 0.75])
    arrow(ax, o, o + [0.5, 0], "black", lw=1.2, ms=10)
    arrow(ax, o, o + [0, 0.5], "black", lw=1.2, ms=10)
    ax.text(o[0] + 0.56, o[1], "$x$", fontsize=11, va="center")
    ax.text(o[0] + 0.06, o[1] + 0.5, "$y$", fontsize=11, ha="left", va="center")

    ax.set_xlim(-0.45, 6.4)
    ax.set_ylim(-0.75, 1.75)
    ax.invert_yaxis()  # 地図座標(yは下向き)
    ax.axis("off")
    save(fig, "fig2_1_pdr_update")


# ---------------------------------------------------------------------------
# 図2.2 パーティクルフィルタの1回の更新(予測・重み計算・リサンプリング)
# ---------------------------------------------------------------------------
CORR_W = 2.0     # 廊下の幅(0 <= y <= CORR_W)
WALL_T = 0.3     # 壁の厚さ
X_MAX = 6.0


def draw_corridor(ax, title):
    ax.add_patch(Rectangle((0, -WALL_T), X_MAX, WALL_T, color=C_WALL, zorder=1))
    ax.add_patch(Rectangle((0, CORR_W), X_MAX, WALL_T, color=C_WALL, zorder=1))
    ax.text(0.12, CORR_W / 2, "廊下", fontsize=10, color="#777777", ha="left",
            va="center")
    ax.text(X_MAX - 0.1, -WALL_T - 0.45, "壁の向こう側", fontsize=9, color="#777777",
            ha="right", va="center")
    ax.set_xlim(0, X_MAX)
    ax.set_ylim(CORR_W + WALL_T + 0.75, -WALL_T - 0.9)  # yは下向き
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_title(title, fontsize=10.5, loc="left")


def outside(p):
    return p[1] < 0.0 or p[1] > CORR_W


def fig_pf_update():
    rng = np.random.default_rng(284)  # 粒子が重なりにくい配置になるシード
    n = 14
    old = np.column_stack([rng.normal(1.2, 0.25, n), rng.normal(1.1, 0.3, n)])
    old[:, 1] = np.clip(old[:, 1], 0.3, 1.8)
    # 推定した移動(説明のため数歩分の長さにしている)を、やや-y(北)寄りに進める。
    step, head = 2.8, np.deg2rad(-14.0)
    ls = step + rng.normal(0, 0.3, n)
    hs = head + rng.normal(0, np.deg2rad(11.0), n)
    new = old + np.column_stack([ls * np.cos(hs), ls * np.sin(hs)])
    # 廊下は直線なので、移動の線分が壁と交差すること = 終点が廊下の外にあること
    dead = np.array([outside(q) for q in new])
    alive = ~dead
    w = alive / alive.sum()
    idx = rng.choice(n, size=n, p=w)
    counts = np.bincount(idx, minlength=n)

    fig, axes = plt.subplots(1, 3, figsize=(14, 3.6))

    # (a) 予測
    ax = axes[0]
    draw_corridor(ax, "(a) 予測：各粒子を、推定した歩幅・方位に\n     ノイズを加えた量だけ進める")
    for p, q in zip(old, new):
        ax.plot([p[0], q[0]], [p[1], q[1]], "-", color="#c0c0c0", lw=0.8, zorder=2)
    ax.plot(old[:, 0], old[:, 1], "o", mfc="white", mec=C_OLD, ms=6, zorder=3,
            label="移動前")
    ax.plot(new[:, 0], new[:, 1], "o", color=C_STEP, ms=6, zorder=4, label="移動後")
    ax.legend(loc="lower right", fontsize=8.5, framealpha=0.95, ncol=2,
              bbox_to_anchor=(1.0, -0.02))

    # (b) 重み計算
    ax = axes[1]
    draw_corridor(ax, "(b) 重み計算：壁を横切った粒子の\n     重みを0にする")
    for p, q, d in zip(old, new, dead):
        if d:
            ax.plot([p[0], q[0]], [p[1], q[1]], "--", color=C_ZERO, lw=0.8, alpha=0.6,
                    zorder=2)
    ax.plot(new[alive, 0], new[alive, 1], "o", color=C_STEP, ms=6, zorder=4,
            label=f"重み 1/{alive.sum()}")
    ax.plot(new[dead, 0], new[dead, 1], "x", color=C_ZERO, ms=8, mew=2, zorder=4,
            label="重み 0")
    ax.legend(loc="lower right", fontsize=8.5, framealpha=0.95, ncol=2,
              bbox_to_anchor=(1.0, -0.02))

    # (c) リサンプリング
    ax = axes[2]
    draw_corridor(ax, "(c) リサンプリング：重みに比例した確率で\n     粒子を選び直す")
    for i in np.flatnonzero(counts):
        ax.plot(*new[i], "o", color=C_POINT, ms=5 + 2.0 * counts[i], alpha=0.85, zorder=4)
        if counts[i] >= 2:
            ax.annotate(f"×{counts[i]}", new[i], textcoords="offset points",
                        xytext=(6 + 1.2 * counts[i], 0), va="center", fontsize=9, color=C_POINT, zorder=5)
    ax.text(X_MAX - 0.1, CORR_W + WALL_T + 0.4, f"粒子数は{n}個のまま(×は重なった個数)",
            fontsize=8.5, color="#333333", ha="right", va="center")

    fig.tight_layout(w_pad=1.2)
    save(fig, "fig2_2_pf_update")


# ---------------------------------------------------------------------------
# 図1.1 PDRの累積誤差(方位が一定量ずれた場合)
# ---------------------------------------------------------------------------
def fig_cumulative_error():
    length, half_w, bias_deg = 50.0, 1.5, 3.0   # 廊下の長さ・半幅[m]、方位のずれ[度]
    t = np.tan(np.deg2rad(bias_deg))
    x = np.linspace(0, length, 200)
    y_pdr = x * t
    x_cross = half_w / t

    fig, ax = plt.subplots(figsize=(10, 3.4))
    ax.add_patch(Rectangle((0, half_w), length, 0.35, color=C_WALL, zorder=1))
    ax.add_patch(Rectangle((0, -half_w - 0.35), length, 0.35, color=C_WALL, zorder=1))
    ax.text(0.3, half_w + 0.75, "壁の向こう側", fontsize=9, color="#777777",
            ha="left", va="center")

    ax.plot([0, length], [0, 0], "-", color="#2e7d32", lw=2.2, zorder=3,
            label="実際に歩いた経路")
    ax.plot(x, y_pdr, "--", color=C_ZERO, lw=2.2, zorder=3,
            label=f"PDRの推定(方位が{bias_deg:g}度ずれた場合)")
    ax.plot(0, 0, "o", color="black", ms=7, zorder=5)
    ax.text(0.3, -0.35, "出発点", fontsize=10, va="top")

    for d in (10, 20, 30, 40, 50):
        e = d * np.sin(np.deg2rad(bias_deg))
        xd = d * np.cos(np.deg2rad(bias_deg))
        ax.annotate("", xy=(xd, e), xytext=(xd, 0),
                    arrowprops=dict(arrowstyle="<->", color="#555555", lw=1.0))
        ax.text(xd + 0.4, e / 2, f"{e:.1f} m", fontsize=9, color="#555555", va="center")
        ax.text(xd, -0.25, f"{d} m", fontsize=8.5, color="#2e7d32", ha="center", va="top")

    ax.plot(x_cross, half_w, "o", mfc="none", mec=C_ZERO, ms=12, mew=1.8, zorder=5)
    ax.annotate("推定が壁を通り抜ける", (x_cross, half_w), textcoords="offset points",
                xytext=(-12, 22), ha="right", fontsize=9.5, color=C_ZERO,
                arrowprops=dict(arrowstyle="-", color=C_ZERO, lw=0.8))

    ax.set_xlim(-1, length + 4)
    ax.set_ylim(-half_w - 0.6, half_w + 1.4)
    ax.axis("off")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.02), ncol=2, fontsize=9.5,
              frameon=False)
    save(fig, "fig1_1_cumulative_error")


# ---------------------------------------------------------------------------
# 図1.2 地図の与え方の違い(人が描いた経路と、地図から自動抽出した経路帯)
# ---------------------------------------------------------------------------
def fig_route_input():
    import sys
    from scipy import ndimage
    from PIL import Image
    sys.path.insert(0, str(HERE.parent / "pdr_program"))
    import pdr_pf_improved as P  # noqa: E402

    cfg = HERE.parent / "pdr_program" / "map_configs" / "kanri_4f.json"
    _map_config, args = P.load_map_config_for_tool(str(cfg))
    binary, binary_for_pf, _ = P.load_preprocessed_map(args.map)
    auto_mask = P.extract_auto_route_mask(
        binary_for_pf, P.AUTO_ROUTE_MAX_HALF_WIDTH_PX, P.AUTO_ROUTE_DILATION_PX,
        exclude_wide_rooms=P.AUTO_ROUTE_EXCLUDE_WIDE_ROOMS,
        exclude_wide_rooms_radius_px=P.AUTO_ROUTE_EXCLUDE_WIDE_ROOMS_RADIUS_PX,
    )
    # 手動経路の帯は、本体の build_route_mask() と同じく折れ線を ROUTE_WIDTH_PX で膨らませる
    pts = [tuple(p) for p in P.ROUTE_POINTS]
    line = np.zeros(auto_mask.shape, dtype=bool)
    for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
        c = max(2, int(np.hypot(x1 - x0, y1 - y0)) + 1)
        xs = np.rint(np.linspace(x0, x1, c)).astype(int)
        ys = np.rint(np.linspace(y0, y1, c)).astype(int)
        line[ys, xs] = True
    r = max(1, int(round(P.ROUTE_WIDTH_PX)))
    yy, xx = np.ogrid[-r:r + 1, -r:r + 1]
    manual_mask = ndimage.binary_dilation(line, structure=(xx * xx + yy * yy <= r * r))

    img = np.array(Image.open(args.map).convert("L"))
    fig, axes = plt.subplots(2, 1, figsize=(10, 7.6))
    panels = [
        (axes[0], manual_mask, "(a) 人が地図上に経路を描いて与える場合(比較条件)",
         "建物や歩く経路が変わるたびに、人が描き直す必要がある"),
        (axes[1], auto_mask, "(b) 本研究：二値地図から通路の領域(経路帯)を自動で抽出する",
         "地図だけから求めるため、建物ごとの人手の作業が要らない"),
    ]
    for ax, mask, title, note in panels:
        ax.imshow(img, cmap="gray", interpolation="nearest")
        overlay = np.zeros((*mask.shape, 4))
        overlay[mask] = (1.0, 0.75, 0.0, 0.55)
        ax.imshow(overlay, interpolation="nearest")
        if mask is manual_mask:
            ax.plot([p[0] for p in pts], [p[1] for p in pts], "-", color="#d62728", lw=1.6)
        ax.set_title(title, fontsize=11, loc="left")
        ax.text(0.99, -0.03, note, transform=ax.transAxes, fontsize=9.5, ha="right",
                va="top", color="#333333")
        ax.axis("off")
    fig.tight_layout(h_pad=2.0)
    save(fig, "fig1_2_route_input")


# ---------------------------------------------------------------------------
# 図2.3 移動様態による粒子の散らし方の違い(移動様態PF論文[6]の考え方)
# ---------------------------------------------------------------------------
def fig_behavior_spread():
    from scipy.stats import norm
    rng = np.random.default_rng(7)
    step = 2.0                      # 次の更新までの移動量(説明用)
    cases = [
        ("(a) 直進：方位のノイズを小さく、粒子数を少なく(10個)", 10, 0.0, 5.0),
        ("(b) 屈折(曲がり)：方位のノイズを大きく、粒子数を多く(20個)", 20, 90.0, 22.0),
    ]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.4))
    for ax, (title, n, head_deg, sig_deg) in zip(axes, cases):
        ax.set_aspect("equal")
        # 直前までの軌跡(左から右へ歩いてきた)
        arrow(ax, (-2.6, 0), (0, 0), C_OLD, lw=1.8, ms=12)
        ax.plot(0, 0, "o", color=C_POINT, ms=8, zorder=6)
        ax.text(-1.3, -0.25, "これまでの軌跡", fontsize=9, color="#777777", ha="center",
                va="top")
        # 推定した方位(y上向きの通常の向きで描く)。粒子より先まで伸ばして見えるようにする。
        h = np.deg2rad(head_deg)
        tip = 1.3 * step * np.array([np.cos(h), np.sin(h)])
        ax.plot([0, tip[0]], [0, tip[1]], "--", color=C_ANGLE, lw=1.4, zorder=5)
        # 方位のノイズは、正規分布の分位点で左右対称に並べる(散らばりの幅を見せるため)
        q = norm.ppf((np.arange(n) + 0.5) / n)
        hs = h + np.deg2rad(sig_deg * q)
        ls = step + rng.normal(0, 0.08, n)
        pts = np.column_stack([ls * np.cos(hs), ls * np.sin(hs)])
        for p in pts:
            ax.plot([0, p[0]], [0, p[1]], "-", color="#c6d7ea", lw=0.8, zorder=2)
        ax.plot(pts[:, 0], pts[:, 1], "o", color=C_STEP, ms=6, zorder=4)
        ax.set_title(title, fontsize=10.5, loc="left")
        ax.set_xlim(-2.9, 3.9)
        ax.set_ylim(-1.0, 3.0)
        ax.axis("off")
    axes[0].text(2.65, 0.12, "推定した方位", fontsize=9, color=C_ANGLE, ha="left",
                 va="bottom")
    axes[1].text(0.1, 2.6, "推定した方位", fontsize=9, color=C_ANGLE, ha="left",
                 va="bottom")
    fig.tight_layout(w_pad=2.0)
    save(fig, "fig2_3_behavior_spread")


if __name__ == "__main__":
    fig_cumulative_error()
    fig_route_input()
    fig_pdr_update()
    fig_pf_update()
    fig_behavior_spread()
