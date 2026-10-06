"""中間発表ポスター(A0)用の図を、大きな文字・高解像度で作る。

使うデータは既存の実測2本(pdr_log_0805_1441/1442、調整用データで評価用データではない)。
作図の部品は figures/make_figures_20260924.py と本体(pdr_pf_improved.py)の関数をそのまま使う。
図の幅は、ポスターの1段(約38cm=15インチ)に原寸で貼る前提。

実行(i22satou/ で):
    python 中間発表/図/make_poster_figures.py
"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "figures"))
sys.path.insert(0, str(ROOT / "pdr_program" / "evaluation"))

import make_figures_20260924 as mf  # noqa: E402  (本体・地図設定もここで読み込まれる)
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402
from verify_route_graph import load_route_mask  # noqa: E402

p = mf.p
DPI = 250
FS = 24  # 図中の基本の文字サイズ(原寸で貼ったときのpt)
FILES = [("pdr_log_0805_1441.csv", "記録1441"), ("pdr_log_0805_1442.csv", "記録1442")]


def poster_style(ax):
    mf.style(ax)
    ax.tick_params(labelsize=FS - 4, width=1.2, length=6)


def figure_route_band(out):
    """二値地図と、自動抽出した経路帯(黄色)だけを描く。"""
    binary, route_mask = load_route_mask(mf.PROG / "map_configs" / "kanri_4f.json")
    fig, ax = plt.subplots(figsize=(15, 6))
    ax.imshow(np.where(binary == 255, 1, 0), cmap=ListedColormap(["#3a3a3a", "#ffffff"]),
              interpolation="nearest", vmin=0, vmax=1)
    overlay = np.zeros((*route_mask.shape, 4))
    overlay[route_mask] = (0.95, 0.76, 0.19, 0.75)
    ax.imshow(overlay, interpolation="nearest")
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)
    ax.legend(handles=[Patch(color="#f2c230", label="自動抽出した経路帯"),
                       Patch(facecolor="#3a3a3a", label="壁・部屋の外"),
                       Patch(facecolor="white", edgecolor="#999999", label="歩ける領域")],
              loc="upper center", bbox_to_anchor=(0.5, -0.02), ncol=3, frameon=False, fontsize=FS - 2)
    fig.savefig(out, dpi=DPI, bbox_inches="tight", pad_inches=0.05)
    plt.close(fig)


def figure_trajectories(out, seed=42):
    """1441・1442の4方式の推定軌跡(seed=42の1回分)を上下に並べる。"""
    binary, _pf, _dist = p.load_preprocessed_map(mf.resolved.map)
    corners, starts, _scale = mf.route_geometry()
    runs = pd.read_csv(mf.EVAL_1001 / "results_long.csv", encoding="utf-8-sig")
    fig, axes = plt.subplots(2, 1, figsize=(15, 13.5))
    for ax, (name, label) in zip(axes, FILES):
        start = starts.loc[name, ["start_x", "start_y"]].to_numpy(float)
        mf.map_panel(ax, binary, start, corners)
        for key, color, linestyle, _lb in mf.TRAJ_STYLE:
            row = runs[(runs["method_key"] == key) & (runs["file"] == name)
                       & ((runs["seed"] == seed) | runs["seed"].isna())].iloc[0]
            t = mf.load_trajectory(mf.EVAL_1001, row)
            ax.plot(np.r_[start[0], t["x_px"]], np.r_[start[1], t["y_px"]], color=color,
                    linestyle=linestyle, linewidth=4.5 if key == "E_proposed" else 3.2, zorder=4 if key == "E_proposed" else 3)
        ax.set_title(label, fontsize=FS + 4, color=mf.INK, loc="left")
    handles = [Line2D([], [], color=c, linestyle=ls, linewidth=4, label=lb) for _k, c, ls, lb in mf.TRAJ_STYLE]
    extra = [Line2D([], [], color=mf.ROUTE, linewidth=16, alpha=0.45, label="歩いた経路(申告)"),
             Line2D([], [], linestyle="none", marker="s", markersize=14, color=mf.INK, label="開始位置"),
             Line2D([], [], linestyle="none", marker="*", markersize=24, color=mf.ROUTE,
                    markeredgecolor=mf.INK, label="終点(申告)")]
    fig.legend(handles=handles + extra, loc="lower center", ncol=4, frameon=False, fontsize=FS - 2,
               bbox_to_anchor=(0.5, -0.01), columnspacing=1.2, handlelength=2.4)
    fig.tight_layout(rect=(0, 0.08, 1, 1))
    fig.savefig(out, dpi=DPI, bbox_inches="tight", pad_inches=0.05)
    plt.close(fig)


def figure_behavior(out):
    """直近1.5秒のヨーレート75%値と、歩ごとの判定(直進/曲がり)。"""
    enter = np.rad2deg(p.TURN_YAW_RATE_THRESHOLD)
    exit_ = np.rad2deg(p.TURN_EXIT_YAW_RATE_THRESHOLD)
    fig = plt.figure(figsize=(15, 12.5))
    gs = fig.add_gridspec(5, 1, height_ratios=[3, 0.45, 1.05, 3, 0.45], hspace=0.06)
    rows = [(gs[0], gs[1]), (gs[3], gs[4])]
    for (g_sc, g_bar), (name, label) in zip(rows, FILES):
        df, t, steps = mf.load(name)
        states, p75 = mf.behavior_sequence(df, t, steps)
        ts = t[steps] - t[steps[0]]
        turning = np.array([s == p.MoveBehavior.TURNING for s in states])
        ax = fig.add_subplot(g_sc)
        bar = fig.add_subplot(g_bar, sharex=ax)
        ax.scatter(ts, p75, s=70, color=mf.INK, zorder=3, linewidths=0)
        ax.axhline(enter, color=mf.ORANGE, linestyle="--", linewidth=3, label=f"曲がり開始の条件({enter:g}度/秒以上)")
        ax.axhline(exit_, color=mf.BLUE, linestyle=":", linewidth=3.5, label=f"曲がり終了の条件({exit_:g}度/秒未満)")
        ax.set_ylim(0, 74)
        ax.set_yticks([0, 20, 40, 60])
        ax.set_ylabel("ヨーレート\n[度/秒]", fontsize=FS - 2, color=mf.MUTED)
        ax.set_title(f"{label}(60度/秒を超える点は省略)", fontsize=FS + 2, color=mf.INK, loc="left")
        ax.legend(loc="upper left", ncol=2, fontsize=FS - 4, borderaxespad=0.2, frameon=True, framealpha=0.95, edgecolor="none")
        poster_style(ax)
        ax.tick_params(labelbottom=False)
        widths = np.diff(np.append(ts, ts[-1] + np.median(np.diff(ts))))
        bar.bar(ts, np.ones(len(ts)), width=widths, align="edge",
                color=np.where(turning, mf.ORANGE, mf.BLUE), linewidth=0)
        bar.set_yticks([])
        bar.set_ylabel("判定", fontsize=FS - 2, color=mf.MUTED, rotation=0, ha="right", va="center")
        poster_style(bar)
        bar.grid(False)
        bar.set_xlabel("歩き始めからの時間 [秒]", fontsize=FS - 2, color=mf.MUTED)
    fig.legend(handles=[Patch(color=mf.BLUE, label="直進と判定"), Patch(color=mf.ORANGE, label="曲がりと判定")],
               loc="lower center", ncol=2, frameon=False, fontsize=FS - 2, bbox_to_anchor=(0.5, 0.0))
    fig.subplots_adjust(left=0.1, right=0.98, top=0.96, bottom=0.14)
    fig.savefig(out, dpi=DPI, bbox_inches="tight", pad_inches=0.05)
    plt.close(fig)


def figure_cumulative_error(out):
    """卒論の図1.1(figures/concept_figures.py の概念図)を、ポスター用に高解像度で書き出す。"""
    import concept_figures as cf

    def save(fig, _stem):
        fig.savefig(out, dpi=500, bbox_inches="tight", facecolor="white")
        plt.close(fig)

    cf.save = save
    cf.fig_cumulative_error()


def figure_neff_equation(out):
    """有効粒子数の式(TeX記法をmatplotlibのmathtextで描く)。"""
    with plt.rc_context({"mathtext.fontset": "cm"}):
        fig = plt.figure(figsize=(6, 1.6))
        fig.text(0.5, 0.5, r"$N_{\mathrm{eff}} = \frac{1}{\Sigma_{i=1}^{N}\, w_i^{2}}$", fontsize=40,
                 ha="center", va="center", color=mf.INK)
        fig.savefig(out, dpi=400, bbox_inches="tight", pad_inches=0.05, transparent=True)
        plt.close(fig)


if __name__ == "__main__":
    out_dir = HERE / "poster"
    out_dir.mkdir(exist_ok=True)
    figure_route_band(out_dir / "poster_経路帯.png")
    figure_trajectories(out_dir / "poster_推定軌跡_1441_1442.png")
    figure_behavior(out_dir / "poster_曲がり判定_1441_1442.png")
    figure_cumulative_error(out_dir / "poster_誤差の累積.png")
    figure_neff_equation(out_dir / "poster_式_Neff.png")
    print("保存:", out_dir)
