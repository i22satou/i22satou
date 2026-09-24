#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2026-09-24 のコード点検で使った説明用の図を、既存の実測3本(pdr_log_0805_1438/1441/1442、
調整用データ。評価用データではない)から作る。図の説明は同じフォルダの README.md。

  20260924_移動様態判定_曲がりが終わらない問題_1441_1442.png
      本体の判定関数(detect_move_behavior)をそのまま呼んで、歩ごとの判定を再現した図。
  20260924_quick_check方位判定_修正前後_1438_1441_1442.png
      tools/quick_check.py の「方位の正味回転」の測り方を、修正前(記録の最初と最後の差)と
      修正後(最初と最後の5歩の平均の差)で比べた図。

どちらも研究結果(RMSE)ではなく、判定・確認の仕組みを説明するための図である。
実行(i22satou/ で。Windowsでは PDR_DATA_DIR を設定する):
    python figures/make_figures_20260924.py
"""
import os
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

HERE = Path(__file__).resolve().parent
PROG = HERE.parent / "pdr_program"
sys.path.insert(0, str(PROG))
import pdr_pf_improved as p  # noqa: E402  (japanize_matplotlib もここで読まれる)

_cfg, resolved = p.load_map_config_for_tool(PROG / "map_configs" / "kanri_4f.json")
DATA = Path(os.environ.get("PDR_DATA_DIR", resolved.data_dir))

INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e6e5e0"
BLUE, ORANGE = "#2a78d6", "#eb6834"


def load(name):
    df = p.validate_log(p.safe_read_csv(DATA / name), name)
    t = df["timestamp"].to_numpy(float)
    df["step_acc"] = p.compute_step_acceleration(p.compute_acc_magnitude(df))
    steps, _ = p.detect_steps_smartpdr(df["step_acc"], 1 / np.mean(np.diff(t)))
    return df, t, steps


def style(ax):
    ax.grid(True, axis="y", color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelsize=8)


def behavior_sequence(df, t, steps):
    """本体と同じ関数で歩ごとの移動様態を判定する(方位はandroid)。"""
    yaw = np.deg2rad(df["yaw_deg"].to_numpy(float))
    heading = p.normalize_angle(yaw - yaw[steps[0]])
    heading[0] = 0.0  # 本体と同じ(heading_history[0]は0のまま)
    g = df[["gyro_x", "gyro_y", "gyro_z"]].to_numpy(float)
    a = df[["acc_x", "acc_y", "acc_z"]].to_numpy(float)
    yaw_rate = (g * a).sum(1) / np.linalg.norm(a, axis=1)  # get_yaw_rate()と同じ式
    states, p75 = [], []
    prev = p.MoveBehavior.STRAIGHT
    for i in steps:
        prev = p.detect_move_behavior(t, heading, yaw_rate, i, prev, step_detected=True)
        _hc, rate = p.behavior_window_stats(t, heading, yaw_rate, i)
        states.append(prev)
        p75.append(np.rad2deg(rate))
    return states, np.array(p75)


def figure_behavior(out):
    enter = np.rad2deg(p.TURN_YAW_RATE_THRESHOLD)
    exit_ = np.rad2deg(p.TURN_EXIT_YAW_RATE_THRESHOLD)
    fig, axes = plt.subplots(3, 2, figsize=(12, 7.4), sharex="col",
                             gridspec_kw={"height_ratios": [2.2, 2.2, 0.8]})
    for col, name in enumerate(["pdr_log_0805_1441.csv", "pdr_log_0805_1442.csv"]):
        df, t, steps = load(name)
        states, p75 = behavior_sequence(df, t, steps)
        ts = t[steps] - t[steps[0]]
        turning = np.array([s == p.MoveBehavior.TURNING for s in states])
        yaw_u = np.rad2deg(np.unwrap(np.deg2rad(df["yaw_deg"].to_numpy(float))))
        sec = slice(steps[0], steps[-1] + 1)

        ax = axes[0, col]
        ax.plot(t[sec] - t[steps[0]], yaw_u[sec] - yaw_u[steps[:5]].mean(), color=MUTED, linewidth=1.0)
        ax.set_ylabel("方位 [度]\n(歩き始めを0)", color=MUTED, fontsize=9)
        ax.set_title(f"{name}(曲がりと判定された歩 {100 * turning.mean():.0f}%)", fontsize=10, color=INK)
        style(ax)

        ax = axes[1, col]
        ax.scatter(ts, p75, s=12, color=INK, zorder=3, linewidths=0)
        for y, label in ((enter, f"曲がり開始の条件({enter:g}度/秒以上)"),
                         (exit_, f"曲がり終了の条件({exit_:g}度/秒未満)")):
            ax.axhline(y, color=MUTED, linestyle="--", linewidth=1.0)
            ax.text(ts[-1], y + 1.0, label, ha="right", va="bottom", fontsize=8, color=MUTED, zorder=5,
                    bbox=dict(facecolor="white", edgecolor="none", pad=1.5, alpha=0.9))
        ax.set_ylabel("直近1.5秒の\nヨーレート75%値 [度/秒]", color=MUTED, fontsize=9)
        ax.set_ylim(bottom=0)
        style(ax)

        ax = axes[2, col]
        widths = np.diff(np.append(ts, ts[-1] + np.median(np.diff(ts))))
        ax.bar(ts, np.ones(len(ts)), width=widths, align="edge",
               color=np.where(turning, ORANGE, BLUE), linewidth=0)
        ax.set_yticks([])
        ax.set_xlabel("歩き始めからの時間 [秒]", color=MUTED, fontsize=9)
        ax.set_ylabel("判定", color=MUTED, fontsize=9)
        style(ax)
        ax.grid(False)

    fig.legend(handles=[Patch(color=BLUE, label="直進と判定"), Patch(color=ORANGE, label="曲がりと判定")],
               loc="lower center", ncol=2, frameon=False, fontsize=9, bbox_to_anchor=(0.5, -0.01))
    fig.suptitle("移動様態の判定が「曲がり」から戻らない問題(2026-09-24時点の設定、android方位、本体の判定関数で再現)\n"
                 f"曲がり終了の条件は「方位変化{np.rad2deg(p.TURN_EXIT_THRESHOLD):g}度未満」かつ"
                 f"「ヨーレート75%値{exit_:g}度/秒未満」。歩行の揺れで75%値は直進中も{exit_:g}度/秒を超える",
                 fontsize=10, color=INK)
    fig.tight_layout(rect=(0, 0.04, 1, 0.94))
    fig.savefig(out, dpi=170, bbox_inches="tight")
    plt.close(fig)


def figure_quick_check(out):
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.2), sharey=True)
    for ax, (name, verdict) in zip(axes, [("pdr_log_0805_1438.csv", "壊れた記録"),
                                          ("pdr_log_0805_1441.csv", "初期方位ずれ、walkingなら使える"),
                                          ("pdr_log_0805_1442.csv", "良好な記録")]):
        df, t, steps = load(name)
        yaw_all = df["yaw_deg"].to_numpy(float)
        yaw_u = np.rad2deg(np.unwrap(np.deg2rad(yaw_all)))
        tt = t - t[0]
        sec = np.arange(steps[0], steps[-1] + 1)
        walk = np.rad2deg(np.unwrap(np.deg2rad(yaw_all[sec])))   # quick_check と同じく歩行区間で連続化
        first5 = np.interp(steps[:5], sec, walk).mean()
        last5 = np.interp(steps[-5:], sec, walk).mean()
        offset = walk[0] - yaw_u[steps[0]]
        old, new = yaw_u[-1] - yaw_u[0], last5 - first5
        ax.axvspan(tt[steps[0]], tt[steps[-1]], color="#f0efec", zorder=0)
        ax.plot(tt, yaw_u + offset - first5, color=MUTED, linewidth=0.9, zorder=2)
        ax.scatter([tt[0], tt[-1]], [yaw_u[0] + offset - first5, yaw_u[-1] + offset - first5], s=46,
                   color=ORANGE, zorder=4, edgecolors="white", linewidths=0.8)
        for idx, val in ((steps[:5], first5), (steps[-5:], last5)):
            ax.plot([tt[idx[0]], tt[idx[-1]]], [val - first5] * 2, color=BLUE, linewidth=4,
                    solid_capstyle="round", zorder=5)
        ax.set_title(f"{name}({verdict})\n"
                     f"修正前: 記録の最初と最後の差 {old:+.0f}度 → {'OK' if abs(old) <= 45 else '異常'}\n"
                     f"修正後: 最初と最後の5歩の平均の差 {new:+.0f}度 → {'OK' if abs(new) <= 45 else '異常'}",
                     fontsize=9, color=INK, loc="left")
        ax.set_xlabel("記録開始からの時間 [秒]", color=MUTED, fontsize=9)
        style(ax)
    axes[0].set_ylabel("方位 [度](最初の5歩の平均を0)", color=MUTED, fontsize=9)
    fig.legend(handles=[
        Patch(color="#f0efec", label="歩行区間(最初の歩〜最後の歩)"),
        Line2D([], [], color=BLUE, linewidth=4, label="修正後: 最初と最後の5歩の平均方位"),
        Line2D([], [], linestyle="none", marker="o", markersize=7, color=ORANGE,
               label="修正前: 記録の最初と最後の1サンプル")],
        loc="lower center", ncol=3, frameon=False, fontsize=8.5, bbox_to_anchor=(0.5, -0.04))
    fig.suptitle("tools/quick_check.py の「方位の正味回転」の判定(想定0度、許容±45度)の修正前後(2026-09-24)",
                 fontsize=10.5, color=INK)
    fig.tight_layout(rect=(0, 0.06, 1, 0.93))
    fig.savefig(out, dpi=170, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    figure_behavior(HERE / "20260924_移動様態判定_曲がりが終わらない問題_1441_1442.png")
    figure_quick_check(HERE / "20260924_quick_check方位判定_修正前後_1438_1441_1442.png")
    print("保存しました:", HERE)
