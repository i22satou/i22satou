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
  pdr_program/results/20260918_133540_evaluation_0805check/20260924_推定軌跡_4方式の比較_1441_1442_1438_seed42.png
      同日の比較実験(pdr_program/results/20260924_120622_evaluation_0805check_recovery/)の
      4方式の軌跡(seed=42)を、申告された歩行経路と一緒に3本まとめて描いた図。軌跡CSVは
      gitに含めない runs/ にあるので、無ければ run_evaluation.py を同じ条件で実行し直す。
  進捗報告/20260924_推定軌跡_9月18日と9月24日の比較_全滅時の復帰の変更前後.png
      9/18の比較実験(20260918_133540_evaluation_0805check/)と9/24の比較実験で、方式B・C・Eの
      軌跡(seed=42)と6シードの終点を重ねた図。2つの実験で主な4方式に効く違いは、全滅時の復帰の
      変更(コミット55e3e74)だけ。方式Aは乱数も全滅も無いので同一(確認済み)で、図には入れない。

推定軌跡の2枚はこのフォルダの外に保存する。どれも研究結果(RMSE)ではなく、判定・確認の仕組みや軌跡の様子を説明するための図である。
実行(i22satou/ で。Windowsでは PDR_DATA_DIR を設定する):
    python figures/make_figures_20260924.py
"""
import json
import os
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402
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


EVAL = PROG / "results" / "20260924_120622_evaluation_0805check_recovery"
EVAL_0918 = PROG / "results" / "20260918_133540_evaluation_0805check"
TRAJ_STYLE = [  # run_evaluation.py の軌跡図と同じ色・線種
    ("A_pdr", MUTED, "--", "A: PDRのみ"),
    ("B_fixed", ORANGE, "-", "B: 固定粒子数PF"),
    ("C_adaptive", "#1baf7a", "-", "C: 移動様態適応PF"),
    ("E_proposed", BLUE, "-", "E: 提案方式"),
]
FILE_NOTES = [  # 方位の質は memo/heading_calibration.md の判断
    ("pdr_log_0805_1441.csv", "方位の記録は使える"),
    ("pdr_log_0805_1442.csv", "方位の記録は使える"),
    ("pdr_log_0805_1438.csv", "方位の記録が壊れている。参考"),
]


ROUTE, OLD = "#f2c230", "#aeada7"


def route_geometry():
    """申告された歩行経路の角と終点、各CSVの開始位置、縮尺(比較実験で使った設定から読む)。"""
    used = json.loads((EVAL / "used_map_config.json").read_text(encoding="utf-8"))
    corners = np.array(used["route_points"][1:], float)  # 先頭は手動経路の西端。実際の開始位置に置き換える
    starts = pd.read_csv(PROG / "start_positions.csv").set_index("file_name")
    return corners, starts, float(used["scale_px_per_m"])


def load_trajectory(eval_dir, row):
    """results_long.csv の1行が指す軌跡CSV(runs/ の下)を読む。"""
    rel = row["trajectory"].replace("\\", "/").split("/runs/", 1)[1]
    return pd.read_csv(eval_dir / "runs" / rel)


def map_panel(ax, binary, start, corners, xmax=880):
    """地図・申告された歩行経路・開始位置・終点を描く。xmaxより東は軌跡も経路も通らないので切る。"""
    ax.imshow(np.where(binary == 255, 1, 0), cmap=ListedColormap(["#d6d5cf", "#ffffff"]),
              interpolation="nearest", vmin=0, vmax=1)
    route = np.vstack([start, corners])
    ax.plot(route[:, 0], route[:, 1], color=ROUTE, linewidth=9, alpha=0.45,
            solid_joinstyle="round", solid_capstyle="round", zorder=2)
    ax.scatter(*start, marker="s", s=60, color=INK, edgecolors="white", linewidths=1, zorder=5)
    ax.scatter(*corners[-1], marker="*", s=220, color=ROUTE, edgecolors=INK, linewidths=1, zorder=5)
    ax.set_xlim(0, xmax)
    ax.set_ylim(binary.shape[0], 0)
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(False)


def map_handles():
    return [Line2D([], [], color=ROUTE, linewidth=9, alpha=0.45, label="歩いた経路(申告)"),
            Line2D([], [], linestyle="none", marker="s", markersize=7, color=INK,
                   markeredgecolor="white", label="開始位置"),
            Line2D([], [], linestyle="none", marker="*", markersize=13, color=ROUTE,
                   markeredgecolor=INK, label="終点(申告、800,115)")]


def figure_trajectories(out, seed=42):
    binary, _pf, _dist = p.load_preprocessed_map(resolved.map)
    corners, starts, _scale = route_geometry()
    runs = pd.read_csv(EVAL / "results_long.csv", encoding="utf-8-sig")
    proxy = pd.read_csv(EVAL / "endpoint_proxy_0805.csv", encoding="utf-8-sig")
    fig, axes = plt.subplots(3, 1, figsize=(11, 13.2))
    for ax, (name, note) in zip(axes, FILE_NOTES):
        start = starts.loc[name, ["start_x", "start_y"]].to_numpy(float)
        map_panel(ax, binary, start, corners)
        distances = []
        for key, color, linestyle, label in TRAJ_STYLE:
            row = runs[(runs["method_key"] == key) & (runs["file"] == name)
                       & ((runs["seed"] == seed) | runs["seed"].isna())].iloc[0]
            t = load_trajectory(EVAL, row)
            ax.plot(np.r_[start[0], t["x_px"]], np.r_[start[1], t["y_px"]], color=color,
                    linestyle=linestyle, linewidth=1.8, zorder=3)
            mean = proxy.loc[(proxy["method_key"] == key) & (proxy["file"] == name), "mean_m"]
            distances.append(f"{label.split(':')[0]} {mean.iloc[0]:.1f}")
        ax.set_title(f"{name}({note})\n終点までの距離 [m](6シードの平均): " + " / ".join(distances),
                     fontsize=10, color=INK)
    handles = [Line2D([], [], color=c, linestyle=ls, linewidth=1.8, label=lb)
               for _k, c, ls, lb in TRAJ_STYLE]
    fig.legend(handles=handles + map_handles(), loc="lower center", ncol=4, frameon=False, fontsize=9,
               bbox_to_anchor=(0.5, 0.035))
    fig.text(0.5, 0.0,
             "既存の実測3本(調整用データで、評価用データではない)。時刻対応した正解位置が無いのでRMSEは出せず、\n"
             "「終点までの距離」は代替指標。軌跡は seed=42 の1回分、方位は全方式 android+walking。"
             "PDRのみ(A)は地図の外へ出た部分を切っている。\n"
             "曲がり終了のしきい値が暫定値(10度/秒)で歩の8割以上が「曲がり」判定のため、"
             "BとCの差は様態適応の有無をほとんど反映していない。",
             ha="center", va="bottom", fontsize=8.5, color=MUTED, linespacing=1.5)
    fig.suptitle("比較する4方式の推定軌跡(2026-09-24、コミット55e3e74の比較実験)",
                 fontsize=12, color=INK)
    fig.tight_layout(rect=(0, 0.1, 1, 0.97))
    fig.savefig(out, dpi=170, bbox_inches="tight")
    plt.close(fig)


def figure_before_after(out, seed=42):
    """9/18と9/24の比較実験で、方式B・C・Eの軌跡(seed固定)と6シードの終点を重ねる。"""
    binary, _pf, _dist = p.load_preprocessed_map(resolved.map)
    corners, starts, scale = route_geometry()
    old_cfg = json.loads((EVAL_0918 / "used_map_config.json").read_text(encoding="utf-8"))
    assert np.allclose(old_cfg["route_points"][1:], corners), "2つの実験で経路の設定が違う"
    end = corners[-1]
    runs = {d: pd.read_csv(d / "results_long.csv", encoding="utf-8-sig") for d in (EVAL_0918, EVAL)}
    styles = {key: (color, label) for key, color, _ls, label in TRAJ_STYLE}
    fig, axes = plt.subplots(3, 3, figsize=(17, 10.0))
    for r, (name, note) in enumerate(FILE_NOTES):
        start = starts.loc[name, ["start_x", "start_y"]].to_numpy(float)
        for c, key in enumerate(["B_fixed", "C_adaptive", "E_proposed"]):
            ax = axes[r, c]
            color, label = styles[key]
            map_panel(ax, binary, start, corners)
            summary = []
            # 変更前は灰色の太線・白抜きの点、変更後は方式の色の細線・塗りの点。同じなら灰色の中に色が重なる
            for eval_dir, line_color, width, face, edge, z in (
                    (EVAL_0918, OLD, 4.5, "white", OLD, 3), (EVAL, color, 1.6, color, "white", 4)):
                df = runs[eval_dir]
                sel = df[(df["method_key"] == key) & (df["file"] == name)]
                t = load_trajectory(eval_dir, sel[sel["seed"] == seed].iloc[0])
                ax.plot(np.r_[start[0], t["x_px"]], np.r_[start[1], t["y_px"]], color=line_color,
                        linewidth=width, solid_capstyle="round", solid_joinstyle="round", zorder=z)
                ax.scatter(sel["final_x"], sel["final_y"], s=28, facecolor=face, edgecolor=edge,
                           linewidths=1.3, zorder=z + 3)
                dist = np.hypot(sel["final_x"] - end[0], sel["final_y"] - end[1]) / scale
                summary.append((sel["extinctions"].sum(), dist.mean()))
            (ext0, d0), (ext1, d1) = summary
            ax.set_title(f"{label}(全滅 {ext0:.0f}→{ext1:.0f}回、6シードの合計)\n"
                         f"終点までの距離 {d0:.1f}→{d1:.1f} m(6シードの平均)", fontsize=9.5, color=INK)
        axes[r, 0].set_ylabel(f"{name}\n({note})", fontsize=9.5, color=INK)
    handles = [Line2D([], [], color=OLD, linewidth=4.5, label="9月18日(変更前)の軌跡"),
               Line2D([], [], color=INK, linewidth=1.6, label="9月24日(変更後)の軌跡(色は方式)"),
               Line2D([], [], linestyle="none", marker="o", markersize=6, markerfacecolor="white",
                      markeredgecolor=OLD, markeredgewidth=1.3, label="9月18日の終点(6シード)"),
               Line2D([], [], linestyle="none", marker="o", markersize=6, color=INK,
                      markeredgecolor="white", label="9月24日の終点(6シード)")]
    fig.legend(handles=handles + map_handles(), loc="lower center", ncol=4, frameon=False,
               fontsize=9, bbox_to_anchor=(0.5, 0.068))
    fig.text(0.5, 0.0,
             "2つの実験で主な方式に効く違いは、粒子が全滅したときの復帰の仕方だけ(9月24日: その歩の移動量だけ"
             "進めた位置へまき直す。壁を越えるなら従来どおり移動前の位置へ)。\n"
             "全滅が起きなかった実行は両日で完全に同じになり、灰色の太線の中に色の線が重なる。"
             "変わった実行には、全滅後に乱数の使い方が変わった影響も含まれる。\n"
             f"軌跡は seed={seed} の1回分、点は6シードの終点。方式A(PDRのみ)は乱数も全滅も無いので両日で同じ"
             "(図には入れていない)。既存の実測3本(調整用データ)、方位は全方式 android+walking。\n"
             "終点までの距離はRMSEではない代替指標。",
             ha="center", va="bottom", fontsize=8.5, color=MUTED, linespacing=1.5)
    fig.suptitle("全滅時の復帰の変更前後の推定軌跡(9月18日と9月24日の比較実験)", fontsize=12, color=INK)
    fig.tight_layout(rect=(0, 0.12, 1, 0.97), h_pad=2.0)
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    figure_behavior(HERE / "20260924_移動様態判定_曲がりが終わらない問題_1441_1442.png")
    figure_quick_check(HERE / "20260924_quick_check方位判定_修正前後_1438_1441_1442.png")
    figure_trajectories(EVAL_0918 / "20260924_推定軌跡_4方式の比較_1441_1442_1438_seed42.png")
    figure_before_after(HERE.parent / "進捗報告" / "20260924_推定軌跡_9月18日と9月24日の比較_全滅時の復帰の変更前後.png")
    print("保存しました")
