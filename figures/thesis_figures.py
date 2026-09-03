#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
卒業論文の第3章・第4章・第8章で使う図を実測データから生成する。

前処理・ステップ検出は pdr_pf_improved.py の関数をそのまま import して使うので、
本文で説明した処理と図が食い違うことはない(定数を変えれば図も追従する)。

出力先は figures/ 直下。実行:
    python3 thesis_figures.py
"""
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import japanize_matplotlib  # noqa: F401  (日本語フォント登録)
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from PIL import Image

HERE = Path(__file__).resolve().parent
PROG_DIR = HERE.parent / "pdr_program"
sys.path.insert(0, str(PROG_DIR))

import pdr_pf_improved as P  # noqa: E402

DATA_DIR = Path(
    "/Users/soma/Library/CloudStorage/GoogleDrive-satosoma0608@gmail.com/マイドライブ/PDR"
)
FILES = ["pdr_log_0805_1438.csv", "pdr_log_0805_1441.csv", "pdr_log_0805_1442.csv"]
MAP_PNG = PROG_DIR / "kanri_4f_binary_final3.png"
SCALE = 11.4  # px/m (map_configs/kanri_4f.json の scale_px_per_m)

# 手動で与えた正解経路の折れ点(map_configs/kanri_4f.json の route_points)
ROUTE = [(100.0, 230.0), (425.0, 230.0), (425.0, 115.0), (800.0, 115.0)]


def short(name):
    """pdr_log_0805_1442.csv -> 1442"""
    return name.replace("pdr_log_0805_", "").replace(".csv", "")


def load(name):
    df = pd.read_csv(DATA_DIR / name)
    t = df["timestamp"].to_numpy()
    sr = 1.0 / float(np.median(np.diff(t)))
    return df, t - t[0], sr


def save(fig, stem):
    out = HERE / f"{stem}.png"
    fig.savefig(out, dpi=200, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"保存: {out.name}")


# ---------------------------------------------------------------------------
# 図3.1 実験環境の二値地図と歩行経路
# ---------------------------------------------------------------------------
def fig_map_route():
    img = np.array(Image.open(MAP_PNG).convert("L"))
    h, w = img.shape
    fig, ax = plt.subplots(figsize=(11, 4.2))
    ax.imshow(img, cmap="gray", interpolation="nearest")

    xs = [p[0] for p in ROUTE]
    ys = [p[1] for p in ROUTE]
    ax.plot(xs, ys, "-", color="#d62728", lw=2.6, zorder=3, label="歩行経路(概略)")

    labels = ["開始点", "曲がり角1", "曲がり角2", "終了点(最長時)"]
    for (x, y), lab in zip(ROUTE, labels):
        ax.add_patch(Circle((x, y), 7, facecolor="#d62728", edgecolor="white",
                            lw=1.2, zorder=4))
        ax.annotate(lab, (x, y), textcoords="offset points", xytext=(0, -20),
                    ha="center", fontsize=10, color="#8b1a1a", fontweight="bold",
                    zorder=5,
                    bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="none", alpha=0.85))

    # スケールバー(10m)
    bar_px = 10 * SCALE
    bx, by = w - bar_px - 40, h - 28
    ax.plot([bx, bx + bar_px], [by, by], "-", color="#1f4e79", lw=3.2, zorder=5)
    ax.text(bx + bar_px / 2, by - 8, "10 m", ha="center", va="bottom",
            fontsize=10, color="#1f4e79", fontweight="bold", zorder=5)

    ax.set_xlim(0, w)
    ax.set_ylim(h, 0)
    ax.set_xlabel("x [px]  (1 px = 1/11.4 m)")
    ax.set_ylabel("y [px]")
    ax.set_title(f"管理棟4階の二値地図({w}×{h} px ≒ {w/SCALE:.0f}×{h/SCALE:.0f} m)"
                 "と歩行経路\n白: 移動可能領域(廊下・室内)  黒: 壁・移動不可領域")
    ax.legend(loc="upper left", fontsize=10)
    save(fig, "fig3_1_map_route")


# ---------------------------------------------------------------------------
# 図3.2 計測データの例
# ---------------------------------------------------------------------------
def fig_sensor_example():
    name = "pdr_log_0805_1442.csv"
    df, t, sr = load(name)
    acc_mag = P.compute_acc_magnitude(df).to_numpy()
    gyro = np.sqrt(df["gyro_x"] ** 2 + df["gyro_y"] ** 2 + df["gyro_z"] ** 2).to_numpy()
    yaw = df["yaw_deg"].to_numpy()

    fig, axes = plt.subplots(3, 1, figsize=(10, 6.4), sharex=True)
    axes[0].plot(t, acc_mag, lw=0.8, color="#1f4e79")
    axes[0].set_ylabel("合成加速度\n[m/s$^2$]")
    axes[1].plot(t, np.rad2deg(gyro), lw=0.8, color="#b5460a")
    axes[1].set_ylabel("角速度の大きさ\n[deg/s]")
    axes[2].plot(t, yaw, lw=0.9, color="#2e7d32")
    axes[2].set_ylabel("方位角 yaw_deg\n[deg]")
    axes[2].set_xlabel("計測開始からの経過時間 [s]")
    for ax in axes:
        ax.grid(alpha=0.3)
    axes[0].set_title(f"計測データの例({short(name)}、"
                      f"サンプリング周波数 {sr:.1f} Hz、計測時間 {t[-1]:.1f} s)")
    fig.align_ylabels(axes)
    save(fig, "fig3_2_sensor_example")


# ---------------------------------------------------------------------------
# 図4.3 ステップ検出の信号処理
# ---------------------------------------------------------------------------
def fig_step_detection():
    name = "pdr_log_0805_1442.csv"
    df, t, sr = load(name)
    acc_mag = P.compute_acc_magnitude(df)
    step_acc = P.compute_step_acceleration(acc_mag)
    peaks, valleys = P.detect_steps_smartpdr(step_acc, sampling_rate_hz=sr)

    lo, hi = 10.0, 18.0  # 見やすい8秒間を切り出す
    m = (t >= lo) & (t <= hi)
    sel = [i for i in range(len(peaks)) if lo <= t[peaks[i]] <= hi]

    fig, axes = plt.subplots(2, 1, figsize=(10, 5.4), sharex=True)
    axes[0].plot(t[m], acc_mag.to_numpy()[m], lw=1.0, color="#888888")
    axes[0].set_ylabel("合成加速度\n[m/s$^2$]")
    axes[0].set_title("ステップ検出の信号処理"
                      f"({short(name)}、{lo:.0f}〜{hi:.0f} s を拡大)")

    v = step_acc.to_numpy()
    axes[1].plot(t[m], v[m], lw=1.2, color="#1f4e79", label="ステップ信号(HPF+LPF後)")
    axes[1].axhline(P.SMART_PEAK_THR, ls="--", lw=1.0, color="#b5460a",
                    label=f"ピーク閾値 {P.SMART_PEAK_THR}")
    if sel:
        axes[1].plot(t[peaks[sel]], v[peaks[sel]], "o", ms=7, mfc="#d62728",
                     mec="white", mew=1.0, label="検出した1歩(ピーク)", zorder=5)
        axes[1].plot(t[valleys[sel]], v[valleys[sel]], "v", ms=6, mfc="#2e7d32",
                     mec="white", mew=0.8, label="直前の谷", zorder=5)
    axes[1].set_ylabel("ステップ信号")
    axes[1].set_xlabel("計測開始からの経過時間 [s]")
    axes[1].legend(fontsize=9, ncol=4, loc="upper center",
                   bbox_to_anchor=(0.5, -0.22), frameon=False)
    for ax in axes:
        ax.grid(alpha=0.3)
    fig.align_ylabels(axes)

    interval = P.step_min_interval_samples(sr)
    print(f"  (図4.3) 総検出歩数 {len(peaks)}、最短ピーク間隔 {interval} サンプル"
          f" = {interval/sr*1000:.0f} ms")
    save(fig, "fig4_3_step_detection")


# ---------------------------------------------------------------------------
# 図8.1 合成加速度の周波数スペクトル(基本波と第2高調波)
# ---------------------------------------------------------------------------
def fig_acc_spectrum():
    fig, ax = plt.subplots(figsize=(9, 4.6))
    colors = {"1438": "#2e7d32", "1441": "#b5460a", "1442": "#1f4e79"}
    for name in FILES:
        df, t, sr = load(name)
        v = P.compute_step_acceleration(P.compute_acc_magnitude(df)).to_numpy()
        v = v - np.nanmean(v)
        v = np.nan_to_num(v)
        spec = np.abs(np.fft.rfft(v * np.hanning(len(v))))
        freq = np.fft.rfftfreq(len(v), d=1.0 / sr)
        band = (freq >= 0.5) & (freq <= 6.0)
        s = spec[band] / spec[band].max()
        k = short(name)
        ax.plot(freq[band], s, lw=1.5, color=colors[k], label=f"{k}")

    ax.axvline(P.MAX_STEP_FREQUENCY_HZ, ls="--", lw=1.6, color="#b71c1c")
    ax.text(P.MAX_STEP_FREQUENCY_HZ - 0.08, 0.62,
            f"上限歩調 {P.MAX_STEP_FREQUENCY_HZ} Hz\n修正後はここより\n右を1歩として採らない",
            fontsize=9, color="#b71c1c", va="center", ha="right",
            bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="#b71c1c", alpha=0.9))
    ax.axvspan(1.4, 1.9, color="#2e7d32", alpha=0.10)
    ax.text(1.65, 1.03, "基本波\n(歩調)", ha="center", fontsize=9, color="#2e7d32")
    ax.axvspan(3.1, 3.6, color="#b5460a", alpha=0.10)
    ax.text(3.35, 1.03, "第2高調波", ha="center", fontsize=9, color="#b5460a")

    ax.set_xlim(0.5, 6.0)
    ax.set_ylim(0, 1.18)
    ax.set_xlabel("周波数 [Hz]")
    ax.set_ylabel("規格化した振幅スペクトル")
    ax.set_title("歩行加速度(ステップ信号)の周波数スペクトル\n"
                 "第2高調波が基本波と同程度に強く、ステップ検出の過検出源となっていた")
    ax.legend(title="CSV", fontsize=9)
    ax.grid(alpha=0.3)
    save(fig, "fig8_1_acc_spectrum")


# ---------------------------------------------------------------------------
# 図8.2 CSVごとの相対方位プロファイル
# ---------------------------------------------------------------------------
def fig_heading_profile():
    fig, ax = plt.subplots(figsize=(9, 4.6))
    colors = {"1438": "#2e7d32", "1441": "#b5460a", "1442": "#1f4e79"}
    n_bin = 10
    for name in FILES:
        df, t, sr = load(name)
        yaw = np.deg2rad(df["yaw_deg"].to_numpy())
        base = np.arctan2(np.mean(np.sin(yaw[:30])), np.mean(np.cos(yaw[:30])))
        rel = np.rad2deg(np.arctan2(np.sin(yaw - base), np.cos(yaw - base)))
        edges = np.linspace(0, len(rel), n_bin + 1).astype(int)
        prof = [float(np.rad2deg(np.arctan2(
            np.mean(np.sin(np.deg2rad(rel[a:b]))),
            np.mean(np.cos(np.deg2rad(rel[a:b])))))) for a, b in zip(edges[:-1], edges[1:])]
        k = short(name)
        ax.plot(range(1, n_bin + 1), prof, "-o", lw=1.8, ms=5,
                color=colors[k], label=k)

    ax.axhline(0, ls="--", lw=1.2, color="#555555")
    ax.axhline(-90, ls="--", lw=1.2, color="#555555")
    ax.text(10.35, 0, " 東向き\n (0°)", fontsize=9, va="center", color="#555555")
    ax.text(10.35, -90, " 北向き\n (-90°)", fontsize=9, va="center", color="#555555")
    ax.set_xticks(range(1, n_bin + 1))
    ax.set_xlabel("計測を10等分した区間番号(左が計測開始)")
    ax.set_ylabel("計測開始時を基準とした相対方位 [deg]")
    ax.set_title("CSVごとの相対方位プロファイル\n"
                 "正しい経路(東→北→東)なら 0° → -90° → 0° をたどるはず")
    ax.set_xlim(0.6, 11.6)
    ax.legend(title="CSV", fontsize=9, loc="lower left")
    ax.grid(alpha=0.3)
    save(fig, "fig8_2_heading_profile")


if __name__ == "__main__":
    fig_map_route()
    fig_sensor_example()
    fig_step_detection()
    fig_acc_spectrum()
    fig_heading_profile()
