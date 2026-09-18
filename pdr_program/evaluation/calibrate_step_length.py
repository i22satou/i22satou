# ============================================================================
# calibrate_step_length.py
#
# 【変更履歴】
# - 2026-09-18: [本研究独自] 新規作成。距離が分かっている直線を「ゆっくり・普通・速歩」で
#               歩いた校正用データから、全CSV共通の歩幅校正ゲイン
#               (step_length_calibration_gain)を求め直す。
#
# 【なぜ必要か】
# kanri_4f.json の 2.10 は、評価用と同じ3CSV(0805_1438/1441/1442)の総距離比から求めた
# 暫定値で、校正用と評価用のデータが分離できていない(memo/step_length_calibration.md)。
# 校正専用のデータで求め直し、評価データとは独立に決める。
#
# 【方針(ユーザーとの合意事項)】
# - ゲインは全ファイル共通の定数1つだけ。ファイルごとの校正や、距離を正解へ合わせ込む
#   校正はしない(target_distance_px を使わない方針と同じ)。
# - 歩数検出と歩幅推定は pdr_pf_improved.py の関数を呼ぶ。自前で計算し直さない
#   (tools/quick_check.py と同じ方式。load_map_config_for_tool() を先に呼ぶ)。
# - 歩調と真の歩幅の関係の図は、歩幅モデルを見直す材料として出すだけで、採用はしない。
#
# 【ゲインの求め方】本体の1歩の歩幅は L = clip(g × r, 0.25m, 1.0m)(r はゲイン1・クリップ前の
# SmartPDR推定値)。クリップがあるので、ゲインと総距離は比例しない。そこで2通りを出す。
#   推奨: 総距離一致方式 … 校正データ全体で「本体と同じ計算(クリップ込み)の推定総距離の
#         合計」が「実際に歩いた距離の合計」と一致する g を求根で求める。
#   参考: 総距離比方式 … 「歩いた距離の合計 ÷ ゲイン1での推定総距離の合計」。2.10はこの式で
#         求めた値と一致する(CHANGELOG_archive 2026-09-02(2)の数値から逆算すると
#         206.2m ÷ 98.0m)。ゲインを掛けた後に上限クリップで削られる分を考えないので、
#         上限に当たる歩が多いほど総距離を過小にする。
# 本体の関数を1歩ずつ呼ぶと求根が遅いので、ゲイン1・クリップ無しの値 r を本体の関数から
# 1回だけ取り、clip(g × r) で計算する。この式が本体と一致することを、現在のゲインと推奨
# ゲインの2点で本体の関数を直接呼んで照合する(一致しなければ止まる)。
#
# 【確認していること】
# - leave-one-out: 1本ずつ外して求め直したゲインの幅と、外した1本の総距離誤差(使って
#   いないデータに対する誤差の目安)。速さ(slow/normal/fast)が2種類以上あれば、速さごとに
#   まとめて外した場合も出す(ある速さを含めずに決めたゲインが、その速さに通用するか)。
#
# 【計測の注意】真の歩幅 = 歩いた距離 ÷ 検出歩数 なので、記録の中に直線以外の歩行
# (スタート地点への移動・折り返し)を含めないこと。START直後の静止は歩数に入らないので
# 問題ない。歩調は最初の歩から最後の歩までの時間で求める(静止時間を含めない)。
#
# 【使い方】pdr_program/ で実行する。CSVフォルダは本体と同じ規則
# (--data-dir > 環境変数PDR_DATA_DIR > JSONのdata_dir)で決める。
#   python evaluation/calibrate_step_length.py --list <計測一覧.csv> [--tag 0925]
#   python evaluation/calibrate_step_length.py --list ground_truth/measurement_list_0805.csv --tag 0805check
#       -> 既存3本(直線ではないが距離既知)で 2.10 が再現できるかの確認
#   python evaluation/calibrate_step_length.py --self-test   # 架空データで計算だけ確認
# 一覧表の形式は evaluation/measurement_list.py の冒頭を参照(purpose=calib の行を使う)。
# 出力(results/): <日時>_step_calibration[_tag].csv(ファイル別の表)・.png(図)・
#   _summary.json(推奨ゲインと実行条件)。kanri_4f.json への反映は手動で行う。
# ============================================================================

import argparse
import contextlib
import json
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # 図はファイルへ保存するだけなので画面を使わない
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from scipy.optimize import brentq  # noqa: E402

EVALUATION_DIR = Path(__file__).resolve().parent
PROGRAM_DIR = EVALUATION_DIR.parent
sys.path.insert(0, str(PROGRAM_DIR))
sys.path.insert(0, str(EVALUATION_DIR))
import pdr_pf_improved as pdrmod  # noqa: E402
from measurement_list import load_measurement_list, resolve_csv_path  # noqa: E402

RESULTS_DIR = PROGRAM_DIR / "results"
GAIN_SEARCH_RANGE = (0.05, 20.0)

# 図の色と印(速さごと)。散布図で3色を同時に使うので、全組合せで色覚多様性の検証を
# 通る3色を使い、印の形も変えて色だけで区別させない。
SPEED_STYLE = {
    "slow": ("#2a78d6", "o", "ゆっくり"),
    "normal": ("#eb6834", "s", "普通"),
    "fast": ("#1baf7a", "^", "速歩"),
    "": ("#52514e", "D", "速さ不明"),
}
INK, INK_MUTED, GRID = "#0b0b0b", "#52514e", "#e6e5e0"


@contextlib.contextmanager
def step_model_override(gain=None, unclipped=False):
    """本体の歩幅関数が参照するグローバル(ゲイン・上下限)を一時的に差し替え、必ず戻す。"""
    saved = (pdrmod.STEP_LENGTH_CALIBRATION_GAIN, pdrmod.MIN_STEP_M, pdrmod.MAX_STEP_M)
    try:
        if gain is not None:
            pdrmod.STEP_LENGTH_CALIBRATION_GAIN = gain
        if unclipped:
            pdrmod.MIN_STEP_M, pdrmod.MAX_STEP_M = -np.inf, np.inf
        yield
    finally:
        pdrmod.STEP_LENGTH_CALIBRATION_GAIN, pdrmod.MIN_STEP_M, pdrmod.MAX_STEP_M = saved


def analyze_file(csv_path):
    """本体の関数で歩数と歩幅(ゲイン1・クリップ前)を求める。本体の main ループと同じ前処理。"""
    df = pdrmod.validate_log(pdrmod.safe_read_csv(csv_path), csv_path.name)
    if len(df) < 2:
        raise ValueError("有効な行が2行未満")
    df["acc_mag"] = pdrmod.compute_acc_magnitude(df)
    df["step_acc"] = pdrmod.compute_step_acceleration(df["acc_mag"])
    dt_mean = df["timestamp"].diff().mean()
    hz = 1.0 / dt_mean if pd.notna(dt_mean) and dt_mean > 0 else None
    steps, valleys = pdrmod.detect_steps_smartpdr(df["step_acc"], hz)
    if len(steps) < 2:
        raise ValueError(f"歩数が{len(steps)}歩しか検出されない")

    def main_program_lengths_m():
        return np.array([
            pdrmod.estimate_smartpdr_step_length_px(df["step_acc"], p, v)
            for p, v in zip(steps, valleys)
        ]) / pdrmod.M_TO_PIXEL

    with step_model_override(gain=1.0, unclipped=True):
        raw_m = main_program_lengths_m()
    t_steps = df["timestamp"].to_numpy(float)[steps]
    walk_sec = float(t_steps[-1] - t_steps[0])
    return {
        "n_steps": len(steps),
        "walk_sec": walk_sec,
        "cadence_hz": (len(steps) - 1) / walk_sec if walk_sec > 0 else np.nan,
        "raw_m": raw_m,
        "main_program_lengths_m": main_program_lengths_m,
    }


def step_lengths_m(raw_m, gain):
    """本体と同じ順序(ゲイン → 上下限クリップ)で1歩ごとの歩幅[m]を返す。"""
    return np.clip(gain * raw_m, pdrmod.MIN_STEP_M, pdrmod.MAX_STEP_M)


def total_m(item, gain):
    return float(step_lengths_m(item["raw_m"], gain).sum())


def gain_by_ratio(items):
    """参考: 歩いた距離の合計 ÷ ゲイン1での推定総距離の合計(2.10と同じ求め方)。"""
    return sum(i["distance_m"] for i in items) / sum(total_m(i, 1.0) for i in items)


def gain_by_matching(items):
    """推奨: クリップ込みの推定総距離の合計が、歩いた距離の合計と一致するゲイン。"""
    target = sum(i["distance_m"] for i in items)

    def gap(gain):
        return sum(total_m(i, gain) for i in items) - target

    lo, hi = GAIN_SEARCH_RANGE
    if gap(hi) < 0 or gap(lo) > 0:
        # 上限クリップ(1歩1.0m)のせいで、どのゲインでも距離が足りない場合など。
        return float("nan")
    return float(brentq(gap, lo, hi, xtol=1e-10))


def check_against_main_program(items, gains):
    """clip(g×r) の計算が本体の歩幅関数そのものと一致するかを照合する。"""
    for gain in gains:
        if not np.isfinite(gain):
            continue
        for item in items:
            with step_model_override(gain=gain):
                expected = item["main_program_lengths_m"]()
            if not np.allclose(expected, step_lengths_m(item["raw_m"], gain),
                               rtol=1e-9, atol=1e-9):
                raise RuntimeError(
                    f"{item['file']}: ゲイン{gain:.4f}で本体の歩幅関数と計算が一致しない。"
                    "estimate_smartpdr_step_length_px() の中身が変わった可能性がある。"
                )


def error_pct(estimated, true):
    return 100.0 * (estimated - true) / true


def git_revision():
    """実行条件の記録用に、コミット番号と未コミットの変更の有無を返す。"""
    try:
        commit = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=PROGRAM_DIR,
                                capture_output=True, text=True, check=True).stdout.strip()
        dirty = subprocess.run(["git", "status", "--porcelain"], cwd=PROGRAM_DIR,
                               capture_output=True, text=True, check=True).stdout.strip()
        return commit, bool(dirty)
    except (OSError, subprocess.CalledProcessError):
        return None, None


def leave_one_out(items):
    """1本ずつ外してゲインを求め直し、外した1本に当てたときの総距離を返す。"""
    results = []
    for k, held in enumerate(items):
        gain = gain_by_matching(items[:k] + items[k + 1:])
        results.append((gain, total_m(held, gain) if np.isfinite(gain) else np.nan))
    return results


def leave_one_speed_out(items):
    """速さごとにまとめて外し、残りで決めたゲインがその速さに通用するかを見る。"""
    speeds = sorted({i["speed"] for i in items if i["speed"]})
    if len(speeds) < 2:
        return None
    out = {}
    for speed in speeds:
        held = [i for i in items if i["speed"] == speed]
        gain = gain_by_matching([i for i in items if i["speed"] != speed])
        errors = [error_pct(total_m(i, gain), i["distance_m"]) for i in held]
        out[speed] = {"gain": gain, "n_files": len(held),
                      "mean_error_pct": float(np.mean(errors))}
    return out


def build_table(items, gain, current_gain, loo):
    rows = []
    for k, item in enumerate(items):
        lengths = step_lengths_m(item["raw_m"], gain)
        est_total = float(lengths.sum())
        current_total = total_m(item, current_gain)
        loo_gain, loo_total = loo[k] if loo else (np.nan, np.nan)
        rows.append({
            "file": item["file"],
            "speed": item["speed"],
            "distance_m": item["distance_m"],
            "n_steps": item["n_steps"],
            "walk_sec": item["walk_sec"],
            "cadence_hz": item["cadence_hz"],
            "true_stride_m": item["distance_m"] / item["n_steps"],
            "raw_stride_m": float(item["raw_m"].mean()),
            "est_stride_m": float(lengths.mean()),
            "est_total_m": est_total,
            "error_pct": error_pct(est_total, item["distance_m"]),
            "clip_upper_pct": 100.0 * float(np.mean(gain * item["raw_m"] >= pdrmod.MAX_STEP_M)),
            "clip_lower_pct": 100.0 * float(np.mean(gain * item["raw_m"] <= pdrmod.MIN_STEP_M)),
            "current_gain_total_m": current_total,
            "current_gain_error_pct": error_pct(current_total, item["distance_m"]),
            "loo_gain": loo_gain,
            "loo_total_m": loo_total,
            "loo_error_pct": error_pct(loo_total, item["distance_m"]),
            "memo": item["memo"],
        })
    return pd.DataFrame(rows)


def plot_calibration(table, gain, ratio_gain, current_gain, out_png):
    """左: 歩調と歩幅(真値と推定)、右: 実際の距離と推定総距離。"""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.5, 4.8))
    handles = []
    for speed, (color, marker, name) in SPEED_STYLE.items():
        part = table[table["speed"] == speed]
        if part.empty:
            continue
        for _, r in part.iterrows():
            ax1.plot([r["cadence_hz"]] * 2, [r["true_stride_m"], r["est_stride_m"]],
                     color="#b9b8b2", linewidth=1.0, zorder=2)
        ax1.scatter(part["cadence_hz"], part["true_stride_m"], s=64, marker=marker,
                    color=color, edgecolors="white", linewidths=1.0, zorder=3)
        ax1.scatter(part["cadence_hz"], part["est_stride_m"], s=64, marker=marker,
                    facecolors="white", edgecolors=color, linewidths=1.5, zorder=3)
        ax2.scatter(part["distance_m"], part["est_total_m"], s=64, marker=marker,
                    color=color, edgecolors="white", linewidths=1.0, zorder=3)
        handles.append(Line2D([], [], linestyle="none", marker=marker, markersize=8,
                              color=color, markeredgecolor="white", label=name))
    handles += [
        Line2D([], [], linestyle="none", marker="o", markersize=8, color=INK_MUTED,
               markeredgecolor="white", label="塗り: 真の歩幅(距離÷歩数)"),
        Line2D([], [], linestyle="none", marker="o", markersize=8, markerfacecolor="white",
               markeredgecolor=INK_MUTED, markeredgewidth=1.5,
               label="白抜き: 推定歩幅(推奨ゲイン)"),
    ]

    cad, stride = table["cadence_hz"].to_numpy(), table["true_stride_m"].to_numpy()
    if len(table) >= 3 and np.ptp(cad) > 0:
        slope, intercept = np.polyfit(cad, stride, 1)
        xs = np.linspace(cad.min(), cad.max(), 2)
        ax1.plot(xs, slope * xs + intercept, linestyle="--", color=INK_MUTED, linewidth=1.2,
                 zorder=1)
        handles.append(Line2D([], [], linestyle="--", color=INK_MUTED, linewidth=1.2,
                              label=f"参考: 真の歩幅の回帰直線 傾き{slope:.2f}m/(歩/s)"
                                    "(モデルには採用しない)"))
    ax1.set_xlabel("歩調 [歩/s](最初の歩から最後の歩まで)")
    ax1.set_ylabel("歩幅 [m/歩]")
    ax1.set_title("歩調と歩幅", color=INK)

    top = max(table["distance_m"].max(), table["est_total_m"].max()) * 1.08
    ax2.plot([0, top], [0, top], linestyle="--", color=INK_MUTED, linewidth=1.2, zorder=1)
    ax2.text(top * 0.97, top * 0.74, "破線: 推定 = 実際", ha="right", va="top",
             color=INK_MUTED, fontsize=9)
    ax2.set_xlim(0, top)
    ax2.set_ylim(0, top)
    ax2.set_xlabel("実際に歩いた距離 [m]")
    ax2.set_ylabel(f"推定総距離 [m](ゲイン {gain:.3f})")
    ax2.set_title("総距離: 実際と推定", color=INK)

    for ax in (ax1, ax2):
        ax.grid(True, color=GRID, linewidth=0.6)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            ax.spines[side].set_color(INK_MUTED)
        ax.tick_params(colors=INK_MUTED)
    fig.legend(handles=handles, loc="lower center", ncol=3, frameon=False, fontsize=9,
               bbox_to_anchor=(0.5, -0.02))
    fig.suptitle(
        f"歩幅校正(校正データ{len(table)}本): 推奨ゲイン {gain:.3f}"
        f" / 参考: 総距離比 {ratio_gain:.3f} / 現在の設定 {current_gain:.3f}",
        color=INK)
    fig.tight_layout(rect=(0, 0.1, 1, 1))
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)


def run_calibration(list_path, data_dir, out_dir, tag=None, map_config=None,
                    make_plot=True):
    """一覧表のcalibの行からゲインを求め、表・図・要約を out_dir へ保存して要約を返す。"""
    rows = load_measurement_list(list_path, "calib")
    if rows.empty:
        raise ValueError(f"{list_path}: purpose=calib かつ use=1 の行がありません。")

    items, errors = [], []
    for _, row in rows.iterrows():
        path = resolve_csv_path(row["file"], data_dir)
        if not path.exists():
            errors.append(f"{row['file']}: ファイルが見つからない ({path})")
            continue
        try:
            item = analyze_file(path)
        except (ValueError, IOError) as error:
            errors.append(f"{row['file']}: {error}")
            continue
        item.update(file=row["file"], speed=row["speed"], distance_m=row["distance_m"],
                    memo=row["memo"])
        items.append(item)
    if errors:
        # 1本でも黙って外すとゲインが変わるので止める。使わないなら一覧表で use=0 にする。
        raise ValueError("校正に使えないファイルがあります(使わない場合は一覧表で use=0):\n  "
                         + "\n  ".join(errors))

    current_gain = float(pdrmod.STEP_LENGTH_CALIBRATION_GAIN)
    gain = gain_by_matching(items)
    ratio_gain = gain_by_ratio(items)
    if not np.isfinite(gain):
        raise ValueError("どのゲインでも総距離が一致しない(上限クリップ1歩"
                         f"{pdrmod.MAX_STEP_M}mを超える歩幅の記録が多い可能性)。")
    check_against_main_program(items, [current_gain, gain])

    loo = leave_one_out(items) if len(items) >= 2 else None
    table = build_table(items, gain, current_gain, loo)
    by_speed = leave_one_speed_out(items)

    total_true = table["distance_m"].sum()
    pooled = {
        "recommended": error_pct(table["est_total_m"].sum(), total_true),
        "ratio": error_pct(sum(total_m(i, ratio_gain) for i in items), total_true),
        "current": error_pct(table["current_gain_total_m"].sum(), total_true),
    }
    loo_summary = None
    if loo:
        loo_gains = table["loo_gain"].to_numpy()
        loo_summary = {
            "gain_min": float(np.nanmin(loo_gains)),
            "gain_max": float(np.nanmax(loo_gains)),
            "max_abs_dev_pct": float(np.nanmax(np.abs(loo_gains / gain - 1.0)) * 100.0),
            "heldout_rms_error_pct": float(np.sqrt(np.nanmean(table["loo_error_pct"] ** 2))),
            "heldout_max_abs_error_pct": float(np.nanmax(np.abs(table["loo_error_pct"]))),
        }

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = time.strftime("%Y%m%d_%H%M%S") + "_step_calibration" + (f"_{tag}" if tag else "")
    csv_out = out_dir / f"{stem}.csv"
    png_out = out_dir / f"{stem}.png"
    json_out = out_dir / f"{stem}_summary.json"
    table.to_csv(csv_out, index=False, encoding="utf-8-sig", float_format="%.4f")
    if make_plot:
        plot_calibration(table, gain, ratio_gain, current_gain, png_out)

    commit, dirty = git_revision()
    summary = {
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "tag": tag,
        "git_commit": commit,
        "git_uncommitted_changes": dirty,
        "measurement_list": str(Path(list_path).resolve()),
        "data_dir": str(data_dir),
        "map_config": str(map_config) if map_config else None,
        "n_files": len(items),
        "files_by_speed": {s or "unknown": int((table["speed"] == s).sum())
                           for s in sorted(table["speed"].unique())},
        "recommended_gain": gain,
        "recommended_method": "総距離一致(本体と同じクリップ込みの推定総距離の合計 = 歩いた距離の合計)",
        "ratio_gain": ratio_gain,
        "ratio_method": "総距離比(歩いた距離の合計 / ゲイン1での推定総距離の合計)。2.10と同じ求め方",
        "current_gain_in_config": current_gain,
        "step_clip_m": [pdrmod.MIN_STEP_M, pdrmod.MAX_STEP_M],
        "pooled_total_error_pct": pooled,
        "leave_one_out": loo_summary,
        "leave_one_speed_out": by_speed,
        "outputs": {"table": str(csv_out), "figure": str(png_out) if make_plot else None},
    }
    json_out.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    summary["outputs"]["summary"] = str(json_out)
    summary["table"] = table
    return summary


def print_report(summary):
    table = summary["table"]
    gain = summary["recommended_gain"]
    counts = " / ".join(f"{k} {v}本" for k, v in summary["files_by_speed"].items())
    print(f"\n=== 歩幅校正: 校正用 {summary['n_files']}本 ({counts}) ===")
    print(f"ファイル別(推奨ゲイン {gain:.3f} を掛けた場合。LOO誤差 = そのファイルを外して"
          "求めたゲインで推定した誤差)")
    view = pd.DataFrame({
        "file": table["file"],
        "速さ": table["speed"].replace("", "不明"),
        "距離m": table["distance_m"].map("{:.1f}".format),
        "歩数": table["n_steps"],
        "歩調/s": table["cadence_hz"].map("{:.2f}".format),
        "真の歩幅": table["true_stride_m"].map("{:.3f}".format),
        "推定歩幅": table["est_stride_m"].map("{:.3f}".format),
        "推定総距離": table["est_total_m"].map("{:.1f}".format),
        "誤差%": table["error_pct"].map("{:+.1f}".format),
        "上限クリップ%": table["clip_upper_pct"].map("{:.0f}".format),
        "LOO誤差%": table["loo_error_pct"].map("{:+.1f}".format),
    })
    print(view.to_string(index=False))
    pooled = summary["pooled_total_error_pct"]
    print(f"\n推奨ゲイン(総距離一致・クリップ込み): {gain:.3f}  "
          f"(全体の総距離誤差 {pooled['recommended']:+.1f}%)")
    print(f"参考ゲイン(総距離比・2.10と同じ求め方): {summary['ratio_gain']:.3f}  "
          f"(全体の総距離誤差 {pooled['ratio']:+.1f}%)")
    print(f"現在の設定値: {summary['current_gain_in_config']:.3f}  "
          f"(全体の総距離誤差 {pooled['current']:+.1f}%)")
    loo = summary["leave_one_out"]
    if loo:
        print(f"leave-one-out: ゲイン {loo['gain_min']:.3f}〜{loo['gain_max']:.3f} "
              f"(全体の値から最大 {loo['max_abs_dev_pct']:.1f}%)、外したファイルの総距離誤差 "
              f"RMS {loo['heldout_rms_error_pct']:.1f}% / 最大 {loo['heldout_max_abs_error_pct']:.1f}%")
    else:
        print("leave-one-out: 校正データが1本なので実行できない")
    if summary["leave_one_speed_out"]:
        parts = [f"{s}を外す→ゲイン{v['gain']:.3f}、{s}の誤差{v['mean_error_pct']:+.1f}%"
                 for s, v in summary["leave_one_speed_out"].items()]
        print("速さごとに外した場合: " + " / ".join(parts))
    print("\n保存先:")
    for path in summary["outputs"].values():
        if path:
            print(f"  {path}")
    print("※ kanri_4f.json の step_length_calibration_gain への反映は手動で行うこと。")


def _write_synthetic_walk(path, cadence_hz, amplitude, n_steps, rng, hz=50.0):
    """架空の歩行記録(前後に静止)。数値は研究結果ではない。"""
    still = 3.0
    walk = n_steps / cadence_hz
    t = np.arange(0.0, still + walk + 2.0, 1.0 / hz)
    walking = (t >= still) & (t < still + walk)
    # 1周期=1歩。位相を-90度ずらし、歩き始めと終わりで信号が0から立ち上がるようにする。
    wave = amplitude * (np.sin(2 * np.pi * cadence_hz * (t - still) - np.pi / 2) + 1.0) / 2
    acc_z = 9.81 + np.where(walking, wave, 0.0) + rng.normal(0, 0.02, len(t))
    pd.DataFrame({
        "timestamp": t,
        "acc_x": rng.normal(0, 0.02, len(t)), "acc_y": rng.normal(0, 0.02, len(t)),
        "acc_z": acc_z,
        "gyro_x": np.zeros(len(t)), "gyro_y": np.zeros(len(t)), "gyro_z": np.zeros(len(t)),
    }).to_csv(path, index=False)


def _self_test():
    """[本研究独自] 架空の歩行記録で、一覧表の検査・本体関数との照合・ゲインの計算・
    leave-one-out・出力を確認する。外部ファイル不要で、書き込みは一時フォルダだけ。

    【重要】ここで使う信号はすべて架空であり、出てくる数値を研究結果として扱わない。
    """
    print("--- self-test 開始(架空データ。数値は研究結果ではない) ---")
    pdrmod.load_map_config_for_tool(PROGRAM_DIR / "map_configs" / "kanri_4f.json")
    rng = np.random.default_rng(0)
    # (速さ, 歩調Hz, 振幅, 歩数, 架空の真の歩幅m)
    walks = [("slow", 1.5, 2.0, 40, 0.60), ("slow", 1.5, 2.1, 40, 0.61),
             ("normal", 1.8, 3.0, 40, 0.72), ("normal", 1.8, 3.1, 40, 0.73),
             ("fast", 2.1, 4.0, 40, 0.84), ("fast", 2.1, 4.2, 40, 0.86)]
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        (td / "calib").mkdir()
        lines = ["file,purpose,route,distance_m,speed,use,memo"]
        for k, (speed, cadence, amp, n, stride) in enumerate(walks):
            name = f"calib/pdr_log_test_{k}.csv"
            _write_synthetic_walk(td / name, cadence, amp, n, rng)
            # memoにカンマを含めても列がずれないことも確かめる
            lines.append(f"{name},calib,,{n * stride:.2f},{speed},1,架空(x=1,y=2)")
        lines.append("calib/pdr_log_retake.csv,calib,,30,fast,0,撮り直し(使わない)")
        lines.append("pdr_log_eval.csv,eval,east_std,,,,評価用の行は読まない")
        good_list = td / "list.csv"
        good_list.write_text("\n".join(lines) + "\n", encoding="cp932")  # Excel保存を想定

        # 一覧表の検査: 誤りは行番号付きでまとめてエラーにする
        bad_list = td / "bad.csv"
        bad_list.write_text("file,purpose,distance_m,speed\n"
                            "a.csv,calib,,slow\nb.csv,calib,30,walk\n", encoding="utf-8")
        try:
            load_measurement_list(bad_list, "calib")
            raise AssertionError("誤りのある一覧表がエラーにならなかった")
        except ValueError as error:
            assert "2行目" in str(error) and "3行目" in str(error), error
        print("  OK: 一覧表の誤り(距離の空欄・想定外の速さ)を行番号付きで検出")

        rows = load_measurement_list(good_list, "calib")
        assert len(rows) == len(walks), rows
        assert (rows["memo"] == "架空(x=1,y=2)").all(), rows["memo"].tolist()
        print("  OK: use=0 の行と eval の行を読み飛ばし、Shift-JIS・memo内のカンマも正しく読める")

        summary = run_calibration(good_list, td, td / "out", tag="selftest")
        table = summary["table"]
        # 歩数検出が本体の関数で正しく動いている(架空信号の歩数を±1歩で再現)
        for (_, _, _, n, _), got in zip(walks, table["n_steps"]):
            assert abs(got - n) <= 1, (n, got)
        print(f"  OK: 本体のステップ検出で歩数を再現 ({table['n_steps'].tolist()})")
        # 総距離一致方式の定義どおり、全体の総距離が一致する
        assert abs(summary["pooled_total_error_pct"]["recommended"]) < 1e-6, summary
        # 上限クリップに当たらなければ、2つの求め方は一致する
        if (table["clip_upper_pct"] == 0).all() and (table["clip_lower_pct"] == 0).all():
            assert abs(summary["recommended_gain"] / summary["ratio_gain"] - 1) < 1e-6
            print("  OK: クリップが無いとき、総距離一致方式と総距離比方式が一致")
        assert summary["leave_one_out"] is not None and table["loo_gain"].notna().all()
        assert set(summary["leave_one_speed_out"]) == {"slow", "normal", "fast"}
        print(f"  OK: 推奨ゲイン {summary['recommended_gain']:.3f}(架空)で全体の総距離が一致、"
              "leave-one-out と速さ別の検証も計算できた")
        for key in ("table", "figure", "summary"):
            assert Path(summary["outputs"][key]).exists(), key
        print("  OK: 表(CSV)・図(PNG)・要約(JSON)を保存できた(一時フォルダ内)")

        # 上限クリップに当たるとき、総距離比方式は総距離を過小にする(推奨方式を選んだ理由)
        # 1本目は g=1.9、2本目は g=2.5 を要する。総距離比(2.125)では1本目が上限1.0mに当たる。
        items = [{"raw_m": np.full(50, 0.50), "distance_m": 50 * 0.95},
                 {"raw_m": np.full(50, 0.30), "distance_m": 50 * 0.75}]
        ratio, matched = gain_by_ratio(items), gain_by_matching(items)
        assert sum(total_m(i, ratio) for i in items) < sum(i["distance_m"] for i in items)
        assert abs(sum(total_m(i, matched) for i in items)
                   - sum(i["distance_m"] for i in items)) < 1e-6
        print(f"  OK: 上限クリップがあると総距離比方式({ratio:.3f})は過小、"
              f"総距離一致方式({matched:.3f})は一致")
    print("--- self-test 全て通過 ---")


def main():
    p = argparse.ArgumentParser(
        description="距離既知の直線歩行データから、全CSV共通の歩幅校正ゲインを求める。"
                    "詳細はこのファイル冒頭のコメントを参照。")
    p.add_argument("--list", type=Path, help="計測一覧表(purpose=calib の行を使う)。")
    p.add_argument("--data-dir", type=Path, default=None,
                   help="CSVフォルダ。環境変数PDR_DATA_DIRとJSONのdata_dirより優先する。")
    p.add_argument("--map-config", type=Path,
                   default=PROGRAM_DIR / "map_configs" / "kanri_4f.json")
    p.add_argument("--tag", default=None, help="出力ファイル名に付ける目印(例: 0925)。")
    p.add_argument("--self-test", action="store_true",
                   help="架空データで計算ロジックだけを確認する(実データ不要)。")
    a = p.parse_args()

    if a.self_test:
        _self_test()
        return
    if a.list is None:
        p.error("--list を指定するか、--self-test を使ってください。")

    # M_TO_PIXEL・STEP_LENGTH_CALIBRATION_GAIN等の本体のグローバルを設定する。
    # これを呼ばずに歩幅関数を使うと、本体と違う定数で計算した数字が出る。
    _cfg, resolved = pdrmod.load_map_config_for_tool(a.map_config)
    data_dir = a.data_dir.expanduser().resolve() if a.data_dir else resolved.data_dir
    print(f"CSVフォルダ: {data_dir}")
    try:
        summary = run_calibration(a.list, data_dir, RESULTS_DIR, tag=a.tag,
                                  map_config=a.map_config.resolve())
    except ValueError as error:
        raise SystemExit(f"エラー: {error}")
    print_report(summary)


if __name__ == "__main__":
    main()
