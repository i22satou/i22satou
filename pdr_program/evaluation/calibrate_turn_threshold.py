# ============================================================================
# calibrate_turn_threshold.py
#
# 【変更履歴】
# - 2026-09-24: [本研究独自] 新規作成。移動様態判定の「曲がり終了のヨーレートしきい値」
#               (adaptive_pf.turn_exit_yaw_rate_threshold_deg_s)を、校正用の直線歩行から決める。
#
# 【なぜ必要か】
# 曲がりを終える条件は「直近1.5秒の方位変化が6度未満」かつ「ヨーレート絶対値の75パーセン
# タイルがしきい値未満」。しきい値は2026-09-24まで開始側(20度/秒)の半分の10度/秒に固定して
# いたが、まっすぐ歩いていても歩行の揺れで75パーセンタイルは18〜20度/秒あり(0805の
# 1441・1442)、10度/秒を下回る歩は1〜3%しかなかった。そのため一度曲がりと判定されると
# 戻らず、歩の81〜99%が曲がり判定になり、移動様態適応PF(方式C)が固定粒子数PF(方式B)と
# ほぼ同じ動きになっていた(memo/comparison_methods.md)。
#
# 【方針(ユーザー承認、2026-09-24)】
# - しきい値は評価用データではなく、校正用の直線歩行(計測一覧表の purpose=calib の行)から
#   決める。calib の記録は「ずっと直進」なので、直進中の揺れの大きさをそのまま測れる。
# - 直進中の各歩について、本体と同じ関数(behavior_window_stats)でヨーレートの75パーセン
#   タイルを求め、全記録をまとめた分布の95パーセンタイル(--percentile)を推奨値とする。
#   「直進中の歩の95%で、終了条件のヨーレート側を満たす」値という意味。
# - 判定窓(1.5秒)が歩き始める前の静止にかかる歩は使わない(静止中は揺れが無く、値が
#   小さく出るため)。
# - 方位変化のしきい値(6度)は変えない。参考として、直進中にそれを満たす歩の割合も出す。
# - JSONは書き換えない。推奨値を確かめてから map_configs/kanri_4f.json の
#   adaptive_pf.turn_exit_yaw_rate_threshold_deg_s を手で書き換える。
# - 確認として、同じ calib の記録で、今のしきい値と推奨値のそれぞれで本体の判定
#   (detect_move_behavior)を再現し、「曲がり」と判定される歩の割合を出す(直進なので
#   小さいほど良い)。方位は比較実験と同じ android(yaw_deg列)を使う。
#
# 【出力】results/<日時>_turn_threshold[_tag].csv(1歩1行)/ _summary.json / .png
#
# 【使い方】pdr_program/ で実行する。CSVフォルダは本体と同じ規則で決める。
#   python evaluation/calibrate_turn_threshold.py --list <計測一覧.csv> [--tag 0925]
#   python evaluation/calibrate_turn_threshold.py --self-test
# ============================================================================

import argparse
import contextlib
import hashlib
import json
import math
import sys
import tempfile
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # 図はファイルへ保存するだけなので画面を使わない
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

EVALUATION_DIR = Path(__file__).resolve().parent
PROGRAM_DIR = EVALUATION_DIR.parent
sys.path.insert(0, str(PROGRAM_DIR))
sys.path.insert(0, str(EVALUATION_DIR))
import pdr_pf_improved as pdrmod  # noqa: E402
from calibrate_step_length import git_revision  # noqa: E402
from measurement_list import load_measurement_list, resolve_csv_path  # noqa: E402

RESULTS_DIR = PROGRAM_DIR / "results"
DEFAULT_PERCENTILE = 95.0
MIN_STEPS_WARN = 100   # これより歩数が少ないと、95パーセンタイルが安定しない
INK, INK_MUTED, GRID = "#0b0b0b", "#52514e", "#e6e5e0"
BAR, RECOMMENDED, CURRENT = "#9ec5f4", "#2a78d6", "#eb6834"


@contextlib.contextmanager
def exit_threshold_override(threshold_rad):
    """本体の曲がり終了のヨーレートしきい値を一時的に差し替え、必ず戻す。"""
    saved = pdrmod.TURN_EXIT_YAW_RATE_THRESHOLD
    try:
        pdrmod.TURN_EXIT_YAW_RATE_THRESHOLD = threshold_rad
        yield
    finally:
        pdrmod.TURN_EXIT_YAW_RATE_THRESHOLD = saved


def analyze_file(csv_path):
    """本体と同じ前処理で、歩ごとの方位変化とヨーレート75パーセンタイルを求める。

    ヨーレートと方位の履歴は、本体のメインループと同じ規則で作る(サンプル間隔が0以下か
    MAX_DTを超える行は0のまま)。方位はandroid(yaw_deg)。列が無ければ方位変化はNaN。
    """
    df = pdrmod.validate_log(pdrmod.safe_read_csv(csv_path), csv_path.name)
    if len(df) < 2:
        raise ValueError("有効な行が2行未満")
    if pdrmod.GYRO_UNIT == "deg":
        cols = ["gyro_x", "gyro_y", "gyro_z"]
        df[cols] = np.deg2rad(df[cols])
    df["acc_mag"] = pdrmod.compute_acc_magnitude(df)
    df["step_acc"] = pdrmod.compute_step_acceleration(df["acc_mag"])
    t = df["timestamp"].to_numpy(float)
    dt_mean = float(np.mean(np.diff(t)))
    steps, _ = pdrmod.detect_steps_smartpdr(df["step_acc"], 1.0 / dt_mean if dt_mean > 0 else None)
    if len(steps) < 2:
        raise ValueError(f"歩数が{len(steps)}歩しか検出されない")

    g = df[["gyro_x", "gyro_y", "gyro_z"]].to_numpy(float)
    a = df[["acc_x", "acc_y", "acc_z"]].to_numpy(float)
    has_yaw = "yaw_deg" in df.columns and df["yaw_deg"].notna().any()
    yaw = np.deg2rad(pd.to_numeric(df["yaw_deg"], errors="coerce").to_numpy(float)) if has_yaw else None
    yaw_rate = np.zeros(len(df))
    heading = np.zeros(len(df))
    reference = None
    for i in range(1, len(df)):
        dt = t[i] - t[i - 1]
        if dt <= 0 or dt > pdrmod.MAX_DT:
            continue
        yaw_rate[i] = pdrmod.get_yaw_rate(*g[i], *a[i])
        if has_yaw and np.isfinite(yaw[i]):
            if reference is None:
                reference = yaw[i]
            heading[i] = pdrmod.normalize_angle(yaw[i] - reference)
        else:
            heading[i] = np.nan

    first_step_t = t[steps[0]]
    rows = []
    for number, i in enumerate(steps, start=1):
        if t[i] - pdrmod.BEHAVIOR_WINDOW_SEC < first_step_t:
            continue  # 判定窓が歩き始める前の静止にかかる歩は使わない
        hc, p75 = pdrmod.behavior_window_stats(t, heading, yaw_rate, i)
        rows.append({"step_no": number, "t_sec": float(t[i] - t[0]),
                     "heading_change_deg": float(np.rad2deg(hc)) if has_yaw else np.nan,
                     "yaw_rate_p75_deg_s": float(np.rad2deg(p75))})
    return {"t": t, "steps": steps, "heading": heading if has_yaw else None,
            "yaw_rate": yaw_rate, "rows": rows, "n_steps": len(steps)}


def turning_fraction(item, threshold_rad):
    """本体の判定を歩ごとに再現し、「曲がり」と判定された歩の割合を返す(方位が無ければNaN)。"""
    if item["heading"] is None:
        return float("nan")
    with exit_threshold_override(threshold_rad):
        previous = pdrmod.MoveBehavior.STRAIGHT
        turning = 0
        for i in item["steps"]:
            previous = pdrmod.detect_move_behavior(item["t"], item["heading"], item["yaw_rate"], i,
                                                   previous, step_detected=True)
            turning += previous == pdrmod.MoveBehavior.TURNING
    return turning / len(item["steps"])


def plot_distribution(table, recommended, current, entry, percentile, out_png, title_prefix=""):
    values = table["yaw_rate_p75_deg_s"].to_numpy(float)
    fig, ax = plt.subplots(figsize=(7.5, 4.2))
    top = max(values.max(), recommended, entry) * 1.1
    ax.hist(values, bins=np.arange(0, top + 1.0, 1.0), color=BAR, edgecolor="white", linewidth=0.6)
    ymax = ax.get_ylim()[1]
    for x, color, style, label in (
            (current, CURRENT, "--", f"今のしきい値 {current:.1f}"),
            (recommended, RECOMMENDED, "-", f"推奨値({percentile:g}パーセンタイル) {recommended:.1f}"),
            (entry, INK_MUTED, ":", f"曲がり開始のしきい値 {entry:.1f}")):
        ax.axvline(x, color=color, linestyle=style, linewidth=1.6)
        ax.text(x, ymax * 0.97, " " + label, color=INK, fontsize=8, va="top", rotation=90,
                ha="right", bbox=dict(facecolor="white", edgecolor="none", pad=1.0, alpha=0.85))
    ax.set_xlabel("直近1.5秒のヨーレート絶対値の75パーセンタイル [度/秒](直進中の1歩ごと)",
                  color=INK_MUTED, fontsize=9)
    ax.set_ylabel("歩数", color=INK_MUTED, fontsize=9)
    ax.set_title(f"{title_prefix}曲がり終了のヨーレートしきい値の校正"
                 f"(校正用の直線歩行 {table['file'].nunique()}本・{len(table)}歩)", color=INK, fontsize=10)
    ax.grid(True, axis="y", color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(INK_MUTED)
    ax.tick_params(colors=INK_MUTED, labelsize=8)
    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)


def calibrate(list_path, data_dir, map_config, out_root=RESULTS_DIR, tag=None,
              percentile=DEFAULT_PERCENTILE, synthetic=False):
    """一覧表のcalibの行から推奨しきい値を求め、表・図・要約を保存して要約を返す。"""
    rows = load_measurement_list(list_path, "calib")
    if rows.empty:
        raise ValueError(f"{list_path}: purpose=calib かつ use=1 の行がありません。")
    pdrmod.load_map_config_for_tool(map_config)
    current = float(np.rad2deg(pdrmod.TURN_EXIT_YAW_RATE_THRESHOLD))
    entry = float(np.rad2deg(pdrmod.TURN_YAW_RATE_THRESHOLD))
    exit_heading = float(np.rad2deg(pdrmod.TURN_EXIT_THRESHOLD))

    items, errors, records = [], [], []
    for _, row in rows.iterrows():
        path = resolve_csv_path(row["file"], data_dir)
        try:
            item = analyze_file(path)
        except (ValueError, IOError) as error:
            errors.append(f"{row['file']}: {error}")
            continue
        item.update({"file": row["file"], "speed": row["speed"] or ""})
        items.append(item)
        for r in item["rows"]:
            records.append({"file": row["file"], "speed": item["speed"], **r})
    if errors:
        raise ValueError("読めない校正用CSVがあります(使わない場合は一覧表で use=0):\n  "
                         + "\n  ".join(errors))
    table = pd.DataFrame(records)
    if table.empty:
        raise ValueError("直進中の歩が1歩もありません(記録が短すぎる可能性)。")

    values = table["yaw_rate_p75_deg_s"].to_numpy(float)
    recommended = float(np.percentile(values, percentile))
    recommended_rounded = math.ceil(recommended * 2.0) / 2.0  # 0.5度/秒単位に切り上げ
    warnings = []
    if len(table) < MIN_STEPS_WARN:
        warnings.append(f"使えた歩が{len(table)}歩と少なく、{percentile:g}パーセンタイルが安定しない"
                        f"(目安{MIN_STEPS_WARN}歩以上)")
    if recommended_rounded >= entry:
        warnings.append(f"推奨値が曲がり開始のヨーレートしきい値({entry:.1f}度/秒)以上。直進中の揺れでも"
                        "開始側のヨーレート条件をほぼ満たすので、曲がりの開始は方位変化"
                        f"({np.rad2deg(pdrmod.TURN_ENTER_THRESHOLD):.1f}度)でほぼ決まる。"
                        "方位変化のしきい値(開始20度・終了6度)の差でヒステリシスは保たれる")

    per_file = []
    for item in items:
        sub = table[table["file"] == item["file"]]
        per_file.append({
            "file": item["file"], "speed": item["speed"], "n_steps": item["n_steps"],
            "n_steps_used": len(sub),
            "p75_median_deg_s": float(sub["yaw_rate_p75_deg_s"].median()) if len(sub) else None,
            "p75_p95_deg_s": float(sub["yaw_rate_p75_deg_s"].quantile(0.95)) if len(sub) else None,
            "heading_change_below_exit_pct": (
                float(100 * np.mean(sub["heading_change_deg"] < exit_heading))
                if len(sub) and sub["heading_change_deg"].notna().any() else None),
            "turning_pct_current": 100 * turning_fraction(item, np.deg2rad(current)),
            "turning_pct_recommended": 100 * turning_fraction(item, np.deg2rad(recommended_rounded)),
        })

    stamp = time.strftime("%Y%m%d_%H%M%S")
    name = f"{stamp}_turn_threshold" + (f"_{tag}" if tag else "") + ("_SYNTHETIC" if synthetic else "")
    out_root = Path(out_root)
    out_root.mkdir(parents=True, exist_ok=True)
    csv_out = out_root / f"{name}.csv"
    png_out = out_root / f"{name}.png"
    json_out = out_root / f"{name}_summary.json"
    saved_table = table.copy()
    if synthetic:
        saved_table.insert(0, "注意", "架空データ(研究結果ではない)")
    saved_table.to_csv(csv_out, index=False, encoding="utf-8-sig", float_format="%.3f")
    plot_distribution(table, recommended_rounded, current, entry, percentile, png_out,
                      "【架空データ・研究結果ではない】" if synthetic else "")
    commit, dirty = git_revision()
    summary = {
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "synthetic": synthetic,
        "git_commit": commit,
        "git_uncommitted_changes": dirty,
        "measurement_list": str(Path(list_path).resolve()),
        "map_config": str(Path(map_config).resolve()),
        "percentile": percentile,
        "n_files": len(items),
        "n_steps_used": int(len(table)),
        "recommended_deg_s": recommended,
        "recommended_rounded_deg_s": recommended_rounded,
        "current_deg_s": current,
        "entry_yaw_rate_deg_s": entry,
        "exit_heading_change_deg": exit_heading,
        "per_file": per_file,
        "warnings": warnings,
        "how_to_apply": ("map_configs の adaptive_pf.turn_exit_yaw_rate_threshold_deg_s を "
                         f"{recommended_rounded:g} に書き換える(このスクリプトはJSONを変更しない)"),
    }
    json_out.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    summary.update({"csv": csv_out, "png": png_out, "json": json_out})
    return summary


def print_report(summary):
    print(f"\n使った歩: {summary['n_files']}本・{summary['n_steps_used']}歩")
    print(f"推奨値({summary['percentile']:g}パーセンタイル): {summary['recommended_deg_s']:.2f} 度/秒"
          f" → {summary['recommended_rounded_deg_s']:g} 度/秒(0.5度/秒単位に切り上げ)")
    print(f"今のしきい値: {summary['current_deg_s']:g} 度/秒")
    print("\n記録ごと(直進なので「曲がり」の割合は小さいほど良い):")
    for f in summary["per_file"]:
        heading = ("" if f["heading_change_below_exit_pct"] is None
                   else f" 方位変化<終了しきい値の歩 {f['heading_change_below_exit_pct']:.0f}%")
        print(f"  {f['file']} ({f['speed'] or '速さ不明'}) 歩 {f['n_steps_used']}/{f['n_steps']}"
              f" 75%値の中央値 {f['p75_median_deg_s']:.1f} 度/秒{heading}"
              f" | 曲がりの割合 今 {f['turning_pct_current']:.0f}% → 推奨値 {f['turning_pct_recommended']:.0f}%")
    for w in summary["warnings"]:
        print(f"[注意] {w}")
    print(f"\n反映のしかた: {summary['how_to_apply']}")
    print(f"保存先: {summary['csv']}\n        {summary['png']}\n        {summary['json']}")


def _write_synthetic_straight_walk(path, n_steps, cadence, sway_deg_s, rng, hz=50.0,
                                   start_swing_deg=30.0):
    """架空の直線歩行。前に3秒・後に2秒の静止。歩行中は歩調と同じ周期でヨーが揺れ
    (振幅sway_deg_s)、歩き始めの1秒で方位がstart_swing_deg動く(0805の1441の歩き始めに
    似せた、曲がり判定に入るきっかけ)。数値は研究結果ではない。"""
    still = 3.0
    walk = n_steps / cadence
    t = np.arange(0.0, still + walk + 2.0, 1.0 / hz)
    walking = (t >= still) & (t < still + walk)
    bounce = 3.0 * (np.sin(2 * np.pi * cadence * (t - still) - np.pi / 2) + 1.0) / 2
    gyro_z = np.where(walking, np.deg2rad(sway_deg_s) * np.sin(2 * np.pi * cadence * (t - still)), 0.0)
    yaw = 37.0 + start_swing_deg * np.clip(t - still, 0.0, 1.0)
    pd.DataFrame({
        "timestamp": 1000.0 + t,
        "acc_x": rng.normal(0, 0.02, len(t)), "acc_y": rng.normal(0, 0.02, len(t)),
        "acc_z": 9.81 + np.where(walking, bounce, 0.0) + rng.normal(0, 0.02, len(t)),
        "gyro_x": np.zeros(len(t)), "gyro_y": np.zeros(len(t)), "gyro_z": gyro_z,
        "yaw_deg": yaw,
    }).to_csv(path, index=False)


def _self_test():
    """[本研究独自] 架空の直線歩行で、推奨値の計算と判定の再現が正しく動くかを確かめる。

    【重要】ここで使う信号はすべて架空であり、出てくる数値を研究結果として扱わない。
    ヨーの揺れを振幅Aの正弦波にすると、|A sin|の75パーセンタイルは A×sin(67.5度)≒0.924A
    になるので、推奨値がその近くに出るかを見る。
    """
    print("--- self-test 開始(架空データ。数値は研究結果ではない) ---")
    config = PROGRAM_DIR / "map_configs" / "kanri_4f.json"
    config_md5 = hashlib.md5(config.read_bytes()).hexdigest()
    results_before = sorted(p.name for p in RESULTS_DIR.iterdir()) if RESULTS_DIR.exists() else []
    sway = 25.0
    rng = np.random.default_rng(0)
    with tempfile.TemporaryDirectory(prefix="turn_threshold_selftest_") as td:
        td = Path(td)
        (td / "data" / "calib").mkdir(parents=True)
        walks = [("calib/pdr_log_9002_0001.csv", "slow", 1.5), ("calib/pdr_log_9002_0002.csv", "normal", 1.8),
                 ("calib/pdr_log_9002_0003.csv", "fast", 2.1)]
        lines = ["file,purpose,route,distance_m,speed,use,memo"]
        for name, speed, cadence in walks:
            _write_synthetic_straight_walk(td / "data" / name, 60, cadence, sway, rng)
            lines.append(f"{name},calib,,30,{speed},1,架空")
        lines.append("pdr_log_9002_0009.csv,eval,east_std,,,1,校正では読まない")
        (td / "list.csv").write_text("\n".join(lines) + "\n", encoding="utf-8")

        summary = calibrate(td / "list.csv", td / "data", config, out_root=td / "out",
                            tag="selftest", synthetic=True)
        expected = sway * math.sin(math.radians(67.5))
        assert abs(summary["recommended_deg_s"] - expected) < 1.5, (summary["recommended_deg_s"], expected)
        print(f"  OK: 推奨値 {summary['recommended_deg_s']:.2f} 度/秒が、揺れの振幅{sway:g}度/秒から"
              f"決まる75%値 {expected:.2f} の近く")
        assert summary["n_files"] == 3 and summary["n_steps_used"] > 150, summary["n_steps_used"]
        print(f"  OK: calibの3本だけを読み(evalの行は読まない)、歩き始めの静止にかかる歩を除いた"
              f"{summary['n_steps_used']}歩を使う")
        for f in summary["per_file"]:
            assert f["turning_pct_current"] > 80, f
            assert f["turning_pct_recommended"] < 20, f
        print("  OK: 今のしきい値(10度/秒)では歩き始めに入った「曲がり」が終わらず8割超、"
              "推奨値では2割未満に下がる(実データで起きていることの再現)")
        assert any("曲がり開始のヨーレートしきい値" in w for w in summary["warnings"]), summary["warnings"]
        print("  OK: 推奨値が曲がり開始のしきい値以上になったときに注意を出す")
        for key in ("csv", "png", "json"):
            assert Path(summary[key]).exists(), key
        saved = json.loads(Path(summary["json"]).read_text(encoding="utf-8"))
        assert saved["synthetic"] is True
        print("  OK: 1歩ごとの表・図・要約を保存し、架空データであることを明記")

        # 本体の判定をこのスクリプトが一時的に差し替えても、元に戻っていること
        assert math.isclose(pdrmod.TURN_EXIT_YAW_RATE_THRESHOLD, math.radians(summary["current_deg_s"]))
    assert hashlib.md5(config.read_bytes()).hexdigest() == config_md5, "設定JSONが変わった"
    results_after = sorted(p.name for p in RESULTS_DIR.iterdir()) if RESULTS_DIR.exists() else []
    assert results_after == results_before, "本物の results/ にファイルが増えた"
    print("  OK: 設定JSONと本物の results/ は変わっていない")
    print("--- self-test 全て通過 ---")


def main():
    p = argparse.ArgumentParser(
        description="校正用の直線歩行から、曲がり終了のヨーレートしきい値の推奨値を求める。"
                    "詳細はこのファイル冒頭のコメントを参照。")
    p.add_argument("--list", type=Path, help="計測一覧表(purpose=calib の行を使う)。")
    p.add_argument("--data-dir", type=Path, default=None,
                   help="CSVフォルダ。環境変数PDR_DATA_DIRとJSONのdata_dirより優先する。")
    p.add_argument("--map-config", type=Path, default=PROGRAM_DIR / "map_configs" / "kanri_4f.json")
    p.add_argument("--percentile", type=float, default=DEFAULT_PERCENTILE,
                   help=f"直進中の歩の分布の何パーセンタイルを推奨値にするか(既定{DEFAULT_PERCENTILE:g})。")
    p.add_argument("--tag", default=None, help="出力ファイル名に付ける目印(例: 0925)。")
    p.add_argument("--self-test", action="store_true", help="架空データで計算を確かめる(実データ不要)。")
    a = p.parse_args()

    if a.self_test:
        _self_test()
        return
    if a.list is None:
        p.error("--list を指定するか、--self-test を使ってください。")
    _cfg, resolved = pdrmod.load_map_config_for_tool(a.map_config)
    data_dir = a.data_dir.expanduser().resolve() if a.data_dir else resolved.data_dir
    try:
        summary = calibrate(a.list, data_dir, a.map_config, tag=a.tag, percentile=a.percentile)
    except ValueError as error:
        raise SystemExit(f"エラー: {error}")
    print_report(summary)


if __name__ == "__main__":
    main()
