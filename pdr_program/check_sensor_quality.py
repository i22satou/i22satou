# ============================================================================
# check_sensor_quality.py
#
# 【変更履歴】
# - 2026-08-16: コード肥大化対策として、fake_args組み立てコード(pick_landmarks.py
#               ・verify_route_graph.pyと3重に重複していた)をpdr_pf_improved.py側の
#               load_map_config_for_tool()に一本化し、このスクリプトはそれを
#               呼ぶだけにした。診断ロジック自体に変更はない。
# - 2026-08-16: pdr_pf_improved.py側でapply_map_config()がmulti_hypothesis_
#               routing_enabled等の新しいargs属性を参照するようになった
#               (2026-08-16の複数経路仮説PF追加)ため、再びfake_argsに存在しない
#               属性を読もうとしてAttributeErrorで落ちるようになっていた
#               (verify_route_graph.pyの動作確認で発見。前回と同種の問題)。
#               fake_argsに不足していた属性を追加して修正。診断ロジック自体に
#               変更はない。
# - 2026-08-16: pdr_pf_improved.py側でapply_map_config()がauto_route_exclude_
#               wide_rooms等の新しいargs属性を参照するようになった(2026-08-16の
#               exclude_wide_rooms追加)ため、本スクリプトのfake_argsに存在しない
#               属性を読もうとしてAttributeErrorで落ちるようになっていた
#               (pick_landmarks.py作成時の動作確認で発見)。fake_argsに不足して
#               いた属性を追加して修正。診断ロジック自体に変更はない。
# - 2026-08-15: 新規作成。CSVごとのジャイロ/加速度/yaw_degの品質を定量化する
#               読み取り専用の診断ツール。
#
# 【本スクリプトの位置づけ】[本研究独自]
# compare_route_source.py(方式間の比較)の結果、pdr_log_0805_1441.csvと
# pdr_log_0805_1442.csvは開始位置・移動様態の判定傾向がほぼ同じにもかかわらず
# 全滅回数・到達距離が大きく異なることが分かった(CLAUDE_MEMO.md参照)。route_mask
# の形状では説明できないため、「CSVごとの生センサーデータの質そのものが違うのでは
# ないか」という仮説を検証するために作成した。
#
# pdr_pf_improved.py本体(PDR/PF処理)には一切手を加えず、同ファイルの
# get_yaw_rate()・load_map_config()・apply_map_config()・validate_log()を
# import して再利用するだけの、生データに対する統計計算のみを行う。
#
# 計算する指標:
#   - yaw_rate(ジャイロ角速度を重力方向へ投影した値。移動様態判定に実際に
#     使われている量そのもの): 標準偏差・絶対値の最大値・
#     turn_yaw_rate_threshold_deg_s(既定20deg/s)を超えるサンプルの割合
#     (この割合が高いCSVほど「曲がり」に誤判定されやすい)。
#   - acc_mag(加速度合成値、重力込み): 標準偏差(手ぶれ・振動の指標)。
#   - yaw_deg(Android回転ベクトルセンサ由来、heading-source=androidで実際に
#     使われる方位そのもの。列が無いCSVでは計算しない):
#     サンプル間の差分の標準偏差・最大絶対値(値が大きいほど、機器融合方位が
#     瞬間的に大きく飛ぶ=磁気外乱や再キャリブレーションが起きている可能性)、
#     および記録全体を通じた正味の方位変化量(単純に不連続差分を積算)。
# ============================================================================

import argparse
import csv as csv_module
import datetime
import sys
from pathlib import Path

import numpy as np
import pandas as pd

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
import pdr_pf_improved as pdrmod  # noqa: E402

RESULTS_DIR = SCRIPT_DIR / "results"


def analyze_csv(path, gyro_unit, turn_yaw_rate_threshold_rad):
    df = pd.read_csv(path)
    df = pdrmod.validate_log(df, path.name)
    if gyro_unit == "deg":
        df[["gyro_x", "gyro_y", "gyro_z"]] = np.deg2rad(df[["gyro_x", "gyro_y", "gyro_z"]])
    if len(df) < 2:
        return None

    duration_sec = float(df["timestamp"].iloc[-1] - df["timestamp"].iloc[0])
    n_samples = len(df)
    sampling_hz = n_samples / duration_sec if duration_sec > 0 else float("nan")

    # pdr_pf_improved.get_yaw_rate()は1行ずつのスカラー入力を想定しており(if norm < 1e-6
    # の分岐が配列だと使えない)、同じ計算式(ジャイロ角速度と重力方向の内積)を配列向けに
    # 書き直したもの。数式自体はget_yaw_rate()と完全に同一。
    gx = df["gyro_x"].to_numpy()
    gy = df["gyro_y"].to_numpy()
    gz = df["gyro_z"].to_numpy()
    ax = df["acc_x"].to_numpy()
    ay = df["acc_y"].to_numpy()
    az = df["acc_z"].to_numpy()
    norm = np.sqrt(ax**2 + ay**2 + az**2)
    safe_norm = np.where(norm < 1e-6, 1.0, norm)
    yaw_rate = np.where(norm < 1e-6, gz, (gx * ax + gy * ay + gz * az) / safe_norm)
    yaw_rate_std_dps = float(np.std(yaw_rate)) * 180.0 / np.pi
    yaw_rate_max_dps = float(np.max(np.abs(yaw_rate))) * 180.0 / np.pi
    turning_ratio = float(np.mean(np.abs(yaw_rate) > turn_yaw_rate_threshold_rad))

    acc_mag = pdrmod.compute_acc_magnitude(df)
    acc_mag_std = float(np.std(acc_mag))

    result = {
        "file": path.name,
        "n_samples": n_samples,
        "duration_sec": round(duration_sec, 1),
        "sampling_hz": round(sampling_hz, 1),
        "yaw_rate_std_dps": round(yaw_rate_std_dps, 1),
        "yaw_rate_max_dps": round(yaw_rate_max_dps, 1),
        "turning_ratio": round(turning_ratio, 3),
        "acc_mag_std": round(acc_mag_std, 3),
    }

    if "yaw_deg" in df.columns and df["yaw_deg"].notna().sum() > 1:
        yaw_deg = pd.to_numeric(df["yaw_deg"], errors="coerce").to_numpy()
        diffs = np.diff(yaw_deg)
        # -180/+180をまたぐ不連続を補正(単純な折り返しのみ、実際の急変とは区別しない)
        diffs = (diffs + 180) % 360 - 180
        result["yaw_deg_diff_std"] = round(float(np.std(diffs)), 2)
        result["yaw_deg_diff_max"] = round(float(np.max(np.abs(diffs))), 2)
        result["yaw_deg_net_change"] = round(float(np.sum(diffs)), 1)
    else:
        result["yaw_deg_diff_std"] = None
        result["yaw_deg_diff_max"] = None
        result["yaw_deg_net_change"] = None

    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-config", type=Path, default=SCRIPT_DIR / "map_configs" / "kanri_4f.json")
    parser.add_argument("--include-excluded", action="store_true",
                         help="exclude_csv設定で除外されているCSVも診断対象に含める。")
    args_cli = parser.parse_args()

    # pdr_pf_improved.pyのapply_map_config()をそのまま流用し、data_dir/gyro_unit/
    # turn_yaw_rate_threshold/exclude_csvをJSONから一貫した形で取得する。
    _map_config, fake_args = pdrmod.load_map_config_for_tool(args_cli.map_config)

    data_dir = fake_args.data_dir
    file_list = sorted(data_dir.glob("pdr_log_*.csv"))
    if not args_cli.include_excluded and pdrmod.EXCLUDED_CSV_NAMES:
        skipped = [f for f in file_list if f.name in pdrmod.EXCLUDED_CSV_NAMES]
        file_list = [f for f in file_list if f.name not in pdrmod.EXCLUDED_CSV_NAMES]
        for f in skipped:
            print(f"[スキップ] {f.name} (exclude_csv設定。--include-excludedで含められます)")

    rows = []
    for path in file_list:
        try:
            result = analyze_csv(path, pdrmod.GYRO_UNIT, pdrmod.TURN_YAW_RATE_THRESHOLD)
        except (ValueError, IOError) as e:
            print(f"[警告] {path.name}: {e}", file=sys.stderr)
            continue
        if result is None:
            print(f"[警告] {path.name}: 有効なデータが少なすぎます。", file=sys.stderr)
            continue
        rows.append(result)

    if not rows:
        print("診断できたCSVがありません。", file=sys.stderr)
        return

    columns = [
        "file", "n_samples", "duration_sec", "sampling_hz",
        "yaw_rate_std_dps", "yaw_rate_max_dps", "turning_ratio", "acc_mag_std",
        "yaw_deg_diff_std", "yaw_deg_diff_max", "yaw_deg_net_change",
    ]

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = RESULTS_DIR / f"{timestamp}_sensor_quality.csv"
    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv_module.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        for row in rows:
            writer.writerow({c: row.get(c, "") for c in columns})
    print(f"\n診断結果をCSVに保存しました: {out_path}\n")

    header = (
        f"{'file':<24}{'n':>6}{'秒':>7}{'Hz':>6}"
        f"{'yawσ(°/s)':>11}{'yawmax(°/s)':>12}{'曲がり率':>9}{'accσ':>8}"
        f"{'yawdegσ(°)':>12}{'yawdegmax(°)':>13}{'yawdeg純変化(°)':>15}"
    )
    print(header)
    print("-" * len(header))
    for r in rows:
        def fmt(v, width, prec=1):
            if v is None:
                return f"{'—':>{width}}"
            return f"{v:>{width}.{prec}f}"
        print(
            f"{r['file']:<24}{r['n_samples']:>6}{r['duration_sec']:>7.1f}{r['sampling_hz']:>6.1f}"
            f"{r['yaw_rate_std_dps']:>11.1f}{r['yaw_rate_max_dps']:>12.1f}"
            f"{r['turning_ratio']:>9.3f}{r['acc_mag_std']:>8.3f}"
            f"{fmt(r['yaw_deg_diff_std'], 12, 2)}{fmt(r['yaw_deg_diff_max'], 13, 2)}"
            f"{fmt(r['yaw_deg_net_change'], 15, 1)}"
        )


if __name__ == "__main__":
    main()
