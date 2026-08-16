# ============================================================================
# evaluate_accuracy.py
#
# 【変更履歴】
# - 2026-08-16: [本研究独自] 新規作成。正解位置列(タイムスタンプ付き)を用いた
#               平均位置誤差・RMSE・最大誤差の計算基盤(進捗反映版メモ§23の
#               Week1タスク)。実測の正解データは2026年10月に収集予定のため、
#               現時点では合成データによる自己テスト(--self-test)のみで動作を
#               確認している。実データでの数値はまだ存在しない。
#
# 【このスクリプトについて】
# pdr_pf_improved.py --save-trajectory-csv で保存した推定軌跡CSV
# (timestamp, x_px, y_px)と、実測時に記録する正解位置CSV(同じ列構成、
# タイムスタンプは推定軌跡と同じ基準)を突き合わせ、以下を計算する。
#   ・平均位置誤差(mean error)
#   ・RMSE(二乗平均平方根誤差)
#   ・最大位置誤差(max error)
# 正解位置列は推定軌跡よりまばら(例: 数歩ごと、曲がり角ごとの記録)である
# ことを想定し、各正解点のタイムスタンプに対して推定軌跡を線形補間して
# 対応する推定位置を求める(整合していない時刻同士を単純に最近傍対応させると
# 誤差が過大・過小評価されるため)。
#
# 【正解位置CSVの形式】(2026年10月の実測時にこの形式でメモを取る)
#   timestamp, x_px, y_px[, point_type]
#   ・timestamp: 推定軌跡CSVと同じ基準の時刻(pdr_log_*.csvのtimestamp列と同じ単位)。
#   ・x_px, y_px: 地図画像上のピクセル座標。
#   ・point_type: 任意。start/turn/end等のラベル(曲がり位置誤差の計算に使う予定、
#     現時点では未使用)。
#
# 【使い方】
#   python evaluate_accuracy.py --self-test
#     -> 合成データで計算ロジックが正しいかだけを確認する(実データ不要)。
#
#   python evaluate_accuracy.py \
#     --estimated results/pdr_log_XXXX_trajectory.csv \
#     --ground-truth path/to/ground_truth.csv \
#     [--scale-px-per-m 11.4]
#     -> 実データでの評価(正解データが揃う2026年10月以降に使用)。
# ============================================================================

import argparse
import logging

import numpy as np
import pandas as pd

REQUIRED_COLUMNS = ["timestamp", "x_px", "y_px"]


def load_trajectory_csv(path):
    """timestamp, x_px, y_pxの3列を持つCSVを読み込み、
    (timestampの昇順ndarray, [x_px, y_px]のNx2 ndarray)を返す。
    必須列が欠けている場合はValueErrorとする(黙って握り潰さない)。
    """
    df = pd.read_csv(path)
    missing = [c for c in REQUIRED_COLUMNS if c not in df.columns]
    if missing:
        raise ValueError(f"{path}: 必須列が不足しています: {missing}")
    df = df.sort_values("timestamp").reset_index(drop=True)
    timestamps = df["timestamp"].to_numpy(dtype=float)
    positions = df[["x_px", "y_px"]].to_numpy(dtype=float)
    return timestamps, positions


def align_by_timestamp(est_timestamps, est_positions, gt_timestamps, gt_positions):
    """各正解タイムスタンプ(gt_timestamps)に対応する推定位置を、推定軌跡
    (est_timestamps, est_positions)の線形補間で求める。

    正解データは推定軌跡よりまばらな時刻で記録される想定のため、正解の
    タイムスタンプ範囲が推定軌跡の範囲からはみ出す場合はその正解点を除外する
    (外挿による誤差の過大評価を避けるため)。

    戻り値: (対応する推定位置Nx2, 使用した正解位置Nx2, 除外件数)
    """
    if len(est_timestamps) < 2:
        raise ValueError("推定軌跡の点数が少なすぎます(2点以上必要)。")

    t_min, t_max = est_timestamps[0], est_timestamps[-1]
    in_range = (gt_timestamps >= t_min) & (gt_timestamps <= t_max)
    excluded = int(np.sum(~in_range))
    if excluded > 0:
        logging.warning(
            "正解点%d件が推定軌跡の時刻範囲外(%.3f〜%.3f)のため評価から除外されました。",
            excluded, t_min, t_max,
        )

    gt_t = gt_timestamps[in_range]
    gt_p = gt_positions[in_range]
    est_x = np.interp(gt_t, est_timestamps, est_positions[:, 0])
    est_y = np.interp(gt_t, est_timestamps, est_positions[:, 1])
    aligned_est = np.column_stack([est_x, est_y])
    return aligned_est, gt_p, excluded


def compute_position_errors(aligned_est, gt_positions):
    """対応付け済みの推定位置・正解位置(いずれもNx2)から、
    点ごとのユークリッド距離誤差(px)のndarrayを返す。
    """
    return np.hypot(*(aligned_est - gt_positions).T)


def compute_mean_error(errors):
    return float(np.mean(errors))


def compute_rmse(errors):
    return float(np.sqrt(np.mean(errors ** 2)))


def compute_max_error(errors):
    return float(np.max(errors))


def summarize_errors(errors, scale_px_per_m=None):
    """平均誤差・RMSE・最大誤差をまとめた辞書を返す。scale_px_per_mを与えると
    メートル換算値も付加する(kanri_4fは11.4px/m、進捗反映版メモ§4.5参照)。
    """
    summary = {
        "n_points": int(len(errors)),
        "mean_error_px": compute_mean_error(errors),
        "rmse_px": compute_rmse(errors),
        "max_error_px": compute_max_error(errors),
    }
    if scale_px_per_m:
        for key in ("mean_error", "rmse", "max_error"):
            summary[f"{key}_m"] = summary[f"{key}_px"] / scale_px_per_m
    return summary


def evaluate(estimated_csv, ground_truth_csv, scale_px_per_m=None):
    """2つのCSVパスから誤差指標を計算して辞書で返す。"""
    est_t, est_p = load_trajectory_csv(estimated_csv)
    gt_t, gt_p = load_trajectory_csv(ground_truth_csv)
    aligned_est, aligned_gt, excluded = align_by_timestamp(est_t, est_p, gt_t, gt_p)
    if len(aligned_gt) == 0:
        raise ValueError("対応付けできる正解点が0件でした(時刻範囲が重なっていません)。")
    errors = compute_position_errors(aligned_est, aligned_gt)
    summary = summarize_errors(errors, scale_px_per_m=scale_px_per_m)
    summary["excluded_points"] = excluded
    return summary


def _self_test():
    """実測正解データが無くても計算ロジックを検証できる合成データテスト。
    既知の位置に既知の標準偏差のガウスノイズを加えた「合成正解データ」を作り、
    RMSEが理論値(≈ノイズ標準偏差×sqrt(2))に近い値になるかを確認する。
    ここで出る数値は本研究の測位精度を示すものではなく、計算ロジックの
    自己検証のみを目的とする。
    """
    rng = np.random.default_rng(seed=42)

    # 合成の「真の軌跡」: 直線区間を100点で表す(推定軌跡の代役)。
    n = 100
    est_t = np.linspace(0.0, 99.0, n)
    est_p = np.column_stack([np.linspace(0, 990, n), np.full(n, 500.0)])

    # 合成の「正解データ」: 推定軌跡より粗い間隔(10歩ごと)で、既知の標準偏差の
    # ノイズを加えた点を作る。真値との誤差の理論値はノイズ標準偏差×sqrt(2)
    # (x, y独立に同じ標準偏差のノイズを加えた場合のユークリッド距離のRMSE)。
    noise_std_px = 15.0
    gt_indices = np.arange(0, n, 10)
    gt_t = est_t[gt_indices]
    gt_p = est_p[gt_indices] + rng.normal(0, noise_std_px, size=(len(gt_indices), 2))

    est_path = "/tmp/_evaluate_accuracy_selftest_est.csv"
    gt_path = "/tmp/_evaluate_accuracy_selftest_gt.csv"
    pd.DataFrame({"timestamp": est_t, "x_px": est_p[:, 0], "y_px": est_p[:, 1]}).to_csv(est_path, index=False)
    pd.DataFrame({"timestamp": gt_t, "x_px": gt_p[:, 0], "y_px": gt_p[:, 1]}).to_csv(gt_path, index=False)

    summary = evaluate(est_path, gt_path, scale_px_per_m=11.4)
    expected_rmse = noise_std_px * np.sqrt(2)

    print("=== evaluate_accuracy.py 自己テスト(合成データ) ===")
    print(f"  対応点数: {summary['n_points']}  除外点数: {summary['excluded_points']}")
    print(f"  平均誤差: {summary['mean_error_px']:.2f}px ({summary['mean_error_m']:.3f}m)")
    print(f"  RMSE    : {summary['rmse_px']:.2f}px ({summary['rmse_m']:.3f}m)  "
          f"[理論値の目安: {expected_rmse:.2f}px]")
    print(f"  最大誤差: {summary['max_error_px']:.2f}px ({summary['max_error_m']:.3f}m)")

    # 理論値から大きく外れていたら実装ミスの疑いがあるため、目安として
    # ±40%の範囲に収まっているかを確認する(合成データの点数が少ないため
    # 緩めの許容範囲にしている)。
    ratio = summary["rmse_px"] / expected_rmse
    if 0.6 <= ratio <= 1.4:
        print("  -> RMSEは理論値の目安の範囲内です。計算ロジックは正常に動作しています。")
        return True
    else:
        print("  -> RMSEが理論値の目安から外れています。実装を確認してください。")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="推定軌跡CSVと正解位置CSVから平均誤差・RMSE・最大誤差を計算する。"
    )
    parser.add_argument("--estimated", type=str, default=None,
                         help="pdr_pf_improved.py --save-trajectory-csv で保存した推定軌跡CSV。")
    parser.add_argument("--ground-truth", type=str, default=None,
                         help="正解位置CSV(timestamp, x_px, y_px[, point_type])。")
    parser.add_argument("--scale-px-per-m", type=float, default=None,
                         help="ピクセル→メートル換算の縮尺(px/m)。指定するとm単位でも表示する。")
    parser.add_argument("--self-test", action="store_true",
                         help="合成データによる自己テストのみを実行する(実データ不要)。")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

    if args.self_test:
        ok = _self_test()
        raise SystemExit(0 if ok else 1)

    if not args.estimated or not args.ground_truth:
        parser.error("--estimated と --ground-truth の両方を指定するか、--self-test を使ってください。")

    summary = evaluate(args.estimated, args.ground_truth, scale_px_per_m=args.scale_px_per_m)
    print(f"対応点数: {summary['n_points']}  除外点数: {summary['excluded_points']}")
    print(f"平均誤差: {summary['mean_error_px']:.2f}px", end="")
    if "mean_error_m" in summary:
        print(f" ({summary['mean_error_m']:.3f}m)", end="")
    print()
    print(f"RMSE    : {summary['rmse_px']:.2f}px", end="")
    if "rmse_m" in summary:
        print(f" ({summary['rmse_m']:.3f}m)", end="")
    print()
    print(f"最大誤差: {summary['max_error_px']:.2f}px", end="")
    if "max_error_m" in summary:
        print(f" ({summary['max_error_m']:.3f}m)", end="")
    print()


if __name__ == "__main__":
    main()
