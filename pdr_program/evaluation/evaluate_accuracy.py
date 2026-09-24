# ============================================================================
# evaluate_accuracy.py
#
# 【変更履歴】
# - 2026-09-24: [本研究独自] 最後の歩から5秒以内に押した正解点(終点で立ち止まってから
#               押す目印)を、最後の推定位置に止まっているとみなして評価に含めるようにした
#               (END_HOLD_SEC、--end-hold-sec)。これまでは時刻範囲外として外れていた。
# - 2026-09-18: --self-test の一時ファイルを/tmp固定からtempfileへ変更(Windowsで失敗していた)。
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
# 誤差が過大・過小評価されるため)。推定軌跡の時刻範囲の外の正解点は外すが、
# 最後の歩からEND_HOLD_SEC秒以内の点(終点で止まってから押した目印)は、最後の
# 推定位置と比べて評価に含める。
#
# 【正解位置CSVの形式】(2026年10月の実測時にこの形式でメモを取る)
#   timestamp, x_px, y_px[, point_type]
#   ・timestamp: 推定軌跡CSVと同じ基準の時刻(pdr_log_*.csvのtimestamp列と同じ単位)。
#   ・x_px, y_px: 地図画像上のピクセル座標。
#   ・point_type: 任意。start/turn/end等のラベル(曲がり位置誤差の計算に使う予定、
#     現時点では未使用)。
#
# 【使い方】
#   python evaluation/evaluate_accuracy.py --self-test
#     -> 合成データで計算ロジックが正しいかだけを確認する(実データ不要)。
#
#   python evaluation/evaluate_accuracy.py \
#     --estimated results/pdr_log_XXXX_trajectory.csv \
#     --ground-truth path/to/ground_truth.csv \
#     [--scale-px-per-m 11.4]
#     -> 実データでの評価(正解データが揃う2026年10月以降に使用)。
# ============================================================================

import argparse
import logging
import tempfile
from pathlib import Path

import numpy as np
import pandas as pd

REQUIRED_COLUMNS = ["timestamp", "x_px", "y_px"]

# [本研究独自] 最後の歩の後、この秒数までに押された正解点は、最後の推定位置に止まって
# いるとみなして評価に含める(2026-09-24、ユーザー承認)。推定軌跡は最後の歩の時刻で
# 終わるが、終点の目印は立ち止まってからボタンを押すので、その時刻は必ず最後の歩より
# 後になる。範囲外として外すと、誤差が最も大きくなりやすい終点が毎回評価から抜け、
# RMSEが小さめに出る。立ち止まった後は動かないので、最後の推定位置を使うのは外挿では
# ない。これより遅く押した点は、歩数の取りこぼしなど別の事情がありうるので従来どおり外す。
# スタート(地点マーク1番)は歩き始める前に押すので、今までどおり範囲外として外れる
# (開始位置は既知なので、含めると全方式で誤差0の点が増えるだけになる)。
END_HOLD_SEC = 5.0


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


def align_by_timestamp(est_timestamps, est_positions, gt_timestamps, gt_positions,
                       end_hold_sec=END_HOLD_SEC):
    """各正解タイムスタンプ(gt_timestamps)に対応する推定位置を、推定軌跡
    (est_timestamps, est_positions)の線形補間で求める。

    正解データは推定軌跡よりまばらな時刻で記録される想定のため、正解の
    タイムスタンプ範囲が推定軌跡の範囲からはみ出す場合はその正解点を除外する
    (外挿による誤差の過大評価を避けるため)。
    ただし最後の歩からend_hold_sec秒以内の正解点は、最後の推定位置に止まっている
    とみなして評価に含める(END_HOLD_SECのコメント参照)。

    戻り値: (対応する推定位置Nx2, 使用した正解位置Nx2, 除外件数,
             最後の推定位置で止まっているとみなした件数)
    """
    if len(est_timestamps) < 2:
        raise ValueError("推定軌跡の点数が少なすぎます(2点以上必要)。")

    hold = max(0.0, float(end_hold_sec or 0.0))
    t_min, t_max = est_timestamps[0], est_timestamps[-1]
    in_range = (gt_timestamps >= t_min) & (gt_timestamps <= t_max + hold)
    held = int(np.sum(in_range & (gt_timestamps > t_max)))
    excluded = int(np.sum(~in_range))
    if excluded > 0:
        logging.warning(
            "正解点%d件が推定軌跡の時刻範囲外(%.3f〜%.3f、最後の歩の後%.1f秒までは含める)"
            "のため評価から除外されました。",
            excluded, t_min, t_max, hold,
        )

    gt_t = gt_timestamps[in_range]
    gt_p = gt_positions[in_range]
    # np.interpは範囲の右外では最後の値を返すので、最後の歩より後の正解点は
    # 「最後の推定位置に止まっている」ものとして比べることになる。
    est_x = np.interp(gt_t, est_timestamps, est_positions[:, 0])
    est_y = np.interp(gt_t, est_timestamps, est_positions[:, 1])
    aligned_est = np.column_stack([est_x, est_y])
    return aligned_est, gt_p, excluded, held


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


def evaluate(estimated_csv, ground_truth_csv, scale_px_per_m=None, end_hold_sec=END_HOLD_SEC):
    """2つのCSVパスから誤差指標を計算して辞書で返す。"""
    est_t, est_p = load_trajectory_csv(estimated_csv)
    gt_t, gt_p = load_trajectory_csv(ground_truth_csv)
    aligned_est, aligned_gt, excluded, held = align_by_timestamp(
        est_t, est_p, gt_t, gt_p, end_hold_sec=end_hold_sec)
    if len(aligned_gt) == 0:
        raise ValueError("対応付けできる正解点が0件でした(時刻範囲が重なっていません)。")
    errors = compute_position_errors(aligned_est, aligned_gt)
    summary = summarize_errors(errors, scale_px_per_m=scale_px_per_m)
    summary["excluded_points"] = excluded
    summary["held_end_points"] = held
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

    # /tmp固定だとWindowsで失敗するため、OSに合った一時フォルダを使う(2026-09-18)。
    with tempfile.TemporaryDirectory() as td:
        est_path = Path(td) / "est.csv"
        gt_path = Path(td) / "gt.csv"
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
    ok = 0.6 <= ratio <= 1.4
    if ok:
        print("  -> RMSEは理論値の目安の範囲内です。計算ロジックは正常に動作しています。")
    else:
        print("  -> RMSEが理論値の目安から外れています。実装を確認してください。")

    # 最後の歩の後に押した点の扱い(2026-09-24)。推定軌跡は t=0〜99、x=10t。
    # 歩き始める前(t=-1)と、最後の歩から11秒後(t=110)は外れ、途中(t=50)と
    # 最後の歩の2秒後(t=101)は入る。t=101 は最後の推定位置(990, 500)と比べる。
    gt_t2 = np.array([-1.0, 50.0, 101.0, 110.0])
    gt_p2 = np.array([[0.0, 500.0], [500.0, 500.0], [990.0, 500.0], [990.0, 500.0]])
    logging.disable(logging.WARNING)  # 外れる点の警告はここでは想定どおり
    try:
        aligned, used, excluded, held = align_by_timestamp(est_t, est_p, gt_t2, gt_p2)
    finally:
        logging.disable(logging.NOTSET)
    hold_ok = (excluded == 2 and held == 1 and len(used) == 2
               and np.allclose(aligned, [[500.0, 500.0], [990.0, 500.0]]))
    print(f"  最後の歩の後の点: 使用{len(used)}点(うち止まっているとみなした点{held}) / 除外{excluded}点"
          f"(最後の歩から{END_HOLD_SEC:g}秒まで含める)")
    if hold_ok:
        print("  -> 終点で止まってから押した点を含め、遅すぎる点と歩き始める前の点は外した。")
    else:
        print("  -> 最後の歩の後の点の扱いが想定と違います。実装を確認してください。")
    return ok and hold_ok


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
    parser.add_argument("--end-hold-sec", type=float, default=END_HOLD_SEC,
                         help="最後の歩の後、この秒数までに押した正解点は最後の推定位置に"
                              f"止まっているとみなして評価に含める(既定{END_HOLD_SEC:g}秒、0で含めない)。")
    parser.add_argument("--self-test", action="store_true",
                         help="合成データによる自己テストのみを実行する(実データ不要)。")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

    if args.self_test:
        ok = _self_test()
        raise SystemExit(0 if ok else 1)

    if not args.estimated or not args.ground_truth:
        parser.error("--estimated と --ground-truth の両方を指定するか、--self-test を使ってください。")

    summary = evaluate(args.estimated, args.ground_truth, scale_px_per_m=args.scale_px_per_m,
                       end_hold_sec=args.end_hold_sec)
    print(f"対応点数: {summary['n_points']}  除外点数: {summary['excluded_points']}  "
          f"(うち最後の推定位置で止まっているとみなした点: {summary['held_end_points']})")
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
