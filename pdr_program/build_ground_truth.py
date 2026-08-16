# ============================================================================
# build_ground_truth.py
#
# 【変更履歴】
# - 2026-08-16: [本研究独自] 新規作成。正解位置データ作成(進捗反映版メモ§23、
#               evaluate_accuracy.pyの実データ運用に向けた準備)のPhase 2用ツール。
#
# 【正解位置データ作成の全体像(3段階)】
# RMSE等を計算するには、実測時に「いつ・どこにいたか」という正解データが必要になる。
# しかしAndroidアプリの地点マークボタンは timestamp と連番(seq) しか記録しない
# (座標は分からない)。そこで、座標の特定と実測を切り離し、次の3段階で正解データを
# 組み立てる。
#
#   Phase 0 (pick_landmarks.py。計測前に一度だけ):
#     地図上の目印(曲がり角・ドア枠・柱など、誰が見ても同じ点だと分かる場所)を
#     順番にクリックして、ピクセル座標の一覧表(landmarks CSV: seq, label,
#     point_type, x_px, y_px)を作る。建物は動かないので、同じ経路を歩く限り
#     一度作れば使い回せる。
#
#   Phase 1 (Android端末で実際に歩きながら):
#     Phase 0で決めた目印に実際に到達するたびに、地点マークボタン(青いボタン)を
#     "その目印の順番通りに" 押す。押すと現在のCSVへ timestamp, seq が記録される
#     (MainActivity.kt の recordWaypoint()。センサー本体CSVと同じ時刻基準
#     [SystemClock.elapsedRealtimeNanos()] なので、後で時刻をそのまま突き合わせ
#     られる)。
#
#   Phase 2 (このスクリプト。計測後):
#     Phase 0のlandmarks CSVとPhase 1の_waypoints.csvをseq番号で結合し、
#     evaluate_accuracy.pyが読める形式(timestamp, x_px, y_px, point_type)の
#     正解位置CSVを作る。
#
# 【このスクリプトの使い方】
#   python build_ground_truth.py \
#       --waypoints /path/to/pdr_log_XXXX_waypoints.csv \
#       --landmarks ground_truth/kanri_4f_landmarks.csv \
#       [--output ground_truth/pdr_log_XXXX_ground_truth.csv]
#
# waypoints.csvとlandmarks.csvはseq番号(1始まりの連番)で対応付ける。件数や
# seqの値が食い違っている場合は、Android側での押し忘れ・押し過ぎ、または
# landmarks.csv側の目印数の不一致が疑われるため、エラーで止めて内容を確認できる
# ようにしている(黙って一部だけ結合して不正確な正解データを作らない)。
# ============================================================================

import argparse
from pathlib import Path

import pandas as pd


def load_waypoints(path):
    df = pd.read_csv(path)
    missing = [c for c in ("timestamp", "seq") if c not in df.columns]
    if missing:
        raise ValueError(f"{path}: 必須列が不足しています: {missing}")
    return df


def load_landmarks(path):
    df = pd.read_csv(path)
    missing = [c for c in ("seq", "label", "point_type", "x_px", "y_px") if c not in df.columns]
    if missing:
        raise ValueError(f"{path}: 必須列が不足しています: {missing}")
    return df


def build_ground_truth(waypoints_df, landmarks_df):
    """waypoints(timestamp, seq)とlandmarks(seq, label, point_type, x_px, y_px)を
    seqで結合し、evaluate_accuracy.pyが読める(timestamp, x_px, y_px, point_type,
    label)形式のDataFrameを返す。seqの集合が完全に一致しない場合はValueErrorとし、
    どのseqが食い違っているかを具体的に示す。
    """
    wp_seqs = set(waypoints_df["seq"])
    lm_seqs = set(landmarks_df["seq"])

    if wp_seqs != lm_seqs:
        only_wp = sorted(wp_seqs - lm_seqs)
        only_lm = sorted(lm_seqs - wp_seqs)
        detail = []
        if only_wp:
            detail.append(f"waypoints側にのみ存在するseq(landmarksに未登録、押し過ぎ?): {only_wp}")
        if only_lm:
            detail.append(f"landmarks側にのみ存在するseq(未計測、押し忘れ?): {only_lm}")
        raise ValueError(
            "waypointsとlandmarksのseqが一致しません。\n  " + "\n  ".join(detail)
        )

    merged = waypoints_df.merge(landmarks_df, on="seq", how="inner")
    merged = merged.sort_values("seq").reset_index(drop=True)
    return merged[["timestamp", "x_px", "y_px", "point_type", "label", "seq"]]


def main():
    parser = argparse.ArgumentParser(
        description=(
            "_waypoints.csv(timestamp, seq)とlandmarks CSV(seq, label, point_type,"
            "x_px, y_px)をseqで結合し、evaluate_accuracy.py用の正解位置CSVを作る。"
            "詳細はこのファイル冒頭のコメントを参照。"
        )
    )
    parser.add_argument("--waypoints", type=Path, required=True,
                         help="Android端末で記録された *_waypoints.csv (timestamp, seq)。")
    parser.add_argument("--landmarks", type=Path, required=True,
                         help="pick_landmarks.pyで作った目印座標表(seq, label, point_type, x_px, y_px)。")
    parser.add_argument("--output", type=Path, default=None,
                         help="出力先。省略時は<waypoints名>_ground_truth.csvを同じ場所に保存。")
    args = parser.parse_args()

    waypoints_df = load_waypoints(args.waypoints)
    landmarks_df = load_landmarks(args.landmarks)
    ground_truth_df = build_ground_truth(waypoints_df, landmarks_df)

    output_path = args.output
    if output_path is None:
        stem = args.waypoints.stem.replace("_waypoints", "")
        output_path = args.waypoints.parent / f"{stem}_ground_truth.csv"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    ground_truth_df.to_csv(output_path, index=False)

    print(f"正解位置CSVを保存しました: {output_path} ({len(ground_truth_df)}点)")
    print(ground_truth_df.to_string(index=False))


if __name__ == "__main__":
    main()
