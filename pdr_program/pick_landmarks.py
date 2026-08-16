# ============================================================================
# pick_landmarks.py
#
# 【変更履歴】
# - 2026-08-16: コード肥大化対策として、fake_args組み立てコード(check_sensor_
#               quality.py・verify_route_graph.pyと3重に重複しており、
#               apply_map_config()の参照属性が増えるたびに3箇所ともAttributeError
#               で落ちる不具合を繰り返していた)をpdr_pf_improved.py側の
#               load_map_config_for_tool()に一本化し、このスクリプトはそれを
#               呼ぶだけにした。
# - 2026-08-16: [本研究独自] 新規作成。正解位置データ作成(進捗反映版メモ§23、
#               evaluate_accuracy.pyの実データ運用に向けた準備)のPhase 0用ツール。
#
# 【正解位置データ作成の全体像(3段階)】
# RMSE等を計算するには、実測時に「いつ・どこにいたか」という正解データが必要になる。
# しかしAndroidアプリの地点マークボタンは timestamp と連番(seq) しか記録しない
# (座標は分からない)。そこで、座標の特定と実測を切り離し、次の3段階で正解データを
# 組み立てる。
#
#   Phase 0 (このスクリプト。計測前に一度だけ):
#     地図上の目印(曲がり角・ドア枠・柱など、誰が見ても同じ点だと分かる場所)を
#     順番にクリックして、ピクセル座標の一覧表(landmarks CSV)を作る。
#     建物は動かないので、同じ経路を歩く限り一度作れば使い回せる。
#
#   Phase 1 (Android端末で実際に歩きながら):
#     Phase 0で決めた目印に実際に到達するたびに、地点マークボタン(青いボタン)を
#     "その目印の順番通りに" 押す。押すと現在のCSVへ timestamp, seq が記録される
#     (MainActivity.kt の recordWaypoint()。センサー本体CSVと同じ時刻基準
#     [SystemClock.elapsedRealtimeNanos()] なので、後で時刻をそのまま突き合わせ
#     られる)。
#
#   Phase 2 (build_ground_truth.py。計測後):
#     Phase 0のlandmarks CSVとPhase 1の_waypoints.csvをseq番号で結合し、
#     evaluate_accuracy.pyが読める形式(timestamp, x_px, y_px, point_type)の
#     正解位置CSVを作る。
#
# 【このスクリプトの使い方】
#   python pick_landmarks.py --map-config map_configs/kanri_4f.json \
#       --landmarks-file ground_truth/kanri_4f_landmark_labels.csv
#
# landmarks-file(事前に用意する、目印のラベル一覧)の形式:
#   label,point_type
#   start,start
#   turn1,turn
#   turn2,turn
#   end,end
# ラベルを書いた順番がそのままクリックの順番=seq番号(1始まり)になる。
# Android側で地点マークボタンを押す順番も、必ずこのラベル順と一致させること。
#
# 表示される地図は pdr_pf_improved.py が実際にPFで使っているのと同じ二値地図
# (load_preprocessed_map()の戻り値そのまま)なので、クリックして得られる座標は
# 推定軌跡CSV(x_px, y_px)と直接比較できる。
# ============================================================================

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
import pdr_pf_improved as pdrmod  # noqa: E402

DEFAULT_OUTPUT_DIR = SCRIPT_DIR / "ground_truth"


def load_map_binary(map_config_path):
    """pdr_pf_improved.pyのload_map_config_for_tool()をそのまま流用し、PFが
    実際に使う地図(binary)を取得する(check_sensor_quality.pyと同じ流用パターン)。
    """
    _map_config, fake_args = pdrmod.load_map_config_for_tool(map_config_path)
    binary, _binary_for_pf, _dist_map = pdrmod.load_preprocessed_map(fake_args.map)
    return binary


def load_landmark_labels(landmarks_file):
    """label, point_type の2列を持つCSVを読み込み、(label, point_type)の
    リスト(記載順=クリック順)を返す。
    """
    df = pd.read_csv(landmarks_file)
    missing = [c for c in ("label", "point_type") if c not in df.columns]
    if missing:
        raise ValueError(f"{landmarks_file}: 必須列が不足しています: {missing}")
    if df.empty:
        raise ValueError(f"{landmarks_file}: 目印が1件もありません。")
    return list(zip(df["label"].astype(str), df["point_type"].astype(str)))


def pick_one_point(ax, fig, label, point_type, seq, total):
    """地図上でクリックされた1点の座標を返す(plt.ginputを1回だけ使う、
    select_start_position_on_map()と同じ操作感)。"""
    ax.set_title(
        f"[{seq}/{total}] '{label}' ({point_type}) をクリックしてください\n"
        "(クリック後、自動的に次の目印に進みます)"
    )
    fig.canvas.draw()
    clicked = plt.ginput(n=1, timeout=0, show_clicks=True)
    if not clicked:
        raise RuntimeError(f"'{label}' の座標が選択されませんでした(ウィンドウを閉じた?)。")
    x, y = clicked[0]
    return float(x), float(y)


def main():
    parser = argparse.ArgumentParser(
        description=(
            "地図上の目印を順番にクリックし、ピクセル座標の一覧表(landmarks CSV)を作る。"
            "詳細はこのファイル冒頭のコメントを参照。"
        )
    )
    parser.add_argument("--map-config", type=Path,
                         default=SCRIPT_DIR / "map_configs" / "kanri_4f.json",
                         help="地図設定JSON。表示する地図とピクセル座標系をこれで決める。")
    parser.add_argument("--landmarks-file", type=Path, required=True,
                         help="目印のラベル一覧CSV(label, point_type の2列。記載順=クリック順)。")
    parser.add_argument("--output", type=Path, default=None,
                         help="出力CSVのパス。省略時は ground_truth/<map-config名>_landmarks.csv")
    args = parser.parse_args()

    labels = load_landmark_labels(args.landmarks_file)
    binary = load_map_binary(args.map_config)

    fig, ax = plt.subplots(figsize=(12, 6))
    ax.imshow(binary, cmap="gray")
    ax.set_xlim(0, binary.shape[1])
    ax.set_ylim(binary.shape[0], 0)
    ax.grid(False)
    plt.show(block=False)

    rows = []
    total = len(labels)
    for seq, (label, point_type) in enumerate(labels, start=1):
        x, y = pick_one_point(ax, fig, label, point_type, seq, total)
        rows.append({"seq": seq, "label": label, "point_type": point_type, "x_px": x, "y_px": y})
        print(f"  [{seq}/{total}] '{label}' -> x={x:.1f}, y={y:.1f}")

    plt.close(fig)

    output_path = args.output
    if output_path is None:
        output_path = DEFAULT_OUTPUT_DIR / f"{args.map_config.stem}_landmarks.csv"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    pd.DataFrame(rows).to_csv(output_path, index=False)
    print(f"\n目印座標表を保存しました: {output_path}")
    print("Android側で地点マークボタンを押す順番も、このラベル順と必ず一致させてください。")


if __name__ == "__main__":
    main()
