# ============================================================================
# verify_route_graph.py
#
# 【変更履歴】
# - 2026-08-16: コード肥大化対策として、fake_args組み立てコード(check_sensor_
#               quality.py・pick_landmarks.pyと3重に重複しており、
#               apply_map_config()の参照属性が増えるたびに3箇所ともAttributeError
#               で落ちる不具合を繰り返していた)をpdr_pf_improved.py側の
#               load_map_config_for_tool()に一本化し、このスクリプトはそれを
#               呼ぶだけにした。
# - 2026-08-16: [本研究独自] --simplify(既定OFF)を追加。指定時はbuild_skeleton_
#               graph()の出力にsimplify_skeleton_graph()(ノード整理)を適用してから
#               描画・出力する。整理前後の交差点・端点・エッジ数をタイトルと標準出力
#               の両方に表示し、比較しやすくした。
# - 2026-08-16: [本研究独自] 新規作成。build_skeleton_graph()(通路グラフ化、
#               進捗反映版メモ§23 Week1後半)の動作確認用。分岐点候補が実際の
#               建物形状と一致しているかを可視化で確認する目的。
#
# 【このスクリプトについて】
# pdr_pf_improved.py本体(PDR/PF処理)には一切手を加えず、同ファイルの
# load_map_config()・apply_map_config()・load_preprocessed_map()・
# extract_auto_route_mask()・build_skeleton_graph()をimportして再利用し、
# route_source=autoの経路帯マスクから構築した通路グラフ(交差点ノード・
# 端点ノード・エッジ)を地図に重ねて画像として保存するだけの、読み取り専用の
# 検証ツール(check_sensor_quality.pyと同じ位置づけ)。
#
# build_skeleton_graph()自体はroute_source=autoの既定パイプラインにまだ
# 接続していない(検証専用の新関数)。このスクリプトはその「まだ繋がっていない」
# 段階の出力を単体で目視確認するためのもの。
#
# 【使い方】
#   python verify_route_graph.py --map-config map_configs/kanri_4f.json \
#       --save results/route_graph_check.png
# ============================================================================

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
import pdr_pf_improved as pdrmod  # noqa: E402

RESULTS_DIR = SCRIPT_DIR / "results"

NODE_COLORS = {"junction": "#e63946", "endpoint": "#2a9d8f"}
NODE_MARKERS = {"junction": "o", "endpoint": "s"}


def load_route_mask(map_config_path, exclude_wide_rooms_override=None):
    """pdr_pf_improved.pyのload_map_config_for_tool()を流用し、route_source=auto
    と同じ手順で経路帯マスクを構築する(check_sensor_quality.py/pick_landmarks.py
    と同じ流用パターン)。
    """
    _map_config, fake_args = pdrmod.load_map_config_for_tool(map_config_path)

    binary, binary_for_pf, _dist_map = pdrmod.load_preprocessed_map(fake_args.map)
    exclude_wide_rooms = (
        pdrmod.AUTO_ROUTE_EXCLUDE_WIDE_ROOMS
        if exclude_wide_rooms_override is None else exclude_wide_rooms_override
    )
    route_mask = pdrmod.extract_auto_route_mask(
        binary_for_pf,
        pdrmod.AUTO_ROUTE_MAX_HALF_WIDTH_PX,
        pdrmod.AUTO_ROUTE_DILATION_PX,
        exclude_wide_rooms=exclude_wide_rooms,
        exclude_wide_rooms_radius_px=pdrmod.AUTO_ROUTE_EXCLUDE_WIDE_ROOMS_RADIUS_PX,
    )
    return binary, route_mask


def draw_graph(ax, graph):
    for edge in graph["edges"]:
        pts = np.asarray(edge["points"])
        ax.plot(pts[:, 0], pts[:, 1], color="#457b9d", linewidth=1.2, zorder=3)

    for kind in ("junction", "endpoint"):
        xs = [n["x"] for n in graph["nodes"] if n["kind"] == kind]
        ys = [n["y"] for n in graph["nodes"] if n["kind"] == kind]
        if xs:
            ax.scatter(
                xs, ys,
                c=NODE_COLORS[kind], marker=NODE_MARKERS[kind],
                s=70 if kind == "junction" else 40,
                edgecolors="white", linewidths=0.8, zorder=4,
                label=f"{kind}({len(xs)})",
            )


def main():
    parser = argparse.ArgumentParser(
        description="build_skeleton_graph()の結果(交差点・端点・エッジ)を地図に重ねて確認する。"
    )
    parser.add_argument("--map-config", type=Path,
                         default=SCRIPT_DIR / "map_configs" / "kanri_4f.json")
    parser.add_argument("--save", type=Path, default=None,
                         help="保存先PNG。省略時はresults/以下へ自動命名で保存。")
    parser.add_argument("--exclude-wide-rooms", dest="exclude_wide_rooms",
                         action="store_true", default=None,
                         help="通路と広い部屋の壁際を区別する処理を強制的に有効にする"
                              "(JSON設定を上書き)。")
    parser.add_argument("--no-exclude-wide-rooms", dest="exclude_wide_rooms",
                         action="store_false",
                         help="上記を強制的に無効にする(JSON設定を上書き)。")
    parser.add_argument("--simplify", action="store_true", default=False,
                         help="simplify_skeleton_graph()によるノード整理(枝刈り・"
                              "近接交差点の統合・通過点の解消)を適用してから描画する。")
    parser.add_argument("--min-spur-length-px", type=float, default=15.0,
                         help="--simplify時: この長さ未満の端点向けエッジを枝刈りする"
                              "(既定15px)。")
    parser.add_argument("--merge-junction-distance-px", type=float, default=20.0,
                         help="--simplify時: この距離未満の交差点間エッジを1つの交差点"
                              "に統合する(既定20px)。")
    args = parser.parse_args()

    binary, route_mask = load_route_mask(args.map_config, args.exclude_wide_rooms)
    raw_graph = pdrmod.build_skeleton_graph(route_mask)

    if args.simplify:
        graph = pdrmod.simplify_skeleton_graph(
            raw_graph,
            min_spur_length_px=args.min_spur_length_px,
            merge_junction_distance_px=args.merge_junction_distance_px,
        )
    else:
        graph = raw_graph

    fig, ax = plt.subplots(figsize=(14, 6))
    ax.imshow(binary, cmap="gray")
    overlay = np.zeros((*route_mask.shape, 4))
    overlay[route_mask] = (1.0, 0.85, 0.2, 0.25)  # 経路帯マスクを薄い黄色で重ねる
    ax.imshow(overlay)
    draw_graph(ax, graph)
    ax.set_xlim(0, binary.shape[1])
    ax.set_ylim(binary.shape[0], 0)
    title = (
        f"{args.map_config.name}: 通路グラフ検証"
        f"(交差点={sum(1 for n in graph['nodes'] if n['kind'] == 'junction')}, "
        f"端点={sum(1 for n in graph['nodes'] if n['kind'] == 'endpoint')}, "
        f"エッジ={len(graph['edges'])})"
    )
    if args.simplify:
        title += (
            f"\n整理前: 交差点={sum(1 for n in raw_graph['nodes'] if n['kind'] == 'junction')}, "
            f"端点={sum(1 for n in raw_graph['nodes'] if n['kind'] == 'endpoint')}, "
            f"エッジ={len(raw_graph['edges'])}"
        )
    ax.set_title(title)
    ax.legend(loc="upper right")

    save_path = args.save
    if save_path is None:
        suffix = "_simplified" if args.simplify else ""
        save_path = RESULTS_DIR / f"{args.map_config.stem}_route_graph_check{suffix}.png"
    save_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(save_path, dpi=150, bbox_inches="tight")
    print(f"検証画像を保存しました: {save_path}")

    print(f"\nノード一覧({len(graph['nodes'])}件):")
    for n in graph["nodes"]:
        print(f"  id={n['id']:>3} kind={n['kind']:<9} x={n['x']:.1f} y={n['y']:.1f} "
              f"pixel_count={n['pixel_count']}")
    print(f"\nエッジ一覧({len(graph['edges'])}件):")
    for e in graph["edges"]:
        print(f"  {e['from']} -> {e['to']}  length_px={e['length_px']:.1f}  "
              f"points={len(e['points'])}")


if __name__ == "__main__":
    main()
