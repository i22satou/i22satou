# ============================================================================
# pdr_route_graph.py
#
# 【変更履歴】
# - 2026-08-30: [本研究独自] 複数経路仮説PFの分岐選択を、完全一様乱択から
#               直前の実測方位に近いエッジを優先するガウス重み付き乱択へ
#               変更(edge_entry_heading/choose_branch_by_heading追加)。
#               pdr_pf_improved.py側のコード増大を避けるため、方位計算を
#               伴う純粋関数はこちらに置いた(呼び出し側の_advance_route_state
#               は関数呼び出し1つに置き換わるだけで数行増に留まる)。
# - 2026-08-16: [本研究独自] 新規作成。pdr_pf_improved.pyが3500行を超えて
#               肥大化してきたため、経路帯マスク抽出・通路グラフ化(骨格化・
#               ノード整理・トポロジー変換)関連の関数をこのファイルへ切り出した。
#               関数の中身・ロジックは一切変更していない(コピーのみ)。
#               pdr_pf_improved.py側は
#               `from pdr_route_graph import (...)` で全関数を再importして
#               いるため、pdr_pf_improved.py内の呼び出し箇所・
#               他スクリプト(verify_route_graph.py等)からの
#               `pdrmod.build_skeleton_graph(...)`のような呼び出し方は
#               一切変更する必要がない。
#
#               分離にあたり、extract_ordered_centerline()内で参照していた
#               pdr_pf_improved.py側のグローバル定数
#               AUTO_ROUTE_CENTERLINE_MAX_DETOUR_RATIO(実行中に書き換わらない
#               純粋な定数)は、この関数の引数max_detour_ratio(既定値2.5、
#               従来の定数値と同じ)に変換した。呼び出し側は追加引数を渡して
#               いないため既定値2.5が使われ、動作は従来と完全に同じ。
#
#               それ以外の関数(extract_auto_route_mask, _rdp_simplify,
#               build_skeleton_graph, _prune_graph_spurs,
#               _merge_close_junction_nodes, _collapse_pass_through_nodes,
#               simplify_skeleton_graph, build_route_graph_topology,
#               nearest_edge_position)はいずれも呼び出し元からの引数だけで
#               完結しており(pdr_pf_improved.py側の可変なグローバル設定値
#               ROUTE_HEADING_WEIGHT等には一切依存しない)、そのまま移動できた。
#               なお、手動route_pointsから経路帯マスクを作るbuild_route_mask()は
#               ROUTE_POINTS・ROUTE_WIDTH_PX(可変なグローバル設定値)に依存する
#               ため、このファイルには移動せずpdr_pf_improved.py側に残している。
#
# 【このファイルの位置づけ】
# route_source=auto(二値地図から通路帯マスクを自動抽出する方式)に関する
# 一連のジオメトリ処理をまとめたモジュール。実行時に変化する設定値
# (ROUTE_HEADING_WEIGHTなど)を一切参照せず、すべて引数で完結する純粋関数の
# 集まりなので、単体でimportして地図データだけを渡してテストできる
# (verify_route_graph.pyが実際にそうしている)。
#
# 処理の流れ:
#   extract_auto_route_mask()  : 二値地図 -> 経路帯マスク(壁までの距離で判定)
#   extract_ordered_centerline(): 経路帯マスク -> 1本の順序付き中心線
#                                  (--auto-route-centerline用、分岐は無視する簡易版)
#   build_skeleton_graph()     : 経路帯マスク -> 分岐・端点を保持した通路グラフ
#   simplify_skeleton_graph()  : 通路グラフの整理(枝刈り・交差点統合・通過点解消)
#   build_route_graph_topology(): 整理済みグラフ -> 複数経路仮説PFが使うトポロジー
#     (区間ごとの方位・長さ、ノードごとの接続エッジ一覧)
#   nearest_edge_position()    : 座標 -> 最も近いエッジ・区間(初期化用)
# ============================================================================

import collections
import logging

import numpy as np
from scipy import ndimage
# [本研究独自] route_source=autoの通路帯マスクを細線化して中心線・通路グラフを
# 得るために使用。
from skimage.morphology import skeletonize


# [本研究独自] 二値地図から通路領域を自動抽出し、経路帯マスクを作る(route_source=auto)。
# build_route_mask()が手動route_pointsを太らせるのに対し、こちらは座標を一切与えず
# 「壁までの距離がmax_half_width_px以下の移動可能領域」を通路候補とみなし、そのうち
# 最大の連結成分を通路網として採用する(広い部屋は距離が大きく候補から外れるため、
# 廊下だけが概ね残る)。曲がり角に連動した方位補正(route_guidance_enabled系)は
# 順序付きroute_pointsが前提のため、autoではまだ対応しない(空間的な経路帯制約のみ)。
#
# [本研究独自] ただし上記の「壁までの距離」だけの判定では、大きな部屋の壁際の帯
# (片側は壁、反対側は広い部屋の内部)を、両側を壁に挟まれた本物の通路と区別できない
# (2026-08-16、kanri_4fの中心線抽出で発見。詳細はmemo/route_source_auto.md)。
# exclude_wide_rooms=Trueを指定すると、半径exclude_wide_rooms_radius_pxの円盤でfreeを
# モルフォロジー開放(収縮→膨張)した結果を通路候補から差し引く。円盤が完全に
# 収まるくらい広い領域(部屋の内部)は開放でほぼ元の形が復元されるため除外され、
# 円盤の直径より狭い領域は収縮で消えて開放結果に含まれず、除外されずに残る。
# 既定はFalse(既存のroute_source=auto比較実験の結果を変えないため)。
#
# [本研究独自] 除外半径はmax_half_width_pxとは別のパラメータにしている。kanri_4fで
# 単体検証したところ、半径=max_half_width_px(20px)では西側廊下・東側廊下を繋ぐ
# ホール自体も「広い部屋」として除外されてしまい、通路網が西西/東に分断されて
# 小さい方(西側、実測データの開始位置を含む)が最大連結成分から漏れ、
# route_constraint_mode=enforceでPFが全滅する事故が起きた(2026-08-16、単体検証で
# 発見)。半径を30px程度まで上げるとホールは開放されず(生き残り)、大きな部屋
# (電子工学実験室等)は開放される、というちょうど良い境界が見つかったため、
# 既定値は「max_half_width_pxそのものではなく、それより広めの別定数」とした。
def extract_auto_route_mask(
    binary_for_pf_local, max_half_width_px, dilation_px=0.0,
    exclude_wide_rooms=False, exclude_wide_rooms_radius_px=None,
):
    free = binary_for_pf_local == 255
    dist = ndimage.distance_transform_edt(free)
    corridor_candidate = free & (dist <= max_half_width_px)
    if exclude_wide_rooms:
        radius_px = (
            exclude_wide_rooms_radius_px if exclude_wide_rooms_radius_px is not None
            else max(max_half_width_px * 1.5, max_half_width_px + 10.0)
        )
        radius = max(1, int(round(radius_px)))
        yy, xx = np.ogrid[-radius:radius + 1, -radius:radius + 1]
        disk = xx * xx + yy * yy <= radius * radius
        wide_rooms = ndimage.binary_opening(free, structure=disk)
        excluded = corridor_candidate & wide_rooms
        corridor_candidate = corridor_candidate & ~wide_rooms
        logging.info(
            f"route_source=auto 広い部屋の壁際を除外: {int(excluded.sum())}px "
            f"(開放半径={radius_px:.1f}px)"
        )
    labeled, n = ndimage.label(corridor_candidate, structure=np.ones((3, 3)))
    if n == 0:
        logging.warning("route_source=autoで通路領域が抽出できませんでした。全域を通路として扱います。")
        return np.ones(free.shape, dtype=bool)
    sizes = ndimage.sum(corridor_candidate, labeled, range(1, n + 1))
    largest_label = int(np.argmax(sizes)) + 1
    mask = labeled == largest_label
    # [本研究独自] 抽出した通路領域は実際の建物形状に忠実な幅を持つため、狭い区間では
    # 手動route_pointsの一様バッファ(半径18px)より制約が厳しくなりPFが不安定化する
    # ことが確認された(CLAUDE_MEMO.md参照)。dilation_px>0を指定すると、抽出した形状は
    # 保ったまま境界へ数px分の余裕(膨張)を追加できる。地図の実形状からの乖離を最小限に
    # 抑えつつ、粒子ノイズへの緩衝を持たせるための後処理。
    if dilation_px > 0:
        radius = max(1, int(round(dilation_px)))
        yy, xx = np.ogrid[-radius:radius + 1, -radius:radius + 1]
        disk = xx * xx + yy * yy <= radius * radius
        mask = ndimage.binary_dilation(mask, structure=disk)
        mask &= free
    return mask


# [本研究独自] extract_auto_route_mask()が作る領域マスクを細線化(skeletonize)して
# 1本の順序付き中心線を抽出し、既存の曲がり角連動方位補正(route_guidance_enabled系、
# もともとroute_points前提)をroute_source=autoでも動かせるようにする。マスクの空間
# 制約自体(PFの重み付け)はextract_auto_route_mask()のまま変更せず、この関数の結果は
# ROUTE_POINTS(方位補正・曲がり角判定用)にのみ使う。
# 現在の実測データ(0805系)の通路は分岐のない単純な形状であることを確認済み
# (memo/route_source_auto.md)なので、骨格化後にできる小さな分岐・突起はノイズと
# みなし、木の直径(最も長い経路)を1本だけ採用する簡易版とした。分岐を含む通路網
# (§6.2の完全版)への対応は今後の課題。
def extract_ordered_centerline(mask, simplify_tolerance_px, max_detour_ratio=2.5):
    """経路帯マスクから順序付き中心線をROUTE_POINTS形式((x, y)タプルのリスト)で返す。
    抽出できない場合は空リストを返す(呼び出し側はROUTE_POINTS=[]のまま、つまり
    方位補正が無効な従来挙動にフォールバックする)。

    max_detour_ratioは下記の「輪」検出の閾値(既定2.5、元はpdr_pf_improved.pyの
    定数AUTO_ROUTE_CENTERLINE_MAX_DETOUR_RATIOだったものをこの関数の引数に変換した。
    呼び出し側で調整が必要になったことはなく、CLI/JSONには公開していない)。
    """
    skeleton = skeletonize(mask)
    ys, xs = np.nonzero(skeleton)
    if len(ys) < 2:
        return []

    pixel_set = set(zip(ys.tolist(), xs.tolist()))
    neighbor_offsets = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

    def neighbors(p):
        y, x = p
        return [q for dy, dx in neighbor_offsets if (q := (y + dy, x + dx)) in pixel_set]

    def bfs_farthest(start):
        """startから幅優先探索し、最も遠い(最終的に訪問した)画素と、経路復元用の
        親ポインタ辞書を返す。木でない(小さな輪)場合も、幅優先で得られる木構造上で
        近似的に最遠点を求めるヒューリスティックとして扱う。"""
        visited = {start: None}
        queue = collections.deque([start])
        farthest = start
        while queue:
            current = queue.popleft()
            farthest = current
            for nxt in neighbors(current):
                if nxt not in visited:
                    visited[nxt] = current
                    queue.append(nxt)
        return farthest, visited

    # 木の直径を求める定番の2回BFS(始点は骨格の任意の画素でよい)。
    any_pixel = next(iter(pixel_set))
    one_end, _ = bfs_farthest(any_pixel)
    other_end, visited = bfs_farthest(one_end)

    if len(visited) < len(pixel_set) * 0.9:
        logging.warning(
            "route_source=auto中心線抽出: 骨格が複数の連結成分に分かれている可能性があります"
            f"(到達画素={len(visited)}/{len(pixel_set)})。最も長い経路のみを採用します。"
        )

    path = []
    node = other_end
    while node is not None:
        path.append(node)
        node = visited[node]
    path.reverse()  # one_end -> other_end の順

    points_xy = [(float(x), float(y)) for y, x in path]

    # [本研究独自] マスクが「輪」状に連結して見える場合への安全装置。
    # extract_auto_route_mask()の最大連結成分には、kanri_4f.jpgのように、2値化した
    # 地図上では2本の帯が両端付近で連結して見える場合がある(2026-08-16、単体検証で
    # 発見)。実際の建物平面図・壁検出結果(kanri_4f_preview_final3.png)と1px単位で
    # 照合したところ、原因は2つの組み合わせだった: (1)高さの異なる西側廊下・東側廊下
    # を繋ぐホール・階段(3Fへの階段)は実在する正しい接続、(2)もう一方は
    # extract_auto_route_mask()が壁までの距離のみで通路候補を判定するため、大きな
    # 部屋(電子工学実験室等)の壁際の帯を通路と区別できず、たまたま別の階段(屋外階段)
    # の踊り場まで繋がって見えるだけの見せかけの経路だった。文字(部屋番号等)を壁と
    # 誤認したことが原因ではない(壁検出結果を平面図と照合して正確だったことを確認
    # 済み)。詳細はmemo/route_source_auto.md参照。理由の組み合わせによらず、
    # このように連結して見える場合は木の直径探索が誤って輪をぐるっと回る経路を
    # 「最も長い経路」として選んでしまうため、実際の経路長が始点・終点間の直線距離に
    # 対して極端に長い(=大きく迂回している)場合はこの誤検出とみなし、抽出を諦めて
    # 空リストを返す(呼び出し側は従来通りROUTE_POINTS=[]のまま、方位補正なしに
    # フォールバックする)。通路と部屋の壁際を区別できるマスク生成方式への改良や、
    # 分岐を含む通路網への正式対応(§6.2の通路グラフ)は今後の課題。
    path_xy = np.asarray(points_xy, dtype=float)
    path_length = float(np.sum(np.hypot(*np.diff(path_xy, axis=0).T)))
    straight_dist = float(np.hypot(*(path_xy[-1] - path_xy[0])))
    detour_ratio = path_length / straight_dist if straight_dist > 1e-6 else float("inf")
    if detour_ratio > max_detour_ratio:
        logging.warning(
            "route_source=auto中心線抽出: 始点・終点間の迂回率が"
            f"{detour_ratio:.1f}倍(経路長={path_length:.0f}px, 直線距離={straight_dist:.0f}px)と"
            f"閾値({max_detour_ratio:.1f}倍)を超えており、"
            "通路網がループ(輪)状になっている可能性があります。誤った経路を採用しない"
            "ため中心線抽出を見送ります。"
        )
        return []

    return _rdp_simplify(points_xy, simplify_tolerance_px)


def _rdp_simplify(points, epsilon):
    """Ramer-Douglas-Peucker法による折れ線の簡略化。細線化直後のジグザグな画素列
    (points、(x, y)のリスト)から、epsilon(px)より線分から離れた点だけを残すことで、
    ROUTE_POINTSとして使える少数の直線区間(曲がり角)へ変換する。再帰ではなく
    明示的なスタックで実装し、画素数が多い経路でも再帰上限に触れないようにする。
    """
    if len(points) < 3 or epsilon <= 0:
        return points

    pts = np.asarray(points, dtype=float)
    keep = np.zeros(len(points), dtype=bool)
    keep[0] = True
    keep[-1] = True
    stack = [(0, len(points) - 1)]

    while stack:
        start_i, end_i = stack.pop()
        if end_i - start_i < 2:
            continue
        start, end = pts[start_i], pts[end_i]
        dx, dy = end[0] - start[0], end[1] - start[1]
        seg_len = float(np.hypot(dx, dy))
        inner_x = pts[start_i + 1:end_i, 0]
        inner_y = pts[start_i + 1:end_i, 1]
        if seg_len < 1e-9:
            dists = np.hypot(inner_x - start[0], inner_y - start[1])
        else:
            cross = np.abs(dy * (inner_x - start[0]) - dx * (inner_y - start[1]))
            dists = cross / seg_len
        max_local = int(np.argmax(dists))
        max_dist = float(dists[max_local])
        split_i = start_i + 1 + max_local
        if max_dist > epsilon:
            keep[split_i] = True
            stack.append((start_i, split_i))
            stack.append((split_i, end_i))

    return [tuple(p) for p, k in zip(points, keep) if k]


# [本研究独自] 通路グラフ化(進捗反映版メモ§23 Week1後半、5.3節・6.2節の完全版に
# 向けた第一段階)。extract_ordered_centerline()が骨格全体から「最も長い1本の経路
# (木の直径)」だけを選んで分岐を無視するのに対し、こちらは骨格の分岐点・端点を
# すべてノードとして保持し、それらを結ぶ経路をエッジとするグラフを構築する。
# 8近傍の骨格画素隣接構造(骨格画素の次数で分類する考え方)自体は
# extract_ordered_centerline()と同じだが、目的が異なるため別関数として独立させて
# おり、あちらの安全装置(迂回率チェック)やroute_source=autoの既定パイプラインには
# まだ接続していない(既定の挙動は一切変更しない、検証専用の新関数)。
# 将来、複数経路仮説(5.7節、進捗反映版メモ§23の次の優先タスク)で
# 交差点(junctionノード)を検出する土台として使う想定。
def build_skeleton_graph(mask):
    """二値マスクを細線化(skeletonize)し、骨格画素を次数で分類してノード・エッジ
    のグラフを構築する。

    ノードの種類:
      - "endpoint": 次数1の画素(通路の行き止まり・端)。
      - "junction": 次数3以上の画素(交差点・分岐点)。隣接する分岐画素同士は
        1つの交差点としてクラスタ化し、クラスタの重心を座標とする
        (骨格化した交差点は数画素の塊になりやすいため)。
    次数2の画素(通路の途中)はノード化せず、エッジの経由点として保持する。
    次数0の孤立画素(ノイズの可能性)は無視する。

    戻り値: {"nodes": [...], "edges": [...]}
      nodes: [{"id", "x", "y", "kind"("endpoint"|"junction"), "pixel_count"}, ...]
      edges: [{"from", "to"(ノードid), "points"([(x,y),...]、from→toの順)、
               "length_px"}, ...]
    骨格画素が2点未満の場合は{"nodes": [], "edges": []}を返す。
    """
    skeleton = skeletonize(mask)
    ys, xs = np.nonzero(skeleton)
    if len(ys) < 2:
        return {"nodes": [], "edges": []}

    pixel_set = set(zip(ys.tolist(), xs.tolist()))
    neighbor_offsets = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

    def neighbors(p):
        y, x = p
        return [q for dy, dx in neighbor_offsets if (q := (y + dy, x + dx)) in pixel_set]

    degree = {p: len(neighbors(p)) for p in pixel_set}
    isolated_count = sum(1 for d in degree.values() if d == 0)
    if isolated_count:
        logging.debug(
            "build_skeleton_graph: 孤立骨格画素%d件を無視します(ノイズの可能性)。",
            isolated_count,
        )

    junction_pixels = {p for p, d in degree.items() if d >= 3}
    endpoint_pixels = {p for p, d in degree.items() if d == 1}

    nodes = []
    pixel_to_node = {}

    def register_node(pixels, kind):
        node_id = len(nodes)
        ys_arr = np.array([p[0] for p in pixels], dtype=float)
        xs_arr = np.array([p[1] for p in pixels], dtype=float)
        nodes.append({
            "id": node_id,
            "x": float(xs_arr.mean()),
            "y": float(ys_arr.mean()),
            "kind": kind,
            "pixel_count": len(pixels),
        })
        for p in pixels:
            pixel_to_node[p] = node_id

    # 隣接する分岐画素同士をBFSでまとめ、1つの交差点(ノード)にする。
    visited_junction = set()
    for start in junction_pixels:
        if start in visited_junction:
            continue
        cluster = []
        queue = collections.deque([start])
        visited_junction.add(start)
        while queue:
            cur = queue.popleft()
            cluster.append(cur)
            for q in neighbors(cur):
                if q in junction_pixels and q not in visited_junction:
                    visited_junction.add(q)
                    queue.append(q)
        register_node(cluster, "junction")

    for p in endpoint_pixels:
        register_node([p], "endpoint")

    # エッジ探索: 各ノードの構成画素からノード外へ1歩踏み出し、次数2の画素列を
    # 辿って別のノード(またはノード自身、自己ループ)へ到達するまで追跡する。
    # 踏破済みの有向ステップ(from_pixel, to_pixel)を記録し、反対向きの踏破も
    # 合わせて記録することで、同じエッジを両端から二重に検出しないようにする。
    visited_steps = set()
    edges = []

    for node in list(nodes):
        node_id = node["id"]
        node_pixels = {p for p, nid in pixel_to_node.items() if nid == node_id}
        for p in node_pixels:
            for q in neighbors(p):
                if q in node_pixels or (p, q) in visited_steps:
                    continue

                path = [p, q]
                visited_steps.add((p, q))
                prev, cur = p, q
                while cur not in pixel_to_node:
                    forward = [n for n in neighbors(cur) if n != prev]
                    if len(forward) != 1:
                        # 次数2でない画素に迷い込んだ(想定外の骨格形状)。
                        # 誤ったエッジを作らないよう安全側に打ち切る。
                        cur = None
                        break
                    nxt = forward[0]
                    visited_steps.add((cur, nxt))
                    path.append(nxt)
                    prev, cur = cur, nxt

                if cur is None or cur not in pixel_to_node:
                    continue

                to_node_id = pixel_to_node[cur]
                for a, b in zip(path[:-1], path[1:]):
                    visited_steps.add((b, a))

                path_xy = [(float(x), float(y)) for (y, x) in path]
                length_px = float(np.sum(np.hypot(
                    *np.diff(np.asarray(path_xy, dtype=float), axis=0).T
                )))
                edges.append({
                    "from": node_id,
                    "to": to_node_id,
                    "points": path_xy,
                    "length_px": length_px,
                })

    n_junction = sum(1 for n in nodes if n["kind"] == "junction")
    n_endpoint = sum(1 for n in nodes if n["kind"] == "endpoint")
    logging.info(
        "build_skeleton_graph: 交差点ノード=%d, 端点ノード=%d, エッジ=%d",
        n_junction, n_endpoint, len(edges),
    )

    return {"nodes": nodes, "edges": edges}


def _prune_graph_spurs(nodes, edges, min_spur_length_px):
    """端点ノードにつながる短いエッジ(骨格化のギザギザで生じる「ヒゲ」)を除去する。

    nodes(id -> ノード辞書)・edges(リスト。除去済み要素はNoneにする)を直接
    書き換える。端点の除去で交差点ノードが行き止まり(次数1以下)になった場合は
    端点へ格下げし、次のループでさらに短ければ連鎖的に刈る。
    """
    changed = True
    while changed:
        changed = False

        adjacency = collections.defaultdict(list)
        for i, e in enumerate(edges):
            if e is None:
                continue
            adjacency[e["from"]].append(i)
            adjacency[e["to"]].append(i)

        for node_id in list(nodes.keys()):
            if nodes[node_id]["kind"] != "endpoint":
                continue
            edge_idxs = adjacency.get(node_id, [])
            if len(edge_idxs) == 0:
                del nodes[node_id]
                changed = True
                continue
            if len(edge_idxs) != 1:
                continue  # 想定外(端点は通常次数1)。安全側でそのまま残す。
            e = edges[edge_idxs[0]]
            if e["length_px"] < min_spur_length_px:
                edges[edge_idxs[0]] = None
                del nodes[node_id]
                changed = True

        # 端点除去の結果、行き止まりになった交差点を端点へ格下げする。
        adjacency = collections.defaultdict(list)
        for i, e in enumerate(edges):
            if e is None:
                continue
            adjacency[e["from"]].append(i)
            adjacency[e["to"]].append(i)

        for node_id in list(nodes.keys()):
            if nodes[node_id]["kind"] != "junction":
                continue
            edge_idxs = adjacency.get(node_id, [])
            if len(edge_idxs) == 0:
                del nodes[node_id]
                changed = True
            elif len(edge_idxs) == 1:
                nodes[node_id]["kind"] = "endpoint"
                changed = True


def _merge_close_junction_nodes(nodes, edges, merge_junction_distance_px):
    """交差点ノード同士を結ぶ短いエッジをUnion-Findで1つの交差点に統合する。

    実際の交差点は骨格化すると数画素〜十数画素のクラスタに分裂しやすく、
    build_skeleton_graph()の隣接画素クラスタ化(直接接する画素同士しかまとめない)
    だけでは1つの交差点として拾いきれないことがあるため、座標が近いノード同士を
    まとめ直す。nodes・edgesは変更せず、統合後の新しいnodes(dict)・edges(list)を
    返す。
    """
    parent = {node_id: node_id for node_id in nodes}

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    for e in edges:
        if e is None:
            continue
        a, b = e["from"], e["to"]
        if a == b:
            continue  # 自己ループは統合対象外
        if (nodes[a]["kind"] == "junction" and nodes[b]["kind"] == "junction"
                and e["length_px"] < merge_junction_distance_px):
            union(a, b)

    groups = collections.defaultdict(list)
    for node_id in nodes:
        groups[find(node_id)].append(node_id)

    merged_nodes = {}
    remap = {}
    for root, members in groups.items():
        if len(members) == 1:
            merged_nodes[root] = dict(nodes[root])
        else:
            # 統合後の座標は画素数で重み付けした重心とする。
            total_px = sum(nodes[m]["pixel_count"] for m in members)
            x = sum(nodes[m]["x"] * nodes[m]["pixel_count"] for m in members) / total_px
            y = sum(nodes[m]["y"] * nodes[m]["pixel_count"] for m in members) / total_px
            merged_nodes[root] = {
                "id": root, "x": x, "y": y, "kind": "junction", "pixel_count": total_px,
            }
        for m in members:
            remap[m] = root

    merged_edges = []
    for e in edges:
        if e is None:
            continue
        new_from, new_to = remap[e["from"]], remap[e["to"]]
        if new_from == new_to and e["from"] != e["to"]:
            continue  # 統合の結果、自己ループ化した短いエッジは不要なので捨てる
        e2 = dict(e)
        e2["from"], e2["to"] = new_from, new_to
        merged_edges.append(e2)

    return merged_nodes, merged_edges


def _collapse_pass_through_nodes(nodes, edges):
    """エッジがちょうど2本になった交差点ノード(実際には分岐していない、単なる
    通路の折れ点)を取り除き、前後のエッジを1本に連結する。

    nodes・edgesを直接書き換える。_merge_close_junction_nodes()による統合後に
    生じることがある(3方向あった交差点のうち2方向が同じ相手に統合された場合など)。
    """
    def orient_away(e, node_id):
        """node_idから離れる向き(node_id -> 反対側)に並んだ点列を返す。"""
        if e["from"] == node_id:
            return list(e["points"])
        return list(reversed(e["points"]))

    changed = True
    while changed:
        changed = False

        adjacency = collections.defaultdict(list)
        for i, e in enumerate(edges):
            if e is None:
                continue
            adjacency[e["from"]].append(i)
            adjacency[e["to"]].append(i)

        for node_id in list(nodes.keys()):
            if nodes[node_id]["kind"] != "junction":
                continue
            edge_idxs = adjacency.get(node_id, [])
            if len(edge_idxs) != 2 or edge_idxs[0] == edge_idxs[1]:
                continue  # 分岐がちょうど2本でない、または自己ループのみは対象外
            i1, i2 = edge_idxs
            e1, e2 = edges[i1], edges[i2]
            other1 = e1["to"] if e1["from"] == node_id else e1["from"]
            other2 = e2["to"] if e2["from"] == node_id else e2["from"]

            seg1 = list(reversed(orient_away(e1, node_id)))  # other1 -> node_id
            seg2 = orient_away(e2, node_id)                   # node_id -> other2
            new_edge = {
                "from": other1,
                "to": other2,
                "points": seg1 + seg2[1:],  # 継ぎ目(node_id)の重複を除く
                "length_px": e1["length_px"] + e2["length_px"],
            }
            edges[i1] = new_edge
            edges[i2] = None
            del nodes[node_id]
            changed = True
            break  # ノード構成が変わったので隣接情報を作り直してから続行する


def simplify_skeleton_graph(graph, min_spur_length_px=15.0, merge_junction_distance_px=20.0):
    """build_skeleton_graph()の出力を整理し、骨格化特有のノイズ(短い枝・
    本来1つの交差点が近接した複数ノードに分裂する現象)を減らす。[本研究独自]

    処理内容(この順で適用):
      1. 枝刈り: 端点につながる長さmin_spur_length_px未満のエッジを除去する。
      2. 交差点統合: 交差点ノード同士を結ぶ長さmerge_junction_distance_px未満の
         エッジを1つの交差点に統合する(Union-Find)。
      3. 通過点の統合: 統合後にエッジがちょうど2本になった交差点ノード(実際には
         分岐していない折れ点)を取り除き、前後のエッジを1本に連結する。
      4. 手順1を再適用し、手順2・3の結果生じうる新たな短い行き止まりを整理する。

    引数:
      graph: build_skeleton_graph()の戻り値。
      min_spur_length_px: この長さ未満の端点向けエッジを枝刈り対象にする
        (既定15px。kanri_4fではscale_px_per_m=11.4のため約1.3m)。
      merge_junction_distance_px: この距離未満の交差点間エッジを統合対象にする
        (既定20px。AUTO_ROUTE_MAX_HALF_WIDTH_PXと同程度=通路1本分の幅以内なら
        同じ交差点とみなす、という目安)。

    戻り値: build_skeleton_graph()と同じ形式({"nodes": [...], "edges": [...]})。
      ノードidは0から振り直される。
    """
    nodes = {n["id"]: dict(n) for n in graph["nodes"]}
    edges = [dict(e) for e in graph["edges"]]

    _prune_graph_spurs(nodes, edges, min_spur_length_px)
    nodes, edges = _merge_close_junction_nodes(nodes, edges, merge_junction_distance_px)
    _collapse_pass_through_nodes(nodes, edges)
    _prune_graph_spurs(nodes, edges, min_spur_length_px)

    # idを0から振り直し、Noneになった除去済みエッジを取り除く。
    ordered_ids = sorted(nodes.keys())
    id_map = {old: new for new, old in enumerate(ordered_ids)}
    final_nodes = []
    for old_id in ordered_ids:
        n = dict(nodes[old_id])
        n["id"] = id_map[old_id]
        final_nodes.append(n)
    final_edges = []
    for e in edges:
        if e is None:
            continue
        e2 = dict(e)
        e2["from"] = id_map[e["from"]]
        e2["to"] = id_map[e["to"]]
        final_edges.append(e2)

    logging.info(
        "simplify_skeleton_graph: 整理後 交差点ノード=%d, 端点ノード=%d, エッジ=%d "
        "(整理前: 交差点=%d, 端点=%d, エッジ=%d)",
        sum(1 for n in final_nodes if n["kind"] == "junction"),
        sum(1 for n in final_nodes if n["kind"] == "endpoint"),
        len(final_edges),
        sum(1 for n in graph["nodes"] if n["kind"] == "junction"),
        sum(1 for n in graph["nodes"] if n["kind"] == "endpoint"),
        len(graph["edges"]),
    )

    return {"nodes": final_nodes, "edges": final_edges}


# ============================================================
# 複数経路仮説PF: グラフのトポロジー変換(粒子単位のグラフ分岐PF方式)
# ============================================================
# 既存のROUTE_POINTS方式は、1本の折れ線を"区間インデックス"で順に辿りながら
# 各区間の方位でセンサー方位を補正する(get_route_segment_heading等)。この考え方
# 自体は分岐のない直線経路に対しては有効に機能している(kanri_4fで全滅回数
# -16.8%・終点誤差-18.5%、CLAUDE.md参照)。複数経路仮説PFは、この"区間インデックス"
# を経路全体で1つのスカラーとして共有するのではなく、粒子ごとに個別の
# (エッジid, 区間インデックス, 進行方向)として持たせ、交差点に到達した粒子ごとに
# 出口エッジを確率的に選ばせる(=粒子群が複数の経路仮説を自然に表現する)ことを
# 目指す。build_route_graph_topology()はその土台となるデータ構造を作る関数。

def build_route_graph_topology(graph, simplify_tolerance_px=10.0):
    """simplify_skeleton_graph()の出力を、粒子単位のグラフ分岐PFが直接使える
    形に変換する。[本研究独自]

    各エッジのpoints(細線化した画素そのままのジグザグな折れ線)を
    _rdp_simplify()で間引き、区間ごとの方位(seg_headings)・長さ(seg_lengths)を
    事前計算する(extract_ordered_centerline()がROUTE_POINTSを作るのと同じ考え方を
    グラフの各エッジに適用したもの)。

    さらに、各ノードに接続する(エッジid, 進行方向)の一覧(adjacency)を作る。
    進行方向は、そのノードを出発点としてエッジのpoints配列をどちら向きに
    辿るかを表す:
      +1: エッジの"from"側のノード。points配列を先頭(from)から末尾(to)へ
          そのままの順で辿る。
      -1: エッジの"to"側のノード。points配列を末尾(to)から先頭(from)へ
          逆順に辿る。
    自己ループ(from == to)は同じノードから両方向に出られるとみなし、
    (+1, -1)の両方を登録する。

    引数:
      graph: simplify_skeleton_graph()(またはbuild_skeleton_graph())の戻り値。
      simplify_tolerance_px: 各エッジの折れ線簡略化の許容誤差(px)。
        既定はAUTO_ROUTE_CENTERLINE_SIMPLIFY_PXの既定値と同じ10.0px。

    戻り値: {
      "edges": [
        {"id", "from", "to",
         "points": 簡略化後の折れ線([(x,y), ...]、from→to順、2点以上),
         "seg_headings": np.ndarray(rad, 長さ=len(points)-1、points[i]->points[i+1]の方位),
         "seg_lengths": np.ndarray(px, 長さ=len(points)-1),
         "total_length_px": float},
        ...
      ],
      "edges_by_id": {edge_id: 上記と同じdict, ...},  # O(1)参照用
      "adjacency": {node_id: [(edge_id, direction), ...], ...},
      "nodes": {node_id: ノード辞書, ...},  # simplify_skeleton_graphのnodesをid引き
    }
    簡略化後に2点未満になったエッジ(通常はsimplify_skeleton_graphの枝刈りで
    既に除去されているはずだが、念のため)は除外する。
    """
    nodes_by_id = {n["id"]: n for n in graph["nodes"]}

    topo_edges = []
    adjacency = collections.defaultdict(list)
    for edge_id, e in enumerate(graph["edges"]):
        points = _rdp_simplify(e["points"], simplify_tolerance_px)
        if len(points) < 2:
            continue
        points_arr = np.asarray(points, dtype=float)
        deltas = np.diff(points_arr, axis=0)
        seg_lengths = np.hypot(deltas[:, 0], deltas[:, 1])
        seg_headings = np.arctan2(deltas[:, 1], deltas[:, 0])

        # 簡略化後も長さ0の区間が残った場合、方位がnanになるのを防ぐ安全策
        # (通常は_rdp_simplifyの間引きで発生しないはずだが念のため)。
        zero_len = seg_lengths < 1e-9
        if np.any(zero_len):
            valid_idx = np.where(~zero_len)[0]
            if len(valid_idx) == 0:
                continue
            seg_headings = np.where(zero_len, seg_headings[valid_idx[0]], seg_headings)

        topo_edges.append({
            "id": edge_id,
            "from": e["from"],
            "to": e["to"],
            "points": [tuple(p) for p in points_arr],
            "seg_headings": seg_headings,
            "seg_lengths": seg_lengths,
            "total_length_px": float(seg_lengths.sum()),
        })
        adjacency[e["from"]].append((edge_id, +1))
        if e["to"] != e["from"]:
            adjacency[e["to"]].append((edge_id, -1))
        else:
            adjacency[e["from"]].append((edge_id, -1))

    return {
        "edges": topo_edges,
        "edges_by_id": {e["id"]: e for e in topo_edges},
        "adjacency": dict(adjacency),
        "nodes": nodes_by_id,
    }


def nearest_edge_position(topology, x, y):
    """指定座標(x, y)に最も近いエッジ上の点を探し、(edge_id, seg_index, t)を返す。

    seg_indexはそのエッジのpoints配列における区間番号(points[seg_index] ->
    points[seg_index+1])、tは区間内の位置(0.0=区間始点、1.0=区間終点)を表す。
    複数経路仮説PFの初期化(開始位置に最も近いエッジ・区間へ粒子を割り当てる)に
    使う想定。nearest_route_segment_index()のグラフ版に相当する。

    トポロジーにエッジが1本もない場合は(None, None, None)を返す。
    """
    best = None  # (distance, edge_id, seg_index, t)
    point = np.array([x, y], dtype=float)
    for e in topology["edges"]:
        points_arr = np.asarray(e["points"], dtype=float)
        for seg_index in range(len(points_arr) - 1):
            start, end = points_arr[seg_index], points_arr[seg_index + 1]
            direction_vec = end - start
            length_sq = float(np.dot(direction_vec, direction_vec))
            if length_sq <= 1e-12:
                continue
            t = float(np.clip(np.dot(point - start, direction_vec) / length_sq, 0.0, 1.0))
            nearest_point = start + t * direction_vec
            distance = float(np.hypot(point[0] - nearest_point[0], point[1] - nearest_point[1]))
            if best is None or distance < best[0]:
                best = (distance, e["id"], seg_index, t)

    if best is None:
        return None, None, None
    _, edge_id, seg_index, t = best
    return edge_id, seg_index, t


def edge_entry_heading(edge, direction):
    """エッジedgeに指定方向(+1=from->to, -1=to->from)で進入した場合の、
    最初の区間の進行方位(rad)を返す。分岐候補の方位重み付け
    (choose_branch_by_heading)に使う小さな純粋関数として切り出した。
    """
    seg_headings = edge["seg_headings"]
    if direction > 0:
        return float(seg_headings[0])
    reversed_heading = seg_headings[-1] + np.pi
    return float(np.arctan2(np.sin(reversed_heading), np.cos(reversed_heading)))


def choose_branch_by_heading(candidates, edges_by_id, reference_heading, sigma_rad):
    """[本研究独自] 複数経路仮説PFの交差点分岐選択(2026-08-30、Week3+改善)。

    候補(edge_id, direction)のうち、reference_heading(直前の実測方位)に
    近い方位で進入するエッジほど選ばれやすいガウス重み付き乱択で1つ選ぶ。
    従来のnp.random.randintによる完全一様乱択(2026-08-16実装)を置き換える。
    全候補の方位差が同程度ならほぼ一様乱択に近づくため、直進が明確な分岐では
    正しい枝を優先しつつ、判断がつかない場合は従来通り確率的に探索する。

    candidates: [(edge_id, direction), ...] (逆走候補は呼び出し側で除外済みの
    前提。1件のみならそのまま返す)
    sigma_rad: 重みの広がり(rad)。小さいほど方位が近い候補に強く偏る。
    """
    if len(candidates) == 1:
        return candidates[0]
    headings = np.array([
        edge_entry_heading(edges_by_id[edge_id], direction)
        for edge_id, direction in candidates
    ])
    diffs = np.arctan2(np.sin(headings - reference_heading), np.cos(headings - reference_heading))
    weights = np.exp(-(diffs ** 2) / (2.0 * sigma_rad ** 2))
    total = float(weights.sum())
    if not np.isfinite(total) or total <= 0.0:
        weights = np.ones(len(candidates))
        total = float(weights.sum())
    probs = weights / total
    choice_idx = np.random.choice(len(candidates), p=probs)
    return candidates[choice_idx]
