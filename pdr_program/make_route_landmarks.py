# ============================================================================
# make_route_landmarks.py
#
# 【変更履歴】
# - 2026-09-03: [本研究独自] 新規作成。正解位置データ作成(進捗反映版メモ§27.1 B)の
#               Phase 0を、クリック方式から「マスター表＋経路定義」方式へ置き換える
#               ためのツール。
#
# 【なぜクリック方式(pick_landmarks.py)を使わないか】
# pick_landmarks.pyはPFが使う二値地図を表示してクリックさせる設計だが、kanri_4fの
# 二値地図は廊下沿いに開口が1つも無く(壁線が連続)、真っ白な帯のどこをクリックすべきか
# 判断できない。座標は、元の平面図から検出した壁位置と、現地でのメジャー実測から
# 決める方が正確である。よってこのプロジェクトではpick_landmarks.pyは使わない。
#
# 【このスクリプトについて】
# 目印のマスター表(ground_truth/kanri_4f_landmarks_master.csv)と経路定義
# (ground_truth/routes.json)から、経路ごとの landmarks CSV を生成する。
#
#   マスター表(物理的な点の一覧。建物は動かないので1つで済む)
#     id, label, point_type, x_px, y_px, source, 見分け方
#        ↓  routes.json で「この順に押す」というIDの並びを与える
#   経路別landmarks CSV(build_ground_truth.pyが読む形式)
#     seq, label, point_type, x_px, y_px
#
# 経路ごとに別ファイルが必要なのは、build_ground_truth.pyがwaypointsとlandmarksの
# seqの集合の完全一致を要求するため。逆方向に歩く経路は、同じ物理点をIDの並びだけ
# 逆にして書けば、seqは並び順に1から振り直される。
#
# 【使い方】
#   python make_route_landmarks.py --list
#     -> 定義済みの経路と、各経路が現地採寸待ちの点を含むかを一覧する。
#
#   python make_route_landmarks.py --route rehearsal
#     -> ground_truth/kanri_4f_landmarks_rehearsal.csv と
#        ground_truth/押す順番_rehearsal.md を生成する。
#
#   python make_route_landmarks.py --self-test
#     -> 合成データで生成ロジックだけを確認する(実ファイル不要)。
# ============================================================================

import argparse
import json
import sys
import tempfile
from pathlib import Path

import numpy as np
import pandas as pd

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_DIR = SCRIPT_DIR / "ground_truth"
DEFAULT_MASTER = DEFAULT_DIR / "kanri_4f_landmarks_master.csv"
DEFAULT_ROUTES = DEFAULT_DIR / "routes.json"
DEFAULT_MAP_CONFIG = SCRIPT_DIR / "map_configs" / "kanri_4f.json"

MASTER_COLUMNS = ("id", "label", "point_type", "x_px", "y_px")
OUTPUT_COLUMNS = ["seq", "label", "point_type", "x_px", "y_px"]

# これ以上xとyが両方動いたら、廊下から直交する廊下へ曲がった区間とみなす(約1.8m)。
CORNER_MIN_PX = 20.0


def load_master(path):
    """マスター表を読み込む。必須列の欠落とidの重複はここでエラーにする。"""
    df = pd.read_csv(path)
    missing = [c for c in MASTER_COLUMNS if c not in df.columns]
    if missing:
        raise ValueError(f"{path}: 必須列が不足しています: {missing}")
    dup = df["id"][df["id"].duplicated()].tolist()
    if dup:
        raise ValueError(f"{path}: idが重複しています: {dup}")
    return df.set_index("id", drop=False)


def load_routes(path):
    """経路定義JSONを読み込む。'_'で始まるキーは説明用として無視する。"""
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    return {k: v for k, v in data.items() if not k.startswith("_")}


def resolve_route(master, ids, route_name):
    """IDの並びからlandmarks DataFrameを作る。seqは並び順に1から振る。

    未定義のid・重複・座標未記入(現地採寸待ち)は、どれが問題かを具体的に示して
    ValueErrorにする。黙って落としたり0で埋めたりすると、押す順番とseqがずれて
    正解位置が丸ごと間違ったまま気づけなくなる。
    """
    if len(ids) < 2:
        raise ValueError(f"経路'{route_name}': 目印が{len(ids)}点しかありません(2点以上必要)。")

    unknown = [i for i in ids if i not in master.index]
    if unknown:
        raise ValueError(
            f"経路'{route_name}': マスター表に存在しないid: {unknown}\n"
            f"  マスター表にあるid: {list(master.index)}"
        )

    dup = sorted({i for i in ids if ids.count(i) > 1})
    if dup:
        raise ValueError(f"経路'{route_name}': 同じidが複数回現れています: {dup}")

    rows = []
    unmeasured = []
    for seq, lm_id in enumerate(ids, start=1):
        r = master.loc[lm_id]
        x, y = r["x_px"], r["y_px"]
        if pd.isna(x) or pd.isna(y):
            unmeasured.append((lm_id, str(r.get("見分け方", ""))))
            continue
        rows.append({
            "seq": seq,
            "label": r["label"],
            "point_type": r["point_type"],
            "x_px": float(x),
            "y_px": float(y),
            "id": lm_id,
            "見分け方": r.get("見分け方", ""),
        })

    if unmeasured:
        detail = "\n".join(f"    {i}: {note}" for i, note in unmeasured)
        raise ValueError(
            f"経路'{route_name}': 座標が未記入の目印が{len(unmeasured)}点あります"
            f"(現地採寸待ち)。\n{detail}\n"
            "  現地採寸メモ.md に従って実測し、マスター表のx_px, y_pxを埋めてください。"
        )

    return pd.DataFrame(rows)


def segment_lengths(df, scale_px_per_m):
    """隣接する目印の間隔[m]を返す。廊下が直交する曲がり部は直線距離ではなく
    L字に歩くため、xとyが両方大きく変わる区間は折れ線長(|dx|+|dy|)も併記できる
    よう2種類を返す。"""
    x = df["x_px"].to_numpy(float)
    y = df["y_px"].to_numpy(float)
    dx = np.diff(x)
    dy = np.diff(y)
    straight = np.hypot(dx, dy) / scale_px_per_m
    polyline = (np.abs(dx) + np.abs(dy)) / scale_px_per_m
    # 廊下から直交する廊下へ曲がった区間だけをL字扱いする。閾値が小さすぎると、
    # 同じ廊下の中で少し斜めに寄っただけの区間まで折れ線長で数えて距離を過大に出す。
    is_corner = (np.abs(dx) > CORNER_MIN_PX) & (np.abs(dy) > CORNER_MIN_PX)
    return straight, polyline, is_corner


def print_summary(df, route_name, description, scale_px_per_m):
    straight, polyline, is_corner = segment_lengths(df, scale_px_per_m)
    walked = np.where(is_corner, polyline, straight)
    print(f"\n経路 '{route_name}': {description}")
    print(f"  目印 {len(df)} 点 / 歩行距離の目安 {walked.sum():.1f} m "
          f"(1m = {scale_px_per_m} px)")
    print(f"  {'seq':>3} {'id':<5} {'label':<16} {'x_px':>7} {'y_px':>6} "
          f"{'前点から[m]':>11}  累積[m]")
    cum = 0.0
    for i, r in df.iterrows():
        if i == 0:
            gap, mark = "     -", " "
        else:
            cum += walked[i - 1]
            gap = f"{walked[i-1]:6.1f}"
            mark = "L" if is_corner[i - 1] else " "
        print(f"  {int(r['seq']):>3} {r['id']:<5} {r['label']:<16} "
              f"{r['x_px']:>7.1f} {r['y_px']:>6.1f} {gap}{mark}      {cum:5.1f}")
    if is_corner.any():
        print("  ※ L印の区間は廊下が直交する曲がり部で、直線ではなくL字に歩く区間。")
    if len(walked):
        print(f"  間隔: 最小 {walked.min():.1f} m / 最大 {walked.max():.1f} m / "
              f"平均 {walked.mean():.1f} m")


def write_sheet(df, path, route_name, description, scale_px_per_m):
    """当日持ち歩く「押す順番シート」を生成する。"""
    straight, polyline, is_corner = segment_lengths(df, scale_px_per_m)
    walked = np.where(is_corner, polyline, straight)
    lines = [
        f"# 押す順番シート — 経路 `{route_name}`",
        "",
        description,
        "",
        f"目印 {len(df)} 点 / 歩行距離の目安 {walked.sum():.1f} m",
        "",
        "**押す順番を絶対に間違えないこと。** 押し忘れ・押し過ぎは build_ground_truth.py が",
        "検出するが、順番だけを取り違えた場合は検出できず、正解位置が丸ごとずれる。",
        "途中で分からなくなったら、その場で記録を中止してやり直す方が安い。",
        "",
        "| 押す順 | 前の点から | 何を見て押すか |",
        "|---|---|---|",
    ]
    for i, r in df.iterrows():
        gap = "スタート" if i == 0 else f"約 {walked[i-1]:.0f} m"
        lines.append(f"| **{int(r['seq'])}** | {gap} | {r['見分け方']} |")
    lines += [
        "",
        "## 記録",
        "",
        "| 項目 | 記入 |",
        "|---|---|",
        "| 日付・時刻 | ____________ |",
        "| CSVファイル名 | ____________ |",
        "| 端末の保持方法 | 手持ち(胸の高さ) / ポケット / その他: ______ |",
        "| 歩き方 | ゆっくり / 普通 / 速歩 |",
        "| 機内モード | ON にした / していない |",
        "| STOP時ダイアログ | 問題なし / 警告あり(内容: ____________) |",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def read_scale(map_config_path):
    """地図設定JSONから scale_px_per_m を読む(必須値。欠落はエラー)。"""
    with open(map_config_path, encoding="utf-8") as f:
        cfg = json.load(f)
    if "scale_px_per_m" not in cfg:
        raise ValueError(f"{map_config_path}: scale_px_per_m がありません。")
    return float(cfg["scale_px_per_m"])


def cmd_list(master, routes):
    print(f"マスター表: {len(master)} 点 "
          f"(座標確定 {int(master['x_px'].notna().sum())} / "
          f"未記入 {int(master['x_px'].isna().sum())})")
    unmeasured_ids = set(master.index[master["x_px"].isna()])
    print(f"\n定義済みの経路 ({len(routes)}):")
    for name, r in routes.items():
        ids = r["ids"]
        pending = [i for i in ids if i in unmeasured_ids]
        state = "今すぐ生成できる" if not pending else f"現地採寸待ち {pending}"
        print(f"  {name:<14} {len(ids):>2}点  {state}")
        print(f"                 {r.get('説明','')}")


def _self_test():
    """[本研究独自] 合成データによる自己テスト。外部ファイルを必要としない。

    【重要】ここで使う座標はすべて架空の値であり、研究結果として報告してはならない。
    確認するのは生成ロジック(seqの振り方・逆順・未記入の検出)だけである。
    """
    print("--- self-test 開始(架空データ。数値は研究結果ではない) ---")
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        master_path = td / "master.csv"
        pd.DataFrame([
            {"id": "A", "label": "west", "point_type": "end", "x_px": 0.0,
             "y_px": 100.0, "source": "図面確定", "見分け方": "西の壁"},
            {"id": "B", "label": "mid", "point_type": "wall", "x_px": 114.0,
             "y_px": 100.0, "source": "図面確定", "見分け方": "室境界"},
            {"id": "C", "label": "corner", "point_type": "corner", "x_px": 228.0,
             "y_px": 100.0, "source": "図面確定", "見分け方": "曲がり角"},
            {"id": "D", "label": "door", "point_type": "door", "x_px": np.nan,
             "y_px": 100.0, "source": "現地採寸", "見分け方": "ドア(未採寸)"},
        ]).to_csv(master_path, index=False)
        master = load_master(master_path)

        # 1) 正順: seqが1から順に振られる
        df = resolve_route(master, ["A", "B", "C"], "t1")
        assert list(df["seq"]) == [1, 2, 3], df
        assert list(df["label"]) == ["west", "mid", "corner"], df
        print("  OK: seqが並び順に1から振られる")

        # 2) 逆順: 同じ物理点でもseqが逆に振り直される
        rev = resolve_route(master, ["C", "B", "A"], "t2")
        assert list(rev["seq"]) == [1, 2, 3], rev
        assert list(rev["label"]) == ["corner", "mid", "west"], rev
        assert rev.loc[0, "x_px"] == 228.0 and rev.loc[2, "x_px"] == 0.0
        print("  OK: 逆方向の経路でseqが振り直される")

        # 3) 間隔の計算(114px, scale 11.4 -> 10.0 m)
        straight, polyline, is_corner = segment_lengths(df, 11.4)
        assert np.allclose(straight, [10.0, 10.0]), straight
        assert not is_corner.any()
        print("  OK: 間隔[m]の計算")

        # 4) 曲がり部はL字長で数える
        corner_master = load_master(master_path)
        corner_master.loc["C", "y_px"] = 0.0   # B(114,100) -> C(228,0) でL字に曲がる
        dfc = resolve_route(corner_master, ["B", "C"], "t3")
        s3, p3, c3 = segment_lengths(dfc, 11.4)
        assert c3[0], "曲がり部として判定されるべき"
        assert np.isclose(p3[0], (114 + 100) / 11.4), p3
        # 同じ廊下の中で少し斜めに寄っただけの区間はL字扱いしない
        near_master = load_master(master_path)
        near_master.loc["C", "y_px"] = 100.0 - CORNER_MIN_PX / 2
        dfn = resolve_route(near_master, ["B", "C"], "t3b")
        _, _, cn = segment_lengths(dfn, 11.4)
        assert not cn[0], "わずかな横ずれを曲がり部と誤判定している"
        print("  OK: 曲がり部の折れ線長と、横ずれとの区別")

        # 5) 未記入の座標はエラーになる(黙って落とさない)
        try:
            resolve_route(master, ["A", "D", "C"], "t4")
        except ValueError as e:
            assert "D" in str(e) and "現地採寸" in str(e), str(e)
            print("  OK: 座標未記入をエラーとして検出")
        else:
            raise AssertionError("未記入の座標がエラーにならなかった")

        # 6) 未定義id・重複・点数不足
        for ids, kw in ((["A", "Z"], "存在しない"), (["A", "B", "A"], "複数回"),
                        (["A"], "2点以上")):
            try:
                resolve_route(master, ids, "t5")
            except ValueError as e:
                assert kw in str(e), str(e)
            else:
                raise AssertionError(f"{ids} がエラーにならなかった")
        print("  OK: 未定義id・重複・点数不足を検出")

    print("--- self-test 全て通過 ---")


def main():
    parser = argparse.ArgumentParser(
        description="マスター表と経路定義から、経路別のlandmarks CSVを生成する。"
                    "詳細はこのファイル冒頭のコメントを参照。")
    parser.add_argument("--route", default=None, help="生成する経路名(routes.jsonのキー)。")
    parser.add_argument("--master", type=Path, default=DEFAULT_MASTER)
    parser.add_argument("--routes", type=Path, default=DEFAULT_ROUTES)
    parser.add_argument("--map-config", type=Path, default=DEFAULT_MAP_CONFIG,
                        help="scale_px_per_m の取得元。")
    parser.add_argument("--output", type=Path, default=None,
                        help="出力CSV。省略時は ground_truth/kanri_4f_landmarks_<route>.csv")
    parser.add_argument("--sheet", type=Path, default=None,
                        help="押す順番シート(md)。省略時は ground_truth/押す順番_<route>.md")
    parser.add_argument("--list", action="store_true", help="経路の一覧を表示して終了する。")
    parser.add_argument("--self-test", action="store_true",
                        help="合成データで生成ロジックだけを確認する。")
    args = parser.parse_args()

    if args.self_test:
        _self_test()
        return

    master = load_master(args.master)
    routes = load_routes(args.routes)

    if args.list or args.route is None:
        cmd_list(master, routes)
        if args.route is None and not args.list:
            print("\n--route <経路名> を指定してください。")
            sys.exit(1)
        return

    if args.route not in routes:
        print(f"経路'{args.route}'は定義されていません。定義済み: {list(routes)}")
        sys.exit(1)

    route = routes[args.route]
    scale = read_scale(args.map_config)
    df = resolve_route(master, list(route["ids"]), args.route)

    out = args.output or (DEFAULT_DIR / f"kanri_4f_landmarks_{args.route}.csv")
    sheet = args.sheet or (DEFAULT_DIR / f"押す順番_{args.route}.md")
    out.parent.mkdir(parents=True, exist_ok=True)
    df[OUTPUT_COLUMNS].to_csv(out, index=False)
    write_sheet(df, sheet, args.route, route.get("説明", ""), scale)

    print_summary(df, args.route, route.get("説明", ""), scale)
    print(f"\n  landmarks CSV: {out}")
    print(f"  押す順番シート: {sheet}")
    print("  Android側で地点マークボタンを押す順番は、必ずこのseqの順にしてください。")


if __name__ == "__main__":
    main()
