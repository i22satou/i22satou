# ============================================================================
# run_evaluation.py
#
# 【変更履歴】
# - 2026-09-18: [本研究独自] 新規作成。計測データと計測一覧表を置いて1回実行すれば、
#               卒論第7章の比較(方式別・ファイル別のRMSE表、箱ひげ図、軌跡図、実行条件の
#               記録)まで作る評価ハーネス。
#
# 【このスクリプトについて】
# 本体(pdr_pf_improved.py)は変更せず、compare_route_source.py と同じく subprocess で外から
# 呼ぶ。方式とオプションは README.md「比較方式と実行オプション」の表どおり(METHODS)。
# 既定OFFの機能も含めてON/OFFを全部明示して渡すので、JSONの既定値が将来変わっても
# 各方式の条件は変わらない。
#
# 【処理の流れ】
# 1. 計測一覧表の eval の行を読み、経路名から ground_truth/<地図名>_landmarks_<経路名>.csv を選ぶ。
#    - 開始位置 = 目印1番(seq=1)の座標。start_positions.csv に未登録なら、本体の既存の
#      save_start_position() で登録する。登録済みならそちらを優先し、目印から10px以上
#      ずれていれば警告する。
#    - 初期方位 = 目印1番→2番の向き。walking方式の基準方位は最初の10歩(約8m)で決まる
#      ので、その範囲に曲がり角がある経路は警告する。
#    - 経路名の書き間違いの確認: 地点マークを押した時刻の間隔と目印間の距離の相関を、
#      同じ点数の他の経路とも比べる(警告のみ。同じ12点の east_short と west_reverse は
#      間隔の並びの相関が -0.35 なので区別できる)。
#    - 経路が定義されていない記録(route空欄)は、一覧表の start_heading_deg と登録済みの
#      開始位置で実行し、RMSEは出さない。
# 2. build_ground_truth.py の関数で正解位置CSVを作る(出力フォルダ内。data_dirには書かない)。
#    地点マークCSVが無い記録は、RMSEを出さずに軌跡だけ出す。
# 3. 本体を「方式 × シード」で実行する。本体は1回の実行で1つの初期方位しか扱えないので、
#    初期方位ごとにCSVのコピーを一時フォルダへ分けて実行する(元のCSVには触れない)。
#    PDRのみ(方式A)は乱数を使わないので、各実行が出す同じ軌跡を1つ使う(全実行で
#    一致することも確かめる)。
# 4. evaluate_accuracy.py の関数でRMSE・平均誤差・最大誤差を計算し、表と図にまとめる。
#
# 【出力】results/<日時>_evaluation[_tag]/
#   table_by_method.csv       方式別。全ファイル×全シードの平均±標準偏差
#   table_by_method_file.csv  方式別・ファイル別。シード間の平均±標準偏差
#                             (方式Aは乱数を使わないので1ファイル1値)
#   table_diagnostics.csv     方式別・ファイル別の全滅回数・最終位置(正解位置が無くても出す。
#                             精度の指標ではない)
#   results_long.csv          1実行・1ファイルごとの全数値(全滅回数・最終位置も)
#   boxplot_rmse.png / boxplot_rmse_by_file.png / trajectory_<CSV名>.png
#   conditions.json           実行条件(gitのコミット番号、各方式のオプション、ファイル別の
#                             開始位置・初期方位・経路確認の結果、失敗した実行)
#   used_map_config.json / measurement_list.csv  使った設定JSONと一覧表のコピー
#   ground_truth/             作った正解位置CSV
#   runs/<方式>/seed-<シード>/  本体の各実行のPNG・ログ・軌跡CSV
#
# 【注意】
# - 正解位置の無い記録は表・箱ひげ図に入れない。
# - --self-test は架空データで本体を実際に動かす通し試験。本体のコピーを一時フォルダで
#   動かすので、本物の start_positions.csv と results/ には書き込まない。架空データから
#   出た数値は研究結果ではない(CLAUDE.md)。
#
# 【使い方】pdr_program/ で実行する。CSVフォルダは本体と同じ規則
# (--data-dir > 環境変数PDR_DATA_DIR > JSONのdata_dir)で決める。
#   python evaluation/run_evaluation.py --list <計測一覧.csv> [--tag 0925] [--seeds 1 7 42]
#   python evaluation/run_evaluation.py --self-test
# ============================================================================

import argparse
import contextlib
import hashlib
import json
import logging
import math
import os
import shutil
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
from matplotlib.colors import ListedColormap  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

EVALUATION_DIR = Path(__file__).resolve().parent
PROGRAM_DIR = EVALUATION_DIR.parent
sys.path.insert(0, str(PROGRAM_DIR))
sys.path.insert(0, str(EVALUATION_DIR))
import pdr_pf_improved as pdrmod  # noqa: E402
from build_ground_truth import build_ground_truth, load_landmarks, load_waypoints  # noqa: E402
from calibrate_step_length import git_revision  # noqa: E402
from compare_route_source import parse_log  # noqa: E402
from evaluate_accuracy import evaluate  # noqa: E402
from measurement_list import load_measurement_list, resolve_csv_path  # noqa: E402

RESULTS_DIR = PROGRAM_DIR / "results"
DEFAULT_SEEDS = [1, 7, 42, 100, 777, 2024]
START_MISMATCH_WARN_PX = 10.0   # 登録済みの開始位置と目印1番のずれの警告しきい値
WALKING_CALIBRATION_DIST_M = 8.0  # walking方式の基準方位(最初の10歩)がおよそ届く距離
EARLY_TURN_TOL_DEG = 20.0       # これ以上向きが変わったら「曲がり角」とみなす
ROUTE_FIT_MIN_R = 0.7           # 押した間隔と目印間の距離の相関の下限の目安
ROUTE_FIT_MARGIN = 0.1          # 他の経路の方がこれ以上よく合えば書き間違いを疑う
RUN_TIMEOUT_SEC = 900           # 本体がクリック待ちなどで止まった場合の保険


def condition_args(pf_mode="adaptive", route_mode="none", route_source="manual", unc=False,
                   mh=False, branch_sigma=0.0, rooms=False, centerline=False):
    """1つの方式の本体オプション。既定OFFの機能もON/OFFを全部明示する。"""
    return [
        "--pf-mode", pf_mode,
        "--route-constraint-mode", route_mode,
        "--route-source", route_source,
        "--uncertainty-adaptive-particles" if unc else "--no-uncertainty-adaptive-particles",
        "--multi-hypothesis-routing" if mh else "--no-multi-hypothesis-routing",
        "--multi-hypothesis-branch-likelihood-sigma-deg", f"{branch_sigma:g}",
        "--auto-route-exclude-wide-rooms" if rooms else "--no-auto-route-exclude-wide-rooms",
        "--auto-route-centerline" if centerline else "--no-auto-route-centerline",
    ]


# README.md「比較方式と実行オプション」の表と同じ内容。変えるときは両方を直すこと。
PROPOSED = dict(route_mode="enforce", route_source="auto", unc=True)
METHODS = [
    {"key": "A_pdr", "tiny": "A", "label": "PDRのみ", "short": "A\nPDRのみ", "section": "7.1",
     "kind": "main", "args": None},  # PFの各実行が出す軌跡を使う(乱数なし)
    {"key": "B_fixed", "tiny": "B", "label": "固定粒子数PF", "short": "B\n固定粒子数PF", "section": "7.2",
     "kind": "main", "args": condition_args(pf_mode="fixed")},
    {"key": "C_adaptive", "tiny": "C", "label": "移動様態適応PF", "short": "C\n移動様態適応PF", "section": "7.3",
     "kind": "main", "args": condition_args()},
    {"key": "E_proposed", "tiny": "E", "label": "提案方式", "short": "E\n提案方式", "section": "7.4",
     "kind": "main", "args": condition_args(**PROPOSED)},
    {"key": "E_mh", "tiny": "E+\n仮説", "label": "提案方式+複数経路仮説", "short": "E+複数\n経路仮説", "section": "",
     "kind": "variant", "args": condition_args(**PROPOSED, mh=True)},
    {"key": "E_mh_bl", "tiny": "E+仮説\n+尤度", "label": "提案方式+複数経路仮説+分岐選別尤度(σ=90度)",
     "short": "E+複数経路\n+選別尤度", "section": "", "kind": "variant",
     "args": condition_args(**PROPOSED, mh=True, branch_sigma=90.0)},
    {"key": "E_rooms", "tiny": "E+\n部屋", "label": "提案方式+広い部屋の除外", "short": "E+部屋\nの除外", "section": "",
     "kind": "variant", "args": condition_args(**PROPOSED, rooms=True)},
    {"key": "E_centerline", "tiny": "E+\n中心線", "label": "提案方式+中心線", "short": "E+中心線", "section": "",
     "kind": "variant", "args": condition_args(**PROPOSED, centerline=True)},
]
METHOD_KEYS = [m["key"] for m in METHODS]
METRICS = [("rmse_m", "RMSE"), ("mean_error_m", "平均誤差"), ("max_error_m", "最大誤差")]

# 図の色。軌跡図は3色まで(全組合せで色覚多様性の検証を通る範囲)にし、方式Aは
# 灰色の破線で区別する。箱ひげ図は方式を軸ラベルで示し、色は「4方式/変種」の区別だけ。
INK, INK_MUTED, WALL = "#0b0b0b", "#52514e", "#d6d5cf"
MAIN_FACE, MAIN_EDGE = "#cde2fb", "#2a78d6"
VARIANT_FACE, VARIANT_EDGE = "#f0efec", "#52514e"
TRAJ_STYLE = [  # 描く順(下から)。方式キー, 色, 線種, 凡例
    ("A_pdr", INK_MUTED, "--", "A: PDRのみ"),
    ("B_fixed", "#eb6834", "-", "B: 固定粒子数PF"),
    ("C_adaptive", "#1baf7a", "-", "C: 移動様態適応PF"),
    ("E_proposed", "#2a78d6", "-", "E: 提案方式"),
]


def wrap_deg(angle):
    """角度[度]を(-180, 180]へ。西向きは-180ではなく180と表す。"""
    wrapped = (angle + 180.0) % 360.0 - 180.0
    return 180.0 if wrapped == -180.0 else wrapped


def landmark_points(landmarks_df):
    return landmarks_df.sort_values("seq")[["x_px", "y_px"]].to_numpy(float)


def start_heading_from_route(points, scale_px_per_m):
    """目印1番→2番の向き[度]と、walking方式の基準区間内にある曲がり角までの距離[m]
    (無ければNone)を返す。向きは地図画像の座標(右=0、下=90、左=180、上=-90)。"""
    first = points[1] - points[0]
    heading = math.degrees(math.atan2(first[1], first[0]))
    limit = WALKING_CALIBRATION_DIST_M * scale_px_per_m
    travelled = 0.0
    for a, b in zip(points[:-1], points[1:]):
        if travelled >= limit:
            break
        seg = b - a
        if abs(wrap_deg(math.degrees(math.atan2(seg[1], seg[0])) - heading)) > EARLY_TURN_TOL_DEG:
            return heading, travelled / scale_px_per_m
        travelled += float(np.hypot(*seg))
    return heading, None


def press_interval_fit(waypoints_df, points):
    """地点マークを押した時刻の間隔と、目印間の距離の相関係数。
    点数が合わない・計算できない場合はNone。歩く速さがおおむね一定なら1に近くなる。"""
    times = waypoints_df.sort_values("seq")["timestamp"].to_numpy(float)
    if len(times) != len(points) or len(times) < 4:
        return None
    dt = np.diff(times)
    dist = np.hypot(*np.diff(points, axis=0).T)
    if np.std(dt) == 0 or np.std(dist) == 0:
        return None
    return float(np.corrcoef(dt, dist)[0, 1])


def check_route_name(waypoints_df, route, landmarks_dir, prefix):
    """経路名の書き間違いを疑うべきかを、同じ点数の他の経路と比べて判定する(警告のみ)。"""
    fits = {}
    head = f"{prefix}_landmarks_"
    for path in sorted(Path(landmarks_dir).glob(f"{head}*.csv")):
        name = path.stem[len(head):]
        try:
            r = press_interval_fit(waypoints_df, landmark_points(load_landmarks(path)))
        except (ValueError, KeyError):
            continue  # master表など、経路別の形式でないファイル
        if r is not None:
            fits[name] = r
    declared = fits.get(route)
    others = {k: v for k, v in fits.items() if k != route}
    best = max(others.items(), key=lambda kv: kv[1]) if others else (None, None)
    warning = None
    if declared is None:
        warning = f"押した回数が経路 {route} の目印数と合わない"
    elif best[1] is not None and best[1] >= declared + ROUTE_FIT_MARGIN:
        warning = (f"経路名の書き間違いの可能性: {best[0]} の方が押した間隔によく合う"
                   f"(r={best[1]:.2f}、{route}はr={declared:.2f})")
    elif declared < ROUTE_FIT_MIN_R:
        warning = (f"押した間隔と経路 {route} の目印間の距離が合わない(r={declared:.2f})。"
                   "押す順番の取り違えや、途中で立ち止まった可能性")
    return {"declared_r": declared, "best_other_route": best[0], "best_other_r": best[1],
            "warning": warning}


@contextlib.contextmanager
def start_position_file(program_dir):
    """本体の開始位置の読み書き関数が、実行する本体と同じ start_positions.csv を使うようにする。"""
    saved = pdrmod.START_POSITION_FILE
    pdrmod.START_POSITION_FILE = Path(program_dir) / "start_positions.csv"
    try:
        yield
    finally:
        pdrmod.START_POSITION_FILE = saved


def prepare_items(rows, data_dir, landmarks_dir, prefix, pf_map, scale, excluded):
    """一覧表のevalの行から、実行に必要な情報(開始位置・初期方位・正解位置)を決める。
    誤りは全件まとめてValueErrorにする。ここでは何も書き込まない。"""
    registered = pdrmod.load_start_positions()
    items, errors = [], []
    for _, row in rows.iterrows():
        path = resolve_csv_path(row["file"], data_dir)
        where = row["file"]
        item = {"file": row["file"], "name": path.name, "stem": path.stem, "path": path,
                "route": row["route"] or None, "warnings": [], "memo": row["memo"],
                "ground_truth_df": None, "route_check": None}
        if not path.exists():
            errors.append(f"{where}: ファイルが見つからない ({path})")
            continue
        if not pdrmod.is_sensor_log_csv(path):
            errors.append(f"{where}: センサーログの名前ではない(pdr_log_*.csv、_waypoints等は不可)")
            continue
        if path.name in excluded:
            errors.append(f"{where}: 設定JSONの exclude_csv に入っているため本体が処理しない")
            continue

        points = landmarks = None
        if item["route"]:
            lm_path = Path(landmarks_dir) / f"{prefix}_landmarks_{item['route']}.csv"
            if not lm_path.exists():
                errors.append(f"{where}: 経路 {item['route']} の目印表が無い ({lm_path})。"
                              "tools/make_route_landmarks.py で作る")
                continue
            landmarks = load_landmarks(lm_path)
            points = landmark_points(landmarks)
            item["landmarks_path"] = lm_path
            if len(points) < 2:
                errors.append(f"{where}: 経路 {item['route']} の目印が2点未満で向きを決められない")
                continue

        # 初期方位: 一覧表に書いてあればそれ、無ければ経路の目印1→2の向き
        if not math.isnan(row["start_heading_deg"]):
            heading = float(row["start_heading_deg"])
            item["heading_from"] = "一覧表の start_heading_deg"
            if points is not None:
                route_heading, _ = start_heading_from_route(points, scale)
                if abs(wrap_deg(route_heading - heading)) > EARLY_TURN_TOL_DEG:
                    item["warnings"].append(
                        f"start_heading_deg({heading:g}度)が経路の向き({route_heading:.0f}度)と"
                        "違う。一覧表の値を使う")
        else:
            heading, early_turn_m = start_heading_from_route(points, scale)
            item["heading_from"] = "経路の目印1→2の向き"
            if early_turn_m is not None:
                item["warnings"].append(
                    f"スタートから約{early_turn_m:.1f}mで曲がる経路。walking方式の基準方位"
                    f"(最初の10歩)に曲がりが混ざるおそれがある")
        item["heading_deg"] = round(wrap_deg(heading), 1) + 0.0  # -0.0 を 0.0 にそろえる

        # 開始位置: 登録済みならそれ、無ければ目印1番(後で登録する)
        registered_xy = registered.get(path.name)
        item["register"] = False
        if points is not None:
            landmark_xy = (float(points[0][0]), float(points[0][1]))
            if registered_xy is None:
                item["start"], item["register"] = landmark_xy, True
                item["start_from"] = "経路の目印1番(start_positions.csvへ登録)"
            else:
                item["start"] = registered_xy
                item["start_from"] = "start_positions.csvの登録済みの値"
                gap = math.hypot(registered_xy[0] - landmark_xy[0],
                                 registered_xy[1] - landmark_xy[1])
                if gap > START_MISMATCH_WARN_PX:
                    item["warnings"].append(
                        f"登録済みの開始位置が目印1番から{gap / scale:.1f}mずれている。"
                        "登録済みの値を使う")
        elif registered_xy is None:
            errors.append(f"{where}: route が空欄なのに start_positions.csv に開始位置が無い")
            continue
        else:
            item["start"], item["start_from"] = registered_xy, "start_positions.csvの登録済みの値"

        px, py = int(round(item["start"][0])), int(round(item["start"][1]))
        inside = 0 <= py < pf_map.shape[0] and 0 <= px < pf_map.shape[1]
        if not inside or pf_map[py, px] != 255:
            errors.append(f"{where}: 開始位置({px}, {py})が地図の通行可能領域の外。"
                          "本体が開始位置のクリック待ちで止まるので実行しない")
            continue

        # 正解位置(地点マーク × 目印表)
        wp_path = path.with_name(path.stem + "_waypoints.csv")
        if landmarks is None:
            item["gt_status"] = "経路が未定義のため正解位置なし(RMSEの対象外)"
        elif not wp_path.exists():
            item["gt_status"] = f"{wp_path.name} が無いため正解位置なし(RMSEの対象外)"
        else:
            try:
                waypoints = load_waypoints(wp_path)
                item["route_check"] = check_route_name(waypoints, item["route"],
                                                       landmarks_dir, prefix)
                if item["route_check"]["warning"]:
                    item["warnings"].append(item["route_check"]["warning"])
                item["ground_truth_df"] = build_ground_truth(waypoints, landmarks)
                item["gt_status"] = f"正解位置 {len(item['ground_truth_df'])}点"
            except (ValueError, KeyError) as error:
                item["gt_status"] = f"正解位置を作れない(RMSEの対象外): {error}"
                item["warnings"].append(item["gt_status"])
        items.append(item)

    if errors:
        raise ValueError("評価に使えない行があります(使わない場合は一覧表で use=0):\n  "
                         + "\n  ".join(errors))
    return items


def run_program(items, methods, seeds, ctx):
    """本体を「方式 × シード × 初期方位のグループ」で実行し、実行記録とログの診断値を返す。"""
    pf_methods = [m for m in methods if m["args"] is not None]
    groups = {}
    for item in items:
        groups.setdefault(item["heading_deg"], []).append(item)
    env = dict(os.environ, PYTHONIOENCODING="utf-8")
    runs, diag = [], {}
    total = len(groups) * len(pf_methods) * len(ctx["seeds"])
    count = 0
    with tempfile.TemporaryDirectory(prefix="pdr_eval_") as td:
        for heading, members in groups.items():
            group = f"h{heading:g}"
            group_dir = Path(td) / group
            group_dir.mkdir()
            for item in members:
                item["group"] = group
                shutil.copy2(item["path"], group_dir / item["name"])  # 元のCSVは読むだけ
            for method in pf_methods:
                for seed in seeds:
                    count += 1
                    run_dir = ctx["out_dir"] / "runs" / method["key"] / f"seed-{seed}"
                    run_dir.mkdir(parents=True, exist_ok=True)
                    options = [
                        "--map-config", str(ctx["map_config"]), "--no-watch", "--no-show",
                        "--seed", str(seed),
                        "--heading-source", ctx["heading_source"],
                        "--heading-calibration-mode", ctx["calibration_mode"],
                        "--initial-heading-deg", f"{heading:g}",
                        "--save", str(run_dir / f"{group}.png"),
                        "--save-trajectory-csv", "--save-pdr-trajectory-csv",
                        "--trajectory-dir", str(run_dir),
                        *method["args"], *ctx["extra_args"],
                    ]
                    cmd = [ctx["python"], str(ctx["program_dir"] / "pdr_pf_improved.py"),
                           "--data-dir", str(group_dir), *options]
                    t0 = time.time()
                    try:
                        res = subprocess.run(cmd, cwd=ctx["program_dir"], capture_output=True,
                                             env=env, timeout=RUN_TIMEOUT_SEC)
                        code = res.returncode
                        log = (res.stdout + res.stderr).decode("utf-8", "replace")
                    except subprocess.TimeoutExpired as error:
                        code = "timeout"
                        log = ((error.stdout or b"") + (error.stderr or b"")).decode(
                            "utf-8", "replace")
                    seconds = time.time() - t0
                    log_path = run_dir / f"{group}.log"
                    log_path.write_text(log, encoding="utf-8")
                    for record in parse_log(log):
                        diag[(method["key"], seed, record["file"])] = record
                    runs.append({"method": method["key"], "seed": seed, "group": group,
                                 "files": [i["name"] for i in members], "returncode": code,
                                 "seconds": round(seconds, 1), "log": str(log_path),
                                 "options": ["--data-dir", f"<{group}のCSVのコピー>", *options]})
                    status = "OK" if code == 0 else f"失敗({code})"
                    print(f"  [{count}/{total}] {method['key']} seed={seed} 初期方位{heading:g}度 "
                          f"({len(members)}本) {status} {seconds:.1f}秒", flush=True)
    return runs, diag


def collect_results(items, methods, seeds, diag, ctx):
    """各実行の軌跡CSVを正解位置と突き合わせ、1実行・1ファイル1行の表にする。"""
    rows, notes = [], []
    pf_methods = [m for m in methods if m["args"] is not None]
    want_pdr = any(m["key"] == "A_pdr" for m in methods)
    info = {m["key"]: m for m in METHODS}
    pdr_name = "{stem}_pdr_traj_" + f"{ctx['heading_source']}-{ctx['calibration_mode']}.csv"

    def base_row(key, item, seed):
        return {"method_key": key, "method": info[key]["label"], "section": info[key]["section"],
                "kind": info[key]["kind"], "file": item["name"], "route": item["route"] or "",
                "group": item["group"], "seed": seed,
                "has_ground_truth": item.get("ground_truth") is not None}

    def add_metrics(row, trajectory, item):
        row["trajectory"] = str(trajectory)
        if item.get("ground_truth") is None:
            return
        try:
            # 地点マーク1番は歩き始める前に押すので、推定軌跡の時刻範囲の外として毎回除外
            # される(想定どおり)。1件ごとの警告は出さず、除外数は表に残して最後にまとめる。
            logging.disable(logging.WARNING)
            summary = evaluate(trajectory, item["ground_truth"], scale_px_per_m=ctx["scale"])
        except ValueError as error:
            row["error"] = str(error)
            notes.append(f"{row['file']} {row['method_key']} seed={row['seed']}: {error}")
            return
        finally:
            logging.disable(logging.NOTSET)
        row.update({k: summary[k] for k in (
            "n_points", "excluded_points", "rmse_m", "mean_error_m", "max_error_m",
            "rmse_px", "mean_error_px", "max_error_px")})

    for item in items:
        pdr_digest = None
        for method in pf_methods:
            for seed in seeds:
                run_dir = ctx["out_dir"] / "runs" / method["key"] / f"seed-{seed}"
                row = base_row(method["key"], item, seed)
                d = diag.get((method["key"], seed, item["name"]), {})
                row.update({"steps": d.get("steps"), "extinctions": d.get("extinctions"),
                            "final_x": d.get("final_x"), "final_y": d.get("final_y")})
                found = sorted(run_dir.glob(f"{item['stem']}_traj_*_seed-{seed}.csv"))
                if len(found) != 1:
                    row["error"] = "推定軌跡CSVが無い(実行の失敗か、歩数0でスキップ)"
                    notes.append(f"{item['name']} {method['key']} seed={seed}: {row['error']}")
                else:
                    add_metrics(row, found[0], item)
                rows.append(row)
                pdr = run_dir / pdr_name.format(stem=item["stem"])
                if pdr.exists():
                    digest = hashlib.md5(pdr.read_bytes()).hexdigest()
                    if pdr_digest is None:
                        pdr_digest = digest
                        pdr_copy = ctx["out_dir"] / "runs" / "A_pdr" / pdr.name
                        pdr_copy.parent.mkdir(parents=True, exist_ok=True)
                        shutil.copy2(pdr, pdr_copy)
                    elif digest != pdr_digest:
                        notes.append(f"{item['name']}: PDRのみの軌跡が実行によって違う"
                                     f"({method['key']} seed={seed})。乱数を使わないはずなので要確認")
        if want_pdr:
            row = base_row("A_pdr", item, np.nan)
            if pdr_digest is None:
                row["error"] = "PDRのみの軌跡CSVが無い"
                notes.append(f"{item['name']} A_pdr: {row['error']}")
            else:
                pdr_path = ctx["out_dir"] / "runs" / "A_pdr" / pdr_name.format(stem=item["stem"])
                trajectory = pd.read_csv(pdr_path)
                row.update({"steps": len(trajectory), "final_x": trajectory["x_px"].iloc[-1],
                            "final_y": trajectory["y_px"].iloc[-1]})
                add_metrics(row, pdr_path, item)
            rows.append(row)

    df = pd.DataFrame(rows)
    df["method_key"] = pd.Categorical(df["method_key"], categories=METHOD_KEYS, ordered=True)
    for column in [c for c, _ in METRICS] + ["trajectory", "excluded_points"]:
        if column not in df.columns:
            df[column] = np.nan
    if df["excluded_points"].fillna(0).sum() > 0:
        per_file = df.groupby("file")["excluded_points"].max().dropna()
        notes.append("推定軌跡の時刻範囲外で評価から外した正解点(ファイルごとの最大): "
                     + ", ".join(f"{f} {int(n)}点" for f, n in per_file.items())
                     + "。地点マーク1番は歩き始める前に押すので、1点は外れるのが正常")
    return df.sort_values(["method_key", "file", "seed"]).reset_index(drop=True), notes


def format_mean_std(mean, std, digits=2):
    if not np.isfinite(mean):
        return ""
    return f"{mean:.{digits}f} ± {std:.{digits}f}" if np.isfinite(std) else f"{mean:.{digits}f}"


def summarize_diagnostics(df):
    """正解位置の有無によらず出せる診断値(全滅回数・最終位置)を方式・ファイルごとにまとめる。
    精度の指標ではない。方式Aは粒子が無いので全滅回数は空欄。"""
    grouped = df.groupby(["method_key", "file"], observed=True, sort=True)
    out = grouped.size().rename("n").to_frame()
    out.insert(0, "方式", grouped["method"].first())
    for column, label, digits in (("extinctions", "全滅回数", 1), ("final_x", "最終位置x[px]", 1),
                                  ("final_y", "最終位置y[px]", 1)):
        values = pd.to_numeric(df[column], errors="coerce").groupby(
            [df["method_key"], df["file"]], observed=True, sort=True)
        mean, std = values.mean(), values.std(ddof=1)
        out[f"{label} 平均±標準偏差"] = [format_mean_std(a, b, digits) for a, b in zip(mean, std)]
        out[f"{column}_mean"], out[f"{column}_std"] = mean, std
    return out.reset_index()


def summarize(df, keys):
    """RMSE・平均誤差・最大誤差の平均±標準偏差(標本標準偏差)。"""
    scored = df[df["has_ground_truth"] & df["rmse_m"].notna()]
    grouped = scored.groupby(keys, observed=True, sort=True)
    out = grouped.size().rename("n").to_frame()
    first = grouped.first()
    out.insert(0, "方式", first["method"])
    out.insert(1, "卒論", first["section"])
    out.insert(2, "区分", first["kind"].map({"main": "比較する4方式", "variant": "提案方式の変種"}))
    for column, label in METRICS:
        mean, std = grouped[column].mean(), grouped[column].std(ddof=1)
        out[f"{label}[m] 平均±標準偏差"] = [format_mean_std(a, b) for a, b in zip(mean, std)]
        out[f"{column}_mean"], out[f"{column}_std"] = mean, std
    return out.reset_index()


def style_axes(ax):
    ax.grid(True, axis="y", color="#e6e5e0", linewidth=0.6)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(INK_MUTED)
    ax.tick_params(colors=INK_MUTED)


def draw_boxes(ax, df, methods, label_key="short"):
    present = [m for m in methods if df.loc[df["method_key"] == m["key"], "rmse_m"].notna().any()]
    data = [df.loc[df["method_key"] == m["key"], "rmse_m"].dropna().to_numpy() for m in present]
    boxes = ax.boxplot(data, patch_artist=True, widths=0.55, showfliers=False,
                       medianprops={"color": INK, "linewidth": 1.6},
                       whiskerprops={"color": INK_MUTED}, capprops={"color": INK_MUTED})
    for patch, method in zip(boxes["boxes"], present):
        main = method["kind"] == "main"
        patch.set_facecolor(MAIN_FACE if main else VARIANT_FACE)
        patch.set_edgecolor(MAIN_EDGE if main else VARIANT_EDGE)
    rng = np.random.default_rng(0)
    for i, values in enumerate(data, start=1):
        ax.scatter(i + rng.uniform(-0.12, 0.12, len(values)), values, s=14, color=INK_MUTED,
                   alpha=0.75, linewidths=0, zorder=3)
    ax.set_xticks(range(1, len(present) + 1), [m[label_key] for m in present], fontsize=8)
    ax.set_ylim(bottom=0)
    style_axes(ax)


def plot_boxplots(df, methods, out_dir, title_prefix):
    scored = df[df["has_ground_truth"]]
    legend = [Patch(facecolor=MAIN_FACE, edgecolor=MAIN_EDGE, label="比較する4方式"),
              Patch(facecolor=VARIANT_FACE, edgecolor=VARIANT_EDGE, label="提案方式の変種"),
              Line2D([], [], linestyle="none", marker="o", markersize=5, color=INK_MUTED,
                     label="1ファイル×1シードの値")]
    fig, ax = plt.subplots(figsize=(max(6.5, 1.1 * len(methods) + 1.5), 4.8))
    draw_boxes(ax, scored, methods)
    ax.set_ylabel("RMSE [m]")
    n_files = scored["file"].nunique()
    ax.set_title(f"{title_prefix}方式別のRMSE(正解位置のある{n_files}本 × シード)", color=INK)
    fig.legend(handles=legend, loc="lower center", ncol=3, frameon=False, fontsize=8,
               bbox_to_anchor=(0.5, -0.02))
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    fig.savefig(out_dir / "boxplot_rmse.png", dpi=200, bbox_inches="tight")
    plt.close(fig)

    files = sorted(scored["file"].unique())
    if len(files) < 2:
        return
    ncols = min(3, len(files))
    nrows = math.ceil(len(files) / ncols)
    fig, axes = plt.subplots(nrows, ncols, figsize=(5.2 * ncols, 4.2 * nrows), sharey=True,
                             squeeze=False)
    for ax, name in zip(axes.flat, files):
        # 1枚が狭いので短い略号で示し、略号の意味は図の下に書く
        draw_boxes(ax, scored[scored["file"] == name], methods, label_key="tiny")
        route = scored.loc[scored["file"] == name, "route"].iloc[0]
        ax.set_title(f"{name}({route})", fontsize=9, color=INK)
    for ax in list(axes.flat)[len(files):]:
        ax.set_visible(False)
    for ax in axes[:, 0]:
        ax.set_ylabel("RMSE [m]")
    fig.suptitle(f"{title_prefix}ファイル別のRMSE(シード間のばらつき)", color=INK)
    fig.legend(handles=legend, loc="lower center", ncol=3, frameon=False, fontsize=8,
               bbox_to_anchor=(0.5, 0.02))
    fig.text(0.5, -0.01, "A: PDRのみ / B: 固定粒子数PF / C: 移動様態適応PF / E: 提案方式 / "
             "仮説: 複数経路仮説 / 尤度: 分岐選別尤度 / 部屋: 広い部屋の除外",
             ha="center", fontsize=8, color=INK_MUTED)
    fig.tight_layout(rect=(0, 0.08, 1, 1))
    fig.savefig(out_dir / "boxplot_rmse_by_file.png", dpi=200, bbox_inches="tight")
    plt.close(fig)


def plot_trajectories(item, df, binary, seed, out_dir, title_prefix):
    """1本のCSVについて、4方式の軌跡と正解位置を地図に重ねる(同じシード)。"""
    fig, ax = plt.subplots(figsize=(11, 5.2))
    ax.imshow(np.where(binary == 255, 1, 0), cmap=ListedColormap([WALL, "#ffffff"]),
              interpolation="nearest", vmin=0, vmax=1)
    handles = []
    for key, color, linestyle, label in TRAJ_STYLE:
        match = df[(df["method_key"] == key) & (df["file"] == item["name"])
                   & ((df["seed"] == seed) | df["seed"].isna())]
        if match.empty or not isinstance(match["trajectory"].iloc[0], str):
            continue
        t = pd.read_csv(match["trajectory"].iloc[0])
        xs = np.concatenate([[item["start"][0]], t["x_px"]])
        ys = np.concatenate([[item["start"][1]], t["y_px"]])
        ax.plot(xs, ys, color=color, linestyle=linestyle, linewidth=1.8, zorder=3)
        handles.append(Line2D([], [], color=color, linestyle=linestyle, linewidth=1.8,
                              label=label))
    ax.scatter(*item["start"], marker="s", s=60, color=INK, edgecolors="white", linewidths=1,
               zorder=5)
    handles.append(Line2D([], [], linestyle="none", marker="s", markersize=7, color=INK,
                          markeredgecolor="white", label="開始位置"))
    if item.get("ground_truth") is not None:
        gt = pd.read_csv(item["ground_truth"])
        ax.scatter(gt["x_px"], gt["y_px"], s=36, color="white", edgecolors=INK, linewidths=1.3,
                   zorder=6)
        for _, p in gt.iterrows():
            ax.annotate(str(int(p["seq"])), (p["x_px"], p["y_px"]), xytext=(0, -11),
                        textcoords="offset points", ha="center", fontsize=7, color=INK)
        handles.append(Line2D([], [], linestyle="none", marker="o", markersize=6,
                              markerfacecolor="white", markeredgecolor=INK,
                              label="正解位置(地点マークの番号)"))
    ax.set_xlim(0, binary.shape[1])
    ax.set_ylim(binary.shape[0], 0)
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(False)
    route = item["route"] or "経路未定義"
    ax.set_title(f"{title_prefix}{item['name']}({route}、seed={seed})", color=INK)
    ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.01), ncol=3,
              frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(out_dir / f"trajectory_{item['stem']}.png", dpi=200, bbox_inches="tight")
    plt.close(fig)


def _json_default(value):
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return None if not np.isfinite(value) else float(value)
    if isinstance(value, Path):
        return str(value)
    raise TypeError(type(value))


def run_evaluation(list_path, data_dir, map_config, out_root=RESULTS_DIR, tag=None,
                   seeds=DEFAULT_SEEDS, method_keys=None, heading_source="android",
                   calibration_mode="walking", gain=None, program_dir=PROGRAM_DIR,
                   landmarks_dir=PROGRAM_DIR / "ground_truth", synthetic=False,
                   python=sys.executable):
    """一覧表のevalの行を評価して、表・図・実行条件を保存し、要約を返す。"""
    methods = [m for m in METHODS if method_keys is None or m["key"] in method_keys]
    if not any(m["args"] is not None for m in methods):
        raise ValueError("方式AはPFの実行から作るので、B以降の方式を1つ以上選んでください。")
    rows = load_measurement_list(list_path, "eval")
    if rows.empty:
        raise ValueError(f"{list_path}: purpose=eval かつ use=1 の行がありません。")

    map_config = Path(map_config).resolve()
    program_dir = Path(program_dir).resolve()
    _cfg, resolved = pdrmod.load_map_config_for_tool(map_config)
    binary, pf_map, _dist = pdrmod.load_preprocessed_map(resolved.map)
    scale = float(pdrmod.M_TO_PIXEL)
    config_gain = float(pdrmod.STEP_LENGTH_CALIBRATION_GAIN)
    with start_position_file(program_dir):
        items = prepare_items(rows, data_dir, landmarks_dir, map_config.stem, pf_map, scale,
                              pdrmod.EXCLUDED_CSV_NAMES)
        for item in items:  # 全行の検査が通ってから登録する(途中で止まって半端に書かない)
            if item["register"]:
                pdrmod.save_start_position(item["name"], *item["start"])

    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_dir = Path(out_root) / (f"{stamp}_evaluation" + (f"_{tag}" if tag else "")
                                + ("_SYNTHETIC" if synthetic else ""))
    (out_dir / "ground_truth").mkdir(parents=True, exist_ok=True)
    for item in items:
        item["ground_truth"] = None
        if item["ground_truth_df"] is not None:
            item["ground_truth"] = out_dir / "ground_truth" / f"{item['stem']}_ground_truth.csv"
            item["ground_truth_df"].to_csv(item["ground_truth"], index=False)
    shutil.copy2(map_config, out_dir / "used_map_config.json")
    shutil.copy2(list_path, out_dir / "measurement_list.csv")

    ctx = {"out_dir": out_dir, "map_config": map_config, "program_dir": program_dir,
           "python": python, "seeds": seeds, "heading_source": heading_source,
           "calibration_mode": calibration_mode, "scale": scale,
           "extra_args": ["--step-length-calibration-gain", f"{gain:g}"] if gain else []}
    title_prefix = "【架空データ・研究結果ではない】" if synthetic else ""
    print(f"評価対象 {len(items)}本 × 方式{len(methods)} × シード{len(seeds)} "
          f"(出力: {out_dir})")
    for item in items:
        print(f"  {item['name']}: 経路={item['route'] or '未定義'} 開始位置=({item['start'][0]:.1f}, "
              f"{item['start'][1]:.1f}) 初期方位={item['heading_deg']:g}度 / {item['gt_status']}")
        for warning in item["warnings"]:
            print(f"    [警告] {warning}")
    runs, diag = run_program(items, methods, seeds, ctx)
    df, notes = collect_results(items, methods, seeds, diag, ctx)
    if synthetic:
        df.insert(0, "注意", "架空データ(研究結果ではない)")
    df.to_csv(out_dir / "results_long.csv", index=False, encoding="utf-8-sig")

    tables = {"table_diagnostics": summarize_diagnostics(df)}
    if synthetic:
        tables["table_diagnostics"].insert(0, "注意", "架空データ(研究結果ではない)")
    tables["table_diagnostics"].to_csv(out_dir / "table_diagnostics.csv", index=False,
                                       encoding="utf-8-sig", float_format="%.4f")
    if df["has_ground_truth"].any() and df["rmse_m"].notna().any():
        for name, keys in (("table_by_method", ["method_key"]),
                           ("table_by_method_file", ["method_key", "file"])):
            table = summarize(df, keys)
            if synthetic:
                table.insert(0, "注意", "架空データ(研究結果ではない)")
            table.to_csv(out_dir / f"{name}.csv", index=False, encoding="utf-8-sig",
                         float_format="%.4f")
            tables[name] = table
        plot_boxplots(df, methods, out_dir, title_prefix)
    else:
        notes.append("正解位置のある記録が無いので、RMSEの表と箱ひげ図は作っていない")
    figure_seed = 42 if 42 in seeds else seeds[0]
    for item in items:
        plot_trajectories(item, df, binary, figure_seed, out_dir, title_prefix)

    failures = [r for r in runs if r["returncode"] != 0]
    commit, dirty = git_revision()
    conditions = {
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "synthetic": synthetic,
        "git_commit": commit,
        "git_uncommitted_changes": dirty,
        "program": str(program_dir / "pdr_pf_improved.py"),
        "map_config": str(map_config),
        "measurement_list": str(Path(list_path).resolve()),
        "data_dir": str(data_dir),
        "seeds": list(seeds),
        "heading_source": heading_source,
        "heading_calibration_mode": calibration_mode,
        "step_length_calibration_gain": gain if gain else config_gain,
        "scale_px_per_m": scale,
        "table_definitions": {
            "table_by_method": "方式ごとに、正解位置のある全ファイル×全シードの値の平均±標本標準偏差",
            "table_by_method_file": "方式・ファイルごとに、シード間の平均±標本標準偏差(方式Aは1値)",
        },
        "methods": [{k: m[k] for k in ("key", "label", "section", "kind", "args")}
                    for m in methods],
        "files": [{"file": i["file"], "route": i["route"], "group": i["group"],
                   "start_xy": list(i["start"]), "start_from": i["start_from"],
                   "start_heading_deg": i["heading_deg"], "heading_from": i["heading_from"],
                   "ground_truth": i["gt_status"], "route_check": i["route_check"],
                   "warnings": i["warnings"], "memo": i["memo"]} for i in items],
        "failed_runs": failures,
        "notes": notes,
        "runs": runs,
    }
    (out_dir / "conditions.json").write_text(
        json.dumps(conditions, ensure_ascii=False, indent=2, default=_json_default),
        encoding="utf-8")
    return {"out_dir": out_dir, "df": df, "tables": tables, "items": items,
            "failures": failures, "notes": notes}


def print_report(result):
    table = result["tables"].get("table_by_method")
    if table is not None:
        print("\n=== 方式別(正解位置のある全ファイル×全シード) ===")
        view = table[["方式", "卒論", "n", "RMSE[m] 平均±標準偏差", "平均誤差[m] 平均±標準偏差",
                      "最大誤差[m] 平均±標準偏差"]]
        print(view.to_string(index=False))
    for note in result["notes"]:
        print(f"[注意] {note}")
    if result["failures"]:
        print(f"\n[失敗] 本体の実行が {len(result['failures'])} 回失敗した。"
              "conditions.json の failed_runs とログを確認すること。")
    print(f"\n保存先: {result['out_dir']}")


def _write_synthetic_walk(path, n_steps, yaw_deg, rng, hz=50.0, cadence=1.8, amplitude=3.0):
    """架空の直線歩行(前に3秒・後に2秒の静止)。yaw_degは一定。数値は研究結果ではない。"""
    still = 3.0
    walk = n_steps / cadence
    t = np.arange(0.0, still + walk + 2.0, 1.0 / hz)
    walking = (t >= still) & (t < still + walk)
    wave = amplitude * (np.sin(2 * np.pi * cadence * (t - still) - np.pi / 2) + 1.0) / 2
    pd.DataFrame({
        "timestamp": 1000.0 + t,
        "acc_x": rng.normal(0, 0.02, len(t)), "acc_y": rng.normal(0, 0.02, len(t)),
        "acc_z": 9.81 + np.where(walking, wave, 0.0) + rng.normal(0, 0.02, len(t)),
        "gyro_x": np.zeros(len(t)), "gyro_y": np.zeros(len(t)), "gyro_z": np.zeros(len(t)),
        "yaw_deg": np.full(len(t), yaw_deg),
    }).to_csv(path, index=False)


def _synthetic_truth(path):
    """本体の関数で、架空の歩行の歩の時刻と、各歩の後の累積距離[px]を求める。"""
    df = pdrmod.validate_log(pdrmod.safe_read_csv(path), path.name)
    df["step_acc"] = pdrmod.compute_step_acceleration(pdrmod.compute_acc_magnitude(df))
    hz = 1.0 / df["timestamp"].diff().mean()
    steps, valleys = pdrmod.detect_steps_smartpdr(df["step_acc"], hz)
    lengths = [pdrmod.estimate_smartpdr_step_length_px(df["step_acc"], p, v)
               for p, v in zip(steps, valleys)]
    return df["timestamp"].to_numpy(float)[steps], np.cumsum(lengths)


def _self_test(keep_dir=None):
    """[本研究独自] 架空データで本体を実際に動かす通し試験。本体のコピーを一時フォルダで
    動かすので、本物の start_positions.csv と results/ には書き込まない(最後に確認する)。

    【重要】ここで使う信号・正解位置はすべて架空であり、出てくる数値を研究結果として
    扱わない。確認するのは「時刻の突き合わせ・初期方位・開始位置の登録・表と図の作成が
    正しく動くか」だけである。
    """
    print("--- self-test 開始(架空データで本体を実際に動かす。数値は研究結果ではない) ---")
    real_start_file = PROGRAM_DIR / "start_positions.csv"
    start_before = real_start_file.read_bytes() if real_start_file.exists() else None
    results_before = sorted(p.name for p in RESULTS_DIR.iterdir()) if RESULTS_DIR.exists() else []
    rng = np.random.default_rng(0)
    with tempfile.TemporaryDirectory(prefix="pdr_eval_selftest_") as td:
        td = Path(td)
        prog, data, lm_dir = td / "prog", td / "data", td / "landmarks"
        for folder in (prog / "map_configs", data, lm_dir):
            folder.mkdir(parents=True)
        for name in ("pdr_pf_improved.py", "pdr_route_graph.py"):
            shutil.copy2(PROGRAM_DIR / name, prog / name)
        config = json.loads((PROGRAM_DIR / "map_configs" / "kanri_4f.json").read_text(encoding="utf-8"))
        config["map_image"] = str((PROGRAM_DIR / "kanri_4f_binary_final3.png").resolve())
        config["data_dir"] = str(data)
        config_path = prog / "map_configs" / "kanri_4f.json"
        config_path.write_text(json.dumps(config, ensure_ascii=False), encoding="utf-8")
        pdrmod.load_map_config_for_tool(config_path)

        # 下側廊下(y=230)を25歩まっすぐ歩く架空の記録。東向き2本(うち1本は地点マーク無し)と
        # 西向き1本。地点マークは歩の時刻に押したことにし、目印の座標は「本体の歩幅で
        # 進んだ位置」に置く。PDRのみの軌跡は正解と一致するはずなので、RMSEはほぼ0になる。
        marks = [2, 5, 11, 14, 22]  # 目印を置く歩の番号(間隔を不ぞろいにする)
        walks = [("pdr_log_9001_0001.csv", "selftest_east", 100.0, +1, True),
                 ("pdr_log_9001_0002.csv", "selftest_east", 100.0, +1, False),
                 ("pdr_log_9001_0003.csv", "selftest_west", 380.0, -1, True)]
        written_routes = set()
        for name, route, x0, direction, with_waypoints in walks:
            _write_synthetic_walk(data / name, 25, 37.0, rng)
            t_steps, cum = _synthetic_truth(data / name)
            assert len(t_steps) == 25, len(t_steps)
            if direction < 0:
                cum_west = cum  # 西向きの記録の、PDRのみの終点の確認に使う
            xs = [x0] + [x0 + direction * cum[j] for j in marks]
            if route not in written_routes:
                pd.DataFrame({"seq": range(1, len(xs) + 1), "label": [f"p{i}" for i in range(len(xs))],
                              "point_type": "wall", "x_px": xs, "y_px": 230.0}).to_csv(
                    lm_dir / f"kanri_4f_landmarks_{route}.csv", index=False)
                written_routes.add(route)
            if with_waypoints:
                times = [t_steps[0] - 0.5] + [t_steps[j] for j in marks]
                pd.DataFrame({"timestamp": times, "seq": range(1, len(times) + 1)}).to_csv(
                    data / f"{Path(name).stem}_waypoints.csv", index=False)
        # 点数が同じで間隔の並びが違う経路(書き間違いの検出の確認用)
        east = pd.read_csv(lm_dir / "kanri_4f_landmarks_selftest_east.csv")
        gaps = np.diff(east["x_px"].to_numpy())[::-1]
        decoy = east.copy()
        decoy["x_px"] = np.concatenate([[100.0], 100.0 + np.cumsum(gaps)])
        decoy.to_csv(lm_dir / "kanri_4f_landmarks_selftest_decoy.csv", index=False)

        list_path = td / "list.csv"
        list_path.write_text(
            "file,purpose,route,distance_m,speed,use,memo\n"
            "pdr_log_9001_0001.csv,eval,selftest_east,,,1,架空(東向き)\n"
            "pdr_log_9001_0002.csv,eval,selftest_east,,,1,架空(地点マーク無し)\n"
            "pdr_log_9001_0003.csv,eval,selftest_west,,,1,架空(西向き)\n"
            "calib/pdr_log_9001_0009.csv,calib,,30,slow,1,評価では読まない\n"
            "pdr_log_9001_0008.csv,eval,selftest_east,,,0,撮り直し(使わない)\n",
            encoding="utf-8")

        # 経路名の確認: 正しい経路は相関が高く、間隔を逆に並べた経路は警告になる
        waypoints = load_waypoints(data / "pdr_log_9001_0001_waypoints.csv")
        right = check_route_name(waypoints, "selftest_east", lm_dir, "kanri_4f")
        wrong = check_route_name(waypoints, "selftest_decoy", lm_dir, "kanri_4f")
        assert right["warning"] is None and right["declared_r"] > 0.9, right
        assert wrong["warning"] and wrong["best_other_r"] > 0.9, wrong
        print(f"  OK: 経路名の確認(正しい経路 r={right['declared_r']:.2f}、"
              f"取り違えた経路 r={wrong['declared_r']:.2f} は警告)")

        result = run_evaluation(list_path, data, config_path, out_root=td / "out",
                                tag="selftest", seeds=[1, 2], program_dir=prog,
                                landmarks_dir=lm_dir, synthetic=True)
        df, out_dir = result["df"], result["out_dir"]
        assert not result["failures"], result["failures"]
        assert not [n for n in result["notes"] if "PDRのみの軌跡が実行によって違う" in n], result["notes"]
        print("  OK: 本体を全方式×2シード×2方向で実行でき、PDRのみの軌跡は全実行で同一")

        registered = pd.read_csv(prog / "start_positions.csv").set_index("file_name")
        for name, _, x0, _, _ in walks:
            assert abs(registered.loc[name, "start_x"] - x0) < 1e-6, registered
        print("  OK: 未登録の開始位置を目印1番から登録(一時フォルダの start_positions.csv)")

        pdr = df[(df["method_key"] == "A_pdr") & df["has_ground_truth"]]
        assert len(pdr) == 2 and (pdr["rmse_m"] < 0.01).all(), pdr[["file", "rmse_m"]]
        print(f"  OK: PDRのみのRMSEが東向き・西向きとも約0m(最大{pdr['rmse_m'].max():.4f}m)"
              "= 時刻の突き合わせと初期方位(0度/180度)が正しい")

        table = result["tables"]["table_by_method"]
        assert list(table["method_key"]) == METHOD_KEYS, table["method_key"].tolist()
        expected_n = [2 if k == "A_pdr" else 4 for k in METHOD_KEYS]
        assert table["n"].tolist() == expected_n, table["n"].tolist()
        no_gt = df[df["file"] == "pdr_log_9001_0002.csv"]
        assert (~no_gt["has_ground_truth"]).all() and no_gt["trajectory"].notna().all()
        assert "pdr_log_9001_0002.csv" not in set(result["tables"]["table_by_method_file"]["file"])
        print("  OK: 正解位置の無い記録は軌跡だけ出して表から外す。全8方式が表にそろう")

        diagnostics = pd.read_csv(out_dir / "table_diagnostics.csv")
        assert len(diagnostics) == len(METHOD_KEYS) * 3, len(diagnostics)  # 8方式×3本
        pdr_final = df[(df["method_key"] == "A_pdr") & (df["file"] == "pdr_log_9001_0003.csv")]
        assert abs(pdr_final["final_x"].iloc[0] - (380.0 - cum_west[-1])) < 1e-6, pdr_final
        for name in ("table_by_method.csv", "table_by_method_file.csv", "results_long.csv",
                     "table_diagnostics.csv",
                     "boxplot_rmse.png", "boxplot_rmse_by_file.png", "conditions.json",
                     "used_map_config.json", "measurement_list.csv",
                     "trajectory_pdr_log_9001_0001.png", "trajectory_pdr_log_9001_0003.png"):
            assert (out_dir / name).exists(), name
        conditions = json.loads((out_dir / "conditions.json").read_text(encoding="utf-8"))
        assert conditions["synthetic"] is True and "架空" in table["注意"].iloc[0]
        print("  OK: 表・箱ひげ図・軌跡図・実行条件を保存し、架空データであることを明記")
        if keep_dir is not None:
            kept = Path(keep_dir) / out_dir.name
            shutil.copytree(out_dir, kept)
            print(f"  (確認用に出力を残した: {kept})")

    start_after = real_start_file.read_bytes() if real_start_file.exists() else None
    results_after = sorted(p.name for p in RESULTS_DIR.iterdir()) if RESULTS_DIR.exists() else []
    assert start_after == start_before, "本物の start_positions.csv が変わった"
    assert results_after == results_before, "本物の results/ にファイルが増えた"
    print("  OK: 本物の start_positions.csv と results/ は変わっていない")
    print("--- self-test 全て通過 ---")


def main():
    p = argparse.ArgumentParser(
        description="計測一覧表のevalの行について、卒論第7章の比較(方式別RMSE表・箱ひげ図・"
                    "軌跡図・実行条件)を作る。詳細はこのファイル冒頭のコメントを参照。")
    p.add_argument("--list", type=Path, help="計測一覧表(purpose=eval の行を使う)。")
    p.add_argument("--data-dir", type=Path, default=None,
                   help="CSVフォルダ。環境変数PDR_DATA_DIRとJSONのdata_dirより優先する。")
    p.add_argument("--map-config", type=Path,
                   default=PROGRAM_DIR / "map_configs" / "kanri_4f.json")
    p.add_argument("--seeds", type=int, nargs="+", default=DEFAULT_SEEDS)
    p.add_argument("--methods", nargs="+", choices=METHOD_KEYS, default=None,
                   help="実行する方式(既定は全部)。動作確認で絞るとき用。")
    p.add_argument("--heading-source", choices=["gyro", "android"], default="android",
                   help="全方式共通の方位源(既定android。memo/heading_calibration.md)。")
    p.add_argument("--heading-calibration-mode", choices=["samples", "walking"],
                   default="walking", help="全方式共通の初期方位校正(既定walking)。")
    p.add_argument("--step-length-calibration-gain", type=float, default=None,
                   help="全方式共通で歩幅校正ゲインを上書きする(既定はJSONの値)。")
    p.add_argument("--tag", default=None, help="出力フォルダ名に付ける目印(例: 0925)。")
    p.add_argument("--self-test", action="store_true",
                   help="架空データで本体を実際に動かす通し試験(実データ不要)。")
    p.add_argument("--self-test-keep", type=Path, default=None,
                   help="--self-test の出力(架空データ)を確認用にこのフォルダへ残す。"
                        "results/ は指定しないこと。")
    a = p.parse_args()

    if a.self_test:
        _self_test(a.self_test_keep)
        return
    if a.list is None:
        p.error("--list を指定するか、--self-test を使ってください。")

    _cfg, resolved = pdrmod.load_map_config_for_tool(a.map_config)
    data_dir = a.data_dir.expanduser().resolve() if a.data_dir else resolved.data_dir
    print(f"CSVフォルダ: {data_dir}")
    try:
        result = run_evaluation(a.list, data_dir, a.map_config, tag=a.tag, seeds=a.seeds,
                                method_keys=a.methods, heading_source=a.heading_source,
                                calibration_mode=a.heading_calibration_mode,
                                gain=a.step_length_calibration_gain)
    except ValueError as error:
        raise SystemExit(f"エラー: {error}")
    print_report(result)
    if result["failures"]:
        sys.exit(1)


if __name__ == "__main__":
    main()
