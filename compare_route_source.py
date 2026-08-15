# ============================================================================
# compare_route_source.py
#
# 【変更履歴】
# - 2026-08-15: 新規作成。route_constraint_mode/route_sourceの条件別に
#               pdr_pf_improved.pyを同一データ・同一乱数シードで実行し、
#               ログから診断値を抽出して比較表(CSV+標準出力)を作る評価ハーネス。
#
# 【本スクリプトの位置づけ】[本研究独自]
# 進捗反映版メモ.txt §18.3で「未完了」と指摘されている「比較実験モード」
# 「定量評価」の第一歩。pdr_pf_improved.py本体(PFアルゴリズム)には一切手を
# 加えず、既存のCLI引数(--route-constraint-mode, --route-source)をsubprocess
# 経由で切り替えて実行し、各実行が標準出力するログ(redraw_all_paths()内の
# logging.info呼び出し)を正規表現で解析するだけの外付けツールである。
#
# 比較する条件(CONDITIONS):
#   none          : route_constraint_mode=none        (地図の経路情報を使わない。
#                   移動様態適応PFのみ。先行研究に最も近いが、本研究独自の
#                   連続壁尤度dist_mapは使用したまま)
#   manual-enforce: route_constraint_mode=enforce, route_source=manual (方式D。
#                   手動route_pointsを経路帯として使用)
#   auto-enforce  : route_constraint_mode=enforce, route_source=auto   (本研究独自。
#                   二値地図から自動抽出した経路帯を使用、手動座標ゼロ)
#
# 出力する指標は、正解位置列(真の毎ステップの位置)を持たないため、真の意味の
# RMSEではない。現時点で正直に計算できるのは以下のみ:
#   - 全滅回数(全粒子の重みが0になり自己リカバリした回数。少ないほど安定)
#   - 有効粒子率・経路内率・Neff平均・位置分散平均(PF診断ログの平均値)
#   - 最終推定位置(x, y)
#   - 終点x誤差 = |最終推定x - EXPECTED_ENDPOINT_X|
#     (ユーザーが実測時の記憶から申告した「x=800前後で停止」という実測值を
#      正解の代用として使う唯一の値。歩幅校正のtarget_distance_pxとは独立に、
#      あくまで結果の評価にのみ使う値であり、パラメータをこれに合わせて
#      調整する用途では使わない)
#
# 真のRMSE・曲がり位置誤差・経路選択成功率を計算するには、実測時に区間ごとの
# 正解位置(タイムスタンプ付き)を記録する必要があり、これは進捗メモ§18.3の
# 「未完了」のまま(今後の課題)。
# ============================================================================

import argparse
import csv
import datetime
import re
import subprocess
import sys
from pathlib import Path
from statistics import mean

SCRIPT_DIR = Path(__file__).resolve().parent
PDR_SCRIPT = SCRIPT_DIR / "pdr_pf_improved.py"
RESULTS_DIR = SCRIPT_DIR / "results"

# ユーザーが実測時の記憶から申告した、点③付近での実際の停止x座標(px)。
# map_configs/kanri_4f.jsonのroute_points末尾([800.0, 115.0])と同じ根拠。
EXPECTED_ENDPOINT_X = 800.0

CONDITIONS = [
    {"label": "none", "route_constraint_mode": "none", "route_source": "manual"},
    {"label": "manual-enforce", "route_constraint_mode": "enforce", "route_source": "manual"},
    {"label": "auto-enforce", "route_constraint_mode": "enforce", "route_source": "auto"},
]

FILE_RE = re.compile(r"^INFO: \[(?P<file>pdr_log_\S+\.csv)\]")
STEPS_RE = re.compile(r"処理ステップ数: (?P<steps>\d+)\s+全滅回数: (?P<ext>\d+)")
DIAG_RE = re.compile(
    r"有効粒子率=(?P<valid>[\d.]+), 経路内率=(?P<route>[\d.]+), "
    r"Neff平均=(?P<neff>[\d.]+), 位置分散平均=(?P<spread>[\d.]+)"
)
FINAL_RE = re.compile(r"最終推定位置: x=(?P<x>[-\d.]+), y=(?P<y>[-\d.]+)")


def run_condition(map_config, seed, heading_source, target_distance_px, condition):
    cmd = [
        sys.executable, str(PDR_SCRIPT),
        "--map-config", str(map_config),
        "--no-watch", "--no-show",
        "--seed", str(seed),
        "--heading-source", heading_source,
        "--route-constraint-mode", condition["route_constraint_mode"],
        "--route-source", condition["route_source"],
    ]
    if target_distance_px is not None:
        cmd += ["--target-distance-px", str(target_distance_px)]
    result = subprocess.run(cmd, capture_output=True, text=True, cwd=SCRIPT_DIR)
    if result.returncode != 0:
        print(f"[警告] 条件'{condition['label']}'の実行がエラー終了しました(returncode={result.returncode})", file=sys.stderr)
        print(result.stderr, file=sys.stderr)
    return result.stdout + result.stderr


def parse_log(log_text):
    rows = []
    current_file = None
    pending = {}

    def flush():
        if current_file and pending:
            rows.append({"file": current_file, **pending})

    for line in log_text.splitlines():
        m = FILE_RE.match(line)
        if m:
            flush()
            current_file = m.group("file")
            pending = {}
            continue
        m = STEPS_RE.search(line)
        if m:
            pending["steps"] = int(m.group("steps"))
            pending["extinctions"] = int(m.group("ext"))
            continue
        m = DIAG_RE.search(line)
        if m:
            pending["valid_ratio"] = float(m.group("valid"))
            pending["route_ratio"] = float(m.group("route"))
            pending["neff_mean"] = float(m.group("neff"))
            pending["pos_spread_mean"] = float(m.group("spread"))
            continue
        m = FINAL_RE.search(line)
        if m:
            pending["final_x"] = float(m.group("x"))
            pending["final_y"] = float(m.group("y"))
            continue
    flush()
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-config", type=Path, default=SCRIPT_DIR / "map_configs" / "kanri_4f.json")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--heading-source", choices=["gyro", "android"], default="android")
    parser.add_argument("--target-distance-px", type=float, default=815.0)
    args = parser.parse_args()

    all_rows = []
    for condition in CONDITIONS:
        print(f"=== 条件: {condition['label']} "
              f"(route_constraint_mode={condition['route_constraint_mode']}, "
              f"route_source={condition['route_source']}) を実行中... ===")
        log_text = run_condition(
            args.map_config, args.seed, args.heading_source, args.target_distance_px, condition
        )
        rows = parse_log(log_text)
        if not rows:
            print(f"[警告] 条件'{condition['label']}'から診断値を抽出できませんでした。", file=sys.stderr)
        for row in rows:
            row["condition"] = condition["label"]
            if "final_x" in row:
                row["endpoint_error_x"] = abs(row["final_x"] - EXPECTED_ENDPOINT_X)
            all_rows.append(row)

    if not all_rows:
        print("比較結果がありません。ログ形式が変わっていないか確認してください。", file=sys.stderr)
        return

    columns = [
        "condition", "file", "steps", "extinctions", "valid_ratio", "route_ratio",
        "neff_mean", "pos_spread_mean", "final_x", "final_y", "endpoint_error_x",
    ]

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = RESULTS_DIR / f"{timestamp}_route_source_comparison.csv"
    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        for row in all_rows:
            writer.writerow({c: row.get(c, "") for c in columns})
    print(f"\n比較結果をCSVに保存しました: {out_path}\n")

    # 条件別サマリを表示
    header = f"{'condition':<16}{'file':<24}{'steps':>6}{'ext':>5}{'valid':>7}{'route':>7}{'neff':>8}{'spread':>8}{'final_x':>9}{'final_y':>9}{'err_x':>8}"
    print(header)
    print("-" * len(header))
    for row in all_rows:
        print(
            f"{row['condition']:<16}{row['file']:<24}"
            f"{row.get('steps', ''):>6}{row.get('extinctions', ''):>5}"
            f"{row.get('valid_ratio', float('nan')):>7.3f}{row.get('route_ratio', float('nan')):>7.3f}"
            f"{row.get('neff_mean', float('nan')):>8.1f}{row.get('pos_spread_mean', float('nan')):>8.1f}"
            f"{row.get('final_x', float('nan')):>9.1f}{row.get('final_y', float('nan')):>9.1f}"
            f"{row.get('endpoint_error_x', float('nan')):>8.1f}"
        )

    print("\n=== 条件別平均 ===")
    for condition in CONDITIONS:
        label = condition["label"]
        subset = [r for r in all_rows if r["condition"] == label]
        if not subset:
            continue
        ext_mean = mean(r["extinctions"] for r in subset if "extinctions" in r)
        valid_mean = mean(r["valid_ratio"] for r in subset if "valid_ratio" in r)
        route_mean = mean(r["route_ratio"] for r in subset if "route_ratio" in r)
        err_vals = [r["endpoint_error_x"] for r in subset if "endpoint_error_x" in r]
        err_mean = mean(err_vals) if err_vals else float("nan")
        print(
            f"{label:<16}: 全滅回数平均={ext_mean:.1f}, 有効粒子率平均={valid_mean:.3f}, "
            f"経路内率平均={route_mean:.3f}, 終点x誤差平均={err_mean:.1f}px (n={len(subset)})"
        )


if __name__ == "__main__":
    main()
