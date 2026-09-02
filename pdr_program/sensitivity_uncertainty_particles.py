# ============================================================================
# sensitivity_uncertainty_particles.py
#
# 【目的】
# 不確実性適応粒子数(§6.5、pdr_pf_improved.pyのUNCERTAINTY_*定数)は、
# neff_low_ratio=0.30 / neff_high_ratio=0.60 / boost_factor=1.5 /
# shrink_factor=0.75 という初期値のままチューニングされていない
# (memo/uncertainty_particles.mdの「今後」節を参照)。本スクリプトは、この4
# パラメータをベースライン値から1つずつ振る one-factor-at-a-time (OFAT) 感度分析を
# 行い、全滅回数・終点x誤差がどう変化するかを確認する。
#
# 【方式】
# compare_route_source.pyのauto-enforce-unc条件(route_constraint_mode=enforce,
# route_source=auto, uncertainty_adaptive_particles=True)を土台とし、4パラメータを
# pdr_pf_improved.pyの--uncertainty-neff-low-ratio等のCLI引数(2026-08-16追加)で
# individually上書きする。JSON設定ファイルを増やさずに済む。
#
# 【変更履歴】
# - 2026-08-16: 新規作成。
# ============================================================================
"""不確実性適応粒子数の4パラメータ(閾値2つ・倍率2つ)についてOFAT感度分析を行う。

使用例:
    python sensitivity_uncertainty_particles.py --seeds 1 7 42 100 777 2024
"""

import argparse
import csv
import datetime
import subprocess
import sys
from pathlib import Path
from statistics import mean

from compare_route_source import (
    DEFAULT_EXPECTED_ENDPOINT_X,
    PDR_SCRIPT,
    RESULTS_DIR,
    SCRIPT_DIR,
    parse_log,
)

# ベースライン(pdr_pf_improved.pyの既定値と一致させる)。
BASELINE = {
    "neff_low_ratio": 0.30,
    "neff_high_ratio": 0.60,
    "boost_factor": 1.5,
    "shrink_factor": 0.75,
}

# 各軸で振る非ベースライン値。ベースライン自体は combos の先頭に1回だけ含める。
SWEEP_AXES = {
    "neff_low_ratio": [0.15, 0.20, 0.40, 0.45],
    "neff_high_ratio": [0.45, 0.50, 0.70, 0.75],
    "boost_factor": [1.2, 1.75, 2.0],
    "shrink_factor": [0.5, 0.65, 0.9],
}

CLI_FLAG = {
    "neff_low_ratio": "--uncertainty-neff-low-ratio",
    "neff_high_ratio": "--uncertainty-neff-high-ratio",
    "boost_factor": "--uncertainty-boost-factor",
    "shrink_factor": "--uncertainty-shrink-factor",
}


def build_combos():
    """[("baseline", パラメータ辞書), ("neff_low_ratio=0.15", ...), ...] を返す。"""
    combos = [("baseline", dict(BASELINE))]
    for axis, values in SWEEP_AXES.items():
        for value in values:
            params = dict(BASELINE)
            params[axis] = value
            combos.append((f"{axis}={value}", params))
    return combos


def run_combo(map_config, seed, heading_source, params):
    cmd = [
        sys.executable, str(PDR_SCRIPT),
        "--map-config", str(map_config),
        "--no-watch", "--no-show",
        "--seed", str(seed),
        "--heading-source", heading_source,
        "--route-constraint-mode", "enforce",
        "--route-source", "auto",
        "--uncertainty-adaptive-particles",
    ]
    for axis, value in params.items():
        cmd += [CLI_FLAG[axis], str(value)]
    result = subprocess.run(cmd, capture_output=True, text=True, cwd=SCRIPT_DIR)
    if result.returncode != 0:
        print(f"[警告] 実行がエラー終了しました(returncode={result.returncode})", file=sys.stderr)
        print(result.stderr, file=sys.stderr)
    return result.stdout + result.stderr


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-config", type=Path, default=SCRIPT_DIR / "map_configs" / "kanri_4f.json")
    parser.add_argument("--seed", type=int, default=42, help="単一シードのみ実行する場合(--seedsと併用不可)。")
    parser.add_argument(
        "--seeds", type=int, nargs="+", default=None,
        help="複数シードで繰り返し実行する。例: --seeds 1 7 42 100 777 2024",
    )
    parser.add_argument("--heading-source", choices=["gyro", "android"], default="android")
    parser.add_argument(
        "--expected-endpoint-x", type=float, default=None,
        help="終点x誤差の評価基準となる実測終点x座標(px)。既知のmap_configなら省略可。",
    )
    args = parser.parse_args()

    seeds = args.seeds if args.seeds is not None else [args.seed]
    expected_endpoint_x = args.expected_endpoint_x
    if expected_endpoint_x is None:
        expected_endpoint_x = DEFAULT_EXPECTED_ENDPOINT_X.get(args.map_config.name)
    if expected_endpoint_x is None:
        print(
            f"[情報] map_config'{args.map_config.name}'の既知の終点x座標が未登録のため、"
            f"終点x誤差の算出をスキップします(--expected-endpoint-xで指定可能)。",
            file=sys.stderr,
        )

    combos = build_combos()
    total = len(combos) * len(seeds)
    print(f"=== {len(combos)}パラメータ組 × {len(seeds)}シード = {total}回の実行を開始します ===")

    all_rows = []
    done = 0
    for combo_label, params in combos:
        for seed in seeds:
            done += 1
            print(f"[{done}/{total}] combo={combo_label} seed={seed} を実行中... ", end="", flush=True)
            log_text = run_combo(args.map_config, seed, args.heading_source, params)
            rows = parse_log(log_text)
            if not rows:
                print("診断値を抽出できませんでした。", file=sys.stderr)
                continue
            print("完了")
            for row in rows:
                row["seed"] = seed
                row["combo"] = combo_label
                for axis, value in params.items():
                    row[axis] = value
                if expected_endpoint_x is not None and "final_x" in row:
                    row["endpoint_error_x"] = abs(row["final_x"] - expected_endpoint_x)
                all_rows.append(row)

    if not all_rows:
        print("感度分析結果がありません。ログ形式が変わっていないか確認してください。", file=sys.stderr)
        return

    columns = [
        "combo", "seed", "file", "neff_low_ratio", "neff_high_ratio", "boost_factor", "shrink_factor",
        "steps", "extinctions", "valid_ratio", "route_ratio", "neff_mean", "pos_spread_mean",
        "final_x", "final_y", "endpoint_error_x",
    ]

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = RESULTS_DIR / f"{timestamp}_uncertainty_sensitivity.csv"
    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        for row in all_rows:
            writer.writerow({c: row.get(c, "") for c in columns})
    print(f"\n感度分析の生データをCSVに保存しました: {out_path}\n")

    # combo別サマリ(全滅回数平均・終点x誤差平均)。ベースラインとの差分も表示する。
    baseline_subset = [r for r in all_rows if r["combo"] == "baseline"]
    baseline_ext = mean(r["extinctions"] for r in baseline_subset if "extinctions" in r)
    baseline_err_vals = [r["endpoint_error_x"] for r in baseline_subset if "endpoint_error_x" in r]
    baseline_err = mean(baseline_err_vals) if baseline_err_vals else float("nan")

    print("=== combo別サマリ(nはファイル×シードの組数) ===")
    header = f"{'combo':<22}{'ext_mean':>10}{'ext_diff':>10}{'err_x_mean':>12}{'err_x_diff':>12}{'n':>5}"
    print(header)
    print("-" * len(header))
    for combo_label, _params in combos:
        subset = [r for r in all_rows if r["combo"] == combo_label]
        if not subset:
            continue
        ext_mean = mean(r["extinctions"] for r in subset if "extinctions" in r)
        err_vals = [r["endpoint_error_x"] for r in subset if "endpoint_error_x" in r]
        err_mean = mean(err_vals) if err_vals else float("nan")
        print(
            f"{combo_label:<22}{ext_mean:>10.2f}{ext_mean - baseline_ext:>+10.2f}"
            f"{err_mean:>12.1f}{err_mean - baseline_err:>+12.1f}{len(subset):>5}"
        )

    print(f"\nベースライン: 全滅回数平均={baseline_ext:.2f}, 終点x誤差平均={baseline_err:.1f}px")


if __name__ == "__main__":
    main()
