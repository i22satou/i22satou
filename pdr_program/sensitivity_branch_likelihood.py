# ============================================================================
# sensitivity_branch_likelihood.py
#
# 【目的】
# 分岐仮説の選別尤度(MULTI_HYPOTHESIS_BRANCH_LIKELIHOOD_SIGMA_DEG、2026-09-03追加)
# の効果を測る。複数経路仮説PFは交差点で粒子をエッジへ分岐させるが、壁尤度も
# 経路帯マスクも「分岐先がどちらも通路」なら仮説を区別できない。そこへ、粒子が
# 乗っている区間の方位と実測センサー方位の一致度を観測尤度として加えたのがこの機能。
#
# 【なぜ提案分布側(--multi-hypothesis-branch-heading-sigma-deg)ではだめだったのか】
# あちらは分岐先の選び方(提案分布)を偏らせるだけで重みを補正しておらず、
# ブートストラップPFでは推定が改善しない。2026-08-30の検証で一様乱択を上回らず
# 既定を100000(一様乱択)に戻した。本スクリプトが測るのは重み側の尤度である。
#
# 【方式】
# 土台は auto-enforce + --multi-hypothesis-routing。選別尤度のσを振り、
# 併せて初期方位の校正方式(samples/walking)も振る。1441の方位は初期基準が
# 約55度ずれており(memo/heading_calibration.md)、選別尤度はセンサー方位を
# 直接使うため、方位の質と強く結びつくと予想されるため。
#
# 【変更履歴】
# - 2026-09-03: 新規作成。
# ============================================================================
"""分岐仮説の選別尤度のσと初期方位校正方式を振って効果を測る。

使用例:
    python sensitivity_branch_likelihood.py --seeds 1 7 42 100 777 2024
"""

import argparse
import csv
import datetime
import subprocess
import sys
from pathlib import Path
from statistics import mean

from compare_route_source import (
    DEFAULT_EXPECTED_ENDPOINT_X, PDR_SCRIPT, RESULTS_DIR, SCRIPT_DIR, parse_log,
)

SIGMAS = [None, 15.0, 30.0, 60.0, 90.0]
HEADING_MODES = ["samples", "walking"]


def run_once(map_config, seed, sigma, heading_mode):
    cmd = [
        sys.executable, str(PDR_SCRIPT), "--map-config", str(map_config),
        "--no-watch", "--no-show", "--seed", str(seed),
        "--heading-source", "android",
        "--heading-calibration-mode", heading_mode,
        "--route-source", "auto", "--route-constraint-mode", "enforce",
        "--multi-hypothesis-routing",
    ]
    if sigma is not None:
        cmd += ["--multi-hypothesis-branch-likelihood-sigma-deg", str(sigma)]
    proc = subprocess.run(cmd, cwd=str(SCRIPT_DIR), capture_output=True, text=True)
    if proc.returncode != 0:
        raise SystemExit(f"エラー: 実行に失敗しました(seed={seed}, σ={sigma})\n{proc.stderr[-2000:]}")
    return parse_log(proc.stderr + proc.stdout)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--map-config", type=Path, default=SCRIPT_DIR / "map_configs" / "kanri_4f.json")
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 7, 42, 100, 777, 2024])
    ap.add_argument("--sigmas", type=float, nargs="+", default=None,
                    help="選別尤度のσ[度]の一覧。0を含めると「無効」条件になる。")
    ap.add_argument("--heading-modes", nargs="+", default=None, choices=["samples", "walking"])
    args = ap.parse_args()

    global SIGMAS, HEADING_MODES
    if args.sigmas is not None:
        SIGMAS = [None if s <= 0 else s for s in args.sigmas]
    if args.heading_modes is not None:
        HEADING_MODES = args.heading_modes

    # 地図ごとの想定終点x(全CSV共通のスカラー)。未登録なら終点x誤差は出さない。
    expected_x = DEFAULT_EXPECTED_ENDPOINT_X.get(args.map_config.name)
    rows = []
    for mode in HEADING_MODES:
        for sigma in SIGMAS:
            for seed in args.seeds:
                for r in run_once(args.map_config, seed, sigma, mode):
                    r.update(seed=seed, sigma=("off" if sigma is None else sigma),
                             heading_mode=mode,
                             endpoint_error_x=(abs(r["final_x"] - expected_x)
                                               if expected_x is not None else None))
                    rows.append(r)
                print(f"  完了: {mode} σ={sigma} seed={seed}", file=sys.stderr)

    RESULTS_DIR.mkdir(exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out = RESULTS_DIR / f"{stamp}_branch_likelihood_sensitivity.csv"
    cols = ["heading_mode", "sigma", "seed", "file", "steps", "extinctions",
            "valid_ratio", "route_ratio", "neff_mean", "pos_spread_mean",
            "final_x", "final_y", "endpoint_error_x"]
    with out.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)

    print(f"\n保存: {out.name}\n")
    print(f"{'方位校正':<9}{'σ':>6}{'CSV':>7}{'全滅':>7}{'有効粒子率':>10}{'終点x誤差':>10}")
    for mode in HEADING_MODES:
        for sigma in ["off"] + [s for s in SIGMAS if s is not None]:
            for name in sorted({r["file"] for r in rows}):
                sel = [r for r in rows if r["heading_mode"] == mode
                       and r["sigma"] == sigma and r["file"] == name]
                if not sel:
                    continue
                errs = [r["endpoint_error_x"] for r in sel if r["endpoint_error_x"] is not None]
                print(f"{mode:<9}{str(sigma):>6}{name[-8:-4]:>7}"
                      f"{mean(r['extinctions'] for r in sel):>7.1f}"
                      f"{mean(r['valid_ratio'] for r in sel):>10.3f}"
                      f"{(mean(errs) if errs else float('nan')):>10.1f}")


if __name__ == "__main__":
    main()
