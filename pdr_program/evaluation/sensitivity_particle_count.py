# ============================================================================
# sensitivity_particle_count.py
#
# 【変更履歴】
# - 2026-10-14: [本研究独自] 新規作成。不確実性適応粒子数の上下限(80/1200)が妥当かを
#               確かめるため、経路帯で制約したPFの粒子数を固定して変え、全滅回数・終点までの
#               距離・PF更新時間を比べる。
#
# 【このスクリプトについて】
# run_evaluation.py の評価ハーネスをそのまま使い、方式の一覧(METHODS)だけを差し替える。
# 本体(pdr_pf_improved.py)は変更しない。
#   - N{粒子数}: 提案方式の地図制約(経路帯の自動抽出+enforce)で、粒子数を固定(--pf-mode fixed
#     --fixed-particles N)。固定粒子数PFなので、ノイズは方式Bと同じ固定値で、移動様態と
#     不確実性適応は使わない(粒子数だけの影響を見るため)。
#   - E_proposed: 比較用の提案方式(run_evaluation.py と同じ条件)。
# 終点までの距離 = 最終推定位置と申告した終点(800,115)pxとの距離 / 縮尺。RMSEではない代替指標。
# 使うデータは既存の調整用3本(評価用データではない)。
#
# 【実行】(pdr_program/ で)
#   python evaluation/sensitivity_particle_count.py
# 【出力】results/<日時>_evaluation_particle_count/ に run_evaluation.py の出力一式と
#   particle_count_summary.csv(粒子数・ファイル別の6シード平均±標準偏差)
# ============================================================================
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

EVALUATION_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(EVALUATION_DIR))
import run_evaluation as rev  # noqa: E402

COUNTS = [50, 80, 120, 188, 250, 400, 600, 900, 1200, 1500]
DECLARED_END_PX = (800.0, 115.0)  # 申告した終点(make_figures_20260924.py と同じ)


def methods():
    fixed = [{"key": f"N{n:04d}", "tiny": str(n), "label": f"経路帯+固定{n}粒子", "short": str(n),
              "section": "", "kind": "variant",
              "args": rev.condition_args(pf_mode="fixed", route_mode="enforce", route_source="auto")
              + ["--fixed-particles", str(n)]} for n in COUNTS]
    base = [m for m in rev.METHODS if m["key"] in ("A_pdr", "E_proposed")]
    return base + fixed


def summarize(out_dir):
    cond = json.loads((out_dir / "conditions.json").read_text(encoding="utf-8"))
    scale = float(cond["scale_px_per_m"])
    df = pd.read_csv(out_dir / "results_long.csv", encoding="utf-8-sig")
    df = df[df["method_key"] != "A_pdr"].copy()
    df["endpoint_m"] = np.hypot(df["final_x"] - DECLARED_END_PX[0],
                                df["final_y"] - DECLARED_END_PX[1]) / scale
    rows = []
    for (key, name), g in df.groupby(["method_key", "file"]):
        rows.append({
            "method_key": key, "file": name, "runs": len(g),
            "particles_mean": g["particles_mean"].mean(),
            "extinctions_mean": g["extinctions"].mean(), "extinctions_std": g["extinctions"].std(),
            "endpoint_m_mean": g["endpoint_m"].mean(), "endpoint_m_std": g["endpoint_m"].std(),
            "pf_update_ms_mean": g["pf_update_ms_median"].mean(),
            "pf_update_total_s_mean": g["pf_update_total_s"].mean(),
        })
    summary = pd.DataFrame(rows)
    summary.to_csv(out_dir / "particle_count_summary.csv", index=False, encoding="utf-8-sig")
    return summary


def main():
    rev.METHODS = methods()
    rev.METHOD_KEYS = [m["key"] for m in rev.METHODS]
    map_config = rev.PROGRAM_DIR / "map_configs" / "kanri_4f.json"
    _cfg, resolved = rev.pdrmod.load_map_config_for_tool(map_config)
    result = rev.run_evaluation(rev.PROGRAM_DIR / "ground_truth" / "measurement_list_0805.csv",
                                resolved.data_dir, map_config, tag="particle_count")
    rev.print_report(result)
    out_dir = Path(result["out_dir"])
    summary = summarize(out_dir)
    with pd.option_context("display.width", 200):
        print(summary.round(2).to_string(index=False))
    print("保存:", out_dir)


if __name__ == "__main__":
    main()
