# 移動様態判定のヨーレート修正(2026-10-01)の前後比較表を作る。pdr_program/ で実行する。
# 入力: 同じ条件で実行した run_evaluation.py の2つの出力フォルダ(修正前・修正後)。
# 終点までの距離は代替指標(RMSEではない)。終点(800,115)px は実測時に止まった位置の申告値で、
# 9/24の endpoint_proxy_0805.csv と同じ定義。11.4 px/m。調整用データで、評価結果ではない。
import re
import sys
from pathlib import Path

import numpy as np
import pandas as pd

BEFORE = Path("results/20261001_095320_evaluation_0805_yawrate_before")
AFTER = Path("results/20261001_095905_evaluation_0805_yawrate_after")
OUT = Path(__file__).resolve().parent
END_XY, PX_PER_M = (800.0, 115.0), 11.4
FILE_RE = re.compile(r"\[(pdr_log_[^\]]+\.csv)\] 保存済み開始位置")
BEH_RE = re.compile(r"移動様態: 直進=(\d+), 曲がり=(\d+), 滞留=(\d+)")


def turning_rows(run_root):
    rows = []
    for log in sorted(run_root.glob("runs/*/seed-*/*.log")):
        method, seed = log.parent.parent.name, int(log.parent.name.split("-")[1])
        current = None
        for line in log.read_text(encoding="utf-8").splitlines():
            m = FILE_RE.search(line)
            if m:
                current = m.group(1)
            m = BEH_RE.search(line)
            if m and current:
                s, t, st = map(int, m.groups())
                rows.append({"method_key": method, "file": current, "seed": seed,
                             "turning_pct": 100.0 * t / (s + t + st)})
    return pd.DataFrame(rows)


def summarize(run_root):
    r = pd.read_csv(run_root / "results_long.csv")
    r["endpoint_m"] = np.hypot(r.final_x - END_XY[0], r.final_y - END_XY[1]) / PX_PER_M
    r = r.merge(turning_rows(run_root), on=["method_key", "file", "seed"], how="left")
    return r.groupby(["method_key", "method", "file"], sort=False).agg(
        n=("endpoint_m", "size"),
        extinctions_mean=("extinctions", "mean"), extinctions_std=("extinctions", "std"),
        endpoint_m_mean=("endpoint_m", "mean"), endpoint_m_std=("endpoint_m", "std"),
        turning_pct_mean=("turning_pct", "mean"), turning_pct_min=("turning_pct", "min"),
        turning_pct_max=("turning_pct", "max"),
    ).reset_index()


before, after = summarize(BEFORE), summarize(AFTER)
# 各実行の曲がり判定割合(runs/ のログはgitに入らないので、ここに残す。方式AはPFを使わないので無い)
pd.concat([turning_rows(BEFORE).assign(version="before"), turning_rows(AFTER).assign(version="after")]).round(2) \
    .to_csv(OUT / "turning_by_run.csv", index=False, encoding="utf-8-sig")
table = before.merge(after, on=["method_key", "method", "file", "n"], suffixes=("_before", "_after"))
table.insert(0, "注意", "調整用データ(0805の3本)。評価結果ではない。終点までの距離は代替指標でRMSEではない")
table.round(2).to_csv(OUT / "comparison.csv", index=False, encoding="utf-8-sig")

pd.set_option("display.width", 250)
cols = ["method_key", "file", "extinctions_mean_before", "extinctions_mean_after",
        "endpoint_m_mean_before", "endpoint_m_std_before", "endpoint_m_mean_after", "endpoint_m_std_after",
        "turning_pct_mean_before", "turning_pct_mean_after", "turning_pct_min_after", "turning_pct_max_after"]
print(table[cols].round(2).to_string(index=False))
