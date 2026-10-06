"""中間発表レジュメ用の簡略版処理フロー図(1段幅)を作る。

出力: 図1_処理の流れ_簡略.png(このスクリプトと同じフォルダ)
     --poster を付けると、ポスター用の高解像度版を poster/poster_処理の流れ.png に出す
色分けは figures/pdr_flow_main.png と同じ(青: 先行研究の手法、橙: 本研究で加えた処理、灰: 入出力)。
"""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

plt.rcParams["font.family"] = "Hiragino Sans"

BLUE = ("#dce6f5", "#3b6db3")
ORANGE = ("#fbe3c4", "#c8781e")
GRAY = ("#eeeeee", "#666666")

fig, ax = plt.subplots(figsize=(3.3, 3.9), dpi=300)
ax.set_xlim(0, 100)
ax.set_ylim(-6, 118)
ax.axis("off")


def box(x, y, w, h, text, color, bold=False, fs=6.2):
    face, edge = color
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h,
                                boxstyle="round,pad=0.4,rounding_size=1.5",
                                fc=face, ec=edge, lw=0.8))
    ax.text(x, y, text, ha="center", va="center", fontsize=fs,
            fontweight="bold" if bold else "normal", linespacing=1.25)


def arrow(x1, y1, x2, y2):
    ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle="-|>",
                                 mutation_scale=6, lw=0.7, color="#333333"))


# 入力
box(26, 112, 44, 7, "スマートフォンのIMU\n(加速度・ジャイロ・方位)", GRAY)
box(76, 112, 40, 7, "建物の平面図", GRAY)

# PDR(左)
box(26, 98, 44, 7, "ステップ検出・歩幅推定\n(SmartPDR)", BLUE)
box(26, 86, 44, 7, "移動様態の判定\n(直進/曲がり)", BLUE)
arrow(26, 108.3, 26, 101.8)
arrow(26, 94.3, 26, 89.8)

# 地図(右)
box(76, 98, 40, 7, "二値化", GRAY)
box(76, 86, 40, 7, "経路帯の自動抽出\n【提案1】", ORANGE, bold=True)
arrow(76, 108.3, 76, 101.8)
arrow(76, 94.3, 76, 89.8)

# PF(1歩ごと)
ax.add_patch(FancyBboxPatch((4, 9), 92, 66, boxstyle="round,pad=0.5,rounding_size=2",
                            fc="none", ec="#999999", lw=0.7, ls=(0, (3, 2))))
ax.text(7, 71.5, "パーティクルフィルタ(1歩ごと)", fontsize=6.2, color="#444444", va="center")

box(48, 62, 80, 8, "① 粒子数の決定: 様態別(直進250/曲がり600)\n＋ Neff比率による増減【提案2】", ORANGE, bold=True)
box(48, 48.5, 80, 7, "② 粒子の移動: PDRの1歩分の移動量＋ノイズ", BLUE)
box(48, 35, 80, 8, "③ 重み付け: 壁を横切ったら0，\n壁までの距離・経路帯の外で低下", ORANGE)
box(48, 21.5, 80, 7, "④ リサンプリング(Neff < N/2 のとき)", BLUE)
arrow(48, 57.6, 48, 52.3)
arrow(48, 44.8, 48, 39.3)
arrow(48, 30.7, 48, 25.3)

# PDR・地図からPFへ
arrow(26, 82.3, 26, 66.3)
ax.plot([76, 76, 94, 94], [82.3, 79, 79, 35], color="#333333", lw=0.7)
arrow(94, 35, 88.6, 35)

box(48, 3, 50, 5.5, "推定位置(粒子の重み付き平均)", GRAY)
arrow(48, 17.8, 48, 6)

# 凡例
for i, (c, label) in enumerate([(BLUE, "先行研究の手法"), (ORANGE, "本研究で加えた処理")]):
    ax.add_patch(FancyBboxPatch((20 + i * 34, -5.3), 3.5, 2.6, boxstyle="square,pad=0",
                                fc=c[0], ec=c[1], lw=0.6))
    ax.text(25 + i * 34, -4, label, fontsize=5.6, va="center")

import sys  # noqa: E402

if "--poster" in sys.argv:
    out = Path(__file__).with_name("poster") / "poster_処理の流れ.png"
    out.parent.mkdir(exist_ok=True)
    fig.savefig(out, dpi=1300, bbox_inches="tight", pad_inches=0.02)
else:
    out = Path(__file__).with_name("図1_処理の流れ_簡略.png")
    fig.savefig(out, bbox_inches="tight", pad_inches=0.02)
print("保存:", out)
