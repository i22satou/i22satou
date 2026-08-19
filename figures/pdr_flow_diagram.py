# -*- coding: utf-8 -*-
"""
PDR屋内測位プログラムの処理フロー・実装状況図 (A4横 / Word添付用)
CLAUDE.md / pdr_pf_improved.py / CLAUDE_MEMO.txt の記述に基づき、
先行研究引用部分と本研究独自部分を色分けし、目標方式(方式E)に対する
未実装要素を明示する。
"""
import os

import matplotlib
import japanize_matplotlib  # noqa: F401  (日本語フォント登録)
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Polygon, Rectangle

matplotlib.rcParams['hatch.linewidth'] = 0.6

# このスクリプト自身のあるディレクトリ(figures/)へ直接出力する。
# 以前は他セッションの一時フォルダへの絶対パス固定で、実行のたびに
# 手動コピーが必要だったため、常に有効なパスに修正。
OUT_DIR = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------------------
STYLES = {
    'prior':    dict(fc='#dbe9f7', ec='#1f4e79', hatch=None,   ls='-',  lw=1.1, tc='#173a5e'),
    'orig':     dict(fc='#ffd8a8', ec='#b5460a', hatch=None,   ls='-',  lw=1.4, tc='#7a2e00'),
    'orig_off': dict(fc='#fff2df', ec='#b5460a', hatch='////', ls='-',  lw=1.1, tc='#7a2e00'),
    'gap':      dict(fc='#ffffff', ec='#b71c1c', hatch=None,   ls='--', lw=1.3, tc='#b71c1c'),
    'data':     dict(fc='#eeeeee', ec='#555555', hatch=None,   ls='-',  lw=1.0, tc='#222222'),
}


def rect(ax, cx, cy, w, h, text, style='prior', fontsize=6.3, bold=False, z=3):
    s = STYLES[style]
    box = FancyBboxPatch((cx - w / 2, cy - h / 2), w, h,
                          boxstyle="round,pad=0,rounding_size=1.1",
                          linewidth=s['lw'], edgecolor=s['ec'], facecolor=s['fc'],
                          linestyle=s['ls'], hatch=s['hatch'], zorder=z)
    ax.add_patch(box)
    ax.text(cx, cy, text, ha='center', va='center', fontsize=fontsize, color=s['tc'],
             fontweight='bold' if bold else 'normal', zorder=z + 1, linespacing=1.2)
    return (cx, cy, w, h)


def para(ax, cx, cy, w, h, text, style='data', fontsize=6.3, z=3):
    s = STYLES[style]
    skew = h * 0.42
    pts = [(cx - w / 2 + skew, cy + h / 2), (cx + w / 2 + skew, cy + h / 2),
           (cx + w / 2 - skew, cy - h / 2), (cx - w / 2 - skew, cy - h / 2)]
    poly = Polygon(pts, closed=True, linewidth=s['lw'], edgecolor=s['ec'],
                    facecolor=s['fc'], linestyle=s['ls'], zorder=z)
    ax.add_patch(poly)
    ax.text(cx, cy, text, ha='center', va='center', fontsize=fontsize, color=s['tc'],
             zorder=z + 1, linespacing=1.15)
    return (cx, cy, w, h)


def diamond(ax, cx, cy, w, h, text, style='prior', fontsize=5.0, z=3):
    s = STYLES[style]
    pts = [(cx, cy + h / 2), (cx + w / 2, cy), (cx, cy - h / 2), (cx - w / 2, cy)]
    poly = Polygon(pts, closed=True, linewidth=s['lw'], edgecolor=s['ec'],
                    facecolor=s['fc'], zorder=z)
    ax.add_patch(poly)
    ax.text(cx, cy, text, ha='center', va='center', fontsize=fontsize, color=s['tc'],
             zorder=z + 1, linespacing=0.95)
    return (cx, cy, w, h)


def arrow(ax, points, color='#333333', lw=0.9, ls='-', z=2):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    ax.plot(xs, ys, color=color, lw=lw, linestyle=ls, zorder=z, solid_capstyle='round')
    ax.annotate('', xy=points[-1], xytext=points[-2],
                 arrowprops=dict(arrowstyle='-|>', color=color, lw=lw, mutation_scale=7),
                 zorder=z)


def R(b, dy=0):
    return (b[0] + b[2] / 2, b[1] + dy)


def L(b, dy=0):
    return (b[0] - b[2] / 2, b[1] + dy)


def T(b, dx=0):
    return (b[0] + dx, b[1] + b[3] / 2)


def B(b, dx=0):
    return (b[0] + dx, b[1] - b[3] / 2)


def panel_frame(ax, x0, y0, x1, y1, label, fontsize=9.5):
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False,
                            edgecolor='#333333', linewidth=1.6, zorder=1))
    ax.text(x0 + 2.5, y1 - 1.4, label, fontsize=fontsize, fontweight='bold',
             color='#111111', ha='left', va='top', zorder=5,
             bbox=dict(boxstyle='square,pad=0.15', fc='white', ec='none'))


def swatch(ax, x, y, w, h, style):
    s = STYLES[style]
    box = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=0.5",
                          linewidth=s['lw'], edgecolor=s['ec'], facecolor=s['fc'],
                          linestyle=s['ls'], hatch=s['hatch'], zorder=3)
    ax.add_patch(box)


# ===========================================================================
fig = plt.figure(figsize=(297 / 25.4, 210 / 25.4), dpi=300)
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, 297)
ax.set_ylim(0, 210)
ax.set_aspect('equal')
ax.axis('off')

# タイトル -------------------------------------------------------------
ax.text(148.5, 206.6, "PDR屋内測位プログラムの処理フローと実装状況",
         fontsize=15.5, fontweight='bold', ha='center', va='center', color='#111111')
ax.text(148.5, 201.8,
         "先行研究(SmartPDR・移動様態PF)からの拡張と、目標方式(方式E)に対する未実装要素の可視化",
         fontsize=9, ha='center', va='center', color='#333333')

# ===========================================================================
# ① Panel 1: 入力・初期化   [196, 176]
# ===========================================================================
panel_frame(ax, 6, 176, 291, 196, "① 入力・初期化")

b_json = para(ax, 26, 187, 28, 5.6, "JSON設定\n(map_configs/*.json)", 'data', 5.6)
b_cfg = rect(ax, 63, 187, 34, 5.6, "設定読み込み・適用\n(必須値欠落はエラー)", 'orig', 5.4)
b_map = para(ax, 26, 180.5, 28, 5.6, "地図前処理\n(文字除去・線分抽出)", 'data', 5.6)
b_bin = rect(ax, 63, 180.5, 34, 5.6, "2値地図\n(map_binarizer.py 等)", 'orig', 5.4)
b_c1 = rect(ax, 116, 183.7, 32, 9, "①\n地図制約情報\n(壁・距離場 dist_map)", 'data', 5.6)

arrow(ax, [R(b_json), L(b_cfg)])
arrow(ax, [R(b_map), L(b_bin)])
arrow(ax, [R(b_cfg), (100, 187), (100, 185.2), L(b_c1, 1.3)])
arrow(ax, [R(b_bin), (100, 180.5), (100, 182.2), L(b_c1, -1.3)])

ax.text(157, 187, "地図規模に応じた移動様態別粒子数(250/600/100等)を設定\n[先行研究の枠組み・値は本研究で調整]",
        fontsize=5.2, ha='left', va='center', color='#173a5e', linespacing=1.35)
ax.text(157, 180.5, "文字列除去・水平垂直線分抽出による壁検出\n(実測地図ごとの前処理は本研究独自の実装)",
        fontsize=5.2, ha='left', va='center', color='#7a2e00', linespacing=1.35)

# ===========================================================================
# ② Panel 2: PDR・移動様態・経路補正   [174, 126]
# ===========================================================================
panel_frame(ax, 6, 126, 291, 174, "② PDR・移動様態・経路補正")

# --- 歩幅系 [SmartPDR] ---
RY = 164
b_acc = para(ax, 21, RY, 22, 6.2, "加速度3軸", 'data', 5.6)
b_sig = rect(ax, 58, RY, 34, 7, "歩行信号生成\n合成加速度・HPF・LPF", 'prior', 5.4)
b_pk = rect(ax, 101, RY, 34, 7, "歩行ピーク検出\n(ピーク/谷差/傾き 3条件)", 'prior', 5.2)
b_str = rect(ax, 144, RY, 34, 7, "動的歩幅推定\n(対数式/4乗根式 切替)", 'prior', 5.2)
b_s2 = rect(ax, 186, RY, 24, 7, "②\n歩幅推定", 'data', 5.6)
arrow(ax, [R(b_acc), L(b_sig)])
arrow(ax, [R(b_sig), L(b_pk)])
arrow(ax, [R(b_pk), L(b_str)])
arrow(ax, [R(b_str), L(b_s2)])
ax.text(213, 164, "[SmartPDR]\nステップ検出・歩幅推定式を踏襲",
        fontsize=5.0, ha='left', va='center', color='#1f4e79', linespacing=1.3)

b_cal = rect(ax, 144, 155.5, 40, 6.5, "総移動距離の一様校正\n(target_distance_px, 本研究独自)", 'orig', 4.7)
arrow(ax, [B(b_str), (144, 160.5), T(b_cal)])

# --- 方位系・移動様態・経路補正 ---
RY2 = 147
b_gyr = para(ax, 21, RY2, 22, 6.2, "ジャイロ角速度", 'data', 5.4)
b_yr = rect(ax, 59, RY2, 32, 7, "ヨーレート算出・積分\n加速度で重力軸へ投影", 'prior', 5.0)
b_fuse = rect(ax, 98, RY2, 28, 7, "方位融合\n(Madgwick/updateIMU)", 'prior', 5.0)
b_beh = rect(ax, 137, RY2, 30, 7, "移動様態判定\n(直進/屈折/滞留)", 'prior', 5.0)
b_seg = rect(ax, 179, RY2, 32, 7, "経路線分切替判定\n(turn_pending+曲がり角距離)", 'orig', 4.8)
b_hcor = rect(ax, 220, RY2, 26, 7, "経路方位補正", 'orig', 5.4)
b_h3 = rect(ax, 260, RY2, 22, 7, "③\n方位補正", 'data', 5.6)
arrow(ax, [R(b_gyr), L(b_yr)])
arrow(ax, [R(b_yr), L(b_fuse)])
arrow(ax, [R(b_fuse), L(b_beh)])
arrow(ax, [R(b_beh), L(b_seg)])
arrow(ax, [R(b_seg), L(b_hcor)])
arrow(ax, [R(b_hcor), L(b_h3)])

b_and = rect(ax, 92, 137.5, 36, 6.5, "+Android yaw_deg選択\n(heading_source, 本研究独自)", 'orig', 4.6)
arrow(ax, [B(b_fuse), (98, 143.5), T(b_and)])

b_turn = rect(ax, 141, 136.8, 46, 8, "屈折判定: ヨーレート75%ile\n+方位変化AND+ヒステリシス\n(手ぶれ誤検出抑制・本研究独自)", 'orig', 4.4)
arrow(ax, [B(b_beh), (137, 143.5), T(b_turn)])

b_gap2 = rect(ax, 202, 136.8, 50, 8, "【未実装】交差点での複数経路仮説\n(直進/左折/右折へ分岐・維持・選択)\n5.7節・分岐を含む実測データも未取得", 'gap', 4.4)
arrow(ax, [(192, 143.5), (192, 140), (202, 140)], color='#b71c1c', ls='--')

ax.text(148, 130,
        "※route_source=manual(手動経路)前提の実装。2026-08-16〜 二値地図から自動抽出した中心線(分岐非対応の単純経路のみ)でも簡易動作するよう拡張済み",
        fontsize=4.9, ha='center', va='center', color='#7a2e00', style='italic')

# ===========================================================================
# ③ Panel 3: 適応型パーティクルフィルタ・出力   [123, 68]
# ===========================================================================
panel_frame(ax, 6, 68, 291, 123, "③ 適応型パーティクルフィルタ・出力")

b_fusion = rect(ax, 45, 113, 42, 7, "①②③を融合\n(壁距離場・歩幅・方位)", 'data', 5.4)
b_prev = rect(ax, 45, 103, 42, 7, "前状態粒子群\n(位置・重み)", 'data', 5.6)
b_beh2 = rect(ax, 45, 93, 42, 7, "移動様態\n(直進/屈折/滞留)", 'data', 5.6)

BX0, BX1 = 74, 184
b_upd = FancyBboxPatch((BX0, 70), BX1 - BX0, 47,
                        boxstyle="round,pad=0,rounding_size=1.5",
                        fill=True, facecolor='white',
                        edgecolor='#222222', linewidth=1.5, zorder=2)
ax.add_patch(b_upd)
ax.text((BX0 + BX1) / 2, 114.2, "適応型PF更新", fontsize=9, fontweight='bold', ha='center')
ax.plot([BX0 + 2, BX1 - 2], [111.8, 111.8], color='#888888', lw=0.8)

bullets = [
    ("粒子数 100/250/600 切替(移動様態別)", "[先行研究の枠組み／地図規模に応じ本研究で値を調整]", 'prior'),
    ("歩幅・方位ノイズ分散の様態別切替", "[先行研究:移動様態PF]", 'prior'),
    ("連続壁尤度(距離場 dist_map)による重み付け", "[本研究独自] 先行研究の0/1二値判定からの拡張", 'orig'),
    ("経路帯内外の重み付け(none/prefer/enforce)", "[本研究独自] 経路制約モードとして実装", 'orig'),
    ("経路帯の自動抽出(距離変換, 二値地図から)", "[本研究独自] ★研究の核心的貢献(route_source=auto)", 'orig'),
    ("通路と広い部屋の壁際を区別(exclude_wide_rooms)", "[本研究独自・既定OFF] 全滅回数/終点誤差の改善を確認", 'orig_off'),
    ("不確実性適応粒子数(Neff比率で1.5倍/0.75倍)", "[本研究独自・既定OFF] 全滅回数は改善、精度は一様でない", 'orig_off'),
    ("Neff計算・ESS基準のリサンプリング要否判定", "[先行研究の枠組み]", 'prior'),
    ("全滅時リカバリ(経路マスク条件を追加)", "[本研究独自] 経路コリドー外への漏れ出しを抑制", 'orig'),
]
y0b, y1b = 109, 73
n = len(bullets)
step = (y0b - y1b) / (n - 1)
for i, (t1, t2, sty) in enumerate(bullets):
    yy = y0b - i * step
    swatch(ax, 76.5, yy - 1.35, 4.0, 2.9, sty)
    ax.text(82, yy + 0.35, t1, fontsize=5.1, ha='left', va='center', color='#111111')
    s = STYLES[sty]
    ax.text(82, yy - 1.5, t2, fontsize=4.15, ha='left', va='center', color=s['ec'], style='italic')

arrow(ax, [B(b_fusion), (45, 108), (60, 108), (60, 106.5), (74, 106.5)])
arrow(ax, [R(b_prev), (74, 103)])
arrow(ax, [R(b_beh2), (74, 93)])

# --- 未実装ギャップ(通路グラフ・複数経路仮説など) ---
b_gap3 = rect(ax, 238, 112.5, 100, 8.5,
              "【未実装】通路グラフ化(分岐/曲がり点のノード化)・複数経路仮説(交差点分岐選択)\n"
              "・地図領域の明示分類(壁/廊下/部屋/ドア) → いずれも目標方式Eに必要",
              'gap', 4.35)
arrow(ax, [(184, 85), (184, 112.5), L(b_gap3)], color='#b71c1c', ls='--')

# --- 判定〜出力ミニフロー(右カラム) ---
d1 = diamond(ax, 240, 101, 17, 9, "重み合計\n>0?", 'prior', 4.6)
b_recover = rect(ax, 204, 94.5, 34, 7, "全滅時リカバリ\n(経路マスク条件を追加・本研究独自)", 'orig', 4.1)
arrow(ax, [B(d1, -2), (232, 97), R(b_recover, 1.2)])
ax.text(220, 97.3, "No", fontsize=4.4, color='#b71c1c', ha='center')
arrow(ax, [L(b_recover), (184, 94.5)], color='#b5460a')

b_norm = rect(ax, 240, 93.6, 24, 6.4, "重み正規化\n[先行研究]", 'prior', 4.6)
arrow(ax, [B(d1), T(b_norm)])
ax.text(246, 97.2, "Yes", fontsize=4.4, color='#1f4e79', ha='center')

d2 = diamond(ax, 240, 85.9, 17, 9, "Neff <\nn/2?", 'prior', 4.6)
arrow(ax, [B(b_norm), T(d2)])

b_resamp = rect(ax, 275, 85.9, 22, 6.4, "リサンプリング\n[先行研究]", 'prior', 4.2)
arrow(ax, [R(d2, 0), L(b_resamp)])
ax.text(262, 88.4, "Yes", fontsize=4.2, color='#1f4e79', ha='center')
ax.text(240, 80.5, "No", fontsize=4.2, color='#555555', ha='center')

b_updated = rect(ax, 240, 78.2, 26, 6.4, "更新後粒子群", 'data', 4.8)
arrow(ax, [B(d2), T(b_updated)])
arrow(ax, [B(b_resamp), (275, 79), T(b_updated, 4)], color='#333333')

b_out = rect(ax, 240, 71, 96, 5.4,
             "現在位置推定(重み付き平均+移動平均平滑化) → 地図上へ表示・CSV監視(PNG自動保存/フォルダ監視/開始位置登録・本研究独自基盤)",
             'orig', 4.3)
arrow(ax, [B(b_updated), T(b_out)])

# ループ:次ステップの粒子群として反復
arrow(ax, [L(b_updated), (198, 78.2), (198, 65.5), (9, 65.5), (9, 103), L(b_prev)],
      color='#333333', ls=':')
ax.text(60, 65.5, "更新後粒子群を次ステップの前状態粒子群として1ステップごとに反復", fontsize=4.8,
        color='#333333', ha='left', va='bottom')

# ===========================================================================
# ロードマップ(方式A〜E)   [61, 48]
# ===========================================================================
panel_frame(ax, 6, 48, 291, 61, "比較方式 A〜E と現在の実装位置(第6章 比較方式)", fontsize=8.6)

labels = [
    ("A", "PDR単体", "(地図補正なし)", "実装・比較実験済み", 'prior', 4.05),
    ("B", "固定粒子数PF", "(壁衝突判定のみ)", "実装・比較実験済み", 'prior', 4.05),
    ("C", "移動様態適応PF", "(地図トポロジー未使用)", "実装・比較実験済み", 'prior', 4.05),
    ("D", "手動経路優先PF", "(route_points 人手指定)", "実装・比較実験済み", 'orig', 4.05),
    ("E", "提案方式(目標)", "自動経路+不確実性適応+複数経路仮説",
     "部分実装:自動経路抽出・不確実性適応(既定OFF)は実装\n/複数経路仮説・通路グラフ化は未実装", 'orig_off', 3.35),
]
# 方式Eは説明文が長いため箱幅を広げる。パネル(x:6〜291)に対し
# 左右マージンを9mmずつ均等に取り、A〜Dは47mm・Eは63mm幅で並べる
# (以前はx0=21,xw=51の計算でAの左端がパネル外(-4.5mm)へはみ出していた)。
widths = [47, 47, 47, 47, 63]
gap = 4
left0 = 15
lefts = []
lx = left0
for w in widths:
    lefts.append(lx)
    lx += w + gap
centers = [lx0 + w / 2 for lx0, w in zip(lefts, widths)]

for i, ((tag, name, desc, status, sty, fs), lx0, w, cx) in enumerate(zip(labels, lefts, widths, centers)):
    txt = f"方式{tag}: {name}\n{desc}\n[{status}]"
    rect(ax, cx, 53.2, w, 8.6, txt, sty, fs)
    if i < len(labels) - 1:
        arrow(ax, [(cx + w / 2, 53.2), (cx + w / 2 + gap, 53.2)])

gap_mid = (lefts[3] + widths[3] + lefts[4]) / 2  # 方式D右端と方式E左端の中間
ax.annotate("現在の実装位置(方式Dは完了・方式Eは部分実装)", xy=(gap_mid, 58.1),
             xytext=(centers[4] + 8, 64.5), fontsize=4.7, ha='center', color='#b5460a',
             fontweight='bold',
             arrowprops=dict(arrowstyle='-|>', color='#b5460a', lw=1.1))

# ===========================================================================
# 凡例 & 未実装ギャップ一覧   [45, 6]
# ===========================================================================
panel_frame(ax, 6, 6, 140, 45, "凡例(色分けの意味)", fontsize=8.8)

leg_items = [
    ('prior', "先行研究の引用・踏襲部分",
     "[SmartPDR]のステップ検出・歩幅推定式、[先行研究:移動様態PF]の様態分類・粒子数切替の枠組みなど"),
    ('orig', "本研究独自の拡張・実装済み(既定で有効)",
     "連続壁尤度、経路帯制約、自動経路抽出、手ぶれに頑健な屈折判定など"),
    ('orig_off', "本研究独自・実装済みだが既定は無効",
     "不確実性適応粒子数、通路/部屋壁際の区別(exclude_wide_rooms)。効果は検証中"),
    ('gap', "未実装(目標の提案方式に必要な要素)",
     "通路グラフ化、複数経路仮説、明示的な地図領域分類、正解位置列によるRMSE評価など"),
    ('data', "データの入力・出力(処理ステップではない)",
     "センサ入力値、中間出力、粒子群など"),
]
ly = 38.5
for sty, title, desc in leg_items:
    swatch(ax, 9, ly - 2.0, 7, 3.9, sty)
    ax.text(18, ly, title, fontsize=6.1, fontweight='bold', ha='left', va='center', color='#111111')
    ax.text(18, ly - 3.3, desc, fontsize=4.7, ha='left', va='center', color='#444444')
    ly -= 6.7

panel_frame(ax, 146, 6, 291, 45, "目標方式(方式E)に対する主な未実装ギャップ", fontsize=8.8)
gaps = [
    "1. 通路グラフ化: 中心線を骨格化して端点・分岐点・曲がり点をノード化する処理(現状は分岐なしの単純経路のみ抽出)",
    "2. 複数経路仮説: 交差点で直進/左折/右折へ粒子群を分岐させ、尤度で経路候補を選択・棄却する処理",
    "3. 地図領域の明示分類: 壁/廊下/部屋/ドアを区別し、ドアを通らない部屋遷移に低確率を与える処理(自動抽出が簡易代替)",
    "4. 不確実性適応粒子数・exclude_wide_roomsの既定有効化と汎用化(他地図での安全動作の検証)",
    "5. 正解位置列(タイムスタンプ対応)によるRMSE等の正式な定量評価(現状は終点誤差のみの代替指標)",
    "6. 磁気センサなしによる方位ドリフトへの対策、複数被験者・分岐を含む経路データでの一般化検証",
]
gy = 38
for g in gaps:
    ax.text(149, gy, g, fontsize=5.6, ha='left', va='center', color='#7a1010')
    gy -= 5.5

# ---------------------------------------------------------------------------
fig.savefig(f"{OUT_DIR}/pdr_flow_diagram.png", dpi=300)
fig.savefig(f"{OUT_DIR}/pdr_flow_diagram.pdf")
print("saved")
