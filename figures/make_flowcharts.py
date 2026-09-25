# -*- coding: utf-8 -*-
"""卒論用フローチャート(.drawio)を生成する。出力後はdraw.ioで手編集してよい。

2026-09-24: pdr_pf_improved.py の現在の処理と卒論雛形の節・式番号に合わせて内容を直した。
矢印は角を丸めない直線・直角の折れ線にした(edgeStyle=none。出口・入口と折れ点をすべて
指定するので、描画ソフトによって経路が変わらない)。書き出し時に check() で、線が斜めに
なっていないか、線が他の図形を貫通していないか、文字が枠からはみ出しそうでないかを確かめる。

    python make_flowcharts.py
"""
import re
import sys
from pathlib import Path
from html import escape

OUT = Path(__file__).resolve().parent

S = {
    "prior":   "rounded=1;whiteSpace=wrap;html=1;fillColor=#DCE6F5;strokeColor=#3B6DA8;fontSize=12;arcSize=10;",
    "orig":    "rounded=1;whiteSpace=wrap;html=1;fillColor=#FBE4C4;strokeColor=#B5651D;fontSize=12;arcSize=10;",
    "origoff": "rounded=1;whiteSpace=wrap;html=1;fillColor=#FFF6E9;strokeColor=#B5651D;fontSize=12;arcSize=10;dashed=1;dashPattern=6 4;",
    "data":    "shape=parallelogram;whiteSpace=wrap;html=1;fillColor=#F0F1F3;strokeColor=#6B7280;fontSize=12;perimeter=parallelogramPerimeter;arcSize=12;size=12;fixedSize=1;",
    "store":   "rounded=1;whiteSpace=wrap;html=1;fillColor=#F0F1F3;strokeColor=#6B7280;fontSize=12;arcSize=10;",
    "dec":     "rhombus;whiteSpace=wrap;html=1;fillColor=#FFFFFF;strokeColor=#37474F;fontSize=11;",
    "big":     "rounded=1;whiteSpace=wrap;html=1;fillColor=#EEF0F4;strokeColor=#37474F;fontSize=13;fontStyle=1;arcSize=10;strokeWidth=1.6;",
    "zone":    "rounded=1;whiteSpace=wrap;html=1;fillColor=#FAFBFC;strokeColor=#C7CDD4;dashed=1;dashPattern=8 6;verticalAlign=top;align=left;spacingLeft=12;spacingTop=6;fontSize=13;fontColor=#5A6570;fontStyle=1;arcSize=4;",
    "note":    "text;html=1;whiteSpace=wrap;fontSize=11;fontColor=#5A6570;align=left;verticalAlign=top;",
}
# 角を丸めない直線の折れ線。経路は出口・入口・折れ点で完全に決める。
EDGE = "edgeStyle=none;rounded=0;html=1;endArrow=block;endFill=1;strokeColor=#37474F;strokeWidth=1.3;"
BR = "&lt;br&gt;"
PARA = 12          # 平行四辺形の傾き(styleのsize)
MIN_LAST = 14      # 矢じりが見える最後の線分の長さ[px]


class Diagram:
    def __init__(self, name, did, w, h):
        self.name, self.did, self.w, self.h = name, did, w, h
        self.cells = []
        self.boxes = {}   # id -> (style名, x, y, w, h, 文字)
        self.edges = []   # (id, source, target, 点列)

    def box(self, cid, style, text, x, y, w, h, extra=""):
        v = escape(text, quote=True).replace("\n", BR)
        self.boxes[cid] = (style, x, y, w, h, text)
        self.cells.append(
            f'<mxCell id="{cid}" value="{v}" style="{S[style]}{extra}" vertex="1" parent="1">'
            f'<mxGeometry x="{x}" y="{y}" width="{w}" height="{h}" as="geometry" /></mxCell>')

    def raw(self, cid, style, text, x, y, w, h):
        self.cells.append(
            f'<mxCell id="{cid}" value="{escape(text, quote=True).replace(chr(10), BR)}" style="{style}" vertex="1" parent="1">'
            f'<mxGeometry x="{x}" y="{y}" width="{w}" height="{h}" as="geometry" /></mxCell>')

    def port(self, cid, fx, fy):
        """図形の枠上の接続点(fx, fy は0〜1)。平行四辺形の左右は傾きの分だけ内側。"""
        style, x, y, w, h, _ = self.boxes[cid]
        px, py = x + fx * w, y + fy * h
        if style == "data" and fy not in (0, 1):
            px += PARA / 2 if fx == 0 else (-PARA / 2 if fx == 1 else 0)
        return px, py

    def edge(self, cid, src, tgt, label="", exit=(0.5, 1), entry=(0.5, 0), points=None,
             dashed=False, label_dy=None):
        """src の exit から、折れ点 points を順に通って、tgt の entry へ直線で結ぶ。"""
        (ex, ey), (nx, ny) = exit, entry
        style = (f"{EDGE}exitX={ex:g};exitY={ey:g};exitDx=0;exitDy=0;"
                 f"entryX={nx:g};entryY={ny:g};entryDx=0;entryDy=0;")
        if dashed:
            style += "dashed=1;"
        if label:
            # label_dy を与えたときは線の上(または下)に背景なしで置く。既定は線上に白地で置く。
            style += "labelBackgroundColor=none;" if label_dy is not None else "labelBackgroundColor=#FFFFFF;"
        pts = list(points or [])
        geo = '<mxGeometry relative="1" as="geometry">'
        if pts:
            geo += '<Array as="points">' + "".join(
                f'<mxPoint x="{px:g}" y="{py:g}" />' for px, py in pts) + '</Array>'
        if label_dy is not None:
            geo += f'<mxPoint x="0" y="{label_dy:g}" as="offset" />'
        geo += '</mxGeometry>'
        self.cells.append(
            f'<mxCell id="{cid}" value="{escape(label, quote=True)}" style="{style}" '
            f'edge="1" parent="1" source="{src}" target="{tgt}">{geo}</mxCell>')
        self.edges.append((cid, src, tgt, [self.port(src, ex, ey)] + pts + [self.port(tgt, nx, ny)]))

    def legend(self, x, y, entries):
        for i, (style, label) in enumerate(entries):
            yy = y + i * 27
            self.raw(f"lg{i}", S[style].replace("fontSize=12;", "").replace("fontSize=11;", ""), "", x, yy, 18, 18)
            self.raw(f"lgt{i}", S["note"] + "verticalAlign=middle;", label, x + 26, yy, 260, 18)
            self.boxes[f"lg{i}"] = ("legend", x, yy, 18, 18, "")

    def check(self):
        """線が斜め・短すぎ・他の図形を貫通、文字のはみ出し、を調べて問題の一覧を返す。"""
        problems = []
        solid = {k: v for k, v in self.boxes.items() if v[0] != "zone"}
        for cid, src, tgt, pts in self.edges:
            for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
                if abs(x0 - x1) > 0.5 and abs(y0 - y1) > 0.5:
                    problems.append(f"{cid}: 斜めの線分 ({x0:g},{y0:g})-({x1:g},{y1:g})")
                for bid, (_, bx, by, bw, bh, _) in solid.items():
                    if bid in (src, tgt):
                        continue
                    if _run_len((x0, y0), (x1, y1), (bx, by, bw, bh), 2.0) > 0:
                        problems.append(f"{cid}: {bid} を貫通")
            (ax, ay), (bx, by) = pts[-2], pts[-1]
            if abs(ax - bx) + abs(ay - by) < MIN_LAST:
                problems.append(f"{cid}: 最後の線分が短く矢じりが見えない")
        for bid, (style, x, y, w, h, text) in self.boxes.items():
            if style in ("zone", "legend"):
                continue
            fs = 13 if style == "big" else (11 if style == "dec" else 12)
            avail = {"data": w - 2 * PARA - 8, "dec": w * 0.55}.get(style, w - 12)
            for line in text.split("\n"):
                line = re.sub(r"&[a-z]+;", "x", line)
                tw = sum(fs if ord(c) > 0x2000 else 0.6 * fs for c in line)
                if style == "big":
                    tw *= 1.05
                if tw > avail:
                    problems.append(f"{bid}: 文字がはみ出しそう「{line}」({tw:.0f}>{avail:.0f}px)")
        return problems

    def save(self, name):
        body = "\n".join(self.cells)
        xml = (f'<mxfile host="app.diagrams.net" agent="claude-code" version="24.0.0">\n'
               f'  <diagram id="{self.did}" name="{self.name}">\n'
               f'    <mxGraphModel dx="900" dy="700" grid="0" gridSize="10" guides="1" tooltips="1" '
               f'connect="1" arrows="1" fold="1" page="1" pageScale="1" pageWidth="{self.w}" '
               f'pageHeight="{self.h}" math="0" shadow="0">\n      <root>\n'
               f'        <mxCell id="0" />\n        <mxCell id="1" parent="0" />\n'
               f'{body}\n      </root>\n    </mxGraphModel>\n  </diagram>\n</mxfile>\n')
        (OUT / name).write_text(xml, encoding="utf-8", newline="\n")
        problems = self.check()
        print("書き出し:", name, "(問題なし)" if not problems else "")
        for p in problems:
            print("  要確認:", p)
        return problems


def _run_len(a, b, rect, shrink):
    """水平・垂直の線分 a-b が、四辺を shrink だけ縮めた矩形の内部を通る長さ。"""
    (ax, ay), (bx, by) = a, b
    x, y, w, h = rect
    x0, y0, x1, y1 = x + shrink, y + shrink, x + w - shrink, y + h - shrink
    if abs(ay - by) < 0.6:
        if not (y0 < ay < y1):
            return 0.0
        lo, hi = sorted((ax, bx))
        return max(0.0, min(hi, x1) - max(lo, x0))
    if abs(ax - bx) < 0.6:
        if not (x0 < ax < x1):
            return 0.0
        lo, hi = sorted((ay, by))
        return max(0.0, min(hi, y1) - max(lo, y0))
    return 0.0


LEG = [("prior", "先行研究の引用・踏襲"),
       ("orig", "本研究独自の拡張(既定で有効)"),
       ("origoff", "本研究独自(既定は無効。提案方式の変種)"),
       ("data", "データ・入出力")]
R, L, T, B = (1, 0.5), (0, 0.5), (0.5, 0), (0.5, 1)   # 右・左・上・下の辺の中央
problems = []

# =====================================================================
# 図4.2 1つのCSVを処理する際の処理フロー(メインフロー)
# =====================================================================
d = Diagram("メインフロー", "pdr-main-flow", 1070, 1210)
d.box("z1", "zone", "① 入力・初期化(起動時に1回)", 30, 40, 950, 300)
d.box("z2", "zone", "② PDR(距離・方位)と移動様態の推定(1歩ごと)", 30, 360, 950, 480)
d.box("z3", "zone", "③ 地図で拘束した位置推定", 30, 865, 950, 320)

# ゾーン①: 設定と地図。地図制約情報は壁距離場・経路帯(と変種の中心線・通路グラフ)
d.box("cfg",   "data",    "map_configs/*.json", 60, 90, 210, 44)
d.box("cfgb",  "orig",    "設定読み込み・適用\n(必須値の欠落はエラー)", 60, 162, 210, 52)
d.box("mapi",  "data",    "建物の平面図", 320, 90, 210, 44)
d.box("mapb",  "orig",    "二値化・壁抽出 (3.4節)\nmap_binarizer.pyで事前に作成", 320, 162, 210, 52)
d.box("dist",  "orig",    "壁距離場 dist_map\n連続的な壁尤度 → 式(5.6)", 600, 85, 240, 50)
d.box("rmask", "orig",    "経路帯の自動抽出 (5.3節)\nroute_source=auto", 600, 160, 240, 56)
d.box("graph", "origoff", "通路の中心線・通路グラフ\n(5.3節。変種のときだけ作る)", 600, 240, 240, 56)
d.box("minfo", "store",   "地図制約\n情報", 870, 85, 90, 211)
d.edge("e1", "cfg", "cfgb")
d.edge("e2", "mapi", "mapb")
d.edge("e3", "mapb", "dist", exit=R, entry=L, points=[(565, 188), (565, 110)])
d.edge("e4", "mapb", "rmask", exit=R, entry=L)
d.edge("e5", "dist", "minfo", exit=R, entry=(0, (110 - 85) / 211))
d.edge("e6", "rmask", "minfo", exit=R, entry=(0, (188 - 85) / 211))
d.edge("e6b", "rmask", "graph", dashed=True)
d.edge("e6c", "graph", "minfo", exit=R, entry=(0, (268 - 85) / 211), dashed=True)

# ゾーン② 左列(距離)
d.box("acc",   "data",  "加速度3軸", 70, 400, 240, 42)
d.box("step",  "prior", "ステップ検出 (4.2節)\nHPF+LPF、ピーク・谷・傾きの3条件", 70, 478, 240, 58)
d.box("stepc", "orig",  "最短ピーク間隔を上限歩調2.9Hz\nから決定 式(4.4)", 70, 570, 240, 58)
d.box("slen",  "prior", "歩幅推定 (4.3節)\n4乗根式 / 対数式 式(2.2)", 70, 662, 240, 58)
d.box("slenc", "orig",  "校正ゲイン g を乗算後にクリップ\n式(4.5)。全CSV共通の定数倍", 70, 754, 240, 58)
d.edge("e7",  "acc", "step")
d.edge("e8",  "step", "stepc")
d.edge("e9",  "stepc", "slen")
d.edge("e10", "slen", "slenc")

# ゾーン② 右列(方位・移動様態)。様態判定は経路補正前のセンサー方位で行う(実装どおり)
d.box("gyro",  "data",  "ジャイロ・加速度 / yaw_deg列", 700, 400, 240, 42)
d.box("head",  "prior", "方位推定 (4.4節)\ngyro: Madgwick(地磁気なし)\nandroid: CSVのyaw_deg列", 700, 464, 240, 70)
d.box("headc", "orig",  "初期方位の校正 (4.4節)\n開始時の向きを地図の向きに合わせる\n基準: 先頭30サンプル / 最初の10歩", 700, 556, 240, 70)
d.box("beh",   "prior", "移動様態判定 (4.7節)\n歩ごとに 直進 / 曲がり", 700, 674, 240, 58)
d.box("behc",  "orig",  "ヨーレート75%ile + 方位変化の\nAND条件 + ヒステリシス (4.7節)", 700, 754, 240, 58)
d.edge("e11", "gyro", "head")
d.edge("e12", "head", "headc")
d.edge("e13", "headc", "beh")
d.edge("e14", "beh", "behc")

# ゾーン② 中央: 地図の経路による方位補正。順序付きの経路(中心線の変種・手動経路)が
# あるときだけ働き、提案方式の本体では「いいえ」側を通る(5.4節)
d.box("dec",   "dec",     "地図の経路で\n方位を補正するか\n(5.4節)", 425, 543, 210, 96)
d.box("rcor",  "origoff", "経路方位補正 式(5.1)\n曲がり検出後、曲がり角25px以内で\n次の区間へ(中心線・手動経路のみ)", 420, 668, 220, 70)
d.box("merge", "store",   "1歩分の歩幅・方位・移動様態", 360, 760, 290, 46)
d.edge("e15", "headc", "dec", exit=L, entry=R)
d.edge("e16", "dec", "rcor", "はい")
d.edge("e25", "beh", "rcor", "曲がり", exit=L, entry=R, label_dy=-10)
d.edge("e17", "rcor", "merge", entry=((530 - 360) / 290, 0))
d.edge("e18", "dec", "merge", "いいえ", exit=L, entry=((385 - 360) / 290, 0), points=[(385, 591)])
d.edge("e19", "slenc", "merge", exit=R, entry=L)
d.edge("e20", "behc", "merge", exit=L, entry=R)

# ゾーン③
d.box("start", "data",  "開始位置\nstart_positions.csv", 60, 919, 235, 48)
d.box("pf",    "big",   "移動様態適応パーティクルフィルタによる位置推定\n(4.6・4.7節、第5章。1歩分の処理は図4.4)", 335, 910, 340, 66)
d.box("out1",  "store", "推定位置・軌跡を二値地図上へ描画", 345, 1000, 320, 48)
d.box("out2",  "data",  "結果PNGを results/ へ保存(毎回)\n推定軌跡CSV(オプション指定時)", 345, 1072, 320, 56)
d.edge("e21", "merge", "pf")
d.edge("e22", "minfo", "pf", exit=R, entry=R, points=[(1015, 190.5), (1015, 943)])
d.edge("e26", "start", "pf", exit=R, entry=L)
d.edge("e23", "pf", "out1")
d.edge("e24", "out1", "out2")

d.legend(70, 1000, LEG)
d.raw("cap", S["note"], "※ 破線の枠は実装済みだが既定では無効。提案方式の変種を比べるときだけ有効にする。\n"
      "※ 歩が検出されない間(滞留)はPFを更新しない。", 70, 1112, 250, 64)
problems += d.save("pdr_flow_main.drawio")

# =====================================================================
# 図4.4 パーティクルフィルタの1歩分の処理(ParticleFilterPDR.update の順)
# =====================================================================
d = Diagram("PF詳細", "pdr-pf-detail", 980, 1130)
d.box("zw", "zone", "重み(尤度)の計算", 40, 455, 870, 270)

d.box("in", "data",    "前の歩の粒子群 + この歩の\n歩幅・方位・移動様態 + 地図制約情報", 330, 40, 300, 56)
d.box("b1", "prior",   "移動様態別に粒子数・分散を設定\n(4.7節。直進250 / 曲がり600)", 345, 120, 270, 58)
d.box("b2", "orig",    "Neff比率で粒子数を補正 (5.6節)\n式(5.3)(5.4)。重みに従い選び直す", 345, 200, 270, 58)
d.box("br", "origoff", "複数経路仮説 (5.7節): 粒子ごとに\n区間を進め、交差点で分岐し\n区間の向きで方位を補正 式(5.1)", 345, 280, 270, 70)
d.box("tr", "prior",   "状態遷移\n歩幅・方位にノイズを付与して移動", 345, 372, 270, 58)
d.edge("f1", "in", "b1")
d.edge("f2", "b1", "b2")
d.edge("f3", "b2", "br")
d.edge("f4", "br", "tr")

d.box("w1", "orig",    "壁尤度 式(5.6)\n距離場による連続値", 80, 495, 230, 58)
d.box("w2", "orig",    "経路帯の重み 式(5.7)\nnone / prefer / enforce", 345, 495, 270, 58)
d.box("w3", "prior",   "壁と交差した粒子は\n重みを0にする", 660, 495, 230, 58)
d.box("w4", "origoff", "分岐仮説の選別尤度 式(5.5)\n区間方位とセンサー方位の一致度\n(5.7節、既定は無効)", 345, 575, 270, 70)
d.box("wn", "prior",   "重みの積を正規化 式(5.8)", 345, 667, 270, 44)
d.edge("f5", "tr", "w2")
d.edge("f6", "tr", "w1", exit=L, points=[(195, 401)])
d.edge("f7", "tr", "w3", exit=R, points=[(775, 401)])
d.edge("f8", "w1", "wn", entry=L, points=[(195, 689)])
d.edge("f9", "w2", "w4")
d.edge("f9b", "w4", "wn")
d.edge("f10", "w3", "wn", entry=R, points=[(775, 689)])

# 全滅時はリサンプリング判定を通らず、全滅回数を数えてから粒子を配置し直す(実装どおり)
d.box("d1",  "dec",   "重みの合計\n&gt; 0 ?", 370, 750, 220, 84)
d.box("r1n", "orig",  "全滅回数を記録\n(評価指標 6.6節)", 60, 766, 250, 52)
d.box("r1",  "prior", "全滅からの自己復帰 (4.6節)\n移動前の平均+この歩の移動量の周りへ\n再配置し、重みを一様に戻す\n(足りなければ移動前の平均の周り)", 60, 842, 250, 84)
d.box("d2",  "dec",   "Neff &lt; N / 2 ?", 370, 860, 220, 84)
d.box("r2",  "prior", "リサンプリング", 650, 877, 240, 50)
d.edge("f11", "wn", "d1")
d.edge("f12", "d1", "r1n", "いいえ", exit=L, entry=R)
d.edge("f13", "r1n", "r1")
d.edge("f14", "d1", "d2", "はい")
d.edge("f15", "d2", "r2", "はい", exit=R, entry=L)

d.box("p5", "prior", "重み付き平均 式(5.9)\n+ 直近2歩の移動平均で平滑化", 345, 975, 270, 58)
d.box("o1", "data",  "その歩の推定位置", 330, 1057, 300, 46)
d.edge("f16", "r1", "p5", entry=L, points=[(185, 1004)])
d.edge("f17", "d2", "p5", "いいえ")
d.edge("f18", "r2", "p5", entry=R, points=[(770, 1004)])
d.edge("f19", "p5", "o1")
d.edge("f20", "o1", "in", "次の歩へ", exit=R, entry=R, points=[(945, 1080), (945, 68)], dashed=True)

d.legend(60, 60, LEG)
d.raw("cap2", S["note"], "※ この図は1歩分の処理を表す。歩が検出されるたびに繰り返す。", 60, 195, 250, 40)
problems += d.save("pdr_flow_pf_detail.drawio")

# =====================================================================
# 図4.1 システム全体の構成(計測・同期・処理・評価)
# =====================================================================
d = Diagram("システム構成", "pdr-system", 1330, 760)
d.box("z_phone", "zone", "現地: スマートフォン", 20, 40, 236, 350)
d.box("z_drive", "zone", "データ同期", 280, 40, 220, 350)
d.box("z_pc",    "zone", "PC (pdr_program/)", 540, 20, 475, 380)
d.box("z_out",   "zone", "出力", 1045, 40, 260, 350)
d.box("z_ev",    "zone", "精度評価 (6.6・6.7節)", 280, 505, 1025, 235)

d.box("phone",     "prior", "PDR計測アプリ (Android)\nMainActivity.kt", 52, 100, 180, 70)
d.box("phone_csv", "data",  "pdr_log_*.csv\n加速度・ジャイロ・yaw_deg", 44, 202, 196, 66)
d.box("phone_wp",  "data",  "_waypoints.csv\n地点マークの時刻と連番", 44, 305, 196, 66)
d.box("drive",     "store", "Google Drive\n(マイドライブ/PDR)\n自動同期フォルダ", 310, 190, 160, 90)
d.edge("s1", "phone", "phone_csv")
d.edge("s2", "phone", "phone_wp", exit=L, entry=L, points=[(28, 135), (28, 338)])
d.edge("s3", "phone_csv", "drive", exit=R, entry=L)
d.edge("s4", "phone_wp", "drive", exit=R, entry=((360 - 310) / 160, 1), points=[(360, 338)])

d.box("cfg",      "data",  "map_configs/*.json", 570, 62, 180, 50)
d.box("mapimg",   "data",  "二値地図画像\n(map_binarizer.py)", 570, 134, 180, 56)
d.box("watch",    "orig",  "フォルダ監視\n(watchdog)", 570, 207, 180, 56)
d.box("startpos", "data",  "start_positions.csv\n(開始位置)", 570, 285, 180, 56)
d.box("main",     "big",   "pdr_pf_improved.py\n+ pdr_route_graph.py\nSmartPDR + 移動様態適応PF\n(図4.2・図4.4)", 790, 140, 205, 110)
d.edge("s5", "drive", "watch", exit=R, entry=L)
for i, (src, cy) in enumerate([("cfg", 87), ("mapimg", 162), ("watch", 235), ("startpos", 313)]):
    d.edge(f"s6{i}", src, "main", exit=R, entry=L, points=[(770, cy), (770, 195)])

d.box("view", "store", "リアルタイム表示\n(matplotlib)", 1075, 95, 190, 56)
d.box("png",  "data",  "results/ へ PNG自動保存", 1075, 167, 190, 56)
d.box("traj", "data",  "推定軌跡CSV\n(オプション指定時)", 1075, 239, 190, 56)
d.edge("s70", "main", "view", exit=R, entry=L, points=[(1030, 195), (1030, 123)])
d.edge("s71", "main", "png", exit=R, entry=L)
d.edge("s72", "main", "traj", exit=R, entry=L, points=[(1030, 195), (1030, 267)])

# 目印表はクリック方式(pick_landmarks.py)ではなく、マスター表+経路定義から作る(2026-09-03以降)
d.box("land", "orig", "make_route_landmarks.py\n目印のマスター表と経路定義から\n経路ごとの目印表を作る(計測前)", 330, 580, 260, 80)
d.box("gt",   "orig", "build_ground_truth.py\n地点マークの連番と目印表を\n突き合わせ正解位置列を作る", 690, 580, 250, 80)
d.box("ev",   "orig", "evaluate_accuracy.py\n平均誤差・RMSE・最大誤差\n(6.6節)", 1050, 580, 240, 80)
d.edge("s8", "land", "gt", exit=R, entry=L)
d.edge("s9", "gt", "ev", exit=R, entry=L)
d.edge("s10", "drive", "gt", exit=((420 - 310) / 160, 1), points=[(420, 470), (815, 470)])
d.edge("s11", "traj", "ev")

d.legend(45, 430, [("prior", "先行研究の引用・踏襲"),
                   ("orig", "本研究独自"),
                   ("data", "データ・入出力")])
d.raw("evnote", S["note"], "※ run_evaluation.py が、計測一覧表の経路名で目印表を選び、本体を方式×シードで"
      "実行して、これらをまとめて行う。\n※ パイプラインは動作確認済み(--self-test)。実測の正解データは"
      "まだ取得できていない。", 330, 675, 660, 50)
problems += d.save("pdr_system_diagram.drawio")

sys.exit(1 if problems else 0)
