# -*- coding: utf-8 -*-
"""卒論用フローチャート(.drawio)を生成する。出力後はdraw.ioで手編集してよい。"""
from pathlib import Path
from html import escape

OUT = Path("/Users/soma/Library/CloudStorage/OneDrive-独立行政法人国立高等専門学校機構/卒業研究/i22satou/figures")

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
EDGE = ("edgeStyle=orthogonalEdgeStyle;rounded=1;html=1;endArrow=block;endFill=1;"
        "strokeColor=#37474F;strokeWidth=1.3;jettySize=auto;orthogonalLoop=1;")
BR = "&lt;br&gt;"


class Diagram:
    def __init__(self, name, did, w, h):
        self.name, self.did, self.w, self.h = name, did, w, h
        self.cells = []

    def box(self, cid, style, text, x, y, w, h, extra=""):
        v = escape(text, quote=True).replace("\n", BR)
        self.cells.append(
            f'<mxCell id="{cid}" value="{v}" style="{S[style]}{extra}" vertex="1" parent="1">'
            f'<mxGeometry x="{x}" y="{y}" width="{w}" height="{h}" as="geometry" /></mxCell>')

    def raw(self, cid, style, text, x, y, w, h):
        self.cells.append(
            f'<mxCell id="{cid}" value="{escape(text, quote=True)}" style="{style}" vertex="1" parent="1">'
            f'<mxGeometry x="{x}" y="{y}" width="{w}" height="{h}" as="geometry" /></mxCell>')

    def edge(self, cid, src, tgt, label="", extra="", points=None):
        geo = '<mxGeometry relative="1" as="geometry">'
        if points:
            geo += '<Array as="points">' + "".join(
                f'<mxPoint x="{px}" y="{py}" />' for px, py in points) + '</Array>'
        geo += '</mxGeometry>'
        self.cells.append(
            f'<mxCell id="{cid}" value="{escape(label, quote=True)}" style="{EDGE}{extra}" '
            f'edge="1" parent="1" source="{src}" target="{tgt}">{geo}</mxCell>')

    def legend(self, x, y, entries):
        for i, (style, label) in enumerate(entries):
            yy = y + i * 27
            self.raw(f"lg{i}", S[style].replace("fontSize=12;", "").replace("fontSize=11;", ""), "", x, yy, 18, 18)
            self.raw(f"lgt{i}", S["note"] + "verticalAlign=middle;", label, x + 26, yy, 260, 18)

    def save(self, name):
        body = "\n".join(self.cells)
        xml = (f'<mxfile host="app.diagrams.net" agent="claude-code" version="24.0.0">\n'
               f'  <diagram id="{self.did}" name="{self.name}">\n'
               f'    <mxGraphModel dx="900" dy="700" grid="0" gridSize="10" guides="1" tooltips="1" '
               f'connect="1" arrows="1" fold="1" page="1" pageScale="1" pageWidth="{self.w}" '
               f'pageHeight="{self.h}" math="0" shadow="0">\n      <root>\n'
               f'        <mxCell id="0" />\n        <mxCell id="1" parent="0" />\n'
               f'{body}\n      </root>\n    </mxGraphModel>\n  </diagram>\n</mxfile>\n')
        (OUT / name).write_text(xml, encoding="utf-8")
        print("書き出し:", name)


LEG = [("prior", "先行研究の引用・踏襲"),
       ("orig", "本研究独自の拡張(既定で有効)"),
       ("origoff", "本研究独自(既定は無効。比較条件)"),
       ("data", "データ・入出力")]

# =====================================================================
# 図4.1 メインフロー
# =====================================================================
d = Diagram("メインフロー", "pdr-main-flow", 1070, 1150)
d.box("z1", "zone", "① 入力・初期化", 30, 40, 950, 270)
d.box("z2", "zone", "② PDR(距離・方位)と移動様態の推定", 30, 330, 950, 430)
d.box("z3", "zone", "③ 地図で拘束した位置推定", 30, 785, 950, 320)

# ゾーン①
d.box("cfg",  "data",  "map_configs/*.json", 60, 90, 210, 44)
d.box("cfgb", "orig",  "設定読み込み・適用\n(必須値の欠落はエラー)", 60, 162, 210, 52)
d.box("mapi", "data",  "建物の平面図", 320, 90, 210, 44)
d.box("mapb", "orig",  "二値化・壁抽出\n(3.4節)", 320, 162, 210, 52)
d.box("dist", "orig",  "壁距離場 dist_map\n連続的な壁尤度 → 式(5.4)", 600, 85, 250, 56)
d.box("rmask","orig",  "経路帯マスクの自動抽出\nroute_source=auto (5.3節)", 600, 157, 250, 56)
d.box("minfo","store", "地図制約情報", 600, 229, 250, 42)
d.edge("e1", "cfg", "cfgb")
d.edge("e2", "mapi", "mapb")
d.edge("e3", "mapb", "dist",  extra="exitX=1;exitY=0.25;entryX=0;entryY=0.5;")
d.edge("e4", "mapb", "rmask", extra="exitX=1;exitY=0.75;entryX=0;entryY=0.5;")
d.edge("e5", "dist", "minfo", extra="exitX=1;exitY=0.5;entryX=1;entryY=0.25;",
       points=[(880, 113), (880, 239)])
d.edge("e6", "rmask","minfo", extra="exitX=0.5;exitY=1;entryX=0.5;entryY=0;")

# ゾーン② 左列(距離)
d.box("acc",   "data",  "加速度3軸", 70, 370, 240, 42)
d.box("step",  "prior", "ステップ検出 (4.2節)\nHPF+LPF、ピーク・谷・傾きの3条件", 70, 432, 240, 58)
d.box("stepc", "orig",  "最短ピーク間隔を上限歩調2.9Hz\nから決定 式(4.4)", 70, 508, 240, 58)
d.box("slen",  "prior", "歩幅推定 (4.3節)\n4乗根式 / 対数式 式(2.2)", 70, 584, 240, 58)
d.box("slenc", "orig",  "校正ゲイン g を乗算後にクリップ\n式(4.5)。全CSV共通の定数倍", 70, 660, 240, 58)
d.edge("e7",  "acc", "step")
d.edge("e8",  "step", "stepc")
d.edge("e9",  "stepc", "slen")
d.edge("e10", "slen", "slenc")

# ゾーン② 右列(方位・移動様態)
d.box("gyro",  "data",  "ジャイロ角速度 / yaw_deg", 700, 370, 240, 42)
d.box("head",  "prior", "方位推定 (4.4節)\nMadgwick updateIMU(地磁気なし)", 700, 432, 240, 58)
d.box("headc", "orig",  "初期方位の校正\n静止区間 または 歩行開始後", 700, 508, 240, 58)
d.box("beh",   "prior", "移動様態判定 (4.7節)\n直進 / 屈折 / 滞留", 700, 584, 240, 58)
d.box("behc",  "orig",  "ヨーレート75%ile + 方位変化の\nAND条件 + ヒステリシス (5.4節)", 700, 660, 240, 58)
d.edge("e11", "gyro", "head")
d.edge("e12", "head", "headc")
d.edge("e13", "headc", "beh")
d.edge("e14", "beh", "behc")

# ゾーン② 中央(経路情報による方位補正・条件分岐)
d.box("dec",   "dec",     "経路情報を方位に\n使うか\n(prefer / enforce)", 400, 440, 210, 104)
d.box("rcor",  "origoff", "経路方位補正 (5.4節)\n区間切替・複数経路仮説\n(auto では中心線抽出が必要)", 395, 570, 220, 76)
d.box("merge", "store",   "1歩分の歩幅・方位", 395, 700, 220, 46)
d.edge("e15", "headc", "dec",  extra="exitX=0;exitY=0.5;entryX=1;entryY=0.5;")
d.edge("e16", "dec", "rcor", "はい",
       extra="exitX=0.5;exitY=1;entryX=0.5;entryY=0;labelBackgroundColor=#FFFFFF;")
d.edge("e17", "rcor", "merge")
d.edge("e18", "dec", "merge", "いいえ",
       extra="exitX=0;exitY=0.5;entryX=0;entryY=0.5;labelBackgroundColor=#FFFFFF;",
       points=[(350, 492), (350, 723)])
d.edge("e19", "slenc", "merge", extra="exitX=1;exitY=0.5;entryX=0;entryY=0.5;")
d.edge("e20", "behc",  "merge", extra="exitX=0;exitY=0.5;entryX=1;entryY=0.5;")

# ゾーン③
d.box("pf",   "big",   "移動様態適応パーティクルフィルタによる位置推定\n(第5章。詳細は図4.4)", 325, 830, 360, 66)
d.box("out1", "store", "推定位置・軌跡を二値地図上へ描画", 325, 940, 360, 48)
d.box("out2", "data",  "results/ へ PNG・診断値CSVを保存", 325, 1015, 360, 48)
d.edge("e21", "merge", "pf")
d.edge("e22", "minfo", "pf", extra="exitX=1;exitY=0.75;entryX=1;entryY=0.5;",
       points=[(1015, 261), (1015, 863)])
d.edge("e23", "pf", "out1")
d.edge("e24", "out1", "out2")

d.legend(70, 900, LEG)
d.raw("cap", S["note"], "※ 破線の枠は実装済みだが既定では無効。比較実験のときだけ有効にする。", 70, 1020, 240, 40)
d.save("pdr_flow_main.drawio")

# =====================================================================
# 図4.3 パーティクルフィルタ1ステップの詳細
# =====================================================================
d = Diagram("PF詳細", "pdr-pf-detail", 980, 1175)
d.box("zw", "zone", "重み(尤度)の計算", 40, 440, 870, 258)

d.box("in", "data",  "前ステップの粒子群\n+ 移動様態 + 地図制約情報", 345, 40, 270, 56)
d.box("b1", "prior", "移動様態別に粒子数・分散を設定\n(4.7節。直進250 / 屈折600 / 滞留100)", 345, 126, 270, 58)
d.box("b2", "orig",  "Neff比率で粒子数を補正\n式(5.1)(5.2)  (5.6節)", 345, 204, 270, 58)
d.box("br", "origoff", "複数経路仮説: 粒子ごとに\n通路グラフを分岐 (5.7節)", 345, 282, 270, 58)
d.box("tr", "prior", "状態遷移\n歩幅・方位にノイズを付与して移動", 345, 360, 270, 58)
d.edge("f1", "in", "b1")
d.edge("f2", "b1", "b2")
d.edge("f3", "b2", "br")
d.edge("f4", "br", "tr")

d.box("w1", "orig",  "壁尤度 式(5.4)\n距離場による連続値", 80, 478, 230, 58)
d.box("w2", "orig",  "経路帯の重み 式(5.5)\nnone / prefer / enforce", 345, 478, 270, 58)
d.box("w3", "prior", "壁と交差した粒子は\n重みを0にする", 660, 478, 230, 58)
d.box("w4", "origoff", "分岐仮説の選別尤度 式(5.3)\n区間方位とセンサー方位の一致度\n(5.7節、既定は無効)", 345, 556, 270, 70)
d.box("wn", "prior", "重みの積を正規化 式(5.6)", 345, 644, 270, 44)
d.edge("f5", "tr", "w2")
d.edge("f6", "tr", "w1", extra="exitX=0;exitY=0.5;entryX=0.5;entryY=0;", points=[(195, 389)])
d.edge("f7", "tr", "w3", extra="exitX=1;exitY=0.5;entryX=0.5;entryY=0;", points=[(775, 389)])
d.edge("f8", "w1", "wn", extra="exitX=0.5;exitY=1;entryX=0;entryY=0.5;")
d.edge("f9", "w2", "w4")
d.edge("f9b", "w4", "wn")
d.edge("f10", "w3", "wn", extra="exitX=0.5;exitY=1;entryX=1;entryY=0.5;")

# 全滅時はリサンプリング判定を通らず、重みを一様に戻して位置推定へ進む(実装どおり)
d.box("d1",  "dec",   "重みの合計\n&gt; 0 ?", 370, 734, 220, 84)
d.box("r1",  "prior", "全滅リカバリ\n直前位置の周囲へ再配置し\n重みを一様に戻す", 80, 736, 230, 64)
d.box("r1n", "orig",  "全滅回数を記録\n(評価指標 6.6節)", 80, 829, 230, 48)
d.box("d2",  "dec",   "Neff &lt; N / 2 ?", 370, 874, 220, 84)
d.box("r2",  "prior", "リサンプリング", 660, 891, 230, 50)
d.edge("f11", "wn", "d1")
d.edge("f12", "d1", "r1", "いいえ", extra="exitX=0;exitY=0.5;entryX=1;entryY=0.5;labelBackgroundColor=#FFFFFF;")
d.edge("f13", "r1", "r1n")
d.edge("f14", "d1", "d2", "はい", extra="exitX=0.5;exitY=1;entryX=0.5;entryY=0;labelBackgroundColor=#FFFFFF;")
d.edge("f15", "d2", "r2", "はい", extra="exitX=1;exitY=0.5;entryX=0;entryY=0.5;labelBackgroundColor=#FFFFFF;")

d.box("p5", "prior", "重み付き平均 式(5.7)\n+ 直近数ステップの移動平均で平滑化", 345, 984, 270, 58)
d.box("o1", "data",  "その1歩の推定位置", 345, 1069, 270, 46)
d.edge("f16", "r1n", "p5", extra="exitX=0.5;exitY=1;entryX=0;entryY=0.5;")
d.edge("f17", "d2", "p5", "いいえ", extra="exitX=0.5;exitY=1;entryX=0.5;entryY=0;labelBackgroundColor=#FFFFFF;")
d.edge("f18", "r2", "p5", extra="exitX=0.5;exitY=1;entryX=1;entryY=0.5;", points=[(775, 1013)])
d.edge("f19", "p5", "o1")
d.edge("f20", "o1", "in", "次の1歩へ",
       extra="exitX=1;exitY=0.5;entryX=1;entryY=0.5;dashed=1;labelBackgroundColor=#FFFFFF;",
       points=[(945, 1092), (945, 68)])

d.legend(60, 60, LEG)
d.raw("cap2", S["note"], "※ この図は1歩分の処理を表す。歩行が検出されるたびに繰り返す。", 60, 195, 250, 40)
d.save("pdr_flow_pf_detail.drawio")

# =====================================================================
# 図3.3 システム全体構成(計測から評価まで)
# =====================================================================
d = Diagram("システム構成", "pdr-system", 1250, 740)
d.box("z_phone", "zone", "現地: スマートフォン", 20, 40, 220, 350)
d.box("z_drive", "zone", "データ同期", 280, 40, 220, 350)
d.box("z_pc",    "zone", "PC (pdr_program/)", 540, 20, 390, 380)
d.box("z_out",   "zone", "出力", 970, 40, 250, 350)
d.box("z_ev",    "zone", "精度評価 (6.7節)", 280, 505, 940, 200)

d.box("phone",     "prior", "PDR計測アプリ (Android)\nMainActivity.kt", 50, 100, 160, 70)
d.box("phone_csv", "data",  "pdr_log_*.csv\n加速度・ジャイロ・yaw_deg", 45, 205, 170, 66)
d.box("phone_wp",  "data",  "_waypoints.csv\n地点マークの時刻と連番", 45, 305, 170, 66)
d.box("drive",     "store", "Google Drive\n(マイドライブ/PDR)\n自動同期フォルダ", 310, 190, 160, 90)
d.edge("s1", "phone", "phone_csv")
d.edge("s2", "phone", "phone_wp", extra="exitX=0;exitY=0.5;entryX=0;entryY=0.5;",
       points=[(32, 135), (32, 338)])
d.edge("s3", "phone_csv", "drive")
d.edge("s4", "phone_wp", "drive", extra="exitX=1;exitY=0.5;entryX=0.5;entryY=1;")

d.box("cfg",      "data",  "map_configs/*.json", 570, 65, 180, 50)
d.box("mapimg",   "data",  "二値地図画像\n(map_binarizer.py)", 570, 130, 180, 56)
d.box("watch",    "orig",  "フォルダ監視\n(watchdog)", 570, 202, 180, 56)
d.box("startpos", "data",  "start_positions.csv\n(開始位置)", 570, 274, 180, 56)
d.box("main",     "big",   "pdr_pf_improved.py\n+ pdr_route_graph.py\nSmartPDR + 移動様態適応PF\n(図4.2・図4.4)", 790, 140, 190, 110)
d.edge("s5", "drive", "watch")
for i, src in enumerate(["cfg", "mapimg", "watch", "startpos"]):
    d.edge(f"s6{i}", src, "main", extra="exitX=1;exitY=0.5;entryX=0;entryY=0.5;")

d.box("view", "store", "リアルタイム表示\n(matplotlib)", 1000, 65, 190, 56)
d.box("png",  "data",  "results/ へ PNG自動保存", 1000, 137, 190, 56)
d.box("traj", "data",  "推定軌跡CSV\n(--save-trajectory-csv)", 1000, 209, 190, 56)
for i, tgt in enumerate(["view", "png", "traj"]):
    d.edge(f"s7{i}", "main", tgt, extra="exitX=1;exitY=0.5;entryX=0;entryY=0.5;")

d.box("land", "orig", "pick_landmarks.py\n地図上の目印の座標表\n(計測前に一度だけ作成)", 320, 560, 210, 76)
d.box("gt",   "orig", "build_ground_truth.py\n地点マークの連番と座標表を\n突き合わせ正解位置列を作る", 600, 560, 230, 76)
d.box("ev",   "orig", "evaluate_accuracy.py\n平均誤差・RMSE・最大誤差\n(6.6節)", 900, 560, 230, 76)
d.edge("s8", "land", "gt")
d.edge("s9", "gt", "ev")
d.edge("s10", "drive", "gt", extra="exitX=0.5;exitY=1;entryX=0.5;entryY=0;",
       points=[(390, 465), (715, 465)])   # ゾーン見出しの高さを避けて上を通す
d.edge("s11", "traj", "ev", extra="exitX=0.5;exitY=1;entryX=0.5;entryY=0;", points=[(1095, 450), (1015, 450)])

d.legend(45, 430, [("prior", "先行研究の引用・踏襲"),
                   ("orig", "本研究独自"),
                   ("data", "データ・入出力")])
d.raw("evnote", S["note"], "※ パイプラインは動作確認済み(--self-test)。\n実測の正解データはまだ取得できていない。",
      320, 655, 300, 40)
d.save("pdr_system_diagram.drawio")
