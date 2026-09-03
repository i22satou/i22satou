#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""フローチャート(.drawio)の線が図形や文字を貫通していないか検査する。

draw.ioのCLIでSVGへ書き出し、エッジの実際の経路と図形・文字の位置を突き合わせる。
目視では見落とすため、.drawioを編集したら必ずこれを通すこと。

    python3 check_flowcharts.py

注意: SVGは内容の外接矩形で切り出されるので、drawio座標とはずれる。図形の位置は
SVG側から取り、drawio側からはエッジの始点・終点(source/target)だけを読む。
文字はforeignObjectで描かれるが、代替の<image>要素が「文字を置ける領域」の矩形を
持つ。実際の字はその中で寄せられるので、字幅を概算して本当の文字の範囲を出す。
"""
import re
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
DRAWIO = "/Applications/draw.io.app/Contents/MacOS/draw.io"
NAMES = ["pdr_flow_main", "pdr_flow_pf_detail", "pdr_system_diagram"]
THRESH_SHAPE = 6.0   # 角の丸めと矢印の食い込みの許容量[px]
THRESH_TEXT = 3.0


def export_svg(name, out_dir):
    dst = out_dir / f"{name}.svg"
    subprocess.run([DRAWIO, "--export", "--format", "svg", "--output", str(dst),
                    str(HERE / f"{name}.drawio")],
                   check=True, capture_output=True)
    return dst


def glyph_box(block, img):
    """<image>が示す領域と、その中での文字の寄せ方から、実際の文字の矩形を求める。"""
    x, y, w, h = img
    fs = 12.0
    fm = re.search(r"font-size:\s*(\d+(?:\.\d+)?)px", block)
    if fm:
        fs = float(fm.group(1))
    body = re.sub(r"<br[^>]*>", "\n", block)
    body = re.sub(r"</div>\s*<div", "\n<div", body)
    body = re.sub(r"<[^>]+>", "", body)
    body = re.sub(r"Text is not SVG.*", "", body)
    lines = [ln.strip() for ln in body.split("\n") if ln.strip()]
    if not lines:
        return None, ""
    # 全角1.0em / 半角0.55em で字幅を概算し、余裕を見て少し広めに取る
    tw = max(sum(fs * (1.0 if ord(c) > 0x2000 else 0.55) for c in ln) for ln in lines)
    tw = min(tw * 1.05, w)
    align = "center"
    am = re.search(r"text-align:\s*(left|right|center)", block)
    if am:
        align = am.group(1)
    if align == "left":
        x0 = x
    elif align == "right":
        x0 = x + w - tw
    else:
        x0 = x + (w - tw) / 2
    return (x0, y, tw, h), " ".join(lines)[:40]


def parse(svg_path, drawio_path):
    svg = svg_path.read_text(encoding="utf-8")
    dio = drawio_path.read_text(encoding="utf-8")
    ends = {m.group(1): (m.group(2), m.group(3)) for m in
            re.finditer(r'<mxCell id="([^"]+)"[^>]*edge="1"[^>]*source="([^"]+)" target="([^"]+)"', dio)}
    zones = {m.group(1) for m in re.finditer(r'<mxCell id="(z[^"]*)"', dio)}

    shapes, texts, segs = {}, [], []
    for m in re.finditer(r'<g data-cell-id="([^"]+)">(.*?)</g></g>', svg, re.S):
        cid, inner = m.group(1), m.group(2)
        if cid in ("0", "1"):        # ルート/レイヤーは図形ではない
            continue
        is_edge = cid in ends
        if not is_edge:
            r = re.search(r'<rect[^>]*\sx="([-\d.]+)"[^>]*\sy="([-\d.]+)"'
                          r'[^>]*\swidth="([\d.]+)"[^>]*\sheight="([\d.]+)"', inner)
            pth = re.search(r'<path d="M ([-\d.]+) ([-\d.]+)', inner)
            if r:
                shapes[cid] = tuple(float(g) for g in r.groups())
            elif pth:
                xs = [float(v) for v in re.findall(r'[MLQ]\s*([-\d.]+)\s+[-\d.]+', inner)]
                ys = [float(v) for v in re.findall(r'[MLQ]\s*[-\d.]+\s+([-\d.]+)', inner)]
                if xs:
                    shapes[cid] = (min(xs), min(ys), max(xs) - min(xs), max(ys) - min(ys))
        for sw in re.finditer(r'<switch>(.*?)</switch>', inner, re.S):
            blk = sw.group(1)
            im = re.search(r'<image[^>]*\sx="([-\d.]+)"[^>]*\sy="([-\d.]+)"'
                           r'[^>]*\swidth="([\d.]+)"[^>]*\sheight="([\d.]+)"', blk)
            if not im:
                continue
            box, s = glyph_box(blk, tuple(float(g) for g in im.groups()))
            if box:
                has_bg = re.search(r"background-color:\s*#[0-9a-fA-F]{6}", blk) is not None
                texts.append(dict(cell=cid, s=s, box=box, bg=has_bg))
        if is_edge:
            for pm in re.finditer(r'<path d="([^"]+)" fill="none"', inner):
                pts = [(float(a), float(b)) for a, b in
                       re.findall(r'[MLQ]\s*([-\d.]+)\s+([-\d.]+)', pm.group(1))]
                for a, b in zip(pts[:-1], pts[1:]):
                    segs.append((cid, a, b))
    return shapes, texts, segs, ends, zones


def run_len(a, b, rect, shrink):
    (ax, ay), (bx, by) = a, b
    x, y, w, h = rect
    x0, y0, x1, y1 = x + shrink, y + shrink, x + w - shrink, y + h - shrink
    if x1 <= x0 or y1 <= y0:
        return 0.0
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


def main():
    ng = 0
    with tempfile.TemporaryDirectory() as td:
        for name in NAMES:
            svg = export_svg(name, Path(td))
            shapes, texts, segs, ends, zones = parse(svg, HERE / f"{name}.drawio")
            found = []
            for eid, a, b in segs:
                src, tgt = ends[eid]
                for sid, rect in shapes.items():
                    if sid in zones or sid in (src, tgt) or sid == eid:
                        continue
                    ov = run_len(a, b, rect, 8.0)
                    if ov > THRESH_SHAPE:
                        found.append((round(ov, 1), "図形を貫通", eid, sid, ""))
                for t in texts:
                    if t["bg"] or t["cell"] == eid or t["cell"] in zones and False:
                        continue
                    ov = run_len(a, b, t["box"], 1.0)
                    if ov > THRESH_TEXT:
                        found.append((round(ov, 1), "文字を横切る", eid, t["cell"], t["s"]))
            for i, t in enumerate(texts):
                for q in texts[i + 1:]:
                    if t["cell"] == q["cell"]:
                        continue
                    (x, y, w, h), (X, Y, W, H) = t["box"], q["box"]
                    ox = min(x + w, X + W) - max(x, X)
                    oy = min(y + h, Y + H) - max(y, Y)
                    if ox > 2 and oy > 2:
                        found.append((round(min(ox, oy), 1), "文字同士が重なる",
                                      t["cell"], q["cell"], f'{t["s"][:18]} / {q["s"][:18]}'))
            print(f"\n===== {name} =====")
            if not found:
                print("  問題なし(線の貫通・文字への重なりは検出されなかった)")
            for ov, kind, c1, c2, s in sorted(found, reverse=True):
                ng += 1
                print(f"  [{kind}] {c1} × {c2} {ov}px {s}")
    return 1 if ng else 0


if __name__ == "__main__":
    sys.exit(main())
