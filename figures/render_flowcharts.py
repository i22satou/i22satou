# -*- coding: utf-8 -*-
"""make_flowcharts.py が作った .drawio を、卒論に貼る .png(2倍)に描画する。

draw.io(デスクトップ版)があればそのCLIで書き出す。無ければ(このWindows機など)、Edge か
Chrome のヘッドレス表示で draw.io 公式ビューア(viewer.diagrams.net の viewer-static.min.js。
ネット接続が要る)に描かせて画面を撮り、内容の外接矩形+余白で切り出す。どちらでも見た目は
ほぼ同じだが、文字の書体は環境によって少し変わる。

    python render_flowcharts.py              # 3つとも
    python render_flowcharts.py pdr_flow_main
"""
import html
import json
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
NAMES = ["pdr_system_diagram", "pdr_flow_main", "pdr_flow_pf_detail"]
SCALE = 2          # 出力の倍率
MARGIN = 12        # 切り出すときの余白(倍率をかける前のpx)
DRAWIO = ["/Applications/draw.io.app/Contents/MacOS/draw.io",
          r"C:\Program Files\draw.io\draw.io.exe", shutil.which("drawio") or ""]
BROWSERS = [r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
            r"C:\Program Files\Google\Chrome\Application\chrome.exe",
            "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
            shutil.which("google-chrome") or "", shutil.which("chromium") or ""]
VIEWER = "https://viewer.diagrams.net/js/viewer-static.min.js"


def first_existing(paths):
    return next((p for p in paths if p and Path(p).exists()), None)


def crop(png):
    """白い余白を落とし、内容の外接矩形 + MARGIN で切り出す。"""
    from PIL import Image, ImageChops
    im = Image.open(png).convert("RGB")
    bbox = ImageChops.difference(im, Image.new("RGB", im.size, (255, 255, 255))).getbbox()
    m = MARGIN * SCALE
    im.crop((max(0, bbox[0] - m), max(0, bbox[1] - m),
             min(im.width, bbox[2] + m), min(im.height, bbox[3] + m))).save(png)


def render_with_browser(browser, src, dst, workdir):
    xml = src.read_text(encoding="utf-8")
    w, h = map(int, re.search(r'pageWidth="(\d+)" pageHeight="(\d+)"', xml).groups())
    cfg = {"xml": xml, "highlight": "none", "nav": False, "toolbar": "", "lightbox": False,
           "resize": False, "border": 16, "zoom": 1, "center": False}
    # data-mxgraph は属性値なので、JSON全体をHTMLエスケープする(しないと &lt;br&gt; が壊れる)
    page = workdir / f"{src.stem}.html"
    page.write_text(
        "<!DOCTYPE html><html><head><meta charset='utf-8'><style>"
        "html,body{margin:0;padding:0;background:#fff;overflow:hidden}</style></head><body>"
        f"<div class='mxgraph' data-mxgraph=\"{html.escape(json.dumps(cfg, ensure_ascii=False), quote=True)}\"></div>"
        f"<script src='{VIEWER}'></script></body></html>", encoding="utf-8")
    subprocess.run([browser, "--headless=new", "--disable-gpu", "--hide-scrollbars",
                    f"--force-device-scale-factor={SCALE}", f"--window-size={w + 100},{h + 100}",
                    "--virtual-time-budget=15000", f"--screenshot={dst}", page.as_uri()],
                   check=True, capture_output=True, timeout=180)
    crop(dst)


def main(names):
    drawio = first_existing(DRAWIO)
    browser = None if drawio else first_existing(BROWSERS)
    if not drawio and not browser:
        sys.exit("draw.io も Edge/Chrome も見つからない。draw.io で開いて手で書き出すこと。")
    with tempfile.TemporaryDirectory() as td:
        for name in names:
            src, dst = HERE / f"{name}.drawio", HERE / f"{name}.png"
            if drawio:
                subprocess.run([drawio, "--export", "--format", "png", "--scale", str(SCALE),
                                "--border", str(MARGIN), "--output", str(dst), str(src)],
                               check=True, capture_output=True, timeout=180)
            else:
                render_with_browser(browser, src, dst, Path(td))
            print("描画:", dst.name, "(draw.io)" if drawio else "(ブラウザ+draw.ioビューア)")


if __name__ == "__main__":
    main(sys.argv[1:] or NAMES)
