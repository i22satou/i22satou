#!/usr/bin/env python3
"""建築平面図から、通路=白・通行不可=黒の2値地図を作る。"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from map_processing import binarize_floorplan


def parse_args():
    p = argparse.ArgumentParser(description='建築平面図をPF用の2値地図へ変換します。')
    p.add_argument('--input', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--preview', type=Path, default=None)
    p.add_argument('--crop', required=True, help='元画像座標 x0,y0,x1,y1')
    p.add_argument('--start', required=True, help='元画像座標 x,y。通路内を指定')
    p.add_argument('--threshold', type=int, default=190)
    p.add_argument('--line-length', type=int, default=12)
    p.add_argument('--min-wall-area', type=int, default=35)
    p.add_argument('--wall-radius', type=int, default=1)
    p.add_argument('--gap-close', type=int, default=5)
    p.add_argument('--passage-open-radius', type=int, default=1)
    p.add_argument(
        '--walkable-seed', action='append', default=[],
        help='切り抜き後座標x,y。外周接触などで除外された連結領域を白く残す。複数指定可。',
    )
    p.add_argument(
        '--walkable-rect', action='append', default=[],
        help='切り抜き後座標x0,y0,x1,y1。指定室内を白く残す。壁線は黒のまま。複数指定可。',
    )
    p.add_argument(
        '--force-walkable-rect', action='append', default=[],
        help='切り抜き後座標x0,y0,x1,y1。文字等の誤検出を除き室内を強制的に白くする。部屋境界の内側を指定。',
    )
    return p.parse_args()


def save_preview(result, path: Path):
    gray_rgb = Image.fromarray(result.gray).convert('RGB')
    overlay = np.asarray(gray_rgb).copy()
    overlay[result.wall_mask] = [220, 40, 40]
    overlay[result.passage_mask] = (
        0.50 * overlay[result.passage_mask] + 0.50 * np.array([40, 180, 80])
    ).astype(np.uint8)
    overlay_img = Image.fromarray(overlay)
    binary_img = Image.fromarray(result.binary).convert('RGB')
    w, h = gray_rgb.size
    canvas = Image.new('RGB', (w, h * 3 + 90), 'white')
    canvas.paste(gray_rgb, (0, 25))
    canvas.paste(overlay_img, (0, h + 55))
    canvas.paste(binary_img, (0, h * 2 + 85))
    draw = ImageDraw.Draw(canvas)
    draw.text((8, 5), '1. cropped original', fill='black')
    draw.text((8, h + 35), '2. red=detected wall, green=selected passage', fill='black')
    draw.text((8, h * 2 + 65), '3. binary: white=passage, black=blocked', fill='black')
    path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(path)


def main():
    args = parse_args()
    result = binarize_floorplan(
        args.input, args.crop, args.start,
        threshold=args.threshold,
        line_length=args.line_length,
        min_wall_area=args.min_wall_area,
        wall_radius=args.wall_radius,
        gap_close=args.gap_close,
        passage_open_radius=args.passage_open_radius,
        extra_walkable_seeds=args.walkable_seed,
        extra_walkable_rects=args.walkable_rect,
        force_walkable_rects=args.force_walkable_rect,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(result.binary).save(args.output)
    if args.preview:
        save_preview(result, args.preview)
    print(f'保存: {args.output}')
    if args.preview:
        print(f'確認画像: {args.preview}')
    print(f'crop={result.crop}, start_local={result.start_local}')
    print(f'passage_ratio={result.passage_mask.mean():.4f}')


if __name__ == '__main__':
    main()
