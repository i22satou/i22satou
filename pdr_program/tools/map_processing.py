"""地図画像の前処理・2値化に関する共通処理。

出力規則:
    255 (白) = 通路
      0 (黒) = 壁・部屋・地図外などの通行不可領域
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence, Tuple

import numpy as np
from PIL import Image
from scipy import ndimage

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


@dataclass
class BinarizeResult:
    gray: np.ndarray
    ink_mask: np.ndarray
    wall_mask: np.ndarray
    passage_mask: np.ndarray
    binary: np.ndarray
    crop: Tuple[int, int, int, int]
    start_local: Tuple[int, int]


def parse_xy(value: str | Sequence[int]) -> Tuple[int, int]:
    if isinstance(value, str):
        parts = [v.strip() for v in value.split(',')]
    else:
        parts = list(value)
    if len(parts) != 2:
        raise ValueError('座標は x,y の2要素で指定してください。')
    return int(parts[0]), int(parts[1])


def parse_crop(value: Optional[str | Sequence[int]], shape) -> Tuple[int, int, int, int]:
    height, width = shape[:2]
    if value is None:
        return 0, 0, width, height
    if isinstance(value, str):
        parts = [v.strip() for v in value.split(',')]
    else:
        parts = list(value)
    if len(parts) != 4:
        raise ValueError('cropは x0,y0,x1,y1 の4要素で指定してください。')
    x0, y0, x1, y1 = map(int, parts)
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(width, x1), min(height, y1)
    if x0 >= x1 or y0 >= y1:
        raise ValueError(f'切り抜き範囲が不正です: {(x0, y0, x1, y1)}')
    return x0, y0, x1, y1


def load_gray(path: str | Path) -> np.ndarray:
    return np.asarray(Image.open(path).convert('L'))


def remove_small_components(mask: np.ndarray, min_area: int) -> np.ndarray:
    if min_area <= 1:
        return mask.astype(bool)
    labels, number = ndimage.label(mask)
    if number == 0:
        return mask.astype(bool)
    sizes = np.asarray(ndimage.sum(mask, labels, np.arange(1, number + 1)))
    valid = np.flatnonzero(sizes >= min_area) + 1
    return np.isin(labels, valid)


def select_indoor_free_space(
    free_mask: np.ndarray,
    start_xy: Tuple[int, int],
    border_margin: int = 3,
    min_space_area: int = 20,
    extra_seeds=None,
) -> np.ndarray:
    """建物内部の全自由空間を残し、図面外側だけを除去する。

    廊下だけでなく、教員室・研究室・演習室などの室内も白く残す。
    開始位置につながる成分は必ず残し、それ以外でも画像外周付近に
    接していない十分な面積の白領域は建物内部として残す。
    """
    x, y = start_xy
    h, w = free_mask.shape
    if not (0 <= x < w and 0 <= y < h):
        raise ValueError(f'開始位置{start_xy}が画像範囲 {(w, h)} の外です。')

    labels, number = ndimage.label(free_mask)
    if number == 0:
        raise ValueError('自由空間候補がありません。')

    start_label = int(labels[y, x])
    if start_label == 0:
        ys, xs = np.where(free_mask)
        if len(xs) == 0:
            raise ValueError('開始位置の近くに自由空間候補がありません。')
        idx = np.argmin((xs - x) ** 2 + (ys - y) ** 2)
        start_label = int(labels[ys[idx], xs[idx]])

    margin = max(1, int(border_margin))
    border_labels = np.unique(np.concatenate([
        labels[:margin, :].ravel(),
        labels[-margin:, :].ravel(),
        labels[:, :margin].ravel(),
        labels[:, -margin:].ravel(),
    ]))
    border_labels = set(int(v) for v in border_labels if v != 0)
    sizes = np.asarray(ndimage.sum(free_mask, labels, np.arange(1, number + 1)))

    keep_labels = {start_label}

    # 外周に接していて自動除外された部屋も、追加シードで明示的に残せる。
    for seed in extra_seeds or []:
        sx, sy = parse_xy(seed)
        if not (0 <= sx < w and 0 <= sy < h):
            raise ValueError(f'追加シード{(sx, sy)}が画像範囲 {(w, h)} の外です。')
        seed_label = int(labels[sy, sx])
        if seed_label == 0:
            ys, xs = np.where(free_mask)
            idx = np.argmin((xs - sx) ** 2 + (ys - sy) ** 2)
            seed_label = int(labels[ys[idx], xs[idx]])
        keep_labels.add(seed_label)

    for label_id, area in enumerate(sizes, start=1):
        if label_id not in border_labels and area >= min_space_area:
            keep_labels.add(label_id)

    return np.isin(labels, list(keep_labels))


def keep_component_at(mask: np.ndarray, start_xy: Tuple[int, int]) -> np.ndarray:
    x, y = start_xy
    h, w = mask.shape
    if not (0 <= x < w and 0 <= y < h):
        raise ValueError(f'開始位置{start_xy}が画像範囲 {(w, h)} の外です。')
    labels, number = ndimage.label(mask)
    if number == 0:
        raise ValueError('通路候補がありません。')
    label = int(labels[y, x])
    if label == 0:
        # 指定点が線上の場合、近傍にある最も近い通路画素を用いる。
        ys, xs = np.where(mask)
        if len(xs) == 0:
            raise ValueError('開始位置の近くに通路候補がありません。')
        idx = np.argmin((xs - x) ** 2 + (ys - y) ** 2)
        label = int(labels[ys[idx], xs[idx]])
    return labels == label


def _line_kernel(length: int, horizontal: bool) -> np.ndarray:
    length = max(3, int(length))
    return np.ones((1, length) if horizontal else (length, 1), np.uint8)


def extract_floorplan_walls(
    gray: np.ndarray,
    threshold: int = 190,
    line_length: int = 12,
    min_wall_area: int = 35,
    wall_radius: int = 1,
    gap_close: int = 5,
) -> tuple[np.ndarray, np.ndarray]:
    """文字や寸法数字を抑え、水平・垂直の構造線を壁候補として抽出する。"""
    ink = gray <= int(threshold)

    if cv2 is not None:
        ink_u8 = (ink.astype(np.uint8) * 255)
        horizontal = cv2.morphologyEx(
            ink_u8, cv2.MORPH_OPEN, _line_kernel(line_length, True)
        ) > 0
        vertical = cv2.morphologyEx(
            ink_u8, cv2.MORPH_OPEN, _line_kernel(line_length, False)
        ) > 0
        walls = horizontal | vertical
    else:
        horizontal = ndimage.binary_opening(ink, np.ones((1, line_length), bool))
        vertical = ndimage.binary_opening(ink, np.ones((line_length, 1), bool))
        walls = horizontal | vertical

    # 抽出線に接している元のインクだけ少し戻し、細い構造線を復元する。
    support = ndimage.binary_dilation(walls, np.ones((3, 3), bool), iterations=1)
    walls |= ink & support
    walls = remove_small_components(walls, min_wall_area)

    # 壁の小さな欠けを閉じる。長すぎるclosingは廊下自体を塞ぐため控えめにする。
    if gap_close > 0:
        g = int(gap_close)
        walls |= ndimage.binary_closing(walls, np.ones((1, g), bool))
        walls |= ndimage.binary_closing(walls, np.ones((g, 1), bool))

    if wall_radius > 0:
        radius = int(wall_radius)
        yy, xx = np.ogrid[-radius:radius + 1, -radius:radius + 1]
        disk = xx * xx + yy * yy <= radius * radius
        walls = ndimage.binary_dilation(walls, disk)

    # 切り抜き領域の外へ通路が漏れないよう外周を壁にする。
    walls[[0, -1], :] = True
    walls[:, [0, -1]] = True
    return ink, walls


def binarize_floorplan(
    image_path: str | Path,
    crop: Optional[str | Sequence[int]],
    start_global: str | Sequence[int],
    threshold: int = 190,
    line_length: int = 12,
    min_wall_area: int = 35,
    wall_radius: int = 1,
    gap_close: int = 5,
    passage_open_radius: int = 1,
    extra_walkable_seeds=None,
    extra_walkable_rects=None,
    force_walkable_rects=None,
) -> BinarizeResult:
    full_gray = load_gray(image_path)
    x0, y0, x1, y1 = parse_crop(crop, full_gray.shape)
    gray = full_gray[y0:y1, x0:x1]
    sx, sy = parse_xy(start_global)
    start_local = sx - x0, sy - y0

    ink, walls = extract_floorplan_walls(
        gray,
        threshold=threshold,
        line_length=line_length,
        min_wall_area=min_wall_area,
        wall_radius=wall_radius,
        gap_close=gap_close,
    )
    free = ~walls
    # 廊下だけでなく、壁で囲まれたすべての室内空間を移動可能領域として残す。
    passage = select_indoor_free_space(
        free,
        start_local,
        border_margin=3,
        min_space_area=max(20, min_wall_area),
        extra_seeds=extra_walkable_seeds,
    )

    # 細い枝や線抽出ノイズを軽く除去する。
    if passage_open_radius > 0:
        r = int(passage_open_radius)
        yy, xx = np.ogrid[-r:r + 1, -r:r + 1]
        disk = xx * xx + yy * yy <= r * r
        cleaned = ndimage.binary_opening(passage, disk)
        if cleaned[start_local[1], start_local[0]]:
            passage = select_indoor_free_space(
                cleaned,
                start_local,
                border_margin=3,
                min_space_area=max(20, min_wall_area),
                extra_seeds=extra_walkable_seeds,
            )

    # 誤って除外された既知の室内は矩形で追加できる。壁画素そのものは黒のまま保つ。
    for rect in extra_walkable_rects or []:
        values = [int(v.strip()) for v in rect.split(',')] if isinstance(rect, str) else [int(v) for v in rect]
        if len(values) != 4:
            raise ValueError(f'walkable-rectは x0,y0,x1,y1 で指定してください: {rect}')
        rx0, ry0, rx1, ry1 = values
        rx0, ry0 = max(0, rx0), max(0, ry0)
        rx1, ry1 = min(gray.shape[1], rx1), min(gray.shape[0], ry1)
        if rx0 < rx1 and ry0 < ry1:
            passage[ry0:ry1, rx0:rx1] |= ~walls[ry0:ry1, rx0:rx1]

    # 室内の文字・家具記号などが壁として誤検出された場合、室内だけを強制的に白くする。
    # 壁境界を消さないよう、矩形は部屋の内側に指定する。
    for rect in force_walkable_rects or []:
        values = [int(v.strip()) for v in rect.split(',')] if isinstance(rect, str) else [int(v) for v in rect]
        if len(values) != 4:
            raise ValueError(f'force-walkable-rectは x0,y0,x1,y1 で指定してください: {rect}')
        rx0, ry0, rx1, ry1 = values
        rx0, ry0 = max(0, rx0), max(0, ry0)
        rx1, ry1 = min(gray.shape[1], rx1), min(gray.shape[0], ry1)
        if rx0 < rx1 and ry0 < ry1:
            passage[ry0:ry1, rx0:rx1] = True

    binary = np.where(passage, 255, 0).astype(np.uint8)
    return BinarizeResult(gray, ink, walls, passage, binary, (x0, y0, x1, y1), start_local)


def apply_block_rectangles(binary: np.ndarray, rectangles) -> np.ndarray:
    """2値地図上の矩形領域を通行不可(黒)にする。

    矩形は切り抜き後のローカル座標 x0,y0,x1,y1 で指定する。
    建築図面の扉開口から部屋へ通路領域が漏れる場合の確実な補正に使う。
    """
    result = binary.copy()
    h, w = result.shape
    for rect in rectangles or []:
        if isinstance(rect, str):
            values = [int(v.strip()) for v in rect.split(',')]
        else:
            values = [int(v) for v in rect]
        if len(values) != 4:
            raise ValueError(f'block-rectは x0,y0,x1,y1 で指定してください: {rect}')
        x0, y0, x1, y1 = values
        x0, y0 = max(0, x0), max(0, y0)
        x1, y1 = min(w, x1), min(h, y1)
        if x0 < x1 and y0 < y1:
            result[y0:y1, x0:x1] = 0
    return result


def load_preprocessed_binary(path: str | Path, threshold: int = 128) -> np.ndarray:
    gray = load_gray(path)
    return np.where(gray >= threshold, 255, 0).astype(np.uint8)


def create_pf_maps(binary: np.ndarray, erosion_radius: int = 1):
    passage = binary == 255
    radius = max(0, int(erosion_radius))
    if radius > 0:
        yy, xx = np.ogrid[-radius:radius + 1, -radius:radius + 1]
        disk = xx * xx + yy * yy <= radius * radius
        passage_pf = ndimage.binary_erosion(passage, disk)
    else:
        passage_pf = passage
    binary_pf = np.where(passage_pf, 255, 0).astype(np.uint8)
    distance = ndimage.distance_transform_edt(passage_pf)
    maximum = float(distance.max())
    if maximum > 0:
        distance /= maximum
    return binary_pf, distance
