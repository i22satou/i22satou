##読み込んだ地図を自動で2値画像に変換してtest7の動作をするプログラム
##今現在は正しく動作しない(地図の2値化が失敗する)
import numpy as np
import pandas as pd
import os
import json

os.environ.setdefault("MPLCONFIGDIR", "/private/tmp/matplotlib")

import matplotlib.pyplot as plt
try:
    import cv2
    HAS_CV2 = True
except ImportError:
    cv2 = None
    HAS_CV2 = False
import glob
import collections
import time
import argparse
import threading
from pathlib import Path
try:
    import japanize_matplotlib #日本語用のライブラリ
except ImportError:
    japanize_matplotlib = None
from scipy.signal import find_peaks
from scipy import ndimage
from PIL import Image
try:
    from watchdog.observers import Observer
    from watchdog.events import FileSystemEventHandler
    HAS_WATCHDOG = True
except ImportError:
    Observer = None
    FileSystemEventHandler = object
    HAS_WATCHDOG = False
try:
    from ahrs.filters import Madgwick
    from scipy.spatial.transform import Rotation as R
    HAS_AHRS = True
except ImportError:
    Madgwick = None
    R = None
    HAS_AHRS = False

# ============================================================
# 1. 定数・スケールの設定
# ============================================================
BASE_DIR = Path(__file__).resolve().parent
DEFAULT_MAP_CONFIG = BASE_DIR / "map_configs" / "l_map.json"

REAL_LENGTH_M = 8.75
MAP_LENGTH_P  = 350.0
M_TO_PIXEL    = MAP_LENGTH_P / REAL_LENGTH_M  # 40 px/m
TARGET_DISTANCE_PX = MAP_LENGTH_P * 2.0       # L字の横350px + 縦350px
DEFAULT_STEP_GAIN = 1.12                      # PFの壁判定で削られる分を少し補正
USE_L_HEADING_CORRECTION = True
WALL_DILATION_PX = 5
KEEP_START_COMPONENT = True
LINE_MAP_DARK_RATIO = 0.20
LINE_DARK_THRESHOLD = 220
LINE_MIN_WALL_COMPONENT_AREA = 80
MAP_CROP = None

# 初期位置：横通路の左端中央
START_X = 70.0
START_Y = 250.0
L_TURN_START_X = 310.0
L_TURN_END_X   = 340.0
HEADING_EAST   = 0.0
HEADING_SOUTH  = np.pi / 2

# ============================================================
# 2. パーティクルフィルタのパラメータ
# ============================================================
N_PARTICLES  = 500
SIGMA_STEP   = 0.5
SIGMA_ANGLE  = np.deg2rad(6)

STEP_PEAK_HEIGHT  = 0.2
STEP_MIN_INTERVAL = 6

WEINBERG_K = 0.35
MIN_STEP_M = 0.25
MAX_STEP_M = 1.00

# SmartPDR風のステップ信号・歩幅推定パラメータ
HPF_ALPHA        = 0.90
LPF_WINDOW       = 5
# 論文値は peak=0.5, peak-to-peak=1.0 だが、手元ログでは少し厳しいため調整。
SMART_PEAK_THR   = 0.35
SMART_PP_THR     = 0.6
SMART_SLOPE_WIN  = 2
SMART_STEP_TAU   = 3.230
ROOT_BETA        = 1.479
ROOT_GAMMA       = -1.259
LOG_BETA         = 1.131
LOG_GAMMA        = 0.159

# SmartPDR論文の方位融合パラメータ。磁気センサ列がある場合だけ使う。
HCOR_THR  = np.deg2rad(5)
HMAG_THR  = np.deg2rad(2)
W_PREV    = 2.0
W_MAG     = 1.0
W_GYRO    = 2.0

SMOOTH_WINDOW  = 5
RECOVERY_SIGMA = 8.0
ANGLE_DECAY    = 0.999
MAX_DT         = 1.0

# ============================================================
# 3. ユーティリティ関数
# ============================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description="SmartPDR風のステップ検出・歩幅推定を加えたPDR推定を行います。"
    )
    parser.add_argument(
        "--map-config",
        type=Path,
        default=DEFAULT_MAP_CONFIG,
        help="地図ごとの設定JSON。未指定ならL字地図用の設定を読み込みます。",
    )
    parser.add_argument(
        "--data-dir",
        type=Path,
        default=None,
        help="pdr_log_*.csv があるディレクトリ。指定するとJSONの値を上書きします。",
    )
    parser.add_argument(
        "--map",
        type=Path,
        default=None,
        help="使用するマップ画像。指定するとJSONの値を上書きします。",
    )
    parser.add_argument(
        "--save",
        type=Path,
        default=None,
        help="結果画像の保存先。指定するとPNGなどで保存します。",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="グラフウィンドウを表示しません。--save と併用すると便利です。",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="乱数シード。結果を再現したいときに指定します。",
    )
    parser.add_argument(
        "--target-distance-px",
        type=float,
        default=None,
        help="歩幅を校正する目標移動距離(px)。指定するとJSONの値を上書きします。",
    )
    parser.add_argument(
        "--no-step-calibration",
        action="store_true",
        help="SmartPDR歩幅の総距離校正を行いません。",
    )
    parser.add_argument(
        "--step-gain",
        type=float,
        default=None,
        help="校正後の歩幅に掛ける追加倍率。指定するとJSONの値を上書きします。",
    )
    parser.add_argument(
        "--no-watch",
        action="store_true",
        help="CSVフォルダ監視を行わず、1回だけ描画して終了します。",
    )
    parser.add_argument(
        "--map-mode",
        choices=["auto", "fixed", "otsu", "adaptive", "line"],
        default=None,
        help=(
            "地図画像から通路/壁を判定する方法。lineは白地に黒線の平面図向け。auto/otsu/adaptiveは白黒以外の画像向け、"
            "fixedは従来通り127で2値化します。指定するとJSONの値を上書きします。"
        ),
    )
    parser.add_argument(
        "--wall-dilation-px",
        type=int,
        default=None,
        help="lineモードで黒い壁線を太らせるピクセル数。指定するとJSONの値を上書きします。",
    )
    parser.add_argument(
        "--line-dark-threshold",
        type=int,
        default=None,
        help="lineモードで壁線候補とみなす濃さのしきい値。大きいほど薄い線や文字も拾います。",
    )
    parser.add_argument(
        "--line-min-wall-area",
        type=int,
        default=None,
        help="lineモードで壁候補として残す黒成分の最小面積。大きいほど文字や数字を消しやすくなります。",
    )
    parser.add_argument(
        "--map-crop",
        type=str,
        default=None,
        help="地図画像の切り抜き範囲 x0,y0,x1,y1。指定するとJSONの値を上書きします。",
    )
    parser.add_argument(
        "--keep-start-component",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="開始位置とつながる通路領域だけを残すかどうか。指定するとJSONの値を上書きします。",
    )
    parser.add_argument(
        "--save-binary-map",
        type=Path,
        default=None,
        help="自動判定した白黒地図を確認用に保存します。",
    )
    return parser.parse_args()


def resolve_config_path(path):
    path = Path(path).expanduser()
    if not path.is_absolute():
        path = (Path.cwd() / path).resolve()
    return path


def resolve_config_value_path(value, config_dir):
    if value is None:
        return None
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = (config_dir / path).resolve()
    return path


def load_map_config(config_path):
    config_path = resolve_config_path(config_path)
    if not config_path.exists():
        raise FileNotFoundError(f"地図設定JSONが見つかりません: {config_path}")

    with config_path.open("r", encoding="utf-8") as f:
        config = json.load(f)
    return config, config_path


def apply_map_config(args, config, config_path):
    global REAL_LENGTH_M, MAP_LENGTH_P, M_TO_PIXEL, TARGET_DISTANCE_PX
    global DEFAULT_STEP_GAIN, START_X, START_Y
    global L_TURN_START_X, L_TURN_END_X, HEADING_EAST, HEADING_SOUTH
    global USE_L_HEADING_CORRECTION, WALL_DILATION_PX, KEEP_START_COMPONENT

    config_dir = config_path.parent

    REAL_LENGTH_M = float(config.get("real_length_m", REAL_LENGTH_M))
    MAP_LENGTH_P = float(config.get("map_length_px", MAP_LENGTH_P))
    M_TO_PIXEL = float(config.get("scale_px_per_m", MAP_LENGTH_P / REAL_LENGTH_M))
    TARGET_DISTANCE_PX = float(config.get("target_distance_px", MAP_LENGTH_P * 2.0))
    DEFAULT_STEP_GAIN = float(config.get("step_gain", DEFAULT_STEP_GAIN))
    WALL_DILATION_PX = int(config.get("wall_dilation_px", WALL_DILATION_PX))
    KEEP_START_COMPONENT = bool(config.get("keep_start_component", KEEP_START_COMPONENT))

    start = config.get("start", {})
    START_X = float(start.get("x", START_X))
    START_Y = float(start.get("y", START_Y))

    heading_config = config.get("heading_correction", {})
    USE_L_HEADING_CORRECTION = heading_config.get("type", "l_shape") == "l_shape"
    L_TURN_START_X = float(heading_config.get("turn_start_x", L_TURN_START_X))
    L_TURN_END_X = float(heading_config.get("turn_end_x", L_TURN_END_X))
    HEADING_EAST = np.deg2rad(float(heading_config.get("heading_before_deg", 0.0)))
    HEADING_SOUTH = np.deg2rad(float(heading_config.get("heading_after_deg", 90.0)))

    data_dir = args.data_dir
    if data_dir is None:
        data_dir = resolve_config_value_path(config.get("data_dir"), config_dir)
    else:
        data_dir = args.data_dir.expanduser().resolve()
    if data_dir is None:
        data_dir = BASE_DIR

    map_path = args.map
    if map_path is None:
        map_path = resolve_config_value_path(config.get("map_image"), config_dir)
    else:
        map_path = args.map.expanduser().resolve()
    if map_path is None:
        map_path = data_dir / "L_map.png"

    args.data_dir = data_dir
    args.map = map_path
    args.target_distance_px = (
        TARGET_DISTANCE_PX if args.target_distance_px is None
        else args.target_distance_px
    )
    args.step_gain = DEFAULT_STEP_GAIN if args.step_gain is None else args.step_gain
    args.map_mode = config.get("map_mode", "auto") if args.map_mode is None else args.map_mode
    if args.wall_dilation_px is not None:
        WALL_DILATION_PX = args.wall_dilation_px
    if args.keep_start_component is not None:
        KEEP_START_COMPONENT = args.keep_start_component

    print(f"地図設定JSON: {config_path}")
    print(f"使用地図: {args.map}")
    print(f"CSVフォルダ: {args.data_dir}")
    print(f"開始位置: x={START_X:.1f}, y={START_Y:.1f}")
    print(f"縮尺: {M_TO_PIXEL:.2f} px/m")
    print(f"地図2値化モード: {args.map_mode}")


def compute_acc_magnitude(df):
    return np.sqrt(df['acc_x']**2 + df['acc_y']**2 + df['acc_z']**2)


def compute_step_acceleration(acc_mag: pd.Series):
    """
    SmartPDRの式(4)-(6)に対応する処理。
    本来はGCSの鉛直加速度を使うが、手元CSVには姿勢/磁気列がないため
    合成加速度にHPF+移動平均LPFをかけて歩行由来の信号を作る。
    """
    gravity = np.zeros(len(acc_mag))
    values = acc_mag.to_numpy()
    if len(values) == 0:
        return pd.Series(dtype=float)

    gravity[0] = values[0]
    for i in range(1, len(values)):
        gravity[i] = HPF_ALPHA * gravity[i - 1] + (1 - HPF_ALPHA) * values[i]

    high_passed = values - gravity
    return pd.Series(high_passed).rolling(
        LPF_WINDOW, center=True, min_periods=1
    ).mean()


def detect_steps_smartpdr(step_acc: pd.Series):
    """
    SmartPDRの式(7)(8)を実装しやすい形にしたステップ検出。
    1. 周辺より高いピーク
    2. 前後の谷との差が十分大きい
    3. ピーク前は上昇、ピーク後は下降
    の3条件を満たす点をステップとする。
    """
    values = step_acc.to_numpy()
    peaks, _ = find_peaks(
        values,
        height=SMART_PEAK_THR,
        distance=STEP_MIN_INTERVAL,
    )

    valid_peaks = []
    valley_indices = []
    search_win = max(STEP_MIN_INTERVAL, 8)
    for peak in peaks:
        left_start = max(0, peak - search_win)
        right_end = min(len(values), peak + search_win + 1)
        if peak <= left_start or peak + 1 >= right_end:
            continue

        left_segment = values[left_start:peak]
        right_segment = values[peak + 1:right_end]
        if len(left_segment) == 0 or len(right_segment) == 0:
            continue

        left_valley = left_start + int(np.argmin(left_segment))
        right_valley = peak + 1 + int(np.argmin(right_segment))
        peak_to_peak = min(
            values[peak] - values[left_valley],
            values[peak] - values[right_valley],
        )
        if peak_to_peak < SMART_PP_THR:
            continue

        front_start = max(1, peak - SMART_SLOPE_WIN)
        back_end = min(len(values) - 1, peak + SMART_SLOPE_WIN)
        front_slope = np.mean(np.diff(values[front_start - 1:peak + 1]))
        back_slope = np.mean(np.diff(values[peak:back_end + 1]))
        if front_slope <= 0 or back_slope >= 0:
            continue

        valid_peaks.append(peak)
        valley_indices.append(left_valley)

    return np.array(valid_peaks, dtype=int), np.array(valley_indices, dtype=int), step_acc


def estimate_step_length_px(acc_mag: pd.Series, center_idx: int, window: int = 10):
    start     = max(0, center_idx - window)
    end       = min(len(acc_mag), center_idx + window + 1)
    segment   = acc_mag.iloc[start:end]
    amplitude = max(segment.max() - segment.min(), 1e-6)
    step_m    = WEINBERG_K * (amplitude ** 0.25)
    step_m    = np.clip(step_m, MIN_STEP_M, MAX_STEP_M)
    return step_m * M_TO_PIXEL


def estimate_smartpdr_step_length_px(step_acc: pd.Series, peak_idx: int, valley_idx: int):
    """
    SmartPDRの式(23)-(28)に対応する動的歩幅推定。
    ピークと直前の谷の差が小さい時は4乗根、大きい時はlogモデルを使う。
    """
    impact = max(float(step_acc.iloc[peak_idx] - step_acc.iloc[valley_idx]), 1e-6)
    if impact < SMART_STEP_TAU:
        step_m = ROOT_BETA * (impact ** 0.25) + ROOT_GAMMA
    else:
        step_m = LOG_BETA * np.log(impact) + LOG_GAMMA
    step_m = np.clip(step_m, MIN_STEP_M, MAX_STEP_M)
    return step_m * M_TO_PIXEL


def get_yaw_rate(gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z):
    norm = max(np.sqrt(acc_x**2 + acc_y**2 + acc_z**2), 1e-6)
    ax, ay, az = acc_x/norm, acc_y/norm, acc_z/norm
    pitch = np.arctan2(ax, np.sqrt(ay**2 + az**2))
    roll  = np.arctan2(ay, np.sqrt(ax**2 + az**2))
    return (  gyro_z * np.cos(roll) * np.cos(pitch)
            - gyro_y * np.sin(roll)
            + gyro_x * np.sin(pitch) * np.cos(roll))


def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def angle_diff(a, b):
    return normalize_angle(a - b)


def weighted_angle_mean(angles, weights):
    sin_sum = np.sum(np.sin(angles) * weights)
    cos_sum = np.sum(np.cos(angles) * weights)
    return np.arctan2(sin_sum, cos_sum)


def correct_heading_with_l_map(x, sensor_heading):
    """
    L_map.png専用の簡易マップマッチング。
    横通路では東向き、曲がり角以降では南向きへ方位を寄せる。
    """
    if not USE_L_HEADING_CORRECTION:
        return sensor_heading

    if x <= L_TURN_START_X:
        map_heading = HEADING_EAST
        map_weight = 3.0
    elif x >= L_TURN_END_X:
        map_heading = HEADING_SOUTH
        map_weight = 4.0
    else:
        ratio = (x - L_TURN_START_X) / (L_TURN_END_X - L_TURN_START_X)
        map_heading = weighted_angle_mean(
            np.array([HEADING_EAST, HEADING_SOUTH]),
            np.array([1.0 - ratio, ratio]),
        )
        map_weight = 3.0

    return weighted_angle_mean(
        np.array([sensor_heading, map_heading]),
        np.array([1.0, map_weight]),
    )


def has_magnetometer(df):
    return {'mag_x', 'mag_y', 'mag_z'}.issubset(df.columns)


def get_mag_heading(row):
    """
    SmartPDRの式(9)-(12)に相当する簡易版。
    CSVに磁気列がある場合だけ、加速度から推定したroll/pitchで傾き補正する。
    """
    norm = max(np.sqrt(row.acc_x**2 + row.acc_y**2 + row.acc_z**2), 1e-6)
    ax, ay, az = row.acc_x / norm, row.acc_y / norm, row.acc_z / norm
    pitch = np.arctan2(ax, np.sqrt(ay**2 + az**2))
    roll = np.arctan2(ay, np.sqrt(ax**2 + az**2))

    mx = row.mag_x * np.cos(pitch) + row.mag_z * np.sin(pitch)
    my = (
        row.mag_x * np.sin(roll) * np.sin(pitch)
        + row.mag_y * np.cos(roll)
        - row.mag_z * np.sin(roll) * np.cos(pitch)
    )
    return normalize_angle(np.arctan2(-my, mx))


def fuse_heading(prev_heading, mag_heading, gyro_heading, prev_mag_heading):
    """
    SmartPDRの式(17)-(19)の考え方を実装。
    磁気とジャイロが近いか、磁気が急変しているかで使う情報源を切り替える。
    """
    if mag_heading is None or prev_mag_heading is None:
        return gyro_heading

    h_cor = abs(angle_diff(mag_heading, gyro_heading))
    h_mag = abs(angle_diff(mag_heading, prev_mag_heading))

    if h_cor <= HCOR_THR and h_mag <= HMAG_THR:
        return weighted_angle_mean(
            np.array([prev_heading, mag_heading, gyro_heading]),
            np.array([W_PREV, W_MAG, W_GYRO]),
        )
    if h_cor <= HCOR_THR and h_mag > HMAG_THR:
        return weighted_angle_mean(
            np.array([mag_heading, gyro_heading]),
            np.array([W_MAG, W_GYRO]),
        )
    if h_cor > HCOR_THR and h_mag <= HMAG_THR:
        return prev_heading

    return weighted_angle_mean(
        np.array([prev_heading, gyro_heading]),
        np.array([W_PREV, W_GYRO]),
    )


def resample_if_needed(particles, weights):
    neff = 1.0 / (np.sum(weights**2) + 1e-300)
    if neff < N_PARTICLES / 2:
        indices   = np.random.choice(N_PARTICLES, size=N_PARTICLES, p=weights)
        particles = particles[indices]
        weights   = np.full(N_PARTICLES, 1.0 / N_PARTICLES)
    return particles, weights

def is_in_wall(x, y):
    ix, iy = np.clip(x.astype(int), 0, w-1), np.clip(y.astype(int), 0, h-1)
    return binary_for_pf[iy, ix] == 0  # 0は黒（壁）


def path_hits_wall(old_particles, new_particles):
    n = len(old_particles)
    x0 = old_particles[:, 0]
    y0 = old_particles[:, 1]
    x1 = new_particles[:, 0]
    y1 = new_particles[:, 1]
    dx = x1 - x0
    dy = y1 - y0
    steps = np.maximum(
        np.abs(dx),
        np.abs(dy)
    ).astype(int)
    hit = np.zeros(n, dtype=bool)
    max_steps = steps.max()

    for s in range(max_steps + 1):
        ratio = s / np.maximum(steps, 1)
        xs = x0 + dx * ratio
        ys = y0 + dy * ratio
        hit |= is_in_wall(xs, ys)
    return hit


def validate_log(df, file_name):
    required = ['timestamp', 'acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z']
    missing = [col for col in required if col not in df.columns]
    if missing:
        raise ValueError(f"{file_name} に必要な列がありません: {', '.join(missing)}")

    df = df.dropna(subset=required).reset_index(drop=True)
    df = df[df['timestamp'].diff().fillna(1) > 0].reset_index(drop=True)
    return df


def _disk_kernel(size):
    if HAS_CV2:
        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
    return np.ones((size, size), dtype=bool)


def _threshold_gray(gray, mode):
    if mode == "fixed":
        return gray > 127

    if HAS_CV2 and mode in {"auto", "otsu"}:
        _, th = cv2.threshold(
            gray.astype(np.uint8),
            0,
            255,
            cv2.THRESH_BINARY + cv2.THRESH_OTSU,
        )
        return th == 255

    if HAS_CV2 and mode == "adaptive":
        block_size = max(15, (min(gray.shape) // 20) | 1)
        th = cv2.adaptiveThreshold(
            gray.astype(np.uint8),
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            block_size,
            2,
        )
        return th == 255

    threshold = float(np.mean(gray))
    if mode in {"auto", "otsu"}:
        # OpenCVがない場合の簡易Otsu。画像全体の明暗が分かれていれば十分効く。
        hist, _ = np.histogram(gray.ravel(), bins=256, range=(0, 256))
        total = gray.size
        sum_total = np.dot(np.arange(256), hist)
        sum_bg = 0.0
        weight_bg = 0.0
        best_var = -1.0
        for t in range(256):
            weight_bg += hist[t]
            if weight_bg == 0:
                continue
            weight_fg = total - weight_bg
            if weight_fg == 0:
                break
            sum_bg += t * hist[t]
            mean_bg = sum_bg / weight_bg
            mean_fg = (sum_total - sum_bg) / weight_fg
            between_var = weight_bg * weight_fg * (mean_bg - mean_fg) ** 2
            if between_var > best_var:
                best_var = between_var
                threshold = t
    return gray > threshold


def _clean_passage_mask(mask):
    if HAS_CV2:
        mask_u8 = np.where(mask, 255, 0).astype(np.uint8)
        close_kernel = _disk_kernel(5)
        open_kernel = _disk_kernel(3)
        mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_CLOSE, close_kernel, iterations=1)
        mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_OPEN, open_kernel, iterations=1)
        return mask_u8 == 255

    closed = ndimage.binary_closing(mask, structure=_disk_kernel(5), iterations=1)
    opened = ndimage.binary_opening(closed, structure=_disk_kernel(3), iterations=1)
    return opened


def _build_line_map_passage_mask(gray):
    """
    白地に黒/灰色の線で壁が描かれた平面図向け。
    黒線を壁として抽出し、少し太らせてから、それ以外を通路候補にする。
    """
    wall_mask = gray <= 220

    if HAS_CV2:
        wall_u8 = np.where(wall_mask, 255, 0).astype(np.uint8)
        kernel = _disk_kernel(max(1, WALL_DILATION_PX))
        wall_u8 = cv2.dilate(wall_u8, kernel, iterations=1)
        wall_u8 = cv2.morphologyEx(wall_u8, cv2.MORPH_CLOSE, _disk_kernel(3), iterations=1)
        wall_mask = wall_u8 == 255
    else:
        wall_mask = ndimage.binary_dilation(
            wall_mask,
            structure=_disk_kernel(max(1, WALL_DILATION_PX)),
            iterations=1,
        )
        wall_mask = ndimage.binary_closing(wall_mask, structure=_disk_kernel(3), iterations=1)

    return ~wall_mask


def _ensure_start_is_passage(mask):
    sx = int(np.clip(round(START_X), 0, mask.shape[1] - 1))
    sy = int(np.clip(round(START_Y), 0, mask.shape[0] - 1))
    y0, y1 = max(0, sy - 5), min(mask.shape[0], sy + 6)
    x0, x1 = max(0, sx - 5), min(mask.shape[1], sx + 6)
    start_ratio = np.mean(mask[y0:y1, x0:x1])

    # 開始位置は必ず通路という前提を使い、白黒の向きを自動で合わせる。
    if start_ratio < 0.5:
        mask = ~mask
    return mask


def _keep_start_component(mask):
    sx = int(np.clip(round(START_X), 0, mask.shape[1] - 1))
    sy = int(np.clip(round(START_Y), 0, mask.shape[0] - 1))
    labels, num = ndimage.label(mask)
    if num == 0:
        return mask

    start_label = labels[sy, sx]
    if start_label == 0:
        # 開始点が細い線やノイズ処理で外れた場合、近傍の最大成分を使う。
        y0, y1 = max(0, sy - 10), min(mask.shape[0], sy + 11)
        x0, x1 = max(0, sx - 10), min(mask.shape[1], sx + 11)
        near_labels = labels[y0:y1, x0:x1]
        near_labels = near_labels[near_labels > 0]
        if len(near_labels) > 0:
            start_label = np.bincount(near_labels).argmax()

    if start_label == 0:
        sizes = ndimage.sum(mask, labels, index=np.arange(1, num + 1))
        start_label = int(np.argmax(sizes) + 1)
    return labels == start_label


def load_map_as_passage_mask(img_path, mode):
    """
    任意の地図画像を、通路=255・壁=0の2値画像へ変換する。
    auto/otsuでは明るい領域を通路候補とし、開始地点が通路になるよう必要なら反転する。
    """
    if not img_path.exists():
        raise FileNotFoundError(f"マップ画像を読み込めません: {img_path}")

    if HAS_CV2:
        img_bgr = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
        if img_bgr is None:
            raise FileNotFoundError(f"マップ画像を読み込めません: {img_path}")
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    else:
        gray = np.array(Image.open(img_path).convert("L"))

    dark_ratio = float(np.mean(gray <= 127))
    actual_mode = "line" if mode == "auto" and dark_ratio < LINE_MAP_DARK_RATIO else mode

    if actual_mode == "line":
        passage_mask = _build_line_map_passage_mask(gray)
    else:
        passage_mask = _threshold_gray(gray, actual_mode)
        passage_mask = _ensure_start_is_passage(passage_mask)
        passage_mask = _clean_passage_mask(passage_mask)

    if KEEP_START_COMPONENT:
        passage_mask = _keep_start_component(passage_mask)

    white_ratio = float(np.mean(passage_mask))
    print(
        f"地図2値化: requested={mode}, actual={actual_mode}, "
        f"dark_ratio={dark_ratio:.3f}, passage_ratio={white_ratio:.3f}, "
        f"wall_dilation_px={WALL_DILATION_PX}"
    )

    binary = np.where(passage_mask, 255, 0).astype(np.uint8)

    if HAS_CV2:
        binary_for_pf = cv2.erode(binary, _disk_kernel(5), iterations=4)
        dist_map = cv2.distanceTransform(binary_for_pf, cv2.DIST_L2, 5)
    else:
        eroded = ndimage.binary_erosion(
            binary == 255,
            structure=np.ones((5, 5), dtype=bool),
            iterations=4,
        )
        binary_for_pf = np.where(eroded, 255, 0).astype(np.uint8)
        dist_map = ndimage.distance_transform_edt(binary_for_pf == 255)

    dist_max = dist_map.max()
    if dist_max > 0:
        dist_map = dist_map / dist_max

    return binary, binary_for_pf, dist_map

# ============================================================
# 4. マップの読み込みと前処理
# ============================================================
args = parse_args()
map_config, map_config_path = load_map_config(args.map_config)
apply_map_config(args, map_config, map_config_path)
data_dir = args.data_dir
img_path = args.map
if args.seed is not None:
    np.random.seed(args.seed)

binary, binary_for_pf, dist_map = load_map_as_passage_mask(img_path, args.map_mode)

if args.save_binary_map is not None:
    binary_map_path = args.save_binary_map.resolve()
    binary_map_path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(binary).save(binary_map_path)
    print(f"白黒化した地図を保存しました: {binary_map_path}")

h, w = binary.shape

# ============================================================
# 5. リアルタイム描画関数
# ============================================================
plt.ion()
fig, ax = plt.subplots(figsize=(10, 8))

processed_files = set()
redraw_requested = threading.Event()


def redraw_all_paths():

    ax.clear()
    ax.imshow(binary, cmap='gray')
    
    ax.set_xlim(0, w)
    ax.set_ylim(h, 0)

    file_list = glob.glob(str(data_dir / "pdr_log_*.csv"))
    file_list.sort()

    if not file_list:
        print("CSVファイルなし")
        return

    start_overall = time.time()

    # ============================================================
    # メインループ（ファイルごと）
    # ============================================================
    for file_name in file_list:

        df = pd.read_csv(file_name)

        df = validate_log(df, file_name)
        if len(df) < 2:
            print(f"[{Path(file_name).name}] 有効なデータが少ないためスキップします。")
            continue

        df['acc_mag']    = compute_acc_magnitude(df)
        df['step_acc']   = compute_step_acceleration(df['acc_mag'])
        step_indices, valley_indices, _ = detect_steps_smartpdr(df['step_acc'])
        valley_by_step = dict(zip(step_indices, valley_indices))

        print(f"[{Path(file_name).name}] 総行数: {len(df)}  検出ステップ数: {len(step_indices)}")

        if len(step_indices) > 0:
            raw_step_lengths = [
                estimate_smartpdr_step_length_px(df['step_acc'], peak, valley)
                for peak, valley in zip(step_indices, valley_indices)
            ]
            raw_total_dist = sum(raw_step_lengths)
            step_scale = 1.0
            if not args.no_step_calibration and raw_total_dist > 0:
                step_scale = args.target_distance_px / raw_total_dist

            step_lengths = [length * step_scale * args.step_gain for length in raw_step_lengths]
            step_length_by_step = dict(zip(step_indices, step_lengths))
            total_dist = sum(step_lengths)
            print(f"  SmartPDR歩幅推定 (px): 平均={np.mean(step_lengths):.1f}  "
                  f"≈ 平均{np.mean(step_lengths)/M_TO_PIXEL:.2f}m/歩")
            print(f"  推定総移動距離: {total_dist:.0f}px  "
                  f"(校正前: {raw_total_dist:.0f}px, scale={step_scale:.2f}, "
                  f"gain={args.step_gain:.2f}, 目標: {args.target_distance_px:.0f}px)")
        else:
            print("  ステップが検出されなかったため、このログは描画されません。")
            continue

        # --- PF 初期化 ---
        particles = np.column_stack([
            START_X + np.random.normal(0, 2, N_PARTICLES),
            START_Y + np.random.normal(0, 2, N_PARTICLES)
        ])
        weights             = np.full(N_PARTICLES, 1.0 / N_PARTICLES)
        estimated_positions = []
        pos_buffer          = collections.deque(maxlen=SMOOTH_WINDOW)
        gyro_angle          = 0.0
        heading             = 0.0
        prev_mag_heading    = None
        heading_history     = np.zeros(len(df))
        step_set            = set(step_indices)
        mag_enabled         = has_magnetometer(df)

        # デバッグ用カウンタ
        extinction_count = 0   # 全滅回数
        step_count       = 0   # 処理したステップ数

        madgwick = Madgwick() if HAS_AHRS else None
        Q = np.tile([1., 0., 0., 0.], (len(df), 1)) if HAS_AHRS else None
        
        # --- 行ループ ---
        for i in range(1, len(df)):
            row      = df.iloc[i]
            prev_row = df.iloc[i - 1]

            dt = row['timestamp'] - prev_row['timestamp']
            if dt <= 0 or dt > MAX_DT:
                continue

            if HAS_AHRS:
                gyro = np.array([
                    row['gyro_x'],
                    row['gyro_y'],
                    row['gyro_z']
                ])

                acc = np.array([
                    row['acc_x'],
                    row['acc_y'],
                    row['acc_z']
                ])

                if mag_enabled:
                    mag = np.array([
                        row['mag_x'],
                        row['mag_y'],
                        row['mag_z']
                    ])

                    Q[i] = madgwick.updateMARG(
                        q=Q[i - 1],
                        gyr=gyro,
                        acc=acc,
                        mag=mag
                    )
                else:
                    Q[i] = madgwick.updateIMU(
                        q=Q[i - 1],
                        gyr=gyro,
                        acc=acc
                    )

                rot = R.from_quat([
                    Q[i][1],
                    Q[i][2],
                    Q[i][3],
                    Q[i][0]
                ])
                gyro_angle = normalize_angle(rot.as_euler('xyz')[2])
            else:
                yaw_rate = get_yaw_rate(
                    row['gyro_x'], row['gyro_y'], row['gyro_z'],
                    row['acc_x'],  row['acc_y'],  row['acc_z']
                )
                gyro_angle = normalize_angle(gyro_angle * ANGLE_DECAY + yaw_rate * dt)

            mag_heading = get_mag_heading(row) if mag_enabled else None
            heading = normalize_angle(
                fuse_heading(heading, mag_heading, gyro_angle, prev_mag_heading)
            )
            heading_history[i] = heading
            if mag_heading is not None:
                prev_mag_heading = mag_heading

            if i not in step_set:
                continue

            step_count += 1
            valley_idx = valley_by_step.get(i, i)
            step_px = step_length_by_step.get(
                i,
                estimate_smartpdr_step_length_px(df['step_acc'], i, valley_idx),
            )
            step_heading = heading_history[i]
            prev_step = 0

            if step_count >= 2:
                prev_step = step_indices[
                max(0, np.where(step_indices == i)[0][0] - 1)
                ]
                
            segment_heading = heading_history[prev_step:i + 1]
            
            step_heading = weighted_angle_mean(
                segment_heading,
                np.ones(len(segment_heading))
            )
            ref_x = np.average(particles[:, 0], weights=weights)
            step_heading = correct_heading_with_l_map(ref_x, step_heading)

            # --- 1. 予測 (Prediction) ---
            noise_dist  = np.random.normal(0, SIGMA_STEP,  N_PARTICLES)
            noise_angle = np.random.normal(0, SIGMA_ANGLE, N_PARTICLES)
            move        = step_px + noise_dist
            p_angle     = step_heading + noise_angle
            
            new_particles = particles.copy()
            new_particles[:, 0] += move * np.cos(p_angle)
            new_particles[:, 1] += move * np.sin(p_angle)
            
            # --- 2. 壁衝突判定 & 重み更新 (Weighting) ---
            hit_wall = path_hits_wall(particles, new_particles)

            # 重みの計算（移動先の距離マップから取得）
            ny_idx = np.clip(new_particles[:, 1].astype(int), 0, h - 1)
            nx_idx = np.clip(new_particles[:, 0].astype(int), 0, w - 1)
            dist_values = dist_map[ny_idx, nx_idx]
            sigma_wall = 0.25
            weights = np.exp(
                -(1.0 - dist_values) ** 2 /
                (2 * sigma_wall ** 2)
            )
            particle_heading_error = angle_diff(
                p_angle,
                step_heading
            )
            weights *= np.exp(
                -(particle_heading_error ** 2) /
                (2 * SIGMA_ANGLE ** 2)
            )
            
            # 壁に当たったパーティクルの重みを0にする
            weights[hit_wall] = 0.0

            # --- 3. リサンプリング (Resampling) ---
            sum_w = weights.sum()
            if sum_w > 0:
                weights /= sum_w
                particles, weights = resample_if_needed(new_particles, weights)
            else:
                # 万が一の全滅リカバリ
                extinction_count += 1
                # 直前の平均位置を基準に再配置
                ref_x, ref_y = np.mean(particles, axis=0) 
                particles[:, 0] = ref_x + np.random.normal(0, RECOVERY_SIGMA, N_PARTICLES)
                particles[:, 1] = ref_y + np.random.normal(0, RECOVERY_SIGMA, N_PARTICLES)
                

                # 通路内に収まるまで再配置を試みる（最大50回）
                for _ in range(50):
                    new_x = ref_x + np.random.normal(0, RECOVERY_SIGMA, N_PARTICLES)
                    new_y = ref_y + np.random.normal(0, RECOVERY_SIGMA, N_PARTICLES)
                    nx_i  = np.clip(new_x.astype(int), 0, w - 1)
                    ny_i  = np.clip(new_y.astype(int), 0, h - 1)
                    valid = binary[ny_i, nx_i] == 255  # 通路内かどうか
                    if valid.sum() > N_PARTICLES // 4:  # 25%以上通路内なら採用
                        # 通路外のパーティクルを通路内のもので補完
                        valid_idx   = np.where(valid)[0]
                        invalid_idx = np.where(~valid)[0]
                        if len(invalid_idx) > 0:
                            fill = np.random.choice(valid_idx, size=len(invalid_idx))
                            new_x[invalid_idx] = new_x[fill]
                            new_y[invalid_idx] = new_y[fill]
                        particles[:, 0] = new_x
                        particles[:, 1] = new_y
                        break
                weights = np.full(N_PARTICLES, 1.0 / N_PARTICLES)

            # 4. 推定位置
            raw_x = np.average(particles[:, 0], weights=weights)
            raw_y = np.average(particles[:, 1], weights=weights)
            pos_buffer.append((raw_x, raw_y))
            avg_pos = np.mean(pos_buffer, axis=0)
            estimated_positions.append(tuple(avg_pos))

        # デバッグ出力
        print(f"  処理ステップ数: {step_count}  全滅回数: {extinction_count}")
        if estimated_positions:
            last = estimated_positions[-1]
            print(f"  最終推定位置: x={last[0]:.1f}, y={last[1]:.1f}  "
                  f"(縦通路下端の目標: x≈370, y≈630)")

        # --- プロット ---
        if estimated_positions:
            pos_arr = np.array(estimated_positions)
            ax.plot(pos_arr[:, 0], pos_arr[:, 1],
                    linewidth=2, label=f'PF Path: {Path(file_name).name}')
            ax.scatter(pos_arr[0, 0], pos_arr[0, 1], s=50, zorder=5,
                       label=f'Start: {Path(file_name).name}')

# ============================================================
# 結果表示
# ============================================================
    end_overall = time.time()
    print(f"\nパーティクル数: {N_PARTICLES}")
    print(f"Overall time: {end_overall - start_overall:.2f} seconds")

    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    title = "test8: 自動地図2値化 + SmartPDR風ステップ検出 + パーティクルフィルタ"
    if japanize_matplotlib is None:
        title = "test8: auto map binarization + SmartPDR step detection + PF"
    ax.set_title(title)

    fig.subplots_adjust(right=0.72)
    fig.canvas.draw()
    fig.canvas.flush_events()

    if args.save is not None:
        save_path = args.save.resolve()
        save_path.parent.mkdir(parents=True, exist_ok=True)
        fig.set_size_inches(10, 8, forward=True)
        fig.savefig(save_path, dpi=200, bbox_inches=None)
        print(f"結果画像を保存しました: {save_path}")


# ============================================================
# watchdog監視
# ============================================================
class CSVHandler(FileSystemEventHandler):

    def request_redraw(self, path):
        csv_path = Path(path)
        if csv_path.suffix.lower() != ".csv":
            return
        if not csv_path.name.startswith("pdr_log_"):
            return

        print("\n===================================")
        print("CSV change detected")
        print(csv_path.name)
        print("===================================\n")
        redraw_requested.set()

    def on_created(self, event):
        if not event.is_directory:
            self.request_redraw(event.src_path)

    def on_modified(self, event):
        if not event.is_directory:
            self.request_redraw(event.src_path)

    def on_moved(self, event):
        if not event.is_directory:
            self.request_redraw(event.dest_path)


# ============================================================
# 初回描画
# ============================================================
redraw_all_paths()

if args.no_watch:
    if not args.no_show:
        plt.ioff()
        plt.show()
else:
    if not HAS_WATCHDOG:
        raise ImportError("watchdog がインストールされていないため監視モードを開始できません。")

    observer = Observer()
    observer.schedule(CSVHandler(), str(data_dir), recursive=False)
    observer.start()

    print("===================================")
    print("Monitoring started...")
    print(data_dir)
    print("===================================")

    try:
        while True:
            plt.pause(1)
            if redraw_requested.is_set():
                redraw_requested.clear()
                time.sleep(2)
                redraw_all_paths()

    except KeyboardInterrupt:
        observer.stop()

    observer.join()
