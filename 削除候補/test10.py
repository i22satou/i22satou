## 管理棟の前処理済み2値地図でPDR + パーティクルフィルタを実行
import numpy as np
import pandas as pd
import os
import json
import tempfile
import logging
from dataclasses import dataclass

# Ensure matplotlib config directory is writable and not hardcoded
mpl_config_dir = os.environ.get("MPLCONFIGDIR")
if not mpl_config_dir:
    mpl_config_dir = tempfile.mkdtemp(prefix="matplotlib-")
    os.environ["MPLCONFIGDIR"] = mpl_config_dir
else:
    try:
        os.makedirs(mpl_config_dir, exist_ok=True)
    except Exception:
        # fallback to temp dir
        mpl_config_dir = tempfile.mkdtemp(prefix="matplotlib-")
        os.environ["MPLCONFIGDIR"] = mpl_config_dir

import matplotlib.pyplot as plt
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
except Exception:
    Madgwick = None
    R = None
    HAS_AHRS = False

# ============================================================
# 1. 管理棟用の既定値（通常はJSONで上書き）
# ============================================================
BASE_DIR = Path(__file__).resolve().parent
DEFAULT_MAP_CONFIG = BASE_DIR / "map_configs" / "kanri_4f.json"
M_TO_PIXEL = 11.4
TARGET_DISTANCE_PX = None
DEFAULT_STEP_GAIN = 1.0
PF_EROSION_RADIUS_PX = 1
START_X = 250.0
START_Y = 230.0

# PF重み・経路優先設定
WALL_WEIGHT_SIGMA = 0.60
WALL_WEIGHT_FLOOR = 0.50
OFF_ROUTE_WEIGHT = 0.15
ROUTE_WIDTH_PX = 18.0
ROUTE_HEADING_WEIGHT = 1.5
ROUTE_POINTS = []
GYRO_UNIT = "rad"

# ============================================================
# 2. パーティクルフィルタのパラメータ
# ============================================================
N_PARTICLES  = 500
SIGMA_STEP   = 0.5
SIGMA_ANGLE  = np.deg2rad(6)

STEP_MIN_INTERVAL = 5

WEINBERG_K = 0.35
MIN_STEP_M = 0.25
MAX_STEP_M = 1.00

# SmartPDR風のステップ信号・歩幅推定パラメータ
HPF_ALPHA        = 0.90
LPF_WINDOW       = 5
SMART_PEAK_THR   = 0.20
SMART_PP_THR     = 0.35
SMART_SLOPE_WIN  = 2
SMART_STEP_TAU   = 3.230
ROOT_BETA        = 1.479
ROOT_GAMMA       = -1.259
LOG_BETA         = 1.131
LOG_GAMMA        = 0.159

HCOR_THR  = np.deg2rad(5)
HMAG_THR  = np.deg2rad(2)
W_PREV    = 2.0
W_MAG     = 1.0
W_GYRO    = 2.0

SMOOTH_WINDOW  = 2
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
        help="地図ごとの設定JSON。未指定なら管理棟4階用の設定を読み込みます。",
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
        "--pf-erosion-radius-px", type=int, default=None,
        help="PF用移動可能領域を内側へ縮める半径(px)。0なら収縮しません。",
    )
    parser.add_argument(
        "--log-level",
        type=str,
        default="INFO",
        help="ログレベル (DEBUG, INFO, WARNING, ERROR)",
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
    global M_TO_PIXEL, TARGET_DISTANCE_PX, DEFAULT_STEP_GAIN
    global START_X, START_Y, PF_EROSION_RADIUS_PX
    global WALL_WEIGHT_SIGMA, WALL_WEIGHT_FLOOR, OFF_ROUTE_WEIGHT
    global ROUTE_WIDTH_PX, ROUTE_HEADING_WEIGHT, ROUTE_POINTS, GYRO_UNIT

    config_dir = config_path.parent
    M_TO_PIXEL = float(config.get("scale_px_per_m", M_TO_PIXEL))
    target = config.get("target_distance_px")
    TARGET_DISTANCE_PX = None if target is None else float(target)
    DEFAULT_STEP_GAIN = float(config.get("step_gain", DEFAULT_STEP_GAIN))
    PF_EROSION_RADIUS_PX = int(config.get("pf_erosion_radius_px", PF_EROSION_RADIUS_PX))
    WALL_WEIGHT_SIGMA = float(config.get("wall_weight_sigma", WALL_WEIGHT_SIGMA))
    WALL_WEIGHT_FLOOR = float(config.get("wall_weight_floor", WALL_WEIGHT_FLOOR))
    OFF_ROUTE_WEIGHT = float(config.get("off_route_weight", OFF_ROUTE_WEIGHT))
    ROUTE_WIDTH_PX = float(config.get("route_width_px", ROUTE_WIDTH_PX))
    ROUTE_HEADING_WEIGHT = float(config.get("route_heading_weight", ROUTE_HEADING_WEIGHT))
    ROUTE_POINTS = [tuple(map(float, point)) for point in config.get("route_points", [])]
    GYRO_UNIT = str(config.get("gyro_unit", GYRO_UNIT)).lower()
    if GYRO_UNIT not in {"rad", "deg"}:
        raise ValueError("gyro_unit は 'rad' または 'deg' を指定してください。")

    start = config.get("start", {})
    START_X = float(start.get("x", START_X))
    START_Y = float(start.get("y", START_Y))

    data_dir = (resolve_config_value_path(config.get("data_dir"), config_dir)
                if args.data_dir is None else args.data_dir.expanduser().resolve())
    map_path = (resolve_config_value_path(config.get("map_image"), config_dir)
                if args.map is None else args.map.expanduser().resolve())
    if data_dir is None:
        raise ValueError("JSONに data_dir を指定してください。")
    if map_path is None:
        raise ValueError("JSONに map_image を指定してください。")

    args.data_dir = data_dir
    args.map = map_path
    args.step_gain = DEFAULT_STEP_GAIN if args.step_gain is None else args.step_gain
    if args.target_distance_px is None:
        args.target_distance_px = TARGET_DISTANCE_PX
    if args.pf_erosion_radius_px is not None:
        PF_EROSION_RADIUS_PX = max(0, args.pf_erosion_radius_px)

    logging.info(f"地図設定JSON: {config_path}")
    logging.info(f"使用地図: {args.map}")
    logging.info(f"CSVフォルダ: {args.data_dir}")
    logging.info(f"開始位置: x={START_X:.1f}, y={START_Y:.1f}")
    logging.info(f"縮尺: {M_TO_PIXEL:.2f} px/m")
    logging.info(f"ジャイロ単位: {GYRO_UNIT}/s")
    logging.info(f"PF収縮半径: {PF_EROSION_RADIUS_PX}px")
    logging.info(f"経路優先: {len(ROUTE_POINTS)}点, 幅={ROUTE_WIDTH_PX:.1f}px, 経路外重み={OFF_ROUTE_WEIGHT:.2f}")
    logging.info("総距離校正: " + ("無効" if args.target_distance_px is None else f"{args.target_distance_px:.1f}px"))


def compute_acc_magnitude(df):
    return np.sqrt(df['acc_x']**2 + df['acc_y']**2 + df['acc_z']**2)


def compute_step_acceleration(acc_mag: pd.Series):
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
    values = step_acc.to_numpy()
    # Attempt to make STEP_MIN_INTERVAL time-aware when possible
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

    return np.array(valid_peaks, dtype=int), np.array(valley_indices, dtype=int)


def estimate_step_length_px(acc_mag: pd.Series, center_idx: int, window: int = 10):
    start     = max(0, center_idx - window)
    end       = min(len(acc_mag), center_idx + window + 1)
    segment   = acc_mag.iloc[start:end]
    amplitude = max(segment.max() - segment.min(), 1e-6)
    step_m    = WEINBERG_K * (amplitude ** 0.25)
    step_m    = np.clip(step_m, MIN_STEP_M, MAX_STEP_M)
    return step_m * M_TO_PIXEL


def estimate_smartpdr_step_length_px(step_acc: pd.Series, peak_idx: int, valley_idx: int):
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


def nearest_route_heading(x, y):
    if len(ROUTE_POINTS) < 2:
        return None
    point = np.array([x, y], dtype=float)
    best_distance = np.inf
    best_heading = None
    for start, end in zip(ROUTE_POINTS[:-1], ROUTE_POINTS[1:]):
        a = np.array(start, dtype=float)
        b = np.array(end, dtype=float)
        segment = b - a
        denom = float(np.dot(segment, segment))
        if denom <= 1e-9:
            continue
        ratio = np.clip(np.dot(point - a, segment) / denom, 0.0, 1.0)
        nearest = a + ratio * segment
        distance = np.linalg.norm(point - nearest)
        if distance < best_distance:
            best_distance = distance
            best_heading = np.arctan2(segment[1], segment[0])
    return best_heading


def correct_heading_with_route(x, y, sensor_heading):
    route_heading = nearest_route_heading(x, y)
    if route_heading is None or ROUTE_HEADING_WEIGHT <= 0:
        return sensor_heading
    return weighted_angle_mean(
        np.array([sensor_heading, route_heading]),
        np.array([1.0, ROUTE_HEADING_WEIGHT]),
    )


def has_magnetometer(df):
    return {'mag_x', 'mag_y', 'mag_z'}.issubset(df.columns)


def get_mag_heading(row):
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
    n = len(weights)
    neff = 1.0 / (np.sum(weights**2) + 1e-300)
    if neff < n / 2:
        indices   = np.random.choice(n, size=n, p=weights)
        particles = particles[indices]
        weights   = np.full(n, 1.0 / n)
    return particles, weights


def is_in_wall(x, y):
    ix, iy = np.clip(x.astype(int), 0, w-1), np.clip(y.astype(int), 0, h-1)
    return binary_for_pf[iy, ix] == 0  # 0は黒（壁）


def path_hits_wall(old_particles, new_particles):
    # Vectorized but capped sampling to avoid pathological long loops
    n = len(old_particles)
    x0 = old_particles[:, 0]
    y0 = old_particles[:, 1]
    x1 = new_particles[:, 0]
    y1 = new_particles[:, 1]
    dx = x1 - x0
    dy = y1 - y0
    steps = np.maximum(np.abs(dx), np.abs(dy)).astype(int)
    # Cap the number of interpolation steps to keep runtime bounded
    steps = np.minimum(steps, 20)
    hit = np.zeros(n, dtype=bool)

    max_steps = steps.max()
    if max_steps <= 0:
        return hit

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


def load_preprocessed_map(img_path):
    if not img_path.exists():
        raise FileNotFoundError(f"マップ画像を読み込めません: {img_path}")
    gray = np.array(Image.open(img_path).convert("L"))
    BINARY_THRESHOLD = 128
    binary = np.where(gray >= BINARY_THRESHOLD, 255, 0).astype(np.uint8)
    passage = binary == 255
    radius = max(0, int(PF_EROSION_RADIUS_PX))
    if radius > 0:
        yy, xx = np.ogrid[-radius:radius + 1, -radius:radius + 1]
        disk = xx * xx + yy * yy <= radius * radius
        passage_pf = ndimage.binary_erosion(passage, structure=disk)
    else:
        passage_pf = passage
    binary_for_pf_local = np.where(passage_pf, 255, 0).astype(np.uint8)
    dist_map = ndimage.distance_transform_edt(passage_pf)
    maximum = float(dist_map.max())
    if maximum > 0:
        dist_map /= maximum
    logging.info(f"前処理済み地図: passage_ratio={passage.mean():.3f}, PF用={passage_pf.mean():.3f}")
    return binary, binary_for_pf_local, dist_map


def build_route_mask(shape):
    mask = np.zeros(shape, dtype=bool)
    if len(ROUTE_POINTS) < 2:
        return np.ones(shape, dtype=bool)
    for start, end in zip(ROUTE_POINTS[:-1], ROUTE_POINTS[1:]):
        x0, y0 = start
        x1, y1 = end
        count = max(2, int(np.hypot(x1 - x0, y1 - y0)) + 1)
        xs = np.rint(np.linspace(x0, x1, count)).astype(int)
        ys = np.rint(np.linspace(y0, y1, count)).astype(int)
        valid = (xs >= 0) & (xs < shape[1]) & (ys >= 0) & (ys < shape[0])
        mask[ys[valid], xs[valid]] = True
    radius = max(1, int(round(ROUTE_WIDTH_PX)))
    yy, xx = np.ogrid[-radius:radius + 1, -radius:radius + 1]
    disk = xx * xx + yy * yy <= radius * radius
    return ndimage.binary_dilation(mask, structure=disk)

# グローバルな描画・監視状態
redraw_requested = threading.Event()

# CSV 変更検出ハンドラ
class CSVHandler(FileSystemEventHandler):

    def request_redraw(self, path):
        csv_path = Path(path)
        if csv_path.suffix.lower() != ".csv":
            return
        if not csv_path.name.startswith("pdr_log_"):
            return

        logging.info("\n===================================")
        logging.info("CSV change detected: %s", csv_path.name)
        logging.info("===================================\n")
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


def redraw_all_paths():
    global binary, binary_for_pf, dist_map, h, w
    ax.clear()
    ax.imshow(binary, cmap='gray')
    ax.set_xlim(0, w)
    ax.set_ylim(h, 0)

    file_list = glob.glob(str(data_dir / "pdr_log_*.csv"))
    file_list.sort()

    if not file_list:
        logging.info("CSVファイルなし")
        return

    start_overall = time.time()

    for file_name in file_list:
        df = pd.read_csv(file_name)
        try:
            df = validate_log(df, file_name)
        except ValueError as e:
            logging.warning(str(e))
            continue

        if GYRO_UNIT == "deg":
            gyro_columns = ["gyro_x", "gyro_y", "gyro_z"]
            df[gyro_columns] = np.deg2rad(df[gyro_columns])
        if len(df) < 2:
            logging.info(f"[{Path(file_name).name}] 有効なデータが少ないためスキップします。")
            continue

        df['acc_mag']    = compute_acc_magnitude(df)
        df['step_acc']   = compute_step_acceleration(df['acc_mag'])
        step_indices, valley_indices = detect_steps_smartpdr(df['step_acc'])
        valley_by_step = dict(zip(step_indices, valley_indices))

        logging.info(f"[{Path(file_name).name}] 総行数: {len(df)}  検出ステップ数: {len(step_indices)}")

        if len(step_indices) > 0:
            raw_step_lengths = [
                estimate_smartpdr_step_length_px(df['step_acc'], peak, valley)
                for peak, valley in zip(step_indices, valley_indices)
            ]
            raw_total_dist = sum(raw_step_lengths)
            step_scale = 1.0
            if (not args.no_step_calibration and args.target_distance_px is not None and raw_total_dist > 0):
                step_scale = args.target_distance_px / raw_total_dist

            step_lengths = [length * step_scale * args.step_gain for length in raw_step_lengths]
            step_length_by_step = dict(zip(step_indices, step_lengths))
            total_dist = sum(step_lengths)
            logging.info(f"  SmartPDR歩幅推定 (px): 平均={np.mean(step_lengths):.1f}  "
                  f"≈ 平均{np.mean(step_lengths)/M_TO_PIXEL:.2f}m/歩")
            target_text = "未指定" if args.target_distance_px is None else f"{args.target_distance_px:.0f}px"
            logging.info(f"  推定総移動距離: {total_dist:.0f}px  "
                  f"(校正前: {raw_total_dist:.0f}px, scale={step_scale:.2f}, "
                  f"gain={args.step_gain:.2f}, 目標: {target_text})")
        else:
            logging.info("  ステップが検出されなかったため、このログは描画されません。")
            continue

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

        extinction_count = 0
        step_count       = 0

        madgwick = Madgwick() if HAS_AHRS else None
        Q = np.tile([1., 0., 0., 0.], (len(df), 1)) if HAS_AHRS else None

        for i in range(1, len(df)):
            row      = df.iloc[i]
            prev_row = df.iloc[i - 1]

            dt = row['timestamp'] - prev_row['timestamp']
            if dt <= 0 or dt > MAX_DT:
                continue

            if HAS_AHRS:
                try:
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
                        Q[i] = madgwick.updateMARG(q=Q[i - 1], gyr=gyro, acc=acc, mag=mag)
                    else:
                        Q[i] = madgwick.updateIMU(q=Q[i - 1], gyr=gyro, acc=acc)

                    rot = R.from_quat([Q[i][1], Q[i][2], Q[i][3], Q[i][0]])
                    gyro_angle = normalize_angle(rot.as_euler('xyz')[2])
                except Exception as e:
                    logging.debug("Madgwick failed: %s", e)
                    # fallback to simple yaw rate
                    yaw_rate = get_yaw_rate(row['gyro_x'], row['gyro_y'], row['gyro_z'], row['acc_x'], row['acc_y'], row['acc_z'])
                    gyro_angle = normalize_angle(gyro_angle * ANGLE_DECAY + yaw_rate * dt)
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
            step_heading = weighted_angle_mean(segment_heading, np.ones(len(segment_heading)))
            ref_x = np.average(particles[:, 0], weights=weights)
            ref_y = np.average(particles[:, 1], weights=weights)
            step_heading = correct_heading_with_route(ref_x, ref_y, step_heading)

            noise_dist  = np.random.normal(0, SIGMA_STEP,  N_PARTICLES)
            noise_angle = np.random.normal(0, SIGMA_ANGLE, N_PARTICLES)
            move        = step_px + noise_dist
            p_angle     = step_heading + noise_angle

            new_particles = particles.copy()
            new_particles[:, 0] += move * np.cos(p_angle)
            new_particles[:, 1] += move * np.sin(p_angle)

            hit_wall = path_hits_wall(particles, new_particles)

            ny_idx = np.clip(new_particles[:, 1].astype(int), 0, h - 1)
            nx_idx = np.clip(new_particles[:, 0].astype(int), 0, w - 1)
            dist_values = dist_map[ny_idx, nx_idx]
            wall_weights = np.exp(
                -(1.0 - dist_values) ** 2 /
                (2 * WALL_WEIGHT_SIGMA ** 2)
            )
            weights = WALL_WEIGHT_FLOOR + (1.0 - WALL_WEIGHT_FLOOR) * wall_weights

            on_route = route_mask[ny_idx, nx_idx]
            weights *= np.where(on_route, 1.0, OFF_ROUTE_WEIGHT)
            particle_heading_error = angle_diff(p_angle, step_heading)
            weights *= np.exp(
                -(particle_heading_error ** 2) /
                (2 * SIGMA_ANGLE ** 2)
            )
            weights[hit_wall] = 0.0

            sum_w = weights.sum()
            if sum_w > 0:
                weights /= sum_w
                particles, weights = resample_if_needed(new_particles, weights)
            else:
                extinction_count += 1
                ref_x, ref_y = np.mean(particles, axis=0)
                particles[:, 0] = ref_x + np.random.normal(0, RECOVERY_SIGMA, N_PARTICLES)
                particles[:, 1] = ref_y + np.random.normal(0, RECOVERY_SIGMA, N_PARTICLES)

                for _ in range(50):
                    new_x = ref_x + np.random.normal(0, RECOVERY_SIGMA, N_PARTICLES)
                    new_y = ref_y + np.random.normal(0, RECOVERY_SIGMA, N_PARTICLES)
                    nx_i  = np.clip(new_x.astype(int), 0, w - 1)
                    ny_i  = np.clip(new_y.astype(int), 0, h - 1)
                    valid = binary_for_pf[ny_i, nx_i] == 255
                    if valid.sum() > N_PARTICLES // 4:
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

            raw_x = np.average(particles[:, 0], weights=weights)
            raw_y = np.average(particles[:, 1], weights=weights)
            pos_buffer.append((raw_x, raw_y))
            avg_pos = np.mean(pos_buffer, axis=0)
            estimated_positions.append(tuple(avg_pos))

        logging.info(f"  処理ステップ数: {step_count}  全滅回数: {extinction_count}")
        if estimated_positions:
            last = estimated_positions[-1]
            logging.info(f"  最終推定位置: x={last[0]:.1f}, y={last[1]:.1f}")

        if estimated_positions:
            pos_arr = np.array(estimated_positions)
            ax.plot(pos_arr[:, 0], pos_arr[:, 1], linewidth=2, label=f'PF Path: {Path(file_name).name}')
            ax.scatter(START_X, START_Y, s=50, zorder=5, label=f'Start: {Path(file_name).name}')

    end_overall = time.time()
    logging.info(f"\nパーティクル数: {N_PARTICLES}")
    logging.info(f"Overall time: {end_overall - start_overall:.2f} seconds")

    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    title = "test10: 管理棟L字経路優先 + SmartPDR + パーティクルフィルタ"
    if japanize_matplotlib is None:
        title = "test10: management route-aware SmartPDR + PF"
    ax.set_title(title)

    fig.subplots_adjust(right=0.72)
    fig.canvas.draw()
    fig.canvas.flush_events()

    if args.save is not None:
        save_path = args.save.resolve()
        save_path.parent.mkdir(parents=True, exist_ok=True)
        fig.set_size_inches(10, 8, forward=True)
        fig.savefig(save_path, dpi=200, bbox_inches='tight')
        logging.info(f"結果画像を保存しました: {save_path}")


def main():
    global args, data_dir, img_path, binary, binary_for_pf, dist_map, h, w, sx, sy, route_mask, fig, ax

    args = parse_args()
    numeric_level = getattr(logging, args.log_level.upper(), None)
    if not isinstance(numeric_level, int):
        numeric_level = logging.INFO
    logging.basicConfig(level=numeric_level, format='%(levelname)s: %(message)s')

    map_config, map_config_path = load_map_config(args.map_config)
    apply_map_config(args, map_config, map_config_path)
    data_dir = args.data_dir
    img_path = args.map
    if args.seed is not None:
        np.random.seed(args.seed)

    binary, binary_for_pf, dist_map = load_preprocessed_map(img_path)
    h, w = binary.shape
    if not data_dir.exists():
        raise FileNotFoundError(f"CSVフォルダが見つかりません: {data_dir}")
    sx, sy = int(round(START_X)), int(round(START_Y))
    if not (0 <= sx < w and 0 <= sy < h):
        raise ValueError(f"開始位置が地図外です: ({START_X}, {START_Y})")
    if binary_for_pf[sy, sx] != 255:
        raise ValueError(f"開始位置が壁です: ({START_X}, {START_Y})")
    route_mask = build_route_mask(binary.shape)

    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 8))

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

        logging.info("===================================")
        logging.info("Monitoring started...")
        logging.info(str(data_dir))
        logging.info("===================================")

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


if __name__ == "__main__":
    main()
