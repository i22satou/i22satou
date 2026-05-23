import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import cv2
import glob
import collections
import time
import argparse
import threading
from pathlib import Path
import japanize_matplotlib #日本語用のライブラリ
from scipy.signal import find_peaks
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from ahrs.filters import Madgwick
from scipy.spatial.transform import Rotation as R

# ============================================================
# 1. 定数・スケールの設定
# ============================================================
BASE_DIR = Path(__file__).resolve().parent

REAL_LENGTH_M = 8.75
MAP_LENGTH_P  = 350.0
M_TO_PIXEL    = MAP_LENGTH_P / REAL_LENGTH_M  # 40 px/m
TARGET_DISTANCE_PX = MAP_LENGTH_P * 2.0       # L字の横350px + 縦350px
DEFAULT_STEP_GAIN = 1.12                      # PFの壁判定で削られる分を少し補正

# 初期位置：横通路の左端中央
START_X = 70.0
START_Y = 250.0

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
#ANGLE_DECAY    = 0.999
MAX_DT         = 1.0

# ============================================================
# 3. ユーティリティ関数
# ============================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description="SmartPDR風のステップ検出・歩幅推定を加えたPDR推定を行います。"
    )
    parser.add_argument(
        "--data-dir",
        type=Path,
        default=Path(
        "/Users/soma/Library/CloudStorage/GoogleDrive-satosoma0608@gmail.com/マイドライブ/PDR"
        ),
        help="pdr_log_*.csv と L_map.png があるディレクトリ",
    )
    parser.add_argument(
        "--map",
        type=Path,
        default=Path(
        "/Users/soma/Library/CloudStorage/OneDrive-独立行政法人国立高等専門学校機構/卒業研究/i22satou/L_map.png"
        ),
        help="使用するマップ画像。未指定なら data-dir/L_map.png",
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
        default=TARGET_DISTANCE_PX,
        help="歩幅を校正する目標移動距離(px)。L字全体なら700px。",
    )
    parser.add_argument(
        "--no-step-calibration",
        action="store_true",
        help="SmartPDR歩幅の総距離校正を行いません。",
    )
    parser.add_argument(
        "--step-gain",
        type=float,
        default=DEFAULT_STEP_GAIN,
        help="校正後の歩幅に掛ける追加倍率。短い場合は1.15〜1.25程度に上げます。",
    )
    return parser.parse_args()


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

# ============================================================
# 4. マップの読み込みと前処理
# ============================================================
args = parse_args()
data_dir = args.data_dir.resolve()
img_path = args.map.resolve() if args.map else data_dir / "L_map.png"
if args.seed is not None:
    np.random.seed(args.seed)

img_gray = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
if img_gray is None:
    raise FileNotFoundError(f"マップ画像を読み込めません: {img_path}")

_, binary = cv2.threshold(img_gray, 127, 255, cv2.THRESH_BINARY)
h, w = binary.shape

kernel = np.ones((5, 5), np.uint8)
binary_for_pf = cv2.erode(binary, kernel, iterations=4)

dist_map = cv2.distanceTransform(binary_for_pf, cv2.DIST_L2, 5)
dist_max = dist_map.max()
if dist_max > 0:
    dist_map = dist_map / dist_max

# ============================================================
# 5. リアルタイム描画関数
# ============================================================
plt.ion()
fig, ax = plt.subplots(figsize=(10, 8))

processed_files = set()


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

        madgwick = Madgwick()
        Q = np.tile([1., 0., 0., 0.], (len(df), 1))
        
        # --- 行ループ ---
        for i in range(1, len(df)):
            row      = df.iloc[i]
            prev_row = df.iloc[i - 1]

            dt = row['timestamp'] - prev_row['timestamp']
            if dt <= 0 or dt > MAX_DT:
                continue

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

            yaw = rot.as_euler('xyz')[2]
            heading_history[i] = normalize_angle(yaw)

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
                # ※ここで再度の通路チェックを入れるとより堅牢になります

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
    ax.set_title("test7: SmartPDR風ステップ検出 + 動的歩幅 + 方位選択 + パーティクルフィルタ")

    fig.tight_layout()
    fig.canvas.draw()
    fig.canvas.flush_events()

    if args.save is not None:
        save_path = args.save.resolve()
        save_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save_path, dpi=200)
        print(f"結果画像を保存しました: {save_path}")


# ============================================================
# watchdog監視
# ============================================================
class CSVHandler(FileSystemEventHandler):

    def on_created(self, event):

        if event.is_directory:
            return

        if not event.src_path.endswith(".csv"):
            return

        print("\n===================================")
        print("New CSV detected")
        print(Path(event.src_path).name)
        print("===================================\n")

        time.sleep(2)

        redraw_all_paths()


# ============================================================
# 初回描画
# ============================================================
redraw_all_paths()

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

except KeyboardInterrupt:
    observer.stop()

observer.join()
