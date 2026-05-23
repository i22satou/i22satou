import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import cv2
import glob
import collections
import time
import argparse
from pathlib import Path
import japanize_matplotlib #日本語用のライブラリ
from scipy.signal import find_peaks

# ============================================================
# 1. 定数・スケールの設定
# ============================================================
BASE_DIR = Path(__file__).resolve().parent

REAL_LENGTH_M = 8.75
MAP_LENGTH_P  = 350.0
M_TO_PIXEL    = MAP_LENGTH_P / REAL_LENGTH_M  # 40 px/m

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

SMOOTH_WINDOW  = 5
RECOVERY_SIGMA = 8.0
ANGLE_DECAY    = 0.999
MAX_DT         = 1.0

# ============================================================
# 3. ユーティリティ関数
# ============================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description="L字マップ上でPDRログをパーティクルフィルタにより推定します。"
    )
    parser.add_argument(
        "--data-dir",
        type=Path,
        default=BASE_DIR,
        help="pdr_log_*.csv と L_map.png があるディレクトリ",
    )
    parser.add_argument(
        "--map",
        type=Path,
        default=None,
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
    return parser.parse_args()


def compute_acc_magnitude(df):
    return np.sqrt(df['acc_x']**2 + df['acc_y']**2 + df['acc_z']**2)


def detect_steps(acc_mag: pd.Series):
    rolling_mean = acc_mag.rolling(20, center=True, min_periods=1).mean()
    detrended    = acc_mag - rolling_mean
    peaks, _     = find_peaks(detrended, height=STEP_PEAK_HEIGHT,
                               distance=STEP_MIN_INTERVAL)
    return peaks, detrended


def estimate_step_length_px(acc_mag: pd.Series, center_idx: int, window: int = 10):
    start     = max(0, center_idx - window)
    end       = min(len(acc_mag), center_idx + window + 1)
    segment   = acc_mag.iloc[start:end]
    amplitude = max(segment.max() - segment.min(), 1e-6)
    step_m    = WEINBERG_K * (amplitude ** 0.25)
    step_m    = np.clip(step_m, MIN_STEP_M, MAX_STEP_M)
    return step_m * M_TO_PIXEL


def get_yaw_rate(gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z):
    norm = max(np.sqrt(acc_x**2 + acc_y**2 + acc_z**2), 1e-6)
    ax, ay, az = acc_x/norm, acc_y/norm, acc_z/norm
    pitch = np.arctan2(ax, np.sqrt(ay**2 + az**2))
    roll  = np.arctan2(ay, np.sqrt(ax**2 + az**2))
    return (  gyro_z * np.cos(roll) * np.cos(pitch)
            - gyro_y * np.sin(roll)
            + gyro_x * np.sin(pitch) * np.cos(roll))


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


def path_hits_wall(old_particles, new_particles, n_checks: int = 5):
    """移動線分上を複数点で確認し、壁をすり抜ける粒子を弾く。"""
    hit_wall = np.zeros(len(old_particles), dtype=bool)
    for ratio in np.linspace(0.0, 1.0, n_checks):
        x = old_particles[:, 0] + (new_particles[:, 0] - old_particles[:, 0]) * ratio
        y = old_particles[:, 1] + (new_particles[:, 1] - old_particles[:, 1]) * ratio
        hit_wall |= is_in_wall(x, y)
    return hit_wall


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

img_gray  = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
if img_gray is None:
    raise FileNotFoundError(f"マップ画像を読み込めません: {img_path}")

_, binary = cv2.threshold(img_gray, 127, 255, cv2.THRESH_BINARY)
h, w = binary.shape

kernel        = np.ones((5, 5), np.uint8)
binary_for_pf = cv2.erode(binary, kernel, iterations=4)

dist_map = cv2.distanceTransform(binary_for_pf, cv2.DIST_L2, 5)
dist_max = dist_map.max()
if dist_max > 0:
    dist_map = dist_map / dist_max

# ============================================================
# 5. CSVファイルの読み込み
# ============================================================
file_list = glob.glob(str(data_dir / "pdr_log_*.csv"))
file_list.sort()

if not file_list:
    raise FileNotFoundError(f"CSVファイルが見つかりません: {data_dir / 'pdr_log_*.csv'}")

# ============================================================
# 6. 描画の準備
# ============================================================
plt.figure(figsize=(10, 8))
plt.imshow(binary, cmap='gray')

start_overall = time.time()

# ============================================================
# 7. メインループ（ファイルごと）
# ============================================================
for file_name in file_list:
    df = pd.read_csv(file_name)

    df = validate_log(df, file_name)
    if len(df) < 2:
        print(f"[{Path(file_name).name}] 有効なデータが少ないためスキップします。")
        continue

    df['acc_mag']    = compute_acc_magnitude(df)
    step_indices, _  = detect_steps(df['acc_mag'])

    print(f"[{Path(file_name).name}] 総行数: {len(df)}  検出ステップ数: {len(step_indices)}")

    if len(step_indices) > 0:
        step_lengths = [estimate_step_length_px(df['acc_mag'], i) for i in step_indices]
        total_dist   = sum(step_lengths)
        print(f"  歩幅推定 (px): 平均={np.mean(step_lengths):.1f}  "
              f"≈ 平均{np.mean(step_lengths)/M_TO_PIXEL:.2f}m/歩")
        print(f"  推定総移動距離: {total_dist:.0f}px  (目標: 700px)")
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
    angle               = 0.0
    step_set            = set(step_indices)

    # デバッグ用カウンタ
    extinction_count = 0   # 全滅回数
    step_count       = 0   # 処理したステップ数

    # --- 行ループ ---
    for i in range(1, len(df)):
        row      = df.iloc[i]
        prev_row = df.iloc[i - 1]

        dt = row['timestamp'] - prev_row['timestamp']
        if dt <= 0 or dt > MAX_DT:
            continue

        yaw_rate = get_yaw_rate(
            row['gyro_x'], row['gyro_y'], row['gyro_z'],
            row['acc_x'],  row['acc_y'],  row['acc_z']
        )
        angle = angle * ANGLE_DECAY + yaw_rate * dt

        if i not in step_set:
            continue

        step_count += 1
        step_px = estimate_step_length_px(df['acc_mag'], i)

        # --- 1. 予測 (Prediction) ---
        noise_dist  = np.random.normal(0, SIGMA_STEP,  N_PARTICLES)
        noise_angle = np.random.normal(0, SIGMA_ANGLE, N_PARTICLES)
        move        = step_px + noise_dist
        p_angle     = angle + noise_angle
        
        new_particles = particles.copy()
        new_particles[:, 0] += move * np.cos(p_angle)
        new_particles[:, 1] += move * np.sin(p_angle)
        
        # --- 2. 壁衝突判定 & 重み更新 (Weighting) ---
        hit_wall = path_hits_wall(particles, new_particles)

        # 重みの計算（移動先の距離マップから取得）
        ny_idx = np.clip(new_particles[:, 1].astype(int), 0, h - 1)
        nx_idx = np.clip(new_particles[:, 0].astype(int), 0, w - 1)
        weights = dist_map[ny_idx, nx_idx].copy()
        
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
        plt.plot(pos_arr[:, 0], pos_arr[:, 1],
                 linewidth=2, label=f'PF Path: {Path(file_name).name}')
        plt.scatter(pos_arr[0, 0], pos_arr[0, 1], s=50, zorder=5,
                    label=f'Start: {Path(file_name).name}')

# ============================================================
# 8. 結果表示
# ============================================================
end_overall = time.time()
print(f"\nパーティクル数: {N_PARTICLES}")
print(f"Overall time: {end_overall - start_overall:.2f} seconds")

plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
plt.title("改善版PDR: ステップ検出 + 動的歩幅 + 距離マップ重み + Neffリサンプリング + ドリフト補正")
plt.tight_layout()
if args.save is not None:
    save_path = args.save.resolve()
    save_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(save_path, dpi=200)
    print(f"結果画像を保存しました: {save_path}")

if not args.no_show:
    plt.show()
