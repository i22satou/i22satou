import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import cv2
import glob
import collections
import time
from scipy.signal import find_peaks

# ============================================================
# 1. 定数・スケールの設定
# ============================================================
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

SMOOTH_WINDOW  = 5
RECOVERY_SIGMA = 8.0
ANGLE_DECAY    = 0.999

# ============================================================
# 3. ユーティリティ関数
# ============================================================

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
    end       = min(len(acc_mag), center_idx + window)
    segment   = acc_mag.iloc[start:end]
    amplitude = max(segment.max() - segment.min(), 1e-6)
    return WEINBERG_K * (amplitude ** 0.25) * M_TO_PIXEL


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


# ============================================================
# 4. マップの読み込みと前処理
# ============================================================
img_path = "L_map.png"
img_gray  = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
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
file_list = glob.glob("pdr_log_*.csv")
file_list.sort()

if not file_list:
    print("CSVファイルが見つかりません。")
    exit()

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

    df = df.dropna(subset=['timestamp','acc_x','acc_y','acc_z',
                            'gyro_x','gyro_y','gyro_z']).reset_index(drop=True)
    df = df[df['timestamp'].diff().fillna(1) > 0].reset_index(drop=True)

    df['acc_mag']    = compute_acc_magnitude(df)
    step_indices, _  = detect_steps(df['acc_mag'])

    print(f"[{file_name}] 総行数: {len(df)}  検出ステップ数: {len(step_indices)}")

    if len(step_indices) > 0:
        step_lengths = [estimate_step_length_px(df['acc_mag'], i) for i in step_indices]
        total_dist   = sum(step_lengths)
        print(f"  歩幅推定 (px): 平均={np.mean(step_lengths):.1f}  "
              f"≈ 平均{np.mean(step_lengths)/M_TO_PIXEL:.2f}m/歩")
        print(f"  推定総移動距離: {total_dist:.0f}px  (目標: 700px)")

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
        if dt <= 0 or dt > 1.0:
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

        # 1. 予測
        noise_dist  = np.random.normal(0, SIGMA_STEP,  N_PARTICLES)
        noise_angle = np.random.normal(0, SIGMA_ANGLE, N_PARTICLES)
        move        = step_px + noise_dist
        p_angle     = angle + noise_angle
        particles[:, 0] += move * np.cos(p_angle)
        particles[:, 1] += move * np.sin(p_angle)

        # 2. 重み更新
        px_i = particles[:, 0].astype(int)
        py_i = particles[:, 1].astype(int)
        in_bounds = (px_i >= 0) & (px_i < w) & (py_i >= 0) & (py_i < h)
        px_c = np.clip(px_i, 0, w - 1)
        py_c = np.clip(py_i, 0, h - 1)

        weights = dist_map[py_c, px_c]
        weights[~in_bounds] = 0.0

        # 3. リサンプリング
        sum_w = weights.sum()
        if sum_w > 0:
            weights /= sum_w
            particles, weights = resample_if_needed(particles, weights)
        else:
            # ---- 全滅リカバリ（改善版） ----
            # 前回推定位置ではなく、直前のパーティクル位置の平均を基準にする
            # → 前回位置への逆戻りを防ぐ
            extinction_count += 1
            ref_x = np.clip(np.mean(particles[:, 0]), 0, w - 1)
            ref_y = np.clip(np.mean(particles[:, 1]), 0, h - 1)

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
        raw_x = np.mean(particles[:, 0])
        raw_y = np.mean(particles[:, 1])
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
                 linewidth=2, label=f'PF Path: {file_name}')
        plt.scatter(pos_arr[0, 0], pos_arr[0, 1], s=50, zorder=5,
                    label=f'Start: {file_name}')

# ============================================================
# 8. 結果表示
# ============================================================
end_overall = time.time()
print(f"\nパーティクル数: {N_PARTICLES}")
print(f"Overall time: {end_overall - start_overall:.2f} seconds")

plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
plt.title("改善版PDR: ステップ検出 + 動的歩幅 + 距離マップ重み + Neffリサンプリング + ドリフト補正")
plt.tight_layout()
plt.show()
