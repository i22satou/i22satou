import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import cv2
import glob
import os
import collections

# --- 1. 定数・スケールの設定 ---
M_TO_PIXEL = 40.0
STEP_LENGTH_M = 0.0175
STEP_LENGTH_P = STEP_LENGTH_M * M_TO_PIXEL
N_PARTICLES = 700

# マップ読み込み
img_path = "L_map.png"
img_bgr = cv2.imread(img_path)
img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
_, binary = cv2.threshold(img_gray, 127, 255, cv2.THRESH_BINARY)
h, w = binary.shape

# 判定用（収縮マップ）
kernel = np.ones((5, 5), np.uint8)
binary_for_pf = cv2.erode(binary, kernel, iterations=4)

def calculate_wall_violation(pos_list, map_img):
    if not pos_list: return 0.0
    count = 0
    valid_samples = 0
    for x, y in pos_list:
        if not np.isfinite(x) or not np.isfinite(y): continue
        valid_samples += 1
        px, py = int(np.clip(x, 0, w - 1)), int(np.clip(y, 0, h - 1))
        if map_img[py, px] == 0: count += 1
    return (count / valid_samples * 100) if valid_samples > 0 else 0.0

# 全CSVファイルの取得
file_list = sorted(glob.glob("pdr_log_*.csv"))
if not file_list:
    print("CSVファイルが見つかりません。")
    exit()

summary_results = []

# 動画保存用のディレクトリを作成 (存在していれば何もしない)
output_dir = "movie"
os.makedirs(output_dir, exist_ok=True)
summary_results = []

# --- 2. メインループ（各ファイル1回ずつ実行） ---
for file_idx, file_name in enumerate(file_list):
    print(f"\n[{file_idx+1}/{len(file_list)}] 解析中: {file_name}")
    
    df = pd.read_csv(file_name).dropna(subset=["timestamp", "gyro_z"]).reset_index(drop=True)
    
    # 動画の保存パス設定
    video_filename = f"video_{os.path.splitext(os.path.basename(file_name))[0]}.mp4"
    video_path = os.path.join(output_dir, video_filename)
    
    # コーデック設定（Mac/Win両対応のため avc1 を推奨、ダメなら mp4v）
    fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
    video = None

    # 変数初期化
    start_x, start_y = 50.0, 250.0
    particles = np.zeros((N_PARTICLES, 2))
    particles[:, 0] = start_x + np.random.normal(0, 2, N_PARTICLES)
    particles[:, 1] = start_y + np.random.normal(0, 2, N_PARTICLES)
    weights = np.ones(N_PARTICLES) / N_PARTICLES
    
    angle = 0.0
    prev_time = df.loc[0, "timestamp"]
    pf_positions, raw_positions = [], [(start_x, start_y)]
    raw_x, raw_y = start_x, start_y
    pos_buffer = collections.deque(maxlen=10)

    # ステップごとの計算（データフレームのループ）
    for i, row in enumerate(df.itertuples()):
        if row.Index == 0: continue
        dt = row.timestamp - prev_time
        if dt <= 0 or dt > 1.0:
            prev_time = row.timestamp
            continue
        prev_time = row.timestamp

        # 推定計算
        angle += row.gyro_z * dt
        raw_x += STEP_LENGTH_P * np.cos(angle)
        raw_y += STEP_LENGTH_P * np.sin(angle)
        raw_positions.append((raw_x, raw_y))

        # PF予測
        particles[:, 0] += (STEP_LENGTH_P + np.random.normal(0, 0.1, N_PARTICLES)) * np.cos(angle + np.random.normal(0, np.deg2rad(3), N_PARTICLES))
        particles[:, 1] += (STEP_LENGTH_P + np.random.normal(0, 0.1, N_PARTICLES)) * np.sin(angle + np.random.normal(0, np.deg2rad(3), N_PARTICLES))

        # 重み更新
        weights.fill(0.0)
        for j in range(N_PARTICLES):
            px, py = int(np.clip(particles[j, 0], 0, w-1)), int(np.clip(particles[j, 1], 0, h-1))
            if binary_for_pf[py, px] > 0: weights[j] = 1.0
        
        sum_w = np.sum(weights)
        if sum_w > 0:
            weights /= sum_w
            particles = particles[np.random.choice(range(N_PARTICLES), size=N_PARTICLES, p=weights)]
        else:
            ref = pf_positions[-1] if pf_positions else (start_x, start_y)
            particles[:, 0] = ref[0] + np.random.normal(0, 10, N_PARTICLES)
            particles[:, 1] = ref[1] + np.random.normal(0, 10, N_PARTICLES)

        current_est = np.mean(particles, axis=0)
        pos_buffer.append(current_est)
        pf_positions.append(tuple(np.mean(pos_buffer, axis=0)))

        # 5ステップごとに動画フレームを作成
        if i % 5 == 0 or i == len(df) - 1:
            fig = plt.figure(figsize=(8, 8), dpi=100)
            plt.imshow(binary, cmap='gray')
            plt.scatter(particles[:, 0], particles[:, 1], s=3, c='red', alpha=0.4)
            path_pf = np.array(pf_positions)
            path_raw = np.array(raw_positions)
            plt.plot(path_raw[:, 0], path_raw[:, 1], c='orange', linestyle='--', label='Raw')
            plt.plot(path_pf[:, 0], path_pf[:, 1], c='blue', label='PF')
            plt.title(f"{file_name} - Step: {i}")
            plt.axis('off')

            fig.canvas.draw()
            frame = np.asarray(fig.canvas.buffer_rgba())
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            
            if video is None:
                f_h, f_w = frame.shape[:2]
                video = cv2.VideoWriter(video_path, fourcc, 20.0, (f_w, f_h))
            
            video.write(frame)
            plt.close(fig)

    # --- 重要：ここで確実に動画を保存（閉じる） ---
    if video is not None:
        video.release()
        print(f"動画を保存しました: {video_path}")

    # 評価
    raw_r = calculate_wall_violation(raw_positions, binary)
    pf_r = calculate_wall_violation(pf_positions, binary)
    summary_results.append({
        "File": file_name,
        "Raw": f"{raw_r:.1f}%",
        "PF": f"{pf_r:.1f}%",
        "Diff": f"{raw_r - pf_r:.1f}pts"
    })
    
# --- 3. 最終結果の表示 ---
print("\n" + "="*65)
print(f"{'File Name':<30} | {'Raw':<7} | {'PF':<7} | {'Diff'}")
print("-" * 65)
for res in summary_results:
    print(f"{res['File']:<30} | {res['Raw']:<7} | {res['PF']:<7} | {res['Diff']}")
print("="*65)