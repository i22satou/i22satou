#test3にパーティクルフィルタの制約を追加したプログラム

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import cv2
import glob
import os
import japanize_matplotlib

# --- 1. 縮尺と定数の設定 ---
REAL_LENGTH_M = 8.75    # 現実の直線距離 (m)
MAP_LENGTH_P = 350.0    # L_map.png上での直線距離 (px)
M_TO_PIXEL = MAP_LENGTH_P / REAL_LENGTH_M  # 変換係数 (40.0)

# 1ステップ（20ms）あたりの移動量 (m)
STEP_LENGTH_M = 0.0175  
# ピクセル単位に変換した正しい移動量 (0.7 px)
STEP_LENGTH_P = STEP_LENGTH_M * M_TO_PIXEL

# --- 設定項目 ---
file_list = glob.glob("pdr_log_*.csv")
file_list.sort()

if not file_list:
    print("CSVファイルが見つかりません。")
    exit()

# パーティクルフィルタのパラメータ
N_PARTICLES = 500  # パーティクル数
SIGMA_STEP = 0.1   # 移動量のノイズ（ピクセル単位に合わせて小さく調整）
SIGMA_ANGLE = np.deg2rad(3) # 角度のバラツキ（ノイズ）

# マップ画像の読み込み
img_path = "L_map.png"
img_gray = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
_, binary = cv2.threshold(img_gray, 127, 255, cv2.THRESH_BINARY)
h, w = binary.shape

plt.figure(figsize=(10, 8))
plt.imshow(binary, cmap='gray')

for file_name in file_list:
    df = pd.read_csv(file_name)
    
    # --- PF初期化 ---
    # 全パーティクルを初期位置 (50, 250) に配置
    particles = np.zeros((N_PARTICLES, 2))
    particles[:, 0] = 50.0 + np.random.normal(0, 2, N_PARTICLES)
    particles[:, 1] = 250.0 + np.random.normal(0, 2, N_PARTICLES)
    weights = np.ones(N_PARTICLES) / N_PARTICLES
    
    estimated_positions = [(50.0, 250.0)]
    angle = 0.0
    prev_time = df.loc[0, "timestamp"]

    for row in df.itertuples():
        if row.Index == 0: continue
        
        current_time = row.timestamp
        gx = row.gyro_z
        if pd.isna(current_time) or pd.isna(gx): continue

        dt = current_time - prev_time
        if dt <= 0 or dt > 1.0:
            prev_time = current_time
            continue
        prev_time = current_time

        # 1. 方位更新
        angle += gx * dt

        # 2. 予測 (Predict): 全パーティクルを個別に動かす
        # それぞれに少しずつ異なるノイズを加えることで「可能性」を広げる
        p_noise_dist = np.random.normal(0, SIGMA_STEP, N_PARTICLES)
        p_noise_angle = np.random.normal(0, SIGMA_ANGLE, N_PARTICLES)
        
        particles[:, 0] += (STEP_LENGTH_P + p_noise_dist) * np.cos(angle + p_noise_angle)
        particles[:, 1] += (STEP_LENGTH_P + p_noise_dist) * np.sin(angle + p_noise_angle)

        # 3. 更新 (Update): 壁判定による重み付け
        # 壁（黒: 0）にめり込んだパーティクルは重みを下げる
        for j in range(N_PARTICLES):
            px, py = int(particles[j, 0]), int(particles[j, 1])
            if 0 <= px < w and 0 <= py < h:
                if binary[py, px] == 0: # 壁（黒）の場合
                    weights[j] = 0.0# 存在確率をゼロにする
                else:
                    weights[j] = 1.0    # 通路なら生存
            else:
                weights[j] = 0.0 # 画面外は即死

        # 重みの正規化
        sum_w = np.sum(weights)
        if sum_w > 0:
            weights /= sum_w
            indices = np.random.choice(range(N_PARTICLES), size=N_PARTICLES, p=weights)
            particles = particles[indices]
        else:
            # 【重要】全滅した場合のリカバリを強化
            # 壁を突き抜けたのではなく「通路内のどこかに引き戻す」
            # 前回の確定位置の周囲に、より広くパーティクルを再散布する
            particles[:, 0] = estimated_positions[-1][0] + np.random.normal(0, 15, N_PARTICLES)
            particles[:, 1] = estimated_positions[-1][1] + np.random.normal(0, 15, N_PARTICLES)
            weights.fill(1.0 / N_PARTICLES)

        # 4. リサンプリング (Resampling): 優秀な（壁に当たっていない）個体をコピーする
        indices = np.random.choice(range(N_PARTICLES), size=N_PARTICLES, p=weights)
        particles = particles[indices]
        weights.fill(1.0 / N_PARTICLES) # 重みを均一に戻す

        # 5. 推定値の計算: 生き残った全パーティクルの平均座標
        est_x = np.mean(particles[:, 0])
        est_y = np.mean(particles[:, 1])
        estimated_positions.append((est_x, est_y))

    # プロット
    pos_arr = np.array(estimated_positions)
    plt.plot(pos_arr[:, 0], pos_arr[:, 1], linewidth=2, label=f'PF Path: {file_name}')
    plt.scatter(pos_arr[0, 0], pos_arr[0, 1], s=30)
    
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
plt.title("地図制約+パーティクルフィルタ補正のPDR経路")
plt.tight_layout()
plt.show()