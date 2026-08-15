# 複数の軌跡を同時に表示するプログラム (スケール調整版)
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

# 1ステップ（20ms相当）あたりの移動量
STEP_LENGTH_M = 0.0175  # m単位
STEP_LENGTH_P = STEP_LENGTH_M * M_TO_PIXEL # ピクセル単位 (0.7 px)

alpha = 0.9        
search_range = 15  

# マップ画像の読み込み
img_path = "L_map.png"
img_gray = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
if img_gray is None:
    print(f"エラー: {img_path} が読み込めません。")
    exit()

_, binary = cv2.threshold(img_gray, 127, 255, cv2.THRESH_BINARY)
h, w = binary.shape
grad_y, grad_x = np.gradient(binary.astype(float))

# 全CSVファイルの取得
file_list = sorted(glob.glob("pdr_log_*.csv"))

if not file_list:
    print("CSVファイルが見つかりません。")
    exit()

# プロットの準備
plt.figure(figsize=(10, 8))
plt.imshow(binary, cmap='gray')

# --- 各ファイルごとに軌跡を計算 ---
for file_name in file_list:
    # データの読み込み
    df = pd.read_csv(file_name)
    
    # 初期位置と方向 (ファイルごとにリセット)
    x, y = 50.0, 250.0 
    angle = 0.0        
    current_positions = [(x, y)]
    
    if len(df) == 0:
        continue
    
    prev_time = df.loc[0, "timestamp"]

    for row in df.itertuples():
        i = row.Index
        current_time = row.timestamp
        gx = row.gyro_z

        if pd.isna(current_time) or pd.isna(gx) or i == 0:
            continue

        dt = current_time - prev_time
        # 極端に長い時間はスキップ（歩行停止とみなす）
        if dt <= 0 or dt > 1.0:
            prev_time = current_time
            continue
        prev_time = current_time

        # PDR計算 (STEP_LENGTH_P を使用)
        angle += gx * dt
        new_x = x + STEP_LENGTH_P * np.cos(angle)
        new_y = y + STEP_LENGTH_P * np.sin(angle)

        ix, iy = int(np.clip(new_x, 0, w-1)), int(np.clip(new_y, 0, h-1))
        
        # 壁判定と補正
        if binary[iy, ix] == 255:
            x, y = new_x, new_y
        else:
            # 最寄りの白いピクセル（通路）を探索
            best_x, best_y = x, y
            min_dist = float('inf')
            for dx in range(-search_range, search_range + 1):
                for dy in range(-search_range, search_range + 1):
                    nx, ny = ix + dx, iy + dy
                    if 0 <= nx < w and 0 <= ny < h and binary[ny, nx] == 255:
                        d = dx**2 + dy**2
                        if d < min_dist:
                            min_dist = d
                            best_x, best_y = nx, ny
            x, y = best_x, best_y

        # 壁付近の勾配による滑らかさ補正
        ix_safe, iy_safe = int(np.clip(x, 0, w-1)), int(np.clip(y, 0, h-1))
        x += 0.1 * grad_x[iy_safe, ix_safe]
        y += 0.1 * grad_y[iy_safe, ix_safe]
        
        # 移動の平滑化 (急激な変化を抑制)
        x = alpha * x + (1 - alpha) * current_positions[-1][0]
        y = alpha * y + (1 - alpha) * current_positions[-1][1]

        current_positions.append((x, y))

    # 計算した軌跡をプロットに追加
    pos_arr = np.array(current_positions)
    plt.plot(pos_arr[:, 0], pos_arr[:, 1], linewidth=2, label=f'{file_name}')
    plt.scatter(pos_arr[0, 0], pos_arr[0, 1], s=50) # 各開始点

# --- 最終表示 ---
plt.legend(loc='upper right', fontsize='x-small', ncol=2)
plt.title(f"地図制約のみのPDR経路")
plt.axis('equal') # 縦横比を正しく保つ
plt.show()