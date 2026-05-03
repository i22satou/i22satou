#何も変更を加えず生身のPDRデータを地図上に表示するプログラム
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import cv2
import glob
import os
import time
import japanize_matplotlib

# --- 1. 定数・スケールの設定 ---
REAL_LENGTH_M = 8.75    # 現実のL字直線部の長さ (m)
MAP_LENGTH_P = 350.0    # 地図上のL字直線部の長さ (px)
M_TO_PIXEL = MAP_LENGTH_P / REAL_LENGTH_M  # 変換係数 (40 px/m)

# 1ステップ（20ms）あたりの移動量 (m) 
# 前回の計算に基づき 0.0175m とします（時速約3.15kmに相当）
STEP_LENGTH_M = 0.0175  
STEP_LENGTH_P = STEP_LENGTH_M * M_TO_PIXEL # ピクセル単位の移動量 (0.7 px)

# --- 2. ファイル読み込み設定 ---
file_list = glob.glob("pdr_log_*.csv")
file_list.sort()

if not file_list:
    print("CSVファイルが見つかりません。")
    exit()

# --- 3. マップ画像の読み込み ---
img_path = "L_map.png"
if not os.path.exists(img_path):
    print(f"エラー: {img_path} が見つかりません。")
    exit()

img_gray = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
_, binary = cv2.threshold(img_gray, 127, 255, cv2.THRESH_BINARY)
h, w = binary.shape

plt.figure(figsize=(10, 8))
plt.imshow(binary, cmap='gray')

# --- 4. 軌跡計算と描画 ---
start_time_all = time.time()

for file_name in file_list:
    df = pd.read_csv(file_name)
    
    # 初期位置（マップ作成時の座標に合わせる）
    # 横通路が (50, 220) から始まるため、中央付近の (50, 250) をスタートに設定
    x, y = 50.0, 250.0 
    angle = 0.0
    raw_positions = [(x, y)]
    prev_time = df.loc[0, "timestamp"]

    for row in df.itertuples():
        if row.Index == 0: continue
        
        current_time = row.timestamp
        gx = row.gyro_z
        if pd.isna(current_time) or pd.isna(gx): continue

        dt = current_time - prev_time
        # サンプリング間隔の異常チェック
        if dt <= 0 or dt > 1.0:
            prev_time = current_time
            continue
        prev_time = current_time

        # --- PDR計算 (ピクセル単位) ---
        # 角度更新 (ラジアン)
        angle += gx * dt
        
        # 座標更新 (メートルからピクセルに変換された移動量を使用)
        x += STEP_LENGTH_P * np.cos(angle)
        y += STEP_LENGTH_P * np.sin(angle)

        raw_positions.append((x, y))

    # 描画
    pos_arr = np.array(raw_positions)
    plt.plot(pos_arr[:, 0], pos_arr[:, 1], linewidth=2, label=f'Scaled PDR: {file_name}')
    plt.scatter(pos_arr[0, 0], pos_arr[0, 1], s=50, color='red', zorder=5) # 開始点

end_time_all = time.time()

# --- 5. 結果表示 ---
print(f"処理完了。計算時間: {end_time_all - start_time_all:.2f} 秒")
print(f"使用スケール: {M_TO_PIXEL} px/m")
print(f"1ステップあたりの移動量: {STEP_LENGTH_P:.3f} px")

plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
plt.title("未修正PDR経路")
plt.xlabel("X [pixel]")
plt.ylabel("Y [pixel]")
plt.tight_layout()
plt.show()