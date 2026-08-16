import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import glob
import os

# =====================
# ① ファイルリストの取得
# =====================
file_list = glob.glob("pdr_log_*.csv")
file_list.sort() 

if not file_list:
    print("CSVファイルが見つかりません。")
    exit()

plt.figure(figsize=(10, 8))

# =====================
# ② ループ処理（重複を削除し整理）
# =====================
for file_path in file_list:
    # 1. データの読み込み
    df = pd.read_csv(file_path)
    df.columns = df.columns.str.strip() # 列名の空白除去

    # 初期設定
    x_coords, y_coords = [0.0], [0.0]
    curr_x, curr_y = 0.0, 0.0
    angle = 0.0
    
    if len(df) < 2:
        continue

    prev_time = df.loc[0, "timestamp"]
    
    # 2. 軌跡の再計算（ここを1回だけにします）
    for i in range(1, len(df)):
        dt = df.loc[i, "timestamp"] - prev_time
        prev_time = df.loc[i, "timestamp"]
        
        if dt <= 0 or dt > 0.5: 
            continue
        
        gz = df.loc[i, "gyro_z"]
        
        # 方位更新
        angle += gz * dt
        
        # 歩幅設定（8.75mに合わせるための調整値）
        step = 0.0175 
        
        curr_x += step * np.cos(angle)
        curr_y += step * np.sin(angle)
        
        x_coords.append(curr_x)
        y_coords.append(curr_y)

    # 3. 描画（ループ内で1回だけ実行）
    label_name = os.path.basename(file_path)
    plt.plot(x_coords, y_coords, label=label_name, alpha=0.7)

# =====================
# ③ 仕上げ
# =====================

# 実際のL字通路（正解）を描画
true_l_x = [0, 8.75, 8.75]
true_l_y = [0, 0, 8.75]
plt.plot(true_l_x, true_l_y, color='black', linewidth=4, label='True Path (L)', zorder=1)

# グラフ設定
plt.axis('equal') 
plt.grid(True, linestyle='--', alpha=0.5)
plt.xlabel("Distance (m)", fontsize=12)
plt.ylabel("Distance (m)", fontsize=12)
plt.title("PDR Trajectories vs True L-shape", fontsize=14)

# 凡例（重複が消えているはずです）
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize='small')

plt.xlim(-1, 15)
plt.ylim(-1, 15)
plt.tight_layout()
plt.show()