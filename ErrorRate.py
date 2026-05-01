#csvファイルのグラフと理論値のグラフの誤差率を計算するプログラム
import pandas as pd
import numpy as np
import glob
import os
import math

# --- 設定 ---
TRUE_END_X = 8.75
TRUE_END_Y = 8.75
TOTAL_TRUE_DIST = TRUE_END_X + TRUE_END_Y # 17.5m
STEP_LENGTH = 0.0175 

file_list = glob.glob("pdr_log_*.csv")
file_list.sort()

if not file_list:
    print("CSVファイルが見つかりません。")
    exit()

results = []
print(f"{'ファイル名':<20} | {'位置誤差(m)':<7} | {'誤差率(%)':<8}")
print("-" * 55)

for file_path in file_list:
    try:
        # 1. まずCSVを読み込む（ヘッダーを無視して読み込み直す準備）
        # skiprows=1 で1行目を飛ばし、namesで強制的に列名を割り当てる
        df = pd.read_csv(file_path, 
                         names=["timestamp","acc_x","acc_y","acc_z","gyro_x","gyro_y","gyro_z","x","y"],
                         skiprows=1)
        
        # 2. データのクレンジング（NaN行を削除）
        df = df.dropna(subset=["timestamp", "gyro_z"]).reset_index(drop=True)

        if len(df) < 2:
            print(f"{os.path.basename(file_path):<25} | データ不足")
            continue

        curr_x, curr_y = 0.0, 0.0
        angle = 0.0
        prev_time = df.loc[0, "timestamp"]

        # 3. 軌跡計算
        for i in range(1, len(df)):
            current_time = df.loc[i, "timestamp"]
            gz = df.loc[i, "gyro_z"]

            dt = current_time - prev_time
            prev_time = current_time
            
            if dt <= 0 or dt > 0.5:
                continue
            
            angle += gz * dt
            curr_x += STEP_LENGTH * np.cos(angle)
            curr_y += STEP_LENGTH * np.sin(angle)

        # 4. 誤差計算
        dist_err = math.sqrt((curr_x - TRUE_END_X)**2 + (curr_y - TRUE_END_Y)**2)
        err_rate = (dist_err / TOTAL_TRUE_DIST) * 100

        results.append(err_rate)
        print(f"{os.path.basename(file_path):<25} | {dist_err:>10.2f}m | {err_rate:>8.2f}%")

    except Exception as e:
        print(f"{os.path.basename(file_path):<25} | エラー: {e}")

# --- 統計情報の表示 ---
if results:
    print("-" * 55)
    print(f"平均誤差率: {sum(results) / len(results):.2f} %")