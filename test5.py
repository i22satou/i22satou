import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import cv2
import glob
import os
import collections
import time
import japanize_matplotlib

# --- 1. 定数・スケールの設定 (L字マップの仕様に合わせる) ---
REAL_LENGTH_M = 8.75    # 現実のL字直線部の長さ (m)
MAP_LENGTH_P = 350.0    # 地図上のL字直線部の長さ (px)
M_TO_PIXEL = MAP_LENGTH_P / REAL_LENGTH_M  # 変換係数 (40 px/m)

# 1ステップ（20ms）あたりの移動量 (m)
# 1秒間に約0.875m（時速3.15km）歩く設定
STEP_LENGTH_M = 0.0175  
STEP_LENGTH_P = STEP_LENGTH_M * M_TO_PIXEL # ピクセル単位の移動量 (0.7 px)

# --- 設定項目 ---
file_list = glob.glob("pdr_log_*.csv") #パターンに合うファイルを一括指定
file_list.sort()

if not file_list:
    print("CSVファイルが見つかりません。")
    exit()

# パーティクルフィルタのパラメータ
N_PARTICLES = 500  # 精度と計算負荷のバランスが良い数に調整
SIGMA_STEP = 0.1   # 移動量のノイズ（ピクセル単位に合わせて小さく調整）
SIGMA_ANGLE = np.deg2rad(3) # 角度のノイズ（少し絞って安定化）

# マップ画像の読み込み
img_path = "L_map.png"
img_gray = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE) #白黒でマップを読み込む
_, binary = cv2.threshold(img_gray, 127, 255, cv2.THRESH_BINARY) #127を基準にそれより明るければ白に暗ければ黒にする
h, w = binary.shape #画像の大きさを取得

plt.figure(figsize=(10, 8))
plt.imshow(binary, cmap='gray')

# --- 地図の加工（通路を中央に誘導） ---
# iterations=4 で通路をしっかり狭くし、中央を歩かせる
kernel = np.ones((5, 5), np.uint8) 
binary_for_pf = cv2.erode(binary, kernel, iterations=4) 
#白い通路を削り、黒い壁を膨らます
#iterations=4はこの操作を4回繰り返す
#操作の理由：通常の地図ではパーティクルは地図ギリギリまで動くことが可能であるが人間は壁を擦りながら歩くことはない(大抵は中央を歩こうとする)
#あえて道を細くすることで壁際に向かっているパーティクルを早めに死滅させることで推定位置を中央に誘導している

# 軌跡平滑化用のバッファ
smooth_window = 10
pos_buffer = collections.deque(maxlen=smooth_window)
#collections.dequeは両側から出し入れ可能なリスト
#smoth_window=10は10個までデータを格納する
#このリストに11個目のデータを格納すると最も古いデータ(1個目)が削除される

start_overall = time.time() #プログラムの実行時間を計算

for file_name in file_list:
    df = pd.read_csv(file_name)
    
    # --- PF初期化 ---
    # 初期位置を通路の開始点 (50, 250) に合わせる
    particles = np.zeros((N_PARTICLES, 2))  #N_PARTICLES x 2 の行列を作成
    particles[:, 0] = 50.0 + np.random.normal(0, 2, N_PARTICLES) #50はスタートのx座標、np.random.normal(0, 2, N_PARTICLES)はノイズを加える(正規分布)
    particles[:, 1] = 250.0 + np.random.normal(0, 2, N_PARTICLES) #250はスタートのy座標、np.random.normal(0, 2, N_PARTICLES)はノイズを加える(正規分布)
    #np.random.normal(平均、標準偏差、出力配列のサイズ)
    #スタート付近に500個のパーティクルをランダムに配置する
    weights = np.ones(N_PARTICLES) / N_PARTICLES #要素がN_PARTICLESの1次元配列を作成
    #要素が1の配列をN_PARTICLESで割る(今回は500のためweight=0.02になる)
    
    estimated_positions = [] #最終的な位置を格納するリスト
    pos_buffer.clear() #pos_bufferを空にする
    angle = 0.0 #歩行者の初期角度を0度に設定する
    prev_time = df.loc[0, "timestamp"] #前回のタイムスタンプを取得

    for row in df.itertuples():
        if row.Index == 0: continue
        
        current_time = row.timestamp
        gx = row.gyro_z #一秒間に何回転したか
        if pd.isna(current_time) or pd.isna(gx): continue 

        dt = current_time - prev_time #前回のタイムスタンプと今回のタイムスタンプの差
        if dt <= 0 or dt > 1.0: #データの欠損などがあれば無視する
            prev_time = current_time
            continue
        prev_time = current_time

        # 1. 予測ステップ (ピクセル単位の STEP_LENGTH_P を使用)
        angle += gx * dt
        p_noise_dist = np.random.normal(0, SIGMA_STEP, N_PARTICLES) #ノイズを加える(距離)
        p_noise_angle = np.random.normal(0, SIGMA_ANGLE, N_PARTICLES) #ノイズを加える(角度)
        particles[:, 0] += (STEP_LENGTH_P + p_noise_dist) * np.cos(angle + p_noise_angle) #角度と距離から地図上のx軸の移動量を計算
        particles[:, 1] += (STEP_LENGTH_P + p_noise_dist) * np.sin(angle + p_noise_angle) #角度と距離から地図上のy軸の移動量を計算

        # 2. 更新 (Update): 狭くした地図で判定
        for j in range(N_PARTICLES):
            px, py = int(particles[j, 0]), int(particles[j, 1])
            if 0 <= px < w and 0 <= py < h:
                if binary_for_pf[py, px] == 0: #移動先が黒(壁)なら
                    weights[j] = 0.0 #重みを0=死亡
                else: #移動先が白(通路)なら
                    weights[j] = 1.0 #重みを1=生存
            else:
                weights[j] = 0.0
                
        # 3. リサンプリング
        sum_w = np.sum(weights) #生き残ったパーティクルの合計の重みを計算
        if sum_w > 0: #パーティクルが一つでも残った場合
            weights /= sum_w #重みを正規化(重みの合計が1になるように再調整する)
            indices = np.random.choice(range(N_PARTICLES), size=N_PARTICLES, p=weights) #重みをもとにランダムにパーティクルを選択
            particles = particles[indices] #選ばれたパーティクルで全体の配列を上書きする=正しい道にいるパーティクルのみが残るようにする
            weights.fill(1.0 / N_PARTICLES) #全ての重みの合計が1になるように正規化する
        else:
            # 全滅リカバリ
            ref_x, ref_y = (estimated_positions[-1] if estimated_positions else (50.0, 250.0)) #前回の地点をスタート地点にするために参照する。もし最初から失敗した場合はスタート地点を参照する
            particles[:, 0] = ref_x + np.random.normal(0, 10, N_PARTICLES) #基準点のx座標周りにランダムにパーティクルを配置する
            particles[:, 1] = ref_y + np.random.normal(0, 10, N_PARTICLES) #基準点のy座標周りにランダムにパーティクルを配置する
            weights.fill(1.0 / N_PARTICLES) #全ての重みの合計が1になるように正規化する

        # 4. 平均値の計算と平滑化
        raw_est_x = np.mean(particles[:, 0]) #生き残ったパーティクルの平均のx座標を計算
        raw_est_y = np.mean(particles[:, 1]) #生き残ったパーティクルの平均のy座標を計算
        pos_buffer.append((raw_est_x, raw_est_y)) #直近10個分のデータを格納
        
        avg_pos = np.mean(pos_buffer, axis=0) #直近10回分のx座標、y座標の平均値を計算。axis=0はxはxで、yはyで計算するという命令
        estimated_positions.append(tuple(avg_pos)) #計算された平均値を格納。描画に使用するためにtupleに変換する

    # プロット
    pos_arr = np.array(estimated_positions) #リスト形式(x1,y1),(x2,y2)...をNumpyの行列に変換
    plt.plot(pos_arr[:, 0], pos_arr[:, 1], linewidth=2, label=f'PF Scaled Path: {file_name}')
    #pos_arr[:,0]は全てのステップのx座標、pos_arr[:,1]は全てのステップのy座標
    #linewidth=2は線の太さを2にする
    #label=f'{file_name}'はラベルをファイル名にする
    plt.scatter(pos_arr[0, 0], pos_arr[0, 1], s=30, zorder=5)
    #全データの中の一番最初(スタート地点)を描画する
    #s=30は描画する点のサイズを指定する
    #zorder=5は描画順を5にする

end_overall = time.time()
print(f"パーティクル数: {N_PARTICLES}")
print(f"Overall time: {end_overall - start_overall:.2f} seconds")

plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
plt.title("地図制約+パーティクルフィルタ+中央補正のPDR経路")
plt.tight_layout()
plt.show()