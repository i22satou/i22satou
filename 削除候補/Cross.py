#十字路を描くプログラム

import numpy as np
import cv2

# 画像サイズ
width, height = 700, 700

# 黒背景
img = np.zeros((height, width), dtype=np.uint8)

# 通路設定
road_width = 60
road_length = 350
road_half_length = road_length // 2  # ←整数にする

# 中心座標
cx, cy = width // 2, height // 2

# 横通路
cv2.rectangle(img, (cx - road_half_length, cy - road_width // 2) , (cx + road_half_length, cy + road_width // 2), 255, -1)

# 縦通路
cv2.rectangle(img, (cx - road_width // 2, cy - road_half_length), (cx + road_width // 2, cy + road_half_length), 255, -1)

# 保存・表示
cv2.imwrite("cross_map.png", img)
cv2.imshow("Cross Map", img)
cv2.waitKey(0)
cv2.destroyAllWindows()