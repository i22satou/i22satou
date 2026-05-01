#L字を描くプログラム

import numpy as np
import cv2

# 画像サイズ
width, height = 700, 700

# 黒背景（全部壁）
img = np.zeros((height, width), dtype=np.uint8)

# 通路の幅
road_width = 60
#通路の長さ
road_length = 350

# L字の通路を描く（白）
# 横通路
#cv2.rectangle(描画対象画像, 左上座標, 右下座標, 色, 塗りつぶし)

cv2.rectangle(img, (50, 220), (400, 220 + road_width), 255, -1)

# 縦通路
cv2.rectangle(img, (400-road_width, 220 + road_width), (400 , 220 + road_width + road_length), 255, -1)

# 保存
cv2.imwrite("L_map.png", img)

# 表示
cv2.imshow("L map", img)
cv2.waitKey(0)
cv2.destroyAllWindows()