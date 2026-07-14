##地図画像のピクセルを求めるプログラム
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


BASE_DIR = Path(__file__).resolve().parent
IMAGE_PATH = BASE_DIR / "kanri_4f_binary_final3.png"


def main():
    if not IMAGE_PATH.exists():
        raise FileNotFoundError(
            f"地図画像が見つかりません: {IMAGE_PATH}"
        )

    image = np.array(Image.open(IMAGE_PATH))

    fig, ax = plt.subplots(figsize=(14, 6))
    ax.imshow(image, cmap="gray", origin="upper")
    ax.set_title(
        "実際の長さが分かっている区間の両端を、順番に2か所クリックしてください"
    )
    ax.set_xlabel("X座標 [px]")
    ax.set_ylabel("Y座標 [px]")
    ax.grid()

    print("実際の長さが分かっている区間の両端を2か所クリックしてください。")

    points = plt.ginput(2, timeout=0)

    if len(points) != 2:
        print("2点を取得できませんでした。")
        return

    (x1, y1), (x2, y2) = points

    dx = x2 - x1
    dy = y2 - y1
    pixel_distance = np.hypot(dx, dy)

    print()
    print("===== 測定結果 =====")
    print(f"始点: x={x1:.2f}, y={y1:.2f}")
    print(f"終点: x={x2:.2f}, y={y2:.2f}")
    print(f"X方向の差: {abs(dx):.2f} px")
    print(f"Y方向の差: {abs(dy):.2f} px")
    print(f"2点間距離: {pixel_distance:.2f} px")

    ax.plot(
        [x1, x2],
        [y1, y2],
        color="red",
        linewidth=2,
    )
    ax.scatter(
        [x1, x2],
        [y1, y2],
        color="blue",
        s=60,
        zorder=5,
    )

    ax.text(
        (x1 + x2) / 2,
        (y1 + y2) / 2,
        f"{pixel_distance:.1f} px",
        color="red",
        fontsize=12,
        bbox={
            "facecolor": "white",
            "alpha": 0.8,
            "edgecolor": "red",
        },
    )

    fig.canvas.draw()
    plt.show()

    real_length_m = input(
        "クリックした区間の実際の長さをm単位で入力してください: "
    )

    try:
        real_length_m = float(real_length_m)
    except ValueError:
        print("実際の長さは数値で入力してください。")
        return

    if real_length_m <= 0:
        print("実際の長さは0より大きい値にしてください。")
        return

    scale_px_per_m = pixel_distance / real_length_m
    meters_per_pixel = real_length_m / pixel_distance

    print()
    print("===== 縮尺計算結果 =====")
    print(f"画像上の長さ: {pixel_distance:.2f} px")
    print(f"実際の長さ: {real_length_m:.3f} m")
    print(f"縮尺: {scale_px_per_m:.4f} px/m")
    print(f"1ピクセル: {meters_per_pixel:.5f} m")
    print()
    print("kanri_4f.jsonには次の値を設定してください。")
    print(f'"scale_px_per_m": {scale_px_per_m:.4f}')


if __name__ == "__main__":
    main()
