# 何も補正を加えていないPDR軌跡を地図上に表示するプログラム
import argparse
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/private/tmp/matplotlib")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from PIL import Image

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    cv2 = None
    HAS_CV2 = False

try:
    import japanize_matplotlib  # noqa: F401
except ImportError:
    japanize_matplotlib = None


# ============================================================
# 1. 定数・スケールの設定
# ============================================================
BASE_DIR = Path(__file__).resolve().parent

REAL_LENGTH_M = 8.75
MAP_LENGTH_P = 350.0
M_TO_PIXEL = MAP_LENGTH_P / REAL_LENGTH_M

# test7.py と同じ開始位置
START_X = 70.0
START_Y = 250.0

# CSVに有効なx,yがない場合の未補正積分用。
# 旧real.pyの 0.0175m / 20ms と同じ速度。
DEFAULT_SPEED_MPS = 0.875
MAX_DT = 1.0


def parse_args():
    parser = argparse.ArgumentParser(
        description="補正なしのPDR軌跡をL_map.png上に表示します。"
    )
    parser.add_argument(
        "--data-dir",
        type=Path,
        default=BASE_DIR,
        help="pdr_log_*.csv があるディレクトリ",
    )
    parser.add_argument(
        "--map",
        type=Path,
        default=BASE_DIR / "L_map.png",
        help="使用するマップ画像",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=DEFAULT_SPEED_MPS,
        help="CSVにx,yがない場合に使う一定速度[m/s]",
    )
    parser.add_argument(
        "--save",
        type=Path,
        default=None,
        help="結果画像の保存先。指定するとPNGなどで保存します。",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="グラフウィンドウを表示しません。--save と併用すると便利です。",
    )
    return parser.parse_args()


def validate_log(df, file_name):
    required = ["timestamp", "gyro_z"]
    missing = [col for col in required if col not in df.columns]
    if missing:
        raise ValueError(f"{file_name} に必要な列がありません: {', '.join(missing)}")

    df = df.dropna(subset=required).reset_index(drop=True)
    df = df[df["timestamp"].diff().fillna(1) > 0].reset_index(drop=True)
    return df


def load_map(img_path):
    if not img_path.exists():
        raise FileNotFoundError(f"マップ画像を読み込めません: {img_path}")

    if HAS_CV2:
        img_gray = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
        if img_gray is None:
            raise FileNotFoundError(f"マップ画像を読み込めません: {img_path}")
    else:
        img_gray = np.array(Image.open(img_path).convert("L"))

    binary = np.where(img_gray > 127, 255, 0).astype(np.uint8)
    return binary


def has_valid_xy(df):
    if not {"x", "y"}.issubset(df.columns):
        return False
    xy = df[["x", "y"]].dropna()
    return len(xy) >= 2


def raw_xy_positions(df):
    """
    ログにx,y列がある場合は、端末側で記録された未補正PDR座標としてそのまま使う。
    x,yはメートル単位として扱い、地図のピクセル座標へ平行移動・拡大だけ行う。
    """
    xy = df[["x", "y"]].dropna()
    x_px = START_X + xy["x"].to_numpy() * M_TO_PIXEL
    y_px = START_Y + xy["y"].to_numpy() * M_TO_PIXEL
    return np.column_stack([x_px, y_px])


def integrate_gyro_positions(df, speed_mps):
    """
    x,y列がないログ用の最小限の未補正PDR。
    壁判定、歩幅校正、方位補正、平滑化、パーティクルフィルタは使わない。
    """
    x = START_X
    y = START_Y
    angle = 0.0
    positions = [(x, y)]

    for i in range(1, len(df)):
        row = df.iloc[i]
        prev_row = df.iloc[i - 1]
        dt = row["timestamp"] - prev_row["timestamp"]
        if dt <= 0 or dt > MAX_DT:
            continue

        angle += row["gyro_z"] * dt
        move_px = speed_mps * dt * M_TO_PIXEL
        x += move_px * np.cos(angle)
        y += move_px * np.sin(angle)
        positions.append((x, y))

    return np.array(positions)


def main():
    args = parse_args()
    data_dir = args.data_dir.resolve()
    img_path = args.map.resolve()

    file_list = sorted(data_dir.glob("pdr_log_*.csv"))
    if not file_list:
        raise FileNotFoundError(f"CSVファイルが見つかりません: {data_dir}")

    binary = load_map(img_path)
    h, w = binary.shape

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.imshow(binary, cmap="gray")
    ax.set_xlim(0, w)
    ax.set_ylim(h, 0)

    for file_path in file_list:
        df = pd.read_csv(file_path)
        df = validate_log(df, file_path.name)
        if len(df) < 2:
            print(f"[{file_path.name}] 有効なデータが少ないためスキップします。")
            continue

        if has_valid_xy(df):
            positions = raw_xy_positions(df)
            source = "log x,y"
        else:
            positions = integrate_gyro_positions(df, args.speed)
            source = f"gyro_z + {args.speed:.3f}m/s"

        if len(positions) < 2:
            print(f"[{file_path.name}] 軌跡を作成できませんでした。")
            continue

        ax.plot(
            positions[:, 0],
            positions[:, 1],
            linewidth=2,
            label=f"Raw PDR: {file_path.name}",
        )
        ax.scatter(positions[0, 0], positions[0, 1], s=50, color="red", zorder=5)
        ax.scatter(positions[-1, 0], positions[-1, 1], s=50, marker="x", zorder=5)
        print(
            f"[{file_path.name}] 点数={len(positions)}  入力={source}  "
            f"最終位置=({positions[-1, 0]:.1f}, {positions[-1, 1]:.1f})"
        )

    title = "補正なしPDR軌跡"
    if japanize_matplotlib is None:
        title = "Raw PDR trajectory"
    ax.set_title(title)
    ax.set_xlabel("X [pixel]")
    ax.set_ylabel("Y [pixel]")
    ax.legend(bbox_to_anchor=(1.05, 1), loc="upper left")
    fig.subplots_adjust(right=0.72)

    print(f"使用スケール: {M_TO_PIXEL:.1f} px/m")

    if args.save is not None:
        save_path = args.save.resolve()
        save_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save_path, dpi=200, bbox_inches="tight")
        print(f"結果画像を保存しました: {save_path}")

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
