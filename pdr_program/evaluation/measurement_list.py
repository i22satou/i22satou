# ============================================================================
# measurement_list.py
#
# 【変更履歴】
# - 2026-09-18: [本研究独自] 新規作成。計測日に書く一覧表(計測一覧)を読む共通部品。
#               calibrate_step_length.py(校正用の行)と、今後作る run_evaluation.py
#               (評価用の行)が同じ読み方・同じ検査を使うために分けた。
#
# 【一覧表の形式】1回の計測で1枚。UTF-8でもExcelの既定(Shift-JIS)保存でも読める。
#   file,purpose,route,distance_m,speed,use,memo
#   calib/pdr_log_0925_1010.csv,calib,,30.0,slow,1,
#   pdr_log_0925_1030.csv,eval,east_std,,,1,
#
#   file      : センサーCSVの名前。data_dirからの相対パス、または絶対パス。
#               校正用CSVはdata_dir直下に置かず、サブフォルダ(例: calib/)へ入れること。
#               直下に置くと、本体(pdr_pf_improved.py)が開始位置の登録を求めて
#               クリック待ちで止まる(開始位置が未登録のCSVはすべてそうなる)。
#   purpose   : calib = 歩幅校正用の直線歩行 / eval = 比較実験用の経路歩行
#   route     : evalのとき必須。ground_truth/kanri_4f_landmarks_<route>.csv の <route>
#   distance_m: calibのとき必須。実際に歩いた距離[m](巻尺などで測った値)
#   speed     : calibのとき slow / normal / fast。空欄は「不明」として扱う
#   use       : 1 = 使う / 0 = 使わない(撮り直した記録も消さずに残せるように)。空欄は1
#   memo      : 自由記述(プログラムは読まない)
#
# 一覧表の誤り(必須欄の空欄・数値でない距離・想定外の速さ)は、黙って行を飛ばさず、
# 表計算ソフトの行番号付きで全件をまとめてエラーにする。計測日に書いた表をその場で
# 直せるようにするため。
# ============================================================================

import csv
import io
from pathlib import Path

import pandas as pd

COLUMNS = ["file", "purpose", "route", "distance_m", "speed", "use", "memo"]
PURPOSES = {"calib", "eval"}
SPEEDS = ["slow", "normal", "fast"]
_TRUE_WORDS = {"", "1", "true", "yes", "y", "o", "○"}
_FALSE_WORDS = {"0", "false", "no", "n", "x", "×"}


def read_list(path):
    """一覧表を読み、全列を文字列(前後の空白を除去、欠損は空文字)で返す。

    memo欄にカンマを書くと列が増える(テキストエディタで書いた場合に起きやすい)。
    memoが最後の列なら、余った欄をカンマでつなぎ直してmemoへ戻す。pandasに任せると
    先頭の列が行ラベル扱いになって列が黙ってずれるため、csvモジュールで読む。
    """
    path = Path(path)
    raw = path.read_bytes()
    for encoding in ("utf-8-sig", "cp932"):
        try:
            text = raw.decode(encoding)
            break
        except UnicodeDecodeError:
            continue
    else:
        raise ValueError(f"{path}: UTF-8でもShift-JISでも読めません")

    records = list(csv.reader(io.StringIO(text)))
    if not records:
        raise ValueError(f"{path}: 中身が空です")
    header = [c.strip().lower() for c in records[0]]
    missing = [c for c in ("file", "purpose") if c not in header]
    if missing:
        raise ValueError(f"{path}: 必須列がありません: {missing}(列は {COLUMNS})")

    rows, errors = [], []
    for sheet_row, record in enumerate(records[1:], start=2):
        if len(record) > len(header):
            if header[-1] != "memo":
                errors.append(f"{sheet_row}行目: 欄の数が見出しより多い(カンマの入れすぎ?)")
                continue
            record = record[:len(header) - 1] + [",".join(record[len(header) - 1:])]
        record = record + [""] * (len(header) - len(record))
        values = dict(zip(header, (v.strip() for v in record)))
        rows.append({"sheet_row": sheet_row, **{c: values.get(c, "") for c in COLUMNS}})
    if errors:
        raise ValueError(f"{path} に誤りがあります:\n  " + "\n  ".join(errors))

    df = pd.DataFrame(rows, columns=["sheet_row", *COLUMNS])
    # 完全な空行は、表計算ソフトで末尾に残りやすいので読み飛ばす。
    return df[(df[COLUMNS] != "").any(axis=1)].reset_index(drop=True)


def _parse_use(value):
    word = value.lower()
    if word in _TRUE_WORDS:
        return True
    if word in _FALSE_WORDS:
        return False
    raise ValueError(f"use は 1 か 0 で書いてください(「{value}」)")


def load_measurement_list(path, purpose):
    """一覧表から、purposeが一致しuse=1の行だけを検査して返す。

    calibの行は distance_m(float)、eval の行は route を検査する。
    誤りがあれば全件をまとめてValueErrorにする(1件ずつ直させない)。
    """
    if purpose not in PURPOSES:
        raise ValueError(f"purpose は {sorted(PURPOSES)} のどれか: {purpose}")
    df = read_list(path)
    errors = []
    uses = []
    for _, row in df.iterrows():
        where = f"{row['sheet_row']}行目({row['file'] or 'file空欄'})"
        if row["purpose"] not in PURPOSES:
            errors.append(f"{where}: purpose は calib か eval(「{row['purpose']}」)")
        try:
            uses.append(_parse_use(row["use"]))
        except ValueError as error:
            errors.append(f"{where}: {error}")
            uses.append(False)
    df["use"] = uses

    selected = df[(df["purpose"] == purpose) & df["use"]].copy()
    distances = []
    for _, row in selected.iterrows():
        where = f"{row['sheet_row']}行目({row['file'] or 'file空欄'})"
        if not row["file"]:
            errors.append(f"{where}: file が空欄")
        if purpose == "calib":
            try:
                distance = float(row["distance_m"])
                if not distance > 0:
                    raise ValueError
            except ValueError:
                errors.append(f"{where}: distance_m は正の数[m](「{row['distance_m']}」)")
                distance = float("nan")
            distances.append(distance)
            if row["speed"] and row["speed"].lower() not in SPEEDS:
                errors.append(f"{where}: speed は {SPEEDS} か空欄(「{row['speed']}」)")
        elif not row["route"]:
            errors.append(f"{where}: eval の行は route が必須")

    if errors:
        raise ValueError(f"{path} に誤りがあります:\n  " + "\n  ".join(errors))
    if purpose == "calib":
        selected["distance_m"] = distances
        selected["speed"] = selected["speed"].str.lower()
    duplicated = selected["file"][selected["file"].duplicated()].tolist()
    if duplicated:
        raise ValueError(f"{path}: 同じファイルが2回以上あります: {duplicated}")
    return selected.reset_index(drop=True)


def resolve_csv_path(file_value, data_dir):
    """一覧表の file 欄を実際のパスにする(絶対パスならそのまま、他はdata_dir基準)。"""
    path = Path(file_value).expanduser()
    return path if path.is_absolute() else Path(data_dir) / path
