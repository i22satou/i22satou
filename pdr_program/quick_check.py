# ============================================================================
# quick_check.py
#
# 【変更履歴】
# - 2026-09-03: [本研究独自] 新規作成。計測当日、学校にいるうちにCSVの健全性を
#               判定して撮り直しの要否をその場で決めるための読み取り専用ツール。
#
# 【なぜ必要か】
# 校正用データ(§27.1 A)と正解位置データ(同 B)は取り直しの効かないデータである。
# 学校にいる間なら撮り直しは無料だが、帰宅後に問題へ気づくと再訪が必要になる。
#
# 【アプリのSTOP時ダイアログとの役割分担】
# Androidアプリは記録停止時に「記録時間・行数・平均サンプリングHz・0.5秒超の欠落・
# バックグラウンド遷移」を端末画面に出す。ここではそれを繰り返さず、
# 計算しないと分からない項目だけを見る:
#   1. 歩行の物理的妥当性(歩数・歩調・平均歩幅・推定総移動距離)
#      2026-09-02の教訓「下流(PF・地図制約)を触る前に、推定総距離と歩調が物理的に
#      妥当かを先に確認する」を、その場で適用できるようにするのが主目的。
#   2. 方位の健全性(正味回転量・振れ幅・最大ジャンプ)
#      pdr_log_0805_1438.csv は歩行中に方位が正味+146度回っており、どの経路仮説でも
#      説明できなかった。同種の記録をその場で弾く。
#   3. 地点マーク(_waypoints.csv)の整合(seq数・本体CSVの時刻範囲に入っているか)
#   4. START直後の静止(計測手順が守られたか)
#
# 【設計方針】
# 歩数・歩幅・総距離は pdr_pf_improved.py の関数を import して計算する。
# 自前で計算し直すと本体の挙動とずれた数字を見て「問題なし」と誤判定するため、
# ここでは一切再実装しない(check_sensor_quality.py と同じ方針)。
#
# 【使い方】
#   python quick_check.py <pdr_log_XXXX.csv> [--expected-distance-m 89.3]
#       [--expected-net-rotation-deg -90] [--landmarks ground_truth/..._east_std.csv]
#   python quick_check.py --all          # data_dir の全CSVをまとめて判定
#   python quick_check.py --self-test    # 合成データで判定ロジックだけ確認
#
# 経路の想定距離(make_route_landmarks.py の出力より):
#   east_std 89.3 m / east_short 71.6 m / west_reverse 71.8 m / rehearsal 89.5 m
# ============================================================================

import argparse
import sys
import tempfile
from pathlib import Path

import numpy as np
import pandas as pd

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
import pdr_pf_improved as pdrmod  # noqa: E402

# 判定のしきい値。研究上の主張ではなく、その場で撮り直しを決めるための目安。
GAP_SEC = 0.5              # これを超えるサンプル間隔を欠落とみなす(アプリ側と同じ)
CADENCE_MIN, CADENCE_MAX = 1.2, 2.6      # 歩/s。普通歩行の生理的な範囲
STRIDE_MIN_M, STRIDE_MAX_M = 0.35, 0.95  # m/歩
# 想定距離からのずれ。2段階にしてある。歩幅校正ゲインが暫定値(2.10、評価用と同じ
# 3CSVから求めたもの)である間、実測は想定より2割ほど短く出ることが分かっている
# (CLAUDE.md「xが800に届かないのは歩幅校正の残差(約20%の過小)」)。ここを±15%で
# 異常にすると計測当日に全ての記録が異常判定になり、本当の問題を見落とす。
# 歩幅モデルを再同定(§27.2 C)したら DISTANCE_BAD を締めること。
DISTANCE_WARN = 0.15
DISTANCE_BAD = 0.40
ROTATION_TOL_DEG = 45.0    # 想定正味回転からの許容ずれ
STILL_SEC = 3.0            # START直後の静止を見る秒数
STILL_ACC_STD_MAX = 0.6    # 静止とみなす加速度ノルムの標準偏差(m/s^2)

OK, WARN, BAD = "OK  ", "警告", "異常"


class Report:
    """判定結果を貯めて、最後に総合判定を出す。"""

    def __init__(self, title):
        self.title = title
        self.lines = []
        self.worst = OK

    def add(self, level, item, detail):
        self.lines.append((level, item, detail))
        if level == BAD or (level == WARN and self.worst == OK):
            self.worst = level

    def show(self):
        print(f"\n{'=' * 72}\n{self.title}\n{'=' * 72}")
        for level, item, detail in self.lines:
            tag = {OK: "  [OK]  ", WARN: "  [警告] ", BAD: "  [異常] "}[level]
            print(f"{tag}{item:<22} {detail}")
        verdict = {
            OK: "この記録は使える。",
            WARN: "使えるかもしれないが、警告の内容を確認すること。",
            BAD: "この記録は使えない。その場で撮り直すこと。",
        }[self.worst]
        print(f"\n  総合判定: [{self.worst.strip()}] {verdict}")
        return self.worst


def check_continuity(df, rep):
    """記録の連続性。アプリのダイアログと重複するが、CSVが正しく届いたかの確認になる。"""
    n = len(df)
    t = df["timestamp"].to_numpy(float)
    duration = float(t[-1] - t[0]) if n > 1 else 0.0
    hz = (n - 1) / duration if duration > 0 else 0.0
    rep.add(OK, "行数・記録時間", f"{n} 行 / {duration:.1f} 秒 / 平均 {hz:.1f} Hz")

    d = np.diff(t)
    gaps = np.where(d > GAP_SEC)[0]
    if len(gaps) == 0:
        rep.add(OK, "欠落", f"{GAP_SEC}秒を超える間隔なし (最大 {d.max():.3f} 秒)")
    else:
        where = ", ".join(f"{t[i]-t[0]:.1f}秒付近({d[i]:.1f}秒)" for i in gaps[:4])
        more = f" 他{len(gaps)-4}件" if len(gaps) > 4 else ""
        rep.add(BAD, "欠落", f"{len(gaps)} 箇所: {where}{more}")
    return duration


def check_walking(df, rep, expected_distance_m):
    """歩行の物理的妥当性。2026-09-02の教訓をその場で適用する部分。"""
    df = df.copy()
    df["acc_mag"] = pdrmod.compute_acc_magnitude(df)
    df["step_acc"] = pdrmod.compute_step_acceleration(df["acc_mag"])
    dt_mean = df["timestamp"].diff().mean()
    hz = 1.0 / dt_mean if pd.notna(dt_mean) and dt_mean > 0 else None
    steps, valleys = pdrmod.detect_steps_smartpdr(df["step_acc"], hz)

    if len(steps) == 0:
        rep.add(BAD, "ステップ検出", "1歩も検出されなかった")
        return
    t = df["timestamp"].to_numpy(float)
    duration = float(t[-1] - t[0])
    cadence = len(steps) / duration if duration > 0 else 0.0

    lengths_px = np.array([
        pdrmod.estimate_smartpdr_step_length_px(df["step_acc"], p, v)
        for p, v in zip(steps, valleys)
    ])
    stride_m = float(np.mean(lengths_px) / pdrmod.M_TO_PIXEL)
    total_m = float(lengths_px.sum() / pdrmod.M_TO_PIXEL)

    lv = OK if CADENCE_MIN <= cadence <= CADENCE_MAX else WARN
    rep.add(lv, "歩数・歩調", f"{len(steps)} 歩 / {cadence:.2f} 歩/s "
                              f"(目安 {CADENCE_MIN}〜{CADENCE_MAX})")
    lv = OK if STRIDE_MIN_M <= stride_m <= STRIDE_MAX_M else WARN
    rep.add(lv, "平均歩幅", f"{stride_m:.2f} m/歩 "
                            f"(目安 {STRIDE_MIN_M}〜{STRIDE_MAX_M}、校正ゲイン適用後)")

    if expected_distance_m is None:
        rep.add(OK, "推定総移動距離", f"{total_m:.1f} m (--expected-distance-m 未指定)")
    else:
        err = (total_m - expected_distance_m) / expected_distance_m
        if abs(err) > DISTANCE_BAD:
            lv, note = BAD, f"異常 ±{DISTANCE_BAD*100:.0f}%超"
        elif abs(err) > DISTANCE_WARN:
            lv, note = WARN, "歩幅校正ゲインが暫定値のため2割程度の過小は既知"
        else:
            lv, note = OK, ""
        rep.add(lv, "推定総移動距離",
                f"{total_m:.1f} m / 想定 {expected_distance_m:.1f} m "
                f"({err*100:+.0f}%){'  ' + note if note else ''}")


def check_heading(df, rep, expected_net_deg):
    """方位の健全性。1438のような記録をその場で弾く。"""
    if "yaw_deg" not in df.columns or df["yaw_deg"].isna().all():
        rep.add(WARN, "方位(yaw_deg)", "列が無い。heading_source=android が使えない")
        return
    yaw = np.rad2deg(np.unwrap(np.deg2rad(df["yaw_deg"].to_numpy(float))))
    net = float(yaw[-1] - yaw[0])
    span = float(yaw.max() - yaw.min())
    jump = float(np.abs(np.diff(yaw)).max())

    if expected_net_deg is None:
        rep.add(OK, "方位の正味回転", f"{net:+.0f} 度 (--expected-net-rotation-deg 未指定)")
    else:
        lv = OK if abs(net - expected_net_deg) <= ROTATION_TOL_DEG else BAD
        rep.add(lv, "方位の正味回転",
                f"{net:+.0f} 度 / 想定 {expected_net_deg:+.0f} 度 "
                f"(許容 ±{ROTATION_TOL_DEG:.0f} 度)")
    lv = OK if span <= 270 else WARN
    rep.add(lv, "方位の振れ幅", f"{span:.0f} 度 (直角2回の経路なら180度程度が目安)")
    lv = OK if jump <= 30 else WARN
    rep.add(lv, "方位の最大ジャンプ", f"{jump:.1f} 度/サンプル")


def check_still_start(df, rep):
    """START後3〜5秒静止という計測手順が守られたか。"""
    t = df["timestamp"].to_numpy(float)
    head = df[df["timestamp"] <= t[0] + STILL_SEC]
    if len(head) < 10:
        rep.add(WARN, "START直後の静止", "先頭のサンプルが少なく判定できない")
        return
    acc = pdrmod.compute_acc_magnitude(head).to_numpy(float)
    std = float(np.nanstd(acc))
    lv = OK if std <= STILL_ACC_STD_MAX else WARN
    rep.add(lv, "START直後の静止",
            f"先頭{STILL_SEC:.0f}秒の加速度ノルム標準偏差 {std:.2f} "
            f"(静止の目安 {STILL_ACC_STD_MAX} 以下)")


def check_waypoints(csv_path, df, rep, landmarks_path):
    """地点マークの整合。押し忘れ・押し過ぎ・時計基準のずれをその場で見つける。"""
    wp_path = csv_path.with_name(csv_path.stem + "_waypoints.csv")
    if not wp_path.exists():
        rep.add(WARN, "地点マーク", f"{wp_path.name} が無い(校正用データなら正常)")
        return
    wp = pd.read_csv(wp_path)
    if not {"timestamp", "seq"} <= set(wp.columns):
        rep.add(BAD, "地点マーク", f"{wp_path.name} に timestamp / seq 列が無い")
        return

    t0, t1 = float(df["timestamp"].iloc[0]), float(df["timestamp"].iloc[-1])
    outside = wp[(wp["timestamp"] < t0) | (wp["timestamp"] > t1)]
    if len(outside):
        rep.add(BAD, "地点マークの時刻",
                f"{len(outside)} 点が本体CSVの範囲({t0:.1f}〜{t1:.1f})の外。"
                "時計基準が食い違っている")
    else:
        rep.add(OK, "地点マークの時刻", f"{len(wp)} 点すべてが本体CSVの時刻範囲内")

    seqs = sorted(wp["seq"].tolist())
    if seqs != list(range(1, len(seqs) + 1)):
        rep.add(BAD, "地点マークのseq", f"1から連番になっていない: {seqs}")
    elif landmarks_path is None:
        rep.add(OK, "地点マークのseq", f"1〜{len(seqs)} の連番 (--landmarks 未指定)")
    else:
        lm = pd.read_csv(landmarks_path)
        if len(lm) == len(seqs):
            rep.add(OK, "地点マークのseq",
                    f"{len(seqs)} 点で {Path(landmarks_path).name} と一致")
        else:
            rep.add(BAD, "地点マークのseq",
                    f"押下 {len(seqs)} 点 / 経路定義 {len(lm)} 点。"
                    f"{'押し忘れ' if len(seqs) < len(lm) else '押し過ぎ'}")

    if len(wp) >= 2:
        d = np.diff(sorted(wp["timestamp"].to_numpy(float)))
        rep.add(OK if d.min() > 0.5 else WARN, "地点マークの間隔",
                f"最小 {d.min():.1f} 秒 / 最大 {d.max():.1f} 秒")


def check_one(csv_path, expected_distance_m, expected_net_deg, landmarks_path):
    rep = Report(f"{csv_path.name}")
    try:
        df = pdrmod.safe_read_csv(csv_path)
        pdrmod.validate_log(df, csv_path.name)
    except Exception as e:
        rep.add(BAD, "読み込み", str(e))
        return rep.show()
    if pdrmod.GYRO_UNIT == "deg":
        df[["gyro_x", "gyro_y", "gyro_z"]] = np.deg2rad(df[["gyro_x", "gyro_y", "gyro_z"]])
    if len(df) < 2:
        rep.add(BAD, "行数", f"{len(df)} 行しかない")
        return rep.show()

    check_continuity(df, rep)
    check_walking(df, rep, expected_distance_m)
    check_heading(df, rep, expected_net_deg)
    check_still_start(df, rep)
    check_waypoints(csv_path, df, rep, landmarks_path)
    return rep.show()


def _self_test():
    """[本研究独自] 合成データで判定ロジックだけを確認する。外部ファイル不要。

    【重要】ここで使う信号はすべて架空であり、ここから出る数値を研究結果として
    報告してはならない。確認するのは「壊れた記録を異常と判定できるか」だけである。
    """
    print("--- self-test 開始(架空データ。数値は研究結果ではない) ---")
    hz, sec = 50.0, 20.0
    t = np.arange(0, sec, 1 / hz)


    # 欠落の検出
    r = Report("gap")
    d = pd.DataFrame({"timestamp": np.concatenate([t[:500], t[500:] + 2.0])})
    check_continuity(d, r)
    assert any(l == BAD and "欠落" in i for l, i, _ in r.lines), r.lines
    print("  OK: 欠落を異常として検出")

    r = Report("nogap")
    check_continuity(pd.DataFrame({"timestamp": t}), r)
    assert all(l == OK for l, _, _ in r.lines), r.lines
    print("  OK: 連続した記録を正常と判定")

    # 方位: 正味回転が想定と大きく違う場合
    r = Report("yaw")
    d = pd.DataFrame({"timestamp": t, "yaw_deg": np.linspace(0, 146, len(t))})
    check_heading(d, r, -90.0)
    assert any(l == BAD and "正味回転" in i for l, i, _ in r.lines), r.lines
    print("  OK: 想定と食い違う正味回転を異常として検出")

    r = Report("yaw_ok")
    check_heading(pd.DataFrame({"timestamp": t, "yaw_deg": np.linspace(0, -88, len(t))}),
                  r, -90.0)
    assert not any(l == BAD for l, _, _ in r.lines), r.lines
    print("  OK: 想定どおりの回転を正常と判定")

    # 方位が ±180 をまたぐ場合に unwrap が効いているか
    r = Report("wrap")
    yaw = np.linspace(170, 190, len(t))
    yaw = ((yaw + 180) % 360) - 180          # 170→-170 へ折り返す
    check_heading(pd.DataFrame({"timestamp": t, "yaw_deg": yaw}), r, 20.0)
    assert not any(l == BAD for l, _, _ in r.lines), r.lines
    print("  OK: ±180度をまたぐ方位を折り返しとして正しく扱う")

    # 静止判定
    rng = np.random.default_rng(0)
    for name, noise, expect_ok in (("still", 0.05, True), ("moving", 3.0, False)):
        r = Report(name)
        d = pd.DataFrame({"timestamp": t,
                          "acc_x": rng.normal(0, noise, len(t)),
                          "acc_y": rng.normal(0, noise, len(t)),
                          "acc_z": rng.normal(9.8, noise, len(t))})
        check_still_start(d, r)
        got_ok = all(l == OK for l, _, _ in r.lines)
        assert got_ok == expect_ok, (name, r.lines)
    print("  OK: START直後の静止/非静止を区別")

    # 地点マークの整合
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        main = td / "pdr_log_test.csv"
        pd.DataFrame({"timestamp": t}).to_csv(main, index=False)
        lm = td / "lm.csv"
        pd.DataFrame({"seq": range(1, 6), "label": list("abcde"),
                      "point_type": ["wall"] * 5, "x_px": [0] * 5,
                      "y_px": [0] * 5}).to_csv(lm, index=False)
        dfm = pd.DataFrame({"timestamp": t})

        # 範囲外のタイムスタンプ(時計基準の食い違い)
        pd.DataFrame({"timestamp": [1e9 + i for i in range(5)],
                      "seq": range(1, 6)}).to_csv(td / "pdr_log_test_waypoints.csv",
                                                  index=False)
        r = Report("wp_out"); check_waypoints(main, dfm, r, lm)
        assert any(l == BAD and "時刻" in i for l, i, _ in r.lines), r.lines

        # 押し忘れ(4点しか押していない)
        pd.DataFrame({"timestamp": t[:4] + 1.0, "seq": range(1, 5)}).to_csv(
            td / "pdr_log_test_waypoints.csv", index=False)
        r = Report("wp_miss"); check_waypoints(main, dfm, r, lm)
        assert any(l == BAD and "押し忘れ" in d for l, _, d in r.lines), r.lines

        # 正常
        pd.DataFrame({"timestamp": np.linspace(t[0]+1, t[-1]-1, 5),
                      "seq": range(1, 6)}).to_csv(
            td / "pdr_log_test_waypoints.csv", index=False)
        r = Report("wp_ok"); check_waypoints(main, dfm, r, lm)
        assert not any(l == BAD for l, _, _ in r.lines), r.lines
    print("  OK: 地点マークの時刻ずれ・押し忘れ・正常を区別")
    print("--- self-test 全て通過 ---")


def main():
    p = argparse.ArgumentParser(
        description="計測当日、その場でCSVの健全性を判定する。"
                    "詳細はこのファイル冒頭のコメントを参照。")
    p.add_argument("csv", nargs="*", type=Path, help="判定するpdr_log_*.csv")
    p.add_argument("--all", action="store_true",
                   help="map-configのdata_dir内の全センサーログを判定する。")
    p.add_argument("--map-config", type=Path,
                   default=SCRIPT_DIR / "map_configs" / "kanri_4f.json")
    p.add_argument("--expected-distance-m", type=float, default=None,
                   help="経路の想定歩行距離[m]。east_std=89.3 / east_short=71.6 など。")
    p.add_argument("--expected-net-rotation-deg", type=float, default=None,
                   help="経路の想定正味回転[度]。西→東の標準経路は約 -90 または +90。")
    p.add_argument("--landmarks", type=Path, default=None,
                   help="make_route_landmarks.pyが出したlandmarks CSV(点数の照合用)。")
    p.add_argument("--self-test", action="store_true")
    a = p.parse_args()

    if a.self_test:
        _self_test()
        return

    # M_TO_PIXEL・STEP_LENGTH_CALIBRATION_GAIN・GYRO_UNIT等のグローバルを設定する。
    # これを呼ばずに歩幅関数を使うと、本体と違う定数で計算した数字が出る。
    _cfg, fake_args = pdrmod.load_map_config_for_tool(a.map_config)

    targets = list(a.csv)
    if a.all:
        targets += sorted(f for f in fake_args.data_dir.glob("pdr_log_*.csv")
                          if pdrmod.is_sensor_log_csv(f))
    if not targets:
        p.error("判定するCSVを指定するか --all を付けてください。")

    worst = OK
    for c in targets:
        v = check_one(c, a.expected_distance_m, a.expected_net_rotation_deg, a.landmarks)
        if v == BAD or (v == WARN and worst == OK):
            worst = v
    if len(targets) > 1:
        print(f"\n{'=' * 72}\n全 {len(targets)} 件の最悪判定: [{worst.strip()}]\n{'=' * 72}")
    sys.exit(0 if worst != BAD else 1)


if __name__ == "__main__":
    main()
