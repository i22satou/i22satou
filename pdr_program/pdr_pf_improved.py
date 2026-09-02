# ============================================================================
# pdr_pf_improved.py
#
# 【変更履歴】
# 全履歴はCHANGELOG.md。変更したらCHANGELOG.mdの先頭へ日付付きで1項目追加すること。
# 直近のみ下記に残す(古い項目は消してよい):
# - 2026-09-02: PDR上流(距離・方位)の系統誤差を4点修正。(1)ステップ検出が歩行加速度
#               の第2高調波に反応し歩数を約1.7〜1.8倍に過検出していた問題を、最短
#               ピーク間隔を上限歩調ベース(MAX_STEP_FREQUENCY_HZ)へ変更して是正。
#               (2)歩幅の約1/2過小推定へ校正ゲインSTEP_LENGTH_CALIBRATION_GAINを追加。
#               (1)(2)は逆向きの誤差でファイルごとに異なる割合で相殺しており、
#               memo/sensor_mystery.mdの1441/1442問題の真因だった。(3)PF重みの
#               「方位ズレの重み」が提案分布の二重計上だったため削除。(4)初期方位校正に
#               歩行開始基準(walking、既定OFF)を追加。(5)乱数シードのCSV独立化と、
#               複数経路仮説PFの方位補正のroute_constraint_mode分岐。
#               詳細はCHANGELOG.md 2026-09-02(1)〜(5)項。
# - 2026-08-30: [本研究独自] 複数経路仮説PFの交差点分岐選択に方位重み付け
#               (choose_branch_by_heading、pdr_route_graph.py)を追加したが、正解経路
#               との照合で一様乱択を上回らず、既定を一様乱択相当(σ=100000)へ戻した。
#               関数は--multi-hypothesis-branch-heading-sigma-degで再検討可能。
#
# 【既知開始位置の設定】
# - 未登録のPDRデータは、地図上で実際の計測開始位置をクリックして指定する。
# - クリックした開始位置はstart_positions.csvへ自動保存する。
# - 登録済みのPDRデータは、次回以降、保存済みの開始位置を自動使用する。
# - 開始位置が地図外、壁内、または移動不可領域の場合は再選択を要求する。
# - 開始位置を変更する場合は、start_positions.csvから対象CSVの行を削除し、
#   プログラムを再実行して開始位置をクリックし直す。
#
# 【初期方位補正】
# - センサ方位の0度と地図画像上の0度は必ずしも一致しないため、
#   計測開始時のセンサ方位を基準として地図座標系へ変換する。
# - --initial-heading-degで、計測開始時の地図上の進行方向を指定する。
# - 地図画像上では、右方向を0度、下方向を90度、
#   左方向を180度、上方向を-90度として扱う。
# - 例えば右方向へ歩き始めた場合は--initial-heading-deg 0、
#   上方向へ歩き始めた場合は--initial-heading-deg -90を指定する。
# - 複数のCSVで開始方向が異なる場合は、開始方向ごとに分けて実行する。
#
# 【方位推定方法】
# - --heading-source gyroを指定した場合は、ジャイロ、加速度、
#   Madgwickフィルタ、利用可能な場合は磁気センサを用いて方位を推定する。
# - --heading-source androidを指定した場合は、
#   AndroidのTYPE_ROTATION_VECTORから記録したyaw_degを使用する。
# - android方式を使用する場合は、入力CSVにyaw_deg列が必要である。
# - Android方位を使用する場合は、先頭の複数サンプルを円平均し、
#   計測開始時の基準方位として使用する。
# - --heading-calibration-samplesで、基準方位の計算に使用する
#   先頭サンプル数を指定できる。既定値は30サンプルである。
# - gyro方式とandroid方式を同一データ、同一開始位置、
#   同一乱数シードで比較できる。
#
# 【経路制約モード】
# - --route-constraint-mode none:
#     手動設定したroute_pointsを使用しない。
#     センサ方位、歩幅、壁制約、移動様態適応PFによって位置を推定する。
# - --route-constraint-mode prefer:
#     route_points周辺の粒子を優遇する。
#     経路外粒子は残すが、設定した経路周辺より低い重みを与える。
# - --route-constraint-mode enforce:
#     route_pointsから作成した経路マスク外を移動不可として扱う。
#     手動経路へ強く拘束した比較用の方式である。
# - preferとenforceは手動設定した経路を使用するため、
#   正解に近い経路を事前に与えた比較条件として扱う。
# - preferやenforceの軌跡が設定経路へ近づいても、
#   その結果だけでは位置推定精度の向上を証明したことにはならない。
#
# 【経路線分切替】(route_source=manual + prefer/enforce のとき)
# - クリック開始位置に最も近い経路線分を初期線分として選ぶ。
# - 曲がり検出はturn_pendingとして保持し、PF推定位置が設定曲がり角付近へ到達した
#   時だけ次の線分へ進める(最近傍線分を毎歩選び直す方式は使わない)。
# - 判定順は「センサー方位 → 移動様態判定 → 経路線分切替 → 方位補正 → PF更新」。
# - 切替後の線分は維持し、前の通路方向へは戻さない。
# - route_constraint_mode=noneでは経路線分による方位補正を使用しない
#   (複数経路仮説PF側も2026-09-02に同じ挙動へ揃えた)。
#
# 【移動様態判定】
# - 歩行者の移動様態をSTOPPED、STRAIGHT、TURNINGの3状態で判定する。
# - 曲がり判定は、一定時間内の方位変化とヨーレートのAND条件で行う。
# - ヨーレートは瞬間最大値ではなく75パーセンタイルを使い、手ぶれの影響を抑える。
# - TURNING判定にヒステリシスを入れ、STRAIGHT/TURNINGの頻繁な振動を防ぐ。
#
# 【移動様態適応型パーティクルフィルタ】
# - 直進/曲がり/滞留で粒子数・歩幅ノイズ・方位ノイズを切り替える
#   (現在のJSON設定は直進250/曲がり600/滞留100)。
# - 粒子数の変更は現在の重みに従うリサンプリングで行い、増加時は複製粒子へ
#   微小な位置摂動を加える。壁衝突判定・全滅復帰も可変粒子数に対応する。
# - パラメータはmap_configs/*.jsonのadaptive_pfセクションで変更する。
#   先行研究は直進10/屈折20だが、本研究は対象地図が複雑なため比率を保って拡大した。
#
# 【PF診断値】(CSVごとに処理終了時へログ出力)
# - 有効粒子率: 壁へ衝突しなかった粒子の割合。低いと壁に当たる粒子が多い。
# - 経路内粒子率: 経路帯マスク内にある粒子の割合。noneではマスクが全領域のため1に近い。
# - Neff: 正規化後の重みから求める実効サンプルサイズ。低いと少数の粒子へ重みが集中。
# - 位置分散: 粒子のx分散とy分散の和。大きいと推定位置の不確実性が高い。
#
# 【キャッシュ判定】
# 入力CSVのmtime・サイズに加え、開始位置・経路制約モード・方位設定・route_points・
# 経路重みが一致した場合だけ再利用する(実行条件を変えた後に古い結果を誤って
# 使い回さないため)。キャッシュはプロセス内のみで、監視モードの再描画用。
#
# 【注意】
# - --heading-source androidは入力CSVにyaw_deg列が必要。
# - **AndroidアプリでSTARTを押した後は3〜5秒静止してから歩き始め、歩行中は端末の
#   持ち方を変えないこと。** 初期方位の基準がここで決まるため、守られていない記録は
#   方位が経路と整合しなくなる(2026-09-02にpdr_log_0805_1438/1441で実害を確認。
#   詳細はmemo/heading_calibration.md)。
# - --initial-heading-degは1回の実行で全CSVへ共通に適用される。開始方向が異なる
#   CSVは分けて実行する。
# - 二値地図では廊下と部屋が同じ移動可能領域になる場合があり、noneモードでは
#   部屋への誤進入や誤った通路選択が起きうる。
# - prefer/enforceは手動route_pointsを使うため、最終提案方式ではなく比較方式。
# - 軌跡画像が正解経路に近く見えることだけでは位置推定精度の改善を証明できない。
#   RMSE等には時刻対応した正解位置データが別途必要(evaluate_accuracy.py)。
#
# 【本プログラムの研究的位置づけ(卒業論文 第2章・第5章の執筆用メモ)】
# 本プログラムは以下2件の先行研究を基礎とし、そこに本研究独自の拡張を加えている。
# 該当処理の直前コメントに [SmartPDR] [先行研究:移動様態PF] [本研究独自] のタグを
# 付け、どの処理がどちらに由来するかをコード上でも追跡できるようにしている。
#
# 参考文献1: SmartPDR: Smartphone-Based Pedestrian Dead Reckoning for Indoor
#            Localization
#   - 加速度のHPF/LPFによるステップ信号生成、ピーク・谷・傾き条件によるステップ
#     検出(detect_steps_smartpdr)、4乗根式/対数式を切り替える歩幅推定式
#     (estimate_smartpdr_step_length_px)の基礎として使用。
#
# 参考文献2: 秋山高行ほか「移動様態に応じたパーティクルフィルタによる歩行者自律
#            測位方式の提案と評価」(FIT2013)
#   - 歩行者の移動様態(直進/屈折/滞留)を判定し、様態ごとにパーティクル数・歩幅と
#     方位のノイズ分散を変更するという考え方の基礎として使用
#     (behavior_parameters, detect_move_behavior)。
#   - 原論文は8秒間に30度以上変化した場合に屈折と判定する単純な閾値判定、粒子数は
#     直進10個・屈折20個、壁判定は通過0/移動可1の二値重みである。
#
# 本研究独自の拡張(上記2件のどちらにも存在しない要素):
#   - 壁までの距離場(distance_transform_edt)に基づく連続的な尤度重み付け。
#     原論文の0/1二値重みに対する拡張(ParticleFilterPDR.update)。
#   - ヨーレート75パーセンタイル・AND条件での屈折開始判定・ヒステリシスによる
#     STRAIGHT/TURNINGの頻繁な振動抑制(detect_move_behavior)。原論文は単純な
#     角度変化閾値のみで、ヨーレートの外れ値対策やヒステリシスは持たない。
#   - 二値地図から抽出したroute_pointsによる経路帯マスク・曲がり角近傍判定・
#     経路線分に連動した方位補正(route_constraint_mode, build_route_mask,
#     correct_heading_with_route_segment, advance_route_segment,
#     is_near_route_corner)。両先行研究には地図の通路方向情報を用いる処理は
#     存在せず、本研究が追加した「地図形状を用いた適応制御」の中心部分にあたる。
#   - 地図規模に合わせた適応的パーティクル数(直進250・曲がり600・滞留100)と、
#     重みに基づくリサンプリングでの粒子数増減(resize_particle_set)。
#   - 既知開始位置のクリック登録・再利用(start_positions.csv)、CSVフォルダ監視
#     による自動再描画(CSVHandler)など、複数経路・複数試行を再現性高く比較する
#     ための実験基盤としての拡張。
#   - 初期方位のセンサ→地図座標系への校正、方位推定方法(gyro/android)の選択、
#     クリック開始位置に最も近い経路線分を初期線分として選ぶ処理など、
#     pdr_pf_clickstart.pyには無い本ファイル固有の拡張(詳細は上部の変更履歴)。
#
# 現状のroute_constraint_mode=prefer/enforceは正解に近い経路を手動入力した比較用
# 方式(進捗メモでいう方式D相当)であり、本研究の最終提案方式そのものではない。
# 手動経路への依存を減らすことが今後の課題(進捗メモ 4.1, 5.3, 20.2 を参照)。
# ============================================================================

import numpy as np
import pandas as pd
import os
import json
import tempfile
import logging
import collections
import time
import argparse
import threading
import glob
import zlib
from pathlib import Path
from enum import Enum

# Ensure matplotlib config directory is writable and not hardcoded
mpl_config_dir = os.environ.get("MPLCONFIGDIR")
if not mpl_config_dir:
    mpl_config_dir = tempfile.mkdtemp(prefix="matplotlib-")
    os.environ["MPLCONFIGDIR"] = mpl_config_dir
else:
    try:
        os.makedirs(mpl_config_dir, exist_ok=True)
    except Exception:
        # fallback to temp dir
        mpl_config_dir = tempfile.mkdtemp(prefix="matplotlib-")
        os.environ["MPLCONFIGDIR"] = mpl_config_dir

import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from scipy import ndimage
from PIL import Image

# [本研究独自] 経路帯マスク抽出・通路グラフ化関連の関数群(2026-08-16、ファイル
# 肥大化を受けてpdr_route_graph.pyへ切り出した)。関数の中身は移動前と完全に同じで、
# ここで再importすることで、このファイル内・他スクリプトからの呼び出し方は
# 従来通り(pdr_pf_improved.build_skeleton_graph(...)等)のまま変更していない。
from pdr_route_graph import (
    extract_auto_route_mask,
    extract_ordered_centerline,
    build_skeleton_graph,
    simplify_skeleton_graph,
    build_route_graph_topology,
    nearest_edge_position,
    choose_branch_by_heading,
)

try:
    import japanize_matplotlib  # 日本語用のライブラリ
except ImportError:
    japanize_matplotlib = None

try:
    from watchdog.observers import Observer
    from watchdog.events import FileSystemEventHandler
    HAS_WATCHDOG = True
except ImportError:
    Observer = None
    FileSystemEventHandler = object
    HAS_WATCHDOG = False

try:
    from ahrs.filters import Madgwick
    from scipy.spatial.transform import Rotation as R
    HAS_AHRS = True
except Exception:
    Madgwick = None
    R = None
    HAS_AHRS = False

# ============================================================
# 1. 管理棟用の既定値（通常はJSONで上書き）
# ============================================================
BASE_DIR = Path(__file__).resolve().parent
DEFAULT_MAP_CONFIG = BASE_DIR / "map_configs" / "kanri_4f.json"
START_POSITION_FILE = BASE_DIR / "start_positions.csv"
RESULTS_DIR = BASE_DIR / "results"
M_TO_PIXEL = 11.4
DEFAULT_STEP_GAIN = 1.0
PF_EROSION_RADIUS_PX = 1

# PF重み・経路優先設定
WALL_WEIGHT_SIGMA = 0.60
WALL_WEIGHT_FLOOR = 0.50
OFF_ROUTE_WEIGHT = 0.15
ROUTE_WIDTH_PX = 18.0
ROUTE_HEADING_WEIGHT = 1.5
ROUTE_CORNER_THRESHOLD_PX = 25.0
ROUTE_CONSTRAINT_MODE = "prefer"
ROUTE_POINTS = []
# [本研究独自] 経路帯マスクの生成元。manual=JSONのroute_pointsを人手で描画、
# auto=二値地図から通路領域を自動抽出(extract_auto_route_mask)。
ROUTE_SOURCE = "manual"
AUTO_ROUTE_MAX_HALF_WIDTH_PX = 20.0
AUTO_ROUTE_DILATION_PX = 0.0
# [本研究独自] 大きな部屋の壁際の帯を通路候補から除外するか(extract_auto_route_mask
# のexclude_wide_rooms引数)。既定は無効(既存のroute_source=auto比較実験の結果を
# 変えないため)。詳細はextract_auto_route_mask()のコメント・memo/route_source_auto.md参照。
AUTO_ROUTE_EXCLUDE_WIDE_ROOMS = False
AUTO_ROUTE_EXCLUDE_WIDE_ROOMS_RADIUS_PX = None
# [本研究独自] route_source=autoの経路帯マスクを細線化して順序付き中心線を作り、
# ROUTE_POINTSとして曲がり角連動の方位補正に使うかどうか。既定は無効(空間マスクの
# みのこれまでの挙動を維持)。extract_ordered_centerline()参照。
AUTO_ROUTE_CENTERLINE_ENABLED = False
AUTO_ROUTE_CENTERLINE_SIMPLIFY_PX = 10.0
# [本研究独自] 抽出した中心線の「始点・終点間の直線距離」に対する実経路長の比率が
# これを超えたら、通路網がループ状で木の直径探索が誤った経路を選んだとみなし抽出を
# 諦める(extract_ordered_centerline参照)。CLI/JSONでは調整不要な内部安全弁のため
# 定数のまま(単純なL字・コの字程度の通路なら通常2倍を大きく超えない)。
AUTO_ROUTE_CENTERLINE_MAX_DETOUR_RATIO = 2.5
# [本研究独自] 複数経路仮説PF(粒子単位のグラフ分岐)。既定は無効。有効時は
# route_source=autoの経路帯マスクからbuild_skeleton_graph()・
# simplify_skeleton_graph()・build_route_graph_topology()で通路グラフを作り、
# ParticleFilterPDRへ渡す(main()参照)。AUTO_ROUTE_CENTERLINE_ENABLEDとは
# 「1本の経路を仮定するか、分岐を扱うか」で排他的(両方有効はapply_map_configで
# エラーにする)。
MULTI_HYPOTHESIS_ROUTING_ENABLED = False
MULTI_HYPOTHESIS_ROUTING_SIMPLIFY_PX = 10.0
# 交差点分岐選択(choose_branch_by_heading、pdr_route_graph.py)の重み広がり(度)。
# 小さいほど直前の実測方位に近いエッジへ強く偏り、大きいほど一様乱択に近づく。
# 既定は100000(事実上の一様乱択)。2026-08-30、route_constraint_mode=enforceでの
# 正しい検証により、方位重み付け(σ=10〜60)が一様乱択を明確に上回らないと判明した
# ため、既定を一様乱択相当に戻した(詳細はCHANGELOG.md 2026-08-30(4)項)。
MULTI_HYPOTHESIS_BRANCH_HEADING_SIGMA_DEG = 100000.0
GYRO_UNIT = "rad"
# 別の図(L字経路用など)で使用しており、このJSONの実行対象からは除外したいCSVファイル名。
# map_configs/*.jsonの"exclude_csv"(任意設定)から読み込む。ファイル名の完全一致で判定する。
EXCLUDED_CSV_NAMES = set()

# ============================================================
# 2. パーティクルフィルタのパラメータ
# ============================================================
# 移動様態に応じた適応型PFパラメータ
N_PARTICLES_STRAIGHT = 250
N_PARTICLES_TURNING = 600
N_PARTICLES_STOPPED = 100

SIGMA_STEP_STRAIGHT = 0.35
SIGMA_STEP_TURNING = 0.70
SIGMA_STEP_STOPPED = 0.10

SIGMA_ANGLE_STRAIGHT = np.deg2rad(3.0)
SIGMA_ANGLE_TURNING = np.deg2rad(15.0)
SIGMA_ANGLE_STOPPED = np.deg2rad(2.0)

BEHAVIOR_WINDOW_SEC = 1.5
TURN_ENTER_THRESHOLD = np.deg2rad(20.0)
TURN_EXIT_THRESHOLD = np.deg2rad(6.0)
TURN_YAW_RATE_THRESHOLD = np.deg2rad(20.0)
PARTICLE_RESIZE_JITTER_PX = 0.50

# [本研究独自] 不確実性適応粒子数。移動様態(直進/曲がり/滞留)による粒子数決定
# (先行研究)に加えて、直前ステップの実効サンプルサイズ(Neff)を粒子数に対する比率
# (neff_ratio = neff / 直前の粒子数)で見て、重みが少数の粒子に偏っている(不確実性が
# 高い)場合は移動様態ベースの粒子数を割り増しし、重みがほぼ均等(不確実性が低い)
# 場合は割り引く。進捗反映版メモ.txt §6.5に相当する機能で、Neffは既存のPF診断用に
# 計算済みだったが、これまで粒子数制御には使われていなかった。既定では無効
# (UNCERTAINTY_ADAPTIVE_PARTICLES=False)で、有効化しても既存の挙動を変えない。
UNCERTAINTY_ADAPTIVE_PARTICLES = False
UNCERTAINTY_NEFF_LOW_RATIO = 0.30
UNCERTAINTY_NEFF_HIGH_RATIO = 0.60
UNCERTAINTY_BOOST_FACTOR = 1.5
UNCERTAINTY_SHRINK_FACTOR = 0.75
UNCERTAINTY_PARTICLES_MIN = 80
UNCERTAINTY_PARTICLES_MAX = 1200

# [本研究独自] ステップ検出の最短ピーク間隔は、歩行の生理的な上限歩調から導出する。
# 従来のSTEP_MIN_INTERVAL=5(サンプル数の直書き)は52.9Hzでは上限10.6歩/sに相当し
# 事実上無制限で、検出器が歩行加速度の第2高調波(基本波1.6〜1.7Hzに対する3.3〜3.4Hz)
# にも反応して歩数を約1.7〜1.8倍に過検出していた(memo/step_length_calibration.md)。
# 上限2.9Hz(=174歩/分)は歩行と走行の境界で、速く歩いても歩行である限り超えない。
# サンプル数ではなく周波数を固定することで、記録レートが変わっても歩調上限を保つ
# (STEP_MIN_INTERVALはサンプリング周波数が不明な場合のフォールバック)。
MAX_STEP_FREQUENCY_HZ = 2.9
STEP_MIN_INTERVAL = 5

MIN_STEP_M = 0.25
MAX_STEP_M = 1.00

# SmartPDR風のステップ信号・歩幅推定パラメータ
HPF_ALPHA        = 0.90
LPF_WINDOW       = 5
SMART_PEAK_THR   = 0.20
SMART_PP_THR     = 0.35
SMART_SLOPE_WIN  = 2
SMART_STEP_TAU   = 3.230
ROOT_BETA        = 1.479
ROOT_GAMMA       = -1.259
LOG_BETA         = 1.131
LOG_GAMMA        = 0.159

# [本研究独自] 歩幅推定式の被験者・端末校正ゲイン。SmartPDRのβ・γ係数は原論文の
# 計測環境で同定された値で、本研究の環境では歩幅を約1/2に過小推定するため導入した。
# 上下限クリップ(MIN_STEP_M/MAX_STEP_M)の前に適用する(クリップ後に掛かるstep_gainと
# 分けたのは、上下限を歩幅の物理的な範囲として保つため)。
# 【重要】総距離を特定の終点へ合わせる機構ではない。全CSV共通の定数を1つ掛けるだけで、
# 短く歩いたデータは短いまま出る。ファイルごとに総距離を目標値へ強制する機構
# (target_distance_px)は2026-09-02に削除した — 今後の計測ではあえて歩行距離を変える
# 予定があり、終点を固定する校正は方法論として使わないため。
# kanri_4f.jsonの2.10は評価用と同じ3CSVで求めた暫定値なので、次回計測の校正専用
# データで再同定すること(memo/step_length_calibration.md)。
STEP_LENGTH_CALIBRATION_GAIN = 1.0

# [本研究独自] 初期方位校正の方式(2026-09-02追加)。
# "samples"(既定・従来方式)は記録先頭のheading_calibration_samples個を円平均する。
# "walking"は歩き始めてからのHEADING_CALIBRATION_STEPS歩の方位を円平均する。
# 既定を"samples"のままにしているのは、既存の比較実験の結果を変えないため。
# 詳細はestimate_initial_sensor_heading()のdocstring参照。
HEADING_CALIBRATION_MODE = "samples"
HEADING_CALIBRATION_STEPS = 10

SMOOTH_WINDOW  = 2
RECOVERY_SIGMA = 8.0
ANGLE_DECAY    = 0.999
MAX_DT         = 1.0


# ============================================================
# 3. キャッシュおよびデータ保護用ユーティリティ
# ============================================================
class PDRResultCache:
    """CSVファイルの読み込み・推定処理結果を格納するキャッシュクラス。
    ファイルの更新時間（mtime）およびファイルサイズ（size）をチェックし、
    変更がない場合は前回の推定結果をそのまま返すことで、監視時のパフォーマンスを劇的に向上します。
    """
    def __init__(self):
        self._cache = {}

    def get(self, file_path_str, context=None):
        """入力CSVだけでなく開始位置・経路モード・方位設定も一致した時だけ再利用する。"""
        p = Path(file_path_str)
        if not p.exists():
            return None
        try:
            stat = p.stat()
            mtime = stat.st_mtime
            size = stat.st_size
        except Exception:
            return None

        cached = self._cache.get(file_path_str)
        if (
            cached
            and cached['mtime'] == mtime
            and cached['size'] == size
            and cached.get('context') == context
        ):
            return cached['data']
        return None

    def set(self, file_path_str, data, context=None):
        p = Path(file_path_str)
        if not p.exists():
            return
        try:
            stat = p.stat()
            self._cache[file_path_str] = {
                'mtime': stat.st_mtime,
                'size': stat.st_size,
                'context': context,
                'data': data
            }
        except Exception:
            pass



def load_start_positions():
    """1つのCSVから、計測CSVごとの既知開始位置を読み込む。"""
    if not START_POSITION_FILE.exists():
        return {}

    try:
        start_df = pd.read_csv(START_POSITION_FILE, encoding="utf-8")
    except Exception as error:
        raise ValueError(
            f"開始位置管理CSVを読み込めません: {START_POSITION_FILE}: {error}"
        ) from error

    required = {"file_name", "start_x", "start_y"}
    missing = required.difference(start_df.columns)
    if missing:
        raise ValueError(
            "start_positions.csvに必要な列がありません: "
            + ", ".join(sorted(missing))
        )

    positions = {}
    for row in start_df.itertuples(index=False):
        file_name = str(row.file_name).strip()
        if file_name:
            positions[file_name] = (float(row.start_x), float(row.start_y))
    return positions


def save_start_position(file_name, start_x, start_y):
    """開始位置を1つの管理CSVへ追加し、同名データは上書きする。"""
    new_row = pd.DataFrame([{
        "file_name": str(file_name),
        "start_x": float(start_x),
        "start_y": float(start_y),
    }])

    if START_POSITION_FILE.exists():
        current_df = pd.read_csv(START_POSITION_FILE, encoding="utf-8")
        if "file_name" in current_df.columns:
            current_df = current_df[
                current_df["file_name"].astype(str) != str(file_name)
            ]
        output_df = pd.concat([current_df, new_row], ignore_index=True)
    else:
        output_df = new_row

    output_df = output_df.sort_values("file_name").reset_index(drop=True)
    output_df.to_csv(
        START_POSITION_FILE,
        index=False,
        encoding="utf-8",
        lineterminator="\n",
    )
    logging.info(
        "[%s] 開始位置を保存: x=%.1f, y=%.1f -> %s",
        file_name,
        start_x,
        start_y,
        START_POSITION_FILE,
    )


def select_start_position_on_map(file_name, map_image):
    """地図上でクリックされた1点を、その計測CSVの既知開始位置として返す。"""
    select_fig, select_ax = plt.subplots(figsize=(12, 6))
    select_ax.imshow(map_image, cmap="gray")
    select_ax.set_xlim(0, map_image.shape[1])
    select_ax.set_ylim(map_image.shape[0], 0)
    select_ax.set_title(
        f"{file_name}\n計測を開始した位置を1回クリックしてください"
    )
    select_ax.set_xlabel("クリック後、この選択画面は自動で閉じます")
    select_ax.grid(False)
    plt.show(block=False)

    selected_points = plt.ginput(n=1, timeout=0, show_clicks=True)
    plt.close(select_fig)
    if not selected_points:
        raise RuntimeError(f"{file_name}の開始位置が選択されませんでした")

    start_x, start_y = selected_points[0]
    return float(start_x), float(start_y)


def get_or_select_start_position(file_name, map_image, pf_map, saved_positions):
    """保存済み位置を再利用し、未登録CSVだけ地図クリックを要求する。"""
    saved_position = saved_positions.get(file_name)

    # 変換前後で '_utf8' の有無だけが異なるファイル名にも対応する。
    if saved_position is None and file_name.endswith("_utf8.csv"):
        original_name = file_name.replace("_utf8.csv", ".csv")
        saved_position = saved_positions.get(original_name)

    if saved_position is not None:
        start_x, start_y = saved_position
        pixel_x = int(round(start_x))
        pixel_y = int(round(start_y))
        inside_map = (
            0 <= pixel_x < pf_map.shape[1]
            and 0 <= pixel_y < pf_map.shape[0]
        )
        if not inside_map or pf_map[pixel_y, pixel_x] != 255:
            logging.warning(
                "[%s] 保存済み開始位置が地図外または移動不可領域です。再選択します。",
                file_name,
            )
        else:
            logging.info(
                "[%s] 保存済み開始位置を使用: x=%.1f, y=%.1f",
                file_name,
                start_x,
                start_y,
            )
            return start_x, start_y

    while True:
        start_x, start_y = select_start_position_on_map(file_name, map_image)
        pixel_x = int(round(start_x))
        pixel_y = int(round(start_y))
        inside_map = (
            0 <= pixel_x < pf_map.shape[1]
            and 0 <= pixel_y < pf_map.shape[0]
        )
        if not inside_map:
            logging.warning(
                "[%s] 選択位置が地図外です。もう一度選択してください。",
                file_name,
            )
            continue
        if pf_map[pixel_y, pixel_x] != 255:
            logging.warning(
                "[%s] 選択位置が壁または移動不可領域です。白い通路上を選択してください。",
                file_name,
            )
            continue

        save_start_position(file_name, start_x, start_y)
        saved_positions[file_name] = (start_x, start_y)
        return start_x, start_y


# [本研究独自] CSVごとに独立した乱数系列を割り当てるためのシード。従来はmain()で
# np.random.seed()を一度呼ぶだけで、複数CSVがファイル名順に同じ乱数ストリームを
# 消費しており、CSVを1本足すだけで後続の結果が変わる・ファイル別の性能差にファイル順が
# 交絡する、という再現性の問題があった。zlib.crc32を使うのは組み込みhash()が
# プロセスごとにランダム化され再現しないため。
def file_random_seed(base_seed, file_name):
    """基準シードとCSVファイル名から、そのCSV専用の乱数シードを返す。"""
    if base_seed is None:
        return None
    return (int(base_seed) + zlib.crc32(file_name.encode("utf-8"))) % (2 ** 32)


def safe_read_csv(file_path, max_retries=5, delay=0.2):
    """ファイルへの同時書き込みによるパーサエラーやロック衝突を防ぎ、
    安全にCSVを読み込むためのリトライ機能付き関数。
    """
    for i in range(max_retries):
        try:
            df = pd.read_csv(file_path)
            if not df.empty and len(df) > 1:
                return df
        except (PermissionError, pd.errors.EmptyDataError, pd.errors.ParserError):
            pass
        time.sleep(delay)
    raise IOError(f"ファイルの安全な読み込みに失敗しました: {file_path}")


# [先行研究:移動様態PF] 秋山ほか(FIT2013)の直進/屈折/滞留の3様態区分を踏襲。
class MoveBehavior(Enum):
    """歩行者の移動様態。"""
    STOPPED = "stopped"
    STRAIGHT = "straight"
    TURNING = "turning"


# [先行研究:移動様態PF] 様態別に粒子数・ノイズ分散を変える考え方は秋山ほかに基づく。
# 数値(直進250/曲がり600/滞留100)は原論文の直進10/屈折20から比率を保ちつつ、
# 本研究の対象地図(管理棟4階、複雑な廊下形状)向けに拡大した [本研究独自] 調整値。
def behavior_parameters(behavior):
    """移動様態に対応する粒子数・ノイズ分散を返す。"""
    if behavior == MoveBehavior.TURNING:
        return {
            "n_particles": N_PARTICLES_TURNING,
            "sigma_step": SIGMA_STEP_TURNING,
            "sigma_angle": SIGMA_ANGLE_TURNING,
        }
    if behavior == MoveBehavior.STOPPED:
        return {
            "n_particles": N_PARTICLES_STOPPED,
            "sigma_step": SIGMA_STEP_STOPPED,
            "sigma_angle": SIGMA_ANGLE_STOPPED,
        }
    return {
        "n_particles": N_PARTICLES_STRAIGHT,
        "sigma_step": SIGMA_STEP_STRAIGHT,
        "sigma_angle": SIGMA_ANGLE_STRAIGHT,
    }


# [先行研究:移動様態PF]をベースに、[本研究独自]の外れ値対策(75パーセンタイル)・
# AND条件・ヒステリシスを追加した屈折判定。原論文は「8秒間に30度以上変化したら
# 屈折」という単純な閾値判定のみで、ヨーレートの併用やヒステリシスは持たない。
def detect_move_behavior(
    timestamps,
    heading_history,
    yaw_rate_history,
    current_index,
    previous_behavior,
    step_detected,
):
    """方位変化と持続的なヨーレートから移動様態を判定する。

    一瞬の手ぶれやジャイロの外れ値だけでTURNINGにならないよう、
    直近区間のヨーレート最大値ではなく75パーセンタイルを用いる。

    STRAIGHTからTURNINGへ移るには、方位変化とヨーレートの
    両条件を満たす必要がある。TURNINGからSTRAIGHTへ戻る際は、
    両方が十分小さくなったことを確認するヒステリシス判定を行う。
    """
    if not step_detected:
        return MoveBehavior.STOPPED

    current_time = float(timestamps[current_index])
    start_index = current_index

    while start_index > 0:
        previous_time = float(timestamps[start_index - 1])
        elapsed = current_time - previous_time
        if elapsed > BEHAVIOR_WINDOW_SEC:
            break
        start_index -= 1

    heading_change = abs(
        angle_diff(
            heading_history[current_index],
            heading_history[start_index],
        )
    )

    recent_yaw = yaw_rate_history[start_index:current_index + 1]
    finite_yaw = np.abs(recent_yaw[np.isfinite(recent_yaw)])

    if len(finite_yaw) > 0:
        representative_yaw_rate = float(
            np.percentile(finite_yaw, 75)
        )
    else:
        representative_yaw_rate = 0.0

    if previous_behavior == MoveBehavior.TURNING:
        turn_finished = (
            heading_change < TURN_EXIT_THRESHOLD
            and representative_yaw_rate
            < TURN_YAW_RATE_THRESHOLD * 0.5
        )
        if turn_finished:
            return MoveBehavior.STRAIGHT
        return MoveBehavior.TURNING

    turn_started = (
        heading_change >= TURN_ENTER_THRESHOLD
        and representative_yaw_rate >= TURN_YAW_RATE_THRESHOLD
    )

    if turn_started:
        return MoveBehavior.TURNING

    return MoveBehavior.STRAIGHT


# ============================================================
# 4. パーティクルフィルタのカプセル化クラス (OOP設計)
# ============================================================
# [先行研究:移動様態PF] 予測→重み計算→リサンプリングという基本枠組み、および
# 様態別の粒子数・分散変更(configure_behavior)は秋山ほかの提案方式に基づく。
# [本研究独自] 壁判定は原論文の0/1二値重みではなく、距離場(dist_map)による
# 連続的な尤度重み付けに拡張している(update内のwall_weights計算)。さらに
# route_mask(地図から抽出した経路帯)を用いた重み付けは両先行研究にない要素。
class ParticleFilterPDR:
    """パーティクルフィルタの状態、重み更新、衝突判定、および
    リサンプリング処理を完全にカプセル化したクラス。
    """
    def __init__(self, n_particles, start_x, start_y, route_mask, binary_for_pf, dist_map, params,
                 route_topology=None):
        self.n_particles = n_particles
        self.start_x = start_x
        self.start_y = start_y
        self.route_mask = route_mask
        self.binary_for_pf = binary_for_pf
        self.dist_map = dist_map
        self.h, self.w = binary_for_pf.shape
        self.params = params
        # [本研究独自] 複数経路仮説PF(粒子単位のグラフ分岐)。Noneなら従来通り
        # 位置(x, y)のみの粒子を使う(既定の挙動は変更しない)。設定時は
        # build_route_graph_topology()の戻り値を渡す。各粒子はさらに
        # (現在のエッジid, エッジ内の区間index, 進行方向+1/-1)を保持する。
        self.route_topology = route_topology
        self.reset()

    def reset(self):
        """パーティクルと状態の初期化"""
        if self.route_topology is not None:
            # 列: [x, y, edge_id, seg_index, direction]。edge_id等はここでは
            # 仮の0で埋める。進行方向(direction)の決定には歩行開始時のセンサー
            # 方位が必要だが、reset()の時点(コンストラクタ内)ではまだ計算されて
            # いないため、実際の値はinitialize_route_state()を呼び出し側(main())
            # から別途呼んで設定する。
            self.particles = np.column_stack([
                self.start_x + np.random.normal(0, 2, self.n_particles),
                self.start_y + np.random.normal(0, 2, self.n_particles),
                np.zeros(self.n_particles),
                np.zeros(self.n_particles),
                np.ones(self.n_particles),
            ])
        else:
            self.particles = np.column_stack([
                self.start_x + np.random.normal(0, 2, self.n_particles),
                self.start_y + np.random.normal(0, 2, self.n_particles)
            ])
        self.weights = np.full(self.n_particles, 1.0 / self.n_particles)
        self.pos_buffer = collections.deque(maxlen=self.params.get('smooth_window', 2))
        self.estimated_positions = []
        self.extinction_count = 0
        self.route_ratio_history = []
        self.valid_ratio_history = []
        self.neff_history = []
        self.position_spread_history = []
        self.uncertainty_boost_count = 0
        self.uncertainty_shrink_count = 0

    def resize_particle_set(self, new_count, jitter_px=0.0):
        """重みに従って粒子集合を増減し、等重みに戻す。"""
        old_count = len(self.particles)
        new_count = max(1, int(new_count))
        if new_count == old_count:
            self.n_particles = new_count
            return

        weight_sum = float(self.weights.sum())
        if weight_sum <= 0 or not np.isfinite(weight_sum):
            probabilities = np.full(old_count, 1.0 / old_count)
        else:
            probabilities = self.weights / weight_sum

        indices = np.random.choice(
            old_count,
            size=new_count,
            replace=True,
            p=probabilities,
        )
        resized = self.particles[indices].copy()
        if new_count > old_count and jitter_px > 0:
            resized[:, 0] += np.random.normal(0, jitter_px, new_count)
            resized[:, 1] += np.random.normal(0, jitter_px, new_count)

        # 地図外・壁内へ出た複製粒子は元の有効粒子から置換する。
        invalid = self.is_in_wall(resized[:, 0], resized[:, 1])
        if np.any(invalid):
            valid_old = ~self.is_in_wall(self.particles[:, 0], self.particles[:, 1])
            valid_indices = np.where(valid_old)[0]
            if len(valid_indices) > 0:
                fill = np.random.choice(valid_indices, size=int(invalid.sum()))
                resized[invalid] = self.particles[fill]

        self.particles = resized
        self.n_particles = new_count
        self.weights = np.full(new_count, 1.0 / new_count)

    def configure_behavior(self, behavior):
        """移動様態に応じて粒子数とサンプリング分散を更新する。
        [本研究独自] UNCERTAINTY_ADAPTIVE_PARTICLES有効時は、直前ステップのNeff比率
        (neff_ratio)によって移動様態ベースの粒子数をさらに増減させる。
        """
        behavior_params = behavior_parameters(behavior)
        target_n = behavior_params["n_particles"]

        if UNCERTAINTY_ADAPTIVE_PARTICLES and self.neff_history:
            last_neff = self.neff_history[-1]
            last_n = len(self.particles)
            neff_ratio = (last_neff / last_n) if last_n > 0 else 0.0
            if neff_ratio < UNCERTAINTY_NEFF_LOW_RATIO:
                target_n = target_n * UNCERTAINTY_BOOST_FACTOR
                self.uncertainty_boost_count += 1
            elif neff_ratio > UNCERTAINTY_NEFF_HIGH_RATIO:
                target_n = target_n * UNCERTAINTY_SHRINK_FACTOR
                self.uncertainty_shrink_count += 1
            target_n = int(np.clip(
                round(target_n), UNCERTAINTY_PARTICLES_MIN, UNCERTAINTY_PARTICLES_MAX
            ))

        self.resize_particle_set(target_n, jitter_px=PARTICLE_RESIZE_JITTER_PX)
        self.params["sigma_step"] = behavior_params["sigma_step"]
        self.params["sigma_angle"] = behavior_params["sigma_angle"]

    # --------------------------------------------------------------
    # [本研究独自] 複数経路仮説PF(粒子単位のグラフ分岐)
    # --------------------------------------------------------------
    # route_topology(build_route_graph_topology()の戻り値)が設定されている場合の
    # み使う一連のメソッド。self.particlesの列2〜4(edge_id, seg_index, direction)
    # を管理する。route_topology=None(既定)では一切呼ばれず、既存の挙動を変えない。

    def initialize_route_state(self, x, y, heading_hint):
        """開始位置(x, y)に最も近いエッジ・区間へ全粒子を割り当てる。

        heading_hintは歩行開始時点の地図座標系での推定方位(rad)。区間の
        前進方向・逆方向のうち、heading_hintに近い方を進行方向として採用する
        (地図上は同じ通路でも、歩く向きが分からないと方位補正の符号を
        決められないため)。route_topology未設定時は何もしない。
        """
        if self.route_topology is None:
            return
        edge_id, seg_index, _t = nearest_edge_position(self.route_topology, x, y)
        if edge_id is None:
            logging.warning(
                "複数経路仮説PF: 開始位置(%.1f, %.1f)に対応する通路グラフの"
                "エッジが見つかりません。経路方位補正なしで続行します。", x, y,
            )
            return
        edge = self.route_topology["edges_by_id"][edge_id]
        n_seg = len(edge["seg_headings"])
        seg_index = int(np.clip(seg_index, 0, n_seg - 1))
        seg_heading = edge["seg_headings"][seg_index]
        forward_diff = abs(angle_diff(heading_hint, seg_heading))
        backward_diff = abs(angle_diff(heading_hint, normalize_angle(seg_heading + np.pi)))
        direction = 1.0 if forward_diff <= backward_diff else -1.0
        # directionの定義(_advance_route_state/_route_corrected_headingsと共通):
        # +1ならseg_indexはpoints[0]側から数えた区間番号、-1ならpoints[-1]側から
        # 数えた区間番号。nearest_edge_position()はpoints配列そのままの順で区間を
        # 返すため、-1を選んだ場合はseg_indexを「to側から数えた番号」に変換する。
        if direction < 0:
            seg_index = (n_seg - 1) - seg_index
        self.particles[:, 2] = float(edge_id)
        self.particles[:, 3] = float(seg_index)
        self.particles[:, 4] = direction
        logging.info(
            "複数経路仮説PF: 開始エッジ=%d (%d->%d), 区間=%d/%d, 進行方向=%s",
            edge_id, edge["from"], edge["to"], seg_index, n_seg,
            "forward" if direction > 0 else "backward",
        )

    def _route_corrected_headings(self, sensor_step_heading):
        """粒子ごとの現在エッジ・区間の方位とセンサー方位を重み付き融合する
        (correct_heading_with_route_segment()の粒子版)。エッジをまたいで
        ループするのではなく、エッジ単位でまとめてブロードキャスト計算する
        (エッジ数は粒子数よりずっと少ないため)。
        """
        if ROUTE_HEADING_WEIGHT <= 0:
            return np.full(self.n_particles, sensor_step_heading)

        edge_ids = self.particles[:, 2].astype(int)
        seg_indices = self.particles[:, 3].astype(int)
        directions = self.particles[:, 4]
        route_headings = np.zeros(self.n_particles)

        for edge_id, edge in self.route_topology["edges_by_id"].items():
            mask = edge_ids == edge_id
            if not np.any(mask):
                continue
            seg_headings = edge["seg_headings"]
            n_seg = len(seg_headings)
            idx = np.clip(seg_indices[mask], 0, n_seg - 1)
            dir_mask = directions[mask]
            # direction=+1: そのままseg_headings[idx]。-1: to側から数えた区間なので
            # 物理的な区間番号は(n_seg-1-idx)で、方位は反転させる。
            physical_idx = np.where(dir_mask > 0, idx, (n_seg - 1) - idx)
            h = seg_headings[physical_idx]
            h = np.where(dir_mask > 0, h, h + np.pi)
            route_headings[mask] = h

        sin_sum = np.sin(sensor_step_heading) + np.sin(route_headings) * ROUTE_HEADING_WEIGHT
        cos_sum = np.cos(sensor_step_heading) + np.cos(route_headings) * ROUTE_HEADING_WEIGHT
        return np.arctan2(sin_sum, cos_sum)

    def _advance_route_state(self, reference_heading):
        """各粒子の現在位置がエッジ区間の終点(次の曲がり角、またはノード)に
        ROUTE_CORNER_THRESHOLD_PX以内まで近づいたら、区間を1つ進める。エッジの
        終端(ノード)に達した粒子は、そのノードに接続する他のエッジの中から
        次の進行先を選ぶ(交差点での分岐=複数経路仮説の枝分かれ)。選択は
        reference_heading(直前の実測方位)に近いエッジを優先するガウス重み付き
        乱択(choose_branch_by_heading、pdr_route_graph.py、2026-08-30)。
        来た道をそのまま戻る選択肢は、他に行き先がある限り除外する(行き止まり
        =端点ノードでは選択肢が無いので、そのまま引き返す)。
        分岐後にどの仮説が正しいかは、この後の通常の重み付け・リサンプリング
        (壁尤度・経路帯マスク・方位整合度)で自然に選別される。
        """
        edge_ids = self.particles[:, 2].astype(int)
        seg_indices = self.particles[:, 3].astype(int)
        directions = self.particles[:, 4]
        xs = self.particles[:, 0]
        ys = self.particles[:, 1]

        edges_by_id = self.route_topology["edges_by_id"]
        adjacency = self.route_topology["adjacency"]

        new_edge_ids = edge_ids.copy()
        new_seg_indices = seg_indices.copy()
        new_directions = directions.copy()

        for edge_id, edge in edges_by_id.items():
            mask = edge_ids == edge_id
            if not np.any(mask):
                continue
            points = np.asarray(edge["points"])
            n_seg = len(edge["seg_headings"])
            idxs_global = np.where(mask)[0]
            idx = np.clip(seg_indices[mask], 0, n_seg - 1)
            dir_mask = directions[mask]

            physical_idx = np.where(dir_mask > 0, idx, (n_seg - 1) - idx)
            target_point_idx = np.where(dir_mask > 0, physical_idx + 1, physical_idx)
            target_points = points[target_point_idx]
            dist_to_target = np.hypot(
                xs[mask] - target_points[:, 0], ys[mask] - target_points[:, 1]
            )
            reached = dist_to_target <= ROUTE_CORNER_THRESHOLD_PX

            # エッジ途中: 次の区間へ進むだけ。
            mid_reach = reached & (idx < n_seg - 1)
            new_seg_indices[idxs_global[mid_reach]] = idx[mid_reach] + 1

            # エッジの終端(ノード)に到達: 接続エッジから次の進行先を選ぶ。
            end_reach = reached & (idx >= n_seg - 1)
            for local_i in np.where(end_reach)[0]:
                gi = idxs_global[local_i]
                d = dir_mask[local_i]
                node_id = edge["to"] if d > 0 else edge["from"]
                candidates = adjacency.get(node_id, [])
                if not candidates:
                    continue  # 通常起きない(終端ノードは必ず自エッジを含む)
                backtrack = (edge_id, -1 if d > 0 else 1)
                non_backtrack = [c for c in candidates if c != backtrack]
                choices = non_backtrack if non_backtrack else candidates
                new_edge_id, new_direction = choose_branch_by_heading(
                    choices, edges_by_id, reference_heading,
                    np.radians(MULTI_HYPOTHESIS_BRANCH_HEADING_SIGMA_DEG),
                )
                new_edge_ids[gi] = new_edge_id
                new_seg_indices[gi] = 0
                new_directions[gi] = float(new_direction)

        self.particles[:, 2] = new_edge_ids
        self.particles[:, 3] = new_seg_indices
        self.particles[:, 4] = new_directions

    def is_in_wall(self, x, y):
        """座標が壁の中、または地図範囲外にあるかを判定する（境界条件の厳格化）。
        範囲外は全て壁（移動不可）として安全に処理します。
        """
        ix = np.round(x).astype(int)
        iy = np.round(y).astype(int)
        
        # マップ境界チェック
        out_of_bounds = (ix < 0) | (ix >= self.w) | (iy < 0) | (iy >= self.h)
        
        in_wall = np.zeros_like(x, dtype=bool)
        valid = ~out_of_bounds
        if np.any(valid):
            in_wall[valid] = self.binary_for_pf[iy[valid], ix[valid]] == 0
        in_wall[out_of_bounds] = True
        return in_wall

    def path_hits_wall(self, old_particles, new_particles):
        """現在のパーティクル位置から移動後の位置までの線分が、
        壁に衝突しているかをチェックする。
        （トンネル効果を防ぐため、サンプリング間隔を常に1px以下に維持）
        """
        n = len(old_particles)
        x0, y0 = old_particles[:, 0], old_particles[:, 1]
        x1, y1 = new_particles[:, 0], new_particles[:, 1]
        
        dx = x1 - x0
        dy = y1 - y0
        
        # 1ピクセル以下でサンプリングするため、最大距離からステップ数を算出
        steps = np.maximum(np.abs(dx), np.abs(dy)).astype(int)
        steps = np.maximum(steps, 1)  # 最低でも1分割
        
        hit = np.zeros(n, dtype=bool)
        max_steps = steps.max()
        
        for s in range(max_steps + 1):
            ratio = s / np.maximum(steps, 1)
            xs = x0 + dx * ratio
            ys = y0 + dy * ratio
            hit |= self.is_in_wall(xs, ys)
        return hit

    def update(self, step_px, step_heading, behavior=MoveBehavior.STRAIGHT):
        """移動様態に応じた粒子数・分散で1歩分のPF更新を実行する。"""
        self.configure_behavior(behavior)

        # [本研究独自] 複数経路仮説PF: 粒子ごとに現在のエッジ・区間を進め
        # (交差点では接続エッジへ確率的に分岐)、粒子ごとの区間方位でセンサー
        # 方位を補正する。route_topology未設定時はstep_headingを従来通りの
        # スカラーのまま使う(既存の単一経路モード・経路制約なしモードは無変更)。
        corrected_step_heading = step_heading
        if self.route_topology is not None:
            self._advance_route_state(step_heading)
            # route_guidance_enabled()と同じく、prefer/enforceのときだけ地図由来の
            # 方位補正を使う(noneは「地図の経路情報を方位に使わない」比較条件)。
            if ROUTE_CONSTRAINT_MODE in {"prefer", "enforce"}:
                corrected_step_heading = self._route_corrected_headings(step_heading)

        # 1. 状態遷移（移動様態ごとのノイズ付与）
        noise_dist = np.random.normal(0, self.params['sigma_step'], self.n_particles)
        noise_angle = np.random.normal(0, self.params['sigma_angle'], self.n_particles)
        move = step_px + noise_dist
        p_angle = corrected_step_heading + noise_angle

        new_particles = self.particles.copy()
        new_particles[:, 0] += move * np.cos(p_angle)
        new_particles[:, 1] += move * np.sin(p_angle)

        # 2. 壁との接触確認
        hit_wall = self.path_hits_wall(self.particles, new_particles)

        # 3. 重み（尤度）の計算
        ny_idx = np.clip(np.round(new_particles[:, 1]).astype(int), 0, self.h - 1)
        nx_idx = np.clip(np.round(new_particles[:, 0]).astype(int), 0, self.w - 1)
        
        # 距離画像による重み付け
        dist_values = self.dist_map[ny_idx, nx_idx]
        wall_weights = np.exp(
            -(1.0 - dist_values) ** 2 /
            (2 * self.params['wall_weight_sigma'] ** 2)
        )
        weights = self.params['wall_weight_floor'] + (1.0 - self.params['wall_weight_floor']) * wall_weights

        # 経路優先重み
        on_route = self.route_mask[ny_idx, nx_idx]
        if ROUTE_CONSTRAINT_MODE == "enforce":
            weights *= np.where(on_route, 1.0, 0.0)
        elif ROUTE_CONSTRAINT_MODE == "prefer":
            weights *= np.where(on_route, 1.0, self.params['off_route_weight'])
        # noneでは経路重みを適用しない。

        # 比較・考察用のPF診断値。noneではroute_maskが全Trueになる。
        valid_particles = ~hit_wall
        self.route_ratio_history.append(float(np.mean(on_route)))
        self.valid_ratio_history.append(float(np.mean(valid_particles)))
        spread = np.var(new_particles[:, 0]) + np.var(new_particles[:, 1])
        self.position_spread_history.append(float(spread))
        
        # [本研究独自] ここにあった「方位ズレの重み」は2026-09-02に削除した。
        # p_angle = corrected_step_heading + noise_angle と定義した直後に
        # angle_diff(p_angle, corrected_step_heading) をガウス密度で評価しており、
        # これは noise_angle と恒等的に一致する = 提案分布の密度でそのサンプル自身を
        # 重み付ける二重計上だった。観測情報を含まないので尤度として意味を持たず、
        # 方位分散を sigma_angle/√2 へ縮め、Neffを約13%押し下げるだけだった。
        # ブートストラップPFでは重みは観測尤度(壁尤度と経路帯マスク)のみであるべき。
        # 再追加を検討する場合はCHANGELOG.md 2026-09-02(3)項の「今後の候補」を読むこと。

        # 壁に当たったものは即座に重み0
        weights[hit_wall] = 0.0

        # 4. 正規化とリサンプリング
        sum_w = weights.sum()
        if sum_w > 0:
            weights /= sum_w
            neff = 1.0 / (np.sum(weights ** 2) + 1e-300)
            self.neff_history.append(float(neff))
            self.particles, self.weights = resample_if_needed(new_particles, weights)
        else:
            self.neff_history.append(0.0)
            # 粒子全滅時の自己リカバリ
            self.extinction_count += 1
            # [本研究独自] 複数経路仮説PF使用時、self.particlesは(x, y, edge_id,
            # seg_index, direction)の5列になっているため、位置(x, y)の平均は
            # 先頭2列だけを使う。
            ref_x, ref_y = np.mean(self.particles[:, :2], axis=0)
            self.particles[:, 0] = ref_x + np.random.normal(0, self.params['recovery_sigma'], self.n_particles)
            self.particles[:, 1] = ref_y + np.random.normal(0, self.params['recovery_sigma'], self.n_particles)

            for _ in range(50):
                new_x = ref_x + np.random.normal(0, self.params['recovery_sigma'], self.n_particles)
                new_y = ref_y + np.random.normal(0, self.params['recovery_sigma'], self.n_particles)
                nx_i = np.clip(np.round(new_x).astype(int), 0, self.w - 1)
                ny_i = np.clip(np.round(new_y).astype(int), 0, self.h - 1)
                valid = self.binary_for_pf[ny_i, nx_i] == 255
                if ROUTE_CONSTRAINT_MODE == "enforce":
                    valid &= self.route_mask[ny_i, nx_i]
                if valid.sum() > self.n_particles // 4:
                    valid_idx = np.where(valid)[0]
                    invalid_idx = np.where(~valid)[0]
                    if len(invalid_idx) > 0:
                        fill = np.random.choice(valid_idx, size=len(invalid_idx))
                        new_x[invalid_idx] = new_x[fill]
                        new_y[invalid_idx] = new_y[fill]
                    self.particles[:, 0] = new_x
                    self.particles[:, 1] = new_y
                    break
            # [本研究独自] 複数経路仮説PF: リカバリで全粒子がref_x, ref_y付近へ
            # テレポートしたため、edge_id/seg_index/directionも新しい位置に
            # 合わせて再割り当てする(古いエッジ情報を持ち越すと、以後
            # _advance_route_state()がそのエッジの終点へ辿り着くまで方位補正が
            # 実際の位置と無関係な値のままになってしまうため)。
            if self.route_topology is not None:
                self.initialize_route_state(ref_x, ref_y, step_heading)
            self.weights = np.full(self.n_particles, 1.0 / self.n_particles)

        # 5. 重み付き平均による現在位置の推定（移動平均で平滑化）
        raw_x = np.average(self.particles[:, 0], weights=self.weights)
        raw_y = np.average(self.particles[:, 1], weights=self.weights)
        self.pos_buffer.append((raw_x, raw_y))
        avg_pos = np.mean(self.pos_buffer, axis=0)
        self.estimated_positions.append(tuple(avg_pos))
        return avg_pos


# ============================================================
# 5. ユーティリティ関数
# ============================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description="SmartPDR風のステップ検出・歩幅推定を加えたPFによるPDR位置推定（最適化版）"
    )
    parser.add_argument(
        "--map-config",
        type=Path,
        default=DEFAULT_MAP_CONFIG,
        help="地図ごとの設定JSON。未指定なら管理棟4階用の設定を読み込みます。",
    )
    parser.add_argument(
        "--data-dir",
        type=Path,
        default=None,
        help="pdr_log_*.csv があるディレクトリ。指定するとJSONの値を上書きします。",
    )
    parser.add_argument(
        "--map",
        type=Path,
        default=None,
        help="使用するマップ画像。指定するとJSONの値を上書きします。",
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
    parser.add_argument(
        "--save-trajectory-csv",
        action="store_true",
        help=(
            "[本研究独自] CSVごとの推定軌跡(timestamp, x_px, y_px)をRESULTS_DIR"
            "以下へ別CSVとして保存します(既定では保存しません)。正解位置データと"
            "突き合わせてRMSE等を計算する評価用(evaluate_accuracy.py参照)。"
        ),
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="乱数シード。結果を再現したいときに指定します。",
    )
    parser.add_argument(
        "--step-gain",
        type=float,
        default=None,
        help="校正後の歩幅に掛ける追加倍率。指定するとJSONの値を上書きします。",
    )
    parser.add_argument(
        "--step-length-calibration-gain",
        type=float,
        default=None,
        help=(
            "[本研究独自] 歩幅推定式の被験者・端末校正ゲイン(上下限クリップの前に適用)。"
            "SmartPDRのβ・γ係数は原論文の計測環境で同定された値のため、本研究の環境では"
            "そのままだと歩幅を約1/2に過小推定する。既定はJSON設定"
            "(kanri_4f.jsonは2.10、暫定値。CHANGELOG.md 2026-09-02(2)項参照)。"
        ),
    )
    parser.add_argument(
        "--no-watch",
        action="store_true",
        help="CSVフォルダ監視を行わず、1回だけ描画して終了します。",
    )
    parser.add_argument(
        "--pf-erosion-radius-px", type=int, default=None,
        help="PF用移動可能領域を内側へ縮める半径(px)。0なら収縮しません。",
    )
    parser.add_argument(
        "--route-constraint-mode",
        choices=["none", "prefer", "enforce"],
        default=None,
        help=(
            "手動route_pointsの使い方。none=使わない、"
            "prefer=経路周辺を優遇、enforce=経路外を禁止。"
        ),
    )
    parser.add_argument(
        "--route-source",
        choices=["manual", "auto"],
        default=None,
        help=(
            "経路帯マスクの生成元。manual=JSONのroute_pointsを使用(方式D)、"
            "auto=二値地図から通路領域を自動抽出し手動route_pointsは使用しない"
            "(本研究独自。曲がり角連動の方位補正は既定では空間マスクのみで無効、"
            "--auto-route-centerlineで有効化可能だが輪状の通路網では安全側に"
            "フォールバックする。詳細はmemo/route_source_auto.md)。"
        ),
    )
    parser.add_argument(
        "--auto-route-dilation-px",
        type=float,
        default=None,
        help=(
            "route-source=autoで抽出した経路帯マスクへ加える緩和的膨張の半径(px)。"
            "0(既定)なら地図形状そのまま。手動route_pointsの一様バッファより"
            "狭い区間でPFが不安定化する場合の緩和用。"
        ),
    )
    parser.add_argument(
        "--auto-route-exclude-wide-rooms",
        dest="auto_route_exclude_wide_rooms",
        action="store_true",
        default=None,
        help=(
            "route-source=autoの通路候補から、大きな部屋の壁際の帯を除外する"
            "(モルフォロジー開放で広い領域を差し引く。本研究独自)。既定はJSON設定"
            "(無指定ならOFF、従来通り除外しない)。半径が小さすぎるとホール等の"
            "本物の合流点まで消えて分断される場合があるので"
            "--auto-route-exclude-wide-rooms-radius-pxで調整する。"
        ),
    )
    parser.add_argument(
        "--no-auto-route-exclude-wide-rooms",
        dest="auto_route_exclude_wide_rooms",
        action="store_false",
        help="--auto-route-exclude-wide-roomsを明示的に無効化する(JSON設定を上書き)。",
    )
    parser.add_argument(
        "--auto-route-exclude-wide-rooms-radius-px",
        type=float,
        default=None,
        help=(
            "広い部屋を除外するモルフォロジー開放の半径(px)。既定は"
            "max(max_half_width_px*1.5, max_half_width_px+10)。大きくすると本物の"
            "広めの合流点(ホール等)を残しやすいが、部屋の除外効果は弱くなる。"
        ),
    )
    parser.add_argument(
        "--auto-route-centerline",
        dest="auto_route_centerline_enabled",
        action="store_true",
        default=None,
        help=(
            "route-source=autoの経路帯マスクを細線化し、順序付き中心線を"
            "ROUTE_POINTSとして曲がり角連動の方位補正に使う(本研究独自)。"
            "既定はJSON設定(無指定ならOFF、従来通り空間マスクのみ)。"
        ),
    )
    parser.add_argument(
        "--no-auto-route-centerline",
        dest="auto_route_centerline_enabled",
        action="store_false",
        help="--auto-route-centerlineを明示的に無効化する(JSON設定を上書き)。",
    )
    parser.add_argument(
        "--auto-route-centerline-simplify-px",
        type=float,
        default=None,
        help=(
            "中心線を折れ線に簡略化する際の許容誤差(px、RDP法、既定10.0)。"
            "大きいほど直線区間が少なく粗い経路になる。"
        ),
    )
    parser.add_argument(
        "--multi-hypothesis-routing",
        dest="multi_hypothesis_routing_enabled",
        action="store_true",
        default=None,
        help=(
            "route-source=autoの経路帯マスクから通路グラフ(交差点・分岐を含む)を"
            "構築し、粒子ごとに現在のエッジ・進行方向を持たせて交差点で確率的に"
            "分岐させる複数経路仮説PF(本研究独自)を有効にする。"
            "--auto-route-centerline(単一経路)とは併用できない。"
            "既定はJSON設定(無指定ならOFF)。"
        ),
    )
    parser.add_argument(
        "--no-multi-hypothesis-routing",
        dest="multi_hypothesis_routing_enabled",
        action="store_false",
        help="--multi-hypothesis-routingを明示的に無効化する(JSON設定を上書き)。",
    )
    parser.add_argument(
        "--multi-hypothesis-routing-simplify-px",
        type=float,
        default=None,
        help=(
            "通路グラフの各エッジを折れ線に簡略化する際の許容誤差(px、RDP法、"
            "既定10.0)。--auto-route-centerline-simplify-pxのグラフ版。"
        ),
    )
    parser.add_argument(
        "--multi-hypothesis-branch-heading-sigma-deg",
        type=float,
        default=None,
        help=(
            "交差点分岐選択(choose_branch_by_heading)の重み広がり(度、既定100000=事実上"
            "一様乱択)。小さいほど直前の実測方位に近いエッジへ強く偏るが、"
            "2026-08-30の検証では一様乱択を明確に上回らなかった(CHANGELOG.md参照)。"
        ),
    )
    parser.add_argument(
        "--uncertainty-adaptive-particles",
        dest="uncertainty_adaptive_particles",
        action="store_true",
        default=None,
        help=(
            "直前ステップのNeff比率に応じて移動様態ベースの粒子数を増減する"
            "(本研究独自、進捗メモ§6.5相当)。既定はJSON設定(無指定ならOFF)。"
        ),
    )
    parser.add_argument(
        "--no-uncertainty-adaptive-particles",
        dest="uncertainty_adaptive_particles",
        action="store_false",
        help="--uncertainty-adaptive-particlesを明示的に無効化する(JSON設定を上書き)。",
    )
    parser.add_argument(
        "--uncertainty-neff-low-ratio", type=float, default=None,
        help="不確実性適応粒子数の下側閾値(neff_ratio未満で粒子を増やす、既定0.30)。"
             "感度分析用にJSON設定を上書きする。",
    )
    parser.add_argument(
        "--uncertainty-neff-high-ratio", type=float, default=None,
        help="不確実性適応粒子数の上側閾値(neff_ratio超で粒子を減らす、既定0.60)。"
             "感度分析用にJSON設定を上書きする。",
    )
    parser.add_argument(
        "--uncertainty-boost-factor", type=float, default=None,
        help="下側閾値未満のときの粒子数倍率(既定1.5)。感度分析用にJSON設定を上書きする。",
    )
    parser.add_argument(
        "--uncertainty-shrink-factor", type=float, default=None,
        help="上側閾値超のときの粒子数倍率(既定0.75)。感度分析用にJSON設定を上書きする。",
    )
    parser.add_argument(
        "--heading-source",
        choices=["gyro", "android"],
        default="gyro",
        help=(
            "方位源。gyro=従来のジャイロ/Madgwick、android=CSVのyaw_deg。"
            "どちらも開始時方位を0として地図方位へ合わせます。"
        ),
    )
    parser.add_argument(
        "--initial-heading-deg",
        type=float,
        default=0.0,
        help="計測開始時の地図上方位。右=0、下=90、左=180、上=-90。",
    )
    parser.add_argument(
        "--heading-calibration-samples",
        type=int,
        default=30,
        help="開始方位の基準値を求める先頭サンプル数(--heading-calibration-mode samples時)。",
    )
    parser.add_argument(
        "--heading-calibration-mode",
        choices=["samples", "walking"],
        default=None,
        help=(
            "[本研究独自] 開始方位の基準の取り方。samples=記録先頭の数サンプル(既定、従来方式)、"
            "walking=歩き始めてからの数歩分。計測開始直後にまだ端末を持ち替えている場合、"
            "samplesでは基準方位が壊れる(2026-09-02の診断で1438/1441に該当)。"
            "既定はJSON設定(無指定ならsamples)。"
        ),
    )
    parser.add_argument(
        "--heading-calibration-steps",
        type=int,
        default=None,
        help="walking方式で基準方位の計算に使う先頭ステップ数(既定10)。",
    )
    parser.add_argument(
        "--log-level",
        type=str,
        default="INFO",
        help="ログレベル (DEBUG, INFO, WARNING, ERROR)",
    )
    return parser.parse_args()


def resolve_config_path(path):
    path = Path(path).expanduser()
    if not path.is_absolute():
        path = (Path.cwd() / path).resolve()
    return path


def resolve_config_value_path(value, config_dir):
    if value is None:
        return None
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = (config_dir / path).resolve()
    return path


def load_map_config(config_path):
    config_path = resolve_config_path(config_path)
    if not config_path.exists():
        raise FileNotFoundError(f"地図設定JSONが見つかりません: {config_path}")

    with config_path.open("r", encoding="utf-8") as f:
        config = json.load(f)
    return config, config_path


def require_config_value(mapping, key, source_name):
    if key not in mapping:
        raise ValueError(f"{source_name} に必須設定 '{key}' がありません。")
    return mapping[key]


def apply_map_config(args, config, config_path):
    global M_TO_PIXEL, DEFAULT_STEP_GAIN
    global STEP_LENGTH_CALIBRATION_GAIN
    global HEADING_CALIBRATION_MODE, HEADING_CALIBRATION_STEPS
    global PF_EROSION_RADIUS_PX
    global WALL_WEIGHT_SIGMA, WALL_WEIGHT_FLOOR, OFF_ROUTE_WEIGHT
    global ROUTE_WIDTH_PX, ROUTE_HEADING_WEIGHT, ROUTE_CORNER_THRESHOLD_PX
    global ROUTE_CONSTRAINT_MODE, ROUTE_POINTS, GYRO_UNIT, EXCLUDED_CSV_NAMES
    global ROUTE_SOURCE, AUTO_ROUTE_MAX_HALF_WIDTH_PX, AUTO_ROUTE_DILATION_PX
    global AUTO_ROUTE_EXCLUDE_WIDE_ROOMS, AUTO_ROUTE_EXCLUDE_WIDE_ROOMS_RADIUS_PX
    global AUTO_ROUTE_CENTERLINE_ENABLED, AUTO_ROUTE_CENTERLINE_SIMPLIFY_PX
    global MULTI_HYPOTHESIS_ROUTING_ENABLED, MULTI_HYPOTHESIS_ROUTING_SIMPLIFY_PX
    global MULTI_HYPOTHESIS_BRANCH_HEADING_SIGMA_DEG
    global N_PARTICLES_STRAIGHT, N_PARTICLES_TURNING, N_PARTICLES_STOPPED
    global SIGMA_STEP_STRAIGHT, SIGMA_STEP_TURNING, SIGMA_STEP_STOPPED
    global SIGMA_ANGLE_STRAIGHT, SIGMA_ANGLE_TURNING, SIGMA_ANGLE_STOPPED
    global BEHAVIOR_WINDOW_SEC, TURN_ENTER_THRESHOLD, TURN_EXIT_THRESHOLD
    global TURN_YAW_RATE_THRESHOLD, PARTICLE_RESIZE_JITTER_PX
    global UNCERTAINTY_ADAPTIVE_PARTICLES, UNCERTAINTY_NEFF_LOW_RATIO, UNCERTAINTY_NEFF_HIGH_RATIO
    global UNCERTAINTY_BOOST_FACTOR, UNCERTAINTY_SHRINK_FACTOR
    global UNCERTAINTY_PARTICLES_MIN, UNCERTAINTY_PARTICLES_MAX

    config_dir = config_path.parent
    config_name = str(config_path)
    M_TO_PIXEL = float(require_config_value(config, "scale_px_per_m", config_name))
    DEFAULT_STEP_GAIN = float(require_config_value(config, "step_gain", config_name))
    # 歩幅校正ゲインは任意設定(無ければ1.0=校正なしで従来通りの挙動)。
    STEP_LENGTH_CALIBRATION_GAIN = (
        args.step_length_calibration_gain
        if args.step_length_calibration_gain is not None
        else float(config.get("step_length_calibration_gain", 1.0))
    )
    # 初期方位校正方式は任意設定(無ければ従来方式samples)。
    HEADING_CALIBRATION_MODE = (
        args.heading_calibration_mode
        if args.heading_calibration_mode is not None
        else str(config.get("heading_calibration_mode", "samples")).lower()
    )
    if HEADING_CALIBRATION_MODE not in {"samples", "walking"}:
        raise ValueError("heading_calibration_modeはsamplesまたはwalkingのいずれかです。")
    HEADING_CALIBRATION_STEPS = (
        args.heading_calibration_steps
        if args.heading_calibration_steps is not None
        else int(config.get("heading_calibration_steps", 10))
    )
    PF_EROSION_RADIUS_PX = int(require_config_value(config, "pf_erosion_radius_px", config_name))
    WALL_WEIGHT_SIGMA = float(require_config_value(config, "wall_weight_sigma", config_name))
    WALL_WEIGHT_FLOOR = float(require_config_value(config, "wall_weight_floor", config_name))
    OFF_ROUTE_WEIGHT = float(require_config_value(config, "off_route_weight", config_name))
    ROUTE_WIDTH_PX = float(require_config_value(config, "route_width_px", config_name))
    ROUTE_HEADING_WEIGHT = float(require_config_value(config, "route_heading_weight", config_name))
    ROUTE_CORNER_THRESHOLD_PX = float(require_config_value(config, "route_corner_threshold_px", config_name))
    configured_route_mode = str(
        require_config_value(config, "route_constraint_mode", config_name)
    ).lower()
    ROUTE_CONSTRAINT_MODE = (
        args.route_constraint_mode
        if args.route_constraint_mode is not None
        else configured_route_mode
    )
    if ROUTE_CONSTRAINT_MODE not in {"none", "prefer", "enforce"}:
        raise ValueError(
            "route_constraint_modeはnone、prefer、enforceのいずれかです。"
        )
    route_points = require_config_value(config, "route_points", config_name)
    ROUTE_POINTS = [tuple(map(float, point)) for point in route_points]
    # route_source・auto_route_max_half_width_pxは任意設定(無ければmanual/20.0px)。
    # autoの場合、JSONのroute_pointsは意図的に使わない(手動座標ゼロで経路帯を作るため)。
    configured_route_source = str(config.get("route_source", "manual")).lower()
    ROUTE_SOURCE = args.route_source if args.route_source is not None else configured_route_source
    if ROUTE_SOURCE not in {"manual", "auto"}:
        raise ValueError("route_sourceはmanualまたはautoのいずれかです。")
    AUTO_ROUTE_MAX_HALF_WIDTH_PX = float(config.get("auto_route_max_half_width_px", 20.0))
    configured_dilation = float(config.get("auto_route_dilation_px", 0.0))
    AUTO_ROUTE_DILATION_PX = (
        args.auto_route_dilation_px if args.auto_route_dilation_px is not None else configured_dilation
    )
    configured_exclude_wide_rooms = bool(config.get("auto_route_exclude_wide_rooms", False))
    AUTO_ROUTE_EXCLUDE_WIDE_ROOMS = (
        args.auto_route_exclude_wide_rooms if args.auto_route_exclude_wide_rooms is not None
        else configured_exclude_wide_rooms
    )
    configured_exclude_radius = config.get("auto_route_exclude_wide_rooms_radius_px")
    AUTO_ROUTE_EXCLUDE_WIDE_ROOMS_RADIUS_PX = (
        args.auto_route_exclude_wide_rooms_radius_px
        if args.auto_route_exclude_wide_rooms_radius_px is not None
        else (float(configured_exclude_radius) if configured_exclude_radius is not None else None)
    )
    if ROUTE_SOURCE == "auto":
        ROUTE_POINTS = []
    # 中心線抽出(自動抽出マスクの細線化)の既定はJSON設定(無ければOFF)。
    # main()でroute_maskが確定した後にextract_ordered_centerline()を呼び、
    # 有効時のみROUTE_POINTSをここでの[]から上書きする。
    configured_centerline_enabled = bool(config.get("auto_route_centerline_enabled", False))
    AUTO_ROUTE_CENTERLINE_ENABLED = (
        args.auto_route_centerline_enabled if args.auto_route_centerline_enabled is not None
        else configured_centerline_enabled
    )
    AUTO_ROUTE_CENTERLINE_SIMPLIFY_PX = (
        args.auto_route_centerline_simplify_px if args.auto_route_centerline_simplify_px is not None
        else float(config.get("auto_route_centerline_simplify_px", 10.0))
    )
    # 複数経路仮説PF(粒子単位のグラフ分岐)の既定はJSON設定(無ければOFF)。
    # AUTO_ROUTE_CENTERLINE_ENABLEDと同様、main()でroute_maskが確定した後に
    # 通路グラフを構築する。「1本の経路を仮定するか、分岐を扱うか」で両者は
    # 排他的なため、両方有効な設定はここでエラーにする。
    configured_multi_hypothesis = bool(config.get("multi_hypothesis_routing_enabled", False))
    MULTI_HYPOTHESIS_ROUTING_ENABLED = (
        args.multi_hypothesis_routing_enabled if args.multi_hypothesis_routing_enabled is not None
        else configured_multi_hypothesis
    )
    if MULTI_HYPOTHESIS_ROUTING_ENABLED and AUTO_ROUTE_CENTERLINE_ENABLED:
        raise ValueError(
            "multi_hypothesis_routing_enabledとauto_route_centerline_enabledは"
            "併用できません(どちらも1本/複数本の経路方位補正を担うため)。"
            "いずれか一方だけを有効にしてください。"
        )
    MULTI_HYPOTHESIS_ROUTING_SIMPLIFY_PX = (
        args.multi_hypothesis_routing_simplify_px
        if args.multi_hypothesis_routing_simplify_px is not None
        else float(config.get("multi_hypothesis_routing_simplify_px", 10.0))
    )
    MULTI_HYPOTHESIS_BRANCH_HEADING_SIGMA_DEG = (
        args.multi_hypothesis_branch_heading_sigma_deg
        if args.multi_hypothesis_branch_heading_sigma_deg is not None
        else float(config.get("multi_hypothesis_branch_heading_sigma_deg", 100000.0))
    )
    GYRO_UNIT = str(require_config_value(config, "gyro_unit", config_name)).lower()
    if GYRO_UNIT not in {"rad", "deg"}:
        raise ValueError("gyro_unit は 'rad' または 'deg' を指定してください。")
    # exclude_csvは任意設定(無ければ除外なし)。別の図で使用中など、このJSONでの
    # 実行対象から外したいCSVファイル名(拡張子込み、完全一致)を列挙する。
    EXCLUDED_CSV_NAMES = set(config.get("exclude_csv", []))

    adaptive = require_config_value(config, "adaptive_pf", config_name)
    adaptive_name = f"{config_name}:adaptive_pf"
    N_PARTICLES_STRAIGHT = int(require_config_value(adaptive, "particles_straight", adaptive_name))
    N_PARTICLES_TURNING = int(require_config_value(adaptive, "particles_turning", adaptive_name))
    N_PARTICLES_STOPPED = int(require_config_value(adaptive, "particles_stopped", adaptive_name))
    SIGMA_STEP_STRAIGHT = float(require_config_value(adaptive, "sigma_step_straight_px", adaptive_name))
    SIGMA_STEP_TURNING = float(require_config_value(adaptive, "sigma_step_turning_px", adaptive_name))
    SIGMA_STEP_STOPPED = float(require_config_value(adaptive, "sigma_step_stopped_px", adaptive_name))
    SIGMA_ANGLE_STRAIGHT = np.deg2rad(float(require_config_value(adaptive, "sigma_angle_straight_deg", adaptive_name)))
    SIGMA_ANGLE_TURNING = np.deg2rad(float(require_config_value(adaptive, "sigma_angle_turning_deg", adaptive_name)))
    SIGMA_ANGLE_STOPPED = np.deg2rad(float(require_config_value(adaptive, "sigma_angle_stopped_deg", adaptive_name)))
    BEHAVIOR_WINDOW_SEC = float(require_config_value(adaptive, "behavior_window_sec", adaptive_name))
    TURN_ENTER_THRESHOLD = np.deg2rad(float(require_config_value(adaptive, "turn_enter_threshold_deg", adaptive_name)))
    TURN_EXIT_THRESHOLD = np.deg2rad(float(require_config_value(adaptive, "turn_exit_threshold_deg", adaptive_name)))
    TURN_YAW_RATE_THRESHOLD = np.deg2rad(float(require_config_value(adaptive, "turn_yaw_rate_threshold_deg_s", adaptive_name)))
    PARTICLE_RESIZE_JITTER_PX = float(require_config_value(adaptive, "particle_resize_jitter_px", adaptive_name))

    # 不確実性適応粒子数(§6.5相当)は任意設定。既定は無効で、JSON/CLIどちらでも
    # 明示的に有効化しない限り既存の挙動(移動様態のみによる粒子数決定)のまま。
    configured_uncertainty = bool(adaptive.get("uncertainty_adaptive_particles", False))
    UNCERTAINTY_ADAPTIVE_PARTICLES = (
        args.uncertainty_adaptive_particles
        if args.uncertainty_adaptive_particles is not None
        else configured_uncertainty
    )
    UNCERTAINTY_NEFF_LOW_RATIO = (
        args.uncertainty_neff_low_ratio if args.uncertainty_neff_low_ratio is not None
        else float(adaptive.get("uncertainty_neff_low_ratio", 0.30))
    )
    UNCERTAINTY_NEFF_HIGH_RATIO = (
        args.uncertainty_neff_high_ratio if args.uncertainty_neff_high_ratio is not None
        else float(adaptive.get("uncertainty_neff_high_ratio", 0.60))
    )
    UNCERTAINTY_BOOST_FACTOR = (
        args.uncertainty_boost_factor if args.uncertainty_boost_factor is not None
        else float(adaptive.get("uncertainty_boost_factor", 1.5))
    )
    UNCERTAINTY_SHRINK_FACTOR = (
        args.uncertainty_shrink_factor if args.uncertainty_shrink_factor is not None
        else float(adaptive.get("uncertainty_shrink_factor", 0.75))
    )
    UNCERTAINTY_PARTICLES_MIN = int(adaptive.get("uncertainty_particles_min", 80))
    UNCERTAINTY_PARTICLES_MAX = int(adaptive.get("uncertainty_particles_max", 1200))

    data_dir = (resolve_config_value_path(config.get("data_dir"), config_dir)
                if args.data_dir is None else args.data_dir.expanduser().resolve())
    map_path = (resolve_config_value_path(config.get("map_image"), config_dir)
                if args.map is None else args.map.expanduser().resolve())
    if data_dir is None:
        raise ValueError("JSONに data_dir を指定してください。")
    if map_path is None:
        raise ValueError("JSONに map_image を指定してください。")

    args.data_dir = data_dir
    args.map = map_path
    args.step_gain = DEFAULT_STEP_GAIN if args.step_gain is None else args.step_gain
    if args.pf_erosion_radius_px is not None:
        PF_EROSION_RADIUS_PX = max(0, args.pf_erosion_radius_px)

    logging.info(f"地図設定JSON: {config_path}")
    logging.info(f"使用地図: {args.map}")
    logging.info(f"CSVフォルダ: {args.data_dir}")
    logging.info(f"縮尺: {M_TO_PIXEL:.2f} px/m")
    logging.info(f"ジャイロ単位: {GYRO_UNIT}/s")
    logging.info(f"PF収縮半径: {PF_EROSION_RADIUS_PX}px")
    logging.info(f"経路制約モード: {ROUTE_CONSTRAINT_MODE}")
    logging.info(f"経路帯生成元: {ROUTE_SOURCE}" + (
        f" (半径閾値={AUTO_ROUTE_MAX_HALF_WIDTH_PX:.1f}px)" if ROUTE_SOURCE == "auto" else ""
    ))
    if ROUTE_SOURCE == "auto":
        logging.info(
            "広い部屋の壁際を除外: " + ("有効" if AUTO_ROUTE_EXCLUDE_WIDE_ROOMS else "無効")
        )
        logging.info(
            "自動中心線抽出(方位補正用): "
            + ("有効" if AUTO_ROUTE_CENTERLINE_ENABLED else "無効")
            + (f" (簡略化閾値={AUTO_ROUTE_CENTERLINE_SIMPLIFY_PX:.1f}px)" if AUTO_ROUTE_CENTERLINE_ENABLED else "")
        )
    logging.info(
        f"経路優先: {len(ROUTE_POINTS)}点, 幅={ROUTE_WIDTH_PX:.1f}px, "
        f"曲がり角判定距離={ROUTE_CORNER_THRESHOLD_PX:.1f}px, "
        f"経路外重み={OFF_ROUTE_WEIGHT:.2f}"
    )
    logging.info(
        f"初期方位校正: {HEADING_CALIBRATION_MODE}"
        + (f" (先頭{HEADING_CALIBRATION_STEPS}歩)" if HEADING_CALIBRATION_MODE == "walking" else "")
    )
    logging.info(
        "歩幅校正ゲイン: " + (
            f"{STEP_LENGTH_CALIBRATION_GAIN:.3f} (暫定値、次回計測の校正用データで再同定)"
            if abs(STEP_LENGTH_CALIBRATION_GAIN - 1.0) > 1e-9 else "1.000 (校正なし)"
        )
    )
    logging.info(
        "不確実性適応粒子数: " + (
            f"有効 (neff比率<{UNCERTAINTY_NEFF_LOW_RATIO:.2f}で×{UNCERTAINTY_BOOST_FACTOR:.2f}, "
            f">{UNCERTAINTY_NEFF_HIGH_RATIO:.2f}で×{UNCERTAINTY_SHRINK_FACTOR:.2f}, "
            f"範囲[{UNCERTAINTY_PARTICLES_MIN},{UNCERTAINTY_PARTICLES_MAX}])"
            if UNCERTAINTY_ADAPTIVE_PARTICLES else "無効"
        )
    )


# [本研究独自] check_sensor_quality.py・pick_landmarks.py・verify_route_graph.pyの
# ような、PF本体を実行せずload_map_config()/apply_map_config()/
# load_preprocessed_map()等だけを再利用したい読み取り専用の検証・診断スクリプトが
# 共通で使う、最小構成のargs.Namespace(通称fake_args)を組み立てるヘルパー。
#
# 【経緯】以前はこのfake_args構築コードを3つのスクリプトへ個別にコピーしており、
# apply_map_config()が参照するargs属性が増えるたび(exclude_wide_rooms追加時、
# 複数経路仮説PF追加時)に3箇所とも同じAttributeErrorで落ちる不具合を2回繰り返した
# (2026-08-16)。ここに1箇所へまとめることで、今後apply_map_config()の参照属性が
# 増えてもここだけ直せばよくなる(3スクリプト側は変更不要)。
#
# fake_argsの各属性をNoneにする意味は、pdr_pf_improved.py本体のCLI引数と同じ
# 「未指定ならJSON設定値をそのまま使う」。地図座標系・設定値だけが必要な
# 読み取り専用スクリプトでは、CLIの全オプションを持つ本物のargparse.Namespaceは
# 不要なため、apply_map_config()が実際に参照する属性だけを埋めた最小限の
# Namespaceで代用する。
def load_map_config_for_tool(map_config_path):
    """検証・診断用スタンドアロンスクリプトから、地図設定を読み込んで
    pdr_pf_improved.pyのグローバル設定値(ROUTE_SOURCE等)を反映させる。

    戻り値: (map_config, resolved_args)。
      map_config: load_map_config()が読み込んだ設定dict。
      resolved_args: apply_map_config()により.map/.data_dirが実際のパスへ
        解決済みのargparse.Namespace。load_preprocessed_map(resolved_args.map)や
        resolved_args.data_dirのように、そのまま後続処理へ渡せる。
    """
    fake_args = argparse.Namespace(
        data_dir=None, map=None, route_constraint_mode=None, route_source=None,
        auto_route_dilation_px=None, step_gain=None,
        step_length_calibration_gain=None,
        heading_calibration_mode=None, heading_calibration_steps=None,
        pf_erosion_radius_px=None,
        auto_route_exclude_wide_rooms=None, auto_route_exclude_wide_rooms_radius_px=None,
        auto_route_centerline_enabled=None, auto_route_centerline_simplify_px=None,
        uncertainty_adaptive_particles=None, uncertainty_neff_low_ratio=None,
        uncertainty_neff_high_ratio=None, uncertainty_boost_factor=None,
        uncertainty_shrink_factor=None,
        multi_hypothesis_routing_enabled=None, multi_hypothesis_routing_simplify_px=None,
        multi_hypothesis_branch_heading_sigma_deg=None,
    )
    map_config, resolved_config_path = load_map_config(map_config_path)
    apply_map_config(fake_args, map_config, resolved_config_path)
    return map_config, fake_args


def compute_acc_magnitude(df):
    return np.sqrt(df['acc_x']**2 + df['acc_y']**2 + df['acc_z']**2)


# [SmartPDR] 重力成分をHPFで除去し、LPFで平滑化してステップ信号を作る前処理。
def compute_step_acceleration(acc_mag: pd.Series):
    gravity = np.zeros(len(acc_mag))
    values = acc_mag.to_numpy()
    if len(values) == 0:
        return pd.Series(dtype=float)

    gravity[0] = values[0]
    for i in range(1, len(values)):
        gravity[i] = HPF_ALPHA * gravity[i - 1] + (1 - HPF_ALPHA) * values[i]

    high_passed = values - gravity
    return pd.Series(high_passed).rolling(
        LPF_WINDOW, center=True, min_periods=1
    ).mean()


# [本研究独自] 上限歩調(MAX_STEP_FREQUENCY_HZ)と実測サンプリング周波数から、
# ステップ検出の最短ピーク間隔(サンプル数)を求める。サンプル数を直書きすると
# 記録レートが変わったときに歩調の上限が意図せず変化するため、周波数側を固定して
# こちらを毎回計算する。周波数が不明・不正な場合のみ従来の固定値へフォールバックする。
def step_min_interval_samples(sampling_rate_hz):
    """サンプリング周波数(Hz)からステップ検出の最短ピーク間隔(サンプル数)を返す。"""
    if (
        sampling_rate_hz is None
        or not np.isfinite(sampling_rate_hz)
        or sampling_rate_hz <= 0
        or MAX_STEP_FREQUENCY_HZ <= 0
    ):
        return STEP_MIN_INTERVAL
    return max(1, int(round(sampling_rate_hz / MAX_STEP_FREQUENCY_HZ)))


# [SmartPDR] ピーク・谷・傾き条件を用いたステップ検出(原論文のステップ検出手法)。
# [本研究独自] 最短ピーク間隔のみ、固定サンプル数から上限歩調ベースへ変更している
# (2026-09-02。過検出の是正。step_min_interval_samples()のコメント参照)。
def detect_steps_smartpdr(step_acc: pd.Series, sampling_rate_hz=None):
    values = step_acc.to_numpy()
    min_interval = step_min_interval_samples(sampling_rate_hz)
    peaks, _ = find_peaks(
        values,
        height=SMART_PEAK_THR,
        distance=min_interval,
    )

    valid_peaks = []
    valley_indices = []
    search_win = max(min_interval, 8)
    for peak in peaks:
        left_start = max(0, peak - search_win)
        right_end = min(len(values), peak + search_win + 1)
        if peak <= left_start or peak + 1 >= right_end:
            continue

        left_segment = values[left_start:peak]
        right_segment = values[peak + 1:right_end]
        if len(left_segment) == 0 or len(right_segment) == 0:
            continue

        left_valley = left_start + int(np.argmin(left_segment))
        right_valley = peak + 1 + int(np.argmin(right_segment))
        peak_to_peak = min(
            values[peak] - values[left_valley],
            values[peak] - values[right_valley],
        )
        if peak_to_peak < SMART_PP_THR:
            continue

        front_start = max(1, peak - SMART_SLOPE_WIN)
        back_end = min(len(values) - 1, peak + SMART_SLOPE_WIN)
        front_slope = np.mean(np.diff(values[front_start - 1:peak + 1]))
        back_slope = np.mean(np.diff(values[peak:back_end + 1]))
        if front_slope <= 0 or back_slope >= 0:
            continue

        valid_peaks.append(peak)
        valley_indices.append(left_valley)

    return np.array(valid_peaks, dtype=int), np.array(valley_indices, dtype=int)


def estimate_smartpdr_step_length_px(step_acc: pd.Series, peak_idx: int, valley_idx: int):
    """[SmartPDR] 論文の4乗根式/対数式切り替えに基づいた歩幅推定"""
    impact = max(float(step_acc.iloc[peak_idx] - step_acc.iloc[valley_idx]), 1e-6)
    if impact < SMART_STEP_TAU:
        step_m = ROOT_BETA * (impact ** 0.25) + ROOT_GAMMA
    else:
        step_m = LOG_BETA * np.log(impact) + LOG_GAMMA
    # [本研究独自] 被験者・端末の校正ゲインを、物理的な上下限クリップの前に適用する
    # (2026-09-02。STEP_LENGTH_CALIBRATION_GAINのコメント参照)。
    step_m *= STEP_LENGTH_CALIBRATION_GAIN
    step_m = np.clip(step_m, MIN_STEP_M, MAX_STEP_M)
    return step_m * M_TO_PIXEL


def get_yaw_rate(gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z):
    """【物理演算最適化】
    ジャイロ角速度ベクトルと重力方向（正規化加速度ベクトル）の内積により、
    三角関数によるロール・ピッチ投影演算およびオイラー角特有の特異点(ジンバルロック)を完全に排除し、
    劇的に高速かつ数値的に安定したヨーレート算出を実現します。
    """
    norm = np.sqrt(acc_x**2 + acc_y**2 + acc_z**2)
    if norm < 1e-6:
        return gyro_z  # 加速度データが得られない場合の安全なフォールバック
    return (gyro_x * acc_x + gyro_y * acc_y + gyro_z * acc_z) / norm


def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def angle_diff(a, b):
    return normalize_angle(a - b)


def weighted_angle_mean(angles, weights):
    sin_sum = np.sum(np.sin(angles) * weights)
    cos_sum = np.sum(np.cos(angles) * weights)
    return np.arctan2(sin_sum, cos_sum)


# [本研究独自] ここから先(route_guidance_enabled 〜 advance_route_segment)は、
# 地図から抽出した通路方向情報(route_points)を用いて曲がり判定・方位補正を行う
# 一連の処理。SmartPDR・移動様態適応PFのどちらにも地図の通路方向を使う処理は
# 存在せず、本研究が「地図形状の利用」として追加した部分の中心にあたる。
def route_guidance_enabled():
    """prefer/enforceかつroute_pointsが有効な場合だけ経路案内を使う。"""
    return (
        ROUTE_CONSTRAINT_MODE in {"prefer", "enforce"}
        and len(ROUTE_POINTS) >= 2
    )


def get_route_segment_heading(route_segment_index):
    """指定された経路線分の画像座標系における方位角を返す。"""
    if not route_guidance_enabled():
        return None

    index = int(np.clip(route_segment_index, 0, len(ROUTE_POINTS) - 2))
    start = np.asarray(ROUTE_POINTS[index], dtype=float)
    end = np.asarray(ROUTE_POINTS[index + 1], dtype=float)
    direction = end - start

    if np.linalg.norm(direction) <= 1e-9:
        return None

    # 画像座標では右=0、下=+pi/2、上=-pi/2。
    return np.arctan2(direction[1], direction[0])


def nearest_route_segment_index(x, y):
    """クリック開始位置に最も近いroute_points線分を返す。"""
    if not route_guidance_enabled():
        return 0

    point = np.array([x, y], dtype=float)
    best_index = 0
    best_distance = float("inf")
    for index, (start, end) in enumerate(zip(ROUTE_POINTS[:-1], ROUTE_POINTS[1:])):
        start = np.asarray(start, dtype=float)
        end = np.asarray(end, dtype=float)
        direction = end - start
        length_sq = float(np.dot(direction, direction))
        if length_sq <= 1e-12:
            continue
        t = float(np.clip(np.dot(point - start, direction) / length_sq, 0.0, 1.0))
        nearest = start + t * direction
        distance = float(np.linalg.norm(point - nearest))
        if distance < best_distance:
            best_distance = distance
            best_index = index
    return best_index


def estimate_initial_sensor_heading(df, source, sample_count, step_indices=None):
    """開始時センサ方位(地図座標系へ合わせるための基準)を円平均で求める。

    HEADING_CALIBRATION_MODE で2方式を切り替える。

    "samples"(既定・従来方式): 記録の先頭sample_count個の有限値を円平均する。

    "walking"([本研究独自]、2026-09-02追加): 検出済みステップの先頭
    HEADING_CALIBRATION_STEPS歩の時点の方位を円平均する。従来方式は先頭
    sample_count(既定30)サンプル=約0.57秒しか見ておらず、計測開始ボタンを押した
    直後にまだ端末を持ち替えている・体の向きを変えている場合に基準方位が壊れる。
    実際、pdr_log_0805_1438/1441では基準窓を30→250サンプルへ広げるだけで基準方位が
    それぞれ-61度/-51度も動き、この窓が短すぎることが確認された(1442は-3度で安定)。
    歩き始めてからの方位を使えば、静止中の持ち替えの影響を受けない。
    なお、この方式は「開始時の歩行方向が--initial-heading-degである」という前提
    (従来方式と同じ、利用者が計測時の事実として与える情報)だけを使っており、
    正解経路や推定結果を参照していないので循環参照にはならない。
    詳細はCHANGELOG.md 2026-09-02(4)項・memo/heading_calibration.md。
    """
    if source != "android":
        # gyro方式の基準方位は、この関数の呼び出し時点ではまだ計算できない
        # (Madgwick等の逐次計算がメインループ内で進むため)。従来通りNoneを返し、
        # 呼び出し側が最初の有効行のraw_headingを基準として採用する。
        return None
    if "yaw_deg" not in df.columns:
        raise ValueError("--heading-source android にはCSVのyaw_deg列が必要です。")
    values = np.deg2rad(pd.to_numeric(df["yaw_deg"], errors="coerce").to_numpy())

    if HEADING_CALIBRATION_MODE == "walking" and step_indices is not None and len(step_indices) > 0:
        picked = np.asarray(step_indices)[:max(1, int(HEADING_CALIBRATION_STEPS))]
        walking_values = values[picked]
        walking_values = walking_values[np.isfinite(walking_values)]
        if len(walking_values) > 0:
            return weighted_angle_mean(walking_values, np.ones(len(walking_values)))
        logging.warning(
            "歩行開始基準の方位校正に有効なyaw_degがありませんでした。"
            "先頭サンプル方式へフォールバックします。"
        )

    finite_values = values[np.isfinite(values)][:max(1, int(sample_count))]
    if len(finite_values) == 0:
        raise ValueError("yaw_degに有効な値がありません。")
    return weighted_angle_mean(finite_values, np.ones(len(finite_values)))


def correct_heading_with_route_segment(sensor_heading, route_segment_index):
    """現在選択中の経路線分方向とセンサー方位を重み付き融合する。"""
    route_heading = get_route_segment_heading(route_segment_index)
    if route_heading is None or ROUTE_HEADING_WEIGHT <= 0:
        return sensor_heading

    return weighted_angle_mean(
        np.array([sensor_heading, route_heading]),
        np.array([1.0, ROUTE_HEADING_WEIGHT]),
    )


def advance_route_segment(route_segment_index):
    """経路線分を1つ進める。前の線分へは戻さない。"""
    if not route_guidance_enabled():
        return 0
    return min(route_segment_index + 1, len(ROUTE_POINTS) - 2)


def distance_to_route_corner(x, y, route_segment_index):
    """現在線分の終点、つまり次の曲がり角までの距離を返す。"""
    if not route_guidance_enabled():
        return float("inf")
    corner_index = min(route_segment_index + 1, len(ROUTE_POINTS) - 1)
    corner_x, corner_y = ROUTE_POINTS[corner_index]
    return float(np.hypot(x - corner_x, y - corner_y))


def is_near_route_corner(x, y, route_segment_index):
    """PF推定位置が設定経路の曲がり角付近にあるか判定する。"""
    return distance_to_route_corner(x, y, route_segment_index) <= ROUTE_CORNER_THRESHOLD_PX


def resample_if_needed(particles, weights):
    n = len(weights)
    neff = 1.0 / (np.sum(weights**2) + 1e-300)
    if neff < n / 2:
        indices   = np.random.choice(n, size=n, p=weights)
        particles = particles[indices]
        weights   = np.full(n, 1.0 / n)
    return particles, weights


def validate_log(df, file_name):
    required = ['timestamp', 'acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z']
    missing = [col for col in required if col not in df.columns]
    if missing:
        raise ValueError(f"{file_name} に必要な列がありません: {', '.join(missing)}")

    df = df.dropna(subset=required).reset_index(drop=True)
    df = df[df['timestamp'].diff().fillna(1) > 0].reset_index(drop=True)
    return df


def load_preprocessed_map(img_path):
    if not img_path.exists():
        raise FileNotFoundError(f"マップ画像を読み込めません: {img_path}")
    gray = np.array(Image.open(img_path).convert("L"))
    BINARY_THRESHOLD = 128
    binary = np.where(gray >= BINARY_THRESHOLD, 255, 0).astype(np.uint8)
    passage = binary == 255
    radius = max(0, int(PF_EROSION_RADIUS_PX))
    if radius > 0:
        yy, xx = np.ogrid[-radius:radius + 1, -radius:radius + 1]
        disk = xx * xx + yy * yy <= radius * radius
        passage_pf = ndimage.binary_erosion(passage, structure=disk)
    else:
        passage_pf = passage
    binary_for_pf_local = np.where(passage_pf, 255, 0).astype(np.uint8)
    dist_map = ndimage.distance_transform_edt(passage_pf)
    maximum = float(dist_map.max())
    if maximum > 0:
        dist_map /= maximum
    logging.info(f"前処理済み地図: passage_ratio={passage.mean():.3f}, PF用={passage_pf.mean():.3f}")
    return binary, binary_for_pf_local, dist_map


# [本研究独自] 二値地図上にroute_pointsを描画して太らせた「経路帯マスク」を作る。
# 経路優先/強制の重み付け(ParticleFilterPDR.update)や曲がり角近傍判定の土台。
def build_route_mask(shape):
    mask = np.zeros(shape, dtype=bool)
    if not route_guidance_enabled():
        return np.ones(shape, dtype=bool)
    for start, end in zip(ROUTE_POINTS[:-1], ROUTE_POINTS[1:]):
        x0, y0 = start
        x1, y1 = end
        count = max(2, int(np.hypot(x1 - x0, y1 - y0)) + 1)
        xs = np.rint(np.linspace(x0, x1, count)).astype(int)
        ys = np.rint(np.linspace(y0, y1, count)).astype(int)
        valid = (xs >= 0) & (xs < shape[1]) & (ys >= 0) & (ys < shape[0])
        mask[ys[valid], xs[valid]] = True
    radius = max(1, int(round(ROUTE_WIDTH_PX)))
    yy, xx = np.ogrid[-radius:radius + 1, -radius:radius + 1]
    disk = xx * xx + yy * yy <= radius * radius
    return ndimage.binary_dilation(mask, structure=disk)


# [本研究独自] 経路帯マスク抽出・通路グラフ化(骨格化・ノード整理・トポロジー化)
# 関連の関数は、ファイル肥大化(3500行超)を受けてpdr_route_graph.pyへ切り出した
# (2026-08-16)。extract_auto_route_mask/extract_ordered_centerline/
# build_skeleton_graph/simplify_skeleton_graph/build_route_graph_topology/
# nearest_edge_positionは、いずれも冒頭のimportで再importしているため、
# このファイル内では従来通りそのまま(プレフィックス無しで)呼び出せる。


# グローバルな描画・監視状態
redraw_requested = threading.Event()
result_cache = PDRResultCache()


# CSV 変更検出ハンドラ
class CSVHandler(FileSystemEventHandler):

    def request_redraw(self, path):
        csv_path = Path(path)
        if csv_path.suffix.lower() != ".csv":
            return
        if not csv_path.name.startswith("pdr_log_"):
            return

        logging.info("\n===================================")
        logging.info("CSV change detected: %s", csv_path.name)
        logging.info("===================================\n")
        redraw_requested.set()

    def on_created(self, event):
        if not event.is_directory:
            self.request_redraw(event.src_path)

    def on_modified(self, event):
        if not event.is_directory:
            self.request_redraw(event.src_path)

    def on_moved(self, event):
        if not event.is_directory:
            self.request_redraw(event.dest_path)


def redraw_all_paths():
    global binary, binary_for_pf, dist_map, h, w, result_cache
    ax.clear()
    ax.imshow(binary, cmap='gray')
    ax.set_xlim(0, w)
    ax.set_ylim(h, 0)

    # [本研究独自] route_source=autoの場合、地図から自動抽出した通路帯を薄く塗って表示する
    # (手動route_pointsの破線と対比できるように、not全域Trueのときだけ描画)。
    if ROUTE_SOURCE == "auto" and route_mask is not None and not route_mask.all():
        overlay = np.zeros((*route_mask.shape, 4))
        overlay[route_mask] = (1.0, 0.0, 0.0, 0.15)
        ax.imshow(overlay, zorder=1)

    # JSONで指定した優先経路を破線表示する。
    if len(ROUTE_POINTS) >= 2:
        route_array = np.asarray(ROUTE_POINTS, dtype=float)
        ax.plot(
            route_array[:, 0],
            route_array[:, 1],
            color='magenta',
            linestyle='--',
            linewidth=1.5,
            alpha=0.8,
            label='Configured route',
        )
        ax.scatter(
            route_array[:, 0],
            route_array[:, 1],
            color='magenta',
            s=24,
            zorder=8,
        )

    file_list = glob.glob(str(data_dir / "pdr_log_*.csv"))
    if EXCLUDED_CSV_NAMES:
        excluded_found = [f for f in file_list if Path(f).name in EXCLUDED_CSV_NAMES]
        file_list = [f for f in file_list if Path(f).name not in EXCLUDED_CSV_NAMES]
        for f in excluded_found:
            logging.info(f"[{Path(f).name}] exclude_csv設定によりスキップ")
    file_list.sort()

    if not file_list:
        logging.info("CSVファイルなし")
        return

    start_overall = time.time()

    saved_start_positions = load_start_positions()

    for file_path_str in file_list:
        file_path = Path(file_path_str)
        file_name = file_path.name

        # [本研究独自] このCSV専用の乱数系列へ切り替える(file_random_seed参照)。
        # 処理順・ファイル集合が変わっても各CSVの結果が変わらないようにするため。
        file_seed = file_random_seed(args.seed, file_name)
        if file_seed is not None:
            np.random.seed(file_seed)

        file_start_x, file_start_y = get_or_select_start_position(
            file_name,
            binary,
            binary_for_pf,
            saved_start_positions,
        )
        logging.info(
            "[%s] 使用開始位置: x=%.1f, y=%.1f",
            file_name,
            file_start_x,
            file_start_y,
        )

        # 開始位置・経路モード・方位設定が変わった場合は古い結果を再利用しない。
        cache_context = (
            round(file_start_x, 6),
            round(file_start_y, 6),
            ROUTE_CONSTRAINT_MODE,
            args.heading_source,
            round(float(args.initial_heading_deg), 6),
            int(args.heading_calibration_samples),
            tuple(ROUTE_POINTS),
            float(ROUTE_HEADING_WEIGHT),
            float(OFF_ROUTE_WEIGHT),
        )

        # キャッシュの取得判定
        cached_result = result_cache.get(file_path_str, context=cache_context)
        if cached_result is not None:
            logging.info(f"[{file_name}] (Cached) 総ステップ数: {cached_result['step_count']}  全滅回数: {cached_result['extinction_count']}")
            estimated_positions = cached_result['estimated_positions']
            behavior_history = cached_result.get('behavior_history', [])
            particle_count_history = cached_result.get('particle_count_history', [])
            step_timestamps = cached_result.get('step_timestamps', [])
            route_segment_index = cached_result.get('route_segment_index', 0)
            turn_pending = cached_result.get('turn_pending', False)
            file_start_x, file_start_y = cached_result.get(
                'start_position',
                (file_start_x, file_start_y),
            )
        else:
            try:
                # 安全なファイル読み込み (同時書き込み時の不完全読み込みを防止)
                df = safe_read_csv(file_path_str)
                df = validate_log(df, file_name)
            except (ValueError, IOError) as e:
                logging.warning(f"[{file_name}] 処理をスキップしました: {e}")
                continue

            if GYRO_UNIT == "deg":
                gyro_columns = ["gyro_x", "gyro_y", "gyro_z"]
                df[gyro_columns] = np.deg2rad(df[gyro_columns])
            if len(df) < 2:
                logging.info(f"[{file_name}] 有効なデータが少ないためスキップします。")
                continue

            df['acc_mag']    = compute_acc_magnitude(df)
            df['step_acc']   = compute_step_acceleration(df['acc_mag'])
            # [本研究独自] ステップ検出の最短間隔を上限歩調から決めるため、CSVごとの
            # 実測サンプリング周波数を求めて渡す(2026-09-02。過検出の是正)。
            timestamp_diff_mean = df['timestamp'].diff().mean()
            sampling_rate_hz = (
                1.0 / timestamp_diff_mean
                if pd.notna(timestamp_diff_mean) and timestamp_diff_mean > 0
                else None
            )
            step_indices, valley_indices = detect_steps_smartpdr(
                df['step_acc'], sampling_rate_hz
            )
            valley_by_step = dict(zip(step_indices, valley_indices))

            duration_sec = float(df['timestamp'].iloc[-1] - df['timestamp'].iloc[0])
            cadence_text = (
                f", 歩調={len(step_indices) / duration_sec:.2f}歩/s"
                if duration_sec > 0 else ""
            )
            logging.info(
                f"[{file_name}] 総行数: {len(df)}  検出ステップ数: {len(step_indices)}"
                + (f"  (サンプリング={sampling_rate_hz:.1f}Hz, 最短間隔="
                   f"{step_min_interval_samples(sampling_rate_hz)}サンプル{cadence_text})"
                   if sampling_rate_hz is not None else "")
            )

            if len(step_indices) > 0:
                raw_step_lengths = [
                    estimate_smartpdr_step_length_px(df['step_acc'], peak, valley)
                    for peak, valley in zip(step_indices, valley_indices)
                ]
                raw_total_dist = sum(raw_step_lengths)
                step_lengths = [length * args.step_gain for length in raw_step_lengths]
                step_length_by_step = dict(zip(step_indices, step_lengths))
                total_dist = sum(step_lengths)
                logging.info(f"  SmartPDR歩幅推定 (px): 平均={np.mean(step_lengths):.1f}  "
                      f"≈ 平均{np.mean(step_lengths)/M_TO_PIXEL:.2f}m/歩")
                logging.info(f"  推定総移動距離: {total_dist:.0f}px  "
                      f"(歩幅校正ゲイン適用後、step_gain={args.step_gain:.2f})")
            else:
                logging.info("  ステップが検出されなかったため、このログは描画されません。")
                continue

            # OOP設計によるパーティクルフィルタ初期化
            pf_params = {
                'sigma_step': SIGMA_STEP_STRAIGHT,
                'sigma_angle': SIGMA_ANGLE_STRAIGHT,
                'wall_weight_sigma': WALL_WEIGHT_SIGMA,
                'wall_weight_floor': WALL_WEIGHT_FLOOR,
                'off_route_weight': OFF_ROUTE_WEIGHT,
                'smooth_window': SMOOTH_WINDOW,
                'recovery_sigma': RECOVERY_SIGMA
            }

            pf = ParticleFilterPDR(
                n_particles=N_PARTICLES_STRAIGHT,
                start_x=file_start_x,
                start_y=file_start_y,
                route_mask=route_mask,
                binary_for_pf=binary_for_pf,
                dist_map=dist_map,
                params=pf_params,
                route_topology=route_topology,
            )

            gyro_angle          = 0.0
            heading             = 0.0
            heading_history     = np.zeros(len(df))
            yaw_rate_history    = np.zeros(len(df))
            step_set            = set(step_indices)
            current_behavior    = MoveBehavior.STRAIGHT
            behavior_history    = []
            particle_count_history = []
            step_timestamps     = []  # [本研究独自] estimated_positionsと1:1対応するtimestamp
            # 曲がり検出はturn_pendingとして保持し、設定曲がり角へ
            # 到達した時だけ次の線分へ進む。
            route_segment_index = nearest_route_segment_index(
                file_start_x,
                file_start_y,
            )
            turn_pending = False
            initial_map_heading = np.deg2rad(args.initial_heading_deg)
            if route_topology is not None:
                # [本研究独自] 複数経路仮説PF: 開始位置に最も近いエッジ・区間へ
                # 全粒子を割り当てる(進行方向はinitial_map_headingとの整合で決める)。
                pf.initialize_route_state(file_start_x, file_start_y, initial_map_heading)
            try:
                # --heading-source androidをyaw_deg列の無い旧CSVへ適用した場合など、
                # この1ファイルだけの設定不備でバッチ全体が中断しないようにする。
                initial_sensor_heading = estimate_initial_sensor_heading(
                    df,
                    args.heading_source,
                    args.heading_calibration_samples,
                    step_indices,
                )
            except ValueError as e:
                logging.warning(f"[{file_name}] 処理をスキップしました: {e}")
                continue
            logging.info(
                "  初期方位補正: source=%s, map=%.1fdeg, route_segment=%d",
                args.heading_source,
                args.initial_heading_deg,
                route_segment_index,
            )

            # 実際のサンプリングレートを推定して、Madgwickフィルタをインスタンス化
            madgwick = None
            if HAS_AHRS:
                dt_mean = df['timestamp'].diff().mean()
                if pd.notna(dt_mean) and dt_mean > 0:
                    fs = 1.0 / dt_mean
                    madgwick = Madgwick(frequency=fs)
                else:
                    madgwick = Madgwick()
            
            Q = np.tile([1., 0., 0., 0.], (len(df), 1)) if HAS_AHRS else None
            step_count = 0

            for i in range(1, len(df)):
                row      = df.iloc[i]
                prev_row = df.iloc[i - 1]

                dt = row['timestamp'] - prev_row['timestamp']
                if dt <= 0 or dt > MAX_DT:
                    continue

                yaw_rate = get_yaw_rate(
                    row['gyro_x'], row['gyro_y'], row['gyro_z'],
                    row['acc_x'], row['acc_y'], row['acc_z']
                )

                if HAS_AHRS:
                    try:
                        gyro = np.array([
                            row['gyro_x'],
                            row['gyro_y'],
                            row['gyro_z']
                        ])
                        acc = np.array([
                            row['acc_x'],
                            row['acc_y'],
                            row['acc_z']
                        ])
                        Q[i] = madgwick.updateIMU(q=Q[i - 1], gyr=gyro, acc=acc)

                        rot = R.from_quat([Q[i][1], Q[i][2], Q[i][3], Q[i][0]])
                        gyro_angle = normalize_angle(rot.as_euler('xyz')[2])
                    except Exception as e:
                        logging.debug("Madgwick failed: %s", e)
                        # fallback to simple yaw rate
                        gyro_angle = normalize_angle(gyro_angle * ANGLE_DECAY + yaw_rate * dt)
                else:
                    gyro_angle = normalize_angle(gyro_angle * ANGLE_DECAY + yaw_rate * dt)

                fused_heading = gyro_angle

                if args.heading_source == "android":
                    yaw_value = pd.to_numeric(row.get("yaw_deg", np.nan), errors="coerce")
                    raw_heading = (
                        np.deg2rad(float(yaw_value))
                        if np.isfinite(yaw_value)
                        else fused_heading
                    )
                else:
                    raw_heading = fused_heading

                if initial_sensor_heading is None:
                    initial_sensor_heading = raw_heading

                heading = normalize_angle(
                    raw_heading - initial_sensor_heading + initial_map_heading
                )
                heading_history[i] = heading
                yaw_rate_history[i] = yaw_rate

                if i not in step_set:
                    continue

                step_count += 1
                valley_idx = valley_by_step.get(i, i)
                step_px = step_length_by_step.get(
                    i,
                    estimate_smartpdr_step_length_px(df['step_acc'], i, valley_idx),
                )
                
                prev_step = 0
                if step_count >= 2:
                    prev_step = step_indices[
                        max(0, np.where(step_indices == i)[0][0] - 1)
                    ]

                segment_heading = heading_history[prev_step:i + 1]
                step_heading = weighted_angle_mean(segment_heading, np.ones(len(segment_heading)))
                
                # 1. センサー方位を使って移動様態を先に判定する。
                #    経路補正後の方位で判定すると、実際の曲がりが水平経路へ
                #    引き戻されるため、必ず補正前に判定する。
                sensor_step_heading = step_heading
                previous_behavior = current_behavior
                current_behavior = detect_move_behavior(
                    df['timestamp'].to_numpy(),
                    heading_history,
                    yaw_rate_history,
                    i,
                    previous_behavior,
                    step_detected=True,
                )

                # 2. 曲がりを検出しても即時に経路を切り替えず、予定として保持する。
                if (
                    route_guidance_enabled()
                    and previous_behavior != MoveBehavior.TURNING
                    and current_behavior == MoveBehavior.TURNING
                    and route_segment_index < len(ROUTE_POINTS) - 2
                ):
                    turn_pending = True
                    logging.info(
                        "  曲がり予定を検出: step=%d, segment=%d",
                        step_count,
                        route_segment_index,
                    )

                if current_behavior != previous_behavior:
                    logging.info(
                        "  移動様態変化: step=%d, %s -> %s",
                        step_count,
                        previous_behavior.value,
                        current_behavior.value,
                    )

                # 3. PF推定位置が設定曲がり角へ近づいた場合だけ線分を切り替える。
                ref_x = np.average(pf.particles[:, 0], weights=pf.weights)
                ref_y = np.average(pf.particles[:, 1], weights=pf.weights)
                corner_distance = distance_to_route_corner(ref_x, ref_y, route_segment_index)

                if (
                    route_guidance_enabled()
                    and turn_pending
                    and route_segment_index < len(ROUTE_POINTS) - 2
                    and is_near_route_corner(ref_x, ref_y, route_segment_index)
                ):
                    old_segment_index = route_segment_index
                    route_segment_index = advance_route_segment(route_segment_index)
                    turn_pending = False
                    logging.info(
                        "  経路線分切替: step=%d, segment=%d -> %d, "
                        "position=(%.1f, %.1f), corner_distance=%.1fpx",
                        step_count,
                        old_segment_index,
                        route_segment_index,
                        ref_x,
                        ref_y,
                        corner_distance,
                    )

                # 4. 現在選択中の経路線分方向でセンサー方位を補正する。
                #    曲がり終了後もroute_segment_indexは維持されるため、
                #    縦通路へ入った後に水平線分へ戻らない。
                step_heading = correct_heading_with_route_segment(
                    sensor_step_heading,
                    route_segment_index,
                )

                route_heading = get_route_segment_heading(route_segment_index)
                logging.debug(
                    "step=%d behavior=%s segment=%d sensor=%.1fdeg "
                    "route=%s corrected=%.1fdeg yaw_rate=%.1fdeg/s",
                    step_count,
                    current_behavior.value,
                    route_segment_index,
                    np.rad2deg(sensor_step_heading),
                    "None" if route_heading is None else f"{np.rad2deg(route_heading):.1f}deg",
                    np.rad2deg(step_heading),
                    np.rad2deg(yaw_rate_history[i]),
                )

                # 4. 移動様態に応じて粒子数・歩幅分散・方位分散を切り替えてPF更新。
                pf.update(step_px, step_heading, current_behavior)
                behavior_history.append(current_behavior.value)
                particle_count_history.append(len(pf.particles))
                step_timestamps.append(float(row['timestamp']))

            estimated_positions = pf.estimated_positions
            extinction_count = pf.extinction_count

            # 計算結果をキャッシュ
            result_cache.set(file_path_str, {
                'estimated_positions': estimated_positions,
                'step_count': step_count,
                'extinction_count': extinction_count,
                'behavior_history': behavior_history,
                'particle_count_history': particle_count_history,
                'step_timestamps': step_timestamps,
                'route_segment_index': route_segment_index,
                'turn_pending': turn_pending,
                'start_position': (file_start_x, file_start_y),
            }, context=cache_context)

            logging.info(f"  処理ステップ数: {step_count}  全滅回数: {extinction_count}")
            logging.info(f"  最終経路線分: segment={route_segment_index}, turn_pending={turn_pending}")
            if particle_count_history:
                straight_count = behavior_history.count(MoveBehavior.STRAIGHT.value)
                turning_count = behavior_history.count(MoveBehavior.TURNING.value)
                stopped_count = behavior_history.count(MoveBehavior.STOPPED.value)
                logging.info(
                    "  移動様態: "
                    f"直進={straight_count}, 曲がり={turning_count}, 滞留={stopped_count}"
                )
                logging.info(
                    "  パーティクル数: "
                    f"平均={np.mean(particle_count_history):.1f}, "
                    f"最小={np.min(particle_count_history)}, "
                    f"最大={np.max(particle_count_history)}"
                )
            if pf.valid_ratio_history:
                logging.info(
                    "  PF診断: 有効粒子率=%.3f, 経路内率=%.3f, "
                    "Neff平均=%.1f, 位置分散平均=%.1f",
                    np.mean(pf.valid_ratio_history),
                    np.mean(pf.route_ratio_history),
                    np.mean(pf.neff_history),
                    np.mean(pf.position_spread_history),
                )
            if UNCERTAINTY_ADAPTIVE_PARTICLES:
                logging.info(
                    "  不確実性適応: 増加ステップ=%d, 減少ステップ=%d",
                    pf.uncertainty_boost_count,
                    pf.uncertainty_shrink_count,
                )
            if estimated_positions:
                last = estimated_positions[-1]
                logging.info(f"  最終推定位置: x={last[0]:.1f}, y={last[1]:.1f}")

        if estimated_positions:
            pos_arr = np.array(estimated_positions)
            line, = ax.plot(
                pos_arr[:, 0],
                pos_arr[:, 1],
                linewidth=2,
                label=file_name,
            )
            ax.scatter(
                file_start_x,
                file_start_y,
                s=55,
                zorder=5,
                color=line.get_color(),
                edgecolors="white",
                linewidths=0.7,
            )

            # [本研究独自] --save-trajectory-csv指定時のみ、正解位置データとの
            # 突き合わせ用にCSVごとの推定軌跡を保存する(既定では保存しない)。
            if args.save_trajectory_csv:
                if len(step_timestamps) != len(estimated_positions):
                    logging.warning(
                        "  推定軌跡CSVを保存できません: step_timestamps(%d件)と"
                        "estimated_positions(%d件)の件数が一致しません。",
                        len(step_timestamps),
                        len(estimated_positions),
                    )
                else:
                    # 実行条件をファイル名に含める(2026-09-02)。従来は
                    # "{CSV名}_trajectory.csv"固定で、条件やシードを変えて実行すると
                    # 黙って上書きされ、条件間の比較ができなかった。
                    seed_text = "none" if args.seed is None else str(args.seed)
                    traj_name = (
                        f"{Path(file_name).stem}_traj"
                        f"_{ROUTE_CONSTRAINT_MODE}-{ROUTE_SOURCE}"
                        f"_{args.heading_source}-{HEADING_CALIBRATION_MODE}"
                        f"_seed-{seed_text}.csv"
                    )
                    traj_path = (RESULTS_DIR / traj_name).resolve()
                    traj_path.parent.mkdir(parents=True, exist_ok=True)
                    pd.DataFrame({
                        "timestamp": step_timestamps,
                        "x_px": pos_arr[:, 0],
                        "y_px": pos_arr[:, 1],
                    }).to_csv(traj_path, index=False)
                    logging.info(f"  推定軌跡CSVを保存しました: {traj_path}")

    end_overall = time.time()
    logging.info("\n適応型パーティクルフィルタを使用")
    logging.info(f"Overall time: {end_overall - start_overall:.2f} seconds")

    unc_tag = "+unc" if UNCERTAINTY_ADAPTIVE_PARTICLES else ""
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    title = (
        "移動様態適応型PF + SmartPDR "
        f"（経路制約: {ROUTE_CONSTRAINT_MODE}/{ROUTE_SOURCE}{unc_tag}, 方位: {args.heading_source}）"
    )
    if japanize_matplotlib is None:
        title = (
            "Behavior-adaptive PF + SmartPDR "
            f"(route={ROUTE_CONSTRAINT_MODE}/{ROUTE_SOURCE}{unc_tag}, heading={args.heading_source})"
        )
    ax.set_title(title)

    fig.subplots_adjust(right=0.72)
    fig.canvas.draw()
    fig.canvas.flush_events()

    # 【本研究独自】卒業論文の証拠画像・実行ログとして、実行のたびに必ず結果PNGを
    # 保存する。--save指定時はそのパスへ、未指定時はRESULTS_DIR以下へ実行日時・
    # 経路制約モード・乱数シードを含む名前で自動保存する(進捗メモ15章に対応)。
    if args.save is not None:
        save_path = args.save.resolve()
    else:
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        seed_text = "none" if args.seed is None else str(args.seed)
        # 方位源と初期方位校正方式もファイル名へ含める(2026-09-02追加)。これらを
        # 変えた実行が同名パターンになり、掃引結果のPNGを後から区別できなかったため。
        heading_tag = f"{args.heading_source}-{HEADING_CALIBRATION_MODE}"
        auto_name = (
            f"{timestamp}_route-{ROUTE_CONSTRAINT_MODE}-{ROUTE_SOURCE}{unc_tag}"
            f"_head-{heading_tag}_seed-{seed_text}.png"
        )
        save_path = (RESULTS_DIR / auto_name).resolve()
    save_path.parent.mkdir(parents=True, exist_ok=True)
    fig.set_size_inches(10, 8, forward=True)
    fig.savefig(save_path, dpi=200, bbox_inches='tight')
    logging.info(f"結果画像を保存しました: {save_path}")


def main():
    global args, data_dir, img_path, binary, binary_for_pf, dist_map, h, w, route_mask, fig, ax
    global ROUTE_POINTS, route_topology

    args = parse_args()
    numeric_level = getattr(logging, args.log_level.upper(), None)
    if not isinstance(numeric_level, int):
        numeric_level = logging.INFO
    logging.basicConfig(level=numeric_level, format='%(levelname)s: %(message)s')

    map_config, map_config_path = load_map_config(args.map_config)
    apply_map_config(args, map_config, map_config_path)
    data_dir = args.data_dir
    img_path = args.map
    if args.seed is not None:
        np.random.seed(args.seed)

    binary, binary_for_pf, dist_map = load_preprocessed_map(img_path)
    h, w = binary.shape
    if not data_dir.exists():
        raise FileNotFoundError(f"CSVフォルダが見つかりません: {data_dir}")
    route_topology = None  # [本研究独自] 複数経路仮説PF。route_source=auto以外・未有効時はNoneのまま。
    if ROUTE_SOURCE == "auto":
        route_mask = extract_auto_route_mask(
            binary_for_pf, AUTO_ROUTE_MAX_HALF_WIDTH_PX, AUTO_ROUTE_DILATION_PX,
            exclude_wide_rooms=AUTO_ROUTE_EXCLUDE_WIDE_ROOMS,
            exclude_wide_rooms_radius_px=AUTO_ROUTE_EXCLUDE_WIDE_ROOMS_RADIUS_PX,
        )
        logging.info(
            f"自動抽出した通路マスク: 有効画素数={int(route_mask.sum())}/{route_mask.size} "
            f"(半径閾値={AUTO_ROUTE_MAX_HALF_WIDTH_PX:.1f}px, 膨張={AUTO_ROUTE_DILATION_PX:.1f}px)"
        )
        if AUTO_ROUTE_CENTERLINE_ENABLED:
            centerline_points = extract_ordered_centerline(
                route_mask, AUTO_ROUTE_CENTERLINE_SIMPLIFY_PX
            )
            if len(centerline_points) >= 2:
                ROUTE_POINTS = centerline_points
                logging.info(
                    f"自動抽出した通路中心線: {len(ROUTE_POINTS)}点 "
                    f"(簡略化閾値={AUTO_ROUTE_CENTERLINE_SIMPLIFY_PX:.1f}px) "
                    f"→ 曲がり角連動の方位補正が有効になります"
                )
            else:
                logging.warning(
                    "route_source=autoで通路中心線を抽出できませんでした。"
                    "方位補正は無効のままです(従来通り空間マスクのみ)。"
                )
        if MULTI_HYPOTHESIS_ROUTING_ENABLED:
            skeleton_graph = build_skeleton_graph(route_mask)
            simplified_graph = simplify_skeleton_graph(skeleton_graph)
            route_topology = build_route_graph_topology(
                simplified_graph, MULTI_HYPOTHESIS_ROUTING_SIMPLIFY_PX
            )
            if route_topology["edges"]:
                n_junction = sum(1 for n in simplified_graph["nodes"] if n["kind"] == "junction")
                n_endpoint = sum(1 for n in simplified_graph["nodes"] if n["kind"] == "endpoint")
                logging.info(
                    f"複数経路仮説PF用の通路グラフ: 交差点={n_junction}, 端点={n_endpoint}, "
                    f"エッジ={len(route_topology['edges'])} "
                    f"(簡略化閾値={MULTI_HYPOTHESIS_ROUTING_SIMPLIFY_PX:.1f}px) "
                    f"→ 粒子ごとのグラフ分岐PFが有効になります"
                )
            else:
                logging.warning(
                    "複数経路仮説PF: 通路グラフのエッジが0本のため無効化します"
                    "(従来通り空間マスクのみで実行します)。"
                )
                route_topology = None
    else:
        route_mask = build_route_mask(binary.shape)

    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 8))

    redraw_all_paths()

    if args.no_watch:
        if not args.no_show:
            plt.ioff()
            plt.show()
    else:
        if not HAS_WATCHDOG:
            raise ImportError("watchdog がインストールされていないため監視モードを開始できません。")

        observer = Observer()
        observer.schedule(CSVHandler(), str(data_dir), recursive=False)
        observer.start()

        logging.info("===================================")
        logging.info("Monitoring started...")
        logging.info(str(data_dir))
        logging.info("===================================")

        try:
            while True:
                plt.pause(1)
                if redraw_requested.is_set():
                    redraw_requested.clear()
                    time.sleep(2)
                    redraw_all_paths()

        except KeyboardInterrupt:
            observer.stop()

        observer.join()


if __name__ == "__main__":
    main()
