# ============================================================================
# pdr_pf_improved.py
#
# 【変更履歴】
# - 2026-08-15: [本研究独自] 経路帯マスクを手動route_pointsに頼らず、二値地図から
#               自動抽出できるようにした(--route-source {manual,auto}、既定はmanual
#               で従来通り)。extract_auto_route_mask()が「壁までの距離が
#               auto_route_max_half_width_px(既定20px)以下の移動可能領域」を通路候補とし、
#               そのうち最大連結成分を通路網として採用する(広い部屋は距離が大きく候補から
#               自然に外れる)。座標を一切与えずにkanri_4f.jpgの実廊下形状を高精度に再現
#               できることを可視化で確認済み(詳細はCLAUDE_MEMO.md)。route_source=autoの
#               場合はJSONのroute_pointsを意図的に無視する(手動座標ゼロにするため)ので、
#               route_guidance_enabled()系の曲がり角連動方位補正は現状autoでは働かない
#               (空間的な経路帯制約のみ。今後の課題)。
# - 2026-08-15: 未使用コードを削除しファイル総行数を縮小。
#               (1) 未使用import(dataclass)を削除。
#               (2) 呼び出し箇所が一つもなかったestimate_step_length_px()
#                   (Weinberg法の予備実装)とWEINBERG_K定数を削除。
#               (3) START_X/START_Y(JSON任意設定"start"由来)を削除。実際の
#                   PF開始位置はstart_positions.csv/クリック選択で決まっており
#                   一切影響していなかった上、ログに実態と異なる開始位置を
#                   出力していたため。
# - 2026-08-15: 地図設定JSONに任意設定"exclude_csv"を追加。別の図(L字経路用など)で
#               使用中のCSVをファイル名指定でこのJSONの実行対象から除外できるように
#               した(redraw_all_paths()のファイル一覧構築時にフィルタ)。
# - 2026-08-15: CLAUDE.mdで指摘されていたpdr_pf_clickstart.pyとの残る差分を解消。
#               (1) --save未指定時にPNGが1枚も保存されない問題を修正し、
#                   pdr_pf_clickstart.py同様resultsディレクトリへ実行日時・
#                   経路制約モード・乱数シードを含む名前で自動保存するようにした。
#               (2) apply_map_config()がconfig.get(key, 旧デフォルト値)で
#                   設定漏れを黙って握り潰していた(ヘッダーの「設定漏れはエラー」
#                   という記述と矛盾)問題を、require_config_value()による必須
#                   チェックへ統一して解消(pdr_pf_clickstart.py同様の挙動)。
#               (3) --heading-source androidをyaw_deg列の無いCSVに使った際の
#                   ValueErrorがredraw_all_paths()のループ外まで伝播し、
#                   以降のCSVを1枚も処理できずバッチ全体が止まっていた問題を、
#                   該当ファイルだけスキップするtry/exceptで解消。
#               (4) [SmartPDR]/[先行研究:移動様態PF]/[本研究独自]の出典タグと、
#                   研究的位置づけの説明コメントをpdr_pf_clickstart.pyから移植。
# - 2026-08-15: route_constraint_mode=enforceで全粒子の重みが0になった際の
#               自己リカバリ処理に経路マスク条件を追加。従来はbinary_for_pf
#               (物理壁)のみを条件に再配置していたため、全滅からの回復時に
#               粒子群が経路コリドー外(隣室など)へ漏れ出す場合があった
#               (is_in_wall()自体は経路を見ない設計のまま据え置き、
#               診断指標valid_ratio_history/route_ratio_historyの分離も維持)。
# - 2026-08-05: CSVごとの既知開始位置を地図クリックで登録し、
#               start_positions.csvから再利用する機能を追加。
# - 2026-08-06: JSONで管理する地図・PF設定値をコード先頭の固定値から削除し、
#               設定漏れは起動時にエラーとして検出するように整理。
# - 2026-08-06: 経路マスク外を移動不可領域として扱えるようにし、
#               部屋側へ推定経路が流れる表示を抑制。
#               設定経路の破線描画も削除。
# - 2026-08-06: 手動L字経路への依存を比較条件として分離するため、
#               route_constraint_modeのnone、prefer、enforceを追加。
# - 2026-08-06: 計測開始時のセンサ方位を地図座標系へ合わせる
#               初期方位補正を追加。
# - 2026-08-06: 方位推定方法として、従来のジャイロ・Madgwick方式と、
#               AndroidのTYPE_ROTATION_VECTORから記録したyaw_degを
#               選択できる機能を追加。
# - 2026-08-06: クリックした開始位置に最も近いroute_pointsの線分を選び、
#               その線分を最初の経路線分として使用する処理を追加。
# - 2026-08-06: 開始位置、経路制約モード、方位推定方法、
#               初期方位、経路重みなどをキャッシュ判定へ追加。
# - 2026-08-06: 有効粒子率、経路内粒子率、Neff、位置分散を記録し、
#               CSVごとのPF診断値としてログ出力する機能を追加。
# - 2026-08-06: 結果画像の凡例とタイトルを整理し、
#               経路制約モードと方位推定方法を画像上へ表示。
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
# 【経路線分切替】
# - 従来は、すべてのCSVでroute_segment_indexを0から開始していた。
# - 修正後は、クリックした開始位置とroute_pointsの各線分との距離を計算し、
#   開始位置に最も近い経路線分を初期線分として選択する。
# - 曲がり検出をturn_pendingとして保持し、
#   PF推定位置が設定曲がり角付近へ到達した場合だけ次の線分へ進める。
# - 判定順を「センサー方位、移動様態判定、経路線分切替、
#   方位補正、PF更新」の順番とする。
# - 最近傍線分を毎歩選び直す方式は使用せず、
#   CSVごとに原則として単調増加するroute_segment_indexを保持する。
# - 曲がり終了後も切り替え後の経路線分を維持し、
#   前の通路方向へ戻らないようにする。
# - 経路制約モードがnoneの場合は、経路線分による方位補正を使用しない。
#
# 【移動様態判定】
# - 歩行者の移動様態をSTOPPED、STRAIGHT、TURNINGの3状態で判定する。
# - 曲がり判定には、一定時間内の方位変化とヨーレートを併用する。
# - ヨーレートの瞬間的な最大値ではなく、75パーセンタイルを使用し、
#   手ぶれなどの外れ値の影響を抑える。
# - 曲がり開始条件は、方位変化とヨーレートの両方が
#   閾値を超えた場合に成立するAND条件とする。
# - 曲がり終了時は、方位変化とヨーレートの両方が
#   終了閾値より小さいことを要求する。
# - TURNING判定にはヒステリシスを導入し、
#   STRAIGHTとTURNINGの頻繁な状態変化を防止する。
#
# 【移動様態適応型パーティクルフィルタ】
# - 直進時、曲がり時、滞留時でパーティクル数を動的に変更する。
# - 現在のJSON設定例は、直進250個、曲がり600個、滞留100個である。
# - 直進時、曲がり時、滞留時で歩幅ノイズと方位ノイズを変更する。
# - 直進時は探索範囲を狭くし、曲がり時は方位探索範囲を広げる。
# - パーティクル数を変更する場合は、現在の重みに従って再サンプリングする。
# - パーティクル数を増加させる場合は、複製粒子へ微小な位置摂動を加える。
# - リサンプリング、壁衝突判定、全滅復帰処理を
#   可変パーティクル数へ対応させる。
# - CSVごとに、各移動様態の更新回数、平均粒子数、
#   最小粒子数、最大粒子数、全滅回数をログへ出力する。
#
# 【PF診断値】
# - 各PF更新において、壁へ衝突しなかった粒子の割合を
#   有効粒子率として記録する。
# - route_pointsから作成した経路マスク内にある粒子の割合を
#   経路内粒子率として記録する。
# - 正規化後の粒子重みから実効サンプルサイズNeffを計算する。
# - 粒子のx座標分散とy座標分散の合計を位置分散として記録する。
# - CSVごとの処理終了時に、有効粒子率、経路内粒子率、
#   平均Neff、平均位置分散をログへ出力する。
# - 有効粒子率が低い場合は、壁へ衝突する粒子が多い可能性がある。
# - Neffが低い場合は、少数の粒子へ重みが集中している可能性がある。
# - 位置分散が大きい場合は、推定位置の不確実性が高い可能性がある。
# - noneモードでは経路マスクが全領域を表すため、
#   経路内粒子率は原則として1に近くなる。
#
# 【キャッシュ判定】
# - 入力CSVの更新時刻とファイルサイズに加えて、
#   次の条件が一致した場合だけ計算結果を再利用する。
#     ・クリック開始位置
#     ・経路制約モード
#     ・方位推定方法
#     ・地図上の初期方位
#     ・初期方位校正に使用するサンプル数
#     ・route_points
#     ・経路方位補正の重み
#     ・経路外粒子の重み
# - 開始位置や実行条件を変更した後に、
#  以前の条件で計算した結果が誤って再利用されることを防止する。
#
# 【結果グラフ】
# - 各CSVの推定軌跡を異なる色で表示する。
# - 開始位置は、対応する軌跡と同じ色の点で表示する。
# - 開始位置の点には白い縁を付け、軌跡との区別をしやすくする。
# - 凡例はCSV名ごとに1項目とし、軌跡と開始点の重複表示を防ぐ。
# - グラフタイトルへ経路制約モードと方位推定方法を表示する。
# - 保存された画像だけを見ても実行条件を判別できるようにする。
#
# 【test11_gemini.pyからの主な変更点】
# 1. 歩行者の移動様態をSTOPPED、STRAIGHT、TURNINGの3状態で判定する。
# 2. 移動様態に応じてパーティクル数を動的に変更する。
# 3. 移動様態に応じて歩幅ノイズと方位ノイズを変更する。
# 4. 曲がり判定に方位変化とヨーレートを併用する。
# 5. TURNING判定にヒステリシスを導入する。
# 6. パーティクル増減時に重みに従って再サンプリングする。
# 7. パーティクル増加時に微小な位置摂動を加える。
# 8. リサンプリングと全滅復帰を可変パーティクル数へ対応させる。
# 9. CSVごとに移動様態、粒子数、全滅回数、PF診断値を出力する。
# 10. CSVごとの既知開始位置を地図クリックで登録して再利用する。
# 11. 初期方位を地図座標系へ合わせる。
# 12. ジャイロ方位とAndroid方位を切り替えて比較できるようにする。
# 13. 手動経路の使用方法をnone、prefer、enforceへ分離する。
#
# 【主な実行例】
#
# 経路制約なし、ジャイロ方位、右方向から開始:
# python pdr_pf_improved.py \
#   --route-constraint-mode none \
#   --heading-source gyro \
#   --initial-heading-deg 0 \
#   --no-watch \
#   --seed 42 \
#   --save result_none_gyro.png
#
# 経路制約なし、Android方位、右方向から開始:
# python pdr_pf_improved.py \
#   --route-constraint-mode none \
#   --heading-source android \
#   --initial-heading-deg 0 \
#   --heading-calibration-samples 30 \
#   --no-watch \
#   --seed 42 \
#   --save result_none_android.png
#
# 経路優先、ジャイロ方位、右方向から開始:
# python pdr_pf_improved.py \
#   --route-constraint-mode prefer \
#   --heading-source gyro \
#   --initial-heading-deg 0 \
#   --no-watch \
#   --seed 42 \
#   --save result_prefer_gyro.png
#
# 経路強制、ジャイロ方位、右方向から開始:
# python pdr_pf_improved.py \
#   --route-constraint-mode enforce \
#   --heading-source gyro \
#   --initial-heading-deg 0 \
#   --no-watch \
#   --seed 42 \
#   --save result_enforce_gyro.png
#
# 【注意】
# - 論文では直進10個、屈折20個としているが、
#   本プログラムでは複雑な地図に対応するため、
#   同程度の比率を保ちながらJSONで直進250個、曲がり600個に設定している。
# - 各パラメータはmap_configs/kanri_4f.jsonの
#   adaptive_pfセクションで変更できる。
# - --heading-source androidを使用する場合は、
#   入力CSVにyaw_deg列が必要である。
# - 初期方位を安定して取得するため、AndroidアプリでSTARTを押した後は、
#   短時間静止してから歩き始めることが望ましい。
# - 現在の--initial-heading-degは1回の実行で全CSVへ共通に適用される。
# - 開始方向が異なるCSVは、開始方向ごとに分けて実行する必要がある。
# - 現在の二値地図では、廊下と部屋が同じ移動可能領域として
#   扱われる場合がある。
# - noneモードでは、部屋への誤進入や誤った通路選択が
#   発生する可能性がある。
# - preferとenforceは手動route_pointsを使用するため、
#   本研究の最終提案方式ではなく比較方式として扱う。
# - 軌跡画像が正解経路に近く見えることだけでは、
#   位置推定精度の改善を証明できない。
# - 平均位置誤差、RMSE、最終位置誤差、曲がり位置誤差を求めるには、
#   正解位置または正解経路データが別途必要である。
# - 今後は、廊下・部屋・ドアの領域分類、通路中心線の自動抽出、
#   手動正解経路を使用しない複数経路推定を追加する必要がある。
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
TARGET_DISTANCE_PX = None
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

HCOR_THR  = np.deg2rad(5)
HMAG_THR  = np.deg2rad(2)
W_PREV    = 2.0
W_MAG     = 1.0
W_GYRO    = 2.0

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
    def __init__(self, n_particles, start_x, start_y, route_mask, binary_for_pf, dist_map, params):
        self.n_particles = n_particles
        self.start_x = start_x
        self.start_y = start_y
        self.route_mask = route_mask
        self.binary_for_pf = binary_for_pf
        self.dist_map = dist_map
        self.h, self.w = binary_for_pf.shape
        self.params = params
        self.reset()

    def reset(self):
        """パーティクルと状態の初期化"""
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
        """移動様態に応じて粒子数とサンプリング分散を更新する。"""
        behavior_params = behavior_parameters(behavior)
        self.resize_particle_set(
            behavior_params["n_particles"],
            jitter_px=PARTICLE_RESIZE_JITTER_PX,
        )
        self.params["sigma_step"] = behavior_params["sigma_step"]
        self.params["sigma_angle"] = behavior_params["sigma_angle"]

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

        # 1. 状態遷移（移動様態ごとのノイズ付与）
        noise_dist = np.random.normal(0, self.params['sigma_step'], self.n_particles)
        noise_angle = np.random.normal(0, self.params['sigma_angle'], self.n_particles)
        move = step_px + noise_dist
        p_angle = step_heading + noise_angle

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
        
        # 方位ズレの重み
        particle_heading_error = angle_diff(p_angle, step_heading)
        weights *= np.exp(
            -(particle_heading_error ** 2) /
            (2 * self.params['sigma_angle'] ** 2)
        )
        
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
            ref_x, ref_y = np.mean(self.particles, axis=0)
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
        "--seed",
        type=int,
        default=None,
        help="乱数シード。結果を再現したいときに指定します。",
    )
    parser.add_argument(
        "--target-distance-px",
        type=float,
        default=None,
        help="歩幅を校正する目標移動距離(px)。指定するとJSONの値を上書きします。",
    )
    parser.add_argument(
        "--no-step-calibration",
        action="store_true",
        help="SmartPDR歩幅の総距離校正を行いません。",
    )
    parser.add_argument(
        "--step-gain",
        type=float,
        default=None,
        help="校正後の歩幅に掛ける追加倍率。指定するとJSONの値を上書きします。",
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
            "(本研究独自、曲がり角の方位補正はまだ対応せず空間マスクのみ)。"
        ),
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
        help="開始方位の基準値を求める先頭サンプル数。",
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
    global M_TO_PIXEL, TARGET_DISTANCE_PX, DEFAULT_STEP_GAIN
    global PF_EROSION_RADIUS_PX
    global WALL_WEIGHT_SIGMA, WALL_WEIGHT_FLOOR, OFF_ROUTE_WEIGHT
    global ROUTE_WIDTH_PX, ROUTE_HEADING_WEIGHT, ROUTE_CORNER_THRESHOLD_PX
    global ROUTE_CONSTRAINT_MODE, ROUTE_POINTS, GYRO_UNIT, EXCLUDED_CSV_NAMES
    global ROUTE_SOURCE, AUTO_ROUTE_MAX_HALF_WIDTH_PX
    global N_PARTICLES_STRAIGHT, N_PARTICLES_TURNING, N_PARTICLES_STOPPED
    global SIGMA_STEP_STRAIGHT, SIGMA_STEP_TURNING, SIGMA_STEP_STOPPED
    global SIGMA_ANGLE_STRAIGHT, SIGMA_ANGLE_TURNING, SIGMA_ANGLE_STOPPED
    global BEHAVIOR_WINDOW_SEC, TURN_ENTER_THRESHOLD, TURN_EXIT_THRESHOLD
    global TURN_YAW_RATE_THRESHOLD, PARTICLE_RESIZE_JITTER_PX

    config_dir = config_path.parent
    config_name = str(config_path)
    M_TO_PIXEL = float(require_config_value(config, "scale_px_per_m", config_name))
    target = config.get("target_distance_px")
    TARGET_DISTANCE_PX = None if target is None else float(target)
    DEFAULT_STEP_GAIN = float(require_config_value(config, "step_gain", config_name))
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
    if ROUTE_SOURCE == "auto":
        ROUTE_POINTS = []
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
    if args.target_distance_px is None:
        args.target_distance_px = TARGET_DISTANCE_PX
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
    logging.info(
        f"経路優先: {len(ROUTE_POINTS)}点, 幅={ROUTE_WIDTH_PX:.1f}px, "
        f"曲がり角判定距離={ROUTE_CORNER_THRESHOLD_PX:.1f}px, "
        f"経路外重み={OFF_ROUTE_WEIGHT:.2f}"
    )
    logging.info("総距離校正: " + ("無効" if args.target_distance_px is None else f"{args.target_distance_px:.1f}px"))


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


# [SmartPDR] ピーク・谷・傾き条件を用いたステップ検出(原論文のステップ検出手法)。
def detect_steps_smartpdr(step_acc: pd.Series):
    values = step_acc.to_numpy()
    peaks, _ = find_peaks(
        values,
        height=SMART_PEAK_THR,
        distance=STEP_MIN_INTERVAL,
    )

    valid_peaks = []
    valley_indices = []
    search_win = max(STEP_MIN_INTERVAL, 8)
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


def estimate_initial_sensor_heading(df, source, sample_count):
    """先頭の有限値から開始時センサ方位を円平均で求める。"""
    if source == "android":
        if "yaw_deg" not in df.columns:
            raise ValueError("--heading-source android にはCSVのyaw_deg列が必要です。")
        values = np.deg2rad(pd.to_numeric(df["yaw_deg"], errors="coerce").to_numpy())
        values = values[np.isfinite(values)][:max(1, int(sample_count))]
        if len(values) == 0:
            raise ValueError("yaw_degに有効な値がありません。")
        return weighted_angle_mean(values, np.ones(len(values)))
    return None


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


def has_magnetometer(df):
    return {'mag_x', 'mag_y', 'mag_z'}.issubset(df.columns)


def get_mag_heading(row):
    norm = max(np.sqrt(row.acc_x**2 + row.acc_y**2 + row.acc_z**2), 1e-6)
    ax, ay, az = row.acc_x / norm, row.acc_y / norm, row.acc_z / norm
    pitch = np.arctan2(ax, np.sqrt(ay**2 + az**2))
    roll = np.arctan2(ay, np.sqrt(ax**2 + az**2))

    mx = row.mag_x * np.cos(pitch) + row.mag_z * np.sin(pitch)
    my = (
        row.mag_x * np.sin(roll) * np.sin(pitch)
        + row.mag_y * np.cos(roll)
        - row.mag_z * np.sin(roll) * np.cos(pitch)
    )
    return normalize_angle(np.arctan2(-my, mx))


def fuse_heading(prev_heading, mag_heading, gyro_heading, prev_mag_heading):
    if mag_heading is None or prev_mag_heading is None:
        return gyro_heading

    h_cor = abs(angle_diff(mag_heading, gyro_heading))
    h_mag = abs(angle_diff(mag_heading, prev_mag_heading))

    if h_cor <= HCOR_THR and h_mag <= HMAG_THR:
        return weighted_angle_mean(
            np.array([prev_heading, mag_heading, gyro_heading]),
            np.array([W_PREV, W_MAG, W_GYRO]),
        )
    if h_cor <= HCOR_THR and h_mag > HMAG_THR:
        return weighted_angle_mean(
            np.array([mag_heading, gyro_heading]),
            np.array([W_MAG, W_GYRO]),
        )
    if h_cor > HCOR_THR and h_mag <= HMAG_THR:
        return prev_heading

    return weighted_angle_mean(
        np.array([prev_heading, gyro_heading]),
        np.array([W_PREV, W_GYRO]),
    )


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


# [本研究独自] 二値地図から通路領域を自動抽出し、経路帯マスクを作る(route_source=auto)。
# build_route_mask()が手動route_pointsを太らせるのに対し、こちらは座標を一切与えず
# 「壁までの距離がmax_half_width_px以下の移動可能領域」を通路候補とみなし、そのうち
# 最大の連結成分を通路網として採用する(広い部屋は距離が大きく候補から外れるため、
# 廊下だけが概ね残る)。曲がり角に連動した方位補正(route_guidance_enabled系)は
# 順序付きroute_pointsが前提のため、autoではまだ対応しない(空間的な経路帯制約のみ)。
def extract_auto_route_mask(binary_for_pf_local, max_half_width_px):
    free = binary_for_pf_local == 255
    dist = ndimage.distance_transform_edt(free)
    corridor_candidate = free & (dist <= max_half_width_px)
    labeled, n = ndimage.label(corridor_candidate, structure=np.ones((3, 3)))
    if n == 0:
        logging.warning("route_source=autoで通路領域が抽出できませんでした。全域を通路として扱います。")
        return np.ones(free.shape, dtype=bool)
    sizes = ndimage.sum(corridor_candidate, labeled, range(1, n + 1))
    largest_label = int(np.argmax(sizes)) + 1
    return labeled == largest_label


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
            step_indices, valley_indices = detect_steps_smartpdr(df['step_acc'])
            valley_by_step = dict(zip(step_indices, valley_indices))

            logging.info(f"[{file_name}] 総行数: {len(df)}  検出ステップ数: {len(step_indices)}")

            if len(step_indices) > 0:
                raw_step_lengths = [
                    estimate_smartpdr_step_length_px(df['step_acc'], peak, valley)
                    for peak, valley in zip(step_indices, valley_indices)
                ]
                raw_total_dist = sum(raw_step_lengths)
                step_scale = 1.0
                if (not args.no_step_calibration and args.target_distance_px is not None and raw_total_dist > 0):
                    step_scale = args.target_distance_px / raw_total_dist

                step_lengths = [length * step_scale * args.step_gain for length in raw_step_lengths]
                step_length_by_step = dict(zip(step_indices, step_lengths))
                total_dist = sum(step_lengths)
                logging.info(f"  SmartPDR歩幅推定 (px): 平均={np.mean(step_lengths):.1f}  "
                      f"≈ 平均{np.mean(step_lengths)/M_TO_PIXEL:.2f}m/歩")
                target_text = "未指定" if args.target_distance_px is None else f"{args.target_distance_px:.0f}px"
                logging.info(f"  推定総移動距離: {total_dist:.0f}px  "
                      f"(校正前: {raw_total_dist:.0f}px, scale={step_scale:.2f}, "
                      f"gain={args.step_gain:.2f}, 目標: {target_text})")
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
                params=pf_params
            )

            gyro_angle          = 0.0
            heading             = 0.0
            prev_mag_heading    = None
            heading_history     = np.zeros(len(df))
            yaw_rate_history    = np.zeros(len(df))
            step_set            = set(step_indices)
            mag_enabled         = has_magnetometer(df)
            current_behavior    = MoveBehavior.STRAIGHT
            behavior_history    = []
            particle_count_history = []
            # 曲がり検出はturn_pendingとして保持し、設定曲がり角へ
            # 到達した時だけ次の線分へ進む。
            route_segment_index = nearest_route_segment_index(
                file_start_x,
                file_start_y,
            )
            turn_pending = False
            initial_map_heading = np.deg2rad(args.initial_heading_deg)
            try:
                # --heading-source androidをyaw_deg列の無い旧CSVへ適用した場合など、
                # この1ファイルだけの設定不備でバッチ全体が中断しないようにする。
                initial_sensor_heading = estimate_initial_sensor_heading(
                    df,
                    args.heading_source,
                    args.heading_calibration_samples,
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
                        if mag_enabled:
                            mag = np.array([
                                row['mag_x'],
                                row['mag_y'],
                                row['mag_z']
                            ])
                            Q[i] = madgwick.updateMARG(q=Q[i - 1], gyr=gyro, acc=acc, mag=mag)
                        else:
                            Q[i] = madgwick.updateIMU(q=Q[i - 1], gyr=gyro, acc=acc)

                        rot = R.from_quat([Q[i][1], Q[i][2], Q[i][3], Q[i][0]])
                        gyro_angle = normalize_angle(rot.as_euler('xyz')[2])
                    except Exception as e:
                        logging.debug("Madgwick failed: %s", e)
                        # fallback to simple yaw rate
                        gyro_angle = normalize_angle(gyro_angle * ANGLE_DECAY + yaw_rate * dt)
                else:
                    gyro_angle = normalize_angle(gyro_angle * ANGLE_DECAY + yaw_rate * dt)

                mag_heading = get_mag_heading(row) if mag_enabled else None
                fused_heading = normalize_angle(
                    fuse_heading(heading, mag_heading, gyro_angle, prev_mag_heading)
                )

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
                if mag_heading is not None:
                    prev_mag_heading = mag_heading

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

            estimated_positions = pf.estimated_positions
            extinction_count = pf.extinction_count

            # 計算結果をキャッシュ
            result_cache.set(file_path_str, {
                'estimated_positions': estimated_positions,
                'step_count': step_count,
                'extinction_count': extinction_count,
                'behavior_history': behavior_history,
                'particle_count_history': particle_count_history,
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

    end_overall = time.time()
    logging.info("\n適応型パーティクルフィルタを使用")
    logging.info(f"Overall time: {end_overall - start_overall:.2f} seconds")

    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    title = (
        "移動様態適応型PF + SmartPDR "
        f"（経路制約: {ROUTE_CONSTRAINT_MODE}/{ROUTE_SOURCE}, 方位: {args.heading_source}）"
    )
    if japanize_matplotlib is None:
        title = (
            "Behavior-adaptive PF + SmartPDR "
            f"(route={ROUTE_CONSTRAINT_MODE}/{ROUTE_SOURCE}, heading={args.heading_source})"
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
        auto_name = f"{timestamp}_route-{ROUTE_CONSTRAINT_MODE}-{ROUTE_SOURCE}_seed-{seed_text}.png"
        save_path = (RESULTS_DIR / auto_name).resolve()
    save_path.parent.mkdir(parents=True, exist_ok=True)
    fig.set_size_inches(10, 8, forward=True)
    fig.savefig(save_path, dpi=200, bbox_inches='tight')
    logging.info(f"結果画像を保存しました: {save_path}")


def main():
    global args, data_dir, img_path, binary, binary_for_pf, dist_map, h, w, route_mask, fig, ax

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
    if ROUTE_SOURCE == "auto":
        route_mask = extract_auto_route_mask(binary_for_pf, AUTO_ROUTE_MAX_HALF_WIDTH_PX)
        logging.info(
            f"自動抽出した通路マスク: 有効画素数={int(route_mask.sum())}/{route_mask.size} "
            f"(半径閾値={AUTO_ROUTE_MAX_HALF_WIDTH_PX:.1f}px)"
        )
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
