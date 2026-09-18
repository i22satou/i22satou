# pdr_program/

PDR(歩行者自律測位)+移動様態適応型パーティクルフィルタの現行プログラム一式。
2026-08-16に、リポジトリ直下(`i22satou/`)に散らばっていた「現在使っているプログラム」
と「過去の試作・参照用スクリプト」を区別しやすくするため、このフォルダへまとめた
(`git mv`で移動、履歴は保持。旧配置場所は`../削除候補/`とは別で、単なる整理)。

## 本体

- **`pdr_pf_improved.py`** — メインプログラム(エントリーポイント)。PDR+PFの全処理
  (センサー処理・移動様態判定・パーティクルフィルタ・可視化・実験基盤)はここに
  集約されている。実行例:
  ```
  python pdr_pf_improved.py --map-config map_configs/kanri_4f.json --no-watch --no-show --seed 42
  ```
- **`pdr_route_graph.py`** — `pdr_pf_improved.py`から分割したモジュール(2026-08-16、
  ファイル肥大化対策)。経路帯マスク抽出・通路グラフ化(骨格化・ノード整理・
  トポロジー変換)関連の関数(`extract_auto_route_mask`, `extract_ordered_centerline`,
  `build_skeleton_graph`, `simplify_skeleton_graph`, `build_route_graph_topology`,
  `nearest_edge_position`等)に加え、交差点分岐選択の方位重み付け計算
  (`edge_entry_heading`, `choose_branch_by_heading`。2026-08-30追加)がここにある。
  `pdr_pf_improved.py`冒頭で
  `from pdr_route_graph import (...)`により再importしているため、呼び出し側からは
  分割を意識せず`pdrmod.build_skeleton_graph(...)`のように呼べる。
- **`CHANGELOG.md`** — `pdr_pf_improved.py`の変更履歴(旧: ファイル冒頭コメントに
  あったものを2026-08-16に分離)。2026-09-02以前の項目は`CHANGELOG_archive.md`。

## 実行のしかた

どのスクリプトも`pdr_program/`をカレントディレクトリにして実行する
(例: `python evaluation/compare_route_source.py --seeds 1 7 42`)。
サブフォルダのスクリプトは、本体・`map_configs/`・`results/`を1つ上の`pdr_program/`から探す。
結果の保存先は、どのスクリプトも`pdr_program/results/`で共通(2026-09-18にフォルダを整理)。

## `evaluation/` — 検証・実験(結果を出すもの)

- `compare_route_source.py` — 経路制約モード・経路帯生成元の比較実験(サブプロセスで`pdr_pf_improved.py`を複数回実行)
- `sensitivity_uncertainty_particles.py` — 不確実性適応粒子数パラメータの感度分析
- `sensitivity_branch_likelihood.py` — 分岐仮説の選別尤度σの感度分析
- `calibrate_step_length.py` — 歩幅校正ゲイン(`step_length_calibration_gain`)の再同定。
  計測一覧表の`calib`の行(距離既知の直線歩行)から全CSV共通のゲインを1つ求め、ファイル別の表・
  leave-one-out・歩調と歩幅の図を`results/`へ出す。`--self-test`あり。JSONへの反映は手動
- `measurement_list.py` — 計測一覧表(下記)を読む共通部品
- `build_ground_truth.py` — 正解位置データ作成Phase 2(waypoints×landmarksをseq結合)
- `evaluate_accuracy.py` — 推定軌跡と正解位置からRMSE等を計算
- `verify_route_graph.py` — 通路グラフ(`build_skeleton_graph`/`simplify_skeleton_graph`)の可視化確認
- `check_sensor_quality.py` — CSVごとの生センサーデータ品質を診断

## `tools/` — 準備・計測日に使う道具

- `quick_check.py` — **計測当日その場で使う健全性判定**(歩数・歩調・平均歩幅・推定総距離・方位の正味回転・地点マークの整合)。撮り直しの要否をその場で決めるためのもの
- `make_route_landmarks.py` — 正解位置データ作成Phase 0(現行方式)。目印のマスター表と経路定義から、経路別landmarks CSVと「押す順番シート」を生成
- `pick_landmarks.py` — 正解位置データ作成Phase 0の旧方式。**現在は使わない**(kanri_4fの二値地図は廊下沿いに開口が無くクリックできないため。詳細は`../memo/ground_truth.md`)
- `map_binarizer.py` / `map_processing.py` — 建築平面図から2値地図を作る
- `measure_map_scale.py` — 地図上の既知区間をクリックしてscale_px_per_mを求める
- `Lmap.py` — L字合成地図(`L_map.png`)を描画するツール(カレントディレクトリへ出力)

## 比較方式と実行オプション(卒論第7章)

評価ハーネス(`evaluation/run_evaluation.py`、作成予定)はこの表どおりに実行する。
方式の定義は卒論雛形6.4、条件を決めた経緯は`../memo/comparison_methods.md`。

**全方式に共通**: `--map-config map_configs/kanri_4f.json --no-watch --no-show --seed <1 7 42 100 777 2024>
--save-trajectory-csv --trajectory-dir <条件ごとのフォルダ>`。方位の設定(`--heading-source`・
`--heading-calibration-mode`)は全方式で同じにする。`--initial-heading-deg`は経路の向きで決める
(east系は0、west_reverseは180)。

| 方式 | 卒論 | 追加するオプション |
|---|---|---|
| A: PDRのみ | 7.1 | どれか1つのPF実行に`--save-pdr-trajectory-csv`を付ける。乱数を使わないので1通り |
| B: 固定粒子数PF | 7.2 | `--route-constraint-mode none --pf-mode fixed`(600粒子・0.7px・15度、不確実性適応は自動でOFF) |
| C: 移動様態適応PF | 7.3 | `--route-constraint-mode none --no-uncertainty-adaptive-particles` |
| E: 提案方式 | 7.4 | `--route-source auto --route-constraint-mode enforce --uncertainty-adaptive-particles` |

提案方式の変種(既定OFFの機能を1つずつONにする。提案方式の本体には含めない):

| 変種 | 提案方式に足すオプション |
|---|---|
| 複数経路仮説 | `--multi-hypothesis-routing` |
| 複数経路仮説+分岐選別尤度 | `--multi-hypothesis-routing --multi-hypothesis-branch-likelihood-sigma-deg 90` |
| 広い部屋の除外 | `--auto-route-exclude-wide-rooms` |
| 中心線による方位補正 | `--auto-route-centerline`(複数経路仮説とは併用不可) |

- 方式BとCはどちらも連続壁尤度(`dist_map`)を使う。違いは様態適応の有無だけ。
- 方式D(手動経路route_points)は比較に含めない。今の`route_points`は0805の経路用で、新しい経路とは合わない。
- 分岐選別尤度のσ=90は、既存3本(調整用)で決めた値。`--multi-hypothesis-branch-heading-sigma-deg`は既定のまま(併用すると二重計上)。

## 計測一覧表(計測日に1枚書く)

校正用と評価用の記録を1枚の表にまとめる。UTF-8でもExcelの既定保存(Shift-JIS)でもよい。
詳細は`evaluation/measurement_list.py`の冒頭。

```
file,purpose,route,distance_m,speed,use,memo
calib/pdr_log_0925_1010.csv,calib,,30.0,slow,1,
pdr_log_0925_1030.csv,eval,east_std,,,1,
```

- `purpose`: `calib`(歩幅校正用の直線歩行、`distance_m`と`speed`=slow/normal/fastを書く)/
  `eval`(比較実験用、`route`=`ground_truth/kanri_4f_landmarks_<route>.csv`の`<route>`を書く)
- `use`: 撮り直した記録は消さずに`0`にする
- **校正用CSVはdata_dir直下に置かず、サブフォルダ(例: `calib/`)へ入れる**。直下に置くと、
  本体が開始位置の登録を求めてクリック待ちで止まる
- 例: `ground_truth/measurement_list_0805.csv`(既存3本。2.10の再現確認用)

## データ

- `map_configs/` — 地図設定JSON(`kanri_4f.json`が主対象、`l_map.json`は技術確認用)
- `kanri_4f_binary_final3.png` / `kanri_4f_preview_final3.png` / `L_map.png` — 地図画像
- `start_positions.csv` — CSVごとの既知開始位置(自動生成・追記される)
- `results/` — 実行結果PNG・診断CSVの保存先(自動生成される)

## ここに含めていないもの

- `../pdr_pf_clickstart.py`(旧版)、`../test7.py`・`../L.png`(初期のL字検証、
  現行パイプラインからは独立)は、現行プログラムと直接の依存関係がないため
  リポジトリ直下に残している(詳細は`../memo/file_cleanup.md`参照)。
- `../削除候補/`は不要と判断した過去のプロトタイプ・実験結果の移動先(削除はしていない)。
- `../memo/`・`../CLAUDE_MEMO.md`・`../CLAUDE_MEMO.txt`は調査記録・卒論下書きで、
  複数トピックにまたがるためリポジトリ直下に残している。
