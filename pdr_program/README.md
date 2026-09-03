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
  あったものを2026-08-16に分離)。

## 検証・診断ツール(`pdr_pf_improved.py`を呼び出す/流用する)

- `check_sensor_quality.py` — CSVごとの生センサーデータ品質を診断
- `quick_check.py` — **計測当日その場で使う健全性判定**(歩数・歩調・平均歩幅・推定総距離・方位の正味回転・地点マークの整合)。撮り直しの要否をその場で決めるためのもの
- `pick_landmarks.py` — 正解位置データ作成Phase 0の旧方式。**現在は使わない**(kanri_4fの二値地図は廊下沿いに開口が無くクリックできないため。詳細は`../memo/ground_truth.md`)
- `verify_route_graph.py` — 通路グラフ(`build_skeleton_graph`/`simplify_skeleton_graph`)の可視化確認
- `compare_route_source.py` — 経路制約モード・経路帯生成元の比較実験(サブプロセスで`pdr_pf_improved.py`を複数回実行)
- `sensitivity_uncertainty_particles.py` — 不確実性適応粒子数パラメータの感度分析

## 正解位置・精度評価ツール(独立、`pdr_pf_improved.py`を直接importしない)

- `make_route_landmarks.py` — 正解位置データ作成Phase 0(現行方式)。目印のマスター表と経路定義から、経路別landmarks CSVと「押す順番シート」を生成
- `build_ground_truth.py` — 正解位置データ作成Phase 2(waypoints×landmarksをseq結合)
- `evaluate_accuracy.py` — 推定軌跡と正解位置からRMSE等を計算

## 地図準備ツール

- `map_binarizer.py` / `map_processing.py` — 建築平面図から2値地図を作る
- `measure_map_scale.py` — 地図上の既知区間をクリックしてscale_px_per_mを求める
- `Lmap.py` — L字合成地図(`L_map.png`)を描画するツール

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
