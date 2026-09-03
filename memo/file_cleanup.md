# リポジトリのファイル整理(削除候補フォルダへの移動・決着済み・圧縮版)

2026-08-15実施、2026-08-16に経緯の詳細を圧縮(圧縮ルールは
[CLAUDE_MEMO.md](../CLAUDE_MEMO.md)参照)。削除ではなく`削除候補/`(と
`削除候補/results/`)への移動のみ。**何も削除していない**、最終判断はユーザー。

---

### 2026-09-03: results/のPNGを代表5枚だけ残して整理(ユーザー依頼)

`pdr_program/results/`のPNGが123枚まで増えていたため、代表5枚を残し118枚を
`削除候補/results/`へ移動した(**削除はしていない**)。CSV17件は数値そのものの記録なので
全て`results/`に残した。

残した5枚と理由:

| ファイル | 理由 |
|---|---|
| `kanri_4f_route_graph_check.png` / `_simplified.png` | `verify_route_graph.py`が出力する通路グラフの検証図。タイムスタンプ無しで上書き更新される。卒論の図候補 |
| `20260902_221353_route-enforce-auto+unc_head-android-walking_seed-42.png` | 上流是正(2026-09-02)後の主提案条件。`results/`内の軌跡CSV3件と対応する |
| `20260903_131331_route-none-manual+unc_head-gyro-samples_seed-42.png` | 分岐仮説の選別尤度を実装する**前**の既定動作 |
| `20260903_144756_route-none-manual+unc_head-gyro-samples_seed-42.png` | 実装**後**の既定動作。1442でx=484.6, y=259.9と一致し、既定が変わっていないことの証拠 |

移動した118枚の大半は`sensitivity_branch_likelihood.py`のσスイープ出力で、
PNGのファイル名にσも対象CSVも入らない(命名がroute_source/route_constraint_mode/
heading/seedのみに基づく)ため1枚単位では識別できず、個々に価値がない。
数値は`results/20260903_*_branch_likelihood_sensitivity.csv`3件に残っている。
移動前に`memo/`・`CLAUDE.md`・`CLAUDE_MEMO.txt`・`CHANGELOG.md`・`進捗反映版メモ.txt`から
PNGファイル名で参照されているものを機械的に洗い出して照合済み(上記の訂正1件を実施)。

**PNGは実行のたびに増え続ける**(CLAUDE.mdの「実行ごとに結果PNGを必ず保存する」方針のため)。
比較実験やスイープを回した後は同じ要領で整理すること。

### 2026-08-16: プログラム一式を`pdr_program/`へ集約 + 累積した検証用PNG/CSVを削除候補へ移動

**ファイル整理**: `Trajectory.py`(初期の単純デッドレコニング試作、PF無し・磁気センサ
無し・ハードコードされた歩幅定数のみ、どこからも参照されておらず現行パイプラインとは
無関係と確認)を`削除候補/`へ移動。`results/`に溜まっていた検証・比較実行の副産物
(前日8/15の重複タイムスタンプ分、当日の複数シード探索・分割検証・統合検証で生成した
sensor_quality.csv/route-*.png/trajectory.csv、計27件)を`削除候補/results/`へ移動。
`memo/*.md`から明示的にファイル名で参照されているCSV/PNG(sensor_mystery.md,
uncertainty_particles.md, route_source_auto.md, このファイル自身が参照するもの)は
全て`results/`に残し、移動前に1件ずつ照合した。`__pycache__/`(コンパイル済み
バイトコードキャッシュ、実行時に自動再生成される)は削除。

**フォルダ再構成**: `pdr_pf_improved.py`・`pdr_route_graph.py`・検証ツール群
(`check_sensor_quality.py`/`pick_landmarks.py`/`verify_route_graph.py`/
`compare_route_source.py`/`sensitivity_uncertainty_particles.py`/
`evaluate_accuracy.py`/`build_ground_truth.py`)・地図準備ツール
(`map_binarizer.py`/`map_processing.py`/`measure_map_scale.py`/`Lmap.py`)・
データ(`map_configs/`/`kanri_4f_binary_final3.png`/`kanri_4f_preview_final3.png`/
`L_map.png`/`start_positions.csv`/`results/`)・`CHANGELOG.md`を、リポジトリ直下から
新設した`pdr_program/`フォルダへ`git mv`で移動した(履歴は保持)。全ファイルが
`Path(__file__).resolve().parent`基準の相対パス解決を使っており、依存関係にある
ファイル同士を常にセットで移動したため、コード側の変更は一切不要だった
(`map_configs/*.json`の`"map_image": "../..."`もmap_configs/と画像ファイルを
同時に移動したことでそのまま解決する)。移動後、`pdr_program/`内から主要スクリプト
全てを再実行し、診断値が移動前と完全一致することを確認済み(詳細は会話ログ参照)。

移動しなかったもの(理由): `pdr_pf_clickstart.py`(旧版、現行パイプラインと無関係)・
`test7.py`/`L.png`(L字検証用の独立した旧パイプライン、`Lmap.py`/`L_map.png`とは
違い現行の`map_configs/l_map.json`から直接参照されないため据え置き。詳細は下記
「残した理由が非自明なもの」参照)・`memo/`・`CLAUDE_MEMO.md`/`.txt`・`figures/`・
`CLAUDE.md`(複数トピック横断/セッション読み込みの都合上リポジトリ直下に維持)。
`CLAUDE.md`のパス表記は全て`pdr_program/`配下に更新済み。

### 2026-08-16: 感度分析(90回分)のPNG106枚を削除候補へ移動

[uncertainty_particles.md](uncertainty_particles.md)のOFAT感度分析で生成された
`results/20260816_*.png`(90回の本番実行+15回のスモークテスト+1回の動作確認、
計106枚)を`削除候補/results/`へ移動。各PNGのファイル名は
`route-enforce-auto+unc_seed-{seed}.png`で、どのパラメータ組(baseline/
neff_low_ratio=0.15等)の結果かはファイル名からは区別できず、タイムスタンプでしか
判別できない(pdr_pf_improved.pyのPNG命名がroute_source/route_constraint_mode/
seedのみに基づき、感度分析用の追加CLI引数を反映しないため)。感度分析の結論は
生データCSV(`results/20260816_110418_uncertainty_sensitivity.csv`)と
[uncertainty_particles.md](uncertainty_particles.md)の集計表に残っており、
個々のPNGに一意の価値はないため全て移動対象とした。代表画像が必要な場合は
2026-08-15時点の`20260815_230748_route-enforce-auto+unc_seed-42.png`
(auto-enforce-unc条件、seed=42)を参照していたが、**2026-09-03の整理で`削除候補/results/`へ
移動した**。そもそもこれは`target_distance_px`が有効だった時期の実行で現行結果と直接
比較できないため、代表画像としては下記2026-09-03の項で残した5枚を使うこと。

### 結論・現状

- ルート直下17個(test3〜test6.py, test8〜test11_gemini.py, 孤立プロトタイプ2本,
  Cross.py/cross_map.png, 旧世代の結果画像6枚)と、`results/`の69個中63個
  (compare_route_source.pyの中間実験PNGの山)を`削除候補/`へ移動。
  `results/`には最新の4条件比較PNG4枚+CSVサマリ2つだけを残した。
- 判断が非自明だった画像は実際に`Read`で開いて中身を確認してから残す/移すを決めた
  (ファイル名だけで判断していない)。

### 残した理由が非自明なもの(要記憶)

- **`L.png`/`test7.py`/`Lmap.py`/`L_map.png`**: 除外4CSV(0410/0414×2/0624)用の
  別図として現役使用中と確認できたため、他のtest系プロトタイプとは別扱いで残した。
  ただし2026-08-16の`pdr_program/`集約時、この4つは分割した: `L_map.png`は
  `map_configs/l_map.json`の`"map_image": "../L_map.png"`から直接参照されるため
  `Lmap.py`(生成元スクリプト)と共に`pdr_program/`へ移動し、`L.png`/`test7.py`
  (現行パイプラインからは参照されない、l_map.jsonの較正定数の"由来"として過去に
  言及されただけ)はリポジトリ直下に残した。4ファイルとも削除・削除候補行きでは
  ないので、この分割は本節の「現役使用中」判断と矛盾しない。
- **`kanri_4f_preview_final3.png`**: 元の平面図→検出壁→二値化の3段階図。卒論3.4節
  「建物地図」にそのまま使える。

### 削除候補に移したが、卒論用に復元候補として要記憶なもの

- **`result_none.png`/`result_prefer.png`**: 「y≈300張り付き」バグと「prefer時に
  部屋へ漏れる」バグの、修正前のスクリーンショット。卒論7.7節「失敗例」の図として
  使える可能性があるため、削除候補フォルダから復元する価値がある(
  →詳細は[pipeline_fixes.md](pipeline_fixes.md)のroute_points修正・preferモードの項目)。
