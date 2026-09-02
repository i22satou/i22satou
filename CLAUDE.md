# CLAUDE.md

## 必須方針

- **2026-08-16、現行プログラム一式を `pdr_program/` フォルダへ集約した。**
  `pdr_pf_improved.py`・`pdr_route_graph.py`・検証ツール群・`map_configs/`・
  `results/`・`start_positions.csv`・`CHANGELOG.md`はすべて`pdr_program/`配下。
  以下のこのファイル内のパス表記(`pdr_pf_improved.py`、`map_configs/*.json`等)は
  すべて`pdr_program/`からの相対パスとして読むこと。全体構成は
  [pdr_program/README.md](pdr_program/README.md)参照。
- 常に日本語で回答し、コメント・docstring・ログも原則日本語にする。
- 現行の編集対象は `pdr_program/pdr_pf_improved.py`。ただし2026-08-16にファイル
  肥大化(3500行超)対策として経路帯マスク抽出・通路グラフ化関連の関数を
  `pdr_program/pdr_route_graph.py` へ切り出した。こちらも同じプログラムの一部として
  通常通り編集対象に含める(`pdr_pf_improved.py`冒頭で
  `from pdr_route_graph import (...)`しているだけで、呼び出し方は従来通り)。
  他の大規模PFスクリプト(`pdr_pf_clickstart.py`等、リポジトリ直下に残したまま)は
  引き続き参照用で、明示指示がない限り変更しない。
- リポジトリ内の実行・編集は確認不要。ただし、削除、リモートへのpush、リポジトリ外への破壊的操作は行わない。
- 入力CSV（`pdr_log_*.csv`）は絶対に変更しない。変換はメモリ上で行う。`start_positions.csv`への既存仕様上の書き込みのみ例外。
- 実行ごとに結果PNGを必ず保存する。`--save`未指定時も `results/` へ自動保存する挙動を維持する。
- `pdr_pf_improved.py`または`pdr_route_graph.py`を変更したら、`CHANGELOG.md`の先頭へ
  絶対日付付きで1項目追加する(2026-08-16に冒頭コメントの変更履歴が肥大化したため
  `CHANGELOG.md`へ切り出した。`pdr_pf_improved.py`冒頭には直近数件のみ残す運用)。
- 過去の詳しい調査経緯・却下した仮説が必要な場合のみ `memo/` を参照する(索引:
  [CLAUDE_MEMO.md](CLAUDE_MEMO.md)。関係するトピックのファイルだけ読む — 全文は
  読まない。`pdr-memo-lookup` skillが絞り込みを助ける)。非自明な決定をしたら
  該当トピックの`memo/*.md`に追記する(ルールはCLAUDE_MEMO.md参照)。
- 不明な研究条件、実験結果、参考文献を推測・捏造しない。

## 研究概要

スマートフォンIMUを用いた屋内PDRに、建物地図で制約した移動様態適応型Particle Filterを組み合わせ、推定軌跡を可視化する卒業研究。

### 出典タグ

- `[SmartPDR]`: HPF/LPFによる歩行信号、ステップ検出、4乗根式／対数式による歩幅推定。
- `[先行研究:移動様態PF]`: 直進・屈折・滞留の分類と、様態別の粒子数・ノイズ変更。
- `[本研究独自]`: 下記の本研究による拡張。変更時はタグを維持する。

### 本研究独自の主な要素

- 距離変換に基づく連続的な壁尤度(`dist_map`。先行研究の0/1二値判定との違い)。
- ヨーレート75パーセンタイル＋方位変化AND条件＋ヒステリシスによる屈折判定(`detect_move_behavior`。手ぶれによる誤検出を抑制)。
- 二値地図からの経路帯自動抽出(`extract_auto_route_mask`, `route_source=auto`)。手動`route_points`を使わない空間制約。
- 経路線分に連動した方位補正(`route_points`が前提の実装で、`route_source=auto`ではまだ未対応)。
- 地図規模に応じ`map_configs/*.json`(`adaptive_pf`)で設定する様態別粒子数(現在の`kanri_4f.json`は250/600/100)＋Neff比率による不確実性適応粒子数(`configure_behavior`、`kanri_4f.json`は既定ON、2026-08-30)。
- 開始位置登録・フォルダ監視・診断値記録・PNG自動保存を含む実験基盤。

## 現在の重要事項

- `route_source=manual` の `prefer/enforce` は正解に近い手動経路を使う比較条件であり、最終提案方式ではない。
- `route_source=auto` は二値地図から空間的な経路帯を抽出する。`extract_auto_route_mask`は通路と大きな部屋の壁際を区別できない設計限界を持つため、`exclude_wide_rooms`(既定OFF)で除外できる。`--auto-route-centerline`(既定OFF)と併用するとkanri_4fで曲がり角連動の方位補正が機能し、6シード検証で全滅回数-16.8%・終点誤差-18.5%を確認済み(除外半径が小さすぎると本物の合流点まで削れて破綻するため既定は無効。詳細はmemo/route_source_auto.md)。通路グラフ・複数経路仮説は未実装のまま。
- 不確実性適応粒子数は2026-08-30に既定ONへ確定(`kanri_4f.json`)。auto-enforce条件で全滅回数が6シード全てで改善(15.6→14.6回)することを根拠に採用。位置精度の一様な改善は未確認で、`route_constraint_mode=none`等の未検証条件ではわずかに悪化する場合もある(詳細はCHANGELOG.md 2026-08-30(5)項)。
- `is_in_wall()`は`route_mask`を意図的に見ない設計(経路外は重みを0にする方式で対応)。`enforce`モードでもここは意図的にそうなっている — バグに見えても直さないこと。
- **PDR上流(距離・方位)の系統誤差を2026-09-02に是正した**。ステップ検出の過検出(第2高調波、約1.7〜1.8倍)と歩幅の過小推定(約1/2)が逆向きに打ち消し合っており、これがmemo/sensor_mystery.mdで未解決だった1441/1442の性能差の真因だった(解決済み)。歩幅校正ゲイン`step_length_calibration_gain`=2.10は**評価用と同じ3CSVで求めた暫定値**で、次回計測の校正専用データで再同定する。
- **総距離校正(`target_distance_px`)は2026-09-02に機構ごと削除した**。ファイルごとに総距離を目標値へ強制する仕組みで、今後あえて歩行距離を変える計測方針と矛盾するため。なお`compare_route_source.py`はこれを既定815pxで使っていたので、**過去の比較実験の数値は距離を正解に合わせ込んだ条件下のもの**であり現在の結果と直接比較できない。校正は全CSV共通の`step_length_calibration_gain`1つに限る。
- 方位の質はCSVごとに大きく異なる。1442は良好、1441は初期基準のずれ(`--heading-calibration-mode walking`で改善)、1438は歩行中に正味+146度回っており初期基準では直らない(詳細はmemo/heading_calibration.md)。
- 真のRMSEには時刻対応した正解位置データが必要。現在の終点x誤差は代替指標であり、RMSEと呼ばない。評価パイプライン(`build_ground_truth.py`→`evaluate_accuracy.py`)は2026-09-02に通し動作を確認済みで、両者とも`--self-test`を持つ。**合成データから出たRMSE値は絶対に研究結果として報告しない。**
- `kanri_4f.json` の主比較対象は `0805_1438/1441/1442` の3CSV。L字合成地図(`map_configs/l_map.json`)は技術確認用で、auto経路抽出の主評価には使わない。
- `heading_source=android` には `yaw_deg` が必要。古いCSVは該当ファイルだけスキップする。

## 実行と確認

すべて`pdr_program/`ディレクトリ内で実行する(`cd pdr_program`してから)。

基本実行例:

```bash
cd pdr_program
python pdr_pf_improved.py \
  --map-config map_configs/kanri_4f.json \
  --no-watch --no-show --seed 42
```

主な比較:

```bash
cd pdr_program
python compare_route_source.py --seeds 1 7 42 100 777 2024
```

**注意(2026-08-30に発覚した失敗例)**: `--route-source auto`単体では経路帯マスクによる空間制約はかからない(`kanri_4f.json`の既定`route_constraint_mode`は`none`)。`--multi-hypothesis-routing`等を単独で動作確認する際も`--route-constraint-mode enforce`を明示的に付けないと、地図制約が実質オフのまま検証してしまい誤った結論を導く(実例: 2026-08-30、これに気づかず出した結論をCHANGELOG.md 2026-08-30(3)で訂正した)。`compare_route_source.py`の標準条件(`auto-enforce`等)を使うか、単独実行時は必ず`--route-constraint-mode enforce`を付けること。

変更後は、少なくとも対象条件でスクリプトを実行し、以下を確認する。

1. 正常終了
2. PNGが保存された
3. 入力CSVが変更されていない
4. 診断値と結果が意図せず変化していない
5. 変更内容と検証結果を報告する

## 記録先

- コード変更内容: `pdr_program/CHANGELOG.md`(旧: `pdr_pf_improved.py` 冒頭の変更履歴。2026-08-16に分離)
- 詳細な実験条件・数値: `pdr_program/results/` 内のCSV・PNG・実験ログ
- 卒研全体の進捗・研究判断・論文構成: `../研究計画系/進捗反映版メモ.txt`(このリポジトリの外、`卒業研究/研究計画系/`にある。パスを省略して探すとi22satou直下に同名の別ファイルを誤って作成しがちなので注意)
- 過去の詳しい調査記録・却下した仮説: `memo/`(索引: `CLAUDE_MEMO.md`。トピック別、関係する1ファイルだけ読む)
- 通常作業では、この `CLAUDE.md` 以外を自動的に全文参照しない。

## 構成上の注意

- 設定は `pdr_program/map_configs/*.json`。必須値の欠落はエラーにする。
- 生データはリポジトリ外を含む。パスやデータを勝手に変更しない。
- リポジトリ直下に残した `pdr_pf_clickstart.py`(旧版)、`test7.py`/`L.png`(初期の
  L字検証、現行パイプラインとは独立、詳細はmemo/file_cleanup.md)は原則履歴・参照用。
  その他の `test*.py` 等も同様(`削除候補/`にあるものは移動済みで未削除)。
- 研究計画、章構成、進捗判定が必要な作業だけ `進捗反映版メモ.txt` を参照する。通常の小修正では全文を毎回読まない。

## このファイルの維持方針

- 「現在の重要事項」は最大6〜7項目程度に保つ。解決済みの事項は削除し、新しい結論は既存項目を書き換える。
- 詳細な数値・調査経緯を追記し続けない(それは`memo/`の役目)。
- 恒久的な作業ルールと、現在の作業に不可欠な情報だけを残す。
