# CLAUDE.md

## 必須方針

- 常に日本語で回答し、コメント・docstring・ログも原則日本語にする。
- 現行プログラムは`pdr_program/`配下。以下のパスは`pdr_program/`からの相対パス
  (全体構成は[pdr_program/README.md](pdr_program/README.md))。
- 編集対象は`pdr_pf_improved.py`と`pdr_route_graph.py`(後者は前者から切り出した通路
  グラフ関連で、同じプログラムの一部)。リポジトリ直下の`pdr_pf_clickstart.py`・
  `test*.py`・`削除候補/`等は参照用で、明示指示がない限り変更しない。
- リポジトリ内の実行・編集は確認不要。削除、リモートへのpush、リポジトリ外への破壊的操作は行わない。
- 入力CSV(`pdr_log_*.csv`)は絶対に変更しない(変換はメモリ上)。例外は`start_positions.csv`への既存仕様上の書き込みのみ。
- 実行ごとに結果PNGを必ず保存する(`--save`未指定時も`results/`へ自動保存する挙動を維持)。
- 本体2ファイルを変更したら`CHANGELOG.md`の先頭へ絶対日付付きで1項目追加する。
  **CHANGELOG.mdは先頭30行だけ読んで追記する**(全文を読まない)。1項目は「何を変えたか・
  挙動が変わったか」を数行で書き、判断理由・調査経緯は`memo/`へ書く。
- 不明な研究条件、実験結果、参考文献を推測・捏造しない。

## 研究概要

スマートフォンIMUを用いた屋内PDRに、建物地図で制約した移動様態適応型Particle Filterを組み合わせ、推定軌跡を可視化する卒業研究。

出典タグ: `[SmartPDR]`(歩行信号・ステップ検出・歩幅推定)、`[先行研究:移動様態PF]`(直進・屈折・滞留の分類と様態別の粒子数・ノイズ)、`[本研究独自]`(下記の拡張。変更時はタグを維持する)。

本研究独自の主な要素: 距離変換による連続的な壁尤度(`dist_map`)、ヨーレート75パーセンタイル+方位変化AND+ヒステリシスの屈折判定(`detect_move_behavior`)、二値地図からの経路帯自動抽出(`route_source=auto`)、経路線分連動の方位補正、様態別粒子数+Neff比率による不確実性適応粒子数(`configure_behavior`)、通路グラフ・複数経路仮説、開始位置登録・フォルダ監視・診断値記録・PNG自動保存の実験基盤。

## 現在の重要事項(詳細は各memo)

- `route_source=manual`の`prefer/enforce`は正解に近い手動経路を使う比較条件であり、最終提案方式ではない。
- **`--route-source auto`単体では地図制約がかからない**(`kanri_4f.json`の既定`route_constraint_mode`は`none`)。単独実行で検証するときは必ず`--route-constraint-mode enforce`を付けるか、`compare_route_source.py`の標準条件を使う(2026-08-30に誤った結論を出した実例あり)。
- `is_in_wall()`が`route_mask`を見ないのは意図的な設計。バグに見えても直さない。
- 2026-09-02にPDR上流(ステップ過検出・歩幅過小)を是正し、総距離校正を削除した。**それ以前の比較実験の数値は現在と直接比較できない**。歩幅校正ゲイン2.10は評価用CSVで求めた暫定値(memo/step_length_calibration.md)。
- 真のRMSEには時刻対応した正解位置が必要。終点x誤差は代替指標でありRMSEと呼ばない。**合成データのRMSE値を研究結果として報告しない**(memo/ground_truth.md)。
- 主比較対象は`kanri_4f.json`の`0805_1438/1441/1442`の3CSV。`l_map.json`は技術確認用。CSVごとに方位の質が大きく違う(memo/heading_calibration.md)。`heading_source=android`には`yaw_deg`列が必要。
- 不確実性適応粒子数は`kanri_4f.json`で既定ON(全滅回数は改善、位置精度の一様な改善は未確認。memo/uncertainty_particles.md)。既定OFFの実験機能(`exclude_wide_rooms`・`--auto-route-centerline`・`--multi-hypothesis-routing`・分岐選別尤度)の効果と注意点はmemo参照。分岐選別尤度と`--multi-hypothesis-branch-heading-sigma-deg`の併用は二重計上になる。

## 実行と確認

すべて`pdr_program/`内で実行する。本体は直下、検証・実験は`evaluation/`、準備・計測日の道具は`tools/`。

```bash
cd pdr_program
python pdr_pf_improved.py --map-config map_configs/kanri_4f.json --no-watch --no-show --seed 42
python evaluation/compare_route_source.py --seeds 1 7 42 100 777 2024
```

変更後は対象条件で実行し、(1)正常終了、(2)PNG保存、(3)入力CSV不変、(4)診断値・結果が意図せず変化していない、を確認して変更内容と検証結果を報告する。

## 記録先と参照ルール(トークン節約のため)

- この`CLAUDE.md`以外は、必要なときに必要な部分だけ読む。
- コード変更: `pdr_program/CHANGELOG.md`(2026-09-02以前は`pdr_program/CHANGELOG_archive.md`)。
- 調査経緯・却下した仮説: `memo/`(索引は[CLAUDE_MEMO.md](CLAUDE_MEMO.md)。関係するトピックの1ファイルだけ読む)。非自明な決定をしたら該当memoの先頭に追記する。
- 実験条件・数値: `pdr_program/results/`。
- 卒研全体の進捗・論文構成: `../研究計画系/進捗反映版メモ.txt`(リポジトリ外。i22satou直下に同名ファイルを誤って作らない)。研究計画・章構成の作業のときだけ読む。
- **次の大きなファイルは、卒論執筆を依頼されたときだけ開く**: `卒論/卒業論文_雛形.md`、`CLAUDE_MEMO.txt`(卒論向け文章の下書き)。
- 設定は`pdr_program/map_configs/*.json`(必須値の欠落はエラー)。JSONの`data_dir`はMacのパス。Windowsでは環境変数`PDR_DATA_DIR`(`G:\マイドライブ\PDR`)で上書きし、Pythonは`~/anaconda3/python.exe`を使う。生データはリポジトリ外を含むので、パスやデータを勝手に変更しない。

## このファイルの維持方針

- 毎回読み込まれるため短く保つ。「現在の重要事項」は7項目まで、1項目1〜2行。
- 数値・調査経緯・日付の詳細は書かない(`memo/`の役目)。解決済みの事項は削除する。
