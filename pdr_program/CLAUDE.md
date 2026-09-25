# pdr_program/CLAUDE.md(コード作業の規則)

リポジトリ直下の`CLAUDE.md`に加えて、このフォルダのファイルを扱うときに読み込まれる。
以下のパスはこのフォルダからの相対パス(全体構成は[README.md](README.md))。

## 編集の規則

- 編集対象は`pdr_pf_improved.py`と`pdr_route_graph.py`(後者は前者から切り出した通路
  グラフ関連で、同じプログラムの一部)。リポジトリ直下の`pdr_pf_clickstart.py`・
  `test*.py`・`削除候補/`等は参照用で、明示指示がない限り変更しない。
- 実行ごとに結果PNGを必ず保存する(`--save`未指定時も`results/`へ自動保存する挙動を維持)。
- 本体2ファイルを変更したら`CHANGELOG.md`の先頭へ絶対日付付きで1項目追加する。
  **CHANGELOG.mdは先頭30行だけ読んで追記する**(全文を読まない。2026-09-02以前は
  `CHANGELOG_archive.md`)。1項目は「何を変えたか・挙動が変わったか」を数行で書き、
  判断理由・調査経緯は`memo/`へ書く。
- 出典タグ(`[SmartPDR]`・`[先行研究:移動様態PF]`・`[本研究独自]`)は変更時も維持する。
  本研究独自の要素の一覧は`pdr_pf_improved.py`冒頭のコメントにある。

## コードの注意点(詳細は各memo)

- **`--route-source auto`単体では地図制約がかからない**(`kanri_4f.json`の既定`route_constraint_mode`は`none`)。単独実行で検証するときは必ず`--route-constraint-mode enforce`を付けるか、`compare_route_source.py`の標準条件を使う(付け忘れると、制約なしの結果を提案方式の結果と取り違える)。
- `is_in_wall()`が`route_mask`を見ないのは意図的な設計。バグに見えても直さない。
- 不確実性適応粒子数は`kanri_4f.json`で既定ON(memo/uncertainty_particles.md)。既定OFFの実験機能(`exclude_wide_rooms`・`--auto-route-centerline`・`--multi-hypothesis-routing`・分岐選別尤度)はmemo参照。分岐選別尤度と`--multi-hypothesis-branch-heading-sigma-deg`の併用は二重計上になる。
- `heading_source=android`には`yaw_deg`列が必要。

## 実行と確認

すべてこのフォルダ内で実行する。本体は直下、検証・実験は`evaluation/`、準備・計測日の道具は`tools/`。
設定は`map_configs/*.json`(必須値の欠落はエラー)。JSONの`data_dir`はMacのパス。Windowsでは
環境変数`PDR_DATA_DIR`(`G:\マイドライブ\PDR`)で上書きし、Pythonは`~/anaconda3/python.exe`を使う。
生データはリポジトリ外を含むので、パスやデータを勝手に変更しない。

```bash
python pdr_pf_improved.py --map-config map_configs/kanri_4f.json --no-watch --no-show --seed 42
python evaluation/compare_route_source.py --seeds 1 7 42 100 777 2024
```

変更後は対象条件で実行し、(1)正常終了、(2)PNG保存、(3)入力CSV不変、(4)診断値・結果が意図せず変化していない、を確認して変更内容と検証結果を報告する。
