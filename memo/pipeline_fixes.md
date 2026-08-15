# pdr_pf_improved.py 本体の初期バグ修正・整理(決着済み・圧縮版)

route_pointsの座標修正、prefer/enforceモードの挙動修正、pdr_pf_clickstart.pyとの
差分解消など、機能追加ではなく既存パイプラインの正しさに関わる初期修正のまとめ。
いずれも実装済み・決着済みのため、2026-08-16に経緯の詳細を圧縮し、結論と運用知識
だけを残した(圧縮ルールは[CLAUDE_MEMO.md](../CLAUDE_MEMO.md)参照)。「何を変えた
か」の詳細は`pdr_pf_improved.py`冒頭の変更履歴コメントを参照。

---

### 未使用コード削除(2026-08-15)

`pyflakes`/`vulture`+手動集計で未使用コードを特定・削除(`estimate_step_length_px()`
とWeinberg法関連定数、実際には使われず誤ったログを出すだけだった`START_X`/
`START_Y`)。**教訓**: `vulture`は`watchdog`のコールバックメソッド(`CSVHandler`の
`on_created`等)を誤って「未使用」報告する。静的解析の「未使用」は必ず実際の呼び出し
経路(フレームワークのコールバック含む)を確認してから判断すること。削除前後で
推定結果が完全一致することを確認済み。

### exclude_csvの導入(2026-08-15)

`pdr_log_0410_1535/0414_0902/0414_0940/0624_14.csv`の4CSVは、`kanri_4f.json`の
経路(Z字)に対して歩幅校正が破綻する(生の推定距離が目標距離の1/4以下)ため、
`map_configs/*.json`の任意設定`exclude_csv`で実行対象から除外。この4CSVは
`map_configs/l_map.json`(L字経路)側で扱う(→[route_source_auto.md](route_source_auto.md)参照)。

### route_pointsの座標修正(2026-08-15)

**症状と原因**: `none`モードで軌跡が開始位置付近に張り付いて見えた。真因は
(1)磁気センサ列が無く方位補正が効かずジャイロ単体積分がドリフトする、
(2)`route_points`が実際に歩いた経路とズレていた、の複合。PF自体のバグではない。

**確定した経路**(`kanri_4f_binary_final3.png`基準ピクセル座標):
`start(100,230) → ①(425,230) → ②(425,115) → ③(900,115)`。
0410/0414×2/0624の4CSVは①②まで(exclude_csv対象)、0805系3CSVは③まで歩いている。

**教訓**: 曲がり角の座標は目視だけで確定させず、二値地図の画素値を直接走査して
通行可能かを確認するのが確実(目視では壁に見えた場所が実際は通行可能だったケースが
あった)。

**運用上の注意**: `route_points`はJSON内で1本のグローバル設定であり、CSVごとに
経路長を変える仕組みがない。この修正の効果は`prefer`/`enforce`モードでのみ見える
(`none`は`route_points`を使わないため、`none`のままだと見た目は修正前と変わらない
— 結果解釈時に混同しないこと)。

### preferモードでの経路逸脱(2026-08-15)

**症状**: `prefer`モードは経路外でも重み`off_route_weight`(0.15)を残すソフトな
制約であり、`is_in_wall()`は`route_mask`を見ない設計(→CLAUDE.mdのScript lineage節
参照)。壁の扉の隙間(物理的に通行可能)から粒子が経路外の部屋へ抜け、そのまま
留まり続ける問題があった。

**運用ルール**: `0805`系3CSVを経路制約ありで実行する場合は`prefer`ではなく
`enforce`を使うこと。`enforce`は経路外の重みを完全に0にするため、扉を抜けようと
した粒子群はその場で重み0になり、全滅時リカバリ(route_mask考慮済み)で経路内へ
引き戻される。

### pdr_pf_clickstart.pyとのparity解消(2026-08-15)

`pdr_pf_improved.py`を`pdr_pf_clickstart.py`と同水準に揃えた(PNG自動保存、
`require_config_value()`による設定必須チェック、CSV単位でのエラー握りつぶし、
出典タグの移植)。現状の差分・意図的に揃えていない点(`is_in_wall()`が
`route_mask`を見ない設計を`pdr_pf_improved.py`側では維持している理由等)は
CLAUDE.mdの「Script lineage」節が最新かつ正なので、そちらを参照すること
(このエントリ側では重複を避けるため詳細を割愛)。
