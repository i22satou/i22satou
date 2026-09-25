# figures/

卒論用の図と、コードの点検・説明に使った図を置くフォルダ。作り方は各スクリプトの冒頭も参照。
使わなくなった図・スクリプトは `../削除候補/figures/` へ移した(理由は同フォルダの README.md)。

## 卒論の図の一覧(`卒論/卒業論文_雛形.md` が貼っているもの)

| 図 | ファイル | 作り方 |
|---|---|---|
| 3.1 | `fig3_1_map_route.png` | `thesis_figures.py` |
| 3.2 | `fig3_2_sensor_example.png` | `thesis_figures.py` |
| 4.1 | `pdr_system_diagram.png` | `make_flowcharts.py` → `render_flowcharts.py` |
| 4.2 | `pdr_flow_main.png` | `make_flowcharts.py` → `render_flowcharts.py` |
| 4.3 | `fig4_3_step_detection.png` | `thesis_figures.py` |
| 4.4 | `pdr_flow_pf_detail.png` | `make_flowcharts.py` → `render_flowcharts.py` |
| 5.1 | `fig5_1_route_mask_graph.png` | `pdr_program/evaluation/verify_route_graph.py`(下の第5章の節) |
| 5.2 | `fig5_2_route_mask_graph_exclude_wide_rooms.png` | 同上 |
| 8.1 | `fig8_1_acc_spectrum.png` | `thesis_figures.py` |
| 8.2 | `fig8_2_heading_profile.png` | `thesis_figures.py` |

`thesis_figures.py` は本体(`pdr_pf_improved.py`)の関数をそのまま使う。CSVフォルダは本体と同じく環境変数
`PDR_DATA_DIR` で指定する(Windowsでは `G:\マイドライブ\PDR`)。2026-09-24 に今の本体で作り直し、5枚とも
画素単位で今のファイルと一致した(図が今の処理と食い違っていないことの確認)。

## 2026-09-24 のコード点検で作った図

どれも研究結果(RMSE)ではなく、判定や確認の仕組み、軌跡の様子を説明するための図。使ったデータは既存の
実測3本(`pdr_log_0805_1438/1441/1442`)で、調整用データであって評価用データではない。
下の3枚は `make_figures_20260924.py` で作り直せる(推定軌跡の2枚はこのフォルダの外に置いてある。下の各項目を参照)(`i22satou/` で `python figures/make_figures_20260924.py`)。
同じスクリプトは quick_check の図も作るが、卒論に使わないので `../削除候補/figures/` へ移した(作り直すとここに再び出る)。

### `20260924_移動様態判定_曲がりが終わらない問題_1441_1442.png`

移動様態(直進・曲がり)の判定が、一度「曲がり」になると戻らない問題を示す図。本体の判定関数
(`detect_move_behavior`)をそのまま呼んで、1441・1442の歩ごとの判定を再現した(方位はandroid)。

- 上の段: 方位(歩き始めを0度)。本当の曲がり角は2か所だけ(1441は20秒と29秒付近)。
- 中の段: 各歩の「直近1.5秒のヨーレート絶対値の75パーセンタイル」。まっすぐ歩いていても、歩行の揺れで
  12〜25度/秒ある。曲がり終了の条件(10度/秒未満)を満たす歩はほとんど無い。
- 下の段: 判定の結果。歩き始めてすぐ(1441は1.0秒、1442は4.1秒)に「曲がり」に入り、最後近くまで戻らない。
  曲がりと判定された歩は1441で81%、1442で87%。

図は2026-09-24時点の設定(曲がり終了のヨーレートしきい値10度/秒)。このしきい値は同日、校正用の直線歩行から
決める仕組み(`pdr_program/evaluation/calibrate_turn_threshold.py`)に変えた。値は計測日の calib の記録で決める
(`memo/comparison_methods.md`)。

### `20260924_推定軌跡_4方式の比較_1441_1442_1438_seed42.png`

置き場所: `pdr_program/results/20260918_133540_evaluation_0805check/`(このフォルダではない)。

比較する4方式(A: PDRのみ、B: 固定粒子数PF、C: 移動様態適応PF、E: 提案方式)の推定軌跡を、3本まとめて
地図に重ねた図。データは同日の比較実験 `pdr_program/results/20260924_120622_evaluation_0805check_recovery/`
(コミット55e3e74、方位は全方式 android+walking)の seed=42 の1回分。評価ハーネスが1本ずつ出す
`trajectory_pdr_log_0805_*.png` と同じ軌跡で、次を足した。

- 黄色の帯: 申告された歩行経路(開始位置 → (425,230) → (425,115) → (800,115))。`kanri_4f.json` の
  手動経路と同じ座標。時刻との対応が無いので、誤差の計算には使っていない。
- 星: 申告された終点(800,115)。各段の見出しの「終点までの距離」は6シードの平均で、RMSEではない代替指標
  (`endpoint_proxy_0805.csv` と同じ値)。
- 順番は方位の記録が使える1441・1442を先にし、方位の記録が壊れている1438は参考として最後に置いた。

調整用データの1回分の軌跡なので、方式の優劣の根拠にはしない。軌跡CSVは git に含めない `runs/` にあるので、
無い環境では `run_evaluation.py` を同じ条件で実行し直してから作る。

### `20260924_推定軌跡_9月18日と9月24日の比較_全滅時の復帰の変更前後.png`

置き場所: `進捗報告/`(このフォルダではない)。

粒子が全滅したときの復帰の仕方を変える前(9/18の比較実験 `pdr_program/results/20260918_133540_evaluation_0805check/`)と
後(9/24の `20260924_120622_evaluation_0805check_recovery/`、コミット55e3e74)で、方式B・C・Eの軌跡を重ねた図
(行がデータ、列が方式)。2つの実験で主な方式に効く本体の違いはこの変更だけ。

- 灰色の太線が変更前、方式の色の細線が変更後の軌跡(seed=42)。同じなら灰色の中に色の線が重なる。
- 点は6シードの終点(白抜きが変更前、塗りが変更後)。
- 見出しは全滅回数(6シードの合計)と終点までの距離(6シードの平均、RMSEではない代替指標)の変更前→変更後。
  数値は `pdr_program/results/20260924_summary.md` の表と同じ。
- 方式Aは乱数も全滅も無いので両日で同じ(軌跡の一致を確認済み)で、図には入れていない。

全滅が起きなかった実行は両日で完全に同じ。目に見えて変わったのは方式Bの1442と提案方式の1441だけで、変わった
実行には全滅後に乱数の使い方が変わった影響も含まれる(`memo/comparison_methods.md` の2026-09-24)。

## 卒論第4章のフローチャート(`pdr_system_diagram`・`pdr_flow_main`・`pdr_flow_pf_detail`)

図4.1(システム全体の構成)・図4.2(1つのCSVの処理フロー)・図4.4(PFの1歩分の処理)。`.drawio` は
`make_flowcharts.py` の出力で、`.png` はそれを描画したもの。2026-09-24 に、`pdr_pf_improved.py` の今の処理と
卒論雛形の節・式番号に合わせて内容を直し、矢印を角の丸めのない直線・直角の折れ線にした。主な修正は次のとおり。

- 式番号を雛形に合わせた(壁尤度5.6、経路帯の重み5.7、正規化5.8、重み付き平均5.9、粒子数の補正5.3・5.4、
  分岐選別尤度5.5、経路方位補正5.1)。曲がり判定の節は5.4節ではなく4.7節。
- 移動様態判定は歩が検出されたときだけ行うので、実際に出るのは直進と曲がりだけ(滞留は出ない)。
- 全滅からの復帰を2026-09-24の変更(移動前の平均+その歩の移動量の周り)に合わせ、全滅回数の記録を復帰の前にした。
- 本体は診断値のCSVを出さない(PNGは毎回、軌跡CSVはオプション指定時だけ)。開始位置の入力と、様態判定から
  経路方位補正への「曲がり」の矢印を足した。経路方位補正は中心線の変種・手動経路のときだけ働く。
- 目印表はクリック方式(`pick_landmarks.py`)ではなく `make_route_landmarks.py` で作る。`run_evaluation.py` が一括実行する。

作り直すときは `figures/` で次の2つを順に実行する。

    python make_flowcharts.py      # .drawio を作る
    python render_flowcharts.py    # .png(2倍)を作る

`make_flowcharts.py` は書き出し時に、線が斜めになっていないか、線が他の図形を貫通していないか、文字が
はみ出しそうでないかを調べ、問題があれば「要確認」と表示する。`render_flowcharts.py` は draw.io(デスクトップ版)が
あればそのCLIで、無ければ Edge/Chrome のヘッドレス表示と draw.io 公式ビューア(ネット接続が要る)で描く。
2026-09-24 の PNG は後者(Windows)で作った。draw.io で手編集した場合も、この2つ目だけ実行すればよい。
`check_flowcharts.py` は Mac の draw.io で書き出した SVG を使う、より細かい検査(文字と線の重なりも見る)。

## 卒論第5章の図(2026-09-24)

### `fig5_1_route_mask_graph.png` / `fig5_2_route_mask_graph_exclude_wide_rooms.png`

卒論の図5.1・図5.2。二値地図から自動抽出した経路帯(黄色)と、その骨格から作った通路グラフ(整理後)。
図5.1は提案方式の既定の設定(交差点14・端点3・通路24)、図5.2は変種「広い部屋の除外」を有効にしたもの
(交差点8・端点7・通路17)。`pdr_program/` で次のように作り、ここへコピーした。

    python evaluation/verify_route_graph.py --map-config map_configs/kanri_4f.json --simplify --no-exclude-wide-rooms --save <出力先>
    python evaluation/verify_route_graph.py --map-config map_configs/kanri_4f.json --simplify --exclude-wide-rooms --save <出力先>

注意: `pdr_program/results/kanri_4f_route_graph_check_simplified.png` は「広い部屋の除外」を有効にした図で、
既定の設定の図ではない。
