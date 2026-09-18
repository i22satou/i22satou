<!-- 新しい項目はこの直下に追加する。2026-09-02以前の項目はCHANGELOG_archive.md。月が変わったら前月分をアーカイブへ移す。 -->

## 2026-09-18

(0) CSVフォルダを環境変数`PDR_DATA_DIR`で指定できるようにした。優先順位は
    `--data-dir` > `PDR_DATA_DIR` > JSONの`data_dir`。JSONの`data_dir`はMacのパスのため、
    Windows(Google Driveは`G:\マイドライブ\PDR`)では環境変数で上書きする。
    `apply_map_config()`内の変更なので、`tools/`・`evaluation/`のスクリプトも同じ規則に従う。
    環境変数が未設定なら従来と同じ動作。kanri_4f・seed=42で`--data-dir`指定時と
    環境変数指定時の出力PNG(MD5)とログが完全一致し、入力CSVが不変であることを確認した。

(1) [本研究独自] 卒論第7章の比較方式を本体から実行できるようにした(既定の動作は不変)。
    `--save-pdr-trajectory-csv`: 方式A(PDRのみ)の軌跡を`--save-trajectory-csv`と同じ形式で
    保存。PFと同じ歩・歩幅を経路補正前のセンサー方位で積算するだけで、乱数を使わない。
    `--pf-mode fixed`(JSONの`pf_mode`でも可): 方式B(固定粒子数PF)。3様態の粒子数・ノイズを
    `adaptive_pf.fixed_*`(kanri_4f.jsonに600粒子・0.7px・15度を追加)にそろえ、不確実性適応を
    OFFにする。`--fixed-particles`で粒子数だけ上書きできる。固定時だけPNG・軌跡CSVの名前に
    `_pf-fixed600`が付き、タイトルとログも変わる。`--trajectory-dir`: 軌跡CSVの保存先
    (未指定なら従来どおりresults/)。方式とオプションの対応はREADME.md「比較方式と実行オプション」。
    kanri_4f・seed=42の既定条件ほか3条件で修正前後のPNG(MD5)とログが完全一致、入力CSVは不変。

## 2026-09-03

(0) data_dirのCSV探索が、計測アプリの派生CSVまでセンサーログとして拾っていた。
    `glob("pdr_log_*.csv")`は`pdr_log_XXXX_waypoints.csv`にもマッチする。計測アプリ
    (AndroidStudioProjects/test2)はSTOP時に本体CSVと地点マークCSVを同時に共有するため、
    次回の実測データをdata_dirへ置いた時点で発現する未発生バグだった。実害は例外だけでなく、
    `get_or_select_start_position`が`validate_log`より先に呼ばれるため、地点マークCSVに
    対して開始位置のクリック選択が始まり、非GUI環境では無限に待ち続ける
    (再現確認: 修正前は10分経過してもタイムアウトせず、修正後は正常終了)。
    判定を`is_sensor_log_csv()`に切り出し、`_waypoints` / `_ground_truth` / `_trajectory`
    の接尾辞と`_traj_`を含む名前を除外した(`pdr_log_XXXX_utf8.csv`と、計測アプリが
    同一分内の再記録で付ける`pdr_log_XXXX_2.csv`はセンサーログとして残す)。
    監視モードの`CSVHandler.request_redraw`と、同じglobを持つ`check_sensor_quality.py`も
    同じ判定に統一した(こちらはexcept節がValueErrorを拾うため停止はせず、
    地点マークCSVに対する無意味な[警告]が出るだけだった)。
    `compare_route_source.py`はsubprocessで`pdr_pf_improved.py`を呼ぶため自動的に追随する。
    既定条件(kanri_4f, seed=42, --no-watch --no-show)の全ログを修正前と比較し、
    診断値・最終推定位置とも完全一致することを確認済み(挙動は不変)。

(1) 冒頭コメントの【方位推定方法】が「利用可能な場合は磁気センサを用いて方位を推定する」
    のままだった。磁気センサ融合(updateMARG)は2026-09-02(6)項で削除済みで、実際には
    ジャイロと加速度によるupdateIMUのみで動作している。フローチャートを実装と突き合わせた
    際に発見。コメントのみの修正で、動作は変更していない。

(2) [本研究独自] 分岐仮説の選別尤度を追加した(進捗反映版メモ §24.5 E)。
    複数経路仮説PFは交差点で粒子をエッジへ分岐させるが、分岐先がどちらも通路であれば
    壁尤度でも経路帯マスクでも仮説を区別できず、選別する尤度項が1つも無かった。
    そこで、粒子が乗っている通路区間の方位と、その歩で観測されたセンサー方位との
    一致度をガウス尤度として重みに掛ける処理を`update()`へ追加した。
    定数`MULTI_HYPOTHESIS_BRANCH_LIKELIHOOD_SIGMA_DEG`(既定None=無効)、
    JSON `multi_hypothesis_branch_likelihood_sigma_deg`、CLI
    `--multi-hypothesis-branch-likelihood-sigma-deg`。既定は無効なので既存の挙動は不変
    (既定条件seed=42で1442の最終位置がx=484.6, y=259.9と変更前に一致することを確認)。

    【既存の`--multi-hypothesis-branch-heading-sigma-deg`との違い】あちらは「どのエッジへ
    分岐させるか」という**提案分布**を方位で偏らせるもので、重みで補正していない。
    ブートストラップPFでは提案分布を偏らせても重みで補正しなければ推定は改善しないため、
    2026-08-30(4)項の検証で一様乱択を上回らなかったのは当然だった。今回追加したのは
    分岐後の状態を観測で評価する**重み側の尤度**なので、提案分布が一様(既定のσ=100000)で
    あれば二重計上にならない。両方を有限値にすると二重計上になるため警告を出す。

    比較対象は`step_heading`(生のセンサー方位)であり、`corrected_step_heading`
    (区間方位を混ぜた補正後の方位)でも`p_angle`(提案サンプル)でもない。前者は区間方位
    から作られているので循環し、後者は2026-09-02(3)項で削除した二重計上と同じ誤りになる。

(3) `_route_corrected_headings()`から、粒子ごとの区間方位を求める部分を
    `_particle_route_headings()`として切り出した((2)と共用するため)。計算内容は不変。

(4) 診断ログに「分岐仮説の選別尤度: σ=○度, 区間方位とセンサー方位の平均ずれ=○度」を
    追加した(選別尤度が有効なときのみ)。この平均ずれは方位データの質をそのまま映し、
    seed=42の実測で1442が19.2度、1441が68.3度、1438が79.2度と、
    memo/heading_calibration.mdの診断と一致した。

(5) キャッシュ判定(`cache_context`)に`MULTI_HYPOTHESIS_BRANCH_LIKELIHOOD_SIGMA_DEG`と
    `HEADING_CALIBRATION_MODE`を追加した。含めないと、フォルダ監視中にこれらだけを
    変えたときに古い結果が返ってしまう(後者は2026-09-02の追加時からの取りこぼし)。

(6) `_advance_route_state()`のdocstringが「分岐後の仮説は壁尤度・経路帯マスク・方位整合度で
    選別される」と書いていたが、方位整合度による選別は2026-09-02(3)項で削除済みで
    誤りだった。訂正し、(2)で追加した選別尤度を指すようにした。

(7) `sensitivity_branch_likelihood.py`を新規作成。選別尤度のσ(off/15/30/60/90)と
    初期方位の校正方式(samples/walking)を振って効果を測る。

# pdr_pf_improved.py 変更履歴

`pdr_pf_improved.py`冒頭コメントの変更履歴が肥大化してきたため、2026-08-16にこのファイルへ切り出した(過去分を全件移動、内容は不変)。新しい変更を記録する際は、このファイルの先頭(最新が上)に追記する。

---

