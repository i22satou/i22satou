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

## 2026-09-02(8) [RMSE評価パイプラインのドライラン(§23項目4)]

正解位置データがまだ実測できていない段階で、`build_ground_truth.py` → `evaluate_accuracy.py`の通し動作を確認した(進捗反映版メモ§23の項目4)。

**(a) `build_ground_truth.py`に`--self-test`を追加**(`evaluate_accuracy.py --self-test`と同じ方式、外部ファイル不要)。Phase 2の結合ロジックについて、(1)seqが一致する正常系でseq順に結合され、waypointsの行順が乱れていてもseqで正しく対応付けられること、(2)waypoints側に余分なseqがある「押し過ぎ」を該当seq付きで検出すること、(3)landmarks側に未計測のseqがある「押し忘れ」を同様に検出すること、の3点を検証する。あわせて`--waypoints`/`--landmarks`を任意引数化した(self-test単独で走らせるため)。

**(b) 入力エラーをトレースバックではなくメッセージで返すようにした**。押し忘れ・押し過ぎ・列不足はいずれも利用者が直せる入力の問題なので、`SystemExit`で終了コード1とメッセージのみを出す。

**(c) 実軌跡を使った通し確認**。`--save-trajectory-csv`で出力した実データの推定軌跡(pdr_log_0805_1442、auto-enforce/android/walking/seed=42)に対し、架空の目印表・地点マークCSVから正解位置CSVを組み立てて`evaluate_accuracy.py`へ渡し、両スクリプトが接続されることを確認した。時刻範囲外の正解点が警告付きで除外される経路も動作を確認済み(4点中1点を範囲外にして、除外1件・対応点数3と正しく報告された)。

**【重要】このドライランで出た数値(平均誤差114.06px、RMSE 160.64px等)は研究結果ではない。** 地点マークのtimestampを推定軌跡の時刻範囲に等間隔で置いた架空の値であり、実際に歩行者がその時刻にその目印にいた事実は一切ない。確認したのは配管であって精度ではない。**実測の正解位置データが取れるまで、RMSEの実数値は一切報告しない。**

**(d) 推定軌跡CSVの命名に実行条件を追加**。従来は`{CSV名}_trajectory.csv`固定で、条件やシードを変えて実行すると黙って上書きされ条件間比較ができなかった。`{CSV名}_traj_{制約モード}-{経路生成元}_{方位源}-{方位校正}_seed-{シード}.csv`へ変更した。

## 2026-09-02(7) [不要コードの削除 — 磁気センサ融合・総距離校正・未使用import]

本研究で使わないことが確定している処理を削除した。**削除前後で出力が完全一致することを確認済み**(auto-enforce/android/walking/seed=42の全滅回数・PF診断値・最終推定位置がすべて同一)。`pdr_pf_improved.py`は2826行→2734行。

**(a) 磁気センサ融合を削除**(`has_magnetometer`, `get_mag_heading`, `fuse_heading`, 定数`HCOR_THR`/`HMAG_THR`/`W_PREV`/`W_MAG`/`W_GYRO`, `updateMARG`分岐, `mag_enabled`/`prev_mag_heading`の追跡)。全7CSVの列は`timestamp, acc_x/y/z, gyro_x/y/z, [yaw_deg], x, y`で`mag_*`が存在せず、`has_magnetometer()`は常にFalse、`fuse_heading()`は先頭の`if`で必ず`gyro_heading`をそのまま返していた。つまり呼ばれてはいるが分岐の中身は一度も実行されない死んだコードだった。CLAUDE.mdでも磁気センサはスコープ外と明記済み。呼び出し側は`fused_heading = gyro_angle`へ簡約した(数学的に等価)。

**(b) 総距離校正(`target_distance_px` / `--no-step-calibration`)を削除**。どの設定JSONにもキーが存在せず、かつ「ファイルごとに総距離を目標値へ強制する」というこの機構は、今後の計測方針(あえて短く/長く歩くデータを取るため終点を固定しない)と正面から矛盾する。残しておくと循環的な校正を誘発する危険があるため、機構ごと削除して構造的に防いだ。`compare_route_source.py`・`sensitivity_uncertainty_particles.py`側の受け渡しも削除。

**【重要な訂正】この2スクリプトは`--target-distance-px`の既定値が815.0だった。** つまり過去の6シード比較実験(route_source_auto.md・uncertainty_particles.mdに記録された数値のほとんど)は、**各CSVの推定総距離を815pxへ強制した状態で測っていた**。このため上流の距離推定誤差(実際には1441/1442で4割不足)が完全に隠され、2026-09-02まで発見できなかった。過去のresults/CSVの数値と現在の実行結果は条件が異なるため直接比較できない。

**(c) 未使用importを削除**: `from skimage.morphology import skeletonize`。2026-08-16のファイル分割で`extract_ordered_centerline`ごと`pdr_route_graph.py`へ移ったのにimportだけ残っていた。

**削除しなかったもの**: `CSVHandler.on_created`/`on_modified`/`on_moved`は静的解析では未参照に見えるが、watchdogがフレームワーク側から呼ぶコールバックであり必要(memo/pipeline_fixes.mdが誤検出の代表例として記録している)。`step_gain`は`l_map.json`が1.12を設定しているため必要。

## 2026-09-02(6) [不確実性適応粒子数の既定ONを再確認]

[本研究独自] コード変更なし(検証のみ)。(3)の重み二重計上削除はNeffを直接押し上げるため、Neff比率を判定材料にするこの機能の前提が変わった。2026-08-30の既定ON判断が成立するか6シードで再検証し、**既定ONを維持**と結論した。全滅回数はONが同等以上、有効粒子率は3ファイルともON優位。

**ただし**、上流修正で全滅回数自体が激減し(平均15.6回→約2.4回)、2026-08-30の判断根拠だった「全滅回数の改善」にはもう伸びしろがほとんど無い。今後この機能の価値を主張するなら有効粒子率や位置精度で示す必要がある。数値はmemo/uncertainty_particles.md、生データはresults/20260902_221000_uncertainty_particles_recheck.csv。

## 2026-09-02(5) [比較条件の健全化]

[本研究独自] 2点修正。(a)**CSVごとに独立した乱数シードを割り当てた**(`file_random_seed()`、ファイル名のCRC32を基準シードへ加算し、ファイルループ先頭で再シード)。従来は`main()`で一度`np.random.seed()`を呼ぶだけで、複数CSVがファイル名順に同じ乱数ストリームを消費しており、CSVを1本足すだけで後続の結果が変わる・ファイル別の性能差にファイル順が交絡する、という問題があった(memo/sensor_mystery.mdが2026-08-15に指摘しつつ未修正だった)。(b)**複数経路仮説PFの方位補正を`route_constraint_mode`で分岐させた**。単一経路側は`prefer`/`enforce`のときだけ経路方位を使う仕様なのに、`_route_corrected_headings()`だけ無条件に適用されていた(仕様と実装の食い違い。2026-08-30(3)の誤りの根本原因)。

**注意**: (a)により今日以前の全数値はシードが変わるため直接比較できない。上流修正(1)〜(3)の影響と併せ、必ず取り直した値を使うこと。

## 2026-09-02(4) [初期方位校正に歩行開始基準を追加(既定OFF)]

[本研究独自] 開始方位の基準を「歩き始めてからの数歩分」から求める方式を追加(`--heading-calibration-mode walking`、歩数は`--heading-calibration-steps`既定10、android方式専用)。**既定は従来の`samples`のまま**。従来方式は先頭30サンプル=約0.57秒しか見ておらず、計測開始直後にまだ端末を持ち替えていると基準方位が壊れる(基準窓を30→250サンプルに広げると1438で-61.0度、1441で-50.9度も動く。1442は-3.1度で安定)。使う前提は「開始時の歩行方向が`--initial-heading-deg`である」という従来方式と同じ利用者提供の事実のみで、正解経路や推定結果は参照しないため循環参照にならない。

6シード検証(auto-enforce, android)で1441は弧長進捗17%→74%・全滅13.2→2.0回と大幅改善、1442は同等、1438のみ悪化。1438が悪化するのは方位記録自体が経路と整合しないため。既定を変えなかったのは、3本中1本が壊れた記録である状態では根拠が弱く、既存の比較実験の数値も無効になるため。数値と1438の詳細はmemo/heading_calibration.md、生データはresults/20260902_215941_heading_calibration_mode_comparison.csv。

## 2026-09-02(3) [PF重みの二重計上を削除]

[本研究独自] `ParticleFilterPDR.update()`の「方位ズレの重み」を削除した。`p_angle = corrected_step_heading + noise_angle`と定義した直後に`angle_diff(p_angle, corrected_step_heading)`をガウス密度で評価しており、これは`noise_angle`と恒等的に一致する。つまりサンプリングに使った提案分布の密度でそのサンプル自身を重み付ける二重計上で、観測情報を一切含まないため尤度として意味を持たなかった。実害は(1)実効的な方位分散が`sigma_angle/√2`へ縮み粒子が曲がりにくくなる、(2)Neffを理論値で約13%押し下げ、Neff比率を判定に使う不確実性適応粒子数(§6.5)がこれを「不確実性が高い」と誤読する、の2点。

検証(auto-enforce, gyro, seed=42): Neff平均が99.9→155.2 / 123.5→129.3 / 181.0→200.7と3ファイルすべて改善。

**今後の候補**: この削除で複数経路仮説PFには分岐仮説を選別する尤度項が無くなった(壁尤度も経路帯マスクも、分岐先がどちらも白い廊下なら区別できない)。センサー方位との一致度を独立した観測尤度(σは別パラメータ)として追加するのが自然な次の一手。2026-08-30に効果が出なかった`choose_branch_by_heading`も、選別機構が無かったことが原因の可能性がある。

## 2026-09-02(2) [歩幅推定の被験者・端末校正ゲインを追加]

[本研究独自] `estimate_smartpdr_step_length_px()`に校正ゲイン`STEP_LENGTH_CALIBRATION_GAIN`(JSON `step_length_calibration_gain`、既定1.0)を追加し、`kanri_4f.json`で2.10を設定。上下限クリップの**前**に適用する(クリップ後に掛かる既存の`step_gain`と分けたのは、上下限を歩幅の物理的な範囲として保つため)。

(1)のステップ検出修正で歩数が正しくなり、歩幅側の系統誤差が単独で見えるようになった。SmartPDRのβ・γ係数は原論文の計測環境で同定された値で、本研究の環境では歩幅を約1/2に過小推定する(推定0.35〜0.40m/歩 対 真値0.68〜0.89m/歩)。経験的歩幅モデルが被験者・端末ごとの校正を要するのは既知の性質であり、係数の不一致自体はバグではない。

**限界(卒論に明記すること)**: 2.10は評価対象と同じ3CSVの総距離比から求めた**暫定値**で、校正用データと評価用データが分離できていない。適用後の総距離誤差は+16.6%/-20.3%/-17.6%(適用前は-41.5%/-59.1%/-57.5%)。上限1.0mでクリップされる歩が18%/9%/18%発生しており、クリップがモデルの一部として働いている。

**この機構の位置づけ**: 総距離を特定の終点へ合わせる機構では**ない**。全CSV共通の定数を1つ掛けるだけなので、短く歩いたデータは短いまま出る。ファイルごとに総距離を目標値へ強制する`target_distance_px`は意図的に未設定(null)のまま維持する(今後の計測ではあえて歩行距離を変えるため、というユーザーの明示的な方針)。

**モデル同定は保留**: 真の歩幅との順位を比較したところ、振幅系の予測量(Weinberg・Kim・加速度標準偏差)は3つとも順位を取り違えた。順位が一致したのは歩調のみだが変動幅6%に対し歩幅は30%変動しており、同一経路の3ファイルからの同定は過適合になる。詳細はmemo/step_length_calibration.md。

## 2026-09-02(1) [ステップ検出の過検出を是正]

[SmartPDR]のステップ検出の最短ピーク間隔を、固定サンプル数(`STEP_MIN_INTERVAL=5`)から上限歩調ベース(`MAX_STEP_FREQUENCY_HZ=2.9`と実測サンプリング周波数から`step_min_interval_samples()`で算出)へ変更。`detect_steps_smartpdr(step_acc, sampling_rate_hz=None)`と引数が増えたが、未指定時は従来値へフォールバックする。

**真因**: `STEP_MIN_INTERVAL=5`は52.9Hzでは上限10.6歩/sに相当し事実上無制限だった。加速度合成値のスペクトルは基本波1.62〜1.72Hz(真の歩調)に対し**ちょうど2倍の第2高調波3.3〜3.4Hzに最大ピーク**を持ち、検出器がこれに反応して歩数を約1.7〜1.8倍に過検出していた。最短間隔を5〜26サンプルで振り、18サンプル(上限2.94Hz)で3ファイルとも真値の±5%に収まることを確認した。

**この修正が解いた既存の未解決問題**: memo/sensor_mystery.mdで未解明だった1441/1442の性能差は、この過検出(×1.7〜1.8)と歩幅の過小推定(×0.5、(2)項)が**ファイルごとに異なる割合で打ち消し合っていた**ことが原因。ゆっくり歩いた1438だけ偶然ほぼ相殺し、速く歩いた1441/1442では相殺しきれず4割短くなっていた。

## 2026-08-30(5) [不確実性適応粒子数の既定化判断(進捗反映版メモ§23の項目)]

[本研究独自] `map_configs/kanri_4f.json`の`adaptive_pf.uncertainty_adaptive_particles`を`true`に変更し、既定で有効化した(4パラメータneff_low_ratio/neff_high_ratio/boost_factor/shrink_factorは2026-08-16の感度分析で確認済みの初期値0.30/0.60/1.5/0.75のまま変更なし)。

判断根拠: 2026-08-15の検証で全滅回数がauto-enforce条件下の6シード全てで例外なく改善(平均15.6回→14.6回)、2026-08-16の感度分析でこの初期値が既に妥当(大きな改善余地なし)と確認済みだった(詳細はmemo/uncertainty_particles.md)。終点x誤差は平均では改善(339.9px→329.7px)するが一様ではなく、pdr_log_0805_1441は一部シードで悪化する。全滅回数(全粒子見失いという明確な不安定性指標)が全条件で確実に改善する一方、精度改善は保証されないという非対称な結果であり、「悪化を確実に防ぐ効果はないが、最も分かりやすい破綻(全滅)を確実に減らす」という理由でONを既定とした。

**注意(スコープ)**: この判断はauto-enforce条件下での検証に基づく。route_constraint_mode=none等、他の条件下での効果は未検証(実際、seed=42のnone条件では全滅回数がわずかに悪化した: 15→16, 1→2)。既定でONにしたため、`--no-uncertainty-adaptive-particles`を明示しない限り全ての実行に影響する点に注意。

## 2026-08-30(4)

[本研究独自] (3)の検証結果を受けて、`MULTI_HYPOTHESIS_BRANCH_HEADING_SIGMA_DEG`の既定値を30.0から100000.0(事実上の一様乱択)へ変更した(CLI既定・JSON設定既定の両方)。方位重み付け分岐選択(choose_branch_by_heading)自体は削除せず、`--multi-hypothesis-branch-heading-sigma-deg`で小さい値を指定すれば従来通り有効化できる。今回の変更は「機能を消す」のではなく「実測で一番良かった設定(一様乱択)を既定にする」という位置づけ。今後、分岐判定に使う方位を直近数ステップの移動平均にする等の改善を試す際は、この既定値を変えて再評価すること。
## 2026-08-30(3) [(2)の訂正・正解経路との照合]

[本研究独自] (2)のσスイープはroute_constraint_mode=none(既定、経路帯マスクによる空間制約が実質オフ)のまま計測してしまっていたミスが判明。ユーザーへの聞き取りでpdr_log_0805_1438/1441/1442.csvの実際の歩行経路(start(124,229)付近→turn1(420,230)で北へ→turn2(420,111)で東へ→end(800,114)付近、弧長795px)が分かったため、route_constraint_mode=enforceで正しく再測定し、この正解経路への横方向誤差・弧長進捗で評価し直した(results/20260830_185500_multi_hypothesis_branch_sigma_sweep_enforce.csv)。

結論(重要、(2)の結論を上書きする): **enforceで正しく計測すると、uniform_equiv(σ=100000度、2026-08-16実装時点の完全一様乱択と数学的に等価)が最も良好**。pdr_log_0805_1442は横方向誤差11.4px・弧長484±7.2px(6シードでほぼ完全に一致)・全滅0回。pdr_log_0805_1441も横方向誤差11.0px・弧長195.6±7.4pxと極めて安定。一方、今回実装した方位重み付け分岐選択(choose_branch_by_heading、σ=10〜60のいずれも)はuniform_equivを明確には上回らず、pdr_log_0805_1438では横方向誤差がむしろ悪化(σ=10/20で最大96〜99px、uniform_equivは最大13.2px)。

(2)で「1438・1442は改善、1441のみ構造的な限界」と書いたのは、route_constraint_mode=noneという本来の想定外の条件下での分析結果であり、正しい条件では成立しない。**現時点でchoose_branch_by_heading(方位重み付け分岐選択)に明確な改善効果は確認できていない。** 既定のσ=30は据え置くが、これは「良いと確認されたから」ではなく「悪化とも言い切れないため保留」という位置づけに変更する。次の一手は、σのさらなるチューニングではなく、(a)本当にuniform_equivで十分なのか他の経路(分岐がより明確なケース)でも確認する、(b)分岐判定方位を直近数ステップの移動平均にする案を試す、のいずれかを検討する。

## 2026-08-30(1)(2) [結論は(3)(4)で覆り、既定も戻した]

[本研究独自] 複数経路仮説PFの交差点分岐選択を、一様乱択からガウス重み付き乱択(`choose_branch_by_heading`・`edge_entry_heading`、pdr_route_graph.py新設)へ置き換え、重み広がりσを振って検証した。ただし当時の測定は`route_constraint_mode=none`(地図制約が実質オフ)のまま行っており、正しい条件で測り直した(3)で結論が覆った。既定値も(4)で一様乱択相当へ戻している。関数自体は`--multi-hypothesis-branch-heading-sigma-deg`で今も有効化できる。当時のσスイープ生データはresults/20260830_180500_multi_hypothesis_branch_sigma_sweep.csv(誤条件)とresults/20260830_185500_..._enforce.csv(正しい条件)に残っている。

## 2026-08-16

[本研究独自] コード肥大化対策(3500行超)の一環として、check_sensor_quality.py・pick_landmarks.py・verify_route_graph.pyの3スクリプトに個別コピーされていたfake_args組み立てコードをload_map_config_for_tool()として1箇所にまとめた(apply_map_config()の直後に定義)。3スクリプト側は`pdrmod.load_map_config_for_tool(...)`を呼ぶだけになった。あわせて経路帯マスク・通路グラフ関連の関数をpdr_route_graph.pyへ切り出し(3506行→約2800行)。いずれも関数の中身・挙動は変更していない。

## 2026-08-16

[本研究独自] 複数経路仮説PF(進捗反映版メモ§23、粒子単位のグラフ分岐PF方式で実装することを確認)の第一段階として、build_route_graph_topology()とnearest_edge_position()を追加。simplify_skeleton_graph()の出力(ノード・エッジ)を、各エッジの折れ線を_rdp_simplify()で間引いて区間ごとの方位・長さを事前計算し、各ノードに接続する(エッジid, 進行方向)の一覧(adjacency)を持つ形に変換する。既存のROUTE_POINTS(1本の折れ線を区間インデックスで辿る既存方式)と同じ考え方を、分岐のあるグラフ全体に拡張したもの。この段階ではデータ構造の構築のみで、ParticleFilterPDR・main()の既存処理には一切接続していない(検証専用、既定の挙動は変更しない)。

## 2026-08-16

[本研究独自] 通路グラフのノード整理(進捗反映版メモ§23 Week1後半、build_skeleton_graph()の続き)として、simplify_skeleton_graph()を追加。骨格化特有のノイズ(短い「ヒゲ」枝、本来1つの交差点が数画素のクラスタ化では拾いきれず近接した複数ノードに分裂する現象)を、(1)端点向けの短いエッジの枝刈り、(2)近接する交差点ノード同士の統合(Union-Find)、(3)統合後にエッジが2本だけになった交差点(実際には分岐していない通路の折れ点)の解消、の順で整理する。build_skeleton_graph()と同様、route_source=autoの既定パイプラインにはまだ接続していない(検証専用)。動作確認はverify_route_graph.pyに--simplifyオプションを追加して行い、kanri_4fのホール接続部で密集していた交差点ノードが整理されることを確認した。

## 2026-08-16

[本研究独自] 通路グラフ化(進捗反映版メモ§23 Week1後半)の第一段階として、build_skeleton_graph()を追加。骨格化した経路帯マスクの画素を次数で分類し(次数1=端点、次数3以上=交差点、隣接する交差点画素はクラスタ化して1ノードにまとめる)、それらを結ぶ経路をエッジとするグラフ(nodes/edges)を構築する。extract_ordered_centerline()(最も長い1本の経路だけを木の直径で選ぶ既存実装)とは別関数として独立させており、route_source=autoの既定パイプラインにはまだ接続していない(既定の挙動は変更しない、検証専用)。動作確認は新規verify_route_graph.pyで行い、kanri_4fの交差点候補が実際の建物形状(西側廊下・東側廊下を繋ぐホール)と一致することを可視化で確認した。分岐を含む複数経路仮説(5.7節)での利用は今後の課題のまま。

## 2026-08-16

[本研究独自] 正解位置データによるRMSE等の定量評価(進捗反映版メモ§23のWeek1タスク)に向けた準備として、(1)各検出ステップのtimestampをstep_timestamps(estimated_positionsと1:1対応)として記録し、result_cacheにも保存するようにした。(2)--save-trajectory-csv(既定False)を追加し、有効時のみCSVごとの推定軌跡(timestamp, x_px,y_px)をRESULTS_DIR以下へ保存するようにした。既定では従来通り何も変化しない(新規追加のCLIフラグを指定しない限り出力・ログ・診断値は変わらない)。正解データ側の読み込み・RMSE計算は新規evaluate_accuracy.pyに実装し、こちらは変更していない。

## 2026-08-16

[本研究独自] extract_auto_route_mask()に、大きな部屋の壁際の帯を通路候補から除外するexclude_wide_rooms引数を追加(既定False、既存のroute_source=auto比較実験の結果は変更しない)。半径max_half_width_pxの円盤でfreeをモルフォロジー開放し、その結果(円盤が収まるほど広い領域=部屋の内部)を通路候補から差し引く。CLI--auto-route-exclude-wide-rooms/JSON auto_route_exclude_wide_roomsで有効化。原因調査(直前のコメント訂正2件)を踏まえた対策の実装。

## 2026-08-16 [route_source=autoの「輪」構造についての解釈訂正(2回)]

extract_ordered_centerline()で見つかった「2本の帯が両端付近で連結して見える」構造を、当初「通路網が輪(ループ)」と誤解し、2度にわたりコメントを訂正した。最終的な結論は、(1)西側廊下・東側廊下を繋ぐホール・階段は実在する正しい接続、(2)もう一方はextract_auto_route_mask()が大きな部屋の壁際を通路と区別できないことによる見せかけの経路、という設計限界だった。安全装置(迂回率チェック)の実装・挙動・検証結果に変更はない(詳細はmemo/route_source_auto.md)。

## 2026-08-16

[本研究独自] route_source=autoでも曲がり角連動の方位補正(route_guidance_enabled系)が働くよう、自動抽出した経路帯マスクを細線化(skimage.morphology.skeletonize)して1本の順序付き中心線を抽出し、簡略化(RDP法)した上でROUTE_POINTSとして使う機能を追加(extract_ordered_centerline())。既定は無効(--auto-route-centerline/JSON auto_route_centerline_enabled、無指定ならFalseで従来通りroute_pointsは空のまま)。現在の実測データ(0805系)の通路は分岐のない単純な形状であることを確認済みのため、骨格化後の小さな分岐・突起はノイズとみなし、最も長い経路(木の直径)1本だけを採用する簡易版とした(分岐を含む通路網への対応=§6.2の完全版は今後の課題のまま)。経路帯マスク自体(extract_auto_route_mask)や既定OFF時の挙動は変更していない。新規依存としてscikit-image(skimage)を追加(環境には導入済み、extract_auto_route_mask設計時の「新規ライブラリ依存なし」方針からの変更。詳細はmemo/route_source_auto.md)。

## 2026-08-16

不確実性適応粒子数の4パラメータ(neff_low_ratio/neff_high_ratio/boost_factor/shrink_factor)をCLIから個別に上書きできる引数(--uncertainty-neff-low-ratio等)を追加。挙動自体は変更せず、感度分析(sensitivity_uncertainty_particles.py)がJSON設定ファイルを増やさずにパラメータを振れるようにするための追加のみ。

## 2026-08-15

[本研究独自] 不確実性適応粒子数(進捗反映版メモ.txt §6.5相当)を追加。直前ステップの実効サンプルサイズ(Neff)を粒子数に対する比率(neff_ratio)で見て、移動様態ベースの粒子数(先行研究の枠組み)をさらに増減させる(ParticleFilterPDR.configure_behavior())。--uncertainty-adaptive-particles/--no-uncertainty-adaptive-particles、またはJSON adaptive_pf内のuncertainty_adaptive_particles等で切り替え、既定は無効(既存挙動を維持)。有効化するとNeffという既に計算済みだが未使用だった診断値を初めて粒子数制御に使う。compare_route_source.pyにauto-enforce-unc条件として追加し検証中(詳細はCLAUDE_MEMO.md)。

## 2026-08-15

extract_auto_route_mask()に緩和的膨張オプション(dilation_px、--auto-route-dilation-px/JSON auto_route_dilation_px、既定0px)を追加。「自動マスクが実際の廊下幅に忠実なせいで手動の一様バッファより制約が厳しくなり全滅回数が増える」という仮説を検証する目的だったが、0〜10pxで振っても全滅回数はほぼ変化せず仮説は棄却された(詳細はCLAUDE_MEMO.md)。機能自体は今後のチューニング用に残す(既定0pxで従来通りの挙動)。

## 2026-08-15

[本研究独自] 経路帯マスクを手動route_pointsに頼らず、二値地図から自動抽出できるようにした(--route-source {manual,auto}、既定はmanualで従来通り)。extract_auto_route_mask()が「壁までの距離がauto_route_max_half_width_px(既定20px)以下の移動可能領域」を通路候補とし、そのうち最大連結成分を通路網として採用する(広い部屋は距離が大きく候補から自然に外れる)。座標を一切与えずにkanri_4f.jpgの実廊下形状を高精度に再現できることを可視化で確認済み(詳細はCLAUDE_MEMO.md)。route_source=autoの場合はJSONのroute_pointsを意図的に無視する(手動座標ゼロにするため)ので、route_guidance_enabled()系の曲がり角連動方位補正は現状autoでは働かない(空間的な経路帯制約のみ。今後の課題)。

## 2026-08-15

未使用コードを削除しファイル総行数を縮小。(1) 未使用import(dataclass)を削除。(2) 呼び出し箇所が一つもなかったestimate_step_length_px()(Weinberg法の予備実装)とWEINBERG_K定数を削除。(3) START_X/START_Y(JSON任意設定"start"由来)を削除。実際のPF開始位置はstart_positions.csv/クリック選択で決まっており一切影響していなかった上、ログに実態と異なる開始位置を出力していたため。

## 2026-08-15

地図設定JSONに任意設定"exclude_csv"を追加。別の図(L字経路用など)で使用中のCSVをファイル名指定でこのJSONの実行対象から除外できるようにした(redraw_all_paths()のファイル一覧構築時にフィルタ)。

## 2026-08-15

CLAUDE.mdで指摘されていたpdr_pf_clickstart.pyとの残る差分を解消。(1) --save未指定時にPNGが1枚も保存されない問題を修正し、pdr_pf_clickstart.py同様resultsディレクトリへ実行日時・経路制約モード・乱数シードを含む名前で自動保存するようにした。(2) apply_map_config()がconfig.get(key, 旧デフォルト値)で設定漏れを黙って握り潰していた(ヘッダーの「設定漏れはエラー」という記述と矛盾)問題を、require_config_value()による必須チェックへ統一して解消(pdr_pf_clickstart.py同様の挙動)。(3) --heading-source androidをyaw_deg列の無いCSVに使った際のValueErrorがredraw_all_paths()のループ外まで伝播し、以降のCSVを1枚も処理できずバッチ全体が止まっていた問題を、該当ファイルだけスキップするtry/exceptで解消。(4) [SmartPDR]/[先行研究:移動様態PF]/[本研究独自]の出典タグと、研究的位置づけの説明コメントをpdr_pf_clickstart.pyから移植。

## 2026-08-15

route_constraint_mode=enforceで全粒子の重みが0になった際の自己リカバリ処理に経路マスク条件を追加。従来はbinary_for_pf(物理壁)のみを条件に再配置していたため、全滅からの回復時に粒子群が経路コリドー外(隣室など)へ漏れ出す場合があった(is_in_wall()自体は経路を見ない設計のまま据え置き、診断指標valid_ratio_history/route_ratio_historyの分離も維持)。

## 2026-08-05〜2026-08-06 [初期実装期のまとめ]

この期間に現在の基本機能が出揃った。個別項目は当時の粒度で10件に分かれていたが、いずれも現行仕様として`pdr_pf_improved.py`冒頭の仕様コメントとREADME.mdに反映済みで、変更履歴として個別に参照する必要がなくなったためここへ集約した。

- CSVごとの既知開始位置を地図クリックで登録し、start_positions.csvから再利用
- 地図・PF設定値をJSONへ移し、設定漏れは起動時エラーとして検出
- 経路マスク外を移動不可として扱う処理、route_constraint_modeのnone/prefer/enforce分離
- 計測開始時のセンサ方位を地図座標系へ合わせる初期方位補正
- 方位推定方法の選択(ジャイロ・Madgwick方式 / AndroidのTYPE_ROTATION_VECTOR由来のyaw_deg)
- クリック開始位置に最も近いroute_points線分を初期線分として選ぶ処理
- キャッシュ判定への実行条件(開始位置・経路制約モード・方位設定・経路重み)の追加
- PF診断値(有効粒子率・経路内粒子率・Neff・位置分散)の記録とログ出力
- 結果画像の凡例・タイトル整理(経路制約モードと方位推定方法を画像上へ表示)
