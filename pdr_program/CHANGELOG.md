# pdr_pf_improved.py 変更履歴

`pdr_pf_improved.py`冒頭コメントの変更履歴が肥大化してきたため、2026-08-16にこのファイルへ切り出した(過去分を全件移動、内容は不変)。新しい変更を記録する際は、このファイルの先頭(最新が上)に追記する。

---

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

## 2026-08-30(2) [(3)により結論が一部訂正されました。上記参照]

[本研究独自] 上記の分岐選択改善について、重み広がりσを10/20/30(既定)/45/60度で振って効果を検証(kanri_4fの3CSV×6シード)。結果はresults/20260830_180500_multi_hypothesis_branch_sigma_sweep.csvに記録(変更前のbaseline行も含む)。
結論: pdr_log_0805_1438はσ>=20でどの値でも終点xの範囲が7.9px前後に収束し、変更前にあった外れ値(seed42で722.9px)が解消された。一方pdr_log_0805_1441はσ=10(ほぼ決定論的な選択)でも159.2pxとむしろ変更前(14.0px、外れ値除く)より悪化しており、σのチューニングだけでは改善しないことを確認した。分岐点到達時点の単発ステップの実測方位だけを参照しているため、センサーノイズにより見かけ上どちらのエッジが近いかがシードごとに入れ替わってしまうことが原因と推定される(候補エッジ同士の方位差が本質的に小さいという当初の仮説とは異なる)。pdr_log_0805_1442はσ=30でのみ改善(43.4px)し他のσでは悪化しており、6シードでは判断材料として不十分。
次善策として、分岐判定に使う方位を単発ステップではなく直近数ステップの移動平均に変えることを検討中(未実装)。既定値はσ=30のまま据え置く(このσだけでは根本解決しないため)。

## 2026-08-30(1)

[本研究独自] 複数経路仮説PFの交差点分岐選択を改善。従来の完全一様乱択(np.random.randint)を、直前の実測方位に近いエッジを優先するガウス重み付き乱択(choose_branch_by_heading・edge_entry_heading、pdr_route_graph.py新設)へ置き換えた。重みの広がりは`--multi-hypothesis-branch-heading-sigma-deg`(既定30.0、JSON設定`multi_hypothesis_branch_heading_sigma_deg`でも可)で調整できる。pdr_pf_improved.py側は`_advance_route_state()`の分岐選択1行を関数呼び出しに置き換えただけで、方位計算そのものはpdr_route_graph.py側の純粋関数に持たせ、コード増大を抑えた。変更前(完全乱択)のベースラインとして、kanri_4fの3CSV×6シード(1,7,42,100,777,2024)で最終推定位置xのばらつきを記録済み(pdr_log_0805_1442が外れ値なしでも385.8〜469.6pxと最大)。変更後、同条件で再検証しばらつきが縮小するか確認する。

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

## 2026-08-16

extract_ordered_centerline()のコメントを再訂正(原因の特定)。kanri_4f_preview_final3.png(生成時の壁検出結果がそのまま残っている確認用画像)と実際の平面図を1px単位で照合した結果、「輪」に見える原因は2つの組み合わせと判明した: (1)西側廊下・東側廊下(高さが異なる)を繋ぐホール・階段は実在する正しい接続、(2)もう一方はextract_auto_route_mask()が大きな部屋の壁際の帯を通路と区別できず、たまたま別の階段の踊り場まで繋がって見える見せかけの経路。壁検出自体(文字を壁と誤認しないための水平・垂直線分抽出)は平面図と照合して正確だったため、原因は文字認識の問題ではなく、通路と部屋の壁際を区別できないマスク生成方式そのものの設計限界だった。安全装置の実装・挙動・検証結果に変更はない(詳細はmemo/route_source_auto.md)。

## 2026-08-16

extract_ordered_centerline()のコメントを訂正。kanri_4f.jpgで単体検証中に発見した「2本の帯が両端付近で連結して見える」構造を、当初は「通路網が輪(ループ)になっている」と誤って解釈していたが、ユーザーから実際の平面図(PDF)で「両端はそれぞれ別の階段(3Fへの階段・屋外階段)であり、通路同士が直接繋がっているわけではない」との訂正を受けた。安全装置(迂回率チェック)自体の実装・挙動・検証結果に変更はないが、その原因についての説明(コメント・memo)を訂正した(詳細はmemo/route_source_auto.md)。

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

## 2026-08-05

CSVごとの既知開始位置を地図クリックで登録し、start_positions.csvから再利用する機能を追加。

## 2026-08-06

JSONで管理する地図・PF設定値をコード先頭の固定値から削除し、設定漏れは起動時にエラーとして検出するように整理。

## 2026-08-06

経路マスク外を移動不可領域として扱えるようにし、部屋側へ推定経路が流れる表示を抑制。設定経路の破線描画も削除。

## 2026-08-06

手動L字経路への依存を比較条件として分離するため、route_constraint_modeのnone、prefer、enforceを追加。

## 2026-08-06

計測開始時のセンサ方位を地図座標系へ合わせる初期方位補正を追加。

## 2026-08-06

方位推定方法として、従来のジャイロ・Madgwick方式と、AndroidのTYPE_ROTATION_VECTORから記録したyaw_degを選択できる機能を追加。

## 2026-08-06

クリックした開始位置に最も近いroute_pointsの線分を選び、その線分を最初の経路線分として使用する処理を追加。

## 2026-08-06

開始位置、経路制約モード、方位推定方法、初期方位、経路重みなどをキャッシュ判定へ追加。

## 2026-08-06

有効粒子率、経路内粒子率、Neff、位置分散を記録し、CSVごとのPF診断値としてログ出力する機能を追加。

## 2026-08-06

結果画像の凡例とタイトルを整理し、経路制約モードと方位推定方法を画像上へ表示。

