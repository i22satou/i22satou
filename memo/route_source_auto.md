# route_source=auto の実装・比較・限界調査(決着済み・2026-09-18圧縮)

関連: [uncertainty_particles.md](uncertainty_particles.md)(auto-enforceに不確実性適応を重ねた条件)、
[sensor_mystery.md](sensor_mystery.md)・[step_length_calibration.md](step_length_calibration.md)
(1441/1442の性能差の真因)。圧縮前の全文(2026-08-15〜08-30の経緯)はgit履歴にある。

**注意: 下記の数値はすべて2026-09-02以前(総距離校正`target_distance_px=815`あり)の条件のもので、
現在の結果と直接比較できない。**

## 結論

- `extract_auto_route_mask`: 距離変換で壁から`auto_route_max_half_width_px`(既定20px)以内の
  移動可能領域を通路候補とし、最大連結成分を採用する。`route_source=auto`ではJSONの
  `route_points`を意図的に無視する(空間制約を完全に地図由来にするため)。
- **設計限界**: 通路と「大きな部屋の壁際の帯」を区別できない(`select_indoor_free_space()`が
  部屋も白領域として残すため)。kanri_4fでは、西側廊下と東側廊下を実際につなぐ中央ホール
  (x≈408〜430)に加えて、東端(x≈945〜955)で部屋の壁際の帯が屋外階段の踊り場とつながり、
  見かけ上の輪ができていた。
- `exclude_wide_rooms`(既定OFF): 円盤によるモルフォロジー開放で広い部屋を除外する。
  `--auto-route-centerline`と併用すると、kanri_4fで曲がり角連動の方位補正が初めて機能した。
  6シード×3CSVで全滅回数15.56→12.94回(-16.8%)、終点x誤差339.9→276.9px(-18.5%)
  (`results/20260816_140200_exclude_wide_rooms_verification.csv`)。本物の広い合流点まで削る
  副作用があるため既定OFFのまま。
- 中心線抽出(`extract_ordered_centerline`): skeletonize → 木の直径探索 → RDP簡略化。
  結果を`ROUTE_POINTS`へ代入し、manualと同じ方位補正コードパスを使う。迂回率が2.5倍を
  超えたら誤検出とみなして`ROUTE_POINTS=[]`へフォールバックする安全装置あり(内部定数)。
- none/manual-enforce/auto-enforceの比較(seed=42): 平均ではautoがmanualより悪い
  (全滅回数15.7 vs 11.7、終点x誤差345.3 vs 268.9px)。ただし1442はautoのほうが良い。
  手動マスクは半径18pxのバッファで幅が約37pxになり、実際の廊下(24〜28px)より広い
  「下駄」を履いていた。これはmanual方式の隠れた有利さで、卒論の考察(8.6節)に書くべき点。
- 汎用化検証(他の建物地図)は、実在する地図がないため見送った。卒論には「今後の課題」と
  書く(2026-08-30、ユーザーの了承済み)。

## 却下した仮説・試み

- 「通路網全体が輪になっている」「両端は無関係な階段」→ どちらも誤り(上記の設計限界が正解)。
- 「autoが劣るのはマスク幅が不足しているから」→ `auto_route_dilation_px`を0/5/8/10pxと
  変えても全滅回数はほぼ変わらず、棄却。機能は残してある。
- 「1441/1442の差は生センサーの質が原因」→ 誤り。真因はPDR上流の距離推定(sensor_mystery.md)。
- L字合成地図(`l_map.json`)を第二の検証環境にする → 部屋が存在しない地図のため、auto抽出の
  効果を原理的に測れず不採用(none条件とauto-enforce条件の診断値が完全に一致した)。
  ファイルは残すが、卒論の比較実験には使わない。サンプル数の不足はkanri_4Fでの追加計測で補う。

## 落とし穴(現在も有効)

- **除外半径を20pxにすると中央ホールまで除外される**。通路網が分断され、開始位置のある
  西側廊下がマスクから消え、enforce条件では開始直後から全滅し続ける。既定の除外半径は
  `max(max_half_width_px*1.5, max_half_width_px+10)`(20pxなら30px。30px付近が境界)。
- `--route-source auto`単体では制約がかからない。`--route-constraint-mode enforce`を併用する。
- 複数シードはPF内部の乱数のばらつきを見ているだけで、独立した歩行サンプルは3本しかない。
- 0805系以外の古いCSV(0410/0414/0624)には`yaw_deg`列がないため、`--heading-source gyro`が必要。
- `compare_route_source.py`の終点x誤差は、map_configごとの既定値辞書(`DEFAULT_EXPECTED_ENDPOINT_X`)
  にある地図だけで計算される(未知の地図では自動的にスキップ)。
- 新規依存として`scikit-image`を使っている(中心線抽出のため)。
