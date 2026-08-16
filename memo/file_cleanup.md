# リポジトリのファイル整理(削除候補フォルダへの移動・決着済み・圧縮版)

2026-08-15実施、2026-08-16に経緯の詳細を圧縮(圧縮ルールは
[CLAUDE_MEMO.md](../CLAUDE_MEMO.md)参照)。削除ではなく`削除候補/`(と
`削除候補/results/`)への移動のみ。**何も削除していない**、最終判断はユーザー。

---

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
2026-08-15時点の`results/20260815_230748_route-enforce-auto+unc_seed-42.png`
(auto-enforce-unc条件、seed=42)が既存の参照可能な1枚として`results/`に残っている。

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
- **`kanri_4f_preview_final3.png`**: 元の平面図→検出壁→二値化の3段階図。卒論3.4節
  「建物地図」にそのまま使える。

### 削除候補に移したが、卒論用に復元候補として要記憶なもの

- **`result_none.png`/`result_prefer.png`**: 「y≈300張り付き」バグと「prefer時に
  部屋へ漏れる」バグの、修正前のスクリーンショット。卒論7.7節「失敗例」の図として
  使える可能性があるため、削除候補フォルダから復元する価値がある(
  →詳細は[pipeline_fixes.md](pipeline_fixes.md)のroute_points修正・preferモードの項目)。
