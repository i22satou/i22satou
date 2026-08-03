#!/usr/bin/env bash
set -eu

# i22satou フォルダー内で実行してください。
# 安全のため、研究ファイルは削除せず移動します。
# __pycache__ と .pyc だけを 99_削除候補/キャッシュ へ退避します。

ROOT="$(pwd)"
if [ "$(basename "$ROOT")" != "i22satou" ]; then
  echo "エラー: i22satou フォルダー内で実行してください。"
  exit 1
fi

mkdir -p \
  "01_現在使用中/プログラム" \
  "01_現在使用中/設定" \
  "01_現在使用中/地図" \
  "02_計測データ" \
  "03_実験結果/画像" \
  "03_実験結果/ログ" \
  "04_解析・補助ツール" \
  "05_過去バージョン/初期実験" \
  "05_過去バージョン/管理棟PF" \
  "06_参考資料" \
  "99_削除候補/キャッシュ" \
  "99_削除候補/要確認"

move_if_exists() {
  src="$1"
  dst="$2"
  if [ -e "$src" ]; then
    mv "$src" "$dst/"
    echo "移動: $src -> $dst/"
  fi
}

# 現行版
move_if_exists "Copilot/adaptive_behavior_particle_filter_pdr.py" "01_現在使用中/プログラム"
if [ -e "Copilot/kanri_4f_adaptive_pf.json" ]; then
  mv "Copilot/kanri_4f_adaptive_pf.json" "01_現在使用中/プログラム/map_configs/kanri_4f.json"
  echo "移動・改名: Copilot/kanri_4f_adaptive_pf.json -> 01_現在使用中/プログラム/map_configs/kanri_4f.json"
fi
move_if_exists "test9_final_project/kanri_4f_binary_final3.png" "01_現在使用中/地図"
move_if_exists "test9_final_project/kanri_4f_preview_final3.png" "01_現在使用中/地図"

# 計測CSV
for f in pdr_log_*.csv; do
  [ -e "$f" ] && mv "$f" "02_計測データ/"
done

# 実験結果
move_if_exists "test9_final_project/test10_result.png" "03_実験結果/画像"
move_if_exists "test9_final_project/test11_gemini_result.png" "03_実験結果/画像"
move_if_exists "test9_final_project/run_output.txt" "03_実験結果/ログ"

# 補助ツール
for f in \
  "test9_final_project/map_processing.py" \
  "test9_final_project/map_binarizer.py" \
  "test9_final_project/measure_map_scale.py" \
  "Trajectory.py" "ErrorRate.py" "movie.py" "real.py"; do
  move_if_exists "$f" "04_解析・補助ツール"
done

# 初期実験・旧版
for f in Cross.py Lmap.py test3.py test4.py test5.py test6.py test7.py test8.py pdr_particle_filter.py; do
  move_if_exists "$f" "05_過去バージョン/初期実験"
done

for f in \
  "test9_final_project/test9.py" \
  "test9_final_project/test10.py" \
  "test9_final_project/test11_gemini.py" \
  "test9_final_project/map_configs/kanri_4f.json" \
  "map_configs/l_map.json"; do
  move_if_exists "$f" "05_過去バージョン/管理棟PF"
done

# 旧地図・初期可視化ファイルは過去版へ
for f in L_map.png cross_map.png L.png Real.png; do
  move_if_exists "$f" "05_過去バージョン/初期実験"
done

# キャッシュを退避
find . -type f -name '*.pyc' -exec mv {} "99_削除候補/キャッシュ/" \; 2>/dev/null || true
find . -type d -name '__pycache__' -empty -delete 2>/dev/null || true

# 空のインベントリは退避
if [ -f "file_inventory.txt" ] && [ ! -s "file_inventory.txt" ]; then
  mv "file_inventory.txt" "99_削除候補/要確認/"
fi

# 空になった既存フォルダーだけ削除
rmdir "Copilot" 2>/dev/null || true
rmdir "test9_final_project/map_configs" 2>/dev/null || true
rmdir "test9_final_project" 2>/dev/null || true
rmdir "map_configs" 2>/dev/null || true

cat <<'EOF'

整理が完了しました。
次に必ず確認してください。
1. 00_メモ_このフォルダーについて.md を読む
2. JSON内の data_dir がMac上のGoogle Drive同期先になっているか確認する
3. 次のコマンドで動作確認する:
   python 01_現在使用中/プログラム/adaptive_behavior_particle_filter_pdr.py --no-watch
4. 問題がなければ 99_削除候補/キャッシュ を削除する
EOF
