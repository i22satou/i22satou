# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project policy (read first)

- **Always respond to the user in Japanese.** All conversation in this project happens in
  Japanese — write replies, questions, and explanations in Japanese regardless of what
  language other context (code comments, this file, paper titles) happens to be in.
- **No permission is required to run scripts or edit files in this repository.** The user
  (owner of this graduation-research project) has granted standing authorization for both
  execution and modification within this repo. Proceed directly instead of asking to confirm
  each run/edit; still use judgment for destructive actions outside this repo (deleting files
  you didn't create, pushing to remotes, etc.), which are unaffected by this policy.
- **The file to modify going forward is `pdr_pf_improved.py`.** As of 2026-08-15 this
  replaces `pdr_pf_clickstart.py` as the active script — implement new features and fixes
  there unless the user explicitly names a different file. `pdr_pf_clickstart.py` still has
  the auto-save-PNG behavior, changelog convention, and `[SmartPDR]`/`[先行研究:移動様態PF]`/
  `[本研究独自]` origin tags described below; when work moves to `pdr_pf_improved.py`, apply
  (or port) the same conventions there rather than assuming they already exist — see the
  "Script lineage" divergence notes for what `pdr_pf_improved.py` is currently missing.
- **Every run of the active script must leave a saved result PNG.** In `pdr_pf_clickstart.py`,
  `redraw_all_paths()` always calls `fig.savefig(...)` — if `--save PATH` is given it saves
  there, otherwise it auto-saves to `results/<timestamp>_route-<mode>_seed-<seed>.png` (see
  `RESULTS_DIR`). Bring this same behavior to `pdr_pf_improved.py` before/while making other
  changes there, and don't reintroduce a code path where a run produces no image — these PNGs
  are the evidence trail for the thesis (see 進捗メモ §15, §21).
- **CSVファイルを読み込む際は、必ずそのままのデータを使用すること。いかなる場合でも
  `pdr_log_*.csv` や `start_positions.csv` など入力CSVファイルの中身を書き換えたり、
  値を補正・間引き・並べ替え・削除したりしてはならない。** 前処理・フィルタリング・
  補間などの変換はすべてプログラム内のメモリ上の処理として行い、読み込んだCSVファイル
  自体には一切変更を加えないこと(`start_positions.csv`への新規開始位置の追記など、
  プログラムの既存の意図された書き込み処理は例外とする)。これはIMUログが実験の生データ
  であり、卒論の証拠資料としての完全性を保つ必要があるため。
- **When you change the active script, add a new dated entry at the very top of its header
  changelog comment** (`# - YYYY-MM-DD: ...`, above the existing entries), describing what
  changed and why. This is the existing convention in both `pdr_pf_clickstart.py` and
  `pdr_pf_improved.py` — keep following it so the in-file history stays usable as a record of
  the research process.
- Convert relative dates the user gives you ("today", "先週") to absolute dates using the
  session's current date before writing them into changelog entries or filenames.
- **Before making any change to the program, or when checking past history/decisions, consult
  all three of**: this file (CLAUDE.md), `進捗反映版メモ.txt` (see "Progress memo" section
  below), and [CLAUDE_MEMO.md](CLAUDE_MEMO.md) (a separate file in this same directory).
  `CLAUDE_MEMO.md` records session-to-session investigation results and the *reasoning*
  behind decisions (not just "what changed", which the `.py` changelog comments already
  cover) — things like why a config value was set the way it is, or what was ruled out and
  why. Add a new dated entry at the top of `CLAUDE_MEMO.md` (same convention as the `.py`
  changelog: newest first) whenever you finish an investigation or make a non-trivial
  decision, so the next session doesn't have to re-derive it.

## Project overview

This is a 卒業研究 (undergraduate graduation research) codebase for indoor pedestrian
localization: it fuses **PDR (Pedestrian Dead Reckoning)** from phone IMU logs with an
**adaptive Particle Filter (PF)** constrained to a building floorplan, and visualizes the
estimated walking trajectory on the map in real time. All comments, docstrings, and log
messages are written in Japanese; keep that convention when editing.

There is no package manifest (no `requirements.txt` / `pyproject.toml`), no test suite, and
no lint config. Treat dependencies as inferred from imports (see below) and verify changes
by actually running the relevant script against sample data/maps in this repo.

## Reference papers and research positioning

This project's PF/PDR design is built on two papers, and the thesis needs to clearly separate
what came from them versus what this research adds. `pdr_pf_clickstart.py`'s header comment
(under "本プログラムの研究的位置づけ") and inline tags carry this mapping in the code itself
— keep both in sync when the logic changes:

- **`[SmartPDR]`** — *SmartPDR: Smartphone-Based Pedestrian Dead Reckoning for Indoor
  Localization*. Basis for the HPF/LPF step-acceleration signal, peak/valley/slope step
  detection (`detect_steps_smartpdr`), and the 4th-root/log step-length formula
  (`estimate_smartpdr_step_length_px`, `ROOT_BETA`/`ROOT_GAMMA`/`LOG_BETA`/`LOG_GAMMA`).
- **`[先行研究:移動様態PF]`** — 秋山高行ほか「移動様態に応じたパーティクルフィルタによる
  歩行者自律測位方式の提案と評価」(FIT2013). Basis for classifying steps into
  直進/屈折/滞留 (`MoveBehavior`) and varying particle count + noise variance per state
  (`behavior_parameters`). The original paper uses a simple ≥30°/8s heading-change threshold,
  fixed particle counts (straight=10, turning=20), and a binary (0/1) wall weight — noted here
  because this codebase deliberately diverges from all three (see below).
- **`[本研究独自]`** (original contributions, present in neither paper) — the parts to foreground
  in 第5章 (提案方式) and 第2章6節 (先行研究との違い) of the thesis:
  - Continuous, distance-transform-based wall likelihood (`dist_map`, in `ParticleFilterPDR.update`)
    instead of the papers' binary passable/blocked weight.
  - Turning detection with 75th-percentile yaw rate, an AND condition on heading-change +
    yaw-rate, and enter/exit hysteresis (`detect_move_behavior`) instead of the papers' single
    threshold — reduces false TURNING triggers from hand tremor.
  - Map-derived route corridor and route-segment-linked heading correction
    (`route_points`, `build_route_mask`, `route_guidance_enabled`, `get_route_segment_heading`,
    `correct_heading_with_route_segment`, `advance_route_segment`, `is_near_route_corner`,
    `route_constraint_mode`). **Neither paper uses map corridor/topology information at all** —
    this is the core of what this research adds ("地図形状を用いた適応制御").
  - Map-scale-adapted particle counts (250/600/100, vs. the paper's 10/20) with
    weight-based resampling on resize (`resize_particle_set`).
  - Experiment infrastructure with no equivalent in either paper: click-to-register start
    positions (`start_positions.csv`), folder-watch auto-redraw (`CSVHandler`), per-run PNG
    archiving (see policy above).
- **Known limitation to state honestly in the thesis**: `route_constraint_mode` in
  `prefer`/`enforce` uses a *manually specified* near-correct route (`route_points`), so
  it's a comparison baseline (進捗メモ's 方式D), not the final proposed method. The memo's
  §4.1/§5.3/§20.2 describe reducing this dependency (auto-extracted corridor centerlines,
  topology graph, multiple route hypotheses) as the next major implementation step — none of
  that is implemented yet (see progress memo below for exact status per feature).

## Progress memo — consult before planning any research/thesis-shaping work

`卒業研究/研究計画系/進捗反映版メモ.txt` (absolute path:
`/Users/soma/Library/CloudStorage/OneDrive-独立行政法人国立高等専門学校機構/卒業研究/研究計画系/進捗反映版メモ.txt`,
outside this git repo) is the authoritative design/status document for this thesis. It contains:
- §1: latest experiment log entries (dated, most recent first).
- §5: this research's proposed direction and its differences from the two papers above.
- §7–§11: proposed pipeline, comparison methods (方式A〜E), evaluation metrics, experiment plan.
- §12–§14: thesis title candidates, full **chapter/section structure** for the thesis, and the
  figures/tables to prepare per chapter.
- §18–§19: a feature-by-feature and chapter-by-section **progress checklist**
  (【完了】/【一部完了】/【未完了】/【要確認】) — this is the ground truth for "what's already
  done vs. still needed" and should be checked (and updated) whenever asked to plan next steps,
  scope a feature, or figure out what a thesis section should currently say.
- §20–§22: prioritized work queue and "definition of done" for the program, the experiments,
  and the thesis.

When asked to plan or implement research features, cross-check against this memo's §18
progress table rather than guessing status from the code alone — several "implemented"
mechanisms (e.g. Neff, position variance) are computed but not yet *used* anywhere (flagged
【一部完了】), which the memo makes explicit and the code alone does not.

## Claude Codeメモ

セッションをまたいだ調査結果・意思決定の理由は、この節ではなく別ファイル
[CLAUDE_MEMO.md](CLAUDE_MEMO.md) に記録する(「何を変えたか」は各`.py`の変更履歴
コメント、「なぜそう判断したか」は`CLAUDE_MEMO.md`、という役割分担)。参照・追記の
ルールは上の「Project policy」節を参照。

## Running the code

Core third-party dependencies (install via pip, no pinned versions in-repo):
`numpy pandas matplotlib scipy pillow opencv-python japanize-matplotlib watchdog ahrs`

- `opencv-python` (`cv2`) is only used by the older `test*.py` / `Cross.py` / `Lmap.py` scripts.
- `japanize_matplotlib`, `watchdog`, and `ahrs` are imported in `try/except` blocks in the
  main scripts and are optional — but folder-watch mode (the default run mode) hard-requires
  `watchdog` and will raise `ImportError` without it.

Run the current main program against the bundled management-building map config:

```bash
python pdr_pf_clickstart.py --map-config map_configs/kanri_4f.json --no-watch --no-show --seed 42
```

`--save` is optional — omit it and a result PNG is still written automatically under
`results/` (see Project policy above). All CSVs currently in the configured `data_dir` already
have registered start positions in `start_positions.csv`, so this runs non-interactively.

Common flags (see `parse_args()` in whichever main script you're running): `--data-dir`,
`--map`, `--seed`, `--step-gain`, `--pf-erosion-radius-px`, `--route-constraint-mode
{none,prefer,enforce}`, `--no-watch` (single render instead of live folder watch),
`--no-show` (headless, pairs with `--save`).

Utility scripts:
- `map_binarizer.py` — CLI that turns an architectural floorplan image into the white
  (passable) / black (wall) binary map the PF scripts consume, via `map_processing.py`.
- `measure_map_scale.py` — interactive: click two points on a map image to derive
  `scale_px_per_m` for a map config JSON.
- `Lmap.py` / `Cross.py` — generate synthetic L-shaped / cross-shaped test maps.
- `Trajectory.py` — plots raw `pdr_log_*.csv` trajectories without a PF.

## Architecture

### Script lineage — read this before editing any of the big files

`test3.py` → `test4.py` → … → `test10.py` → `test11_gemini.py` are successive prototypes,
each one a full copy-and-extend of the previous (not imports/inheritance). They were fixed
up by different AI tools in sequence (see header comments, e.g. "使用AI: GPT-5 mini" in
`adaptive_behavior_particle_filter_pdr_route_fixed.py`). Treat everything at or below
`test11_gemini.py` as historical/reference only unless the user explicitly asks about one.

The current working scripts are three large (1500–2000 line) near-duplicate single-file
programs that all implement the same pipeline but have **drifted independently** and are not
kept in sync:
- `pdr_pf_improved.py` — **the active script as of 2026-08-15** (see Project policy above);
  implement new work here. It's the more feature-complete branch (initial-heading calibration,
  selectable heading source `gyro`/`android`, PF diagnostic logging, route-segment
  auto-selection from click position). As of 2026-08-15 it has been brought to parity with
  `pdr_pf_clickstart.py` on auto-save-PNG behavior, changelog convention, and
  `[SmartPDR]`/`[先行研究:移動様態PF]`/`[本研究独自]` origin tags (see
  [CLAUDE_MEMO.md](CLAUDE_MEMO.md) and the file's own changelog for what changed); the
  `enforce`-mode and config-loading
  gaps described below have also been narrowed — check the current bullets, not just this
  summary, before assuming either script's behavior.
- `pdr_pf_clickstart.py` — the previously-active script; kept as reference, but no longer the
  target for new changes unless the user says otherwise.
- `adaptive_behavior_particle_filter_pdr_route_fixed.py` — an earlier "fixed" iteration.

**Before changing PF/heading/route logic, check which of these three files the user means**
— a fix applied to one will not appear in the others, and their in-file changelog comments
at the top of each file are the best record of what has already diverged. A few concrete
divergences worth knowing before touching `enforce` mode or config loading:
- Both scripts' `apply_map_config()` now use `require_config_value()` and raise if a required
  key is missing from the map config JSON (`pdr_pf_improved.py` was fixed on 2026-08-15 — it
  used to silently fall back to hardcoded defaults via `config.get(key, default)`, contradicting
  its own header comment; that contradiction is resolved now).
- `pdr_pf_clickstart.py`'s `ParticleFilterPDR.is_in_wall()` treats off-route pixels as walls
  when `route_constraint_mode == "enforce"` (a hard constraint enforced every step, including
  inside `path_hits_wall()`). `pdr_pf_improved.py`'s `is_in_wall()` deliberately does **not**
  check `route_mask` — off-route particles are zeroed via the weight term each step instead
  (numerically equivalent in the common case where at least one particle stays on-route). This
  is an intentional, narrower fix, not full parity: as of 2026-08-15 the all-particles-extinct
  recovery path *does* respect `route_mask` in `enforce` mode (previously it didn't, letting
  the particle cloud leak into off-route rooms after a collapse and stay there) — see
  [CLAUDE_MEMO.md](CLAUDE_MEMO.md) for why the narrower fix (recovery-only) was chosen over
  full parity.
- `pdr_pf_improved.py`'s `estimate_initial_sensor_heading()` raises `ValueError` when
  `--heading-source android` is used against a CSV without a `yaw_deg` column. As of
  2026-08-15 this is caught in `redraw_all_paths()`'s per-file loop (the offending CSV is
  skipped with a warning; the batch continues) — it no longer aborts the whole batch. The real
  `data_dir` configured in `map_configs/kanri_4f.json` still has a mix of old CSVs
  (`pdr_log_0410_*`, `_0414_*`, `_0624_*`) without `yaw_deg` and newer ones (`_0805_*`) with
  it, so `--heading-source android` against that directory will still skip the old ones.

### Common pipeline (in the three main scripts)

1. `parse_args()` → `load_map_config()` / `apply_map_config()`: load a per-map JSON config
   (`map_configs/*.json`) into module-level globals (see the divergence note above re:
   required vs. soft-fallback config loading between scripts).
2. `load_preprocessed_map()` — loads the binary map PNG, builds an eroded version for PF
   collision checks (`pf_erosion_radius_px`) and a distance-transform map for soft wall
   weighting `[本研究独自]`. `build_route_mask()` rasterizes `route_points` from the config
   into a route corridor mask used by `route_constraint_mode` `[本研究独自]`.
3. Folder watch: a `watchdog` `Observer` watches `data_dir` for `pdr_log_*.csv` files;
   `CSVHandler` sets an event flag, and `redraw_all_paths()` re-runs the full pipeline for
   every CSV and redraws the matplotlib figure. `PDRResultCache` short-circuits recompute for
   unchanged files (by mtime+size, plus run-condition context in `pdr_pf_improved.py`).
4. Per-CSV start position `[本研究独自]`: `get_or_select_start_position()` reuses a saved
   position from `start_positions.csv` (columns `file_name,start_x,start_y`) or prompts a
   one-click map selection (`select_start_position_on_map()`) and persists it via
   `save_start_position()`.
5. Signal processing per CSV: `validate_log()` checks required IMU columns
   (`timestamp, acc_x/y/z, gyro_x/y/z`) → step detection/step-length estimation `[SmartPDR]`
   (`detect_steps_smartpdr`, `estimate_smartpdr_step_length_px`) →
   heading fusion (gyro + accel via Madgwick from the `ahrs` package, optional magnetometer,
   optional Android `yaw_deg` rotation-vector source in the `_improved` variant).
6. `detect_move_behavior()` classifies each step as `MoveBehavior.STOPPED / STRAIGHT /
   TURNING` `[先行研究:移動様態PF]` for the 3-state idea, `[本研究独自]` for the
   75th-percentile yaw-rate + AND-condition + hysteresis refinements that reduce false
   TURNING triggers from hand tremor (the original paper uses a single fixed threshold).
7. `ParticleFilterPDR` (the OOP PF core, shared shape across the three scripts):
   `configure_behavior()` resizes the particle set and swaps step/angle noise sigmas per
   `MoveBehavior` (via `behavior_parameters()` / the `adaptive_pf` config block —
   `[先行研究:移動様態PF]` for the mechanism, `[本研究独自]` for the map-scale-adapted
   250/600/100 particle counts vs. the paper's 10/20); `update()` propagates particles,
   rejects paths that cross a wall pixel (`path_hits_wall`, sub-pixel-stepped to avoid
   tunneling), weights by a **continuous** distance-to-wall likelihood `[本研究独自]` and
   (if `route_constraint_mode` is `prefer`/`enforce`) by route-mask membership `[本研究独自]`,
   then resamples (`resample_if_needed`) or self-recovers by respawning particles near the
   last mean position if every particle's weight collapses to zero.
8. Result: a weight-averaged, buffer-smoothed position per step, plotted over the binary map,
   with the run's `route_constraint_mode` shown in the plot title, and a PNG always saved
   (see Project policy above).

### Map config JSON (`map_configs/*.json`)

Required top-level keys: `map_image`, `data_dir`, `scale_px_per_m`, `step_gain`,
`pf_erosion_radius_px`, `gyro_unit` (`rad`/`deg`), `wall_weight_sigma`, `wall_weight_floor`,
`off_route_weight`, `route_width_px`, `route_constraint_mode`, `route_heading_weight`,
`route_corner_threshold_px`, `route_points` (polyline in map pixel coords), plus a nested
`adaptive_pf` block with per-behavior particle counts and step/angle noise sigmas. `data_dir`
in the shipped `kanri_4f.json` points outside this repo (a OneDrive/Google Drive path) — that
is expected, since raw `pdr_log_*.csv` IMU recordings are not checked into this repo.

`route_constraint_mode` semantics (also documented in the scripts' own header comments):
`none` = ignore `route_points` entirely (sensor+wall-only PF); `prefer` = upweight particles
near the manual route without forbidding others; `enforce` = treat outside-route pixels as
walls in `pdr_pf_clickstart.py` (a strong comparison baseline, not the "real" estimator) —
see the divergence note above for how `pdr_pf_improved.py` implements `enforce` differently.

### Input CSV format

`pdr_log_*.csv` files require `timestamp, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z`
columns; optional magnetometer columns enable magnetic heading fusion, and `yaw_deg` enables
the Android-rotation-vector heading source in `pdr_pf_improved.py` (see the caveat about mixed
old/new CSVs above).
