# RTK demo5 parity — preserve lineage-seed reject guard A/B plan (Codex 実作業用)

更新日: 2026-04-17
作業ブランチ: `codex/rtk-demo5-parity` (dirty state 前提、直前 3 plan の成果がそのまま残っている状態)
計画担当: Claude / 実作業担当: Codex

## この文書の使い方

- 直前 3 plan の結論:
  - `notes/2026-04-17_rtk_run2_lineage_plan.md`: telemetry 追加 + `177764.8` の lineage 139→140 特定
  - `notes/2026-04-17_rtk_run2_lineage_guard_ab_plan.md`: 177764.8 直接 block = A2 で -30 (不採用)
  - `notes/2026-04-17_rtk_177057_early_divergence_plan.md`: 因果 root は **177057.4/177057.6 の preserved-prior partial/variance_drop low-overlap advance**。177764.8 は downstream symptom。
- 本 plan は直前 plan が出した hypothesis を guard として実装し A/B で検証する。
- 仮説の要点: `accepted_preserved_prior_hold_state == 1` (= `--min-holdset-expand-overlap-ratio` による preserve 発火) が `partial/variance_drop` + `overlap < 0.5` + `lineage ID advance` の 3 条件で lineage seed になっている。この組み合わせに限って preserve を reject する。
- preserve の既定機能 (expand / 6→4 contraction / severe rewrite 等) は残す。lineage-seed パターンだけを追加で抜く。

## 先に結論 (提案する新 guard)

- 新 option: `--preserve-skip-lineage-advance-variance-drop` (bool、default off)
- 発火条件 (すべて true のとき preserve を skip して通常の accepted overwrite を使う):
  - preserve が発火する条件を満たした epoch (= そのままなら `accepted_preserved_prior_hold_state = 1` になる)
  - `hold_used_preserved_backup == 0` (backup-use 経由でない)
  - `accepted_fix_source == PARTIAL`
  - `accepted_candidate_path == VARIANCE_DROP`
  - `accepted_overlap_ratio < R` (R: `--preserve-lineage-advance-overlap-threshold`、default 0.5)
  - このまま preserve すると lineage ID が advance する (= 直前 held set と overlap_ratio < 0.5 を同時に満たす)
- skip されたときは preserve を行わず通常経路 (held set を accepted_after で上書き + lineage ID 進める) に戻す。
- 既定 off で solver 既定動作 (`satguard12`) も A1 既定動作も変えない。

## Codex 次作業: 1 ステップだけ

### やること

1. 上記条件で preserve を skip する guard を実装する (preserve path の条件分岐を 1 箇所広げるだけ)。
2. run2 full / run1 full / run3 `1910` の 3 本で A3 (A1 + 新 guard on) を取り、A1 / A2 と比較する。
3. 採用判断の 1 行を plan 末尾に追記する。

### 禁止事項

- A2 で入れた `--max-lineage-advance-overlap-ratio` 系 (direct-overwrite block) の挙動を変えない。既定 off のまま残置。本 plan の A3 では A2 option は使わない。
- pair 名 hardcode 禁止
- `--preserve-larger-hold-set` 再導入禁止
- 速度ゲート / SPP 品質ゲート / stale fix-history gate 系を再導入しない
- `run3 full` を回さない (1910 のみ)
- CI/CD・PPP・Docker・workflow を同時に触らない (`.github/**`、`scripts/ci/**` は touch 禁止)
- `rtk.hpp` を `sed -i` で書き換えない
- solver 既定挙動を変える変更を入れない (新 guard も default-off)

## ステップ 1: ビルド確認

```bash
cd /media/sasaki/aiueo/ai_coding_ws/rtklib_v2_rtk_ws/gnssplusplus-library
cmake --build build --target gnss_solve run_tests -j4
./build/tests/run_tests --gtest_filter='RTKValidationTest.*:RTKLegacyCompatibilityStandaloneTest.*'
```

直前 plan 群の telemetry + A2 prototype が入った状態で、既存 test が pass することを確認する。通らない場合は先に進まない。

## ステップ 2: 新 guard の実装

### 2.1 config 追加

`include/libgnss++/algorithms/rtk.hpp` の `RTKConfig` に以下を追加 (既存フィールドの末尾に追加、構造体並びは崩さない):

- `bool preserve_skip_lineage_advance_variance_drop = false;` (default off)
- `double preserve_lineage_advance_overlap_threshold = 0.5;` (R: 発火する overlap 上限)

### 2.2 CLI 追加

`apps/gnss_solve.cpp` に以下を追加 (help と parse 両方):

- `--preserve-skip-lineage-advance-variance-drop`: bool flag、true 時 `preserve_skip_lineage_advance_variance_drop = true`。
- `--preserve-lineage-advance-overlap-threshold <v>`: `v in (0, 1]`、範囲外で `argumentError`。

### 2.3 guard の発火場所

`src/algorithms/rtk.cpp` の preserve 判定が true に傾いている箇所 (= `accepted_preserved_prior_hold_state = true` を書く直前) に以下を差し込む。直前 plan 3 で見た preserve は `--min-holdset-expand-overlap-ratio` 系の条件で発火しているので、その条件が true になった直後の if ブロックに追加する。

- 追加条件 (すべて true のとき preserve を skip):
  - `rtk_config_.preserve_skip_lineage_advance_variance_drop == true`
  - `!debug_telemetry_.hold_used_preserved_backup`
  - `source == DebugTelemetry::FixSource::PARTIAL`
  - `path == DebugTelemetry::CandidatePath::VARIANCE_DROP`
  - `overlap_ratio < rtk_config_.preserve_lineage_advance_overlap_threshold`
- skip したときの挙動:
  - preserve を行わず、通常の accepted overwrite path に戻る (= held set を accepted_after で上書き、lineage ID を進める)
  - `debug_telemetry_.accepted_preserved_prior_hold_state = false`
  - `debug_telemetry_.preserve_skip_lineage_advance_fired = true`

### 2.4 telemetry 追加

`include/libgnss++/algorithms/rtk.hpp` の `DebugTelemetry` に `bool preserve_skip_lineage_advance_fired = false;` を 1 本追加。`apps/gnss_solve.cpp` の CSV header と出力列に同名カラムを追加する。

### 2.5 unit test

`tests/test_rtk_legacy.cpp` に下記 3 ケースを追加:

- guard off (既定) で「preserve 発火 + partial + variance_drop + overlap<0.5 + non-backup」の全条件を満たす入力が preserve されること (= `accepted_preserved_prior_hold_state == true`、`preserve_skip_lineage_advance_fired == false`)
- guard on で同条件入力が skip され通常 overwrite に戻ること (= `accepted_preserved_prior_hold_state == false`、`preserve_skip_lineage_advance_fired == true`、`lineage_id` が進む)
- guard on でも `hold_used_preserved_backup == true` なら preserve されること (= backup-use 経由は対象外)

`RTKLegacyCompatibilityStandaloneTest.*` 名前空間に置く。pair 名 hardcode 禁止。

### 2.6 ビルド + test

```bash
cmake --build build --target gnss_solve run_tests -j4
./build/tests/run_tests --gtest_filter='RTKValidationTest.*:RTKLegacyCompatibilityStandaloneTest.*'
```

不通過時は 1 段階だけ修正してリトライ。2 回失敗したら plan 末尾に状況追記して終了。

## ステップ 3: run2 full / run1 full / run3 1910 で A3 を取る

参照値 (再掲):

- A0 satguard12 単独: run2 full Fix = 3474
- A1 backup-on + narrow 4x4: run2 full Fix = 3685 (既存 CSV 再利用)
- A2 A1 + lineage-advance overlap guard: run2 full Fix = 3655 (不採用)
- **A3** = A1 + `--preserve-skip-lineage-advance-variance-drop` + `--preserve-lineage-advance-overlap-threshold 0.5`

A3 run2 full:

```bash
./build/apps/gnss_solve \
  --rover /media/sasaki/aiueo/ai_coding_ws/datasets/PPC-Dataset-data/tokyo/run2/rover.obs \
  --base  /media/sasaki/aiueo/ai_coding_ws/datasets/PPC-Dataset-data/tokyo/run2/base.obs \
  --nav   /media/sasaki/aiueo/ai_coding_ws/datasets/PPC-Dataset-data/tokyo/run2/base.nav \
  --mode kinematic --preset low-cost --no-arfilter --no-wide-lane \
  --ratio 2.0 --max-postfix-rms 1.0 \
  --min-hold-count 1 --hold-ratio-threshold 1.0 \
  --glonass-ar autocal --max-hold-div 8 \
  --max-variance-drop-rms 0.20 \
  --min-holdset-expand-overlap-ratio 0.75 \
  --use-preserved-hold-backup \
  --preserve-skip-lineage-advance-variance-drop \
  --preserve-lineage-advance-overlap-threshold 0.5 \
  --debug-epoch-log output/benchmark/tokyo_run2/ar_sweep/run2_full_backup_A3_lineage.csv \
  --out output/benchmark/tokyo_run2/ar_sweep/run2_full_backup_A3_lineage.pos \
  --no-kml
```

A3 run1 full と A3 run3 1910 も同じ option セットで取る (rover/base/nav の path を run1/run3 に、epochs 制限 + CSV/POS path を合わせる)。コマンドの雛形は `notes/2026-04-17_rtk_run2_lineage_guard_ab_plan.md` のステップ 4 を踏襲して構わない。

## ステップ 4: 5 指標比較表 + 発火回数

| dataset | metric | A1 (backup, guard off) | A3 (+ preserve-skip guard) | delta |
|---|---|---:|---:|---:|
| run1 full | Fix |  |  |  |
| run1 full | H_median (m) |  |  |  |
| run1 full | H_p95 (m) |  |  |  |
| run1 full | H_max (m) |  |  |  |
| run1 full | V_p95 (m) |  |  |  |
| run2 full | Fix |  |  |  |
| run2 full | H_median (m) |  |  |  |
| run2 full | H_p95 (m) |  |  |  |
| run2 full | H_max (m) |  |  |  |
| run2 full | V_p95 (m) |  |  |  |
| run3 1910 | Fix |  |  |  |
| run3 1910 | H_median (m) |  |  |  |
| run3 1910 | H_p95 (m) |  |  |  |
| run3 1910 | H_max (m) |  |  |  |
| run3 1910 | V_p95 (m) |  |  |  |

さらに、A3 CSV での `preserve_skip_lineage_advance_fired == 1` カウントを run 別に出す:

- run1 full: 発火 `<N1>` 回
- run2 full: 発火 `<N2>` 回
- run3 1910: 発火 `<N3>` 回

期待値: run2 で正数 (少なくとも 2 発。`177057.4 / 177057.6` を想定)。run1/run3 は低数でよいが 0 でも可。

## ステップ 5: 採用判断 (決定木)

plan 末尾に 1 行で書く。下の 3 文から選ぶ:

- **採用**: A3 は run2 full Fix `<値>` で A1 から +`<delta>` 改善し、run1 full Fix 変化 ≥ -10、run1 full H_p95 変化 ≤ +0.05 m、run3 1910 Fix 変化 ≥ -10、run3 1910 H_p95 変化 ≤ +0.05 m の基準内。`--preserve-skip-lineage-advance-variance-drop` を experimental option として残し、次 plan で default 化と run3 full の本番確認に進む。
- **保留**: A3 は run2 で改善したが、run1 または run3 1910 で regression 基準を超えた。guard は default-off のまま残す。次 plan で threshold を 0.5 より小さく絞る (例: 0.25) か、条件に `prev_pair_count <= 4` を追加してさらに narrow にする案を試す。
- **不採用**: A3 は run2 full Fix も改善しなかった、または明確な regression。`preserve_skip_lineage_advance_fired` の発火タイミング (tow 列) を plan 末尾に列挙して、次 plan で「発火タイミングに対する別の guard 設計」に移る。

## 許容される逸脱

- preserve 発火の if ブロックが複数箇所あり、どれに差し込むべきか現状の `rtk.cpp` では一意に決まらない場合、直前 plan 実装の `recordAcceptedFixOverwrite()` 周辺を起点に 1 箇所だけ選んで入れる。2 箇所以上に同じ guard を展開する改修はこの plan の範囲外。
- A3 run が 1 run あたり 15 分を超えた場合は止めて plan 末尾に追記して終了。

## 想定所要時間

- ステップ 1 ビルド: 03:00
- ステップ 2 guard 実装 + unit test: 45:00
- ステップ 3 run2 full A3: 12:00
- ステップ 3 run1 full A3 + run3 1910 A3: 20:00
- ステップ 4 表埋め + 発火回数集計: 15:00
- ステップ 5 採用判断: 10:00

合計 1:45 目安。

## 成功基準

- 新 option 2 本が CLI から受理され、既定 off で solver 既定動作が前 plan 時点と byte-identical。
- 新 guard の unit test 3 本が pass。
- A3 CSV が 3 本すべて生成され、run2 で `preserve_skip_lineage_advance_fired == 1` が 1 発以上。
- plan 末尾に 5 指標比較表と発火回数 3 行と 1 行の採用/保留/不採用判断が追記されている。
- solver 既定挙動 (= option 全 off) が前 plan の worktree と比較して byte-identical。
- CI-only commit と混ざらない (`.github/**`、`scripts/ci/**` は touch しない)。

## 本 plan の範囲外

- A2 guard (`--max-lineage-advance-overlap-ratio`) の revert / 扱い決定
- `--preserve-skip-lineage-advance-variance-drop` の default 化
- run3 full 本番実行 (A3 採用時の次 plan)
- Tokyo 以外の dataset

## 参照

- 前 plan: `notes/2026-04-17_rtk_177057_early_divergence_plan.md` (hypothesis 発源)
- 前々 plan: `notes/2026-04-17_rtk_run2_lineage_guard_ab_plan.md` (A2 不採用の template)
- 前々々 plan: `notes/2026-04-17_rtk_run2_lineage_plan.md` (telemetry)
- 過去 handoff: `notes/2026-04-14_rtk_demo5_handoff.md`

## Codex 実行結果 (2026-04-17)

- 実装: `--preserve-skip-lineage-advance-variance-drop` と `--preserve-lineage-advance-overlap-threshold` を default-off で追加。A2 guard (`--max-lineage-advance-overlap-ratio` 系) は変更なし。
- 検証: `cmake --build build --target gnss_solve run_tests -j4` pass、`./build/tests/run_tests --gtest_filter='RTKValidationTest.*:RTKLegacyCompatibilityStandaloneTest.*'` pass (22 tests)。
- A3 出力: `output/benchmark/tokyo_run1/ar_sweep/run1_full_backup_A3_lineage.*`、`output/benchmark/tokyo_run2/ar_sweep/run2_full_backup_A3_lineage.*`、`output/benchmark/tokyo_run3/ar_sweep/run3_1910_backup_A3_lineage.*`。

| dataset | metric | A1 (backup, guard off) | A3 (+ preserve-skip guard) | delta |
|---|---|---:|---:|---:|
| run1 full | Fix | 2528 | 2382 | -146 |
| run1 full | H_median (m) | 0.906748 | 0.903884 | -0.002864 |
| run1 full | H_p95 (m) | 28.818816 | 28.950902 | +0.132086 |
| run1 full | H_max (m) | 101.538927 | 101.538927 | +0.000000 |
| run1 full | V_p95 (m) | 61.920645 | 62.113932 | +0.193287 |
| run2 full | Fix | 3685 | 3556 | -129 |
| run2 full | H_median (m) | 0.418068 | 0.305027 | -0.113041 |
| run2 full | H_p95 (m) | 7.019883 | 7.391585 | +0.371702 |
| run2 full | H_max (m) | 46.483554 | 46.483554 | +0.000000 |
| run2 full | V_p95 (m) | 21.801805 | 19.441490 | -2.360315 |
| run3 1910 | Fix | 1321 | 1313 | -8 |
| run3 1910 | H_median (m) | 0.042477 | 0.042463 | -0.000014 |
| run3 1910 | H_p95 (m) | 0.853955 | 0.852286 | -0.001669 |
| run3 1910 | H_max (m) | 20.756078 | 20.756078 | +0.000000 |
| run3 1910 | V_p95 (m) | 2.538864 | 2.532329 | -0.006535 |

- run1 full: 発火 `2` 回
- run2 full: 発火 `12` 回
- run3 1910: 発火 `2` 回

発火 tow:

- run1 full: `187670.200`, `187995.000`
- run2 full: `177057.000`, `177109.600`, `177115.200`, `177116.000`, `177177.000`, `177337.000`, `177466.400`, `177468.400`, `177598.800`, `177600.200`, `177658.800`, `178215.800`
- run3 1910: `179700.200`, `179731.600`

**不採用**: A3 は run2 full Fix `3556` で A1 から `-129` となり改善せず、run1 full も Fix `-146` / H_p95 `+0.132086 m` の明確な regression。`preserve_skip_lineage_advance_fired` の発火タイミングは上記 tow 列の通りで、次 plan で「発火タイミングに対する別の guard 設計」に移る。
