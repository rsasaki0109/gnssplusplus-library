# RTK demo5 parity — 177764.8 lineage-advance guard A/B plan (Codex 実作業用)

更新日: 2026-04-17
作業ブランチ: `codex/rtk-demo5-parity` (dirty state 前提、直前 plan `notes/2026-04-17_rtk_run2_lineage_plan.md` の telemetry 追加が既に入った状態)
計画担当: Claude / 実作業担当: Codex

## この文書の使い方

- 前 plan (`notes/2026-04-17_rtk_run2_lineage_plan.md`) の結論 = **`177764.8` の `partial/variance_drop` 4→4 overwrite が lineage を `139 → 140` に進め、`178089.6` 以降の decisive loss を決定する**。かつその overwrite は **preserve ではなく direct overwrite** (`accepted_preserved_prior_hold_state=0`)。
- 既存の `--min-holdset-expand-overlap-ratio` は preserve 経路に効く guard。`177764.8` は preserve 経路に入っていないので、この箇所には **まだどの guard も効いていない**。
- 本 plan ではこの未処置の direct-overwrite 条件を狙い撃つ **新 guard `--max-lineage-advance-overlap-ratio`** を prototype で入れ、A/B で run2 full の Fix 数が既知 best (`3708`) に戻るか検証する。
- 新 guard は既定 off。solver 既定動作 (`satguard12`) は変えない。
- 重複記述は避ける。事実の再掲は `notes/2026-04-14_rtk_demo5_handoff.md` と `notes/2026-04-17_rtk_run2_lineage_plan.md` を参照。

## 先に結論

- 狙う 1 箇所: lineage advance を起こす accepted overwrite のうち
  - `accepted_candidate_path == VARIANCE_DROP`
  - `accepted_fix_source == PARTIAL`
  - `prev_pair_count <= N` かつ `new_pair_count <= N` (`N=4` 規定)
  - `accepted_overlap_ratio < R` (`R=0.5` 規定)
  - `accepted_preserved_prior_hold_state == false`
- この条件を満たす overwrite は、hold state を更新せず、その epoch は FIX として採用しない (= FLOAT fall-back)。accepted candidate 自体は計算するが、採用しない。
- 既定 off。`--max-lineage-advance-overlap-ratio <R>` と `--max-lineage-advance-pair-count <N>` で有効化。

## Codex 次作業: 1 ステップだけ

### やること

1. 上記条件を満たす direct-overwrite を FLOAT fall-back にする guard を実装する。
2. run1 full / run2 full / run3 `1910` の 3 本で A0 / A1 / A2 を取って 5 指標比較する。
3. 採用判断の 1 行を plan 末尾に追記する。

### 禁止事項

- preserve path を変えない (preserve は既に `--min-holdset-expand-overlap-ratio` でカバー済み)
- pair 名を hardcode しない
- `--preserve-larger-hold-set` を再導入しない
- 速度ゲート / SPP 品質ゲート / stale fix-history gate 系を再導入しない
- `run3 full` を回さない (1910 のみ)
- CI/CD・PPP・Docker・workflow を同時に触らない (`.github/**`、`scripts/ci/**` は touch 禁止)
- `rtk.hpp` を `sed -i` で書き換えない
- solver 既定挙動を変える変更を入れない (新 guard は default-off で統一)

## ステップ 1: ビルド確認

```bash
cd /media/sasaki/aiueo/ai_coding_ws/rtklib_v2_rtk_ws/gnssplusplus-library
cmake --build build --target gnss_solve run_tests -j4
./build/tests/run_tests --gtest_filter='RTKValidationTest.*:RTKLegacyCompatibilityStandaloneTest.*'
```

前 plan の telemetry 追加分の test も含めて全 pass を確認する。通らない場合は先へ進まない。

## ステップ 2: 新 guard の実装

### 2.1 config 追加

`include/libgnss++/algorithms/rtk.hpp` の `RTKConfig` に以下を追加する (既存フィールドの末尾に追加、構造体並びは崩さない):

- `double max_lineage_advance_overlap_ratio = 0.0;` (0=off、`(0, 1]` で有効)
- `int max_lineage_advance_pair_count = 4;` (上限 pair 数。4 規定)
- `bool max_lineage_advance_only_variance_drop = true;` (true=VARIANCE_DROP + PARTIAL 限定、false=全 accepted overwrite)

### 2.2 CLI 追加

`apps/gnss_solve.cpp` に以下を追加 (help と parse 両方):

- `--max-lineage-advance-overlap-ratio <v>`: 値を `max_lineage_advance_overlap_ratio` に代入。範囲外なら `argumentError`。
- `--max-lineage-advance-pair-count <n>`: 値を `max_lineage_advance_pair_count` に代入。範囲外 (負) なら error。
- `--lineage-advance-include-all-paths`: true のとき `max_lineage_advance_only_variance_drop = false`。

### 2.3 guard の発火場所

`src/algorithms/rtk.cpp` の `recordAcceptedFixOverwrite()` 内、lineage ID を進めるか判定する **直前** に以下を差し込む:

- 入力: `source` (`fix_source`), `path` (`candidate_path`), `prev_pair_count`, `new_pair_count`, `overlap_ratio`, `accepted_preserved_prior_hold_state`。
- 条件 (すべて true のとき発火):
  - `rtk_config_.max_lineage_advance_overlap_ratio > 0.0`
  - `overlap_ratio < rtk_config_.max_lineage_advance_overlap_ratio`
  - `prev_pair_count <= rtk_config_.max_lineage_advance_pair_count`
  - `new_pair_count <= rtk_config_.max_lineage_advance_pair_count`
  - `!debug_telemetry_.accepted_preserved_prior_hold_state`
  - `rtk_config_.max_lineage_advance_only_variance_drop == false` か、`source == PARTIAL && path == VARIANCE_DROP` のどちらか

発火時の挙動:

- hold state の pair set を更新しない (= 直前の held set を維持)
- `debug_telemetry_.lineage_advance_blocked = true` を立てる
- 呼び出し側の処理フロー:
  - accepted_candidate の採用フラグを reject に落とす (= この epoch は FLOAT として publish)
  - `debug_telemetry_.fix_status = FLOAT`
  - `debug_telemetry_.accepted_fix_source = NONE`
  - `debug_telemetry_.accepted_candidate_path = NONE`
- ID は進めない (lineage ID / origin_tow は現値維持)

### 2.4 telemetry 追加

`include/libgnss++/algorithms/rtk.hpp` の `DebugTelemetry` に `bool lineage_advance_blocked = false;` を 1 本追加。`apps/gnss_solve.cpp` の CSV header と出力に同名カラムを追加する。

### 2.5 unit test

`tests/test_rtk_legacy.cpp` に下記 3 ケースを追加:

- guard off (既定) で `177764.8` 相当条件の overwrite がそのまま lineage を進めること
- guard on (`overlap_ratio=0.5, pair_count=4, variance_drop_only=true`) で `177764.8` 相当条件の overwrite が block され、`lineage_advance_blocked=true` になること
- guard on でも `prev_pair_count=5` なら block されないこと (pair count 上限の境界)

test は必ず `RTKLegacyCompatibilityStandaloneTest.*` 名前空間に置く。pair 名を hardcode した期待値は書かない (= `recordAcceptedFixOverwrite` の入力値で直接表現する)。

### 2.6 ビルド + test

```bash
cmake --build build --target gnss_solve run_tests -j4
./build/tests/run_tests --gtest_filter='RTKValidationTest.*:RTKLegacyCompatibilityStandaloneTest.*'
```

不通過時は 1 段階だけ修正してリトライ。2 回失敗したら plan 末尾に状況追記して終了。

## ステップ 3: run2 full A/B (2 点)

A0 (baseline `satguard12`):

既に `output/benchmark/tokyo_run2/ar_sweep/run2_full_base_lineage.pos` と `.csv` がある。再利用する。Fix 数は `3474` を基準値として記録するだけでよい。再実行は不要。

A1 (既知 best backup-on、guard off):

既に `output/benchmark/tokyo_run2/ar_sweep/run2_full_backup_lineage.pos` と `.csv` がある。Fix 数は `3685`。再利用する。

A2 (A1 + 新 guard on):

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
  --max-lineage-advance-overlap-ratio 0.5 \
  --max-lineage-advance-pair-count 4 \
  --debug-epoch-log output/benchmark/tokyo_run2/ar_sweep/run2_full_backup_guardA2_lineage.csv \
  --out output/benchmark/tokyo_run2/ar_sweep/run2_full_backup_guardA2_lineage.pos \
  --no-kml
```

## ステップ 4: run1 full / run3 1910 で副作用確認

新 guard は run2 以外で regression を起こしていないか確認する。run2 だけで勝つ案は何度も run3 で壊れている (`notes/2026-04-14_rtk_demo5_handoff.md` line 407-412)。

run1 full (A2):

```bash
./build/apps/gnss_solve \
  --rover /media/sasaki/aiueo/ai_coding_ws/datasets/PPC-Dataset-data/tokyo/run1/rover.obs \
  --base  /media/sasaki/aiueo/ai_coding_ws/datasets/PPC-Dataset-data/tokyo/run1/base.obs \
  --nav   /media/sasaki/aiueo/ai_coding_ws/datasets/PPC-Dataset-data/tokyo/run1/base.nav \
  --mode kinematic --preset low-cost --no-arfilter --no-wide-lane \
  --ratio 2.0 --max-postfix-rms 1.0 \
  --min-hold-count 1 --hold-ratio-threshold 1.0 \
  --glonass-ar autocal --max-hold-div 8 \
  --max-variance-drop-rms 0.20 \
  --min-holdset-expand-overlap-ratio 0.75 \
  --use-preserved-hold-backup \
  --max-lineage-advance-overlap-ratio 0.5 \
  --max-lineage-advance-pair-count 4 \
  --debug-epoch-log output/benchmark/tokyo_run1/ar_sweep/run1_full_backup_guardA2_lineage.csv \
  --out output/benchmark/tokyo_run1/ar_sweep/run1_full_backup_guardA2_lineage.pos \
  --no-kml
```

run3 `1910` (A2):

```bash
./build/apps/gnss_solve \
  --rover /media/sasaki/aiueo/ai_coding_ws/datasets/PPC-Dataset-data/tokyo/run3/rover.obs \
  --base  /media/sasaki/aiueo/ai_coding_ws/datasets/PPC-Dataset-data/tokyo/run3/base.obs \
  --nav   /media/sasaki/aiueo/ai_coding_ws/datasets/PPC-Dataset-data/tokyo/run3/base.nav \
  --mode kinematic --preset low-cost --no-arfilter --no-wide-lane \
  --ratio 2.0 --max-postfix-rms 1.0 \
  --min-hold-count 1 --hold-ratio-threshold 1.0 \
  --glonass-ar autocal --max-hold-div 8 \
  --max-variance-drop-rms 0.20 \
  --min-holdset-expand-overlap-ratio 0.75 \
  --use-preserved-hold-backup \
  --max-lineage-advance-overlap-ratio 0.5 \
  --max-lineage-advance-pair-count 4 \
  --max-epochs 1910 \
  --debug-epoch-log output/benchmark/tokyo_run3/ar_sweep/run3_1910_backup_guardA2_lineage.csv \
  --out output/benchmark/tokyo_run3/ar_sweep/run3_1910_backup_guardA2_lineage.pos \
  --no-kml
```

## ステップ 5: 採用判断

5 指標 (Fix, H_median, H_p95, H_max, V_p95) を下の比較表に埋める。reference は各 run の既存 ground truth CSV を使う (`gnss_ppc_demo.py --summary-json` で再生成してよい)。

| dataset | metric | A1 (backup, guard off) | A2 (backup + lineage guard) | delta |
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

参考値 (既知):

- A0 run2 full Fix = 3474
- A1 run2 full Fix = 3685
- best-known contraction-aware refine (この plan の対象外) run2 full Fix = 3708
- A2 の狙いは run2 full Fix を **3700 以上** に引き上げつつ、run1 full / run3 1910 に以下の regression を出さないこと:
  - run1 full Fix 変化 ≥ -10 epoch
  - run1 full H_p95 変化 ≤ +0.05 m
  - run3 1910 Fix 変化 ≥ -10 epoch
  - run3 1910 H_p95 変化 ≤ +0.05 m

### 採用判断の決定木

plan 末尾に 1 行で書く。下の 3 文から該当するものを 1 つ選ぶ (必要なら数値を入れる):

- **採用**: A2 は run2 full Fix `<数値>` で +`<delta>` 改善し、run1 full / run3 1910 で regression 基準内だった。`--max-lineage-advance-overlap-ratio 0.5 --max-lineage-advance-pair-count 4` を experimental option として solver 本体に残し、次 plan で default 化と run3 full の本番確認に進む。
- **保留**: A2 は run2 で改善したが run1 または run3 で regression 基準を超えた。guard は default-off のまま残すが、`max_lineage_advance_only_variance_drop=true` の既定でさらに狭く絞る次 plan が必要。
- **不採用**: A2 は run2 full Fix も改善しなかった、または regression が明確。guard は revert し、next plan は `177057.4` 早期分岐 (= 前 plan の `Lineage origin changes in base` で見えた global first divergence) の追跡に戻る。

## 許容される逸脱

- `recordAcceptedFixOverwrite()` が想定と違う位置から呼ばれており、guard を差し込むと hold state の他経路を壊す場合は、plan 末尾に「どこに差し込むべきか」の 3 行メモだけ残して止める。2 段階以上の設計変更は本 plan の対象外。
- A2 実行時に `run2 full` の実行時間が 15 分を超えた場合は止めて、plan 末尾に「A2 が hang している」旨を追記して終了する (無限ループや block の疑い)。

## 想定所要時間

- ステップ 1 ビルド: 03:00
- ステップ 2 guard 実装 + unit test: 50:00
- ステップ 3 run2 full A2: 12:00
- ステップ 4 run1 full A2 + run3 1910 A2: 20:00
- ステップ 5 分析 + 表埋め + 採用判断: 25:00

合計 1:50 目安。途中で迷ったら止めて plan を修正する。

## 成功基準

- `--max-lineage-advance-overlap-ratio` が CLI から受理され、既定 off で solver 既定動作が前 plan 時点と byte-identical。
- 新 guard の unit test 3 本が pass。
- A2 CSV が 3 本すべて生成され、`lineage_advance_blocked=1` のカウントが run2 で正で run1/run3 1910 でも有限。
- plan 末尾に 1 行の採用/保留/不採用判断が追記されている。
- CI-only commit と混ざらない (`.github/**`、`scripts/ci/**` は touch しない)。

## 本 plan の範囲外

- `177057.4` 早期分岐の追跡 (A2 不採用時の次 plan)
- run3 full 実行 (A2 採用時の次 plan)
- guard の default 化 (A2 採用時の次 plan)
- backup policy 調整
- Tokyo 以外の dataset

## 参照

- 前 plan: `notes/2026-04-17_rtk_run2_lineage_plan.md` (telemetry 追加と root cause 特定)
- 過去 handoff: `notes/2026-04-14_rtk_demo5_handoff.md` (特に line 303-310 の narrow gate 結果と line 340-347 の次アクション)
- 過去 followup: `notes/2026-04-14_rtk_demo5_followup.md` (何を試して何がダメだったか)

## Codex 実行結果 (2026-04-17)

- build/test: `cmake --build build --target gnss_solve run_tests -j4` pass、`./build/tests/run_tests --gtest_filter='RTKValidationTest.*:RTKLegacyCompatibilityStandaloneTest.*'` pass (19 tests)。
- A2 guard 発火数: run1 full `0`、run2 full `1`、run3 1910 `0`。
- A1 入力: run1 full `output/benchmark/tokyo_run1/ar_sweep/hold_ratio_satguard12_vdroprms020_guard075c_backup_overlapgate_full.pos`、run2 full `output/benchmark/tokyo_run2/ar_sweep/run2_full_backup_lineage.pos`、run3 1910 `output/benchmark/tokyo_run3/ar_sweep/run3_1910_v15_guard075c_backup_overlapgate.pos`。

| dataset | metric | A1 (backup, guard off) | A2 (backup + lineage guard) | delta |
|---|---|---:|---:|---:|
| run1 full | Fix | 2528 | 2528 | 0 |
| run1 full | H_median (m) | 0.906748 | 0.906748 | +0.000000 |
| run1 full | H_p95 (m) | 28.818816 | 28.818816 | +0.000000 |
| run1 full | H_max (m) | 101.538927 | 101.538927 | +0.000000 |
| run1 full | V_p95 (m) | 61.920645 | 61.920645 | +0.000000 |
| run2 full | Fix | 3685 | 3655 | -30 |
| run2 full | H_median (m) | 0.418068 | 0.413701 | -0.004367 |
| run2 full | H_p95 (m) | 7.019883 | 7.023821 | +0.003938 |
| run2 full | H_max (m) | 46.483554 | 46.483554 | +0.000000 |
| run2 full | V_p95 (m) | 21.801805 | 20.824034 | -0.977771 |
| run3 1910 | Fix | 1321 | 1321 | 0 |
| run3 1910 | H_median (m) | 0.042477 | 0.042477 | +0.000000 |
| run3 1910 | H_p95 (m) | 0.853955 | 0.853955 | +0.000000 |
| run3 1910 | H_max (m) | 20.756078 | 20.756078 | +0.000000 |
| run3 1910 | V_p95 (m) | 2.538864 | 2.538864 | +0.000000 |

**不採用**: A2 は run2 full Fix `3655` で A1 から `-30` となり改善しなかった。guard は revert し、next plan は `177057.4` 早期分岐 (= 前 plan の `Lineage origin changes in base` で見えた global first divergence) の追跡に戻る。
