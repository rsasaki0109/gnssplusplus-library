# RTK demo5 parity — preserve threshold sweep (Codex 実作業用)

更新日: 2026-04-17
作業ブランチ: `codex/rtk-demo5-parity`
計画担当: Claude / 実作業担当: Codex

## この文書の使い方

- 直前 P2 plan (`notes/2026-04-17_rtk_preserve_net_effect_ab_plan.md`) の結論は「preserve + backup 合成は net beneficial で不可欠」だった。
- 現在値 `--min-holdset-expand-overlap-ratio 0.75` が最適かは未検証。本 plan で **threshold sweep** をかけて最適値を切り分ける。
- コード変更なし。既存 options の値だけ変える。想定 55 分。
- 2 phase 構成で短く切る:
  - **Phase A**: run2 full で threshold `0.5 / 0.9 / 1.0` を取り A1 (0.75) と比較してスクリーニング。
  - **Phase B**: Phase A で勝ち筋があれば、その threshold で run1 full + run3 1910 を取って副作用確認。

## 先に結論 (測定内容)

A1 基準点:

- variance_drop 0.20 / preserve threshold `0.75` / backup on + narrow gate on = run2 full Fix 3685 (既知)

Phase A の sweep 候補:

| config | preserve threshold | 狙い |
|---|---|---|
| A5 | `0.5` (less aggressive) | lineage-seed 発火を物理的に減らして harmful preserve 回避 |
| A7 | `0.9` (more aggressive) | more preserve → lineage stability 強化、run3 型 drift 抑止 |
| A8 | `1.0` (preserve on every overwrite) | preserve を常時発火させて extreme 側を見る |

他の option は A1 と同じ (`--max-variance-drop-rms 0.20`、`--use-preserved-hold-backup`、narrow gate は既存のまま)。

Phase B はスクリーニング後に選ぶ 1 値のみ。

## Codex 次作業: 1 ステップだけ

### やること

1. ビルド確認 (コード変更なしで既存 test pass)。
2. Phase A: run2 full で A5 / A7 / A8 を取る。
3. Phase A 比較: A1 と突き合わせ、run2 Fix が A1 (3685) より +10 以上改善した threshold を「勝ち筋」として 1 つ選ぶ。
4. Phase B: 勝ち筋あり → その threshold で run1 full + run3 1910 を取る。勝ち筋なし → Phase B skip。
5. 結果表と採用判断を plan 末尾に追記。

### 禁止事項

- コード変更禁止 (solver / test / apps を touch しない)
- A2 / A3 guard option を使わない
- `--preserve-larger-hold-set` / 速度ゲート / stale fix-history gate 系の再導入禁止
- `run3 full` を回さない (1910 のみ)
- `.github/**`、`scripts/ci/**`、`apps/ppp_*.cpp` は touch 禁止
- gnssplusplus-library ワークツリー外に書き込まない

## ステップ 1: ビルド確認

```bash
cd /workspace/ai_coding_ws/rtklib_v2_rtk_ws/gnssplusplus-library
cmake --build build --target gnss_solve run_tests -j4
./build/tests/run_tests --gtest_filter='RTKValidationTest.*:RTKLegacyCompatibilityStandaloneTest.*'
```

通らない場合は plan 末尾にだけ状況追記して終了。

## ステップ 2: Phase A (run2 full のみ)

### 雛形

```bash
./build/apps/gnss_solve \
  --rover /workspace/ai_coding_ws/datasets/PPC-Dataset-data/tokyo/run2/rover.obs \
  --base  /workspace/ai_coding_ws/datasets/PPC-Dataset-data/tokyo/run2/base.obs \
  --nav   /workspace/ai_coding_ws/datasets/PPC-Dataset-data/tokyo/run2/base.nav \
  --mode kinematic --preset low-cost --no-arfilter --no-wide-lane \
  --ratio 2.0 --max-postfix-rms 1.0 \
  --min-hold-count 1 --hold-ratio-threshold 1.0 \
  --glonass-ar autocal --max-hold-div 8 \
  --max-variance-drop-rms 0.20 \
  --min-holdset-expand-overlap-ratio <R> \
  --use-preserved-hold-backup \
  --debug-epoch-log output/benchmark/tokyo_run2/ar_sweep/run2_full_<label>_lineage.csv \
  --out output/benchmark/tokyo_run2/ar_sweep/run2_full_<label>_lineage.pos \
  --no-kml
```

- A5: `R=0.5`, `label=A5`
- A7: `R=0.9`, `label=A7`
- A8: `R=1.0`, `label=A8`

1 run あたり 15 分超えたら止めて plan 末尾追記。

## ステップ 3: Phase A 比較表

| config | threshold | run2 full Fix | Fix vs A1 | 判定 |
|---|---|---:|---:|---|
| A1 | 0.75 | 3685 | - | baseline |
| A5 | 0.5 |  |  |  |
| A7 | 0.9 |  |  |  |
| A8 | 1.0 |  |  |  |

「勝ち筋」の定義:

- run2 full Fix が A1 から **+10 以上改善** (ランダム性ではなく実力差と見なせる幅)
- かつ A1 と同じ精度指標 (H_median / H_p95) で明確 regression しない (H_p95 +0.05 m 以内)

候補が複数ある場合は Fix 数が最大のものを 1 つ選ぶ。

## ステップ 4: Phase B (勝ち筋あり時のみ)

選んだ threshold で以下を取る:

### run1 full

```bash
./build/apps/gnss_solve \
  --rover /workspace/ai_coding_ws/datasets/PPC-Dataset-data/tokyo/run1/rover.obs \
  --base  /workspace/ai_coding_ws/datasets/PPC-Dataset-data/tokyo/run1/base.obs \
  --nav   /workspace/ai_coding_ws/datasets/PPC-Dataset-data/tokyo/run1/base.nav \
  --mode kinematic --preset low-cost --no-arfilter --no-wide-lane \
  --ratio 2.0 --max-postfix-rms 1.0 \
  --min-hold-count 1 --hold-ratio-threshold 1.0 \
  --glonass-ar autocal --max-hold-div 8 \
  --max-variance-drop-rms 0.20 \
  --min-holdset-expand-overlap-ratio <WINNER_R> \
  --use-preserved-hold-backup \
  --debug-epoch-log output/benchmark/tokyo_run1/ar_sweep/run1_full_<WINNER_LABEL>_lineage.csv \
  --out output/benchmark/tokyo_run1/ar_sweep/run1_full_<WINNER_LABEL>_lineage.pos \
  --no-kml
```

### run3 1910

rover/base/nav を run3 に変え `--max-epochs 1910` 付き。

### Phase B 比較表

| dataset | metric | A1 | 勝ち筋 `<WINNER_LABEL>` | delta |
|---|---|---:|---:|---:|
| run1 full | Fix |  |  |  |
| run1 full | H_median (m) |  |  |  |
| run1 full | H_p95 (m) |  |  |  |
| run1 full | H_max (m) |  |  |  |
| run1 full | V_p95 (m) |  |  |  |
| run2 full | Fix | 3685 | (Phase A の値) | (同) |
| run3 1910 | Fix | 1321 |  |  |
| run3 1910 | H_median (m) | 0.042477 |  |  |
| run3 1910 | H_p95 (m) | 0.853955 |  |  |
| run3 1910 | H_max (m) | 20.756078 |  |  |
| run3 1910 | V_p95 (m) | 2.538864 |  |  |

## ステップ 5: 採用判断 (決定木)

plan 末尾に 1 行で書く。

- **採用**: Phase A で勝ち筋あり、かつ Phase B で run1 / run3 1910 の regression が基準内 (run1 Fix ≥ -10、run1 H_p95 ≤ +0.05 m、run3 Fix ≥ -10、run3 H_p95 ≤ +0.05 m)。次 plan で default 化と run3 full 本番確認。
- **保留**: Phase A で勝ち筋あり、だが Phase B で副作用大。勝った threshold を experimental として残し、次 plan で他条件と組み合わせた narrow 化を検討。
- **不採用**: Phase A で勝ち筋なし (3 点すべて A1 より悪いか同等)。現在の `0.75` がこのパラメータ次元では local optimum と確定。次は P4 撤退 (`satguard12` final 化) か別 signal への pivot を user に提示する。

## 許容される逸脱

- Phase A の 3 run いずれかが 15 分超えた場合は、その config だけスキップして残り 2 点で判定。3 点全部 skip になったら「Phase A 実行不可」を plan 末尾に書き終了。
- A1 の既存 CSV/POS は再利用する (再測定しない)。

## 想定所要時間

- ステップ 1 ビルド確認: 02:00
- ステップ 2 Phase A (run2 full × 3): 24:00
- ステップ 3 Phase A 比較: 05:00
- ステップ 4 Phase B (run1 full + run3 1910): 10:00 (勝ち筋あり時)
- ステップ 5 判断: 05:00

合計 46 分目安 (Phase B skip 時 35 分)。

## 成功基準

- Phase A の 3 CSV / POS が生成 (hang case 除く)。
- Phase A 比較表が埋まり、勝ち筋の有無が 1 文で書ける。
- Phase B 実行時は Phase B 比較表も埋まる。
- 採用判断 1 行が追記。
- コード差分なし (`git diff --stat src/ include/ apps/ tests/` が空)。

## 本 plan の範囲外

- 新 guard 実装
- default 化
- run3 full 実行
- Tokyo 以外 dataset
- A2 / A3 prototype の revert / PR 化

## 参照

- P2 結果: `notes/2026-04-17_rtk_preserve_net_effect_ab_plan.md` (preserve + backup 不可欠を再確認)
- A3 結果: `notes/2026-04-17_rtk_preserve_lineage_seed_plan.md`
- A2 結果: `notes/2026-04-17_rtk_run2_lineage_guard_ab_plan.md`
- 過去 handoff: `notes/2026-04-14_rtk_demo5_handoff.md`

## Codex 実行結果 (2026-04-17)

- build/test: `cmake --build build --target gnss_solve run_tests -j4` pass、`./build/tests/run_tests --gtest_filter='RTKValidationTest.*:RTKLegacyCompatibilityStandaloneTest.*'` pass (22 tests)。
- Phase A 出力: `output/benchmark/tokyo_run2/ar_sweep/run2_full_A5_lineage.*`、`output/benchmark/tokyo_run2/ar_sweep/run2_full_A7_lineage.*`、`output/benchmark/tokyo_run2/ar_sweep/run2_full_A8_lineage.*`。
- 15 分超過なし。Phase B は Phase A で勝ち筋なしのため skip。

| config | threshold | run2 full Fix | Fix vs A1 | H_median (m) | H_p95 (m) | 判定 |
|---|---:|---:|---:|---:|---:|---|
| A1 | 0.75 | 3685 | - | 0.418068 | 7.019883 | baseline |
| A5 | 0.5 | 3365 | -320 | 0.525446 | 7.950980 | 不採用: Fix 低下、H_p95 悪化 |
| A7 | 0.9 | 3542 | -143 | 0.420063 | 7.400365 | 不採用: Fix 低下、H_p95 悪化 |
| A8 | 1.0 | 3563 | -122 | 0.420063 | 7.555276 | 不採用: Fix 低下、H_p95 悪化 |

**不採用**: Phase A で勝ち筋なし (A5/A7/A8 はすべて A1 `3685` から Fix が `-320` / `-143` / `-122`)。現在の `0.75` がこのパラメータ次元では local optimum と判定し、Phase B は skip。
