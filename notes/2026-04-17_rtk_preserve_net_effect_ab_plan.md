# RTK demo5 parity — preserve + backup 合成 net effect 測定 (Codex 実作業用)

更新日: 2026-04-17
作業ブランチ: `codex/rtk-demo5-parity`
計画担当: Claude / 実作業担当: Codex

## この文書の使い方

- 3 連続不採用 (A2 / A3 / A3 発火分析) で、現在の guard approach は飽和した。
- 直前 narrative の前提「preserve off = base と合流」が不成立 (A3 で preserve skip 後に別 lineage が連鎖発生)。
- 本 plan は **preserve + backup + narrow gate の composite が net beneficial か net harmful か** を 3 run で直接測定する。
- コード変更なし。既存 options の on/off 切替のみ。想定 35 分。

## 先に結論 (測定内容)

比較する 2 点:

| config | variance_drop | preserve (`--min-holdset-expand-overlap-ratio`) | backup-use + narrow gate | 既知値 (run2 full Fix) |
|---|---|---|---|---:|
| A1 | ON (0.20) | ON (0.75) | ON | 3685 |
| A4 | ON (0.20) | **OFF** (0) | **OFF** | 未測定 (v15_020 相当) |

A0 (satguard12 単独 = 3474) と A1 (既存 CSV = 3685) は既知。A4 を新規取得。

## Codex 次作業: 1 ステップだけ

### やること

1. ビルド確認 (既存ワークツリーで全 test pass を再確認のみ)。
2. run1 full / run2 full / run3 `1910` の 3 本で A4 を取る。
3. A0 / A1 / A4 の 5 指標比較表と 1 行判断を plan 末尾に追記する。

### 禁止事項

- コード変更禁止 (solver / test / apps すべて touch しない)
- A2 / A3 guard option は使わない (全 off)
- `--preserve-larger-hold-set` / 速度ゲート / stale fix-history gate 系の再導入禁止
- `run3 full` を回さない (1910 のみ)
- `.github/**`、`scripts/ci/**`、`apps/ppp_*.cpp` は touch 禁止
- gnssplusplus-library ワークツリー外に書き込まない

## ステップ 1: ビルド確認

```bash
cd /media/sasaki/aiueo/ai_coding_ws/rtklib_v2_rtk_ws/gnssplusplus-library
cmake --build build --target gnss_solve run_tests -j4
./build/tests/run_tests --gtest_filter='RTKValidationTest.*:RTKLegacyCompatibilityStandaloneTest.*'
```

既存の A2 / A3 prototype が入った状態で全 test pass を確認。通らない場合は plan 末尾にだけ状況追記して終了。

## ステップ 2: A4 実行 (3 run)

### A4 run2 full

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
  --debug-epoch-log output/benchmark/tokyo_run2/ar_sweep/run2_full_A4_lineage.csv \
  --out output/benchmark/tokyo_run2/ar_sweep/run2_full_A4_lineage.pos \
  --no-kml
```

A4 run1 full と A4 run3 1910 も同 option (preserve / backup / A2 / A3 全 off、variance_drop のみ ON)。rover/base/nav の path と CSV/POS path を各 run に合わせる。run3 は `--max-epochs 1910` を付ける。

### 1 run あたり 15 分超えたら

止めて plan 末尾に「A4 `<run name>` が hang」旨を追記して終了。無限ループ / block 疑い。

## ステップ 3: 比較表 + summary json

A0 と A1 の既存 CSV/POS は再利用。A0 `.pos` は `output/benchmark/tokyo_run2/ar_sweep/run2_full_base_lineage.pos` 他を使う。A1 は `run2_full_backup_lineage.pos` 他を使う。

各 run に対して 5 指標 (Fix, H_median, H_p95, H_max, V_p95) を `reference.csv` 突合で計算する。`gnss_ppc_demo.py --use-existing-solution --summary-json` が使える。

| dataset | metric | A0 (satguard12) | A1 (backup + narrow) | A4 (preserve + backup OFF) | A4 - A1 |
|---|---|---:|---:|---:|---:|
| run1 full | Fix |  |  |  |  |
| run1 full | H_median (m) |  |  |  |  |
| run1 full | H_p95 (m) |  |  |  |  |
| run1 full | H_max (m) |  |  |  |  |
| run1 full | V_p95 (m) |  |  |  |  |
| run2 full | Fix | 3474 | 3685 |  |  |
| run2 full | H_median (m) |  | 0.418068 |  |  |
| run2 full | H_p95 (m) |  | 7.019883 |  |  |
| run2 full | H_max (m) |  | 46.483554 |  |  |
| run2 full | V_p95 (m) |  | 21.801805 |  |  |
| run3 1910 | Fix |  | 1321 |  |  |
| run3 1910 | H_median (m) |  | 0.042477 |  |  |
| run3 1910 | H_p95 (m) |  | 0.853955 |  |  |
| run3 1910 | H_max (m) |  | 20.756078 |  |  |
| run3 1910 | V_p95 (m) |  | 2.538864 |  |  |

A0 の空欄は既存 CSV から埋める。run1/run3 の A0 数値が手元に無ければ `--out` は既存 `.pos` (handoff memo 記載のもの) を使って `gnss_ppc_demo.py` で 5 指標を再計算する。

## ステップ 4: 結論 4 択 (1 行で plan 末尾に書く)

- **preserve harmful**: A4 が A1 より run2 Fix +0 以上かつ run1 / run3 1910 の Fix / H_p95 で regression なし。次 plan は preserve mechanism を解除する方向 (ただし run3 full の本番確認が必要)。
- **preserve neutral**: A4 と A1 が全 5 指標 × 3 run で ±1% 以内。preserve mechanism は寄与なし。次 plan は preserve を削除して approach を簡素化してから再設計。
- **preserve beneficial**: A4 が run3 で明確 regression (Fix delta ≤ -100 など、2026-04-14 handoff の v15_020 再現)。preserve + backup は run3 保護に必須。次 plan は preserve を残したまま「narrow gate をさらに緊縮する」or「backup 側に移る signal 設計」へ戻す。
- **異常**: A4 が 3 run 全部でも A1 より明確に劣る、かつ run3 再現する。preserve + backup 合成が必要という過去結論を再確認するだけ。この場合 P4 撤退 (`satguard12` を final 扱い) も現実解として plan 末尾に記録する。

## 想定所要時間

- ステップ 1 ビルド確認: 03:00
- ステップ 2 A4 run2 full: 08:00
- ステップ 2 A4 run1 full: 06:00
- ステップ 2 A4 run3 1910: 04:00
- ステップ 3 summary json + 表埋め: 10:00
- ステップ 4 判断: 05:00

合計 36 分目安。

## 成功基準

- A4 CSV / POS が 3 本生成。
- 5 指標比較表が A0 / A1 / A4 で埋まる。
- 結論 4 択のうち 1 つを 1 行で plan 末尾に追記。
- コード差分なし (`git diff --stat src/ include/ apps/ tests/` が空)。

## 本 plan の範囲外

- 新 guard 実装
- default 化判断
- run3 full 実行
- Tokyo 以外 dataset
- A2 / A3 prototype の revert

## 参照

- A2 plan: `notes/2026-04-17_rtk_run2_lineage_guard_ab_plan.md` (-30)
- A3 plan: `notes/2026-04-17_rtk_preserve_lineage_seed_plan.md` (-129 / -146)
- 因果分析: `notes/2026-04-17_rtk_177057_early_divergence_plan.md`
- v15_020 過去結果: `notes/2026-04-14_rtk_demo5_handoff.md` line 407-411 (run1 +, run2 +, run3 -828 の記録)

## Codex 実行結果 (2026-04-17)

- build/test: `cmake --build build --target gnss_solve run_tests -j4` pass、`./build/tests/run_tests --gtest_filter='RTKValidationTest.*:RTKLegacyCompatibilityStandaloneTest.*'` pass (22 tests)。
- A4 出力: `output/benchmark/tokyo_run1/ar_sweep/run1_full_A4_lineage.*`、`output/benchmark/tokyo_run2/ar_sweep/run2_full_A4_lineage.*`、`output/benchmark/tokyo_run3/ar_sweep/run3_1910_A4_lineage.*`。

| dataset | metric | A0 (satguard12) | A1 (backup + narrow) | A4 (preserve + backup OFF) | A4 - A1 |
|---|---|---:|---:|---:|---:|
| run1 full | Fix | 2397 | 2528 | 2381 | -147 |
| run1 full | H_median (m) | 0.903884 | 0.906748 | 0.903884 | -0.002864 |
| run1 full | H_p95 (m) | 28.950902 | 28.818816 | 28.950902 | +0.132086 |
| run1 full | H_max (m) | 101.538927 | 101.538927 | 101.538927 | +0.000000 |
| run1 full | V_p95 (m) | 62.113932 | 61.920645 | 62.113932 | +0.193287 |
| run2 full | Fix | 3474 | 3685 | 3467 | -218 |
| run2 full | H_median (m) | 0.647082 | 0.418068 | 0.534305 | +0.116237 |
| run2 full | H_p95 (m) | 7.084249 | 7.019883 | 7.083094 | +0.063211 |
| run2 full | H_max (m) | 46.483554 | 46.483554 | 46.483554 | +0.000000 |
| run2 full | V_p95 (m) | 19.577396 | 21.801805 | 19.577216 | -2.224589 |
| run3 1910 | Fix | 1303 | 1321 | 1298 | -23 |
| run3 1910 | H_median (m) | 0.040815 | 0.042477 | 0.042988 | +0.000511 |
| run3 1910 | H_p95 (m) | 0.881393 | 0.853955 | 0.851941 | -0.002014 |
| run3 1910 | H_max (m) | 22.266542 | 20.756078 | 20.756078 | +0.000000 |
| run3 1910 | V_p95 (m) | 2.745112 | 2.538864 | 1.484460 | -1.054404 |

**異常**: A4 は A1 に対して Fix が run1 `-147` / run2 `-218` / run3 1910 `-23` と全 run で劣り、preserve + backup 合成が必要という過去結論を再確認する。
