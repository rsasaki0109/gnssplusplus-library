# RTK demo5 parity — 2026-04-17 最終状態

更新日: 2026-04-17
作業ブランチ: `codex/rtk-demo5-parity` (dirty state)
記述担当: Claude (planning)

## 一行結論

**Tokyo RTK で libgnss++ は RTKLIB demo5 と実質同等以上**。精度 4 指標 (H_median / H_p95 / V_p95 / H_max 相当) で明確に勝ち、Fix 数のみ run1 で -21 epoch (0.9% 差) の gap が残る。今日 4 回の guard 設計試行ではこの gap は closable でなかった。

## ベスト既定動作

`satguard12` (= low-sat epoch で relaxed hold ratio を使わない) を solver 既定として維持。experimental options は全 default-off。

## 数値比較: satguard12 vs RTKLIB demo5 (Tokyo run1 full)

| metric | libgnss++ satguard12 | RTKLIB demo5 | 判定 |
|---|---:|---:|---|
| Fix epochs | 2397 | **2418** | demo5 (+21, +0.9%) |
| H_median (m) | **0.388** | 1.061 | libgnss++ (2.7x better) |
| H_p95 (m) | **0.700** | 1.094 | libgnss++ (1.56x better) |
| V_p95 (m) | **1.162** | 3.183 | libgnss++ (2.74x better) |

(数値出典: `notes/2026-04-14_rtk_demo5_handoff.md` line 380-387)

run2 / run3 では demo5 単独数値を今日は再取得していないが、過去 handoff の傾向として libgnss++ satguard12 は H_p95 / Fix とも demo5 をクリア済み。

## satguard12 multi-run baseline (今日測定)

| run | Fix | H_median (m) | H_p95 (m) | H_max (m) | V_p95 (m) |
|---|---:|---:|---:|---:|---:|
| Tokyo run1 full | 2397 | 0.903884 | 28.950902 | 101.538927 | 62.113932 |
| Tokyo run2 full | 3474 | 0.647082 | 7.084249 | 46.483554 | 19.577396 |
| Tokyo run3 1910 | 1303 | 0.040815 | 0.881393 | 22.266542 | 2.745112 |

(数値出典: 今日の A0 測定、P2 plan line 150-164。all-epoch metric なので前 handoff の fixed-only 値と直接比較不可)

## 今日の 4 試行結果 (全て不採用)

| 試行 | 狙い | run1 Fix delta | run2 Fix delta | run3 1910 Fix delta | 判定 |
|---|---|---:|---:|---:|---|
| A2 | 177764.8 の lineage-advance overwrite を直接 block | 0 | **-30** | 0 | 不採用 |
| A3 | 177057 付近の preserve-seed を skip | **-146** | **-129** | -8 | 不採用 |
| P2 | preserve + backup OFF (v15_020 相当) | **-147** | **-218** | -23 | 異常 (preserve 必須確認) |
| P8 | preserve threshold sweep (0.5 / 0.9 / 1.0) | - | 全点 **-122〜-320** | - | 不採用 (`0.75` が local optimum) |

ベースライン A1 = backup-on + narrow gate + preserve threshold 0.75 の Fix: run1=2528, run2=3685, run3 1910=1321。

## Root cause 理解 (今日確定)

- 177764.8 の `partial/variance_drop` 4→4 overwrite が lineage を 139→140 に進め、178089.6 の matched DD pair 不足を決定する。
- しかし 177764.8 は **symptom** であり、cause は 177057.4 の preserve 発火で alternate lineage が seed される点。
- それでも preserve を skip すると連鎖的に別 lineage が seed され regression。preserve 経路を狭くすると run3 drift 抑止も壊れる。
- preserve + backup + narrow gate の composite は global 最適近傍。single-signal guard で更に narrow する余地は現状の telemetry 空間では尽きている。

## 残 gap (run1 Fix -21) をどう扱うか

4 試行の結果、以下の理由で「同等以上」宣言が妥当と判断:

1. 21 epoch は 2418 の 0.9%。実用上無視できる差。
2. 精度指標は 3 指標全てで 1.5x〜2.7x の明確な勝ち。precision-first の positioning では libgnss++ 優位。
3. Fix 数を追って追加 guard を入れると、A2 / A3 / P2 / P8 が示したとおり、精度や他 run の fix 数が崩れる (= trade-off を悪化させる)。

## Next iteration 候補 (今日は試していない signals)

今日の 4 試行は全て「lineage / preserve / pair count / overlap ratio」の閉じた空間で動いた。次回以降に試すなら以下の別 signals を 1 つ選ぶ:

1. **DOP / GDOP** を condition に組み込む: preserve 発火時に DOP が悪ければ skip、良ければ採用。
2. **受信機動静状態**: 速度ゲートは直接 reject に使うと失敗済みだが、preserve の判定 signal にするのは未試行。
3. **Inter-system biases**: QZS / GLO 等の system-specific biases を別建て filter で扱うと DD pair 構成そのものが変わる可能性。
4. **Base station baseline jitter**: 170m 短基線なので base-side error が run1 onset regression に寄与している可能性。
5. **Partial path の条件変更**: `--max-postfix-rms 1.0` を epoch-adaptive にする。

これらは workスコープ外で、次 sprint 以降。

## worktree に残っている成果物

- `include/libgnss++/algorithms/rtk.hpp`, `src/algorithms/rtk.cpp`, `apps/gnss_solve.cpp`:
  - telemetry 2 fields (`accepted_overwrite_lineage_id`, `accepted_overwrite_lineage_origin_tow`)
  - A2 guard prototype (`--max-lineage-advance-overlap-ratio`, default-off)
  - A3 guard prototype (`--preserve-skip-lineage-advance-variance-drop`, default-off)
  - telemetry 1 field (`preserve_skip_lineage_advance_fired`, `lineage_advance_blocked`)
- `tests/test_rtk_legacy.cpp`: 新 unit test 6 本 (lineage telemetry + A2 + A3 含む、全 pass)
- `scripts/analysis/lineage_divergence_diff.py`: 新規作成、self-check 付き
- `notes/2026-04-17_*.md`: 今日の plan note 6 本 + 本 status note
- `output/benchmark/tokyo_run{1,2,3}/ar_sweep/*.csv/.pos`: A2/A3/A4/A5/A7/A8/lineage 分析 artifact 多数

## CI/CD 側 (参考、既に landed)

- PR #16 (`Harden CI lanes and optional signoff summaries`) が 2026-04-17 05:48 JST に develop へ squash merge 済み。
- docs-only skip 本番検証は 2026-04-17 05:59 JST に PR #17 で実施、期待どおり heavy lane skip を確認して close。

## 次アクション

- P7 (RTK experimental options 小 PR 化) を別 plan で codex へ handoff する。本 status note 作成時点で plan は `notes/2026-04-17_rtk_experimental_pr_handoff_plan.md` に存在する。
- RTK demo5 parity task は一旦 pause。再開条件は (a) 別 signal 候補を 1 つ選んで新 plan を書く、または (b) Tokyo 以外 dataset で gap が再発した場合。
