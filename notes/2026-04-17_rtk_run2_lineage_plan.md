# RTK demo5 parity — run2 pre-177764 lineage 調査計画 (Codex 実作業用)

更新日: 2026-04-17
作業ブランチ: `codex/rtk-demo5-parity` (既存の dirty state 前提)
計画担当: Claude / 実作業担当: Codex

## この文書の使い方

- 目的は「全 5 指標 × 3 run で RTKLIB demo5 超え」の残り gap を 1 段縮めること。
- 今回は新しい guard を入れるのではなく、**既存 telemetry で run2 alternate lineage を切り分ける** フェーズ。
- `notes/2026-04-14_rtk_demo5_followup.md` と `notes/2026-04-14_rtk_demo5_handoff.md` を既読前提。重複は書かない。

## 先に結論

- 既定 `satguard12` は run1 で精度勝ち、Fix 数だけ `-21` 負け。
- `one-shot backup + partial-overlap 4->4 narrow gate` は run1 full / run3 full を伸ばしたが、**run2 full は `3685` で best-known contraction-aware refine (`3708`) に届かない**。
- run2 harmful point は single preserve (`177764.8`) ではなく、**その前 `177664.2 / 177667.2 / 177667.6 / 177688.6 / 177689.2` の backup-use 群が開いた alternate lineage**。
- 次のブレイクは「どの backup-use が alternate lineage を決定づけたか」を直接特定することにある。

## Codex 次作業: 1 ステップだけ

### やること

run2 の backup-use 群と、その後 `177761.6` の 4-pair lambda path を因果として繋ぐ overwrite lineage trace を出す。新しい guard は入れない。telemetry 追加と分析のみ。

### 禁止事項

- `--preserve-larger-hold-set` を再導入しない
- 速度ゲート系 (前回 fix / 前回公開 solution / SPP PDOP) を再導入しない
- stale fix-history gate 系を再導入しない
- pair 名 (`G19>G06`, `E10>E12` 等) を hardcode した条件を書かない
- run3 full を先に回さない
- CI/CD・PPP・Docker・workflow を同時に触らない
- `rtk.hpp` を `sed -i` で書き換えない

## ステップ 1: ビルド確認

```bash
cd /workspace/ai_coding_ws/rtklib_v2_rtk_ws/gnssplusplus-library
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target gnss_solve run_tests -j4
./build/tests/run_tests --gtest_filter='RTKValidationTest.*:RTKLegacyCompatibilityStandaloneTest.ResolveAmbiguitiesUsesConfiguredHoldRatioThreshold'
```

unit test が通らないまま本番 run は回さない。

## ステップ 2: telemetry 追加 (最小限)

既存の `recordAcceptedFixOverwrite()` (`src/algorithms/rtk.cpp:2803` 付近) は overwrite の before/after pair keys / overlap を記録している。run2 alternate lineage 追跡には以下 2 フィールドを追加する。これだけ。

追加するフィールド:

- `accepted_overwrite_lineage_id` (int64)
  - 意味: held set が「連続して同じ lineage にある」epoch 群を同じ ID にまとめる識別子。
  - 付け方:
    - 直前 epoch の held pair set と **overlap ratio >= 0.5** の更新は同 lineage として現在 ID を維持。
    - それ未満の overwrite で ID を `+1` 進める (`partial + variance_drop` も `hold` も区別しない)。
    - `hold` が通らず held set 不変の epoch は現在 ID を維持。
    - solver 起動時は ID=0、最初の accepted overwrite で 1 にする。
- `accepted_overwrite_lineage_origin_tow` (double)
  - 意味: 現 lineage ID が始まった epoch の TOW。
  - 付け方: ID が進んだ瞬間の `currentEpoch.tow`。同一 lineage 内では固定。

触るファイル:

- `include/libgnss++/algorithms/rtk.hpp` (`DebugTelemetry` と `HoldState` 周辺にフィールド追加)
- `src/algorithms/rtk.cpp` (`recordAcceptedFixOverwrite()` 内で overlap 計算して ID 更新)
- `apps/gnss_solve.cpp` (CSV header と出力列に追加)

このフィールド追加は既定動作を変えない。solver 挙動は純粋に telemetry-only。

unit test は既存の `recordAcceptedFixOverwrite` 周辺に 1 本だけ追加する:

- prev set と overlap 100% の update で lineage ID が不変
- prev set と overlap 0% の update で lineage ID が +1
- prev set と overlap 50% 境界で ID が進まない (>= 0.5 は同 lineage)

## ステップ 3: run2 full を backup-on / backup-off の 2 パターンで取る

backup-off (= `satguard12` 単独):

```bash
./build/apps/gnss_solve \
  --rover /workspace/ai_coding_ws/datasets/PPC-Dataset-data/tokyo/run2/rover.obs \
  --base  /workspace/ai_coding_ws/datasets/PPC-Dataset-data/tokyo/run2/base.obs \
  --nav   /workspace/ai_coding_ws/datasets/PPC-Dataset-data/tokyo/run2/base.nav \
  --mode kinematic --preset low-cost --no-arfilter --no-wide-lane \
  --ratio 2.0 --max-postfix-rms 1.0 \
  --min-hold-count 1 --hold-ratio-threshold 1.0 \
  --glonass-ar autocal --max-hold-div 8 \
  --debug-epoch-log output/benchmark/tokyo_run2/ar_sweep/run2_full_base_lineage.csv \
  --out output/benchmark/tokyo_run2/ar_sweep/run2_full_base_lineage.pos \
  --no-kml
```

backup-on (one-shot + partial-overlap 4->4 gate):

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
  --min-holdset-expand-overlap-ratio 0.75 \
  --use-preserved-hold-backup \
  --debug-epoch-log output/benchmark/tokyo_run2/ar_sweep/run2_full_backup_lineage.csv \
  --out output/benchmark/tokyo_run2/ar_sweep/run2_full_backup_lineage.pos \
  --no-kml
```

## ステップ 4: lineage diff を分析する

artifact 配置:

- `output/benchmark/tokyo_run2/ar_sweep/run2_full_base_lineage.csv`
- `output/benchmark/tokyo_run2/ar_sweep/run2_full_backup_lineage.csv`

必要なのは `tow, accepted_overwrite_lineage_id, accepted_overwrite_lineage_origin_tow, accepted_fix_source, accepted_candidate_path, accepted_overwrite_before_pair_keys, accepted_overwrite_after_pair_keys, hold_used_preserved_backup, fix_status` 列を TOW で join した比較表。

ピンポイントで見る window:

- `tow ∈ [177640.0, 177770.0]`: backup-use 群 (`177664.2 / 177667.2 / 177667.6 / 177688.6 / 177689.2`) と `177764.8` の 4->4 preserve を同時に俯瞰。
- `tow ∈ [178070.0, 178190.0]`: decisive loss cluster (`178089.6-178177.8`)。
- `tow ∈ [177760.0, 177770.0]`: 4-pair lambda path が入る転換点。

判定項目 (この順で答えを文面で書き残す):

1. base と backup で lineage ID が最初に分岐する epoch
2. その分岐 epoch における overwrite の before / after pair set と source/path
3. 分岐後、どの backup-use が同 lineage を延命させているか
4. `177764.8` preserve は lineage 延命に unique に寄与しているか (= preserve を抑えたら lineage ID が進むか)
5. decisive loss cluster の時点での lineage ID が base と backup で異なるか同じか

## ステップ 5: 許容される結論の形

この plan で出すのは **root cause の一行記述** だけ。新 guard の実装はこの plan の範囲外。

書き残すべき 1 行:

> run2 の backup regression は `<分岐 epoch>` の `<source/path>` overwrite が lineage を `<ID A -> ID B>` に進めたことに起因し、以後 `<証拠 epoch 群>` で同 lineage が延命された結果 `178089.6` 付近で matched DD pair 不足に陥る。

この 1 行が埋まれば次 PR に進んで良い。埋まらないうちに guard は書かない。

## 許容される逸脱

次のどれかが事実であった場合は plan を上書きしてよい:

- `recordAcceptedFixOverwrite()` が想定と違う位置で呼ばれており、lineage ID を overwrite 時点で正しく進められない。この場合は呼び出し側を書き換えるより前に「どこで呼ばれるべきか」の 3 行メモだけ残して止める。
- `one-shot backup` のコード path が退化して backup use が 0 になる。この場合は `notes/2026-04-14_rtk_demo5_handoff.md` の「one-shot backup」節で紹介されている直近結果との乖離だけ記録して止める。

## 評価順 (固定)

1. 上の 2 run を回して CSV 生成
2. diff を人間が読める形に整形 (1 CSV、同じ schema、行を TOW で合わせる)
3. 上記 5 項目を埋める
4. root cause 一行を `notes/2026-04-17_rtk_run2_lineage_plan.md` に追記する
5. run3 / run1 は触らない。触る必要が出たら別 plan にする。

## 想定所要時間

- ステップ 1 ビルド: 03:00
- ステップ 2 telemetry 実装 + unit test: 45:00
- ステップ 3 run2 full × 2 本: 08:00 (既存インフラ再利用)
- ステップ 4 分析: 30:00
- ステップ 5 所見記述: 15:00

合計 1:41 目安。途中で迷ったら止めて plan を修正する。

## 成功基準

- `accepted_overwrite_lineage_id` が両 CSV に出ており、run2 base / backup 間で分岐 epoch が一意に特定できる。
- root cause 一行が追記されている。
- solver 既定動作 (`satguard12`) の挙動が変わっていない (新 guard 入れていないため)。
- unit test 全通過。
- CI-only commit と混ざらない (`.github/**`, `scripts/ci/**` は touch しない)。

## 本 plan の範囲外

- 新しい guard の実装
- run1 onset regression の再試行
- v15_020 / v16_020 の default 化
- backup policy 調整
- Tokyo 以外の dataset

これらは lineage root cause が埋まった後に別 plan で扱う。

Root cause: run2 の backup regression は `177764.8` の `partial/variance_drop` 4->4 overwrite が lineage を `139 -> 140` に進めたことに起因し、以後 `177765.0 / 178089.6 / 178176.6-178177.8 / 178179.2-178190.0` で同 lineage が延命された結果 `178089.6` 付近で matched DD pair 不足に陥る。
