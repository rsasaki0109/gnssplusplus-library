# RTK demo5 parity — 177057.4 早期分岐追跡計画 (Codex 実作業用)

更新日: 2026-04-17
作業ブランチ: `codex/rtk-demo5-parity` (dirty state 前提、直前 2 plan の telemetry + A2 guard prototype が入った状態)
計画担当: Claude / 実作業担当: Codex

## この文書の使い方

- 直前 2 plan:
  - `notes/2026-04-17_rtk_run2_lineage_plan.md` (telemetry 追加で `177764.8` の lineage 139→140 を特定)
  - `notes/2026-04-17_rtk_run2_lineage_guard_ab_plan.md` (177764.8 直接 block は -30 で不採用 → symptom であり cause ではないと確定)
- 既存 CSV は前 plan が既に生成済み:
  - `output/benchmark/tokyo_run2/ar_sweep/run2_full_base_lineage.csv` (A0 baseline)
  - `output/benchmark/tokyo_run2/ar_sweep/run2_full_backup_lineage.csv` (A1 backup-on)
- 本 plan は **analysis のみ**。solver コード変更なし。benchmark 再実行なし。A2 guard prototype には触らない。
- 目的は `177057.4` (base vs backup の global first divergence) がなぜ起きて、どの経路で `177764.8` の lineage 140 まで続くかの **narrative** を出すこと。
- narrative が出たら、次 plan で試す「1 hypothesis」を末尾に書く。それで本 plan 終了。

## 先に結論 (仮説)

- backup-on が `177057.4` で base と異なる accepted path を取るのは、**早い段階の backup-use が alternate held set を seed** したため。
- その alternate held set は `177057.4 → 177764.8` の 700 秒間、overlap は常に 0.5 以上で lineage ID は進まない (= 同一 lineage 内の drift)。
- しかし held set の content は徐々に base と乖離し、`177764.8` で overlap <0.5 の variance_drop overwrite が入ったとき、base は G06/G09 中心、backup は G06/G19 中心という根本的に違う集合に着地する。
- つまり `177764.8` の harmful さは「その epoch 単独では判断できない」=「lineage 内 drift の累積指標」を見ないといけない。

仮説が正しければ、次に試すのは「accepted overwrite を block する guard」ではなく、**lineage 内 drift 累積量をベースにした held-set 健全性メトリクス** の導入。

## Codex 次作業: 1 ステップだけ

### やること

既存 2 CSV (`run2_full_base_lineage.csv` と `run2_full_backup_lineage.csv`) を突き合わせて `177057.4 → 177770.0` の 700 秒窓で以下を炙り出す:

1. 最初に `held_pair_keys` が差分を持った epoch (= first divergence)
2. その epoch での base/backup の accepted path 差分
3. `177057.4 → 177764.8` 間で発生した backup-use イベント全件の timeline
4. lineage ID trajectory (base と backup をそれぞれ列方向で並べて可視化)
5. `held_pair_keys` overlap ratio を base vs backup で per-epoch 計算し、0.5 を下回った epoch (lineage 内 drift が破綻した瞬間)
6. `accepted_overwrite_after_pair_keys` が base と backup で完全に disjoint になった最初の epoch

これらから、`177057.4` が:

- 単なる early backup-use の副作用で自己修復不能なのか
- それとも明示的に block できる 1 つの決定点を持つのか

を narrative で判定する。

### 禁止事項

- solver コード (`src/`、`include/`、`apps/gnss_solve.cpp`、`tests/*.cpp`) を触らない
- benchmark を再実行しない (既存 CSV のみ使う)
- A2 guard prototype (`--max-lineage-advance-overlap-ratio` 等) を revert しない (= 現状の worktree 差分に手を入れない)
- 新しい C++ telemetry field を追加しない
- pair 名を分析ロジックに hardcode しない (pair key は opaque string として扱う)
- CI/CD・PPP・Docker を触らない (`.github/**`、`scripts/ci/**`、`apps/ppp_*.cpp` は禁止)
- run3 / run1 を再実行しない

## ステップ 1: 分析スクリプト作成

ファイル: `scripts/analysis/lineage_divergence_diff.py` (新規作成可)

要求仕様:

- 入力: `--base-csv <path> --backup-csv <path> --out-md <path> --window-start <tow> --window-end <tow>`
- 出力 markdown (`--out-md`) に以下のセクションを生成:
  - `## Window summary`: window 内 epoch 数、base/backup それぞれの fix 率、backup-use 回数
  - `## First divergence`: `held_pair_keys` が最初に異なった epoch と、その epoch の両 run の accepted_fix_source / accepted_candidate_path / accepted_overwrite_before_pair_keys / accepted_overwrite_after_pair_keys / hold_used_preserved_backup / lineage ID
  - `## Overlap trajectory`: window 内で overlap ratio (base_held vs backup_held) の min / mean / median / <0.5 になった最初の epoch / 0.5 から回復せず lineage ID が進んだ最初の epoch
  - `## Lineage ID trajectory`: base と backup それぞれの lineage ID 変化点 (list of `(tow, id, origin_tow, source, path, overlap_ratio)`)
  - `## Backup-use timeline`: backup-on の CSV から `hold_used_preserved_backup == 1` の epoch 全件 (tow, source, path, before/after pair keys)
  - `## Disjoint point`: base と backup の `held_pair_keys` が初めて完全に交わらなくなった epoch (該当なしなら "none")
  - `## Decisive branch candidates`: 上記から推定される 1-3 個の決定点候補とその根拠 (1-2 文)
- pair key は opaque string として `|` split してそのまま set 比較する。名前 (`G19>G06` 等) の中身は touch しない。
- `window-start` / `window-end` 未指定時は CSV 全範囲。デフォルト window は `--window-start 177050 --window-end 177770`。
- 欠損列があれば `KeyError` で落ちて、どの列が足りないかを stderr に出す。

実装は pandas でよい (既存 `requirements-docs.txt` に pandas 入っていれば流用、なければ pure csv で書く)。

unit test:

- `tests/` に Python test を作らない。既存 C++ test 層とは別口なので、`tests/` には手を入れない。
- 代わりに `scripts/analysis/lineage_divergence_diff.py` 本体末尾に `if __name__ == "__main__":` の引数処理と、`--self-check` フラグを付けて 8-10 行の合成データで smoke test を走らせる。
  - `--self-check` 時は in-memory DataFrame を組み、expected な `overlap_ratio` / `disjoint_point` 値を assert する。
  - 失敗時 exit code 1。

## ステップ 2: 実行

```bash
cd /media/sasaki/aiueo/ai_coding_ws/rtklib_v2_rtk_ws/gnssplusplus-library

python3 scripts/analysis/lineage_divergence_diff.py --self-check

python3 scripts/analysis/lineage_divergence_diff.py \
  --base-csv output/benchmark/tokyo_run2/ar_sweep/run2_full_base_lineage.csv \
  --backup-csv output/benchmark/tokyo_run2/ar_sweep/run2_full_backup_lineage.csv \
  --out-md output/benchmark/tokyo_run2/ar_sweep/run2_early_divergence_diff.md \
  --window-start 177050 \
  --window-end 177770
```

## ステップ 3: narrative 作成

`notes/2026-04-17_rtk_177057_early_divergence_plan.md` の末尾 (本ファイル) に以下の構成で追記する:

### 3.1 確認した事実 (最大 10 項目、1 行ずつ)

- `Disjoint point` の epoch と、そのときの両 run の accepted path
- `177057.4` における両 run の accepted path と lineage ID の状態
- 窓内の backup-use 数と分布 (連続クラスタ / 単発の判定)
- `held_pair_keys` が最初に overlap <1.0 になった epoch

### 3.2 Narrative (3-5 文)

- `177057.4` で base と backup が分岐する mechanism を 1 段の因果で説明する。
- 自己修復不能性の有無を述べる (= 以後 overlap が 1.0 まで戻ったかどうか)。
- `177764.8` への因果を 1 文で繋ぐ。

### 3.3 次 1-hypothesis

下記の形式で 1 案だけ書く (複数は書かない):

> hypothesis: `<識別子>` = `<判定条件>` の epoch で `<予防アクション>` を入れると run2 alternate lineage の seed を止められ、`177764.8` まで届かずに base lineage 139 と合流する。

判定条件には pair 名を hardcode しない。識別子は既存 telemetry 列 (`hold_used_preserved_backup`, `accepted_preserved_prior_hold_state`, `accepted_overlap_ratio`, `accepted_overwrite_lineage_id`, `accepted_fix_source`, `accepted_candidate_path`) の組み合わせで書く。

## ステップ 4: 採用判断

narrative と hypothesis が書けた時点で、次のどれかの決定を plan 末尾に 1 行で書く:

- **進行**: hypothesis が検証可能な形で書けた。次 plan で guard または telemetry を prototype する。
- **撤退**: `177057.4` での分岐が既存 telemetry だけでは説明しきれず、追加 C++ telemetry が必要。必要 field 名を 3 行までで書き、次 plan で telemetry 追加に戻る。
- **仕切り直し**: CSV の既存列では `held_pair_keys` 同定が曖昧で Narrative が書けない。CSV スキーマの問題点を 5 行までで書き、次 plan で CSV 書き換えに戻る。

## 許容される逸脱

- 既存 CSV に `accepted_overwrite_lineage_id` または `accepted_overwrite_lineage_origin_tow` が欠けていた場合 (= 前 plan の run 以前の古い CSV だった場合)、停止して「CSV 再生成が必要」とだけ plan 末尾に書く。再実行はしない。
- pandas が使えない環境なら `csv` モジュールだけで書く。遅くても受容する。

## 想定所要時間

- ステップ 1 script 作成 + self-check: 25:00
- ステップ 2 実行 + 結果読み: 05:00
- ステップ 3 narrative: 25:00
- ステップ 4 採用判断: 10:00

合計 1:05 目安。前 2 plan より短い。これ以上掛かるなら仕切り直し。

## 成功基準

- `scripts/analysis/lineage_divergence_diff.py` が self-check pass。
- `run2_early_divergence_diff.md` が生成され、6 つのセクションすべてが空でない。
- plan 末尾に narrative 3-5 文と 1 hypothesis が追記されている。
- 採用判断 1 行が追記されている。
- solver コード未変更 (`git diff --stat src/ include/ apps/gnss_solve.cpp tests/` が空)。

## 本 plan の範囲外

- 新 guard 実装
- solver 既定挙動の変更
- A2 guard prototype の revert / 残置判断
- benchmark 再実行
- Tokyo 以外の dataset
- run3 full / run1 full 実行

## 参照

- 直前 plan: `notes/2026-04-17_rtk_run2_lineage_guard_ab_plan.md` (A2 不採用の根拠)
- 前々 plan: `notes/2026-04-17_rtk_run2_lineage_plan.md` (telemetry 追加と `177057.4` を global first divergence として初記録)
- 過去 handoff: `notes/2026-04-14_rtk_demo5_handoff.md`

## Codex 実行結果 (2026-04-17)

生成物: `output/benchmark/tokyo_run2/ar_sweep/run2_early_divergence_diff.md`

### 3.1 確認した事実

- window 内 common epoch は 3601、fix 率は base 3065/3601 (85.12%)、backup 3373/3601 (93.67%)。
- `hold_used_preserved_backup == 1` は 5 回で、177664.2、177667.2/177667.6、177688.6/177689.2 の後半クラスタに限られ、177057.4 近傍には出ていない。
- `held_pair_keys` (`hold_held_pair_keys`) が最初に差分を持つのは 177057.2 で、両 run とも accepted path は `hold/hold`、lineage ID は 2。
- `held_pair_keys` overlap は 177057.2 で初めて 1.0 未満かつ 0.5 未満になり、177057.4 で 0.5 回復前に backup lineage が 3 へ進んだ。
- 177057.4 では base が `partial/variance_drop` で lineage 2 に残り、backup は `partial/variance_drop` で `accepted_preserved_prior_hold_state == 1` のまま lineage 3 に進んだ。
- 177057.6 でも backup は `accepted_preserved_prior_hold_state == 1` の `partial/variance_drop` で lineage 4 に進み、base は lineage 2 に残った。
- overlap は後続で 1.0 まで戻る epoch があるため、初期分岐は held-set content としては完全な自己修復不能ではない。
- first accepted-overwrite-after disjoint point は 177281.2 で、base は `partial/variance_drop`、backup は `hold/hold`。
- first held-set disjoint point は 177281.4 で、両 run とも accepted path は `partial/variance_drop`。
- 177764.8 では base が `none/none` かつ `drop_nonfixed_jump` で lineage 59 に残り、backup は `partial/variance_drop` で lineage 140 に進んだ。

### 3.2 Narrative

177057.2 で base と backup の held-set は先にずれ始めるが、この epoch 自体は両 run とも `hold/hold` で lineage ID も 2 のままなので、決定的な採用分岐は次の 177057.4 にある。177057.4 では backup だけが `accepted_preserved_prior_hold_state == 1` の `partial/variance_drop` 採用で lineage 3 に進み、177057.6 でも同条件で lineage 4 に進むため、これは `hold_used_preserved_backup` ではなく preserved-prior state を通じた alternate lineage seed と見るのが妥当。held-set overlap は後で 1.0 に戻るため、初期分岐は content レベルでは一度も修復不能にならないわけではないが、177281.2/177281.4 で accepted-after と held-set が完全 disjoint になり、以後の backup lineage は低 overlap の枝を多数作る。したがって 177764.8 は単独の初発 harmful overwrite ではなく、177057.4 付近の preserved-prior low-overlap lineage advance から始まった分岐が、177281 台の完全 disjoint を経由して lineage 140 まで届いた downstream symptom と判定する。

### 3.3 次 1-hypothesis

> hypothesis: `preserved-prior-low-overlap-advance` = `hold_used_preserved_backup == 0 && accepted_preserved_prior_hold_state == 1 && accepted_overlap_ratio < 0.5 && accepted_fix_source == partial && accepted_candidate_path == variance_drop && accepted_overwrite_lineage_id advances` の epoch で preserved-prior held-state adoption を拒否して非 preserved 採用または prior lineage 維持へ戻すと run2 alternate lineage の seed を止められ、`177764.8` まで届かずに base lineage 139 と合流する。

**進行**: hypothesis が既存 telemetry 列だけで検証可能な形で書けた。次 plan で guard または telemetry を prototype する。
