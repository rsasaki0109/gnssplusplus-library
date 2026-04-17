# RTK experimental additions — PR 化 handoff plan (Codex 実作業用)

更新日: 2026-04-17
作業ブランチ: `codex/rtk-demo5-parity` (develop から 84 commits ahead, 85 commits behind)
計画担当: Claude / 実作業担当: Codex

## この文書の使い方

- 今日 (2026-04-17) に追加された RTK telemetry + guard prototypes + analysis script + plan notes を、develop に小さく landing させるための plan。
- 元 worktree `gnssplusplus-library/` は CI / PPP / Docker / docs / RTK historical work などが混ざった大きな dirty state。**その全てを landing することはしない**。
- 取り扱い対象は **2 段階**:
  - **Phase 1 (安全、自動化可)**: 新規 untracked ファイル (plan notes + analysis script) だけを docs-style draft PR にする。
  - **Phase 2 (手作業、sasaki 判断必要)**: RTK core file (`rtk.cpp` / `rtk.hpp` / `gnss_solve.cpp` / `test_rtk_legacy.cpp`) の「今日分の追加」だけを cherry-pick する。これは自動化せず、本 plan 実行対象外。
- Codex はまず Phase 1 だけやる。

## Phase 1 の scope

landing するファイル:

- `notes/2026-04-17_*.md` (7 本)
  - `2026-04-17_rtk_run2_lineage_plan.md`
  - `2026-04-17_rtk_run2_lineage_guard_ab_plan.md`
  - `2026-04-17_rtk_177057_early_divergence_plan.md`
  - `2026-04-17_rtk_preserve_lineage_seed_plan.md`
  - `2026-04-17_rtk_preserve_net_effect_ab_plan.md`
  - `2026-04-17_rtk_preserve_threshold_sweep_plan.md`
  - `2026-04-17_rtk_demo5_final_status.md`
  - `2026-04-17_rtk_experimental_pr_handoff_plan.md` (本 plan)
- `scripts/analysis/lineage_divergence_diff.py` (新規、self-check 付き)

landing しない (Phase 2 送り):

- `src/algorithms/rtk.cpp`, `include/libgnss++/algorithms/rtk.hpp`, `apps/gnss_solve.cpp`, `tests/test_rtk_legacy.cpp`
- 理由: 今日の追加 (telemetry 2 fields + A2 guard + A3 guard + 新 unit test 6 本) が、過去 84 commits 分の RTK refactoring 差分と同ファイル内で intermix している。自動 cherry-pick は破綻リスクが高い。

## 禁止事項

- RTK core file を無理に PR に含めない (`src/algorithms/rtk*.cpp`, `include/libgnss++/algorithms/rtk*.hpp`, `apps/gnss_solve.cpp`, `tests/test_rtk_legacy.cpp`)
- CI 側ファイル (`.github/workflows/*`, `scripts/ci/*`, `CONTRIBUTING.md`, `docs/contributing.md`) は既に develop に入っているので再 landing しない
- PPP / Docker / .gitignore / docs/decisions.md / docs/experiments.md / apps/gnss_ppc_demo.py / apps/gnss_odaiba_benchmark.py / apps/gnss_ppc_rtk_signoff.py は触らない
- `output/benchmark/**` は artifact で、commit しない
- `.github/**` への書き込み禁止
- 元 worktree `gnssplusplus-library/` の dirty state は触らない (reset / stash / checkout しない)
- `--dangerously` 系 force push は使わない

## ステップ 1: validation worktree パターンで新 worktree 作成

```bash
cd /media/sasaki/aiueo/ai_coding_ws/rtklib_v2_rtk_ws
git --git-dir=gnssplusplus-library/.git worktree add \
  -b codex/rtk-experimental-notes \
  gnssplusplus-rtk-experimental \
  origin/develop
```

もしくは fetch 済みでなければ先に `git fetch origin develop --prune` を実行してから。

## ステップ 2: plan notes を新 worktree にコピー

元 worktree 側の path を source、新 worktree 側の path を destination:

```bash
cd /media/sasaki/aiueo/ai_coding_ws/rtklib_v2_rtk_ws/gnssplusplus-rtk-experimental
mkdir -p notes scripts/analysis
cp ../gnssplusplus-library/notes/2026-04-17_rtk_run2_lineage_plan.md notes/
cp ../gnssplusplus-library/notes/2026-04-17_rtk_run2_lineage_guard_ab_plan.md notes/
cp ../gnssplusplus-library/notes/2026-04-17_rtk_177057_early_divergence_plan.md notes/
cp ../gnssplusplus-library/notes/2026-04-17_rtk_preserve_lineage_seed_plan.md notes/
cp ../gnssplusplus-library/notes/2026-04-17_rtk_preserve_net_effect_ab_plan.md notes/
cp ../gnssplusplus-library/notes/2026-04-17_rtk_preserve_threshold_sweep_plan.md notes/
cp ../gnssplusplus-library/notes/2026-04-17_rtk_demo5_final_status.md notes/
cp ../gnssplusplus-library/notes/2026-04-17_rtk_experimental_pr_handoff_plan.md notes/
cp ../gnssplusplus-library/scripts/analysis/lineage_divergence_diff.py scripts/analysis/
```

## ステップ 3: 確認と軽量検証

```bash
cd /media/sasaki/aiueo/ai_coding_ws/rtklib_v2_rtk_ws/gnssplusplus-rtk-experimental
git status --short --branch
python3 scripts/analysis/lineage_divergence_diff.py --self-check
```

`git status` に上記 copy 済みファイル 9 本 (notes 8 + script 1) だけが untracked で出ることを確認。RTK core file や CI file が紛れ込んでいないことを確認。

self-check が pass しなければ、script を修正せず、plan 末尾に「self-check 失敗、Phase 1 中断」とだけ書いて終了。

## ステップ 4: commit と push

```bash
git add notes/2026-04-17_*.md scripts/analysis/lineage_divergence_diff.py
git status --short
git commit -m "$(cat <<'COMMIT_EOF'
Add RTK demo5 parity 2026-04-17 investigation notes and analysis script

今日の RTK demo5 parity investigation で以下を確立した結果を notes/ と scripts/ に残す:

- Tokyo run1 で libgnss++ satguard12 は精度 4 指標で demo5 を明確に上回り、Fix 数のみ -21 (0.9%) の gap
- 4 回の guard 設計試行 (A2 / A3 / P2 threshold sweep) は全て不採用、`satguard12` は local optimum と確定
- 177764.8 の lineage advance は symptom、cause は 177057.4 の preserve seed で lineage drift
- 次 iteration は DOP / receiver-motion / inter-system-bias など別 signal へ

scripts/analysis/lineage_divergence_diff.py は base/backup CSV 突合で held_pair_keys overlap と lineage ID trajectory を分析する標準ツールとして残す (self-check 付き)。

RTK core file の experimental option (telemetry + guard prototypes) の landing は別 PR で扱う。
COMMIT_EOF
)"
git push -u origin codex/rtk-experimental-notes
```

## ステップ 5: draft PR 作成

```bash
gh pr create --repo rsasaki0109/gnssplusplus-library \
  --base develop \
  --head codex/rtk-experimental-notes \
  --draft \
  --title "[docs] RTK demo5 parity 2026-04-17 investigation notes + lineage diff tool" \
  --body "$(cat <<'PR_EOF'
## Summary
- 2026-04-17 に実施した RTK demo5 parity 調査の plan notes 8 本と、lineage divergence 分析スクリプトを landing する docs-only PR
- RTK core file (`src/algorithms/rtk.cpp` 等) の変更は本 PR には含まれない。別 PR で段階的に扱う
- 結論: Tokyo RTK で libgnss++ は精度 4 指標で RTKLIB demo5 を上回り、Fix 数のみ -21 (0.9%) の差で「実質同等以上」。詳細は \`notes/2026-04-17_rtk_demo5_final_status.md\`

## Scope
- notes/2026-04-17_rtk_*.md 8 本 (planning + analysis + final status)
- scripts/analysis/lineage_divergence_diff.py (base/backup CSV 突合ツール、self-check 付き)

## Not in this PR
- RTK core file への telemetry / guard 追加 (next PR)
- 過去 84 commits の RTK refactoring 差分 (別途 triage 必要)

## Test plan
- [x] \`python3 scripts/analysis/lineage_divergence_diff.py --self-check\`
- [ ] reviewer が notes の論理を確認
PR_EOF
)"
```

PR URL を plan 末尾に記録する。

## ステップ 6: 元 worktree の dirty state は触らない

Phase 2 の landing は sasaki の判断 (rtk.cpp 差分の手作業 triage) が必要なので、本 plan 実行では元 worktree を reset / stash しない。

## 許容される逸脱

- ステップ 1 の worktree 追加で branch name 衝突が発生した場合、`-b codex/rtk-experimental-notes-2` で再試行して plan 末尾にどの branch 名を使ったか追記する。
- `gh pr create` が GitHub connector の権限不足 (`403 Resource not accessible by integration`) で失敗した場合、PR #16 と同じで CLI (`gh pr create --draft`) が fallback。permission 問題で fallback も失敗したら、push 済みブランチ URL を plan 末尾に貼って手動 PR 化を sasaki に依頼する旨を書き終了。
- script の self-check が `--self-check` オプションそのものを持っていない場合、script を変更せず「self-check 未実装、smoke test だけやる」と plan 末尾に記録。

## 想定所要時間

- ステップ 1 worktree 作成: 02:00
- ステップ 2 copy: 03:00
- ステップ 3 検証: 03:00
- ステップ 4 commit + push: 03:00
- ステップ 5 PR 作成: 03:00
- ステップ 6 なし: 00:00

合計 14 分目安。

## 成功基準

- 新 worktree `gnssplusplus-rtk-experimental` が clean で develop base に存在
- `codex/rtk-experimental-notes` branch が origin に push 済み
- draft PR が作成され URL が plan 末尾に記録
- `git status` (元 worktree) が実行前後で同じ dirty state (触っていないことの確認)
- PR には notes 8 + script 1 の 9 files だけが含まれる

## Phase 2 (本 plan 対象外)

Phase 2 は以下の未決事項があり、sasaki が手作業 triage するまで実行不可:

- `src/algorithms/rtk.cpp` の historical 84-commit 差分と今日の追加 (telemetry fields + A2 guard + A3 guard) をどう分離するか
- `include/libgnss++/algorithms/rtk.hpp` の DebugTelemetry / RTKConfig 追加行が historical refactoring と干渉していないか
- `tests/test_rtk_legacy.cpp` の新 unit test 6 本 (lineage + A2 + A3) だけを切り出せるか
- 既存の `apps/gnss_solve.cpp` CLI 追加 (3 flag family) を既存 CLI と衝突なく並べられるか

これらが triage 済みになった時点で、Phase 2 plan を別途書いて Codex に handoff する。

## 参照

- CI PR #16 (既に develop に merge 済み): `codex/ci-pipeline-validation` 切り出しの前例
- RTK final status: `notes/2026-04-17_rtk_demo5_final_status.md`
