# Development backlog (2026-08-08)

Status after PR #437 (FGO quarantine/arc-restart audits), PR #438
(adaptive-R phase-only/per-system alpha + low-count AR surplus-quality
relaxation), and the adaptive-R real-data validation
(`docs/fgo_adaptive_r_validation.md`). Sealed run2/run3 have no entitlement
yet.

## 1. Adaptive-R phase-only variant / per-system alpha (navi776 thread) — DONE

Ref: `docs/navi776_techniques.md` line 87-88 ("phase-only adaptation,
per-system alpha" listed as not pursued).

- Implemented in PR #438: `--rtk-adaptive-noise-phase-only` and
  `--rtk-adaptive-noise-per-system-alpha`, both default OFF and
  bit-identical when off.
- Real-data validation (PR #439, `docs/fgo_adaptive_r_validation.md`):
  first-500-epoch Tokyo run1 A/B shows adaptive-on rmsH 0.0508->0.0471 m,
  phase-only tracking OFF (0.0507 m, confirming the code term is the main
  driver), and per-system marginally best (0.0469 m). The design-slice
  window is FLOAT-dominated and not an adaptive-R evaluation window.
- Open (future): full run1 A/B for fix-rate impact; no preset promotion
  without a fresh run1-only plan and the usual 3-city bars.

## 2. Low-count AR surplus-quality relaxation — DESIGN-SLICE PASSED

- Implemented in PR #438: `--low-count-relax-surplus-quality` (default OFF).
  Corrects the prior falsification that scored against `ref_e_pos_m`.
- Design slice (Tokyo run1 5000--5499): FIX 333->335 (+2 at 0.021/0.032 m),
  zero wrong FIX, FIXED RMS unchanged. The +12.1% wall comes from the
  low-count attempts.
- **In progress**: full run1 A/B to establish the activation bar and the
  run2/run3 pathway.

## 3. PPC submission pipeline (user-decision item)

Ref: `HANDOFF_TO_CODEX.md` section 3c.

- `ppc-demo` scores the `gnss_solve` path, which ignores IMU. The
  `--navi776-tc` combined configuration only produces its PPC stream through
  `gnss_fuse --rtk-pos-out`. A submission flow over the fused stream needs
  separate design and is a user-judgement item. Not started.

## 4. Phase 3 TC main line (M0--M5) reflection

Ref: `docs/tight_coupling.md`.

- The navi776 combined configuration's improvements (adaptive noise + SD
  Doppler rows, `--navi776-tc`) have not been folded into the Phase 3 TC
  main line. Open for planning; no run entitlement.

## 5. Workspace hygiene (done 2026-08-07)

- main worktree moved to `build-base` (tracks origin/develop) at the PR #437
  merge commit; local build directories are now git-ignored
  (`build-fgo-*`, `build-gtsam-*`, `build-madoca-*`, `build-wsl-madoca-*`,
  `build-fuse-current`).
- Development continues in the `gnssplusplus-library-ffrt-shadow` worktree on
  `develop`.

## Discipline reminders

- Default-OFF, OFF bit-identical (md5/CSV equality) for every new feature.
- Negative results committed honestly; sealed run2/run3 used only after a
  full-run1 activation pass.
- No push/PR/merge without explicit user instruction.
