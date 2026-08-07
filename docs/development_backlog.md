# Development backlog (2026-08-07)

Status after the FGO quarantine/arc-restart audit thread (PR #437) and the
design-slice FLOAT root-cause analysis (`docs/fgo_design_slice_float_root_cause.md`).
All items below are open; none has sealed-run2/run3 entitlement yet.

## 1. Adaptive-R phase-only variant / per-system alpha (navi776 thread)

Ref: `docs/navi776_techniques.md` line 87-88 ("phase-only adaptation,
per-system alpha" listed as not pursued).

- Base: `--rtk-adaptive-noise` (+ `--rtk-adaptive-noise-max-baseline 1000`)
  is implemented and documented. It adapts the measurement variance at the
  SD unit with a shared alpha (phase 0.9, code 0.5).
- Open: a phase-only variant (adapt carrier variance only, leave code
  fixed) and/or per-system alpha (GPS vs GAL vs BDS vs QZSS weights).
- Gates: default-OFF, OFF bit-identical (md5), full 3-city A/B (tokyo
  run1/run3 + nagoya run1) with the usual bars (fix +/-0.5 pp, p95 no
  regression, wall <= +5%). One retune per phase.
- Precondition: a fresh run1-only plan before any activation work.

## 2. Float-ambiguity quality improvement (design-slice thread)

Ref: `docs/fgo_design_slice_float_root_cause.md` conclusion.

- The Tokyo run1 5000--5499 slice has 167 FLOAT epochs; 166 sit 50--500 m
  from truth and 45 fail AR because CMC level exclusion drops the eligible
  ambiguity count below the floor of 6.
- All four tested recoveries (arc-restart, low-count AR, surplus-gate
  relaxation, CMC-level relaxation) produced km-scale wrong FIX, proving the
  safety mechanisms are correct.
- Open: improve the quality of the float ambiguity estimate itself so the
  CMC-excluded / held epochs carry a correct float solution before AR. This
  is a distinct research thread from threshold relaxation; no activation
  without a fresh run1-only plan and a wrong-FIX gate.

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
