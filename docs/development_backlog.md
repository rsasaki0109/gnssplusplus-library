# Development backlog (2026-08-08)

Product priority is defined by the
[application use-case roadmap](application_use_case_roadmap.md). This file is
the supporting engineering and station-operations backlog; it must not
pre-empt the active application release unless that release depends on it.

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
- Full run1 A/B (2026-08-08): fix 9085->9130 (+45) but <50cm 8644->8583
  (-61). adaptive-on adds 100 correct FIX and drops 55 correct FIX to FLOAT
  (status 4->3 at ~same position). Fix-count up but fix continuity and
  <50cm down, so **no default preset promotion** on this evidence.
- Open (future): full 3-city A/B and the usual bars if activation is ever
  desired; no preset promotion without a fresh run1-only plan.

## 2. Low-count AR surplus-quality relaxation — NOT ACTIVATED (root cause corrected)

- Implemented in PR #438: `--low-count-relax-surplus-quality` (default OFF).
  Corrects the prior falsification that scored against `ref_e_pos_m`.
- Design slice (Tokyo run1 5000--5499): FIX 333->335 (+2 at 0.021/0.032 m),
  zero wrong FIX, FIXED RMS unchanged.
- Full run1 (2026-08-08): FIX 6409->6423 but wrong FIX>0.5m 971->976.
  Corrective control (relax OFF) shows `--low-count-ar` alone gives the same
  976 -- the +5 wrong FIX is from the low-count path itself, NOT the
  relaxation. A separation witness
  (`--low-count-require-separation-witness`, default OFF; float/IMU
  separation <= 0.1 m) rejects the one low-count wrong fix (e3006) and drops
  the delta to +4, but the remaining +4 are `low_count_used=0` graph-side
  effects it cannot gate.
- Conclusion: neither the relaxation nor the witness is activated; the
  low-count AR path itself needs an independent gate before any activation.
  Sealed run2/run3 untouched.

## 3. PPC submission pipeline (user-decision item)

Ref: `HANDOFF_TO_CODEX.md` section 3c.

- `ppc-demo` scores the `gnss_solve` path, which ignores IMU. The
  `--navi776-tc` combined configuration only produces its PPC stream through
  `gnss_fuse --rtk-pos-out`. A submission flow over the fused stream needs
  separate design and is a user-judgement item. Not started.

## 4. Phase 3 TC main line (M0--M5) — DONE

Ref: `docs/tight_coupling.md`, `docs/navi776_techniques.md`.

- M0--M5 are implemented and evaluated in `docs/tight_coupling.md` (all
  default-off; M1/M3/M4 mixed-negative, M2/M5 gate-only).
- `--navi776-tc` already enables M3 closed loop + M4 velocity + SD Doppler
  + shared-alpha adaptive noise, with a five-run sign-off (tokyo1 fix
  79.77%). The prior "not folded into TC main line" note is outdated.
- A full-run1 A/B (`docs/fgo_navi776_tc_adaptr_combo.md`) shows combining the
  new adaptive-R knobs with the TC preset is worse: + phase-only 55.06% fix,
  + per-system 77.90% (vs 79.77% shared alpha). The TC main line keeps the
  shared-alpha adaptive noise; the new knobs are not folded in.

## 5. Workspace hygiene (done 2026-08-07)

- main worktree moved to `build-base` (tracks origin/develop) at the PR #437
  merge commit; local build directories are now git-ignored
  (`build-fgo-*`, `build-gtsam-*`, `build-madoca-*`, `build-wsl-madoca-*`,
  `build-fuse-current`).
- Development continues in the `gnssplusplus-library-ffrt-shadow` worktree on
  `develop`.

## 6. Long-term operational product roadmap (planned; reviewed 2026-08-20)

The long-term product direction is to evolve `gnss station` from a process
supervisor into a trusted GNSS station service. The service should cover
provisioning, continuous operation, measurement-quality judgement, recovery,
and reproducible delivery of solution artifacts. This section records the
plan only; it does not activate any of the items below.

### Phase 1: make measurement quality explicit

- Define a versioned station-quality contract that separates process health
  from measurement health.
- Report FIX/FLOAT/SINGLE/NONE, correction age, solution freshness, satellite
  count, position jumps, and time gaps.
- Emit `usable`, `degraded`, or `unusable` plus machine-readable
  `reason_codes`.
- Build a replay corpus and long-running soak tests before changing defaults.

The first acceptance bar is that a live process with stale or unusable
measurements cannot be reported as healthy.

### Phase 2: make unattended field operation safe

- Package the station as a systemd/Docker service with graceful shutdown and
  recovery after process, network, or power interruption.
- Add disk-budget enforcement, retention and rotation for logs, RTCM, RINEX,
  and solution artifacts.
- Export health and quality metrics for monitoring and add actionable alerts.
- Produce a support bundle containing configuration, status history, and
  relevant logs without leaking credentials.

### Phase 3: make the data contract reusable

- Expose station status and solution events through REST/WebSocket and
  Prometheus-compatible interfaces.
- Keep the JSON contract usable by the local web UI, ROS2, robotics tooling,
  and offline analysis without per-consumer parsing rules.
- Attach configuration, software version, receiver identity, and correction
  source metadata to each result for reproducibility.

### Phase 4: manage multiple stations securely

- Add station inventory, fleet views, remote status/log collection, and
  controlled restart/configuration operations.
- Version configurations with validation, staged rollout, and rollback.
- Add TLS/mTLS, role-based access control, secret management, and audit logs.
- Support correction-source failover and station redundancy where the field
  deployment requires it.

### Phase 5: scale the data and extension surface

- Preserve raw and derived data with checksums, retention policies, and
  replayable provenance.
- Add stable adapters for GIS, CSV/Parquet, MQTT, and other downstream users.
- Define plugin boundaries for receiver protocols, correction sources, quality
  policies, and output sinks.
- Publish hardware compatibility, release, and support matrices.

### Provisional operating targets

These are planning targets to calibrate with real deployments, not current
guarantees:

- recover from a routine service failure within 60 seconds;
- pass 24-hour and 7-day continuous-operation soaks;
- keep credentials out of logs and run artifacts;
- reproduce a result from the same recorded input and configuration; and
- bring a new station from configuration to validated operation in 15 minutes.

The sequencing rule is measurement trust first, then unattended operation,
then remote fleet control. Deterministic quality rules and replay evidence
must precede predictive or automated tuning features.

## Discipline reminders

- Default-OFF, OFF bit-identical (md5/CSV equality) for every new feature.
- Negative results committed honestly; sealed run2/run3 used only after a
  full-run1 activation pass.
- No push/PR/merge without explicit user instruction.
