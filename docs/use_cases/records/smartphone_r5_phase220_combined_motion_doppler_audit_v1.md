# Phase220: combined native main motion/Doppler admission audit

Primary-agent read-only production audit after Phase219; no solver invocation,
candidate/truth read, MAT access, or score calculation in this audit.

## Findings from the current implementation

- `src/algorithms/fgo_gtsam_backend.cpp:178` explicitly rejects Phase213
  when Phase217 is selected. The CLI repeats this at
  `apps/native/gnss_fgo_imu_no_base.cpp:868`. Thus passing both existing
  flags cannot currently produce the intended experiment.
- Main Doppler insertion at backend line 1955 consumes validated corrected
  raw range-rate rows. It rotates ECEF LOS once into the IMU navigation ENU
  frame and constrains the existing velocity and scalar drift states.
- Main motion insertion at backend line 2412 constrains existing Pose3
  translations and those same ENU velocity states with trapezoidal motion.
  It adds no velocity or drift states and does not read a saved trajectory.
- These are distinct likelihoods sharing velocity states, not duplicate
  insertions of the same measurement. Their coexistence is structurally
  plausible; this source inspection alone does not prove numerical validity
  or accuracy improvement.
- Both selectors are explicitly disabled for GNSS-first configuration at
  app lines 12091-12092. Main corrected Doppler rows are transferred from
  the same-run raw stage at app line 12185.
- The Phase171 synthetic handoff fixture already accepts both selector
  parameters, but its line 7373 currently expects rejection. The CLI
  Phase217 fixture also expects rejection for the paired flags.

## Bounded next implementation and evidence requirements

Permit the two explicit existing selectors together by removing only their
mutual exclusion in CLI/backend admission; retain Pixel5, Phase171, legacy
IMU, zero-lever-arm and no duplicate position-motion requirements. Leave
both defaults off and retain all Phase201/205/209 exclusions. Update the
combined synthetic handoff case to assert both factor counts and convergence;
retain standalone and malformed corrected-D-row tests. Replace the obsolete
CLI pair-rejection case with a configuration-admission test that does not
mistake missing raw files for a successful solve.

Build and run focused C++/CLI regression tests before freezing a raw-only
combined experiment. Do not change noise, robust thresholds, integration,
initialization, or epoch masks in the combination. Expected H graph size is
252384 = 182560 baseline + 66685 main Doppler + 3139 main motion; values
remain 15700. These are pre-run expectations, not observed results.

Compare the eventual scalar with Phase218/219 best 1.2689788175187473 m.
H remains repeatedly used development data, not heldout or leaderboard proof.
