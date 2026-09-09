# Phase275: relative-height production handoff audit

Read-only code audit at implementation commit e2e8ae7; no raw, candidate,
truth or MAT payload reads. Production route ownership is distinct from
the deliberately fabricated velocities in the small insertion test.

In apps/native/gnss_fgo_imu_no_base.cpp:

- validatePhase91GnssFirstHandoff (around 3030) constructs retained source
  index/UTC/GPST keys for main and GNSS-first, plus solution times, and
  calls validateRetainedRawDAlignment against the immutable raw epoch table.
- The main flow around 12504 invokes that validator for the meter-state
  handoff. It also requires GNSS-first convergence and finite progress;
  failures in this lane return without a fallback.
- deriveGnssFirstVelocities (2833) requires complete optimized ECEF velocity
  coverage and converts those vectors to one ENU frame defined by the first
  same-run solution. It does not differentiate positions or read a file.
- The successful handoff around 12606 copies same-run solution positions
  into problem.epochs, after exact alignment validation. Clock C/D exports
  are separately validated and remain in memory.
- Around 12704 the same-run velocity vector is passed to buildImuInput.
  At 5737 it is copied directly into stop_velocity_seeds_nav. Heading
  smoothing is a separate computation; this vector is not replaced by
  smoothed heading velocities.
- The relative-height backend reads these epoch positions and velocities
  before solving, and the IMU detector's epoch_stop mask. It does not use
  generic position/velocity fallbacks. The GNSS-first configuration has
  the relative-height selector explicitly disabled.

The selected CLI lane therefore has a code-supported raw-to-same-run
handoff. The public FGOProblem value API cannot authenticate who constructed
its memory; configuration flags alone are not proof of arbitrary caller
provenance. Raw experimental evidence must include the pinned argv/binary,
native invocation count, exact alignment report, same-run handoff report,
and actual pair/factor counters. Do not infer this from the synthetic test.

Next freeze one H development run based on the operational Phase234 recipe
plus only --native-relative-height-pairs. Keep initial bias/velocity priors,
heading, affine geometry, and resL options at the baseline settings. Require
positive pair insertion, matching counters, finite no-fallback convergence
and exact output coverage before freezing the one-shot evaluator. If no
pairs are selected, do not lower thresholds after seeing the outcome.
H remains development data, not heldout or leaderboard evidence.
