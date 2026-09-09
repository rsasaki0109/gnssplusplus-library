# Phase328: preregister a finite-support-only correction ablation

To separate the remaining paired regression, add explicit default-off
`--native-base-mask-only-ablation`. It requires paired states and dense base
smoothing. Build the actual raw base model and query its actual corrections,
preserving missing streams, failed lookup and nonfinite returned corrections.
Only a finite returned correction is replaced with zero before the existing
application transaction. Retained P measurements therefore remain unchanged;
all source state, zero-group-delay, SPP, IMU and solver settings stay fixed.
This is diagnostic selection, not a source positioning correction algorithm.

Provenance includes `native_base_mask_only_ablation` and the top-level
`native_base_nonzero_correction_enabled` (false for this ablation). Reused
per-row applied/pass telemetry means the zero-valued transaction executed,
not that the actual base residual was subtracted. Base-model correction
statistics still describe actual residuals; application statistics describe
zero applied values. Read these with the explicit ablation flag, never label
this run a full compensated candidate or promote it silently.

Synthetic test compares real correction with mask-only on finite, NaN,
unavailable and missing-stream cases: retained identity/count and failure
taxonomy match, and the retained mask-only P value equals its input exactly.
An absent underlying callback stays absent, preserving contract rejection.

Before any run/evaluation, define the comparison: frozen Phase326 paired/dense
recipe plus the single ablation flag, no tuning or missing-observation fill.
Require the same retained count/identities when checked in the raw experiment,
valid in-memory handoff and exact UTC output before scoring. If the large
regression disappears, subtraction of correction values is implicated in this
fixed configuration; if it persists, investigate the mask and code-bias/seed
policy. Neither result alone establishes an isolated universal causal effect.
Do not fit coordinate shifts or correction scales to H truth.

No raw route, truth, saved candidate, MAT payload or accuracy evaluation was
accessed in this implementation phase. Baseline stays operational and the
0.782-class/leaderboard objective remains unverified.

Both `gnss_run_tests` and `gnss_fgo_imu_no_base` built successfully. All 38
base-compensation tests and 11 executable CLI argument tests passed. These
are focused synthetic/argument checks, not full CTest or real-data parity.
