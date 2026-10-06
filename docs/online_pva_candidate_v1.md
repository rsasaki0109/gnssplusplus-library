# Frozen vehicle NHC candidate v1

Freeze this contract before running the candidate. Baseline: `c0c5e709`, native
replay SHA256 `de39172892f30643f8160007ba20453c626c5fe3b24003daea2fd88ff73d7548`.
All six existing PPC runs have full, exactly truth-matched baseline results.
These are development data. No application holdout is reopened.

Tokyo run1's 600-epoch loose-only ablation reduces fused position RMSE from
99.677 to 3.213 m, while full-rotation RMSE remains poor (92.601 to 99.932 deg).
Thus disabling tight feedback alone does not solve attitude. A diagnostic
replay exports the learned gyro bias without changing inference: initial
z bias is -0.014 deg/s; at 30 seconds it is 12.885 deg/s although the observed
z gyro at that epoch is -0.243 deg/s. This supports investigating spurious
bias learning/weak heading observability. It does not prove a single cause.

Test one causal vehicle constraint candidate: enable the existing lateral
0.3 m/s and vertical 0.2 m/s nonholonomic constraints only after the first
multi-epoch GNSS course latch. The gate is new and opt-in; existing NHC users
retain their previous behavior. No changes to sensor axes, lever, time,
initialization window, truth alignment, RTK settings or tuning thresholds.
Before the latch, inference must remain numerically identical to the control.
Reverse motion is allowed: no positive-forward-speed constraint is added.

The six full normal runs and fixed 60–70 s GNSS removal / 60–64 s IMU gap
scenarios are the frozen comparison population. Require every run's RTK/fused
position, RTK/fused velocity and full-rotation RMSE/P95 to regress no more
than 1% on both all-output and common-valid timestamp cohorts. Require no
coverage loss over 0.1 percentage point, no later initial latch or scenario
GNSS/fresh-attitude/heading recovery (null is censored, never zero), and at
least one normal-run full-rotation RMSE/P95 improvement. Processor P95 must
not exceed twice its control value; host contention limits latency claims.
Missing metrics, missing scenarios or mismatched input hashes fail the gate.
Keep the original defaults unless every gate passes. Record a failed gate
as No-Go and do not tune this candidate after seeing its results.
