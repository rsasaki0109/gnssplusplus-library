# Phase474 — opt-in numerical lambda floor implementation

Added FGOConfig.use_native_lm_lambda_floor (default false) and CLI
--native-lm-lambda-floor. CLI requires Phase171 ECEF-D and excludes the
frequency-state experiment. Config propagates by value into GNSS-first and
main; neither stage explicitly resets the new option. Backend/application
scope guards require the native raw C7 GTSAM QR path, preventing silent use
on unsupported optimizers. Application of the floor occurs after solver
selection, before optimizer construction.

Helper applyNativeLmLambdaFloor changes only lambdaLowerBound to at least
1e-8, preserving a pre-existing higher floor and rejecting an initial/upper
lambda below that value. Disabled mode is a no-op. No factor, mask, noise,
ordering, initial lambda or tolerance changes. The numerical schedule is an
explicit experiment, not a source-parity claim or a production default.

Standalone GTSAM parameter test passed: disabled preservation, enabled lower
bound, unchanged initial/upper/factor/solver/damping settings, invalid scope
and too-small initial lambda rejection. Registered in GTSAM-only CMake tests.
This does not yet verify end-to-end flag propagation or output identity.
git diff --check passed.

Native build running at record creation: session 61861, command
cmake --build build --target gnss_fgo_imu_no_base -j1.
FGOConfig layout changed: previously compiled standalone backend test objects
must NOT be reused without recompiling. No raw experiment launched yet.

Next verify build and CLI scope, then freeze one LAX candidate with floor ON
and compare both stage lambdaLowerBound, failure counters, costs, iteration
counts and candidate SHA to Phase472. Do not use the baseline-only verifier
unchanged: it currently requires positive failures, which this experiment
intends to remove. No truth read if output is identical. Any differing output
is unpromoted until the predeclared broader evaluation is designed/executed.

Runtime efficiency is secondary; an identical-output result cannot establish
0.782-class accuracy or leaderboard leadership. Overall goal remains unmet.
