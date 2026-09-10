# Phase205: sample-count bias density experiment

Status: implemented and synthetic-verified, primary agent only; no raw run yet.

CLI: `--native-phase205-source-count-bias-density` (default off). Requires
Pixel5 Phase171 ECEF-D staging, UTC fallback and legacy IMU integration.
Phase201 must be off. Phase197 offset and Phase194 measurement noise remain
independent; the intended comparison keeps Phase197 on and Phase194 off.

For each main-graph interval, N is the number of mapped IMU samples inside the
exact inclusive GNSS bounds. T is the duration of positive legacy integration
steps plus its existing tail step, not an assumed one-second interval. A
prepass computes T and the actual preintegrator duration is checked against it.
Bias covariance densities are multiplied by N/T in a private copy of the
preintegration parameters for that interval. Shared parameters are not mutated.

This matches the source's N-times-variance bias marginal while retaining native
CombinedImuFactor correlations, starting-bias correction and finite initial
bias prior. It is not full source-factor equivalence. Measurement noise,
integration noise, IMU sample values/timestamps and the legacy integration
loop remain unchanged. GNSS-first staging explicitly disables the selector.

Telemetry reports requested/enabled, interval and inclusive sample totals,
and minimum/maximum density multiplier. Invalid sample streams or invalid
counts/durations fail before optimization, without a legacy fallback.

Verification: fresh `gnss_run_tests` and `gnss_fgo_imu_no_base` build passed.
The focused Phase205/204/time-indexing/Phase171 filter passed 20/20; additional
Android IMU/mapping, Phase194/167/143/164/165/101/93/135, raw seed, TDCP and stop
regressions passed 85/85. Runtime CLI rejection tests passed 4/4. This is not
a full CTest claim. Helper controls cover 10/53/100 samples over 0.5/1/2 seconds,
invalid counts/durations, and preservation of shared baseline parameters.
The actual staged-result/main fixture checks inclusive N=11, T=1 and multiplier
11; malformed IMU data and Phase201 coexistence throw before optimization.
`git diff --check` passed.

No raw run, truth read, score or submission has been performed for Phase205.
