# Phase209 separate IMU factors — implemented and synthetically verified

Primary agent only. No native raw run or truth evaluation has been performed
for this option. Best Phase198 recipe unchanged; default remains off.

Implemented `--native-phase209-source-separate-imu-factors` with dedicated
Pixel5 Phase171 ECEF-D/explicit UTC fallback CLI guards, configuration wiring,
GNSS-first-stage disable, backend checks, and summary factor-count telemetry.
Reject Phase201/205 combinations. Backend rejects invalid/nonfinite ordered
IMU input, empty/non-integrable intervals and epoch gaps outside (0, 1.5) s;
it does not silently substitute a different factor for these conditions.

The standard PreintegratedImuMeasurements receives precisely the measurements
and durations selected by the unchanged legacy preceding-delta/tail loop.
It has independent standard covariance propagation. Existing Combined PIMs
remain for the unchanged attitude initialization only; in the new branch,
no CombinedImuFactor is inserted. Duration and rotation consistency are
checked against the seed preintegration. This is same-run raw preintegration,
not a saved trajectory input.

Each valid interval inserts an ImuFactor with ending bias plus a separate
BetweenFactor<ConstantBias> with sqrt(inclusive sample count) times the
configured bias sigmas. Finite native initial pose/velocity/bias priors and
the original integration schedule remain, so this is not full source parity.
Rejecting >=1.5 s gaps bounds this experiment; supporting the source's
bias-only long-gap branch is deliberately not claimed.

Production handoff tests now request the option, check actual factor/sample
counts and exercise malformed input and incompatible selectors. Five new
runtime CLI tests supply only synthetic dataset names and no raw paths.

Current verification: `git diff --check` passed. The fresh
`gnss_fgo_imu_no_base` build succeeded. Runtime CLI tests for Phase209 and
Phase205 passed **9/9** (pytest session 52793, exit zero); no raw paths were
supplied. This validates configuration rejection, not graph construction.
The `gnss_run_tests` build completed successfully in session 48188, exit zero.
The fresh binary passed **21/21** focused tests: Phase208, Phase204, Phase205,
GtsamImuTimeIndexing and all Phase171 suites. This includes the actual new
production handoff branch, convergence, one motion plus one bias factor,
11 inclusive samples, default-off controls, malformed IMU rejection, and
Phase201/205 incompatibility rejection.

The curated regression filter also passed **85/85**: AndroidImuCsv,
AndroidUtcGpsMapping, Phase194/167/143, Phase164 raw-no-D graph and
observability, Phase165, Phase101SourceClockVector, Phase93 clock C0/D,
RawPSeed/Adapter, Phase135 affine family, TDCP robust-k and upstream stops.
Thus C++ total is **106/106**, plus CLI **9/9**, not a full CTest claim.
Long-gap/empty-interval and bias-sigma checks are currently supported by code
inspection, not dedicated production-branch negative fixtures; do not imply
that the handoff test covers every guard.

Next raw comparison: Phase198 best recipe plus Phase209 only, with Phase201,
Phase205 and Phase194 off and Phase197 on. Freeze current source/test/binary
hashes and fresh output paths before one invocation. Expect 3139 motion and
3139 bias factors (net +3139 graph factors relative to Combined), unchanged
GNSS-first stage, and 3139 published rows. Validate these before separately
freezing any accuracy evaluation. No raw comparison has yet run for Phase209.
