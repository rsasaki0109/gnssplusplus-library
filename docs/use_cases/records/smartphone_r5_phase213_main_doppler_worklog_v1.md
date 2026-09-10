# Phase213 final-graph Doppler — in progress

## Current status: implemented, synthetic tests passed

Build session 8445 completed with exit zero. Fresh C++ binary passed 22/22
focused tests (Phase213 frame, Phase208/204/205, time indexing, all Phase171),
including the newly extended actual main-graph handoff and rejection cases.
Curated related regression tests passed 85/85 (Android IMU/time mapping,
Phase194/167/143/164/165/101/93/135, raw seeds, TDCP robust-k, upstream stops).
Total C++ **107/107**, plus previously verified fresh CLI **15/15**.
This is not a full CTest or real-data accuracy claim.

Next: freeze a Phase198-best-recipe plus Phase213-only raw comparison.
Expected dedicated main D factors: 66685, generic D remains zero; main total
graph factors should increase from 182560 to 249245 with unchanged values.
Stage D count, P/TDCP counts, IMU schedule, finite initial priors and the
-20 ms offset must remain unchanged. No raw Phase213 run has occurred yet.

The entries below preserve the implementation chronology; their pending
build/test statements are superseded by this status.

Primary agent only. No raw invocation yet. Backend integration is drafted;
CLI wiring and production-branch tests are now drafted; fresh build and
execution of those tests are pending in session **8445**.

Added `FGOGtsamPhase213MainDopplerFrameTest` to the backend test TU. It uses
the actual existing source-clock ECEF and ENU factors, a synthetic corrected
measurement with known 0.25 m/s residual, one-dimensional drift in m/s and
0.5 m/s sigma. It checks frame-invariant residual, velocity finite-difference
Jacobians, positive clock derivative, graph likelihood, an unrotated-LOS
negative control, and rejection of a two-dimensional drift value.

Build session **10076** completed successfully. The new frame test and
Phase171 ECEF-D graph/transfer suites passed **5/5** on the fresh binary.
This was before the backend integration below; it does not validate that
new integration or claim full graph convergence.

Drafted config `use_native_phase213_main_doppler`, a dedicated
`FGOProblem.native_phase213_main_doppler_rows` field, and separate diagnostics.
The backend rejects populated dedicated rows when the selector is off and
rejects missing/invalid rows or Phase201/205/209 combinations when on. It
reuses exact-identity remapping for validation and inserts the existing
source-clock vector Doppler factor with one ECEF-to-ENU LOS transform,
preserving row sigma and robust-noise configuration. Generic Doppler guards
are unchanged. This integration has passed diff whitespace checks only;
compile and test execution remain to be completed before raw execution.

CLI `--native-phase213-main-doppler` now requires Pixel5 Phase171 ECEF-D and
explicit UTC fallback, rejecting Phase201/205/209 options. Main config and
summary requested/enabled/factor counters are wired. GNSS-first config has
the new selector disabled. After stage raw-D construction/remapping, a
second exact identity remap populates only the dedicated main field, before
any GNSS-first optimized values are produced. P/TDCP and generic D are not
replaced.

Extended actual Phase171 handoff tests to insert the dedicated D family,
check actual counts and convergence, and reject empty, zero-sigma, duplicate,
selector-off rows and incompatible options. Six runtime CLI rejection cases
are in `tests/test_smartphone_phase213_main_doppler.py`, with synthetic route
names and no raw paths. These additions are **not yet reported passing**.

Inspected the current staging transfer at
`apps/native/gnss_fgo_imu_no_base.cpp` lines 12111–12143. It already builds
corrected D rows from the same raw epochs/navigation and remaps them by exact
source/UTC/time identity using `raw_p_ecef_doppler::remapDopplerFactors`.
The main problem remains separate from this GNSS-first copy. Next use a
dedicated final-graph row field populated through that same validated remap;
do not replace main P/TDCP rows or admit generic Doppler into the established
Phase171 no-D contract. Backend must reject dedicated rows unless explicitly
selected, validate them before inserting factors, and expose separate counts.
Keep the GNSS-first solve and its in-memory state handoff unchanged.

This record captures implementation preparation, not successful production
graph integration, convergence or accuracy improvement.

Verification update: the fresh measurement app build in session 8445
succeeded. Phase213/209/205 runtime CLI tests passed **15/15** (pytest session
38489), using no raw paths. The C++ test executable is still building in
session 8445, so production handoff/convergence tests remain pending.
