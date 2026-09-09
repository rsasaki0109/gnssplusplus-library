# Phase279: pre-mask population ownership (work in progress)

Code inspection only, alongside build session 90789. No real coordinate,
raw payload, truth or MAT access. The centered admission predicate was
extracted verbatim to acceptsCenteredPseudorangeResidual; a synthetic
four-observation test now exercises recovery and rejection at fixed 20 m.
Build 90789 completed with exit 0. The new synthetic admission test passed.
An initial extra filter *UpstreamPreprocessing* matched no tests; it is
not evidence of additional coverage. The corrected filter covers the
actual UpstreamObservablePreprocessingTest and UpstreamSourceTdcpMetersTest
suites as well: all 11 selected tests passed. No full CTest or real-data
byte-parity claim is made.

Native PseudorangeFactor retains epoch index, satellite/signal, corrected
measurement, corrected satellite position, sigma and seed residual. At
fgo_problems.cpp:695 the residual is corrected_P - range(seed) - seed_clock.
Around 1114-1156 grouped residual filtering swaps a retained vector into
problem.pseudorange_factors, destroying rejected rows when the temporary
vector goes out of scope. Refiltering this surviving vector cannot recover
previously rejected signals.

The CLI keeps the in-memory ObservationData epochs (around 10978) and nav
through the main solve. Thus raw reconstruction is possible without a
file read, but would also recompute geometry, atmosphere, elevation masks
and possibly Doppler/TDCP admission. That is broader than a residual-only
experiment and must not be mislabeled as one.

Prefer an explicit default-off pre-residual-mask pool retained in the
same FGOProblem construction transaction. Keep existing quality, health,
signal and epoch admission unchanged; retain exact epoch/satellite/signal
identity and measurement/sigma values. After the validated GNSS-first
handoff, recompute only range/clock residuals on that pool, derive the
whole-pool system/band median, and replace main P rows. Count recovered,
removed and unchanged rows against the original selection. GNSS-first
must use the original selection. Do not retain or consume pool data across
processes, and do not apply this in incompatible base/affine paths.

Clarification to Phase278: the builder comment says SPP seed, but the
selected Phase165 CLI path supplies an earlier same-run raw-P bootstrap
position before factor construction (around 11007 onward). The relevant
difference is earlier-bootstrap versus later-GNSS-first residuals, not
necessarily an SPP solve in the selected lane. The synthetic test is a
predicate/geometry control, not an end-to-end raw builder or median test.

Remaining: finish current test build, validate exact grouped median and
pool identity/conservation with tests, implement default-off integration,
and verify unchanged default behavior before a frozen raw experiment.
