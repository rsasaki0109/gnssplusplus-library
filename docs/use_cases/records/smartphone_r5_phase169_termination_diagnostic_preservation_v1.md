# Phase169 termination-diagnostic preservation

Phase169 is a metadata-only correction for the dedicated Phase167 raw-P,
no-Doppler GNSS-first diagnostic. It does not change graph factors, initial
values, LM tolerances, lambda policy, or iteration limits.

## Source findings

The pinned GTSAM SUMMARY surface can emit a trial row such as
`0 inf 0 1e-5 0 0.01` when a damped linear solve is indeterminate. A local
C++ probe confirmed that `std::istringstream >> double` rejects the `inf`
token. The Phase143 parser therefore could silently omit that trial and fail
the strict trace-count contract without exposing the counts to the app.

The parser now uses explicit numeric-token conversion for `inf`/`nan`, keeps
the row counted, and retains the existing fail-closed finite/trace checks.
The synthetic regression covers both `inf` and `nan`; it does not treat either
as a successful solver row. The strict completeness gate is unchanged.

When Phase167 validation is incomplete, the GTSAM backend now returns an
unsuccessful `FGOResult` carrying the native scalar termination sidecar and
cost/iteration counters, but returns before copying optimized `Values` into
solution or state-export vectors. Historical Phase143 selector behavior still
throws on the same invalid contract.

The sidecar adds numeric `parsed_trial_count`, `native_inner_iterations`, and
`expected_trial_count`; the existing terminal branch/reason remains the
bounded source terminal reason. No unbounded native trace text is serialized.

## Verification

- Focused Phase143/164/165/167 GTSAM tests: 14/14 passed.
- Actual `gnss_fgo_imu_no_base` target build: passed.
- No raw, truth, MAT, Kaggle, or solver route execution occurred in Phase169.

The Phase168 H failure remains sealed separately. Its underlying trial-count
mismatch is not inferred from the discarded result; Phase170 is authorized to
rerun the pinned H recipe once after this correction to capture the preserved
numeric counters.
