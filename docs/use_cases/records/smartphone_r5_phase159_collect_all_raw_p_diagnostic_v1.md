# Smartphone R5 Phase159 full-epoch raw-P diagnostic collection

Status: implemented as an opt-in metadata-only diagnostic lane.  No real
GNSS, IMU, navigation, solver, truth, MAT, Kaggle, or coordinate payload was
read or run for Phase159.

## Contract

`libgnss::raw_p_seed::Config::collect_all_epochs_for_diagnostics` defaults to
`false`.  When enabled, `raw_p_seed::solve` evaluates every input epoch
independently with a fresh native `SPPProcessor` and a fresh raw-P bootstrap
decision.  A failed epoch stays invalid and keeps its terminal reason,
post-filter row/group counts, geometry rank/required-rank report, and native
SPP status when native preprocessing was reached.  No failed state is reused
as a later prior, and no position, clock, or velocity value is held, filled,
interpolated, or promoted to a seed.

The application exposes this only under the existing Phase149 prep selector as
`--native-phase159-collect-all-epochs`; argument parsing rejects it outside
that selector.  The diagnostic lane intentionally disables velocity
derivation and reports `velocity_diagnostics_disabled=true`, so it does not
claim velocity coverage across rejected or unassessed epochs.  Its aggregate
counts satisfy `input=evaluated=accepted+rejected` when all epochs are
visited; the route result remains not-ok if any epoch is rejected.  The
ordinary fail-first path and all existing numerical settings remain unchanged.

The implementation also serializes the per-epoch native SPP status and the
aggregate collection counts.  Pre-SPP failures (for example duplicate time,
insufficient raw P rows, or unsupported raw clock groups) explicitly retain
`native_spp_status_available=false`; failures after native preprocessing retain
the status returned by that native call.  Velocity endpoint policy is not
applied in collection mode.

## Synthetic verification

`RawPSeedTest.*` passes 23/23.  The added cases prove:

* valid-invalid-valid epochs are all visited, accepted positions remain
  finite, and the invalid seed remains nonfinite;
* an unsupported clock group rejects only its epoch while later valid epochs
  retain the two-group/reference/rank metadata;
* duplicate time rejects only its epoch while a later epoch is evaluated; and
* collection mode produces the same accepted P+clock solutions as the normal
  path when every epoch is valid, while leaving velocity unavailable by the
  documented diagnostic policy.

The affected targets `gnss_run_tests` and `gnss_fgo_imu_no_base` build
successfully.  No Phase159 raw route was launched.  Phase158 remains the
immutable empirical boundary: its first rank failure was rank 4 at epoch 17
with 21 corrected rows and required-rank details were not serialized there;
this change adds the missing per-epoch diagnostics for a separately
authorized future run and does not reinterpret or repair that result.

## Boundary

This lane is for broad structural diagnosis only.  It does not admit the H
no-Doppler graph, change rank tolerances, tune filters, derive accuracy, or
establish route success.  A future one-shot H diagnostic may use the flag to
measure later-epoch readiness, with failed/unassessed coverage reported
explicitly.
