# Smartphone R5 Phase149 raw-P same-run seed stage

Status: implemented as a default-off preparatory lane.  This record does not
authorize or report a real-route run.

## Contract

`libgnss::raw_p_seed::solve` accepts in-memory `ObservationData` epochs and
`NavigationData`.  It preserves each input epoch's index, GNSS time, raw
source index, and raw UTC key.  It runs the existing native
`SPPProcessor::preprocessEpoch` on a private copy with all Doppler/range-rate
fields cleared, so position and receiver-clock seeds are same-run
pseudorange-only results.  No trajectory, coordinate, or seed file is read;
the raw D initializer remains a separate path.

Each accepted epoch carries finite position/clock values, native SPP quality
and iteration fields, a four-column position-plus-shared-clock geometry-rank
report, and an explicit convergence/iteration-limit reason.  Velocity is
derived only after all epochs are accepted, using the source-style
one-sided-endpoint/centered-interior gradient.  Nonfinite, duplicate,
nonmonotonic, or over-gap times; insufficient P rows; invalid/rank-deficient
geometry; unsupported clock groups; rejected/nonfinite SPP output; and
invalid velocity intervals fail closed without holding or filling a prior
state.  GPS and QZSS are the only systems coalesced into the supported shared
clock group; mixed or unknown groups are rejected until per-group rank and
clock-seed provenance is implemented.

The opt-in application flag
`--native-phase149-raw-p-seed-stage` accepts only `--obs`/`--nav` or
`--android-gnss`/`--nav`, plus `--summary-json`.  It writes
`smartphone-r5-phase149-raw-p-seed-stage.v1` structural metadata (including
finite-ness and failure fields, not coordinate payloads), and exits before
IMU/FGO construction.  Existing H/Doppler admission guards and all default
recipes are unchanged.

## Verification

The focused synthetic suite `RawPSeedTest.*` passes 8/8.  It covers
stationary and moving P+clock observations, explicit velocity endpoints,
Doppler-clearing equivalence, insufficient P, rank-deficient and nonfinite
geometry, unsupported mixed clock groups, and duplicate/nonmonotonic/gapped /
nonfinite timestamps.  The affected targets
`gnss_run_tests` and `gnss_fgo_imu_no_base` build successfully, and the binary
help exposes the opt-in flag.  No real GNSS/IMU/nav route, truth, MAT,
precomputed coordinate, or accuracy/solver evaluation was run.

## Known boundary

This stage is a raw-P seed/preflight API only.  H-route coverage and a
no-Doppler FGO graph admission remain unverified and disabled; this change
does not remove those guards or claim algorithmic accuracy.
