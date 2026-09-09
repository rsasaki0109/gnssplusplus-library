# Phase191 native raw-P ECEF-D GNSS-first implementation

Status: implementation tested; no Android/raw route was executed in this record.

The new default-off selector `--native-phase171-raw-p-ecef-doppler-gnss-first`
is valid only with the Phase171 same-run raw-P GNSS-first/IMU-main lane.  It
builds a temporary corrected raw-D view from the already-mutated in-memory
epochs, then transfers only validated D rows into the authoritative
GNSS-first problem.  Epochs are joined by `(raw_source_index,
raw_utc_time_millis)` and an exact finite TOW check; duplicate, missing,
near-tolerance, nonfinite, or unsupported rows fail closed.  P/TDCP rows,
epoch seeds, and the main Pose3+IMU graph remain unchanged.

The dedicated stage factor keeps the corrected source LOS and velocity state
in ECEF with residual/sigma/clock terms in m/s.  The existing main handoff
still performs the sole ECEF-to-ENU conversion and keeps generic Doppler
empty.  Phase184 and legacy/incompatible compositions are rejected; selector
off retains the existing Phase171/Phase164 behavior.

Verification on the current source:

- `gnss_fgo_imu_no_base` app target built successfully.
- Curated affected native C++ filter: 83/83 passed, including perturbed-seed
  ECEF velocity/clock recovery, corrected-row/frame/sign and fail-closed
  checks, same-run stage-to-main handoff, and compiled transfer-helper tests
  for permutation, duplicate/missing identity, near-1-us mismatch, invalid D,
  and P/TDCP preservation.
- Phase176/178/184/187 source-contract tests: 14/14 passed.
- `git diff --check`: clean.

No accuracy claim or real H/raw result is made here.  Main IMU Doppler remains
disabled in this first ablation; the new D rows are GNSS-first-stage only.

Scope note: an unrelated wildcard legacy check remains 80/81 because the
pre-existing Phase99 carried-selector assertion expects Phase93 fixture fields
on a non-raw-D fixture; the curated affected filter above is 83/83.  Two
historical Phase93 source-pin checks also report their previously frozen pin
mismatch; neither was changed or used to qualify this implementation.
