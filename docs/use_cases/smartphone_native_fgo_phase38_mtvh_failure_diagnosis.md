# Phase 38 MTV-h native-state failure diagnosis

Phase 38 is sealed **No-Go** under the frozen, truth-free contract in
[the Phase 38 freeze record](records/smartphone_r5_phase38_mtvh_failure_diagnosis_freeze_v1.json).
The complete evidence is recorded in the
[diagnosis result](records/smartphone_r5_phase38_mtvh_failure_diagnosis_result_v1.json).

Only the already-materialized MTV-h `device_gnss.csv`, `device_imu.csv`, and
`brdc.nav` inputs were used.  Ground truth, MATLAB files, precomputed
coordinates, validation/holdout data, Kaggle, and tokens were not opened.
The Phase 37 binary and outputs were preserved.

The first existing-binary attempt returned rc=127 because the runtime
`LD_LIBRARY_PATH=/home/sasaki/.local/lib` was missing.  This environment-only
failure is recorded explicitly.  A direct rerun with the Phase 37 runtime
environment returned rc=1 with:

```text
failed to align output to raw UTC keys: native solution has non-finite or out-of-Earth ECEF position
```

The failure is native-state, not UTC projection-only.  The alignment guard
in `src/io/android_raw_gnss.cpp` checks every native ECEF state for finiteness
and the 6,000,000--7,000,000 m Earth norm before UTC key matching.  A GDB
scan recorded 1325 GNSS-first states, 611 invalid states, first invalid index
0, and first invalid ECEF norm 7,432,673.743891388 m.  An independent isolated
debug build observed 610 invalid states and the same first index; the small
solver-build variation does not change the classification.  The pre-handoff
raw-only graph seeds were 1325/1325 Earth-valid epochs (invalid count zero).

An exploratory `--native-mtvh-failure-recovery` handoff rejection was run
twice and returned rc=1 both times: TDCP factors were built=2085 but inserted=0.
The rejection retained the existing in-memory raw-only graph seeds and did not
introduce another trajectory or coordinate, so its scope was not an alternate
trajectory substitution.  It nevertheless did not provide a generalized
recovery: the unchanged raw-only Android heading gate failed closed, and the
follow-on fallback violated the TDCP insertion contract.  No candidate source
was committed and no six-route matrix was executed.  Production/default
behavior remains unchanged; the frozen binary hash and flag-off path are
preserved.

Any future recovery requires a new freeze and a genuinely justified,
truth-free initialization contract.  This result does not authorize
validation, holdout, tuning, or promotion.
