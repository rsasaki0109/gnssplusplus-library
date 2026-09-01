# Phase71 base additional-frequency-band structural candidate

Phase70 found no same-frequency SignalType variants, so key canonicalization
has no evidence. It did find 34,592 missing-frequency proxy rows, concentrated
in GPS L5 because the base `RINEXReader` emits only its primary/secondary
bands by default. The source already exposes
`setPreserveAdditionalFrequencyBands(bool)`, whose default remains false.

Phase71 freezes an opt-in candidate that sets this option true only on the
RINEX reader used by the native base-pseudorange compensation model. Rover,
navigation, SPP, TDCP, Doppler, interpolation, and exact
`(satellite,SignalType)` matching are unchanged. The candidate uses the
existing source-corrected base residual and `P_rover_corrected=P_rover-pc`
formula with coefficient one.

The structural evaluator runs one flag-off Phase43 control and two candidate
repeats per each of four Pixel5 routes in a new output root, with no truth,
MAT, validation, archive, solver-after-truth, or post-score tuning. Candidate
telemetry must report exact selected-band row/stream counts and distinguish:

The global prediction-domain gate is fixed at exactly 1.0: every declared
`(phone,UnixTimeMillis)` key must occur once, with no extras, duplicates,
interpolation, or endpoint hold. This is independent of the base matching
fractions below.

* `matched_factor_fraction`: exact base stream-present adopted FGO factors / all
  adopted FGO pseudorange factors (minimum 0.80 per route);
* `finite_correction_fraction_among_matched`: finite in-domain correction rows /
  those exact-stream-matched factors (minimum 0.99 per route).

These denominators are fixed before execution and do not relax the Phase62/
Phase67 coverage policy. The candidate is not an accuracy evaluation and does
not claim `0.782` reachability. See the [machine-readable freeze](records/smartphone_r5_phase71_base_additional_frequency_bands_freeze_v1.json).

## Phase71 one-shot integrity record

The sealed Phase71 runner stopped after the new MTV-a flag-off control. Its
submission and summary were byte-identical to the pinned Phase43 control, but
the runner's `artifact_report` expanded parsed diagnostics over the summary
artifact metadata and then raised `KeyError: sha256` before launching any
candidate. Candidate runs and accuracy truth were therefore zero. This is an
evaluator-integrity failure, not a coverage or physical finding; the partial
output is not reused. The actual native process read raw GNSS/IMU/nav once
each, base RINEX zero times, and truth/MAT/validation/archive zero times. A
separately frozen Phase72 recovery must use distinct summary artifact/payload
keys and a new output root.
