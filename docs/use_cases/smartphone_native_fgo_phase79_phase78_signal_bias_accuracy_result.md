# Phase79 Phase78 signal-bias accuracy result

The corrected Phase79 scorer completed one single-process development
evaluation using each of the four pinned Phase44 truth files exactly once. It
read one Phase78 candidate, one exact Phase43 control, and one Phase73 no-bias
submission/summary pair per route. The Phase43 path was resolved
deterministically from the SHA-pinned `phase43_structural_seal.json`
`candidate_runs[route].run1` metadata; no glob or native rerun was used.

The candidate improved the macro score to `2.7404307830545833` from the exact
Phase43 control macro `3.536446745838451` (improvement
`0.7960159627838679`) and was `0.6517530097075528` m better than the Phase73
no-bias macro `3.392183792762136`. The frozen promotion gates still failed:
MTV-a improved only `0.016861660062385653` m, MTV-h regressed by
`0.27976395493017714` m and scored `3.886729662333128` m, the macro remained
above `2` m, and MTV-u scored `3.712194858995004` m. LAX improved by
`3.2520022083907887` m. Prediction-domain coverage was exactly 1.0, all
coordinates were finite, and over-70-m/s counts were zero.

Therefore this is a scientifically valid no-go, not a reachability claim.
The `0.782` target was not met and remains report-only. Raw GNSS/IMU/nav,
native/solver, MAT, validation/holdout, and Kaggle/token reads were zero;
truth reads were 4 total (one per route). Phase43 remains champion and no
validation or Kaggle action is authorized. The next single source-supported
candidate is official raw-IMU stop velocity/pose graph constraints, which
requires a new pre-truth freeze. See the [result record](records/
smartphone_r5_phase79_phase78_signal_bias_accuracy_result_v1.json), [freeze](
records/smartphone_r5_phase79_phase78_signal_bias_accuracy_freeze_v1.json), and
[manifest](records/smartphone_r5_phase79_phase78_signal_bias_accuracy_manifest_v1.json).
