# Phase67 Phase66 structural-integrity recovery

Phase67 was a fresh recovery of the Phase66 evaluator collision. The Phase65
and Phase66 outputs were audit records only; a new output root was used, and
neither partial output was passed to the evaluator.

The direct-artifact normalization worked for the MTV-a control: its submission
and summary were byte-identical to Phase43. The candidate native process then
returned successfully, but the frozen Phase65 coverage gate rejected it:
61,754 adopted pseudorange rows contained finite base corrections for only
46,556 rows (`0.7538944845678013`, required at least `0.99`). There were 15,198
in-domain interpolation misses. The correction magnitude itself was material
(absolute p50 `7.093666204009779 m`, p95 `14.90088104925494 m`, maximum
`15.637516129413319 m`), so this is a coverage failure rather than an
identity or materiality pass.

The candidate process emitted files but was rejected by the immutable helper
before candidate repeat 2; no other route was run. The helper's failure record
reports one started invocation because its counter increments after
`run_case()` returns; two native processes actually completed (control and
candidate run 1). GNSS/IMU/nav process reads were 2 each and base-RINEX was 1;
truth, MAT, validation/holdout, archive, and Kaggle/token reads were zero.
No accuracy evaluation or `0.782` claim was made. Phase43 remains champion and
the base compensation is not authorized for accuracy.

The gate was not relaxed. A future phase may perform a separately frozen,
truth-free diagnosis of the 15,198 matching misses by constellation, signal
and time domain; this record does not authorize canonicalization, interpolation
changes, or a rerun. See the [Phase67 freeze](records/smartphone_r5_phase67_phase66_structural_integrity_recovery_freeze_v1.json),
[manifest](records/smartphone_r5_phase67_phase66_structural_integrity_recovery_manifest_v1.json),
and [sealed failure record](records/smartphone_r5_phase67_phase66_structural_integrity_recovery_failure_v1.json).
