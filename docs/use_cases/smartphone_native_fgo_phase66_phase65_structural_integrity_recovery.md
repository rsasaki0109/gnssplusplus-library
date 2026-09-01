# Phase66 Phase65 structural-integrity recovery

Phase66 attempted a fresh structural recovery of the Phase65 base-RINEX
pseudorange candidate. The Phase66 freeze and evaluator manifest were sealed
before execution, with control run 1 and candidate runs 1--2 for each of four
Pixel5 routes, no Phase65 partial-output reuse, and no truth/MAT/validation/
Kaggle reads.

The new matrix fail-closed after one MTV-a control invocation. The control
submission and summary were byte-identical to the pinned Phase43 artifacts
(submission SHA `d4d7652e...c5426c`, summary SHA
`d4980260...2303fe`), but the evaluator stopped while comparing the report
schema. `P65.run_case()` receives an `artifact_report()` whose diagnostics
merge overwrites the artifact-summary metadata with the parsed summary JSON;
thus `control["summary"]` has no `sha256` field. The Phase66 fixture covered
the nested `phase43_control()` reference but did not cover this key collision.
This is an evaluator-integrity failure, not a candidate metric result.

Exactly one native control process completed; no candidate or base-RINEX
process was run. Hash verification read each pinned route input once (four
per input type), while process reads were GNSS 1, IMU 1, broadcast nav 1, and
base RINEX 0. Truth, MAT, validation, holdout, archive reopen, and Kaggle
reads were zero. The partial Phase66 output is preserved only for audit and
must not be reused by a future recovery.

See the [freeze](records/smartphone_r5_phase66_phase65_structural_integrity_recovery_freeze_v1.json),
[manifest](records/smartphone_r5_phase66_phase65_structural_integrity_recovery_manifest_v1.json),
and [sealed failure record](records/smartphone_r5_phase66_phase65_structural_integrity_recovery_failure_v1.json).
Phase43 remains the champion; no accuracy evaluation or `0.782` claim was
made. A new Phase67 freeze is required before any fresh matrix.
