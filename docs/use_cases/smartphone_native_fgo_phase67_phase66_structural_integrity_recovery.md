# Phase67 Phase66 structural-integrity recovery

Phase67 is a fresh recovery of the Phase66 evaluator collision. The Phase65
and Phase66 outputs are audit records only; the matrix uses a new output root
and does not reuse either partial output.

The frozen defect is precise: `P65.artifact_report()` creates a `summary`
artifact metadata object, then expands diagnostics from `validate_summary()`.
Those diagnostics contain a parsed native summary under the same `summary` key,
so `P65.run_case()['summary']` has no artifact `sha256`. Phase67 keeps the
native summary separate and hashes each newly emitted `submission.csv` and
`summary.json` directly under `submission_artifact` and `summary_artifact`.
The candidate repeat and Phase43 control checks use those direct hashes.

The matrix is frozen at four Pixel5 routes, one flag-off control and two
base-compensation candidate runs per route (12 native invocations). It is
truth-free: no truth, MAT, validation/holdout, archive reopen, or Kaggle/token
read is allowed. Any mismatch fails closed and no accuracy evaluation is
authorized in this phase.

See the [Phase67 freeze](records/smartphone_r5_phase67_phase66_structural_integrity_recovery_freeze_v1.json)
for the pinned source/input hashes and accounting contract.
