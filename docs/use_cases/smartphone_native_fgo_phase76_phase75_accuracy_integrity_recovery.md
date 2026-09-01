# Phase76: Phase75 accuracy-integrity recovery

Phase76 is a scorer-only recovery for the Phase75 evaluator failure. Phase75
read the MTV-a development truth once, then failed before publishing a score
because it looked for a `PHASE43_CONTROL` helper constant that is not part of
the sealed Phase74 scorer. That failure and its accounting remain immutable.

The Phase76 scorer reads the four immutable Phase73 structural run-1
candidate/control artifacts and the four declared Phase44 development truths,
once per artifact/truth in one scorer process. Control identity is obtained
directly from the pinned Phase74 freeze at
`sealed_phase73_artifacts.routes[route].control`; no retyped helper constant is
used. The truth parser accepts required fields by name, an optional `phone`
column, and additional named columns, while preserving exact key and warm-up
contracts.

Metric, candidate/control artifacts, route order, accuracy gates, and the
no-validation/no-Kaggle policy are unchanged. A new Phase76 output root is
used, and the Phase75 output is not reread or reused. The Phase75 post-read
source patch is pinned for provenance only and is not rerun. Any exception,
hash mismatch, unexpected truth key, second read, or forbidden input is
fail-closed.

See the machine-readable pre-read freeze:
`docs/use_cases/records/smartphone_r5_phase76_phase75_accuracy_integrity_recovery_freeze_v1.json`.
