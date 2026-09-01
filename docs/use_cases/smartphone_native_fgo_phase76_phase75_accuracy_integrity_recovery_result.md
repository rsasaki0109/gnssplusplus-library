# Phase76 result: Phase75 accuracy-integrity recovery

The Phase76 scorer recovered the Phase75 truth-parser and control-identity
failures without rerunning native code. It read each immutable Phase73
candidate/control artifact once per artifact and each declared Phase44
development truth once in a single scorer process. Control identity was
checked directly against
`sealed_phase73_artifacts.routes[route].control` in the pinned Phase74 freeze;
no retyped `PHASE43_CONTROL` helper table was used.

The result is no-go under the unchanged gates. Candidate/control scores were:

| Route | Candidate | Phase43 control | Improvement |
| --- | ---: | ---: | ---: |
| MTV-a | 2.568570558495357 | 2.1200062768576293 | -0.44856428163772755 |
| MTV-h | 4.738130658761164 | 3.606965707402951 | -1.1311649513582127 |
| LAX-t | 2.077479775443125 | 4.511656202485746 | 2.4341764270426207 |
| MTV-u | 4.184554178348899 | 3.9071587966074777 | -0.277395381741421 |

The candidate macro score is `3.392183792762136` versus control
`3.536446745838451` (improvement `0.1442629530763151`). The macro-improvement
gate passes, but routewise improvement, MTV-h improvement, candidate macro
`<=2 m`, route `<=3 m`, and MTV-h P95 `<=5 m` gates fail. Prediction-domain
coverage is exactly 1.0, all scores are finite, and over-70-m/s counts are
zero. The `0.782` target is not met and is report-only.

Truth-row coverage remains informational: the pinned leading warm-up row is
the sole missing truth row for MTV-a and MTV-u. No interpolation or fill was
used. Phase43 remains champion; Phase73 remains experimental. No validation,
Kaggle, native rerun, or post-score tuning is authorized.

Machine-readable details and hashes are in the [sealed Phase76 result record](records/smartphone_r5_phase76_phase75_accuracy_integrity_recovery_result_v1.json),
[freeze](records/smartphone_r5_phase76_phase75_accuracy_integrity_recovery_freeze_v1.json),
and [manifest](records/smartphone_r5_phase76_phase75_accuracy_integrity_recovery_manifest_v1.json).
