# Phase72 base additional-frequency-band evaluator recovery

Phase72 is a separately frozen integrity recovery for the Phase71 structural
matrix. Phase71 launched only the MTV-a flag-off control; its output matched
the Phase43 control exactly, but the evaluator accidentally overwrote summary
artifact metadata with the parsed summary payload and raised `KeyError:
sha256`. That failure and its zero candidate runs are immutable evidence; no
Phase71 partial output is reused here.

The Phase72 native candidate remains exactly the Phase71 source implementation:
`--native-base-pseudorange-preserve-additional-frequency-bands` is enabled only
on the base compensation RINEX reader. Exact `(satellite,SignalType)` matching,
existing source-corrected base residual, interpolation, and all gates are
unchanged. The recovery uses a new output root and launches a fresh control and
two candidate runs for each of the four routes.

The evaluator's run-case return shape is frozen as separate
`summary_artifact` (path/bytes/SHA-256) and `summary_payload` (parsed JSON)
keys; focused tests exercise the actual shape and reject a collision.
Unexpected evaluator exceptions are captured in a fail-closed failure artifact.
Prediction-domain coverage remains exactly 1.0, matched/all remains at least
0.80, and finite/matched remains at least 0.99. No truth, MATLAB, validation,
archive, post-truth solver, or `0.782` evaluation is allowed.
