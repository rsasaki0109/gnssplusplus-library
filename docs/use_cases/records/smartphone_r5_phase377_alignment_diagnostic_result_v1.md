# Phase377 — no GNSS-first solution, not retained epoch loss

Fresh diagnostic binary built; 25 CLI admission tests passed. These tests
do not exercise solver success. One frozen raw diagnostic invocation then
returned 1 after 1.9220548199955374 seconds, PID 3480496.
Manifest SHA256: `cae7f0f46623dc655577e00de028dc5155f702149ce947b5a539ad1e9f42184e`.

Failure telemetry: main_epochs=1466, gnss_first_epochs=1466,
solution_epochs=0, raw_epochs=1466.

The problems retain the same counts; the immediate failure is absence of
GNSS-first solution output. This rules out repairing retained counts as
the next step. Counts alone do not establish key-by-key identity, but the
zero solution count already explains this guard failure.

Next inspect the GNSS-first optimizer failure/status before the alignment
validator overwrites the user-visible reason. Do not bypass validation or
fabricate solutions. No truth, MAT, saved trajectory or accuracy score was
used. Preserve all prior failed runs; the objective remains unmet.
