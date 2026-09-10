# Phase475 — numerical candidate launched

Phase474 native build completed exit 0, session 61861 closed. Standalone
parameter-helper test passed earlier; new candidate-verifier regression test
passed. Bare CLI switch without required inputs exits 2 (input-shape check,
not a full positive or negative recipe scope test). No end-to-end candidate
outcome established before launch.

Fresh manifest SHA256:
9bb390262db4c76c5a61f90728c4b5606b7dbe275099f1a8ca6138ba4255ce7b.
One Phase475 LAX raw run launched 2026-09-08T20:34:43.687088Z. Session 81118,
PID 3727143. Output directory:
output/smartphone-r5/phase475-lax-t-robust-diagnostic-v1/lax-t.

New selector --native-lm-lambda-floor is ON; prospective comparison baseline
is Phase472. No other recipe change. This is NOT a diagnostic-only replay:
effective LM schedule changes in both stages. Default stays OFF.
After completion run scripts/verify_native_lambda_floor_experiment.py.
It checks fresh pins, both effective lower bounds, unchanged QR/isotropic
damping and initial lambda, convergence, available failure ranges, and
reports output identity without assuming it. Zero failures is allowed.

No accuracy scoring or promotion authorized by an output difference alone;
first evaluate structural/numerical results. No truth/MAT/saved-position
inference input. Existing raw files and broadcast navigation only, same-run
in-memory GNSS-first handoff. The original accuracy goal remains unmet.

Terminal update: exited with SIGABRT (-6) after 2.953909543924965 seconds;
session 81118 closed. New scope guard rejected GNSS-first before FGO solve.
Baseline Phase472 metadata proves GNSS-first uses MULTIFRONTAL_CHOLESKY,
not QR; main uses QR. The staging config sets raw_p_ecef_doppler_gnss_first
and clears the main/raw-P-no-D selectors. Corrected runtime scope to admit
that exact existing stage without changing its solver. Correction is not
rebuilt yet. The frozen Phase475 verifier also incorrectly expects QR in
both stages: preserve its pins as failed-experiment history and create a
corrected verifier/manifest for the next experiment. No result or speedup.
