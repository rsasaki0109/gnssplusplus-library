# Phase479 — fixed floor transfer to H

One raw H run launched with unchanged Phase438 recipe plus only the fixed
--native-lm-lambda-floor selector. Current Phase476 binary/source/test pins
reused and verified before launch. Manifest SHA256
9380f21551c567268a242c0d77de695ddfeaefb9df1c657f33e88df24b1ee73f.
Session 48414, PID 3734326; start 2026-09-08T20:44:13.170715Z.
Live process rechecked. No automatic retry or parallel U solve.

After completion use scripts/verify_phase479_lambda_floor.py. It checks both
stage-specific solvers/floors, pins, convergence and reports output identity
without assuming equality. H retains its historical first-output-row exclusion
and expected 3139 output rows; do not alter that domain in comparison.

Phase480 U manifest/launcher prepared from exact Phase249 U inputs/argv plus
the same floor. U retains include-first-native-epoch. Not launched yet; launch
only after H is terminal and inspected. U expected output hash is derived
from the frozen baseline output for evaluation-only comparison, never an
inference input. No truth or accuracy scoring planned for either run.

The fixed value is not tuned separately by route. Default OFF and candidate
unpromoted. Runtime transfer does not complete the 0.782/LB accuracy goal.
