# Phase524 — U/LAX diagnostic transfer verified

Added run_phase524_transfer_monitor.py. Reuses exact argv/input pins from
Phase480 U and Phase476 LAX (including LAX sparse-P staging), changes only output
paths, and uses the current binary/source pins frozen by Phase522 H. The old
trajectory is hashed for opaque output comparison only; never inference input.
Checks full graph and GNSS-first summaries against each route baseline and
both termination contracts, complete IRLS rows/epochs and residual diagnostics.

Initial launcher failed before Popen because digest received __file__ as str
instead of Path. Empty v1 directories remain; no native process was started in
either. Fixed path handling and explicitly used new v2 output directories;
no output overwrite or interrupted-process retry.

Both native processes completed exit 0:

- U PID 3862986 / session 8141: 213.83825403393712 seconds.
- LAX PID 3863031 / session 3312: 334.0119693160523 seconds.

Both output hashes equal their floor-only baselines; both full graph and
GNSS-first summaries identical. Native source code was not changed in this
phase. No truth reads, scoring, MAT or saved positioning inputs. No live jobs.
Artifacts: output/smartphone-r5/phase524-transfer-monitor-v2/{u,lax}/.

| Route | Epochs | Code rows | Downweighted | Nominal info median | IRLS info median | Residual alignment median | Supported / unsupported | Same-sign / adjacent |
|---|---:|---:|---:|---:|---:|---:|---|---|
| U | 1102 | 32475 | 31444 | 1.25186370406 | 0.101623353947 | -0.204363840889 | 1100 / 2 | 836 / 1097 |
| LAX | 1466 | 46487 | 43585 | 1.2167053045 | 0.188056373167 | 0.138391561301 | 1462 / 4 | 1034 / 1459 |

Information units m^-2. Compare Phase523 H IRLS median .0802815904466 and
alignment .0652986616768. Positive projected information exists, but code-only
support is weak under initial Huber weighting. Different alignment signs rule
out treating these diagnostics as one common signed offset; they do NOT refute
time/route-varying ionosphere. These are all reused development routes.

Decision boundary: initial-state diagnostics are now reproduced across three
routes; do not repeat them or sweep thresholds. A next model experiment needs
a C7-compatible code factor with explicit nuisance-state priors and clear
code/TDCP scope, or evidence at optimized state before claiming an ionosphere
explanation. Keep guards/defaults intact until that contract is implemented
and tested. These results prove diagnostic invariance, not improved accuracy
or the .782/LB objective, which remains active and unmet.
