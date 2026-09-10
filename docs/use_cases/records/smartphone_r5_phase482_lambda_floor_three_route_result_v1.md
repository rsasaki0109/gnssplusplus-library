# Phase482 — fixed lambda floor: three-route numerical result

Phase480 U completed once, exit 0, 101.02009622007608 seconds; session 30191
closed. Manifest SHA256:
c6014addbda1f401ec48beee412a842060f19677109a4dd1f518d260a0205e4d.
verify_phase480_lambda_floor.py passed frozen pins, effective floors in both
stages, original solver types and convergence checks. Candidate SHA256:
b93739b27fe449c245af01f23af348690283df1457592f9d4d0d563c0f235a0d.
GNSS-first/main accepted iterations equal total trials: 84/80. No failed
lambda records and main failure count zero. No retries remain in either stage.

U evaluation-only output comparison matched all 1102 keys. Mean separation
0.00004232995909538328 m, P50 0.000023930783195681306 m,
P95 0.00009719581300606942 m, maximum 0.00027296875894385205 m.
Counts match: 66885 factors, 5510 states, 1101 IMU intervals, 106 stop pose
factors, 111 stop velocity factors and 111 stop epochs.

## Consolidated fixed-value experiment

| Route | Main failed linear trials, baseline / candidate | Max output separation m | Candidate single-run seconds |
|---|---:|---:|---:|
| LAX-T | 91 / 0 | 0.00129505 | 135.610 |
| H | 37 / 0 | 0.000304757 | 179.569 |
| U | 74 / 0 | 0.000272969 | 101.020 |

Both candidate stages in all routes used the same fixed 1e-8 floor, with
GNSS-first Cholesky and main QR retained. No per-route tuning. All candidate
stages converged without failed/rejected inner trials. Output CSVs are not
byte-identical; the differences are millimetre/submillimetre scale. None of
these comparisons used ground truth, and no output was fed back to inference.

## Decision and remaining work

Keep the opt-in implementation as a supported experimental numerical option,
default OFF. No additional floor sweep. Full regression/default-off native
replay and controlled repeated timing are still needed before general default
promotion or a reliable speedup claim. This closes the three-route feasibility
experiment, not the user's performance objective.

On matching keys and the same spherical distance metric, maximum separations
bound the change in each route's percentile-based error metric. Thus this
option cannot explain or close the remaining metre-scale accuracy gap. Do
not spend additional truth evaluations to select among these numerical floors.
Return to measurement/trajectory modeling with a distinct prospective
multi-route accuracy hypothesis and an untouched evaluation boundary. Preserve
negative findings on frequency states, re-masking, robust seed initialization
and held-out clock certification; do not recycle them as established fixes.

No Kaggle submission, MAT input, saved-position inference or accuracy scoring.
All native jobs launched for this experiment are terminal. Original goal
remains active: 0.782-class native raw-only accuracy and LB leadership unproven.
