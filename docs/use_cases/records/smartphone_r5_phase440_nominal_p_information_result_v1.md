# Phase440 — nominal single-epoch P information

Phase438 H and Phase439 LAX-T each completed once, exit 0. Current source/test,
algorithm, binary and raw pins verified; both stages converged; both candidate
CSVs exactly match their original baselines. Bias aggregates remain unchanged.
No truth reads, additional scores or parameter changes.

| Route | Epochs | Numerically deficient epochs | Minimum eigenvalue m^-2 |
|---|---:|---:|---:|
| H | 3140 | 0 | 0.007220089475656193 |
| LAX-T | 1466 | 2 | 0 |

Native C7 clock columns were projected from nominal-sigma whitened ECEF
antenna position Jacobians using same-run optimized states. Numerical
deficiency convention: minimum eigenvalue <= max(1e-12, maximum*1e-10).
This omits robust weights and all temporal/IMU/Doppler/TDCP constraints.
It is NOT posterior covariance or full-graph observability. Two deficient
epochs do not explain the route-wide positioning error by themselves.
H's full numerical rank likewise does not prove unbiased positioning.

H elapsed 446.61622614902444 s, manifest SHA
`6e575063d354d61191d1c9128a1ec4dfbf2b88dcbd513acc9204d03395e08c3a`.
LAX-T elapsed 379.35889171704184 s, manifest SHA
`cf5eaf6d62a91eab8eead634d529339e38522cb008c733f67894b46c5f3006ed`.
Both took longer than preceding runs; no controlled timing attribution yet.
Do not claim diagnostic overhead is negligible or a speed regression proven.

Next examine support and temporal connectivity of the two deficient LAX-T
epochs, using raw/admitted observation identities, not truth or saved position
inputs. Avoid treating generic prior strengthening as a demonstrated fix.
