# Phase452 — independent raw adjacent P-D context

Extended native raw epoch audit to summarize signed incoming adjacent-epoch
P-D differences with the existing pseudorangeDopplerDifference formula.
Require matching satellite/signal, both P and Doppler present, positive P,
positive finite wavelength, 0 < dt <= 1.5 s and finite computed difference.
No nav, saved position, FGO inference or truth input. Compiled and ran once.

| Epoch | P-D pairs | Min m | Lower median m | Max m |
|---|---:|---:|---:|---:|
| 850 | 12 | -7.62119 | 0.18046 | 39.4466 |
| 851 | 10 | -59.9355 | -5.4922 | 29.2788 |
| 852 | 10 | -7.77499 | 3.20544 | 51.3097 |
| 853 | 13 | -46.475 | -2.39576 | 26.357 |

Values are printed to default stream precision. These raw temporal proxies
have mixed signs and large extremes near the sparse interval. They do not
show a uniform negative P-D displacement corresponding to every rejected
seed residual. Populations differ: do NOT match extremes to a rejected P
row, infer an independent error probability, or conclude which sensor is bad.
Exact identity-level overlap remains unmeasured. This evidence does not
justify restoring all rejected rows or changing thresholds.

Phase451 remains live in session 16010 / native PID 3643229; signed rejection
counts from its initial log are provisional until final pin/output checks.
No additional accuracy evaluation was performed.
