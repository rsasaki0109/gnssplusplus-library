# Phase356: reject main-P Cauchy scale 4

Frozen Phase355 ran once, exit 0, 277.9240805490408 seconds. Native raw-only
base-off recipe plus fixed-scale Cauchy for main undifferenced P only.
GNSS-first summary aggregates equal Phase234 exactly; 101916 P factors,
3139 matched output rows; both stages converged. No new data admissions,
sigma changes, interpolation or saved positioning input.

One frozen H development evaluation: score 1.3403416712498095 m,
P50 0.9603483679362179 m, P95 1.7203349745634013 m. Baseline
1.0769392017393964 m; degradation +0.26340246951041313 m. Not promoted.
Changing this robust kernel alone does not improve the operational recipe.
Do not tune the Cauchy scale repeatedly against H as independent evidence.
The result does not rule out other robust methods, but it provides no basis
for changing the default Huber loss or weakening the existing quality gates.

No holdout/LB conclusion or submission. 0.782-class target unachieved.
The flag remains default-off and restricted to its diagnostic configuration.
