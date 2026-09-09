# Phase333: inverse-frequency-squared cancellation does not remove residual structure

Extended the frozen base diagnostic with fixed frequency combinations:
gamma=(fL1/fL5)^2, neutral=(gamma*L1-L5)/(gamma-1), and L1-equivalent
dispersive=(L5-L1)/(gamma-1). Known synthetic common and inverse-frequency-
squared components recovered within 1e-10 using the same QR fit routine.
Rank/count/nonfinite checks also passed. No empirical scale was fitted.

One hash-verified native run on raw base/nav completed with exit 0 and no
stderr. All 3500 fits succeeded on 23564 common rows. Previous L1/L5 summary
values repeated exactly. The neutral combination's median apparent position
norm was 5.040080966014585 m and its median post-fit RMS was
1.8952987266161494 m; the dispersive L1-equivalent norm was
2.857295306302563 m. These are norms of per-epoch fits, not vectors that can
be added/subtracted using their median magnitudes.

The residual structure does not disappear through this algebraic cancellation.
This does not support an ionosphere-only explanation, nor establish a 5 m
station coordinate error. The combination magnifies noise and tracking biases,
and distinct smoothing histories can violate the ideal common-component model.
It is not a calibrated ionosphere measurement or atmosphere-free position.
No correction or base coordinate is changed from this result.

Next inspect time/reference conventions and explicit modeled atmosphere on
raw un-smoothed common observations, where the cancellation assumptions can
be tested without differing window support. Prefer synthetic known-error and
source-equation checks before another smartphone accuracy run. No truth,
phone, saved trajectory, MAT payload, inference feedback, or new accuracy
score/submission occurred. The established base-OFF baseline remains in use;
0.782-class and leaderboard objectives remain unachieved.
