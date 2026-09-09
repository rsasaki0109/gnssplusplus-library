# Phase329/330: numerical base corrections implicated in paired regression

Frozen run 4aafcf30 completed once in 268.804 seconds, exit 0. The preregistered
mask-only ablation used actual dense base correction availability but supplied
zero correction to the estimator. A private same-process actual-correction
copy verified retained epoch/satellite/signal identity and order. No saved
graph or positioning input was used. The private copy does not feed inference.

Both stages converged, with valid in-memory handoff and 3139 exact published
UTC rows. The retained P count was 81591, matching Phase326, from 101847 initial
rows. Missing stream count 19163 and unavailable count 1093 also matched.
Application correction maximum was exactly zero. Reused correction-pass
telemetry means a zero-valued transaction, not actual residual subtraction.
CLI tests passed (11) before the run; evaluator static verification and ten
kernel tests passed before freeze 5ca324ef and the one-shot score.

- Mask-only score: 1.1370801747544546 m.
- P50: 0.9253215244662408 m; P95: 1.3488388250426682 m.
- Same dense support with real correction subtraction: 2.2517127747561947 m.
- Base-OFF operational baseline: 1.0769392017393964 m.
- Mask-only delta from baseline: +0.06014097301505816 m.

Candidate and truth read once each, one score, no interpolation/rescore.
This comparison implicates applying the numerical correction values in the
large paired regression under the frozen H configuration. Missing-P selection
alone does not reproduce that magnitude. It does not yet identify a wrong
sign, station reference error, clock/atmosphere bias, or general causal effect.
Do not promote mask-only as a production correction algorithm or claim a win:
it remains worse than the baseline and H is reused development data.

Next audit the base residual reference and satellite/receiver clock and
atmosphere conventions with raw-only stationary-base diagnostics and synthetic
known-error tests. Do not infer or fit a base coordinate shift, sign flip or
correction scale from H truth. Keep the operational baseline unchanged.
No MAT, saved positioning inference input, or submission was used. The
0.782-class and leaderboard objectives remain unachieved.
