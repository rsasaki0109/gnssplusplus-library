# FGO multi-band AR FIX-rate audit

## Decision

Keep conditional multi-band ambiguity resolution shadow-only. The two-stage
candidate generator is useful telemetry, but the available Tokyo run1 evidence
does not support turning its candidates into FIX labels or graph priors.

This work intentionally does not make TDCP the main FIX mechanism. TDCP and
Doppler remain complementary arc-integrity evidence; ambiguity acceptance is
controlled by integer-estimation evidence and independent integrity checks.

## Research basis

- Teunissen and Verhagen explain why ambiguity acceptance must control failure
  probability, and why a conventional ratio threshold is not by itself a
  guaranteed failure-rate test: <https://doi.org/10.1007/978-3-540-74584-6_22>.
- Odijk, Arora, and Teunissen describe partial ambiguity resolution by choosing
  a subset whose bootstrapped success-rate lower bound exceeds a required
  probability: <https://doi.org/10.1017/S037346331400006X>.
- Hou, Verhagen, and Wu evaluate a two-step success-rate criterion for PAR:
  <https://doi.org/10.1016/j.asr.2016.07.029>.
- RTKLIB supplies the operational comparison for multi-frequency arc handling,
  ambiguity validation, and consecutive-fix holding:
  <https://github.com/tomojitakasu/RTKLIB/blob/master/src/rtkpos.c>.
- GICI-LIB supplies the tightly-coupled comparison for ambiguity lifecycle and
  geometry-free cycle-slip handling:
  <https://github.com/chichengcn/gici-open/blob/master/src/gnss/ambiguity_common.cpp>.
- Suzuki's GNSS Odometry work shows the value of TDCP in a factor graph when
  cycle slips are explicitly detected and estimated, supporting its use as
  continuity evidence rather than an unconditional FIX authority:
  <https://arxiv.org/abs/2312.02424>.

The resulting design follows three constraints: select a primary subset,
condition the remaining Gaussian ambiguity distribution correctly, and never
activate a candidate until an offline truth audit demonstrates a low empirical
failure rate under deployable gates.

## Implementation

`FGOConfig::monitor_conditional_multiband_ar` is opt-in and independent of the
existing satellite-exclusion ratio-impact monitor. For every eligible epoch:

1. choose one primary band per satellite using the established deterministic
   selector;
2. run top-2 LAMBDA on the primary float vector and covariance;
3. if its configured ratio gate passes, compute the secondary conditional mean
   and Schur-complement covariance given the primary integer candidate;
4. run top-2 LAMBDA on the conditional secondary problem;
5. reconstruct a complete counterfactual integer vector and position; and
6. export dedicated ratios, bootstrapped success rates, counts, position, and
   float/IMU separation telemetry.

No estimator state, ambiguity generation, hold state, FIX/FLOAT status, or
reported position reads the shadow result.

## Reproducible 500-epoch A/B baseline

Dataset: PPC Tokyo run1. Both arms use the README fixed-lag-5 multi-frequency
preset including geometry-free slip reset. The shadow arm adds only
`--conditional-mf-shadow`; both use the same cached input problem.

| Metric | Baseline | Conditional shadow |
|---|---:|---:|
| Epochs | 500 | 500 |
| FIX epochs | 499 | 499 |
| Correct FIX, 3D error <= 0.5 m | 499 | 499 |
| Wrong FIX | 0 | 0 |
| Positioning epochs within 0.5 m | 500 | 500 |
| Conditional candidates | 0 | 197 |
| Changed status/position/ratio/AR-outcome rows | - | 0 |

Artifacts are
`build-ffrt-msvc/validation/multiband_fixrate_baseline_e500.csv` and
`build-ffrt-msvc/validation/multiband_fixrate_shadow_e500.csv`. Their SHA-256
digests are respectively
`BE3AFB5280E00767AE23856BE5351FE2C568A5C0F35DD908E766E15A27B31E6E` and
`F4766B1BDC442FA3EAF276A954C4B1D36EC1A6711CC6DEF5F1AED9836CD646CE`.

## Promotion audit

A historical full-length Tokyo run1 shadow replay generated 1,376 complete
two-stage candidates. Only 77 occurred on epochs not already labelled FIXED;
41 of those candidates were within 0.5 m and 36 were wrong. Tight IMU-position
agreement did not isolate a safe region: at 0.05 m separation, 9 candidates
were correct and 28 were wrong. These figures use reference truth only after
the solver run and are therefore evaluation results, not runtime inputs.

Consequently, ratio passage, high bootstrapped success rate, and proximity to
the IMU prediction are insufficient for activation on this data. Promoting the
shadow now would raise raw FIX rate while materially worsening wrong FIX/FIX.

## Next gate before activation

Activation requires a fresh full three-run Tokyo replay and a truth-free gate
that adds genuinely independent evidence, such as surplus satellites excluded
from the candidate construction or a clock-safe GF-triggered Doppler/TDCP arc
check. It must improve correct-FIX distance without increasing wrong-FIX
distance on any held-out run. Until that gate exists, the shipping preset is
unchanged.
