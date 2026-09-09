# Phase496 — main-only code uncertainty floor rejected on H

Structural verification repeated successfully, then evaluation manifest and
candidate/evaluator hashes frozen before one exact-key development evaluation.
Existing Phase203/199/189 scoring kernel unchanged. Exclusive attempt marker
prevents repeat scoring. Evaluation returned 0; all 3139 keys matched, both
coverages 1.0, finite metrics, no over-70-m/s steps.

- P50: 0.9570547923236163 m
- P95: 1.3788651939352206 m
- Route score: 1.1679599931294185 m
- Historical operational baseline: 1.0769392017393964 m
- Difference: +0.09102079139002206 m (worse)

Keep --native-main-code-uncertainty-floor default off, do not promote. Both
P50 and P95 worsen; this is not a tradeoff hidden by the scalar metric.
Baseline includes no lambda floor whereas candidate does; the independently
measured floor-only maximum output difference on H (~0.305 mm) cannot explain
the approximately 91 mm degradation. This is reused H development evidence,
not heldout/LB evidence or a universal rejection of Android uncertainty.

Do not tune a multiplier on this route. The raw diagnostic showed the floor
weakens every retained P factor; native robust loss additionally changes its
physical transition scale. Historical older-recipe improvement does not
transfer automatically. A both-stage floor remains untested but should not
be selected merely to chase this score. Next seek independent raw evidence
for selective error modelling or admission, rather than global weakening of
all P factors. No further truth reads, submission or production-default change.
Overall .782/leaderboard objective remains unmet.
