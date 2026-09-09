# Phase322/323: state-only ablation does not reproduce the large paired regression

Executed the Phase321-preregistered comparison once: Phase234 flags plus
`--native-rover-epoch-states`, base correction/mask and explicit-code-bias
override OFF. Freeze commit a954c1dd; native exit 0 in 282.854 seconds.
GNSS-first and IMU/main converged with valid finite termination and aligned
in-memory handoff. There were 101915 P factors, versus baseline 101916 and
paired 81394. Candidate had 3139 exact UTC rows and was frozen at 496ea7d2.

The Phase323 evaluator was frozen at aa3653bc after static verification and
ten existing kernel tests. Candidate/truth each read once, score once:

- P50: 0.8626503772165134 m.
- P95: 1.326993308030869 m.
- Score: 1.094821842623691 m.
- Baseline: 1.0769392017393964 m.
- Delta: +0.017882640884294698 m (worse).

Do not promote. Retain baseline as operational. This state-only configuration
slightly regresses on reused H development data but does not reproduce the
paired configuration's 2.2515351668966965 m score. This directs investigation
toward the remaining coupled base correction, code-bias, and missing-P changes.
Do not subtract scores to claim independent causal contributions: interactions
and the three internal state changes are not separately identified.

Next use source equations and raw-only/synthetic diagnostics to test the base
residual reference/sign, group-delay pairing and effect of missing correction
streams. No truth-derived coordinate offsets, correction scaling, score sweep,
or rescore of these candidates. No MAT or saved positioning input was used;
saved candidate output was used only for integrity checks and evaluation.
No submission; neither 0.782-class accuracy nor leaderboard standing established.
