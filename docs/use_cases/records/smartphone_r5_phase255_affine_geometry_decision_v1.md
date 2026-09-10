# Phase255 H affine geometry decision

The Phase254 raw-only native run completed once (349.021182 seconds), with
69,270 affine TDCP factors inserted in each stage, finite converged native
stages, and 3,139 exact published epochs without interpolation or edge hold.
The implementation remains default-off. No MAT or precomputed solution
input was used. The main initialization was the same-run in-memory handoff.

Before scoring, 18 selected metadata/evaluator tests passed; evaluator,
candidate identity, metric dependencies, and authorization were frozen in
commit 5ee4f88. Phase255 then read candidate and H truth once each and
calculated one score. All 3,139 prediction and truth keys matched; finite
Earth-position and over-70-m/s gates passed. No offset was reapplied.

| H development metric | Phase234/235 reference | Phase254/255 affine |
| --- | ---: | ---: |
| (P50 + P95) / 2, metres | 1.0769392017393964 | 1.1176830315705608 |
| P50, metres | 0.8369083196573837 | 0.876012905544532 |
| P95, metres | 1.3169700838214091 | 1.3593531575965896 |

Delta is +0.040743829831164424 m: both quantiles and the scalar worsened.
Do not promote the affine-only recipe. Keep the native endpoint geometry,
metre-domain dynamic sigma, k=4, and resL-off reference.

H is repeatedly used development data. This is neither independent
generalization evidence nor a leaderboard result. The 0.782-class goal is
unmet; no submission was made.

Next investigate the coupled measurement convention before another raw
experiment: compare the source resL plus previous-LOS affine formulation
against native atmosphere-corrected carrier and satellite geometry using
independent synthetic residuals and Jacobians. The isolated resL experiment
(Phase244) and isolated affine experiment both worsened H; neither proves
that their combination is source-consistent or beneficial. Do not assume
additivity, launch a parameter sweep, or tune against U truth. Establish
the physical/coordinate convention first, then decide whether a single
predeclared combined test is justified.
