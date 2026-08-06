# DD pseudorange GNC shadow and activation plan

## Objective

Test whether graduated non-convexity (GNC) can improve the float state supplied
to RTK ambiguity resolution without relaxing LAMBDA, changing the carrier/IMU
model, or increasing wrong FIX. The experiment is DD-pseudorange-specific and
default-off. TDCP is not part of this experiment.

The method is motivated by Wen et al.'s GNC factor-graph formulation, which
uses a graduated Geman--McClure kernel to avoid the poor local minima observed
when a non-convex loss is enabled directly:
<https://ira.lib.polyu.edu.hk/bitstream/10397/92728/1/Wen_Gnss_Outlier_Mitigation.pdf>.
Unlike that pseudorange-positioning experiment, this project evaluates a
clock-free DD RTK/INS fixed-lag graph and treats correct and wrong FIX as the
primary outcomes.

## Milestone 1: fixed-linearization shadow

`FGOConfig::monitor_ddpr_gnc` evaluates the current epoch's post-fit DDPR
residuals after the normal smoother update. For normalized residual `r`, shape
`c`, and graduation scale `mu`, the diagnostic weight is

```text
w = mu*c^2 / (mu*c^2 + r^2)
```

The initial `mu` is chosen from the largest normalized residual, then divided
by 1.4 until it reaches 1 or the 32-stage limit. Defaults use `c=2`. This first
milestone deliberately holds the optimized state fixed: it measures the
candidate weight distribution but never replaces a factor, runs another
optimization, or changes FIX/FLOAT.

Enable it with one harness option:

```text
--ddpr-gnc-shadow
```

`--dump-csv result.csv` adds per-epoch `ddpr_gnc_*` columns and writes the
normalized per-factor trace `result.csv.ddpr_gnc.csv`. The trace identifies
time, satellite/reference pair, signal, residual, graph sigma, normalized
residual, and final shadow weight.

## Frozen experiment gates

1. **Correctness and non-interference.** Deterministic tests cover the weight
   ordering, gross-outlier suppression, invalid-input failure, and an exact
   fixed-lag monitor-off/monitor-on solution comparison.
2. **Tokyo run1 design slice.** Use source epochs 5000--5499 only. Compare the
   candidate-weight distribution with reference-labelled solution quality and
   freeze the GNC schedule before any active re-optimization.
3. **Counterfactual alternating solve.** Rebuild only DD pseudorange noise at
   each GNC stage, batch-refine a copy of the active graph, and record the
   resulting float position and ambiguity posterior. It must not update the
   live smoother or report FIX.
4. **Active run1.** Advance only if the counterfactual shows more correct FIX
   opportunities, zero additional wrong FIX, no lost correct FIX, no
   NONE/non-finite increase, at most 5% float/fixed RMS regression, and at most
   20% solver overhead with the frozen iteration cap.
5. **Sealed holdouts.** Run Tokyo run2 and run3 once with the frozen settings.
   Each must independently have zero wrong-FIX increase and no correct-FIX or
   matched-distance regression. A failure stops activation.

Reference truth is used only by offline scoring. It is never an input to GNC
weights or the estimator.

## Gate 1 smoke result

A shipping-profile A/B replay used 50 Tokyo run1 epochs beginning at source
epoch 5000. All pre-existing CSV fields were exactly identical between the
monitor-off and monitor-on runs. Both produced 49 FIX and one FLOAT, with
fixed horizontal RMS 0.0222491 m and no non-finite/NONE epoch.

The shadow evaluated all 1,051 DDPR factors:

| Metric | Result |
|---|---:|
| GNC-evaluated epochs | 50 / 50 |
| factors with weight below 0.5 | 175 |
| factors with weight below 0.1 | 2 |
| minimum weight | 0.055790389 |
| mean epoch weight | 0.73618 |
| effective factor count | 773.268 / 1,051 |

The strongest observed downweight was GPS G30 relative to G11 at TOW
188472.8: residual 3.271963 m, sigma 0.397671 m, normalized residual 8.227815,
and weight 0.055790. This proves useful dynamic range and exact estimator
non-interference, but not yet positioning benefit: the next required result is
the full 500-epoch design-slice analysis followed by a shadow-only alternating
solve.

The first 500-epoch diagnostic pass also found that a 15-stage research
default stopped before `mu=1` in 127 epochs (maximum initial `mu=6956.123`).
That run is not used for weight-quality conclusions. The default was raised to
32 stages, enough to reach the same final kernel for this urban residual
range; the design slice must be regenerated before Gate 1 is scored.

## Gate 1 design-slice result

The corrected 32-stage shadow was regenerated over all 500 frozen design
epochs. Every epoch reached `mu=1`; the estimator result remained 333 FIX and
167 FLOAT with zero NONE/non-finite epochs. Fixed horizontal RMS was
0.0288956 m. No FIX in this slice exceeded the 0.5 m correctness aperture, so
this slice cannot by itself test wrong-FIX rejection.

The final-kernel distribution was:

| Metric | Result |
|---|---:|
| DDPR factors | 7,104 |
| weight below 0.5 | 1,817 |
| weight below 0.1 | 366 |
| minimum weight | 0.000143738 |
| epochs truncated before `mu=1` | 0 |

Bad FLOAT epochs (horizontal error at least 1 m) had mean GNC weight 0.61 and
mean GNC-weighted residual RMS 0.87 m. FIX epochs had mean weight 0.72 and mean
weighted RMS 0.56 m. As an offline ranking diagnostic over 159 bad epochs and
335 sub-0.5 m epochs, GNC-weighted RMS achieved ROC AUC 0.75 versus 0.73 for
raw DDPR RMS. Mean weight, minimum weight, and downweighted fraction were
weaker (AUC 0.70, 0.63, and 0.67).

This is a modest but real signal, not an activation result. It supports the
next counterfactual alternating-solve milestone, while showing that a simple
weight-threshold FIX gate would add little beyond existing DDPR residual
telemetry and should not be implemented.

## Gate 2 counterfactual alternating-solve result

The default-off counterfactual now copies the active fixed-lag graph for each
final FLOAT epoch, replaces only DDPR noise with graduated Geman--McClure
weights, and runs at most eight one-iteration LM stages. Carrier, IMU, motion,
and marginal factors remain unchanged. Candidate Values never update iSAM2,
reported positions, ambiguity hold, or FIX/FLOAT. The final copied graph also
attempts a diagnostic full-set LAMBDA ratio from its batch marginal; this is
not a reproduction of the production partial-AR cascade and is labelled as
such in the CSV.

A synthetic 30 m single-DDPR outlier test verifies that the copied solve can
improve position while the live status, ratio, and ECEF trajectory remain
exactly unchanged. The frozen Tokyo run1 source-epoch 5000--5499 design slice
then produced the following result:

| Metric | Result |
|---|---:|
| final FLOAT epochs / successful candidates | 167 / 166 |
| candidate position improved | 26 / 166 (15.7%) |
| candidate position worsened | 140 / 166 (84.3%) |
| FLOAT horizontal RMS, live -> candidate | 4.849 -> 7.370 m |
| live errors >=1 m rescued below 0.5 m | 1 / 158 |
| candidate full-set LAMBDA marginal evaluated | 39 / 166 |
| candidate full-set ratio passed | 1 / 39 |
| bad FLOAT epochs converted to a ratio-passing candidate | 0 |
| original graph cost decreased | 134 / 166 |
| DDPR RMS decreased | 119 / 166 |
| solver time, monitor off -> on | 15.753 -> 52.954 s (+236%) |

The only ratio-passing counterfactual was the already-correct first FLOAT
epoch: its live ratio rose from 1.243 to 3.483, but the GNC candidate moved
about 1.24 m from the final reported solution. This is precisely the unsafe
failure mode the shadow was intended to expose: a stronger integer ratio does
not imply a better absolute position when DDPR is reweighted independently of
the carrier/IMU basin.

Gate 2 therefore fails both quality and runtime requirements. Do **not** feed
this DDPR-only alternating solve into the live smoother, do not use its ratio
to report FIX, and do not spend sealed run2/run3 holdouts on threshold tuning.
The fixed-linearization weights remain useful diagnostic telemetry. A future
robust-state experiment should instead use independent absolute-position or
temporal-consistency evidence to gate candidate hypotheses, and must establish
a new run1-only plan before any activation work.
