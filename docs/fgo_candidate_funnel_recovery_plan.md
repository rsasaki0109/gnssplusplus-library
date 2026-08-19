# Candidate funnel and CP-hold float-recovery plan

## Objective

Increase correct FIX opportunities by repairing the float ambiguity state
before LAMBDA, without relaxing the integer ratio or allowing held/quarantined
carrier rows to create a FIX.

This experiment follows the staged-fault-detection direction of Song et al.,
who combine Doppler change checks with IMU/odometer-predicted DD pseudorange
checks before changing an RTK/INS factor graph:
<https://arxiv.org/abs/2510.00524>. It also follows the integrity lesson from
Wang et al. that measurement faults and incorrect integer hypotheses must be
considered together: <https://doi.org/10.33012/2023.18607>.

The preceding candidate-integrity witness was safe but could not evaluate a
single final-FLOAT candidate. The frozen Tokyo run1 source-epoch 5000--5499
CSV already contains a complete ambiguity attrition trace, so no duplicate
monitor is added.

## Frozen funnel audit

The 167 final-FLOAT epochs stopped at:

| Terminal stage | Epochs |
|---|---:|
| no candidates | 128 |
| fewer than six candidates | 37 |
| ratio rejected | 1 |
| fixed then plausibility-demoted | 1 |
| marginal failure | 0 |
| LAMBDA search failure | 0 |

Of the 128 no-candidate epochs, 127 were inside CP-hold. They had 621 carrier
rows suppressed in total. Only two final-FLOAT epochs produced a provisional
LAMBDA candidate. The dominant problem is therefore carrier/ambiguity
reacquisition during recovery, not LAMBDA search or candidate validation.

## Frozen experiment

Evaluate the existing default-off `FGOConfig::use_cp_hold_float_recovery`
path with its existing scale of 10.0. During CP-hold it keeps carrier factors
in the float graph at ten times their normal sigma, but marks every associated
ambiguity as hold-quarantined and excludes it from LAMBDA. It cannot produce a
FIX or pin an integer during the hold. The hypothesis is narrower: a weak
carrier channel may preserve float ambiguity observability and improve
post-release reacquisition.

Expose exactly one research CLI switch:

```text
--cp-hold-float-recovery
```

No sigma sweep, ratio relaxation, early hold release, selective hold, or
witness-aperture tuning is allowed in this experiment.

## Gates

### Gate 0: deterministic safety

- default-off behavior remains unchanged;
- a real CP-hold fixture keeps carrier factors in the graph only when the
  switch is on;
- held carrier ambiguities remain excluded from LAMBDA;
- no held epoch can be labelled FIXED from the quarantined rows.

### Gate 1: Tokyo run1 source epochs 5000--5499

- zero additional wrong FIX above 0.5 m 3-D;
- zero lost correct FIX;
- at least three additional correct FIX epochs;
- no NONE/non-finite increase;
- fixed horizontal RMS/P95 regression at most 5%;
- FLOAT horizontal RMS improves at least 5%; and
- solver overhead at most 10%.

Failure stops without tuning. Only a Gate-1 pass permits full run1. Tokyo
run2/run3 remain sealed until a full-run1 activation passes.

## Gate-1 result

The single frozen Tokyo run1 replay failed decisively. It kept the same 333
FIX / 167 FLOAT labels and did not add a wrong FIX, but it added no correct
FIX either. Fixed positions stayed byte-for-byte equivalent at the CSV
precision; the FLOAT trajectory did not:

| Metric | Baseline | Float recovery |
|---|---:|---:|
| correct FIX | 333 | 333 |
| wrong FIX above 0.5 m 3-D | 0 | 0 |
| FLOAT horizontal RMS | 4.843 m | 72.469 m |
| FLOAT horizontal P95 | 11.796 m | 193.629 m |
| FLOAT horizontal max | 20.801 m | 288.538 m |
| solver time | 15.753 s | 19.288 s |
| final-FLOAT provisional candidates | 2 | 32 |

The mechanism did open the candidate funnel, but in the wrong state basin.
All 32 final-FLOAT provisional candidates were wrong at the 0.5 m 3-D
aperture; their candidate-error RMS was 107.565 m, and all eight with ratio at
least 2 were also wrong. Recovery produced 80 CP-hold triggers, 13 fast
resets, four smoother-recovery epochs, and large rank-deficiency episodes.
Weak carrier factors preserved contaminated ambiguity continuity instead of
recovering it.

Gate 1 fails the required +3 correct FIX, FLOAT improvement, and runtime
limits. Do not tune the sigma scale, do not activate this path, and do not run
full run1 or sealed run2/run3. The next experiment must break contaminated
ambiguity continuity and obtain a fresh, independently anchored carrier
state; merely retaining old carrier information is unsafe.

## Selected next experiment

Do not immediately activate the existing residual-only selective CP-hold.
Its satellite attribution is computed at the live float pose and is therefore
correlated with the wrong basin this experiment exposed. First add a
monitor-only per-satellite **two-stage quarantine witness**:

1. current/causal Doppler or TDCP change evidence;
2. IMU-predicted DD pseudorange disagreement;
3. agreement with the existing per-satellite DDPR residual attribution.

Only rows supported by independent temporal evidence become quarantine
candidates in the shadow. The scorer must report coverage and false-positive
rates on finally-correct FIX rows and bad FLOAT stretches. A later active
experiment, if justified, will generation-bump and remove only witnessed bad
arcs while preserving clean carrier arcs. It will not keep contaminated arcs
alive at a weaker weight.
