# Satellite quarantine shadow plan

## Objective

Identify individual carrier ambiguity arcs that should be restarted after an
urban fault, without retaining contaminated carrier continuity and without
changing the live graph, LAMBDA, CP-hold, or reported solution.

Song et al. first compare pseudorange change with Doppler and then compare DD
pseudorange with short-term inertial/odometer prediction before modifying an
RTK/INS factor graph: <https://arxiv.org/abs/2510.00524>. Wang et al. show why
measurement faults and incorrect integer hypotheses must be monitored
together: <https://doi.org/10.33012/2023.18607>.

## Frozen monitor rule

For each observed satellite at an epoch:

1. aggregate the existing post-fit DDPR residual attribution;
2. require the existing causal predicted-DDPR row to exceed 5 sigma in both
   its Doppler-change and IMU-geometry innovations for the same DD pair;
3. require the satellite's DDPR residual to exceed both 10 m and five times
   the epoch median residual.

The satellite becomes a shadow quarantine candidate only when all conditions
agree. A DD pair contributes temporal support to both its target and reference
satellites because differencing alone cannot identify which endpoint is
faulty. The output records that ambiguity explicitly; it has no estimator
authority.

Enable with `--satellite-quarantine-shadow`.

## Gates

### Gate 0

- monitor off/on solutions, ratios, and statuses are exactly equal;
- a synthetic single-satellite DDPR jump with zero Doppler-predicted change
  is attributed to the involved pair;
- no candidate changes LAMBDA membership or ambiguity generation.

### Tokyo run1 source epochs 5000--5499

- report candidate epochs/satellites and temporal-support coverage;
- zero quarantine candidates on correct FIX epochs;
- identify at least one candidate during a bad-FLOAT or CP-hold stretch;
- exact solution non-interference and no NONE/non-finite increase;
- monitor overhead at most 20%.

Failure stops without threshold tuning. An active selective arc restart is
considered only after this shadow passes; full run1 and run2/run3 remain
sealed until then.

## Implementation

The GTSAM fixed-lag backend records the maximum absolute DDPR post-fit
residual attributed to each satellite. After the live solve has finished, it
joins those rows to the existing causal predicted-DDPR analysis. The joined
rows and their explicit target/reference ambiguity are exported to
`<epoch.csv>.satellite_quarantine.csv`. The monitor does not add, remove, or
reweight a graph factor and does not modify an ambiguity generation.

`FGOSatelliteQuarantineShadowTest` injects a 30 m DDPR jump with a zero
Doppler-predicted change. Both endpoints are reported as candidates while the
OFF/ON position, status, ratio, fixed ambiguity count, and generation count
remain equal.

## Frozen Tokyo 500 result

Command profile: run1 source epochs 5000--5499, 5 s fixed lag, multi-frequency
partial AR, hold, CMC, CP-hold, exception recovery, DDPR anchor, FDE, varerr,
and FIX demotion. No threshold was changed after observing the result.

- Baseline and shadow both produced 333 FIX, 167 FLOAT, and zero NONE epochs.
- The complete epoch CSV files are byte-identical (SHA-256
  `6A3A70CF4F755FB7756E565136037D4BAC5FFBFA5D9EA08420117AF590713F76`).
- 6,891 satellite-epoch rows produced two satellite candidates in one epoch:
  BeiDou C11 and C14 at epoch 461 (TOW 188563 s). The pair residual was
  106.630 m versus a 1.631 m epoch median; both Doppler and IMU innovations
  exceeded 5 sigma.
- Epoch 461 was FLOAT with 3.846 m horizontal error and CP-hold active.
- Candidate epochs on the 333 correct FIX epochs: zero.
- Coverage: one candidate epoch among 165 bad-FLOAT epochs and one among 127
  CP-hold epochs. This is deliberately high precision and low recall.
- Wall time was 21.099 s OFF and 22.689 s ON: 7.54% overhead, below the frozen
  20% ceiling.

All frozen gates pass. The monitor is suitable as evidence for a narrowly
scoped active experiment, but the low recall does not justify broad
satellite deletion. The next experiment should restart only the implicated
carrier ambiguity arcs for both endpoints at a candidate epoch, preserve the
DDPR factors, and retain a shadow/off control plus an immediate rollback gate
for any loss of correct FIX epochs.
