# FGO FIX-rate observation-quality plan

## Objective

Close the remaining approximately 0.7955 percentage-point gap to the current
`+2 pp` correct-FIX-distance goal without increasing wrong-FIX distance on a
held-out Tokyo run. Raw FIX labels are not the optimization target: a change
that promotes more wrong integer solutions fails even when the reported FIX
rate rises.

The next experiment targets the float observation model before LAMBDA. It does
not further tune TDCP thresholds and does not promote the existing multi-epoch
or conditional multi-band shadow candidates.

## Evidence already consumed

Do not repeat the following rejected directions:

- temporal integer agreement, BSR, and ratio alone;
- held-out carrier rows from the same urban measurement basin;
- constrained graph cost using the same active factors;
- conditional multi-band AR;
- direct temporal-carrier promotion or a TDCP-based FIX gate; and
- uncapped satellite-badness feedback, which starved ambiguity resolution.

The current three-run logs also separate the remaining opportunity:

- run1 correct FLOAT epochs are split mainly between no/insufficient
  candidates and ratio rejection;
- run2 and run3 correct FLOAT epochs are dominated by ratio rejection; and
- lowering the ratio threshold is unsafe because the rejected population also
  contains many wrong candidates.

The working hypothesis is therefore that biased DD pseudorange factors are
degrading the float state and ambiguity covariance before integer search. The
candidate generator should be improved before any acceptance rule is relaxed.

## Research basis

- Song et al. (2025) compare pseudorange change with Doppler-predicted range
  change, then compare DD pseudorange change with IMU/odometer-predicted DD
  geometry. Measurements that fail are downweighted before FGO:
  <https://arxiv.org/html/2510.00524v1>.
- Wen and Hsu's GraphGNSSLib supplies an open FGO/RTK reference using DD
  pseudorange, carrier phase, Doppler, and LAMBDA, and points to graduated
  non-convexity work for urban outlier mitigation:
  <https://github.com/weisongwen/GraphGNSSLib>.
- Gao et al. connect pseudorange, carrier, and Doppler in an urban RTK factor
  graph and perform ambiguity resolution after the float optimization:
  <https://doi.org/10.33012/2022.18190>.
- Teunissen-style integer aperture remains the safety model: observation
  cleanup may improve the float distribution, but it does not authorize
  relaxing the existing ambiguity acceptance tests.

## Proposed experiment

Add an opt-in, monitor-only `predicted DDPR quality` trace. For every DD
pseudorange row with an uninterrupted target/reference pairing, calculate two
causal innovations:

1. **DD Doppler consistency:** compare the epoch-to-epoch measured DD
   pseudorange change with integrated DD Doppler range change.
2. **IMU geometry consistency:** compare the same measured change with the DD
   geometric-range change from the one-step IMU-predicted antenna pose.

Both tests must use satellite double differences. This cancels common rover
and base receiver-clock terms and avoids importing the raw clock-jump failure
seen by the temporal-carrier investigation. The current epoch's optimized pose
must not be used to classify its own input factor.

The monitor exports, per DD row:

- epoch, target, reference, signal, and arc/pair age;
- measured DDPR change;
- Doppler- and IMU-predicted change;
- signed innovation and propagated sigma for each witness;
- elevation, rover/reference SNR, existing sigma, CMC/GF/FDE state;
- a causal robust center/scale estimate; and
- proposed action (`keep`, `downweight`, or `unavailable`).

No graph factor, covariance, ambiguity set, hold state, reported position, or
FIX/FLOAT status reads the monitor result.

Only after the monitor passes its gates may a separate experimental switch
apply a bounded sigma inflation to DD pseudorange factors. Hard exclusion is
out of scope for the first activation experiment. Carrier factors and LAMBDA
thresholds remain unchanged.

## Dataset and holdout protocol

1. **Development:** Tokyo run1 only. Select normalization and bounded
   downweighting parameters.
2. **Validation:** freeze all rules, then run Tokyo run2 once. A failure ends
   the experiment; do not tune against run2.
3. **Final holdout:** run Tokyo run3 only after run2 passes. Do not unlock or
   adjust any threshold after seeing run3.
4. Preserve the existing baseline CSVs and problem caches. Every experimental
   output uses a new prefix and records the exact command and cache
   fingerprint.

Reference truth is used only for offline scoring. Runtime classification may
use GNSS observations, Doppler, IMU prediction, and causal state, but never
`reference.csv`.

## Gates

### Gate 0: monitor non-interference

- status, ECEF position, ratio, fixed ambiguity count, and AR outcome are
  identical to baseline at every epoch;
- common rover/base clock-jump unit tests do not create a DD innovation; and
- missing Doppler, pair changes, gaps, or non-finite covariance fail to
  `unavailable`, never `bad`.

### Gate 1: witness usefulness

Using truth-position DDPR residuals only as offline labels, the frozen monitor
must, on run2, capture at least 50% of gross DDPR rows while flagging no more
than 5% of clean rows. Report the complete precision/recall curve; the stated
operating point is the minimum continuation gate, not a post-hoc optimum.

### Gate 2: short solver A/B

On a representative 500-epoch run1 slice:

- zero additional wrong FIX epochs;
- no loss of correct FIX epochs;
- no additional smoother failure or non-finite epoch; and
- wall-clock overhead at or below 10%.

### Gate 3: full-run activation

With parameters frozen after full run1:

- wrong-FIX distance must not increase on run2 or run3 individually;
- correct-FIX distance must not decrease on either holdout;
- aggregate correct-FIX distance must close the remaining 0.7955 pp goal;
- official score and all-solution `<50 cm` distance must not regress by more
  than 0.1 pp on any run;
- fixed horizontal RMS and P95 must not regress by more than 5%; and
- matched positioning distance and solver-failure counts must not regress.

If the improvement is positive but below the target, keep the switch
experimental and document the measured contribution. If any safety condition
fails, retain telemetry and reject activation.

## Implementation sequence

1. Checkpoint the completed temporal-carrier work, then create a dedicated
   observation-quality branch so the two experiments remain reviewable.
2. Add solver-independent DDPR innovation records and deterministic unit
   tests for geometry, clock cancellation, gaps, pair changes, and missing
   witnesses.
3. Wire a default-off CLI monitor and normalized per-factor CSV.
4. Run baseline/monitor A/B and Gate 1 without changing the graph.
5. If Gate 1 passes, add bounded DDPR sigma inflation behind a second
   default-off switch and run the staged A/B protocol.
6. Promote nothing unless every full-run gate passes; otherwise record the
   negative result and move to a different observation model.

## Stop conditions

Stop before activation if the score is mainly explained by the optimized
current pose, if DD formation does not cancel injected receiver-clock jumps,
if run2 misses Gate 1, or if the first bounded-downweight run adds any wrong
FIX. These conditions prevent another expensive threshold search over a
non-independent witness.
