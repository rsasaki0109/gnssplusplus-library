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
- the previous/current DDPR residual against the causal predicted geometry;
- elevation and rover/reference SNR; and
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

## Development checkpoint

The default-off monitor passed Gate 0 on both the first 500 epochs and the
full 11,905-epoch run1: status, ECEF position, ratio, fixed ambiguity count,
and AR outcome were byte-for-byte identical to the preserved baseline CSV.
The focused clock-cancellation, fail-closed, and fixed-lag non-interference
tests also pass.

The original rule requiring both normalized temporal innovations to exceed a
common threshold did not meet the full-run development target.  At 0.2 sigma
it captured 50.973% of gross truth-labelled rows but flagged 12.531% of clean
rows; at 0.3 sigma the rates were 40.675% and 5.634%.  This rule is rejected.

The causal absolute residual against the one-step IMU-predicted geometry is a
substantially stronger development score.  Freezing the rule
`abs(current predicted DDPR residual) > 1.5 m` on run1 captures 85.522% of
14,498 gross rows while flagging 2.848% of 153,347 clean rows.  The 30,023
rows between 1 m and 4 m are an explicit ambiguity band and are excluded from
Gate 1 scoring.  This 1.5 m threshold is now frozen for the run2 validation;
it must not be adjusted using run2 or run3.

With that rule unchanged, run2 passes Gate 1: it captures 5,705 of 5,894
gross rows (96.793%) and flags 477 of 166,600 clean rows (0.286%).  The
monitor remains non-interfering across all 9,147 run2 epochs, with exact
agreement in status, ECEF position, ratio, fixed ambiguity count, and AR
outcome.  This unlocks the bounded-downweight experiment on run1.  Run3
remains sealed until the activation rule, inflation scale, and all other
parameters have been frozen.

The bounded activation sweep on the first 500 run1 epochs rejected 3x and 5x
because each introduced one wrong FIX.  The 2x variant introduced no wrong
FIX but exchanged one previously correct FIX epoch for another and therefore
failed the strict per-epoch no-loss gate.  The 1.5x and 1.75x variants changed
no FIX/FLOAT labels.  The 1.25x variant was the only improving safe candidate:
one additional correct FIX, no lost correct FIX, no wrong FIX, and no NONE or
non-finite epoch.  Consequently 1.25x is the sole candidate advanced to the
full-run1 activation A/B; the stronger multipliers are permanently rejected.

The full-run1 A/B rejects that remaining 1.25x candidate.  It increased
correct-FIX distance by 148.467 m (+1.439 percentage points) and improved the
official 50 cm distance by 144.574 m (+1.401 pp), but it also increased
wrong-FIX distance by 189.135 m (+1.833 pp) and lost 619 previously correct
FIX epochs.  This violates the zero-wrong-FIX-increase stop condition even
though the net correct-FIX count increased.  Therefore the activation switch
was removed, run2/run3 activation was not run, and only the non-interfering
telemetry is retained.  The run3 observation-quality labels remain sealed;
no activation result was inspected there.

As the next independent observation-model check, the existing receiver-clock-
drift-free single-difference Doppler velocity factors were evaluated on the
same run1 500-epoch development slice at 0.5 m/s (the less aggressive setting
favoured by the earlier RTK study).  They added 18,356 Doppler rows but changed
no FIX/FLOAT decision: both baseline and candidate had 499 correct FIX and
zero wrong FIX.  Meanwhile fixed horizontal RMS regressed from 0.0422 m to
0.0539 m (about 28%).  With no correct-FIX gain and a clear accuracy
regression, this model was not advanced to a full run.  The temporary harness
sigma override was removed as well.

## Stop conditions

Stop before activation if the score is mainly explained by the optimized
current pose, if DD formation does not cancel injected receiver-clock jumps,
if run2 misses Gate 1, or if the first bounded-downweight run adds any wrong
FIX. These conditions prevent another expensive threshold search over a
non-independent witness.
