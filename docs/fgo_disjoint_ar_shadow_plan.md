# FGO disjoint-constellation AR shadow plan

## Objective

Improve correct-FIX distance without increasing wrong-FIX distance.  The next
experiment does not relax the shipped ratio threshold and does not make TDCP
an acceptance authority.  It asks whether an ambiguity candidate is reproduced
by two satellite-disjoint constellation partitions before any lower-ratio
candidate is considered for promotion.

The first implementation is monitor-only.  It cannot change the active graph,
integer priors, ambiguity hold state, reported position, or FIX/FLOAT status.

## Why this experiment is next

The completed Tokyo audits reject the following as sufficient acceptance
evidence:

- temporal integer agreement, bootstrapped success rate, and ratio alone;
- conditional multi-band ambiguity resolution;
- held-out carrier rows from the same urban measurement basin;
- constrained graph cost evaluated on the same active factors;
- direct DD pseudorange downweighting or causal bias subtraction; and
- a TDCP-specific gate while raw receiver-clock jumps remain unresolved.

All of those checks can agree inside one biased urban measurement basin.  A
satellite-disjoint split changes the carrier observations used to form each
integer candidate and exposes whether the resulting fixed positions separate.

## Research and OSS basis

- Wang et al. apply modified multiple-hypothesis solution separation to
  ambiguity-resolved GNSS and compare subset solutions with the all-in-view
  solution while accounting for incorrect-fix events:
  <https://doi.org/10.33012/2023.18607>.
- Teunissen's ambiguity-validation framework treats acceptance as a separate
  decision from integer estimation.  Extra availability is not evidence of a
  controlled failure rate:
  <https://www.ion.org/publications/abstract.cfm?articleID=11030>.
- Verhagen describes success-rate bounds and the pitfalls of common
  discrimination tests, supporting a fail-closed shadow before activation:
  <https://doi.org/10.1002/j.2161-4296.2005.tb01736.x>.
- RTKLIB performs LAMBDA validation and preserves explicit satellite/arc
  lifecycle and residual checks rather than treating a ratio result as a
  complete integrity proof:
  <https://github.com/tomojitakasu/RTKLIB/blob/master/src/rtkpos.c>.
- GICI-LIB retries partial ambiguity subsets and rejects a constrained graph
  update when range cost worsens.  Those mechanisms are useful comparisons,
  but the existing Tokyo audit shows that same-graph cost is not independent
  enough on its own:
  <https://github.com/chichengcn/gici-open/blob/master/src/gnss/ambiguity_resolution.cpp>.

The repository already contains a stricter disjoint-satellite evidence
contract for the standalone RTK path.  The FGO shadow deliberately does not
claim that same level of independence: both partition marginals originate
from the same float graph and share IMU/code information.  The carrier
ambiguity rows and DD reference satellites are disjoint, but common-state
correlation remains.  This limitation is why the first stage is telemetry,
not an activation switch.

## Shadow construction

For each epoch with at least two represented GNSS constellations:

1. group eligible ambiguity rows by constellation, keeping every target and
   its same-constellation DD reference in one group;
2. assign whole constellation groups greedily to two balanced partitions,
   with deterministic constellation-ID tie breaking;
3. require at least four ambiguities in each partition;
4. extract each partition's float ambiguity subvector, covariance submatrix,
   and position/ambiguity cross-covariance;
5. run the existing top-two LAMBDA implementation independently on A and B;
6. form each partition's covariance-conditioned candidate position; and
7. export partition ratios, bootstrapped success rates, system masks,
   candidate positions, A/B separation, and separation from the primary
   all-in-view candidate.

No thresholded consensus verdict is needed in the runtime monitor.  Offline
scoring must report complete trade-off curves and freeze a rule using run1
only.

## Precommitted gates

### Gate 0: exact non-interference

- monitor off/on epoch CSVs are identical in status, ECEF position, ratio,
  fixed ambiguity count, and AR outcome;
- missing partitions, singular covariance, a failed LAMBDA search, or a
  non-finite candidate reports unavailable and never bad; and
- neither partition candidate can reach graph mutation or ambiguity hold.

### Gate 1: development usefulness on Tokyo run1

Using reference truth only after replay:

- report coverage separately for correct FIX, wrong FIX, correct FLOAT, and
  wrong FLOAT primary candidates;
- identify a truth-free separation rule that catches at least 50% of wrong
  primary candidates while rejecting at most 5% of correct primary
  candidates;
- require at least 100 scored correct and 100 scored wrong primary candidates;
  and
- report results by partition composition so one favourable constellation
  split cannot hide a failing split.

If support is insufficient or the operating point fails, retain telemetry and
stop.  Do not tune on run2.

### Gate 2: frozen validation on Tokyo run2

Apply the run1-frozen rule once.  It must meet the same 50% wrong-candidate
capture and 5% correct-candidate harm limits.  Any failure ends activation;
run3 remains sealed.

### Gate 3: bounded activation experiment

Only after Gates 0-2 pass may a separate default-off switch use consensus to
permit a narrowly relaxed-ratio candidate.  Start with a representative
500-epoch run1 slice and require:

- zero additional wrong FIX epochs;
- no lost correct FIX epochs;
- no new NONE/non-finite epoch;
- fixed RMS and P95 regression no greater than 5%; and
- runtime overhead no greater than 15%.

Then freeze every threshold on full run1, validate run2, and inspect sealed
run3 only after run2 passes.  Wrong-FIX distance may not increase and
correct-FIX distance may not decrease on either holdout individually.

## Execution sequence

1. Add the default-off shadow, CLI flag, normalized CSV, and deterministic
   partition helper.
2. Unit-test balanced whole-constellation assignment, fail-closed behavior,
   and candidate separation.
3. Prove exact baseline non-interference on a short replay.
4. Run and score full Tokyo run1.
5. Continue to run2, activation, and sealed run3 only when every preceding
   gate passes.

## Development result

The default-off implementation passed a 50-epoch Tokyo run1 non-interference
A/B.  Baseline and shadow matched in all 50 rows for time, status, ECEF
position, ratio, fixed ambiguity count, and AR outcome.  Both partition
candidates were available in every epoch; their position separation had a
5.388 mm median and 21.439 mm maximum in this clean opening segment.

Full Tokyo run1 produced 7,049 truth-scorable primary candidates from 11,905
epochs.  The population contained 4,244 correct candidates (3D error at most
0.5 m) and 2,805 wrong candidates, so the precommitted support requirement was
comfortably met.  The run1-selected rule rejects a primary candidate when the
maximum of A/B, A/primary, and B/primary position separation exceeds
0.421369 m.  It gives:

| Metric | Result | Gate |
|---|---:|---:|
| Correct candidates rejected | 212 / 4,244 = 4.995% | at most 5% |
| Wrong candidates caught | 1,242 / 2,805 = 44.278% | at least 50% |

Gate 1 therefore fails on development data.  The failure is not caused by
insufficient labels: satellite-disjoint ambiguity subsets can still agree
inside a shared biased float-state basin.  The full shadow solver time was
848.646 s; the extra two LAMBDA searches per eligible epoch also make this
substantially more expensive than the existing same-preset baseline reports.

Decision: retain the raw monitor and scorer as diagnostic evidence, but do not
add a consensus veto, relaxed-ratio rescue, graph prior, or FIX label.  Tokyo
run2 and sealed run3 were not inspected for this experiment.
