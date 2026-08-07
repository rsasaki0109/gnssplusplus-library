# Selective carrier-arc restart plan (active quarantine follow-up)

## Objective

The preceding satellite quarantine witness passed every frozen gate
(docs/fgo_satellite_quarantine_shadow_plan.md): exact OFF/ON estimator
equality, zero candidates on correct FIX epochs, one bad-FLOAT/CP-hold
candidate epoch (BDS C11/C14 at run1 epoch 461), and 7.54% overhead. This
experiment advances from witness to a narrowly scoped ACTIVE mechanism: at a
quarantine-candidate epoch, restart only the implicated carrier ambiguity arcs
for both endpoints, preserve the DD pseudorange factors, and keep a
shadow/off control plus an immediate rollback gate for any loss of correct FIX
epochs.

It is default-off and bit-identical when off, exactly like every prior FGO
audit in this project.

## Causal candidate rule (frozen)

The witness is post-hoc. The active path must decide **before** an epoch's
carrier factors are added, so it runs as a one-epoch-delayed arm, mirroring
`use_selective_cp_hold` (detect at epoch i post-fit, apply at epoch i+1 factor
build).

For each DD pseudorange pair observed at epoch i:

1. **Doppler innovation** (existing `monitor_predicted_ddpr_quality`
   arithmetic, kept causal): |measured DD-PR change - trapezoidal DD-Doppler
   prediction| / sigma > 5.
2. **IMU-geometry innovation** (same source): |measured change - predicted
   change at the IMU-propagated epoch seed| / sigma > 5.
3. **Post-fit gross attribution** (existing `per_sat_res`, charged to both
   endpoints): residual > `cp_hold_fast_worst_satellite_min_m` (10 m) and
   > `cp_hold_multipath_median_ratio` * epoch median.

A satellite becomes a restart candidate only when all three agree, and the DD
pair attributes support to BOTH its target and reference satellite (as in the
witness). Candidate satellites are accumulated for epoch i, then used to bump
generations of their carrier ambiguity arcs at epoch i+1.

## Active mechanism (default-off `--selective-arc-restart`)

- Reuse the existing `amb_generation` arc-regeneration overlay
  (`invalidateArcs` semantics): bumping an ambiguity index's generation mints a
  fresh backend-local symbol on its next observation, which the fresh-arc
  bookkeeping seeds like a genuinely new arc.
- Restart ONLY the implicated arcs: the set of ambiguity indices whose DD
  carrier factor touches a candidate satellite (as target or reference).
- Preserve every DDPR factor; do not suppress, reweight, or remove pseudorange
  rows, and do not touch clean carrier arcs.
- Do not enter CP-hold, do not mass-reset, do not relax LAMBDA, do not change
  FIX/FLOAT labeling rules.
- Apply at most one restart per candidate epoch per ambiguity index.

## Rollback gate

The live estimator may not consult reference truth. Instead:

- The mechanism keeps a shadow/off control: the same run profile is executed
  with the flag off and on; all pre-existing solution fields must be identical
  whenever no candidate fires.
- Any epoch that was FIXED in the OFF run and becomes FLOAT (or worse) in the
  ON run, or any new wrong FIX, or any NONE/non-finite increase, stops the
  experiment without threshold tuning.
- A candidate restart is only applied if the epoch is not currently FIXED with
  a strong ratio; if the epoch is clean-FIX, no restart fires.

## Frozen gates

### Gate 0: deterministic safety

- default-off bit-identical (OFF/ON solution, ratio, status, CSV equal);
- a synthetic 30 m single-DDPR jump with zero Doppler-predicted change arms the
  implicated pair's satellites at epoch i and bumps ONLY their ambiguity
  generations at i+1;
- clean carrier arcs and DDPR factors keep identical generations/factors;
- no held epoch becomes FIXED from a restarted arc.

### Gate 1: Tokyo run1 source epochs 5000--5499

- zero additional wrong FIX above 0.5 m 3-D;
- zero lost correct FIX relative to the OFF replay;
- at least one additional correct FIX opportunity over the 167-FLOAT baseline
  (or a clear demonstration that the candidate epoch itself was already
  unrecoverable);
- no NONE/non-finite increase;
- fixed horizontal RMS/P95 regression at most 5%;
- solver overhead at most 10%.

Failure stops without tuning. Only a Gate-1 pass permits full run1. Tokyo
run2/run3 remain sealed until a full-run1 activation passes.

## CLI

```text
--selective-arc-restart
```

plus the existing `--dump-csv result.csv` path, which gains
`selective_arc_restart_*` epoch columns and a
`result.csv.selective_arc_restart.csv` trace (epoch, satellite, role
target/reference, ambiguity index, epoch-median, post-fit residual, Doppler
innovation, IMU innovation, action applied).

## Gate 0 result

Three deterministic tests pass (all default-off bit-identical where no
candidate fires):

- `OffOnSolutionsAreBitIdenticalWhenNoQuarantineCandidateFires`: clean
  fixed-lag problem, OFF/ON positions, status, ratio, fixed count, and
  generation bumps exactly equal; zero detection epochs and zero applied arcs.
- `SyntheticGrossJumpArmsImplicatedPairAndRestartsOnlyItsArc`: a 30 m
  single-DDPR jump with zero Doppler-predicted change arms the implicated
  pair's arcs at epoch i (Doppler 5.93-sigma, IMU 6.09-sigma) and bumps ONLY
  their ambiguity generations at i+1; the clean arcs keep identical
  generations; the OFF/ON estimator positions, ratios, and statuses remain
  exactly equal.
- `MonitorOnlyReportsCandidatesWithoutBumpingAnyGeneration`:
  `--selective-arc-restart-monitor` reports the same detection with
  `applied_arcs == 0` and zero generation bumps.

No held epoch becomes FIXED from a restarted arc, and every pre-existing
solution field is equal whenever no candidate fires.

## Gate 1: Tokyo run1 source epochs 5000--5499

The frozen shipping profile (5 s lag, multi-frequency partial AR, hold, CMC,
CP-hold, exception recovery, DDPR anchor, FDE, varerr, FIX demotion) was run
with the same binary OFF and ON:

| Metric | OFF | ON |
|---|---:|---:|
| FIX / FLOAT / NONE | 333 / 167 / 0 | 333 / 167 / 0 |
| FLOAT horizontal RMS | 7.87162 m | 7.87162 m |
| FIXED horizontal RMS | 0.024057 m | 0.024057 m |
| fix-rate | 66.6% | 66.6% |
| estimator rows with status/pos/ratio diff | - | 0 / 500 |
| detection epochs | - | 1 |
| candidate satellites / pairs | - | 2 / 1 |
| applied arc restarts | - | 0 |
| skipped_fixed | - | 329 |
| wall time | 61.59 s | 61.10 s |

The one detection is the witness's own BDS C11/C14 pair at slice epoch 461
(TOW 188563, FLOAT, CP-hold active, residual 106.08 m, epoch median 1.878 m,
Doppler 5.93-sigma, IMU 6.09-sigma). No DD carrier arc in that epoch's graph
touches either satellite (the live carrier factors are GPS/GAL/QZSS-only:
G30->G11, E11->E09, E36->E09, J03->J04), so the candidate is detected but not
armable: `applied_arcs == 0` and the estimator is bit-identical to OFF.

Gate-1 verdict: the mechanism is safe (zero wrong-FIX, zero lost correct-FIX,
zero NONE/non-finite increase, ~0.8% overhead) but produces **zero incremental
correct-FIX opportunities** on this slice. The single witness candidate epoch
has no carrier arc to restart, so the experiment cannot demonstrate the
required "+1 correct FIX" on the development slice. Do not spend sealed run2
/run3 holdouts on this threshold set. The active selective arc-restart
remains a safe, default-off, bit-identical mechanism whose utility is bounded
by the quarantine witness's own low recall (see the witness plan). A future
active experiment needs a candidate rule that also fires on epochs whose
implicated satellites actually carry a DD carrier arc, or an additional
temporal-consistency witness with higher recall.

## Post-hoc slice analysis (design-slice candidates)

A separate read of the OFF replay's epoch CSV was made to find FLOAT epochs
that DO carry carrier arcs yet still fail to fix, and to test whether an
arc-restart rule could plausibly recover them:

| Category | Count |
|---|---:|
| FLOAT epochs with carrier factors added | 47 / 167 |
| FLOAT epochs with ddpr_rms > 5 m | 90 / 167 |
| FLOAT, carrier added, LAMBDA-insufficient (ar_outcome=5) | 45 |
| ... of those with ddpr_rms > 5 m | 8 (slice epochs 360--367) |

The 45 LAMBDA-insufficient epochs all had `lambda_attempts == 0`: their
eligible ambiguity count (3--5) never reached the per-epoch LAMBDA floor, so
AR was not even attempted. The candidate trace shows the cause: those epochs
are dominated by **build-time-excluded** carrier rows (E10->E09, J03->J04,
C13->C14 are excluded at almost every epoch; G20->G11 drops out from
TOW 188538.2 onward; E36->E09 flickers in and out). Excluded rows have no
ambiguity index in the graph, so an ambiguity-generation restart cannot
restore them -- there is no arc to restart. This is exactly the
`use_low_count_ambiguity_resolution` (default-off) responsibility: lower the
candidate-count floor and let LAMBDA run on the surviving candidates, gated
by surplus validation and a ratio floor.

Conclusion for the next experiment: **arc-restart and low-count AR address
disjoint failure modes.** An arc-restart that fires on the quarantine rule
cannot help the design-slice FLOATs because (a) the witness fires on
CP-hold-only epochs whose implicated satellites have no carrier arcs, and (b)
the carrier-bearing FLOAT epochs fail for build-time exclusions, not stale
arcs. A more promising next experiment targets the 45 low-count FLOAT epochs
directly: evaluate `use_low_count_ambiguity_resolution` on the frozen
design-slice profile (it is currently off), with the existing surplus +
ratio gates as the safety net, and count correct-FIX recoveries. That is a
threshold-free, already-implemented path that does not need a new candidate
rule.

## Low-count AR evaluation (design slice)

`--low-count-ar` was run on the frozen design-slice profile (default
min_candidates=4, then 3, and ratio 0) and produced **zero accepted fixes** in
every configuration:

| Configuration | attempts | accepted |
|---|---:|---:|
| min_candidates=4, min_ratio=1.5 | 20 | 0 |
| min_candidates=3, min_ratio=1.5 | 48 | 0 |
| min_candidates=3, min_ratio=0 (surplus alone) | 48 | 0 |

With min_candidates=3 the 48 epochs that were previously LAMBDA-insufficient
each ran one LAMBDA search (`lambda_attempts` 0 -> 1 across slice epochs
330--443, matching the 45 ar_outcome=5 epochs plus a few neighbours), yet
`surplus-validation` rejected every candidate and no epoch changed status
(333 FIX / 167 FLOAT identical to OFF).

This closes the "candidate-count floor" hypothesis: the design-slice FLOATs
do not fail because LAMBDA never runs; they fail because their surviving
candidates do not pass the surplus/ratio acceptance gates. That is a
candidate-QUALITY problem, not a candidate-COUNT or arc-continuity problem.
Arc-restart cannot help (the arcs exist and are already eligible), and
low-count AR cannot help (the integer hypotheses are rejected by the gates).
Any future experiment that wants more correct FIX on this slice must improve
the quality of the surviving LAMBDA candidates (e.g. better float-ambiguity
conditioning or a different integer candidate), which is a distinct problem
from the quarantine/arc-restart thread and is out of scope for this plan.

