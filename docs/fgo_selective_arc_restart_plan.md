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

