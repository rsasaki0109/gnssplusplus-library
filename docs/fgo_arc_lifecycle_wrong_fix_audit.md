# FGO ambiguity lifecycle wrong-FIX audit

## Objective

Determine whether wrong FGO FIX events begin soon after a causal carrier-arc
discontinuity.  The audit does not add another ambiguity acceptance gate and
does not tune TDCP.  It joins telemetry that the estimator already produces:
receiver clock-jump detection, reference-satellite changes, ambiguity-index
churn, short arc age, geometry-free and Doppler slip shadows, generation
bumps, and carrier FDE exclusions.

Reference truth is used only offline to label a reported FIX as correct or
wrong.  No truth label, score, or lifecycle verdict is available to the
runtime estimator.

## Why the scope changed

The repository already contains completed clock-resilient temporal-carrier,
geometry-free slip, Doppler slip, predicted-DDPR, causal DDPR-bias, and
multi-epoch ambiguity audits.  Their documented holdout results reject direct
TDCP insertion, direct DDPR downweighting, and same-state ambiguity consensus
as safe FIX-rate improvements.  Reimplementing any of those monitors would
repeat a closed experiment.

The missing artifact is an event-level attribution report.  Existing epoch
CSV output contains most reset and slip counters, while its normalized
`.ar_candidates.csv` sidecar identifies the eligible ambiguity index and DD
reference for each satellite/signal row.  The builder clock data was not
exported; the epoch CSV now appends the existing positive-only boolean, the
signed common GPS pseudorange change, and its matched-satellite support
without changing any estimator state.  The signed value prevents a negative
receiver-clock jump from disappearing from the offline audit.

## Research and OSS basis

- Momoh, Bhattarai, and Ziebart separate common receiver clock jumps from
  individual cycle slips before reconstructing observations:
  <https://doi.org/10.1007/s10291-019-0832-4>.
- Guo and Zhang classify clock jumps and reconstruct the affected observable
  rather than interpreting every temporal phase discontinuity as a slip:
  <https://doi.org/10.1007/s10291-012-0307-3>.
- RTKLIB uses LLI and geometry-free phase jumps to reset carrier bias states;
  its Doppler slip detector is disabled explicitly because of clock-jump
  sensitivity:
  <https://github.com/tomojitakasu/RTKLIB/blob/master/src/rtkpos.c>.
- EPIC's position-domain integrity work treats satellite geometry changes,
  cycle slips, and safely re-fixing reacquired satellites as explicit
  implementation concerns:
  <https://www.ion.org/publications/abstract.cfm?articleID=102542>.

These sources support lifecycle separation and fail-closed reacquisition.
They do not establish that a clock jump or reference change alone proves an
incorrect integer solution, so this stage remains an offline association
audit.

## Causal features

For each truth-matched epoch, `scripts/analyze_fgo_arc_lifecycle.py` derives:

- `clock_jump`: absolute signed common GPS pseudorange change above 100 km
  with at least one matched satellite (falling back to the legacy boolean for
  older CSVs);
- `reference_change`: a changed `(constellation, signal) -> reference` map
  among LAMBDA-eligible rows;
- `high_ambiguity_churn`: Jaccard churn of eligible ambiguity indices at or
  above 25%;
- `young_arc`: at least one eligible ambiguity index is at most two epochs
  old;
- `geometry_free_slip` and `doppler_slip`: existing shadow events;
- `generation_bump`: any hold, FDE, reset, warm-reset, or stale-pin generation
  bump;
- `fde_exclusion`: at least one ambiguity removed by carrier FDE; and
- `any_lifecycle_discontinuity`: the union of the preceding events.

Ambiguity age is based on the first prior appearance of an ambiguity index.
Reference maps and active indices use only disposition `LambdaEligible` from
the candidate sidecar.  An event can explain an onset only when it occurs in
the onset epoch or one of the preceding five epochs; future epochs are never
examined.

## Frozen development gate

Tokyo run1 is the only development sequence for this audit.  A lifecycle
category may justify a later runtime experiment only if, at the fixed
five-epoch causal lookback, it meets all of the following:

The lookback is bounded by time as well as row count: an observation gap over
0.25 s starts a new causal segment.  Wrong-FIX onset detection and all
reference, ambiguity-churn, arc-age, and event-exposure history are reset at
that boundary, so missing epochs cannot create a false temporal association.

- at least 20 wrong-FIX event onsets and 100 correct-FIX control epochs exist;
- at least 20% of wrong-FIX onsets are exposed;
- no more than 5% of correct-FIX epochs are exposed; and
- onset coverage is at least four times correct-FIX exposure.

The scorer also reports zero-, one-, and three-epoch windows, but those are
diagnostic and cannot replace the frozen five-epoch verdict.  If no individual
category passes, do not add a lifecycle veto/reset and do not inspect run2 or
run3 for threshold selection.

If a category passes, the next change must still be default-off and bounded.
It must first demonstrate exact non-interference in monitor mode, then zero
additional wrong FIX and zero lost correct FIX on a representative run1
slice.  Only a frozen implementation may advance to run2 and sealed run3.

## Usage

Run a normal parity replay with `--ref` and `--dump-csv`, then score it:

```text
python scripts/analyze_fgo_arc_lifecycle.py \
  --epoch-csv <run.csv> \
  --json <audit.json> \
  --markdown <audit.md>
```

The candidate path defaults to `<run.csv>.ar_candidates.csv`.  The process
returns success only when at least one event meets the frozen gate.  JSON uses
strict finite values; an unbounded relative risk caused by zero correct-FIX
exposure is represented as `null` while still satisfying the risk component
of the gate.

## Tokyo run1 result

The shipping preset produced 11,905 truth-matched epochs: 4,490 correct FIX
epochs, 1,919 wrong FIX epochs, and 133 causal wrong-FIX event onsets.  A
50-epoch before/after CSV comparison matched exactly in every pre-existing
column.  The full run also matched the prior output in time, status, ECEF,
ratio, fixed ambiguity count, and AR outcome for all 11,905 rows.

At the frozen five-epoch lookback:

| Event | Wrong onset coverage | Correct FIX exposure | Relative risk | Gate |
|---|---:|---:|---:|---:|
| Clock jump | 0 / 133 = 0.000% | 6 / 4,490 = 0.134% | 0.000 | fail |
| Reference change | 73 / 133 = 54.887% | 883 / 4,490 = 19.666% | 2.791 | fail |
| Ambiguity churn >=25% | 47 / 133 = 35.338% | 374 / 4,490 = 8.330% | 4.242 | fail |
| Young arc | 108 / 133 = 81.203% | 2,604 / 4,490 = 57.996% | 1.400 | fail |
| Geometry-free slip | 25 / 133 = 18.797% | 341 / 4,490 = 7.595% | 2.475 | fail |
| Doppler slip | 28 / 133 = 21.053% | 386 / 4,490 = 8.597% | 2.449 | fail |
| Generation bump | 22 / 133 = 16.541% | 207 / 4,490 = 4.610% | 3.588 | fail |
| FDE exclusion | 18 / 133 = 13.534% | 196 / 4,490 = 4.365% | 3.100 | fail |
| Any lifecycle event | 124 / 133 = 93.233% | 2,964 / 4,490 = 66.013% | 1.412 | fail |

Only one signed common clock jump occurred in the complete run: a positive
299,715.527 m event supported by four satellites at TOW 188500.200.  No
negative event exceeded 100 km, and the single positive event preceded no
wrong-FIX onset.  Reference changes and young arcs have high coverage because
they are also routine during correct operation.  Ambiguity churn is the most
discriminative category, but its 8.441% correct-FIX exposure exceeds the
precommitted 5% maximum.  Generation bumps and FDE exclusions meet the
correct-exposure limit but miss both the 20% onset-coverage and four-times
risk requirements.

Decision: retain the exported clock flag and offline event ledger, but do not
add a clock-jump, reference-change, young-arc, churn, slip, generation-bump,
or FDE-based FIX veto/reset.  Run2 and run3 are not inspected for this audit.
The remaining wrong-FIX population is not primarily initiated by a rare
ambiguity lifecycle discontinuity; the next improvement should target a
persistent observation-model error with a genuinely external witness rather
than another temporal acceptance rule.
