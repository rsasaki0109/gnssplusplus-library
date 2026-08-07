# Low-count AR recovery plan (nsat-quality-gate rebalance)

## Objective

`use_low_count_ambiguity_resolution` (default-off) lowers the per-epoch
LAMBDA candidate-count floor so sparse-geometry epochs can attempt AR, but its
mandatory `surplus_rescue_quality_pass` requires `nsat >= 10` and
`ddpr_rms <= 5 m`. A low-count epoch is sparse by construction, so those two
floors can never be met -- the rescue is structurally dead on exactly the
epochs it exists for.

On the frozen Tokyo run1 design slice (source epochs 5000--5499), 8 FLOAT
epochs (slice 360--367) carry a correct, unambiguous LAMBDA candidate yet
remain FLOAT purely because of these floors:

| slice epoch | nsat | float horiz. err | ddpr_rms | DDCP post-fit RMS | LAMBDA ratio | FFRT |
|---|---:|---:|---:|---:|---:|---|
| 360 | 7 | 0.094 m | 9.58 m | 0.018 m | 4.3 | 1 |
| 361 | 7 | 0.094 m | 9.53 m | 0.018 m | 4.1 | 1 |
| 362 | 6 | 0.093 m | 10.57 m | 0.013 m | 5.8 | 1 |
| 363 | 6 | 0.092 m | 10.55 m | 0.012 m | 5.1 | 1 |
| 364 | 6 | 0.090 m | 10.55 m | 0.011 m | 4.6 | 1 |
| 365 | 7 | 0.088 m | 10.56 m | 0.012 m | 31.9 | 1 |
| 366 | 7 | 0.085 m | 10.57 m | 0.014 m | 35.3 | 1 |
| 367 | 5 | 0.084 m | 12.23 m | 0.015 m | 73.9 | 1 |

These epochs sit inside the 0.5 m 3-D aperture (the float solution is already
sub-10-cm horizontal) yet are labelled FLOAT. The candidate carrier
observation is clean (DDCP post-fit ~1--2 cm), so the large `ddpr_rms` is a
code-observation (multipath) signature, not an integer problem. This is
exactly the regime where a correct FIX is being lost to a quality gate.

## Diagnostic result (falsified the proposed relaxation)

The relaxation was tested before any implementation was written. With
`kSurplusRescueMinSatellites` lowered to 1 and `kSurplusRescueMaxDdprRmsM`
raised to 100 m, `--low-count-ar --low-count-min 3 --surplus-validation-min-n
1` on the frozen design slice turned 6 FLOAT epochs FIXED:

| slice epoch | FIXED horiz. err |
|---|---:|
| 356 | 2909.78 m |
| 360 | 2909.78 m |
| 361 | 2909.78 m |
| 362 | 2909.78 m |
| 363 | 2909.78 m |
| 364 | 2909.78 m |

All six are **wrong FIX at ~2.9 km**; none is within the 0.5 m aperture. The
float solutions these epochs reported were indeed ~0.09 m horizontal (the
`horiz_err_m` column), but the integer candidates LAMBDA selected at ratio
4--74 and FFRT-pass were not the true integers -- FFRT and the ratio validate
internal consistency of the integer hypothesis against the float covariance,
not its agreement with truth. The `nsat >= 10` and `ddpr_rms <= 5 m` floors
were therefore blocking exactly the wrong fixes they exist to block: a sparse
epoch with a large code residual and a self-consistent-but-wrong integer
candidate.

**Conclusion: the low-count AR recovery plan as designed does not work.** The
8 target epochs' correct-looking float solutions are not accompanied by
correct integer candidates, and no amount of ratio/DDCP evidence (already
clean at 1--2 cm) can distinguish the wrong integers without the
independent-satellite coverage the `nsat` floor encodes. Do not implement the
sparse-geometry witness, do not relax the quality gate, and do not spend
sealed run2/run3 holdouts on this direction. A correct recovery for these
epochs would require better float-ambiguity conditioning (a distinct problem
from the quarantine/arc-restart thread) or an independent absolute-position
witness, which the candidate-integrity witness already showed has zero yield
on this slice.

## (Retained record) Frozen candidate-quality witness proposal

**Superseded by the diagnostic result above; kept for the audit trail.**

For a low-count epoch ONLY (`n < min_candidates`, same entry rule as today),
replace the `nsat >= 10` and `ddpr_rms <= 5 m` arms of
`surplus_rescue_quality_pass` with a sparse-geometry substitute that uses
evidence which does not vanish as the satellite count shrinks:

1. **Carrier post-fit consistency** (kept): `fixed_postfit_ddcp_factors >= 4`
   and `fixed_postfit_ddcp_rms_m <= 0.05 m` -- already evaluated and passed
   (1--2 cm) on all 8 target epochs.
2. **IMU-propagated aperture** (kept from `imu_aided`): the candidate lies
   within `imu_aided_max_prediction_separation_m` of the IMU-propagated epoch
   seed, and within `imu_aided_max_float_separation_m` of the float position.
3. **New ratio floor for sparse geometry**: `ratio >= 10` (between today's
   `low_count_min_ratio` default of 1.5 and the strict 3.0) to compensate for
   losing the `nsat >= 10` independent-satellite coverage.

Everything else is unchanged: the epoch still enters via the existing
low-count entry gate, still requires `use_surplus_satellite_validation`, still
goes through surplus validation when the pool allows, and still must pass the
existing fixed-vs-float separation and plausibility gates. The change is
strictly narrower -- it relaxes two floors that cannot be satisfied by a
sparse epoch and adds one sparse-geometry ratio floor.

## Gates

### Gate 0: deterministic safety

- default-off bit-identical (OFF/ON solutions, ratios, statuses, CSV equal);
- a synthetic 4-ambiguity epoch with a clean carrier post-fit and a ratio of
  ~50 is accepted (previously LowCountRejected), while:
- the same epoch with a contaminated DDCP (RMS > 0.05 m) or ratio < 10 is
  still rejected; and
- a normal (n >= min_candidates) epoch is never routed through the sparse
  witness (established path untouched).

### Gate 1: Tokyo run1 source epochs 5000--5499

- the 8 target epochs (360--367) become FIXED at their ~0.09 m positions;
- zero additional wrong FIX above the 0.5 m 3-D aperture anywhere in the
  slice;
- zero lost correct FIX relative to the OFF replay;
- no NONE/non-finite increase;
- fixed horizontal RMS/P95 regression at most 5%;
- solver overhead at most 10%.

Failure stops without tuning. Only a Gate-1 pass permits full run1. Tokyo
run2/run3 remain sealed until a full-run1 activation passes.

## CLI

Expose one research switch alongside the existing `--low-count-ar`:

```text
--low-count-sparse-witness
```

No sigma sweep, ratio sweep, or aperture tuning is allowed in this
experiment. Reference truth is used only by the offline scorer.
