# Low-count AR surplus-quality relaxation plan

## Objective

`use_low_count_ambiguity_resolution` lowers the per-epoch LAMBDA candidate
floor so sparse epochs can attempt AR, but its mandatory
`surplus_rescue_quality_pass` requires `nsat >= 10` and `>= 4` post-fit DDCP
factors. A sparse epoch is sparse by construction, so those two floors can
never be met -- the rescue is structurally dead on exactly the epochs it
exists for.

The prior low-count evaluation (`docs/fgo_low_count_ar_recovery_plan.md`)
concluded this was unfixable because relaxing the gate "produced wrong FIX at
2.9 km". That conclusion was an artifact of scoring against `ref_e_pos_m`
(the ENU coordinate from the reference origin) instead of `horiz_err_m` (the
true horizontal error). Re-scoring with `horiz_err_m` shows the relaxation
produces **correct** fixes.

## Change (default OFF, bit-identical when off)

`FGOConfig::low_count_relax_surplus_quality` (+
`--low-count-relax-surplus-quality`) exempts a low-count attempt from the
`nsat >= 10` and `ddcp_factors >= 4` arms of `surplus_rescue_quality_pass`,
keeping the `ddpr_rms <= 5 m`, `DDCP RMS <= 0.05 m`, and
`fallback_level == 0` floors and the mandatory surplus pass. A low-count
epoch still needs `nsat >= 4` and `ddcp_factors >= 2` (the minimum to form a
meaningful candidate at all).

## Gate 1: Tokyo run1 source epochs 5000--5499

Frozen shipping profile (5 s lag, multi-frequency partial AR, hold, CMC,
CP-hold, exception recovery, DDPR anchor, FDE, varerr, FIX demotion) with
`--low-count-ar --low-count-min 3 --low-count-relax-surplus-quality`:

| Metric | OFF | ON |
|---|---:|---:|
| FIX / FLOAT | 333 / 167 | 335 / 165 |
| new correct FIX (3-D < 0.5 m) | - | 2 |
| wrong FIX > 0.5 m (whole slice) | 0 | 0 |
| FIXED horizontal RMS | 0.024057 m | 0.024075 m |
| FIXED horizontal max | 0.065650 m | 0.065650 m |
| FLOAT horizontal RMS | 7.87162 m | 7.91918 m |
| fix-rate | 66.6% | 67.0% |
| wall time | 61.59 s | 69.03 s (+12.1%) |

The two recovered epochs (slice 335, 336) had a correct LAMBDA candidate
(ratio in the millions, FFRT pass, candidate within 0.01 m of the float
position, float already at 0.05 m horizontal) but were structurally rejected
by `nsat >= 10` / `ddcp_factors >= 4`. They now fix at 0.021 m and 0.032 m.
The remaining low-count FLOATs (334: candidate count below the floor;
337--340: surplus evaluation unavailable) are not affected.

Gate-1 verdict: **passes** the zero-wrong-FIX and zero-RMS-regression bars;
delivers +2 correct FIX. The +12.1% wall overhead comes from the low-count AR
attempts (48 in the slice), not the relaxation itself. The formal
activation bar for the combined low-count path (wall <= +10%) is not met on
this slice, so the pair is reported but not made a default preset.

## Gate 0

- default-off bit-identical (all 915 tests green; when the knob is off the
  floors are unchanged);
- a synthetic 3-ambiguity epoch with a clean DDCP and a correct candidate is
  accepted when the knob is on and rejected (LowCountRejected) when off;
- a contaminated DDCP (RMS > 0.05 m) or a ratio < floor is still rejected.

## Conclusion

The prior "falsified" conclusion in `fgo_low_count_ar_recovery_plan.md` is
corrected: the relaxation recovers 2 correct fixes with zero wrong FIX and no
RMS regression on the design slice. The change is a safe, default-off,
bit-identical option. Sealed run2/run3 remain available for a full run1
activation pass if the user wants to pursue the +12.1% overhead (e.g. by
making low-count attempts cheaper or restricting them to genuine
candidate-floor epochs).
