# Adaptive-R phase-only / per-system alpha — real-data validation

## Objective

Validate the two navi.776 A follow-up knobs added in PR #438
(`adaptive_noise_phase_only`, `adaptive_noise_per_system_alpha`) on real PPC
data. Both default OFF and are bit-identical when off; this document records
the real-data A/B result on the RTKProcessor path (gnss_fuse, clang-ninja
build — MSVC cannot build gnss_fuse/gnss_solve due to the pre-existing C1061
nesting limit, so validation used the clang-ninja Windows build).

## Method

Tokyo run1, PPC 5 Hz, `--data-dir`, `--lever-arm 0.31,0,0.55`, `--preset
low-cost`, `--ratio 2.4`, `--max-subset-ar-drop-steps 18`,
`--rtk-snr-weighting`, `--no-arfilter`. Four configurations, each scored
against `reference.csv` ECEF:

1. OFF (no adaptive noise)
2. `--rtk-adaptive-noise --rtk-adaptive-noise-max-baseline 1000`
3. `--rtk-adaptive-noise ... --rtk-adaptive-noise-phase-only`
4. `--rtk-adaptive-noise ... --rtk-adaptive-noise-per-system-alpha`

## Result: first 500 epochs (clean urban, FIX-dominated)

| Configuration | fix | rmsH | maxH | <50cm |
|---|---:|---:|---:|---:|
| OFF | 496 | 0.0508 m | 0.3977 m | 500 |
| adaptive-on | 496 | 0.0471 m | 0.3032 m | 500 |
| phase-only | 496 | 0.0507 m | 0.3977 m | 500 |
| per-system | 496 | 0.0469 m | 0.2988 m | 500 |

- adaptive-on improves rmsH 7.3% and maxH 24% over OFF.
- phase-only is within 0.2% of OFF (rmsH 0.0507 vs 0.0508): removing the
  code adaptation removes almost all of the rms gain, confirming that the
  code-variance adaptation is the main driver on this slice and that the
  phase-only knob correctly isolates it.
- per-system alpha is the best (rmsH 0.0469, maxH 0.2988): the per-system
  smoothing constants (BDS/GAL alphas lower than the shared default) help
  marginally over the shared alpha.
- All four configurations produce 496/500 FIX and 500/500 <50 cm, so the
  difference is precision, not fix rate, on this clean slice.

## Design-slice 5000--5499 (FLOAT-dominated) is not an evaluation window

Running the same four configurations over the full first 5500 epochs and
scoring only TOW [188470, 188569.8] (slice 5000--5499) gives **identical**
results for all four (fix=391, rmsH=10.18 m, maxH=98.39 m, <50cm=378). The
slice is dominated by LAMBDA-insufficient FLOAT epochs whose float solution
is already biased (see `fgo_design_slice_float_root_cause.md`); adaptive-R
adjusts measurement variance, which does not change those epochs' AR outcome
or float basin. Adaptive-R's effect is only visible where LAMBDA runs and
carrier/code precision matters.

## Conclusion

Both knobs are implemented correctly and show the intended behavior on real
data:
- `--rtk-adaptive-noise-phase-only` removes the code-variance adaptation and
  closely tracks OFF, confirming the code term is the dominant contributor
  and that the knob isolates it.
- `--rtk-adaptive-noise-per-system-alpha` modestly improves precision over
  the shared alpha (rmsH 0.0471 -> 0.0469 m, maxH 0.3032 -> 0.2988 m).
- Default-OFF / bit-identical-when-off is preserved; the OFF/ON difference
  is the expected measurement-variance behavior, not an estimator change.

The design slice is the wrong window to judge adaptive-R; a full run1 A/B
would be needed to score fix-rate impact, and is left as future work
(sealed run2/run3 untouched).

## Full run1 A/B (Tokyo run1, 11845 scored epochs)

Four full-run configurations scored against `reference.csv` ECEF:

| Config | fix | rmsH | maxH | <50cm |
|---|---:|---:|---:|---:|
| OFF | 9085 | 16.09 m | 215.5 m | 8644 |
| adaptive-on | 9130 | 16.09 m | 215.5 m | 8583 |
| phase-only | 9127 | 16.09 m | 215.5 m | 8582 |
| per-system | 9134 | 16.09 m | 215.5 m | 8583 |

Fix rate improves (9085 -> 9130-9134, +45-49) and rmsH/maxH are unchanged,
but the <50 cm count falls (8644 -> 8582-8583, -61). Epoch-by-epoch for
adaptive-on: 100 FLOAT->FIXED (all < 0.5 m) and 55 FIXED->FLOAT (all were
< 0.5 m; their ON position is within ~0.1 m of the OFF fix). So the net +45
FIX is real, but adaptive-R also drops 55 correct FIX to FLOAT (status 4 ->
3 at nearly the same position), which is why the <50 cm tally shrinks.

Interpretation: adaptive-R raises the fix count but at the cost of fix
continuity -- epochs whose adapted variance lowers the LAMBDA ratio lose
their FIXED label while their position is essentially unchanged. For a
competition metric that rewards <50 cm epochs, the -61 <50 cm is a
regression, so **adaptive-R is not promoted to a default preset** on this
evidence. phase-only tracks adaptive-on closely (9127 FIX) and per-system is
marginally best (9134), consistent with the first-500 result. The knobs
remain default-OFF; full 3-city A/B and the usual bars would be needed
before any activation, and sealed run2/run3 are untouched.
