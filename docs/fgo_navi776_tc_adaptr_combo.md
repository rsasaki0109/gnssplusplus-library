# `--navi776-tc` + adaptive-R extension A/B (Tokyo run1 full)

## Objective

PR #438 added two default-off adaptive-R variants
(`--rtk-adaptive-noise-phase-only`, `--rtk-adaptive-noise-per-system-alpha`).
The Phase 3 TC main line ships `--navi776-tc` (M3 closed loop + M4 velocity +
SD Doppler rows + gated shared-alpha adaptive noise). This A/B checks whether
combining the new knobs with the TC preset helps or hurts, before any
preset-promotion decision.

## Method

Tokyo run1 full (11845 scored epochs), clang-ninja gnss_fuse,
`--preset low-cost --ratio 2.4 --max-subset-ar-drop-steps 18
--rtk-snr-weighting --no-arfilter --max-epochs 0`. Scored against
`reference.csv` ECEF.

## Result

| Config | Fix % | rmsH | maxH | <50cm |
|---|---:|---:|---:|---:|
| OFF (basic only) | 76.70% | 16.09 m | 215.5 m | 8644 |
| `--navi776-tc` | **79.77%** | **14.54 m** | 215.5 m | **9337** |
| `--navi776-tc` + phase-only | 55.06% | 17.25 m | 215.5 m | 6151 |
| `--navi776-tc` + per-system | 77.90% | 15.01 m | 215.5 m | 8772 |

The `--navi776-tc` shared-alpha preset is reproduced (79.77% fix, matching
the sign-off table). Adding **phase-only** collapses the fix rate to 55.06%
and <50 cm to 6151: the code-variance adaptation is essential to the TC
preset's fix-rate gain, so removing it is strongly negative. Adding
**per-system** alpha also regresses (77.90% fix, 8772 <50 cm): the shared
phase 0.9 / code 0.5 constants are better on this dataset than the
per-system table.

## Conclusion

The Phase 3 TC main line should keep the shared-alpha adaptive noise inside
`--navi776-tc`. The two new adaptive-R knobs are **not** folded into the TC
preset; combining them is measurably worse. Both knobs remain default-OFF and
independent; no preset-promotion, sealed run2/run3 untouched. This closes the
"reflect adaptive-R into the Phase 3 TC main line" backlog item with a
negative result.
