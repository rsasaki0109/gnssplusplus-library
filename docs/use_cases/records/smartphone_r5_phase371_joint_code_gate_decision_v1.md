# Phase370/371 — mixed H/U code-gate results

Both evaluation manifests were frozen in `daba7370` before either score.
Each evaluator verified its metadata and read its candidate/truth once.
No solver rerun, interpolation, edge hold, offset reapplication or submission.

| Reused development route | Baseline m | Gate OFF m | Delta m |
| --- | ---: | ---: | ---: |
| H | 1.0769392017393964 | 1.0816901550069105 | +0.004750953267514069 |
| U | 1.305940315797287 | 1.2749917901947554 | -0.030948525602531696 |

All 3139 H and 1102 U keys matched. Both solver stages converged on both
routes. Added TDCP factors: H 2229, U 617. The rule changed both GNSS-first
and main admission; downstream initialization can therefore change too.

Do not promote globally on these mixed, repeatedly reused development
results. Do not route-switch the gate based on which scored better. U's
improvement is evidence of route-dependent benefit, not heldout success or
proof that every restored phase pair is good. Neither route meets 0.782 m.

Keep this fixed alternative for broader route-grouped development testing;
avoid a threshold sweep or H-specific reversal. The next useful comparison
requires another documented development route with raw-only inference and
its own baseline, preserving the same rule before either score. Existing
truth-access history must be audited before labeling any route heldout.
Operational defaults remain unchanged and the full goal remains unmet.
