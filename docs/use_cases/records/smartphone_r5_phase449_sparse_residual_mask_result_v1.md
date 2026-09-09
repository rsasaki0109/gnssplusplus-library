# Phase449 — centered seed-residual screening removes sparse-epoch P

Phase448 completed once, exit 0, 218.81347793503664 s; manifest SHA
`66c1bba5d94e06fb62d35609393e4d74e57890137cda7f98d5d61c918f25d79f`.
Current source/test/algorithm and raw/binary pins, stage convergence, baseline
candidate SHA, bias/information aggregates and topology log checks passed.
No truth reads or additional accuracy evaluations.

| Epoch | Before centered P-residual screen | After | Removed |
|---|---:|---:|---:|
| 851 | 8 | 1 | 7 |
| 852 | 12 | 6 | 6 |

The final row counts agree with native nominal information diagnostics.
The residual is corrected P minus seed range minus seed receiver clock;
screening subtracts a whole-observation-matrix system/band median and applies
the unchanged L1 20 m / L5 15 m absolute threshold. Counts identify the stage,
not why those residuals are large or whether they reflect bad measurements.

Phase286 previously found no H aggregate improvement from residual-only
re-admission. Do not repeat threshold sweeps, bypass screening, or interpret
this localized result as the cause of LAX-T's route-wide error. Next examine
signed rejected residuals and independent raw P-D consistency to distinguish
common seed/clock displacement from satellite-specific observation errors.
Any new diagnostics must remain aggregate-only, with no saved position input.
