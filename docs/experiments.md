# PPP-AR Experiments

Generated: 2026-04-02 12:05 UTC

This page is the current externalized state of the PPP-AR experiment lane.
All rows below use the same strategy catalog, the same metrics, and comparable CLAS inputs.

## Shared Interface

- Suite: `CLAS PPP phase continuity split suite`
- Cases: `2`
- Primary mode: `static`
- Shared strategy catalog: `experiments/ppp_ar/strategies.toml`
- Shared output schema: `wall_time_s`, `ppp_solution_rate_pct`, `ppp_fixed_solutions`, `fallback_solutions`, `clas_hybrid_fallback_epochs`, `mean_3d_error_m`, `median_3d_error_m`, `p95_3d_error_m`, `max_3d_error_m`, `readability_score`, `extensibility_score`

## Suite Summary

| Strategy | Cases passed | Mean solution rate | Mean 3D (m) | P95 3D (m) | Mean delta vs baseline (m) | Fixed total | Hybrid fallback total | Promotion |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| IFLC Float Baseline | 0/2 | 99.58 | 11.957 | 9.930 | 0.000 | 0 | 0 | stable_control |
| OSR Float Strict Pipeline | 0/2 | 50.00 | 4.898 | 5.134 | 7.059 | 0 | 0 | exploratory |
| OSR Float SIS-Continuity-Only Pipeline | 0/2 | 50.00 | 4.898 | 5.134 | 7.059 | 0 | 0 | exploratory |
| OSR Float Repair-Only Pipeline | 0/2 | 50.00 | 4.898 | 5.134 | 7.059 | 0 | 0 | exploratory |
| OSR Float Raw-Phase-Bias Pipeline | 0/2 | 50.00 | 4.898 | 5.134 | 7.059 | 0 | 0 | exploratory |
| OSR Float No-Phase-Bias Pipeline | 0/2 | 50.00 | 4.897 | 5.144 | 7.060 | 0 | 0 | exploratory |
| OSR AR Strict Pipeline | 0/2 | 50.00 | 4.898 | 5.134 | 7.059 | 0 | 0 | exploratory |
| OSR AR SIS-Continuity-Only Pipeline | 0/2 | 50.00 | 4.898 | 5.134 | 7.059 | 0 | 0 | exploratory |
| OSR AR Repair-Only Pipeline | 0/2 | 50.00 | 4.898 | 5.134 | 7.059 | 0 | 0 | exploratory |
| OSR AR Raw-Phase-Bias Pipeline | 0/2 | 50.00 | 4.898 | 5.134 | 7.059 | 0 | 0 | exploratory |
| OSR AR No-Phase-Bias Pipeline | 0/2 | 50.00 | 4.897 | 5.144 | 7.060 | 0 | 0 | exploratory |

## Case Results

### CLAS 2019-08-27 static PPP

- Case key: `clas_static_2019_239`
- Observation file: `0627239Q.obs`
- Navigation file: `sept_2019239.nav`
- QZSS L6 source: `2019239Q.l6`
- Max epochs: `120`
- Mode: `static`

#### Strategy Comparison

| Strategy | Style | Solution rate | Fixed | Fallback | Hybrid fallback | Mean 3D (m) | P95 3D (m) | Wall (s) | Readability | Extensibility | Promotion |
|---|---|---:|---:|---:|---:|---:|---:|---:|---|---|---|
| IFLC Float Baseline | monolithic | 99.17 | 0 | 1 | 0 | 4.540 | 4.638 | 5.70 | low | low | stable_control |
| OSR Float Strict Pipeline | pipeline+boundary | 100.00 | 0 | 0 | 0 | 4.921 | 5.193 | 4.96 | low | low | exploratory |
| OSR Float SIS-Continuity-Only Pipeline | pipeline+continuity | 100.00 | 0 | 0 | 0 | 4.921 | 5.193 | 4.95 | low | low | exploratory |
| OSR Float Repair-Only Pipeline | pipeline+continuity | 100.00 | 0 | 0 | 0 | 4.921 | 5.193 | 4.91 | low | low | exploratory |
| OSR Float Raw-Phase-Bias Pipeline | pipeline+continuity | 100.00 | 0 | 0 | 0 | 4.921 | 5.193 | 4.95 | low | low | exploratory |
| OSR Float No-Phase-Bias Pipeline | pipeline+continuity | 100.00 | 0 | 0 | 0 | 4.919 | 5.212 | 4.93 | low | low | exploratory |
| OSR AR Strict Pipeline | pipeline+solver+boundary | 100.00 | 0 | 0 | 0 | 4.921 | 5.193 | 5.05 | low | low | exploratory |
| OSR AR SIS-Continuity-Only Pipeline | pipeline+solver+continuity | 100.00 | 0 | 0 | 0 | 4.921 | 5.193 | 5.04 | low | low | exploratory |
| OSR AR Repair-Only Pipeline | pipeline+solver+continuity | 100.00 | 0 | 0 | 0 | 4.921 | 5.193 | 5.00 | low | low | exploratory |
| OSR AR Raw-Phase-Bias Pipeline | pipeline+solver+continuity | 100.00 | 0 | 0 | 0 | 4.921 | 5.193 | 5.03 | low | low | exploratory |
| OSR AR No-Phase-Bias Pipeline | pipeline+solver+continuity | 100.00 | 0 | 0 | 0 | 4.919 | 5.212 | 5.01 | low | low | exploratory |

#### Promotion Gate

| Strategy | Mean delta vs baseline (m) | P95 delta vs baseline (m) | Fixed delta | Solution-rate delta | Gate | Rationale |
|---|---:|---:|---:|---:|---|---|
| IFLC Float Baseline | 0.000 | 0.000 | 0 | 0.00 | hold | Stable control arm used to judge candidate strategies. |
| OSR Float Strict Pipeline | -0.381 | -0.555 | 0 | 0.83 | hold | Still exploratory because these gates are not met: mean_error_improved, p95_error_improved, readability_non_regression |
| OSR Float SIS-Continuity-Only Pipeline | -0.381 | -0.555 | 0 | 0.83 | hold | Still exploratory because these gates are not met: mean_error_improved, p95_error_improved, readability_non_regression |
| OSR Float Repair-Only Pipeline | -0.381 | -0.555 | 0 | 0.83 | hold | Still exploratory because these gates are not met: mean_error_improved, p95_error_improved, readability_non_regression |
| OSR Float Raw-Phase-Bias Pipeline | -0.381 | -0.555 | 0 | 0.83 | hold | Still exploratory because these gates are not met: mean_error_improved, p95_error_improved, readability_non_regression |
| OSR Float No-Phase-Bias Pipeline | -0.379 | -0.574 | 0 | 0.83 | hold | Still exploratory because these gates are not met: mean_error_improved, p95_error_improved, readability_non_regression |
| OSR AR Strict Pipeline | -0.381 | -0.555 | 0 | 0.83 | hold | Still exploratory because these gates are not met: mean_error_improved, p95_error_improved, readability_non_regression, fixed_epochs_improved |
| OSR AR SIS-Continuity-Only Pipeline | -0.381 | -0.555 | 0 | 0.83 | hold | Still exploratory because these gates are not met: mean_error_improved, p95_error_improved, readability_non_regression, fixed_epochs_improved |
| OSR AR Repair-Only Pipeline | -0.381 | -0.555 | 0 | 0.83 | hold | Still exploratory because these gates are not met: mean_error_improved, p95_error_improved, readability_non_regression, fixed_epochs_improved |
| OSR AR Raw-Phase-Bias Pipeline | -0.381 | -0.555 | 0 | 0.83 | hold | Still exploratory because these gates are not met: mean_error_improved, p95_error_improved, readability_non_regression, fixed_epochs_improved |
| OSR AR No-Phase-Bias Pipeline | -0.379 | -0.574 | 0 | 0.83 | hold | Still exploratory because these gates are not met: mean_error_improved, p95_error_improved, readability_non_regression, fixed_epochs_improved |

### CLAS 2018-11-24 static PPP

- Case key: `clas_static_2018_329`
- Observation file: `0161329A.obs`
- Navigation file: `tskc2018329.nav`
- QZSS L6 source: `2018328X_329A.l6`
- Max epochs: `120`
- Mode: `static`

#### Strategy Comparison

| Strategy | Style | Solution rate | Fixed | Fallback | Hybrid fallback | Mean 3D (m) | P95 3D (m) | Wall (s) | Readability | Extensibility | Promotion |
|---|---|---:|---:|---:|---:|---:|---:|---:|---|---|---|
| IFLC Float Baseline | monolithic | 100.00 | 0 | 0 | 0 | 19.374 | 15.223 | 15.47 | low | low | stable_control |
| OSR Float Strict Pipeline | pipeline+boundary | 0.00 | 0 | 120 | 0 | 4.875 | 5.076 | 15.27 | low | low | exploratory |
| OSR Float SIS-Continuity-Only Pipeline | pipeline+continuity | 0.00 | 0 | 120 | 0 | 4.875 | 5.076 | 15.32 | low | low | exploratory |
| OSR Float Repair-Only Pipeline | pipeline+continuity | 0.00 | 0 | 120 | 0 | 4.875 | 5.076 | 15.12 | low | low | exploratory |
| OSR Float Raw-Phase-Bias Pipeline | pipeline+continuity | 0.00 | 0 | 120 | 0 | 4.875 | 5.076 | 15.24 | low | low | exploratory |
| OSR Float No-Phase-Bias Pipeline | pipeline+continuity | 0.00 | 0 | 120 | 0 | 4.875 | 5.076 | 15.30 | low | low | exploratory |
| OSR AR Strict Pipeline | pipeline+solver+boundary | 0.00 | 0 | 120 | 0 | 4.875 | 5.076 | 15.33 | low | low | exploratory |
| OSR AR SIS-Continuity-Only Pipeline | pipeline+solver+continuity | 0.00 | 0 | 120 | 0 | 4.875 | 5.076 | 15.54 | low | low | exploratory |
| OSR AR Repair-Only Pipeline | pipeline+solver+continuity | 0.00 | 0 | 120 | 0 | 4.875 | 5.076 | 15.42 | low | low | exploratory |
| OSR AR Raw-Phase-Bias Pipeline | pipeline+solver+continuity | 0.00 | 0 | 120 | 0 | 4.875 | 5.076 | 15.45 | low | low | exploratory |
| OSR AR No-Phase-Bias Pipeline | pipeline+solver+continuity | 0.00 | 0 | 120 | 0 | 4.875 | 5.076 | 15.77 | low | low | exploratory |

#### Promotion Gate

| Strategy | Mean delta vs baseline (m) | P95 delta vs baseline (m) | Fixed delta | Solution-rate delta | Gate | Rationale |
|---|---:|---:|---:|---:|---|---|
| IFLC Float Baseline | 0.000 | 0.000 | 0 | 0.00 | hold | Stable control arm used to judge candidate strategies. |
| OSR Float Strict Pipeline | 14.498 | 10.147 | 0 | -100.00 | hold | Still exploratory because these gates are not met: solution_rate_non_regression, readability_non_regression |
| OSR Float SIS-Continuity-Only Pipeline | 14.498 | 10.147 | 0 | -100.00 | hold | Still exploratory because these gates are not met: solution_rate_non_regression, readability_non_regression |
| OSR Float Repair-Only Pipeline | 14.498 | 10.147 | 0 | -100.00 | hold | Still exploratory because these gates are not met: solution_rate_non_regression, readability_non_regression |
| OSR Float Raw-Phase-Bias Pipeline | 14.498 | 10.147 | 0 | -100.00 | hold | Still exploratory because these gates are not met: solution_rate_non_regression, readability_non_regression |
| OSR Float No-Phase-Bias Pipeline | 14.498 | 10.147 | 0 | -100.00 | hold | Still exploratory because these gates are not met: solution_rate_non_regression, readability_non_regression |
| OSR AR Strict Pipeline | 14.498 | 10.147 | 0 | -100.00 | hold | Still exploratory because these gates are not met: solution_rate_non_regression, readability_non_regression, fixed_epochs_improved |
| OSR AR SIS-Continuity-Only Pipeline | 14.498 | 10.147 | 0 | -100.00 | hold | Still exploratory because these gates are not met: solution_rate_non_regression, readability_non_regression, fixed_epochs_improved |
| OSR AR Repair-Only Pipeline | 14.498 | 10.147 | 0 | -100.00 | hold | Still exploratory because these gates are not met: solution_rate_non_regression, readability_non_regression, fixed_epochs_improved |
| OSR AR Raw-Phase-Bias Pipeline | 14.498 | 10.147 | 0 | -100.00 | hold | Still exploratory because these gates are not met: solution_rate_non_regression, readability_non_regression, fixed_epochs_improved |
| OSR AR No-Phase-Bias Pipeline | 14.498 | 10.147 | 0 | -100.00 | hold | Still exploratory because these gates are not met: solution_rate_non_regression, readability_non_regression, fixed_epochs_improved |


## Structural Heuristics

| Strategy | Files | LOC | Max file LOC | Args | CLASLIB reference |
|---|---:|---:|---:|---:|---|
| IFLC Float Baseline | 1 | 4097 | 4097 | 0 | Control arm used to measure how far CLASLIB-inspired observation-space work actually moves the solver. |
| OSR Float Strict Pipeline | 3 | 6076 | 4097 | 5 | Uses the CLASLIB-style observation-space boundary directly, without falling back to the standard PPP epoch path. |
| OSR Float SIS-Continuity-Only Pipeline | 3 | 6076 | 4097 | 7 | Used to test whether the accepted CLAS path degrades because of the SIS continuity delta injection itself rather than because of repair-offset accumulation. |
| OSR Float Repair-Only Pipeline | 3 | 6076 | 4097 | 7 | Used to test whether the accepted CLAS path degrades because of repair-offset accumulation even when SIS continuity deltas are suppressed. |
| OSR Float Raw-Phase-Bias Pipeline | 3 | 6076 | 4097 | 7 | Used to test whether the accepted CLAS path degrades because of the repair/continuity layer rather than because of the raw CLAS phase-bias values. |
| OSR Float No-Phase-Bias Pipeline | 3 | 6076 | 4097 | 7 | Used to isolate whether the accepted CLAS path still degrades when the phase-bias layer is removed entirely. |
| OSR AR Strict Pipeline | 4 | 6896 | 4097 | 8 | Measures the CLASLIB-style observation-space boundary directly so hybrid fallback arms can be judged against a strict CLAS control. |
| OSR AR SIS-Continuity-Only Pipeline | 4 | 6896 | 4097 | 10 | Used to test whether ambiguity fixing is destabilized by SIS continuity delta injection itself rather than by repair-offset accumulation. |
| OSR AR Repair-Only Pipeline | 4 | 6896 | 4097 | 10 | Used to test whether ambiguity fixing is destabilized by repair-offset accumulation even when SIS continuity deltas are suppressed. |
| OSR AR Raw-Phase-Bias Pipeline | 4 | 6896 | 4097 | 10 | Used to test whether ambiguity fixing becomes stable once the accepted CLAS path stops applying continuity repair on top of raw CLAS phase biases. |
| OSR AR No-Phase-Bias Pipeline | 4 | 6896 | 4097 | 10 | Used to isolate whether ambiguity fixing recovers once the phase-bias layer is removed completely from the accepted CLAS update path. |

## Notes

- `IFLC Float Baseline` is the control arm and remains the fallback path.
- `OSR * Pipeline` arms keep the CLASLIB-inspired observation-space boundary and differ in how epoch fallback, atmosphere-token selection, and ambiguity fixing are staged.
- `Strict` arms stay inside the CLAS epoch path. `Hybrid` arms restore the pre-epoch state and re-run the stable standard PPP path when the CLAS OSR stage cannot complete.
- `Guarded` arms keep the nearest grid but reject stale atmosphere tokens completely once they age out.
- `Balanced` arms are stale-aware: they keep the nearest grid while corrections are fresh, then prefer freshness once the current network ages out.
- `Freshness` arms prioritize correction recency over grid proximity so selector failures can be isolated from the cssr2osr-style measurement path.
- `Clock-Bound-*` arms keep the same accepted CLAS update path but tighten expanded-SSR timing acceptance relative to the selected clock epoch.
- `Orbit-* Source` arms change the compact-to-expanded row emission policy before the solver sees any CLAS SSR values.
- `*Atmos-Merge*` arms change how compact STEC polynomial terms are carried or reset before expanded SSR rows are materialized.
- `*Subtype-Merge*` arms change how subtype 8/9/12 atmosphere families replace or coexist before expanded SSR rows are materialized.
- `*Value*` arms change how expanded atmosphere values are constructed from polynomial terms and residual lists before residual formation.
- `*Residual-Sampling*` arms keep the same composed atmosphere model but change whether expanded residual lists use indexed samples, mean fallback, or mean-only sampling.
- `*Subtype12-Value*` arms keep residual sampling fixed and only change how subtype-12 rows retain constant, linear, or higher-order surface terms before residual formation.
- Promotion is decided against the control arm and only becomes stable after repeated suite wins.
- Readability and extensibility are heuristic scores generated from experiment-owned files, file size, and command surface. They are not code-review substitutes.
