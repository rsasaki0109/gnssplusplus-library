# PPP-AR Experiments

Generated: 2026-04-02 21:34 UTC

This page is the current externalized state of the PPP-AR experiment lane.
All rows below use the same strategy catalog, the same metrics, and comparable CLAS inputs.

## Shared Interface

- Suite: `CLAS PPP phase-bias bank suite`
- Cases: `2`
- Primary mode: `static`
- Shared strategy catalog: `experiments/ppp_ar/strategies.toml`
- Shared output schema: `wall_time_s`, `ppp_solution_rate_pct`, `ppp_fixed_solutions`, `fallback_solutions`, `clas_hybrid_fallback_epochs`, `mean_3d_error_m`, `median_3d_error_m`, `p95_3d_error_m`, `max_3d_error_m`, `readability_score`, `extensibility_score`

## Suite Summary

| Strategy | Cases passed | Mean solution rate | Mean 3D (m) | P95 3D (m) | Mean delta vs baseline (m) | Fixed total | Hybrid fallback total | Promotion |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| IFLC Float Baseline | 0/2 | 99.58 | 11.957 | 9.930 | 0.000 | 0 | 0 | stable_control |
| OSR Float Strict Pipeline | 0/2 | 50.00 | 4.898 | 5.134 | 7.059 | 0 | 0 | exploratory |
| OSR Float Base-Plus-Network Phase-Bias-Composition Pipeline | 0/2 | 50.00 | 4.898 | 5.134 | 7.059 | 0 | 0 | exploratory |
| OSR Float Same-30s Phase-Bias-Bank Pipeline | 0/2 | 50.00 | 4.898 | 5.134 | 7.059 | 0 | 0 | exploratory |
| OSR Float Close-30s Phase-Bias-Bank Pipeline | 0/2 | 50.00 | 4.898 | 5.134 | 7.059 | 0 | 0 | exploratory |
| OSR Float Latest-Preceding Phase-Bias-Bank Pipeline | 0/2 | 50.00 | 4.898 | 5.134 | 7.059 | 0 | 0 | exploratory |
| OSR AR Strict Pipeline | 0/2 | 50.00 | 4.898 | 5.134 | 7.059 | 0 | 0 | exploratory |
| OSR AR Base-Plus-Network Phase-Bias-Composition Pipeline | 0/2 | 50.00 | 4.898 | 5.134 | 7.059 | 0 | 0 | exploratory |
| OSR AR Same-30s Phase-Bias-Bank Pipeline | 0/2 | 50.00 | 4.898 | 5.134 | 7.059 | 0 | 0 | exploratory |
| OSR AR Close-30s Phase-Bias-Bank Pipeline | 0/2 | 50.00 | 4.898 | 5.134 | 7.059 | 0 | 0 | exploratory |
| OSR AR Latest-Preceding Phase-Bias-Bank Pipeline | 0/2 | 50.00 | 4.898 | 5.134 | 7.059 | 0 | 0 | exploratory |

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
| IFLC Float Baseline | monolithic | 99.17 | 0 | 1 | 0 | 4.540 | 4.638 | 5.40 | low | low | stable_control |
| OSR Float Strict Pipeline | pipeline+boundary | 100.00 | 0 | 0 | 0 | 4.921 | 5.193 | 4.70 | low | low | exploratory |
| OSR Float Base-Plus-Network Phase-Bias-Composition Pipeline | pipeline+materialization | 100.00 | 0 | 0 | 0 | 4.921 | 5.193 | 4.75 | low | low | exploratory |
| OSR Float Same-30s Phase-Bias-Bank Pipeline | pipeline+materialization | 100.00 | 0 | 0 | 0 | 4.921 | 5.193 | 4.71 | low | low | exploratory |
| OSR Float Close-30s Phase-Bias-Bank Pipeline | pipeline+materialization | 100.00 | 0 | 0 | 0 | 4.921 | 5.193 | 4.70 | low | low | exploratory |
| OSR Float Latest-Preceding Phase-Bias-Bank Pipeline | pipeline+materialization | 100.00 | 0 | 0 | 0 | 4.921 | 5.193 | 4.70 | low | low | exploratory |
| OSR AR Strict Pipeline | pipeline+solver+boundary | 100.00 | 0 | 0 | 0 | 4.921 | 5.193 | 4.80 | low | low | exploratory |
| OSR AR Base-Plus-Network Phase-Bias-Composition Pipeline | pipeline+solver+materialization | 100.00 | 0 | 0 | 0 | 4.921 | 5.193 | 4.81 | low | medium | exploratory |
| OSR AR Same-30s Phase-Bias-Bank Pipeline | pipeline+solver+materialization | 100.00 | 0 | 0 | 0 | 4.921 | 5.193 | 4.83 | low | medium | exploratory |
| OSR AR Close-30s Phase-Bias-Bank Pipeline | pipeline+solver+materialization | 100.00 | 0 | 0 | 0 | 4.921 | 5.193 | 4.80 | low | medium | exploratory |
| OSR AR Latest-Preceding Phase-Bias-Bank Pipeline | pipeline+solver+materialization | 100.00 | 0 | 0 | 0 | 4.921 | 5.193 | 4.83 | low | medium | exploratory |

#### Promotion Gate

| Strategy | Mean delta vs baseline (m) | P95 delta vs baseline (m) | Fixed delta | Solution-rate delta | Gate | Rationale |
|---|---:|---:|---:|---:|---|---|
| IFLC Float Baseline | 0.000 | 0.000 | 0 | 0.00 | hold | Stable control arm used to judge candidate strategies. |
| OSR Float Strict Pipeline | -0.381 | -0.555 | 0 | 0.83 | hold | Still exploratory because these gates are not met: mean_error_improved, p95_error_improved, readability_non_regression |
| OSR Float Base-Plus-Network Phase-Bias-Composition Pipeline | -0.381 | -0.555 | 0 | 0.83 | hold | Still exploratory because these gates are not met: mean_error_improved, p95_error_improved, readability_non_regression |
| OSR Float Same-30s Phase-Bias-Bank Pipeline | -0.381 | -0.555 | 0 | 0.83 | hold | Still exploratory because these gates are not met: mean_error_improved, p95_error_improved, readability_non_regression |
| OSR Float Close-30s Phase-Bias-Bank Pipeline | -0.381 | -0.555 | 0 | 0.83 | hold | Still exploratory because these gates are not met: mean_error_improved, p95_error_improved, readability_non_regression |
| OSR Float Latest-Preceding Phase-Bias-Bank Pipeline | -0.381 | -0.555 | 0 | 0.83 | hold | Still exploratory because these gates are not met: mean_error_improved, p95_error_improved, readability_non_regression |
| OSR AR Strict Pipeline | -0.381 | -0.555 | 0 | 0.83 | hold | Still exploratory because these gates are not met: mean_error_improved, p95_error_improved, readability_non_regression, fixed_epochs_improved |
| OSR AR Base-Plus-Network Phase-Bias-Composition Pipeline | -0.381 | -0.555 | 0 | 0.83 | hold | Still exploratory because these gates are not met: mean_error_improved, p95_error_improved, readability_non_regression, fixed_epochs_improved |
| OSR AR Same-30s Phase-Bias-Bank Pipeline | -0.381 | -0.555 | 0 | 0.83 | hold | Still exploratory because these gates are not met: mean_error_improved, p95_error_improved, readability_non_regression, fixed_epochs_improved |
| OSR AR Close-30s Phase-Bias-Bank Pipeline | -0.381 | -0.555 | 0 | 0.83 | hold | Still exploratory because these gates are not met: mean_error_improved, p95_error_improved, readability_non_regression, fixed_epochs_improved |
| OSR AR Latest-Preceding Phase-Bias-Bank Pipeline | -0.381 | -0.555 | 0 | 0.83 | hold | Still exploratory because these gates are not met: mean_error_improved, p95_error_improved, readability_non_regression, fixed_epochs_improved |

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
| IFLC Float Baseline | monolithic | 100.00 | 0 | 0 | 0 | 19.374 | 15.223 | 14.68 | low | low | stable_control |
| OSR Float Strict Pipeline | pipeline+boundary | 0.00 | 0 | 120 | 0 | 4.875 | 5.076 | 14.65 | low | low | exploratory |
| OSR Float Base-Plus-Network Phase-Bias-Composition Pipeline | pipeline+materialization | 0.00 | 0 | 120 | 0 | 4.875 | 5.076 | 14.43 | low | low | exploratory |
| OSR Float Same-30s Phase-Bias-Bank Pipeline | pipeline+materialization | 0.00 | 0 | 120 | 0 | 4.875 | 5.076 | 14.90 | low | low | exploratory |
| OSR Float Close-30s Phase-Bias-Bank Pipeline | pipeline+materialization | 0.00 | 0 | 120 | 0 | 4.875 | 5.076 | 14.54 | low | low | exploratory |
| OSR Float Latest-Preceding Phase-Bias-Bank Pipeline | pipeline+materialization | 0.00 | 0 | 120 | 0 | 4.875 | 5.076 | 14.46 | low | low | exploratory |
| OSR AR Strict Pipeline | pipeline+solver+boundary | 0.00 | 0 | 120 | 0 | 4.875 | 5.076 | 14.51 | low | low | exploratory |
| OSR AR Base-Plus-Network Phase-Bias-Composition Pipeline | pipeline+solver+materialization | 0.00 | 0 | 120 | 0 | 4.875 | 5.076 | 14.45 | low | medium | exploratory |
| OSR AR Same-30s Phase-Bias-Bank Pipeline | pipeline+solver+materialization | 0.00 | 0 | 120 | 0 | 4.875 | 5.076 | 14.49 | low | medium | exploratory |
| OSR AR Close-30s Phase-Bias-Bank Pipeline | pipeline+solver+materialization | 0.00 | 0 | 120 | 0 | 4.875 | 5.076 | 14.24 | low | medium | exploratory |
| OSR AR Latest-Preceding Phase-Bias-Bank Pipeline | pipeline+solver+materialization | 0.00 | 0 | 120 | 0 | 4.875 | 5.076 | 14.25 | low | medium | exploratory |

#### Promotion Gate

| Strategy | Mean delta vs baseline (m) | P95 delta vs baseline (m) | Fixed delta | Solution-rate delta | Gate | Rationale |
|---|---:|---:|---:|---:|---|---|
| IFLC Float Baseline | 0.000 | 0.000 | 0 | 0.00 | hold | Stable control arm used to judge candidate strategies. |
| OSR Float Strict Pipeline | 14.498 | 10.147 | 0 | -100.00 | hold | Still exploratory because these gates are not met: solution_rate_non_regression, readability_non_regression |
| OSR Float Base-Plus-Network Phase-Bias-Composition Pipeline | 14.498 | 10.147 | 0 | -100.00 | hold | Still exploratory because these gates are not met: solution_rate_non_regression, readability_non_regression |
| OSR Float Same-30s Phase-Bias-Bank Pipeline | 14.498 | 10.147 | 0 | -100.00 | hold | Still exploratory because these gates are not met: solution_rate_non_regression, readability_non_regression |
| OSR Float Close-30s Phase-Bias-Bank Pipeline | 14.498 | 10.147 | 0 | -100.00 | hold | Still exploratory because these gates are not met: solution_rate_non_regression, readability_non_regression |
| OSR Float Latest-Preceding Phase-Bias-Bank Pipeline | 14.498 | 10.147 | 0 | -100.00 | hold | Still exploratory because these gates are not met: solution_rate_non_regression, readability_non_regression |
| OSR AR Strict Pipeline | 14.498 | 10.147 | 0 | -100.00 | hold | Still exploratory because these gates are not met: solution_rate_non_regression, readability_non_regression, fixed_epochs_improved |
| OSR AR Base-Plus-Network Phase-Bias-Composition Pipeline | 14.498 | 10.147 | 0 | -100.00 | hold | Still exploratory because these gates are not met: solution_rate_non_regression, readability_non_regression, fixed_epochs_improved |
| OSR AR Same-30s Phase-Bias-Bank Pipeline | 14.498 | 10.147 | 0 | -100.00 | hold | Still exploratory because these gates are not met: solution_rate_non_regression, readability_non_regression, fixed_epochs_improved |
| OSR AR Close-30s Phase-Bias-Bank Pipeline | 14.498 | 10.147 | 0 | -100.00 | hold | Still exploratory because these gates are not met: solution_rate_non_regression, readability_non_regression, fixed_epochs_improved |
| OSR AR Latest-Preceding Phase-Bias-Bank Pipeline | 14.498 | 10.147 | 0 | -100.00 | hold | Still exploratory because these gates are not met: solution_rate_non_regression, readability_non_regression, fixed_epochs_improved |


## Structural Heuristics

| Strategy | Files | LOC | Max file LOC | Args | CLASLIB reference |
|---|---:|---:|---:|---:|---|
| IFLC Float Baseline | 1 | 4097 | 4097 | 0 | Control arm used to measure how far CLASLIB-inspired observation-space work actually moves the solver. |
| OSR Float Strict Pipeline | 3 | 6145 | 4097 | 5 | Uses the CLASLIB-style observation-space boundary directly, without falling back to the standard PPP epoch path. |
| OSR Float Base-Plus-Network Phase-Bias-Composition Pipeline | 4 | 8871 | 4097 | 5 | Used to test whether the compact-to-expanded SSR lane should follow CLASLIB-style add_base_value_to_pbias semantics instead of using raw subtype-6 network values directly. |
| OSR Float Same-30s Phase-Bias-Bank Pipeline | 4 | 8871 | 4097 | 5 | Used to test whether the compact-to-expanded SSR lane should follow the CLASLIB-style 30-second base-bank anchoring implied by add_base_value_to_pbias instead of only using the current pending epoch. |
| OSR Float Close-30s Phase-Bias-Bank Pipeline | 4 | 8871 | 4097 | 5 | Used to test whether the compact-to-expanded SSR lane should follow CLASLIB-style close-bank lookup semantics rather than requiring the exact pending anchor or accepting any older preceding bank. |
| OSR Float Latest-Preceding Phase-Bias-Bank Pipeline | 4 | 8871 | 4097 | 5 | Used as a negative control to test whether ambiguity quality changes simply because the compact-to-expanded SSR lane reuses the most recent base phase-bias bank rather than the exact 30-second anchor. |
| OSR AR Strict Pipeline | 4 | 6965 | 4097 | 8 | Measures the CLASLIB-style observation-space boundary directly so hybrid fallback arms can be judged against a strict CLAS control. |
| OSR AR Base-Plus-Network Phase-Bias-Composition Pipeline | 5 | 9691 | 4097 | 6 | Used to test whether ambiguity fixing recovers once the compact-to-expanded SSR lane follows CLASLIB-style add_base_value_to_pbias semantics for network phase-bias rows. |
| OSR AR Same-30s Phase-Bias-Bank Pipeline | 5 | 9691 | 4097 | 6 | Used to test whether ambiguity fixing recovers once the compact-to-expanded SSR lane follows the CLASLIB-style 30-second base-bank anchoring implied by add_base_value_to_pbias. |
| OSR AR Close-30s Phase-Bias-Bank Pipeline | 5 | 9691 | 4097 | 6 | Used to test whether ambiguity fixing recovers once the compact-to-expanded SSR lane follows CLASLIB-style close-bank lookup semantics rather than requiring the exact pending anchor or accepting any older preceding bank. |
| OSR AR Latest-Preceding Phase-Bias-Bank Pipeline | 5 | 9691 | 4097 | 6 | Used as a negative control to test whether ambiguity fixing changes simply because the compact-to-expanded SSR lane reuses the most recent base phase-bias bank rather than the exact 30-second anchor. |

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
- `*Phase-Bias-Merge*` arms change how compact subtype-5/6 phase-bias rows reset or prune stale satellite scope before expanded SSR rows are materialized.
- `*Phase-Bias-Source*` arms keep the same compact epoch contents but change whether subtype-5, subtype-6, or raw arrival order wins when both provide the same signal row before expanded SSR materialization.
- `*Phase-Bias-Composition*` arms keep the same accepted source rows but change whether subtype-6 network rows are used directly, added on top of base rows, or replaced by base rows before expanded SSR materialization.
- `*Phase-Bias-Bank*` arms keep the same accepted source rows and composition rule but change whether subtype-6 network rows can look up base phase-bias banks only inside the pending epoch, at the same 30-second anchor, from the closest preceding anchor within 30 seconds, or from the latest preceding anchor before expanded SSR materialization.
- `*Value*` arms change how expanded atmosphere values are constructed from polynomial terms and residual lists before residual formation.
- `*Residual-Sampling*` arms keep the same composed atmosphere model but change whether expanded residual lists use indexed samples, mean fallback, or mean-only sampling.
- `*Subtype12-Value*` arms keep residual sampling fixed and only change how subtype-12 rows retain constant, linear, or higher-order surface terms before residual formation.
- `*Phase-Bias-Only*` and `*Compensation-Only*` arms keep continuity timing fixed and only change whether accepted CLAS updates use compact phase-bias values, derived phase compensation, or both before SIS/repair injection.
- `*Reference-Time*` arms keep accepted phase-bias values fixed and only change which epoch anchors SIS continuity and repair-offset state before phase-bias corrections are injected.
- Promotion is decided against the control arm and only becomes stable after repeated suite wins.
- Readability and extensibility are heuristic scores generated from experiment-owned files, file size, and command surface. They are not code-review substitutes.
