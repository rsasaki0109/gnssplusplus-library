# Phase96 main C0/D diagnostic structural result

- Status: `captured-phase96-main-c0d-diagnostic`
- Decision: Diagnostic telemetry captured; no convergence, accuracy, solution, or promotion claim is made.
- Diagnostic-only: **true**; solution rows and accuracy: **withheld**
- Matrix: exactly two primary routes × one native invocation, sequential, no rerun/fallback
- Raw inputs: device_gnss.csv, device_imu.csv, brdc.nav only; wrapper raw-byte reads: 0
- Truth/MAT/base/precomputed-coordinate/Kaggle/token/accuracy activity: 0

## Route diagnostic summary

| Route | Return | GNSS-first accepted / cost | Main accepted / cost | Factor families | Norm buckets | LM trials | Capture |
|---|---:|---|---|---:|---:|---:|---|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | `1` | `167 / 5618000.786274777 → 21298.114523521894` | `0 / 79358354.39651252 → 79358354.39651252` | `7` | `16` | `10` | `True` |
  - Capture gates: `{'native_process_completed': True, 'expected_diagnostic_return_code': True, 'summary_present': True, 'phase96_graph_observed': True, 'factor_family_costs': True, 'gradient_normal_norms': True, 'first_ten_lm_trials': True, 'linear_solver_exception_telemetry': True, 'no_solution_or_accuracy_publication': True}`
  - Phase96 terminal branch: `maximum_lambda`
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | `1` | `262 / 6419784.856579111 → 12818.288538454106` | `0 / 166205998.11910567 → 166205998.11910567` | `7` | `16` | `10` | `True` |
  - Capture gates: `{'native_process_completed': True, 'expected_diagnostic_return_code': True, 'summary_present': True, 'phase96_graph_observed': True, 'factor_family_costs': True, 'gradient_normal_norms': True, 'first_ten_lm_trials': True, 'linear_solver_exception_telemetry': True, 'no_solution_or_accuracy_publication': True}`
  - Phase96 terminal branch: `maximum_lambda`

The factor-family costs, variable/family gradient and normal-diagonal norms, and first-ten existing LM trials are diagnostic observations only. No solution coordinate, raw observation, truth value, or accuracy score is published.
