# Phase95 raw-input path-corrected diagnostic structural result

- Status: `no-go-phase95-path-corrected-diagnostic-structural-gates`
- Decision: NO-GO: one or more Phase95 diagnostic gates failed; fail closed before truth/accuracy.
- Diagnostic-only: **true**; solution and accuracy output: **withheld**
- Truth/MAT/precomputed-coordinate/base/Kaggle/token reads: **0**
- Matrix: exactly four routes × one native invocation, sequential, no rerun/fallback

## Route summary

| Route | Return | Resolved raw files | GNSS accepted / cost | Main accepted / cost | Output |
|---|---:|---|---|---|---|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | `1` | `device_gnss.csv,device_imu.csv,brdc.nav` | `167 / 5618000.786274777 → 21298.114523521894` | `0 / 79358354.39651252 → 79358354.39651252` | `withheld` |
  - Failure stage: `coverage-contract`; gates: `{'gnss_first_guard_predicate_and_counts': True, 'gnss_first_progress_active_cost_decrease': True, 'main_validation_predicates': True, 'accepted_cost_lambda_terminal_telemetry': False, 'finite_earth_valid_exact_handoff': True, 'official_c0d_units_sigma_and_skip_telemetry': True, 'no_solution_or_accuracy_publication': True, 'command_policy_no_pdc_external_precomputed': True}`
| `2021-08-24-20-32-us-ca-mtv-h/pixel5` | `1` | `device_gnss.csv,device_imu.csv,brdc.nav` | `0 / None → None` | `0 / None → None` | `withheld` |
  - Failure stage: `gnss-first-optimize`; gates: `{'gnss_first_guard_predicate_and_counts': True, 'gnss_first_progress_active_cost_decrease': False, 'main_validation_predicates': False, 'accepted_cost_lambda_terminal_telemetry': False, 'finite_earth_valid_exact_handoff': False, 'official_c0d_units_sigma_and_skip_telemetry': True, 'no_solution_or_accuracy_publication': True, 'command_policy_no_pdc_external_precomputed': True}`
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | `1` | `device_gnss.csv,device_imu.csv,brdc.nav` | `262 / 6419784.856579111 → 12818.288538454106` | `0 / 166205998.11910567 → 166205998.11910567` | `withheld` |
  - Failure stage: `coverage-contract`; gates: `{'gnss_first_guard_predicate_and_counts': True, 'gnss_first_progress_active_cost_decrease': True, 'main_validation_predicates': True, 'accepted_cost_lambda_terminal_telemetry': False, 'finite_earth_valid_exact_handoff': True, 'official_c0d_units_sigma_and_skip_telemetry': True, 'no_solution_or_accuracy_publication': True, 'command_policy_no_pdc_external_precomputed': True}`
| `2023-03-08-21-34-us-ca-mtv-u/pixel5` | `1` | `device_gnss.csv,device_imu.csv,brdc.nav` | `1000 / 23108268994.98668 → 82026870.12520209` | `0 / 15619537525702.566 → 15619537525702.566` | `withheld` |
  - Failure stage: `coverage-contract`; gates: `{'gnss_first_guard_predicate_and_counts': True, 'gnss_first_progress_active_cost_decrease': True, 'main_validation_predicates': False, 'accepted_cost_lambda_terminal_telemetry': False, 'finite_earth_valid_exact_handoff': False, 'official_c0d_units_sigma_and_skip_telemetry': True, 'no_solution_or_accuracy_publication': True, 'command_policy_no_pdc_external_precomputed': True}`

Structural diagnostics only; no truth, accuracy, or submission activity is authorized.
