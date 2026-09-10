# Phase94 source-clock C0/D diagnostic structural result

- Status: `no-go-phase94-diagnostic-structural-gates`
- Decision: NO-GO: one or more diagnostic gates failed; fail closed before truth/accuracy.
- Diagnostic-only: **true**; solution and accuracy output: **withheld**
- Truth/MAT/precomputed-coordinate/base/Kaggle/token reads: **0**
- Matrix: exactly four routes × one native invocation, sequential, no controls/reruns/fallbacks

## Route telemetry

| Route | Return | Raw files present | Failure stage | Retained epochs | D init finite/full | Guard failures | GNSS accepted / cost | Main accepted / cost | Lambda / conditioning | Output |
|---|---:|---|---|---:|---|---|---|---|---|---|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | `1` | `none` | `native-launch-or-pre-summary` | `n/a` | `n/a` | `none` | `n/a / n/a → n/a` | `n/a / n/a → n/a` | `n/a → n/a / n/a` | `withheld` |
  - Failure: `native summary was not produced; raw input or an earlier native stage failed closed`
  - Telemetry gates: `{'gnss_first_guard_predicate_and_counts': False, 'gnss_first_progress_active_cost_decrease': False, 'main_validation_predicates': False, 'accepted_cost_lambda_terminal_telemetry': False, 'finite_earth_valid_exact_handoff': False, 'official_c0d_units_sigma_and_skip_telemetry': False, 'no_solution_or_accuracy_publication': True, 'command_policy_no_pdc_external_precomputed': True}`
| `2021-08-24-20-32-us-ca-mtv-h/pixel5` | `1` | `none` | `native-launch-or-pre-summary` | `n/a` | `n/a` | `none` | `n/a / n/a → n/a` | `n/a / n/a → n/a` | `n/a → n/a / n/a` | `withheld` |
  - Failure: `native summary was not produced; raw input or an earlier native stage failed closed`
  - Telemetry gates: `{'gnss_first_guard_predicate_and_counts': False, 'gnss_first_progress_active_cost_decrease': False, 'main_validation_predicates': False, 'accepted_cost_lambda_terminal_telemetry': False, 'finite_earth_valid_exact_handoff': False, 'official_c0d_units_sigma_and_skip_telemetry': False, 'no_solution_or_accuracy_publication': True, 'command_policy_no_pdc_external_precomputed': True}`
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | `1` | `none` | `native-launch-or-pre-summary` | `n/a` | `n/a` | `none` | `n/a / n/a → n/a` | `n/a / n/a → n/a` | `n/a → n/a / n/a` | `withheld` |
  - Failure: `native summary was not produced; raw input or an earlier native stage failed closed`
  - Telemetry gates: `{'gnss_first_guard_predicate_and_counts': False, 'gnss_first_progress_active_cost_decrease': False, 'main_validation_predicates': False, 'accepted_cost_lambda_terminal_telemetry': False, 'finite_earth_valid_exact_handoff': False, 'official_c0d_units_sigma_and_skip_telemetry': False, 'no_solution_or_accuracy_publication': True, 'command_policy_no_pdc_external_precomputed': True}`
| `2023-03-08-21-34-us-ca-mtv-u/pixel5` | `1` | `none` | `native-launch-or-pre-summary` | `n/a` | `n/a` | `none` | `n/a / n/a → n/a` | `n/a / n/a → n/a` | `n/a → n/a / n/a` | `withheld` |
  - Failure: `native summary was not produced; raw input or an earlier native stage failed closed`
  - Telemetry gates: `{'gnss_first_guard_predicate_and_counts': False, 'gnss_first_progress_active_cost_decrease': False, 'main_validation_predicates': False, 'accepted_cost_lambda_terminal_telemetry': False, 'finite_earth_valid_exact_handoff': False, 'official_c0d_units_sigma_and_skip_telemetry': False, 'no_solution_or_accuracy_publication': True, 'command_policy_no_pdc_external_precomputed': True}`

## Gate summary

```json
{
  "accepted_cost_lambda_terminal_telemetry": false,
  "accuracy_not_scored": true,
  "all_gates_anded": false,
  "all_passed": false,
  "command_policy_no_pdc_external_precomputed": true,
  "diagnostic_return_code_fail_closed": true,
  "exactly_four_routes_one_run_each": true,
  "finite_earth_valid_exact_handoff": false,
  "gnss_first_guard_predicate_and_counts": false,
  "gnss_first_progress_active_cost_decrease": false,
  "implementation_and_binary_pins": true,
  "main_validation_predicates": false,
  "native_process_completed": true,
  "no_solution_or_accuracy_publication": true,
  "official_c0d_units_sigma_and_skip_telemetry": false,
  "solution_output_withheld": true,
  "truth_free": true
}
```

Structural diagnostics only; no truth, accuracy, or submission activity is authorized.
