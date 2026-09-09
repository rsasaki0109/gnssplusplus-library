# Phase93 raw structural result

- Status: `no-go-phase93-source-staging-clock-state-handoff-structural-gates`
- Decision: NO-GO: one or more authorized raw-only structural gates failed; fail closed before truth/accuracy.
- Truth/accuracy/MAT/Kaggle/base/precomputed-coordinate reads: **0**
- Matrix: exactly four routes × one native invocation, sequential, no controls/reruns/fallbacks

## Route gates

| Route | Return | GNSS-first accepted | Main accepted | Initial → final (main) | All gates |
|---|---:|---:|---:|---:|---|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | `1` | `n/a` | `n/a` | `n/a → n/a` | `False` |
  - Failure: `native route did not return zero: 2021-03-16-18-59-us-ca-mtv-a/pixel5/1`
| `2021-08-24-20-32-us-ca-mtv-h/pixel5` | `-6` | `n/a` | `n/a` | `n/a → n/a` | `False` |
  - Failure: `native route did not return zero: 2021-08-24-20-32-us-ca-mtv-h/pixel5/-6`
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | `1` | `n/a` | `n/a` | `n/a → n/a` | `False` |
  - Failure: `native route did not return zero: 2022-04-01-18-22-us-ca-lax-t/pixel5/1`
| `2023-03-08-21-34-us-ca-mtv-u/pixel5` | `1` | `n/a` | `n/a` | `n/a → n/a` | `False` |
  - Failure: `native route did not return zero: 2023-03-08-21-34-us-ca-mtv-u/pixel5/1`

The native processes fail closed before publishing a summary or structural
output, so the summary-derived GNSS-first/main telemetry is intentionally
`n/a` and output coverage is zero published rows on every route.  Preserved
native stderr provides the following failure evidence (not an accuracy
measurement):

- `2021-03-16-18-59-us-ca-mtv-a/pixel5`: Phase93 optimized-D/progress gate,
  D coverage `true`, position/clock coverage `true`, velocity coverage
  `true`, C0/D factors `2158`, main accepted iterations `0`, cost
  `79,358,400 → 79,358,400` (not strict); expected output `2158` rows /
  `2159` summary epochs, published `0`.
- `2021-08-24-20-32-us-ca-mtv-h/pixel5`: native `invalid_argument` because
  `ClockFactor_CCDD` requires direct observable quality, no PDC bridge, and
  an approved GTSAM batch path; expected output `3139` rows / `3140` summary
  epochs, published `0`; no GNSS-first/main, C/D handoff, or conditioning
  telemetry was published.
- `2022-04-01-18-22-us-ca-lax-t/pixel5`: Phase93 optimized-D/progress gate,
  D coverage `true`, position/clock coverage `true`, velocity coverage
  `true`, C0/D factors `1465`, main accepted iterations `0`, cost
  `166,206,000 → 166,206,000` (not strict); expected output `1465` rows /
  `1466` summary epochs, published `0`.
- `2023-03-08-21-34-us-ca-mtv-u/pixel5`: Phase93 optimized-D/progress gate,
  D coverage `true`, position/clock coverage `false`, velocity coverage
  `true`, C0/D factors `671`, main accepted iterations `0`, cost
  `15,619,500,000,000 → 15,619,500,000,000` (not strict); expected output
  `1101` rows / `1102` summary epochs, published `0`.

For routes 1, 3, and 4, the native failure line is main-stage evidence only;
GNSS-first iterations/costs, exact retained-key alignment, and conditioning /
lambda values were not emitted before fail-closed termination.  The D,
position/clock, and velocity booleans above are the native contract's own
failure evidence, not post-hoc coordinate or accuracy evaluation.

The native stderr/log paths and hashes are retained in the JSON result; no
route was rerun after failure.

## Gate summary

```json
{
  "accuracy_not_scored": true,
  "all_gates_anded": false,
  "all_passed": false,
  "c0d_skip_conditioning_lambda_telemetry": false,
  "exactly_four_routes_one_run_each": true,
  "expected_output_coverage": false,
  "finite_earth_valid_output_and_speed": false,
  "gnss_first_iterations_and_strict_cost_decrease": false,
  "implementation_and_binary_pins": true,
  "main_iterations_and_strict_cost_decrease": false,
  "meter_clock_isb_units_and_official_sigma": false,
  "no_pdc_external_or_precomputed_coordinates": false,
  "optimized_C_D_finite_full_exact_handoff": false,
  "return_code_zero": false,
  "truth_free": true
}
```

Structural result only; no accuracy or submission release is authorized.
