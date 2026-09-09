# Phase99 main multifrontal-QR structural result

- Status: `go-phase99-main-multifrontal-qr-structural`
- Decision: Structural gates passed; solution and accuracy lanes remain unauthorized.
- Matrix: exactly MTV-A and LAX-T, one native diagnostic invocation each, sequential; no rerun/fallback
- Candidate: Phase93 meter-state main graph only, `MULTIFRONTAL_QR` / `EliminateQR`; GNSS-first remains historical Cholesky
- Raw inputs: Phase95 exact device_gnss.csv, device_imu.csv, brdc.nav paths only; wrapper raw-byte reads: 0
- Truth/MAT/base/precomputed-coordinate/Kaggle/token/accuracy/solution-row publication: unauthorized and withheld

## Route summary

| Route | Native return | GNSS-first cost / accepted | Main solver | Main cost / accepted | D handoff | Coverage | Structural gates |
|---|---:|---|---|---|---|---|---|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | `1` | `5618000.786274777 → 21298.114523521894 / 167` | `MULTIFRONTAL_QR / EliminateQR` | `79358354.39651252 → 34843.513194108804 / 12` | `2159/2159 exact=True` | `2159 epochs; earth=True; finite=True` | `{'native_diagnostic_process_completed': True, 'summary_present': True, 'finite_structural_summary': True, 'gnss_first_strict_progress': True, 'gnss_first_full_finite_d_exact_handoff': True, 'main_selected_multifrontal_qr': True, 'main_accepted_outer_iterations': True, 'main_strict_cost_decrease': True, 'main_finite_expected_coverage': True, 'no_fallback_or_solution_publication': True}` |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | `1` | `6419784.856579111 → 12818.288538454106 / 262` | `MULTIFRONTAL_QR / EliminateQR` | `166205998.11910567 → 20384.75538283932 / 12` | `1466/1466 exact=True` | `1466 epochs; earth=True; finite=True` | `{'native_diagnostic_process_completed': True, 'summary_present': True, 'finite_structural_summary': True, 'gnss_first_strict_progress': True, 'gnss_first_full_finite_d_exact_handoff': True, 'main_selected_multifrontal_qr': True, 'main_accepted_outer_iterations': True, 'main_strict_cost_decrease': True, 'main_finite_expected_coverage': True, 'no_fallback_or_solution_publication': True}` |

The withheld CSV path is recorded as metadata only and is never opened, published, or committed. Structural GO does not authorize accuracy evaluation or submission.
