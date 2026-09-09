# Phase113 remaining-route diagnostic result

- status: `no-go-phase113-remaining-routes-diagnostic`
- routes: exactly MTV-H then MTV-U, one native invocation each
- raw phone/base/nav/IMU only; truth/MAT/PDC/precomputed coordinates/Kaggle/accuracy: not read
- solution CSV: opaque, never opened or published

| Route | Return | Summary | Structural gates |
|---|---:|---|---:|
| `2021-08-24-20-32-us-ca-mtv-h/pixel5` | `-6` | `False` | `1/8` |
  - failure stage: `native-summary-unavailable-fail-closed`; failed: `['native_process_completed', 'summary_present', 'stage_and_admission_telemetry', 'eligible_vs_active_c0d_telemetry', 'raw_base_corrected_miss_telemetry', 'main_coverage_earth_valid_solver_progress', 'finite_handoff_when_reached']`
| `2023-03-08-21-34-us-ca-mtv-u/pixel5` | `1` | `False` | `1/8` |
  - failure stage: `native-summary-unavailable-fail-closed`; failed: `['native_process_completed', 'summary_present', 'stage_and_admission_telemetry', 'eligible_vs_active_c0d_telemetry', 'raw_base_corrected_miss_telemetry', 'main_coverage_earth_valid_solver_progress', 'finite_handoff_when_reached']`

The native stderr also records H's uncaught C0/D admission exception and U's
partial coverage-contract observation (`c0d_factor_count=671`, 12 accepted
main iterations, strict cost decrease, position/clock coverage false).  Full
per-family/key and raw-base taxonomy were not emitted before the native
fail-closed returns and remain `null`; the sealed Phase95 telemetry is kept
under `historical_only` for context.

Missing native summaries and pre-handoff failures remain sealed fail-closed; no rerun or fallback is available.
