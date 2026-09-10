# Phase98 compact solver-rank diagnostic structural result

- Status: `captured-phase98-compact-solver-diagnostic`
- Decision: Compact solver-boundary telemetry captured; no solution, accuracy, or promotion claim is made.
- Diagnostic-only: **true**; solution rows and accuracy: **withheld**
- Matrix: exactly MTV-A and LAX-T × one native invocation each, sequential, no rerun/fallback
- Raw inputs: Phase95 exact device_gnss.csv, device_imu.csv, brdc.nav paths only; wrapper raw-byte reads: 0
- Truth/MAT/base/precomputed-coordinate/Kaggle/token/accuracy activity: 0
- Phase97 large incidence artifact: referenced by hash only; not opened, copied, or re-emitted

## Route summary

| Route | Return | Phase98 solver | Ordering | 10 trials | Nearby key | Capture |
|---|---:|---|---|---:|---|---|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | `1` | `MULTIFRONTAL_CHOLESKY` / `multifrontal` | `COLAMD` / `EliminatePreferCholesky` | `10` | `0 exact / 10 unavailable` | `True` |
  - Phase96 costs/accepted: `79358354.39651252` → `79358354.39651252`, `0`; damping `False`; ordering digest `fnv1a64:817655d4560ba216`
  - Trial lambda/rejection/status: `[(1e-05, 'indeterminate_linear_system', 'nearby_variable_unavailable'), (0.0001, 'indeterminate_linear_system', 'nearby_variable_unavailable'), (0.001, 'indeterminate_linear_system', 'nearby_variable_unavailable'), (0.01, 'indeterminate_linear_system', 'nearby_variable_unavailable'), (0.1, 'indeterminate_linear_system', 'nearby_variable_unavailable'), (1, 'indeterminate_linear_system', 'nearby_variable_unavailable'), (10, 'indeterminate_linear_system', 'nearby_variable_unavailable'), (100, 'indeterminate_linear_system', 'nearby_variable_unavailable'), (1000, 'indeterminate_linear_system', 'nearby_variable_unavailable'), (10000, 'maximum_lambda_stop', 'nearby_variable_unavailable')]`
  - Exception text digests: `['sha256:142e28caa48effa2e6db54e4d80460250f7cd4e6c1c3c22fe8fd86d7053fd10d']`
  - Phase97 nearby interpretation: `Sealed Phase97 route records expose no nearby key; family, factor incidence, and anchor/prior attribution are therefore unavailable and are not inferred.`
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | `1` | `MULTIFRONTAL_CHOLESKY` / `multifrontal` | `COLAMD` / `EliminatePreferCholesky` | `10` | `0 exact / 10 unavailable` | `True` |
  - Phase96 costs/accepted: `166205998.11910567` → `166205998.11910567`, `0`; damping `False`; ordering digest `fnv1a64:337c93d4c55426f4`
  - Trial lambda/rejection/status: `[(1e-05, 'indeterminate_linear_system', 'nearby_variable_unavailable'), (0.0001, 'indeterminate_linear_system', 'nearby_variable_unavailable'), (0.001, 'indeterminate_linear_system', 'nearby_variable_unavailable'), (0.01, 'indeterminate_linear_system', 'nearby_variable_unavailable'), (0.1, 'indeterminate_linear_system', 'nearby_variable_unavailable'), (1, 'indeterminate_linear_system', 'nearby_variable_unavailable'), (10, 'indeterminate_linear_system', 'nearby_variable_unavailable'), (100, 'indeterminate_linear_system', 'nearby_variable_unavailable'), (1000, 'indeterminate_linear_system', 'nearby_variable_unavailable'), (10000, 'maximum_lambda_stop', 'nearby_variable_unavailable')]`
  - Exception text digests: `['sha256:142e28caa48effa2e6db54e4d80460250f7cd4e6c1c3c22fe8fd86d7053fd10d']`
  - Phase97 nearby interpretation: `Sealed Phase97 route records expose no nearby key; family, factor incidence, and anchor/prior attribution are therefore unavailable and are not inferred.`

All graph/solver/LM data above is compact diagnostic telemetry only. No solution coordinate, raw observation, truth value, or accuracy score is published.
