# Native FGO v2.1 independent confirmation

The original v2.1 30-epoch Pixel 7 Pro pilot remains a closed No-Go: its H
P95 improved, but H P50 regressed. This confirmation does not reinterpret that
pilot, change its code/parameters, or reopen its truth result.

Before materialization, the independent confirmation freeze selected the two
unused identities listed by the route-disjoint v2 inventory:

- `2021-03-10-23-13-us-ca-mtv-h/pixel5`
- `2022-04-01-18-22-us-ca-lax-t/pixel5`

Only central-directory metadata was used for selection and authorization.
After the freeze hash was verified, device GNSS, device IMU, broadcast nav,
and reference RINEX were atomically materialized. Ground truth was not
materialized or opened.

Both truth-free runs used the exact v2.1 Release binary and fixed 200/30
window. Both failed closed during the existing GNSS-course/IMU heading
observability gate, before graph construction. This is a structural No-Go;
there is no valid candidate output or truth metric to compare. The fresh
validation and future holdout remain sealed.

Evidence:

- `docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_v2_1_confirmation_freeze.json`
- `docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_v2_1_confirmation_freeze_manifest.json`
- `docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_v2_1_confirmation_result.json`
- `docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_v2_1_confirmation_result_manifest.json`

The failure is intentionally retained as a reproducibility artifact. No
baseline rerun, truth read, validation opening, parameter search, production
default change, token access, or external mutation was performed.
