# Phase339: low-elevation troposphere difference through dense smoothing

Extended the Phase338 raw-base diagnostic (original preserved in commit
3bf9a53b) to evaluate native and independently compiled MADOCA Saastamoinen
delay at each observed source L1/L5 slot. Same raw base/nav pins as Phase338.
Use humidity 0.7 and the same header position; no receiver solutions or truth.
The signed base-residual change is native delay minus oracle delay. Group by
satellite/storage slot; missing epochs are NaN on the full 3500-epoch grid;
apply the native centered finite-only 151-sample moving mean to differences.
Linearity applies because the two residual variants have identical support.
Count differences above 1e-6 m, and separately report observed points with
base elevation >=5 and >=15 degrees. These are NOT actual rover queries.

One raw pass after successful C++17/O1 build exited 0. Phase338 support
counts repeated exactly. There were 38 streams, 2516 changed raw rows and
3615 changed smoothed grid points. Maximum absolute raw and smoothed change
was 2502.8471084125231 m. The smoothed maximum includes otherwise missing
grid epochs supported by finite samples and is not a retained rover factor.
Both changed-observed-above-5 and changed-observed-above-15 counts were zero;
both maximum differences in those observed subsets were reported as zero.

The very large discrepancy occurs in near-horizon standard-atmosphere
extrapolation, not a verified physical correction or positioning improvement.
This boundary mismatch exists but does not spread into the measured base
observations above 5 degrees under this smoothing window. Do not promote an
unclamped model based on the large maximum. Actual rover correction-query
overlap remains untested, including points where base observations are missing.
The all-health support superset and native parser assumptions remain as in
Phase338. No production code, global atmospheric helper or score changed.

Build: Phase338 native dependency link plus separately compiled MADOCA
rtkcmn.o, external header include, and linker --gc-sections. Executable
`/dev/shm/gnss-test-build-recovery.BJvQt9/base_atmosphere_smoothing_audit`.
The source/thresholds were set before the raw pass; this is a diagnostic,
not a hash-frozen accuracy experiment or independent full pipeline parity.
