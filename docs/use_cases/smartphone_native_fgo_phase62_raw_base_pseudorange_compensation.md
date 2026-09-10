# Phase 62: raw base-station pseudorange compensation

Phase 62 freezes one source-supported, non-overlapping candidate before any
new raw read. The candidate is the base-station path in the official
[`taroz/gsdc2023`](https://github.com/taroz/gsdc2023) repository at commit
`29923f9f370f09ebc00f96d8cca375007a18e7d5`, specifically
`functions/correct_pseudorange.m` (SHA-256
`b0536ccff478b0aff253448ffb7a203c715b8064dd8dc85898e38f1f05d0441e`).

The frozen source contract is exact: match the same satellite and frequency,
form the modeled base pseudorange residual, smooth it with a 151-sample
moving mean for 1 Hz data or an 11-sample moving mean for 15 s data, linearly
interpolate at rover observation times, and subtract the result from the raw
rover pseudorange. In symbols,
`P_rover_corrected = P_rover - pc_f(t_rover)`. No extrapolation, nearest-time
fill, cross-signal fill, fitted coefficient, or truth-derived correction is
allowed.

The later raw preflight permits exactly one hash-pinned base-station RINEX per
route, with a finite header `APPROX POSITION XYZ` or independently published
base-station coordinate and explicit station provenance. The existing upstream
`base_position.csv`/`base_offset.csv` files are not silently imported, and
MAT/MATLAB products, handset truth, WLS/Sv coordinates, validation/holdout,
Kaggle/token inputs, and solver reruns are forbidden. If the four routes do
not have verified base RINEX, coordinate, station role, and time overlap, the
candidate fails closed without a substitute.

The source-only freeze has read zero raw GNSS, base RINEX, navigation, truth,
IMU, MAT, archive, or solver inputs. Later gates require all four routes,
finite correction coverage of at least 99%, at least 80% overlap with adopted
rover pseudorange factors, fixed same-satellite/frequency keys, routewise
raw-only correction materiality, and a routewise improvement in a declared
adjacent code-rate innovation diagnostic. Phase 43 remains the no-base
champion; Phase 62 does not score accuracy or claim 0.782 reachability.

The machine-readable freeze is
[`smartphone_r5_phase62_raw_base_pseudorange_compensation_freeze_v1.json`](records/smartphone_r5_phase62_raw_base_pseudorange_compensation_freeze_v1.json).

## Preflight result

The sealed preflight opened the archive once and read `settings_train.csv`
once, but failed closed because the observed settings SHA-256 was
`3e6ae65388b2809088b16732b87744e673f860c24a1fe0f709ef903a87397f39`, not the
predeclared `cb868652632a90919d9b21decaa9b77627d75d16d75287a5430a92b6cf29e080`.
No base observation member was opened or materialized, so base availability,
station identity, header coordinate, and time overlap are deliberately
unknown rather than inferred. The archive was not reopened. No native
correction or accuracy evaluation is authorized. See the
[`preflight result`](records/smartphone_r5_phase62_raw_base_preflight_result_v1.json).
