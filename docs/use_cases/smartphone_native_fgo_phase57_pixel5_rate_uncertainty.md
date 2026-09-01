# Phase 57 Pixel5 PseudorangeRateUncertainty identifiability audit

Phase 57 audited the sole nonduplicate factor selected by Phase 56:
Android per-satellite/per-signal `PseudorangeRateUncertaintyMetersPerSecond`.
The audit is truth-free and raw-only.  It uses the Phase25 raw pseudorange
reconstruction and the Phase41 Android range-rate sign contract, with a
same-epoch/HCDC median center to remove receiver common mode.  It does not
read truth, navigation, solver output, coordinates, IMU, MAT, validation,
holdout, archive, or prior metric payloads.

The freeze was pushed before any Phase 57 raw read in `0dd78a7`; its SHA-256 is
`11501f4a03dd03cefecb3ff931a9ce8a1b4322953daacd84a95e62c19397fafd`.
The evaluator, focused tests, CMake registration, and manifest were sealed and
pushed in `61fbe15`; the manifest SHA-256 is
`bd9d1a405788b7e808c863f3974cedb7909855e38ce6a44c32d73c55ef023e73`.

## Fixed observable and current sigma paths

For each same-system/SVID/signal consecutive raw pair, the fixed residual is

```text
e_k = (P_k - P_prev)
      - 0.5 * (PseudorangeRate_prev + PseudorangeRate_k) * dt_seconds
```

The pair uncertainty and normalized residual are

```text
u_pair_mps       = sqrt(u_prev_mps^2 + u_current_mps^2)
integrated_sigma = 0.5 * dt_seconds * u_pair_mps
normalized_abs   = abs(centered_e_k) / integrated_sigma
```

Pairs require `0 < dt <= 1.5 s`, unchanged hardware-clock discontinuity and
segment, valid Android code/transmit state, finite in-range Phase25 P, and
clear existing multipath/C/N0 masks.  No new mask or weighting is applied.

The candidate native rule, if later authorized, is fixed as
`max(existing_doppler_sigma_mps, raw_rate_uncertainty_mps)` with coefficient
one and no cap, for FGO Doppler factors only; SPP is out of scope.  The audit
reports actual impact against the current 0.2 m/s undifferenced sigma lower
bound and also computes the source-exact SNR-derived path
`10^(-(Cn0DbHz-p85_band)/20)/12` when upstream quality is enabled.  The current
default has upstream quality disabled.  No elevation or navigation is
inferred, so the fixed impact number is a conservative lower-bound proxy.

## One-shot observations

The four pinned raw files were each read exactly once in one process.  All
routes have the candidate header and 100% finite positive uncertainty among
eligible transitions.  Every route populated all four fixed uncertainty bins
and passed the routewise relation thresholds:

| Route | Raw rows | Epochs | Eligible transitions | Satellites | Unsupported rows | Spearman | High/low p95 excess (m) | Ratio | Fixed affected | SNR-enabled affected | Proxy p95 (m) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| MTV-a | 87,705 | 2,159 | 13,431 | 7 | 266 | 0.412972 | 9.074598 | 3.150780 | 0.225151 | 0.225151 | 0.659250 |
| MTV-h | 112,833 | 3,140 | 15,630 | 7 | 435 | 0.437863 | 12.350815 | 4.098093 | 0.245617 | 0.245745 | 0.728500 |
| LAX-t | 51,243 | 1,466 | 11,732 | 11 | 247 | 0.465283 | 11.940191 | 3.511166 | 0.349472 | 0.386635 | 0.728500 |
| MTV-u | 35,810 | 1,102 | 9,612 | 11 | 138 | 0.496254 | 12.525919 | 4.115355 | 0.312526 | 0.312630 | 0.757000 |

The route medians of absolute centered closure residual were 1.346134 m,
1.388083 m, 1.841842 m, and 1.746348 m (aggregate 1.567215 m; route-median
MAD 0.200107 m).  Pairwise route-median distances are retained in the result
as descriptive values only.

## Gate failures and decision

The routewise monotonic relation is repeatable: leave-one-route-out Spearman
is 0.427705–0.461928 and every pooled high/low p95 difference remains
positive.  The relation is not calibrated as the candidate sigma, however.
Normalized residual medians are **19.7973 / 15.4834 / 16.0525 / 15.5422** and
the normalized-absolute-residual fraction at or below one is only
**0.120468 / 0.164044 / 0.111575 / 0.099563**, failing the frozen median range
`[0.25,4.0]` and lower fraction `0.05`–`0.95` requirement.  Composition also
fails independently: all four routes contain only one supported signal family,
`GALILEO:GAL_E1`, below the frozen minimum of two.  This prevents separating a
signal/constellation-specific residual structure from a general rate-
uncertainty mechanism.

The result is **no-go-rate-uncertainty-not-stable-or-material**.  The strongest
finding is a large, repeatable positive uncertainty/residual relation, but its
normalized scale is far above one and it is entirely single-family, so no
native sigma floor is authorized.  The exactly one next source-supported raw
factor is **Android per-satellite `Cn0DbHz`/Doppler residual calibration**.
`Cn0DbHz` is already parsed and participates in existing raw masks/SNR paths;
this phase does not implement a new C/N0 rule.  Phase 43 remains champion and
Phase 51 remains experimental.  The `0.782` target is not evaluated without
truth.

## Source audit

The pinned source contract shows that the adapter parses ordinary
`PseudorangeRateMetersPerSecond`, but not
`PseudorangeRateUncertaintyMetersPerSecond`; `Observation` does not retain the
candidate field and FGO does not consume it.  Existing undifferenced and
single-difference Doppler paths use configured sigma, while the optional
upstream-quality path computes SNR-derived Doppler sigma.  The fixed defaults
are 0.2 m/s for each single-observation path.  All source hashes are recorded
in the machine-readable result.

## Read accounting and artifacts

Read accounting is one process with four raw GNSS reads (one per route), and
zero truth, navigation, solver, trajectory, coordinate, IMU, MAT,
validation/holdout, archive, rematerialization, Phase45/Phase56 metric-payload,
device-WLS, `SvPosition`/`SvElevation`, Kaggle, or token reads.  No C++ change or
post-read tuning occurred.

The machine-readable record is
[`smartphone_r5_phase57_pixel5_rate_uncertainty_result_v1.json`](records/smartphone_r5_phase57_pixel5_rate_uncertainty_result_v1.json).
Output artifacts under
`output/smartphone-r5/phase57-pixel5-rate-uncertainty-v1/` are:

| Artifact | Bytes | SHA-256 |
|---|---:|---|
| `phase57_pixel5_rate_uncertainty.json` | 85,732 | `2439e9fdec20c91e4285b5788f27ba4d3632c8d0b27fa6ecfe1e70b0ec05d680` |
| `phase57_pixel5_rate_uncertainty.routes.json` | 68,235 | `ab93014d8669065a6c2df7e1e8a450cc0a827494f73fc3472e0a93f0c91d2e2b` |
| `phase57_pixel5_rate_uncertainty.events.json` | 8,332,563 | `c25150110f80519c00abba0d8bbe5044de9dcdc85183376fd157a94f56aea36d` |
| `phase57_pixel5_rate_uncertainty.manifest.json` | 1,849 | `385cbcabacdb6b215a04b5c43d271c2ea40c53733468b2bdb489165e60e8e5d7` |

The focused Python suite passed 6 tests, `py_compile` passed, and
`--verify-freeze` passed before the raw audit.  CMake registers
`python_smartphone_phase57_pixel5_rate_uncertainty_audit_tests`.
