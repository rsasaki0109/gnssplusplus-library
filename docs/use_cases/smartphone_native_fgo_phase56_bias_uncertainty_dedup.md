# Phase 56 Pixel5 BiasUncertaintyNanos deduplication

Phase 56 is a sealed-artifact/source evidence-map recovery after the Phase 55
proposal of Android `BiasUncertaintyNanos` / receiver-clock uncertainty.  It
does not open a new raw file.  The question is whether that field is a new
satellite-specific geometry factor or the same receiver-clock common-mode lane
already audited in Phase 46.  Truth, navigation, solver, coordinate, IMU,
MAT, validation/holdout, archive, and prior metric payload inputs are all
excluded.

The Phase 56 freeze was pushed first in `297d1de`; its SHA-256 is
`b6242ed176fbe124b523d635575572f12d25494082f5c57c837d3c3c5fe35567`.
The evaluator, tests, CMake registration, and manifest were sealed and pushed
in `41d6364`.  The evaluator manifest SHA-256 is
`81f29b12964236a07fc30a6c3de4512988f8213b2abd001a0575b76610932efc`.

## Deduplication contract

Android defines `BiasUncertaintyNanos` as an uncertainty associated with the
`GnssClock` receiver clock bias, in nanoseconds.  It is not an independent
per-satellite pseudorange-rate uncertainty.  Phase 46's sealed raw audit found
the following across the same four Pixel5 routes:

| Evidence | MTV-a | MTV-h | LAX-t | MTV-u |
|---|---:|---:|---:|---:|
| BiasUncertainty rows | 87,705 | 112,833 | 51,243 | 35,810 |
| BiasUncertainty median (ns) | 15.780499 | 17.204216 | 19.740919 | 18.550338 |
| BiasUncertainty p95 abs (ns) | 22.869957 | 25.019744 | 34.437445 | 27.501060 |
| same-epoch receiver clock spread | 0 m | 0 m | 0 m | 0 m |
| same-epoch signal-group spread | 0 ms | 0 ms | 0 ms | 0 ms |
| geometry-changing non-common TOW effect | 0 m | 0 m | 0 m | 0 m |

Phase 46 was already a no-go for receiver-clock correction because its clock
structure was common mode with no satellite/signal spread or geometry-changing
component.  Therefore `BiasUncertaintyNanos` is deduplicated as the uncertainty
of that rejected receiver-clock gauge, and no second raw audit or correction is
authorized.  The Phase 46 output does not separately serialize
BiasUncertaintyNanos same-epoch dispersion; this conclusion is explicitly a
source-semantic deduplication supported by the sealed receiver-clock evidence,
not a claim that an unreported per-row dispersion table was observed.

## Current source plumbing

The pinned source contract proves that the current Android loader parses
`PseudorangeRateMetersPerSecond` and `BiasUncertaintyNanos`, but does not parse
`PseudorangeRateUncertaintyMetersPerSecond`.  The `Observation` model retains
the rate but neither uncertainty field.  FGO does not consume raw rate
uncertainty: undifferenced and single-difference Doppler paths use their fixed
configured sigma (with the existing upstream SNR-derived path when enabled).
The Android-rate-to-RINEX-Doppler sign contract remains pinned.  Source hashes
are recorded in the machine-readable result and are checked by the evaluator.

The exactly one next nonduplicate factor is Android
`PseudorangeRateUncertaintyMetersPerSecond`, an opt-in native FGO Doppler sigma
floor candidate:

```text
sigma_doppler = max(existing_doppler_sigma_mps,
                    raw_rate_uncertainty_mps)
```

The coefficient is one and there is no cap or Phase 56 implementation.  A
future phase must separately freeze source availability, route stability,
identifiability, and estimator adoption before considering implementation.

## Decision and accounting

The Phase 56 result is **phase56-no-go-bias-uncertainty-duplicate-common-mode**.
No native correction, raw audit, truth score, or `0.782` claim was made.  The
Phase 43 champion and Phase 51 experimental option remain preserved.

The static evaluator read each pinned Phase 46 artifact and the Phase 55 policy
record once in one process.  Phase 56 new raw GNSS reads, truth reads,
navigation reads, solver invocations, coordinate inputs, IMU reads, archive
reopens, rematerialization, validation/holdout, MAT, and Kaggle/token access
were all zero; Phase 46 raw files were not reopened.

Artifacts:

| Artifact | Bytes | SHA-256 |
|---|---:|---|
| `phase56_bias_uncertainty_dedup.json` | 6,029 | `321e775fbc9e03d8001a38dd3ba1c9d31d0d54cead0cabe0bb77e923190d3467` |
| `phase56_bias_uncertainty_dedup.manifest.json` | 1,353 | `01ba15ebfb58d49e095c201e8577071e0bc07e95cfb455d89b8b7a104a787334` |

The machine-readable record is
[`smartphone_r5_phase56_bias_uncertainty_dedup_result_v1.json`](records/smartphone_r5_phase56_bias_uncertainty_dedup_result_v1.json),
and the output directory is
`output/smartphone-r5/phase56-pixel5-bias-uncertainty-dedup-v1/`.
The focused Python suite passed 4 tests, `py_compile` passed, and
`--verify-freeze` passed before the sealed-only audit.  CMake registers
`python_smartphone_phase56_bias_uncertainty_dedup_tests`.
