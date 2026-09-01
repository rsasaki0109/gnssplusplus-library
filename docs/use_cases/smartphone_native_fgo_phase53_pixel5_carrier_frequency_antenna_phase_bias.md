# Phase 53 Pixel5 carrier-frequency/antenna phase-bias audit

Phase 53 is a truth-free audit of the single raw factor selected by Phase 52:
the Android per-satellite carrier-phase accumulated-delta-range (ADR)
carrier-frequency/antenna phase-bias residual.  It did not implement a
correction, invoke a solver, read truth or navigation, use coordinates, or
reuse a prior metric payload.  The four frozen Pixel5 `device_gnss.csv` files
were read once each by one evaluator process.

The freeze was resealed and pushed first in `2d84e99` (freeze SHA
`618b351764876fb0eafd3c2431076acf0ac8fd43670ea1a5e33f8d7ee43dce1a`).  The
evaluator, focused tests, CMake registration, and manifest were then sealed in
`c6205ba`; the evaluator manifest SHA is
`b810c13ad3696cea29511e47074b6621e71283c0b30ee058313894ed88198b2c`.

## Fixed raw-only contract

For each same-system/SVID/signal adjacent pair, the evaluator requires
`0 < dt <= 1.5 s`, unchanged hardware-clock discontinuity count and segment,
finite Phase 25 raw-clock pseudorange in the frozen range, existing
code/C/N0/multipath masks clear, and Android ADR `VALID` with `RESET` and
`CYCLE_SLIP` clear.  Excluded reset, slip, invalid, gap, HCDC/segment, and
existing-mask pairs remain separate accounting groups.

The source-sign contract is fixed by the Phase 41 audit and Android's
`GnssMeasurement` definition: Pixel5 preserves the adapter ADR sign, and
Android's RF carrier phase has the opposite sign (`ADR = -k * carrier phase`).
With `c = 299792458 m/s`,

```text
phi_k = signed_ADR_k * CarrierFrequencyHz_k / c
f_ref = 0.5 * (f_prev + f_k)
r_freq = (phi_k - phi_prev) * c / f_ref
         - 0.5 * (PseudorangeRate_prev + PseudorangeRate_k) * dt
r_control = signed_ADR_k - signed_ADR_prev
            - 0.5 * (PseudorangeRate_prev + PseudorangeRate_k) * dt
r_leak = (phi_k - phi_prev) * c / f_ref
         - (signed_ADR_k - signed_ADR_prev)
```

The signed and median-centered relations are retained per endpoint
`utcTimeMillis + HardwareClockDiscontinuityCount`, so a receiver common mode is
not presented as a satellite-specific effect.  `CarrierPhase` and
`CarrierCycles`, when present, are diagnostic source-sign checks only;
`CarrierPhase` is fractional and no full ambiguity is inferred.  A direct
Android antenna phase-center/bias field would be required for an antenna
interpretation.  Missing fields are reported as missing rather than set to
zero.

## Static source and field availability

The frozen source audit confirms that the Android adapter parses ADR and
`CarrierFrequencyHz`, preserves Pixel5 ADR sign, and that the TDCP contract
contains loss-of-lock and clock-boundary handling.  `Observation` exposes only
the generic/default antenna PCO slot; neither the Android adapter source nor
its header parses an Android antenna phase-center/bias field.  The FGO source
uses the observation wavelength helper; the evaluator's literal search for a
direct `source_carrier_frequency_hz` use in `fgo_problems.cpp` is false because
that path is indirect, and is not treated as positive adoption evidence.
All nine static source hashes are recorded in the result record and freeze.

## One-shot observations

The machine-readable output is
`output/smartphone-r5/phase53-pixel5-carrier-frequency-antenna-phase-bias-v1/`.
The result record is
[`smartphone_r5_phase53_pixel5_carrier_frequency_antenna_phase_bias_result_v1.json`](records/smartphone_r5_phase53_pixel5_carrier_frequency_antenna_phase_bias_result_v1.json).

| Route | Raw rows | Epochs | All pairs | Ordinary pairs | Satellites | Signal/frequency groups | Unsupported rows | Frequency-aware p95 (m) | Control p95 (m) | Leakage p95 (m) | Antenna fields |
|---|---:|---:|---:|---:|---:|---|---:|---:|---:|---:|---|
| MTV-a | 87,705 | 2,159 | 87,645 | 7,508 | 7 | GAL_E1 | 266 | 0.059884 | 0.059884 | 6.49e-12 | none |
| MTV-h | 112,833 | 3,140 | 112,778 | 8,763 | 6 | GAL_E1 | 435 | 0.038575 | 0.038575 | 1.07e-11 | none |
| LAX-t | 51,243 | 1,466 | 51,191 | 4,176 | 10 | GAL_E1 | 247 | 0.062970 | 0.062970 | 5.54e-12 | none |
| MTV-u | 35,810 | 1,102 | 35,767 | 3,728 | 11 | GAL_E1 | 137 | 0.054942 | 0.054942 | 7.13e-12 | none |

The ordinary-pair coverage and clean retention gates passed, but every route
has exactly one observed signal/frequency group (`GALILEO:GAL_E1`, nominal
1,575,420,000 Hz), zero finite direct-phase pair values, and no finite antenna
phase-center/bias field.  Frequency-aware and constant-frequency residuals
are numerically identical: route median absolute residuals are
`0.010478932`, `0.005052213`, `0.010934773`, and `0.007036717 m`, while the
frequency-vs-control p95 excesses are approximately `1.19e-13`, `-6.54e-13`,
`1.04e-12`, and `-5.21e-14 m`.  Frequency leakage p95 is only
`5.54e-12–1.07e-11 m`; route satellite-median MAD is
`0.000585–0.003028 m`; routewise Spearman association is `0.0` in all four
routes.  Leave-one-route-out p95 excess is between `-2.53e-12` and
`1.44e-12 m`, and no fold has a positive frozen material effect.

The six pairwise route median distances range from `0.000455842` to
`0.005882561 m`.  These small route differences do not identify a
carrier-frequency or antenna phase-bias mechanism and are not used as a
correction signal.

## Gate disposition

The result is **no-go-carrier-frequency-antenna-phase-bias-not-identifiable**.
The antenna-source, two-group, frequency-materiality, relation-identifiability,
and direct-phase-source gates fail.  The strongest finding is the complete
absence of a finite Android antenna phase-center/bias source field on all four
routes, reinforced by a one-group-only population and effectively zero
frequency leakage.  No native FGO/TDCP correction is authorized or
implemented.  Phase 43 remains the champion and the Phase 51 sigma-floor
option remains experimental.  The `0.782` target is not evaluated without
truth.

The result also honestly reports `raw_input_integrity=false`.  This is not a
hash, finite-core, duplicate, or ordering failure: all frozen SHA/byte checks,
core relation fields, duplicate epoch checks, and nonmonotonic epoch checks
passed.  The evaluator defines that gate as an AND including
`unsupported_signal_rows == 0`; the raw files contain 266, 435, 247, and 137
unsupported rows respectively.  Those rows are therefore retained in the
integrity accounting and are not silently relabeled.  This secondary gate
failure is preserved in `raw_input_integrity_detail` of the result record and
does not change the primary antenna/frequency no-go finding.

## Accounting, artifacts, and tests

Read accounting is one process and four raw CSV reads total (one per route).
Truth, prior truth/metric payloads, BRDC navigation, solver/trajectory,
correction implementation, validation/holdout, archive reopen,
rematerialization, Kaggle/token, MAT, device WLS, `SvPosition`/`SvElevation`,
and raw IMU reads are all zero.  The event table is empty because no ordinary
pair met the frozen candidate threshold.

| Artifact | Bytes | SHA-256 |
|---|---:|---|
| `phase53_pixel5_carrier_frequency_antenna_phase_bias.json` | 2,427,740 | `d158cbc6f9a8591552bbb160bfc1c760fce831411fed5c4f0101476fac871d4e` |
| `phase53_pixel5_carrier_frequency_antenna_phase_bias.routes.json` | 2,414,155 | `906ecd38d914fdb59da58294253eaeb1117e79a15d06d37762b7a06426ecabbc` |
| `phase53_pixel5_carrier_frequency_antenna_phase_bias.events.json` | 147 | `fe23e0dacf156bc5d512aec5a63ebc5f6c5c9c12b662d8192ac9187de32881b5` |
| `phase53_pixel5_carrier_frequency_antenna_phase_bias.manifest.json` | 2,135 | `014a226274ce98a9825611dc6b8589c25f9d96fec40441d9c5ff9cffbaa312fd` |

The focused suite passed 6 tests before the raw seal, `py_compile` passed, and
`--verify-freeze` passed without raw/truth reads.  CMake registers
`python_smartphone_phase53_pixel5_carrier_frequency_antenna_phase_bias_audit_tests`.

No correction is authorized.  The exactly one next source-supported raw
physical factor is **Android per-satellite accumulated-delta-range uncertainty
(`AccumulatedDeltaRangeUncertaintyMeters`)**.  It is recorded for a future
phase only; this phase does not implement it.
