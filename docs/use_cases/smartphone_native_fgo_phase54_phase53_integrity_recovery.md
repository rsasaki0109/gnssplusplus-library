# Phase 54 Phase53 integrity recovery

Phase 54 is a scorer-only recovery of a presentation-contract mismatch in the
Phase 53 raw audit.  It does not reopen any Phase53 raw `device_gnss.csv`,
truth, navigation, solver output, trajectory, coordinate, or prior metric
payload.  It reads only the immutable Phase53 output manifest, result, routes,
and empty events table once each, verifies their hashes, and recomputes the
integrity predicates in memory.  The physical Phase53 no-go evidence is
rechecked and preserved; no physical gate, metric, algorithm, or authorization
is changed.

The Phase54 freeze was pushed first in `f716c32` (SHA
`82d4235b4b7cfc8d0536ecc615d92be535c3bf8b6ba4b1369e40994675e0ed00`).  The
scorer, focused tests, CMake registration, and evaluator manifest were sealed
and pushed in `2fcd864`; the manifest SHA is
`eb460cc9b6c2901d379085e27d98fdd2081380cef2e3ab5357de521972705d51`.

## Corrected integrity contract

Phase 53's frozen `raw_input_integrity` contract was the conjunction of:

```text
exact frozen SHA/byte checks
AND all core relation fields finite
AND nonmonotonic epoch count == 0
AND duplicate epoch-key count == 0
```

Its evaluator additionally inserted `unsupported_signal_rows == 0` into that
AND gate, although unsupported rows were declared excluded/informational in
the freeze.  Phase 54 corrects only this mismatch: unsupported rows remain
reported, but are not a raw-input-integrity gate.  The Phase53 output remains
byte-for-byte immutable.  Phase 54 also recomputes pair-reason, state,
signal/frequency, satellite, route-median, aggregate, LOO-fold, event-count,
header-map, and output-manifest presentation checks from the sealed artifacts.

## Recovery result

The machine-readable result is
`output/smartphone-r5/phase54-phase53-integrity-recovery-v1/`, with the
record in
[`smartphone_r5_phase54_phase53_integrity_recovery_result_v1.json`](records/smartphone_r5_phase54_phase53_integrity_recovery_result_v1.json).
All four corrected route integrity checks pass despite informational
unsupported-row counts **266 / 435 / 247 / 137**.  Core finite, duplicate,
nonmonotonic, group-count, route-retention, aggregate, LOO, event, header,
and artifact-map checks are all true.

The physical decision remains the exact Phase53 decision:

- no finite Android antenna phase-center/bias field on any route;
- one `GALILEO:GAL_E1:1575420000Hz` group per route;
- frequency leakage p95 `6.49e-12`, `1.07e-11`, `5.54e-12`, and `7.13e-12 m`;
- frequency-vs-control p95 excess `1.19e-13`, `-6.54e-13`, `1.04e-12`, and
  `-5.21e-14 m`;
- routewise Spearman `0.0` for all four routes; and
- no native correction authorization.

The corrected result is therefore
**phase53-integrity-recovered-physical-no-go**, not a promotion or accuracy
result.  Phase 43 remains the champion and Phase 51 remains experimental.
The `0.782` target is not evaluated without truth.  The exactly one next
source-supported raw factor remains **Android per-satellite accumulated-
delta-range uncertainty (`AccumulatedDeltaRangeUncertaintyMeters`)**.

## Read accounting and artifacts

Phase 54 performed one process with one read each of the Phase53 output
manifest, result, routes, and events artifacts.  Phase53 raw-device-GNSS
rereads, raw GNSS reads, truth, navigation, solver, trajectory, validation,
holdout, archive, rematerialization, Kaggle/token, MAT, device WLS,
`SvPosition`/`SvElevation`, and output mutations are all zero.

The Phase53 input artifact hashes are independently recorded in the Phase54
freeze, result, and output manifest:

| Artifact | Bytes | SHA-256 |
|---|---:|---|
| Phase53 result | 2,427,740 | `d158cbc6f9a8591552bbb160bfc1c760fce831411fed5c4f0101476fac871d4e` |
| Phase53 routes | 2,414,155 | `906ecd38d914fdb59da58294253eaeb1117e79a15d06d37762b7a06426ecabbc` |
| Phase53 events | 147 | `fe23e0dacf156bc5d512aec5a63ebc5f6c5c9c12b662d8192ac9187de32881b5` |
| Phase53 output manifest | 2,135 | `014a226274ce98a9825611dc6b8589c25f9d96fec40441d9c5ff9cffbaa312fd` |

Phase54 output hashes are:

| Artifact | Bytes | SHA-256 |
|---|---:|---|
| `phase54_phase53_integrity_recovery.json` | 9,864 | `7c20ed795f04f726546400061df0fbbacc8f697e94410bc067be750314460ebd` |
| `phase54_phase53_integrity_recovery.manifest.json` | 1,782 | `c754314b70b68e58078ec9656f1326dac2160aa782e8be87bd685151fecb1b60` |

The focused suite passed 6 tests before the sealed recovery, `py_compile`
passed, and `--verify-freeze` passed without reading Phase53 artifacts.  CMake
registers `python_smartphone_phase54_phase53_integrity_recovery_tests`.
