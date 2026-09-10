# Native FGO carrier-compatibility result

This document is the post-score companion to the immutable
`smartphone_r5_gsdc2023_native_fgo_carrier_compatibility_freeze_v1.json`.
It records the development-only result without changing the frozen solver,
adapter contract, selector parameters, or production defaults.

## Sealed decision

The result is **No-Go**. The exact native-FGO v1 output remains the fallback
for every route, the earlier all-device float-carrier No-Go remains sealed,
and no new validation, holdout, or test truth was opened. The candidate was
selected on only three of eight routes by truth-free diagnostics. Only one
route then passed the strict four-diagnostic improvement gate; the selected
aggregate diagnostic mean regressed from `3.6546197641323706 m` to
`3.84108005650606 m`.

The authoritative machine-readable result is:

```text
docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_carrier_compatibility_result_v1.json
sha256: 8ce1de1e436d8e79d234fb65e9fc7583291a0802a6e0e78690f48f3c53366924
```

Its scoring-only recovery artifacts are:

```text
output/smartphone-r5/native-fgo-carrier-compatibility-v1/train_score_recovery.json
sha256: 78f2395e30af419bf4c4cf25ac1fd7866a60ca5f2113a3a0a318df4952c4fe3b
output/smartphone-r5/native-fgo-carrier-compatibility-v1/train_score_recovery.manifest.json
sha256: b9c6adb3b429880635d4062bd49feb1378139f487c9a0a41ec84e9a1738ac9ab
```

## Evaluation recovery and leakage boundary

The first scoring process read the eight already-authorized development
truth files and stopped on an evaluator schema mismatch: route metrics use
`kaggle_diagnostic_score_variants_m`, whereas aggregate metrics use
`mean_kaggle_diagnostic_score_variants_m`. A separate scoring-only recovery
read those same eight existing files once, without materializing them again,
and verified every truth hash before/after. The sealed record therefore
reports 16 total development truth reads (8 failed-process reads plus 8
recovery reads), unchanged truth hashes, and zero validation/holdout/test
truth reads. No solver or candidate artifact was rerun or changed.

Use the recovery command only when an explicitly authorized audit permits a
read of the already materialized development truth; it is not a new training
or tuning command:

```sh
PYTHONHASHSEED=0 python3 apps/commands/benchmarks/gnss_smartphone_native_fgo_carrier_compatibility_eval.py verify-freeze
PYTHONHASHSEED=0 python3 apps/commands/benchmarks/gnss_smartphone_native_fgo_carrier_compatibility_score_recovery.py
```

The original `train-score` process is retained as historical evidence of the
schema failure; the recovery CLI is the reproducible path for the sealed
result. Validation and holdout remain sealed, and a genuinely new validation
asset would be required before any future candidate is considered.

## Aggregate evidence

| lane | H P50 (m) | H P95 (m) | V P95 (m) | availability | diagnostic mean (m) |
| --- | ---: | ---: | ---: | ---: | ---: |
| native-FGO v1 baseline8 | 2.415335 | 4.887412 | 10.732462 | 1.0 | 3.654620 |
| carrier candidate, all routes | 2.448841 | 6.357268 | 13.299610 | 1.0 | 4.406465 |
| truth-free selected lane | 2.384906 | 5.293438 | 10.877348 | 1.0 | 3.841080 |

The four official diagnostic variants and route/family leave-out folds are
in the recovery JSON. The strict route, fold, aggregate, vertical-safety,
and promotion gates are false where required by the frozen contract.

## Adapter diagnosis

The old 214--254 m carrier residuals were reproduced as an adapter arc-boundary
problem, not a unit/sign problem. `AccumulatedDeltaRangeState=16` has no
valid bit and carries zero ADR, while the original adapter omitted the RINEX
loss-of-lock/arc-reset marker; the native `>1500 ms` gap rule could then join
unrelated arcs. A lane-local sanitizer reduced the affected truth-free
carrier RMS to about `0.0197 m` and `0.0113 m`. The original adapter and the
previous all-device No-Go artifacts were not modified. The compatibility
candidate still failed the route-group generalization gate and is not
promoted.

## Reproduction and verification

The freeze record and its companion manifest pin the archive, source,
Release binaries, role inventory, selector parameters, and truth-free route
manifest hashes. Focused contract tests are:

```sh
python3 tests/test_smartphone_native_fgo_carrier_compatibility_eval.py
python3 tests/test_smartphone_native_fgo_carrier_compatibility_score_recovery.py
LD_LIBRARY_PATH=/home/sasaki/.local/lib:/opt/ros/jazzy/lib ctest --test-dir build -j1 --output-on-failure
git diff --check
```

This lane is development evidence only. It does not authorize Kaggle access,
submission, validation reuse, holdout reuse, or a production RTK/SPP change.
