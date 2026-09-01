# Phase78 result: sealed-artifact reclassification passed

Phase78 reclassified the completed Phase77 signal-bias/base-pc composition
using only its immutable failure record and candidate run-1/run-2 artifacts.
It did not rerun native code and read no raw GNSS/IMU/navigation/base/truth,
MAT, validation, holdout, Kaggle, or archive input.

All four Pixel5 routes passed the fixed artifact gates: candidate hashes and
repeats were identical, prediction keys/rows were unchanged between repeats,
retained finite-`pc` fraction was exactly 1.0, retained pseudorange factors
matched the frozen telemetry, TDCP factors were finite with built=inserted,
signal-bias states/factors and estimates were finite/material, coordinates
were valid, speed exceeded 70 m/s zero times, and output epochs/IMU intervals
were repeat-invariant. No route-specific selection was made.

This pass only authorizes a separate development-accuracy freeze for the
immutable Phase77 candidate; it is not an accuracy score and makes no `0.782`
claim. Phase43 remains champion and the Phase77 option remains experimental.

See the [sealed result record](records/smartphone_r5_phase78_phase77_structural_reclassification_result_v1.json),
[freeze](records/smartphone_r5_phase78_phase77_structural_reclassification_freeze_v1.json),
[manifest](records/smartphone_r5_phase78_phase77_structural_reclassification_manifest_v1.json),
and [runner](../../apps/commands/benchmarks/gnss_smartphone_phase78_phase77_structural_reclassification.py).
