# Runtime FIX containment and recovery

`gnss_solve --fix-recovery` enables an optional residual guard after the RTK
candidate is produced. It uses the candidate's runtime diagnostics, with no
reference trajectory, external score or covariance-confidence shortcut.
The option defaults to off. It adds no output buffering or future-observation
dependency; a demoted candidate keeps its coordinates and reports `FLOAT`.
Demotion by itself does not improve position error.

Add `--fix-recovery-log new.csv` to record every processed decision. That
option enables the guard; `--no-fix-recovery` can disable it later on the command
line. The advanced options are listed by the native solver's full help.

## State transitions

| State | Trigger and behavior |
| --- | --- |
| NORMAL | Ordinary FIX candidates pass. |
| SUSPECT | A FIX with prefit RMS above 10 m, at least 50% suppressed rows and ratio below 6 is immediately demoted. Two consecutive matches enter quarantine. A nonmatching next epoch cancels the streak. |
| QUARANTINE | Post-suppression RMS above 4 m or NIS per observation above 50 on a FIX enters quarantine immediately. Entry requests one primary-filter reset. Persistent bad evidence does not request repeated resets. FIX candidates remain demoted. |
| RECOVERY | Consecutive clean FIX candidates accumulate. Missing, FLOAT or insufficient diagnostic evidence returns to quarantine. Five clean candidates restore NORMAL. |

A clean candidate requires at least eight satellites, ratio at least 3,
prefit RMS at most 5 m, post-suppression RMS at most 2 m, NIS per observation
at most 10 and suppressed-row fraction at most 20%. Position and diagnostics
must be finite, residual/NIS values nonnegative, and update row count positive.
A gap over one second clears consecutive evidence; it cannot complete recovery.
Observation timestamps must be strictly increasing.

The library header exposes configurable thresholds through `FixRecoveryGuard`.
The command uses these frozen values for the development comparison. Additional
threshold tuning requires a separately recorded recipe and evaluation.

## Evaluation contract

Run an identical native binary and input population with the guard off and on.
Record exact argv, binary/source/input hashes, runtime, POS and decision CSV.
The source-tree replay command supports an RTK-only comparison:

```sh
python apps/gnss.py ppc-native-replay --dataset-root /datasets/PPC-Dataset \
  --build-dir build --paths rtk --output-dir output/native-baseline
python apps/gnss.py ppc-native-replay --dataset-root /datasets/PPC-Dataset \
  --build-dir build --paths rtk --fix-recovery --output-dir output/native-fix-recovery
```

Keep source contents and the selected build unchanged between these commands.
The guard option records its decision CSV and recipe flag in the provenance
manifest. It does not apply to the fusion recipe. `--compare-to` is for identical
recipe repeatability and rejects an off/on pair; use the accuracy audit below.

`scripts/analysis/audit_ppc_native_integrity.py` labels existing outputs offline:

```sh
python scripts/analysis/audit_ppc_native_integrity.py \
  --dataset-root /datasets/PPC-Dataset \
  --replay-dir output/native-baseline \
  --candidate-dir output/native-fix-recovery \
  --output-json output/native-fix-recovery-audit.json
```

The audit retains both 0.5 m and 2 m 3D wrong-FIX populations, baseline-correct
FIX losses, wrong-FIX removal, missing outputs, actual accuracy recovery,
recovery delays and right-censored events. It keeps horizontal P95 and runtime
from the native replay scorer. Its descriptive runtime classes are not the
guard's decision rule or a guarantee of position accuracy.

An off/on audit requires both replay manifests to have passed a full run.
It verifies source contents, executable hashes, input population, runtime
libraries, CMake settings and effective solver arguments; only the recovery
flag and output destinations may differ. It rechecks recorded artifact hashes
before labeling. An equal epoch count alone is insufficient provenance.

The default-adoption gate for this development comparison is conservative:
for every run, neither wrong-FIX threshold may increase; baseline-correct FIX
loss must stay within 1%; missing/unmatched output count and horizontal P95
must not increase; recovery-delay P95 and right-censored event counts must not
increase at either threshold. At least one wrong-FIX population must decrease.
All runtime measurements are retained, with concurrent-job contention noted.
A failed gate leaves the feature an opt-in diagnostic experiment. Demoting a
wrong coordinate to FLOAT is never counted as a correct position recovery.

The new Tokyo run1 baseline contains 784 wrong FIX epochs above 0.5 m, including
247 without any of the audit's named runtime warnings. At 2 m the counts are
217 and 77. These are development-data observations: the guard cannot promise
to detect all wrong ambiguity basins from ordinary residuals.

## Six-run decision: NO_GO

The full off/on experiment used frozen source `8235764f`, the same executable,
inputs, configuration, runtime libraries and algorithm environment. Both replay
manifests passed. The paired audit verified and rehashed their provenance before
labeling. [ppc_fix_recovery_evaluation.json](ppc_fix_recovery_evaluation.json)
retains all run/threshold summaries, correct-FIX losses, accuracy recovery P50/P95,
right-censored counts, missing outputs, runtime and runtime-state counts. It also
pins the complete event-level audit and both replay manifests by SHA-256.

All six runs fail at least one predeclared adoption condition. The decision is
**NO_GO; keep the default off**. No thresholds were retuned after this result.

| Run | Wrong FIX >0.5 m OFF / ON | Wrong FIX >2 m OFF / ON | Baseline-correct FIX loss at 2 m | Horizontal P95 m OFF / ON | Missing/unmatched OFF / ON |
| --- | ---: | ---: | ---: | ---: | ---: |
| Tokyo 1 | 784 / 406 | 217 / 225 | 7.91% | 1.380763 / 1.404645 | 1702 / 1777 |
| Tokyo 2 | 133 / 131 | 44 / 44 | 2.09% | 0.870171 / 0.875513 | 706 / 692 |
| Tokyo 3 | 341 / 131 | 34 / 34 | 3.25% | 0.883091 / 1.063567 | 1510 / 1496 |
| Nagoya 1 | 837 / 173 | 314 / 64 | 8.07% | 0.755371 / 1.251883 | 903 / 898 |
| Nagoya 2 | 434 / 51 | 27 / 12 | 20.04% | 5.198496 / 5.746931 | 2512 / 2575 |
| Nagoya 3 | 694 / 56 | 288 / 20 | 38.29% | 10.641817 / 10.638017 | 1892 / 1999 |

Aggregated wrong-FIX counts decrease from 3,223 to 948 at 0.5 m and from 924
to 399 at 2 m. Baseline-correct FIX losses are 1,608/36,833 (4.37%) and
3,339/39,132 (8.53%), respectively. These exceed the 1% gate. Tokyo 1 also
increases wrong FIX above 2 m; five runs increase horizontal P95. A lower wrong
FIX count therefore does not justify default adoption or an accuracy claim.
The 28 quarantine entries all reach runtime clean-candidate recovery; that is
separate from the offline accuracy recovery and censoring records.

With the guard disabled, every POS numeric field including status matches the
original native baseline on all six complete RTK runs, as recorded in
[ppc_native_default_parity_verification.json](ppc_native_default_parity_verification.json).
Seven native mechanism tests and seven audit tests pass. The guard remains an
explicit opt-in research feature, with no held-out integrity guarantee.
