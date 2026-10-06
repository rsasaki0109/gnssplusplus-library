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

The guard builds and its seven native mechanism tests pass. A 120-epoch PPC
smoke with the guard disabled preserves every POS numeric field against the
frozen native baseline, including status. The six-run off/on comparison and
adoption decision are pending. The default remains off until that comparison
is reviewed.
