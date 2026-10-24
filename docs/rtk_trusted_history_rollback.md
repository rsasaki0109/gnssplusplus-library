# Trusted-position history and candidate rejection

Two reproducible state-management defects were found while investigating the
rejected FIX recovery experiments. The repair changes no acceptance threshold,
integer search recipe or default runtime option.

## Same-epoch records collapsed the velocity history

`generateSolution()` remembers a provisional FLOAT, and the epoch path can
remember that FLOAT again or replace it with a FIX at the same observation time.
Previously every trust refresh shifted the current position into the previous
slot. The two trust samples then had identical timestamps; `CV_PREDICT` could
not derive a velocity from their difference and fell back to zero.

The history now advances only at a later timestamp. Multiple records within an
epoch replace the current position while retaining the previous distinct epoch.
For example, positions y=0 at TOW 100 and y=1 at 101, followed by a FIX y=1.2 at
101, retain the TOW 100 predecessor and yield 1.2 m/s rather than a zero-span
history. Nonmonotonic observation handling is not redesigned by this change.

## Rollback omitted the previous trust sample

The existing rollback restored current trusted position/time/presence but omitted
previous trusted position/time/presence. A rejected provisional FLOAT could
therefore change the velocity history even when the epoch returned an SPP
fallback. Partial rollback after FIX validation/post-validation rejection had
the same omission.

The epoch snapshot now retains both samples, and all three rollback paths use
one restoration of that complete trust history. Kept outputs continue through
the existing recording paths. Integer search, hold application and filter
update equations are unchanged.

## Regression evidence

Two native regressions fail against the pre-repair libraries and pass after the
repair:

- Same-epoch FLOAT/FIX replacements retain the previous distinct epoch and the
  expected constant-velocity estimate.
- A real synthetic RTK epoch deliberately rejected by the FLOAT/SPP divergence
  gate returns SPP and restores both trust samples, including the case where no
  previous sample was present.

Existing synthetic full/subset/multi-frequency integer-search cases additionally
assert that `resolveAmbiguities()` leaves the FLOAT state and covariance exactly
unchanged before acceptance. These tests constrain the candidate search boundary;
they do not prove rollback for every optional mode or every incorrect ambiguity
basin. Full Release build and all 100 core CTest entries pass, with the local
Eigen include directory supplied for the two helper-compilation tests.

## Public-data evaluation

The comparison uses the same Odaiba/Shinjuku inputs, surveyed base, exact GPS
week/TOW matching and scoring-only ground truth as the
[continuity investigation](urbannav_doppler_consensus.md). Two complete drives
were replayed before/after with default settings and with the frozen
`--max-fixed-anchor-age 5 --max-fixed-doppler-consensus 3` experiment: eight runs.
All four before/after POS pairs are byte-identical. In these recipes the
`LEGACY` trust policy does not consume the previous trust sample. The repair
therefore does **not** explain or remove the wrong FIX populations observed in
those experiments, and it does not justify enabling their options by default.

The follow-up also compares the existing `--float-trust-policy cv-predict` mode
on both complete drives, preserving native argv and executable/input/output
hashes. This policy consumes the repaired history and must be assessed separately
from default-output parity. Both drives have already been inspected and share a
collection/receiver family; no held-out generalization claim is made.

The [complete audit](urbannav_trusted_history_evaluation.json) retains all twelve
full runs and their provenance. The separate CV comparison is descriptive:

| Drive | Version, CV policy | FIX epochs | FIX >0.5 m / >2 m / >3 m | Missing epochs | Horizontal P95 m |
| --- | --- | ---: | ---: | ---: | ---: |
| Odaiba | Before | 1,298 | 0 / 0 / 0 | 1,809 | 11.3394 |
| Odaiba | After | 1,298 | 0 / 0 / 0 | 1,827 | 16.5589 |
| Shinjuku | Before | 122 | 0 / 0 / 0 | 3,156 | 11.1764 |
| Shinjuku | After | 122 | 0 / 0 / 0 | 3,601 | 9.7648 |

The correct-FIX population is unchanged, but availability deteriorates on both
drives and horizontal P95 worsens on Odaiba. This is a state-invariant repair,
not evidence to enable `cv-predict` or claim improved accuracy/recovery. The
policy remains opt-in and is not recommended as the FIX recovery recipe. Its
existing predictor needs renewed validation with a correct time-separated
history; measurements previously made with the collapsed history do not
validate its actual constant-velocity behavior. No parameter was retuned after
this comparison.

The branch's earlier gate-coverage and rejected-experiment records are historical
checkpoints. The repaired solver executable differs from that checkpoint, while
all tested LEGACY-recipe POS files remain byte-identical. Runtime/source/input/
output hashes distinguish these claims in the audit.
