# Phase247 U evaluation boundary, before payload access

Phase246 structural result is sealed. Candidate has 1101 published rows,
from 1102 raw epochs with warmup exclusion. Historical Phase44 U metadata
records 1102 truth rows, not H's matching 3139/3139 pattern.

Truth identity from existing `output/smartphone-r5/phase44-pixel5-development-accuracy-v1/truth_materialization.json`:

- Path: `output/smartphone-r5/phase44-pixel5-development-accuracy-v1/truth/2023-03-08-21-34-us-ca-mtv-u/pixel5/ground_truth.csv`
- SHA256: `7f27caff1f87f4e43821b8efdbbbc87b75b95c0820291cc906a21ea5aee4f080`
- Bytes: 108045; rows: 1102 (Phase44 accuracy record).
- Archive member: `dataset_2023/train/2023-03-08-21-34-us-ca-mtv-u/pixel5/ground_truth.csv`
- Archive CRC32: bc251725.

No truth CSV read in this audit. Future evaluation must retain the entire
truth payload, report unmatched truth coverage, and must not call the
candidate full-coverage merely because all non-warmup raw keys are present.
Do not delete the extra truth row, interpolate, hold, or offset predictions.
The identity of the unmatched key remains to be checked; do not assert it
is the warmup key solely from aggregate row counts.

Inspected Phase203 score_payloads delegates to Phase199/189. The actual
Phase189 score kernel accepts route and row counts from supplied pins,
unlike its historical H manifest verifier. Use only that scoring path,
preserve its exact-key metric and report truth_domain_exact even if false.
Freeze U metadata and route-specific structural gates separately, without
an H-score improvement comparison. No native rerun or parameter tuning.

Scope: previously used U development transfer, not independent validation
or leaderboard evidence. Missing full truth coverage, if confirmed, is a
remaining requirement rather than an acceptable completion substitute.
