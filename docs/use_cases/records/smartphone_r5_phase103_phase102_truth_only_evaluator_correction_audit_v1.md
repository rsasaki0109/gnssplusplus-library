# Phase103 Phase102 truth-only evaluator correction audit

- Execution label: `Luna Max`
- Audit date: `2026-09-03`
- Status: `sealed-read-only-audit-qualified-for-one-truth-only-correction-freeze`
- Scope: the already completed Phase102 raw-only native outputs for MTV-A
  and LAX-T; no native rerun is permitted.

## Finding

Phase102 commit `811af21` sealed two native return-code-zero runs and two
candidate solution seals. Its evaluator failed closed before opening truth
because the Phase102 contract treated the absent top-level summary field
`mat_used` as if it had to be present and equal to `false`:

`summary/.../mat_used: expected False, got None`

The native summaries use the established Phase101 schema. They contain
`base_factors: false`, `no_base_contract: true`,
`native_pdc_state_bridge: false`, and the raw-only selector/lineage fields;
they do not define `mat_used`. Absence is therefore not evidence that may be
fabricated into a summary field. The failure occurred before any truth file
was opened, so Phase102 recorded `truth_reads: 0` and no accuracy score.

## Correction decision

Freeze exactly one correction candidate:

`phase103-phase102-truth-only-evaluator-correction-v1`

This is an evaluator-only correction over the immutable Phase102 candidate
outputs. It changes no native command, solver, graph, factor, filter, LM
configuration, initialization, metric, alignment, gate, or output. It does
not rerun either native route. The correction treats `mat_used` as an
optional summary key: if present it must be `false`; if absent it remains
absent and is not synthesized. MAT/base/PDC/precomputed-coordinate safety is
instead established from the frozen native command selector/forbidden-token
check, raw input lineage metadata, wrapper accounting, and sealed solution
metadata. Existing summary fields (`base_factors`, `no_base_contract`, and
`native_pdc_state_bridge`) remain checked when present as required by the
Phase101 contract.

The correction may open each official pinned truth exactly once, after
candidate solution hash/row/schema/finite/Earth-valid preflight. Truth is
read only by one evaluator subprocess. The native solution CSVs are the
sealed Phase102 outputs and are never replaced, repaired, or regenerated.

## Immutable authorities and metric boundary

- Phase102 raw/evaluator failure result: commit `811af21`, SHA-256
  `d4fcd2246407ad22f75dbf528ef0bbc41c391af341df66f8958b2eff0c48ada2`.
- Phase102 audit/freeze: audit `1a91a0e`, freeze `2c86aa9`, freeze SHA-256
  `e964c73160c5851a4e3f6db9497630dad7c8de5081301fa678ed52753a8b0bd1`.
- Phase102 evaluator/manifest/auth contract: commits `4a45c12` and
  `ef71224`; the correction may consume their sealed metadata but cannot
  authorize a native rerun.
- Phase101 structural authority remains result `59a60fb`; it is qualification
  evidence only and its withheld solution is not reused as a candidate.

The correction reuses Phase102's exact metric and alignment contract: key
`(phone, UnixTimeMillis)`, exact integer-key matching, exact prediction-domain
coverage `1.0`, only the pinned leading warm-up truth key may be absent,
spherical Haversine radius `6371008.8 m`, linear percentile rank
`(n - 1) * q`, route scalar `(P50 + P95) / 2`, fixed MTV-A/LAX-T unweighted
macro, finite/Earth-valid coordinates, and zero over-70-m/s transitions.
The Phase82 same-route and Phase100 QR scalar-clock values remain comparison
baselines. Every Phase102 promotion gate is immutable, including the strict
`0.782 m` macro AND gate.

## Read and process accounting for this audit

| Activity | Count |
|---|---:|
| Native solver invocations | `0` |
| Raw GNSS/IMU/navigation byte reads | `0` |
| Truth-file reads | `0` |
| Phase102 solution CSV reads | `0` |
| MAT/base/PDC/precomputed-coordinate reads | `0` |
| Accuracy calculations | `0` |
| Kaggle/token access | `0` |
| Reruns/fallbacks | `0` |

The next boundary is one separately committed correction/tests contract,
one truth-only authorization, and one evaluator invocation over the sealed
Phase102 output root. A correction failure is sealed fail-closed; no native
or evaluator rerun is authorized after truth access.
