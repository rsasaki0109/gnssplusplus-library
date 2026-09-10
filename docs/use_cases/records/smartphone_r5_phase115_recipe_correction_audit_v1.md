# Phase115 direct-seed recipe correction audit

Status: read-only audit of the sealed Phase110 NO-OP decision and the sealed
Phase114 direct-seed execution attempt.  No raw phone GNSS/IMU/navigation
payload, raw base RINEX bytes/header, truth, MAT, phone-result coordinates,
PDC, native solver, accuracy evaluator, or Kaggle resource was read or run.

## Question and decision

Phase114 implementation `44771ef` was exercised with the frozen direct
same-run WLS/C7/D seed, Phase99 QR, raw-base correction, and final Pixel5
offset recipe.  Both authorized routes failed at argument validation before a
native summary was produced.  The preserved stderr is identical:

```
--native-base-pseudorange-source-miss-mask plus --native-base-pseudorange-preserve-additional-frequency-bands requires the Phase101 meter-state handoff
```

The direct selector intentionally forbids the GNSS-first meter-state handoff,
so the Phase114 command required two mutually incompatible admissions.  This
is a recipe/selector composition failure, not evidence about the direct WLS
seed, C7 mapping, D alignment, QR solver, or graph convergence.

The unique source-backed correction is to remove only
`--native-base-pseudorange-preserve-additional-frequency-bands` from the
Phase114 argv.  The remaining two existing raw-base selectors
(`--native-base-pseudorange-compensation` and
`--native-base-pseudorange-source-miss-mask`) are the already admitted
Phase107 path and are explicitly accepted by the direct Phase114 branch.
No source or binary change is included in this audit.

## Exact source evidence

In `apps/native/gnss_fgo_imu_no_base.cpp`:

* the generic parser predicate at lines 624--630 rejects the miss-mask plus
  additional-band pair unless the GNSS-first meter-state handoff is enabled;
* the direct selector rejects that handoff at lines 1234--1242;
* the direct branch's Phase107 admission at lines 1259--1264 requires
  compensation, miss-mask, exact base path/SHA, and *no* additional-band
  selector;
* the direct branch's Phase109 admission at lines 1265--1270 requires the
  additional-band selector, but cannot satisfy the earlier generic predicate
  while GNSS-first remains bypassed.

Thus removing that single flag makes the argv satisfy the existing Phase107
admission without changing an equation, observation, state, C7/D handoff,
solver, filter, LM schedule, sigma, offset, or output boundary.

## Sealed Phase110 evidence

The Phase110 audit compared the existing Phase107 recipe with Phase109's
additional-band recipe without interpreting solution fields.  MTV-A and
LAX-T solution bytes, hashes, and row counts were identical; retained rover
factor vectors, correction aggregates, GNSS-first/main numeric telemetry, C7/D
handoff, and QR output telemetry were unchanged.  Only the base reader's
selected observation set changed.  Therefore admitting the additional-band
flag is not justified for this direct candidate, while retaining it would
repeat a proven no-op and still fail the direct parser.

## Phase115 candidate boundary

Exactly one default-off candidate is frozen for a future one-shot structural
run:

* direct selector `--native-direct-wls-ephemeral-c7d-main-seed`;
* C0/D meter state, official seven-component C vector, raw retained
  `EpochSeed.receiver_clock_drift_mps`, exact keys, and same-run WLS velocity;
* Phase99 `MULTIFRONTAL_QR` / `EliminateQR` main solve;
* existing Phase107 raw-base correction and miss-mask selectors only;
* existing final Pixel5 offset at the final output boundary;
* GNSS-first stage bypassed by design; no fallback, zero-fill, interpolation,
  synthesized velocity, global ISB duplicate, or alternate selector;
* exactly MTV-A then LAX-T, one invocation per route.

The structural gates remain direct full finite exact-key position/C7/D/velocity
handoff, base correction exactly once, QR accepted iteration with strict cost
decrease, finite expected output coverage, offset exactly once, and no fallback.
Truth/accuracy/solution release remains outside the boundary.

## Pinned records and accounting

| Record | Commit | SHA-256 | Role |
|---|---|---|---|
| Phase110 NO-OP audit | `0b1ab4b0df811a6f419c08f633232d60aa9cf417` | `2b134217568797aeeaee85f1d2836202702b89222c4950f133dbde3c6a0fbb05` | selector comparison |
| Phase110 freeze | `52d0903` (sealed record) | `91064f377228a506951599c946cbaaf6acf5fb7c5feb33a27dad8a6ba0172033` | no-op disposition |
| Phase114 implementation | `44771ef6b52eddb0df6193b8b9d9f1b52b7fcea5` | source/binary pinned in Phase114 manifest | direct adapter |
| Phase114 result | `5335a2b` | `fc2674dde4c7641f393dd7ad1fb7f06660e5bdedc4c008b667ef1448bf01275b` | two parser failures |

Phase114 wrapper activity after authorization was limited to metadata-only
raw member stats and one base hash preflight per route.  Native invocation
count was two; argument parsing failed before summary/solution creation.
Truth, MAT, PDC, precomputed phone coordinates, accuracy, Kaggle, and solution
reads were zero.  The Phase114 result and logs are preserved unchanged; this
audit does not repair or rerun them.

## Authorization boundary

This audit authorizes neither implementation nor execution.  A separate
Phase115 pre-raw manifest/evaluator and independent authorization must pin the
single argv deletion, all source/binary hashes, exact raw/base metadata, and
the no-truth/no-solution policy before any future native process is launched.
