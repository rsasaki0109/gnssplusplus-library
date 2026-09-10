# Phase118 isolated truth-only accuracy gate audit

- Execution label: `Luna Max`
- Audit date: `2026-09-03`
- Status: `sealed-read-only-audit-qualified-for-phase118-truth-only-freeze`
- Scope: the immutable Phase118 structural outputs for exactly MTV-A and
  LAX-T.  This audit does not open a candidate CSV, truth CSV, raw input,
  base RINEX, MAT file, or coordinate payload, and it does not run a solver or
  accuracy calculation.

## Structural authority

The Phase118 structural result is pinned to commit
`b4ea80d96d7156f9fd62cc554f2deeedcc550cb0` and SHA-256
`88e8799050fd396eeb14e83d4f5923339279f46d0219a385e303d3cb50731538`.  It is a
structural GO with exactly one native invocation per route, return code zero,
the Phase118 Highway Huber threshold selected, finite strict progress in both
stages, exact finite C7/D handoff, QR main selection, raw-base correction and
Pixel5 offset each applied once, and expected output coverage.  Its
`accuracy_scored` and `solution_output_published` flags are false.

The structural result contains only opaque solution seals.  The sealed
candidate metadata (path, bytes, hash, and row count) is:

| route | opaque candidate path | bytes | SHA-256 | rows |
|---|---|---:|---|---:|
| MTV-A | `output/smartphone-r5/phase118-tdcp-robust-k-v1/2021-03-16-18-59-us-ca-mtv-a__pixel5/withheld_solution_output.csv` | 172694 | `aec1ce4f83ad520822f98dcd4d0b0e0cc325d49db470fa2048a0352021496d2f` | 2158 |
| LAX-T | `output/smartphone-r5/phase118-tdcp-robust-k-v1/2022-04-01-18-22-us-ca-lax-t__pixel5/withheld_solution_output.csv` | 117254 | `c5217a8f4871194bfffe7912268bbaa27bcf9691cfd399b731d43a2bd976dfdb` | 1465 |

These candidate members remain withheld, unmodified, and unpublished.  The
audit reads the Phase118 result metadata only; it does not verify those bytes
or interpret any row.

The structural result records the final Pixel5 offset boundary as applied
exactly once (`corrected_epochs` equals 2159 and 1466 respectively), so the
truth evaluator must not apply a second offset.  The evaluator consumes the
already-produced output CSV as-is.

## Metric and parser parity

The metric is inherited without algorithmic changes from the sealed
Phase102/103/112 truth-only contracts and the pinned Phase82 scorer.  The
source references audited here are:

| artifact | SHA-256 | role |
|---|---|---|
| `apps/commands/benchmarks/gnss_smartphone_phase76_phase75_accuracy_integrity_recovery.py` | `513e5970ee58846963f077b9960944e5fbf5f3535e5be24ca6e72493785f7761` | corrected CSV DictReader/parser and scoring helper |
| `apps/commands/benchmarks/gnss_smartphone_phase82_phase80_direct_quality_accuracy.py` | `232ba0ab1fa7ebde6ddfaaf851d190d1ed32377ea056ab6c31e244139c2977fb` | pinned official metric reference |
| `apps/commands/benchmarks/gnss_smartphone_phase112_main_output_offset_accuracy.py` | `d34c7df1b48be999141013018a78f5b61e70f831afc2977871e0ac756741b6cc` | prior isolated truth evaluator |
| `apps/commands/benchmarks/gnss_smartphone_phase117_tdcp_weighting_accuracy.py` | `88e8020052a0bdaed3f7262dedc46600f6608a84a7bcdabe82ac009b8db82ebf` | most recent truth-only evaluator reference |
| `tests/test_smartphone_phase112_main_output_offset_accuracy.py` | `414c501cc23283aa42f73b3cd7802cc0946b58494ac7fef1417945af2b554176` | prior parser/evaluator regression reference |
| `tests/test_smartphone_phase117_tdcp_weighting_accuracy.py` | `1b2bcc567569a46f4c0e27c09535b113643e9cf4d59eebfe19cde6d3804c49f3` | most recent truth-only regression reference |

The exact candidate and truth contract is:

- submission header `phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees`;
  required truth fields are `UnixTimeMillis`, `LatitudeDegrees`, and
  `LongitudeDegrees`; optional columns are allowed;
- a present `phone` column must equal the declared route, while an absent
  phone is assigned that route;
- the join key is exact `(phone, UnixTimeMillis)` with integer timestamps and
  exact integer-key intersection only;
- duplicate prediction or truth keys fail closed, and prediction keys absent
  from truth fail closed;
- only the exact pinned leading warm-up truth key may be absent for MTV-A;
  LAX-T has no missing truth key.  There is no interpolation, nearest fill,
  endpoint hold, extrapolation, or repair;
- each matched error is spherical Haversine distance with Earth radius
  `6371008.8 m`; P50/P95 use linear interpolation at rank `(n - 1) * q`;
- each route scalar is `(P50 + P95) / 2` metres, and the macro is the
  unweighted arithmetic mean in fixed order MTV-A then LAX-T;
- prediction-domain coverage must equal exactly `1.0`, all candidate and
  derived values must be finite and latitude/longitude Earth-valid, and the
  prediction transition speed count above `70 m/s` must be zero.

The prior Phase112 result is the requested baseline, pinned to commit
`0151bd070f8029413f8f2a17a28fe982dddb735f` and result SHA-256
`46d8d836758ac1a88b95ec2c1a0c0080cedfb753c12411ed74e4c2501549c57e`:

| route | Phase112 final route scalar (m) |
|---|---:|
| MTV-A | 0.9976852530 |
| LAX-T | 0.6659910918 |
| macro, unweighted fixed two-route mean | 0.8318381724000121 |

The Phase112 and Phase117 accuracy manifests are pinned references (SHA-256
`29aac6bd43f56183142c84219a7924d6172733a73385a5725a9e3e3013566768` and
`cbba73a941d2cd46c5d9a6cfacfa55404c4e716b4c05191fa5d5e8fd3140789a`).  Their
truth metadata identifies the same official cohort:

| route | truth path metadata | bytes | SHA-256 | rows | allowed missing key |
|---|---|---:|---|---:|---|
| MTV-A | `output/smartphone-r5/phase29-no-bridge-train-eval-v1/truth/2021-03-16-18-59-us-ca-mtv-a/pixel5/ground_truth.csv` | 210009 | `7c84ed6a80b1bbb08c0ffad57493513833b9d5474e22a43c5a44da82824ee22d` | 2159 | `(MTV-A, 1615921153434)` only |
| LAX-T | `output/smartphone-r5/phase44-pixel5-development-accuracy-v1/truth/2022-04-01-18-22-us-ca-lax-t/pixel5/ground_truth.csv` | 143441 | `29e0861dd1ecb8865c10adab69396d98ed96618e8877b09d04aa8d671edf79e8` | 1465 | none |

The paths, hashes, byte counts, rows, and missing-key policy above are copied
from sealed Phase112/117 metadata only.  Neither truth member is opened or
hashed by this audit or by the future solver process.

## Promotion and isolation decision

Exactly one Phase118 accuracy candidate is frozen:
`phase118-official-tdcp-huber-k-mapping-truth-only-accuracy-v1`.  The strict
promotion predicate is an AND of all route/schema/finite/coverage/speed
checks and:

```text
candidate_macro_score_m < 0.782
```

The inequality is strictly less-than; equality is not a pass.  Accuracy GO,
even if achieved, does not authorize release, validation, or Kaggle
submission.  Any route or macro failure is no-go and immutable; there is no
rerun, fallback, repair, post-truth tuning, or candidate replacement.

The future evaluator must run as one isolated truth-only subprocess and, for
each route exactly once, perform the following order:

1. validate the pinned Phase118 opaque seal (path, bytes, SHA-256), parse only
   the candidate after authorization, and enforce the exact header/key/finite/
   Earth-valid/row policy;
2. open and hash the one pinned official truth file for that route, parse it
   with the pinned DictReader contract, and score using the unchanged Phase82
   helper;
3. seal route metrics, the unweighted two-route macro, gate booleans, hashes,
   and read accounting without writing solution or truth rows to the result.

The future evaluator may read no raw GNSS/IMU/navigation, base RINEX, MAT,
precomputed phone coordinates, PDC, Kaggle, or token data; it launches no
native process; and it performs no additional offset.  A new evaluator,
manifest, focused tests, and an independent truth-only authorization must be
committed and pinned after this freeze.  Truth reads are exactly two total,
one per route; candidate solution reads are exactly two total, one per route;
accuracy calculations are exactly two; all solver/raw/base rerun, truth
reads-before-authorization, MAT/PDC/coordinate reads, Kaggle access,
fallbacks, and reruns are zero.

## Read accounting for this audit

| activity | count |
|---|---:|
| native solver invocations | `0` |
| raw GNSS/IMU/navigation or base RINEX reads | `0` |
| candidate solution payload reads or coordinate interpretations | `0` |
| truth payload reads | `0` |
| accuracy calculations | `0` |
| MAT/precomputed phone-coordinate/PDC reads | `0` |
| Kaggle/token access | `0` |
| reruns/fallbacks | `0` |

The audit is therefore eligible for a separate Phase118 accuracy freeze.  No
truth or candidate row has been used to select or tune the candidate.
