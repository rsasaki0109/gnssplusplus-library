# Phase108 Phase107 truth-only accuracy audit

- Execution label: `Luna Max`
- Audit date: `2026-09-03`
- Status: `sealed-read-only-audit-qualified-for-phase108-truth-only-freeze`
- Scope: the two already completed Phase107 raw-base structural runs,
  MTV-A and LAX-T. No solver, raw GNSS/IMU, broadcast-navigation, or base
  RINEX rerun is permitted.

## Audit finding

Phase107 result commit `5b076bb647566a4789cadf637bb1ba611584c304` is a sealed
structural GO result. It records exactly one native run per route, return code
zero, active raw-base correction exactly once, complete finite C7/D handoff,
the Phase99 QR main branch, finite Earth-valid expected output coverage, and
no truth or accuracy reads. Its result SHA-256 is
`e7eb3f1d2670672414744a100dc1e2fc3e0b019ae7efc0dbfef1518300ce8d6e`.
The sealed Phase107 manifest and authorization are pinned as
`b46a4bfa23ac1604d9e51487734be18262f027612eede40fdbcba15fa75c2a9c` and
`ba975dd93f11cd64fdaec6cc6d57a080367fc2e6a04ee58513cb3a0b0542eb56`.

The two withheld candidate files were sealed before any truth access. This
audit performed metadata/hash sealing only: each file was opened as opaque
bytes for SHA-256 and byte length; no CSV header, row, timestamp, latitude,
longitude, or coordinate value was interpreted.

| route | opaque candidate path | bytes | SHA-256 |
|---|---|---:|---|
| MTV-A | `output/smartphone-r5/phase107-raw-base-source-parity-v1/2021-03-16-18-59-us-ca-mtv-a__pixel5/withheld_solution_output.csv` | 172694 | `b401156e76d5a1a1cfc948aa666d69d292fe4ba43ea3e3f4580cde7e5f0339f8` |
| LAX-T | `output/smartphone-r5/phase107-raw-base-source-parity-v1/2022-04-01-18-22-us-ca-lax-t__pixel5/withheld_solution_output.csv` | 117254 | `6e85865825a250628db4d6aa842fc5992ebdc63aa51ffaf7f1dd513c1fe600d8` |

The files remain withheld and are not copied, repaired, published, or
committed. Their paths and hashes are the only candidate-content facts
admitted into the Phase108 freeze. Candidate CSV parsing is deferred to the
single truth-only evaluator subprocess after the independent authorization.

## Metric and gate parity

The Phase108 evaluator will reuse the sealed Phase102/103 contract and the
Phase82 parser/scorer reference, without changing its equations or thresholds:

- exact `(phone, UnixTimeMillis)` integer-key alignment;
- CSV `DictReader` truth fields `UnixTimeMillis`, `LatitudeDegrees`, and
  `LongitudeDegrees`, with an optional matching `phone` field;
- duplicate or extra prediction keys fail closed, and only the pinned leading
  MTV-A warm-up truth key may be absent;
- spherical Haversine distance with radius `6371008.8 m`;
- linear percentile rank `(n - 1) * q`, route scalar `(P50 + P95) / 2`, and
  unweighted macro in the fixed order MTV-A then LAX-T;
- exact prediction-domain coverage `1.0`, finite/Earth-valid values, and zero
  prediction transitions above `70 m/s`.

The comparison anchors are sealed, route-specific aggregates rather than new
reads of their solution or truth payloads:

| source | MTV-A (m) | LAX-T (m) | macro (m) |
|---|---:|---:|---:|
| Phase82 same-route direct-quality | 1.1139384500152307 | 0.9389644134001871 | 1.0264514317077089 |
| Phase103 no-base C7 correction | 1.139793072101309 | 3.310820065300743 | 2.225306568701026 |

The promotion decision is an AND over the Phase102/103 structural accuracy
guards, including route coverage, finite/Earth-valid coordinates, speed,
route limits, no route regression against the Phase82 same-route baseline,
and strict macro score `<= 0.782 m`. The Phase103 no-base C7 aggregate is
recorded as the same-pipeline comparison anchor. A failed predicate is a
sealed no-go; no rerun, fallback, tuning, coordinate repair, or submission is
allowed.

## Truth-only boundary and read accounting

The Phase108 freeze will pin the two official truth paths, expected sizes,
rows, and SHA-256 values from the existing Phase102 contract. Those files are
not opened by this audit or by the solver. After the independent authorization,
one evaluator subprocess may read each truth file exactly once, after parsing
and finite/Earth-valid preflight of the corresponding withheld candidate. The
result will contain metrics, gate booleans, hashes, and accounting only; it
will not contain solution rows or truth coordinate rows.

| activity in this audit | count |
|---|---:|
| native solver invocations | 0 |
| raw GNSS/IMU/navigation or base RINEX reads | 0 |
| truth reads | 0 |
| candidate coordinate interpretation | 0 |
| MAT/precomputed phone coordinates/PDC reads | 0 |
| accuracy calculations | 0 |
| Kaggle/token access | 0 |
| reruns/fallbacks | 0 |

Exactly one candidate is frozen: `phase108-phase107-withheld-solution-truth-only-accuracy-v1`.
The next implementation boundary is an evaluator/manifest/tests contract,
followed by a new independent truth-only authorization. It may consume the
opaque Phase107 files above but may not invoke the native executable or open
any raw/base input.
