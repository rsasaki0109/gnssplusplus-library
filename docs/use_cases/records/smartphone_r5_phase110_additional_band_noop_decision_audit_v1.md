# Phase110 additional-band no-op decision audit

Status: read-only comparison of the sealed Phase107 and Phase109
structural runs.  No raw phone GNSS/IMU/navigation payload, raw base RINEX
byte/header, truth payload, native solver, accuracy evaluator, Kaggle
resource, MAT file, or coordinate payload was read or run.  The four
candidate solution files were read only as opaque bytes for SHA-256 and line
count because the Phase110 decision explicitly requires solution hash/row
comparison; no CSV field or coordinate value was interpreted.

## Decision

The Phase109 additional-frequency-band selector changes the native base
RINEX observation selection set, but it does not change the retained rover
code-factor vector, correction summary, GNSS-first/main graph telemetry, or
the emitted solution bytes for either audited route.  The candidate solution
hash and row count are identical for Phase107 and Phase109 on MTV-A and
LAX-T.  Phase108's already sealed Phase107 solution hashes independently
cross-check the Phase107 side of this comparison.

Disposition: **NO-OP**.  Do not re-run the truth evaluator or perform a new
accuracy evaluation for Phase109.  The companion Phase110 freeze seals this
disposition; it authorizes no solver, raw/base, truth, accuracy, or
publication action.  The only unresolved follow-up is a read-only source-key
taxonomy audit; it is not a frozen algorithmic candidate.

## Pinned artifacts

| Artifact | SHA-256 | Use |
|---|---|---|
| Phase107 structural result | `docs/use_cases/records/smartphone_r5_phase107_raw_base_source_parity_result_v1.json` / `e7eb3f1d2670672414744a100dc1e2fc3e0b019ae7efc0dbfef1518300ce8d6e` | sealed Phase107 aggregate and summary hashes |
| Phase108 accuracy result | `docs/use_cases/records/smartphone_r5_phase108_phase107_solution_accuracy_result_v1.json` / `9f9a2528745c06ba06e774139f56892bcea0ef208a3d064e4b70042e48f29a9d` | sealed Phase107 solution hashes/rows; no new truth read |
| Phase109 structural result | `docs/use_cases/records/smartphone_r5_phase109_raw_base_frequency_parity_result_v1.json` / `b443c2fcd1ed3b067094b47bf0b81f7fa404ed20547e7234d1c174dcf9ca2225` | sealed Phase109 aggregate and summary hashes |
| Native application pin | `apps/native/gnss_fgo_imu_no_base.cpp` / `be643811cfaf82304f9434bc969796b2a14cd0f6e5a2c8b46b043af7b326c5c2` | Phase109 selector/admission and telemetry |
| Native RINEX reader pin | `src/io/rinex.cpp` / `92f683e34b4052f8d105f69fc4bf4397ca990be3b5a6b220181a56423f465533` | primary/secondary plus optional bands |
| Base correction model pin | `src/algorithms/base_pseudorange_compensation.cpp` / `f3c3c859793c4bed9c7ae93825d32565f20eab0badb09412a4fab2a5930c9817` | exact satellite/signal stream correction |
| Source miss-mask pin | `src/algorithms/source_pseudorange_miss_mask.cpp` / `ce85e428ac0a7af93ee1ad3eadf082b5361454dff5c920245be7c16c940b6949` | exact-key and in-domain retention |

## Solution hash and row comparison

Phase108 seals the Phase107 candidate hashes and rows.  Phase110 computed
the Phase109 hashes and line counts as opaque output metadata only.  The
Phase109 result itself intentionally contains no solution rows and records
the files as withheld.

| Route | Phase107 bytes / rows | Phase107 SHA-256 | Phase109 bytes / rows | Phase109 SHA-256 | Equal |
|---|---:|---|---:|---|---|
| MTV-A | 172,694 / 2,158 | `b401156e76d5a1a1cfc948aa666d69d292fe4ba43ea3e3f4580cde7e5f0339f8` | 172,694 / 2,158 | `b401156e76d5a1a1cfc948aa666d69d292fe4ba43ea3e3f4580cde7e5f0339f8` | yes |
| LAX-T | 117,254 / 1,465 | `6e85865825a250628db4d6aa842fc5992ebdc63aa51ffaf7f1dd513c1fe600d8` | 117,254 / 1,465 | `6e85865825a250628db4d6aa842fc5992ebdc63aa51ffaf7f1dd513c1fe600d8` | yes |

No candidate coordinate, timestamp, latitude, longitude, or truth value was
parsed.  The files remain uncommitted and are not published.

## Summary and solver comparison

The native summary hashes differ because Phase109 adds exactly-once and
per-signal telemetry and enables the additional-band reader.  The structural
solver/output fields are otherwise equal.

| Route | Summary SHA Phase107 → Phase109 | Base selected rows / streams Phase107 → Phase109 | Adopted → corrected | Miss exact / out-of-domain |
|---|---|---:|---:|---:|
| MTV-A | `3399cdb0…42a8a` → `a4b7fd27…486e3` | 121,673 / 58 → 167,020 / 69 | 57,281 → 43,259 | 12,894 / 1,128 → 12,894 / 1,128 |
| LAX-T | `bf90eb60…1114f` → `b111620e…07dd` | 6,342 / 61 → 10,332 / 85 | 30,798 → 30,664 | 0 / 134 → 0 / 134 |

The exact base correction distribution is unchanged:

| Route | p50 metres | p95 metres | max metres |
|---|---:|---:|---:|
| MTV-A | 6.860445285925226 | 14.845409692524433 | 15.637516129413319 |
| LAX-T | 6.154975089218844 | 11.179839421284049 | 11.88553089346821 |

GNSS-first and main graph telemetry is bitwise equal in the sealed numeric
fields:

| Route | GNSS-first iterations / cost | Main factors / values | Main iterations / cost |
|---|---:|---:|---:|
| MTV-A | 218; `4516236.0222890135 → 16097.714277915162` | 106,071 / 10,795 | 12; `130723525.58263575 → 29729.5954122218` |
| LAX-T | 234; `6412859.293244721 → 12298.593335217001` | 60,948 / 7,330 | 12; `162184507.52202186 → 19851.9725625441` |

Both runs also retain the same problem/output epochs (MTV-A 2,159/2,159;
LAX-T 1,466/1,466), pseudorange factors (43,259/30,664), TDCP factors
(31,269/14,012), C7 dimension 7 and full finite handoff, D full finite
handoff, zero global ISB states, and selected `MULTIFRONTAL_QR` /
`EliminateQR`.  The Phase109 result records correction application exactly
once; Phase107's older result schema does not expose those newer fields.

## What the selector changed

The source reader default is primary/secondary only.  In
`include/libgnss++/io/rinex.hpp:91-99` the additional-band switch is
default-off; `src/io/rinex.cpp:283-303` appends one selected observation for
each supported additional band only when it is enabled.  The Phase109
summary's selected-band maps show the following additions over Phase107:

| Route | Newly selected signal/band rows | Newly selected streams |
|---|---|---|
| MTV-A | `GAL_E5B` +35,266; `GPS_L5` +10,081 | `GAL_E5B` +7; `GPS_L5` +4 |
| LAX-T | `GAL_E5B` +2,368; `GAL_E6` +1,182; `GPS_L5` +440 | `GAL_E5B` +10; `GAL_E6` +11; `GPS_L5` +3 |

All other selected signal/band map entries are equal between the two
summaries.  These extra rows belong to the base correction reader.  The
retained rover factor count and every reported correction quantile remain
unchanged, so the extra streams do not affect the emitted solution in these
two runs.

## MTV-A exact-stream miss taxonomy

Phase107 only sealed aggregate miss counts.  Phase109 adds the required
signal taxonomy, which closes MTV-A's 57,281 adopted rows as follows:

| Native signal / band | Original rows | Exact stream matched | Retained/corrected | Missing exact stream | Out of domain | Nonfinite |
|---|---:|---:|---:|---:|---:|---:|
| `BDS_B1I` / B1 | 12,849 | 0 | 0 | 12,849 | 0 | 0 |
| `GLO_L1CA` / G1 | 9,379 | 9,334 | 9,334 | 45 | 0 | 0 |
| `GPS_L1CA` / L1 | 21,434 | 21,434 | 20,306 | 0 | 1,128 | 0 |
| `GAL_E1` / E1 | 13,619 | 13,619 | 13,619 | 0 | 0 | 0 |
| **total** | **57,281** | **44,387** | **43,259** | **12,894** | **1,128** | **0** |

The missing exact-stream reason is structural, not an inferred distance:
the source mask calls `hasStream(SatelliteId, SignalType)` before
`correctionAt`; a false exact pair is counted as `dropped_missing_exact_stream`
and is not tested as a time-domain miss (`src/algorithms/source_pseudorange_miss_mask.cpp:84-106`).
`BDS_B1I` has zero exact matches because no corresponding B1I stream appears
in the sealed Phase109 selected base-band map.  For `GLO_L1CA`, 45 retained
rover rows have no exact `(SatelliteId, GLO_L1CA)` base stream even though
9,334 exact streams match.  The sealed telemetry cannot identify those 45
individual satellite IDs or distinguish source payload absence from a
signal-key mapping absence without reopening base bytes; no such reopening
was performed.  `GPS_L1CA`'s 1,128 losses are separately out-of-domain after
an exact stream exists, not exact-key misses.

The native model stores streams by `{SatelliteId, SignalType}`
(`src/algorithms/base_pseudorange_compensation.cpp:255`), and
`correctionAt` rejects times outside the first/last sample
(`:307-339`).  No alias, frequency-only match, endpoint hold, or
extrapolation is used.

## Next-candidate disposition

Three actions were considered:

1. Re-score Phase109 against truth.  Rejected: solution SHA/bytes/rows are
   identical to the already evaluated Phase107 candidate; this would be a
   duplicate evaluation with no changed prediction.
2. Add a new BDS/GLO mapping or fill missing rows.  Rejected: the audit has
   no per-key proof for a new mapping, and that would change the frozen
   source-key/equation or time-domain contract.
3. Audit existing source-key taxonomy and reader mapping read-only.  Kept as
   the next audit-only question, not frozen for implementation or execution.

No Phase108-equivalent truth-only evaluation contract is frozen.  No
accuracy threshold, metric, truth path, or promotion boundary is opened by
this Phase110 decision.

## Read accounting

| Resource/action in Phase110 | Count |
|---|---:|
| Sealed Phase107/108/109 JSON and summary metadata reads | nonzero |
| Native/source code reads | nonzero, source-only |
| Opaque Phase107/Phase109 solution hash and line-count reads | 4 |
| Solution coordinate fields interpreted | 0 |
| Raw phone GNSS/IMU/navigation payload reads | 0 |
| Raw base RINEX byte/header/stat/hash reads | 0 |
| Truth payload reads | 0 |
| MAT/precomputed/phone-coordinate/PDC reads | 0 |
| Native solver invocations | 0 |
| Accuracy evaluator invocations | 0 |
| Kaggle/token access | 0 |
| Reruns/fallbacks/publication | 0 |

The Phase109 result's recorded post-run wrapper taxonomy-key exception is
preserved as result metadata; this audit does not repair that result or
rerun its wrapper.  The native summaries and opaque solution comparisons
remain unchanged.
