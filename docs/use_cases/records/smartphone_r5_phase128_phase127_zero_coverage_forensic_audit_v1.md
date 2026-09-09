# Smartphone R5 Phase128 Phase127 zero-coverage forensic audit

- Execution label: `Luna Max`
- Audit date: `2026-09-04`
- Repository: `/home/sasaki/workspace/rtklib_v2_ws/gnssplusplus-library`
- Branch: `feat/rtk-smartphone-performance-pr`
- HEAD at audit start: `25f144f4943b30d6e74bd6582a5d378f0aad7748`
- Scope: read-only audit of the sealed Phase127 result, ignored Phase127
  partial inventory metadata, the authorized runner, and native/RTKLIB source.

This audit did not open a raw phone GNSS/IMU file, broadcast-navigation
payload, raw-base payload/header, solver output, truth, MAT, PDC, precomputed
coordinate/correction, accuracy, Kaggle, or token resource.  No solver or
evaluator was run.  The three ignored partial-inventory files were metadata
only and were read explicitly because the requested forensic counts are not
all duplicated in the committed result:

| ignored metadata file | bytes |
|---|---:|
| `output/smartphone-r5/phase127-inventory-first-structural-v1/2021-03-16-18-59-us-ca-mtv-a__pixel5/inventory.json` | 2,954 |
| `output/smartphone-r5/phase127-inventory-first-structural-v1/2022-04-01-18-22-us-ca-lax-t__pixel5/inventory.json` | 2,858 |
| `output/smartphone-r5/phase127-inventory-first-structural-v1/partial_result.json` | 8,893 |
| **total ignored metadata read** | **12,705** |

The historical Phase127 sealed accounting records two authorized Stage-1
reads per route (phone GNSS, phone IMU, broadcast nav, and base RINEX), zero
solver launches, zero solution-row reads, and zero truth/MAT/PDC/precomputed-
coordinate/accuracy/Kaggle access.  Those are historical facts, not new reads
performed by this audit.

## Decision

The first deterministic failure is a pre-resolution inventory gate, not a
failed GLONASS query-time lookup.  For rover rows the first operand evaluated
by the runner is `phone.ok`, which is false because 236 MTV-A and 247 LAX-T
selected GLONASS rows are classified invalid.  Independently, the broadcast
navigation gate is false because 480 and 442 records are counted malformed,
and the base gate is false because each header ledger reports one malformed
entry.  Consequently neither `resolve_query()` loop is entered, all
certified/coverage reason counters remain at their initialized zero values,
and no satellite key, FCN, or time delta is actually resolved.

The single source-backed candidate frozen in the companion JSON is therefore
one parser/provenance admission boundary:

`phase128-glonass-provenance-parser-admission-v1`

It makes the RINEX/Android key and time representations canonical, parses
GLONASS navigation records field-for-field, and admits complete valid records
per observation instead of poisoning the whole route because an unrelated
record was malformed.  A missing header ledger is represented as missing (and
uses only a corroborated time-valid broadcast FCN); an actually malformed
header slot still fails closed.  The required row-level FCN, exact-time,
header/geph, tie, and finite/domain predicates remain strict.  This is one
parser/provenance candidate, not a solver, factor, frequency-table, or tuning
change.  It is a design freeze only; it does not authorize raw execution.

## Authoritative pins and sealed evidence

| artifact | commit or digest |
|---|---|
| sealed Phase127 result commit | `25f144f4943b30d6e74bd6582a5d378f0aad7748` |
| result JSON SHA-256 | `dad0bd0c9b627a11ebe794d67bbdb28eead808e5903d924e08dafc481a424e45` |
| result Markdown SHA-256 | `7acb2a6fa4f780d908d839ecda05526bf50edaa78b9c18feaec1297a1e54abf7` |
| independent authorization commit | `e2e6c3a30c0e1391c464bb29c487601d674dd6dc` |
| authorized runner commit | `93ee772e595aac01a5de4e7357becd88581f70ec` |
| authorized runner SHA-256 | `cf5fc5530d8f79b1e56a9e435630ef84d0c8df6e5dd982311cf2d5b5617a71e3` |
| Phase127 implementation commit | `f41d082e5170fe5dcbebbb4526c7d513f9512b60` |
| Phase127 design freeze commit | `5c3e66fc4b62d86b52f8e9ee8aa1f26f4cb2a8a4` |
| Phase127 inventory contract audit commit | `d4b28a8dbe85ac67bf8957e28de16f072874d072` |
| Phase127 pre-raw accounting commit | `da8d50cb37e9adebcd6b140b2f67a482276323be` |
| Phase127 runner/manifest commit | `93ee772e595aac01a5de4e7357becd88581f70ec` |

The sealed status is `no-go-phase127-inventory-first-structural`; route order
was MTV-A then LAX-T, one attempt each, and `native_solver_invocations=0`.

## Route counts and earliest stop

The runner's `glonass_rows` is the number of rows appended after the
pseudorange, satellite, and time checks; it is not the number of all selected
GLO rows.  Thus the arithmetic below is directly supported by the sealed
metadata.

| route | selected GLO rows | invalid selected rows | retained GLO rows | nav records / malformed | unique accepted nav satellites | base GLO rows | header entries / conflicts / malformed | rover certified | base certified |
|---|---:|---:|---:|---:|---:|---:|---|---:|---:|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` (MTV-A) | 12,068 | 236 | 11,832 | 671 / 480 | 14 | 17,199 | 0 / 0 / 1 | 0 | 0 |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` (LAX-T) | 8,873 | 247 | 8,626 | 686 / 442 | 14 | 920 | 0 / 0 / 1 | 0 | 0 |

For the nav counts, `671+480=1,151` and `686+442=1,128` are the runner's
record flush attempts split into accepted records and malformed records.  The
sealed record has no reason subcounts for those malformed records.  Source
logic shows that the bucket can include fewer than 15 fields, a non-finite
field, an unparsable/non-integral FCN, an out-of-range FCN after normalization,
or an unparsable epoch; the metadata cannot distinguish those cases without
reopening the nav payload.

The phone invalid bucket is similarly coarse: it combines missing/non-finite
or non-positive pseudorange with missing/unparseable satellite or epoch.  The
sealed metadata says `missing_satellite_rows=0` and `missing_time_rows=0` for
both routes, so the remaining invalid-row reason is not decomposed further by
the already sealed runner.  It is not evidence of an Android-to-RTKLIB key
failure.

The resolver's own counters are all zero (`coverage_gaps=0`, `ties=0`,
`mismatches=0`, `max_age_s=0`, and no header/geph fallback rows) because the
runner calls the rover resolver only when both `phone.ok` and `nav.ok` are
true, and calls the base resolver only when both `base.ok` and `nav.ok` are
true.  The `coverage.exact_query_time_records=true` field is emitted
unconditionally by the runner after those loops; it is not evidence that an
exact query was made.  No time delta or sample satellite key is therefore
available from the sealed metadata.  The only non-coordinate identifiers
available are route labels, constellation codes, and signal labels such as
`GLO_G1_CA`.

The one `header_malformed` value per route means the Python inventory saw a
line containing the `GLONASS SLOT / FRQ #` label but found no `Rnn fcn` pair in
its inspected prefix.  It does not mean one FCN conflict: `header_entries=0`
and `header_conflicts=0`.  Without reopening the base payload, it is
impossible to distinguish an intentionally empty header ledger from a
formatting/position mismatch.  The native reader's fixed-slot parser treats
an absent entry as absence and increments its malformed counter only for a
truncated `R` slot, empty slot fields, lexical parse failure, or invalid PRN
range.  This makes the Python `label-without-match => malformed` rule a
source-backed parser concern, but it is secondary to the nav/phone gates and
cannot by itself prove any channel coverage.

## Forensic stage table

| stage | implementation evidence | sealed observation | finding |
|---|---|---|---|
| Rover GLONASS key | runner `:305` recognizes constellation `3`/`GLO`; `:315` stores `int(float(Svid))`; `:323` stores `{sat,query_time,signal}` | `glonass_rows=11,832/8,626`, `missing_satellite_rows=0` | An integer Svid is present for retained rows, but the runner does not prove a typed `(GLONASS, PRN)` key or native range mapping. |
| Native Android key | `src/io/android_raw_gnss.cpp:818-837` maps constellation/signal and enforces GLONASS PRN 1..24; `:909-910` constructs `SatelliteId(system,svid)` | no native solver started | Native construction is source-defined; it was not exercised in Phase127 because Stage 1 failed. |
| Android Svid → RTKLIB key | RTKLIB `satid2no`: `external/madocalib/src/rtkcmn.c:578-603` maps `Rnn` to `satno(SYS_GLO,prn)`; `:482-503` applies the constellation PRN offset | no runtime bridge metadata | The Python inventory's bare integer is not a proof of equivalence to RTKLIB's internal satellite number. The candidate must compare the typed system+PRN key, never a global integer or fixed offset guessed from a payload. |
| RINEX nav satellite ID | runner `:411-417` matches `Rnn` and stores an integer; native parser `src/io/rinex.cpp:1773-1788` creates `SatelliteId(system,prn)`; RTKLIB `:1300-1309` uses `satid2no` | 14 unique accepted nav satellite keys per route | Some R records were recognized, but no sample ID or cross-source join was available. |
| Android time | runner `:268-281` prefers UTC/Unix millisecond or nanosecond aliases and converts to POSIX seconds without a leap-second step | `missing_time_rows=0`, but no resolver calls | The inventory time is UTC-like POSIX metadata, not proven to be the native GPST query time. |
| Native Android time | `src/io/android_raw_gnss.cpp:484-511` computes GPS week/TOW from `TimeNanos-FullBiasNanos` and bias; `:789-805` stores that `GNSSTime` per UTC epoch | solver not launched | Native raw input has an explicit GPST construction; this is the required query-time authority for a future candidate. |
| RINEX GLONASS time | native `src/io/rinex.cpp:1823-1836` rounds the GLONASS UTC epoch to 900 s and applies `utcToGpst`; `external/madocalib/src/rinex.c:1211-1222` documents the same UTC rounding and GPST toe/tof | runner rounds a POSIX epoch at `:423`; no age observed | The 900-s unit/rounding is equivalent in intent, but UTC-vs-GPST conversion is not carried by the Python inventory. No actual age can be claimed. |
| `toe`/validity | native `src/core/navigation.cpp:59-68` uses GLONASS `abs(time-toe)<=1800`; Phase127 helper `:127-150` and `:155-184` repeats exact-time selection | `max_age_s=0` is an initializer, not a measurement | Validity policy is known; query-time coverage is unobserved because resolution was skipped. |
| `geph.frq` field/offset | runner `:391-407` requires 15 finite fields, reads `values[10]`, normalizes `>128` by `-256`, and requires `[-7,6]`; RTKLIB `rinex.c:1230-1247` reads `data[10]`, normalizes `>128`, and reports range violations; continuation layout is `:1331-1345` | accepted/malformed `671/480` and `686/442` | Field index, signed normalization, and Phase127 strict range are source-backed. The all-record `malformed==0` gate and coarse field parser prevent row-level attribution. |
| Header ledger | runner `:445-470` uses a broad label/regex rule; native `src/io/rinex.cpp:1180-1221` scans eight fixed 7-character slots and preserves an ordered ledger; Phase127 helper rejects nonzero malformed header count at `:112-116` | 0 entries, 0 conflicts, 1 malformed each | `1` is a parser ledger status, not a conflict or an FCN value. Absence versus malformed cannot be proven from metadata alone. |
| Provenance resolver | runner `:561-587` checks coverage, nearest age, equal-FCN ties, range, and header/geph match; `:607-645` invokes it only after whole-item gates pass | all reason buckets and certified rows zero | No query-time failure reason was evaluated; zero coverage is a pre-query short-circuit. |

## Root-cause classification

### Facts

1. The earliest executed rover admission predicate is `phone.ok`, and it is
   false for both routes because the sealed runner counts 236/247 invalid
   selected GLO rows (`authorized_runner.py:327-340`).
2. The shared navigation predicate is also false: `parse_nav_geph` defines
   `ok` as `bool(records) and malformed == 0` (`:377-443`), while the sealed
   counts are 671/480 and 686/442.
3. The base predicate is false because its `header_ok` requires zero malformed
   entries (`:528-543`), while both sealed ledgers have 0 entries, 0 conflicts,
   and 1 malformed status.
4. Both resolver loops are guarded by those whole-item predicates
   (`:607-629`), so no FCN selection, key join, time-age calculation, tie, or
   mismatch reason was observed. Native solver invocation is correctly zero.

### Inference, bounded by the no-reread rule

The most likely shared implementation failure is an over-coarse inventory
parser/admission boundary, not proof that the broadcast file has no usable
`geph.frq`. The parser silently collapses distinct malformed causes into one
counter and rejects a route before testing whether records required by a
specific retained observation are complete and time-valid. The broad header
label rule can likewise turn a valid empty `GLONASS SLOT / FRQ #` ledger into
`malformed=1`. These are source-backed hypotheses, not payload-level proof;
the 480/442 composition and the meaning of the one header line remain
unknown without a prohibited payload reread.

The phone invalid rows remain a real fail-closed input condition. Nothing in
the sealed metadata authorizes repairing, synthesizing, or dropping them for
the solver. The candidate only makes the parser's row-level reasons explicit;
it does not waive the final full-retained-row gate. Likewise, zero query-time
gaps or ties must not be reported: those counters were never reached.

## Exactly one frozen candidate boundary

The companion freeze JSON records one default-off candidate with this exact
scope:

1. Decode Android and RINEX identifiers into the same typed
   `SatelliteId(GNSSSystem::GLONASS, prn)` key, with explicit PRN range and no
   global RTKLIB integer guessed from Svid.
2. Construct/obtain the query time from the existing native `GNSSTime` path
   (Android raw clock GPST and native RINEX UTC-to-GPST conversion), rather
   than comparing independently invented POSIX seconds.
3. Parse each RINEX GLONASS nav record in the canonical fixed fields
   `data[0..14]`, preserving field positions.  Read `data[10]` as FCN,
   normalize only an encoded value `>128` by `-256`, and require a finite
   integral Phase127 FCN in `[-7,6]`; do not use the local RTKLIB permissive
   diagnostic upper bound of 13.
4. Keep complete valid records in a per-satellite index and retain malformed
   records in typed counters.  Do not reject the whole route merely because
   an unrelated record is malformed.  For every retained rover/base GLO row,
   require a selected exact-time record, `abs(t_query-toe)<=1800 s`, finite
   state/FCN, and the existing minimum-age/tie policy.  Missing/out-of-window,
   nonfinite/nonintegral/out-of-range FCN, different-FCN tie, or header/geph
   mismatch still fails closed.
5. Parse the header by fixed RINEX slots and preserve the uncollapsed ledger.
   Zero entries means `header_missing`, not `header_malformed`; a slot that
   starts with `R` but is truncated/lexically invalid remains malformed.  A
   missing header can use only the selected broadcast FCN fallback, never a
   fixed channel or Android carrier frequency.

These are one parser/provenance admission correction.  The candidate does not
change C7/D/CCDD, Phase126 raw-base correction, factors, graph topology,
frequency equations, sigma, filters, TDCP/IMU/Doppler, QR, LM, initialization,
Pixel5 offset, output, legacy/default behavior, or fail-closed/no-fallback
policy.  It adds no coordinate, solution, truth, or external channel data.

## Source evidence and hashes

The source files inspected for this audit were not modified:

| source | SHA-256 | key lines |
|---|---|---|
| `apps/commands/benchmarks/gnss_smartphone_phase127_inventory_first_structural_authorized_execute.py` | `cf5fc5530d8f79b1e56a9e435630ef84d0c8df6e5dd982311cf2d5b5617a71e3` | `:268-340`, `:364-443`, `:445-587`, `:590-682` |
| `src/algorithms/phase127_glonass_channel_provenance.cpp` | `cc8e3105abc8367786b089e8d0492f8217550f0a69473869ae8a89cc01941bb0` | `:69-246` |
| `include/libgnss++/algorithms/phase127_glonass_channel_provenance.hpp` | `dc1de4ae6de270f306f1c07a54ab1799df26344f8cc6fdd910d7abfaf1914c8b` | `:24-56`, `:78-104` |
| `src/io/android_raw_gnss.cpp` | `a020ce900db6a9d3a994cc3adb3dcebbe84cbf67c1ff9793991b7a75039529c5` | `:202-256`, `:484-511`, `:789-910` |
| `src/io/rinex.cpp` | `6fa2778e871587d105382fbd3c20589ccfcf5378cc73f62cb2e33ddbb5cf2771` | `:1180-1221`, `:1749-1890`, `:2006-2064` |
| `src/core/navigation.cpp` | `f876803d2e61fa62f370419f4ee4db16afb6b12b2d7ab00cd8b613af6f2649d3` | `:59-86`, `:146-168` |
| `src/core/navigation_internal.hpp` | `1d922769774bed46a7cdd9cdc513ff63391768f13fd7e6c708af49c4c977a56e` | `:110-137` |
| `external/madocalib/src/rinex.c` | `7973649a511f1aad13a65dbe51fb8cbdffec8ccabe105d26c235fb88b34da413` | `:1192-1247`, `:1296-1345` |
| `external/madocalib/src/rtkcmn.c` | `259419b4f8099f5893e43f95b24e83a064cb0d62efef28b675e57c35e0912e75` | `:482-503`, `:524-604` |
| `include/libgnss++/core/types.hpp` | `475d524280fec85d3e959e318447f917a614d93314207a04f84501b0f7735ee3` | `:171-211` |

## Future focused tests and authorization boundary

No tests were run that ingest payloads.  Before any implementation or raw
authorization, focused synthetic tests must cover:

- blank/absent header versus a truncated or lexically malformed RINEX slot;
- canonical `Rnn`/Android Svid key equality and invalid PRN rejection;
- all 15 GLONASS fields with blank/nonfinite non-FCN fields, missing or
  non-integral FCN, `128/129` normalization, and `-7/6/-8/7` boundaries;
- per-record malformed accounting that still admits an unrelated valid record;
- exact GPST query time, 900-second toe rounding, 1,800-second acceptance,
  out-of-window rejection, missing satellite, same-FCN duplicate, and
  different-FCN tie;
- header-primary exact FCN corroboration and nav-only fallback;
- retained-row full coverage and finite-positive wavelength gates; and
- selector-off semantic regression and zero solver launches on any failed
  inventory predicate.

The candidate remains default-off and raw/solver unauthorized.  A new
implementation pin, launch-free manifest, pre-raw accounting seal, and
independent authorization are mandatory before either route may read a raw
payload.  No route rerun, fallback, repair, fixed FCN, external table, or
truth/accuracy action is authorized by this audit.
