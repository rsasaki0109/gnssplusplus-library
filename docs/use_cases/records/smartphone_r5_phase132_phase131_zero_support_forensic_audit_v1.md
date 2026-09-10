# Smartphone R5 Phase132 Phase131 zero-support forensic audit

- Execution label: `Luna Max`
- Audit date: `2026-09-04`
- Repository: `/home/sasaki/workspace/rtklib_v2_ws/gnssplusplus-library`
- Branch: `feat/rtk-smartphone-performance-pr`
- HEAD at audit start: `2e39f7c909d0be2d901ae17ed4b2830f92772f65`
- Worktree at audit start: clean.
- Scope: read-only forensic audit of the sealed Phase131 result, its ignored
  route metadata, the authorized runner, the Phase131 implementation, and
  already-sealed Phase130 comparison metadata.

This audit did not open a phone GNSS/IMU file, broadcast-navigation payload,
raw-base RINEX payload, solution row, truth row, MAT/PDC/precomputed
coordinate artifact, or Kaggle/token resource.  It did not launch the native
solver and did not rerun a route.  The ignored Phase131 files listed below
were read only as already-produced metadata.  Their hashes are artifact
hashes; none is a hash of a reconstructed route key set.

## Disposition

Exactly one candidate is frozen in the companion JSON:

`phase132-runner-canonical-preflight-admission-v1`

The candidate is a runner-boundary correction: replace the reused Phase130
literal signal-descriptor gate with the existing Phase131 source-locked typed
physical-band mapping before the Phase131 inventory is admitted.  It does not
change the native graph, factors, equations, units, correction values, FCN
policy, sigma, QR, IMU, TDCP, LM schedule, output, or legacy default.

The earliest zero-support predicate is proven to be a runner/preflight
boundary mismatch.  The actual route signal-token distribution and actual
native Phase131 behavior remain unestablished because the native process was
never invoked and the sealed metadata contains no distinct rover key set.
The data itself therefore cannot be classified as the root cause.

Implementation and any new raw execution remain unauthorized by this audit.

## Authoritative evidence and hashes

| item | value |
|---|---|
| Phase131 sealed result commit | `2e39f7c909d0be2d901ae17ed4b2830f92772f65` |
| Phase131 result JSON SHA-256 | `2f02879a7b06704e588cacaae92f6c509fc2a4733f0efac4a4d61702defd22d1` |
| Phase131 result MD SHA-256 | `e222329977296ae4ee50c3ddd7ec2650fed164a99287d73d9814a435a654b0de` |
| Phase131 authorization commit | `ed746428861699b941384c7ea73cecae33a44d55` |
| Phase131 authorization JSON SHA-256 | `49392c23f94d88b3c2d6dc7df073ca8960a340fbaf9a4650cc30483c99d8cfac` |
| authorized runner commit | `615a63c54802a5d7a3da725f2e64fc5bd26d64a5` |
| authorized runner SHA-256 | `95e4df869caaf14158a0688c7fd2c548fced7ae9bb23306fe2e09785b8dd6127` |
| Phase131 implementation commit | `c3af051e46f3685c0d3406537fa8f7c12eea75f2` |
| Phase131 launch-free contract commit | `c5afacef8416aace5e8f713ad8a4b08654666457` |
| Phase131 contract freeze commit | `322f1278c9b802d3414b1fd275baeb3e731542a9` |
| Phase131 manifest commit | `41299c13832af3eaefade4c2e50a5ea1bcaebced` |
| Phase131 pre-raw accounting commit | `927b8fe2d4d20be8849f0afb91f3c7454c662603` |
| target binary SHA-256 | `ed24b57b29f679123c3a52dbb04fad9b979092564abc4a35905bdb2abfaa0c8e` |
| Phase130 sealed comparison commit | `3d16fe289409d3aa6290d861ec23d51f69941db5` |
| Phase130 result JSON SHA-256 | `f9cf9226889fc4a4e6141e54f6447bc09998b3f42a7cf45269cc8ed0cb0e0553` |

The current ignored Phase131 metadata artifacts are:

| route | metadata path | metadata SHA-256 |
|---|---|---|
| MTV-A | `output/smartphone-r5/phase131-canonical-correction-structural-v1/2021-03-16-18-59-us-ca-mtv-a__pixel5/inventory.json` | `6f51fe4bcdc8682bd7d402b214ff95e2baeb6c60fd7fed0d31b8f98644ee8593` |
| LAX-T | `output/smartphone-r5/phase131-canonical-correction-structural-v1/2022-04-01-18-22-us-ca-lax-t__pixel5/inventory.json` | `40e56a26a4c52a7e5efd0ae67b466ac65dcfd6b8edda452119d2c80b6c9f28c2` |
| matrix partial metadata | `output/smartphone-r5/phase131-canonical-correction-structural-v1/partial_result.json` | `616c26daa18fd8506979d7ba66b88a660f87eb1543f679429d7f76d53fde7cb4` |

Both current route inventories contain only the same fail-closed record:

```text
failure: all retained rover/base exact-key support is empty
stage: post-independent-authorization-pre-solver
ok: false
phase131.enabled: true
phase131.source_complete_a_b_c: false
solver_invocations: 0
solver_may_start: false
```

The `phase131.enabled` field is emitted by the runner's inventory failure
record.  It is not native C++ telemetry and does not prove that the selector
reached the binary.

## Selector and invocation path

The launch-free Phase131 command template contains
`--native-phase131-canonical-correction-band-key` exactly once, together with
the Phase126--130 selectors and the Phase118 recipe.  The authorized runner's
`command_for` function would substitute the sealed raw paths and base digest
into that template.  However, the actual control flow is:

1. `execute_matrix` verifies the static authorization and reads each route's
   four authorized payloads once.
2. It calls Phase131 `build_inventory`.
3. Phase131 `build_inventory` immediately delegates to
   `p130.build_inventory` and raises if that old inventory is not `ok`.
4. Only an `ok` inventory reaches `execute_one_route`; only there are
   `command_for` and `subprocess.run` called.

For both routes step 3 raised.  Consequently:

* the Python Phase131 canonical conversion (`canonical_side` and the
  Phase131-shaped record) did not execute;
* no rover row reached the Phase130 rover-side `resolve_query` or
  `classify_stream` calls, because every valid rover row was rejected by the
  literal descriptor predicate.  The old builder's separate base-side
  `resolve_query` loop is reached by the control flow before it returns the
  global failure, but its call count is not sealed in the current metadata;
* the native `base_pseudorange_compensation` Phase131 canonical resolver did
  not execute;
* `command_for` did not construct a route command and the native binary was
  not invoked; and
* `phase131.enabled: true` is a fail-record marker only.

Thus the Phase131 selector was present in the pinned contract, but it was not
actually executed in the Stage1 inventory path or in the native process.
There is no native Phase131 summary, C++ counter, selected canonical stream,
factor vector, cost, or solver return code in this run.

The runner source order is visible at:

* Phase131 authorized runner `execute_matrix`: lines `785-859`;
* Phase131 authorized runner delegation: lines `335-340`;
* Phase131 authorized runner command construction: lines `468-476` and
  `689-709`; and
* Phase131 command template: `gnss_smartphone_phase131_canonical_correction_structural.py`
  lines `300-324`.

## Current-run versus copied Phase130 counters

The following are facts from the current Phase131 sealed result and ignored
metadata, not native solver output:

| route | current inventory `ok` | current canonical support marker | native solver invocations | return code | current key set |
|---|---:|---:|---:|---|---|
| MTV-A | false | 0 | 0 | `null` | not sealed |
| LAX-T | false | 0 | 0 | `null` | not sealed |

The current run's per-route inventory reads were one each for
`device_gnss.csv`, `device_imu.csv`, `brdc.nav`, and `base.obs`.  This audit
did not perform any of those reads again.

The result embeds a separate historical Phase130 comparison block.  Those
counters came from the already-sealed Phase130 result (JSON SHA-256
`f9cf9226889fc4a4e6141e54f6447bc09998b3f42a7cf45269cc8ed0cb0e0553`); they are
explicitly not a Phase131 rerun:

| route | rover input / certified / local miss / missing exact key | base input / certified / local miss | canonical support | factor input / retained / missing exact stream | base unused |
|---|---:|---:|---:|---:|---:|
| MTV-A | `12068 / 0 / 12068 / 11832` | `17199 / 9642 / 7557` | `0` | `11832 / 0 / 11832` | accounted |
| LAX-T | `8873 / 0 / 8873 / 8626` | `920 / 396 / 524` | `0` | `8626 / 0 / 8626` | accounted |

The historical Phase130 route metadata hashes are `767e5fb895b707e9bab3a516e1379384d5c5fbbc0e0c9487e7c78855d57615a8`
(MTV-A) and `41cdfdc0f6fd5b79579fdfbdafb1803bd1656192b3a5c9ef8621748e22674543`
(LAX-T).  They are metadata hashes only.  The current Phase131 result has
not copied these numbers into a native Phase131 telemetry claim.

## Field-by-field key trace

| field | Phase131 Stage1/current runner representation | native Phase131 representation | finding |
|---|---|---|---|
| system | Phase130's Python preflight stores the GLONASS satellite as the string/int tuple `("R", prn)`; its exact RINEX parser also recognizes `Rnn` | `GNSSSystem::GLONASS` is enum value `0x02`; `SatelliteId` stores `(system, uint8_t prn)` | representation differs at the boundary; the runner does not perform a native enum conversion before its gate |
| PRN / satellite ID | `parse_satellite` accepts optional `R`, leading zeroes, and PRN `1..27`, returning `("R", int)`; base exact rows use the same tuple | `SatelliteId` ordering/equality compares system then direct PRN | no RTKLIB offset is applied by the audited runner or by `phase131_canonical::Key`; no row index is used |
| physical band | `signal_descriptor` strips selected prefixes/letters and compares a textual suffix such as `1C`, `2P`; it never calls typed family mapping | `familyForSignal` maps typed signals to `L1`, `L5`, or `Unknown`; RINEX mapping goes through `signal_policy` and then the same family function | this is the earliest semantic mismatch: literal descriptor admission precedes physical-family mapping |
| literal signal/tracking code | base descriptor set is derived from RINEX `R` codes; Android text and RINEX text must produce the same descriptor | original `SignalType` and literal code remain provenance, but `Key` excludes tracking-code text | Phase131 native design intentionally removes literal CA/P/code text from the correction join |
| GLONASS FCN | Phase130 calls `resolve_query` only after descriptor membership; no FCN is part of the textual descriptor key, and the rover call supplies an absent header | Phase127/128 exact query-time certification is required; `canonicalize` rejects missing/out-of-range FCN and includes certified FCN in the GLONASS key | FCN is not the observed cause of the current zero because no row reached the resolver; it remains mandatory for any future admitted GLO key |
| query time | Android epoch fields are converted to native GPST seconds; base exact epochs are converted likewise | `correctionAtCanonical` takes finite `GNSSTime` and queries the canonical stream | time is a support query, not a key field and not a row-index pair |
| support stream | Phase130 stream key is `(R/prn tuple, literal descriptor)`; `classify_stream` checks exact endpoint or adjacent bracket only after the textual gate | native stream is `map<phase131_canonical::Key, vector<Sample>>`; lookup is exact canonical key plus in-domain finite support | current run has no constructed Phase131 stream; historical Phase130 streams had zero used samples |

The native canonical key is therefore:

```text
(GNSSSystem, SatelliteId.prn, PhysicalFrequencyFamily,
 certified_GLONASS_FCN when system is GLONASS)
```

The native `Key` does not contain a RTKLIB satellite offset, row index,
literal tracking code, query time, or a carrier-frequency-derived FCN.  The
FCN is a certified provenance/wavelength discriminator only.

## Distinct keys, counts, and examples

No current Phase131 route-level distinct rover key inventory was sealed.  In
particular, neither current inventory JSON contains `rover_distinct_signal_tokens`,
canonical key strings, satellite-level intersections, or a canonical-key-set
digest.  The current metadata SHA values above must not be misreported as key
set hashes.  The historical Phase130 result likewise records
`retained_exact_keys: []` and does not seal the distinct rover signal-token
set.  Exact route Android signal tokens, satellite-specific intersections,
and per-key time-support samples are therefore **missing**, not inferred.

One non-route, source-only synthetic key example is present in the Phase131
unit test: `keyString` renders a GLONASS key as `"2:7:L1:fcn=-4"`, corresponding
to the typed tuple `(GLONASS, 7, L1, -4)`.  This is a test vector, not a route
sample.  The source's Python synthetic contract uses the equivalent tuple
`("GLONASS", 7, "L1", -4)`.  No route key example is claimed.

The historical Phase130 sealed header-code lists are metadata facts:

* MTV-A base `R` codes: `C1C,L1C,S1C,C2P,L2P,S2P`; the old descriptor set is
  `{"1C","2P"}`.
* LAX-T base `R` codes: `C1C,L1C,S1C,C2C,L2C,S2C,C2P,L2P,S2P`; the old
  descriptor set is `{"1C","2C","2P"}`.

Those lists explain the base side of the textual boundary.  They do not
provide the omitted rover token set and cannot establish a route-level
canonical intersection without a new authorized raw inventory.

## Earliest empty intersection

The first decisive branch is in
`gnss_smartphone_phase130_shared_ledger_structural_authorized_execute.py`:

```python
descriptor = signal_descriptor(row.get("signal"))
if descriptor is None or descriptor not in descriptors:
    rover_reasons["missing-exact-key"] = (
        rover_reasons.get("missing-exact-key", 0) + 1)
    continue
```

For each rover row, only after this branch does the runner call `resolve_query`
for GLONASS provenance and `classify_stream` for endpoint/bracket support.
The old builder separately scans base rows with `resolve_query` after the
rover loop; the current record has no per-call telemetry for that base scan.
The Phase131 wrapper calls this old builder first and propagates its global
failure.  The historical `missing-exact-key` counts equal every valid rover
GLONASS row (11,832 MTV-A and 8,626 LAX-T), while the current Phase131 output
records no native resolver or support counters at all.

The likely source-level explanation for an alias such as `GLO_G1_CA` is also
visible without opening a route: after the helper removes the `GLO` prefix,
the remaining underscore-bearing text is not a digit-leading descriptor and
is rejected.  This is a source behavior and a non-route illustration only;
the exact route token remains unavailable in sealed metadata.

## Root-cause classification

| candidate class | classification | evidence |
|---|---|---|
| runner-only mapping/instrumentation | **proven earliest cause** | Phase131 delegates to Phase130; the literal descriptor membership check rejects before nav/time/support, and no native command is built after the failure |
| Phase131 native implementation | not proven faulty and not proven successful in this run | the C++ canonical helper/source and synthetic tests define the intended family/FCN key, but native `build` was never invoked |
| actual route data | undetermined | raw payload and route-level distinct key inventories are forbidden in this audit and absent from sealed output |

The current `phase131.enabled` failure metadata is also an instrumentation
limitation: it reports the requested selector, not selector execution.

## Exactly one frozen fix

The companion freeze JSON freezes only
`phase132-runner-canonical-preflight-admission-v1`:

1. At the Phase131 runner boundary, derive a typed system/PRN and physical
   family from the existing parsed Android/RINEX observations using the
   source-locked Phase131 policy.  Do not compare Android text literally with
   RINEX text.
2. Use `(GNSSSystem, PRN, physical family[, certified GLO FCN])` for the
   preflight correction join.  Keep original signal/tracking text as
   provenance only.  Same-family aliases may collapse; different or unknown
   families are explicit misses.
3. Require the existing Phase127/128 exact query-time FCN certification for
   GLONASS, with range `[-7,6]`.  Do not derive FCN from Android carrier
   frequency, fix a channel, use an external table, or infer a missing value.
4. Require one finite exact-endpoint or adjacent finite in-domain support
   result per retained rover correction.  Missing key/support, nonfinite,
   out-of-domain, duplicate, or ambiguous/conflicting source rows remain
   explicit fail-closed misses or global failures according to the existing
   contract.  No raw, zero, nearest, hold, extrapolated, or uncorrected
   fallback is permitted.
5. Preserve side-local conservation, unused-base accounting, exactly-once
   correction, and zero solver launches on inventory failure.  Do not expose
   a partial remap.

This candidate is runner-only.  It does not alter native C++ equations,
factor topology, C7/D/C0D/CCDD, base correction math, moving mean,
interpolation, sigma, Huber, QR, IMU, TDCP, LM, output, or legacy/default
behavior.  It also does not authorize raw materialization, solver execution,
truth, accuracy, or publication.

## Focused validation and fresh authorization plan

No raw-dependent or solver-dependent test was run in this audit.  Before
using the candidate, implementation must add or exercise only synthetic and
static tests for:

1. Android aliases and RINEX `C1C`/`C1P`/same-band forms mapping to one typed
   physical family without literal-token equality;
2. system/PRN identity, direct PRN handling, and no row-index/RTKLIB-offset
   join;
3. certified GLO FCN required, including missing, out-of-range,
   header/geph mismatch, tie, and conflict fail-closed cases;
4. source-defined same-family precedence and ambiguous finite duplicate
   rejection;
5. exact endpoint and adjacent in-domain bracket support with unequal
   cadence/counts, plus explicit missing/out-of-domain/nonfinite misses;
6. rover/base side-local and factor/miss conservation, unused-base
   accounting, exactly-once correction, all-miss/empty-route gates, and no
   partial remap; and
7. selector-off legacy regression and static proof that no solver is started
   when preflight fails.

After implementation, a new launch-free contract/runner/manifest and a new
pre-raw zero-read seal are required.  A new independent authorization must
pin the changed runner, source, contract, binary, and artifacts.  The old
Phase131 authorization cannot be reused after this runner change.  Only then
may a one-shot MTV-A then LAX-T inventory-first run be considered, with raw
phone GNSS/IMU, broadcast navigation, and sealed raw base RINEX as the only
solver inputs.  A failed route must launch zero solver processes and be
sealed without rerun, fallback, repair, truth, accuracy, or publication.

## Audit read accounting

| category | reads in this audit |
|---|---:|
| sealed Phase131/Phase130 JSON/MD and ignored route metadata | read-only metadata |
| pinned runner, contract, implementation, headers, source, tests | read-only source |
| raw phone GNSS payload reads | `0` |
| raw phone IMU payload reads | `0` |
| broadcast-navigation payload reads | `0` |
| raw-base RINEX payload/header/bytes reads | `0` |
| raw payload copies/transforms | `0` |
| native solver invocations | `0` |
| solution rows or coordinate interpretations | `0` |
| truth reads / accuracy calculations | `0` |
| MAT/PDC/precomputed-coordinate reads | `0` |
| Kaggle/token access | `0` |
| reruns/fallbacks/repairs | `0` |

No implementation or data-state change was made by this audit.
