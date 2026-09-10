# Smartphone R5 Phase127 inventory-first structural contract audit

- Execution label: `Luna Max`
- Audit date: `2026-09-04`
- Repository: `/home/sasaki/workspace/rtklib_v2_ws/gnssplusplus-library`
- Branch: `feat/rtk-smartphone-performance-pr`
- HEAD at audit start: `f41d082e5170fe5dcbebbb4526c7d513f9512b60`
- Scope: read-only source, build metadata, and sealed Phase126/Phase127 records.

This audit does not authorize raw materialization, navigation or base-payload
reads, solver execution, solution-row access, truth/MAT/accuracy evaluation,
PDC, precomputed coordinates/corrections, Kaggle, or reruns.  The target
binary hash below is a static build pin, not a payload read.

## Decision

Freeze exactly one execution-contract candidate:

`phase127-inventory-first-raw-base-glonass-structural-v1`

The candidate is a launch-free, two-stage admission contract around the
already implemented Phase127 selector.  It does not change the Phase126
correction operator, any factor, state, equation, unit, sigma, filter, Huber
threshold, LM schedule, ordering, initialization, output offset, or legacy
default.

The exact execution selector is:

`--native-phase127-glonass-channel-provenance`

and it remains valid only when the existing Phase126 selector and the frozen
Phase118 recipe are present:

`--native-phase126-raw-base-source-complete`

`--native-phase118-official-tdcp-huber-k`

Phase117 dynamic sigma, Phase120 TDCP normalization, and additional-frequency
bands remain absent.  The candidate is structural only; truth and accuracy
remain separately unauthorized.

## Authoritative pins and evidence

| artifact | commit | static SHA-256 or status |
|---|---|---|
| Phase126 design freeze | `583a6c7a4788f82373a4e71436d2953703bc764c` | sealed design |
| Phase126 implementation | `9e9972667ee009e1bcd0dd1ea4732ae45415c3ce` | sealed implementation |
| Phase126 structural result | `df27546fafa4fb6fbd96a520dcd3fd91acc3c0ae` | `a0ab7f6ca35f06277953646f0f406a2c850ea73cef3b52ec4b681d0d48236b33` |
| Phase127 design freeze | `a6df42a774f1b832796b9fa81a1e75217f7cb038` | `e0c28812ae8c28594f1286ca2512de17bcd4262bb7343b7bb985c6ba25cc901c` |
| Phase127 implementation | `f41d082e5170fe5dcbebbb4526c7d513f9512b60` | implementation pin |
| current target binary | `build/apps/gnss_fgo_imu_no_base` | `653797970fbe65f199fc98dfe47ddd57ec26da66fadc4df40dff930a6d1c436a` |

The sealed Phase126 structural result is `no-go` because its pre-solver
inventory did not prove GLONASS header FCN coverage.  It records MTV-A with
17,199 GLONASS base observation rows and LAX-T with 920, while both routes
have zero sealed header FCN entries.  The sealed nav metadata is finite and
in-domain but does not expose per-satellite `geph.frq`, `toe`/`tof`,
query-time coverage, or duplicate/conflict status.  That result is evidence
for a new inventory gate, not evidence that a channel may be guessed.

The Phase127 implementation already provides the source-backed proof helper,
legacy-map-preserving header ledger, explicit FCN-presence bit, exact
time-valid ephemeris selection, duplicate/conflict counters, and fail-closed
admission.  The missing execution boundary is a pre-solver inventory that
proves those predicates for every GLONASS row before the Phase126 solver is
started.

## Exact two-stage contract

### Stage 1: post-authorization inventory, solver forbidden

After a new independent raw authorization, the runner may materialize only
the route's raw phone GNSS, raw phone IMU, broadcast navigation, and sealed
raw base RINEX.  It must first produce compact per-route inventory metadata;
it may not invoke the native solver during this stage.

The inventory must establish all of the following:

1. The base RINEX header is parsed without collapsing away the raw
   `GLONASS SLOT / FRQ #` ledger.  For every header FCN entry, the satellite
   key is exact, the FCN is an integer in `[-7,6]`, and malformed, conflicting,
   or unresolved entries are counted and fail closed.
2. For each GLONASS observation retained by the existing Phase126 base
   correction path, and for each rover GLONASS observation retained by the
   existing FGO path, the inventory records the exact existing second-pass
   FGO query/transmission time.  No new time iteration or nearest-time fill is
   allowed.
3. At each exact `(satellite, query_time)`, the existing time-aware
   `NavigationData::getEphemeris(satellite, query_time)` selection has a valid
   broadcast record with `abs(query_time - toe) <= 1800` seconds and a
   present, finite, integral FCN in `[-7,6]`.  Out-of-window records are not
   candidates.
4. A header FCN is primary but requires exact selected-`geph.frq`
   corroboration.  With no header entry, the selected broadcast FCN is the
   only permitted fallback.  Different-FCN minimum-age ties, missing FCN,
   header/geph mismatch, duplicate conflict, query-time gap, and invalid or
   out-of-domain values fail closed.
5. Rover and base GLONASS coverage are complete: the count of certified
   provenance rows equals the count of GLONASS rows that Phase126/FGO would
   retain, unresolved/missing rows are zero, and all certified frequency and
   wavelength values are finite-positive.  The inventory must preserve exact
   satellite/signal/key counts rather than infer coverage from aggregate file
   size.
6. The inventory explicitly records `solver_invocations=0` and
   `solver_may_start=false` when any predicate is false.  A failed route is
   sealed as `inventory-fail-closed`; it is not repaired, retried, or passed
   to Stage 2.

The inventory is metadata-only.  It must not contain solution coordinates,
truth rows, MAT data, precomputed phone/base coordinates, correction tables,
PDC state, or external/fixed GLONASS channel tables.

### Stage 2: Phase126 structural run, admitted only after Stage 1

Only a route whose Stage 1 inventory has passed may launch the already pinned
Phase126/Phase127 native command once.  The route command must contain each
of the two required selectors exactly once, plus Phase118, and must contain
none of Phase117, Phase120, additional-frequency, PDC, direct-WLS, or
diagnostic fallback selectors.  The command uses only the four permitted raw
inputs and opaque output/summary paths.

The structural result must verify, without opening solution coordinate rows:

- Phase127 is active and no legacy/default fallback is selected;
- all Phase126 A/B/C transaction markers are true and correction is applied
  exactly once;
- rover/base GLONASS certified rows equal the Stage 1 inventory and all
  source/mismatch/coverage counters pass;
- GNSS-first and main progress, finite costs, strict cost decrease, QR branch,
  finite earth-valid expected output coverage, and exact C7/D/CCDD handoff
  pass the inherited Phase126 gates; and
- output is sealed only as opaque hash/row metadata.  Structural GO does not
  authorize truth or accuracy evaluation.

The fixed route order is MTV-A followed by LAX-T, exactly one admitted attempt
per route, with no controls, reruns, fallback, repair, or sweep.  If Stage 1
fails, the native solver invocation count for that route must remain zero.

## Fixed inputs and forbidden lineage

Allowed after the independent authorization, and only after the inventory
boundary is opened:

- raw phone `device_gnss.csv`;
- raw phone `device_imu.csv`;
- broadcast navigation;
- the separately sealed raw base RINEX and its station metadata from the
  existing Phase126 provenance.

Forbidden at every stage:

- truth, MAT, accuracy, Kaggle, or token access;
- phone, base, result, or precomputed coordinate/correction input;
- PDC or an external/fixed GLONASS channel/frequency table;
- Android carrier-frequency inference or fixed zero FCN;
- solver launch before complete per-row inventory;
- stale/future ephemeris extrapolation, endpoint hold, nearest fill, repair,
  fallback, rerun, or sweep.

## Implementation boundary for the contract artifacts

The next artifacts may add only a launch-free validator, manifest, two-stage
runner facade, synthetic inventory predicates, and zero-activity accounting.
They must pin the Phase127 implementation commit and the current target
binary hash.  They must not modify the estimator or materialize a payload.
The future authorized runner may implement Stage 1 inventory and Stage 2
dispatch, but its independent authorization must pin all route paths/hashes,
the binary hash, the manifest hash, and the exact command vector before any
raw read.

## Focused test plan

The launch-free suite must cover:

- complete synthetic rover/base inventory admission;
- header-primary matching geph and nav-only fallback;
- 1800-second boundary and out-of-window rejection;
- missing, invalid, mismatched, conflicting, duplicate, and tied FCN ledger
  outcomes;
- partial rover/base coverage and nonfinite/domain failures;
- Stage 1 failure forcing zero solver launches;
- exact selector isolation and legacy/default-off behavior; and
- static absence of child-process/payload operations and zero pre-raw
  accounting.

This audit freezes no raw execution.  The audit and its machine-readable
freeze are intentionally separate commits; a later independent authorization
is mandatory before Stage 1 can read any route payload.
