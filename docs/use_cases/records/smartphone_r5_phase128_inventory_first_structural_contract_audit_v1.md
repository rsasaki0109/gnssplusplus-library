# Smartphone R5 Phase128 inventory-first structural contract audit

- Execution label: `Luna Max`
- Audit date: `2026-09-04`
- Repository: `/home/sasaki/workspace/rtklib_v2_ws/gnssplusplus-library`
- Branch: `feat/rtk-smartphone-performance-pr`
- Audited implementation: `50357e8f3eabbb2eca672b00a16eb2ce32cc2f16`
- Scope: read-only source, sealed Phase127/Phase128 records, and the existing
  build artifact.

This audit does not authorize raw phone GNSS/IMU, broadcast-navigation, or
raw-base reads; solver launch; solution-row or coordinate access; truth, MAT,
PDC, precomputed-coordinate, accuracy, token, or Kaggle access; or rerun,
repair, fallback, or frequency-channel guessing.  The target binary hash is a
static build pin only.  No route payload was opened.

## Decision

Freeze exactly one launch contract candidate:

`phase128-inventory-first-glonass-parser-admission-structural-v1`

The candidate is the Phase128 parser/admission overlay already implemented in
the pinned implementation.  It is default-off and composes, in this exact
order, with Phase126 raw-base source-complete, Phase127 GLONASS provenance,
and Phase118 official TDCP Huber selection.  Phase117 dynamic sigma,
Phase120 atmosphere-cancellation, and additional-frequency-band selectors are
off.  It does not alter equations, factors, values, state topology, units,
sigma, robust loss, filter, QR, LM, initialization, output, or the legacy
default.

The future structural matrix is exactly two routes, MTV-A then LAX-T, one
admitted attempt per route.  A route must first pass an inventory stage after a
separate authorization.  A failed inventory seals that route with zero solver
invocations.  Only a complete inventory may enter the existing Phase126
compound A/B/C structural solver; no fallback or rerun is permitted.

## Evidence and earliest failure boundary

The sealed Phase127 result (`25f144f...`) established zero certified rows for
both routes because its inventory applied whole-input gates before entering
the query resolver: selected phone GLONASS rows contained invalid rows, the
navigation ledger contained aggregate malformed records (`671/480` for MTV-A
and `686/442` for LAX-T), and the base header ledger had one malformed label
entry per route.  Thus the historical zero coverage did not prove a query-time
FCN gap, tie, or header/geph mismatch.  Phase128 addresses only that
parser/admission boundary; it does not waive a required-row failure.

The implementation preserves the old permissive values for the legacy path,
while adding a strict sidecar and typed admission path:

| boundary | Phase128 rule | failure behavior |
|---|---|---|
| Satellite key | `SatelliteId(GNSSSystem::GLONASS, PRN)`; no bare global integer | missing/invalid key rejected |
| Query/toe | existing native `GNSSTime` GPST query and native RINEX GLONASS UTC-to-GPST toe conversion | invalid or uncovered time rejected |
| Nav record | canonical fixed positions `data[0..14]`; no field compaction | that record rejected and counted |
| FCN | read `data[10]`; only raw value `>128` maps to `raw-256`; finite integral `[-7,6]` required | nonfinite, fractional, or out-of-domain rejected |
| Header | explicit `absent`, `valid-empty`, `entries`, `malformed` states | malformed label/slot fails closed; absent/empty may use corroborated geph |
| Selection | exact satellite/query, existing minimum-age and `<=1800 s` validity, tie policy | missing, stale, different-FCN tie, mismatch rejected |
| Phone metadata | `carrierFrequencyHz` is never an FCN source | no inference or fixed-channel fallback |
| Coverage | every retained rover and base GLONASS row is certified and finite | partial coverage prevents solver launch |

Malformed navigation records are counted per record.  An unrelated malformed
record does not poison a valid record, but a malformed record selected for a
required observation fails closed.  Header absence is not treated as header
malformation; a present header still requires exact selected-geph FCN
corroboration.  Duplicate, conflict, tie, out-of-time, and range conditions
remain strict.

## Static implementation review

The pinned implementation contains the following source-backed controls:

1. `glonass_provenance.*` validates all fifteen positional values and applies
   the RTKLIB encoded FCN normalization and Phase127 signed-domain gate.
2. `RINEXReader` records header-label status independently from the legacy
   channel map and records a strict canonical geph sidecar without changing
   legacy parsed values.
3. `phase128_glonass_provenance.*` delegates query-time/header-vs-geph choice
   to the sealed Phase127 helper, then rejects a selected/tied canonical
   record that failed the strict sidecar validation.
4. The native app accepts only
   `--native-phase128-glonass-provenance-parser-admission`; it requires the
   Phase126/127 composed raw-base contract and exposes parser counters in
   structural metadata.  The selector is false by default.
5. The Phase128 annotation path writes only the typed admitted FCN to a local
   observation copy.  It does not read or derive an FCN from phone carrier
   frequency and does not add a factor or state.

Relevant static hashes at audit time:

| artifact | SHA-256 |
|---|---|
| `apps/native/gnss_fgo_imu_no_base.cpp` | `898b979c51d8fa4ec98432a3f2aeb8a99ff2603689e8cbe396aecad501985138` |
| `include/libgnss++/algorithms/phase128_glonass_provenance.hpp` | `fe743866c3c4815b75e87104523c53d914e4f4e7485fc6f10f5d04b20feade55` |
| `src/algorithms/phase128_glonass_provenance.cpp` | `5d930d8be16c70c4956de41822160c0c44c1e2761f893a9a491cb9d1a076bbc7` |
| `include/libgnss++/core/glonass_provenance.hpp` | `f3659dd826f742e381f21f3d6f5ecfbb2cfbbbdf637fa66405bc2073f74b1df6` |
| `src/core/glonass_provenance.cpp` | `ae1db078adcc8db21db6cbc7a660d24be21a681dc8668d6d29e1eda13b34e1cf` |
| `src/io/rinex.cpp` | `308450b371d80105e9d23ff31523a2a7aef32b0cc0a6e69c8b12744c98d0aada` |
| `include/libgnss++/io/rinex.hpp` | `b08705073aaf944daa5e6e899eebc8856e974d07f09282605dd14461b8e38669` |
| `build/apps/gnss_fgo_imu_no_base` | `d823b031d865bde973d3bd6ffcf2957288bccf29d66bf89ade3dea11d108bfe0` |

The implementation commit's qualification was synthetic/launch-free: the
Phase128 C++ focused lane passed 19/19, the Python focused lane passed 8/8,
and the full C++ binary passed 1,160 tests with 1,102 passed, 58 skipped, and
zero failures.  These counts authorize no raw execution.

## Frozen inventory stage

After independent authorization, inventory reads are limited to the raw phone
GNSS/IMU, broadcast nav, and sealed raw base RINEX/station metadata named by
the route manifest.  The runner must read each permitted input only within the
authorized boundary and must not copy or transform payload bytes.  Before any
solver launch it must seal, for both rover and base:

- typed GLONASS satellite identities and exact native GPST query keys;
- header status and uncollapsed entry/duplicate/conflict/malformed ledger;
- per-record canonical nav parse/reject reasons and accepted records;
- selected geph FCN, native toe/query age, tie/mismatch/out-of-time/range
  counts, and finite state/wavelength predicates;
- retained-row denominator, certified-row numerator, and full-coverage result.

The inventory gate is true only when all required retained GLONASS rows on both
sides are certified, all selected values are finite and in domain, and no
strict failure bucket is nonzero.  Inventory failure must seal the route and
keep the native solver count at zero.  The structural Stage 2 gate additionally
requires Phase126 A/B/C and base correction exactly once, Phase127/128 active
exactly once, GNSS-first progress and strict cost decrease, main QR progress
and strict cost decrease, finite exact C7/D/CCDD handoff, finite earth-valid
expected output coverage, and Pixel5 offset exactly once.  Output is opaque
hash/row metadata only.

The static manifest will pin the implementation commit, target binary,
Phase126/127 provenance freezes, Phase118 recipe, exact selector counts,
route order/domain epoch counts, forbidden lineage, and zero pre-raw read
accounting.  The launch-free validator must not import a payload reader or
spawn a process.

## Authorization boundary

This audit and its companion freeze are design/contract artifacts only.  They
do not authorize raw materialization, inventory reads, solver execution,
solution access, truth/accuracy evaluation, or Kaggle.  Before the two-route
matrix, an independent authorization must pin the audit/freeze, implementation,
runner, manifest, pre-raw seal, target binary, and route input path/hash
metadata.  Authorization order is:

`independent authorization -> one inventory pass per route -> inventory gate ->
Phase126 structural solver at most once for an admitted route`.

No historical Phase127 hash artifact is to be rewritten when its old source
pin differs from the new Phase128 implementation.
