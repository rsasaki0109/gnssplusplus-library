# Smartphone R5 Phase131 inventory-first structural contract audit

- Execution label: `Luna Max`
- Audit date: `2026-09-04`
- Repository: `/home/sasaki/workspace/rtklib_v2_ws/gnssplusplus-library`
- Implementation pin: `c3af051e46f3685c0d3406537fa8f7c12eea75f2`
- Scope: read-only source and sealed-contract audit for a launch-free
  inventory/structural contract.

This audit does not authorize raw materialization, raw payload reads, a native
solver, truth/MAT/PDC/precomputed-coordinate access, accuracy evaluation, or
Kaggle access.  The later raw authorization boundary is intentionally separate.

## Disposition

The Phase131 implementation matches the previously frozen candidate
`phase131-canonical-correction-band-key-v1`.  The implementation changes only
the base-correction lookup boundary.  It retains the original typed
`SignalType` on each FGO factor and carries certified GLONASS FCN provenance
separately.  It does not add a factor or state and does not alter an equation,
unit, sigma, filter, robust kernel, C7/D/CCDD state, TDCP/IMU graph, QR/LM
setting, output calculation, or offset.

The structural contract therefore admits exactly one candidate and exactly two
ordered routes, MTV-A then LAX-T, one run per route.  It is inventory-first:
the raw input inventory and canonical correction support ledger must pass before
the native process can be considered launchable.  A failed inventory has zero
solver invocations.  The launch-free artifacts created from this audit contain
placeholders only; they do not open or hash payloads.

## Source and sealed evidence

The preceding Phase131 audit/freeze established the source-backed key:

`K = (GNSSSystem, SatelliteId.prn, PhysicalFrequencyFamily [, certified GLO FCN])`.

The relevant implementation boundaries are:

| area | source evidence | contract consequence |
|---|---|---|
| typed family mapping | `include/libgnss++/algorithms/phase131_canonical_correction_key.hpp` | only source-defined L1/L5 families are admitted; unknown bands are explicit misses |
| RINEX aliases | `familyForRinexObservationType` delegates to `signal_policy::trySignalForObservationType` | tracking-code spelling is retained as provenance, but not used as the correction join key |
| Android rows | `familyForAndroidSignalType` consumes the already typed `SignalType` | Android carrier frequency is not a GLONASS FCN source |
| GLONASS provenance | `fgo_problems.cpp` carries the exact Phase127/128-certified channel into `PseudorangeFactor` | missing, invalid, or non-GLO FCN is fail-closed; no fixed/inferred channel |
| base correction | `base_pseudorange_compensation.cpp` builds `canonical_streams_` only after finite source-complete residual admission | CA/P and equivalent same-band aliases can share a family key; cross-band fill is forbidden |
| factor admission | `source_pseudorange_miss_mask::applyCanonical` | one finite in-domain correction per retained factor, transactional swap, exactly-once marker |
| CLI/config | `apps/native/gnss_fgo_imu_no_base.cpp`, `fgo_config.hpp`, and base config | selector is default-off and requires Phase126–130 composition plus the frozen Phase118 recipe |

The implementation pin is checked by the contract manifest.  Its target binary
is likewise pinned by SHA-256; no execution is implied by that hash.

## Inventory contract

The inventory stage is post-authorization and pre-solver.  It records typed
rows on each side as either certified or an explicit local miss:

`input_rows = certified_rows + explicit_local_miss_rows`.

Certification source counts must also partition exactly into header-primary and
time-valid broadcast-ephemeris provenance.  Each row carries its original
literal tracking-code/SignalType provenance, but admission uses only the
canonical family key and certified FCN where applicable.

For every canonical key:

1. Android and RINEX aliases may collapse only when the source-locked mapping
   yields the same physical family.
2. GLONASS FCN must be present, in `[-7, 6]`, and certified at the exact GPST
   query time by Phase127/128.  FCN mismatch, tie, conflict, missing, or
   out-of-validity is an explicit miss or global fail according to the existing
   provenance ledger; it is never inferred.
3. A different physical family is not a match.  An unknown family is an
   explicit miss.  A divergent/ambiguous multi-code stream, duplicate canonical
   time, or non-monotonic stream is fail-closed; no first/last/nearest winner is
   selected.
4. Every retained rover code factor has one finite certified base correction at
   an exact endpoint/interior sample or an adjacent finite in-domain bracket.
   Row index pairing, nearest/hold/extrapolation, raw/zero correction, and
   cross-band/cross-satellite fill are forbidden.
5. Base rows/streams not used by any rover factor are permitted but must be
   counted as unused.  Whole rover/base count or reason-map equality is not an
   admission predicate.

The validator checks both the side-local ledger and the shared correction
ledger.  An all-miss or empty usable population fails before launch.  A global
Phase126 A/B/C source-complete failure remains a route abort.

## Structural solver gates

Only after inventory admission may a future independently authorized runner
launch the pinned native binary once for a route.  The structural result must
keep solution content opaque and record only hash/row metadata.  It must show:

- Phase126–131 selectors and Phase118 official TDCP Huber mode exactly once;
- Phase117 dynamic TDCP sigma, Phase120 atmosphere-cancellation, and
  additional-frequency selectors absent;
- GNSS-first and main accepted iterations greater than zero with finite strict
  cost decrease;
- Phase131 canonical rows/rejections/conflicts/duplicate counters and exact
  source-miss/factor conservation;
- finite expected epoch/output coverage, C7/D/CCDD/QR/IMU/TDCP/base/offset
  exactly-once telemetry, and no fallback/rerun; and
- no published solution coordinates and no truth/accuracy reads.

The launch-free validator and synthetic tests exercise these predicates in
memory.  They do not materialize input paths, start subprocesses, read
solution rows, or inspect truth data.

## Risks and fail-closed boundary

The sealed Phase130 inventories did not contain distinct rover signal-token
lists, so this contract does not claim a route-level recovery count.  It only
defines the source-backed canonical admission and the accounting needed to
measure it later.  A malformed inventory, unsupported mapping, missing FCN,
ambiguous key, missing support, invalid source-complete state, or exposed
partial factor vector must stop the route before solver launch.  No fallback,
repair, rerun, sweep, or truth evaluation is permitted in this contract.

## Read accounting for this audit

| activity | count |
|---|---:|
| official/native source text reads | read-only |
| sealed Phase131/Phase130 metadata reads | read-only |
| raw phone GNSS/IMU/navigation payload reads | 0 |
| raw-base RINEX payload/header reads | 0 |
| raw copies/transforms | 0 |
| solver invocations | 0 |
| solution/truth coordinate-row reads | 0 |
| MAT/PDC/precomputed-coordinate reads | 0 |
| accuracy calculations/Kaggle access | 0 |
| reruns/fallbacks/repairs/sweeps | 0 |

