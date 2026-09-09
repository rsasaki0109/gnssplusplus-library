# Phase116 carrier/TDCP source-parity audit

Status: read-only source and sealed-record audit.  This audit reads the
official MATLAB source and tracked native source, plus the sealed Phase107 and
Phase112 JSON records.  It does not read a raw phone payload, raw base RINEX
payload or header, truth, MAT data, precomputed phone coordinates, PDC data,
solution rows, or Kaggle resources, and it does not invoke the native solver
or an accuracy evaluator.

## Decision

The official `fgo_gnss.m` and `fgo_gnss_imu.m` use L1/L5 carrier phase to form
an adjacent-epoch, same-satellite/same-signal time-differenced carrier phase
(TDCP) factor.  They do not add a standalone carrier-phase ambiguity factor in
the shown graph, and their base helper corrects pseudorange only.  Native raw
Android parsing creates carrier phase from ADR in metres/cycles, and the
current `--native-pdc-imu-tdcp-no-bridge` recipe already builds and inserts the
same ordinary TDCP family.  The sealed Phase107 and Phase112 records confirm
nonzero TDCP counts and zero double-difference (DD) carrier and pseudorange
counts for both audited routes.

The native generic DD builder is a different path: it requires rover and base
carrier streams and creates a per-arc `AmbiguityState`.  The no-base native
application does not call that builder and rejects any DD factor in its
contract.  Therefore a source-backed, ambiguity-free, base-dependent TDCP/DD
factor is not present to enable.  Adding one would be a new observation model
and graph topology, not a parity-preserving selector change.  The official
base correction also reads precomputed `base_position.csv` and
`base_offset.csv`, so it is not a raw-only base-carrier recipe.

Exactly one candidate is consequently frozen: a default-off,
diagnostic-only carrier/TDCP incidence report.  It observes the existing raw
carrier and TDCP admission path, per signal/epoch counts and rejection
taxonomy, and the explicit absence/admission guard for carrier-phase and DD
families.  It does not change a factor, value, key, state, solver, noise,
filter, LM schedule, base correction, C7/D handoff, Pixel5 offset, output, or
legacy default.  This record is not execution authorization.

## Facts from the official source

The source files are the cached official repository files listed below.  They
are intentionally treated as source evidence only; their MATLAB data inputs
are not opened in this audit.

* `fgo_gnss.m:23,30` loads `phone_data.mat` and selects `FTYPE = ["L1","L5"]`.
  Its epoch graph adds `PseudorangeFactor_XC` and `DopplerFactor_VD`
  (`:123-148`), `ClockFactor_CCDD` (`:154-175`), and, when adjacent carrier
  samples pass the finite/clock-jump checks, `TDCPFactor_XXDD` or
  `TDCPFactor_XXCC` (`:182-200`).
* `fgo_gnss_imu.m:23,36` likewise loads `phone_data.mat` and selects L1/L5.
  Its P/D factors are at `:189-218`; the IMU, clock and TDCP loop is at
  `:252-323`.  The TDCP measurement is
  `resL(i+1,j)-resL(i,j)`, with the device-specific `Loffset` branch for the
  same five phones.  The clock factor uses C/D states, but TDCP is still a
  temporal single-receiver factor, not a rover/base DD factor.
* `functions/exobs.m:12-75` masks carrier by SNR, slip/valid and multipath,
  excludes the documented GLONASS/device combinations, and removes large
  adjacent carrier jumps.  `functions/exobs_residuals.m:82-99` additionally
  forms the carrier/Doppler difference, applies the device offset where
  applicable, and masks the 1.5 m threshold from `parameters.m:117-128`.
* `functions/correct_pseudorange.m:1-34` selects common satellites, computes
  base residuals, smooths them, and linearly interpolates a pseudorange
  correction.  It does not correct carrier or Doppler.  Its base position and
  antenna offset come from `base_position.csv` and `base_offset.csv` at
  `:8-24`; that dependency is not an admissible raw-only phone/base
  ambiguity-free carrier input.

The official topology therefore establishes two separate facts: carrier
phase is used through ordinary adjacent TDCP, while the official base path is
pseudorange-only and its published implementation is not MAT/precomputed-
coordinate-free.

## Facts from the native source

### Raw carrier construction

`src/io/android_raw_gnss.cpp:630-642` records the raw-domain conversion:

* ADR metres are divided by `c/frequency` to obtain carrier cycles;
* the published five-device ADR sign correction is applied;
* Doppler is converted separately as `-PseudorangeRateMetersPerSecond /
  wavelength`.

At `:874-900` the native adapter applies the raw status, SNR, multipath,
loss-of-lock and device-specific carrier masks.  At `:907-977` it stores the
finite carrier phase and carrier observation type.  Thus native carrier input
can be formed from raw Android CSV in the same process; no MAT carrier array
is required.

### Ordinary TDCP path

The frozen defaults in `include/libgnss++/algorithms/fgo_config.hpp:166-168`
are `use_tdcp_factors=true`, `use_carrier_phase_factors=false`, and
`use_double_difference_factors=false`; the app selects TDCP only for the
explicit raw recipe.  `fgo_problems.cpp:628-677` converts usable carrier phase
to metres and applies the existing satellite-clock/troposphere/ionosphere
model.  `:1375-1441` then:

1. looks up an exact `(SatelliteId, SignalType)` in the previous epoch;
2. computes the corrected-carrier and corrected-code differences;
3. applies `tdcp_contract::evaluateAdjacentPair`;
4. stores a `TimeDifferencedCarrierFactor` with the two epoch indices and
   `delta_carrier_m`.

The contract at `include/libgnss++/algorithms/tdcp_contract.hpp:7-67` is
truth-free and adjacent-pair-only.  It rejects invalid/gapped time, clock
discontinuity, configured loss-of-lock, nonfinite measurements, or a code-
phase jump over the configured threshold.  The native raw recipe freezes the
ordinary TDCP sigma at 0.03 m, the maximum gap at 2 s, and the loss-of-lock and
10 m code-phase gates (`gnss_fgo_imu_no_base.cpp:6923-6932`).

`fgo_gtsam_backend.cpp:1193-1232` inserts these ordinary TDCP factors in the
Pose3/IMU graph.  The source-clock meter-state variant uses the C vector only
for its C0 arm; it is not a base DD factor.  Native standalone
`carrier_phase_factors` remain disabled, and the carrier-phase sigma/default
does not turn on a separate ambiguity graph.

### What `--native-pdc-imu-tdcp-no-bridge` actually does

The spelling is historical and does not mean that a PDC state bridge is
enabled:

* `gnss_fgo_imu_no_base.cpp:573-582` maps `no-bridge` to
  `native_pdc_imu_tdcp=true` while leaving `native_pdc_state_bridge=false`.
* `:688-703` requires Android raw input, raw UTC keys, all epochs and rejects
  an explicit state bridge, upstream quality lane, or incompatible MAT path.
* `:6868-6882` builds the existing Pose3 + IMU P/D graph with ordinary TDCP
  enabled, standalone carrier-phase and DD factors disabled, and corrected
  undifferenced Android Doppler enabled.
* The graph is built with `buildPseudorangeProblem` in the native app.  The
  no-base contract at `:8126-8129` rejects nonempty DD pseudorange or DD
  carrier vectors, and `:8132-8143` fails closed when enabled TDCP is empty,
  not fully inserted, or nonfinite.

Accordingly, under the PDC-prohibited no-bridge recipe the active carrier
family is ordinary same-satellite/same-signal temporal TDCP plus the existing
raw IMU/Doppler path.  It is not PDC state bridging, base carrier DD, or an
ambiguity solver.

### Generic DD path is not ambiguity-free

`FGOProcessor::buildDoubleDifferenceProblem` at
`fgo_problems.cpp:1486-1497` is separate from the app's builder.  It requires
rover and base epochs, navigation and a valid base ECEF position.  Its carrier
construction (`:2168-2245`) uses common rover/base carrier streams.  At
`:2317-2354` each carrier arc creates an `AmbiguityState` with wavelength and
initial ambiguity before the DD carrier factor is appended.  The backend
inserts the real ambiguity key plus a pinned dummy key at
`fgo_gtsam_backend.cpp:1276-1321`, and ambiguity priors are handled at
`:1324-1337`.

This is evidence of an ambiguity-bearing DD implementation, not an
ambiguity-free TDCP/DD implementation.  It is also outside the current
no-base app path and would violate this audit's no-new-topology boundary if
selected merely to obtain more carrier rows.

## Sealed Phase107/Phase112 evidence

Only the tracked sealed JSON records were read; their raw/base member bytes
were not opened.

| Route | Record | ordinary TDCP factors built | DD carrier factors | DD pseudorange factors | selected solver |
|---|---|---:|---:|---:|---|
| MTV-A | Phase107 result | 31,269 | 0 | 0 | `MULTIFRONTAL_QR` |
| LAX-T | Phase107 result | 14,012 | 0 | 0 | `MULTIFRONTAL_QR` |
| MTV-A | Phase112 structural result | 31,269 | 0 | 0 | `MULTIFRONTAL_QR` |
| LAX-T | Phase112 structural result | 14,012 | 0 | 0 | `MULTIFRONTAL_QR` |

Phase112 also records `base.tdcp_applied=false` and describes the base scope
as “adopted undifferenced FGO pseudorange factors only”.  Its output epoch
records retain 2,159 MTV-A epochs and 1,466 LAX-T epochs, with the DD counts
still zero.  The C7/C0-D meter-state and QR telemetry in those records is
orthogonal to carrier-family admission; it does not imply that base carrier
DD was active.

The sealed evidence therefore proves activation of ordinary TDCP and proves
non-activation of standalone carrier-phase and DD carrier factors in these
recipes.  It does not prove that a new base-dependent ambiguity-free factor
would improve accuracy, and no such accuracy claim is made here.

## Raw-only viability and candidate comparison

| Candidate | Source support | Raw-only status | Decision |
|---|---|---|---|
| Enable existing ordinary phone TDCP | Already implemented and active; official topology matches adjacent same-satellite/same-signal TDCP | Yes; same-run raw ADR is sufficient | Reject as a no-op, not a new candidate |
| Add/enable base carrier DD or a base-dependent ambiguity-free TDCP | Native generic DD is ambiguity-bearing and unused by the no-base app; official base helper is PR-only and uses precomputed base tables | Not source-parity; would change graph/state topology and risks PDC/DD admission | Reject |
| Observe carrier/TDCP incidence and rejection taxonomy without changing the graph | Uses existing raw carrier/TDCP preparation and counters; no new factor or value | Yes, but future execution must remain raw-only and solution-withheld | Freeze exactly this one |

The ordinary phone TDCP answer is therefore “yes, already present and
ambiguity-free with respect to a receiver's adjacent phase difference,” while
the requested raw phone + raw base ambiguity-free DD answer is “no supported
implementation is present.”  A base carrier DD can be built by the generic
library only as an ambiguity-bearing factor path, and the published base
correction does not turn it into TDCP.

## Frozen candidate boundary

Candidate ID: `phase116-raw-carrier-tdcp-incidence-diagnostic-v1`.

The future default-off diagnostic may copy counters after existing admission
decisions and report, per route and signal where available:

* raw carrier rows presented by the Android adapter;
* exact-key carrier rows retained for each problem epoch;
* adjacent candidate pairs, accepted TDCP pairs, inserted ordinary TDCP
  factors, and each existing rejection reason;
* ordinary TDCP, standalone carrier-phase, single-difference TDCP and DD
  carrier/pseudorange family counts;
* whether a base-carrier/DD builder or PDC bridge was requested, admitted, or
  rejected by the existing no-base guard.

The diagnostic must not reinterpret a rejected observation as raw, zero, or
endpoint-filled data.  It must not create a carrier phase, TDCP, DD,
ambiguity, C7, D, ISB, prior or output row.  The following remain exact
invariants: C7/D key alignment and units, existing C0D equation and sigma,
Phase99 QR branch, raw-base pseudorange correction scope, final Pixel5 offset,
all carrier/TDCP masks and sigma/robust settings, IMU/P/D factors, LM
schedule, legacy selector-off behavior, fail-closed guards, and solution
withholding.

The diagnostic is not an accuracy or truth evaluator and is not authorized to
read or publish a solution.  It may be implemented and separately qualified
only after the parent freezes that boundary; this Phase116 audit itself
authorizes no raw/base/solver/truth execution.

## Evidence pins and read accounting

Observed source hashes (official cached source is ignored output and is not
added to the repository):

| Source | SHA-256 |
|---|---|
| `output/reproducibility-cache/gsdc2023/fgo_gnss.m` | `5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3` |
| `output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m` | `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3` |
| `output/reproducibility-cache/gsdc2023/functions/exobs.m` | `e22db383f372f26597333d674a12ee1e2adfdc97b00828fbbc5470903947f456` |
| `output/reproducibility-cache/gsdc2023/functions/exobs_residuals.m` | `50c954a825edbdeaf5c9884486aff56c058f51d47610a06722d4f42b33095324` |
| `output/reproducibility-cache/gsdc2023/functions/obserrmodel.m` | `43d0671a25c81fa6d7df0e1fbe81bf48a14eb3a35c8399600200b44961a7f9ef` |
| `output/reproducibility-cache/gsdc2023/parameters.m` | `518925e9c75c7a14fceb5cd99432fe883311e0d64c72b94396744ac765120f52` |
| `output/reproducibility-cache/gsdc2023/functions/correct_pseudorange.m` | `b0536ccff478b0aff253448ffb7a203c715b8064dd8dc85898e38f1f05d0441e` |
| `src/io/android_raw_gnss.cpp` | read as tracked source; current hash is not needed for the freeze boundary |
| `src/algorithms/fgo_problems.cpp` | `e7607ce8f0fe27fd382a01b5b6f147b027b7bf51453e920eae458c86ddac0c17` |
| `src/algorithms/fgo_gtsam_backend.cpp` | `78792c9306abe339228a3004e3d23f1486fac77822d0f1785b69dc07af14f578` |
| `apps/native/gnss_fgo_imu_no_base.cpp` | `301265974631f20781ff4460e1ede4032165e11c79ef59d0510a273aebb1a59e` |
| `include/libgnss++/algorithms/tdcp_contract.hpp` | `1d56692d93f216703e2283b206b7bd0bca9c6593d51524d843fc6e385d96cbcf` |

The sealed records used for the numeric structural claims are:

* `docs/use_cases/records/smartphone_r5_phase107_raw_base_source_parity_manifest_v1.json`
  (SHA-256 `b46a4bfa23ac1604d9e51487734be18262f027612eede40fdbcba15fa75c2a9c`)
* `docs/use_cases/records/smartphone_r5_phase107_raw_base_source_parity_result_v1.json`
  (SHA-256 `e7eb3f1d2670672414744a100dc1e2fc3e0b019ae7efc0dbfef1518300ce8d6e`)
* `docs/use_cases/records/smartphone_r5_phase112_main_output_offset_manifest_v1.json`
  (SHA-256 `d8be91f42f07196e07b293558cc6b83dd261226af577c1b26b167eead75902e4`)
* `docs/use_cases/records/smartphone_r5_phase112_main_output_offset_structural_result_v1.json`
  (SHA-256 `087b1bf51b4e584b86a7b558560df4ceb78e19b01e572f2d60aeef4c3d759ba0`)

Read accounting for this audit:

| Resource/action | Count |
|---|---:|
| official/native source reads | allowed source inspection; no payload |
| sealed manifest/result reads | 4 records |
| raw phone payload reads | 0 |
| raw base payload/header reads | 0 |
| raw/base member hash reads | 0 |
| MAT/data payload reads | 0 |
| truth reads | 0 |
| precomputed phone-coordinate reads | 0 |
| native solver invocations | 0 |
| accuracy evaluations | 0 |
| solution rows opened/published | 0 |
| PDC/Kaggle/token access | 0 |

