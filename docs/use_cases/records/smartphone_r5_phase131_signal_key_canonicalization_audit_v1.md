# Smartphone R5 Phase131 rover/base signal-key canonicalization audit

- Execution label: `Luna Max`
- Audit date: `2026-09-04`
- Repository: `/home/sasaki/workspace/rtklib_v2_ws/gnssplusplus-library`
- Branch: `feat/rtk-smartphone-performance-pr`
- HEAD at audit start: `3d16fe289409d3aa6290d861ec23d51f69941db5`
- Worktree at audit start: clean.
- Scope: read-only forensic audit of the sealed Phase130 inventory/result and
  the official MATLAB and current native source mappings.

This record does not authorize code implementation, raw materialization, a
solver run, truth/accuracy evaluation, MAT/PDC/precomputed-coordinate access,
or Kaggle access.  No phone GNSS/IMU/navigation payload, raw-base RINEX
payload, solution row, truth row, or coordinate payload was opened.  The two
Phase130 route inventories were read only as already-sealed metadata; their
payload files were not reopened.

## Finding and disposition

The Phase130 zero-intersection result is real for the Phase130 boundary's
textual descriptor predicate, but it is not evidence that the official
rover/base correction has no common signal.  The earliest failed predicate is
the boundary's exact descriptor test, before GLONASS provenance, ephemeris
selection, or time-support lookup.  Phase130 stored counts and the derived
`retained_exact_keys: []`, but did not seal the distinct rover signal-token
set.  Therefore the exact route token that caused every miss cannot be named
as a fact without reopening forbidden raw content.

The official path is frequency-family based.  `fgo_gnss.m` and
`fgo_gnss_imu.m` call `correct_pseudorange` once for each `FTYPE = ["L1",
"L5"]`; `correct_pseudorange.m` first keeps common satellites and then
interpolates the selected frequency-field residual at rover times.  The
native raw loader maps the supported Android GLONASS aliases to
`GLO_L1CA`, while the RINEX policy maps a GLONASS band-1 code to
`GLO_L1CA` or `GLO_L1P` based on the code character.  The native base model
currently keys the stream by `(SatelliteId, SignalType)`, so its CA/P split is
stricter than the official frequency-family correction field.

Exactly one source-backed, default-off candidate is consequently frozen in
the companion JSON:

`phase131-canonical-correction-band-key-v1`

with proposed selector:

`--native-phase131-canonical-correction-band-key`

The candidate is a join/admission-boundary change only.  It canonicalizes a
rover and base observation to `(constellation, satellite PRN,
physical-frequency family)` and appends a certified GLONASS FCN provenance
component for GLONASS.  It does not add a factor, state, measurement, code
value, correction, fallback, or tuning.  Implementation and raw execution
remain unauthorized by this audit.

## Sealed Phase130 evidence

The authoritative sealed structural result is:

| item | value |
|---|---|
| result commit | `3d16fe289409d3aa6290d861ec23d51f69941db5` |
| result JSON SHA-256 | `f9cf9226889fc4a4e6141e54f6447bc09998b3f42a7cf45269cc8ed0cb0e0553` |
| result MD SHA-256 | `46f9a9d56a8ce7afb452c323f2b3c1894a626943cab5575b3069e9a7a90b6a79` |
| Phase130 contract audit | `355b6225bcc3bc838dad8a3044bc3aadd34d80f3` |
| Phase130 contract freeze | `94085c90ac33ce987c3b9d29a6462c5d77795e88` |
| Phase130 runner/validator | `36b0977e47d9b24eda975ab224e05aadfdc860bb` |
| Phase130 manifest | `ab1a6bfa2e1878bb01d784a2f926a1732d34b004` |
| Phase130 pre-raw seal | `e2821f8b5c2c8ebfd6e0a1078cf7cb5fe89ae9b6` |
| Phase130 raw authorization | `787b61b56491027e548689f684bf6a29fb9ba686` |

The result is `no-go-phase130-shared-ledger-key-local-support-structural`.
Both routes stopped before the solver, with zero solution rows opened and no
truth/accuracy calculation.

### Route metadata (facts)

| route | valid rover GLONASS rows | rover `missing-exact-key` | base GLONASS rows | base certified/miss | base RINEX R codes | streams / finite samples | retained exact keys |
|---|---:|---:|---:|---:|---|---:|---:|
| MTV-A `2021-03-16-18-59-us-ca-mtv-a/pixel5` | 11,832 | 11,832 | 17,199 | 9,642 / 7,557 | `C1C,L1C,S1C,C2P,L2P,S2P` | 18 / 34,398 | 0 |
| LAX-T `2022-04-01-18-22-us-ca-lax-t/pixel5` | 8,626 | 8,626 | 920 | 396 / 524 | `C1C,L1C,S1C,C2C,L2C,S2C,C2P,L2P,S2P` | 27 / 2,760 | 0 |

The Phase130 runner's audited descriptor transformation maps the MTV-A R
codes to the descriptor set `{"1C","2P"}` and the LAX-T R codes to
`{"1C","2C","2P"}`.  This is derived from the sealed header code list and
the runner function; it is not a distinct rover key inventory.

The sealed result has no `rover_distinct_signal_tokens`, no per-rover
`(satellite, signal)` key list, and no successful pre-descriptor comparison.
The ignored per-route inventory JSONs likewise contain only counts and
`retained_exact_keys: []`; they do not supply the missing distinct rover key
set.  Consequently:

* Fact: the Phase130 descriptor intersection was zero for all 11,832 and
  8,626 valid rover GLONASS rows, respectively.
* Fact: the zero occurred while recording `missing-exact-key`; no row reached
  the subsequent nav/provenance or interpolation branch.
* Not established: the exact raw Android signal token(s), satellite-specific
  key distribution, or whether a particular base code had a same-satellite
  time support.  Establishing those would require a new authorized raw run.

The relevant Phase130 runner order is visible at
`gnss_smartphone_phase130_shared_ledger_structural_authorized_execute.py:495-509`:
`signal_descriptor`/descriptor membership is checked first; only a matching
descriptor proceeds to `resolve_query` and then `classify_stream`.  The
sealed `missing-exact-key` totals therefore identify the earliest failure
boundary, without implying a nav or time-domain cause.

## Source-pinned mapping evidence

The following source blobs were read and hash-checked.  These are source
files, not raw payloads.

| source | SHA-256 | relevant lines and conclusion |
|---|---|---|
| `output/reproducibility-cache/gsdc2023/fgo_gnss.m` | `5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3` | `30`, `72-78`, `123-148`: official fields are `L1/L5`; correction is applied per field, then finite P factors are added per satellite/field observation. |
| `output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m` | `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3` | `36`, `85-92`, `202-218`: the IMU graph uses the same frequency-field correction and per-observation finite gate. |
| `output/reproducibility-cache/gsdc2023/functions/correct_pseudorange.m` | `b0536ccff478b0aff253448ffb7a203c715b8064dd8dc85898e38f1f05d0441e` | `5-6`, `26-34`: common satellite selection, frequency-field residual, moving mean, and rover-time linear interpolation. |
| `output/reproducibility-cache/gsdc2023/functions/gnsslog2obs.m` | `665ce43d1fc3c5a9c3b3a6fd76f81539126e3ab897fbba484ab0ccd18964acff` | `91-95`, `158-188`, `254-290`: Android carrier frequency is used to classify physical frequency/family; P/L/D units are converted before FGO. |
| `output/reproducibility-cache/gsdc2023/functions/gnsslog2obs.m` | same as above | `300-316`: GLONASS L1 family is mapped to `C1C`; tracking-code text is not the FGO key. |
| `output/reproducibility-cache/gsdc2023/functions/sysfreq2sigtype.m` | `5ec1d299e04b45f604921cbae3b1fac8f78f3f76a0152496d86011a32f1d0079` | `1-16`: FGO `L1` maps GLONASS to one signal slot (`1`), and `L5` maps supported systems to their family slots. |
| `include/libgnss++/core/types.hpp` | `475d524280fec85d3e959e318447f917a614d93314207a04f84501b0f7735ee3` | `28-60`, `174-194`: `SignalType` includes separate GLONASS CA/P variants; `SatelliteId` is system plus PRN. |
| `include/libgnss++/core/signals.hpp` | `bb8cd20d4a22581ed2beb83f4e1b4ed6d397f28a2d5104d4736778c5b279ba56` | `26-39`, `116-139`: GLONASS CA/P share L1/L2 physical formulas, with certified channel changing frequency/wavelength. |
| `include/libgnss++/core/signal_policy.hpp` | `d3f8edbdd785f0292c2b8a1596248c00a2b5e9284c09de439e5c7340c1dbca99` | `40-108`, `110-145`, `172-235`: RINEX band and `P` code select CA/P; primary/secondary priority is source-defined. |
| `src/io/android_raw_gnss.cpp` | `a020ce900db6a9d3a994cc3adb3dcebbe84cbf67c1ff9793991b7a75039529c5` | `202-315`, `815-935`, `965-975`: accepted Android aliases/frequency map to typed signals; GLO FCN is retained on the observation but carrier frequency is not a Phase127 provenance source. |
| `src/io/rinex.cpp` | `308450b371d80105e9d23ff31523a2a7aef32b0cc0a6e69c8b12744c98d0aada` | `149-186`, `209-274`, `1586-1715`: RINEX code is parsed by system/band, selected by source priority, and exact tracking records are kept separately. |
| `include/libgnss++/io/android_raw_gnss.hpp` | `e833570103a521b9e051086b6ea75192ae3a575b83eee33d978143eaf279c52c` | `185-205`: raw Android P/L/D conversion is a boundary contract; imported coordinates are not part of this audit. |
| `include/libgnss++/algorithms/base_pseudorange_compensation.hpp` | `f6e5e9c1436138e9b0ef943d8b8fa68f3deca51bca650bab71526703d3ad0272` | `25-27`, `125-158`: native stream key is `(SatelliteId, SignalType)` and lookup is exact-key/in-domain. |
| `src/algorithms/base_pseudorange_compensation.cpp` | `b6c6a9b6a2f29c66c4f2abecbe8d7e56c4d7cf81e9c145273c3ea1255da6eb9d` | `137-145`, `499`, `536-581`, `584-621`: native keyed streams, monotonic finite samples, exact endpoint/interior interpolation, no extrapolation. |
| `src/algorithms/source_pseudorange_miss_mask.cpp` | `291ff2591dbd62ecba92c24d1d74c686274568f2af6182bc4dc52f237b0b9946` | `53-181`: current factor admission uses exact typed key and local missing/domain/nonfinite reasons with conservation. |
| `include/libgnss++/algorithms/phase127_glonass_channel_provenance.hpp` | `dc1de4ae6de270f306f1c07a54ab1799df26344f8cc6fdd910d7abfaf1914c8b` | `66-104`: FCN domain `[-7,6]`, exact query-time provenance, no inferred channel. |
| `src/algorithms/phase127_glonass_channel_provenance.cpp` | `cc8e3105abc8367786b089e8d0492f8217550f0a69473869ae8a89cc01941bb0` | `69-246`: header/geph selection, time validity, duplicate/conflict/mismatch fail-closed, certified channel result. |
| `apps/commands/benchmarks/gnss_smartphone_phase130_shared_ledger_structural_authorized_execute.py` | `47ca20d4417841a44766c696bfd69e8c1723fd02cfaf667d7cf65b1f332aa1c4` | `394-409`, `469-509`: textual descriptor normalization and the earliest zero-intersection branch. |

## Itemized semantic comparison

| item | official semantics | native/current boundary | classification | Phase131 consequence |
|---|---|---|---|---|
| satellite identity | `sameSat` retains common satellites; no row-index pairing | `SatelliteId` is `(GNSSSystem, PRN)` | equivalent | retain exact system/PRN; never join by row index |
| official correction frequency | `FTYPE = L1,L5`; one `obs.(f).resPc` stream per satellite/family | typed `SignalType` carries family plus CA/P distinctions | family equivalent, key divergent | canonicalize to physical family at correction join |
| Android physical band | `estimatecarrfreq` snaps `CarrierFrequencyHz` to constellation frequency candidates, then `freq2ftype` | `parseSignal` accepts token/frequency aliases and emits typed signal; source frequency is metadata, not FCN provenance | equivalent physical classification; textual aliases divergent | map aliases through typed band, never compare raw token text |
| GLONASS L1 | all valid FDMA channels are assigned `L1`; `sysfreq2code` uses `C1C`, `sysfreq2sigtype` uses one GLO L1 slot | `GLO_L1CA` and `GLO_L1P` are distinct enum values but share L1 physical formula | official/native correction-family divergence | collapse CA/P only for correction-band join, while retaining source signal and conflict ledger |
| GLONASS L2 | native policy has L2CA/L2P and FCN-dependent wavelength | official cached FGO `freq2ftype` does not assign GLO L2 to the frozen `L1/L5` FGO fields | unresolved/out of frozen official scope | do not admit a new L2 stream through this candidate |
| RINEX code identity | `C1C/L1C/S1C` occupy the L1 field; code tracking text is not passed as the FGO signal key | parser uses system+band and P-vs-other policy; exact tracking map is separate | exact text is not equivalent | map `R:C1C` (and source-equivalent L1 CA forms) to GLO/L1; map C1P to same physical family only with source-selected precedence |
| GLO FCN | official frequency ingestion selects physical channel before family grouping | Phase127 requires exact query-time header/geph certification and annotates the typed observation | provenance must be preserved | append certified FCN to the GLO canonical key; never infer/fix it |
| time support | `interp1` at rover time over the selected family stream, no explicit extrapolation | `correctionAt` exact endpoint or adjacent finite bracket, in-domain only | equivalent | preserve exact endpoint/bracket and local miss policy |
| multi-code same band | official struct has one selected field/slot after ingestion | RINEX selection uses `signalPriority` and tracking-code continuity; ambiguous source combinations are not a free-form alias | source mapping partly explicit; unresolved if multiple streams survive | use source-defined precedence; if two finite candidates remain ambiguous, fail closed |
| factor admission | finite corrected pseudorange gates a factor; Doppler/TDCP are separate | miss-mask subtracts one finite base correction and swaps only after conservation | equivalent | candidate must not alter factor topology or equations |

### Android and RINEX mapping table

The following is a source-defined mapping, not a claim about an unsealed raw
route token inventory.

| source representation | typed/native representation | canonical correction family | FCN component |
|---|---|---|---|
| Android `GLO_G1_CA`, `GLO_G1C`, `GLO_L1`, or accepted frequency-only GLO L1 alias | `GLO_L1CA` from `parseSignal` | `GLONASS/L1` | required certified Phase127 FCN |
| RINEX `R:C1C` / `L1C` / `S1C` after native selection | `GLO_L1CA` via `signal_policy` | `GLONASS/L1` | required certified Phase127 FCN |
| RINEX `R:C1P` / `L1P` / `S1P` if declared and selected | `GLO_L1P` via `signal_policy` | `GLONASS/L1` | required certified Phase127 FCN |
| RINEX `R:C2C` or `C2P` | `GLO_L2CA` or `GLO_L2P` | `GLONASS/L2` (native-only unless the frozen official field is explicitly extended) | required if admitted |
| GPS/Galileo/BeiDou supported primary/tertiary typed signals | existing native enum | corresponding source physical family (`L1` or `L5` where official FGO has that field) | none |

The first three rows are the relevant source-backed Phase131 case.  The
`GLO_L1CA`/`GLO_L1P` distinction remains available for the native factor and
diagnostics, but is not used as the canonical base-correction band.  A
certified FCN is a provenance discriminator and wavelength input, not an
Android carrier-frequency guess.

## Before/after key examples (illustrative, source-defined)

These examples are deliberately labelled illustrative.  They are not raw
route samples and no raw payload was opened.

| Phase130 boundary representation | Phase131 canonical representation | reason |
|---|---|---|
| rover token `GLO_G1_CA`, satellite `R07` -> `signal_descriptor = None` in the existing text helper | `(` `GLONASS`, `7`, `L1`, `FCN_certified` `)` | Android parser accepts the alias and official/native correction is L1-family based |
| base header `R:C1C` -> text descriptor `1C` | `(` `GLONASS`, `7`, `L1`, `FCN_certified` `)` | RINEX system+band policy maps C1C to GLO L1CA; exact code suffix is not a correction key |
| base header `R:C1P` -> text descriptor `1P` | same L1 family only if source precedence selects/validates it | CA/P share physical L1, but multiple finite source streams must not be arbitrarily merged |
| native `(SatelliteId(R,7), GLO_L1P)` vs rover `(SatelliteId(R,7), GLO_L1CA)` | same canonical GLO/L1/FCN key | removes the non-official CA/P key split only at correction join |
| rover/base different epoch counts or row multiplicity | same canonical key queried at each rover GPST | official interpolation is time keyed, not row-count keyed |

The first example is the most plausible explanation for the sealed all-row
zero, because the helper explicitly rejects underscore-bearing `GLO_G1_CA`
and `L1` aliases.  It remains an inference about the omitted raw token, not a
route fact.  The fact-level conclusion is that textual alias normalization
was incomplete and ran before any typed/source mapping.

## Frozen Phase131 candidate contract

The candidate's canonical key is:

`K = (GNSSSystem, SatelliteId.prn, PhysicalFrequencyFamily [, certified_GLO_FCN])`

where:

1. `GNSSSystem` and PRN come from the typed satellite identity; no string
   prefix or row index is used.
2. `PhysicalFrequencyFamily` is the source-defined family used by the frozen
   official FGO field (`L1` or `L5`); unsupported/unknown family is a hard
   local admission miss, not a guessed alias.
3. For GLONASS, a Phase127/128 certified FCN in `[-7,6]` is mandatory for an
   admitted correction key and is included in the key/provenance ledger.
   Android `CarrierFrequencyHz` may describe the already parsed physical
   observation but cannot certify or replace this FCN.
4. RINEX code/tracking text is mapped through the existing system+band policy.
   It is not compared literally to Android `signal` text.
5. If multiple finite source streams collapse to one canonical key, the
   existing source-defined selection/priority must identify one winner.  If
   source metadata cannot establish a unique winner, or if selected streams
   disagree, the key is rejected as `canonical-key-conflict`; no arbitrary
   first/last/nearest winner is allowed.
6. A retained rover code factor still requires one certified finite base
   correction at its exact query time or adjacent finite in-domain bracket.
   Missing canonical key, missing support, out-of-domain, nonfinite, or
   provenance conflict is an explicit typed miss; raw, zero, cross-band,
   nearest, hold, extrapolated, or uncorrected fallback is forbidden.

The candidate changes only the join key used to locate the existing base
correction stream.  It preserves the Phase126 A/B/C source-complete
transaction, Phase127/128 provenance, Phase129 local-miss semantics, the
Phase130 side-local accounting, correction equation, moving mean,
interpolation, C7/D/C0D/CCDD, TDCP, IMU, QR, sigma, filter, LM, output, and
factor topology.

### Conservation and admission invariants

For each side `s` and canonical key `K`:

`eligible_s(K) = certified_s(K) + explicit_local_miss_s(K)`.

For each retained rover factor:

`retained(r) => certified_rover(r) AND certified_GLO_FCN(r) [if GLO] AND one certified finite base support(K,t_r) AND finite(P_raw - pc(K,t_r))`.

The validator must additionally enforce:

* every input row is retained or assigned one explicit reason;
* every base source row/stream is retained, rejected, or marked unused;
* duplicate canonical rows and ambiguous multi-code collapse are fail-closed;
* exact endpoint/interior bracket support is counted; no extrapolation or
  endpoint hold is admitted;
* local misses are shared between the rover factor ledger and its correction
  ledger without requiring equal rover/base totals;
* an unused base stream is telemetry, not a failure, if all retained rover
  factors have support;
* all-miss/empty usable populations fail before solver launch;
* exactly-once correction and no second application remain hard gates; and
* a failure never exposes a partially remapped factor vector.

## Candidate risks and limits

The sealed artifacts do not contain raw distinct rover signal tokens or
satellite-level intersections.  Therefore this audit cannot quantify how many
rows would be recovered by canonicalization, nor prove that all matching
satellite/time streams have in-domain support.  The Phase131 candidate is
source-backed as a correction-key design, not an accuracy claim.

The native reader's selected observation vector and its separate literal
tracking-code map are different representations.  A future implementation
must not accidentally merge carrier/Doppler/TDCP identities or add a second
global ISB/state key.  GLONASS L2 and non-frozen official frequency families
must remain fail-closed unless a separately sourced field mapping is proven.

## Required focused tests before reauthorization

No tests requiring raw files or a solver were run in this read-only audit.  A
future implementation must first add source-locked synthetic tests for:

1. Android accepted aliases (`GLO_G1_CA`, `GLO_G1C`, `GLO_L1`, `L1` with
   explicit physical frequency) collapsing to one typed L1 family;
2. RINEX `R:C1C`, `R:C1P`, and other declared band-1 forms mapping through
   `signal_policy`, with no literal Android-token comparison;
3. certified FCN required for GLO; missing, out-of-range, mismatch, tie, or
   conflict fails closed and never uses the carrier-frequency column as FCN;
4. one source-priority winner for a multi-code family, and ambiguous finite
   duplicates rejected without choosing first/last/nearest;
5. same satellite/family with unequal rover/base cadence, epoch count, and
   row multiplicity; exact endpoint and adjacent bracket accepted;
6. missing canonical key, out-of-domain, nonfinite, all-miss, and empty-route
   local/global gates;
7. side-local row conservation, canonical factor/miss conservation, unused
   base-stream accounting, and exactly-once correction;
8. non-GLONASS existing semantics, C7/D/C0D/CCDD, TDCP/IMU, and factor count
   invariance; and
9. selector-off legacy behavior byte/semantic regression and no partial
   selector acceptance.

## Future reauthorization boundary

This audit freezes design only.  Before any raw read, a separate implementation
commit, focused-test result, launch-free runner/validator/manifest commit, and
pre-raw zero-accounting seal must be pinned.  Then an independent one-shot
authorization must pin all commits, the target binary, the two existing raw
routes in order (MTV-A then LAX-T), and exactly one run per route.  Raw phone
GNSS/IMU, broadcast navigation, and sealed raw base RINEX are the only allowed
inputs.  The route runner must materialize/read payloads only after
authorization, validate canonical inventories first, launch the solver only
after the inventory gate, and seal opaque solution hashes/row counts only.
Any failed route is sealed NO-GO with no rerun, fallback, repair, truth read,
accuracy calculation, or publication.  A new authorization is required even
if an earlier Phase130 authorization is reused as provenance.

## Read accounting for this audit

| activity | count/status |
|---|---:|
| official/native source text | read-only |
| sealed Phase130 result/ignored inventory metadata | metadata-only |
| raw phone GNSS/IMU/navigation payload reads | `0` |
| raw-base RINEX payload/header/bytes reads | `0` |
| raw payload copies/transforms | `0` |
| truth/accuracy/solution coordinate-row reads | `0` |
| MAT/PDC/precomputed-coordinate reads | `0` |
| solver invocations | `0` |
| test/solver/evaluator executions | `0` |
| Kaggle/token access | `0` |
| code/config changes authorized by this audit | `0` |
| reruns/fallbacks/repairs/sweeps | `0` |

The only files added by this audit are this Markdown record and its companion
machine-readable freeze, committed separately.  Legacy/default behavior and
all Phase126-130 artifacts remain unchanged.
