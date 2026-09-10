# Smartphone R5 Phase129 GLONASS local-miss admission audit

- Execution label: `Luna Max`
- Audit date: `2026-09-04`
- Repository: `/home/sasaki/workspace/rtklib_v2_ws/gnssplusplus-library`
- Branch: `feat/rtk-smartphone-performance-pr`
- HEAD at audit start: `7ed294c2d6357520e795bb57b880171a63adb7c4`
- Scope: read-only audit of the official source cache, native source, and
  already sealed Phase111/126/127/128 records.

This record does not authorize implementation, raw materialization, solver
execution, truth evaluation, or accuracy evaluation.  No raw phone GNSS/IMU
or navigation payload, raw-base RINEX payload/header, solution coordinate
row, MAT file, precomputed coordinate/correction, PDC artifact, solver output,
Kaggle resource, or token was opened.  Sealed Phase128 inventory numbers are
quoted as historical metadata only; the underlying payloads are not reread.

## Decision

The official base-correction and graph-building source does **not** require
100% GLONASS FCN coverage.  It performs local common-satellite/frequency
stream selection and local factor admission.  A missing base stream or
out-of-domain interpolation produces a non-finite corrected code value, and
the official graph simply does not add that row's pseudorange factor.  The
native Phase111 source miss mask has the same exact-key, in-domain, finite
local-drop semantics.

The current Phase126/127/128 native path is stricter at an earlier boundary:
an unaccepted GLONASS provenance row causes the source-complete base model to
return false and the application to abort the route before the local miss
mask can run.  Thus the sealed Phase128 route-level full-FCN gate is a
preflight/implementation policy, not evidence of an official algorithmic
100% FCN requirement.

Exactly one source-backed, default-off candidate is frozen in the companion
JSON:

`phase129-glonass-certified-local-miss-admission-v1`

with proposed selector:

`--native-phase129-glonass-local-miss-mask`

This candidate changes only the GLONASS provenance failure boundary.  It
admits a GLO row only when the existing Phase127/128 header/geph resolver
certifies its channel at the existing FGO query time.  An uncertified GLO row
is an explicit local miss and is removed from both the base correction stream
and the shared rover pseudorange factor vector.  It is never retained raw,
assigned a zero correction, inferred from Android carrier frequency, filled
from another key, or extrapolated.  Non-GLONASS rows and certified GLO rows
continue through the existing Phase126/Phase111 source-equivalent path.

The candidate is not an accuracy claim and does not authorize a raw run.  It
is not a new factor, state, equation, sigma, solver, filter, or tuning change.

## Answer to the coverage question

### What the official source actually does

The official correction helper first intersects base observations with the
rover satellite set:

`obsb = obsb.sameSat(obsr)` (`correct_pseudorange.m:5-6`).

For each requested frequency it computes the base residual stream, smooths
it with the route's fixed window, and evaluates

`pc = interp1(obsb.time.t, pc_, obsr.time.t, "linear")`
(`correct_pseudorange.m:26-34`).

The call does not request extrapolation.  A missing common satellite/frequency
stream or a rover time outside the finite base-domain therefore has no finite
correction at that row.  The helper is called per nonempty frequency field,
not after a global all-GLO-rows certification (`fgo_gnss.m:72-78`,
`fgo_gnss_imu.m:85-92`).

The two graph builders add a `PseudorangeFactor_XC` only under
`~isnan(obsr.(f).resPc(i,j))` (`fgo_gnss.m:123-142`,
`fgo_gnss_imu.m:189-212`).  Doppler has its own `resD` guard, and TDCP has a
separate two-epoch carrier guard; neither is filled by a missing pseudorange
correction.  This is per-observation admission, with the correction stream
organized per frequency field and satellite intersection.  It is not a
route-wide GLONASS FCN coverage assertion.

The official Android conversion does not itself establish a RINEX FCN
provenance contract.  It estimates a supported carrier frequency from the
raw `CarrierFrequencyHz` and system (`gnsslog2obs.m:91-95,254-269`), computes
`lambda = c/freq` for a materialized satellite/frequency field
(`gnsslog2obs.m:158-185`), and recognizes the GLONASS frequency range in
`freq2ftype` (`gnsslog2obs.m:281-315`).  A field with no matching raw carrier
rows is not materialized (`fexist` remains false); a row with no finite
observation value remains `NaN` and is excluded by the graph guard.  The
source does not use an uncorrected base residual or zero correction as a
missing-stream fallback.  Phase129 intentionally replaces this raw-frequency
channel inference for its opt-in native path with strict header/geph
provenance; the official source is evidence for local observation admission,
not permission to infer an FCN in native code.

### What the sealed Phase111 evidence shows

The sealed Phase111 taxonomy classifies every requested MTV-A miss as a
dropped factor: BDS exact-stream miss `12,849`, GLO exact-stream miss `45`,
and GPS out-of-domain `1,128`.  It records zero raw-uncorrected and zero
zero-fallback rows.  The total conservation is `57,281 = 43,259 + 12,894 +
1,128 + 0`, with the same exact-key and finite correction predicates applied
per signal.  LAX-T has zero exact-stream misses but still exceeds its sealed
strict accuracy boundary; this is not a causal accuracy claim, but it shows
that complete exact-stream coverage is not a sufficient route-wide success
criterion.

### What current native Phase126/127/128 does

The native source miss mask is already local.  For each factor it checks the
exact `(SatelliteId, SignalType)` stream, then the epoch/time and
`correctionAt`, then finite subtraction.  Each failure increments a typed
counter and `continue`s; only a finite corrected factor is appended, and the
caller vector is swapped after per-signal/global conservation passes
(`source_pseudorange_miss_mask.cpp:53-181`).  This is the Phase111 rule.

The source-complete base model has a different earlier behavior for GLO
provenance.  After selecting the broadcast state it calls Phase127/128
resolution and returns false on any unaccepted provenance row
(`base_pseudorange_compensation.cpp:298-368`, especially `:338-352`).
State, ephemeris, geometry/Sagnac, atmosphere, residual, interval, and
duplicate/non-monotonic stream failures are likewise global source-complete
failures (`:252-293`, `:369-487`, `:490-535`).  Those global integrity rules
are not relaxed by Phase129.

The main problem builder already has a row-local `continue` when the
provenance annotation fails (`fgo_problems.cpp:515-524`), and the carrier
preparation path also skips a failed local GLO annotation
(`fgo_internal.hpp:674-710`).  But the application checks the accumulated
diagnostic failure and returns `1` for the main problem
(`gnss_fgo_imu_no_base.cpp:8362-8369`) and for GNSS-first
(`:8771-8776`).  Therefore those local continues do not yield a local route
admission under the current Phase127 selector: the later app-level guard
converts any such failure into a global abort.  The Phase129 candidate is
limited to making the GLO provenance miss an explicit local drop at this
boundary; it does not hide or downgrade any other source-complete failure.

## Behavior classification

| Missing item | Official source behavior | Current native behavior | Phase129 opt-in behavior |
| --- | --- | --- | --- |
| Common base satellite/frequency stream | `sameSat`/frequency field leaves no usable correction | exact `hasStream` miss after model build | local explicit stream miss; no factor |
| Base correction outside interpolation domain | `interp1` yields `NaN`; P factor not added | `correctionAt` false; miss mask drops factor | local explicit out-of-domain miss |
| Non-finite corrected code | `NaN` graph gate excludes row | finite subtraction check drops row | local explicit nonfinite miss |
| GLO header/geph FCN missing, invalid, tie, mismatch, or query gap | official code has no RINEX FCN admission gate; upstream raw-frequency mapping may omit a field/row | Phase127/128 resolver rejects; source-complete model/app route-aborts | local explicit GLO provenance miss, with certified rows only |
| Certified GLO wavelength non-finite/non-positive | no graph factor can be constructed for that value; no zero/raw base fallback | frequency helper/factor preparation may skip | explicit wavelength miss; never infer or synthesize |
| Satellite state/ephemeris/geometry/atmosphere unavailable in Phase126 source-complete | helper/preprocessing may abort on its own prerequisites; not a GLO FCN rule | global source-complete failure | unchanged global fail-closed |
| All rows/streams removed | no graph rows remain for that field/epoch | app/model structural gates fail | route fails closed; zero solver if no usable finite graph/base stream |

The phrase “local” therefore applies to an observation/stream whose value is
unavailable, not to a global source-integrity failure.  Phase129 must preserve
that distinction in both code and telemetry.

## Exact Phase129 candidate contract

### Selector composition and scope

The proposed selector is default off and may be enabled only together with
the complete Phase126/127/128 recipe:

* `--native-phase126-raw-base-source-complete`
* `--native-phase127-glonass-channel-provenance`
* `--native-phase128-glonass-provenance-parser-admission`
* `--native-phase129-glonass-local-miss-mask`

The Phase129 selector is not a standalone parser, a partial Phase126 mode, or
a replacement for canonical FCN parsing.  Phase128's canonical fixed-field
and typed-key checks remain required.  The opt-in changes only the
route-level “all GLO rows must be certified” admission to an explicit
per-row/per-stream certified-or-missed ledger.  Selector-off behavior,
including the historical Phase128 full-coverage preflight, remains unchanged.

### Per-row provenance predicate

For every GLO observation that could enter either the rover graph or the
base correction stream, use the existing Phase127/128 resolver at the
existing FGO transmission/query time.  The row is **certified** only if all
of the following hold:

1. The typed `(GNSSSystem::GLONASS, PRN)` key is canonical and the row's
   satellite/signal identity is exact.
2. Header FCN, when present, is valid in `[-7,6]`; a selected broadcast
   `geph.frq` at the exact query time is also present, valid, and equal to the
   header value.  With no header FCN, only that selected time-valid broadcast
   FCN may supply the value.
3. The existing validity predicate and tie/conflict policy pass; no missing,
   stale/out-of-window, nonintegral/nonfinite, different-FCN tie, malformed
   required record, or header/geph mismatch is hidden.
4. The existing signal helper derives a finite positive wavelength from this
   certified channel.  This is an admission check, not a new frequency
   equation.

If any predicate fails, the row is recorded as an explicit miss and omitted
from the corresponding GLO factor/stream.  No carrier-frequency inference,
fixed channel, zero channel, external table, cross-satellite or cross-signal
fill, nearest channel, endpoint hold, extrapolation, or raw uncorrected
factor is permitted.  Certified non-GLONASS and certified GLO rows use the
existing equation and correction path unchanged.

### Shared correction and factor semantics

The base model must build a candidate stream from only finite certified rows.
An unaccepted GLO base row contributes no sample to its exact
`(SatelliteId, SignalType)` stream.  If the rover has a GLO code row but the
base stream is absent or its correction is outside domain/non-finite, the
existing Phase111 miss mask drops the rover row.  If a GLO row is certified on
both sides and its correction is finite/in-domain, it is corrected exactly
once and retained.  A base stream for a satellite/signal with no matching
rover factor may remain unused; it must not create a factor or state.

The Phase126 A/B/C transaction remains all-or-nothing for source-complete
integrity: raw/reference ingress (A), correction stream construction and
conservation (B), and one shared-vector application (C) must all succeed.
The only newly local outcome is a GLO provenance row classified as a miss.  A
state/geometry/atmosphere/interval/stream transaction failure still discards
candidate streams and fails the route closed.

### Minimum-data and empty-route gates

The candidate must fail closed, without solver launch, if any of these holds:

* no finite retained rover epoch or no usable raw GNSS/IMU/navigation epoch
  remains after the existing input checks;
* no finite positive certified base residual stream remains after local GLO
  misses and the existing non-GLONASS processing;
* no finite corrected rover pseudorange factor remains after exact-key,
  in-domain, finite correction admission;
* every factor for a required graph input is removed, or the graph's existing
  C7/D/CCDD/position/velocity alignment gates cannot be satisfied;
* any source-complete A/B/C transaction or finite/monotonic/route-interval
  invariant fails; or
* any fallback, duplicate correction, unclassified row, conservation mismatch,
  or partially applied candidate state is observed.

It is valid for certified GLO coverage to be zero **only** when the remaining
non-GLONASS population still satisfies the existing finite stream, factor,
epoch, and graph gates.  This is not a permission to silently keep a raw GLO
row.  A route with only uncertified GLO code rows, or a route whose local
misses leave no finite graph/base factor, is a fail-closed NO-GO.

### Required admission accounting

The compact structural ledger must be exact for each side (`rover`, `base`),
stage (`GNSS-first`, `main`), constellation/signal, and route:

* original eligible GLO rows;
* certified rows by header-primary versus broadcast-geph fallback;
* explicit provenance misses by reason: missing FCN, invalid/range,
  query-time gap, header conflict/malformed, geph missing/invalid,
  different-FCN tie, header/geph mismatch, canonical record failure, and
  non-finite wavelength;
* exact-key stream matched/missing rows; in-domain/out-of-domain/nonfinite
  correction counts; retained corrected rows;
* correction model build count, application pass count, exactly-once marker,
  duplicate marker, raw/zero fallback count (required zero), and per-signal
  conservation;
* total factor conservation, finite-positive wavelength count, and the
  existing C7/D/CCDD/accepted-iteration/cost/output gates.

The conservation identity is recorded independently for each side/signal
and globally:

`original = retained_corrected + explicit_provenance_miss + missing_stream + out_of_domain + nonfinite`

Rows rejected before the correction-mask population (invalid raw row,
unavailable state, or global source failure) must have their own typed bucket;
they may not disappear from the conservation denominator.  `factor_count` and
`stream_count` are kept separate.  No solution coordinate or accuracy value
is published by this structural ledger.

## Sealed evidence and source pins

The following hashes were checked in this audit.  Source files are read as
text only; sealed records are read as aggregate/provenance metadata only.

| Evidence | SHA-256 | Relevant lines/claim |
| --- | --- | --- |
| `output/reproducibility-cache/gsdc2023/fgo_gnss.m` | `5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3` | `72-78,123-149`: correction then finite `resPc` P-factor gate |
| `output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m` | `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3` | `85-92,189-221`: same code/Doppler gate in IMU graph |
| `output/reproducibility-cache/gsdc2023/functions/correct_pseudorange.m` | `b0536ccff478b0aff253448ffb7a203c715b8064dd8dc85898e38f1f05d0441e` | `5-6,26-34`: `sameSat`, moving mean, in-domain interpolation |
| `output/reproducibility-cache/gsdc2023/functions/gnsslog2obs.m` | `665ce43d1fc3c5a9c3b3a6fd76f81539126e3ab897fbba484ab0ccd18964acff` | `91-95,158-195,254-315`: raw carrier-frequency mapping and NaN materialization |
| `src/algorithms/source_pseudorange_miss_mask.cpp` | `291ff2591dbd62ecba92c24d1d74c686274568f2af6182bc4dc52f237b0b9946` | `53-181`: exact-key/local miss, finite correction, conservation |
| `src/algorithms/base_pseudorange_compensation.cpp` | `03459df5505a5e32e4ee356126a30742c693f32fe0d6eccc3bfc243c7fe4cca4` | `244-368,369-487,490-535,538-576`: source-complete global failures, GLO provenance abort, streams/interpolation |
| `src/algorithms/fgo_problems.cpp` | `f3f86db94c8cf3f43e25d402e21b5d3f95d9259435e559fbe76e7564af6c3d55` | `489-529`: existing query-time and row-local provenance continue |
| `src/algorithms/fgo_internal.hpp` | `f3980a5190ef1cc59aa479665c7ccbe6fe4ec0096e0aef1aa3eb451b53241acb` | `674-710,785-795`: carrier admission and local skip |
| `src/algorithms/phase127_glonass_channel_provenance.cpp` | `cc8e3105abc8367786b089e8d0492f8217550f0a69473869ae8a89cc01941bb0` | `69-246`: exact-time header/geph FCN resolve/fail reasons |
| `src/algorithms/phase128_glonass_provenance.cpp` | `5d930d8be16c70c4956de41822160c0c44c1e2761f893a9a491cb9d1a076bbc7` | `37-82`: canonical record gate around Phase127 |
| `apps/native/gnss_fgo_imu_no_base.cpp` | `898b979c51d8fa4ec98432a3f2aeb8a99ff2603689e8cbe396aecad501985138` | `8362-8369,8371-8451,8771-8776`: app-global provenance guards and shared miss-mask application |
| sealed Phase111 audit | `d22eae620504768c555879620592aee65ae450e242befa3e8ce5b918d8fc25ae` | exact miss/drop/no-fallback taxonomy and conservation |
| sealed Phase111 freeze | `176cc4411cb53262b638feffb0ff41cb231139644d7ce618cf94e7c6150d241e` | source-backed local retention contract |
| sealed Phase126 contract audit | `8ad6fcedc7a3b23c8f13b82968f67bada2fbe039d90f919c104e9a910f71f4ee` | A/B/C transaction and source-complete invariants |
| sealed Phase128 result | `071fd33634ccdd3060d286f166d0b0d79f72b8dfbcbe87c31a03e372734cc39b` | historical route-level full-FCN preflight and solver-zero NO-GO |
| sealed Phase128 contract freeze | `467b35e14ed3235244a6403eb5ac1fbcad777f9e675fd1a08db20209781ac385` | Phase128 full-coverage preflight is a contract gate, not official proof |

The Phase128 result commit is `7ed294c2d6357520e795bb57b880171a63adb7c4`.
Its historical route metadata reports incomplete rover/base FCN coverage and
zero native solver invocations.  The candidate does not reinterpret those
numbers as an official requirement or as an accuracy result.

## Implementation, test, and reauthorization boundary

Implementation is not authorized by this audit.  If the parent approves the
companion freeze, the smallest implementation is:

1. Add one default-off config/CLI selector and a typed Phase129 report.  The
   selector must require the complete Phase126/127/128 recipe and must leave
   selector-off behavior byte/semantically unchanged.
2. At the existing base-model and shared-problem admission boundary, turn
   only a failed GLO provenance row into a staged local miss.  Build the
   candidate stream/factor vectors transactionally, and keep all non-GLO and
   certified-GLO rows on the existing operators.  Preserve global failure for
   source state, ephemeris, geometry, atmosphere, interval, duplicate stream,
   nonfinite model, A/B/C, and graph integrity violations.
3. Carry the explicit miss ledger into both GNSS-first and main structural
   telemetry.  Require zero fallback/repair and exact conservation before
   exposing the shared vector.

Focused synthetic tests must cover mixed GPS/GAL plus certified/missing GLO,
missing/invalid/range FCN, query-time gap, header/geph mismatch, same-FCN
duplicate and different-FCN tie, nonfinite wavelength, absent exact base
stream, out-of-domain correction, all-row removal, and no-stream/empty-route
fail-closed behavior.  They must assert that non-GLONASS and certified GLO
rows are retained, uncertified GLO rows are not uncorrected, and the
per-signal/global conservation equations hold.  Selector-off regression must
remain unchanged.  No raw payload or real solver is needed for these tests.

Before any future structural run, a new implementation pin, launch-free
manifest/validator, pre-raw read-accounting seal, target-binary hash, and
independent authorization are mandatory.  Authorization-time inventory must
verify per-row certified/missed GLO provenance and all Phase126 A/B/C gates;
there is no rerun, fallback, repair, or truth evaluation in that lane.

## Read accounting and authorization status

| Resource/action in this audit | Count/status |
| --- | ---: |
| Official/native source text reads | nonzero, source-only |
| Sealed Phase111/126/127/128 record metadata reads | nonzero, aggregate/lineage-only |
| Raw phone GNSS/IMU/navigation payload reads | 0 |
| Raw-base RINEX payload/header/stat/hash reads | 0 |
| Solution coordinate rows opened/interpreted | 0 |
| Truth payload reads/evaluator invocations | 0 |
| MAT/precomputed coordinate/PDC reads | 0 |
| Native solver invocations | 0 |
| Accuracy calculations/Kaggle/token access | 0 |
| Raw reruns/fallbacks/repairs/publication | 0 |

Conclusion: the official source proves local corrected-code admission, so
this audit freezes one default-off Phase129 candidate rather than
`NO-CANDIDATE`.  It does not authorize code changes, raw reads, solver
launches, truth access, accuracy promotion, or Kaggle submission.
