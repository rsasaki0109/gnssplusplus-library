# Smartphone R5 Phase130 shared-ledger join-semantics audit

- Execution label: `Luna Max`
- Audit date: `2026-09-04`
- Repository: `/home/sasaki/workspace/rtklib_v2_ws/gnssplusplus-library`
- Branch: `feat/rtk-smartphone-performance-pr`
- HEAD at audit start: `4855e2a5bebb417b51fcd8ef958c498f38bbbb14`
- Worktree at audit start: clean.
- Scope: read-only audit of the official source cache, native source, and the
  sealed Phase129 structural result/metadata.

This record does not authorize implementation, raw materialization, solver
execution, truth evaluation, accuracy evaluation, or solution release.  No
raw phone GNSS/IMU or navigation payload, raw-base RINEX payload/header,
solution coordinate row, MAT file, precomputed coordinate/correction, PDC
artifact, solver output, Kaggle resource, or token was opened in this audit.
The Phase129 result was read as sealed structural metadata only.

## Decision

The official and native correction paths require a **local keyed support
join**, not equality of the rover and base ledgers as whole vectors.  The
correct invariant is:

1. each side independently partitions its own eligible rows into certified and
   explicit local-miss rows;
2. each retained rover code factor has exactly one certified finite base
   correction for its exact `(constellation, satellite, signal, query-time)`
   key and interpolation domain; and
3. each side's row/factor/correction counters conserve its own input.

There is no source requirement that rover and base have equal eligible-row
counts, equal certified counts, equal miss counts, equal miss-reason maps,
equal epoch counts, or equal row multiplicity.  A finite base stream may have
unused rows.  An unmatched rover row is an explicit factor miss; it must not
be retained raw or filled with zero.  The existing Phase126 source-complete
station/state/geometry/atmosphere/interval/duplicate integrity failures remain
global fail-closed conditions.

Exactly one source-backed default-off contract candidate is frozen in the
companion JSON:

`phase130-shared-ledger-key-local-support-v1`

with proposed selector:

`--native-phase130-shared-ledger-key-local-support`

This is a contract/admission-boundary candidate, not a factor or equation
change.  It replaces only the Phase129 pre-solver whole-ledger equality gate
with keyed local-support validation and explicit unused-base-row accounting.
It requires a later implementation, focused tests, a new launch-free
manifest, and independent raw authorization.  No such implementation or raw
run is authorized by this audit.

## Source and sealed-state evidence

The source blobs were hash-checked at audit time:

| source or sealed record | SHA-256 | relevant lines/role |
|---|---|---|
| `output/reproducibility-cache/gsdc2023/functions/correct_pseudorange.m` | `b0536ccff478b0aff253448ffb7a203c715b8064dd8dc85898e38f1f05d0441e` | `5-6`, `26-34`: common-satellite selection, smoothing, linear interpolation |
| `output/reproducibility-cache/gsdc2023/fgo_gnss.m` | `5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3` | `72-78`, `123-148`: per-frequency correction and finite per-observation factor gates |
| `output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m` | `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3` | `85-92`, `202-218`: same correction/admission in the IMU graph |
| `src/algorithms/base_pseudorange_compensation.cpp` | `b6c6a9b6a2f29c66c4f2abecbe8d7e56c4d7cf81e9c145273c3ea1255da6eb9d` | `144-145`, `499`, `536-581`, `584-617`: keyed stream construction and in-domain lookup |
| `include/libgnss++/algorithms/base_pseudorange_compensation.hpp` | `f6e5e9c1436138e9b0ef943d8b8fa68f3deca51bca650bab71526703d3ad0272` | `27`, `140-158`, `168`: exact key and lookup contract |
| `src/algorithms/source_pseudorange_miss_mask.cpp` | `291ff2591dbd62ecba92c24d1d74c686274568f2af6182bc4dc52f237b0b9946` | `53-181`: local exact-key/domain/finite filtering and conservation |
| `apps/native/gnss_fgo_imu_no_base.cpp` | `3ad1c235721a2ee0b550a01b2446e300d96fa87fd78bfadc6ab7600beb501547` | `8495-8598`: one shared factor-vector mask/application transaction |
| `src/algorithms/fgo_problems.cpp` | `b01c9c48f9bdfd470be8aba3bab5247923be4d0ec52d5bc1a3c26f78be139a9c` | `503-540`: exact query/key annotation and row-local skip before shared mask |
| Phase129 structural result JSON | `fb703473f05bd647fd30b9abaa230a4e6f9930f1798616f463061a74359aa83c` | sealed route inventory and failure metadata |
| Phase129 structural result MD | `62504623a8ad96b0d0f187ba0c7ab375938e0de683a013b43239d61bbf9d650f` | sealed no-go summary |
| Phase129 structural runner | `30e2c5975020c194ec68e2af1ff233c5c999960e543767e1acaeb5946d344c47` | `591-601`: whole-ledger equality gate |

The current source hashes intentionally differ from older audit records where
the files were later extended with Phase129 telemetry.  The official MATLAB
files and current native source are the evidence for this audit; no generated
payload is used.

## Official control flow

The correction helper starts with:

`obsb = obsb.sameSat(obsr)` (`correct_pseudorange.m:5-6`).

The operation selects the base observations common to the rover satellite
population.  The subsequent correction is executed once per nonempty
frequency field by both graph entry points (`fgo_gnss.m:72-78` and
`fgo_gnss_imu.m:85-92`).  Thus the effective key is the common satellite plus
the selected frequency field; it is not a row-index or whole-vector key.

For that field, the helper computes a base residual stream, applies the fixed
route moving mean, and evaluates:

`pc = interp1(obsb.time.t, pc_, obsr.time.t, "linear")`

(`correct_pseudorange.m:26-34`).  MATLAB receives no extrapolation argument.
At a finite exact endpoint the interpolation is supported; an interior rover
time requires finite samples on the surrounding stream interval; a rover time
outside the finite base domain has no correction.  A missing common satellite
or frequency field likewise has no finite value for that rover row.

The graph builders then add a pseudorange factor only when the corrected value
is finite (`~isnan(obsr.(f).resPc(i,j))`, `fgo_gnss.m:138-142`,
`fgo_gnss_imu.m:208-212`).  Doppler and TDCP have independent guards.  No
official statement or control-flow branch compares the number of rover rows
to the number of base rows, or compares the two sides' miss-reason vectors.

This is a local factor-admission rule: one rover observation can consume one
interpolated base value, while many rover observations can query one base
stream and many base samples can remain unused.  The source does not define a
one-to-one base-row pairing.

## Native control flow

The native base model builds a map keyed by
`std::pair<SatelliteId, SignalType>` (`base_pseudorange_compensation.cpp:144`,
`499`).  Each key contains a time-ordered finite residual stream.  Its
`correctionAt` first rejects a missing key or non-finite query time, then
rejects a query before the first or after the last sample, and otherwise uses
an exact endpoint or the adjacent finite interval
(`base_pseudorange_compensation.cpp:584-617`).  This is the native form of the
official frequency-field/satellite/time join.

The model applies the existing fixed source-complete residual and smoothing
operator before the join.  The Phase126 source-complete equation remains:

`pc_K(t_b) = P_base(t_b) + c_sat,m(t_b) - rho_b(t_b) - I_b(t_b) - T_b(t_b)`

with the frozen official no-explicit-TGD/BGD policy.  After smoothing,

`pc_K(t_r) = LinInterpInDomain({(t_b, pc_K(t_b))}, t_r)`

and the rover factor measurement is:

`P_rover,corrected(t_r) = P_rover,raw(t_r) - pc_K(t_r)`.

The correction is in metres.  No source-backed change to this operator is
part of Phase130.

The miss-mask callback is local and ordered (`source_pseudorange_miss_mask.cpp:84-131`):

1. `hasStream(satellite, signal)` false means an exact-stream miss and the
   factor is dropped;
2. invalid epoch/time or a false `correctionAt` means an out-of-domain miss;
3. non-finite correction or subtraction means a non-finite miss; and
4. only a finite corrected factor is appended.

The vector is swapped only after global and per-signal conservation checks
(`source_pseudorange_miss_mask.cpp:134-181`).  The application invokes this
helper on `problem.pseudorange_factors` and passes that same post-mask vector
to the two stage graph paths (`gnss_fgo_imu_no_base.cpp:8495-8598`).  There is
therefore no native requirement that all finite base stream samples be used,
or that the base and rover input row counts match.

## Phase129 failure is a contract overreach

The Phase129 runner's `verify_inventory_record` requires all of the following
at `gnss_smartphone_phase129_glonass_local_miss_structural.py:591-601`:

* `base_factor_ledger_equal == true`;
* `exact_key_decision_equal == true`;
* `same_local_miss_reasons == true`; and
* the complete rover tuple equals the complete base tuple.

That is a stronger condition than the official/native join.  The sealed
Phase129 result shows why it failed before the solver:

| route | rover GLO certified/miss | base GLO certified/miss | route failure |
|---|---:|---:|---|
| MTV-A | `6858/5210` | `9642/7557` | `rover/base shared GLONASS ledger differs` |
| LAX-T | `4058/4815` | `396/524` | `rover/base shared GLONASS ledger differs` |

Each side independently conserves its own input, but the side totals and
reason maps differ.  Solver invocations were zero for both routes.  This is
consistent with the runner equality predicate and does not demonstrate that
the official correction operator needs equal ledgers.  It also does not
justify reopening any raw payload or repeating the sealed run.

## Join-semantics classification

| case | official source semantics | current native semantics | Phase130 disposition |
|---|---|---|---|
| Different rover/base cadence | Base `obsb.time.t` is sampled at rover `obsr.time.t` by `interp1`; equal cadence is not stated or required. | Key stream is queried at each factor epoch; the frozen route interval/window contract remains 1 s/151 or 15 s/11. | Permit cadence difference at the join; preserve route interval/window and fail globally if the frozen source-complete interval contract itself is invalid. |
| Different row multiplicity | No one-to-one pairing; one stream supports many rover queries and base samples can be unused. | `std::map<key, vector<Sample>>` retains all finite samples; `correctionAt` queries by time. | Permit unequal counts and record `base_unused_rows`; never require tuple equality. |
| Certified GLO row | Phase129 provenance is not in official MATLAB, but certified native rows continue through the existing correction stream. | Certified GLO rows are inserted under the exact key and use the existing wavelength/residual path. | Retain unchanged. |
| GLO local miss | A missing/non-finite field yields no finite corrected code and the official P factor is not added. | Phase129 records the explicit miss; no sample/factor is admitted for that row. | Keep as local miss; no cross-side equality. |
| Non-GLO | Same common frequency-field and finite corrected-code gate; no GLONASS FCN rule. | Existing exact-key/domain/finite mask applies without Phase129 GLO provenance. | Retain unchanged; no non-GLO count comparison. |
| Exact endpoint | `interp1` is defined on the finite endpoint of the source domain. | `correctionAt` returns the first/last finite sample at equality. | Accept exact endpoint support. |
| Interior interpolation | Two adjacent finite samples in the same key stream are required. | `lower_bound` selects the adjacent left/right samples and linearly interpolates. | Accept only finite in-domain brackets. |
| Out of domain | No extrapolation argument; corrected value is non-finite/NaN and factor guard omits it. | `correctionAt` returns false before/after stream domain. | Explicit factor miss; no endpoint hold/extrapolation. |
| Missing exact key | `sameSat`/frequency field leaves no usable correction for that row. | `hasStream` returns false and the mask drops the factor. | Explicit `missing_exact_stream` miss. |
| Base duplicate/non-monotonic time | The cached helper does not define a safe duplicate policy for `interp1`. | Source-complete native build rejects non-increasing samples globally (`base_pseudorange_compensation.cpp:536-551`). | Keep global fail-closed; do not deduplicate or choose a winner. |
| GLONASS header/geph duplicate/conflict/tie | Not an official RINEX provenance rule. | Phase127/128 resolver classifies these and Phase129 makes only eligible GLO provenance failures local. | Keep existing typed reason and no fixed/inferred channel; do not use join relaxation to hide it. |
| Unused base stream/sample | No factor is created merely because a base row exists. | A base stream can exist without a rover factor; `hasStream` is only queried by rover factors. | Permit and count as unused; it is not a failure. |
| Empty/all-miss population | No finite corrected code means no pseudorange factor for that field. | Existing model/factor/graph gates fail closed when no usable finite population remains. | Preserve zero-solver route failure. |

The distinction is important: local conservation is per side and per source
key; factor conservation is per rover factor vector.  There is no cross-side
row conservation equation because the base and rover are different time
series.

## Formal Phase130 contract candidate

For a side (s\in\{R,B\}), let (K=(g, p, f)) be constellation, satellite
PRN, and signal/frequency.  Let (E_s(K)) be eligible rows and let (C_s(K)
and (M_s(K)) be certified and explicit-miss rows.  The required local
partition is:

`|E_s(K)| = |C_s(K)| + |M_s(K)|`.

For each certified base key (K), let (S_B(K)) be the finite, strictly
time-ordered smoothed residual samples.  A rover row (r=(K,t_r)) has base
support iff either:

* (t_r) equals one finite endpoint/sample in (S_B(K)); or
* there are two adjacent finite samples (t_l < t_r < t_u) in (S_B(K)).

The interpolated correction is:

`pc(K,t_r) = pc_l + (t_r-t_l)/(t_u-t_l) * (pc_u-pc_l)`.

No support exists for a missing key, non-finite sample/query, non-positive
bracket width, or (t_r\notin[t_first,t_last]).  A retained rover code
factor must satisfy:

`retained(r) => certified_R(r) AND support_B(K,t_r) AND finite(P_rover_raw - pc(K,t_r))`.

Otherwise it is an explicit local miss with one typed reason.  The rover-side
factor conservation is:

`N_adopted = N_retained + N_missing_key + N_out_of_domain + N_nonfinite`.

Base-side finite stream samples need not equal any rover count.  The contract
must report, but not reject solely on, `N_base_unused_samples` and
`N_base_stream_keys_without_rover_query`.  A retained corrected factor must
have exactly one correction-application marker; a second application,
uncorrected retention, zero fallback, cross-key fill, nearest fill, endpoint
hold, or extrapolation is a hard failure.

The Phase126 source-complete A/B/C transaction remains global and
all-or-nothing: station/reference, satellite state/ephemeris, geometry/Sagnac,
atmosphere, residual finiteness, route interval, strictly increasing source
stream, and existing graph gates are unchanged.  If those fail, or if all
usable factors/streams disappear, the route fails before solver launch.

## Candidate scope and non-goals

The sole candidate is a default-off admission/contract change:

* replace only `rover_base_glonass_ledger` whole-tuple equality with local
  keyed join predicates and side-local conservation;
* require every retained rover factor to have one exact-key finite support
  result;
* explicitly account for unused certified base rows/streams;
* preserve certified-or-explicit-miss GLO provenance and all non-GLO rows;
* preserve Phase126 source-complete A/B/C, Phase129 reason taxonomy, raw-base
  correction equation, moving mean, interpolation, C7/D/C0D, TDCP, IMU, QR,
  sigma, filter, LM, initialization, output, and exactly-once behavior; and
* add no factor, state, measurement, correction, fallback, or tuning.

The future implementation boundary is limited to the Phase130 selector,
runner/validator/manifest schema, compact local-join telemetry, and focused
synthetic tests.  If preserving the existing native code requires changing a
factor/equation or synthesizing a missing correction, implementation must
fail closed and this candidate must not be expanded.

## Reauthorization and tests required before any run

No raw execution is authorized here.  A future implementation would require
all of the following in separate, hash-pinned commits:

1. implementation and focused tests for side-local partition, key-local
   support, unequal cadence/multiplicity, unused base rows, endpoint and
   interior brackets, missing key, out-of-domain, nonfinite, GLO certified /
   local-miss, non-GLO retention, duplicate/tie rejection, and exactly-once
   correction;
2. a launch-free manifest/validator that rejects any raw/truth/MAT/PDC/
   precomputed-coordinate/Kaggle access and proves solver invocation zero
   before inventory pass;
3. an independent one-shot authorization pinning implementation, runner,
   manifest, validator, target binary, route order (MTV-A then LAX-T), and
   exact route count; and
4. one sealed structural result per route, with opaque solution hash/row
   metadata only.  A failed route is sealed NO-GO with no rerun, fallback,
   repair, or truth evaluation.

Structural gates must include side-local and factor conservation, exact-key
support coverage for every retained factor, base-unused accounting,
Phase126 A/B/C transaction, C7/D/CCDD alignment, finite progress and strict
cost decrease where solver is reached, expected output coverage, and all
existing no-fallback/no-extrapolation/no-raw-or-zero predicates.  Truth,
accuracy, MAT, and Kaggle remain outside this contract.

## Candidate disposition

`candidate_count = 1` and `implementation_authorized = false` for this
read-only audit.  The source-backed candidate is the Phase130 keyed local
support gate above.  It is selected because it corrects the demonstrated
Phase129 pre-solver overconstraint without changing the native factor model.
No accuracy conclusion is drawn from the Phase129 no-go result, and no score
is used to select or tune this candidate.

## Read accounting

| activity | this audit |
|---|---:|
| official/native source text reads | read-only |
| sealed Phase129 result/metadata reads | aggregate/structural metadata only |
| raw phone GNSS/IMU/navigation payload reads | `0` |
| raw-base RINEX payload/header reads | `0` |
| raw-base bytes/hash reads | `0` |
| truth payload or coordinate-row reads | `0` |
| solution coordinate-row reads | `0` |
| MAT/precomputed-coordinate/PDC reads | `0` |
| native solver invocations | `0` |
| accuracy/evaluator calculations | `0` |
| Kaggle/token access | `0` |
| code/config/test changes | `0` |
| reruns/fallbacks/repairs/sweeps | `0` |

This audit authorizes no implementation or execution.  Legacy/default behavior
and all prior Phase126-129 freezes remain unchanged until a separately
authorized implementation and structural run.
