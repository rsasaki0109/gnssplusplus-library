# Smartphone R5 Phase125 next-candidate triage audit

- Execution label: `Luna Max`
- Audit date: `2026-09-04`
- Scope: read-only repository source and already sealed record metadata.
- Starting repository: `/home/sasaki/workspace/rtklib_v2_ws/gnssplusplus-library`
- Starting branch: `feat/rtk-smartphone-performance-pr`
- Starting HEAD: `11df5c17ef79279f5f36f348a1549915bbb800fd`
- Starting worktree: clean.

No code, configuration, test, raw input, raw-base payload, solver, truth
payload, MAT data, precomputed coordinate, PDC, accuracy evaluator, Kaggle,
or token resource was read or executed.  The Phase112/118/120 values below
are sealed aggregate metadata only; solution and truth coordinate rows were
not reopened.

## Decision

`NO-CANDIDATE` is frozen for this triage (`candidate_count = 0`).  The only
unclosed difference in the requested focus set that is both source-visible and
not already closed by a previous audit is the raw-base station-reference and
base-residual operator boundary.  It cannot be reduced to one default-off
variable while preserving factor topology and admission:

* the official helper takes a dataset station table and ENU offset, then asks
  `gt.Gsat`/`Gobs` to form a base residual;
* the native raw-only path takes RINEX header `APPROX POSITION XYZ`, retains
  (but does not currently consume) `ANTENNA: DELTA H/E/N`, and explicitly
  performs transmit-time iteration, Earth-rotation geometry, atmosphere,
  group-delay, smoothing, and in-domain stream interpolation; and
* changing any one of station reference, satellite-clock/group-delay order,
  geometry, atmosphere, smoothing, or interpolation can change correction
  finiteness and the exact source-miss mask, hence the retained code-factor
  population.

The required next boundary is therefore a source-complete raw-base compound
port and proof of exact factor-population preservation, not a Phase125 code
candidate.  No implementation or execution is authorized by this record.

## Authoritative state and sealed score evidence

All listed commits are ancestors of the starting HEAD.  Their worktree blobs
were hash-checked at audit time.

| sealed item | record commit | record SHA-256 | sealed aggregate (m) |
|---|---|---|---|
| Phase112 output-offset result | `0151bd070f8029413f8f2a17a28fe982dddb735f` | MD `d098f643925b4107620485c8bd0a482643e88555a2a07dfe6c4470bfd7cf114e`; JSON `46d8d836758ac1a88b95ec2c1a0c0080cedfb753c12411ed74e4c2501549c57e` | MTV-A `0.997685253035948`; LAX-T `0.6659910917640763`; macro `0.8318381724000121` |
| Phase118 TDCP robust-k result | `6b37f92077a8ede43cb24e7a5cada580feccea5c` | MD `698381199eaedc981172745bd42dc70c9c3afb8341401b1fc4c1783910aff5d6`; JSON `918597aab19f462a2aadfff606f461ce47f48f2fdcd997e5fdeb89ab459a0164` | MTV-A `1.002826677831249`; LAX-T `0.6255696228129851`; macro `0.814198150322117` |
| Phase120 TDCP atmosphere-cancellation result | `7553328e90a80d2127f95b6d0c0bab02b1b78a3f` | MD `d0007f446d89cc101dbfffa1ba6b2cbed9aced1220d05f0105778ae4b7d39acb`; JSON `608e5a1711c458240d4be7e481e93d8ba76d4d7677720f76033c9262d92a1dc2` | MTV-A `1.0080567510569665`; LAX-T `0.6260079250519545`; macro `0.8170323380544604` |

The sealed promotion comparator remains strict macro `< 0.782 m`; none of
these aggregates meets it.  This is context only, not a tuning signal.

The immediately preceding source audits and freezes are reachable and intact:

| phase | audit commit / blob SHA-256 | freeze commit / blob SHA-256 | disposition |
|---|---|---|---|
| 121 TDCP geometry | `f824879696920ab065fb3afab88f8cd8e2c93495` / `f8f9f1fa59cca62303ebea738d69d4af5c73f247fefcf5d6b6c0a8bccbb42212` | `801db5381a6ed476fc83145db4358a0c5bdf2ce4` / `aee8018adafe82249014d9b3e59e92e25699794ce15116bc0eda974f183a2424` | NO-CANDIDATE: fixed-LOS versus native endpoint geometry changes equation |
| 122 pseudorange weighting/robust | `4480b05a2686353a72b523ea0b32cf9d4142296d` / `26a05cb3a2f4b98c1f9bec9740dcd70e63c49c7f8c56c342703dca0800473906` | `310b529d325b09d83a71214bff602db363d3c745` / `1687c5980acf62d4966798f5dc457a67d9ec1dc8d16485f470ca01538c75a4e3` | NO-CANDIDATE: SNR/k exact; TGD is coupled to rover/base/SPP |
| 123 Doppler/velocity | `db4e23255529b6212f86f37409a1931d60641fcc` / `1a0c5afdbccdda64e9c238d8d1966af101a4ffdf899570d699bd5f9fefe5ffb7` | `e1642294c53176326fd9d277cd5fa6172b49857d` / `982b56655eeeb004663b5ca5b894f99a5be57be76e4ab9dd4e9ab778c9b8e948` | NO-CANDIDATE: Earth-rotation/interpolation and admission are coupled |
| 124 IMU factor | `b0a0c161ef39791321429464d2dc6ac6f5968da2` / `36d3aaf0ac752422930df4e2e10c9f3a43419346e8f26af29c55b9aa085adb8b` | `59586b424341ec0e36c52c3b28a349a473b837f2` / `607d28eaa4d66183f09f21bb65bf6e19a61e03b8c5b175fe862a19a36c7572b2` | NO-CANDIDATE: factor model, interval, and bias initialization are coupled |
| 125 motion factor | `33487ba2587e4b7fee790a1af10b85e5124440b0` / `41a5ba257c004b60747e39abaa20b90c0d96c098b1b803776d19d642b1becf40` | `11df5c17ef79279f5f36f348a1549915bbb800fd` / `b9e382c228bea02c886e2c823b60de240d3457c57f5631289e83d0252d62b98f` | NO-CANDIDATE: official X/V versus native Pose3 motion topology |

## Source evidence for the remaining raw-base boundary

The official source tree is the sealed reproducibility cache at commit
`29923f9f370f09ebc00f96d8cca375007a18e7d5`.

| source | SHA-256 | relevant behavior |
|---|---|---|
| `output/reproducibility-cache/gsdc2023/functions/correct_pseudorange.m` | `b0536ccff478b0aff253448ffb7a203c715b8064dd8dc85898e38f1f05d0441e` | `:5-6` same-satellite selection; `:8-24` reads station tables, computes mean XYZ, forms `gt.Gsat` base residual, then calls `addOffset`; `:26-34` moving mean and linear interpolation |
| `output/reproducibility-cache/MatRTKLIB/+gt/Gobs.m` | `be27fd1a075829a00f0aec5f32a0040a884077a8ac256a10a2bc26def255fc88` | `:1154-1164` code/carrier residual equations; `resPc` has satellite clock, ionosphere, and troposphere terms but no explicit TGD/BGD term |
| `output/reproducibility-cache/MatRTKLIB/+gt/Gsat.m` | `a56c323664660c1b7e771180e9bc18b08f809fe8a6d71b795366b96207871f6` | `:137-172,259-306` satellite state, geometry, atmosphere, and range-rate/Sagnac implementation |
| `output/reproducibility-cache/MatRTKLIB/+rtklib/satpos.m` | `20b7fd78065a22d9d2e02b0f394aabef2437e991ed2c63b04c65194a96827f5e` | `:20-28` states that satellite clock output excludes TGD/BGD |
| `src/algorithms/base_pseudorange_compensation.cpp` | `f3c3c859793c4bed9c7ae93825d32565f20eab0badb09412a4fab2a5930c9817` | `:186-252` two-pass transmit-time state, Earth-rotation geometry, atmosphere, group delay, and explicit residual; `:259-339` smoothing and in-domain interpolation |
| `src/algorithms/fgo_problems.cpp` | `5675e82fc595933da4e7c14ad468ae69437dc04bac79a7ef55f1edfa03e098da` | `:485-573` native rover corrected-P construction with clock, atmosphere, and group delay |
| `apps/native/gnss_fgo_imu_no_base.cpp` | `df77ab6556f90d6d9a8464d360e85d2a49a50d6fc04ef396833f354f821a1176` | `:7886-8044` applies the existing base correction/miss mask to adopted code factors exactly once |
| `src/io/rinex.cpp` | `92f683e34b4052f8d105f69fc4bf4397ca990be3b5a6b220181a56423f465533` | `:1102-1111` parses `APPROX POSITION XYZ` and `ANTENNA: DELTA H/E/N` as separate header fields |

The sealed Phase107 raw-base audit independently records the same boundary:
official `base_position.csv` plus ENU offset versus native RINEX approximate
XYZ, and opaque official `gt.Gsat` versus native explicit residual terms
(`smartphone_r5_phase107_raw_base_source_parity_audit_v1.md:82-90,131-135`).
That audit froze only guard-only admission of the existing native raw-base
path, not a station-reference or residual-model rewrite.

## Unresolved-difference inventory

| requested focus | source/native status | why it is not a fresh Phase125 one-variable candidate |
|---|---|---|
| Raw-base station reference / residual model | divergent and partly unresolved | station table/ENU offset versus RINEX header XYZ/unused antenna delta; `gt.Gsat` timing/geometry/atmosphere versus explicit native model; exact stream smoothing/interpolation is also involved. A single change cannot establish official parity or preserve miss/admission counts. |
| Satellite clock/TGD application ordering | already audited, divergent but coupled | Phase122 explicitly compared official no-explicit-TGD `Gobs`/`satpos` behavior with native group delay and rejected rover-only or rover+base changes. Re-labeling it as a new Phase125 candidate would duplicate that NO-CANDIDATE and would also change GNSS-first seed construction. |
| Clock/ISB process factors and priors | already source-fixed in the C7/D lane | Phase93/101 established official C/ISB/D staging and exact same-run handoff; current Phase101 structural result has full finite C7/D alignment and QR selection. Changing process/prior noise changes the objective or state conditioning, not one fixed measurement variable. |
| Carrier slip / TDCP gate | already audited in Phases116-121 | gate or pairing changes alter factor admission; Phase120/121 froze the only measurement/geometry candidates and did not establish a topology-preserving remaining scalar. |
| GNSS-first to main handoff priors | already audited and structurally healthy | Phase112/Phase101 evidence shows same-run position/velocity/C/D handoff and exact key alignment. Adding/changing priors changes graph information and the initial objective, and is not an output-only parity fix. |
| FGO output interpolation | closed by output contract | Phase104/112 records and the sealed runs use exact raw-UTC keys with no interpolation, edge hold, or extrapolation in the target lane. Introducing interpolation would change evaluation rows rather than repair a source mismatch. |
| Motion factor | separately closed by the immediately preceding Phase125 audit | Official X/V midpoint factor requires new state topology and dt admission; native Pose3 motion is not convertible by a scalar setting. |

## Candidate disposition

The following source observations were assessed without executing a route:

1. **`phase125-raw-base-rinex-antenna-reference-v1` — rejected.**  The RINEX
   reader exposes both approximate XYZ and antenna delta, but the native model
   consumes only approximate XYZ.  The official helper adds its ENU offset
   after obtaining `obsb.residuals(satb)`, so the source text does not prove
   that applying the native header delta to the current correction stream is
   equivalent.  It can double-count a marker/antenna reference and can change
   the residual stream and miss mask.  It therefore fails the source-backed,
   admission-invariant requirement.
2. **`phase125-raw-base-official-residual-operator-v1` — rejected as a
   compound port.**  Matching the official `resPc` requires station-reference
   provenance plus `Gsat` satellite-state timing, Earth rotation, atmosphere,
   group-delay policy, smoothing, and interpolation.  Selecting only one term
   would leave the rest unresolved and would not establish exact factor
   population.  This is the required future compound boundary, not a frozen
   one-variable candidate.
3. **`phase125-satellite-clock-tgd-order-v1` — rejected as duplicate/coupled.**
   The official/native difference is already recorded in the Phase122
   NO-CANDIDATE freeze.  Native applies the same group-delay policy to rover
   and base and uses it in the GNSS-first path; changing only one occurrence is
   asymmetric, while changing all occurrences is a measurement and seed port.

No fourth candidate is formed from TDCP, handoff priors, or output mapping,
because those dimensions are already sealed by the preceding audits above.
Thus no source-backed default-off, one-variable, topology/admission-invariant
candidate remains.  `implementation_authorized = false`.

## Required compound-port boundary

If work resumes, a new audit must first freeze a complete raw-only base
contract with all of the following jointly specified:

1. use only the raw RINEX header station reference and an explicitly proven
   antenna-reference convention; never use `base_position.csv`,
   `base_offset.csv`, phone coordinates, result coordinates, PDC, or any
   precomputed correction;
2. compare/port the official base `Gsat`/`Gobs.resPc` satellite state, clock,
   Earth-rotation, ionosphere, and troposphere semantics as one operator, or
   provide a source proof that the native operator is equivalent;
3. freeze same-satellite/same-signal stream selection, moving-mean window,
   in-domain interpolation, and exact miss-mask accounting together, with a
   proof that retained code-factor keys and counts do not change unexpectedly;
4. apply one finite correction stream exactly once to the permitted raw code
   factors in both GNSS-first and main use, while retaining Phase101 C7/D,
   CCDD, QR, IMU, all sigmas, filters, LM schedule, initialization, and
   output/truth separation.

That port is necessarily multi-variable and would need new synthetic/source
regression tests, a new structural manifest, and independent authorization.
Until then, legacy/default behavior and all prior freezes remain unchanged.

## Fixed invariants and authorization

The following are explicitly unchanged and unauthorized for this triage:

- legacy/default selector behavior;
- Phase101 epoch-local C7 mapping, metre C/ISB states, raw metre/second D,
  exact-key handoff, and no global ISB double state;
- CCDD equation and sigma, GNSS/IMU/carrier/TDCP factor topology and gates;
- Phase99 `MULTIFRONTAL_QR`, LM schedule, initialization, filters, and noise;
- raw-base exactly-once/miss-mask policy as currently implemented (no rewrite);
- output alignment, solution publication, truth evaluation, accuracy promotion,
  Kaggle, rerun, fallback, and repair.

## Read accounting

| activity | count/result |
|---|---:|
| official/native source text reads | read-only; no execution |
| sealed record metadata reads | aggregate/state metadata only |
| raw phone GNSS/IMU/navigation payload reads | `0` |
| raw-base bytes or headers read | `0` |
| truth payload or coordinate-row reads | `0` |
| solution coordinate-row reads | `0` |
| MAT/precomputed-coordinate/PDC reads | `0` |
| native solver invocations | `0` |
| accuracy calculations/evaluator invocations | `0` |
| Kaggle/token access | `0` |
| code/config/test changes | `0` |
| reruns/fallbacks/repairs/sweeps | `0` |

This is an audit record only.  It authorizes no code, raw, solver, truth,
accuracy, or submission action.
