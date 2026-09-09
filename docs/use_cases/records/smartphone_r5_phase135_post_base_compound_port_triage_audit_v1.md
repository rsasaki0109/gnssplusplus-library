# Smartphone R5 Phase135 post-base compound-port triage audit

- Execution label: `Luna Max`
- Audit date: `2026-09-04`
- Repository: `/home/sasaki/workspace/rtklib_v2_ws/gnssplusplus-library`
- Branch: `feat/rtk-smartphone-performance-pr`
- HEAD at audit start: `156e650c414b01493c3bc7355448e1bd760b0ebf`
- Worktree at audit start: clean.
- Scope: read-only comparison of the pinned official GSDC2023 source, native
  Phase118/126-134 source and records, sealed Phase118/120/134 aggregates,
  and the current native graph construction.

This audit did not open a phone GNSS/IMU payload, broadcast-navigation
payload, raw-base payload/header, truth payload, solution coordinate row,
MAT/PDC/precomputed-coordinate artifact, or Kaggle/token resource.  It did
not launch a solver, run a synthetic solver, calculate a score, materialize
raw input, or rerun a route.  The sealed numeric aggregates below are used
as historical evidence only; they are not tuning targets.

## Decision

Exactly one compound candidate is selected in the companion freeze JSON:

`phase135-official-affine-measurement-family-sagnac-key-order-transactional-v1`

The proposed opt-in selector is
`--native-phase135-official-affine-measurement-family`.  It is a design
boundary only.  No implementation or execution is authorized by this audit.

The candidate ports the official P/D/ordinary-TDCP measurement-factor
family, its fixed-initial-geometry/key ordering, and its single Sagnac
convention together.  The subcomponents are inseparable: a route is admitted
only if all supported P, D, and TDCP rows have a common source-equivalent
geometry/key representation and all factor-family counts, state dimensions,
and retained-row ledgers agree.  A partial P-only, D-only, or TDCP-only port,
native fallback, retry, or hybrid factor set is forbidden.

This is the first remaining candidate that is both close to the end-state
graph and source-complete without importing MAT or precomputed coordinates.
The official source explicitly supplies the factor equations and key order,
while the current native main graph still evaluates nonlinear endpoint
geometry for P and TDCP and uses a different prepared-state/Sagnac
convention.  Phase123 already established that changing Doppler geometry
alone is not safe; this candidate treats the coupled measurement family as
one transactional unit.  Existing Phase126 raw-base correction and Phase118
measurement normalization remain fixed prerequisites, not subcomponents to
be retuned here.

## Authoritative state and sealed outcome

The current authoritative state is the clean Phase134 accuracy seal:

| item | value |
|---|---|
| HEAD | `156e650c414b01493c3bc7355448e1bd760b0ebf` |
| Phase134 structural result | `f658d52ce666882c1516d9ede8d396885d8adbd6` |
| Phase134 structural result JSON SHA-256 | `426adb4d951d226a4e99e36813e0873fd0b10ec47a6b62a10f2df4d67327f0fa` |
| Phase134 accuracy result commit | `156e650c414b01493c3bc7355448e1bd760b0ebf` |
| Phase134 accuracy result status | `no-go-phase134-truth-only-accuracy` |
| Phase134 MTV-A score | `1.0130009613234514 m` |
| Phase134 LAX-T score | `0.6188718940067218 m` |
| Phase134 macro | `0.8159364276650867 m` |
| Phase118 champion macro | `0.814198150322117 m` |
| Phase120 macro | `0.8170323380544604 m` |
| strict promotion gate | `macro < 0.782 m` |

Phase134 is structurally GO for both routes (native return `0`, 12 main
iterations, finite progress and raw-base/Phase131 telemetry), but its
truth-only macro is NO-GO and MTV-A regresses the Phase112 baseline.  The
summary bridge was telemetry-only; it did not change graph equations or
factor values.  This makes a source-parity graph audit appropriate, but does
not authorize changing the graph based on the score.

The inherited source and contract chain is preserved by these full commits:

| phase | audit/freeze or implementation evidence | disposition |
|---|---|---|
| 121 | `f824879696920ab065fb3afab88f8cd8e2c93495` / `801db5381a6ed476fc83145db4358a0c5bdf2ce4` | fixed-LOS TDCP alone rejected; geometry/key/Sagnac are coupled |
| 122 | `4480b05a2686353a72b523ea0b32cf9d4142296d` / `310b529d325b09d83a71214bff602db363d3c745` | no independent P weighting/robust candidate |
| 123 | `db4e23255529b6212f86f37409a1931d60641fcc` / `e1642294c53176326fd9d277cd5fa6172b49857d` | D units/sigma/Huber match; geometry/Sagnac is coupled |
| 124 | `b0a0c161ef39791321429464d2dc6ac6f5968da2` / `59586b424341ec0e36c52c3b28a349a473b837f2` | official ImuFactor topology differs from native CombinedImuFactor |
| 125 | `4ddfb4b757a44a9ef6864cb964ed5b65eb1f33a6` / `7d292931e5c70bede47701cf9c7a19a0d04b7e54` | official motion topology differs; no candidate |
| 126 | `88f9af2c585647d261506aa118fc322438281cf7` / `583a6c7a4788f82373a4e71436d2953703bc764c` / `9e9972667ee009e1bcd0dd1ea4732ae45415c3ce` | raw-base source-complete compound boundary implemented |
| 127–131 | provenance, parser, local-miss, shared-ledger, and canonical-band chain ending at `c3af051e` | raw-base admission and key provenance are fixed |
| 132–133 | selector/preflight boundary ending at `f32e0132` and `8b1f75be` | native selector ownership and typed preflight are fixed |
| 134 | `28301f56afb3a418b89252aa2106d2ffb329f61d` / `41a9fa8dd52878cd7992313f151014dc9cca0fd8` / `130a7f8f12e191cc12ebd0ff66ad591d732775f7` | native Phase131 counter bridge only; structural run remained algorithmically unchanged |

The abbreviated Phase127–131 rows refer to the already sealed source and
contract records; no historical artifact is modified or reinterpreted here.

## Candidate comparison

The four requested candidates were compared against the fixed Phase118
recipe and the completed Phase126–134 boundaries.  “Impact” describes the
number of graph terms affected, not a claim about score improvement.

| candidate | source completeness | native feasibility / reuse | MAT-free and leakage risk | impact | disposition |
|---|---|---|---|---|---|
| **A. P/D/TDCP fixed-initial-LOS affine family + official Sagnac/key order** | **Strong.** Official scripts and factor headers specify P `[X,C]`, D `[V,D]`, Pixel5 TDCP `[X1,X2,C1,C2]`, residuals, Jacobians, and `Gsat` geometry/Sagnac. | **Implementable but compound.** Reuse C7 mapping, C/D handoff, raw nav state, existing Phase126 corrected streams, and `Pose3Point3Factor_PX`; add a candidate-local affine geometry/factor adapter and, in the IMU main graph, the official auxiliary `x`/PX binding. | **Raw-only.** Same-run phone raw/nav/base and in-memory seeds suffice. No result/MAT coordinate enters. Leakage is controlled by keeping Phase126 measurement preparation and Phase118 gates fixed and rejecting unsupported rows. | P, D, and ordinary TDCP across GNSS-first/main; changes objective/linearization coherently. | **Selected as one all-or-nothing candidate.** |
| B. Official `ImuFactor` + bias-between/preintegration semantics | Strong official source, but Phase124 proves a 5-way 9-state `ImuFactor` plus separate bias-between versus native 6-way 15-state `CombinedImuFactor`; interval, bias prior, and admission also differ. | Broad state/covariance/topology rewrite; little safe reuse beyond raw IMU parsing. | Raw-only in principle, but changing bias/gauge/admission has high confounding risk. | Entire IMU/bias chain and conditioning. | Rejected; not topology/admission preserving and farther from a bounded port. |
| C. Official output interpolation/smoothing/offset chain | Source output/offset behavior is documented and Phase104/112 already froze final output alignment and Pixel5 offset. | Easy to implement mechanically, but it acts after graph handoff and would alter evaluated rows rather than the source graph. | High leakage risk: it changes evaluator-facing output and can look like post-hoc route repair. | Output only, not the remaining graph discrepancy. | Rejected; fixed output contract and no score-driven postprocessing. |
| D. GNSS-first→main handoff priors/clock-state changes | Handoff of position/velocity/C/D and C7 is already source-locked and structurally finite; source basis for changing priors is weaker than for the factor equations. | Feasible, but changes anchoring/information and can mask the singular/conditioning boundary previously audited. | Raw-only possible, but prior changes are difficult to attribute and resemble tuning. | Priors, gauge, clock/conditioning, potentially all epochs. | Rejected; no evidence supports changing priors/state semantics. |

The choice is not based on the sealed route scores.  A is selected because its
equations, keys, units, and geometry convention are explicit in the official
source, while the other choices either repeat a prior NO-CANDIDATE finding or
change output/priors without a source-complete, attribution-safe boundary.

## Source parity and frozen candidate equations

The official source tree is pinned at `29923f9f370f09ebc00f96d8cca375007a18e7d5`.
The relevant source hashes and evidence are:

| source | SHA-256 | evidence |
|---|---|---|
| `output/reproducibility-cache/gsdc2023/fgo_gnss.m` | `5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3` | `50-70`, `89-120`, `123-200` |
| `output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m` | `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3` | `63-106`, `129-143`, `169-221`, `252-323` |
| `output/reproducibility-cache/MatRTKLIB/+gt/Gsat.m` | `a56c323664660c1b7e771180e9bc18b08f809fe8a6d71b795366b96207871f6` | `259-306` |
| `output/reproducibility-cache/MatRTKLIB/+gt/Gobs.m` | `be27fd1a075829a00f0aec5f32a0040a884077a8ac256a10a2bc26def255fc88` | `1151-1165` |
| `output/reproducibility-cache/MatRTKLIB/+rtklib/geodist.m` | `4ddced92aea78defd7eb8b2040dd23b1b0575eb0c1ac719a128c8db62ab4a91f` | `1-20` |
| `output/reproducibility-cache/gtsam_gnss/src/PseudorangeFactor_XC.h` | `7f467698f2239818724eba4485b830fbc543586bbee06a21b8b2c03ae5b56415` | `15-64` |
| `output/reproducibility-cache/gtsam_gnss/src/DopplerFactor_VD.h` | `de2c11d06ff860a95785c84620e5ed57dcfd5ca24995a0e642e9a5984fe093e1` | `15-54` |
| `output/reproducibility-cache/gtsam_gnss/src/TDCPFactor_XXCC.h` | `cc8f5acabd43db25b5b4aca48f6c8ae6a0e2a399820fb950de9004555c9cd4b2` | `15-76` |
| `output/reproducibility-cache/gtsam_gnss/src/Pose3Point3Factor_PX.h` | `3893b16fe226fc253eafda16d634c0e229cb593373113f4de63cc66f47a1cb13` | `17-47` |
| `src/algorithms/fgo_gtsam_internal.hpp` | `7aba732064d2cd6b3858c60226b84aa904423becdf66cb386e628fae16fc5afd` | `2330-2352`, `2437-2477`, `2831-2875`, `3429-3500`, `3647-3679` |
| `src/algorithms/fgo_gtsam_backend.cpp` | `c0da12d7a1d5eeb9492b02ad80f0e9868cbda32e452bbdd9c3f90cc57b4db0fa` | `930-1000`, `1128-1252`, `1374-1516` |
| `src/algorithms/fgo_problems.cpp` | `9f75eda838a6a885657fb78d038733d6e1e027794c869c7a26b2dd610ff96aaa` | `518-567`, `624-781`, `790-842`, `1487-1615` |
| `apps/native/gnss_fgo_imu_no_base.cpp` | `efddfed4104d71db3df13a9b4cd0240bd603e4c78a3be313750dfe96c9273769` | Phase126-134 selector/configuration and summary bridge boundary |

### Official factor family

For each retained epoch `i`, let `x_i` and `v_i` be the official ENU
position/velocity state, `c_i` the seven-component metre clock vector, and
`d_i` the one-component metre/second drift state.  Let `x0_i` and `v0_i` be
the same-run initial states, `ell_i` the source `Gsat` LOS, and `h_s` the
source clock selector with `h_s[0]=1` and the mapped constellation component
also set to one (the official `sysfreq2sigtype` mapping).

The frozen candidate uses these source equations and key order:

```text
P:  eP_i = ell_iᵀ (x_i - x0_i) + h_sᵀ c_i - resPc_i
   keys [X_i, C_i], J = [ell_iᵀ, h_sᵀ]

D:  eD_i = ell_iᵀ (v_i - v0_i) + d_i - resD_i
   keys [V_i, D_i], J = [ell_iᵀ, 1]

TDCP (Pixel5 / XXCC):
   eL_i = ell_iᵀ ((x_{i+1}-x0_{i+1}) - (x_i-x0_i))
          + h_0ᵀ (c_{i+1}-c_i) - tdcp_i
   keys [X_i, X_{i+1}, C_i, C_{i+1}],
   J = [-ellᵀ, +ellᵀ, -h_0ᵀ, +h_0ᵀ].
```

The ordinary Pixel5 TDCP route is explicitly the official `XXCC` branch;
this candidate does not synthesize a D-based TDCP state or add ambiguity/DD
states.  `resPc_i`, `resD_i`, `tdcp_i`, their existing Phase126 raw-base
correction/miss ledger, fixed Phase118 sigma and Huber settings, and the
existing pair/gap/slip/finiteness gates are inputs to this adapter and are
not changed.

The official geometry provider must produce one finite, normalized LOS and
one consistent range/range-rate convention from the same source satellite
state.  `geodist.m` documents Sagnac-inclusive range, and `Gsat.m:304-306`
adds the explicit first-order range-rate Sagnac term.  The candidate therefore
requires exactly one equivalent Sagnac representation.  The current native
path rotates satellite state at build time and then uses `plainRange` in the
factor (`fgo_problems.cpp:564-567`; `fgo_gtsam_internal.hpp:2330-2352`).
The candidate must replace that convention coherently inside its local
adapter; it must not rotate and then add an unproved second Sagnac term.

### State and key boundary

The GNSS-first official graph already has `X/V/C/D` vector states.  The
official IMU graph additionally has a `Pose3 p_i` and a zero-noise
`Pose3Point3Factor_PX(p_i,x_i)` at each epoch (`fgo_gnss_imu.m:169-200`).
The Phase135 main-graph adapter therefore must materialize the official
auxiliary `x_i` vector and PX bridge before inserting affine P/TDCP factors;
D continues to bind the existing `V_i,D_i` states.  This is a deliberate
compound topology port, not a claim that the current native Pose3 graph is
already equivalent.  Existing IMU, bias, clock CCDD, motion, raw-base,
TDCP admission, and final output factors remain present exactly as frozen
unless the all-or-nothing candidate cannot prove their key/state contract,
in which case the route fails closed before optimization.

## Fixed settings and non-goals

The following are immutable for any separately authorized implementation:

- legacy/default and selector-off behavior is byte/semantic unchanged;
- same-run raw phone GNSS/IMU, broadcast navigation, and the existing raw-base
  RINEX path are the only data lineage; MAT, PDC, precomputed coordinates,
  station tables, truth, and saved solution coordinates are forbidden;
- Phase126/131/134 raw-base correction, canonical physical-band/GLONASS
  provenance, local-miss and shared-ledger rules remain exactly once;
- C7 is the seven-component metre clock state, D is metre/second, and the
  source CCDD equation/sigma (`0.1 m`) is unchanged;
- Phase118 fixed TDCP sigma (`0.03 m`) and official Highway Huber `k=0.5`
  remain; Phase117 dynamic sigma and Phase120 measurement toggle remain off;
- Phase99 `MULTIFRONTAL_QR` remains the main solver selection; GNSS-first
  solver, LM schedule, ordering, damping, max iterations, filters, initial
  seeds, and fail-closed policy remain unchanged;
- no new global ISB state, D synthesis, carrier ambiguity/DD state, factor
  duplication, final Pixel5 offset change, or output/evaluator change.

The candidate is not an accuracy claim.  It cannot be selected or modified
because a route score moved, and no route-specific sigma, threshold, or
initialization adjustment is permitted.

## Transactional implementation boundary

If a later implementation authorization is granted, the work must be
transactional even if developed in three local steps:

1. **Source geometry adapter:** construct one immutable initial state and
   official-equivalent range/LOS/rate per retained row, with finite,
   normalized, health, signal, time, and single-Sagnac checks.
2. **Factor/key adapter:** insert the complete P/D/TDCP affine family with
   exact state dimensions/order, C7 mapping, official `X/V/C/D` keys, and the
   Pose3-to-X bridge where required.  Record one-to-one source-row/factor
   counts by family.
3. **Atomic graph admission:** validate all family ledgers, keys, values,
   geometry, fixed settings, and handoff before exposing the candidate graph
   to the optimizer.  Any failure rejects the whole candidate; no partial
   family and no native fallback may run.

Required fail-closed predicates include missing/unsupported signal or C7
mapping, duplicate/reordered/nonfinite keys or values, zero/nonfinite range or
LOS, unproven/double Sagnac, factor-count mismatch, state-dimension mismatch,
TDCP pair mismatch, unsupported route configuration, Phase126 ledger drift,
LM/solver/config drift, or any forbidden input lineage.  A structural
qualification must also record that selector-off graph/factor/value hashes
are unchanged; the candidate does not authorize raw execution by itself.

## Qualification and reauthorization plan

Before any raw run, a separate implementation commit, focused synthetic
tests, target build, source/config hash manifest, launch-free evaluator, and
pre-raw accounting must be sealed.  A further independent authorization must
pin the implementation/binary and authorize exactly the intended route matrix
(initially MTV-A then LAX-T, one pass each).  Only after inventory and all
transactional predicates pass may each route invoke the candidate solver once.
Solutions must remain opaque (hash/row metadata only) until an independently
authorized truth lane, if ever granted.  This audit authorizes none of those
actions.

Focused tests must cover:

- exact P/D/TDCP residuals and Jacobians, metre versus metre/second units,
  key order, C7 mapping, `XXCC` clock difference, and fixed-LOS behavior;
- known Sagnac geometry/range-rate vectors and rejection of double correction;
- GNSS-first vector graph and IMU main Pose3+X/PX bridge dimensions,
  same-run C/D/position/velocity handoff, and no duplicate ISB/D state;
- family count conservation, duplicate/nonfinite/missing-row failure, and
  atomic rejection with no partial graph exposure;
- selector-off legacy regression, fixed sigma/Huber/filter/LM/QR/config
  invariance, and forbidden lineage rejection.

## Read accounting for this audit

| class | reads | execution |
|---|---:|---:|
| official/native source text | read-only | no solver |
| sealed aggregate/record metadata | metadata-only | no evaluator |
| phone GNSS/IMU/navigation payload | 0 | 0 |
| raw-base payload/header | 0 | 0 |
| truth/solution coordinate rows | 0 | 0 |
| MAT/PDC/precomputed coordinate data | 0 | 0 |
| solver invocations | 0 | 0 |
| accuracy calculations / score reads | 0 | 0 |
| Kaggle/token access | 0 | 0 |
| rerun/fallback/repair/sweep | 0 | 0 |

This document records the Phase135 triage and selected design boundary.  It
does not authorize implementation, raw input, solver execution, truth
evaluation, accuracy promotion, publication, or Kaggle submission.
