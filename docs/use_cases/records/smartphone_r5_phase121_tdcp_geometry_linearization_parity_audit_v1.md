# Phase121 TDCP geometry linearization parity audit

Status: sealed read-only source audit. This record compares the pinned
official GSDC2023 MATLAB TDCP factors with the native Phase118 ordinary TDCP
factor. It does not authorize a code change, raw execution, solver
execution, truth evaluation, accuracy calculation, solution publication, or
Kaggle action. No raw phone GNSS/IMU/navigation or base payload, MAT file,
precomputed coordinate, PDC input, or solution row was read.

## Decision

NO-CANDIDATE is frozen for the geometry-linearization question.

The official Pixel5 TDCP factor is an affine factor whose position Jacobian
is one LOS vector captured at the initial receiver geometry. The native
Phase118 factor is a nonlinear two-end range-difference factor: it evaluates
both endpoint antenna ranges and their Jacobians at the current Pose3 state,
using build-time Earth-rotation-corrected satellite positions. The two forms
agree only to first order at a specially aligned initial point. Away from
that point, GTSAM relinearization intentionally produces a different
objective and different Jacobians.

This difference cannot be isolated to a source-style frozen LOS row without
also deciding the coordinate/state mapping, lever arm, endpoint LOS choice,
satellite-motion convention, and Sagnac representation. Replacing the
native factor, adding a parallel factor, or silently changing endpoint
geometry would change the factor equation or graph information. Those are
not authorized by a read-only parity audit. The candidate is therefore
rejected before implementation; there is no Phase121 implementation or raw
run to authorize.

The Phase118 champion remains the baseline: fixed ordinary-TDCP sigma
0.03 m, official Type-dependent Huber k=0.5 for the two Highway routes,
legacy corrected-carrier measurement, C7/D/C0D handoff, MULTIFRONTAL_QR main,
raw-base correction and Pixel5 final offset. None of those contracts is
changed by this audit.

## Pinned baseline and source

The baseline references the already sealed Phase118 records, without opening
candidate coordinate rows:

| item | commit or SHA-256 | role |
| --- | --- | --- |
| Phase118 source-parity freeze | 5fcc06ab64189dd5dfb8001664bdb8496f85224f | fixed sigma / Type-k candidate anchor |
| Phase118 structural contract | 25f3bada02c4073175054b5d668de02020761208 | structural recipe and factor invariants |
| Phase118 structural result | b4ea80d96d7156f9fd62cc554f2deeedcc550cb0 | sealed structural provenance only |
| Phase118 accuracy freeze | ef8d11bc4fa64e2ed6cd5fface133979655953ff | isolated evaluator contract anchor |
| Phase118 official source tree | 29923f9f370f09ebc00f96d8cca375007a18e7d5 | pinned gsdc2023 source commit |

The official tree pins FTYPE=["L1","L5"] in both graph scripts. The source
files and native files examined in this audit are pinned below. The listed
line ranges are the evidence used for the equations; the file hashes are
computed from the current clean worktree/cache.

| source | SHA-256 | relevant lines |
| --- | --- | --- |
| output/reproducibility-cache/gsdc2023/fgo_gnss.m | 5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3 | 50-70, 89-120, 154-200 |
| output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m | c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3 | 63-83, 102-106, 263-323 |
| output/reproducibility-cache/gsdc2023/functions/gnsslog2obs.m | 665ce43d1fc3c5a9c3b3a6fd76f81539126e3ab897fbba484ab0ccd18964acff | 152-188 |
| output/reproducibility-cache/MatRTKLIB/+gt/Gobs.m | be27fd1a075829a00f0aec5f32a0040a884077a8ac256a10a2bc26def255fc88 | 1151-1165 |
| output/reproducibility-cache/MatRTKLIB/+gt/Gsat.m | a56c3236646606c1b7e771180e9bc18b08f809fe8a6d71b795366b96207871f6 | 259-306 |
| output/reproducibility-cache/MatRTKLIB/+rtklib/geodist.m | 4ddced92aea78defd7eb8b2040dd23b1b0575eb0c1ac719a128c8db62ab4a91f | 1-20 |
| output/reproducibility-cache/MatRTKLIB/src/mex/geodist.c | 3929ec3d57575634a921a81957ce91f2dce9314457694e3bd7e00bd82ec765c4 | 1-7, 50-80 |
| output/reproducibility-cache/gtsam_gnss/src/TDCPFactor_XXCC.h | cc8f5acabd43db25b5b4aca48f6c8ae6a0e2a399820fb950de9004555c9cd4b2 | 15-46, 50-90 |
| output/reproducibility-cache/gtsam_gnss/src/TDCPFactor_XXDD.h | dc6b92e824f439df56286b2c2f81c0acb744f698c45648baaf5da67d6b239aed | 15-49, 53-88 |
| src/algorithms/fgo_problems.cpp | 5675e82fc595933da4e7c14ad468ae69437dc04bac79a7ef55f1edfa03e098da | 485-512, 641-715, 1416-1535 |
| src/algorithms/fgo_gtsam_internal.hpp | 7aba732064d2cd6b3858c60226b84aa904423becdf66cb386e628fae16fc5afd | 1278-1340, 2330-2352, 3342-3502 |
| src/algorithms/fgo_gtsam_backend.cpp | c0da12d7a1d5eeb9492b02ad80f0e9868cbda32e452bbdd9c3f90cc57b4db0fa | 455-464, 503-515, 1206-1252 |
| src/algorithms/fgo.cpp | 145ffa3958d3a867ba53d962f07d346f9efa36e72140ad03d7a404ffdd3b5023 | 942-1008 |
| src/algorithms/fgo_internal.hpp | ee3a2bed13971c9bce52d043a293990703b92a2bc9d916376a896eadbf385ca7 | 279-290 |

## Official construction and equations

Let z_i = L_i lambda_i be accumulated carrier phase in metres, s_i the
broadcast satellite position used by official Gsat, r_i^0 the initial
receiver position, and d_i the satellite-clock term in metres.
gnsslog2obs.m:163-184 chooses a wavelength and stores ADR in cycles;
Gobs.m:1155 then forms

    resL_i = z_i - (rho_i^0 - d_i)

where rho_i^0 is the Sagnac-inclusive geodist range from Gsat.setRcvPos.
The atmospheric alternate is a separate field:

    resLc_i = z_i - (rho_i^0 - d_i - ion_i + trp_i)

at Gobs.m:1163-1164. Both official graph scripts consume resL, not resLc,
at fgo_gnss.m:185-186 and fgo_gnss_imu.m:307-308. Therefore the official
adjacent measurement is

    m_off = resL_2 - resL_1
          = (z_2-z_1) - (rho_2^0-d_2) + (rho_1^0-d_1).

The scripts compute one losvec at the first endpoint
(fgo_gnss.m:179-182) and pass the initial ENU positions
orgx1=posini.enu(i,:)' and orgx2=posini.enu(i+1,:)' to TDCPFactor_XXCC.
Its exact error is

    e_off(x1,x2,C1,C2) = losvec' * ((x2-orgx2) - (x1-orgx1))
                         + hc' * (C2-C1) - m_off,
    hc(0)=1, all other hc components=0.

TDCPFactor_XXCC.h:37-46 fixes the key order to X1, X2, C1, C2, and
:62-76 fixes Jacobians to [-losvec, +losvec, -hc, +hc]. Position values are
3-vectors in the official ENU graph, the C contribution is the selected
epoch-local C0 component in metres, and tdcp is in metres. The factor does
not evaluate a new range when the state changes.

The official TDCPFactor_XXDD is a different phone branch, not the Pixel5
branch used by the audited route recipe. Its exact clock term is

    dt * (D1 + D2) / 2

with +dt/2 derivatives for both D keys (TDCPFactor_XXDD.h:64-74). It must
not be substituted for the official Pixel5 C0 equation merely because the
native graph also carries D states.

## Native Phase118 construction and equations

The Phase118 baseline leaves the Phase120 atmosphere-cancellation selector
off. In fgo_problems.cpp:659-673, the retained legacy carrier value is

    corrected_carrier_i = raw_carrier_i + satellite_clock_i
                          - troposphere_i + ionosphere_i
    tdcp_carrier_i = corrected_carrier_i       (Phase118 baseline)

and :1459-1462 forms the accepted adjacent measurement

    m_nat = tdcp_carrier_2 - tdcp_carrier_1.

The pair lookup is exact on (SatelliteId, SignalType) and its gap,
clock-discontinuity, loss-of-lock, finite-measurement, and code-phase gates
are applied before factor creation (fgo_problems.cpp:1416-1508). This audit
does not propose changing that measurement or admission contract.

For the Phase118 IMU/main graph, the backend inserts
TimeDifferencedCarrierFactorSourceClockArm
(fgo_gtsam_backend.cpp:1213-1249). For endpoint pose P_i, the lever-arm
helper produces an antenna point a_i(P_i). The factor stores endpoint
satellite points s'_1,s'_2, prepared before graph construction. Its exact
predicted-minus-measured residual is

    rho_i(P_i) = ||a_i(P_i) - s'_i||
    e_nat(P1,C1,P2,C2) = rho_2(P2) + h' C2
                         - rho_1(P1) - h' C1 - m_nat,
    h = sourceClockComponentJacobian(0).

fgo_gtsam_internal.hpp:3473-3500 recomputes both ranges and endpoint
Jacobians on every factor evaluation. plainRange at :2340-2351
differentiates the range with respect to the receiver as
(receiver-satellite)/range; the previous endpoint receives the negative
of that row and the current endpoint the positive row. The lever-arm pose
Jacobian is then applied (:3409-3423 and :3482-3491). The source-clock
vector uses the C0 row with -h at the previous endpoint and +h at the
current endpoint.

The factor key order is instead

    previous Pose3, previous clock, current Pose3, current clock

(fgo_gtsam_internal.hpp:3447-3457), and the backend passes exactly that
order (fgo_gtsam_backend.cpp:1234-1239). The values are therefore Pose3
plus epoch-local Vector C in the source-parity path, not the official four
Vector keys in their official order. The scalar legacy class has the same
native interleaving with scalar clocks.

The native IMU backend documents the frame distinction at
fgo_gtsam_backend.cpp:263-268 and :455-464: the main state is a body pose
in local navigation ENU and ecef_T_nav plus the lever arm maps it to the
antenna position used with ECEF satellite points. This transform is part of
the native factor path and is absent from the official Vector-X factor
signature.

The Eigen implementation independently confirms the nonlinear geometry:
fgo.cpp:957-973 computes each endpoint range and clock prediction, while
:969-1007 forms separate endpoint LOS Jacobians and the weighted residual.
Its assembly uses the opposite raw residual sign (measured-predicted), but
the squared robust cost has the same sign invariance; this does not make the
official and native Jacobians equal.

## Term-by-term parity

| term | classification | source evidence and finding |
| --- | --- | --- |
| Carrier cycles, wavelength, metre unit | equivalent for audited L1/L5 | Official gnsslog2obs.m:152-184 converts ADR cycles with lambda=c/f; native fgo_problems.cpp:641-665 does the same before preparing metre fields. |
| Adjacent order and sign | equivalent at measurement contract | Both use current minus previous (fgo_gnss.m:185-186; native fgo_problems.cpp:1461-1462). GTSAM residuals are predicted minus measured in both custom/native factors. |
| Clock bias term | equivalent only for C0 role | Official XXCC selects C2[0]-C1[0] (TDCPFactor_XXCC.h:62-76); native source-clock TDCP selects the same C0 component (fgo_gtsam_internal.hpp:3481-3500). The official XXDD D branch is separate and not the Pixel5 path. |
| Position state type/frame | divergent | Official factor takes ENU Vector X states and fixed initial ENU vectors. Native IMU factor takes Pose3 body-in-nav plus a lever-arm/ecef transform and ECEF endpoint satellite points (fgo_gtsam_backend.cpp:263-268,455-464). |
| Satellite endpoint positions/times | not source-identical | Official Gsat builds range/LOS through geodist after its observation/nav calculation (Gsat.m:259-272). Native evaluates transmit time from pseudorange, applies a second state calculation after satellite-clock correction, and stores each endpoint position (fgo_problems.cpp:485-512,689-714). The endpoints are retained, but no proof of instruction-level equality exists. |
| Initial LOS | divergent | Official computes one LOS and passes it unchanged to the factor (fgo_gnss.m:63-69,179-194). Native stores endpoints but derives each LOS from the current endpoint state during evaluateError (fgo_gtsam_internal.hpp:3395-3425,3473-3500). |
| Geometric increment | divergent after first order | Official uses losvec' * (delta X2-delta X1). Native uses ||a2-s2'||-||a1-s1'||, including lever arm and endpoint-specific geometry. |
| Sagnac/Earth rotation | divergent representation; physical intent only broadly equivalent | Official geodist.m:18-20 documents Sagnac-inclusive range and the MEX wrapper calls RTKLIB geodist (geodist.c:50-80). Native rotates each satellite once at build time by Rz(OMEGA_E*||s-r_seed||/c) (fgo_internal.hpp:279-290) and intentionally uses a plain norm in TDCP (fgo_gtsam_internal.hpp:2331-2343) to avoid GTSAM's second Sagnac correction. A frozen official LOS cannot be inserted without preserving this one-time convention. |
| Atmospheric terms | divergent but unrelated to this audit | Official TDCP consumes resL, while native Phase118 uses corrected carrier with -trop+iono (Gobs.m:1155,1163-1164; fgo_problems.cpp:661-673). Phase120 already isolated this measurement candidate; Phase121 does not combine or revisit it. |
| Noise and Huber | baseline-preserved, not a geometry candidate | Phase118 fixes 0.03 m and Type mapping Highway=0.5; backend attaches the existing noise wrapper (fgo_gtsam_backend.cpp:1230-1232). No sigma, robust-k, pair gate, or LM change is proposed. |
| Pair/reject predicate | equivalent in native contract; out of scope | Native exact (SatelliteId,SignalType) lookup and existing gates are preserved (fgo_problems.cpp:1436-1508). Official upstream masks differ, but changing them would change factor population. |
| Key order and value dimensions | divergent | Official XXCC base is (X1,X2,C1,C2) with Vector/Vector/Vector/Vector (TDCPFactor_XXCC.h:15,37-46). Native base is (Pose1,C1,Pose2,C2) with Pose3/Vector/Pose3/Vector in source-clock mode (fgo_gtsam_internal.hpp:3434-3457). |

## Linearization derivation

Let g_i = d rho_i / d a_i be the endpoint native range gradient at the
native seed, after the chosen satellite rotation and lever arm transform.
Taylor expansion of the native prediction gives

    rho_2(P2) - rho_1(P1)
      = (rho_2^0-rho_1^0) + g_2' delta_a_2 - g_1' delta_a_1
        + O(||delta_a_1||^2 + ||delta_a_2||^2).

Thus its first-order residual is

    g_2' delta_a_2 - g_1' delta_a_1 + h'(C2-C1) - m_nat.

The official residual is

    losvec' delta_X_2 - losvec' delta_X_1 + h'(C2-C1) - m_off.

These are equal only if all of the following are established for the same
state convention: m_nat=m_off, the endpoint gradients are the same losvec,
the lever-arm map reduces to the official X coordinates, and the official
and native Sagnac/endpoint satellite positions coincide. The current source
proves none of these as a general identity. Even when they happen to hold at
the seed, native second-order terms remain and the native factor changes its
Jacobian after a state update; the official factor does not. A GTSAM
relinearization therefore does not make the two factors equivalent. It
repeatedly relinearizes the native nonlinear range factor, whereas it can
only re-evaluate the official affine expression with the same fixed LOS and
initial-point offsets.

## Candidate assessment

The only plausible geometry candidate was considered as
phase121-official-frozen-los-linear-tdcp-v1: add or replace the ordinary
native TDCP factor with the official TDCPFactor_XXCC frozen-LOS equation,
default off, while retaining Phase118 measurement, pair gates, sigma, Huber,
C7/D/C0D, QR, base correction, Pixel5 offset, IMU, and LM settings.

It is rejected before implementation for these source-backed reasons:

1. It requires a nontrivial ENU Vector-X to native Pose3/antenna mapping,
   including the lever arm and the ecef_T_nav transform; that is an
   equation and key/value-topology change, not a measurement-only change.
2. It requires choosing one official initial LOS while the native factor has
   two endpoint positions and two endpoint Jacobians. The choice is
   initial-point dependent and is not invariant under relinearization.
3. It must choose between the official Sagnac-inclusive geodist geometry and
   the native build-time-rotated-satellite/plain-norm contract. Mixing them
   risks omitting or double-applying Earth rotation.
4. Replacing or adding the factor changes the nonlinear objective, Jacobian,
   or factor information. It cannot satisfy the Phase118 requirement that
   graph factors, equations, and factor/reject counts remain unchanged.

Two alternatives were also ruled out rather than frozen:

* A parallel frozen-LOS factor would add information and change factor
  count/cost, so it is not an isolated diagnostic.
* A native diagnostic that compares the two residual/Jacobian forms without
  adding a factor is safe, but it is an observation-only future audit, not a
  source-backed correction candidate and is not implemented here.

Consequently the Phase121 freeze has candidate count zero. Future work must
retain the Phase118 nonlinear native TDCP factor unless a separately scoped
source audit resolves the complete geometry/frame/Sagnac contract. No truth
score, accuracy result, parameter sweep, or runtime behavior was used to
choose this decision.

## Read accounting and authority boundary

| activity | Phase121 count/status |
| --- | ---: |
| Official/native source text and git metadata | source-only, permitted |
| Sealed Phase118 contract/freeze metadata | metadata-only, permitted |
| New raw phone GNSS/IMU/navigation payload reads | 0 |
| New raw-base bytes, headers, or hashes | 0 |
| Truth payload, coordinate rows, or solution rows | 0 |
| MAT reads or generated MAT | 0 |
| Precomputed phone coordinates/trajectories | 0 |
| Native/solver process invocation | 0 |
| Accuracy calculations or score reads | 0 |
| PDC, Kaggle, or token access | 0 |
| Sweep, rerun, fallback, tuning, or implementation | 0 |

This audit is documentation only. The companion freeze record must retain the
exact audit hash, record decision=no-candidate, and keep raw execution,
solver, truth, accuracy, solution-publication, and Kaggle authority false.

