# Phase138 Phase135 affine catastrophic-error forensic audit

- Execution label: `Luna Max`
- Audit date: `2026-09-04`
- Repository: `/home/sasaki/workspace/rtklib_v2_ws/gnssplusplus-library`
- Branch: `feat/rtk-smartphone-performance-pr`
- HEAD at audit start: `b0c596a76acbeea9cb426c2e12efd304e87d9a5e`
- Worktree at audit start: clean.

This is a read-only source and sealed-metadata audit.  It did not materialize
or read raw phone GNSS/IMU, broadcast-navigation, or raw-base payloads; truth,
MAT, PDC, precomputed-coordinate, or saved solution rows; and it did not
launch a solver, calculate an accuracy score, rerun a route, or access Kaggle.
The Phase137 score aggregate is cited only as a sealed fact, never as a tuning
target.

## Decision

One source-backed, default-off correction is frozen in the companion JSON:

`phase138-affine-tdcp-anchor-range-constant-v1`

The first mathematically inconsistent boundary is the Phase135 affine TDCP
measurement argument.  `Phase135TdcpAffinePointFactor` and
`Phase135TdcpAffineVectorFactor` implement the official fixed-initial-LOS
residual, but the backend passes the legacy prepared **carrier difference**
(`factor.delta_carrier_m`) as that factor's `tdcp_m` argument.  The official
scripts pass a difference of carrier **residuals**, which includes the
negative initial geometric-range difference.  This missing geometric
constant is independent of the ENU/ECEF representation, C7/D units, PX bridge,
solver, noise, or factor admission, and is therefore a separable
measurement-constant correction.  The candidate deliberately preserves the
frozen Phase118 corrected-carrier/atmosphere input; the separate Phase120
atmosphere-normalization question is not silently folded into this candidate.

The candidate is design-only.  It authorizes no implementation, raw run,
truth evaluation, accuracy promotion, or solution publication.  The existing
Phase120 atmosphere selector remains off; the candidate's source quantity is
defined explicitly rather than silently toggling another selector.  Existing
P/base re-materialization and IMU velocity-initialization divergences remain
documented secondary risks and are not changed by this audit.

## Sealed evidence (not re-read as coordinates)

The Phase136 structural seal is commit
`65b24a939a373421015f07b119fdbec04d5c6ebb`, JSON SHA-256
`d79dbac9dba4ce352d1d2b23f92d87fa44057221822a67f94c494f41f789c2a7`.
It records, for MTV-A and LAX-T respectively, affine P/D/TDCP counts
`43259/20748/31269` and `30664/8942/14012`, finite complete C7/D handoff,
main `MULTIFRONTAL_QR`, and finite strict structural cost decreases.  It also
records eight and six C0D indeterminate attempts, respectively.  Those
structural facts do not establish a correct measurement semantic.

The Phase137 truth-only seal is the current HEAD,
`b0c596a76acbeea9cb426c2e12efd304e87d9a5e`; its JSON SHA-256 is
`cd3d57f445a9f5bc07206600edc5e27fd1638f857723f9fa3b2aa7d490caaf5e`.
Its sealed aggregate reports MTV-A `335.1005139049 m`, LAX-T
`197.8309465842 m`, and macro `266.4657302445 m`, with finite/domain guards
passing but the strict `macro < 0.782 m` promotion gate failing.  Candidate and
truth rows were not interpreted here, and no score was recomputed.

## Coordinate, frame, and bridge audit

### Official contract

The official GNSS-only and GNSS/IMU scripts use `x_ini = posini.enu'` and
`v_ini = velini.enu'` (`fgo_gnss.m:89-93`, `fgo_gnss_imu.m:102-106`).
`fgo_gnss_imu.m:145-149` constructs each `Pose3` translation from the same
ENU position.  Thus the official `X` is a local ENU three-vector, not an
absolute ECEF point.  Official LOS is `-satr.e` transformed from ECEF to ENU
(`fgo_gnss.m:63-70`; the same operation is at
`fgo_gnss_imu.m:76-83`).

### Native Phase135 chain

The GNSS-first Point3 path keeps its source seed and affine point state in ECEF
(`fgo_gtsam_backend.cpp:1120-1127`).  The IMU main path converts the same
seed difference to ENU (`:831-860`, `:1120-1148`), constructs
`ecef_T_nav` from `ecefFromEnuRotation` (`:491-505`), and maps the auxiliary
ENU `X` back to ECEF in `antennaPositionOf` (`:623-640`).  With the frozen
Pixel5 lever arm equal to zero, `Pose3` translation and auxiliary `X` are
bound by the constrained PX factor (`:861-864`; factor implementation in
`fgo_gtsam_internal.hpp:1439-1467`).

Let `R` be the ENU-to-ECEF rotation and `o` the nav origin.  The native
mapping is

```text
r_ecef = o + R x_enu,       los_enu = R^T los_ecef.
```

The matrix in `include/libgnss++/core/coordinates.hpp:87-115` and the matching
`ecefFromEnuRotation` matrix are orthogonal.  Therefore

```text
los_ecef^T (r_ecef-r0_ecef)
  = (R^T los_ecef)^T (x_enu-x0_enu),
||R u|| = ||u||.
```

Changing the origin by an ENU translation `a` gives
`o' = o + R a`, `x'_i = x_i-a`, so every increment and the PX equality are
unchanged.  The source and native code therefore prove a consistent rigid
frame/bridge convention; no isolated `los^T R` or bridge-sign correction is
supported.

### Clock, drift, and output units

The source C7 mapping sets a base component and the source constellation
component (`fgo_gtsam_internal.hpp:1302-1342`).  Native Phase135 uses seven
metre-valued C components and one metre/second D component, with the CCDD
equation and `0.1 m` sigma (`:1705-1710`; backend `:983-1041`).  The public
clock conversion divides a meter state by the speed of light exactly once
(`:2629-2640`).  These are source-consistent units, not a plausible
catastrophic scale error.

The official Pixel5 offset is `offsetRL=-0.10`, `offsetUD=-0.30`
(`add_position_offset.m:29-38`).  Native applies the same mapped ENU offset
once after the main solve and converts it to ECEF
(`gnss_fgo_imu_no_base.cpp:10220-10267`).  Stage/handoff values are not passed
through this post-solve operation.  Output offset duplication is therefore
not the first inconsistent boundary.

## Factor and observation parity

| family/term | classification | evidence |
| --- | --- | --- |
| P affine equation | equivalent at one common anchor, but main preparation is stale | Official `PseudorangeFactor_XC.h:48-64` uses `los^T(X-X0)+h^TC-resPc`. Native Phase135 uses the same form (`fgo_gtsam_internal.hpp:1493-1510`) and computes `corrected_pseudorange_m-source_range` (`fgo_gtsam_backend.cpp:1180-1215`). However, the measurement was prepared before GNSS-first and is reused after stage handoff (`apps/native/gnss_fgo_imu_no_base.cpp:8974-9020,9685-9715`), while official `fgo_gnss_imu.m:63-92` recomputes Gsat/Gobs/base correction at the stage anchor. |
| D affine equation | equivalent source convention after the Phase135 Doppler correction | Official rate/residual are `Gsat.m:299-306` and `Gobs.m:1157-1158`; native uses the source ECEF state, receiver velocity, clock drift, and one Sagnac term (`fgo_gtsam_internal.hpp:1402-1437`; backend `:1402-1456`). Main measurement preparation can still be stale, but no D sign/unit error is proven. |
| Ordinary TDCP affine equation | **divergent measurement semantic** | Official `fgo_gnss.m:179-194` and `fgo_gnss_imu.m:302-317` pass `resL(i+1)-resL(i)` to `TDCPFactor_XXCC`. Native builds `tdcp_carrier` and stores `current.tdcp_carrier_m-previous.tdcp_carrier_m` (`fgo_problems.cpp:731-745,1537-1545,1593-1607`), then passes that field directly to the affine factor (`fgo_gtsam_backend.cpp:1543-1583`). |
| TDCP key/Jacobian | equivalent for the selected Pixel5 XXCC shape | Official key order and Jacobians are `TDCPFactor_XXCC.h:37-76`; native affine class uses the same four logical X/C endpoints and signs (`fgo_gtsam_internal.hpp:1593-1645`). |
| PX bridge | equivalent under zero lever arm | Official `Pose3Point3Factor_PX.h:35-47`; native local copy and constrained insertion are `fgo_gtsam_internal.hpp:1442-1467`, backend `:861-864`. |
| CCDD | equivalent source equation/unit | Official `ClockFactor_CCDD.h:39-63`; native source-meter CCDD implementation is at `fgo_gtsam_internal.hpp:1705-1710` and backend clock construction. |
| IMU/bias topology | divergent, but later and not isolated here | Official uses `ImuFactor` plus a separate bias-between factor (`fgo_gnss_imu.m:280-299`); native uses `CombinedImuFactor` (`fgo_gtsam_backend.cpp:954-968`). Phase124 already rejected a topology-only port. |

## The first inconsistent boundary: TDCP constant, not frame

Let `z_i=L_i lambda_i` be carrier phase in metres, `rho_i` the source
Sagnac-inclusive range at the affine initial endpoint, `d_i` the satellite
clock term in metres, and `m_i` the native prepared `tdcp_carrier` value.
The official Gobs expression is (`Gobs.m:1154-1165`)

```text
resL_i = z_i - (rho_i - d_i),
tdcp_official = resL_2 - resL_1
               = (z_2-z_1) - (rho_2-rho_1) + (d_2-d_1).
```

The native Phase135 preparation instead uses, with Phase120 off,

```text
m_i = z_i + d_i - trop_i + iono_i,
tdcp_native = m_2-m_1.
```

Phase120 on removes the atmosphere terms but still does not subtract the
geometric range.  The selected Phase135 backend passes `tdcp_native` to a
factor whose position term is affine about the initial endpoints:

```text
e_affine = los^T ((X_2-X0_2)-(X_1-X0_1)) + h0^T(C_2-C_1) - tdcp_native.
```

At the initial values, the affine position term is exactly zero:

```text
e_affine(X0) = h0^T(C_2-C_1) - tdcp_native.
```

The existing nonlinear legacy factor has (`fgo_gtsam_internal.hpp:3696-3780`)

```text
e_legacy = rho_2(X_2)+h0^T C_2 - rho_1(X_1)-h0^T C_1 - tdcp_native.
```

Consequently, at the same initial state and with the same clock convention,

```text
e_affine(X0) - e_legacy(X0) = -(rho_2-rho_1).
```

This is an exact algebraic difference, not an estimate from the sealed score.
Satellite motion alone means `rho_2-rho_1` is not generally sub-metre; no
route-specific magnitude is asserted because payloads were not read.  The
official residual difference proves the missing geometric term.  The selected
correction is the separable geometric part applied to the existing Phase118
measurement:

```text
tdcp_phase138 = tdcp_native - (rho_2^0-rho_1^0).
```

This makes the affine initial residual agree with the legacy first-order
residual under the same source-range convention.  It does not change the
frozen atmosphere normalization; replacing `tdcp_native` by the full official
`resL` difference is a different compound candidate and is explicitly not
selected here.

The current implementation itself records the boundary: its comment calls the
input the “historical prepared-carrier delta” (`fgo_problems.cpp:1537-1545`),
while the affine class subtracts its argument as `tdcp_m`
(`fgo_gtsam_internal.hpp:1639-1645`).  Finite values and equal factor counts
do not test this semantic distinction, explaining why Phase136 structural
gates could pass it.

## Secondary source divergences (not selected)

1. `fgo_problems.cpp:520-631,709-781` materializes corrected P/carrier values
   and atmosphere terms using the pre-stage raw SPP seed.  The application
   copies GNSS-first positions/C/D into the existing problem but does not
   rebuild those observation values (`gnss_fgo_imu_no_base.cpp:9685-9715`).
   Official `fgo_gnss_imu.m:63-92` recomputes Gsat, residuals, and base
   compensation after the GNSS-only result.  This is a real P/base/TDCP
   anchor divergence, but correcting it is a compound re-materialization
   change, not a single frame/bridge constant.
2. Official main inserts every GNSS-first `v_i` (`fgo_gnss_imu.m:169-187`).
   Native main can initialize each velocity by finite-differencing copied
   positions (`fgo_gtsam_backend.cpp:867-930`); the optimized stage velocity
   sequence is otherwise used as stop/heading seeds by the app.  This is a
   later IMU initialization semantic difference and is not a justification
   for synthesizing or tuning velocity here.
3. Native Phase135 TDCP also retains the Phase118 corrected-carrier
   atmosphere convention when Phase120 is off, whereas official `resL` has no
   explicit ionosphere/troposphere terms.  That atmosphere choice was already
   isolated as Phase120 and is not used to tune this audit.  The selected
   correction is the missing affine-anchor range constant; any source-exact
   atmosphere change requires its own contract and must not be hidden in a
   score-driven patch.

## Candidate boundary and future implementation contract

The single candidate is a measurement-constant adapter for **ordinary TDCP
only** when Phase135 is enabled:

```text
tdcp_affine_i = tdcp_native_i - (rho_{i+1}^0-rho_i^0).
```

`rho_i^0` must be evaluated using the same source satellite state, exact
affine initial endpoint, and exactly one official Sagnac convention used by
the factor LOS.  The existing `tdcp_native` remains the only carrier/clock /
atmosphere input; no raw carrier, satellite-clock, atmosphere, range, zero,
held, inferred, or saved-solution fallback may be introduced.  Full official
`resL` replacement (including its atmosphere convention) is not part of this
freeze and requires a separate source-complete contract.

The existing pair lookup, loss-of-lock/clock/gap/code-phase gates, admitted
row count, fixed sigma `0.03 m`, official Highway Huber `k=0.5`, C7/D/CCDD,
P/D factors, C7 mapping, QR/LM, IMU topology, raw-base correction, and final
Pixel5 offset are immutable.  The legacy nonlinear TDCP path must remain
unchanged and selector-off must be byte/semantic identical.  Missing source
endpoint or residual data, nonfinite values, key/order mismatch, or an
attempted mixed legacy/affine TDCP family must fail closed before optimizer
admission.  This candidate does not authorize fixing the secondary P/base
re-materialization or IMU velocity-init divergences.

## Source and sealed-record pins

| item | SHA-256 or commit | relevant evidence |
| --- | --- | --- |
| official source tree | commit `29923f9f370f09ebc00f96d8cca375007a18e7d5` | pinned official GSDC2023 tree |
| `fgo_gnss.m` | `5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3` | `50-79,89-120,154-200` |
| `fgo_gnss_imu.m` | `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3` | `63-106,129-149,169-221,252-323` |
| `Gsat.m` | `a56c323664660c1b7e771180e9bc18b08f809fe8a6d71b795366b96207871f6` | `259-306` |
| `Gobs.m` | `be27fd1a075829a00f0aec5f32a0040a884077a8ac256a10a2bc26def255fc88` | `1151-1165` |
| `geodist.m` | `4ddced92aea78defd7eb8b2040dd23b1b0575eb0c1ac719a128c8db62ab4a91f` | `1-20` |
| `TDCPFactor_XXCC.h` | `cc8f5acabd43db25b5b4aca48f6c8ae6a0e2a399820fb950de9004555c9cd4b2` | `15-76` |
| `Pose3Point3Factor_PX.h` | `3893b16fe226fc253eafda16d634c0e229cb593373113f4de63cc66f47a1cb13` | `17-47` |
| native `fgo_problems.cpp` | `6443b13d9df54e627bc73270b7d28df9c5d309043e2e194ba2b6b5c1b274bc25` | `520-745,1499-1630` |
| native `fgo_gtsam_backend.cpp` | `c6e2e41eccafcf37bdcaf371b3a7b2d84de945706cc6d520efd47f50ff042903` | `491-640,831-930,1120-1215,1381-1618` |
| native `fgo_gtsam_internal.hpp` | `e20d9eb7d3da77277cf89f4d080ad5d260d71f98f054394d8d327f682e063ff0` | `1245-1400,1439-1703,3696-3855` |
| native `fgo_internal.hpp` | `0f2a71a0d12a52dd1b9efbfa91414ae695111a5ac3a4c37e5fe4ed123f48087f` | `376-387,820-845` |
| native coordinate helpers | `d90af3973c14704ffed2b8a83ea1d9f3e241cb613c46dc9be0a435b674b2c1fe` | `ecef2enu/enu2ecef:87-115` |
| native Pixel5 offset helper | `a37fc5d93f2f32cf9e9c6d684b38d9ce855db3acaca743d765d870530ebb2fce` | `70-109` |
| native app handoff/output | `4ce95d3ef322c83fca2aa5327e9150ee2d2890fe012d6cc4d3b80d439939656e` | `8974-9020,9462-9715,10170-10267` |

## Confidence and limits

| finding | classification | confidence |
| --- | --- | ---: |
| official X is local ENU; native transform/PX bridge is internally consistent | source proof | 0.99 |
| C7 metres, D metres/second, CCDD, clock export, and Pixel5 offset have no isolated scale/sign error | source proof | 0.99 |
| Phase135 affine TDCP receives carrier delta instead of official residual delta | source proof | 0.99 |
| stale post-stage P/base/atmosphere materialization exists | source/control-flow proof | 0.98 |
| TDCP semantic error is the sole cause of the sealed accuracy aggregate | not established; no factor residual norms or solution rows were read | <=0.20 |

The Phase136/137 sealed records contain costs, counts, and finite flags but no
per-family initial residuals, affine-vs-legacy residuals, Jacobian norms, or
factor rows.  The requested numeric legacy/affine comparison is therefore not
available without forbidden payload reads or a rerun.  The algebra above is
the only valid residual comparison in this audit.

## Focused tests required before any future implementation

No tests or builds were run in this read-only audit.  A separately authorized
implementation must add synthetic tests for:

- official `resL` and adjacent TDCP delta versus the legacy initial residual,
  including the exact `-(rho_2-rho_1)` identity;
- ECEF/ENU rotation and origin-translation invariance, PX equality, C7/D
  dimensions/units, and one Sagnac representation;
- raw carrier, satellite-clock, source-range, and endpoint-key finiteness;
- pair/gate/factor-count conservation, no mixed TDCP family, and atomic
  fail-closed admission;
- selector-off legacy byte/semantic regression, fixed sigma/Huber/LM/QR,
  raw-base exactly-once, and Pixel5 offset exactly-once.

## Read accounting and authority

| activity | Phase138 count/status |
| --- | ---: |
| tracked native/official source and git metadata | source-only, permitted |
| sealed Phase136/137 aggregate metadata | metadata-only, permitted |
| new raw phone GNSS/IMU/navigation payload reads | 0 |
| new raw-base payload/header reads | 0 |
| truth payload or truth-coordinate rows | 0 |
| solution-coordinate rows or candidate payload | 0 |
| MAT/PDC/precomputed-coordinate reads | 0 |
| solver/native execution | 0 |
| accuracy recalculation or score evaluation | 0 |
| Kaggle/token/network dataset access | 0 |
| rerun, fallback, repair, sweep, or tuning | 0 |
| implementation/build/test execution | 0 |

The companion freeze JSON records the one design-only candidate and keeps all
implementation, raw, solver, truth, accuracy, publication, and Kaggle
authority false.
