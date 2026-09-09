# Phase112 remaining main-FGO source-parity audit

- Execution label: `Luna Max`
- Audit date: `2026-09-03`
- Status: `sealed-read-only-audit-one-candidate-qualified`
- Scope: official GSDC2023 MATLAB source, the native Phase101/107/109
  implementation, and sealed aggregate metadata only.
- No raw, raw-base, truth, MAT, solver, accuracy, PDC, precomputed-coordinate,
  Kaggle, or token resource was opened or executed by this audit.

## Decision

The one source-backed remaining main-output mismatch is the upstream
phone-to-position physical offset.  The official `fgo_gnss_imu.m` always calls
`add_position_offset` after converting the optimized ENU position back to the
public position (`fgo_gnss_imu.m:359-365`).  The Phase101/109 native main result
is emitted at the optimized antenna position; the corresponding native helper
exists, but `--native-upstream-position-offset` is currently excluded by the
Phase101 C7/D/QR guard (`apps/native/gnss_fgo_imu_no_base.cpp:921-939`).

For both in-scope routes the phone is `pixel5`.  The pinned official branch is
`offsetRL=-0.10 m`, `offsetUD=-0.30 m` (`add_position_offset.m:29-31`), whose
rotation-preserving magnitude is
`sqrt(0.10^2 + 0.30^2) = 0.31622776601683794 m`.  That is a bounded physical
correction in the requested 0.2--0.4 m residual scale; it is not a tuned
weight, sigma, LM setting, or coordinate seed.

The candidate frozen by the companion JSON is therefore
`phase112-main-output-upstream-position-offset-v1`: enable the already
implemented helper only after the finite optimized Phase101/109 main result
and before existing raw-UTC-key alignment/output.  The GNSS-first C/D/velocity
handoff, base correction path, C7 topology, QR branch, IMU graph, and all
solver parameters remain unchanged.  The native default remains off.

This is a source-parity hypothesis, not a promotion or causal proof.  The
sealed accuracy records establish the remaining error but cannot prove that a
fixed output offset explains all of it.

## Sealed evidence

The records below were read as sealed metadata; no solution or truth payload
was reopened.

| Authority | MTV-A | LAX-T | What it establishes |
|---|---:|---:|---|
| Phase105 stage/main attribution | stage `3.334004080470148` → main `1.139793072101309` m | stage `5.782811093104809` → main `3.310820065300743` m | main improves stage, but does not localize the remaining error |
| Phase108 score of sealed Phase107 raw-base output | `1.1396560717187856` m | `0.8241679860216076` m | post-C7/base/QR route scalars; strict `0.782 m` macro gate still failed |
| Phase101 structural C7/D/QR | GNSS-first `157` accepted; main `12` accepted | GNSS-first `270` accepted; main `12` accepted | strict finite cost progress and exact full handoff already work |
| Phase109 raw-base structural | all route gates passed | all route gates passed | existing raw-base/additional-band path is not changed by this candidate |

Phase105's stage sidecars were finite, Earth-valid, and exactly timestamp
aligned.  Phase101 and Phase109 likewise sealed full C7/D coverage, QR main
selection, and no fallback.  Therefore the evidence does not support changing
the solver, damping, graph rank, C/D handoff, or base correction to address this
specific remaining output-scale discrepancy.

Historical Phase21 sealed development metadata provides an independent
boundedness check for the same already-implemented helper: graph factor count,
finite coverage, and deterministic repeat were unchanged; the observed fixed
offset magnitudes were `0.22360679774997905 m` (pixel7pro) and
`0.43011626335213143 m` (mi8).  Its development score was not used to tune
Phase112 and is not a validation or release claim.

## Source-to-native parity audit

### IMU units, axes, and navigation frame

The official `deviceimu2imu.m:6-28` copies `UncalAccel` and `UncalGyro`
`MeasurementX/Y/Z` fields without rescaling or an axis permutation.
`parameters.m:176-203` fixes `g=9.80665`, `imu_sync="gyro"`,
`imu_sync_coefficient=0.5`, mounting angles `[-85,178,-94]` degrees, and zero
mounting translation.  `fgo_gnss_imu.m:129-143` uses Z-up gravity
`[0;0;-g]`, the same sensor mounting rotation, and the synchronized streams.

The native raw contract states metres/second squared and radians/second and no
silent rescale (`include/libgnss++/io/imu.hpp:202-228`); the Android loader
keeps gyro measurements in rad/s and does not fold the device bias into the
measurement (`src/io/imu.cpp:1092-1107`).  The native adapter applies the
same `Rz(-94) * Ry(178) * Rx(-85)` matrix to the raw vectors
(`apps/native/gnss_fgo_imu_no_base.cpp:1413-1424,3566-3579`) and the backend
uses body-FLU Pose3 in local ENU (`fgo_gtsam_backend.cpp:381-390,610-623`).

Finding: no source-backed unit, sign, or mounting-axis mismatch remains strong
enough to freeze.  The native class is `PreintegratedCombinedMeasurements`
instead of the MATLAB wrapper's `PreintegratedImuMeasurements`, but its
gravity/noise/bias state contract is explicitly pinned below; changing the
class would be a graph/solver experiment, not a safe parity correction.

### Gravity, Coriolis, and earth rotation

The official explicitly sets zero Coriolis and `[0;0;-9.80665]` gravity
(`fgo_gnss_imu.m:136-143`).  Native `MakeSharedU(problem.imu.noise.gravity_mps2)`
sets the same negative-Z gravity (`fgo_gtsam_backend.cpp:547-555`); the pinned
GTSAM implementation returns zero from `integrateCoriolis` when no omega is
provided (`gtsam/navigation/PreintegrationCombinedParams.h:66-71`,
`gtsam/navigation/PreintegratedRotation.cpp:139-142`).  Satellite geometry is
already earth-rotation corrected by the GNSS factor path, so this audit found
no additional native earth-rotation term to change.

Finding: effective gravity/Coriolis/earth-rotation semantics are aligned.

### Bias initialization, priors, and random walk

There is a real source difference, but it is not the selected candidate.  The
official creates `imuBiasZero` (`fgo_gnss_imu.m:151-155`), inserts it at every
epoch (`:169-179`), uses a zero-bias prior (`:180-187`), and scales the bias
between-factor sigma by `sqrt(numel(IMUindices))` (`:296-300`).  Native
`alignStatic` estimates gyro mean and accelerometer bias
(`src/fusion/fusion_initialization.cpp:13-48`), passes those values into the
preintegrator (`fgo_gtsam_backend.cpp:543-545`), and uses finite first-state
bias priors plus the configured random walk (`:554-555,685-690`).

This is a plausible initialization sensitivity, but replacing the raw-only
static alignment with official zero bias would change the linearization and
preintegrated residuals throughout the main graph.  No sealed bias trajectory
or route-level ablation is available, so its 0.2--0.4 m impact is unproven.
It remains rejected while the fixed post-solve geometry correction is both
source-exact and bounded.

### Preintegration covariance and IMU/GNSS time policy

The official uses synchronized gyro-anchored samples and sets
accelerometer/gyro covariance from the `0.5` synchronization coefficient,
integration sigma `0.05`, accelerometer bias RW `0.00025`, and gyro bias RW
`0.0000005` (`parameters.m:176-203`, `fgo_gnss_imu.m:136-143,154-155`).
Native carries the same effective public values (`buildImuInput` at
`apps/native/gnss_fgo_imu_no_base.cpp:3722-3736`) and squares them exactly once
when constructing the GTSAM covariance (`fgo_gtsam_backend.cpp:548-555`).

The interval policy is not bit-identical: MATLAB maps elapsed clocks and uses
linear extrapolation, synchronizes acceleration to gyro with
`interp1(...,"linear","extrap")`, repeats the final `dt`, and includes both
interval endpoints (`imuprocessing.m:7-40`, `fgo_gnss_imu.m:280-285`).  Native
uses GNSS elapsed anchors, interior linear acceleration interpolation, a
bounded endpoint-nearest sample, and no extrapolation
(`imu.hpp:202-228`, `imu.cpp:1008-1054,1064-1107`); the backend integrates
`[t_i,t_{i+1})` and a finite tail sample (`fgo_gtsam_backend.cpp:572-599`).

This is a genuine future IMU-ablation axis, but it changes every preintegrated
factor and gap policy and has no sealed route telemetry proving a 0.2--0.4 m
effect.  It is not frozen in Phase112.

### Lever arm, pose/velocity priors, and stage handoff

The official mounting translation is exactly zero (`parameters.m:202-203`).
Native's default `pose3_lever_arm_body_m` is also zero
(`include/libgnss++/algorithms/fgo_config.hpp:833-837`), and its antenna/body
conversion is explicit (`fgo_gtsam_backend.cpp:418-426,618-623`).  The native
Phase101/109 in-memory handoff copies the same-run GNSS-first position and
receiver clock after exact C/D validation (`apps/native/gnss_fgo_imu_no_base.cpp:
7221-7275,7326-7357`).

The official graph has effectively unconstraining per-epoch state priors and
conditionally adds stop/height factors (`fgo_gnss_imu.m:165-249`), whereas
native uses finite first-state Pose3/velocity/bias priors and leaves upstream
stop constraints opt-in (`fgo_gtsam_backend.cpp:672-705`; the stop selector is
not in the Phase101/109 recipe).  A height reference would also cross the
forbidden MAT/reference-data boundary.  Adding those factors is therefore a
different graph candidate, not a safe explanation for a fixed output offset.

### Output geometry and epoch mapping

The official post-solve helper selects a phone-specific local vector and maps
it through `Rx*Ry*Rz(rpy-[0,0,pi])` (`functions/add_position_offset.m:10-38`,
`functions/eul2rotm.m:1-16`).  The native helper is already a source-exact
raw-only port (`include/libgnss++/algorithms/upstream_position_offset.hpp:37-110`)
and its application is after the finite optimized Pose3 result and before raw
UTC-key alignment (`apps/native/gnss_fgo_imu_no_base.cpp:7729-7812,7814-7848`).
However, Phase101/109 reject the flag in their strict optional-switch guard,
so their public main position is the uncorrected optimized antenna position.
This is the selected mismatch.

The timestamp contract itself is not the mismatch: native performs exact
integer raw-UTC alignment with no interpolation/edge hold in the sealed
Phase105 run (`apps/native/gnss_fgo_imu_no_base.cpp:7814-7848`; Phase105 seal),
and the helper does not change timestamps, C/D, velocities, or row ordering.

The official two-pass script also calls the helper in `fgo_gnss.m:236-238`
before saving the GNSS-only result and again in `fgo_gnss_imu.m:364-365` for
the final result.  Phase112 deliberately freezes only the remaining
*main-output* post-processing boundary: the existing Phase101 GNSS-first
position/velocity/C/D handoff remains unchanged so this one candidate cannot
confound the already healthy structural stage-to-main experiment.  A future
request to reproduce the historical two-pass double application would require
a separately audited candidate and is not included here.

## Candidate and rejected alternatives

| Candidate | Source evidence | Why not selected / selected |
|---|---|---|
| Official phone-position offset (selected) | `add_position_offset.m` fixed branch; pixel5 norm `0.31622776601683794 m`; existing native helper and fail-closed wiring | post-solve only, bounded, no graph/solver/noise/input change; **selected** |
| Official zero IMU bias initialization | `imuBiasZero` versus native `alignStatic` bias | changes all preintegrated residuals/linearization; no sealed impact evidence |
| MATLAB inclusive/extrapolated IMU synchronization | `imuprocessing.m` versus native bounded endpoint and half-open backend interval | changes the full IMU sample/time policy; no route-level evidence and higher confounding risk |
| Official stop/height/prior additions | `fgo_gnss_imu.m:180-249` | changes graph factors; height branch requires a forbidden reference/MAT lane |

No new diagnostic residual/time-offset candidate is needed because the selected
fixed geometry mismatch is source-exact and quantitatively bounded at the
requested scale.  This does not claim it will pass the strict accuracy gate.

## Frozen candidate boundary

The companion freeze JSON defines the only permitted future implementation
boundary:

1. Keep the exact Phase101 C7 epoch-local clock vector, raw D handoff, and
   Phase99 `MULTIFRONTAL_QR` main selector.  If the existing Phase107 or
   Phase109 raw-base selector is composed, keep its sealed correction,
   source-miss mask, and exactly-once policy unchanged.
2. Permit the existing `--native-upstream-position-offset` flag only in that
   exact recipe by a guard-only admission change.  Do not remove any other
   forbidden-switch, raw-input, or fail-closed check.
3. Use only same-run optimized native Pose3 `Rot3::rpy()` and the fixed official
   phone table.  For Pixel5, use exactly `[-0.10,-0.30]` in the official
   `[RL,UD]` naming and `[offsetUD,offsetRL,0]` multiplication order.
4. Apply the ENU displacement through the existing finite nav origin to ECEF
   conversion, recompute geodetic fields, and then run the existing exact raw
   UTC-key alignment.  Require every finite output epoch to be corrected;
   unknown phone, nonfinite attitude/origin/position, out-of-Earth result,
   missing epoch, fallback, or duplicate/misaligned key fails closed.
5. Do not read truth or publish/score solution rows in the future structural
   qualification.  A separate truth-only authorization would be required for
   any accuracy evaluation.

The implementation already present at historical commit `c0bbfc4` is not
modified by this audit.  No raw execution is authorized by this record.

## Read accounting

| Activity | Count in this audit |
|---|---:|
| Official source files read | 8 source files/functions (text only) |
| Native/GTSAM source files read | source/header text only |
| Sealed aggregate metadata consulted | Phase101, Phase105, Phase108, Phase109, and historical Phase21 |
| Raw device GNSS/IMU/nav reads | `0` |
| Raw-base bytes/headers/metadata reads | `0` |
| Truth payload reads | `0` |
| MAT payload reads/generated | `0` |
| Solver invocations | `0` |
| New accuracy calculations | `0` |
| PDC/precomputed-coordinate reads | `0` |
| Kaggle/token access | `0` |
| Reruns, fallback, tuning, or external mutation | `0` |

This audit and its companion freeze are design records only.  They authorize
neither implementation, raw execution, truth evaluation, release, nor Kaggle
submission.

## Pinned authorities

| Authority | SHA-256 or commit |
|---|---|
| `output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m` | `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3` |
| `output/reproducibility-cache/gsdc2023/fgo_gnss.m` | `5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3` |
| `output/reproducibility-cache/gsdc2023/preprocessing.m` | `976629d187e7fab5868eb8e5676a4d40520eb23db1254f7320d3b4270d7dffcf` |
| `output/reproducibility-cache/gsdc2023/functions/imuprocessing.m` | `d942df6fe046841d69f6a30290e7878679359f5015b13c805c86e8aa5c91f7ae` |
| `output/reproducibility-cache/gsdc2023/functions/deviceimu2imu.m` | `9aff2984b0aa1709bbb1b397b48216ab46c213a1e3cbcfa99c0546a48a6df632` |
| `output/reproducibility-cache/gsdc2023/functions/add_position_offset.m` | `2308b612b6aab9816ddcf3c1efef621b8ebe15a1bdae5d167df2783e6c831797` |
| `output/reproducibility-cache/gsdc2023/functions/eul2rotm.m` | `0214826c3a1dd662330acb034ab25756ea772961f0063092c887e535013c3da8` |
| `output/reproducibility-cache/gsdc2023/parameters.m` | `518925e9c75c7a14fceb5cd99432fe883311e0d64c72b94396744ac765120f52` |
| `apps/native/gnss_fgo_imu_no_base.cpp` | `be643811cfaf82304f9434bc969796b2a14cd0f6e5a2c8b46b043af7b326c5c2` |
| `src/algorithms/fgo_gtsam_backend.cpp` | `781d65e34826ec08e04d1dfdac0ab10b2ed9409726078465f3020b57afac4baa` |
| `include/libgnss++/algorithms/upstream_position_offset.hpp` | `a37fc5d93f2f32cf9e9c6d684b38d9ce855db3acaca743d765d870530ebb2fce` |
| Phase101 structural result | `5ffb15c04fbdeac33d907f4ed3a1fdacb031508c6e3db8d7ef5b230dcab8d36a` |
| Phase105 attribution result | `d4a1f4c05c00a65abe38f900279c6bd3a9cc9c1e0c4d0eb571608929c0da381c` |
| Phase108 accuracy result | `9f9a2528745c06ba06e774139f56892bcea0ef208a3d064e4b70042e48f29a9d` |
| Phase109 structural result | `b443c2fcd1ed3b067094b47bf0b81f7fa404ed20547e7234d1c174dcf9ca2225` |
| Phase21 position-offset freeze/result | `aeff90b9ddda713ab4cce9f4564f0418c3f1d269bf9d0c32aea0e78d622c4928` / `d6c9fda6aa35f13b001bf48ae1a26d7673069948ce407f85af973a74f80c8df7` |

