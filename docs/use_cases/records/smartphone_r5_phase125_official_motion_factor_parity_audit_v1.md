# Smartphone R5 Phase125 official MotionFactor_XXVV parity audit

Status: read-only audit, 2026-09-04.  Execution label: `Luna Max`.
Decision: `NO-CANDIDATE`.

This record compares the official `MotionFactor_XXVV` graph block with the
native Phase118 Pixel5 Android raw GTSAM Pose3+IMU graph.  Only pinned source
text and sealed phase metadata were read.  No phone GNSS/IMU/navigation
payload, raw base, truth, solution row, coordinate, MAT, PDC, precomputed
coordinate, solver, accuracy evaluator, or Kaggle artifact was read or run.

## Decision and fixed boundary

The next un-audited coherent dimension is the official position/velocity
motion factor.  The audit is intentionally limited to that factor and its
own state/topology/admission contract.  Clock-factor, stop/height,
pose-point, initialization, IMU, filter, LM, output, raw-base, truth, and
Kaggle behavior remain inherited from the sealed Phase124/Phase118 boundary;
the older ClockFactor_CCDD audit is not reopened here.

No implementation candidate is frozen.  The official factor is not a scalar
or representation-only correction to the Phase118 native graph:

* Official `fgo_gnss_imu.m` has separate `p` Pose3, `x` 3-vector position,
  and `v` 3-vector velocity states.  It adds an exact `Pose3Point3Factor_PX`
  for every epoch, then adds `MotionFactor_XXVV(x_i,x_{i+1},v_i,v_{i+1},dt)`
  only when the measured epoch interval is strictly below `1.5 s`.
* `MotionFactor_XXVV` has three residuals and the midpoint equation
  `x2-x1-(v1+v2)*dt/2`, with Jacobians `[-I,+I,-dt/2 I,-dt/2 I]`.
  For both target Pixel5 routes the source setting is `Highway`, so
  `parameters.m` gives `sigma_motion = 0.01 m`.
* The Phase118 native target selects GTSAM, Pose3, and IMU.  In that path
  `positionKey(i)` is one `Pose3` body-in-nav state; there is no separate
  `x` vector or PX coupling factor.  Consecutive Pose3 states are linked by
  `CombinedImuFactor` when the interval has integrated IMU samples.
* The native position-motion branch instead adds a six-residual identity
  `BetweenFactor<Pose3>` for every adjacent index, with a loose rotation
  sigma of `10 rad`, translation `motion_sigma_m = 50 m`, and no measured
  `dt` or `dt < 1.5 s` admission gate.  The separate Eigen backend has an
  optional velocity-row implementation, but it is not the Phase118 GTSAM
  target and is disabled by the target's staging configuration.

Replacing the native Pose3 random walk with the official factor would require
new per-epoch `x` values, PX factors and their priors, or a new custom factor
that couples Pose3 translation to both endpoint velocities.  Either choice
changes state topology, residual dimension, linearization, covariance, and
factor incidence.  Adding the official factor in parallel changes the
objective and information.  Changing only `motion_sigma_m` cannot turn a
zero-motion Pose3 factor into the midpoint X/V equation, and adopting the
official interval gate changes admission and counts.  These are not
topology-, equation-, and admission-preserving one-variable changes.

The correct Phase125 outcome is therefore `NO-CANDIDATE`: keep the sealed
Phase118 native motion graph unchanged and authorize no code, parameter,
raw, solver, truth, accuracy, rerun, tuning, publication, or Kaggle action.

## Official source contract

The official source tree is pinned to commit
`29923f9f370f09ebc00f96d8cca375007a18e7d5`.

`fgo_gnss_imu.m:169-200` inserts `p`, `x`, `v`, `c`, `d`, and `b` at every
epoch, gives them infinite priors, and adds the zero-noise `p`-to-`x`
coupling factor.  Its adjacent block at `:252-278` computes

```text
dtgps = (obs.utcms(i+1)-obs.utcms(i))/1000
```

and, when `dtgps < prm.time_diff_th`, adds
`MotionFactor_XXVV(x_i,x_{i+1},v_i,v_{i+1},dtgps,noise_motion)`.  The same
predicate also controls the neighboring official clock factor, but clock
semantics are outside this phase.  `parameters.m:46-47` pins
`time_diff_th = 1.5 s`; `:134-142` pins `sigma_motion = 0.01 m` for
non-Street routes (with only `mi8`/`xiaomimi8` overridden to `0.1 m`).

The official factor source is explicit:

```text
e = (x2-x1) - (v1+v2)*dt/2
Hx1 = -I3, Hx2 = +I3, Hv1 = -dt/2 I3, Hv2 = -dt/2 I3.
```

This is a 3D affine kinematic constraint, not a Pose3 identity random walk.

## Native Phase118 contract

The entry point sets `backend=GTSAM`, `max_iterations=12`,
`use_pose3_state=true`, and `use_imu=true` at
`apps/native/gnss_fgo_imu_no_base.cpp:7588-7608`.  With these selectors,
`fgo_gtsam_backend.cpp:262-270` documents that the pose is body-in-nav and
that velocity and bias states are linked through `CombinedImuFactor`.
The IMU graph inserts those factors at `:793-809`; this is already frozen by
Phase124 and is not replaced by this audit.

The key/type contract is in `fgo_gtsam_backend.cpp:691-705` and
`fgo_gtsam_internal.hpp:948-975`: `positionKey(i)` is the Pose3 state,
`velocityKey(i)` is a Vector3, and `biasKey(i)` is a ConstantBias.  There is
no independent Vector3 position key space in this path.  The configured
Phase118 Pixel5 lever arm is zero (`gnss_fgo_imu_no_base.cpp:7825`), but a
zero lever arm does not make a Pose3 key equivalent to the official `x`
state: the official PX factor and its separate state/prior are still absent.

The native motion branch at `fgo_gtsam_backend.cpp:1374-1400` adds, when
`use_motion_factors && use_position_motion_factors`, one identity
`BetweenFactor<Pose3>` per adjacent index.  Its residual has six Pose3
components, the rotational sigmas are `10.0`, and each translation sigma is
`config.motion_sigma_m`.  `fgo_config.hpp:84-87,393-396` pins the target
defaults (`use_motion_factors=true`, `use_position_motion_factors=true`,
`motion_sigma_m=50.0`, `velocity_motion_sigma_m=0.01`).  The Pose3 branch
does not consume the interval `dt`; it loops all `i=1..num_epochs-1`.

The optional Eigen implementation at `src/algorithms/fgo.cpp:1241-1325`
does contain the same midpoint expression when
`use_velocity_motion_factors` is enabled, but this is a separate backend and
state layout.  `fgo_config.hpp:351-352` defaults that selector off, and the
GNSS-first staging setup explicitly sets it false at
`gnss_fgo_imu_no_base.cpp:8227-8229`.  It cannot be used to claim parity for
the Phase118 Pose3+IMU graph.

## Term-by-term classification

| Term | Classification | Source consequence |
|---|---|---|
| Official equation | divergent | Official is `x2-x1-(v1+v2)dt/2`; the target GTSAM path has no X state and uses an identity Pose3 between. |
| Residual dimension | divergent | Official motion has 3 residuals; native Pose3 `BetweenFactor` has 6 (rotation plus translation). |
| State keys | divergent | Official consumes X1, X2, V1, V2; native consumes Pose3 position keys only for the position-motion row, while V keys are consumed by CombinedImuFactor. |
| Pose-to-position coupling | missing in native target | Official inserts `Pose3Point3Factor_PX` with separate X state at every epoch; native has no PX factor in the IMU path. |
| Interval | divergent | Official passes measured `dtgps`; native position motion uses index adjacency and no dt. |
| Edge admission | divergent | Official motion is present only for `dtgps < 1.5 s`; native inserts every adjacent position pair when enabled. |
| Motion sigma | divergent | Target Pixel5/Highway official sigma is `0.01 m`; native translation sigma is `50 m`, but changing it alone cannot change the equation or topology. |
| Robust loss | not a candidate | Official motion noise is the declared Gaussian `noise_motion`; native BetweenFactor uses its configured Gaussian noise. The factor shape remains divergent. |
| Clock neighbor | out of scope | ClockFactor_CCDD and phone/jump handling are retained from the previously audited clock boundary; this phase does not combine or re-audit them. |
| Stop/height/priors/LM | out of scope | These neighboring official blocks are not changed or used to manufacture a motion candidate. |

## Candidate assessment

Three source-backed ideas were checked; none satisfies the fixed Phase118
boundary:

1. **Replace the native Pose3 random walk with `MotionFactor_XXVV`.**
   Rejected: it requires independent X states (or a new Pose3/X/V custom
   equation), changes residual dimension and factor keys, removes the
   current Pose3 regularizer, and introduces the official dt gate.
2. **Add the official motion factor alongside the native Pose3 and IMU
   factors.**  Rejected: it requires PX/state materialization and adds
   independent information, changing the objective, factor incidence, and
   covariance while retaining all other settings.
3. **Change only `motion_sigma_m` from `50 m` to the official Pixel5
   `0.01 m`, or enable the optional Eigen velocity rows.**  Rejected: the
   native GTSAM factor remains a zero-motion six-dimensional Pose3 factor;
   the Eigen rows belong to a different backend/state layout and are disabled
   in the target.  Neither is official motion parity.

No candidate is frozen and no source-level numeric change is authorized.

## Source pins

### Official

| Source | SHA-256 | Relevant lines |
|---|---|---|
| `output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m` | `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3` | `102-114,169-200,252-299` |
| `output/reproducibility-cache/gsdc2023/fgo_gnss.m` | `5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3` | `89-101,103-178` |
| `output/reproducibility-cache/gsdc2023/parameters.m` | `518925e9c75c7a14fceb5cd99432fe883311e0d64c72b94396744ac765120f52` | `46-51,130-142` |
| `output/reproducibility-cache/gtsam_gnss/src/MotionFactor_XXVV.h` | `0ca60daded389edfc2619350ca7fae89ccc8c3186624db1a96aa3ef5e3b1858f` | `16-62` |
| `output/reproducibility-cache/gtsam_gnss/src/Pose3Point3Factor_PX.h` | `3893b16fe226fc253eafda16d634c0e229cb593373113f4de63cc66f47a1cb13` | `17-47` |

### Native

| Source | SHA-256 | Relevant lines |
|---|---|---|
| `apps/native/gnss_fgo_imu_no_base.cpp` | `df77ab6556f90d6d9a8464d360e85d2a49a50d6fc04ef396833f354f821a1176` | `369-413,7588-7608,7807-7825,8227-8229` |
| `include/libgnss++/algorithms/fgo_config.hpp` | `e864b31ab04dc0f8f0f4764b4e89406282349012283944ed64e88c01ee8163e6` | `84-87,351-352,393-396` |
| `src/algorithms/fgo_gtsam_backend.cpp` | `c0da12d7a1d5eeb9492b02ad80f0e9868cbda32e452bbdd9c3f90cc57b4db0fa` | `262-270,691-809,1374-1400,1520-1523` |
| `src/algorithms/fgo_gtsam_internal.hpp` | `7aba732064d2cd6b3858c60226b84aa904423becdf66cb386e628fae16fc5afd` | `948-975` |
| `src/algorithms/fgo.cpp` | `145ffa3958d3a867ba53d962f07d346f9efa36e72140ad03d7a404ffdd3b5023` | `387-415,1241-1325` |

## Sealed baselines and read accounting

The native baseline is Phase124 freeze commit
`59586b424341ec0e36c52c3b28a349a473b837f2`, whose audit is commit
`b0a0c161ef39791321429464d2dc6ac6f5968da2`.  The inherited Phase118 source
freeze is `5fcc06ab64189dd5dfb8001664bdb8496f85224f`, with structural
contract `25f3bada02c4073175054b5d668de02020761208`.  The source-only clock
dimension was separately recorded in Phase83 and is not combined here.

| Class | Reads / execution |
|---|---:|
| Official/native source text | read-only |
| Sealed phase metadata | metadata-only |
| Phone GNSS/IMU/navigation payload | 0 |
| Raw base payload/provenance bytes | 0 |
| Truth, solution, or coordinate rows | 0 |
| MAT/PDC/precomputed coordinates | 0 |
| Native solver or accuracy evaluator | 0 |
| Reruns, fallbacks, repairs, or parameter sweeps | 0 |
| Kaggle/token access | 0 |

This document is an audit record only.  It authorizes no code change, raw
execution, solver invocation, truth read, accuracy calculation, publication,
or Kaggle action.  Any future motion implementation would require a fresh
source-complete topology/admission contract and a separate authorization.
