# Smartphone R5 Phase124 official IMU-factor parity audit

Status: read-only audit, 2026-09-03.  Decision: `NO-CANDIDATE`.

This record compares the official `fgo_gnss_imu.m` IMU path and the native
Phase112/118 Pixel5 IMU path.  The comparison is source-only, with sealed
phase metadata used only to identify the already-frozen recipe.  No phone
GNSS/IMU/navigation payload, raw base, truth, solution rows, coordinate rows,
MAT data, solver, accuracy evaluator, or Kaggle artifact was read or run.

## Decision and boundary

No implementation candidate is frozen.  The values that can be compared
without ambiguity are already equivalent: Pixel5 white-noise densities after
the official synchronization coefficient, bias random-walk values,
integration sigma, gravity, zero Coriolis behavior, and zero lever arm.  The
remaining source differences are coupled to factor topology, state priors,
sample admission, or interval integration.  Correcting any one of those
would violate the Phase124 boundary that factor topology/admission and all
other graph, filter, sigma, and solver settings remain unchanged.

The Phase112/118 target is the Android raw path with GTSAM Pose3+velocity+IMU,
the frozen C7/D/clock recipe, raw-base and Pixel5 output boundaries, and no
upstream-stop candidate switch.  In the application source, Android input
enables corrected undifferenced Doppler (`gnss_fgo_imu_no_base.cpp:7603-7608`),
while Phase118 validation rejects `native_upstream_stop_constraints`
(`gnss_fgo_imu_no_base.cpp:1407-1414`).

## Source pins

The official source pins used here are:

* `fgo_gnss_imu.m` — SHA-256
  `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3`
  (`:40-60, :129-187, :252-299, :325-365`).
* `functions/imuprocessing.m` — SHA-256
  `d942df6fe046841d69f6a30290e7878679359f5015b13c805c86e8aa5c91f7ae`
  (`:5-40, :42-58`).
* `functions/deviceimu2imu.m` — SHA-256
  `9aff2984b0aa1709bbb1b397b48216ab46c213a1e3cbcfa99c0546a48a6df632`
  (`:5-28`).
* `functions/vel2rpy.m` — SHA-256
  `1fc09b66f3e6548174e0eb6777d42becd0fa131b821013864e4fbfa9ff6e6a31`
  (`:5-14`).
* `parameters.m` — SHA-256
  `518925e9c75c7a14fceb5cd99432fe883311e0d64c72b94396744ac765120f52`
  (`:46-51, :176-203`).

The native source pins are:

* `apps/native/gnss_fgo_imu_no_base.cpp` — SHA-256
  `df77ab6556f90d6d9a8464d360e85d2a49a50d6fc04ef396833f354f821a1176`
  (`:1787-1798, :4341-4552, :7603-7608, :7780-7825, :8540-8674`).
* `src/io/imu.cpp` — SHA-256
  `79efa99a4b24cc7895fb8756a59967604c1048a1a297d2662f00cb23b239809a`
  (`:729-785, :850-958, :1001-1141`).
* `include/libgnss++/io/imu.hpp` — SHA-256
  `154675958cc37c865dd76165ad92d922689c81670206efaf9584ebbb7bd87d8c`
  (`:21-30, :63-85, :202-228`).
* `include/libgnss++/algorithms/fgo.hpp` — SHA-256
  `b2224fd4cbe6958eed491ae0551485c4cdb86ef8cd5c5a03f5572f1c41400b86`
  (`:440-491`).
* `src/algorithms/fgo_gtsam_backend.cpp` — SHA-256
  `c0da12d7a1d5eeb9492b02ad80f0e9868cbda32e452bbdd9c3f90cc57b4db0fa`
  (`:624-809`).
* `src/fusion/fusion_initialization.cpp` — SHA-256
  `c1f7d6bb00179afd471f4287c296da0c8718e23b146ea9518d8a002f70dfd6aa`
  (`:13-48, :61-165`).

The audited local GTSAM source pins are:

* `ImuFactor.h` — `d26230ccc4ad1784d26c5145e49ca68e34aea602a611d106135ad09954f04156`;
  `ImuFactor.cpp` — `6500ec260c4b1157a4c0b76857fbe284d5684dabe4864d8ed443813b368bb66`.
* `CombinedImuFactor.h` — `be3fe486859127aeb3503a928cda99366b9ac21929b6e11428f1c33ea266be70`;
  `CombinedImuFactor.cpp` — `780a612c661ca6a445178e52b5484dee28c1719ef11747a201ffca93433ddd32`.
* `PreintegrationParams.h` — `f629799dd7ffde1be2e1b41f9d001c2e51978ce858b6178a88c76b6d6006c409`;
  `PreintegrationCombinedParams.h` — `754532ff66c6d2105322374f8288b8ddc0c2601ecb83eb84a6300c5103604f7e`.
* `PreintegratedRotation.h` — `97571b5d0649e7a1b4e66492510b77fb7def7ee0601ab9f1cf06371c7b9d9f20`;
  `PreintegratedRotation.cpp` — `24c8e5e3adf8e96837246356eef4f4b4a9ad0580f9fae7a269d7fe02c1c7bdc7`.
* `PreintegrationBase.h` — `508d2ddcc9805df837452b5178edb444731d083c9854a8eb804c861b4ea48c1c`;
  `PreintegrationBase.cpp` — `713f0c8490f2edebe3fbf9e1fa7a82539ebcc6144b6b80d8dbd4ba2b462ffd0d`.

The source tree identity is the previously sealed official cache commit
`29923f9f370f09ebc00f96d8cca375007a18e7d5`.  The prior native boundary is
Phase118 source freeze `5fcc06ab64189dd5dfb8001664bdb8496f85224f`, structural
contract `25f3bada02c4073175054b5d668de02020761208`, and the immediately
preceding read-only Phase123 freeze `e1642294c53176326fd9d277cd5fa6172b49857d`.

## Term-by-term comparison

| Term | Classification | Evidence and consequence |
|---|---|---|
| Accelerometer white-noise density | **exact for Pixel5 effective value** | Official Pixel `AccSigma=0.05` and `imu_sync_coefficient=0.5` (`parameters.m:176-197`), then squares `(0.5*AccSigma)` (`fgo_gnss_imu.m:136-142`): `0.025 m/s²/√Hz`. Native sets `0.025` and squares it (`gnss_fgo_imu_no_base.cpp:4538-4545`, `fgo_gtsam_backend.cpp:629-635`). |
| Gyroscope white-noise density | **exact for Pixel5 effective value** | Official `0.001 rad/s/√Hz` times `0.5`; native pins `0.0005 rad/s/√Hz` and squares it. The values and units are the same. |
| Accelerometer/gyro bias random walk | **numeric value equivalent; model realization divergent** | Official uses `0.00025` and `0.0000005` in a separate `BetweenFactorConstantBias` scaled by `sqrt(numel(IMUindices))` (`fgo_gnss_imu.m:154-155, :296-299`). Native places the same squared values in `PreintegrationCombinedParams` (`fgo_gtsam_backend.cpp:624-636`). The continuous-time units in the two source comments are algebraically equivalent (`m*sqrt(Hz)/s² = m/s³/sqrt(Hz)` and `rad*sqrt(Hz)/s = rad/s²/sqrt(Hz)`), but the factor model is not. |
| Integration uncertainty | **parameter value exact; full covariance divergent** | Both use `0.05² I` (`fgo_gnss_imu.m:136-142`, `fgo_gtsam_backend.cpp:629-636`). Official ordinary PIM is 9-state; native combined PIM is 15-state and carries bias correlation, so the resulting factor covariance is different. |
| Gravity | **exact** | Official constructs `PreintegrationParams([0;0;-prm.g])` with `g=9.80665` (`fgo_gnss_imu.m:136-139`, `parameters.m:176-178`). Native `MakeSharedU(9.80665)` has the same `(0,0,-g)` vector (`fgo_gtsam_backend.cpp:628-630`; local `PreintegrationParams.h:41-58`). |
| Coriolis | **equivalent behavior, representation differs** | Official explicitly sets `omegaCoriolis=[0;0;0]` (`fgo_gnss_imu.m:137-143`). Native does not set the optional value; local GTSAM returns zero when it is absent (`PreintegratedRotation.cpp:139-142`), and second-order Coriolis defaults false (`PreintegrationParams.h:25-48`). No numerical correction is present in either path. |
| Sampling clock and accel/gyro synchronization | **divergent** | Official maps each stream from GNSS elapsed clock with linear extrapolation and synchronizes to the gyro grid with `interp1(...,"linear","extrap")` (`imuprocessing.m:5-40`). Native requires validated elapsed anchors (or a separately validated UTC fallback), pairs each gyro to a nearest/interior-linear accelerometer sample, and omits rows beyond a fixed `0.025 s` bound (`imu.cpp:729-785, :1001-1054`). This changes input samples and is not a noise-only change. |
| Integration `dt` and epoch endpoint | **divergent** | Official uses `acc.dt=diff(...); [acc.dt;acc.dt(end)]` and an inclusive `>= obs.utcms(i) & <= obs.utcms(i+1)` selection (`imuprocessing.m:37-40`, `fgo_gnss_imu.m:280-285`). Native integrates samples in `[t0,t1)`, rejects nonpositive `dt`, and adds a final held-sample tail to `t1` (`fgo_gtsam_backend.cpp:638-680`). |
| Body/sensor frame and axis rotation | **physically equivalent for the zero lever arm; implementation placement differs** | Official passes raw vectors to GTSAM and sets `body_P_sensor` to `RzRyRx([-85,178,-94])` (`fgo_gnss_imu.m:138-143`); GTSAM rotates sensor vectors in `PreintegrationBase.cpp:59-80`. Native applies the exact same `Rz(-94)*Ry(178)*Rx(-85)` before storing body-FLU samples (`gnss_fgo_imu_no_base.cpp:1787-1798, :4378-4391`). Bias correction occurs in different frames because the bias models themselves differ; this is covered by the bias row above. |
| Lever arm | **exactly absent in the target recipe** | Official `mountingPosition=[0;0;0]` (`parameters.m:202-203`), and native sets `pose3_lever_arm_body_m=Zero()` (`gnss_fgo_imu_no_base.cpp:7825`). |
| Initial attitude | **divergent** | Official initializes each `Pose3` from `vel2rpy` (zero roll/pitch and velocity-derived heading) (`fgo_gnss_imu.m:40-49, :145-149`; `vel2rpy.m:5-14`). Native static-aligns gravity and gyro bias, aligns heading, then dead-reckons each attitude seed through preintegrated `deltaRij()` (`fusion_initialization.cpp:13-48`; `gnss_fgo_imu_no_base.cpp:4433-4453, :683-689`). |
| Initial bias and bias prior | **divergent** | Official inserts zero bias at every epoch and an infinite-sigma prior (`fgo_gnss_imu.m:151-152, :169-187`). Native uses the static-window estimated accel/gyro bias and finite first-state priors `0.1` and `0.01` (`gnss_fgo_imu_no_base.cpp:4433-4437, :4536-4550`; `fgo_gtsam_backend.cpp:773-791`). Matching official values would change gauge/conditioning and is not an isolated factor-preserving change. |
| IMU factor topology | **divergent** | Official adds a 5-way `ImuFactor` keyed by pose/velocity at both ends plus one bias (`fgo_gnss_imu.m:280-294`), then a separate bias-between factor (`:296-299`). Native adds a 6-way `CombinedImuFactor` keyed by both endpoint biases (`fgo_gtsam_backend.cpp:793-809`). GTSAM documents that Combined carries bias random walk, a 15x15 covariance, and measurement/bias correlation, whereas ImuFactor is 5-way with 9x9 preintegrated covariance (`ImuFactor.h:72-80, :161-174`; `CombinedImuFactor.h:66-78, :187-207`). |
| IMU covariance propagation | **divergent as a factor model** | Official `ImuFactor.cpp:53-84` propagates only the 9-state preintegrated covariance and its constructor uses that covariance (`:115-118`). Native `CombinedImuFactor.cpp:106-197` propagates bias-init covariance, cross terms, and bias random walk before constructing a Gaussian 15-residual factor (`:202-207, :234-295`). |
| IMU factor robust loss/noise units | **Gaussian IMU factor equivalent; stop path divergent** | Both ordinary IMU factors use Gaussian preintegration covariance; no Huber wrapper is attached to the ordinary IMU factor. Official final-mode stop velocity/pose constraints are robust (`fgo_gnss_imu.m:160-163, :223-227, :287-290`). The native stop detector/factors are a separate opt-in and Phase118 validation excludes that option (`fgo_gtsam_backend.cpp:324-359, :1525-1591`; app `:1407-1414`). |
| IMU factor admission on long GNSS gaps | **divergent** | Official only adds motion/clock/IMU factors when `dtgps < 1.5`, asserts that every interval has at least one IMU index, and always adds the separate bias-between factor (`fgo_gnss_imu.m:263-299`; `parameters.m:46-51`). Native attempts each adjacent interval, marks it valid only when a sample was integrated, and otherwise adds loose velocity/bias continuity factors (`fgo_gtsam_backend.cpp:653-680, :793-809`); the Android candidate wrapper later rejects zero-interval output rather than silently selecting a fallback (`gnss_fgo_imu_no_base.cpp:8565-8585, :8646-8673`). |
| Raw IMU admission and nonfinite handling | **divergent** | Official uses interpolation/extrapolation and an assertion. Native rejects `.mat`, nonfinite samples, mixed timestamp domains, invalid anchors, and out-of-bound accel/gyro pairs fail-closed (`imu.cpp:738-785, :850-958, :1024-1058`). This is a raw-input/admission contract, not an isolated factor parameter. |

## Candidate assessment

Three source-backed candidates were considered and all were rejected:

1. **Replace native `CombinedImuFactor` with official `ImuFactor` plus a
   separate bias-between factor.**  This is the clearest source parity
   candidate, but changes the factor arity (6-way to 5-way plus a new factor),
   covariance dimension (15 to 9), bias random-walk placement, and bias
   linearization.  It violates topology and covariance invariance.
2. **Use the official gyro-grid interpolation/extrapolation and inclusive
   interval policy.**  This changes raw sample values, endpoint handling,
   per-interval measurement count, and the gap admission predicate.  It is
   explicitly outside the fixed-admission boundary.
3. **Adopt official zero bias and infinite bias priors.**  This changes the
   initial state, bias gauge, and first-state prior rather than one IMU
   equation.  It also removes the native static-leveling bias initialization.

Explicitly setting native Coriolis to a zero vector would be only a
representation-level no-op: the current optional-absent path already returns
zero.  It is not a meaningful candidate and is not frozen.

The official/native noise, gravity, mounting, and ordinary Gaussian behavior
therefore provide no remaining source-backed, topology/admission-preserving
single-variable correction.  The correct Phase124 outcome is
`NO-CANDIDATE`; no raw run, solver run, truth evaluation, accuracy calculation,
implementation, tuning, or rerun is authorized.

## Read accounting

| Class | Reads | Execution |
|---|---:|---:|
| Official/native source text | read-only | 0 solver/raw runs |
| Sealed phase metadata | metadata-only | 0 |
| Phone GNSS/IMU/navigation payload | 0 | 0 |
| Raw base bytes/provenance payload | 0 | 0 |
| Truth/solution/coordinate rows | 0 | 0 |
| MAT files or MAT data | 0 | 0 |
| Precomputed coordinates/PDC | 0 | 0 |
| Accuracy evaluator | 0 | 0 |
| Kaggle/token access | 0 | 0 |
| Rerun/fallback/repair/parameter sweep | 0 | 0 |

This document is an audit record only.  It authorizes no code change or
execution action.
