# Smartphone R5 Phase123 Doppler/velocity factor parity audit

Status: read-only audit, 2026-09-03.  Decision: `NO-CANDIDATE`.

This audit compares the official `fgo_gnss.m`/`fgo_gnss_imu.m` source path with
the native Phase112/118 raw-only Doppler path.  Only source files and sealed
phase metadata were read.  No phone, base, navigation, truth, MAT, solution,
solver, accuracy, or Kaggle artifact was read or executed.

## Decision and boundary

No implementation or new freeze is authorized by this audit.  The source
comparison establishes several exact/equivalent contracts, but the remaining
differences are not an independently safe one-variable change:

* The official range-rate expression has an explicit first-order Sagnac term
  containing receiver position and receiver velocity.  The native Android
  target uses the already-frozen Earth-rotation-corrected satellite state and
  omits a second explicit Sagnac term.  Making the expression official-style
  would also require making the prepared LOS/state convention agree; that is a
  geometry/equation change, not a telemetry-only or weighting change.
* The official residual screen is absolute after subtracting the receiver
  clock-drift contribution, while the Phase112/118 direct-quality lane uses an
  epoch median-centered screen.  The native corrected branch also omits the
  first epoch because it has no preceding clock interval.  Replacing either
  behavior changes factor admission/counts and is outside the requested
  boundary.
* Wavelength/sign, receiver/satellite clock units, D SNR weighting, and the
  Highway Huber value are already source-equivalent for the target routes.
  There is consequently no remaining isolated source-backed weighting or
  kernel candidate.

The correct next boundary is to keep the Phase118 Doppler recipe unchanged and
perform a future audit only if a source-complete, count-preserving geometry
contract becomes available.  No truth score or parameter sweep is evidence for
such a change.

## Frozen comparison scope

The native scope is the Phase112/118 direct-quality, corrected-undifferenced
Doppler lane used by the Pixel5 target routes.  The application source sets
`use_undifferenced_doppler_factors` and
`use_corrected_undifferenced_doppler_factors` for Android raw input
(`apps/native/gnss_fgo_imu_no_base.cpp:7603-7608`).  The Phase118 recipe keeps
the direct quality screen and does not enable the alternate absolute screen
(`apps/native/gnss_fgo_imu_no_base.cpp:1336-1415`).  The route table assigns
MTV-A and LAX-T (Pixel5/Highway) D Huber `0.8`
(`apps/native/gnss_fgo_imu_no_base.cpp:378-413`); direct quality wiring is at
`apps/native/gnss_fgo_imu_no_base.cpp:7693-7713`.

Official source pins used for the audit:

* `fgo_gnss.m` SHA-256
  `5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3`.
* `fgo_gnss_imu.m` SHA-256
  `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3`.
* `gnsslog2obs.m` SHA-256
  `665ce43d1fc3c5a9c3b3a6fd76f81539126e3ab897fbba484ab0ccd18964acff`.
* `Gobs.m` SHA-256
  `be27fd1a075829a00f0aec5f32a0040a884077a8ac256a10a2bc26def255fc88`.
* `Gsat.m` SHA-256
  `a56c3236646606c1b7e771180e9bc18b08f809fe8a6d71b795366b96207871f6`.
* `exobs_residuals.m` SHA-256
  `50c954a825edbdeaf5c9884486aff56c058f51d47610a06722d4f42b33095324`.
* `obserrmodel.m` SHA-256
  `43d0671a25c81fa6d7df0e1fbe81bf48a14eb3a35c8399600200b44961a7f9ef`.
* `parameters.m` SHA-256
  `518925e9c75c7a14fceb5cd99432fe883311e0d64c72b94396744ac765120f52`.

Native source pins used for the audit:

* `src/algorithms/fgo_problems.cpp` SHA-256
  `5675e82fc595933da4e7c14ad468ae69437dc04bac79a7ef55f1edfa03e098da`.
* `include/libgnss++/algorithms/doppler_contract.hpp` SHA-256
  `66d97c9ca2cc474247464d8af782cc3ca1837195967e601576ee25b6f8e1f56d`.
* `include/libgnss++/algorithms/doppler_velocity_wls.hpp` SHA-256
  `ef888b3e6c2f15d0dd035c0905bafdee463e346c549f81cf892ec30c1c4dcde0`.
* `src/algorithms/fgo_gtsam_internal.hpp` SHA-256
  `7aba732064d2cd6b3858c60226b84aa904423becdf66cb386e628fae16fc5afd`.
* `src/algorithms/fgo_gtsam_backend.cpp` SHA-256
  `c0da12d7a1d5eeb9492b02ad80f0e9868cbda32e452bbdd9c3f90cc57b4db0fa`.
* `src/io/android_raw_gnss.cpp` SHA-256
  `a020ce900db6a9d3a994cc3adb3dcebbe84cbf67c1ff9793991b7a75039529c5`.
* `src/core/navigation.cpp` SHA-256
  `f876803d2e61fa62f370419f4ee4db16afb6b12b2d7ab00cd8b613af6f2649d3`.

The `navigation.cpp` hash is recorded only as a source identity from the
working tree; the relevant central-difference implementation is cited below.

## Equation comparison

### Official MATLAB path

`gnsslog2obs.m:158-184` forms the per-signal wavelength
`lambda = c/frequency` and stores RINEX Doppler as

```
D = -PseudorangeRateMetersPerSecond / lambda
```

`gnsslog2obs.m:118` converts receiver clock drift from ns/s to metres/s.
`Gobs.m:1151-1158` then forms the Doppler residual in metres/s:

```
resD = -D*lambda - (satellite_range_rate - satellite_clock_drift)
```

There is no ionosphere or troposphere term in this D residual
(`Gobs.m:1160-1165` only applies those corrections to P/L).  `Gsat.m:259-306`
computes range rate from satellite-minus-receiver velocity projected on the
LOS and adds its explicit Earth-rotation term.  The scripts pass a fixed LOS
and the initial receiver velocity to `DopplerFactor_VD` in
`fgo_gnss.m:123-151` and `fgo_gnss_imu.m:195-221`.

The official factor source `DopplerFactor_VD.h:15-56` is affine:

```
error = los * (v - initial_v) + d - prr
```

with a 3-vector velocity key and one scalar clock-drift key.

### Native Phase112/118 path

`android_raw_gnss.cpp:907-955` uses the raw carrier frequency to form
`wavelength = c/frequency` and the same signed RINEX D value from the Android
rate.  `doppler_contract.hpp:18-40` gives the corresponding D-to-range-rate
conversion and `:140-150` defines the receiver-only residual:

```
residual = measured_range_rate -
           (known_satellite_range_rate - satellite_clock_drift_mps)
```

`fgo_problems.cpp:719-843` prepares the native LOS and known satellite rate,
then stores this receiver-only residual.  The native GTSAM factor
`fgo_gtsam_internal.hpp:3620-3680` is:

```
error = los_nav * velocity_nav + clock_drift_mps - measured_mps
```

with the same 3-vector velocity plus one scalar D key.  Substituting the
receiver-only residual into the official factor shows the same affine receiver
prediction when the known satellite state and LOS convention are identical;
the native source-clock variant removes the explicit `initial_v` because it is
already included in the prepared residual.

## Term-by-term classification

| Term | Classification | Source evidence and consequence |
|---|---|---|
| Android Hz to RINEX D, wavelength, sign | **exact** | Official `gnsslog2obs.m:158-184`; native `android_raw_gnss.cpp:907-955` and `doppler_contract.hpp:18-40`. Both use `D=-rate/lambda`, then metres/s. |
| Receiver clock-drift unit | **equivalent** | Official `gnsslog2obs.m:118` is metres/s; native `EpochSeed.receiver_clock_drift_mps` is raw m/s (`fgo_problems.cpp:357-410`) and the GTSAM D key is scalar/vector one-dimensional. |
| Receiver clock-drift key/topology | **equivalent for retained rows; first-row admission differs** | Official has `d(i)` in every loop (`fgo_gnss.m:123-151`); native source-clock factor has one D key (`fgo_gtsam_internal.hpp:3620-3680`). Native corrected construction skips epoch 0 and starts after a clock interval (`fgo_problems.cpp:997-1015`). |
| Satellite clock drift unit/sign | **equivalent** | Official subtracts `ddts` in `Gobs.m:1151-1158`; native multiplies seconds/s by `c` and uses `measured-(sat_rate-drift*c)` (`fgo_problems.cpp:719-781`, `doppler_contract.hpp:140-150`). |
| Satellite motion / LOS affine factor | **equivalent algebraically** | Official `DopplerFactor_VD.h:15-56` and native `fgo_gtsam_internal.hpp:3620-3680` have the same receiver velocity and D coefficients after the initial-velocity term is absorbed into the residual. Prepared-state conventions are addressed separately below. |
| Earth rotation / Sagnac | **divergent** | Official `Gsat.m:259-306` adds the explicit first-order term involving satellite/receiver position and receiver velocity. Native corrected Android mode rotates satellite position and velocity in `doppler_contract.hpp:42-134` and does not add that explicit term. A full source-parity change would alter prepared LOS/known rate and the D equation together. |
| Satellite velocity interpolation | **unresolved** | Native `navigation.cpp:39-51` obtains velocity/rate by a central difference at +/-0.5 s. The available official `satposs.m` source documents transmission-time state but does not expose an equivalent velocity-difference implementation in the audited source. No claim of numerical identity is justified. |
| Atmosphere in D | **exact in scope** | Official `Gobs.m:1160-1165` applies atmospheric terms only to P/L; native D construction adds none. |
| Lever arm / angular velocity in D | **equivalent: absent** | Official D factor has only V/D keys (`DopplerFactor_VD.h:15-56`); native D factor has only velocity and D keys (`fgo_gtsam_internal.hpp:3620-3680`). IMU pose/lever-arm terms are separate factors, not part of the Doppler residual. |
| Signal frequency mapping | **equivalent** | Official uses per-signal frequency/wavelength (`gnsslog2obs.m:158-184`); native uses the observation carrier frequency and fail-closed supported-band checks (`android_raw_gnss.cpp:907-955`, `fgo_problems.cpp:429-475`). |
| D SNR sigma | **exact for the target lane** | Official `obserrmodel.m:11-30` and `parameters.m:22-44` use SNR percentile/20 and D ratio `1/12`, without the P/L signal factor. Native `observable_upstream_preprocessing.hpp:186-199` uses the same D scale/12 and the direct lane pins p85=85. |
| D SNR/elevation admission | **equivalent at the basic threshold** | Official D masking is SNR/multipath plus elevation (`exobs.m:23-31`, `exobs_residuals.m:14-24`); native raw status, SNR >=20, supported band, and elevation checks are in `android_raw_gnss.cpp:874-905` and `fgo_problems.cpp:429-515,785-843`. Native also has explicit health/eligibility checks. |
| D robust kernel and application point | **exact for MTV-A/LAX-T** | Official Highway D kernel is `.8` (`parameters.m:85-108`); route table is `.8` (`gnss_fgo_imu_no_base.cpp:378-413`). Native GTSAM wraps the scalar factor with Huber at the configured threshold (`fgo_gtsam_backend.cpp:1128-1164`, `fgo_gtsam_internal.hpp:3733-3742`). |
| Residual outlier screen | **divergent** | Official `exobs_residuals.m:18-24` screens `resD - dclk/dt` against the absolute threshold. Phase112/118 direct quality uses the epoch median-centered residual screen (`fgo_problems.cpp:869-941`). Switching screens changes the retained factor set. |
| Same-satellite/signal identity | **equivalent key identity** | Official loops the satellite/signal matrix; native requires exact `(SatelliteId, SignalType)` identity (`fgo_problems.cpp:429-475`). There is no ambiguity/DD pairing in either ordinary D factor. |
| First/adjacent epoch timing | **equivalent for CCDD interval; admission divergent** | Official clock factors use adjacent `dtgps` (`fgo_gnss.m:154-175`, `fgo_gnss_imu.m:252-278`). Native uses adjacent epoch interval for the corrected D/C handoff but intentionally has no epoch-0 D factor (`fgo_problems.cpp:997-1015`). |
| Factor linearization | **equivalent receiver model; geometry representation divergent** | Both are fixed-LOS affine V/D factors. Native corrected satellite rotation and official explicit Sagnac can produce different fixed LOS/known terms, so they cannot be declared byte-identical. |

## Candidate assessment

Three source-backed ideas were considered; none satisfies the required
factor-count/admission and fixed-configuration boundary.

1. **Official absolute D residual screen.**  This is source-backed, but it
   changes the gate center from the native per-epoch median and therefore
   changes factor admission/counts.  It is rejected.
2. **Restore the official first-order Sagnac/receiver-velocity term.**  The
   difference is source-backed, but the native target's corrected branch also
   rotates the satellite state and uses its LOS.  A coherent port changes both
   the known rate and geometry linearization; changing only one is not source
   parity.  It is rejected as an unsafe equation/geometry change, not frozen.
3. **Admit the official epoch-0 D factor.**  This would preserve equations but
   explicitly changes factor count and clock observability topology.  It is
   rejected by the admission-invariance requirement.

The already-matching D sigma/Huber behavior is not a candidate: re-freezing it
would be a no-op.  No candidate is therefore frozen in Phase123.

## Read accounting and reproducibility

| Class | Reads | Execution |
|---|---:|---:|
| Official/native source | source-only | 0 solver/raw runs |
| Phone GNSS/IMU/nav payload | 0 | 0 |
| Raw base/provenance bytes | 0 | 0 |
| Truth/MAT/coordinate rows | 0 | 0 |
| Sealed solution rows | 0 | 0 |
| Accuracy/Kaggle | 0 | 0 |

This document is an audit record only.  It authorizes no raw execution,
accuracy evaluation, implementation, tuning, or rerun.
