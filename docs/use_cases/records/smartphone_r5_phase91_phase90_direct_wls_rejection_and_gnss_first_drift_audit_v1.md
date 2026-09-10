# Smartphone R5 Phase91: direct-WLS rejection and GNSS-first drift audit

## Decision

The Phase90 candidate
`phase90_source_clock_c0d_raw_drift_direct_wls_handoff` is an **unimplemented
NO-GO**. Its freeze permits only a same-run raw Android D initializer plus the
existing direct Doppler WLS velocity handoff, and explicitly preserves the
existing WLS equations, physical gates, and all-epoch coverage gate. The sealed
Phase40 MTV-h result already proves that this unchanged direct-WLS sequence has
zero valid coverage. A D initializer in the later main graph cannot change
those earlier WLS estimates. Therefore the Phase90 candidate cannot satisfy
its frozen direct-WLS coverage contract on all routes without a forbidden
algorithm or gate change.

The supported next candidate is different: preserve the standard same-run,
in-memory GNSS-first native FGO position/clock/velocity staging used by the
Phase80/85 path, and change only the main Pose3+IMU graph's initial D state to
the same retained raw Android `receiver_clock_drift_mps` value. This is a
source-aligned implementation hypothesis, not a route or accuracy result.
Implementation is authorized by the separate Phase91 freeze; raw execution is
not authorized by either this audit or that freeze.

No raw Android, broadcast-navigation, base, truth, MAT, Kaggle, token, or
precomputed-coordinate artifact was read or rerun for Phase91.

## Formal rejection of the Phase90 direct-WLS candidate

The Phase90 freeze is
`docs/use_cases/records/smartphone_r5_phase90_source_clock_c0d_raw_drift_direct_wls_handoff_freeze_v1.json`
(SHA-256
`c6543b28d069c5a2d3a3c270991545c3d1cf8ced234caa4e9d8e0ab1a477fcea`). Its
allowed change is limited to a raw-D initializer and the existing
`FGOProblem::doppler_velocity_wls_estimates` velocity-only handoff. It requires
direct-WLS physical, rank, residual, and coverage gates to remain unchanged.

The sealed Phase40 record
`docs/use_cases/records/smartphone_r5_phase40_direct_doppler_wls_handoff_result_v1.json`
(SHA-256
`ea1d92b2f69bf6810260e17d05ae4f3c0276e1ab7be6cec84a1f35b0fd5607b1`) reports
for `2021-08-24-20-32-us-ca-mtv-h/pixel5`:

| quantity | sealed value |
|---|---:|
| target epochs | 1,325 |
| corrected undifferenced-Doppler rows | 4,724 |
| epochs with at least four rows | 1,181 |
| insufficient-row epochs | 144 |
| direct valid estimates | 0 |
| propagated valid estimates | 0 |
| rejected estimates | 1,325 |
| nonfinite estimates | 0 |
| velocity-bound failures (`>70 m/s`) | 1,181 |
| clock-rate-bound failures (`>2000 m/s`) | 577 |
| maximum solved velocity norm | 8,919.7537472980548 m/s |
| maximum solved clock-rate norm | 7,316.9109537785716 m/s |
| first solved state | 4 rows, `physical-gate` |
| first solved velocity / clock-rate | 8,919.7537472980548 / 7,314.3850372387406 m/s |

The sealed coverage gate is `direct + propagated == all epochs` and
`rejected == 0`; it is false as `0 + 0 != 1325` and `1325 != 0`.

The algorithmic ordering makes the rejection conclusive. The problem builder
constructs corrected raw Doppler rows and calls WLS before the main graph
(`src/algorithms/fgo_problems.cpp:1106-1250`). The unchanged solver uses the
four-column `[los,1]` system, minimum four rows, rank/condition and residual
checks, then the 70 m/s velocity and 2,000 m/s clock-rate gates
(`include/libgnss++/algorithms/doppler_velocity_wls.hpp:17-46,89-275`). The
Phase90 D initializer is downstream of this calculation and cannot alter its
MTV-h row geometry, solved state, or rejection count. Thus the Phase40 result
is a proof of impossibility under the Phase90 contract, not merely a
low-confidence warning.

The Phase90 candidate was not implemented and no route was run after its
freeze. The exact failed requirement is the frozen direct-WLS all-epoch
coverage gate; no implementation, source, sigma, physical-bound, or route
change is authorized to make it pass.

## Evidence for the standard Phase80/85 GNSS-first handoff

The sealed Phase85-v2 aggregate
`output/smartphone-r5/phase85-source-clock-c0d-structural-v2/phase85_source_clock_c0d_structural_result.json`
(SHA-256
`2ac9919e57bb18e52ccb81bc73471af017af77fe3dad65d5750f919d5b1cc4a6`) is
truth-free and records the standard path used without either
`--native-gnss-first-velocity-only-handoff` or
`--native-direct-doppler-wls-handoff`. Its candidate run 1 values (run 2 is
byte-identical) are:

| route | GNSS-first epochs / velocity states | GNSS-first iterations | initial → final cost |
|---|---:|---:|---:|
| MTV-a / pixel5 | 2,159 / 2,159 | 192 | 10,699,626.599385893 → 19,418.316034639287 |
| MTV-h / pixel5 | 3,140 / 3,140 | 269 | 5,466,760.601142286 → 11,613.071870175912 |
| LAX-t / pixel5 | 1,466 / 1,466 | 163 | 6,797,577.308577215 → 15,277.217287551828 |
| MTV-u / pixel5 | 1,102 / 1,102 | 1,000 | 2,701,034.033147221 → 5,417.875948627148 |

This does not promote Phase85: its later main graph had zero iterations and
equal initial/final costs in every run, which Phase86 correctly reclassified as
NO-GO. It does establish that the ordinary GNSS-first stage reached a complete
position/clock/velocity state sequence before the main graph, unlike the
Phase89 velocity-only/direct-WLS boundary.

The current native entry point preserves that distinction
(`apps/native/gnss_fgo_imu_no_base.cpp:4257-4381,4417-4431`):

* with the direct-WLS selector it bypasses GNSS-first and validates the WLS
  sequence, which is the rejected Phase90 branch;
* without the direct-WLS selector it keeps `gnss_first_problem = problem`,
  disables C0/D only for the GNSS-first initializer, optimizes the same raw
  problem in memory, and derives its velocity sequence;
* without the velocity-only selector it copies the GNSS-first position and
  clock states back into the same in-memory `problem.epochs`, then passes the
  velocity sequence to `buildImuInput` in memory; and
* the velocity-only selector is the exceptional branch that rebuilds a second
  problem and refuses the position/clock copy. It must remain off for the new
  candidate.

The Phase80 freeze
`docs/use_cases/records/smartphone_r5_phase80_source_exact_direct_p_quality_freeze_v1.json`
(SHA-256
`9ebdde21fe7fd955029243d3bdbba3a9f9d3832388d67480760394177c8f685e`) and
Phase85 manifest
`docs/use_cases/records/smartphone_r5_phase85_source_clock_c0d_structural_manifest_v1.json`
(SHA-256
`20adc2e3388b92d045580739d025106c18aa068fcf26f8d117d48d3d2f5f7058`) are
used here only as sealed handoff/quality provenance. The Phase91 candidate's
future raw domain excludes base or coordinate-bearing external artifacts and
does not consume any Phase80/85 output file.

## Official source and raw-drift parity

The pinned official `fgo_gnss_imu.m`
(`output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m`, SHA-256
`c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3`) defines
the same staging semantics: the GNSS pass supplies `posini`, `velini`, `clk`,
and `dclk` (lines 40-61); those become `x_ini`, `v_ini`, `c_ini`, and `d_ini`
(lines 102-106); and the GNSS/IMU graph inserts those states before adding
P/D and C0/D factors (lines 169-218,252-277). The official file loading is
not reused as an input interface: the native path carries the equivalent
GNSS-first result in memory, so no result MAT or coordinate file is consumed.

The official `gnsslog2obs.m`
(`output/reproducibility-cache/gsdc2023/functions/gnsslog2obs.m`, SHA-256
`665ce43d1fc3c5a9c3b3a6fd76f81539126e3ab897fbba484ab0ccd18964acff`, lines
103-118) converts Android `DriftNanosPerSecond` to metres/second as
`c * DriftNanosPerSecond / 1e9`. The native Android adapter performs the same
conversion and preserves NaN when the raw field is absent
(`src/io/android_raw_gnss.cpp:783-800`); the problem builder carries it into
the retained `EpochSeed.receiver_clock_drift_mps`
(`src/algorithms/fgo_problems.cpp:344-392`). No coordinate or clock-bias
difference is used to manufacture this raw D value.

The current no-PDC main graph initializes D from a PDC seed when explicitly
enabled and otherwise inserts `0.0`
(`src/algorithms/fgo_gtsam_backend.cpp:267-280`). The Phase91 candidate's one
new state change is to replace only that no-PDC fallback with the finite raw
`problem.epochs[i].receiver_clock_drift_mps` value. The GNSS-first initializer,
its in-memory position/clock/velocity handoff, and all raw P/D factors remain
unchanged.

The official C0/D equation is metre-valued
`(C2-C1) - (D1+D2)*dt/2` with Jacobian `[-1,+1,-dt/2,-dt/2]`
(`output/reproducibility-cache/gtsam_gnss/src/ClockFactor_CCDD.h`, SHA-256
`7c174217218aa0059155e1e5a7182c69d56a3cd389d0ebd95c242d6d8c65b9fc`, lines
39-63). The native candidate preserves the existing seconds/m/s adaptation
`(c2-c1)-((d1+d2)*dt/(2*C_LIGHT))`, Jacobian
`[-1,+1,-dt/(2*C_LIGHT),-dt/(2*C_LIGHT)]`, and sigma `0.1/C_LIGHT`; it does
not authorize a unit rewrite or sigma tuning.

## Phase91 candidate boundary

Exactly one implementation candidate is supported and separately frozen as
`phase91_source_clock_c0d_gnss_first_in_memory_raw_drift_d_initializer`.
The implementation selector may be a new default-off opt-in, but its contract
is fixed:

1. Build the main raw Android GNSS problem once. Use the existing standard
   GNSS-first native FGO staging with `gnss_first_problem = problem`; do not
   enable the velocity-only or direct-WLS selectors.
2. Require exact retained-epoch identity before handoff: equal count, equal
   order, identical raw UTC key, and identical GNSS week/TOW for every
   GNSS-first solution and main `EpochSeed`. Reject any missing, duplicate,
   reordered, nearest, interpolated, or padded epoch. The existing native
   `GNSSTime` equality semantics may be used only as an asserted source-key
   check; no broader time tolerance is introduced.
3. Preserve the existing same-run in-memory handoff of GNSS-first optimized
   position, receiver clock, and velocity to the main Pose3+IMU graph. The
   handoff is raw GNSS→GNSS+IMU staging, not a precomputed coordinate input.
4. Initialize only the main graph's per-epoch D state from the corresponding
   retained raw `receiver_clock_drift_mps`, already in metres/second. Require
   one finite value per retained epoch. Missing, nonfinite, count-mismatched,
   or reordered drift fails closed; zero, hold, interpolation, WLS, residual,
   coordinate-difference, and PDC fallback are forbidden.
5. Preserve direct source-observable P/D quality, the C0/D equation/Jacobian/
   sigma, raw factors, optimizer, all physical and coverage gates, and Phase88
   active-solve telemetry. The candidate may not alter source sigma, direct
   quality, WLS, iteration limits, or termination rules.

Required dependencies are the raw Android GNSS/IMU and broadcast navigation
inputs, `--native-source-direct-observable-quality`,
`--native-source-clock-c0d-factor`, and explicit no-bridge mode. Forbidden are
`--native-gnss-first-velocity-only-handoff`,
`--native-direct-doppler-wls-handoff`, `--native-pdc-state-bridge`, legacy
upstream/PDC quality, base or external coordinate input, device-WLS/result
files, MAT/truth/Kaggle/token artifacts, and any accuracy score.

Before any later raw execution, implementation tests must prove the exact
epoch/UTC/drift alignment, finite full D coverage, same-run position/clock/
velocity handoff, D provenance in m/s, no PDC/coordinate input, unchanged C0/D
factor accounting, and fail-closed behavior. A separate pre-raw execution
manifest must then pin the implementation, binary, evaluator, and routes.
This Phase91 freeze authorizes no route execution.

## Read and execution accounting

| item | count/setting |
|---|---:|
| sealed Phase90 freeze read | 1 |
| sealed Phase90 audit read | 1 |
| sealed Phase40 direct-WLS result read | 1 |
| sealed Phase80 freeze and structural manifest reads | 2 |
| sealed Phase85 manifest and aggregate result reads | 2 |
| official pinned source files read | 3 |
| repository source/header files read | 6 |
| synthetic solver tests | 0 |
| native solver invocations/reruns | 0 |
| raw Android GNSS/IMU reads | 0 |
| broadcast navigation reads | 0 |
| base RINEX reads | 0 |
| truth/MAT/precomputed-coordinate reads | 0 |
| validation-holdout reads | 0 |
| Kaggle/token access | 0 |
| accuracy scoring/route selection | false |

Repository source pins:

* `apps/native/gnss_fgo_imu_no_base.cpp` —
  `a8511d42148dbff1d3833efd7ca3ff95aa575e955ea9da71e5db28b4aa59a6d3`
* `src/algorithms/fgo_problems.cpp` —
  `725167ccc21a62e61c4da9852726ed5cafbfc05ab794eaa76c4641676f576245`
* `src/algorithms/fgo_gtsam_backend.cpp` —
  `3935f1efa9a7fda4700188a3c57c0d466fc808f61c13d467e1954fe385990f7d`
* `src/io/android_raw_gnss.cpp` —
  `1bb9369e7db651e53fddab2a92c2f217512667bb349f70aa52934aeb80cbf387`
* `include/libgnss++/algorithms/fgo.hpp` —
  `438e0ed049c9c43c68af4b9b9b79e7b08fa18f38307f758821deff79be81ca52`
* `include/libgnss++/core/types.hpp` —
  `0dce831efb63bb4f7615752ba64f33ba339a6b0e479cc99a913f5a7cef6dc31b`

