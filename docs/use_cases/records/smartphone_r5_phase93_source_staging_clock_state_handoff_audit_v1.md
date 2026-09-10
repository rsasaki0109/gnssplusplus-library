# Smartphone R5 Phase93: source staging and clock-state handoff audit

## Decision

The official source staging is unambiguous: the GNSS-first stage owns a
per-epoch metre-valued receiver clock `C`, a metre/second receiver clock drift
`D`, and the source `ClockFactor_CCDD` row.  It optimizes both states and
exports `clkest` and `dclkest`.  `fgo_gnss_imu.m` consumes both outputs as its
initial clock states for the main GNSS+IMU graph.

The current native entry point does not have that staging.  Its GNSS-first
configuration is a Point3+velocity graph with all source C0/D flags cleared;
the main graph receives GNSS-first position and C, but its D state is seeded
from retained raw drift (Phase91) or zero/PDC state in the non-Phase91 path.
`FGOResult` exports velocity but has no optimized per-epoch D sequence.

Exactly one implementation-only candidate is therefore frozen in the
separate Phase93 freeze record: enable source-meter C0/D in the GNSS-first
Point3+velocity graph, export optimized per-epoch D, and hand off optimized C
and D together with the existing same-run position/velocity handoff into the
main meter-state graph.  This audit authorizes neither raw execution nor
accuracy scoring.  The legacy selector-absent path remains unchanged.

No raw Android file, broadcast navigation file, truth file, MAT artifact,
Kaggle/token endpoint, precomputed coordinate, or validation/holdout artifact
was opened or generated for this audit.  Only source text and sealed
Phase80/81/91/92 records and summaries were read.

## Authority and read-only evidence

The source and artifact pins used by this audit are:

| item | path | SHA-256 |
|---|---|---|
| official GNSS-first source | `output/reproducibility-cache/gsdc2023/fgo_gnss.m` | `5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3` |
| official GNSS+IMU source | `output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m` | `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3` |
| official clock unit conversion | `output/reproducibility-cache/gsdc2023/functions/gnsslog2obs.m` | `665ce43d1fc3c5a9c3b3a6fd76f81539126e3ab897fbba484ab0ccd18964acff` |
| official observation class | `output/reproducibility-cache/gsdc2023/functions/GobsPhone.m` | `11d43c1e76370b9d393075ec88071585efb3efb5d2fa97b3be58c0e1bd196986` |
| official residual screen | `output/reproducibility-cache/gsdc2023/functions/exobs_residuals.m` | `50c954a825edbdeaf5c9884486aff56c058f51d47610a06722d4f42b33095324` |
| official parameter source | `output/reproducibility-cache/gsdc2023/parameters.m` | `518925e9c75c7a14fceb5cd99432fe883311e0d64c72b94396744ac765120f52` |
| official C0/D factor | `output/reproducibility-cache/gtsam_gnss/src/ClockFactor_CCDD.h` | `7c174217218aa0059155e1e5a7182c69d56a3cd389d0ebd95c242d6d8c65b9fc` |
| official P factor | `output/reproducibility-cache/gtsam_gnss/src/PseudorangeFactor_XC.h` | `7f467698f2239818724eba4485b830fbc543586bbee06a21b8b2c03ae5b56415` |
| official D factor | `output/reproducibility-cache/gtsam_gnss/src/DopplerFactor_VD.h` | `de2c11d06ff860a95785c84620e5ed57dcfd5ca24995a0e642e9a5984fe093e1` |
| native entry point | `apps/native/gnss_fgo_imu_no_base.cpp` | `3ef5d9ce8dcc0b6e2ef72a019dd0ede67b9403730f5566310ea5d77d32acfc1e` |
| native problem staging | `src/algorithms/fgo_problems.cpp` | `e7607ce8f0fe27fd382a01b5b6f147b027b7bf51453e920eae458c86ddac0c17` |
| native GTSAM backend | `src/algorithms/fgo_gtsam_backend.cpp` | `0f97a40a6e863f068facc5113481b921c4050dc29afc7a820e4ad003187615bb` |
| native FGO result API | `include/libgnss++/algorithms/fgo.hpp` | `eb75b5ca5cbf41845721efc5cb97a2cb71c827828c7fc6a0526ef952cb52c2b3` |
| sealed Phase80 manifest | `docs/use_cases/records/smartphone_r5_phase80_source_exact_direct_p_quality_structural_manifest_v1.json` | `6d20541560b726ed2409cd05902e5105874ac72e113af0c3812f029130df310f` |
| sealed Phase80 structural output | `output/smartphone-r5/phase80-source-exact-direct-observable-quality-structural-v2/phase80_direct_observable_quality_structural_failure.json` | `767ef5f7089e535063ec6cbe4e9dc3e6d061bdab62eb521f794c3fab0b171903` |
| sealed Phase81 reclassification | `docs/use_cases/records/smartphone_r5_phase81_phase80_direct_observable_quality_reclassification_result_v1.json` | `f5809f173c3e346aec775ca6dd152de5436eb68ea348d3fe90dffc7f82153b14` |
| sealed Phase91 result | `docs/use_cases/records/smartphone_r5_phase91_source_clock_c0d_gnss_first_raw_drift_d_initializer_result_v1.json` | `9929e285b58b5d9a65350ba1a3d497c736968bb896d72dbe5fb014534b6d58c8` |
| sealed Phase92 result | `docs/use_cases/records/smartphone_r5_phase92_source_meter_clock_state_parity_structural_result_v1.json` | `8488306b3ec61d0360418de73fa1596271597b6971d77de676fcd279d7e1b01c` |

The Phase80 freeze is `9ebdde21fe7fd955029243d3bdbba3a9f9d3832388d67480760394177c8f685e`.
The Phase91 execution freeze and manifest are respectively
`456634b0a41da74de557185620bcb4d8c7b0572ebb7f3f1e415d8b8c82aa6fa4` and
`2078a4e2c3b963f07744c435ec303ea848bc372c8efd5003f6cb7b3d4b4ce988`.
The Phase92 execution freeze and manifest are respectively
`c07ac587ebcfde6391de0b8a824687316de854442166a93daadb416d4e9b74d8` and
`d6370b03f5d9dc48e8eb8da5667f06252caff444a2d8998cdac4380aea8f29a3`.

## Official GNSS-first staging

`fgo_gnss.m` establishes the first-stage state in four linked places:

* Initialization starts from SPP `posbl` and its gradient, and assigns
  `clk=[obs.clk zeros(n,6)]` and `dclk=obs.dclk` (`fgo_gnss.m:34-40`).  A
  subsequent run instead loads `clkest` and `dclkest` from the previous
  GNSS result (`:41-47`).
* The graph state is inserted as `c_ini=clk'` and `d_ini=dclk'`
  (`:89-94`), with priors on both state families (`:107-120`).
* Pseudorange and Doppler factors consume `keyC` and `keyD` directly
  (`:123-151`).  For an eligible adjacent epoch, the source inserts
  `ClockFactor_CCDD(keyC1,keyC2,keyD1,keyD2,dtgps,...)` with the ordinary or
  jump noise (`:154-177`).
* The GNSS-first LM solve is capped at 1000 iterations (`:203-218`).  It then
  retrieves both `clkest` and `dclkest` for every optimized epoch
  (`:220-230`) and saves both in `result_gnss.mat` (`:251-253`).

The source's unit boundary is explicit rather than inferred from magnitude:
`GobsPhone.m` labels `clk` as metres and `dclk` as metres/second
(`GobsPhone.m:9-16`), and `gnsslog2obs.m` computes both clock quantities by
scaling the Android nanosecond fields with `gt.C.CLIGHT` (`gnsslog2obs.m:103-118`).
The official `ClockFactor_CCDD.h` computes

```
(C2-C1) - (D1+D2)*dt/2
[-1,+1,-dt/2,-dt/2]
```

(`ClockFactor_CCDD.h:37-63`), so its active residual and Jacobian are in
metres when `dt` is seconds.  `parameters.m` fixes
`prm.sigma_motion_clk=0.1` m (`parameters.m:130-133`); this is the official
sigma and is not a tuning parameter.  The official P and D factors likewise
use direct `C`/ISB vector and `D` terms with unit derivatives
(`PseudorangeFactor_XC.h:46-66`, `DopplerFactor_VD.h:43-56`).

`fgo_gnss_imu.m` consumes the GNSS-first state as a true two-state handoff:

* On `initflag`, it loads `result_gnss.mat` and assigns `posini=posest`,
  `velini=velest`, `clk=clkest`, and `dclk=dclkest` (`fgo_gnss_imu.m:40-48`).
* It inserts `c_ini=clk'` and `d_ini=dclk'` and priors for both
  (`:102-106`, `:169-187`).  Its P/D factors use both keys
  (`:189-221`), and its adjacent clock row is again `ClockFactor_CCDD`
  (`:252-277`).
* The main IMU graph is optimized with the same 1000-iteration cap
  (`:325-340`), retrieves `clkest` and `dclkest` (`:342-357`), and saves both
  in `result_gnss_imu.mat` (`:378-380`).

The source residual screen also treats the clock drift as a physical
quantity, subtracting `dclk/obs.dt` from the Doppler residual
(`functions/exobs_residuals.m:14-24`).  This is source evidence for the
`D` unit contract only; no MATLAB execution was performed.

## Native contrast

The current native path differs at the exact staging boundary:

1. The raw entrypoint clones the main config for GNSS-first, sets
   `use_imu=false`, `use_pose3_state=false`, and explicitly clears
   `use_native_source_clock_c0d_factor`, phone, meter-state parity, active
   diagnostics, and raw-D initializer (`apps/native/gnss_fgo_imu_no_base.cpp:4641-4655`).
   It then enables the Point3 velocity-state path (`:4656-4667`).  Thus the
   GNSS-first graph cannot insert the source C0/D row; it uses the legacy
   scalar clock row selected by the flag-free backend path.
2. The backend gates the current C0/D factor on `use_imu`
   (`src/algorithms/fgo_gtsam_backend.cpp:206-212`), which independently
   confirms that the current Point3+velocity GNSS-first stage cannot use the
   candidate factor.
3. The standard in-memory handoff copies GNSS-first position and converts
   its public clock seconds to the main internal metre seed
   (`apps/native/gnss_fgo_imu_no_base.cpp:4751-4768`).  It does not copy an
   optimized D sequence.  In the main backend, D is initialized from the raw
   `EpochSeed.receiver_clock_drift_mps` only when the Phase91 raw-D selector is
   set; otherwise it takes a finite PDC seed or zero
   (`src/algorithms/fgo_gtsam_backend.cpp:341-361`).  Therefore the current
   path is “GNSS-first optimized C + raw/zero D,” not the official “optimized
   C + optimized D” handoff.
4. `EpochSeed` already carries raw receiver drift and immutable source identity
   (`src/algorithms/fgo_problems.cpp:352-397`), and the builder can retain an
   ordered subset after usable-measurement filtering (`:902-917`).  This is
   the correct place to preserve exact key identity, but retained vector index
   must not be confused with full raw vector index.  `FGOResult` currently
   exports per-epoch velocity and attitude vectors but no optimized per-epoch
   D vector (`include/libgnss++/algorithms/fgo.hpp:1343-1380`).

The legacy behavior is therefore structurally well-defined and must stay
default-off for the new candidate.  The implementation must not enable the
current C0/D selector in GNSS-first by simply bypassing the backend guard or
by reusing raw D; it must add the explicit source-meter GNSS-first state and
handoff contract below.

## Sealed solver evidence

The sealed Phase80 direct-quality artifact proves the no-C0D baseline behavior
on all four routes.  In each case the main GNSS+IMU graph accepted 12 outer
iterations and strictly reduced cost:

| route | graph iterations | initial cost | final cost | IMU intervals |
|---|---:|---:|---:|---:|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | 12 | 155976340.1623715 | 28128.450524519692 | 2158 |
| `2021-08-24-20-32-us-ca-mtv-h/pixel5` | 12 | 296097006.7793359 | 19159.621845562328 | 3139 |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | 12 | 190528071.24200463 | 22206.857308543094 | 1465 |
| `2023-03-08-21-34-us-ca-mtv-u/pixel5` | 12 | 59252349.484390005 | 8166.863690687162 | 1101 |

The Phase80 aggregate remains a sealed structural artifact; its original
fail-closed record and Phase81 reclassification are not accuracy evidence.

The Phase92 meter-state C0/D summaries show the opposite active-solve result
on the routes that reached the main graph:

| route | C0/D rows | raw D coverage | accepted outer iterations | initial cost | final cost | conditioning proxy |
|---|---:|---:|---:|---:|---:|---:|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | 2158 | 2159/2159 finite | 0 | 162235823.07086447 | 162235823.07086447 | 1.9999999980791472 |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | 1465 | 1466/1466 finite | 0 | 284569697.4514789 | 284569697.4514789 | 1.9999999980209395 |

Both summaries recorded ten inner lambda attempts, finite costs, an incomplete
termination trace, and `no_progress_unclassified`.  MTV-h failed closed in
the retained-epoch handoff; MTV-u published a structurally invalid domain
coverage.  The Phase92 formal result is consequently NO-GO and stops before
accuracy.  Its read accounting remains raw-only for the four authorized
native attempts, with zero truth/MAT/precomputed-coordinate/accuracy reads.

This comparison matters: the Phase92 meter conversion removed the measured
C0/D column-scale disparity (proxy near 2), but did not supply the missing
GNSS-first optimized D state.  Conditioning parity is necessary source
alignment, not a convergence guarantee.

## Phase93 implementation boundary

The following is the sole candidate to be frozen in the companion JSON:

`phase93_source_meter_c0d_gnss_first_retained_clock_state_handoff`

The candidate must satisfy all of these constraints:

* It is opt-in and default-off.  Legacy selector-absent GNSS-first and main
  graph behavior is unchanged.
* The GNSS-first stage remains the same raw in-memory problem and same direct
  raw P+D observable-quality path, with `use_pose3_state=false`,
  `use_imu=false`, and explicit Point3 position plus velocity states.  No PDC,
  direct-Doppler-WLS handoff, velocity-only handoff, external coordinate, or
  precomputed coordinate is allowed.
* GNSS-first inserts source `ClockFactor_CCDD` for eligible adjacent retained
  epochs with C and ISB in metres, D in metres/second, `dt` in seconds, the
  exact official residual/Jacobian, unchanged phone exclusion/jump/gap rules,
  and sigma `0.1` m.  The GNSS-first optimizer algorithm and its published
  iteration bound remain unchanged; no LM or sigma tuning is authorized.
* The backend exports an optimized per-epoch D sequence in `FGOResult`, with
  one finite value for every retained GNSS-first epoch.  The existing solution
  positions and public clock values remain available; C is converted to the
  main internal metre state exactly once at the handoff boundary.
* The main graph receives, in the existing same-run handoff, optimized GNSS-
  first position and velocity plus optimized C and optimized D.  Main D must
  not silently fall back to raw D, zero, hold, interpolation, WLS, residual,
  velocity-difference, or coordinate inference when this selector is active.
* Alignment is an exact retained-key contract.  Every retained main epoch,
  GNSS-first epoch, and GNSS-first solution maps exactly once, in source order,
  by immutable raw source index, integer raw UTC millisecond key, and GNSS
  week/TOW identity.  Raw epochs filtered before retention may remain
  unmatched; missing, duplicated, reordered, nonfinite, nearest-time,
  interpolated, padded, or truncated keys fail closed.
* All retained C/D/position/velocity values must be finite, and output
  structural gates remain mandatory.  No raw run is implied by this audit.

The candidate is an implementation boundary, not a result claim.  A later
implementation commit must precede any new execution freeze/authorization.

## Required implementation checks before any raw authorization

The next implementation must add or exercise read-only/unit tests for:

1. exact retained-key mapping with filtered-out raw epochs accepted, and
   missing, duplicate, reorder, time, week/TOW, nonfinite, and wrong-source
   keys rejected;
2. GNSS-first Point3 C/D factor insertion, official equation/Jacobian/sigma,
   finite full D coverage, and unchanged edge/phone/jump behavior;
3. `FGOResult` D export size/order/finiteness and same-run C/D/position/velocity
   handoff provenance;
4. main meter-state graph initialization from optimized C/D, with no raw/zero
   fallback under the candidate selector;
5. legacy selector-absent behavior and all existing direct-quality defaults;
6. compile and focused structural tests, including finite Earth-valid output,
   active iterations at least one, and strict cost decrease, before a separate
   raw execution authorization is considered.

## Phase93 read accounting

This audit made no data or solver invocation:

```text
native_solver_invocations       = 0
raw device_gnss reads           = 0
raw device_imu reads            = 0
broadcast navigation reads     = 0
base RINEX reads                = 0
truth reads                     = 0
MAT reads or generated         = 0
precomputed coordinate reads   = 0
validation/holdout reads       = 0
Kaggle/token access             = 0
accuracy calculations           = 0
route score selection           = false
```

