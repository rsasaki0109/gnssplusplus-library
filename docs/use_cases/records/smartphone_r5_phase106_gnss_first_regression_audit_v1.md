# Phase106 GNSS-first regression root-cause audit

Status: read-only source/sealed-artifact audit.  No raw input, truth, MAT,
solver, or accuracy evaluation was read or run for Phase106.

## Scope and pinned evidence

The audit compares the sealed Phase82 direct-quality aggregate with the
sealed Phase105 same-run stage/main attribution, and compares the official
GNSS source with the native no-base implementation.  The following source
and aggregate pins were inspected or carried forward from the sealed record:

| Item | Path | SHA-256 |
| --- | --- | --- |
| Phase82 aggregate | `docs/use_cases/records/smartphone_r5_phase82_phase80_direct_quality_accuracy_result_v1.json` | `39c5a38b7e3e61fda0306582fffb961dad3e6fb0bd49fcfe70e66674e2c90873` |
| Phase105 raw structural seal | `docs/use_cases/records/smartphone_r5_phase105_runtime_linkage_raw_execution_seal_v1.json` | `45e194fa59c25fa94cfac8e4a8b9b0d17747ae239497d8785b39f113afa74272` |
| Phase105 attribution aggregate | `docs/use_cases/records/smartphone_r5_phase105_runtime_linkage_truth_only_attribution_result_v1.json` | `d4a1f4c05c00a65abe38f900279c6bd3a9cc9c1e0c4d0eb571608929c0da381c` |
| Official GNSS source | `output/reproducibility-cache/gsdc2023/fgo_gnss.m` | `5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3` |
| Official GNSS/IMU source | `output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m` | `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3` |
| Native no-base app | `apps/native/gnss_fgo_imu_no_base.cpp` | `c81381b0a8fa8cea5c101fc5d54e6364173c0e8cffb83f02eac18c8b6cafde5b` |
| Native problem construction | `src/algorithms/fgo_problems.cpp` | `e7607ce8f0fe27fd382a01b5b6f147b027b7bf51453e920eae458c86ddac0c17` |
| Native SPP/WLS | `src/algorithms/spp.cpp` | `dee40bacfab037c0fc49488af8e5c137deb1005596e7da402153c2e7c1687b76` |
| Native GTSAM backend | `src/algorithms/fgo_gtsam_backend.cpp` | `781d65e34826ec08e04d1dfdac0ab10b2ed9409726078465f3020b57afac4baa` |
| Native factors | `src/algorithms/fgo_gtsam_internal.hpp` | `cc6327e36800afa8217e5a834260adc81c0a74a5aefbfc956bb4679c8d86105f` |
| FGO configuration | `include/libgnss++/algorithms/fgo_config.hpp` | `3cc60abe514ef8900012064accb17f27d3927d0b8a8776ad76ce6aefa83ce32c` |

The Phase82 number is a sealed baseline only.  Its Phase80-derived recipe
included base-pseudorange compensation, a base RINEX input/miss mask, and
signal-bias states.  Those inputs are not permitted in the Phase106 no-base
contract, so the Phase82 score is not a clean no-base ablation; it is still
the requested historical quality reference.

## Sealed regression evidence

The sealed Phase105 attribution reports the following route scalars.  The
stage and main values are same-run outputs; no Phase106 re-evaluation was
performed.

| Route | Phase82 scalar | Phase105 stage | Phase105 main | Stage delta | Main delta |
| --- | ---: | ---: | ---: | ---: | ---: |
| MTV-A | 1.1139384500152307 | 3.334004080470148 | 1.139793072101309 | +2.2200656304549176 | +0.02585462208607825 |
| LAX-T | 0.9389644134001871 | 5.782811093104809 | 3.310820065300743 | +4.843846679704622 | +2.371855651900556 |

Phase105 structural telemetry says both routes had finite, exact-key,
full C/D handoff, earth-valid outputs, QR main selection, 12 accepted main
iterations, and strict main cost decrease.  The stage was therefore not
rejected by the structural gate.  The sealed attribution classifies both
routes as `stage-regression-present`; that classification does not identify
which stage factor family caused the error.

## Source comparison

| Concern | Official source | Native Phase105 path | What is established |
| --- | --- | --- | --- |
| Raw seed and timing | `fgo_gnss.m:34-61` starts from `posbl`/clock data and computes satellite states and residuals before graph construction. | `fgo_problems.cpp:344-391,416-495` runs native SPP least-squares per raw epoch, estimates a position/clock seed, and carries raw receiver drift/time keys. | Both have an upstream seed, but the exact numerical equivalence of the preprocessed MATLAB path and native raw SPP path is not established by sealed aggregates. |
| Pseudorange correction | `fgo_gnss.m:72-86` applies the official observation/error model and a base correction. | `fgo_problems.cpp:505-580` applies broadcast satellite clock, ionosphere/troposphere/group-delay terms when enabled, earth-rotation-corrected satellite geometry, and a native sigma model; `spp.cpp:1143-1184` has the SPP correction/variance path. | The no-base native path necessarily differs from the official base branch. Sign/unit/order parity for every correction is not proven by the available aggregate. |
| Doppler | `fgo_gnss.m:123-177` uses `DopplerFactor_VD` and a scalar drift state. | `fgo_problems.cpp:683-807` converts Doppler to m/s, models satellite range rate/clock drift, and forms receiver-only residuals; backend factors are in `fgo_gtsam_internal.hpp:3603-3670`. | Both use velocity-plus-drift factors, but native corrected range-rate and sign conventions are an additional parity surface. |
| Earth rotation and atmosphere | Official satellite geometry is supplied by `gt.Gsat`; the source does not expose the helper implementation in these two files. | Native explicitly rotates satellite state before geometry and uses a plain range (`fgo_problems.cpp:498-504`; `fgo_gtsam_internal.hpp:2320-2342`); atmosphere is conditional. | The native correction is explicit, but equivalence to the official helper, including timing/atmosphere choices, cannot be inferred from scores. |
| Weighting and filtering | Official `obserrmodel`, diagonal sigmas, and robust kernels are set in `fgo_gnss.m:81-86`. | Direct-quality mode selects upstream observable quality, SNR 20 dB-Hz, elevation 5 degrees, p85 residual screening, adjacent-gap 1.5 s, and route Huber settings (`gnss_fgo_imu_no_base.cpp:6523-6544`; construction in `fgo_problems.cpp:828-975`). | This is a real configuration/model difference; no sealed residual distribution attributes the stage regression to it. |
| C7 mapping and clock topology | Official graph inserts per-epoch 7-component `clk` plus scalar `dclk` and CCDD rows (`fgo_gnss.m:89-177`). | Native Phase101 maps constellation/frequency components in `fgo_gtsam_internal.hpp:1297-1329`, uses epoch-local C7 plus D, meter CCDD sigma 0.1 m, raw D initialization, and exact C/D handoff (`fgo_gtsam_backend.cpp:445-540,727-769`; factor definition `1430-1534`). | Topology, component order, meter units, and CCDD equation are pinned for the candidate and structurally passed, but this does not prove numerical source parity. |
| Priors/gauge | Official inserts infinite per-epoch x/v/c/d priors (`fgo_gnss.m:107-121`). | Native uses its configured initial values and backend priors (`fgo_gtsam_backend.cpp:1486-1587`), with Phase101 global ISB disabled. | Gauge/initial-value behavior differs in implementation; the sealed structural result does not provide enough rank or residual attribution to call it the cause. |
| Time alignment | Official CCDD uses adjacent `utcms` differences and skips long gaps/phone-specific cases (`fgo_gnss.m:154-177`). | Native retains exact raw epoch identity and uses adjacent finite gaps, with the same 1.5 s CCDD gap contract and exact-key handoff. | Structural key alignment is valid, but correction timing and retained-observation membership remain possible differences. |
| Velocity WLS | Official initialization uses `posbl.gradient(obs.dt)` (`fgo_gnss.m:34-48`). | Native has a dedicated Doppler velocity-WLS path, but Phase105 explicitly disables it; GNSS-first uses velocity states and the raw D initializer. | The phrase “raw WLS seed” is precise for the native SPP position/clock seed. It must not be conflated with the disabled dedicated Doppler WLS initializer. |

## Root-cause triage

Facts do not select a unique algorithmic fix.  The following are the three
most plausible, mutually distinguishable hypotheses from source differences;
none is promoted without factor/residual evidence:

1. A raw SPP seed versus graph pseudorange-model mismatch (satellite clock
   transmit-time correction, earth rotation, atmosphere/group delay, sign,
   or timing) puts the GNSS-first solve in a poor basin.  The source shows
   several different correction paths, but no sealed family residual or
   seed-to-optimized displacement proves one term.
2. The stage Doppler/velocity path is the dominant error.  Phase105 disables
   dedicated Doppler velocity WLS, while the graph still uses corrected
   Doppler and D factors; this is a factual difference, not proof of
   causality.
3. Direct-quality membership/weighting or C7/C0D initialization changes the
   stage balance.  The p85/SNR/elevation/Huber settings and epoch-local
   clock topology are all structurally enabled, but the available seal has
   no code/D/C0D residual breakdown.

The stage-only deterioration on MTV-A, and the deterioration of both stage
and main on LAX-T, also prevents assigning a single common failure from
the aggregate alone.  Changing sigma, LM behavior, filtering, equations,
or enabling a base correction would therefore be unsupported and outside
the no-base contract.

## Decision and next boundary

No unique source/raw-only algorithmic correction is frozen.  The only
candidate frozen for a later, separately authorized run is a default-off,
read-only diagnostic that observes the same-run native GNSS-first seed to
optimized-stage change and factor-family residual summaries.  It must not
modify graph construction, values, equations, units, sigmas, filters,
robust loss, LM settings, ordering, initialization, or fallback behavior.

The diagnostic should compactly record, per retained key and aggregate (no
coordinate-row publication):

- finite/earth-valid raw SPP seed count and seed-to-stage displacement
  scalars, plus optimized-stage displacement scalars;
- pseudorange/code, Doppler, C0D, motion, and prior factor counts and finite
  residual summaries (including normalized summaries where available);
- correction provenance/counts for satellite clock, earth rotation,
  ionosphere, troposphere, and group delay, and exact retained-key/time
  alignment;
- C7 component mapping counts, D initialization source, and the factual
  Phase105 dedicated-Doppler-WLS-disabled flag.

The future contract is exactly one diagnostic run for MTV-A and one for
LAX-T, using only `device_gnss.csv`, `device_imu.csv`, and broadcast nav.
MAT, base, PDC, precomputed/external coordinates, truth during solving,
accuracy output, and Kaggle submission are forbidden.  Truth evaluation and
any solver/algorithm change remain separately unauthorized.  The companion
JSON freeze records these boundaries and pins this audit.
