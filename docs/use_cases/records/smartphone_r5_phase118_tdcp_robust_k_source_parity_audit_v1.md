# Phase118 TDCP robust-k/source-parity audit

Status: read-only audit of the official MATLAB graph source, tracked native
source, and sealed Phase112/116/117 structural records.  No raw phone
GNSS/IMU/navigation payload, raw base payload, truth payload, MAT data,
precomputed phone coordinate, PDC artifact, solution row, or Kaggle resource
was opened.  No native solver or evaluator was invoked.  This record freezes
one candidate design only; it is not implementation or execution
authorization.

## Decision

The official TDCP robust kernel is a single `L_kernel` shared by every
accepted L1/L5 TDCP factor in one run.  `parameters.m` selects its Huber
threshold from the route setting type: `0.2` for `Street` or `Mix`, and `0.5`
otherwise.  The source does not select the TDCP Huber threshold from SNR,
frequency, constellation, or individual observation type.

The official SNR/frequency mapping is a separate carrier-noise contract:
`obserrmodel.m` computes `obserr.L` using the all-observation SNR 85th
percentile, denominator 20, `L_sn_ratio = 1/400`, and a seven-entry
system/frequency factor.  That value is passed to `noise_sigmas` and then
wrapped by the same `L_kernel`.  Thus official robust-k selection and
official sigma selection are source-separable.

Native Phase112/116 keeps ordinary TDCP sigma at `0.03 m` and the Huber
threshold at `4.0` in whitened-residual units.  Phase117 changed only the
ordinary TDCP sigma to the official SNR/type model; its native TDCP robust
threshold remained `4.0`.  The single candidate frozen here therefore keeps
the Phase112 fixed `0.03 m` weighting and changes only the TDCP Huber-k
mapping to the official route-type mapping.  Phase117 dynamic sigma is not
combined with this candidate.

## Official source facts

### TDCP topology and robust application

Both official functions declare `FTYPE = ["L1","L5"]`:

* `fgo_gnss.m:30` and `fgo_gnss_imu.m:36` select the two frequency bands.
* `fgo_gnss.m:179-200` and `fgo_gnss_imu.m:301-321` iterate satellite and
  frequency, require finite carrier residuals at both adjacent epochs and no
  receiver clock jump, form `resL(i+1,j)-resL(i,j)`, and add the existing
  `TDCPFactor_XXDD` or `TDCPFactor_XXCC`.
* `fgo_gnss.m:185-188` and `fgo_gnss_imu.m:307-310` construct
  `noise_sigmas(obserr.(f).L(i,j))` and wrap that noise with
  `noise_robust(prm.L_kernel, noise)`.  The same `prm.L_kernel` is used for
  every retained signal in that run; there is no inner SNR/frequency branch
  for the Huber parameter.

### Robust-k mapping

`parameters.m:57-60` obtains the GTSAM Huber constructor.  Its carrier/TDCP
  branch at `parameters.m:110-125` is:

| Official `setting.Type` | `prm.L_robust_prm` / Huber k | Scope |
| --- | ---: | --- |
| `Street` | 0.2 | all retained L1/L5 TDCP factors in that run |
| `Mix` | 0.2 | all retained L1/L5 TDCP factors in that run |
| any other official type (including `Highway`) | 0.5 | all retained L1/L5 TDCP factors in that run |

This is a route-setting mapping, not an observation metadata mapping.  The
official source does use different P and D robust values elsewhere in
`parameters.m:77-108`; those values are not TDCP k and are outside this
candidate.

### SNR, observation type, and frequency mapping

The source-side mapping that feeds `obserr.L` is explicit and independent of
the robust-k branch:

* `parameters.m:22-44` selects SNR models for P, D, and L, sets
  `sn_ptile=85`, `sn_den=20`, `L_sn_ratio=1/400`, and defines the factor order
  `[L1,G1,E1,B1,L5,E5,B2a]` with values
  `[0.8,1.5,0.8,0.8,0.5,0.5,0.5]` (an eighth unsupported entry is `NaN`).
* `obserrmodel.m:11-16,32-37` computes
  `sn_base = 10.^(-(S-prctile(S,85,"all"))/20)` and, for the active SNR
  model, `obserr.L=(1/400)*sn_base.*sigfactor`.
* `sysfreq2sigtype.m:6-17` maps L1 to GPS/GLO/GAL/BDS indices 0/1/2/3 and
  L5 to GPS/GAL/BDS indices 4/5/6; unsupported systems map to index 7 and
  therefore the `NaN` factor.  These indices choose the sigma factor, not
  Huber k.

Consequently, official k is constant across L1 versus L5, across GPS/GLO/
GAL/BDS, and across SNR values within a route type.  SNR and signal type
  change the sigma that precedes robust whitening, while `setting.Type`
  changes the Huber threshold.

## Native source comparison

The native Phase112/116 contract is fixed by
`apps/native/gnss_fgo_imu_no_base.cpp:7465-7475` and the current config:

* `tdcp_sigma_m = 0.03`;
* existing exact-key adjacent TDCP admission, loss-of-lock and code-phase
  rejection remain enabled;
* `fgo_config.hpp:385` defaults `tdcp_sigma_m` to `0.03` and
  `fgo_config.hpp:477` defaults `tdcp_huber_threshold_sigma` to `4.0`.

`fgo_gtsam_backend.cpp:1206-1227` passes the existing factor sigma and
  `config.tdcp_huber_threshold_sigma` to the ordinary TDCP factor's
  `makeNoise`.  The application telemetry helper at
  `gnss_fgo_imu_no_base.cpp:3492-3506` evaluates the same Huber contract on a
  whitened residual.  Phase117's opt-in block at
  `gnss_fgo_imu_no_base.cpp:7593-7598` enables only
  `use_official_tdcp_snr_type_sigma`; it does not change the native Huber
  threshold.  Therefore the statement that Phase117 retained `k=4.0` is
  established by tracked source/config, not inferred from a truth score.

The existing native route table at
`gnss_fgo_imu_no_base.cpp:369-381` provides a deterministic source-type
  crosswalk without truth or result data:

| Sealed route ID | Existing source type | Official TDCP k if the candidate is enabled |
| --- | --- | ---: |
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | `Highway` | 0.5 |
| `2021-08-24-20-32-us-ca-mtv-h/pixel5` | `Street` | 0.2 |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | `Highway` | 0.5 |
| `2023-03-08-21-34-us-ca-mtv-u/pixel5` | `Street` | 0.2 |

Unknown route/type identity must fail closed.  This table is only a pinned
source-setting lookup; it is not a score- or truth-derived choice.

## Sealed Phase112/116/117 comparison

The following values are read from sealed structural JSON fields only.  The
TDCP robust costs are the native Huber-on-whitened-residual telemetry; graph
costs include all graph families and are shown only to identify the sealed
run.

| Route | Record | TDCP factors | sigma contract | TDCP robust cost initial -> final | Reject counters (gap / code-phase jump) |
| --- | --- | ---: | --- | ---: | ---: |
| MTV-A / Pixel5 | Phase112 champion | 31,269 | fixed 0.03 m; native k=4.0 | 6,081,820.758650316 -> 3,186.2009960318637 | 0 / 1,756 |
| LAX-T / Pixel5 | Phase112 champion | 14,012 | fixed 0.03 m; native k=4.0 | 2,713,558.838696147 -> 1,655.092719465339 | 0 / 949 |
| MTV-A / Pixel5 | Phase116 fixed-sigma diagnostic | 31,269 inserted exactly | fixed 0.03 m; native k=4.0 | same fixed-sigma diagnostic contract | 0 / 1,756 |
| LAX-T / Pixel5 | Phase116 fixed-sigma diagnostic | 14,012 inserted exactly | fixed 0.03 m; native k=4.0 | same fixed-sigma diagnostic contract | 0 / 949 |
| MTV-A / Pixel5 | Phase117 dynamic-sigma structural run | 31,269 inserted exactly | official SNR/type sigma; native k=4.0 | 529,041,652.9457475 -> 2,025,636.9723952692 | 0 / 1,756 |
| LAX-T / Pixel5 | Phase117 dynamic-sigma structural run | 14,012 inserted exactly | official SNR/type sigma; native k=4.0 | 136,745,893.39872664 -> 405,627.73866347654 | 0 / 949 |

Phase117's sealed per-signal sigma values demonstrate the distinct weighting
dimension without changing factor population:

| Route | Signal (official band) | Phase117 sigma in native metres |
| --- | --- | ---: |
| MTV-A | GPS_L1CA (L1) | 0.00029204807234936797 |
| MTV-A | GLO_L1CA (G1) | 0.0014977098023663752 |
| MTV-A | GAL_E1 (E1) | 0.0004319707535603056 |
| MTV-A | BDS_B1I (B1) | 0.0003158061122364722 |
| LAX-T | GPS_L1CA (L1) | 0.0005564861296186086 |
| LAX-T | GLO_L1CA (G1) | 0.0008555752446766727 |
| LAX-T | GAL_E1 (E1) | 0.0006538141930466653 |

The Phase117 `tdcp_contract` records `official_snr_type_sigma_enabled=true`,
the 85th percentile, zero invalid-weight rejections, finite residuals, and
the same factor/reject counts shown above.  It does not enable a robust-k
change.  No accuracy or truth result is used to select this candidate.

## Exactly one candidate freeze

Candidate ID: `phase118-official-tdcp-huber-k-mapping-v1`.

The future opt-in selector is proposed as
`--native-phase118-official-tdcp-huber-k`.  On admission, it must use the
existing exact route-to-`setting.Type` table above and set only the robust
threshold supplied to the existing ordinary TDCP `makeNoise` call:

* `Street` or `Mix`: Huber k `0.2`;
* any other official type, including `Highway`: Huber k `0.5`.

The candidate starts from the Phase112 champion recipe and leaves
`use_official_tdcp_snr_type_sigma=false`, so every ordinary TDCP factor keeps
the fixed `0.03 m` sigma.  It is source-backed and not truth-tuned.  The
candidate is default-off, raw-only, fail-closed for unknown route/type, and
must withhold solution rows.

The exact change boundary is:

| Contract | Candidate behavior |
| --- | --- |
| Changed scalar | TDCP Huber threshold only: `4.0` -> official `0.2`/`0.5` by source type |
| Sigma | Fixed `0.03 m`; no Phase117 dynamic sigma |
| Residual/equation/units | unchanged; existing metre-valued ordinary TDCP residual |
| Pair keys and count | unchanged exact `(SatelliteId, SignalType)` adjacent pairs |
| Reject predicates | unchanged gap, clock, loss-of-lock, nonfinite, and code-phase gates |
| Other robust kernels | unchanged P, Doppler, carrier, and all non-TDCP settings |
| C7/D/C0D, QR, raw-base, Pixel5 offset | unchanged and applied exactly as the pinned recipe |
| IMU, filters, initialization, LM, ordering, damping, output | unchanged |
| Default/legacy | unchanged; selector off is the current behavior |

The Phase117 sigma candidate and any admission/jump/filter candidate are not
combined here.  They remain separate dimensions and are not authorized by
this freeze.  This audit also does not authorize code changes, a raw run, a
solver run, truth evaluation, accuracy scoring, rerun, fallback, or Kaggle
submission.

## Evidence pins

### Official source

| Source | SHA-256 | Relevant lines |
| --- | --- | --- |
| `output/reproducibility-cache/gsdc2023/fgo_gnss.m` | `5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3` | `30,154-200` |
| `output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m` | `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3` | `36,252-321` |
| `output/reproducibility-cache/gsdc2023/parameters.m` | `518925e9c75c7a14fceb5cd99432fe883311e0d64c72b94396744ac765120f52` | `22-44,57-125` |
| `output/reproducibility-cache/gsdc2023/functions/obserrmodel.m` | `43d0671a25c81fa6d7df0e1fbe81bf48a14eb3a35c8399600200b44961a7f9ef` | `11-37` |
| `output/reproducibility-cache/gsdc2023/functions/sysfreq2sigtype.m` | `5ec1d299e04b45f604921cbae3b1fac8f78f3f76a0152496d86011a32f1d0079` | `6-17` |
| `output/reproducibility-cache/gsdc2023/functions/exobs.m` | `e22db383f372f26597333d674a12ee1e2adfdc97b00828fbbc5470903947f456` | `53-75` |
| `output/reproducibility-cache/gsdc2023/functions/exobs_residuals.m` | `50c954a825edbdeaf5c9884486aff56c058f51d47610a06722d4f42b33095324` | `82-99` |

### Native source

| Source | SHA-256 | Relevant lines |
| --- | --- | --- |
| `apps/native/gnss_fgo_imu_no_base.cpp` | `ae89f38d5672474bcd63a7134f852f1fb63c44c55e70990116875001508fe667` | `369-381,3492-3506,7465-7475,7593-7598` |
| `include/libgnss++/algorithms/fgo_config.hpp` | `b0bba8f8f1c0f5aac1e1dca19705e7291b3134fe1d3155436ff05995da177342` | `385,477` |
| `src/algorithms/fgo_gtsam_backend.cpp` | `78792c9306abe339228a3004e3d23f1486fac77822d0f1785b69dc07af14f578` | `1206-1227` |
| `src/algorithms/fgo_problems.cpp` | `327fdc792afbeb29f150bc8566acaef63156c519d2dcb0a40f626659658edddc` | `1408-1529` |
| `include/libgnss++/algorithms/tdcp_contract.hpp` | `1d56692d93f216703e2283b206b7bd0bca9c6593d51524d843fc6e385d96cbcf` | `34-67` |

### Sealed structural records

| Record | SHA-256 | Used fields |
| --- | --- | --- |
| `docs/use_cases/records/smartphone_r5_phase112_main_output_offset_structural_result_v1.json` | `087b1bf51b4e584b86a7b558560df4ceb78e19b01e572f2d60aeef4c3d759ba0` | fixed-sigma recipe, route factors/cost/progress |
| `docs/use_cases/records/smartphone_r5_phase116_carrier_tdcp_incidence_structural_result_v1.json` | `394ce1660ae5f181967b672bffab14d9228ffbe62ec6b748ca3854d30a14876f` | fixed sigma, factor/reject/cost telemetry |
| `docs/use_cases/records/smartphone_r5_phase117_tdcp_weighting_structural_result_v1.json` | `2517dcc805146dc34e790c78a392caf000c79eaba837fe099b6d522c147e9cf2` | dynamic sigma, unchanged factor/reject telemetry |

## Read accounting and authorization boundary

| Resource/action in this audit | Count/status |
| --- | ---: |
| Official cached source reads | source code only |
| Tracked native source reads | source code only |
| Sealed structural JSON records | 3; structural/aggregate fields only |
| Raw phone GNSS/IMU/navigation payload reads | 0 |
| Raw base payload or member hash reads | 0 |
| MAT reads or generation | 0 |
| Truth reads | 0 |
| Precomputed phone-coordinate reads | 0 |
| PDC reads | 0 |
| Native solver invocations | 0 |
| Accuracy/evaluator invocations | 0 |
| Solution rows opened or published | 0 |
| Kaggle/token access | 0 |
| Reruns/fallbacks | 0 |

The companion Phase118 freeze JSON records the same closed authorization
boundary.  Any implementation or experiment requires a later independently
pinned implementation contract and authorization.
