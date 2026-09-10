# Phase117 TDCP weighting/source-parity audit

Status: read-only audit of the official MATLAB source, tracked native source,
and sealed aggregate records.  No raw phone GNSS/IMU/navigation payload, raw
base payload, truth payload, MAT data, precomputed phone coordinate, PDC
artifact, solution row, or Kaggle resource was opened.  No native solver or
accuracy evaluator was invoked.  The Phase108 and Phase112 records were used
only for their already-sealed aggregate fields; their truth and solution
payloads were not re-read.

## Decision

The official `fgo_gnss.m` and `fgo_gnss_imu.m` use the same ordinary temporal
TDCP family as the native Phase116 recipe: an adjacent pair for one satellite
and one signal, with a residual formed from the difference of the two carrier
residuals.  The source does **not** use one fixed carrier/TDCP sigma,
however.  `obserrmodel.m` computes a carrier noise value from the observation
SNR percentile and the signal-type factor, and the graph passes the value from
the previous endpoint to `noise_sigmas`.  Native Phase116 freezes every
ordinary TDCP factor at `0.03 m`.

The robust and admission contracts also differ.  The official L Huber
parameter is `0.2` for `Street`/`Mix` and `0.5` otherwise.  Native Phase116
uses the existing `4.0` normalized-residual threshold.  Official preprocessing
removes status/SNR/multipath rows, raw carrier jumps over `2e4` cycles, and
carrier/Doppler differences over `1.5 m`; native applies its exact-key pair
contract with a `2 s` gap limit, clock/loss/nonfinite gates, and a `10 m`
corrected-code-versus-carrier jump threshold.  These are not one-to-one
parameter substitutions.

Exactly one source-backed candidate is frozen: an opt-in, raw-only
**official SNR/signal-type TDCP weighting candidate**.  It changes only the
scalar noise attached to an already accepted ordinary TDCP factor.  It does
not change the residual equation, factor count, key incidence, pair-reject
predicate, robust threshold, graph topology, solver, or output.  The candidate
is frozen as a proposal only; this audit authorizes no implementation, raw
execution, solver run, truth read, accuracy evaluation, or solution release.

## Official source, line-by-line

The official graph construction is identical in the GNSS-only and GNSS/IMU
files:

* `fgo_gnss.m:50-60` and `fgo_gnss_imu.m:63-74` run `exobs`, residual
  calculation, and `exobs_residuals` before graph construction.
* `fgo_gnss.m:30` and `fgo_gnss_imu.m:36` select `FTYPE = ["L1","L5"]`.
* `fgo_gnss.m:179-200` and `fgo_gnss_imu.m:301-321` check finite `resL` at
  both endpoints and `~obs.clkjump(i+1)`, form
  `tdcp = resL(i+1,j)-resL(i,j)`, then add `TDCPFactor_XXDD` or
  `TDCPFactor_XXCC`.
* The `dtgps < prm.time_diff_th` block at
  `fgo_gnss.m:166-178` / `fgo_gnss_imu.m:266-278` encloses motion and clock
  factors.  The TDCP loop follows that block, so the graph source does not
  apply the `1.5 s` condition as a separate TDCP gate.  Clock discontinuity is
  still a TDCP gate through `obs.clkjump`.
* `preprocessing.m:181-195` derives `obs.clkjump` from a receiver-clock
  discontinuity.  It is not the carrier-phase jump test.

The official noise and robust contracts are:

* `parameters.m:22-44` selects SNR models for P, D, and L, sets
  `L_sn_ratio = 1/400`, and defines the signal factors in the order
  `L1,G1,E1,B1,L5,E5,B2a` as
  `[0.8,1.5,0.8,0.8,0.5,0.5,0.5]`.
* `obserrmodel.m:11-16,32-37` computes
  `sn_base = 10.^(-(S-prctile(S,85,"all"))/20)` and, for the SNR model,
  `obserr.L = (1/400)*sn_base.*sigfactor`.
* `fgo_gnss.m:185-188` and `fgo_gnss_imu.m:307-310` pass
  `obserr.(f).L(i,j)` to `noise_sigmas`, then wrap it with the official
  `gtsam.noiseModel.mEstimator.Huber` kernel.
* `parameters.m:110-125` sets L elevation/SNR masks, the `1.5 m` dDL mask,
  and `L_robust_prm = 0.2` for `Street`/`Mix`, otherwise `0.5`, before
  creating `L_kernel`.

The upstream carrier admission is also material to a jump comparison:

* `exobs.m:53-75` masks carrier rows for SNR below `20 dB-Hz`, slip or invalid
  status, multipath, and the documented device/GLONASS exclusions.  It then
  masks `abs([0; diff(L)]) > 2e4` in carrier samples (cycles).
* `exobs_residuals.m:82-99` applies the L elevation mask and computes
  `dDL = -(D2+D1)*lambda/2*dt - diff(L)*lambda`, applies the documented
  device offset where applicable, and masks `abs(dDL) > 1.5 m` at both
  adjacent endpoints.
* `gnsslog2obs.m:182-192` shows the source-domain representation: ADR metres
  are divided by wavelength for `L`, pseudorange rate is divided by wavelength
  for `D`, and SNR/status/multipath/wavelength fields are retained.  The
  source snapshot does not expose the implementation of the external custom
  TDCP factor, so this audit does not assume a numeric conversion of the
  official `obserr.L` value into the native metre domain.

## Native Phase116 source, line-by-line

Native ordinary TDCP construction and insertion are:

* `apps/native/gnss_fgo_imu_no_base.cpp:7427-7436` freezes the Phase116 raw
  recipe at `tdcp_sigma_m = 0.03`, `max_tdcp_gap_s = 2.0`, loss-of-lock and
  code-phase rejection enabled, and a `10.0 m` code-phase threshold.
* `src/algorithms/fgo_problems.cpp:1399-1429` rejects nonpositive or over-gap
  adjacent times and performs an exact previous-epoch lookup by
  `(SatelliteId, SignalType)`.
* `src/algorithms/fgo_problems.cpp:1431-1484` computes corrected carrier and
  code differences and calls `tdcp_contract::evaluateAdjacentPair`; accepted
  rows are stored at `:1486-1496` with `factor.delta_carrier_m` and the one
  scalar sigma.
* `include/libgnss++/algorithms/tdcp_contract.hpp:34-67` gives the exact
  reject order: gap, clock discontinuity, loss of lock, nonfinite
  measurement, then `abs(delta_carrier_m-delta_code_m) > threshold`.
* `src/algorithms/fgo_gtsam_backend.cpp:1193-1232` inserts the existing
  ordinary TDCP factor and calls `makeNoise(factor.sigma_m, ...,
  config.tdcp_huber_threshold_sigma)`.  The ordinary TDCP arm is therefore
  whitened by its factor sigma and optionally wrapped in the existing robust
  noise model; it does not use `carrier_phase_sigma_m` as a per-observation
  TDCP sigma.
* `include/libgnss++/algorithms/fgo_config.hpp:466-470` sets the native
  `tdcp_huber_threshold_sigma` default to `4.0`.  The read-only Phase116
  report defines robust cost as GTSAM Huber loss on the whitened residual,
  unwhitened cost as `sum residual_m^2`, and whitened cost as
  `sum (residual_m/sigma_m)^2`.

Thus the native factor family and key/pair selection are source-compatible at
the topology level, while its scalar weighting, robust threshold, and
pre-admission tests are not numerically identical to the official source.

## Parity matrix

| Aspect | Official source | Native Phase116 | Finding |
| --- | --- | --- | --- |
| Residual | Adjacent `resL(i+1)-resL(i)` for the same signal/satellite; device offset branch in the factor call | Corrected-carrier metre difference in `TimeDifferencedCarrierFactor` | Same ordinary temporal family; source-domain unit crosswalk must be explicit |
| Noise | SNR percentile model, `1/400`, and signal factor at the previous endpoint | Fixed `0.03 m` for every ordinary TDCP factor | **Mismatch; selected candidate** |
| Robust loss | GTSAM Huber, `k=0.2` for Street/Mix or `0.5` otherwise | Existing GTSAM Huber wrapper, `k=4.0` | Mismatch; not selected |
| Carrier prefilter | Status/SNR/multipath, raw `>2e4` cycle jump, then dDL `>1.5 m` | Existing raw masks plus pair contract; `2 s` gap and corrected code/carrier `>10 m` | Mismatch in predicate structure; not selected |
| Graph admission | Finite endpoints and no `obs.clkjump` in the graph loop | Exact key, finite/gap/clock/lock/code-phase gates | Same high-level family, different gates |
| Factor whitening | `noise_sigmas(obserr.L(i,j))` then official Huber | `makeNoise(factor.sigma_m, ..., 4.0)` | Same mechanism, different sigma and Huber input |

The selected candidate is deliberately one-dimensional: it does not combine a
noise change with either a robust-threshold change or a new jump detector.

## Sealed Phase116 cost attribution

The Phase116 result contains the requested TDCP cost fields.  The values below
are sums of the sealed per-signal TDCP reports; graph costs are shown
separately because they include all GNSS/IMU/clock factors.

| Route | TDCP factors built/inserted | Pair candidates | Missing previous | Code-phase rejects | sigma | TDCP robust cost initial -> final | TDCP unwhitened cost initial -> final | TDCP whitened cost initial -> final | Total graph cost initial -> final |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| MTV-A / Pixel5 | 31,269 / 31,269 | 33,025 | 4,282 | 1,756 | 0.03 m | 6,081,820.758650316 -> 3,186.2009960318637 | 126,783.1092066225 -> 6.2981007939537 | 140,870,121.3406915 -> 6,997.889771059641 | 130,723,525.58263575 -> 29,729.5954122218 |
| LAX-T / Pixel5 | 14,012 / 14,012 | 14,961 | 2,373 | 949 | 0.03 m | 2,713,558.838696147 -> 1,655.092719465339 | 59,652.584594572116 -> 17.03325281872271 | 66,280,649.54952459 -> 18,925.83646524745 | 162,184,507.52202186 -> 19,851.9725625441 |

The sealed reports also show native robust-outlier counts changing from
29,594 to 11 on MTV-A and from 13,238 to 2 on LAX-T under the fixed native
`k=4.0` contract.  These are attribution telemetry, not evidence that the
official `k` or SNR weighting would produce the same counts.  An official
source-side initial/final cost is unavailable because no official solver run
is part of this read-only audit.

## Sealed accuracy context (aggregate only)

The already-sealed aggregate fields provide context but cannot isolate TDCP
weighting from the other Phase changes:

| Sealed record | MTV-A score | LAX-T score | Macro | Other relevant change |
| --- | ---: | ---: | ---: | --- |
| Phase108 QR/C7 candidate | 1.1396560717187856 m | 0.8241679860216076 m | 0.9819120288701966 m | Offsetless Phase107/101 candidate |
| Phase112 Pixel5-offset candidate | 0.997685253035948 m | 0.6659910917640763 m | 0.8318381724000121 m | Final-output Pixel5 offset added |
| Phase82 same-route baseline | 1.1139384500152307 m | 0.9389644134001871 m | 1.026451431707709 m | Sealed prior baseline |

The Phase108/112 records are aggregate metadata only in this audit.  No truth
payload or solution row was reopened, and no causal accuracy claim is made.

## Exactly one candidate freeze

Candidate ID: `phase117-official-tdcp-snr-type-weighting-v1`.

The future opt-in candidate is bounded as follows:

* For every already accepted ordinary same-satellite/same-signal TDCP pair,
  derive the official L noise from the retained pair's previous endpoint:
  `sigma_L = (1/400) * 10^(-(S_i - percentile(S_f,85,"all"))/20) *
  sigfactor(system/frequency)`.
* Preserve the official signal-factor order and the exact retained
  `(SatelliteId, SignalType)` key.  The native factor is metre-valued, so a
  future implementation must prove the wavelength conversion from the
  source-domain L/noise representation to `sigma_m`; copying the numeric
  `obserr.L` value into metres is forbidden.  Missing/nonfinite SNR,
  wavelength, or signal mapping must fail closed rather than fall back to
  `0.03`, zero, an average, or a synthesized value.
* Change no pair admission or residual equation.  Native `2 s` timing,
  clock, loss-of-lock, nonfinite, and `10 m` code-phase gates remain fixed.
  The official upstream status/SNR/multipath/cycle/dDL filters remain outside
  this one-item candidate and are not silently substituted.
* Keep the native Huber threshold at `4.0` for this isolated candidate.  Keep
  ordinary TDCP factor count, C7/D/C0D state topology, QR branch, IMU/P/D
  factors, initialization, filter, LM schedule, output conversion, and
  legacy selector-off behavior unchanged.
* The candidate is default-off, raw-only, solution-withholding, and requires a
  separate implementation contract and focused unit tests before any run.
  This audit does not authorize a raw or solver invocation.

The robust-threshold alternative is not selected: although the official
source documents `0.2`/`0.5` and native documents `4.0`, selecting it would
mix a route/type-dependent robust change into this audit and would not test
the direct weighting mismatch.  The jump alternative is not selected because
the official behavior is a composition of status/SNR/multipath, a cycle-unit
threshold, a wavelength-scaled dDL test, and a clock-jump gate; changing the
single native `10 m` predicate cannot reproduce that composition.  Both remain
read-only findings.

## Evidence pins

Only source and sealed aggregate hashes are pinned here.  Raw member payloads
and their contents are not part of this audit.

| Item | Path | SHA-256 | Relevant lines/fields |
| --- | --- | --- | --- |
| Official GNSS graph | `output/reproducibility-cache/gsdc2023/fgo_gnss.m` | `5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3` | `30,154-200` |
| Official GNSS/IMU graph | `output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m` | `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3` | `36,252-321` |
| Official parameters | `output/reproducibility-cache/gsdc2023/parameters.m` | `518925e9c75c7a14fceb5cd99432fe883311e0d64c72b94396744ac765120f52` | `22-44,110-128` |
| Official observation noise | `output/reproducibility-cache/gsdc2023/functions/obserrmodel.m` | `43d0671a25c81fa6d7df0e1fbe81bf48a14eb3a35c8399600200b44961a7f9ef` | `11-37` |
| Official carrier masks | `output/reproducibility-cache/gsdc2023/functions/exobs.m` | `e22db383f372f26597333d674a12ee1e2adfdc97b00828fbbc5470903947f456` | `53-75` |
| Official residual masks | `output/reproducibility-cache/gsdc2023/functions/exobs_residuals.m` | `50c954a825edbdeaf5c9884486aff56c058f51d47610a06722d4f42b33095324` | `82-99` |
| Official clock-jump preprocessing | `output/reproducibility-cache/gsdc2023/preprocessing.m` | `976629d187e7fab5868eb8e5676a4d40520eb23db1254f7320d3b4270d7dffcf` | `181-195` |
| Official raw representation | `output/reproducibility-cache/gsdc2023/functions/gnsslog2obs.m` | `665ce43d1fc3c5a9c3b3a6fd76f81539126e3ab897fbba484ab0ccd18964acff` | `139-192` |
| Native TDCP builder | `src/algorithms/fgo_problems.cpp` | `c33f75c94b0fb9145ff4987905e32f4a1955192ee2ad3df10ada7142d45d99b4` | `1399-1496` |
| Native GTSAM insertion | `src/algorithms/fgo_gtsam_backend.cpp` | `78792c9306abe339228a3004e3d23f1486fac77822d0f1785b69dc07af14f578` | `1193-1232` |
| Native app contract | `apps/native/gnss_fgo_imu_no_base.cpp` | `1c5002fb9d9c614e139a7928ad8e0cd28149e82456505b71c4cdfbfba5fcad37` | `7427-7436,2920-2998` |
| Native config | `include/libgnss++/algorithms/fgo_config.hpp` | `a7009ba418601b7b0840c8ce7878aea402c820bb7c95e7114284df54c29db4f0` | `377-384,466-470` |
| Pair contract | `include/libgnss++/algorithms/tdcp_contract.hpp` | `1d56692d93f216703e2283b206b7bd0bca9c6593d51524d843fc6e385d96cbcf` | `34-67` |
| Phase116 sealed structural result | `docs/use_cases/records/smartphone_r5_phase116_carrier_tdcp_incidence_structural_result_v1.json` | `394ce1660ae5f181967b672bffab14d9228ffbe62ec6b748ca3854d30a14876f` | `phase116`, `tdcp_contract`, `read_accounting` |
| Phase108 sealed aggregate | `docs/use_cases/records/smartphone_r5_phase108_phase107_solution_accuracy_result_v1.json` | `9f9a2528745c06ba06e774139f56892bcea0ef208a3d064e4b70042e48f29a9d` | `aggregate`, route score fields only |
| Phase112 sealed aggregate | `docs/use_cases/records/smartphone_r5_phase112_main_output_offset_accuracy_result_v1.json` | `46d8d836758ac1a88b95ec2c1a0c0080cedfb753c12411ed74e4c2501549c57e` | `aggregate`, route score fields only |
| Phase116 result commit | repository commit | `3564f2870e31bad85ddb0b007c62dd296b5577a6` | sealed result commit |

## Read accounting and authorization boundary

| Resource/action in Phase117 | Count/status |
| --- | ---: |
| Official cached source reads | source-only; yes |
| Tracked native source reads | source-only; yes |
| Sealed aggregate record reads | 3 records; aggregate fields only |
| Raw phone GNSS/IMU/navigation payload reads | 0 |
| Raw base payload/header reads | 0 |
| Raw/base member hash computation | 0 |
| MAT data reads | 0 |
| Truth payload reads | 0 |
| Precomputed phone-coordinate reads | 0 |
| PDC reads | 0 |
| Native solver invocations | 0 |
| Accuracy/truth evaluator invocations | 0 |
| Solution rows opened or published | 0 |
| Kaggle/token access | 0 |
| Raw reruns/fallbacks | 0 |

The companion freeze JSON records the one candidate and keeps
`raw_execution_authorized`, `solver_execution_authorized`,
`truth_evaluation_authorized`, `accuracy_authorized`, and
`solution_publication_authorized` false.  Any implementation or experiment
requires a later, independently pinned contract and authorization.
