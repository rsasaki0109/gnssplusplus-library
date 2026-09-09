# Phase111 common-satellite/miss-mask parity audit

Status: sealed read-only source and aggregate audit.  This audit reads only
official/native source code and sealed Phase107/108/109/110 metadata.  It does
not read raw phone GNSS/IMU/navigation payloads, raw base RINEX bytes or
headers, truth payloads, MAT files, precomputed phone coordinates, PDC
artifacts, solver output, or Kaggle resources.  No solution CSV is opened in
Phase111; Phase110 already sealed the opaque Phase107/109 solution identity.
No solver, truth evaluator, accuracy evaluator, or raw execution is run.

## Answer to the classification question

All three MTV-A categories in the question are determined by control flow:

* `BDS_B1I` exact-stream-miss `12,849`: **(a) factor dropped** before a
  correction lookup.
* `GLO_L1CA` exact-stream-miss `45`: **(a) factor dropped** before a
  correction lookup.  The other `9,334` GLO rows have an exact stream and are
  retained after finite correction.
* `GPS_L1CA` out-of-domain `1,128`: **(a) factor dropped** after the exact
  stream exists but the in-domain interpolation callback fails.

There is no **(b) raw uncorrected retained** or **(c) zero-correction
fallback retained** path for these misses.  A valid correction whose numeric
value happens to be zero would be a normal finite correction, but neither the
official nor native miss path substitutes zero when a stream/time lookup is
missing.  The sealed telemetry reports zero nonfinite drops and closes the
factor accounting exactly.

## Pinned evidence

| Item | Path | SHA-256 | Use |
| --- | --- | --- | --- |
| Phase107 structural result | `docs/use_cases/records/smartphone_r5_phase107_raw_base_source_parity_result_v1.json` | `e7eb3f1d2670672414744a100dc1e2fc3e0b019ae7efc0dbfef1518300ce8d6e` | source-miss aggregate and route gates |
| Phase108 accuracy result | `docs/use_cases/records/smartphone_r5_phase108_phase107_solution_accuracy_result_v1.json` | `9f9a2528745c06ba06e774139f56892bcea0ef208a3d064e4b70042e48f29a9d` | already sealed route scores; no truth payload read |
| Phase109 structural result | `docs/use_cases/records/smartphone_r5_phase109_raw_base_frequency_parity_result_v1.json` | `b443c2fcd1ed3b067094b47bf0b81f7fa404ed20547e7234d1c174dcf9ca2225` | additional-band run and per-signal miss taxonomy |
| Phase110 audit | `docs/use_cases/records/smartphone_r5_phase110_additional_band_noop_decision_audit_v1.md` | `2b134217568797aeeaee85f1d2836202702b89222c4950f133dbde3c6a0fbb05` | sealed identical solution decision and aggregate comparison |
| Phase110 freeze | `docs/use_cases/records/smartphone_r5_phase110_additional_band_noop_decision_freeze_v1.json` | `91064f377228a506951599c946cbaaf6acf5fb7c5feb33a27dad8a6ba0172033` | no-op/no-new-accuracy boundary |
| Official GNSS source | `output/reproducibility-cache/gsdc2023/fgo_gnss.m` | `5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3` | L1/L5 correction and factor NaN gate |
| Official GNSS/IMU source | `output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m` | `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3` | same correction and factor NaN gate in IMU graph |
| Official correction helper | `output/reproducibility-cache/gsdc2023/functions/correct_pseudorange.m` | `b0536ccff478b0aff253448ffb7a203c715b8064dd8dc85898e38f1f05d0441e` | common-satellite intersection and interpolation |
| Native miss mask | `src/algorithms/source_pseudorange_miss_mask.cpp` | `ce85e428ac0a7af93ee1ad3eadf082b5361454dff5c920245be7c16c940b6949` | exact-key, in-domain, finite retention |
| Native application | `apps/native/gnss_fgo_imu_no_base.cpp` | `be643811cfaf82304f9434bc969796b2a14cd0f6e5a2c8b46b043af7b326c5c2` | shared factor-vector application and telemetry |
| Native correction model | `src/algorithms/base_pseudorange_compensation.cpp` | `f3c3c859793c4bed9c7ae93825d32565f20eab0badb09412a4fab2a5930c9817` | exact stream map and interpolation domain |

## Official common-satellite and interpolation behavior

The official helper first executes `obsb = obsb.sameSat(obsr)`
(`correct_pseudorange.m:5-6`).  It then computes base residuals for the
selected frequency field, applies the frozen 1-Hz/15-Hz moving mean, and
evaluates `interp1(obsb.time.t, pc_, obsr.time.t, "linear")`
(`correct_pseudorange.m:26-34`).  The call has no extrapolation argument, so a
missing frequency value or a rover time outside the available base domain is
NaN.  The two official graph builders subtract `pc` before graph creation and
add `PseudorangeFactor_XC` only under `~isnan(obsr.(f).resPc(i,j))`
(`fgo_gnss.m:72-78, 132-142`; `fgo_gnss_imu.m:85-92, 202-212`).  Doppler and
TDCP have separate guards and are not filled by the pseudorange path.

Thus the official behavior is equivalent to retaining only a common,
frequency-field-specific, finite, in-domain corrected code row.  It does not
retain the raw rover residual when the base row is missing and does not turn a
missing interpolation into a zero correction.

## Native control-flow proof

The native model stores finite base residual samples under the exact key
`{SatelliteId, SignalType}` (`base_pseudorange_compensation.cpp:255`) and its
`correctionAt` rejects an absent key, a nonfinite time, or a time before the
first/after the last sample before doing linear interpolation
(`base_pseudorange_compensation.cpp:307-339`).

The source miss mask is decisive (`source_pseudorange_miss_mask.cpp:65-116`):

1. `has_stream(factor.satellite, factor.signal)` is checked first.  False
   increments `dropped_missing_exact_stream_rows` and immediately `continue`s;
   the original factor is not appended to the retained vector.
2. An invalid epoch/time or a false `correction_at(...)` increments
   `dropped_out_of_domain_rows` and immediately `continue`s.
3. A nonfinite correction or nonfinite subtraction increments
   `dropped_nonfinite_correction_rows` and immediately `continue`s.
4. Only after those checks does the code subtract the finite correction, mark
   `native_base_pseudorange_correction_applied`, and append the factor.
   `factors.swap(retained)` therefore removes every failed row.

The application invokes that mask on the already-built raw rover
`problem.pseudorange_factors` (`gnss_fgo_imu_no_base.cpp:6822-6901`).  The
backend later iterates that same post-mask vector and passes each retained
`factor.corrected_pseudorange_m` to the graph factor
(`fgo_gtsam_backend.cpp:850-875`).  Phase101 GNSS-first copies the shared
problem before its staging solve (`gnss_fgo_imu_no_base.cpp:7037-7112`), and
the main handoff continues from the same problem after staging
(`:7326-7357`).  Consequently the Phase107/109 runs cannot have retained a
raw or zero-filled miss in either graph.

## Sealed MTV-A taxonomy

The Phase109 per-signal report closes the 57,281 adopted code rows:

| Signal / band | Original rows | Exact stream matched | Retained corrected | Exact-stream miss | Out of domain | Nonfinite | Classification of requested miss |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `BDS_B1I` / B1 | 12,849 | 0 | 0 | 12,849 | 0 | 0 | (a) dropped at `hasStream` |
| `GLO_L1CA` / G1 | 9,379 | 9,334 | 9,334 | 45 | 0 | 0 | (a) dropped at `hasStream` |
| `GPS_L1CA` / L1 | 21,434 | 21,434 | 20,306 | 0 | 1,128 | 0 | (a) dropped at `correctionAt` |
| `GAL_E1` / E1 | 13,619 | 13,619 | 13,619 | 0 | 0 | 0 | no miss |
| **Total** | **57,281** | **44,387** | **43,259** | **12,894** | **1,128** | **0** | **all failed rows dropped** |

The corresponding sealed graph counts are 57,281 raw/source-quality
pseudorange candidates before the mask and 43,259 pseudorange factors after
the mask.  The Phase109 report also records one correction application pass,
exactly-once correction, and no fallback.  This is direct aggregate evidence
that the 14,022 failed MTV-A rows did not remain as uncorrected or zero-filled
graph factors.

## LAX-T and the sealed accuracy difference

LAX-T has no exact-stream misses: its sealed taxonomy is 30,798 adopted,
30,664 retained corrected, zero exact misses, 134 out-of-domain, and zero
nonfinite.  The already sealed Phase108 truth-only aggregate scored MTV-A at
`1.1396560717187856 m` and LAX-T at `0.8241679860216076 m`, with macro
`0.9819120288701966 m` against the strict `0.782 m` gate.

This comparison rules out the claim that exact-stream misses alone determine
accuracy: LAX-T has zero such misses but still exceeds the strict route/macro
promotion boundary, while MTV-A has 12,894 exact misses and a larger score.
The retained-fraction difference and the additional 1,128 versus 134
out-of-domain rows are consistent with coverage being relevant, but route
difficulty and the other sealed graph quantities are confounded.  The sealed
aggregate cannot establish a causal score improvement or identify a missing
source key; no new truth evaluation is warranted or authorized.

## Single source-backed candidate freeze decision

The source evidence supports exactly one candidate contract:

`phase111-source-common-satellite-in-domain-corrected-code-retention-v1`

This is a default-off, source-parity retention candidate.  In the GNSS-first
Point3/velocity graph and the main meter-state graph, it retains only raw
rover code factors with an exact common `(SatelliteId, SignalType)` base
stream and a finite in-domain interpolated correction, subtracts that
correction once on the shared factor vector, and passes the retained vector to
both stages.  It changes no equation, signal mapping, C7/D topology, CCDD,
Doppler, TDCP, IMU, QR branch, LM schedule, filter, sigma, initialization, or
output alignment.  It adds no raw fallback, zero fill, endpoint hold,
extrapolation, MAT/PDC/precomputed coordinate, truth, or Kaggle input.

This formalizes the behavior already observed in the Phase107/109 source-mask
runs; it is not evidence of an accuracy gain and does not authorize a new raw
or truth execution.  Legacy/default selector-off behavior remains unchanged.

Rejected alternatives are: retaining raw rows on a missing base stream or
using zero as a fallback (contradicts both source paths), inventing a new
satellite/signal alias (no sealed per-key evidence), and extrapolating or
endpoint-holding out-of-domain rows (contradicts official `interp1` and
native `correctionAt`).  Any future structural run must report exact-key
common counts, in-domain counts, retained/dropped counts by signal, and
exactly-once application before any accuracy decision.

## Read accounting and boundary

| Resource/action in Phase111 | Count |
| --- | ---: |
| Official/native source reads | nonzero, source-only |
| Sealed Phase107/108/109/110 JSON and summary metadata reads | nonzero, aggregate/lineage only |
| Solution CSV reads | 0 |
| Raw phone GNSS/IMU/navigation payload reads | 0 |
| Raw base RINEX payload/header/stat/hash reads | 0 |
| Truth payload reads | 0 |
| MAT/precomputed phone-coordinate/PDC reads | 0 |
| Native solver invocations | 0 |
| Accuracy/truth evaluator invocations | 0 |
| Kaggle/token access | 0 |
| Raw reruns/fallbacks/publication | 0 |

The companion Phase111 freeze JSON records the one source-backed candidate.
No implementation, raw authorization, truth authorization, accuracy
evaluation, or solution publication is opened by this audit.
