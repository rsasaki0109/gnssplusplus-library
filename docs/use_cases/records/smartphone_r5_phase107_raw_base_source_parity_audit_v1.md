# Phase107 raw-base RINEX source-parity audit

Status: read-only source and sealed-artifact audit.  No raw device file, raw
base RINEX byte, truth payload, MAT payload, solver, accuracy evaluator, or
Kaggle/token resource was read or run for Phase107.  The base file sizes,
hashes, paths, and route metadata below are carried from sealed Phase64/65/73
records; they were not re-statted, re-hashed, opened, or copied in this audit.

## Scope and decision

The audit asks whether the Phase82 base-backed direct-quality recipe can be
made source-parity compatible with the sealed Phase101 C7/D/QR pipeline using
only a raw base RINEX processed in the same native C++ process.  A base
station coordinate is permitted only when it comes from the raw RINEX header
or official station metadata.  A handset coordinate, result coordinate,
device-WLS coordinate, PDC coordinate, or any other precomputed coordinate is
forbidden.

The source/data candidate is reproducible for the four frozen train routes
listed in the sealed Phase64/65 records.  It is not established for arbitrary
competition/test inference routes.  The exact Phase101-plus-base command is
*not currently executable*: the pinned native application explicitly rejects
base flags at the Phase93/101 meter-state handoff admission guard
(`apps/native/gnss_fgo_imu_no_base.cpp:838-846`).  Therefore no Phase107 raw
run is authorized.  The current fail-closed path remains the Phase106
diagnostic.  A single future candidate is frozen in the companion JSON as a
guard-only admission extension; it is not an algorithm or parameter change.

## Pinned evidence

| Item | Path | SHA-256 | Use |
| --- | --- | --- | --- |
| Phase82 direct-quality aggregate | `docs/use_cases/records/smartphone_r5_phase82_phase80_direct_quality_accuracy_result_v1.json` | `39c5a38b7e3e61fda0306582fffb961dad3e6fb0bd49fcfe70e66674e2c90873` | historical scalar reference only |
| Phase80 source-exact freeze | `docs/use_cases/records/smartphone_r5_phase80_source_exact_direct_p_quality_freeze_v1.json` | `9ebdde21fe7fd955029243d3bdbba3a9f9d3832388d67480760394177c8f685e` | recipe/provenance |
| Phase73 base miss-mask structural result | `docs/use_cases/records/smartphone_r5_phase73_source_exact_pseudorange_miss_mask_structural_result_v1.json` | `8493d16b3d2c5ce8b09b65ab0dada6903fcbac01257fb8db7977cb41c85577fd` | four-route source/base structural evidence |
| Phase65 base manifest | `docs/use_cases/records/smartphone_r5_phase65_native_base_pseudorange_compensation_manifest_v1.json` | `1306480f6b7fb839e55309e9c2c77b41f2ac0a1baf9f6971949612fd485d2255` | sealed base member metadata |
| Phase64 recovery result | `docs/use_cases/records/smartphone_r5_phase64_base_preflight_policy_recovery_result_v4.json` | `70e620b6726c089b6ebc65d494d45405860e37e2a072b34239fb9c93673f8e80` | route/member/header policy evidence |
| Phase101 structural result | `docs/use_cases/records/smartphone_r5_phase101_epoch_clock_vector_structural_result_v1.json` | `5ffb15c04fbdeac33d907f4ed3a1fdacb031508c6e3db8d7ef5b230dcab8d36a` | no-base C7/D/QR baseline |
| Phase101 structural manifest | `docs/use_cases/records/smartphone_r5_phase101_epoch_clock_vector_structural_manifest_v1.json` | `ddda124d762e7f15854c65da35b322eaee7a7547fb87f6847f1b4965d3215b9b` | exact selector and input contract |
| Phase106 audit fallback | `docs/use_cases/records/smartphone_r5_phase106_gnss_first_regression_audit_v1.md` | `ed8997df27e8c4b481f678a4824685db20295c8a214d7940a76c9423c2860e7c` | active diagnostic fallback |
| Phase106 diagnostic freeze | `docs/use_cases/records/smartphone_r5_phase106_gnss_first_regression_diagnostic_freeze_v1.json` | `2e3fe2727f5a121115adc4de54623f30d87b934b95653621edb5225a0a9598af` | fallback contract |

Source pins inspected:

| Item | Path | SHA-256 |
| --- | --- | --- |
| Official GNSS source | `output/reproducibility-cache/gsdc2023/fgo_gnss.m` | `5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3` |
| Official GNSS/IMU source | `output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m` | `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3` |
| Official correction helper | `output/reproducibility-cache/gsdc2023/functions/correct_pseudorange.m` | `b0536ccff478b0aff253448ffb7a203c715b8064dd8dc85898e38f1f05d0441e` |
| Native application | `apps/native/gnss_fgo_imu_no_base.cpp` | `c81381b0a8fa8cea5c101fc5d54e6364173c0e8cffb83f02eac18c8b6cafde5b` |
| Native problem builder | `src/algorithms/fgo_problems.cpp` | `e7607ce8f0fe27fd382a01b5b6f147b027b7bf51453e920eae458c86ddac0c17` |
| Native base model | `src/algorithms/base_pseudorange_compensation.cpp` | `f3c3c859793c4bed9c7ae93825d32565f20eab0badb09412a4fab2a5930c9817` |
| Native base model header | `include/libgnss++/algorithms/base_pseudorange_compensation.hpp` | `a182091ef47af8692e3f4dfb0d714cf365f5b24bc6927a8204d05648248e026b` |
| RINEX reader | `src/io/rinex.cpp` | `92f683e34b4052f8d105f69fc4bf4397ca990be3b5a6b220181a56423f465533` |
| GTSAM backend | `src/algorithms/fgo_gtsam_backend.cpp` | `781d65e34826ec08e04d1dfdac0ab10b2ed9409726078465f3020b57afac4baa` |
| Native factor definitions | `src/algorithms/fgo_gtsam_internal.hpp` | `cc6327e36800afa8217e5a834260adc81c0a74a5aefbfc956bb4679c8d86105f` |

## Sealed base availability and station provenance

The Phase65 manifest records one exact materialized `base.obs` member for each
of the four frozen train route IDs.  This table is metadata copied from that
seal, not a current filesystem probe:

| Route | Sealed relative path | Bytes | SHA-256 | Observed dt / moving mean | Header approximate XYZ (m) |
| --- | --- | ---: | --- | --- | --- |
| MTV-A | `output/smartphone-r5/phase63-settings-integrity-recovery-v1/routes/2021-03-16-18-59-us-ca-mtv-a__pixel5/base.obs` | 10,708,536 | `380b8ff9091344fb756697e27f0983d9a0ba2cf0c201b96849bfc1ecc1af0e52` | 1 s / 151 | (-2703115.921, -4291767.2078, 3854247.9066) |
| MTV-H | `output/smartphone-r5/phase63-settings-integrity-recovery-v1/routes/2021-08-24-20-32-us-ca-mtv-h__pixel5/base.obs` | 11,854,125 | `4d3e37cbe0347fa56216db54ede9e0f30731885f337f1653ab5a86afb2bb2150` | 1 s / 151 | (-2698116.8365, -4301328.0461, 3847285.9307) |
| LAX-T | `output/smartphone-r5/phase63-settings-integrity-recovery-v1/routes/2022-04-01-18-22-us-ca-lax-t__pixel5/base.obs` | 719,969 | `d731e0e8a7ba4396d62340c85b6238e66c50c6a349621f9dcea0ea6885fd4cfe` | 15 s / 11 | (-2507798.7984, -4676369.6918, 3526890.8008) |
| MTV-U | `output/smartphone-r5/phase63-settings-integrity-recovery-v1/routes/2023-03-08-21-34-us-ca-mtv-u__pixel5/base.obs` | 5,950,275 | `aedb7a39e7b6612ea97964b363c74cd2c10318255b7f2287f4720d18e71803e6` | 1 s / 151 | (-2698116.8365, -4301328.0461, 3847285.9307) |

Phase64's recovery policy established, from its sealed metadata, that all
four members had settings/member identity, time overlap, finite approximate
XYZ, and materialized hash/byte agreement.  It also records that the actual
header version was `3.03` while the settings selection was `RINEX=V2`, and
that header station ID and header interval were absent.  The accepted policy
therefore uses the settings filename contract, observed base interval, and
finite header approximate XYZ.  This is enough to establish historical
four-route availability under that policy, but not generic competition/test
route coverage.  No current existence claim is made because checking the
filesystem or opening these members would violate this audit's read boundary.

The station coordinate is not a phone/result coordinate.  The official helper
`correct_pseudorange.m:8-24` obtains a year/Base-selected mean XYZ from
`base_position.csv` and adds the matching ENU `base_offset.csv` value.  The
native same-run implementation instead reads `RINEXHeader.approximate_position`
(`gnss_fgo_imu_no_base.cpp:6295-6300`); the RINEX parser exposes marker,
approximate XYZ, and antenna delta (`src/io/rinex.cpp:1095-1111`), but the
current base model consumes only approximate XYZ.  The official station CSV
payloads were not read here.  This header-versus-metadata reference mismatch,
including the currently ignored antenna delta, is an explicit parity risk;
it is not permission to supply a precomputed handset coordinate.  A future
run must fail closed unless the raw header reference is finite, Earth-valid,
and the allowed station-reference contract is exact.

## Phase82 base-backed recipe versus Phase101 baseline

The Phase80 freeze preserves the Phase78 flags
`--native-base-pseudorange-compensation`, `--native-base-rinex`,
`--native-base-rinex-sha256`, and
`--native-base-pseudorange-source-miss-mask`, in addition to direct observable
quality and signal-bias states.  Thus the Phase82 direct-quality candidate is
base-backed; it is not a clean no-base ablation.  Its sealed aggregate reports
status `no-go-phase82-accuracy-gates`, candidate macro scalar
`1.7643320853515223 m` versus the Phase43 control `3.536446745838451 m`, and
the decision to preserve the Phase43 champion while retaining the Phase80/81
experiments.  The Phase73 structural seal nevertheless proves that the
base-derived correction path was finite/material for all four frozen routes:
the correction absolute p50 values were 7.0937, 3.9124, 6.1548, and 2.7450 m
for MTV-A, MTV-H, LAX-T, and MTV-U respectively (sealed values; no raw rows
were reopened).

Phase101 is the relevant no-base C7/D/QR comparator.  Its sealed structural
result reports the following same-run observations:

| Route | GNSS-first epochs / accepted | GNSS-first cost | C0D factors | C7 / D finite | Main QR accepted | Main cost | Main factors / values |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| MTV-A | 2159 / 157 | 5,618,000.7863 -> 21,298.1127 | 2158 | 15,113 / 2,159 | 12 | 76,503,985.9706 -> 34,843.2354 | 120,093 / 10,795 |
| LAX-T | 1466 / 270 | 6,419,784.8566 -> 12,818.2893 | 1465 | 10,262 / 1,466 | 12 | 164,902,215.2281 -> 20,386.0266 | 61,082 / 7,330 |

The Phase101 seal says base use was false, C7 dimension was 7, D units were
metres/second and initialized from retained `EpochSeed.receiver_clock_drift_mps`,
CCDD sigma was 0.1 m, global ISB count was zero, and the main selected
`MULTIFRONTAL_QR`/`EliminateQR`.  These are invariants of the proposed base
candidate, not results of a Phase107 run.

## Official and native correction paths

| Stage | Official source | Native source | Parity finding |
| --- | --- | --- | --- |
| Base stream selection | `correct_pseudorange.m:5-7` intersects base observations with rover satellites. | `base_pseudorange_compensation.cpp:255-305` builds same-satellite/same-signal streams from the raw base RINEX and broadcast nav. | Satellite/signal matching is structurally compatible. |
| Station reference | `correct_pseudorange.m:8-24` uses official station mean XYZ plus ENU antenna offset. | `gnss_fgo_imu_no_base.cpp:6295-6300` uses raw header approximate XYZ; antenna delta is currently not consumed. | Not bit-exact; future candidate must use allowed header reference only and fail closed on unresolved reference equivalence. |
| Base residual | `gt.Gsat(...); obsb.residuals(satb)` in `correct_pseudorange.m:17-24` supplies the helper residual. | `base_pseudorange_compensation.cpp:215-252` explicitly uses earth-rotation-corrected geometry and `P + satellite_clock_m - ionosphere - troposphere - group_delay - geometric_range`. | Native terms/sign are explicit; equivalence to the opaque `gt.Gsat` helper and all timing details is not proven line-by-line. |
| Smoothing | `smoothdata(...,"movmean", prm.base_movmean_n_1s/15s)` in `correct_pseudorange.m:26-29`. | Native centered moving mean with frozen 151/11 samples. | Window contract matches the sealed route metadata. |
| Time transfer | `interp1(obsb.time.t, pc_, obsr.time.t, "linear")` in `correct_pseudorange.m:31-34`. | `correctionAt` requires in-domain exact stream/time and linearly interpolates; no extrapolation/fill (`base_pseudorange_compensation.cpp:307-339`). | Native source-miss-mask is needed to preserve official missing/out-of-domain factor omission. |
| Application | Official subtracts `pc` from rover `resPc` in both `fgo_gnss.m:72-78` and `fgo_gnss_imu.m:85-95`. | Native applies positive `pc` subtraction to the existing pseudorange factors after problem construction (`gnss_fgo_imu_no_base.cpp:6672-6792`). | Applying the same corrected factor vector to the GNSS-first copy and main handoff can preserve C7/D/QR; no new base factor family is required. |

The native model's same-process sequence is also source-preserving: it reads
the base RINEX/header, builds the correction model, builds the raw
pseudorange problem, applies the exact-stream/in-domain miss mask and
correction, then passes the resulting problem to the GNSS-first Point3/
velocity solve.  The current code consequently does not need a base-derived
phone seed.  The Phase101 handoff carries the resulting optimized C7/D and
same-run position/velocity; those state, unit, sigma, filter, LM, and QR
contracts are to remain unchanged.

## Exact executable blocker

The Phase101 selector set is:

```text
--native-source-clock-c0d-gnss-first-meter-state-handoff
--native-source-clock-c0d-epoch-vector-parity
--native-source-clock-c0d-phase99-main-multifrontal-qr-solver
```

At `apps/native/gnss_fgo_imu_no_base.cpp:838-846`, the first selector rejects
any of the following: native base compensation, additional-band preservation,
the source miss mask, a base RINEX path, or a base RINEX hash.  The rejection
is before optimization and says that base/external coordinate inputs are
forbidden.  This is an intentional fail-closed policy in the pinned Phase101
implementation, not a solver failure or evidence that base corrections are
numerically invalid.  Consequently the current executable cannot realize the
requested raw-base-plus-Phase101 combination, and no raw invocation is
permitted by this audit.

The only admissible future boundary is a default-off guard admission change
that allows the already implemented native raw-base model and source miss
mask to coexist with the Phase101 selectors.  The change must not alter the
base equations, station-coordinate source, C7/D topology, handoff, solver,
ordering, filter, robust loss, sigma, LM schedule, initialization, fallback,
or output publication.  It must require exact sealed route path/hash/bytes,
finite Earth-valid header XYZ, observed 1/15-s interval and 151/11 window,
exact same-satellite/signal in-domain coverage, and no base CSV/phone/result
coordinate input.  If any reference or route preflight condition is missing,
it must fail closed and retain the Phase106 diagnostic fallback.

## Single candidate and execution boundary

Candidate ID: `phase107-raw-base-rinex-source-exact-pseudorange-phase101-c7d-qr-v1`.

This is a future opt-in source-admission candidate, not an executed result.
Its intended operation is:

1. Read only the exact raw route device GNSS/IMU, broadcast nav, and sealed
   route base RINEX in the same native process.
2. Use only `RINEXHeader.approximate_position` for the station reference;
   never use a phone/result/WLS/PDC/precomputed coordinate, and never read
   `base_position.csv` or `base_offset.csv` at runtime.
3. Build same-satellite/same-signal base residual streams with the existing
   native broadcast corrections, centered 151/11 smoothing, and in-domain
   linear interpolation.  Apply the positive correction subtraction and
   source miss mask to raw code factors before both GNSS-first and main graph
   use.
4. Keep Phase101 C7 (dimension 7), raw retained D initialization in m/s,
   CCDD sigma 0.1 m, exact-key/full/finite C/D handoff, same-run position and
   velocity, Phase99 QR main, all filters/sigmas/LM settings, and legacy
   default behavior unchanged.

The future structural contract is one run per each of MTV-A, MTV-H, LAX-T,
and MTV-U, only after a separate implementation qualification and one-shot
authorization.  It must record raw/base path/hash/byte provenance, header
reference and interval checks, factor-retention/miss-mask counts, C7/D
coverage, GNSS-first/main progress and strict cost decrease, QR selection,
finite Earth-valid output, and no fallback.  Truth, MAT, accuracy scoring,
Kaggle, solution publication, and generic competition/test routes remain
outside this candidate.  Since generic competition base availability is not
sealed, no competition inference authorization can be inferred from the four
train members.

## Read accounting and disposition

| Resource | Phase107 audit reads/runs |
| --- | ---: |
| Raw device GNSS/IMU/nav payloads | 0 |
| Raw base RINEX bytes/headers/stat/hash | 0 |
| Truth payloads | 0 |
| MAT payloads | 0 |
| Phone/result/precomputed coordinate payloads | 0 |
| Native solver/accuracy subprocesses | 0 |
| Kaggle/token resources | 0 |

Disposition: source artifacts and sealed historical base members support a
four-route future candidate, but the current Phase101 command is blocked by
the explicit base-input guard and generic competition coverage is unproven.
No Phase107 raw run is authorized.  Until a separately authorized guard-only
implementation is qualified, retain the Phase106 diagnostic freeze as the
only next step.
