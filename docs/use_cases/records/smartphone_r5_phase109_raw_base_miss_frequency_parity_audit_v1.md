# Phase109 raw-base miss/frequency parity audit

Status: sealed read-only source and metadata audit.  This audit did not open,
hash, stat, or run against raw device GNSS/IMU/navigation payloads, raw base
RINEX bytes, truth payloads, a solver, an accuracy evaluator, or Kaggle.  The
Phase107 counts and Phase108 scores below are copied from their sealed JSON
artifacts; no payload is re-read.  Official MATLAB files are source code only.

## Question and disposition

The question is whether the Phase107 pseudorange misses are explained by the
native reader's primary/secondary-only base stream selection, and whether the
existing additional-frequency-band selector can be admitted alongside the
Phase101 C7/D and Phase99 QR recipe without changing a correction equation or
applying a correction twice.

The evidence supports exactly one *guard-only* candidate: admit the existing
`--native-base-pseudorange-preserve-additional-frequency-bands` selector with
the already sealed Phase101 raw-only meter-state recipe, Phase107 native
raw-base compensation, and the Phase107 source miss mask.  This is an
admission of an existing RINEX selection path, not a new frequency mapping,
correction equation, interpolation policy, factor family, state topology,
noise value, filter, initialization, LM setting, or QR change.  The sealed
aggregate does not contain per-signal/band counts, so it does **not** prove
that every MTV-A missing row is an additional-band row.  The next authorized
structural run must measure that attribution and fail closed if the selected
band path is empty, ambiguous, or not applied exactly once.

No raw execution is authorized by this audit.  The companion freeze JSON
records the one candidate and its future qualification boundary.

## Pinned sealed evidence

| Item | Path | SHA-256 | Relevant fact |
| --- | --- | --- | --- |
| Phase101 structural result | `docs/use_cases/records/smartphone_r5_phase101_epoch_clock_vector_structural_result_v1.json` | `5ffb15c04fbdeac33d907f4ed3a1fdacb031508c6e3db8d7ef5b230dcab8d36a` | no-base C7/D handoff and Phase99 QR baseline |
| Phase101 structural manifest | `docs/use_cases/records/smartphone_r5_phase101_epoch_clock_vector_structural_manifest_v1.json` | `ddda124d762e7f15854c65da35b322eaee7a7547fb87f6847f1b4965d3215b9b` | exact raw-only selectors and route contract |
| Phase107 structural result | `docs/use_cases/records/smartphone_r5_phase107_raw_base_source_parity_result_v1.json` | `e7eb3f1d2670672414744a100dc1e2fc3e0b019ae7efc0dbfef1518300ce8d6e` | raw-base factors, misses, and exact-once handoff |
| Phase107 manifest | `docs/use_cases/records/smartphone_r5_phase107_raw_base_source_parity_manifest_v1.json` | `b46a4bfa23ac1604d9e51487734be18262f027612eede40fdbcba15fa75c2a9c` | sealed raw/base lineage and flags |
| Phase107 authorization | `docs/use_cases/records/smartphone_r5_phase107_raw_base_source_parity_authorization_v1.json` | `ba975dd93f11cd64fdaec6cc6d57a080367fc2e6a04ee58513cb3a0b0542eb56` | one-shot structural execution boundary |
| Phase108 accuracy result | `docs/use_cases/records/smartphone_r5_phase108_phase107_solution_accuracy_result_v1.json` | `9f9a2528745c06ba06e774139f56892bcea0ef208a3d064e4b70042e48f29a9d` | sealed aggregate only; no new accuracy read |

Phase107 is the relevant raw-base observation.  Its sealed route telemetry is:

| Route | Original adopted code rows | Retained finite corrected rows | Missing exact stream | Out of correction domain | Nonfinite correction | Preserve additional bands |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| MTV-A (`2021-03-16-18-59-us-ca-mtv-a/pixel5`) | 57,281 | 43,259 | 12,894 | 1,128 | 0 | false |
| LAX-T (`2022-04-01-18-22-us-ca-lax-t/pixel5`) | 30,798 | 30,664 | 0 | 134 | 0 | false |

The counts close exactly: MTV-A loses `12,894 + 1,128 = 14,022` rows and
LAX-T loses `0 + 134 = 134`.  The source miss mask reports exact factor-count
consistency, unchanged retained epoch indices, finite retained corrections,
and the sign `P_rover_corrected_m = P_rover_raw_m - pc_m`.  MTV-A therefore has
a large exact-stream miss component; LAX-T has no such component.  The
out-of-domain category is independently identified and is not evidence of a
frequency-band mismatch.

Phase108's already sealed truth-only aggregate was no-go at the strict gate:
MTV-A `1.1396560717187856 m`, LAX-T `0.8241679860216076 m`, macro
`0.9819120288701966 m` versus the strict `0.782 m` limit.  This is a
historical outcome, not a frequency attribution: Phase108 has no per-signal
miss telemetry and this audit performs no truth or solution read.

## Official L1/L5 source path

The pinned official files are:

| Source | SHA-256 | Source-level behavior |
| --- | --- | --- |
| `output/reproducibility-cache/gsdc2023/fgo_gnss.m` | `5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3` | `FTYPE = ["L1","L5"]`; correct each available field before factor insertion |
| `output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m` | `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3` | same L1/L5 correction before GNSS/IMU graph construction |
| `output/reproducibility-cache/gsdc2023/functions/correct_pseudorange.m` | `b0536ccff478b0aff253448ffb7a203c715b8064dd8dc85898e38f1f05d0441e` | same-satellite base intersection, residual smoothing, in-domain linear time interpolation |
| `output/reproducibility-cache/gsdc2023/functions/sysfreq2sigtype.m` | `5ec1d299e04b45f604921cbae3b1fac8f78f3f76a0152496d86011a32f1d0079` | L1 maps GPS/GLO/GAL/BDS to signal slots 0..3; L5 maps GPS/GAL/BDS to 4..6; other systems use 7 |
| `output/reproducibility-cache/gsdc2023/functions/gnsslog2obs.m` | `665ce43d1fc3c5a9c3b3a6fd76f81539126e3ab897fbba484ab0ccd18964acff` | raw carrier frequency is classified into L1/L2/L5/... and a system/frequency code |

The official sequence is explicit in both FGO entry points: for each `f` in
L1/L5, call `correct_pseudorange`, subtract `pc` from `resPc`, and later add a
`PseudorangeFactor_XC` only when the corrected value is not NaN.  The helper
uses `obsb.sameSat(obsr)`, smooths the base residual with the frozen 1-Hz or
15-Hz moving window, and calls `interp1(..., "linear")` at rover times.  With
no extrapolation argument, an out-of-domain interpolation is NaN and the
factor is omitted.  Official `gnsslog2obs.m` recognizes L1/L5 among the raw
frequency classes, while `sysfreq2sigtype.m` supplies the graph signal slot.

This establishes the relevant source parity contract: correction is
frequency-field-specific, satellite-specific, and time-interpolated; a
missing/nonfinite corrected pseudorange is not replaced by an endpoint or a
different signal.

## Native selection, key, and time mapping

The pinned native source files are:

| Source | SHA-256 | Role |
| --- | --- | --- |
| `apps/native/gnss_fgo_imu_no_base.cpp` | `0a14710768f107d025ef2505450ca85c545e3c0a47e52191f685094838a9c51f` | option guard, base model setup, one-pass factor correction and telemetry |
| `src/io/rinex.cpp` | `92f683e34b4052f8d105f69fc4bf4397ca990be3b5a6b220181a56423f465533` | primary/secondary and optional per-band observation selection |
| `include/libgnss++/io/rinex.hpp` | `571ac4e0e3e301e6a8b865ea89caaae3acd40dc8b25f4f1bdb3ede751e9e6e5d` | default-off `setPreserveAdditionalFrequencyBands` contract |
| `src/algorithms/base_pseudorange_compensation.cpp` | `f3c3c859793c4bed9c7ae93825d32565f20eab0badb09412a4fab2a5930c9817` | same-satellite/signal model, smoothing and in-domain interpolation |
| `src/algorithms/source_pseudorange_miss_mask.cpp` | `5f89b471ea4c1753901f396eedb04baeef1b8811d33f270eadb38620fe13094e` | exact stream/time/finiteness retention and miss taxonomy |
| `include/libgnss++/algorithms/source_pseudorange_miss_mask.hpp` | `0fc470feeffa7a223dace8c5b4119760a26805325070acda22f786d5379e58b5` | report contract for the miss mask |

The native mapping is not keyed by an unnormalized RINEX observation string.
The reader maps an observation's constellation, band, tracking code, and (for
GLONASS) channel context to a `SignalType`.  A base correction stream is then
keyed by `(SatelliteId{system,prn}, SignalType)`.  The source miss mask tests
that exact pair, uses the retained factor's epoch index to obtain its
`GNSSTime`, and asks `correctionAt(time, satellite, signal)` for an in-domain
linear value.  It does not match by epoch alone, by frequency alone, or by
satellite alone.

The native RINEX reader always emits its selected primary and secondary
signals.  When `preserve_additional_frequency_bands` is false it returns
without constructing the optional per-band selections; when true it selects
one supported observation per additional band and appends those selections.
The existing signal policy maps, for example, GPS band 1/2/5 to GPS L1CA/L2C/
L5, Galileo band 1/5/6/7/8 to E1/E5A/E6/E5B, and BeiDou bands to its B1/B2/B3
signal classes.  This is a source-level opportunity to make the base stream
set cover more raw rover `SignalType` keys, but the sealed Phase107 aggregate
does not identify which signals produced its 12,894 MTV-A misses.

The mapping risks that remain observable only in a future diagnostic run are:

* multiple RINEX tracking codes in one supported band are intentionally
  collapsed to one selected observation;
* different raw frequency classes can map to the same native signal class in
  a constellation-specific policy; and
* a base stream can be present but still be outside the base sample time
  interval.

None of these is permission to invent a new map or extrapolate.  The candidate
must expose selected-band rows/streams and miss counts by signal so these
possibilities are distinguished rather than inferred from the aggregate.

## Miss taxonomy: facts versus inference

| Category | Sealed fact | Source explanation | Frequency attribution available now |
| --- | --- | --- | --- |
| Missing exact stream | MTV-A 12,894; LAX-T 0 | `hasStream(satellite, signal)` is false for the retained factor | Not per-signal; additional-band selection is a supported hypothesis only |
| Out of correction domain | MTV-A 1,128; LAX-T 134 | exact stream exists but epoch is invalid or `correctionAt` rejects time outside first/last sample | Time-domain failure, not a band count |
| Nonfinite correction/corrected value | 0 on both routes | finite correction and corrected pseudorange checks reject these rows | No evidence of a frequency failure |
| Retained | MTV-A 43,259; LAX-T 30,664 | exact stream, valid time, finite correction, finite subtraction | Exact retained order/epoch indices are sealed |

Therefore the only strong aggregate conclusion is that MTV-A's dominant
category is an exact `(satellite, SignalType)` stream miss while LAX-T's miss
category is exclusively time-domain.  The source supports testing the
existing additional-band selector for MTV-A, but no sealed artifact proves
that the selector accounts for all or any particular fraction of 12,894.
Enabling it on LAX-T may be a no-op and must be reported as such, not treated
as a success by assertion.

## Double-correction and guard audit

There is no evidence of two independent base correction passes in the pinned
native path.  The native code has a source-miss-mask branch and an alternate
direct per-factor branch, selected mutually exclusively.  The source-mask
branch subtracts the positive correction once, swaps the retained factor
vector, and preserves factor epoch identity.  The corrected shared problem is
then used for the GNSS-first copy and the main handoff.  Doppler, TDCP, IMU,
and SPP paths are unchanged.

The current fail-closed guard is the actual blocker.  The command parser
rejects `source-miss-mask` together with
`preserve-additional-frequency-bands` (`gnss_fgo_imu_no_base.cpp:558-563`).
The Phase101 meter-state admission additionally requires compensation plus
source miss mask, nonempty base path/hash, and `!preserve` (`:838-861`).
Thus the current Phase107 command cannot enable the existing additional-band
reader under Phase101, even though the reader and model already support it.

The candidate boundary is only to narrow that predicate: allow
`preserve=true` when and only when all Phase101 selectors, the exact Phase107
raw-base path/hash contract, compensation, and source miss mask are present.
The future implementation must retain one correction pass on the shared
factor vector and must report `correction_application_pass_count == 1` and
`source_model_build_count == 1`.  Any fallback, second subtraction, alternate
signal map, out-of-domain fill, or separate correction of the GNSS-first/main
copies is a fail-closed failure.

## Candidate comparison and single freeze decision

Three possible explanations/actions were compared against the sealed/source
evidence:

1. **Chosen:** admit the existing additional-frequency-band reader under the
   exact Phase101 + Phase107 source-miss-mask guard.  It directly tests the
   only source-supported explanation for the large exact-stream category while
   preserving all equations and state/solver settings.
2. **Rejected:** add or alter a constellation/frequency mapping, or create a
   new L1/L5/additional-band factor.  The sealed data lacks per-band proof, and
   this would change signal topology or equation semantics rather than test an
   existing path.
3. **Rejected:** extrapolate, endpoint-hold, change smoothing, or alter time
   alignment to recover out-of-domain rows.  This contradicts both official
   `interp1` behavior and the sealed native no-extrapolation contract, and it
   cannot explain the exact-stream category.

The sole frozen candidate is
`phase109-phase101-raw-base-existing-additional-frequency-band-guard-v1`.
It is opt-in and default-off.  It keeps C7, raw retained D initialization,
meter units, CCDD sigma 0.1 m, source-miss filtering, all GNSS/IMU/carrier
equations, filters, sigmas, LM schedule, initialization, and Phase99
`MULTIFRONTAL_QR`/`EliminateQR` unchanged.  It admits no base-derived phone
coordinate, MAT/PDC/precomputed coordinate, truth, or Kaggle input.

The candidate is supported as a source-admission experiment, not as a proven
accuracy fix.  A future two-route or four-route structural authorization must
record at least:

* selected-band observation rows/streams and per-signal maps;
* original, exact-stream-matched, out-of-domain, nonfinite, and retained rows
  by signal/frequency class;
* exact rover/base satellite-signal-time key alignment;
* one model build and one correction application pass;
* unchanged C7/D full finite handoff, GNSS-first/main progress and strict cost
  decrease, QR selection, finite Earth-valid output, and no fallback.

This audit does not authorize that execution, a truth evaluation, or a
solution publication.  It also does not establish base availability for
arbitrary competition/test routes; only the already sealed Phase107 route
lineage may be considered by a later authorization.

## Read accounting

| Resource/action in this Phase109 audit | Count |
| --- | ---: |
| Official/native source files read | nonzero, source-only |
| Sealed JSON metadata read | nonzero, aggregate/lineage only |
| Raw device GNSS/IMU/navigation payload reads | 0 |
| Raw base RINEX byte/header/stat/hash reads | 0 |
| Truth payload reads | 0 |
| MAT/precomputed phone-coordinate/PDC reads | 0 |
| Native solver invocations | 0 |
| Accuracy evaluator invocations | 0 |
| Kaggle/token access | 0 |
| Reruns/fallbacks | 0 |

Disposition: one guard-only existing-selector candidate is frozen in the
companion JSON; no Phase109 raw or truth execution occurred.
