# Smartphone R5 Phase134 native Phase131 zero-telemetry forensic audit

- Execution label: `Luna Max`
- Audit date: `2026-09-04`
- Repository: `/home/sasaki/workspace/rtklib_v2_ws/gnssplusplus-library`
- Branch: `feat/rtk-smartphone-performance-pr`
- HEAD at audit start: `e95c000245a74e550e1989334e51c726f456e93f`
- Worktree at audit start: clean.
- Scope: read-only audit of the sealed Phase133 structural result, its
  already-produced route logs and normalized summaries, the Phase133 runner,
  and the native Phase131/base-correction source and compiled CLI.

This audit did not open a phone GNSS/IMU payload, broadcast-navigation
payload, raw-base RINEX payload or header, solution row, truth row,
MAT/PDC/precomputed-coordinate artifact, or Kaggle/token resource.  It did
not launch the native process, run a synthetic solver, or rerun a route.  The
ignored route files were read only as existing logs/metadata.  Solution
artifacts were not opened; the sealed result's opaque hash/row metadata was
not interpreted as coordinates.

## Disposition

Exactly one candidate is frozen in the companion JSON:

`phase134-native-phase131-summary-diagnostics-bridge-v1`

The candidate is a native summary-wiring correction.  It copies the already
computed Phase131 canonical-correction diagnostics from the
`BasePseudorangeCompensationReport` into the `FGOProblemDiagnostics` instance
that emits the top-level native `phase131_canonical_correction_band_key`
object.  It changes no observation, correction, factor, state, equation,
unit, sigma, filter, robust kernel, QR/LM, initialization, output, or default
behavior.  It also does not relax the Phase133 gate: the existing positive
canonical-row/selected-stream predicate remains required.

Implementation and a fresh raw authorization remain outside this audit.

## Authoritative evidence and hashes

| item | value |
|---|---|
| Phase133 sealed result commit | `e95c000245a74e550e1989334e51c726f456e93f` |
| Phase133 result JSON SHA-256 | `e786da3c84b0577b3963b340724f39e1a750040a0f30adcc1dbf25a0925cc17c` |
| Phase133 result MD SHA-256 | `4c662f7082966e7f8263c0009f1deabc335dc6f84a79d29eb54feeee98a992c5` |
| Phase133 authorized runner commit | `c69ed14c96a13fefbd9c40f783051c98e5483a4a` |
| Phase133 authorized runner SHA-256 | `ca5f0d3c0a8ec8b2c0f380cda3f1d7ed37b7a0525a04073a9019b87600b85be7` |
| Phase133 independent authorization commit | `2b5f3633d20457ee5a335c2d6d447c727374a26b` |
| Phase133 authorization SHA-256 | `029300f689fd738365e6707a66dd175b4cf642e0e9e5fce5e4c8673d5ca9f524` |
| native app source SHA-256 | `6fcee581af70535b8af09a674a234352abaaf83a34b722d4e91577abaf352170` |
| FGO problem-builder source SHA-256 | `9f75eda838a6a885657fb78d038733d6e1e027794c869c7a26b2dd610ff96aaa` |
| base model source SHA-256 | `5ec8c119a55a8e974394531a7585a6ce094756c127b048d61fba330991b48ba2` |
| canonical-key header SHA-256 | `b9e464d16551e547f9b1bc298b7c6a90f91f5b0038d97b46625ff56cdf497123` |
| target binary SHA-256 | `ed24b57b29f679123c3a52dbb04fad9b979092564abc4a35905bdb2abfaa0c8e` |

The source files are unchanged from the Phase133 execution.  The compiled
CLI was inspected only by hash and static strings; no binary invocation was
made during this audit.

Existing ignored route metadata/log hashes used for this audit:

| route | stdout SHA-256 | stderr SHA-256 | normalized summary SHA-256 |
|---|---|---|---|
| MTV-A | `b35f4d12f5a8cedf60b343804156d3faf4db6fe0798748495a023dda2d53f5e1` | `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855` | `71ac73292d7c95a363b45ddfe74e1e0dd9461571d66380573f255e8cdc4ba69f` |
| LAX-T | `25e4f2865c0c10f79f3c904da276ae56ed7f8ade48aac5e7cfe45858710ad06` | `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855` | `67b54bcb1f1ea8aa57fb97b3d447d8c82879e5f5e19a02bc23ebfe2c2df3fa0f` |

The normalized summary files are not a second native run.  They were written
over the native summary path by the Phase133 wrapper after parsing it.

## Sealed Phase133 observation

The Phase133 result is structurally `no-go-phase133-runner-native-selector-boundary-structural`,
but both native processes returned `0`.  The native graph and finite progress
telemetry were present:

| route | native return | GNSS-first cost | main cost | main iterations | base finite corrected rows | native top-level canonical rows / selected streams / resolver calls |
|---|---:|---|---|---:|---:|---|
| MTV-A | `0` | `5009591.0291709 -> 16104.433292069118` | `122809034.24737301 -> 28266.35349384695` | `12` | `43244` | `0 / 0 / 0` |
| LAX-T | `0` | `6466234.225070638 -> 12279.574633102011` | `143982373.9341658 -> 17995.990758732205` | `12` | `30653` | `0 / 0 / 0` |

The sealed base report also records, for MTV-A/LAX-T respectively,
`matched_factor_rows=44350/30789`, one correction application pass,
`correction_applied_exactly_once=true`, and all Phase126 A/B/C markers true.
The Phase131 selector/configuration flags are true and valid in that same
base report.  Thus the zero top-level Phase131 values are the sole failed
structural predicate; they do not explain the native process return or the
positive base-correction accounting.

The native stdout is intentionally compact and contains only the graph-level
completion line:

```text
MTV-A: native no-base FGO: epochs=2159 output=2158 imu_intervals=2158 graph_factors=106055 graph_values=10795 fallback=no
LAX-T: native no-base FGO: epochs=1466 output=1465 imu_intervals=1465 graph_factors=60937 graph_values=7330 fallback=no
```

The normalized summaries retain no nested native base report.  Therefore the
numeric native canonical counters are not recoverable from the sealed route
artifact after normalization.  This loss is an observability limitation,
not evidence that the native canonicalizer was skipped.

## Exact call graph

The selector and diagnostic values follow two different in-memory paths.

| stage | source location | audited behavior |
|---|---|---|
| argv template | `apps/commands/benchmarks/gnss_smartphone_phase133_runner_native_selector_boundary.py:240-262` | Phase131 native selector is present exactly once; Phase130 is absent |
| authorized launch | `apps/commands/benchmarks/gnss_smartphone_phase133_runner_native_selector_boundary_authorized_execute.py:347-421` | the sealed run constructs/invokes the native command once per route, parses the native summary, then overwrites the summary path with the normalized summary |
| native parser | `apps/native/gnss_fgo_imu_no_base.cpp:563-574` | `--native-phase131-canonical-correction-band-key` sets `Options::native_phase131_canonical_correction_band_key` |
| native config validation | `apps/native/gnss_fgo_imu_no_base.cpp:1655-1670` | Phase131 requires GTSAM, Phase126/127/128/129, raw-base compensation/miss-mask, and the frozen route; no selector is silently dropped |
| FGO config | `apps/native/gnss_fgo_imu_no_base.cpp:8571-8576` | the option is copied to `FGOConfig::use_native_phase131_canonical_correction_band_key` |
| base report/config | `apps/native/gnss_fgo_imu_no_base.cpp:8019-8030,8125-8162` | the option is recorded and copied to `base_config.use_phase131_canonical_correction_band_key` |
| raw-base model | `apps/native/gnss_fgo_imu_no_base.cpp:8163-8169` | `base_pseudorange_model.build(base_series, nav, base_config)` is called before FGO graph construction |
| canonical row admission | `src/algorithms/base_pseudorange_compensation.cpp:418-437` | each eligible base row is canonicalized; accepted rows increment `Diagnostics::phase131_canonical_rows` |
| canonical stream construction | `src/algorithms/base_pseudorange_compensation.cpp:542-545,611-701` | accepted residuals enter canonical candidates, are selected/merged by physical key, and a zero selected-stream set is a hard build failure |
| model diagnostics copy | `apps/native/gnss_fgo_imu_no_base.cpp:8175-8268` | model diagnostics, including all Phase131 counters, are copied into `BasePseudorangeCompensationReport` |
| FGO graph build | `apps/native/gnss_fgo_imu_no_base.cpp:8671-8672` | `buildPseudorangeProblem` constructs the typed graph independently of the base model report |
| canonical correction branch | `apps/native/gnss_fgo_imu_no_base.cpp:8682-8721` | Phase131 selects `source_pseudorange_miss_mask::applyCanonical`, which calls `hasCanonicalStream` and `correctionAtCanonical`; the legacy `apply` branch is not selected |
| correction accounting | `apps/native/gnss_fgo_imu_no_base.cpp:8738-8821` and `src/algorithms/source_pseudorange_miss_mask.cpp:53-189` | retained factors are swapped only after conservation; exactly-once and A/B/C markers are set |
| nested native report | `apps/native/gnss_fgo_imu_no_base.cpp:6083-6142` | `base_report.phase131_*` is serialized with the actual model counters |
| top-level native report | `apps/native/gnss_fgo_imu_no_base.cpp:7232-7266` | a separate `problem.diagnostics.phase131_*` object is serialized |
| wrapper gate | `apps/commands/benchmarks/gnss_smartphone_phase131_canonical_correction_structural_authorized_execute.py:809-862` | the wrapper reads only `native["phase131_canonical_correction_band_key"]`; its resolver count is `canonical_rows + canonical_rejected_rows` from that object |
| wrapper output | `...phase133_runner_native_selector_boundary_authorized_execute.py:396-421` | the native summary is parsed, then the normalized summary replaces it; only selected base fields are retained |

## Root-cause proof

### The flag was not lost

The command reached the native process with the Phase131 selector.  The
parser branch sets the option at lines 563--574.  The native summary's
top-level option field is emitted from that option at lines 5941--5942, and
the sealed base report records
`phase131_canonical_correction_band_key=true` and
`phase131_configuration_valid=true`.  The model configuration receives the
same value at lines 8155--8156.  The Phase133 result also records selector
forwarding exactly once and native return `0` for both routes.

### The canonical correction conditional was true

The Phase131 branch at lines 8695--8721 calls `applyCanonical`; the legacy
`apply` branch is the `else` path.  The sealed miss-mask accounting reports
positive retained finite corrected rows and exactly one pass.  The selected
key mode is therefore not merely a requested command token: the source path
that can only be entered with the option true is the configured correction
branch.

### The resolver was not bypassed by preprocessing or stream type

The model receives the in-memory `base_series` read by the native RINEX
reader and `nav`; it does not receive precomputed coordinates or a saved
solution.  Its canonicalization operates on typed `SatelliteId` and
`SignalType`, with certified GLONASS FCN passed separately.  The model's
source-complete build has a hard check at lines 693--700: if no finite
canonical stream is selected it sets a failure and returns `false`.  The
native caller returns `1` on a failed model build at lines 8164--8169.

In both sealed routes the native return is `0`, the source-complete A/B/C
markers are true, the base correction pass is exactly once, and finite
corrected rows are positive.  Given the source predicates, at least one
canonical row and one selected canonical stream must have been admitted in
the native model.  This is a source-backed implication from the sealed
success/correction metadata, not a claim reconstructed from solution rows.

The audit cannot recover the exact counter values because the wrapper
overwrites the native summary file after parsing it.  It can, however,
distinguish “counter not exposed” from “resolver did not run.”

### The canonical counters were not aggregated into the emitted object

`FGOProblemDiagnostics` declares Phase131 counter fields in
`include/libgnss++/algorithms/fgo.hpp:407-421`.  In the FGO problem builder,
`src/algorithms/fgo_problems.cpp:54-55` initializes only
`phase131_canonical_correction_band_key_enabled`; lines 71--82 initialize
configuration-failure metadata on invalid configuration.  A source search
finds no assignment in that builder to `canonical_rows`,
`canonical_selected_streams`, `canonical_streams`, the rejection/conflict
counters, or `failure_counts`.

In contrast, the native base model diagnostics are explicitly copied into a
different object at `apps/native/gnss_fgo_imu_no_base.cpp:8246-8268`, including
all of those fields, and the nested writer emits them at lines 6125--6142.
The top-level writer at lines 7244--7261 reads the untouched
`problem.diagnostics` fields, whose default values are zero/empty.  This is
the exact namespace/aggregation defect.

The Phase133 normalizer compounds the false negative: it reads
`native.get("phase131_canonical_correction_band_key", {})` at lines 809--812,
uses those zero fields at lines 853--862, and does not copy the nested
`base_report.phase131_canonical_rows` fields into its `base_correction`
selection at lines 951--960.  It consequently seals
`native_resolver_call_count=0` even though the native model path was admitted.

### Potential causes classified

| question | result | evidence |
|---|---|---|
| Phase131 flag lost between argv and config? | **No** | parser, config assignment, base report flag, selector forwarding, and native return all agree |
| Phase131 conditional never true? | **No** | `applyCanonical` is the option-true branch; source-complete correction accounting is positive and exactly once |
| Canonical resolver bypassed by preprocessing/stream types? | **No** | raw base `ObservationSeries` enters `Model::build`; typed canonicalization is on that path; zero selected stream would fail the native build |
| Actual route data caused zero support? | **Not supported by this audit** | route solution/raw payloads are forbidden, and the sealed normalized summary omits the original native nested counters |
| Native canonical counters reset/not aggregated? | **Proven** | model counters are copied only to `BasePseudorangeCompensationReport`; top-level `FGOProblemDiagnostics` counters are never populated but are used by the top-level writer and runner gate |

## Exactly one frozen candidate

`phase134-native-phase131-summary-diagnostics-bridge-v1` is a telemetry-only
bridge at the native summary boundary:

1. After the existing base model has built and its diagnostics have been
   copied to `base_pseudorange_report`, synchronize the Phase131 fields
   (`enabled`, configuration validity/failure, canonical/rejected/unknown
   rows, conflict/duplicate rows, stream/selected/merged stream counts, and
   failure map) into `problem.diagnostics`.
2. Perform that synchronization once before `makeSummary`; it must not touch
   `problem.pseudorange_factors`, epoch seeds, navigation, or any optimizer
   object.
3. Keep the existing nested base report and top-level schema names.  The
   existing runner then consumes the authoritative top-level counters without
   a gate relaxation or a second raw-base pass.
4. When the selector is off, the bridge must preserve disabled/zero legacy
   diagnostics.  Configuration failures remain fail-closed; no positive
   counter may be fabricated from a requested flag alone.

This is one wiring candidate, not a proposal to change the canonical key,
GLONASS FCN policy, stream admission, correction equation, miss mask, factor
topology, or solver.  A wrapper-only fallback to nested report fields was not
selected because it would leave the native top-level telemetry false and
would conceal the source namespace defect.

## Required implementation and qualification tests

Implementation remains unauthorized by this audit.  If separately authorized,
the smallest focused tests should cover:

- exact one-to-one copy of every Phase131 field/map from a synthetic base
  report into `FGOProblemDiagnostics`;
- selector-off legacy diagnostics remain disabled/zero and no graph/config
  values change;
- valid canonical stream plus rejected-row synthetic diagnostics produce the
  same values in nested and top-level summary objects;
- empty/invalid canonical build remains fail-closed and does not acquire a
  fabricated positive counter;
- the existing Python normalizer derives resolver attempts from the bridged
  top-level summary, while no solution bytes or coordinates are opened;
- source/default regression proving factor counts, correction pass count,
  A/B/C markers, C7/D/QR, IMU/TDCP, sigma, LM, and output contracts are
  untouched.

Before any new raw execution, a new binary hash, launch-free qualification,
pre-raw zero-read seal, and independent authorization must be created.  The
old Phase133 authorization and result cannot be reused after a native binary
change.  A future structural run must remain exactly MTV-A then LAX-T once,
with no fallback/rerun/repair and no truth/accuracy/Kaggle lane.

## Read accounting for this audit

| resource/action | count or disposition |
|---|---:|
| sealed Phase133 result/MD | read-only metadata |
| ignored Phase133 inventory/normalized summary/stdout/stderr | read-only metadata/logs |
| native source, FGO source, base model/header, runner source | read-only source |
| compiled CLI | hash/static strings only; no launch |
| raw phone GNSS payload reads | `0` |
| raw phone IMU payload reads | `0` |
| broadcast-navigation payload reads | `0` |
| raw-base RINEX payload/header reads | `0` |
| raw payload copies/transforms | `0` |
| native solver invocations | `0` |
| solution coordinate rows/opened or interpreted | `0` |
| truth reads | `0` |
| accuracy calculations | `0` |
| MAT/PDC/precomputed-coordinate reads | `0` |
| Kaggle/token access | `0` |
| reruns/fallbacks/repairs/sweeps | `0` |

