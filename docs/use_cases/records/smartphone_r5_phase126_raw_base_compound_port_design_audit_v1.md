# Smartphone R5 Phase126 source-complete raw-base compound-port design audit

- Execution label: `Luna Max`
- Audit date: `2026-09-04`
- Scope: read-only repository source and already sealed record metadata.
- Repository: `/home/sasaki/workspace/rtklib_v2_ws/gnssplusplus-library`
- Branch: `feat/rtk-smartphone-performance-pr`
- HEAD at audit start: `7d292931e5c70bede47701cf9c7a19a0d04b7e54`
- Worktree at audit start: clean.

No raw phone GNSS/IMU/navigation payload, raw-base bytes or headers, truth
payload, solution coordinate row, MAT data, precomputed coordinate/correction,
PDC, solver, accuracy evaluator, Kaggle, or token resource was read or
executed.  Phase112/118/120 values and Phase107 base-route values below are
sealed metadata only.  This record is a source audit and design freeze; it is
not an implementation or execution authorization.

## Decision

One and only one compound candidate is frozen for a future implementation:

`phase126-raw-base-source-complete-compound-v1`

The proposed opt-in name is
`--native-phase126-raw-base-source-complete`.  It is default-off and has no
partial selector.  Three implementation steps may be developed atomically,
but the selector is admitted only when all three steps pass the same contract:

1. raw RINEX/header reference and canonical observation/satellite-state ingress;
2. the official base `Gsat`/`Gobs.resPc` correction stream, including its
   atmosphere, geometry, time, smoothing, interpolation, and miss semantics;
3. exactly-once rover-code application and structural invariant verification.

A failure in any step fails the route closed.  There is no fallback to the
current native operator, no partial official/native hybrid, no retry, and no
new factor family.  The candidate is a design boundary only: implementation,
raw execution, truth evaluation, accuracy promotion, and publication remain
unauthorized by this audit.

The reason for freezing this compound boundary is that the remaining
official/native difference is not one scalar parameter.  The official helper
uses station-table XYZ plus an ENU offset and an opaque `Gsat`/`Gobs` residual;
the native raw-only path uses RINEX-header XYZ and an explicit satellite-state,
Earth-rotation, atmosphere, group-delay, stream, and interpolation operator.
Changing only one term can change the finite correction stream and therefore
the exact pseudorange-factor admission.  Phase125 correctly identified this as
a required compound port; Phase126 specifies its source-complete acceptance
boundary without running it.

## Authoritative state and sealed context

The current branch contains the preceding Phase125 triage audit and freeze:

| item | commit | purpose |
|---|---|---|
| Phase125 triage audit | `4ddfb4b757a44a9ef6864cb964ed5b65eb1f33a6` | read-only inventory; compound raw-base boundary identified |
| Phase125 triage freeze | `7d292931e5c70bede47701cf9c7a19a0d04b7e54` | no one-variable candidate; no implementation/execution |

The sealed accuracy aggregates are context, not tuning targets:

| sealed recipe | commit | MTV-A (m) | LAX-T (m) | macro (m) |
|---|---|---:|---:|---:|
| Phase112 output-offset | `0151bd070f8029413f8f2a17a28fe982dddb735f` | 0.997685253035948 | 0.6659910917640763 | 0.8318381724000121 |
| Phase118 TDCP official-k | `6b37f92077a8ede43cb24e7a5cada580feccea5c` | 1.002826677831249 | 0.6255696228129851 | 0.814198150322117 |
| Phase120 TDCP atmosphere candidate | `7553328e90a80d2127f95b6d0c0bab02b1b78a3f` | 1.0080567510569665 | 0.6260079250519545 | 0.8170323380544604 |

The frozen Phase112/118 recipe for the future structural candidate is fixed:
Phase101 epoch-local C7 and raw metre/second D, metre CCDD state, exact
same-run position/velocity/C/D handoff, Phase99 `MULTIFRONTAL_QR`, existing
IMU/carrier/TDCP topology and gates, fixed Phase118 TDCP sigma and Huber
mapping, existing raw-base application point, Pixel5 output offset exactly
once, existing filters/noise/LM schedule/initialization, and no global ISB
duplicate.  Phase126 adds only the source-complete raw-base operator; it does
not tune or replace any of those settings.

The Phase107 sealed raw-base metadata establishes the two in-scope members
without reopening their payloads:

| route | sealed base member metadata | observed interval / window | header approximate XYZ (m) |
|---|---|---|---|
| MTV-A | `base.obs`, 10,708,536 bytes, SHA-256 `380b8ff9091344fb756697e27f0983d9a0ba2cf0c201b96849bfc1ecc1af0e52` | 1 s / 151 | `(-2703115.921, -4291767.2078, 3854247.9066)` |
| LAX-T | `base.obs`, 719,969 bytes, SHA-256 `d731e0e8a7ba4396d62340c85b6238e66c50c6a349621f9dcea0ea6885fd4cfe` | 15 s / 11 | `(-2507798.7984, -4676369.6918, 3526890.8008)` |

These coordinates, paths, and hashes are sealed metadata only.  They are not
inputs read by this audit and do not authorize a future run.

## Source corpus and hashes

The official source is the sealed reproducibility cache at source commit
`29923f9f370f09ebc00f96d8cca375007a18e7d5`.  Hashes below make the line
references reproducible.

| source | SHA-256 | relevant lines |
|---|---|---|
| `output/reproducibility-cache/gsdc2023/functions/correct_pseudorange.m` | `b0536ccff478b0aff253448ffb7a203c715b8064dd8dc85898e38f1f05d0441e` | `5-6`, `8-24`, `26-34` |
| `output/reproducibility-cache/MatRTKLIB/+gt/Gobs.m` | `be27fd1a075829a00f0aec5f32a0040a884077a8ac256a10a2bc26def255fc88` | `126-220`, `983-1025`, `1135-1169` |
| `output/reproducibility-cache/MatRTKLIB/+gt/Gsat.m` | `a56c323664660c1b7e771180e9bc18b08f809fe8a6d71b795366b96207871f6` | `137-195`, `237-306` |
| `output/reproducibility-cache/MatRTKLIB/+rtklib/satpos.m` | `20b7fd78065a22d9d02b0f394aabef2437e991ed2c63b04c65194a96827f5e` | `1-28` |
| `output/reproducibility-cache/MatRTKLIB/+rtklib/geodist.m` | `4ddced92aea78defd7eb8b2040dd23b1b0575eb0c1ac719a128c8db62ab4a91f` | `1-19` |
| `output/reproducibility-cache/gsdc2023/fgo_gnss.m` | `5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3` | `50-82`, `89-120`, `123-230` |
| `output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m` | `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3` | `63-106`, `129-187`, `189-221` |
| `output/reproducibility-cache/gsdc2023/functions/gnsslog2obs.m` | `665ce43d1fc3c5a9c3b3a6fd76f81539126e3ab897fbba484ab0ccd18964acff` | `38-121`, `139-218` |
| `output/reproducibility-cache/gsdc2023/preprocessing.m` | `976629d187e7fab5868eb8e5676a4d40520eb23db1254f7320d3b4270d7dffcf` | `70-95` |
| `output/reproducibility-cache/gsdc2023/parameters.m` | `518925e9c75c7a14fceb5cd99432fe883311e0d64c72b94396744ac765120f52` | `46-55`, `62-83` |
| `src/io/rinex.cpp` | `92f683e34b4052f8d105f69fc4bf4397ca990be3b5a6b220181a56423f465533` | `97-180`, `207-300`, `373-401`, `1102-1112` |
| `src/algorithms/base_pseudorange_compensation.cpp` | `f3c3c859793c4bed9c7ae93825d32565f20eab0badb09412a4fab2a5930c9817` | `132-159`, `163-257`, `259-339` |
| `src/algorithms/fgo_problems.cpp` | `5675e82fc595933da4e7c14ad468ae69437dc04bac79a7ef55f1edfa03e098da` | `365-405`, `430-640` |
| `apps/native/gnss_fgo_imu_no_base.cpp` | `df77ab6556f90d6d9a8464d360e85d2a49a50d6fc04ef396833f354f821a1176` | `7420-7524`, `7886-8044` |

## Complete official pipeline trace

### 1. Entry, route trimming, and observation provenance

The official GNSS-only and GNSS/IMU entry points load a preprocessed
`phone_data.mat` (`fgo_gnss.m:18-30`; `fgo_gnss_imu.m:18-38`) and select
`FTYPE=["L1","L5"]`.  That is source provenance, not an allowed Phase126
input.  A raw-only C++ candidate must use the native Android raw parser and
broadcast navigation in memory; it must not recreate `phone_data.mat`, read a
WLS/result coordinate, or use a MATLAB artifact.

Official preprocessing chooses the base RINEX member from `setting.RINEX` and
`setting.Base1`, trims it to the rover span with 180-second margins, requires
nonempty overlap, and checks that the base covers the rover span
(`preprocessing.m:70-95`).  The Phase126 contract therefore requires a
declared raw base member, an exact hash, a finite overlap containing the full
rover interval, and no station/phone coordinate side channel.

### 2. Base RINEX loading, header reference, and observation fields

`gt.Gobs(file)` calls `rtklib.readrnxobs` (`Gobs.m:126-158`).  A nonzero
header position becomes `gt.Gpos`; GLONASS FCNs are retained and later used for
frequency calculation.  `Gobs.m:197-220` maps each selected observation code
through `code2freq` and computes `lam=CLIGHT/freq`; the class contract records
pseudorange in metres, carrier in cycles, Doppler in Hz, SNR in dB-Hz,
frequency in Hz, and wavelength in metres (`Gobs.m:13-38`).

The native RINEX reader parses `APPROX POSITION XYZ` and
`ANTENNA: DELTA H/E/N` as separate header fields (`src/io/rinex.cpp:1102-1112`),
selects primary/secondary bands through the signal policy
(`src/io/rinex.cpp:97-160,207-300`), and stores code/carrier/Doppler/SNR with
their native units (`src/io/rinex.cpp:373-401`).  It can retain the antenna
delta in the header, but the current base model consumes only
`approximate_position` (`apps/native/gnss_fgo_imu_no_base.cpp:7464-7489`).

Official `correct_pseudorange.m:8-24` instead reads `base_position.csv`, takes
the mean XYZ for the selected station/year, and obtains an ENU
`base_offset.csv` value.  The offset call occurs *after* `obsb.residuals(satb)`
and there is no assignment of the changed `posbase` back into `pc` in that
function.  The source therefore does not prove that the offset affects the
returned correction stream.  Phase126 must not guess by applying RINEX delta,
table offset, or both.  A future raw-only port may use only a finite RINEX
header reference under an explicitly proven antenna convention; otherwise it
fails closed.

### 3. Rover raw parsing, signal/frequency/wavelength, and time

The official raw converter removes invalid clock/received-time rows and
unsupported constellations, estimates carrier frequency, derives
`lam=CLIGHT/freq`, maps GLONASS and BeiDou time, and computes

`P=(tow_rx-tow_tx)*CLIGHT`, `L=ADR/lam`, and `D=-pseudorange_rate/lam`

(`gnsslog2obs.m:38-121,139-218`).  The native Android parser performs the
same raw-domain responsibility without reading the MATLAB result: signal and
frequency admission is explicit (`src/io/android_raw_gnss.cpp:202-315`),
raw pseudorange uses signal offsets plus GLONASS/BDS time conversion and
nearest-week unwrapping (`:529-560`), and the optional device WLS position is
not an allowed seed for this candidate (`:563-570`).

For the base, official `Gobs` time is the RINEX/RTKLIB GPST representation.
`correct_pseudorange.m:33-34` interpolates the base stream at the rover
`obsr.time.t`; it does not match rows by nearest index.  The native contract
must therefore canonicalize both streams to the same GPST week/TOW, reject
ambiguous time-system conversion or non-monotonic/duplicate epochs, and use
the exact `(SatelliteId, SignalType, epoch time)` key.  No nearest, extrapolated,
or floating-point epoch repair is permitted.

### 4. Satellite state, clock, and health

`Gsat(obsb,nav)` calls `setSatObs` (`Gsat.m:116-129,137-172`).  Before
`rtklib.satposs`, the observation pseudorange is adjusted by the receiver clock
argument; the returned satellite position/velocity are ECEF and the satellite
clock bias/drift are metres and metres/second (`Gsat.m:30-38`,
`satpos.m:10-28`).  The official `satpos` notes that the satellite clock does
not include code-bias TGD/BGD.  Unhealthy satellites are masked
(`Gsat.m:174-185`).

The native current base model begins with `epoch.time-P/c`, calculates a
satellite state, subtracts the satellite clock bias, calculates again, checks
ephemeris health, and then forms the correction (`base_pseudorange_compensation.cpp:186-214`).
The native rover factor builder uses the same two-pass pattern
(`fgo_problems.cpp:480-509`).  This is observable source behavior, but not yet
proof of bit-equivalence to RTKLIB `satposs` with the official receiver-clock
argument.  Phase126 must select one source-equivalent propagation contract and
record failed ephemeris/health/state rows; it cannot silently combine both
conventions.

### 5. Geometry, Sagnac, ionosphere, and troposphere

After satellite state construction, official `Gsat.setRcvPos` calls
`rtklib.geodist` for ECEF range and LOS, then `satazel`, `tropmodel`, and
`ionmodel` (`Gsat.m:237-273`).  `geodist.m:1-19` explicitly documents that
the geometric distance includes Sagnac.  `setRcvVel` adds the range-rate
Sagnac term (`Gsat.m:275-306`), although velocity is not part of the base
pseudorange correction itself.

The native base and rover paths instead rotate the satellite by a travel-time
Earth-rotation matrix and call `NavigationData::calculateGeometry`, then use
native Klobuchar/Saastamoinen models (`base_pseudorange_compensation.cpp:215-240`;
`fgo_problems.cpp:511-540`).  A source-complete operator must use exactly one
Sagnac representation and exactly one ionosphere/troposphere model per
residual.  Rotating a state and then applying an independent Sagnac correction
without a proof is forbidden; a model mismatch is a compound failure, not a
free tuning choice.

### 6. Official residual equation, units, sign, and TGD/BGD

`Gobs.residuals` defines the official base code residual in metres
(`Gobs.m:1151-1164`):

```text
resPc_b(t,s,f) = P_b(t,s,f)
                  - (rho_b(t,s) - dts_b(t,s) + I_b(t,s,f) + T_b(t,s))
              = P_b + dts_b - rho_b - I_b - T_b .
```

Here `P_b`, `rho_b`, `dts_b`, `I_b`, and `T_b` are all metre-valued.  The
official satellite clock `dts_b` is already metres and, per `satpos.m:20-28`,
does not include TGD/BGD.  `Gobs.resPc` has no explicit TGD/BGD term.

The official rover path applies the interpolated correction with

```text
resPc_r_corrected = resPc_r - pc,
pc = interpolated(smoothed(resPc_b)).
```

(`correct_pseudorange.m:26-34`, `fgo_gnss.m:72-79`,
`fgo_gnss_imu.m:85-92`).  The native current operator instead forms

```text
P_native_corrected = P + satellite_clock_m
                       - ionosphere_m - troposphere_m - group_delay_m
                       - geometric_range_m
```

for both rover and base (`fgo_problems.cpp:542-573`,
`base_pseudorange_compensation.cpp:241-252`).  Thus native group-delay use is
not proven equivalent to the official `resPc` source.  Phase126 freezes the
rule that official no-explicit-TGD/BGD semantics must be established
*symmetrically for rover and base* inside the compound operator, or the
selector fails closed.  A rover-only, base-only, or standalone TGD toggle is
outside scope and would violate the Phase122 closure.

### 7. Same-satellite/same-signal stream, smoothing, and interpolation

`correct_pseudorange.m:5-6` calls `obsb.sameSat(obsr)`.  `Gobs.sameSat`
preserves the rover satellite ordering and inserts NaN for a satellite absent
from the base (`Gobs.m:983-1025`).  The helper then processes one `freq` at a
time, namely the official FGO `L1` and `L5` paths.

For every exact satellite/signal stream, the official helper applies a centered
`movmean`: 151 samples at 1 second and 11 samples at 15 seconds
(`correct_pseudorange.m:26-31`; `parameters.m:53-55`).  It then uses linear
`interp1` from base times to rover times (`correct_pseudorange.m:33-34`).
Outside the source interval MATLAB returns NaN, and the FGO loops add a
pseudorange factor only when `resPc` is finite (`fgo_gnss.m:123-149`).  The
source does not authorize endpoint hold, nearest fill, extrapolation, or
interpolation across a missing exact stream.

The native model uses the key `(SatelliteId, SignalType)`, a centered finite
moving mean, and in-domain linear interpolation with no extrapolation or
endpoint hold (`base_pseudorange_compensation.cpp:259-339`).  The native
source miss mask removes factors without an exact stream or finite in-domain
correction and proves conservation per signal (`source_pseudorange_miss_mask.cpp:53-178`).
Those mechanisms are useful building blocks but do not by themselves prove
that native signal selection, state/atmosphere terms, or MATLAB edge/NaN
semantics are source-complete.

### 8. FGO observation correction and fixed graph boundary

The official graph inserts a pseudorange factor only for finite corrected code
residuals, with the existing LOS, sigma, and robust model
(`fgo_gnss.m:123-149`; `fgo_gnss_imu.m:189-221`).  The base correction is not a
new graph factor: it changes the code measurement before factor insertion.

Phase126 must preserve the existing Phase112/118 graph boundary: GNSS-first
Point3/velocity C7/D staging, same-run optimized C7/D handoff, main Pose3,
velocity, IMU, C7/D and CCDD states, Phase99 QR, TDCP/IMU/Doppler topology,
all gates/noise/LM/initialization, and final Pixel5 offset exactly once.  The
source-complete base operator may alter only the permitted raw code measurement
and its explicit official miss mask.  It must not add base factors, modify
clock/ISB units, synthesize a coordinate, or feed any saved output back into
the graph.

## Official/native component comparison

| component | official source behavior | current native behavior | Phase126 classification and contract |
|---|---|---|---|
| Input lineage | Base RINEX is selected and trimmed in `preprocessing.m:70-95`; helper reads station tables in `correct_pseudorange.m:8-15`. | Same-process raw base RINEX is read by the app; Android raw rover parser is in-memory; no RINEX intermediate. | Divergent provenance. Use phone raw GNSS/IMU, broadcast nav, and exact sealed base RINEX only; reject tables, phone/result coordinates, MAT, PDC, and precomputed corrections. |
| Station reference | Mean `base_position.csv` XYZ plus `base_offset.csv` ENU call at `correct_pseudorange.m:8-24`. | RINEX `APPROX POSITION XYZ`; `ANTENNA: DELTA H/E/N` is parsed but not consumed (`apps/...:7464-7489`). | Unresolved. Header reference is admissible only with a proven antenna convention and finite Earth-valid XYZ; ambiguous/nonzero unproven delta fails closed. |
| Base observation parse | `Gobs(file)` -> `readrnxobs`, header position/GLONASS FCN (`Gobs.m:126-158`). | `RINEXReader` selects policy primary/secondary bands and stores fields (`rinex.cpp:97-180,207-300,373-401`). | Not proven exact. Freeze signal/tracking selection and GLONASS FCN behavior with synthetic RINEX fixtures before raw authorization. |
| Signal/frequency | `code2freq`, `lam=CLIGHT/freq`; official FGO loops L1/L5 (`Gobs.m:197-220`, `fgo_gnss.m:28-30`). | `SignalType` policy and native frequency helpers; Android parser has explicit frequency tolerance (`android_raw_gnss.cpp:202-315`). | Equivalent units are visible; exact code/band mapping remains a compound acceptance test. No additional-band selector in Phase126. |
| Satellite state/clock | `Gsat.setSatObs` -> `satposs`; clock m, drift m/s, no TGD/BGD (`Gsat.m:137-172`, `satpos.m:20-28`). | Two-pass `P/c` then satellite-clock time in base and rover (`base_pseudorange_compensation.cpp:186-214`, `fgo_problems.cpp:485-509`). | Divergent implementation shape. Require source-equivalent transmission time/state/health output; no mixed iteration. |
| Geometry/Sagnac | `geodist` range/LOS includes Sagnac (`Gsat.m:259-262`, `geodist.m:1-19`). | Explicit satellite rotation then `calculateGeometry` (`base_pseudorange_compensation.cpp:215-219`). | Unresolved representation. Apply Sagnac once, with a source-vector golden test; double rotation fails closed. |
| Ionosphere | RTKLIB `ionmodel` at base position/frequency (`Gsat.m:264-273`). | Native Klobuchar with frequency scaling (`base_pseudorange_compensation.cpp:223-234`). | Model equivalence must be demonstrated for the selected nav/time/frequency; missing/nonfinite model fails closed. |
| Troposphere | RTKLIB `tropmodel` at base position/elevation (`Gsat.m:264-266`). | Native Saastamoinen (`base_pseudorange_compensation.cpp:236-240`). | Same requirement: source parity or closed failure; no sigma compensation. |
| Code residual | `P + dts - rho - ion - trop`, no explicit TGD/BGD (`Gobs.m:1160-1164`). | `P + dts - ion - trop - group_delay - rho` for rover/base (`fgo_problems.cpp:542-573`, `base_pseudorange_compensation.cpp:241-252`). | Divergent/coupled. Official no-explicit-TGD/BGD semantics must be symmetric in the compound operator; a local TGD edit is forbidden. |
| Time transfer | `obsb.sameSat(obsr)` then `interp1(base.t,pc_,rover.t)` (`correct_pseudorange.m:5-6,33-34`). | Exact key stream, linear in-domain `correctionAt`, no extrapolation/hold (`base_pseudorange_compensation.cpp:307-339`). | Close in shape; require canonical GPST, strict monotonicity, exact endpoint and gap semantics. |
| Smoothing | `smoothdata(...,"movmean",151/11)` (`correct_pseudorange.m:26-31`). | Centered finite moving mean, window selected from observed 1/15-s interval (`base_pseudorange_compensation.cpp:259-304`). | Window fixed per route; NaN/edge behavior must be synthetic-source matched, never adaptive. |
| Missing policy | NaN corrected residual means no FGO P factor (`fgo_gnss.m:138-142`). | Miss mask drops missing stream/out-of-domain/nonfinite correction with conservation accounting (`source_pseudorange_miss_mask.cpp:84-155`). | Retain only explicit official misses; report per exact signal and preserve epoch indices. Any unexplained population change fails closed. |
| Application point | `obsr.resPc -= pc` before `obserrmodel` and graph creation (`fgo_gnss.m:72-82`). | Current base correction is applied to adopted native factors after problem construction (`apps/...:7886-8044`). | Compound port must establish one source-equivalent pre-factor boundary, mark exactly once, and reject a pre-applied factor. |
| FGO topology | P/D plus motion/clock/TDCP/IMU factors; no base factor (`fgo_gnss*.m:123-230`, `fgo_gnss_imu.m:189-323`). | Phase112/118 C7/D/QR/IMU/TDCP graph. | Fixed. Base correction cannot add/remove non-code factors or alter C7/D/CCDD/QR/LM/filter settings. |

## Exact candidate scope and atomic implementation plan

The single selector represents one source-complete operator, not three
independent knobs.

### Atomic step A: raw ingress and reference contract

- Read the exact caller-declared raw base RINEX once and verify its hash before
  parsing; use broadcast navigation already admitted by the raw recipe.
- Accept only a finite, Earth-valid `APPROX POSITION XYZ` and an explicitly
  resolved RINEX antenna reference.  `ANTENNA: DELTA H/E/N` may not be applied
  merely because it is present; if its relation to the official reference is
  not proven, fail closed.  Never read station CSVs, base offset tables, phone
  coordinates, result coordinates, PDC, MAT, or precomputed corrections.
- Parse the source observation types, GLONASS FCN, signal/frequency/wavelength,
  epoch time system, and finite code values with exact `(satellite, signal)`
  identity.  Unsupported or ambiguous mapping is a route failure.
- Require base coverage for the rover span and frozen route sampling: MTV-A
  1 s/151 samples; LAX-T 15 s/11 samples.  Do not infer a window from a
  malformed or mixed interval.

### Atomic step B: official base correction stream

For each selected exact satellite/signal stream and base epoch, implement or
prove the official operator as one unit:

```text
t_tx       = source-equivalent signal transmission epoch
(r_s,dts)  = source-equivalent broadcast satellite state at t_tx
rho        = source-equivalent geodist(base_reference, r_s), Sagnac once
pc_raw     = P_base + dts - rho - ionosphere - troposphere
pc_smooth  = centered source movmean(pc_raw, 151 or 11)
pc(t_rover)= linear interpolation of pc_smooth in-domain only
```

The no-explicit-TGD/BGD line above is the official `Gobs.resPc` contract.  If
the native rover measurement path cannot use the same source semantics without
changing the frozen factor/admission boundary, the compound selector is not
admitted.  The stream must use source-equivalent Sagnac, atmosphere, clock
units, time-scale conversion, health filtering, and NaN behavior.  No endpoint
hold, nearest fill, extrapolation, gap bridging, or adaptive smoothing is
allowed.

### Atomic step C: exactly-once application and structural gate

- Apply `P_rover_corrected = P_rover_raw - pc` once to the permitted raw code
  measurement before the GNSS-first/main code factor is finalized; do not
  change carrier, Doppler, TDCP, IMU, timestamps, or state initialization.
- Preserve exact retained epoch keys, C7 mapping/gauge, raw D m/s, metre CCDD,
  Phase99 QR, and same-run position/velocity/C/D handoff.  The base operator
  must not create a base factor or a second clock/ISB state.
- Emit only structural telemetry at qualification: source model build count,
  exact stream counts, per-signal retained/miss/out-of-domain/nonfinite counts,
  correction pass count, duplicate marker, and factor-key conservation.  Do
  not emit solution coordinates.
- The compound selector is active only when `A && B && C` and every invariant
  below is true.  A/B/C intermediate artifacts are internal implementation
  steps; there is no selector that runs one without the others.

## Acceptance invariants and fail-closed rules

The future implementation and tests must prove all of these before any raw
authorization.  A failed predicate is a route `NO-GO`; no fallback or rerun.

1. **Lineage:** exactly one declared phone raw GNSS stream, raw IMU stream,
   broadcast navigation stream, and sealed raw base RINEX.  Forbidden path
   tokens and MAT/precomputed coordinate/correction inputs are rejected before
   any solver call.
2. **Station reference:** header XYZ is finite, Earth-valid, and hash-bound;
   antenna reference semantics are explicit.  An unproven header delta,
   station-table substitution, or coordinate side channel fails closed.
3. **Observation canonicalization:** all accepted code rows have finite metre
   P, exact satellite/signal identity, valid frequency/wavelength/GLONASS FCN,
   source-compatible time, and one deterministic tracking-code selection.
4. **Satellite state:** every correction row has source-equivalent transmission
   time, finite satellite position/clock, healthy ephemeris, and one Sagnac/
   geometry evaluation.  Any missing nav state is accounted as an explicit
   source miss; no silently reconstructed state is accepted.
5. **Atmosphere and units:** clock, range, ionosphere, troposphere, and
   correction are metres; no hidden seconds-to-metres conversion or TGD/BGD
   double application.  Source no-TGD/BGD semantics are symmetric for rover
   and base or the route fails closed.
6. **Stream:** same-satellite/same-signal streams are exact; base interval and
   151/11 centered window match the frozen route; smoothing is finite and
   source-edge/NaN equivalent.
7. **Interpolation:** only in-domain linear interpolation is accepted.  Every
   factor either has one finite correction or one classified missing,
   out-of-domain, or nonfinite reason.  No extrapolation, endpoint hold,
   nearest fill, repair, or gap bridge.
8. **Conservation/exactly once:**
   `original_code_rows = retained_rows + missing_stream_rows + out_of_domain_rows + nonfinite_rows`
   per signal and globally; source model build count is one; application pass
   count is one; duplicate application is rejected before mutation.
9. **Graph invariance:** no base factor and no new variable; P-only correction;
   C7/D/CCDD units and exact key order, Phase99 QR, factor families, IMU/
   carrier/TDCP/Doppler, filters, sigma, robust kernels, LM schedule,
   initialization, and Pixel5 final offset are unchanged.
10. **Structural route gate:** for both MTV-A and LAX-T, GNSS-first and main
    have accepted iterations and strict finite cost decrease; C7/D handoff is
    full, finite, and exact-key aligned; main output coverage is finite and
    earth-valid; there is no fallback.  This is structural only and does not
    authorize truth or accuracy.

Any unresolved official/native item in the comparison table, any unexpected
factor-key/count change, station-reference ambiguity, nonfinite value, or
forbidden input is fail-closed.  The candidate cannot fall back to the current
native group-delay/rotation operator while claiming source-complete parity.

## Planned implementation files and focused tests

No implementation file was changed in Phase126.  The following is the minimum
future implementation boundary, subject to a code review against this freeze:

- `include/libgnss++/algorithms/base_pseudorange_compensation.hpp` and
  `src/algorithms/base_pseudorange_compensation.cpp`, or a new
  `phase126_raw_base_compound` module, for the source-complete in-memory
  operator and typed diagnostics;
- `src/io/rinex.cpp` and `include/libgnss++/io/rinex.hpp` only if header
  reference/antenna semantics or exact signal/FCN admission cannot be exposed
  by the current API;
- `src/algorithms/fgo_problems.cpp` only at the opt-in corrected-P boundary,
  with the official clock/TGD/geometry semantics applied symmetrically and
  legacy code untouched;
- `src/algorithms/source_pseudorange_miss_mask.cpp` and its header for exact
  stream/miss conservation, with no change to non-code factors;
- `apps/native/gnss_fgo_imu_no_base.cpp` for one compound CLI/config selector,
  preflight guards, and opaque structural telemetry;
- focused synthetic/source tests under the existing test target, plus the
  target build and full C++ suite only after implementation qualification.

The focused test plan is source-only and must run without raw, truth, MAT, or
solver data:

- official residual sign/unit golden vectors, including no-explicit-TGD/BGD;
- header XYZ/antenna-reference acceptance, ambiguous/nonfinite/earth-invalid
  rejection, and forbidden station-table/coordinate lineage rejection;
- RINEX V2/V3 field parsing, primary/secondary tracking selection, GLONASS FCN,
  frequency/wavelength, time-system conversion, and unsupported-signal failure;
- source-equivalent satellite clock/transmission-time/health and one-Sagnac
  geometry path; atmosphere finite/nonfinite behavior;
- centered 151/11 moving mean edge/NaN behavior, exact in-domain interpolation,
  endpoint and out-of-domain rejection, and strict same-satellite/same-signal
  matching;
- per-signal/global conservation, exactly-once and duplicate-application
  rejection, no factor-family/key changes, and no-solution telemetry;
- selector-off semantic/byte regression against the Phase112/118 default and
  all-or-nothing compound admission when any one atomic step fails.

No tests or target build are run by this design-only audit because the frozen
candidate has not been implemented and raw/solver execution is explicitly
forbidden.

## Open issues that remain inside the compound boundary

These are not reasons to invent a second candidate.  They are mandatory
source-parity decisions before implementation can pass the freeze:

- **Station point:** the official table/ENU offset is forbidden at runtime and
  the helper applies it after residual construction without showing a `pc`
  recomputation.  Header XYZ versus antenna point is therefore unresolved;
  nonzero/ambiguous delta must fail closed.
- **Satellite state and Sagnac:** official RTKLIB `satposs`/`geodist` and native
  two-pass/rotation code have different visible control shapes.  A synthetic
  source-vector equivalence proof is required to avoid missing or double
  applying a correction.
- **TGD/BGD:** official `resPc` excludes explicit code bias while native rover
  and base include group delay.  A symmetric source-complete policy is required
  as one compound change; a one-sided change is prohibited.
- **Atmosphere:** official RTKLIB `ionmodel`/`tropmodel` and native
  Klobuchar/Saastamoinen calls must be shown equivalent for the selected nav,
  epoch, elevation, and frequency, or the route fails closed.
- **Sampling/NaN:** official `obsb.dt`/`smoothdata` edge behavior and native
  observed-median/finite-window behavior must be matched for 1 s/151 and
  15 s/11 without adaptive windows or gap repair.
- **Signal selection:** official `readrnxobs` code/FCN mapping and native
  signal-policy selection must produce the same exact keys for the Phase112/118
  allowed bands.  Additional-frequency preservation is not part of Phase126.

Until each issue is resolved by implementation tests and a fresh pre-raw
contract, this freeze authorizes no raw-base read, solver invocation, truth
evaluation, accuracy calculation, or submission.

## Read accounting

| activity | count/result |
|---|---:|
| official/native source text | read-only; source lines and hashes only |
| sealed record metadata | aggregate/state/provenance metadata only |
| raw phone GNSS/IMU/navigation payload | `0` |
| raw-base bytes or headers | `0` |
| truth payload or coordinate rows | `0` |
| solution coordinate rows | `0` |
| MAT data | `0` |
| precomputed coordinates/corrections | `0` |
| PDC/Kaggle/token access | `0` |
| native solver invocations | `0` |
| accuracy evaluator invocations | `0` |
| code/configuration/test changes | `0` |
| raw/solver/truth executions, reruns, fallbacks, repairs, sweeps | `0` |

This audit freezes one default-off compound design boundary only.  It does not
authorize implementation, qualification, raw execution, truth, accuracy, or
publication.
