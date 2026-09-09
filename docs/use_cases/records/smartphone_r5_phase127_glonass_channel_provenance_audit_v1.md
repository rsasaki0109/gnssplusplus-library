# Smartphone R5 Phase127 GLONASS channel provenance audit

- Execution label: `Luna Max`
- Audit date: `2026-09-04`
- Scope: read-only repository source and already sealed Phase126 metadata.
- Repository: `/home/sasaki/workspace/rtklib_v2_ws/gnssplusplus-library`
- Branch: `feat/rtk-smartphone-performance-pr`
- HEAD at audit start: `df27546fafa4fb6fbd96a520dcd3fd91acc3c0ae`
- Worktree at audit start: clean.

No raw phone GNSS/IMU/navigation payload, raw-base bytes or headers, truth
payload, solution coordinate row, MAT data, precomputed coordinate or
correction, PDC, solver, accuracy evaluator, Kaggle, or token resource was
read or executed.  The Phase126 route values below are sealed inventory
metadata only.  This record is an audit and candidate design; it is not a raw
execution authorization.

## Decision

Exactly one source-backed, default-off candidate is frozen for a future
implementation:

`phase127-glonass-geph-channel-provenance-v1`

with proposed selector:

`--native-phase127-glonass-geph-channel-provenance`

The candidate is a provenance/admission adapter only.  It does not add a
factor, state, correction, signal, or fallback.  When the Phase126
source-complete raw-base selector is also explicitly enabled, it supplies a
GLONASS FDMA channel only from the existing RINEX header or from a selected,
time-valid broadcast GLONASS ephemeris.  It requires agreement when both
sources exist and fails closed for a missing, invalid, out-of-window, or
conflicting channel.  It never derives a channel from a fixed table, from a
base/phone/result coordinate, from an Android carrier-frequency heuristic, or
from a default zero channel.

The old Phase126 selector, graph, raw-base operator, C7/D/CCDD states, QR
solver, TDCP/IMU/Doppler factors, sigmas, filters, LM schedule, initialization,
Pixel5 offset, output contract, and legacy default remain unchanged.  The
candidate is not implementation-authorized by this audit; the exact future
implementation boundary and tests are specified below.

## Authoritative sealed context

The audited sealed structural result is:

| item | value |
|---|---|
| Phase126 result commit | `df27546fafa4fb6fbd96a520dcd3fd91acc3c0ae` |
| result JSON | `docs/use_cases/records/smartphone_r5_phase126_raw_base_compound_structural_result_v1.json` |
| result JSON SHA-256 | `a0ab7f6ca35f06277953646f0f406a2c850ea73cef3b52ec4b681d0d48236b33` |
| Phase126 selector | `--native-phase126-raw-base-source-complete` |
| Phase126 structural status | `no-go-phase126-raw-base-compound-structural` |
| route order | MTV-A, then LAX-T |
| authorized Phase126 solver launches | `0` |

The sealed Phase126 inventory says:

| route | GLO observation body rows | header `GLONASS SLOT / FRQ #` entries | nav inventory | conclusion here |
|---|---:|---:|---|---|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | 17,199 | 0 | finite/in-domain metadata only | `unknown` for `geph.frq` coverage |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | 920 | 0 | finite/in-domain metadata only | `unknown` for `geph.frq` coverage |

Both base members therefore need a non-header provenance path if their GLO
rows are to be retained, but the sealed nav metadata contains only byte,
body-line, finite-domain, and header-boundary facts.  It does not contain the
number of GLONASS ephemerides, their `frq` values, `toe`/`tof`, per-satellite
coverage at the actual observation/transmission times, or duplicate/tie
conflicts.  It is consequently not evidence that either route is ready for a
fallback channel.  A future authorization must materialize and inventory
those fields after authorization; this audit does not open the nav member.

The phone inventory's GLO channels are explicitly not admissible for this
candidate.  They are Android raw-frequency-derived metadata, whereas the
candidate's permitted provenance is RINEX observation header followed by
broadcast GLONASS ephemeris.  No external satellite/channel table is allowed.

## Source corpus and line evidence

### Official MATLAB/RTKLIB reproducibility cache

The sealed local official source cache is based on source tree commit
`29923f9f370f09ebc00f96d8cca375007a18e7d5`.  Relevant source files and the
hashes checked at this audit are:

| file | SHA-256 | evidence |
|---|---|---|
| `output/reproducibility-cache/MatRTKLIB/+gt/Gobs.m` | `be27fd1a075829a00f0aec5f32a0040a884077a8ac256a10a2bc26def255fc88` | `:126-158` reads `readrnxobs` and maps `fcn==0` to missing then subtracts 8; `:197-220` uses header FCN for `code2freq`; `:223-258` exposes the separate nav-backed `sat2freq` path |
| `output/reproducibility-cache/MatRTKLIB/src/mex/readrnxobs.c` | `b20c05a830d2d42ccf09c45eccfd1b4d9f9983eefff376a3b941c4562feedbb9` | `:44-75` calls RTKLIB `readrnxt` and exports `nav.glo_fcn` |
| `output/reproducibility-cache/MatRTKLIB/src/mex/sat2freq.c` | `39b2486be2e2e35424643c506f81c1b51d24e6de394c8c823703bcd42475e0be` | `:2-6` identifies the RTKLIB `sat2freq` wrapper; `:16-37` converts nav and calls it for each satellite/code |
| `output/reproducibility-cache/MatRTKLIB/src/mex/code2freq.c` | `80116512b2be2ea531938a48ca5b366ac13761563801264553c86723e69023c6` | `:43-53` delegates frequency construction to RTKLIB and warns for GLO FCN outside `-7..6` |
| `output/reproducibility-cache/MatRTKLIB/src/mex/eph2eph.c` | `3fd9be814c35b6e51fd6c9582ad12704625b3426187e47ced64ec39f44d377c2` | `:89-104` exports `geph.frq`; `:186-205` imports it as an integer with `toe`/`tof` |
| `output/reproducibility-cache/MatRTKLIB/+gt/Gnav.m` | `264d186444807c74d164042233c654f6f028578e4d35eea25196931afb32fbab` | `:63-94` reads navigation through RTKLIB; `:96-112` retains the returned navigation struct |
| `output/reproducibility-cache/MatRTKLIB/+gt/C.m` | `42cfdc2abb408159494de4e20bc2c2ba78c6e8267c459d42432fa374b070067a` | `:33-39` gives GLO base/step frequencies; `:79-82` gives the GLO PRN range |

The official split is important: `Gobs.setFrequency` is explicitly header-FCN
based, while `Gobs.setFrequencyFromNav` is explicitly navigation-FCN based.
The wrapper passes the full nav struct to RTKLIB `sat2freq`; it does not
invent a channel when the returned frequency is zero.  The official RTKLIB
source pinned by the cache's upstream family is also inspected at commit
`71db0ffa0d9735697c6adfd06fdf766d0e5ce807`:

* `src/rinex.c:419-424` parses `Rnn fcn` from `GLONASS SLOT / FRQ #` and
  stores the encoded `fcn+8` in `nav.glo_fcn`; zero is the absent sentinel.
* `src/rinex.c:1143-1150` normalizes a GLONASS navigation `frq` value greater
  than 128 by subtracting 256 and reports values outside the parser's
  accepted range.  The candidate must still enforce the observation
  frequency API's strict `-7..6` range rather than inherit a permissive parser
  diagnostic.
* `src/rtkcmn.c:3162-3174` shows the RTKLIB satellite-dependent GLONASS
  wavelength path using the matching `geph.frq` and the base/step constants.

The current upstream `satwavelen` helper has no time argument.  Therefore this
audit does not incorrectly claim that RTKLIB's convenience frequency helper
performs temporal ephemeris selection.  The native time-aware selection below
is the required adapter around the existing navigation API.

### Native source

| file | SHA-256 at `df27546` | evidence |
|---|---|---|
| `src/io/rinex.cpp` | `34f9d00fc151bccedaa7ae89724954e6742f40e73d7d3da5102723090a39ee40` | `:1180-1200` parses header FCN into a satellite map; `:1810-1836` converts GLONASS nav time and parses `glonass_frequency_channel`, including the `>128` signed normalization |
| `src/core/navigation.cpp` | `f876803d2e61fa62f370419f4ee4db16afb6b12b2d7ab00cd8b613af6f2649d3` | `:59-74` defines validity and age; `:76-86` stores/sorts ephemerides; `:146-168` chooses the age-closest valid record for a satellite/time |
| `include/libgnss++/core/navigation.hpp` | `578aca8779f7e2bbc891f8a5936ca1eeaa51eef28c85b9172d3700f5ae1f4bed` | `Ephemeris.glonass_frequency_channel` is an integer metadata field; `NavigationData::getEphemeris(sat,time)` is the time-aware API |
| `include/libgnss++/core/observation.hpp` | `7a81b4f1c7d36b4af2ed8a11c0f1edcbb998591a22a4d1406d75aeb43641f11b` | observation channel fields document the `-7..+6` domain |
| `include/libgnss++/core/signals.hpp` | `bb8cd20d4a22581ed2beb83f4e1b4ed6d397f28a2d5104d4736778c5b279ba56` | `:10-39` uses ephemeris FCN in GLO frequency formulas; `:116-139` uses an observation FCN only when its presence flag is true and otherwise falls back to the base frequency |
| `include/libgnss++/core/constants.hpp` | `fa0b7a0e53c6fac89568a33c9399bef0bbcc849222ae0086b0c56f221229dc99` | defines the GLO L1/L2 base and step frequencies |
| `src/algorithms/fgo_problems.cpp` | `1c1c24699e82945b7ee71f036e4cf969b0bd303ab49205d998ec5338b09714b4` | `:487-508` performs the existing two-pass transmission-time calculation and `:508-510` obtains the selected ephemeris at that time; `:531-535` consumes ephemeris-dependent frequency |
| `src/algorithms/fgo_internal.hpp` | `73aefdc6f14b78a08f3854fc90cda4d894985043a739b855cdde95a6fd5ff6d0` | `:633-655` repeats the same transmission-time/selected-ephemeris boundary; `:863-866` refuses to blend interpolated observations with unequal/missing channel metadata |
| `src/algorithms/ppp_observations.cpp` | `a49b15dbbc2f97d6681955ce89e1a7779a018fe1a10440cfdd6baba9424f06fd` | `:98-120` is a precedent for reading the selected ephemeris channel for a GLO observation, but is not itself the Phase127 candidate |
| `src/io/android_raw_gnss.cpp` | `a020ce900db6a9d3a994cc3adb3dcebbe84cbf67c1ff9793991b7a75039529c5` | `:239-256` derives a channel from Android carrier frequency; this is explicitly excluded from Phase127 provenance |

The native reader currently stores header entries in a map, so a later entry
can overwrite an earlier one and malformed/out-of-range header values are not
reported as a typed conflict.  Likewise, `getEphemeris` returns only the
nearest valid record and does not expose ties.  These are implementation
gaps the opt-in adapter must close in its admission telemetry; legacy parsing
and selection remain untouched when the selector is off.

## Exact provenance and selection policy

The candidate uses the existing FGO satellite/time boundary.  For each
retained GLONASS observation, `t_query` is the already established second-pass
transmission time after the existing pseudorange and satellite-clock update
(`fgo_problems.cpp:487-508`, or its exact equivalent in the GNSS-first path).
No new time iteration, nearest epoch, extrapolation, or state mutation is
introduced.

### FCN value and units

* FCN `n` is a dimensionless signed integer and must satisfy `-7 <= n <= 6`.
* RINEX header text carries a signed channel in `Rnn fcn`; RTKLIB's internal
  header array encodes it as `fcn+8`, with `0` meaning absent.  Native header
  parsing keeps the signed value directly.
* A GLONASS L1 observation uses
  `f_L1 = 1602.0e6 + n * 0.5625e6` Hz.
* A GLONASS L2 observation uses
  `f_L2 = 1246.0e6 + n * 0.4375e6` Hz.
* The wavelength is `lambda = 299792458.0 / f` metres.  Every derived value
  must be finite and positive.  The candidate does not change any residual,
  sigma, whitening, or factor equation; it only supplies the already existing
  channel metadata to the existing frequency helper.

### Header-first provenance with time-valid broadcast confirmation

The strict adapter policy is:

1. Parse all header FCN entries for the satellite before the map is collapsed.
   No entry is a fixed default.  A repeated satellite with the same signed FCN
   is a benign duplicate; a repeated satellite with different FCNs is a
   header conflict and fails closed.
2. Independently select the broadcast GLONASS ephemeris using the existing
   `NavigationData::getEphemeris(satellite, t_query)` age-closest valid rule.
   A candidate record is valid only when `valid` is true and
   `abs(t_query - toe) <= 1800` seconds, as enforced by
   `Ephemeris::isValid` (`navigation.cpp:59-63`).
3. The selected `geph.frq`/`glonass_frequency_channel` must be a finite
   integer in `[-7,6]`.  If a header FCN exists, it is the primary provenance
   value and the selected ephemeris FCN must match exactly.  If the header FCN
   is absent, the selected time-valid ephemeris FCN is the only permitted
   fallback and is attached to the observation.
4. For provenance completeness, a GLO row with a retained GLO code/carrier
   signal requires a selected time-valid ephemeris even when its header FCN is
   present.  Thus a missing selected ephemeris, invalid/out-of-window
   ephemeris, malformed/nonfinite FCN, or header/geph mismatch fails closed;
   the header is never silently accepted without a verifiable broadcast
   confirmation.
5. A tie between equally age-close valid records is accepted only when every
   tied record has the same FCN.  Same-FCN duplicate records are recorded as a
   duplicate-but-consistent provenance event.  Different-FCN ties fail closed.
   Non-tied records are not re-ranked or merged: the existing minimum-age
   selection remains authoritative, and only the selected record is used.
   Invalid or out-of-window records are not candidates; if no candidate remains,
   the row fails closed.  No fixed channel, base frequency, external table,
   Android-frequency inference, or stale ephemeris is allowed.

This policy is deliberately stricter than a convenience API that falls back
from one source to another.  It combines the official header and
nav-derived routes into an auditable source contract, while the exact
time-aware selection is supplied by the native navigation API.  It prevents a
satellite-only frequency lookup from applying a stale channel at a different
observation time.

## Route sufficiency determination

The sealed Phase126 metadata is insufficient to answer whether the candidate
can admit either route:

| route | known from sealed metadata | not present in sealed metadata | Phase127 decision |
|---|---|---|---|
| MTV-A | 17,199 GLO body rows; zero header entries; finite nav member and finite-domain inventory | GLO `geph.frq` count, per-satellite `toe`/`tof`, valid-window coverage at `t_query`, selected FCNs, duplicate/tie conflicts | `unknown`; no solver admission |
| LAX-T | 920 GLO body rows; zero header entries; finite nav member and finite-domain inventory | same missing fields as MTV-A | `unknown`; no solver admission |

The prior Phase126 failure is therefore not evidence that broadcast FCN
fallback is absent; it is evidence only that the then-current preflight could
not prove source-complete header FCN.  Conversely, finite nav bytes/body
lines do not prove per-row `geph.frq` availability.  Both routes must be
re-inventoried under a new authorization.

## Candidate boundary and invariants

The candidate may add only:

* a default-off, Phase126-composed provenance selector;
* typed header/geph FCN source metadata and compact admission counters;
* exact-key annotation of the existing `Observation` channel flag/value at
  the existing selected-ephemeris/transmission-time boundary; and
* fail-closed checks before any factor or correction mutation.

The candidate may not change:

* Phase126 raw-base source-complete correction equations, station reference,
  Sagnac/atmosphere/TGD policy, smoothing, interpolation, or exactly-once
  application;
* C7/D/CCDD state topology, units, initialization, handoff, or coverage;
* factor families, factor admission, TDCP/IMU/Doppler behavior, sigmas,
  whitening, Huber, filters, QR, ordering, LM schedule, or iteration limits;
* output timestamps, Pixel5 offset, solution opacity, or truth separation; or
* legacy/default behavior when either opt-in selector is absent.

## Implementation and focused-test plan

Implementation is not authorized in this audit.  If the parent agent later
approves the freeze, the smallest planned change is:

1. Add a typed helper (planned names:
   `include/libgnss++/algorithms/phase127_glonass_channel_provenance.hpp` and
   `src/algorithms/phase127_glonass_channel_provenance.cpp`) that validates
   the header ledger, selected ephemeris, range, validity, exact match, and
   tie policy without changing `NavigationData::getEphemeris`.
2. Add the selector at the existing FGO observation admission boundary (the
   GNSS-first and Phase126 main path only), with no partial selector and no
   fallback.  If the RINEX header API cannot expose duplicate entries, add a
   diagnostics-only ledger while preserving the legacy map when the selector
   is off.  Record only opaque counts/source labels, not solution rows.
3. Wire CLI/config/result provenance metadata and a validator that rejects
   selector-on runs lacking Phase126, header/geph agreement, or exact FCN
   coverage.  No source payload is printed.

Focused synthetic tests must cover:

* header FCN plus selected geph with equal FCN: accepted;
* no header FCN plus selected valid geph at both boundary values `-7` and `6`:
  accepted;
* header/geph mismatch: fail closed;
* absent header plus no selected geph, missing satellite, or
  `abs(t_query-toe)>1800`: fail closed;
* FCN `-8`, `7`, nonfinite, nonintegral, malformed, and invalid parser
  normalization: fail closed;
* tied duplicate geph records with equal FCN: accepted and counted;
* tied duplicate geph records with differing FCN: fail closed;
* duplicate header entries with equal/different FCN: consistent/failed;
* interpolation or observation-key mismatch never synthesizes a channel;
* Android source-frequency, fixed-zero, base-frequency, and external-table
  paths are rejected as provenance;
* selector off preserves legacy map, observation fields, factor counts, and
  configuration semantics; selector on does not alter any non-GLO graph
  family; and
* route preflight reports missing per-satellite/time geph coverage without
  launching the solver.

No real raw, nav, base, truth, MAT, or solver test is part of this audit.

## Future raw reauthorization requirements

The Phase126 authorization cannot be reused.  A future run needs separate
implementation qualification, a new structural manifest, and a new
independent one-shot authorization pinned to the implementation and this
freeze.  After authorization only, the runner may read, in order, the raw
phone GNSS/IMU, broadcast nav, and the sealed raw base RINEX for MTV-A then
LAX-T exactly once each.

Before solver launch for each route, the runner must inventory and seal:

* every GLO observation satellite/signal/time key and header FCN ledger;
* every candidate broadcast GLO ephemeris satellite, signed `frq`, `toe`/`tof`,
  valid-window result, and selected age;
* selected geph coverage, header/geph agreement, same-FCN duplicate counts,
  conflicting-tie counts, missing/out-of-window/invalid-FCN counts, and
  conservation of required GLO rows; and
* the exact selector composition and zero forbidden-lineage reads.

Any missing required geph row, invalid/out-of-range channel, header conflict,
different-FCN tie, unresolved duplicate, or nonfinite frequency/wavelength
must fail closed before solver launch.  No rerun, fallback, fixed channel,
external lookup, raw-content copy/transform, solution interpretation, truth
evaluation, accuracy calculation, MAT/PDC/precomputed coordinate read, or
Kaggle action is authorized.  If the inventory passes, the existing Phase126
structural gates remain mandatory: GNSS-first/main progress and strict cost
decrease, finite exact C7/D handoff, QR/base/offset exactly once, finite
coverage, and opaque solution seal only.  Truth would require a later,
independent authorization.

## Read accounting

| activity | count/result |
|---|---:|
| official/native source text | read-only |
| sealed Phase126 metadata | aggregate/provenance metadata only |
| raw phone GNSS/IMU/navigation payload | `0` |
| raw-base bytes or headers | `0` |
| truth payload or coordinate rows | `0` |
| solution coordinate rows | `0` |
| MAT/precomputed coordinates/corrections | `0` |
| PDC | `0` |
| solver invocations | `0` |
| accuracy/evaluator invocations | `0` |
| Kaggle/token access | `0` |
| code/config/test changes | `0` |
| reruns/fallbacks/repairs/sweeps | `0` |

This record freezes one candidate design only.  It grants no implementation,
raw, solver, truth, accuracy, or publication authorization.
