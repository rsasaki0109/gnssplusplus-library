# Smartphone R5 Phase126 sealed structural raw-base contract audit

- Execution label: `Luna Max`
- Audit date: `2026-09-04`
- Scope: read-only tracked source and already sealed record metadata.
- Repository: `/home/sasaki/workspace/rtklib_v2_ws/gnssplusplus-library`
- Branch: `feat/rtk-smartphone-performance-pr`
- Design freeze: `583a6c7a4788f82373a4e71436d2953703bc764c`
- Implemented candidate pin: `9e9972667ee009e1bcd0dd1ea4732ae45415c3ce`

No phone GNSS/IMU/navigation payload, raw-base bytes or headers, truth,
solution rows, MAT, PDC, precomputed coordinates/corrections, solver,
accuracy evaluator, Kaggle, or token resource was read or executed.  This is
a launch-free structural contract.  All raw, solver, truth, and solution
read counters are required to remain zero until an independently committed
authorization is made.

## Decision

The sole admitted structural candidate is the already implemented,
default-off compound selector:

`phase126-raw-base-source-complete-compound-v1`

`--native-phase126-raw-base-source-complete`

It composes the sealed Phase118 champion recipe.  Phase117 dynamic sigma,
Phase120 atmosphere-cancellation, and Phase109 additional-frequency-band
selectors are explicitly off.  No partial Phase126 selector or fallback is
permitted.  The selector is not a raw execution authorization.

The exact structural matrix is two routes in order, MTV-A then LAX-T, one run
per route:

| route | dataset id | sealed base member | interval/window | sealed header XYZ (m) |
|---|---|---|---:|---|
| MTV-A | `2021-03-16-18-59-us-ca-mtv-a/pixel5` | `base.obs` | 1 s / 151 | `(-2703115.921, -4291767.2078, 3854247.9066)` |
| LAX-T | `2022-04-01-18-22-us-ca-lax-t/pixel5` | `base.obs` | 15 s / 11 | `(-2507798.7984, -4676369.6918, 3526890.8008)` |

The base byte counts and SHA-256 values are copied only from the sealed
Phase107/126 metadata; they are not read or re-hashed in this audit.

## Frozen recipe and invariants

The following are fixed and must be asserted before launch:

- Phase118 official route-Type Huber mapping is active; both frozen routes
  are `Highway`, therefore `k=0.5`.
- TDCP sigma remains fixed at `0.03 m`; Phase117 dynamic sigma is absent.
- Phase99 `MULTIFRONTAL_QR` is selected for the Phase126 meter-state main
  graph.  GNSS-first, C7/D/CCDD, IMU, TDCP, factor topology, LM schedule,
  filters, initialization, and Pixel5 final offset are unchanged.
- Raw phone GNSS, raw phone IMU, broadcast navigation, and one caller-declared
  sealed raw base RINEX are the only runtime inputs.  The RINEX station
  reference must be finite and Earth-valid, and the permitted reference
  convention must be proven by the input inventory; no coordinate side
  channel is accepted.
- The Phase126 source formula is, in metres,

  `resPc_b = P_b + satellite_clock_m - geometric_range_m - ionosphere_m - troposphere_m`.

  The official no-explicit-TGD/BGD policy is retained.  Geometry includes one
  Sagnac term, satellite state/clock and atmosphere must be finite and in the
  supported source domain, and each correction is interpolated only in-domain.
- The official centered `movmean` windows are exactly 151 samples for the
  1-second route and 11 samples for the 15-second route.  The stream key is
  exact satellite/signal/time; no nearest, endpoint, or extrapolated repair is
  allowed.
- A/B/C is transactional: (A) raw RINEX/header/signal/state ingress, (B)
  official correction stream construction and conservation, and (C) exactly
  once rover-code application.  A failure in any step discards all candidate
  streams and fails closed.  Required telemetry includes A/B/C completion,
  retained/corrected/miss/out-of-domain counts, finite/domain checks,
  exactly-once marker, and conservation equations.

## Pre-authorization inventory boundary

The launch-free runner carries placeholders only.  It may inspect tracked
source, sealed JSON/Markdown, and the target binary metadata, but it must not
materialize, `stat`, hash, open, or parse any raw phone/base path.  After a
future authorization, the runner must materialize the raw RINEX/header/signal
inventory exactly once and fail closed if any of these are outside this
contract:

1. complete signal-to-frequency/wavelength mapping, including a finite
   GLONASS channel/frequency when GLONASS observations are present;
2. finite, in-domain satellite state, clock, health, geometry/Sagnac and
   ionosphere/troposphere values for every admitted correction row;
3. exact route interval/window and stream/miss conservation;
4. no MAT, saved/result coordinate, base-position/offset, PDC, precomputed
   correction, or alternate seed input.

Unknown or missing signal metadata, GLONASS channel/frequency, satellite
state, atmosphere domain, header reference convention, or route inventory is
an authorization-time NO-GO.  It is not repaired by fallback, extrapolation,
zero synthesis, or rerun.

## Required structural gates and accounting

Each route result must be opaque (solution rows are never displayed or read)
and must report only structural metadata: return code, expected/output epoch
coverage, GNSS-first and main accepted iterations, finite strict cost
decrease, C7/D exact/full/finite handoff, C0D/base/offset exactly-once
markers, Phase126 A/B/C telemetry and counts, selected solver branch, and
failure reason if any.  A structural GO requires every gate; otherwise the
sealed result is a fail-closed NO-GO and there is no rerun.

This audit freezes only the contract and the launch boundary.  Raw execution,
solution interpretation, truth scoring, accuracy promotion, and Kaggle
submission remain unauthorized and out of scope.
