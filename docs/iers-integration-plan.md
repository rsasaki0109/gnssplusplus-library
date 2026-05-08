# IERS Conventions 2010 integration — Phase C design

This document describes the planned integration of the
`libgnss::iers::*` wrapper API (introduced by PRs #52–#55) into the
existing PPP solver, and the truth-bench validation strategy that
gates the change.

It is **not** a description of the wrapper API itself; for that, see
the headers under `include/libgnss++/iers/` and the per-PR
descriptions of the four foundation PRs:

| PR  | Branch              | Foundation work |
|-----|---------------------|-----------------|
| #52 | `feat/vendor-sofa`         | IAU SOFA vendoring under `third_party/sofa/`. |
| #53 | `feat/vendor-iers2010`     | ginan-iers2010 subset (`fcul_a`, `fcul_zd_hpa`, `dehanttideinel`). |
| #54 | `feat/iers-wrappers`       | `libgnss::iers::{earthRotationAngle, icrsToItrs, gnssTimeToMjdUtc, solidEarthTideDisplacement, EarthOrientationParams}`. |
| #55 | `feat/iers-ephemeris`      | `libgnss::iers::{sunPositionIcrs, moonPositionIcrs}`. |

## 1. Goal

Replace the simplified IERS Conventions 2010 **Step-1** body-tide
approximation currently used by `PPPProcessor::calculateSolidEarthTides`
with the full IERS Conventions 2010 §7.1.1 (Dehant) **Step-1 + Step-2**
solid-earth-tide displacement model surfaced by the wrapper API,
gated behind an opt-in feature flag for safe rollout.

The change is expected to produce a few-millimeter periodic refinement
to the receiver-position correction, primarily during peak diurnal /
semidiurnal tidal phases. The cm-level PPP truth-bench targets
(Tokyo / Nagoya 6-run) are the validation oracle.

## 2. Non-goals (deferred)

- **Earth rotation matrix wiring (`icrsToItrs`)**. The PPP solver is
  currently entirely ECEF-native; satellite ephemerides arrive in
  ECEF, observation residuals are formed in ECEF, the Kalman state
  is in ECEF. There is no consumer for `icrsToItrs` yet. It is
  available in the wrapper API for future work (e.g., LEO satellite
  position, attitude, or external ephemeris ingestion) but Phase C
  does not consume it.
- **`calculateSatelliteAntennaPCO`** at `src/algorithms/ppp.cpp:3331`
  is fully stubbed and has zero call sites in the PPP code today.
  Phase C does not wire it up; revisiting satellite antenna
  PCO/PCV handling is its own scope.
- **Ocean tide loading (HARDISP)**. Vendored separately as a future
  Phase A-2b PR; not consumed here.
- **Pole tide**. Out of scope for this PR; covered by a possible
  Phase E.

## 3. Concrete change

### 3.1 PPPConfig flag

`include/libgnss++/algorithms/ppp_shared.hpp` currently has, near
line 178:

```cpp
bool apply_ocean_loading      = false;
bool apply_solid_earth_tides  = true;
bool apply_relativity         = true;
```

Phase C adds, alongside the existing flags:

```cpp
/// When true (and apply_solid_earth_tides is also true), use the
/// IERS Conventions 2010 §7.1.1 (Dehant) Step-1 + Step-2 model from
/// libgnss::iers::solidEarthTideDisplacement instead of the built-in
/// simplified Step-1-only approximation. Default off; opt-in for
/// safe rollout pending truth-bench validation.
bool use_iers_solid_tide = false;
```

Reading the flag inside the existing dispatcher:

```cpp
// src/algorithms/ppp.cpp:2680 (current)
Vector3d PPPProcessor::calculateSolidEarthTides(
    const Vector3d& position, const GNSSTime& time) const {
    constexpr double kSunGM = 1.32712440018e20;
    constexpr double kMoonGM = 4.902801e12;
    if (!position.allFinite() ||
        position.norm() < constants::WGS84_A * 0.5) {
        return Vector3d::Zero();
    }
    const Vector3d sun_position  = approximateSunPositionEcef(time);
    const Vector3d moon_position = approximateMoonPositionEcef(time);
    return bodyTideDisplacement(position, sun_position, kSunGM) +
           bodyTideDisplacement(position, moon_position, kMoonGM);
}
```

becomes:

```cpp
// src/algorithms/ppp.cpp:2680 (Phase C)
Vector3d PPPProcessor::calculateSolidEarthTides(
    const Vector3d& position, const GNSSTime& time) const {
    if (!position.allFinite() ||
        position.norm() < constants::WGS84_A * 0.5) {
        return Vector3d::Zero();
    }

    if (ppp_config_.use_iers_solid_tide) {
        const double mjd_utc = libgnss::iers::gnssTimeToMjdUtc(time);
        const Vector3d sun_icrs  = libgnss::iers::sunPositionIcrs(mjd_utc);
        const Vector3d moon_icrs = libgnss::iers::moonPositionIcrs(mjd_utc);
        return libgnss::iers::solidEarthTideDisplacement(
            mjd_utc, position, sun_icrs, moon_icrs);
    }

    // Existing Step-1-only Love-number formula (default path).
    constexpr double kSunGM  = 1.32712440018e20;
    constexpr double kMoonGM = 4.902801e12;
    const Vector3d sun_position  = approximateSunPositionEcef(time);
    const Vector3d moon_position = approximateMoonPositionEcef(time);
    return bodyTideDisplacement(position, sun_position, kSunGM) +
           bodyTideDisplacement(position, moon_position, kMoonGM);
}
```

The dispatcher upstream at `src/algorithms/ppp.cpp:2671` is
unchanged — it still gates the entire correction on the existing
`apply_solid_earth_tides` flag, so users who have disabled solid
tides altogether see no change.

### 3.2 CLI plumbing

`apps/gnss_ppp.cpp` follows the canonical "three-line" pattern from
the recent `--enable-ppp-holdamb` change (commit `0293f54`):

1. Add field to local `Options` struct.
2. Add `--use-iers-solid-tide` / `--no-iers-solid-tide` to
   `printUsage()`.
3. Parse in `parseArguments()` and copy to `ppp_config` in `main()`.

No other CLI tools require updates: `gnss_solve` and the Python
benchmark scripts pass through `PPPConfig` directly.

### 3.3 Build wiring

Already in place from PR #54:
`target_link_libraries(gnss_lib PRIVATE sofa ginan_iers2010)`. The
new `ppp.cpp` consumers of `libgnss::iers::*` symbols compile and
link without further CMake changes.

## 4. Tests

### 4.1 Unit / golden tests

Add `tests/test_ppp_iers_solid_tide.cpp` exercising the dispatch:

- With `use_iers_solid_tide = false` (default): displacement
  matches the existing Step-1 result to 1 µm. Guards against
  accidental breakage of the legacy path.
- With `use_iers_solid_tide = true` at the IERS Conventions 2010
  reference epoch (2009-04-13 0h UT), displacement matches the
  published reference (0.077, 0.063, 0.055) m to 1 mm — the same
  bound already validated by `IersEphemeris.SolidEarthTideUsingComputedEphemeris`,
  but now exercised through the PPP processor's dispatcher rather
  than the wrapper directly.
- Both paths return zero for invalid receiver positions
  (`!allFinite()`, sub-Earth-radius), preserving existing behavior.

### 4.2 Truth-bench validation

Phase C-1 only affects the PPP code path
(`PPPProcessor::calculateSolidEarthTides`). The existing PPC
Tokyo / Nagoya 6-run truth-bench (`apps/gnss_ppc_rtk_signoff.py`)
operates on the RTK code path and does **not** exercise the PPP
solid-earth-tide dispatcher; it cannot be used as the validation
oracle for this change.

Phase C-2 introduces a PPP-specific paired-comparison harness at
`apps/gnss_ppp_iers_solid_tide_bench.py`, registered with the
dispatcher as `gnss ppp-iers-solid-tide-bench`. Behavior:

1. Run `gnss ppp` twice on the same PPP setup — once with
   `--no-iers-solid-tide` (the default Step-1-only Love-number
   path) and once with `--use-iers-solid-tide` (the IERS Step-1
   + Step-2 Dehant path).
2. Read both `.pos` outputs, match epochs, compute per-epoch
   ECEF displacement between the two solutions.
3. Emit a `comparison.json` summary with
   `{min, mean, median, p95, max}` displacement plus
   matched-epoch counts.
4. Optional acceptance gate: `--require-max-displacement-m`
   fails the run if the maximum per-epoch displacement exceeds
   the supplied bound.

The harness is **data-agnostic**: the user supplies whichever
PPP dataset they have available — bundled signoff data, a real
IGS station, a public PPP dataset — and the script wraps the
paired runs around it. Acceptance criteria are workflow-specific
and live in CI scripts or local invocations, not in the harness
itself.

**Acceptance heuristic** for promoting the IERS path to the
default (the flip-default PR): on a representative PPP dataset,
the per-epoch displacement between the two paths should be
millimeters to a few centimeters at most (the IERS Step-2 model
adds diurnal/semidiurnal corrections that peak at ~1 cm), and
the IERS solution should not increase any error metric versus a
known ground truth when one is available.

Because the flag is opt-in, even a regression on the IERS path
leaves default behavior intact, so the go/no-go for the
flip-default PR is bench evidence only — no live-system risk.

## 5. Rollout

1. Land Phase C-1 (the PPP opt-in flag): opt-in, default off.
2. Land Phase C-2 (the comparison harness
   `gnss ppp-iers-solid-tide-bench`).
3. Run the harness on a representative PPP dataset (bundled
   signoff data, an IGS station, or a public PPP dataset
   available locally) and attach the resulting
   `comparison.json` to the flip-default PR for review.
4. If the bench shows neutral-or-better results, follow up with
   a small "flip default" PR that flips
   `use_iers_solid_tide = true` in `PPPConfig`. Rollback path:
   revert that single-line PR.

This three-step rollout (opt-in, harness, flip-default) is
preferred over a single big-bang change because the bench result
is the only honest oracle for "did we improve?" and we want it on
record before the change becomes the default for downstream
consumers.

## 6. Risks & mitigations

| Risk | Mitigation |
|------|------------|
| Frame mismatch in `dehanttideinel_impl`. The IERS routine takes the station in ITRS but Sun/Moon in ICRS (see `tides.hpp` "FRAME NOTE"). PPP currently passes ECEF sun/moon to the legacy path; the new IERS path uses ICRS via `sunPositionIcrs` / `moonPositionIcrs`. | Wrapper API enforces correct types at the call site; the Phase C code passes whatever each path expects. The 1-mm-tolerance reference test in `IersTides` and `IersEphemeris` validate the wrapper end-to-end through the same dispatcher pattern Phase C uses. |
| `iauUtctai` warning for epochs after 2026 (last leap second known to vendored SOFA issue 2021-05-12). | SOFA returns the warning code but the result remains correct as long as no leap second has been inserted since the SOFA snapshot. Future leap seconds (currently none scheduled) require a SOFA refresh. PPP receives no warning in normal flow. |
| The two-iteration leap-second handshake in `gnssTimeToMjdUtc` has a single-iteration approximation that is wrong by up to 30 s on a leap-second day's midnight boundary. | Documented in the wrapper. PPP epochs do not sit on leap-second-midnight boundaries in the truth-bench data; if this becomes a concern (real-time on a leap-second day), promote `gnssTimeToMjdUtc` to a true Newton iteration. |
| Truth-bench shows regression on a specific run. | Default-off opt-in means no production impact. Investigate bench specifics (which metric, which epochs) before flipping default. |

## 7. Out-of-band follow-ups (separate scope)

These are tracked here for context but are independent work that
does not block Phase C:

- **Phase A-2b**: Vendor or re-implement HARDISP (ocean-tide-loading)
  with native libgnss++ time-type adaptation. Re-uses the BLQ
  parser already present in libgnss++.
- **Phase D-0** *(in progress)*: EOP plumbing scaffolding.
  Adds `libgnss::iers::EopTable` (IERS 20 C04 daily series with
  linear interpolation and leap-second snap on UT1-UTC) and a
  `PPPConfig::eop_path` / `--eop-c04` CLI knob. PPPProcessor
  loads the table on init and exposes
  `getEarthOrientationParams(GNSSTime)`. No PPP code path consumes
  the table yet — D-0 is a strictly additive scaffold for D-1+
  (pole tide and sub-daily EOP). Output is bit-identical to the
  no-EOP baseline.
- **Phase D-1** *(in progress)*: Pole tide (IERS Conventions 2010
  §7.1.4) opt-in. Adds `libgnss::iers::poleTideDisplacement` (post-2018
  IERS linear mean-pole secular drift + Sr/Sθ/Sλ Stokes formulation)
  and `PPPConfig::use_iers_pole_tide` / `--use-iers-pole-tide` CLI
  flag. Requires the D-0 EOP scaffold; gracefully degrades to a no-op
  when no EOP table is loaded or the requested epoch is out of
  coverage. At TSKB on 2026-04-15 (xp ≈ 0.149", yp ≈ 0.414") the
  raw displacement is ~1.3 mm; the PPP estimate shifts by ~0.2 mm
  in Z after the static-mode KF integrates across the arc. Default
  off pending a real-data truth-bench validation cycle.

  Phase D-1 also includes `apps/gnss_ppp_iers_pole_tide_bench.py`
  (mirroring the solid-tide harness from Phase C-1): a paired-PPP
  driver that runs `gnss_ppp` with and without the pole-tide flag
  and emits per-epoch displacement statistics (max / p95 / median,
  per-component medians, aggregate-to-first-epoch ratio). On TSKB
  2026-04-15 the bench reports max = 0.95 mm, median = 0.41 mm,
  per-component median dz = −0.21 mm — within the IERS §7.1.4
  expected sub-cm envelope at mid-latitudes during normal polar
  motion.
- **Phase D-2**: Sub-daily EOP corrections (IERS §8.2). Shares the
  D-0 scaffold.
- **Phase D-3**: Atmospheric loading. Smaller cm-level effect; needs
  a separate gridded-data ingestion path, independent of EOP.
- **`icrsToItrs` consumers**: when satellite-side processing
  (LEO orbits, external SP3 ingestion in non-ECEF frames, attitude
  for satellite antenna PCO/PCV) is wired up, the wrapper is
  ready.
