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

### 4.2 Truth-bench validation (Tokyo / Nagoya 6-run)

Driver: `apps/gnss_ppc_rtk_signoff.py`. The bench already exercises
both cities (`PROFILE_DEFAULTS` keyed on `tokyo` / `nagoya`, runs
`run1` / `run2` / `run3` for each).

Phase C adds a paired comparison harness:

1. Run each of the 6 PPC datasets twice — once with the legacy
   Step-1 model (`use_iers_solid_tide = false`) and once with the
   IERS Step-1+Step-2 model (`use_iers_solid_tide = true`).
2. Compare against `reference.csv` ground truth on the headline
   metrics already used by `gnss_ppc_rtk_signoff.py`:
   `coverage`, `fix_rate`, `fix95_h_m`, `wrong_fix_rate`, and
   `median_h_m`.
3. **Acceptance criterion**: opt-in must not regress any metric
   beyond noise (defined as ±2× the run-to-run standard deviation
   measured from the existing v237 baseline ledger). Improvements
   are expected at the few-mm level on `median_h_m` and `fix95_h_m`
   during peak-tide epochs but are not required for merge.

The opt-in flag means even a regression on the IERS path leaves the
default behavior intact, so the go/no-go for the flip-default
(below) is bench evidence only — no live-system risk.

## 5. Rollout

1. Land Phase C as drafted: opt-in, default off.
2. Run truth-bench on the 6 PPC datasets, attach the comparison
   table to the PR for review.
3. If the bench shows neutral-or-better results, follow up with a
   small "flip default" PR that flips `use_iers_solid_tide = true`
   in `PPPConfig`. Rollback path: revert that single-line PR.

This two-PR rollout (opt-in, then flip-default) is preferred over a
single big-bang change because the bench result is the only honest
oracle for "did we improve?" and we want it on record before the
change becomes the default for downstream consumers.

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
- **Phase D**: Pole tide, sub-daily EOP corrections, atmospheric
  loading. Smaller cm-level effects.
- **`icrsToItrs` consumers**: when satellite-side processing
  (LEO orbits, external SP3 ingestion in non-ECEF frames, attitude
  for satellite antenna PCO/PCV) is wired up, the wrapper is
  ready.
