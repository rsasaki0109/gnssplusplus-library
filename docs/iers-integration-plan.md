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
/// simplified Step-1-only approximation. Default on after the
/// IGS-grade bench evidence in §5; pass --no-iers-solid-tide to
/// revert.
bool use_iers_solid_tide = true;
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

- With `use_iers_solid_tide = false` (legacy path, opt-out via
  `--no-iers-solid-tide`): displacement matches the existing
  Step-1 result to 1 µm. Guards against accidental breakage of
  the legacy path that remains available after Phase C-3.
- With `use_iers_solid_tide = true` (default after Phase C-3) at
  the IERS Conventions 2010
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
   `--no-iers-solid-tide` (the legacy Step-1-only Love-number
   path) and once with `--use-iers-solid-tide` (the IERS Step-1
   + Step-2 Dehant path, default after Phase C-3).
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

1. **Phase C-1 (PR #56, merged)**: PPP opt-in flag, default off.
2. **Phase C-2 (PR #57+#58, merged)**: comparison harness
   `gnss ppp-iers-solid-tide-bench` with tide-signal diagnostics
   (first_epoch / median per-component / aggregate-to-first ratio).
3. **Bench run on IGS-grade products (this PR's evidence)**: TSKB
   IGS station, 2026-04-15, IGS final SP3+CLK from BKG mirror,
   600 epochs static. Result: max paired displacement 4.8 cm,
   median 4.0 cm, per-component median (-0.7, +3.5, +1.5) cm,
   aggregate_to_first_epoch_ratio 122. The displacement
   magnitude lies inside the IERS Step-2 ~1-5 cm physical
   envelope. Compare to broadcast-derived products (max 2808 m,
   ratio 268,680) to see how dependent the bench is on
   IGS-grade ephemerides.
4. **Phase C-3 (this PR)**: flip
   `PPPConfig::use_iers_solid_tide` and gnss_ppp CLI default to
   `true`. Rollback path: pass `--no-iers-solid-tide` (preserved)
   or revert this PR.

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
  Adds `libgnss::iers::EopTable` (daily series with linear
  interpolation and leap-second snap on UT1-UTC) and a
  `PPPConfig::eop_path` / `--eop-c04` CLI knob. PPPProcessor loads
  the table on init and exposes `getEarthOrientationParams(GNSSTime)`.
  No PPP code path consumes the table yet — D-0 is a strictly
  additive scaffold for D-1+ (pole tide and sub-daily EOP). Output
  is bit-identical to the no-EOP baseline.

  Two upstream formats are supported, auto-detected at load time:
  (1) **IERS 20 C04** (Paris Observatory `eopc04.1962-now`) — the
  canonical final-values series, ITRF2020-consistent, ~1-week
  publication lag. (2) **IERS Bulletin A** (`finals2000A.daily` from
  USNO maia.usno.navy.mil) — combined observed (`I`) and predicted
  (`P`) rows, the prediction extending ~12 months past the last
  observed epoch and filling the C04 publication-lag gap. Bulletin A
  is the recommended source for benches that need fresh real-data
  EOP coverage of the previous month or two.
- **Phase D-1** *(landed 2026-05-09)*: Pole tide (IERS Conventions
  2010 §7.1.4). Adds `libgnss::iers::poleTideDisplacement`
  (post-2018 IERS linear mean-pole secular drift + Sr/Sθ/Sλ Stokes
  formulation) and `PPPConfig::use_iers_pole_tide` /
  `--use-iers-pole-tide` CLI flag. Requires the D-0 EOP scaffold;
  gracefully degrades to a no-op when no EOP table is loaded or
  the requested epoch is out of coverage. At TSKB on 2026-04-15
  (xp ≈ 0.149", yp ≈ 0.414") the raw displacement is ~1.3 mm;
  the PPP estimate shifts by ~0.2 mm in Z after the static-mode
  KF integrates across the arc.

  **Default flipped on 2026-05-09**, gated on the multi-site bench
  (PR #69, 5 IGS stations across mid- and high-latitudes plus
  PERT southern-hemisphere): median 0.4 mm at mid-latitudes, sign
  reversal across the equator consistent with §7.1.4 sin(2θ)
  modulation, all within the IERS-stated sub-cm envelope. The
  flip is inert by construction — pole tide is a no-op until the
  user supplies `--eop-c04` / `--eop-bulletin-a`. `--no-iers-pole-tide`
  is a permanent escape hatch.

  Phase D-1 also includes `apps/gnss_ppp_iers_pole_tide_bench.py`
  (mirroring the solid-tide harness from Phase C-1): a paired-PPP
  driver that runs `gnss_ppp` with and without the pole-tide flag
  and emits per-epoch displacement statistics (max / p95 / median,
  per-component medians, aggregate-to-first-epoch ratio). On TSKB
  2026-04-15 the bench reports max = 0.95 mm, median = 0.41 mm,
  per-component median dz = −0.21 mm — within the IERS §7.1.4
  expected sub-cm envelope at mid-latitudes during normal polar
  motion.

  Phase D-1 also includes
  `apps/gnss_ppp_iers_pole_tide_multisite_bench.py` — a multi-site
  driver that runs the per-site harness across an arbitrary list of
  IGS stations and emits an aggregate distribution summary (gates
  the eventual `use_iers_pole_tide` flip-default). On 5-station IGS
  data (ALGO, GRAZ, MIZU, TSKB, PERT) for 2026-04-15 the median
  pole-tide displacement at mid-latitudes is 0.4 mm (TSKB / MIZU),
  the median dz reverses sign across the equator (TSKB / MIZU
  −0.20 mm vs GRAZ +0.35 mm) consistent with the pole-tide formula,
  and PERT (−32°S Australia) shows the static-mode KF fully
  absorbing the sub-mm signal across the arc (median 0). ALGO and
  MIZU show transient max-displacement outliers from PPP convergence
  events that are unrelated to pole tide; the medians and dz values
  remain physical.

  The same multi-site driver supports per-site product overrides
  (`nav` / `sp3` / `clk` / `eop_c04`), so a campaign can span
  multiple days. Across 3 days × 2 sites (TSKB and GRAZ on
  2026-04-13 / 04-15 / 04-17) the median dz tracks the polar-motion
  drift monotonically: TSKB sees −0.188 / −0.196 / −0.202 mm and
  GRAZ +0.333 / +0.346 / +0.358 mm. Day-to-day spread is ±5%, in
  good agreement with the ~5 mas/day xp/yp drift observed in the
  Bulletin A series for that window.
- **Phase D-2** *(landed 2026-05-09)*: Sub-daily EOP corrections.
  Adds `libgnss::iers::subDailyEopCorrection(mjd_utc, ut1_utc)` →
  `{dxp, dyp, dut1, dlod}`, applying the full IERS Conventions 2010
  set: §5.5.1.1 Tables 5.1a/5.1b libration of CIP and UT1
  (10 + 11 = 21 terms) plus §8.2 Table 8.2 ocean-tide corrections
  (Eanes-Ray model, 71 terms). `PPPConfig::use_iers_sub_daily_eop`
  gates it; when on, `getEarthOrientationParams` adds the deltas
  to the daily-interpolated value. The pole tide path automatically
  picks up the higher-frequency CIP wobble when both flags
  (`--use-iers-pole-tide` and `--use-iers-sub-daily-eop`) are on.
  At TSKB 2026-04-15 0h UTC the raw delta is dxp ≈ −265 µas,
  dyp ≈ +255 µas, dut1 ≈ −24 µs; the PPP receiver-position effect
  on top of the pole tide is RMS 1.5 µm in Z (0.17% relative
  perturbation, matching the daily-vs-sub-daily xp/yp amplitude
  ratio).

  **Default flipped on 2026-05-09** alongside D-1. The corrections
  are pure deterministic harmonic series (no per-site data) and
  produce sub-µm-level RMS effects on the receiver position;
  inert without an EOP table loaded. `--no-iers-sub-daily-eop` is
  a permanent escape hatch.
- **Phase D-3** *(in progress)*: Atmospheric tidal loading (IERS
  Conventions 2010 §7.1.5) opt-in. Adds
  `libgnss::iers::atmosphericTidalLoadingDisplacement` and the
  `AtmosphericTidalLoadingCoefficients` struct (S1 + S2 amplitudes
  / phases × 3 components). PPP wires via
  `PPPConfig::use_iers_atm_tidal_loading` + `atm_tidal_loading_path`
  with `--use-iers-atm-tidal-loading` / `--atm-tidal-loading <file>`
  CLI knobs. Per-site coefficient file format mirrors the BLQ
  pattern but with only S1 / S2 rows (mid-latitude peak ~1 mm
  radial). The non-tidal pressure-loading component (which
  dominates at storm-driven sites) requires a gridded reanalysis
  ingestion path and is deferred to a follow-up PR. At TSKB
  2026-04-15 with synthetic mid-latitude coastal coefficients
  (S1 = 0.8 mm radial, S2 = 0.4 mm radial), the PPP estimate
  shifts by mean −10.6 µm / RMS 106 µm in Z. Default off pending
  real TU Wien per-site coefficients.

  Phase D-3 also includes `apps/gnss_ppp_iers_atm_tidal_loading_bench.py`
  (mirroring the pole-tide harness from Phase D-1): a paired-PPP
  driver that runs `gnss_ppp` with and without the ATL flag and
  emits per-epoch displacement statistics. On TSKB 2026-04-15 with
  the synthetic ATL fixture the bench reports max = 0.90 mm,
  p95 = 0.36 mm, median = 0.17 mm, per-component median dz = −50 µm
  — within the IERS §7.1.5 expected sub-cm envelope at mid-latitudes
  during normal pressure conditions.
- **`icrsToItrs` consumers**: when satellite-side processing
  (LEO orbits, external SP3 ingestion in non-ECEF frames, attitude
  for satellite antenna PCO/PCV) is wired up, the wrapper is
  ready.
