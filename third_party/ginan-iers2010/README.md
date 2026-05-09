# ginan-iers2010 — IERS Conventions 2010 routines (subset)

C++ wrappers around IERS Conventions 2010 routines, vendored from
Geoscience Australia's Ginan project for use by libgnss++ to provide
solid-earth-tide displacement and Mendes-Pavlis tropospheric mapping.

## Provenance

- **Origin**: GeoscienceAustralia/ginan, commit `535ef0a` (release v4.1.1),
  path `src/cpp/3rdparty/iers2010/`.
- **Underlying scientific source**: IERS Conventions (2010) software, see
  https://iers-conventions.obspm.fr/. The C++ wrappers in ginan are
  translations/wrappers of the original FORTRAN routines distributed by
  the IERS.
- **License**: Apache License, Version 2.0 — see `./LICENSE` in this
  directory and `./NOTICE` for attribution.

## Scope of this vendoring

The Mendes-Pavlis tropospheric mapping (Chapter 9), the solid-earth-tide
displacement (Chapter 7 §7.1.1) and the HARDISP ocean-tide-loading
(Chapter 7 §7.1.2) routines are vendored:

| Routine | File | Purpose |
|---------|------|---------|
| `iers2010::fcul_a` | `ch9/fcul_a.cpp` | Mendes-Pavlis FCULa total mapping function (latitude, height, surface temperature, elevation) |
| `iers2010::fcul_zd_hpa` | `ch9/fcul_zd_hpa.cpp` | Mendes-Pavlis total zenith delay (lat, height, pressure, water-vapor pressure, temperature) |
| `iers2010::dehanttideinel_impl` | `dehanttideinel/dehanttide_all.cpp` | Solid-earth-tide station displacement (Dehant et al., IERS Conventions 2010 §7.1.1) |
| `iers2010::hisp::hardisp_impl` and helpers | `hardisp/{admint,eval,hardisp_impl,recurs,shells,spline,tdfrph}.cpp` | HARDISP ocean-tide-loading time-domain displacement (IERS Conventions 2010 §7.1.2). Driven by spline-interpolated tidal admittance over 342 reference harmonics. Time argument is a UTC Modified Julian Date (`double mjd_utc`); the libgnss++ shim replaces ginan's `GTime` / `MjDateUtc` / `MjDateTT` helpers with the IAU SOFA UTC->TAI->TT chain (`iauUtctai`, `iauTaitt`). |

## Modifications

Per Apache License 2.0 §4(b), the following modifications were made
relative to the upstream ginan sources in
`GeoscienceAustralia/ginan@535ef0a:src/cpp/3rdparty/iers2010/`:

1. **`iers2010.hpp`** — adapted from the upstream header:
   - Removed `#include "common/gTime.hpp"` (no longer required because
     the HARDISP API now takes `double mjd_utc` rather than ginan's
     `GTime` — see (4) below).
   - Replaced the `namespace hisp { ... }` declarations to use
     `double mjd_utc` arguments instead of `GTime`.
   - Replaced `#include "common/eigenIncluder.hpp"` with a direct
     `#include <Eigen/Dense>` so this public header is self-contained
     and does not depend on the internal `shim/` directory (which is
     PRIVATE to the vendored `.cpp` files only).
   - Added `#include <vector>` (was previously pulled in transitively
     via `gTime.hpp`).
   - Added a header banner describing the modifications and pointing
     readers to this README.

2. **`ch9/fcul_a.cpp`, `ch9/fcul_zd_hpa.cpp`,
   `dehanttideinel/dehanttide_all.cpp`,
   `hardisp/{eval,recurs,shells,spline}.cpp`** — single-line include
   path adjustment:
   - `#include "3rdparty/iers2010/iers2010.hpp"` →
     `#include "iers2010.hpp"`
   - Reason: the libgnss++ vendor layout places this header alongside
     the source files rather than under a `3rdparty/iers2010/` prefix.
   - No algorithmic or behavioral change.

3. **`shim/common/constants.hpp`** — *new* minimal stand-in header
   added by libgnss++. It satisfies the single `common/constants.hpp`
   include that survives in `dehanttideinel/dehanttide_all.cpp`
   (providing the `D2R` degrees-to-radians constant) without pulling
   in ginan's full `common/` tree (which depends on
   boost/serialization, custom EigenDenseBase plugins, ginan time
   types, etc.). The value is bit-identical to ginan's equivalent.

4. **`hardisp/{tdfrph,admint,hardisp_impl}.cpp`** — `GTime epoch`
   parameter replaced with `double mjd_utc` (UTC Modified Julian Date)
   throughout. Inside `tdfrph.cpp`, the upstream `MjDateUtc` /
   `MjDateTT` helpers used to compute "TT centuries since J2000" are
   replaced with the IAU SOFA UTC->TAI->TT chain
   (`iauUtctai`+`iauTaitt`); this requires the vendored `sofa` target
   to be linked into `ginan_iers2010` (handled in this directory's
   `CMakeLists.txt`). The Doodson argument expansion, Delaunay
   variable expressions and tidal admittance spline are byte-for-byte
   identical to upstream.

No algorithmic, numerical, or behavioral modifications were made to the
vendored science computations themselves; only the time-argument shim
described in (4) was applied.
