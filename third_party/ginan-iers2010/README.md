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

Only the **GTime-independent** subset of ginan's iers2010 wrappers is
vendored here. Specifically:

| Routine | File | Purpose |
|---------|------|---------|
| `iers2010::fcul_a` | `ch9/fcul_a.cpp` | Mendes-Pavlis FCULa total mapping function (latitude, height, surface temperature, elevation) |
| `iers2010::fcul_zd_hpa` | `ch9/fcul_zd_hpa.cpp` | Mendes-Pavlis total zenith delay (lat, height, pressure, water-vapor pressure, temperature) |
| `iers2010::dehanttideinel_impl` | `dehanttideinel/dehanttide_all.cpp` | Solid-earth-tide station displacement (Dehant et al., IERS Conventions 2010 §7.1.1) |

The HARDISP ocean-tide-loading routines (`hardisp/`) are **deliberately
not included** in this PR because their public APIs depend on ginan's
internal `GTime` / `MjDateUtc` / `MjDateTT` time types. That subset
will land in a follow-up PR with native libgnss++ time-type adaptation,
or via a fresh re-implementation on top of SOFA's Julian Date primitives.

## Modifications

Per Apache License 2.0 §4(b), the following modifications were made
relative to the upstream ginan sources in
`GeoscienceAustralia/ginan@535ef0a:src/cpp/3rdparty/iers2010/`:

1. **`iers2010.hpp`** — adapted from the upstream header:
   - Removed `#include "common/gTime.hpp"` (no longer required because
     of (3) below).
   - Removed the `namespace hisp { ... }` block (HARDISP routines not
     vendored — see "Scope" above).
   - Replaced `#include "common/eigenIncluder.hpp"` with a direct
     `#include <Eigen/Dense>` so this public header is self-contained
     and does not depend on the internal `shim/` directory (which is
     PRIVATE to the vendored `.cpp` files only).
   - Added `#include <vector>` (was previously pulled in transitively
     via `gTime.hpp`).
   - Added a header banner describing the modifications and pointing
     readers to this README.

2. **`ch9/fcul_a.cpp`, `ch9/fcul_zd_hpa.cpp`,
   `dehanttideinel/dehanttide_all.cpp`** — single-line include path
   adjustment:
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

No algorithmic, numerical, or behavioral modifications were made to any
of the vendored `.cpp` source files.
