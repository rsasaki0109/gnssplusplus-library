# SOFA — Standards Of Fundamental Astronomy

This directory contains an **intact, unchanged copy** of the SOFA ANSI C
library, vendored for use by libgnss++ to provide IAU-standard earth
rotation, polar motion, time scale, and astronomical computations.

## Provenance

- **Origin**: IAU Standards Of Fundamental Astronomy Board, http://www.iausofa.org/
- **Distribution channel**: extracted from
  https://github.com/GeoscienceAustralia/ginan
  (commit `535ef0a`, release v4.1.1, path `src/cpp/3rdparty/sofa/`).
- **SOFA issue**: 2021-05-12 (see Copyright (C) 2021 in source headers).
- **License**: SOFA Software License — see `LICENSE` in this directory.

## Modifications

Per SOFA Software License clause 3(b): there are **no modifications** to the
SOFA source files in this directory. Files under `src/` are byte-identical
to the IAU SOFA distribution as redistributed by ginan.

The only file added by libgnss++ is `CMakeLists.txt`, which builds the
SOFA sources as a CMake `STATIC` library named `sofa`. The CMake build
script is licensed under the libgnss++ project's MIT license; it does
not modify SOFA source code.

The SOFA test program `src/t_sofa_c.c` is present on disk for fidelity
to the IAU distribution but is **excluded from compilation** (it contains
its own `main()` and is intended as a self-test, not as library code).

## Acknowledgement

> This product uses computations derived from software provided by the
> International Astronomical Union's SOFA collection.
> See http://www.iausofa.org/

(Per SOFA Software License clause 3(a) and "acknowledgement is
appreciated" notice.)
