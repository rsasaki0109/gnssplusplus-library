# third_party/

Vendored copies of upstream libraries used by libgnss++.

Each subdirectory contains an unmodified (or minimally adapted, as noted)
copy of an external project, together with that project's original
license file and a `README.md` describing provenance and any
modifications.

## Index

| Directory | Upstream | License | Purpose |
|-----------|----------|---------|---------|
| `sofa/`   | IAU SOFA (Standards Of Fundamental Astronomy), issue 2021-05-12 | SOFA Software License | Earth rotation, polar motion, time scales, astronomical computations for PPP cm-level positioning |

## Conventions

- Each vendor directory is added to the build via
  `add_subdirectory(third_party/<name>)` from the project root
  `CMakeLists.txt`.
- Vendored sources are **not modified** unless explicitly noted in that
  vendor's `README.md`; modifications, when present, are described per
  the upstream license's "derived work" requirements.
- The CMake build scripts (`CMakeLists.txt`) inside each vendor
  directory are part of libgnss++ and licensed under MIT, even though
  the source code they build may be under a different license.
- A consolidated index of all third-party licenses is maintained at the
  project root in `LICENSE-third-party.md`. Attribution notices required
  by upstream licenses are aggregated in the project root `NOTICE`
  file.
