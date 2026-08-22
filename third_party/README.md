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
| `ginan-iers2010/` | GeoscienceAustralia/ginan (`535ef0a`) — IERS Conventions 2010 wrappers, GTime-independent subset | Apache License 2.0 | Solid-earth-tide displacement (Dehant) and Mendes-Pavlis tropospheric mapping (FCUL) |

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

## Relationship to `external/`

`external/` is a separate, gitignored location and must not be confused
with this directory:

| | `third_party/` | `external/` |
|---|---|---|
| Tracked in git | Yes | No (gitignored) |
| Role | Vendored sources compiled into libgnss++ targets | Full checkouts of upstream projects used as opt-in parity oracles or reference implementations |
| Build integration | Always configured via `add_subdirectory` | Only when explicitly enabled (e.g. `-DMADOCALIB_PARITY_LINK=ON -DMADOCALIB_ROOT_DIR=external/madocalib`) |
| License obligations | Aggregated in `LICENSE-third-party.md` / `NOTICE` | Remains with the upstream checkout; not redistributed |

Currently `external/` holds a MADOCALIB (RTKLIB-derived) checkout used to
verify MADOCA-USB behavior against an independent implementation. See
`docs/madocalib_native_migration.md` for usage.
