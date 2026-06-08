# Third-Party Licenses

libgnss++ itself is distributed under the MIT License (see `LICENSE`
at the project root).

This project additionally bundles the following third-party software
components in the `third_party/` directory, each under its own license.
Attribution notices required by these licenses are aggregated in the
project root `NOTICE` file.

| Component | Path | Upstream | License | License File |
|-----------|------|----------|---------|--------------|
| IAU SOFA  | `third_party/sofa/`  | http://www.iausofa.org/ | SOFA Software License | `third_party/sofa/LICENSE` |
| ginan-iers2010 | `third_party/ginan-iers2010/` | https://github.com/GeoscienceAustralia/ginan (commit `535ef0a`) | Apache License 2.0 | `third_party/ginan-iers2010/LICENSE` |

## Compatibility

All third-party licenses bundled in this repository are compatible with
the project's MIT distribution: each permits redistribution in source
form provided that the original license text and attribution are
preserved. The `NOTICE` file and per-vendor `LICENSE` files in
`third_party/` satisfy these requirements.

The SOFA Software License is a permissive, attribution-style license
(not OSI-approved but widely used in scientific software). Notable
clauses for downstream users:

- **Clause 3(c)**: Names of routines in *derived works* (i.e. modified
  copies) must not begin with `iau` or `sofa`. libgnss++ ships the
  SOFA sources unmodified, so this restriction does not affect
  libgnss++'s own symbol naming. Downstream users who modify the SOFA
  copy in `third_party/sofa/` must observe this clause.
- **Clauses 5–6**: SOFA is provided "as is" with no warranty.

The Apache License 2.0 (used by `third_party/ginan-iers2010/`) is
OSI-approved and broadly compatible with MIT redistribution. Per Apache
2.0 §4(b), the modifications libgnss++ has made relative to the
upstream ginan sources are documented in
`third_party/ginan-iers2010/README.md`. Per §4(d), upstream
attribution and the original NOTICE are preserved at
`third_party/ginan-iers2010/NOTICE`.
