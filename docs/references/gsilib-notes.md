# GSILIB Notes

This page tracks what is worth borrowing from `GSILIB` as a reference, without
trying to reproduce its GUI tools.

Primary upstream:

- [GSILIB download page](https://terras.gsi.go.jp/geo_info/gsilib/gsilib_download.html)
- [GSILIB manual (PDF)](https://terras.gsi.go.jp/geo_info/gsilib/GSILIB_manual.pdf)

## Scope

`GSILIB` matters here because it is a Japanese multi-GNSS toolkit built on
`RTKLIB` with:

- post-processing workflows,
- PPP and relative positioning modes,
- support for `SP3`, `CLK`, `IONEX`, and `RTCM 3 SSR`,
- Japanese operational examples and correction workflows.

For libgnss++, the useful parts are:

- product/correction input expectations,
- post-processing runbook ideas,
- specific Japanese correction examples such as `IFB`, `ISB`, and `L2C`
  handling mentioned on the GSILIB download page.

The GUI itself is **not** the target.

## What GSILIB appears to cover

Based on the official download page and manual:

- `GSIPOST_GUI` for post-processing baseline analysis
- `GSIPLOT` and observation/result plotting
- `PPP-Kinematic` and `PPP-Static`
- `SP3`, `CLK`, `IONEX`, `RTCM3 SSR`, SBAS/EMS inputs
- config save/load around post-processing workflows
- Japanese examples around:
  - `IFB correction`
  - `ISB estimation / correction`
  - `L2P(Y)-L2C` cycle-slip correction

## Why this matters to libgnss++

GSILIB is useful as a **reference for correction workflows and examples**, even
though libgnss++ is intentionally non-GUI.

The most relevant questions for libgnss++ are:

- do we need `IONEX` and richer PPP product handling,
- do we need explicit `IFB / ISB / L2C` correction paths,
- should post-processing docs include more correction-oriented examples,
- which of these belong in solver code versus auxiliary tooling.

## Current libgnss++ position

`libgnss++` already has:

- native `SPP / RTK / PPP / CLAS-style PPP`,
- `SP3/CLK` loading,
- `RTCM SSR` handling,
- sign-off and benchmark tooling,
- local web visualization and docs site.

What it does **not** yet have as first-class features:

- `IONEX` loading,
- explicit `IFB / ISB / L2C` correction workflows,
- a GUI-focused post-processing story like `GSIPOST_GUI`.

## Immediate work items derived from this note

1. Check whether `IFB / ISB / L2C` corrections from GSILIB should become part of the PPP or RTK roadmap.
2. Fold `IONEX` into the same first product-expansion discussion already opened by the `laika` and `MADOCALIB` analyses.
3. Reuse GSILIB as an example source for Japanese post-processing docs, not as a UI target.

## Exit condition for this note

This page is considered "done enough" when:

- `IONEX` and related correction support has a clear roadmap decision,
- and `IFB / ISB / L2C` are either promoted into tracked implementation work or explicitly left out of scope.
