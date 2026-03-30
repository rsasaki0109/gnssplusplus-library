# laika Gap Analysis

This page compares the current `libgnss++` product/tooling path against the
areas where `laika` is the most relevant primary reference.

Primary upstream:

- [commaai/laika](https://github.com/commaai/laika)
- [laika README](https://github.com/commaai/laika/blob/master/README.md)

## Scope

`laika` matters here because it is the strongest public reference for:

- product download and cache handling,
- `IONEX` and `DCB` parsing,
- online product acquisition ergonomics,
- a "data helper" layer sitting above raw solver math.

For libgnss++, `laika` is mainly a reference for **product acquisition and
cache design**, not for replacing native C++ solver code.

## Current libgnss++ baseline

Relevant code entrypoints today:

- `src/core/navigation.cpp`
- `include/libgnss++/core/navigation.hpp`
- `apps/gnss_ppp.cpp`
- `apps/gnss_ppp_static_signoff.py`
- `apps/gnss_ppp_kinematic_signoff.py`

## Already aligned enough to build on

| Area | libgnss++ status |
|---|---|
| Native precise product containers | `PreciseProducts` and `SSRProducts` are explicit in-core data structures |
| PPP sign-off around products | PPP sign-off commands already exist for broadcast and precise-product paths |
| CLAS/MADOCA transport hooks | RTCM SSR and sampled correction inputs are already wired into PPP |
| Packaging and dogfooding | Installed-prefix CLI validation already exists |

## Main gaps versus a laika-style product pipeline

| Gap | Current libgnss++ state | Why it matters |
|---|---|---|
| Product fetch/cache layer | No first-class downloader/cache utility exists yet | `laika` shows that product handling can be made much easier for end users |
| `IONEX` support | No in-tree `IONEX` loader is present today | Important for widening PPP product coverage |
| `DCB` support | No in-tree `DCB` loader is present today | Useful for richer PPP correction models |
| Product mirror/fallback logic | Current workflow expects user-supplied files rather than managed mirrors and fallback URLs | This is the main usability win from `laika` |
| Product-specific regression family | Existing regressions focus on solver and sign-off behavior, not downloader/cache correctness | Once fetch/cache exists, it needs its own failure-mode tests |

## Important design difference

`laika` is a Python GNSS processing library with a strong focus on readable data
preparation and online product acquisition.

`libgnss++` already has the solver core and non-GUI operations stack, so the
adoption target is narrower:

- bring in the useful product-fetch and cache ideas,
- keep solver ownership in native C++,
- keep installable CLI and sign-off behavior stable.

## What should *not* be copied directly

Do not re-center libgnss++ around a Python-first processing model.

Keep these libgnss++ constraints:

- native C++ solver core,
- installable CLI first,
- product handling that works in CI and installed-prefix runs,
- explicit regression coverage for fetch/cache failures.

## Immediate work items derived from this gap

1. Decide whether product fetch belongs as a standalone command such as `gnss fetch-products`.
2. Decide the first fetch scope: `SP3/CLK` only, or `SP3/CLK/IONEX/DCB`.
3. Define cache layout and cache invalidation behavior before implementation.
4. Add downloader/cache regressions before wiring the feature into PPP docs.

## Exit condition for this analysis

This page is considered "done enough" when the product-fetch/cache plan is
clear enough to cut a small implementation PR.
