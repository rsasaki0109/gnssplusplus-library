# CLASLIB Gap Analysis

This page compares the current `libgnss++` CLAS/MADOCA path against the areas
where `CLASLIB` is the most relevant primary reference.

Primary upstream:

- [QZSS-Strategy-Office/claslib](https://github.com/QZSS-Strategy-Office/claslib)
- [CLASLIB README](https://github.com/QZSS-Strategy-Office/claslib/blob/main/README.md)

## Scope

`CLASLIB` matters here because it is the strongest public reference for:

- Compact SSR decode
- CLAS grid handling
- `SSR2OSR`
- `SSR2OBS`
- PPP-RTK / VRS-RTK flows
- newer `QZSS L6` subtype support such as `SubType12`

For libgnss++, this is the main upstream reference for the **correction
transport and application boundary**, not just for low-level packet parsing.

## Current libgnss++ baseline

Relevant code entrypoints today:

- `apps/gnss_qzss_l6_info.py`
- `apps/gnss_clas_ppp.py`
- `include/libgnss++/algorithms/ppp_atmosphere.hpp`
- `src/algorithms/ppp_atmosphere.cpp`
- `src/algorithms/ppp.cpp`
- `src/core/navigation.cpp`
- `data/clas_grid.def`
- `src/algorithms/clas_grid_points.inc`

## Already aligned enough to build on

| Area | libgnss++ status |
|---|---|
| Raw `QZSS L6` parsing | Implemented through `gnss qzss-l6-info` |
| Compact SSR inventory and export | Implemented through CSV export and compact sampled transport |
| Supported direct raw subtype coverage | Current path handles `1/2/3/4/5/6/7/8/9/10/11/12` |
| Sampled correction ingestion | Implemented through `SSRProducts` and `gnss clas-ppp` |
| Grid-aware atmosphere application | Implemented with nearest-grid selection and atmospheric token application |
| CLAS/MADOCA sign-off path | Implemented through `gnss clas-ppp` and PPP-side atmospheric correction counters |

## Main gaps versus a CLASLIB-style reference stack

| Gap | Current libgnss++ state | Why it matters |
|---|---|---|
| Native `SSR2OSR` equivalent | Current path converts raw/compact transport into sampled correction rows and applies them inside PPP; it does not expose a first-class observation-space conversion utility | `CLASLIB` treats `SSR2OSR` as a core utility boundary, which is useful for verification and interoperability |
| Native `SSR2OBS` / virtual-observation path | No virtual observation generator exists today | This is the basis for `VRS-RTK` style workflows in `CLASLIB` |
| Full `PPP-RTK / VRS-RTK` mode split | Current libgnss++ focuses on PPP with sampled CLAS/MADOCA corrections, not a separate `PPP-RTK` and `VRS-RTK` solver story | `CLASLIB` makes those modes explicit |
| In-core C++ Compact SSR decoder | The raw `QZSS L6` and Compact SSR decode path still lives mainly in Python helper tools | A native C++ path would make replay, profiling, and solver coupling cleaner |
| Dual-channel L6 workflow | `CLASLIB` explicitly documents two-channel L6 handling in newer versions; libgnss++ currently exposes a single-source raw L6 path | This affects parity with richer real-time CLAS ingestion scenarios |
| Broader grid / OSR validation matrix | Current atmospheric application is covered, but the checked regression matrix is still thinner than a full `SSR2OSR` / `VRS-RTK` reference stack | Grid and atmosphere bugs are subtle, so breadth matters |

## Important design difference

The current libgnss++ design intentionally stops at a **sampled correction
application model**:

- raw `QZSS L6` or compact transport is decoded,
- corrections are flattened into sampled rows,
- PPP consumes those rows with explicit counters and sign-off thresholds.

That is a valid architecture, but it is not the same architectural boundary as
`CLASLIB`, where `cssr.c`, `cssr2osr.c`, `grid.c`, and `ppprtk.c` are explicit
core components.

This means the next step is not "copy CLASLIB", but:

- decide which CLASLIB boundaries should also exist in libgnss++,
- keep them native and testable.

## What should *not* be copied directly

Do not re-import a full RTKLIB-style runtime layout just to mirror CLASLIB.

Keep these libgnss++ constraints:

- explicit product containers,
- explicit PPP / RTK solver ownership,
- sign-off JSON and browser-visible summaries,
- shared correction state that is reusable across CLI and web tooling.

The goal is to borrow:

- subtype coverage ideas,
- grid/application ideas,
- validation utilities,
- mode boundaries where they are genuinely useful.

## Immediate work items derived from this gap

1. Decide whether libgnss++ needs a native `SSR2OSR`-style utility boundary.
2. Decide whether `VRS-RTK` belongs in scope, or should remain out of scope.
3. Move more Compact SSR decoding from Python-side helpers into native C++.
4. Add wider CLAS sign-off datasets once a richer native transport path exists.
5. Evaluate whether dual-channel `L6` matters for the intended non-GUI scope.

## Exit condition for this analysis

This page is considered "done enough" when the CLAS roadmap is clear about:

- what remains sampled-correction-only by design,
- what should graduate into native C++ correction modules,
- and whether `SSR2OSR` / `VRS-RTK` are future scope or intentional non-goals.
