# MADOCALIB Gap Analysis

This page compares the current `libgnss++` PPP stack against the areas where
`MADOCALIB` is the most relevant primary reference.

Primary upstream:

- [QZSS-Strategy-Office/madocalib](https://github.com/QZSS-Strategy-Office/madocalib)
- `readme.txt`: [MADOCALIB overview](https://github.com/QZSS-Strategy-Office/madocalib/blob/master/readme.txt)

## Scope

`MADOCALIB` matters here because it is a public reference implementation for:

- `PPP`
- `PPP-AR`
- `L6E / L6D`
- `cssr2ssr`-style correction handling

This is the strongest upstream reference for the next `PPP` improvement wave in
`libgnss++`.

## Current libgnss++ baseline

Relevant code entrypoints today:

- `include/libgnss++/algorithms/ppp.hpp`
- `src/algorithms/ppp.cpp`
- `include/libgnss++/algorithms/ppp_atmosphere.hpp`
- `src/core/navigation.cpp`
- `apps/gnss_ppp.cpp`
- `apps/gnss_clas_ppp.py`
- `apps/gnss_ppp_static_signoff.py`

## Already aligned enough to build on

| Area | libgnss++ status |
|---|---|
| PPP float filter | Implemented in native `PPPProcessor` with explicit filter state |
| PPP ambiguity resolution | Implemented with `enable_ambiguity_resolution`, ratio threshold, and real-data sign-off |
| Precise product loading | `SP3` and `CLK` products are loaded through `PreciseProducts` |
| RTCM SSR to PPP path | Implemented through sampled `SSRProducts` and direct RTCM SSR conversion |
| CLAS/MADOCA transport | Implemented through compact sampled transport and raw `QZSS L6` preprocessing |
| Atmospheric application | Factored into `ppp_atmosphere` and applied inside PPP |
| Geophysical corrections | Solid Earth tides, ocean loading hooks, receiver ANTEX support are present |
| Validation | Static/kinematic PPP sign-off commands and CLI regressions already exist |

## Main gaps versus a MADOCALIB-style reference stack

| Gap | Current libgnss++ state | Why it matters |
|---|---|---|
| Native in-core `L6E/L6D -> correction object` path | Current CLAS/MADOCA transport still depends on Python-side preprocessing in `gnss_clas_ppp.py` / `gnss_qzss_l6_info.py` before the C++ PPP core sees sampled corrections | A first-class C++ path would make correction handling easier to test, replay, and profile |
| First-class `IONEX / DCB` workflow | No in-tree `IONEX` or `DCB` loader is present today | `MADOCALIB`-style PPP studies often assume a fuller product pipeline than `SP3/CLK` alone |
| Explicit multi-frequency PPP beyond the current IF-centric path | Current solver structure is centered on ionosphere-free combinations and PPP ambiguity state handling in `ppp.cpp` | A wider multi-frequency story needs to be explicit before claiming parity with richer PPP references |
| Broader PPP-AR reference matrix | Current real-data sign-off is useful but still relatively narrow compared with a dedicated upstream PPP reference stack | `PPP-AR` robustness needs more dataset coverage before it can be considered mature |
| Correction ordering audit against upstream reference behavior | The current code works, but the exact correction-application order has not yet been documented as a deliberate compatibility target | This is the kind of subtle mismatch that creates hard-to-debug centimeter-level differences |

## What should *not* be copied directly

The goal is not to import RTKLIB-style runtime structure back into libgnss++.

Keep these libgnss++ design constraints:

- explicit solver state over globals,
- reusable product containers over ambient process state,
- sign-off JSON and regression gates as first-class outputs,
- one solver core reused by CLI, web, Python, and ROS2 paths.

So the adoption target is:

- **measurement and correction ideas**
- **product coverage**
- **sign-off coverage**

and **not** a direct code transplant.

## Immediate work items derived from this gap

1. Document the current PPP correction-application order inside `ppp.cpp`.
2. Decide whether `IONEX` and `DCB` belong in the first product expansion wave.
3. Move more `CLAS/MADOCA` transport handling from Python preprocessing toward
   native C++ product ingestion.
4. Widen PPP-AR sign-off datasets and keep the results as checked artifacts.

## Exit condition for this analysis

This page is considered "done enough" when each of the gaps above has been
translated into a concrete small PR or explicitly deferred with a reason.
