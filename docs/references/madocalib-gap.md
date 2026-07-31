# MADOCALIB Gap Analysis

For the active issue-#148 execution status and quantitative exit gates, see the
[MADOCALIB Native Migration Ledger](../madocalib_native_migration.md).  This
page remains the broader architectural gap analysis.

This page compares the current `libgnss++` PPP stack against the areas where
`MADOCALIB` is the most relevant primary reference.

Issue #148 completed the supported native MADOCA migration on 2026-07-30.  The
current release contract is the 18-case MIZU/ALIC matrix in the migration
ledger; this page now records broader architectural follow-ups rather than
blockers for that completed contract.

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
- `apps/native/gnss_ppp.cpp`
- `apps/commands/positioning/gnss_clas_ppp.py`
- `apps/commands/positioning/gnss_ppp_static_signoff.py`

## Already aligned enough to build on

| Area | libgnss++ status |
|---|---|
| PPP float filter | Implemented in native `PPPProcessor` with explicit filter state |
| PPP ambiguity resolution | Implemented with `enable_ambiguity_resolution`, ratio threshold, and real-data sign-off |
| Precise product loading | `SP3` and `CLK` products are loaded through `PreciseProducts` |
| RTCM SSR to PPP path | Implemented through sampled `SSRProducts` and direct RTCM SSR conversion |
| CLAS/MADOCA transport | Implemented through compact sampled transport and raw `QZSS L6` preprocessing |
| Native MADOCA L6 products | L6E SSR materialization and L6D causal ionosphere snapshots/application are implemented in C++ and parity-tested |
| Atmospheric application | Factored into `ppp_atmosphere` and applied inside PPP |
| Geophysical corrections | Solid Earth tides, ocean loading hooks, receiver ANTEX support are present |
| Validation | A versioned 18-case MIZU/ALIC × PPP/PPP-AR/PPP-AR-ion × 1 h/6 h/24 h release matrix gates status, trajectory, rows, artifacts, and runtime |

## Main gaps versus a MADOCALIB-style reference stack

| Gap | Current libgnss++ state | Why it matters |
|---|---|---|
| First-class `IONEX / DCB` workflow | Native loaders and PPP hooks are implemented; production workflow and sign-off coverage remain narrower than SP3/CLK | `MADOCALIB`-style PPP studies need these products to be reproducible across normal CLI and validation workflows |
| Triple/quad-frequency solver expansion | The completed #148 profiles use the frozen supported contract and do not claim complete MADOCALIB triple/quad-frequency PPP-AR parity | [Issue #387](https://github.com/rsasaki0109/gnssplusplus-library/issues/387) requires dedicated fixtures, ambiguity/state contracts, oracle traces, and release baselines before support is claimed |
| Broader PPP-AR reference matrix | The MIZU/ALIC 18-case gate is the accepted #148 release baseline; additional climates, receivers, and days remain useful expansion | New datasets should extend a versioned baseline without weakening the frozen regression gates |
| Correction ordering audit against upstream reference behavior | The native order is now an explicit, tested contract in `ppp_correction_contract.hpp`; whole-run parity still guards numerical behavior | Future reordering is reviewable because it changes a named contract rather than incidental control flow |

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

1. Keep the correction-order contract and its MADOCALIB sign conventions covered
   when adding new correction sources.
2. Finish integrating the existing `IONEX` and `DCB` loaders across production
   workflows and sign-off datasets.
3. Keep native L6E/L6D materialization, row, and whole-solver release gates
   green when adding correction sources.
4. Use [#387](https://github.com/rsasaki0109/gnssplusplus-library/issues/387)
   for triple/quad-frequency expansion and widen PPP-AR datasets as versioned
   checked artifacts.

## Exit condition for this analysis

This analysis is complete for issue #148: its native L6/PPP gaps were
implemented and release-gated, while triple/quad-frequency expansion is
explicitly deferred to
[#387](https://github.com/rsasaki0109/gnssplusplus-library/issues/387).  The
remaining rows above are independent product and dataset breadth improvements,
not hidden #148 exit requirements.
