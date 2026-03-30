# demo5 / rtklibexplorer Gap Analysis

This page compares the current `libgnss++` RTK tuning path against the areas
where `rtklibexplorer/RTKLIB` is the most relevant primary reference.

Primary upstream:

- [rtklibexplorer/RTKLIB](https://github.com/rtklibexplorer/RTKLIB)
- demo5 branch `readme.txt`: [demo5 overview](https://raw.githubusercontent.com/rtklibexplorer/RTKLIB/demo5/readme.txt)

## Scope

`demo5` matters here because it is the strongest public reference for:

- low-cost and mixed-quality RTK tuning,
- practical ambiguity-resolution heuristics,
- slip-detection tuning,
- measurement-variance tuning,
- receiver-oriented presets.

For libgnss++, this is the main upstream reference for the next RTK tuning
wave, not for overall architecture.

## Current libgnss++ baseline

Relevant code entrypoints today:

- `include/libgnss++/algorithms/rtk.hpp`
- `src/algorithms/rtk.cpp`
- `include/libgnss++/algorithms/rtk_selection.hpp`
- `include/libgnss++/algorithms/rtk_measurement.hpp`
- `include/libgnss++/algorithms/rtk_update.hpp`
- `include/libgnss++/algorithms/rtk_ar_selection.hpp`
- `include/libgnss++/algorithms/rtk_ar_evaluation.hpp`
- `include/libgnss++/algorithms/rtk_validation.hpp`
- `apps/gnss_ppc_demo.py`
- `apps/gnss_ppc_rtk_signoff.py`

## Already aligned enough to build on

| Area | libgnss++ status |
|---|---|
| RTKLIB-style `varerr` path | Present in `RTKProcessor::varerr()` and used in the RTK measurement model |
| Multi-stage RTK pipeline | Already split into selection, measurement, update, AR selection, AR evaluation, and validation modules |
| Geometry-free slip guard | Present and tuned in `rtk.cpp`, including a relaxed kinematic threshold path |
| RTKLIB side-by-side quality checks | Present through `gnss ppc-demo` and `gnss ppc-rtk-signoff` |
| Partial-AR-capable mode enum | `RTKConfig::AmbiguityResolutionMode::PARTIAL` exists in the public config |
| Real-data benchmarking | PPC Tokyo and Nagoya sign-off plus RTKLIB delta checks are already in place |

## Main gaps versus a demo5-style tuning stack

| Gap | Current libgnss++ state | Why it matters |
|---|---|---|
| Explicit `arfilter` policy switch | No direct `arfilter`-style runtime policy is exposed yet | demo5 uses this as a practical false-fix guard for noisy low-cost data |
| Explicit `PAR` implementation | `PARTIAL` exists in config, but there is no separately documented demo5-style partial ambiguity-resolution path | This matters if we want to claim parity with demo5 tuning ideas rather than just naming compatibility |
| Doppler-aided slip threshold path | No explicit `detslp_dop` equivalent was found in the current RTK solver path | demo5 uses Doppler/carrier disagreement as an additional practical slip signal |
| Code-based slip path | No explicit `detslp_code`-style RTK path is exposed today | This is another low-cost-receiver guard often useful when carrier-only logic is not enough |
| Receiver-oriented tuning presets | Current tuning is mostly coded into the solver and sign-off thresholds, not yet exposed as named presets | demo5 is strong partly because it is operationally easy to tune for receiver classes |
| Hold tuning knobs such as `varholdamb` / `gainholdamb` | Conservative hold behavior exists, but these knobs are not surfaced as user-facing tuning parameters | Useful when pushing harder on low-cost data without forking the solver |

## Important design difference

`demo5` is mainly a **tuning-oriented fork**.

`libgnss++` is trying to keep:

- native modular solver boundaries,
- sign-off-based quality gates,
- reproducible RTKLIB comparisons,
- a single RTK core shared by CLI, web, and bindings.

So the adoption target is not a raw port of demo5 logic. The adoption target is
to bring over:

- the tuning ideas that demonstrably improve real datasets,
- the configuration surface that helps operational use,
- the regressions that stop quality from drifting.

## What should *not* be copied directly

Do not collapse the current staged RTK design back into a large pile of
heuristics just to mirror demo5 option names.

Keep these libgnss++ constraints:

- staged RTK pipeline,
- explicit validation and hold logic,
- PPC / RTKLIB side-by-side sign-off,
- benchmarkable tuning changes.

If a demo5-inspired change cannot be expressed as:

- a small tuning rule,
- a named config knob,
- and a regression on PPC/Odaiba,

it probably should not be imported yet.

## Immediate work items derived from this gap

1. Decide whether `arfilter` should become an explicit libgnss++ config option.
2. Audit whether `PARTIAL` should become a real documented mode or remain a placeholder.
3. Evaluate Doppler-aided slip detection against PPC Tokyo and Nagoya.
4. Evaluate code-based slip detection against the same PPC windows.
5. Decide whether receiver-oriented preset profiles are worth exposing in CLI and sign-off wrappers.

## Exit condition for this analysis

This page is considered "done enough" when the RTK tuning roadmap is clear
about:

- which demo5-style heuristics are already represented,
- which ones are genuinely still missing,
- and which should be adopted only if they beat current PPC / RTKLIB gates.
