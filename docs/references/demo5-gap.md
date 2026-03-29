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
| Explicit `arfilter` policy switch | Present as an explicit subset-candidate ratio-margin knob in solver CLI/config, but intentionally narrower than demo5's broader tuning surface | Useful as a practical false-fix guard without collapsing the staged RTK design |
| Explicit `PAR` implementation | `PARTIAL` exists in config, but there is no separately documented demo5-style partial ambiguity-resolution path | This matters if we want to claim parity with demo5 tuning ideas rather than just naming compatibility |
| Doppler-aided slip threshold path | A basic Doppler/carrier consistency slip guard now exists in the RTK bias-update path, but it is not yet a broader demo5-style tuning surface with receiver-class-specific knobs | demo5 uses Doppler/carrier disagreement as an additional practical slip signal |
| Code-based slip path | A basic single-difference code-minus-phase slip guard now exists in the RTK bias-update path, but it is intentionally smaller than a full demo5 tuning surface | This is another low-cost-receiver guard often useful when carrier-only logic is not enough |
| Receiver-oriented tuning presets | Named CLI presets now exist for `survey`, `low-cost`, and `moving-base`, and PPC RTK sign-off now carries city-specific tuning defaults (`Tokyo`: low-cost + arfilter, `Nagoya`: low-cost + no-arfilter) | demo5 is strong partly because it is operationally easy to tune for receiver classes |
| Hold tuning knobs such as `varholdamb` / `gainholdamb` | `min_hold_count` and hold-active ratio threshold are now public knobs and are exercised through PPC RTK sign-off tuning profiles, but the broader hold policy remains intentionally simpler than demo5 | Useful when pushing harder on low-cost data without forking the solver |

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

1. Audit whether `PARTIAL` should become a real documented mode or remain a placeholder.
2. Evaluate the current Doppler-aided slip detection against PPC Tokyo and Nagoya and decide whether it needs a broader named tuning surface.
3. Evaluate whether the current basic code-slip guard needs a broader named tuning surface against the same PPC windows.
4. Decide whether the current preset set is enough or whether receiver-class-specific presets are still worth exposing.
5. Decide whether more hold-tuning knobs should become public config or remain internal policy.

## Exit condition for this analysis

This page is considered "done enough" when the RTK tuning roadmap is clear
about:

- which demo5-style heuristics are already represented,
- which ones are genuinely still missing,
- and which should be adopted only if they beat current PPC / RTKLIB gates.
