# Reference Analyses

This section tracks the upstream OSS implementations used as **primary design
references** for the next round of libgnss++ improvements.

The rule is simple:

- study the upstream projects directly,
- translate the useful ideas into native libgnss++ modules, tests, and
  sign-off gates,
- avoid adopting large external stacks wholesale.

## Current reference set

| Upstream | Main value for libgnss++ | Current status |
|---|---|---|
| [MADOCALIB](https://github.com/QZSS-Strategy-Office/madocalib) | PPP, PPP-AR, L6E/L6D correction handling | Gap analysis published |
| [CLASLIB](https://github.com/QZSS-Strategy-Office/claslib) | Compact SSR, CLAS grid/atmosphere, PPP-RTK ideas | Gap analysis published |
| [MALIB](https://github.com/JAXA-SNU/MALIB/tree/feature/1.2.0) | replay, config, regression, MADOCA-PPP ops patterns | Ops gap published |
| [rtklibexplorer/RTKLIB](https://github.com/rtklibexplorer/RTKLIB) | RTK tuning for low-cost and mixed-quality receivers | Gap analysis published |
| [commaai/laika](https://github.com/commaai/laika) | precise product fetch/cache and product tooling ideas | Gap analysis published |
| [tomojitakasu/PocketSDR](https://github.com/tomojitakasu/PocketSDR) | SDR frontend, IF capture/replay, PocketSDR `convbin`, `L6D/L6E` ingest ideas | Future note published |
| [globsky/SignalSim](https://github.com/globsky/SignalSim) | synthetic scenario, observation, and IF generation for validation | Future note published |

## Execution order

1. `MADOCALIB` gap analysis
2. `CLASLIB` gap analysis
3. `demo5 / rtklibexplorer` RTK tuning delta review
4. `MALIB` replay/config/testing review
5. `laika` product-fetch/cache review
6. `PocketSDR` replay/import interoperability review
7. `SignalSim` synthetic-validation interoperability review

## Why this ordering

- `RTK` quality and runtime are already much stronger than they were earlier in
  this project.
- The next large payoff is `PPP / PPP-AR`.
- After that, `CLAS/MADOCA` accuracy and correction coverage become the next
  meaningful step.
- Operations and product tooling should follow once solver-side gaps are
  clearer.

## Available analyses

- [MADOCALIB gap analysis](madocalib-gap.md)
- [CLASLIB gap analysis](claslib-gap.md)
- [demo5 / rtklibexplorer gap analysis](demo5-gap.md)
- [MALIB operations gap analysis](malib-ops-gap.md)
- [laika gap analysis](laika-gap.md)
- [PocketSDR notes](pocketsdr-notes.md)
- [SignalSim notes](signalsim-notes.md)
- [GSILIB notes](gsilib-notes.md)
- [Upstream adoption roadmap](roadmap.md)

## What "done" means here

For this section, "done" means:

- each upstream has a published note or gap analysis,
- the note points to concrete PR families rather than vague inspiration,
- the adoption roadmap stays in terms of native libgnss++ modules, tests, and
  sign-off gates.
