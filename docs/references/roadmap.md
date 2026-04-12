# Upstream Adoption Roadmap

This is the long-running plan for using external OSS as **design references**
without turning libgnss++ into an integration fork.

## Principle

- Study upstream code directly.
- Import ideas selectively.
- Keep libgnss++ native module boundaries intact.
- Every imported idea must earn a local regression or sign-off gate.

## Phase A — MADOCALIB

Focus:

- PPP measurement model
- PPP-AR gating and state management
- L6E / L6D correction flow
- product coverage around precise PPP

Deliverables:

- `madocalib-gap.md`
- a prioritized list of PPP-side PRs

Expected PR families:

- `ppp-measurement-improvement`
- `ppp-ar-gating`
- `ppp-product-pipeline`

## Phase B — CLASLIB

Focus:

- Compact SSR subtype coverage
- grid / atmosphere handling
- `SSR2OSR`-style thinking
- PPP-RTK / VRS-RTK boundary

Deliverables:

- `claslib-gap.md`
- subtype-by-subtype adoption table

Expected PR families:

- `clas-grid-application`
- `clas-subtype-expansion`
- `clas-product-loader`

## Phase C — demo5 / rtklibexplorer

Focus:

- slip detection tuning
- `PAR`
- `arfilter`
- measurement variance tuning
- low-cost receiver presets

Deliverables:

- `demo5-gap.md`
- a small set of RTK tuning PRs tied to PPC/Odaiba results

Expected PR families:

- `rtk-demo5-slip-tuning`
- `rtk-demo5-par-tuning`
- `rtk-demo5-varerr`

## Phase D — MALIB

Focus:

- replay workflow
- config layout
- sample data handling
- regression organization
- real-time replay validation

Deliverables:

- `malib-ops-gap.md`
- CI / replay / sign-off improvements

Expected PR families:

- `replay-and-config-hardening`
- `live-replay-signoff-expansion`

## Phase E — laika

Focus:

- precise product fetch
- cache layout
- product conversion tooling

Deliverables:

- `laika-gap.md`
- product downloader and cache design

Expected PR families:

- `laika-fetch-products`
- `laika-ppp-pipeline`

## Supplemental reference — PocketSDR

Use `PocketSDR` mainly as an SDR frontend, IF replay, and `L6D/L6E`
interoperability reference.

Focus:

- recorded IF / SDR replay workflow,
- `PocketSDR convbin` interop,
- future `QZSS L6` replay datasets,
- keeping SDR tracking outside the libgnss++ core solver.

Expected future PR families:

- `pocketsdr-convbin-interop`
- `pocketsdr-l6-replay-signoff`
- `pocketsdr-if-import-evaluation`

## Supplemental reference — SignalSim

Use `SignalSim` mainly as a synthetic scenario and validation reference.

Focus:

- synthetic observation datasets,
- future `SignalSim -> RINEX` or replay recipes,
- solver edge-case regression beyond real-data collections,
- keeping signal simulation outside the libgnss++ core runtime.

Expected future PR families:

- `signalsim-rinex-interop`
- `signalsim-synthetic-signoff`
- `signalsim-edge-case-regression`

## Supplemental reference — GSILIB

Use `GSILIB` mainly as a correction-workflow and Japanese post-processing
reference.

Focus:

- `IONEX`
- `IFB / ISB / L2C` correction examples
- post-processing runbook ideas

## Ground rules for every adoption PR

- `1 PR = 1 user-visible value`
- `1 PR = 1 regression or sign-off improvement`
- no mixed protocol/solver/docs mega-PRs
- measured quality must stay visible through checked summaries

## Benchmark and sign-off datasets

The current plan assumes these stay in the loop:

- bundled static PPP sample
- bundled kinematic sample
- UrbanNav Tokyo Odaiba
- PPC Tokyo run1
- PPC Nagoya run1
- recorded live replay datasets

## Success criteria

The adoption plan succeeds when:

- PPP quality moves up without losing reproducibility,
- CLAS coverage grows without collapsing module boundaries,
- RTK tuning improvements are benchmarked against `RTKLIB`,
- replay and sign-off operations become more routine, not more ad hoc.
