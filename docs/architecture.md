# libgnss++ Architecture Notes

This repo is not a line-by-line C++ port of RTKLIB.

The goal is to keep the useful GNSS algorithms while redesigning the software
boundaries around explicit state, modular processing stages, reproducible
sign-off workflows, and installable tooling.

![Architecture diagram](libgnsspp_architecture.png)

## Design Goals

- Keep solver state explicit instead of hiding behavior behind global runtime
  switches.
- Separate protocol ingest from solver logic so `RINEX`, `RTCM`, `UBX`, and
  direct `QZSS L6` paths can evolve without rewriting the solvers.
- Make quality and performance regressions testable through sign-off commands,
  checked artifacts, and CI gates.
- Keep the stack usable as a native library, CLI toolchain, local web UI,
  Python package, and ROS2 playback node without forking solver logic for each
  interface.

## High-Level Layout

The repository is organized as a stack:

1. `include/libgnss++/core`, `src/core`
   - Shared data model, navigation products, coordinate/time helpers, solution
     loading, signal policy.
2. `include/libgnss++/io`, `src/io`
   - Protocol and file ingest: `RINEX`, `RTCM`, `UBX`, and related formats.
3. `include/libgnss++/algorithms`, `src/algorithms`
   - `SPP`, `RTK`, `PPP`, `LAMBDA`, CLAS/SSR application, and supporting
     modules.
4. `apps/`
   - CLI entrypoints, sign-off wrappers, benchmarking, and the local web UI.
5. `tests/`
   - Unit, CLI, packaging, browser, and dogfooding regressions.

## Explicit State Instead of Hidden Globals

The codebase uses explicit solver state structs rather than a shared mutable
runtime singleton.

- `RTKProcessor` owns RTK filter state and ambiguity-history state.
- `PPPProcessor` owns PPP filter state, precise product context, and
  atmosphere-related tracking state.
- `PreciseProducts` and `SSRProducts` carry sampled orbit/clock/bias products
  as data, not as ambient process-global state.
- Sign-off commands emit machine-readable JSON summaries instead of relying on
  logs as the only source of truth.

This is important because it makes solver behavior testable:

- the same state update can be exercised in unit tests,
- the same command path can be checked in CLI smoke tests,
- the same installed binaries can be dogfooded after `cmake --install`.

## Shared Policy Layer

One of the main redesign steps was removing duplicated signal-selection rules
from parser and solver code.

- `include/libgnss++/core/signal_policy.hpp`

This shared policy layer centralizes:

- signal-to-band mapping,
- constellation-specific priority,
- BeiDou GEO handling,
- preferred observation selection.

`RINEX` ingest, `SPP`, and `RTK` now consult the same policy instead of
embedding slightly different rules in multiple places.

## RTK Pipeline Decomposition

The RTK implementation started as a large monolithic path. It has been split
into explicit stages so the code matches the processing model more closely.

Current RTK modules:

- `rtk_selection`
  - reference satellite selection,
  - double-difference pair construction.
- `rtk_measurement`
  - residual block construction,
  - double-difference covariance assembly,
  - direct ambiguity-space transforms without the older dense `D'PD` path.
- `rtk_update`
  - Kalman update application and measurement-system validation.
- `rtk_ar_selection`
  - subset generation policy for ambiguity resolution.
- `rtk_ar_evaluation`
  - subset extraction, candidate comparison, and fixed-state adoption.
- `rtk_validation`
  - fix validation, jump checks, and conservative hold-fix policy.

This changes the code story from:

- "RTK is one huge function"

to:

- "RTK is a staged pipeline with explicit responsibilities".

The subset-search strategy itself is preserved:

- full-set attempt,
- preferred constellation subsets,
- progressive variance-drop subsets.

Performance work focused on making those stages cheaper without reducing the
search coverage below the current RTKLIB-comparison behavior.

## PPP and CLAS Decomposition

PPP and CLAS-style atmospheric handling also moved toward module boundaries.

- `ppp_atmosphere`
  - CLAS/SSR atmospheric token parsing,
  - nearest-grid resolution,
  - trop/STEC correction application,
  - ionosphere conversion helpers.

This keeps `ppp.cpp` focused on state propagation, measurement modeling, and
ambiguity handling rather than embedding all atmospheric transport logic in the
main solver file.

## Runtime and Tooling Model

The runtime layer is intentionally thin:

- CLI commands call the same solver and I/O code as the library path.
- Sign-off scripts wrap real commands and enforce thresholds rather than
  implementing separate hidden logic.
- `gnss web` renders existing summary artifacts and status JSON rather than
  reimplementing solver logic in a separate service.

This means the same code paths are exercised by:

- local development,
- CI,
- installed-prefix dogfooding,
- browser smoke tests.

## Error Handling Philosophy

The target is graceful failure with structured context.

Examples:

- `gnss live` reports source-open failures with the failing path.
- live summaries include termination mode, decoder errors, realtime factor, and
  effective rate.
- sign-off commands fail on explicit threshold violations rather than silently
  producing a weak result.

This is deliberate: GNSS tooling is operational software, not just offline
solver code.

## Reproducibility as a First-Class Design Feature

The repository treats reproducibility as part of the architecture.

Key mechanisms:

- checked sign-off commands,
- benchmark summary JSON,
- image generation scripts,
- installed-prefix smoke tests,
- browser-level tests for the local web UI,
- optional external-dataset gates such as PPC RTK sign-off.

The point is to make "performance", "quality", and "works after install"
properties that are continuously checked, not just claimed.

## What Is Still Intentionally Thin

Some parts remain lightweight by design:

- the web UI is a local visualization layer, not a large application framework,
- the ROS2 path is playback/telemetry oriented,
- sign-off scripts are wrappers around existing commands, not separate solver
  implementations.

That is intentional. The core design priority is keeping one native GNSS stack
with multiple interfaces, not creating multiple stacks that drift apart.

## Why This Matters

The strongest claim this repo can make is not "it was rewritten in C++".

The stronger, more accurate claim is:

- protocol ingest, solver stages, state ownership, validation, and sign-off
  workflows were redesigned into explicit modules,
- and those modules are exercised by unit tests, CLI tests, installed-prefix
  dogfooding, and benchmark pipelines.

That is the difference between a rewrite and a software architecture.
