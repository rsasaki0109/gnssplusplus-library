# SignalSim Notes

This page tracks what is worth borrowing from `SignalSim` as a future
integration and validation reference, without turning libgnss++ into a signal
simulator.

Primary upstream:

- [globsky/SignalSim](https://github.com/globsky/SignalSim)
- [SignalSim README](https://github.com/globsky/SignalSim/blob/main/README.md)

## Scope

`SignalSim` matters here because it is a strong public reference for:

- synthetic GNSS scenario generation,
- observation generation from scenario descriptions,
- baseband correlation and digital IF generation,
- JSON-driven simulation inputs,
- replayable synthetic datasets for receiver and solver validation.

For libgnss++, the useful parts are:

- synthetic validation inputs for `SPP / RTK / PPP / CLAS` regression,
- future `IF` or replay-oriented test fixtures,
- scenario-driven benchmark generation beyond the current bundled datasets,
- a cleaner story for validating edge cases that are hard to collect from real
  datasets alone.

The simulator itself is **not** the target.

## What SignalSim appears to cover

Based on the public repository README:

- multi-stage simulation outputs:
  - reference receiver trajectory,
  - observations,
  - baseband correlation results,
  - digital IF data,
  - real-time signal simulation,
- multi-constellation support for `GPS / BDS / Galileo / GLONASS`,
- multiple frequency bands,
- `RINEX 2/3/4` observation and navigation data support,
- JSON-based scenario configuration,
- IF sample generation with configurable sampling rate and bit depth.

## Why this matters to libgnss++

SignalSim is useful as a **synthetic dataset and validation reference**.

The most relevant questions for libgnss++ are:

- should synthetic scenario-driven datasets become part of sign-off and CI,
- should `SignalSim` outputs be accepted directly or only through converted
  `RINEX` / observation inputs,
- can it be used to widen PPP/CLAS edge-case regression coverage,
- can it complement real-data datasets such as Odaiba and PPC rather than
  replace them.

## Current libgnss++ position

libgnss++ already has:

- bundled static and kinematic samples,
- synthetic unit tests around solver internals,
- sign-off and benchmark flows for real data,
- local web summaries and JSON-first regression outputs.

What it does **not** yet have as first-class features:

- a scenario-driven synthetic dataset pipeline,
- direct `SignalSim` interoperability helpers,
- a documented `SignalSim -> libgnss++` replay or sign-off workflow,
- synthetic IF-oriented solver validation outside current low-level tests.

## Immediate work items derived from this note

1. Decide whether `SignalSim` should be treated as:
   - an external scenario generator whose outputs are converted to `RINEX`, or
   - a supported synthetic input source for dedicated validation commands.
2. Evaluate whether a small `SignalSim -> sign-off` recipe belongs in docs
   before any direct integration code is added.
3. Decide whether future PPP/CLAS regression should include one synthetic
   scenario family in addition to real datasets.
4. Keep SignalSim as a validation/reference lane, not as a demand to add a
   native signal simulator inside libgnss++.

## Exit condition for this note

This page is considered "done enough" when:

- there is a clear decision on `SignalSim` as conversion-only versus supported
  validation interoperability,
- and any future work is captured as a small PR family rather than a simulator
  rewrite.
