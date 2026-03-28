# Validation

## Sign-off commands

Sign-off commands are quality gates around real solver commands. They emit
summary JSON and fail when thresholds are violated.

Main sign-off entrypoints:

- `gnss short-baseline-signoff`
- `gnss rtk-kinematic-signoff`
- `gnss ppp-static-signoff`
- `gnss ppp-kinematic-signoff`
- `gnss live-signoff`
- `gnss ppc-rtk-signoff`
- `gnss odaiba-benchmark --require-*`

## Realtime validation

`gnss live-signoff` verifies:

- termination mode,
- aligned epochs,
- written and fixed solutions,
- decoder errors,
- solver wall time,
- realtime factor,
- effective epoch rate.

## Test layers

The repo validates functionality through multiple layers:

- C++ unit and solver tests
- CLI regressions
- benchmark/image-generation tests
- packaging and installed-prefix smoke tests
- Python bindings tests
- ROS2 playback tests
- browser smoke tests for `gnss web`

Main command:

```bash
ctest --test-dir build --output-on-failure
```

## Dogfooding

Installed-prefix smoke tests verify that:

- installed `gnss` commands run correctly,
- sign-off scripts behave after `cmake --install`,
- generated assets still render correctly outside the source tree.
