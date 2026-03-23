# Development Notes

## Build

```bash
cmake --build build -j$(nproc)
```

## Test

```bash
# Regression tests (all 3 datasets)
bash tests/run_regression.sh

# Unit tests
./build/tests/run_tests
```

## Test Data Setup

Symlinks in `data/` point to active dataset:
```bash
# Kinematic (default)
cd data && ln -sf rover_kinematic.obs rover.obs && ln -sf base_kinematic.obs base.obs && ln -sf navigation_kinematic.nav navigation.nav

# Short static (36m)
cd data && ln -sf short_baseline/rover.obs rover.obs && ln -sf short_baseline/base.obs base.obs && ln -sf short_baseline/navigation.nav navigation.nav

# Long static (3.3km) — use RTK_MODE=static
cd data && ln -sf rover_static.obs rover.obs && ln -sf base_static.obs base.obs && ln -sf navigation_static.nav navigation.nav
```

## Architecture

- `RTKProcessor` is the main class. `processRTKEpoch()` is the entry point.
- State vector: `[pos(3), N1_SD(MAXSAT), N2_SD(MAXSAT)]` — NX=83, sparse (x=0 for unused)
- DD formed in observation model, not state. SD ambiguities in state.
- `resolveAmbiguities()` does SD→DD transform, LAMBDA, WL-NL fallback.
- KF filter uses `kalmanFilter()` in `kalman.hpp` (Eigen, skips x=0 states).
- LAMBDA uses `lambdaSearch()` in `lambda.cpp` (LD factorization + reduction + search).

## Key Files

| File | Lines | Role |
|------|-------|------|
| `src/algorithms/rtk.cpp` | ~1200 | RTK core (DD model, KF, AR, holdamb) |
| `src/algorithms/spp.cpp` | ~450 | SPP (WLS + iono + trop) |
| `src/algorithms/lambda.cpp` | ~220 | LAMBDA integer search |
| `src/io/rinex.cpp` | ~900 | RINEX 2/3 parser |

## No RTKLIB Dependency

All algorithms are self-contained C++17 + Eigen. No RTKLIB source files linked.
