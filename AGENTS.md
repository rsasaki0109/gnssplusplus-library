# Agent guide

Practical notes for agents and contributors working in this repository.

## Build & test

```bash
cmake -S . -B build
cmake --build build -j$(nproc)
ctest --test-dir build -j$(nproc)   # expect 71/71
```

`ctest` covers the C++ gtest suites plus the Python lanes (CLI tools,
bindings, web UI, ROS2 node smoke). A few Python CLI tests are timing
sensitive; if one fails under `-j`, re-run it serially before treating it
as a regression.

The `RTKRealDataTest.*` gtests skip unless `data/{rover,base,navigation}`
exist; they assert against a historical fixture dataset that is not the
PPC data (see below), so do not point those symlinks at PPC runs.

## Optional GTSAM backend

`find_package(GTSAM QUIET CONFIG PATHS /usr/local/lib/cmake/GTSAM)` gates
the FGO GTSAM backend (`fgo_gtsam_backend.cpp`,
`fgo_gtsam_fixed_lag.cpp`). The backend needs GTSAM 4.3.x
(`gtsam/navigation/CarrierPhaseFactor.h` is not in 4.2).

Local setup used for verification:

```bash
# built from source with system Eigen, no TBB:
cmake -S <gtsam-src> -B <gtsam-build> -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=$HOME/.local -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_WITH_TBB=OFF -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_UNSTABLE=OFF
cmake --build <gtsam-build> -j$(nproc) && cmake --install <gtsam-build>

# configure this repo against it:
cmake -S . -B build -DGTSAM_DIR=$HOME/.local/lib/cmake/GTSAM

# runtime: gtsam's bundled libs live in ~/.local/lib
export LD_LIBRARY_PATH=$HOME/.local/lib:${LD_LIBRARY_PATH:-}
ctest --test-dir build -j8
```

Do not replace `LD_LIBRARY_PATH`; append to it or ROS 2 libraries stop
resolving.

## Behavior-preserving refactor gate

Solver refactors must be pure code motion. Verify on real data:

```bash
ln -sf PPC-Dataset/tokyo/run1/rover.obs data/rover.obs
ln -sf PPC-Dataset/tokyo/run1/base.obs  data/base.obs
ln -sf PPC-Dataset/tokyo/run1/base.nav  data/navigation.nav
./build/examples/rtk_positioning data > after.txt 2>&1
```

Diff `after.txt` against a baseline captured from the pre-change tree
(e.g. a `git worktree` at the base commit). Output must be byte-identical.
Remove the symlinks afterwards so `RTKRealDataTest.*` returns to skipping.

## Conventions

- Conventional commits (`feat:`, `fix:`, `refactor:`, `build:`, `docs:`,
  `test:`).
- Solver implementations follow the per-concern TU pattern:
  `ppp_*.cpp`, `rtk_*.cpp` / `fgo_*.cpp` with shared helpers as inline
  functions in a sibling `*_internal.hpp`
  (`libgnss::ppp_internal` / `rtk_internal` / `fgo_internal` /
  `fgo_gtsam_internal`). Add new solver sources to the canonical lists at
  the top of the root `CMakeLists.txt`; RTK runtime sources compile once
  via the `gnss_rtk_core` OBJECT library and are packaged into both
  `gnss_lib_solvers` and `gnss_rtk_runtime`.
- Test fixtures live under `tests/fixtures/`; generated artifacts
  (`*.png`, `*.pos`) need an explicit negation rule in `.gitignore`.
- `third_party/` holds vendored, always-built dependencies;
  `external/` is gitignored and reserved for opt-in parity checkouts
  (see `third_party/README.md`).
