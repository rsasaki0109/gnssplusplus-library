# Phase360 — raw stationary gyro initializer and synthetic controls

Implemented `stationary_gyro_initializer.hpp` as a batch-only, currently
unconnected helper. It uses the existing source-stop detector, complete
nonoverlapping 250-sample stationary blocks, a 0.1-second gap reset, and a
minimum of two blocks. It returns their vector mean and aggregate support
and scatter. Input samples are const. It throws for inadequate support or
invalid data; no initial-window fallback exists.

Five fresh standalone C++ gtests passed:

- Known constant bias recovered after an initially turning interval.
- Empty / insufficient stationary support and all-turning stream rejected.
- Gaps cannot combine short fragments into valid support.
- Nonfinite gyro and duplicate timestamps rejected.
- Fixed mounting rotation rotates the bias estimate consistently.

Tests were registered in `tests/CMakeLists.txt`; the full CTest suite was
not rebuilt or run. No native solver wiring or production default changed.
No raw accuracy run, truth read, MAT input, saved positioning input or
submission occurred. Synthetic recovery does not demonstrate real-world
bias accuracy or improved positioning.

Next integrate via an explicit default-off Phase171 batch-only selector,
changing only init_gyro_bias after mounting, leaving acceleration bias,
attitude construction, covariance settings and graph topology unchanged.
Report support and scatter without exporting raw series. Require CLI
negative tests and a single frozen comparison before promotion. Changing
this bias also changes preintegration linearization and first-bias prior
mean; it must not be represented as only an initial-Values change.

The repeatedly scored H operational baseline remains unchanged. The
0.782-class / leaderboard objective remains unachieved.
