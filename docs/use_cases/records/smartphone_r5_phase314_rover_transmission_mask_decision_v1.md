# Phase314: rover transmission-selection mask boundary

## Evidence and decision

Inspected `functions/exobs.m` in the existing gsdc2023 source cache,
`src/io/android_raw_gnss.cpp`, and `src/algorithms/fgo_problems.cpp`.
The Android loader already applies code-status, multipath, fixed 20 dB-Hz
SNR, and range masks to `has_pseudorange` before emitting observations.
Therefore the source satellite-state adapter should consume the parser's
observations before the builder's additional epoch jump/residual masks.
Do not apply `upstream_epoch_masks` as though it were the exobs pre-state mask.
This establishes an integration boundary, not full source parity: optional
missing status handling and parameterized source SNR thresholds still differ.

Added a synthetic raw CSV integration test: same satellite and epoch with
masked L1 and valid L5. The actual Android loader retains both rows but disables
L1 pseudorange availability; `selectNativeEpoch` then selects L5. Added a
selector-level test covering both codes masked and preservation of input rows.

## Verification

- `cmake --build build --target gnss_run_tests -j4`: exit 0.
- `AndroidRawGnssTest.*:BasePseudorangeCompensationTest.*`: 52 passed,
  including the new raw-parser-to-selector test.
- No production solver change, real route run, truth read, MAT payload,
  saved positioning input, submission, or accuracy claim in this phase.
- This is not the real-data byte-identity solver-refactor gate or full CTest.

## Remaining integration

The rover builder still performs its original per-row two-pass satellite
calculation. Connect receive-time-selected, per-satellite shared states there
with explicit missing-navigation handling; the current strict epoch adapter
throws for a single missing satellite and must not accidentally become a
whole-route failure. Keep the operational base-OFF recipe unchanged until
the paired rover/base path is implemented and a frozen evaluation is ready.
The 0.782-class and leaderboard objectives remain unverified.
