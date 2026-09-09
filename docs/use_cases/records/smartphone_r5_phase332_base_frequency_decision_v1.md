# Phase332: common-satellite GPS L1/L5 residual decomposition

Added a reused QR fit routine and self-test to the stationary-base diagnostic.
Synthetic known geometry/clock and frequency-difference components recovered
within 1e-10; rank-deficient, insufficient-row and nonfinite-input cases were
rejected. These are algebra checks, not satellite propagation/model parity.

After the source/binary/input freeze, ran once using only raw base and nav.
All 3500 epoch fits had full rank, using 23564 same-satellite/same-time finite
L1/L5 rows above the fixed 15-degree cutoff. Both frequencies and their
difference use exactly the same design matrix and row selection.

Median apparent displacement norms were 2.8105363824492557 m for L1,
3.304143437582394 m for L5 and 2.2666075658540787 m for L5-minus-L1. The latter
had median absolute clock-like offset 8.068521075762765 m and post-fit RMS
0.7518818622955185 m. These aggregate diagnostics do not identify a surveyed
station position error, hardware delay or ionosphere error. Differing finite
samples within each frequency's smoothing window can also prevent exact
cancellation of common errors despite matched query-time rows.

Inspected production C7 mapping: `raw_p_seed.cpp::c7ClockComponentFor` maps GPS
L1 to component 0 and L5 to 4. `sourceClockComponentJacobian` in
`fgo_gtsam_internal.hpp` gives L1 C0 and L5 C0+C4. Thus a frequency-common
offset already has a model component; the 8 m offset is not evidence of a
missing inter-frequency clock parameter. State priors/couplings and the
direction-dependent remainder require separate consideration.

Next compare dispersive/electrically neutral combinations and explicit modeled
ionosphere contributions on common raw rows, with fixed formulas and synthetic
tests before access. Do not shift the base reference, add fitted offsets to
phone data, or tune correction scales using H truth. No phone, truth, candidate
trajectory, MAT payload, or saved-positioning input was read; no inference
feedback, new smartphone score, or submission occurred. Baseline is unchanged.
