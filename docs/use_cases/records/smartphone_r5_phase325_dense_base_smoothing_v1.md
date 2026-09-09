# Phase325: preserve the base epoch grid through correction smoothing

Added default-off `Config::use_dense_epoch_smoothing`. It requires source
epoch states and typed stream keys; canonical Phase131 composition is rejected
until its separate smoothing path is adapted. Each admitted correction stream
is expanded onto the original strictly monotonic base epoch array, with NaN
for missing observations. The existing finite-value moving mean then operates
on this grid. Its NaN outputs remain in the published stream as interpolation
barriers, rather than being dropped and bridged across a long outage.

An exact finite sample immediately following a NaN is explicitly returned;
an interior query between NaN and finite remains unavailable. The guard only
changes this previously absent NaN-adjacent case, preserving the legacy finite
interpolation arithmetic. No extrapolation outside the base epoch grid is
allowed. A stream can now have finite smoothed corrections before its first
raw observation or after its last, where a window overlaps observed data;
this is smoothing on existing base epochs, not endpoint extrapolation.
`streamTimeDomain` now describes the grid extent for this mode, not a guarantee
of finite correction coverage within it.

Production IMU CLI flag `--native-dense-base-smoothing` is explicit, default
off, requires paired epoch states, and is recorded in both provenance outputs.
The historical compact mode and frozen Phase319/322 recipes remain unchanged.

Synthetic model test uses 401 base epochs with two separated observations and
the fixed 151-sample window. It checks a 45 m compact-vs-dense difference,
long-gap unavailability, exact recovery after NaN, both grid endpoints, no
extrapolation, and failed-build stream clearing. All 37 base tests passed after
building `gnss_run_tests`. No real-data inference, candidate or truth access,
accuracy evaluation, MAT payload, or saved-positioning input occurred here.
The IMU CLI also built successfully and all nine executable argument tests
passed, including dense/paired admission and rejection of dense without paired.
These focused tests are not full CTest or a real-data byte-identity gate.

This implements the identified grid-shape correction; it does not establish
full source numerical parity or improved smartphone positioning. Next freeze
a paired raw GNSS/IMU run changing only dense smoothing relative to Phase319,
then inspect correction support, graph admission and convergence before a
separately frozen one-shot evaluation. Do not tune the window against truth.
