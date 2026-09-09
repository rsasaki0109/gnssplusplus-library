# Phase427 — raw stop-window duration audit

Following the negative frequency-state result, checked whether the fixed
500-sample stop detector spans substantially different durations on H/LAX-T.
Native `upstream_stop_constraints.hpp` uses 250 previous/current-side and 249
future-side samples for full centered windows. Phase388 confirmed alignment
to gyro UTC timestamps for these explicit-fallback raw streams.

Read only MessageType and utcTimeMillis from Phase37 raw device_imu.csv,
selected UncalGyro, deduplicated/sorted UTC keys, and computed every full
window as `times[i+499]-times[i]`. No truth, saved position input or solver.

| Route | Full windows | Min ms | P50 ms | P95 ms | Max ms |
|---|---:|---:|---:|---:|---:|
| H | 166003 | 9404 | 9408 | 9409 | 9411 |
| LAX-T | 77259 | 9407 | 9409 | 9409 | 9417 |

P50/P95 use lower order statistics. This raw full-window audit excludes
truncated endpoint windows and does not recompute stop decisions or velocity
gates. The median durations differ by only 1 ms, not a factor of two despite
the different accelerometer rates. No basis here to alter stop-window length
or sensor interpolation. Do not score a changed stop window on this evidence.

Also inspected source `.m` pseudorange/Doppler factor setup and the negative
Phase356 main-P Cauchy result; neither justifies repeating that robust-kernel
candidate. The source's optional truth-height branch is outside the raw-only
goal and must not be ported into inference. Goal remains unmet; further work
must identify a distinct supported error mechanism.
