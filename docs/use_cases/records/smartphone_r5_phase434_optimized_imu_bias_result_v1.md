# Phase434 — baseline optimized IMU bias aggregates

Phase431 H and Phase432 LAX-T completed once, exit 0. Source/test/algorithm,
binary/raw pins and baseline candidate SHA identity passed the verifier.
Both graph stages converged; all frequency state/factor/prior counters are
zero. No truth reads or accuracy evaluations were performed.

| Route | Bias states | Max accel bias norm m/s² | Max gyro bias norm rad/s |
|---|---:|---:|---:|
| H | 3140 | 0.05074852446399027 | 0.0011605795219775865 |
| LAX-T | 1466 | 0.019204407198595647 | 0.0007523887656316793 |

H elapsed 317.43743136292323 s, manifest SHA
`569250aa48cbe66e251affb0547d8e2e4223ed580714e19415925474697e1a10`.
LAX-T elapsed 254.95835770398844 s, manifest SHA
`585e593690adc1e5b2c2bc56dbbe7b570dfc0ab37486f47e5f67c5b4539f6b66`.
These timings are not controlled speed measurements.

This confirms nonzero estimated biases despite all-zero Android reported bias
fields, with finite estimates at every epoch and unchanged baseline outputs.
LAX-T's larger known positioning error is not accompanied by larger maximum
bias norms than H. This does not establish causality or rule out incorrect
bias estimation. Maximum norms cannot establish drift, posterior uncertainty,
time correlation, or physical calibration. No bias covariance/prior change
is justified by this aggregate comparison alone; keep the baseline settings.

The raw-only goal remains unmet. Next prioritize distinguishing absolute GNSS
position support from relative IMU/TDCP constraints using existing factor
diagnostics; do not repeatedly score speculative bias parameter changes.
