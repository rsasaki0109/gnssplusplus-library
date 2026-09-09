# Phase388 — raw IMU interpolation support

Ran `scripts/audit_native_imu_cadence.py` directly on the Phase37 H and
LAX-T raw `device_imu.csv` files. Hashes match the existing experiment pins:

- LAX-T: `2e39a3e9f294c64b8ecfd452d0960025d1013b97f2d7497e6e48a2a1997b38c5`
- H: `fa3f17d07570fdbb8030f307130f33ca3613e8125475c2a907c48f7db3455480`

Only timestamp columns are analyzed. No truth or trajectory input, coordinate
output, solver change, or new accuracy score. The diagnostic is not an
inference intermediate. It supports only the explicit UTC fallback domain.

| Raw timestamp aggregate | LAX-T | H |
| --- | ---: | ---: |
| Unique accelerometer rows | 90008 | 243457 |
| Unique gyro rows | 77758 | 166502 |
| Accelerometer median cadence, ms | 19 | 10 |
| Gyro median cadence, ms | 19 | 19 |
| Interpolated gyro timestamps | 75374 | 77352 |
| Exact timestamps | 2384 | 89150 |
| Interpolation span median / P95 / max, ms | 19 / 19 / 28 | 19 / 19 / 28 |
| Interpolation spans over 100 ms | 0 | 0 |
| Gyro timestamps outside accelerometer range | 0 | 0 |
| Duplicate UTC timestamps, either sensor | 0 | 0 |

Counts agree with native summary pairing counts. LAX-T's higher interpolation
fraction is accompanied by a lower accelerometer sampling rate, not a longer
maximum interpolation span. No evidence of long-gap bridging in these runs.
The native branch uses the actual bracketing timestamp fraction for linear
acceleration interpolation. This audit does not prove absolute sensor latency,
signal fidelity, or complete upstream algorithm parity.

Do not change interpolation or fit a timing offset based on these counts.
Next prioritize raw GNSS factor residual/weight support and main-stage versus
GNSS-first model differences for the LAX-T baseline; no saved stage position
may be reused as an inference input. Goal remains active and unachieved.
