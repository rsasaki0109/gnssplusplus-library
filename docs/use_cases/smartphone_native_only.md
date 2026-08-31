# Native-only smartphone inference boundary

This is an opt-in development path for raw Android smartphone observations. It
does not count the imported upstream `result_gnss*.mat` oracle as native
libgnss++ performance. The current native best remains 3.952 m public / 4.276
m private; a native 0.782-class result has not been achieved.

## Contract

Run `apps/commands/benchmarks/gnss_smartphone_native_only.py` before the native
executable. The approved input set is:

- raw `device_gnss.csv` with Android clock, pseudorange-rate, ADR, frequency,
  signal and constellation fields;
- raw `device_imu.csv` containing both `UncalAccel` and `UncalGyro` streams;
- broadcast RINEX navigation; and
- optionally, a raw `phone_data.mat` with raw observation and accelerometer /
  gyroscope containers instead of the GNSS CSV.

`result_gnss.mat`, `result_gnss_imu.mat`, ground truth, sample/submission
coordinates, and imported v5 output are rejected before opening. A finite
device-provided WLS ECEF field in a GNSSLogger export is permitted only as the
approximate receiver seed supported by `ObservationData`; it is not imported
result data, and the solver summary counts its use. The manifest hashes only
the explicitly approved files, lists the files read, and sets every forbidden-
read flag to false. All manifest and solver outputs are atomically published.

## Reproduction

```bash
cd gnssplusplus-library
python3 apps/commands/benchmarks/gnss_smartphone_native_only.py \
  --device-gnss /path/to/device_gnss.csv \
  --device-imu /path/to/device_imu.csv \
  --broadcast-nav /path/to/brdc.nav \
  --manifest /tmp/native-only-input.json

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target gnss_smartphone_fgo -j2
LD_LIBRARY_PATH=/home/sasaki/.local/lib:${LD_LIBRARY_PATH:-} \
  build/apps/gnss_smartphone_fgo \
  --device-gnss /path/to/device_gnss.csv \
  --device-imu /path/to/device_imu.csv \
  --nav /path/to/brdc.nav \
  --trip-id route/phone \
  --out /tmp/native.csv \
  --summary-json /tmp/native-summary.json
```

The first converter component is the raw Android clock and P/L/D mapping in
`include/libgnss++/io/android_raw_gnss.hpp` and
`src/io/android_raw_gnss.cpp`. It follows the pinned taroz
`functions/gnsslog2obs.m` equations for receiver time, nearest-week
pseudorange, ADR/wavelength and negative pseudorange-rate/wavelength. The
entry point calls the existing native Eigen graph with its frozen P + ordinary
TDCP + position/clock motion defaults; it does not consume IMU samples yet.
That missing IMU graph, upstream preprocessing/error masks, full Doppler state
coupling, and the multi-pass result handoff remain explicit gaps in the
[gap matrix](records/smartphone_r5_gsdc2023_native_only_gap_matrix_v1.json).

## Tests and evidence

```bash
python3 tests/test_smartphone_native_only.py
LD_LIBRARY_PATH=/home/sasaki/.local/lib:${LD_LIBRARY_PATH:-} \
  build/tests/run_tests --gtest_filter=AndroidRawGnssTest.*
```

The contract regression covers raw manifest hashing, a poisoned forbidden MAT
sibling, coordinate/submission path rejection, and a raw MAT sensor/container
fixture. The C++ regression covers the integer-clock equation, ADR sign,
Doppler sign, GPS + Galileo E1 selection, unsupported signals and duplicate
rows. The 30-epoch development smoke is truth-free and result-MAT-free; its
sealed summary reports 420 pseudorange factors, 309 ordinary TDCP factors, 29
motion factors, convergence in 7 iterations, and finite keyed coordinates.

The pinned upstream source is MIT-licensed at commit
`29923f9f370f09ebc00f96d8cca375007a18e7d5`; its precomputed MAT files remain
external development/oracle assets and are not vendored or accepted as native
inference inputs.
