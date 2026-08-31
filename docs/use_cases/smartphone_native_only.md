# Native-only smartphone inference boundary

This is an opt-in development path for raw Android smartphone observations.
The current native best remains 3.952 m public / 4.276 m private; a native
0.782-class result has not been achieved.

## Contract

Run `apps/commands/benchmarks/gnss_smartphone_native_only.py` before the native
executable. The approved input set is:

- raw `device_gnss.csv` with Android clock, pseudorange-rate, ADR, frequency,
  signal and constellation fields;
- raw `device_imu.csv` containing both `UncalAccel` and `UncalGyro` streams;
- broadcast RINEX navigation; and
- no MATLAB `.mat` file of any kind; `.m` files are specification-only and are
  never runtime inputs.

Any `.mat` path or file, including `result_gnss.mat`, `result_gnss_imu.mat`,
`phone_data.mat`, and `gt.mat`, is rejected before opening. Ground truth,
sample/submission coordinates, and imported v5 output are also rejected as
inference inputs. Both native smartphone entry points (`gnss_pos_vel_pdc` and
`gnss_smartphone_fgo`) apply this path guard before their file checks/loaders.
A finite device-provided WLS ECEF field may be present in a
GNSSLogger export for adapter compatibility, but the dedicated raw PDC
executable ignores it and seeds from native SPP. The manifest hashes only the
explicitly approved raw CSV/nav files, lists the files read, and sets every
forbidden-read flag to false. All manifest and solver outputs are atomically
published.

## Reproduction

```bash
cd gnssplusplus-library
python3 apps/commands/benchmarks/gnss_smartphone_native_only.py \
  --device-gnss /path/to/device_gnss.csv \
  --device-imu /path/to/device_imu.csv \
  --broadcast-nav /path/to/brdc.nav \
  --manifest /tmp/native-only-input.json

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target gnss_pos_vel_pdc -j2
LD_LIBRARY_PATH=/home/sasaki/.local/lib:${LD_LIBRARY_PATH:-} \
  build/apps/gnss_pos_vel_pdc \
  --android-raw /path/to/device_gnss.csv \
  --nav /path/to/brdc.nav \
  --device-model pixel7pro \
  --trip-id route/phone \
  --keyed-out-csv /tmp/native-keyed.csv \
  --out-csv /tmp/native-per-epoch.csv \
  --factor-debug-csv /tmp/native-factor.csv \
  --summary-json /tmp/native-summary.json

# Optional orchestration: validate raw/nav first and atomically seal all
# native artifacts plus their input/binary hashes.
python3 apps/commands/benchmarks/gnss_smartphone_native_gnss_pdc.py \
  --android-raw /path/to/device_gnss.csv \
  --nav /path/to/brdc.nav --trip-id route/phone \
  --output-dir /tmp/native-gnss-pdc-run

# Development-only score recovery after the truth-free run is sealed.  The
# recovery authorization is required; its 1000 ms one-to-one time window is
# fixed and unmatched epochs are reported, never interpolated or modified.
python3 apps/gnss.py smartphone-native-gnss-pdc-evaluate \
  --candidate /tmp/native-gnss-pdc-run/keyed.csv \
  --run-manifest /tmp/native-gnss-pdc-run/run_manifest.json \
  --ground-truth /path/to/ground_truth.csv \
  --recovery-authorization docs/use_cases/records/smartphone_r5_gsdc2023_native_gnss_pdc_train_score_recovery_v2.json \
  --phone route/phone --output-json /tmp/native-gnss-pdc-score.json
```

The first converter component is the raw Android clock and P/L/D mapping in
`include/libgnss++/io/android_raw_gnss.hpp` and
`src/io/android_raw_gnss.cpp`. It follows the pinned taroz
`functions/gnsslog2obs.m` equations for receiver time, nearest-week
pseudorange, ADR/wavelength and negative pseudorange-rate/wavelength. The
entry point calls the existing native Eigen graph with its frozen P + ordinary
TDCP + position/clock motion defaults; it does not consume IMU samples yet.
That missing IMU graph, upstream ENU/GTSAM state/backend, base compensation,
full Doppler state coupling, and the multi-pass result handoff remain explicit gaps in the
[dedicated GNSS/PDC gap matrix](records/smartphone_r5_gsdc2023_native_gnss_pdc_gap_matrix_v1.json).

## Tests and evidence

```bash
python3 tests/test_smartphone_native_only.py
LD_LIBRARY_PATH=/home/sasaki/.local/lib:${LD_LIBRARY_PATH:-} \
  build/tests/run_tests --gtest_filter=AndroidRawGnssTest.*
LD_LIBRARY_PATH=/home/sasaki/.local/lib:${LD_LIBRARY_PATH:-} \
  build/tests/run_tests --gtest_filter=UpstreamObservablePreprocessingTest.*
```

The contract regression covers raw manifest hashing, poisoned `.mat` and
forbidden-result paths, coordinate/submission path rejection, and enriched
timing rejection. The C++ regression covers the integer-clock equation, ADR
sign, Doppler sign, GPS/Galileo/GLONASS/BeiDou signal selection, unsupported
signals and duplicate rows. The 30-epoch development smoke is truth-free and
MAT-free; its
sealed summary reports 420 pseudorange factors, 309 ordinary TDCP factors, 29
motion factors, convergence in 7 iterations, and finite keyed coordinates.

The pinned upstream `.m` source is MIT-licensed at commit
`29923f9f370f09ebc00f96d8cca375007a18e7d5` and is used only as a written
specification. No precomputed MAT artifact is accepted, read, or vendored by
the native pipeline.

The cleanup boundary and the explicit classification of all retained
precomputed-MAT records as historical rejected experiments are sealed in
`records/smartphone_r5_gsdc2023_native_only_cleanup_v1.json` and its companion
manifest. Those records remain available for audit history only; no current
command, test registration, or native binary depends on them.

The first full raw development route (`2023-05-24-20-26-us-ca-sjc-ge2/pixel7pro`)
sealed 1,384 native epochs with 16,025 pseudorange, 16,025 Doppler, and 13,806
ordinary-TDCP factors. Its fixed timestamp recovery score matched 1,383 of
1,384 truth rows (all 999 ms apart): WGS84/Vincenty P50/P95 was
6.513/180.497 m and the local phone score was 93.505 m. This is a plumbing
and diagnostic result, not an improvement over the native FGO v5 server score
(3.952/4.276 m); the large residual tail confirms that upstream preprocessing,
state/factor, and base-compensation gaps remain. See the sealed recovery
authorization and report records for the failed exact-key attempt and the
no-tuning/no-MAT policy.

The follow-up raw-stage `exobs.m` status/multipath mask port is sealed as a
separate opt-in experiment. It retained finite raw-only execution (1,384
epochs; 15,704 P, 15,704 D, and 13,464 TDCP factors), but its one authorized
development score was 93.620 m versus 93.505 m for the prior bridge; horizontal
P95 and the combined diagnostic worsened. It is therefore a No-Go and is not
promoted. The freeze, score report, and source-level gap matrix are
`records/smartphone_r5_gsdc2023_native_gnss_pdc_preprocess_freeze_v1.json`,
`records/smartphone_r5_gsdc2023_native_gnss_pdc_preprocess_score_result_v1.json`,
and `records/smartphone_r5_gsdc2023_native_gnss_pdc_gap_matrix_v1.json`.

### Opt-in residual/SNR preprocessing candidates (2026-08-31)

The raw-observable portions of upstream `exobs_residuals.m` and
`obserrmodel.m` are available behind `--upstream-residual-snr`. Both
candidates apply deterministic P-D/L-D adjacent endpoint masks and the
published MATLAB midpoint-rank 85th-percentile SNR model (P=1, D=1/12,
L=1/400 with signal-type multipliers). `correct_pseudorange.m` base-station
compensation is explicitly not applied because base observations are
forbidden by the raw+navigation contract; all existing defaults remain
unchanged.

The first scored artifact used a per-epoch P/D center as a raw-only proxy. It
sealed 1,384 finite keyed epochs, converged in 253 iterations, had no
transition over 70 m/s, and scored 68.202 m (WGS84/Vincenty) versus 93.505 m
for the legacy raw bridge. It remains a historical, development-only proxy;
its P center is not an exact port of MATLAB `splitapply`.

The follow-up exactness correction implements the MATLAB column-group
semantics: P residual ISB is a global all-epoch center per system and
frequency after subtracting the Android receiver `clk(t)` estimate. The
truth-free artifact remained finite and converged (1,384 epochs, 33,510 P,
26,175 D, 24,397 TDCP factors; 38.101 s wall), and the one authorized train
score was 76.978 m WGS84/Vincenty. It improves the legacy bridge but is worse
than the earlier proxy on this route, so it is a sealed No-Go and is not
promoted. The exact v1 proxy and v2.1 correction records are:

- `records/smartphone_r5_gsdc2023_native_gnss_pdc_upstream_residual_snr_freeze_v1.json`
- `records/smartphone_r5_gsdc2023_native_gnss_pdc_upstream_residual_snr_score_result_v1.json`
- `records/smartphone_r5_gsdc2023_native_gnss_pdc_upstream_residual_snr_score_result_v1_manifest.json`
- `records/smartphone_r5_gsdc2023_native_gnss_pdc_upstream_residual_snr_freeze_v2_1.json`
- `records/smartphone_r5_gsdc2023_native_gnss_pdc_upstream_residual_snr_global_isb_v2_1_score_result.json`
- `records/smartphone_r5_gsdc2023_native_gnss_pdc_upstream_residual_snr_global_isb_v2_1_score_result_manifest.json`

To reproduce the current truth-free exactness candidate, add
`--upstream-residual-snr` to the orchestration command above. Development
scoring remains report-only and must use a sealed run manifest plus an
explicit train authorization; no validation, holdout, test truth, `.mat` data,
or external mutation is allowed. The D center remains a documented raw-only
proxy because the upstream `dclk` state is not available without prohibited
precomputed state.
