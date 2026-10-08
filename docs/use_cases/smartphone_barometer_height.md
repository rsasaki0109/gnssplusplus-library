# Smartphone barometer-aided height (Nantes feasibility study)

Status: **feasibility study with a weak reference.**  Default OFF; with the
flag absent the native SPP output is bit-identical to the pre-change binary
(checked by `md5sum` on five real runs and by unit tests).  Nothing here is an
accuracy claim against a survey-grade reference.

## Question

Does fusing a phone's barometric pressure improve standalone smartphone
positioning in urban / light-indoor conditions, vertically and (through better
geometry) horizontally?

## Dataset and licence

"Multi-Sensor Dataset in outdoor and indoor environment from Android Smart
Devices and ULISS", Zenodo record 12566912, `Android_GNSS_Dataset_Nantes.zip`
(5.6 GB), CC-BY 4.0 (`LICENSE.txt` in the archive).  Source:
<https://zenodo.org/records/12566912>.  Data are **not** committed.  Extracted
files and their SHA-256 are recorded in
`/media/sasaki/aiueo2/datasets/nantes_mimir/extracted/manifest_*.json`; the
broadcast navigation file is the public IGS
`BRDC00IGS_R_20240740000_01D_MN.rnx` (sha256
`903986e0f1fdf8558d4af8b4ef5ccfe95e43623d33155f40b1a87ae33db8bd42`).

Date check: `notes.txt` says "2023.03.14" but the Mimir logs are named
`log_mimir_20240314…` and the RAW `FullBiasNanos`/`TimeNanos` give GPS week
2305 (14 March 2024, GPST TOW 380 867 s ≙ 09:47 UTC).  The dataset header
says surveys were made 12-19.03.2024.  The 2023 in `notes.txt` is a typo; the
2024-day-074 broadcast file was used.

### Data inventory (device GP7 = Pixel 7, Broadcom BCM4776, pressure sensor ICP20100)

| Scenario | Run | GP7 position | Raw+PSR | Awinda pos CSV | Use |
|---|---|---|---|---|---|
| S3 | A1 | left hand (swinging) | yes | no (README only) | dev, reference-free checks |
| S3 | A2 | left hand (texting) | yes | yes | dev, scored |
| S3 | A3 | left trouser pocket | yes | no | dev, reference-free checks |
| S3 | A4 | left trouser pocket | yes | yes | dev, scored |
| S3 | A5 | no Android device | no | no | unusable |
| S3 | A6 | left trouser pocket | yes | yes | dev, scored (different path) |
| S4 | A1 | left hand (texting) | yes | yes | sealed holdout |
| S4 | A2 | left trouser pocket | yes | yes | sealed holdout |

`Raw.csv` has **no header** (the header only exists in `log_mimir_*.txt`) and
35 fields; the adapter pins that 35-field contract.  No `CodeType`,
`SignalType`, `ArrivalTimeNanosSinceGpsEpoch` or WLS columns exist.  Pixel 7
logs L1/E1, L5/E5a, BDS B1I and GLONASS; only GPS L1 C/A and Galileo E1 are
used (about 8.8 GPS + 6.2 Galileo rows per epoch), everything else is kept in
`observations.csv` with an exclusion reason.  Android reports the L1 carrier as
1575 420 030 Hz (+30 Hz); the adapter accepts ±1 kHz and writes the nominal
frequency.

## Reference characterisation (Awinda body suit, `pos_awinda_60hz.csv`)

Accuracy is not documented, timestamps "will be available after
post-processing by ZL" (`README_AWINDA.txt`).  Evidence collected on S3 A2/A4/A6
only:

* **Not a measured trajectory at the metre level.** 97-99 % of the 60 Hz
  altitude second differences are below 1 mm: the altitude is a piecewise
  linear model (ground level, a 10.4 m ramp, mezzanine level, ramp back), not a
  measurement; the horizontal track has ≈ 0.9-1.1 m start/end closure and
  1.1-1.4 m/s median speed (max 1.9 m/s), which is plausible for walking.
* **Start/end closure** (same point by protocol): horizontal 0.87 / 1.05 /
  1.10 m, vertical 0.03 / 0.01 / 0.00 m (A2/A4/A6).  The closure is by
  construction and says nothing about interior accuracy.
* **Time base.** Aligning the phone barometer height to the reference altitude
  profile gives a best lag of 0.0 s (A2, correlation 0.9994, residual 0.17 m),
  0.0 s (A4) and +1.0 s (A6), so `Awinda_TOW` is consistent with GPST to about
  one second for the vertical profile.  Horizontal lag scans against noisy SPP
  are shallow (median error minimum within ±8 s) and were not used.  **The lag
  is frozen to 0 s for all runs.**
* **Vertical datum is not the GNSS ellipsoid.** Reference start altitude is
  52.8 m while the dataset's own approximate station coordinate is 59.5 m;
  the chipset `Fix.csv` altitude is within 0.5-9 m of the reference but the
  standalone SPP is 8-14 m above it.  Absolute vertical error against this
  reference is therefore **not interpretable**; we report vertical error with
  (a) the per-run median removed ("demeaned", relative height accuracy) and (b)
  a common offset taken from the OFF baseline of the same run so bias changes
  of the ON solution stay visible.
* **Baro-vs-reference scale** is 1.02-1.13 (pressure-height rises 11-12 m, the
  reference 10.4 m): either the reference ramp or the ISA scale is off by a
  few percent.  This bounds the vertical comparison to roughly 1 m.
* **Body vs phone.** The reference is a body-suit point; phones were in the
  hand or trouser pocket (0.3-0.8 m lever arm, not modelled).
* SPP horizontal errors here are ≈ 12 m (median), so a ≈ 1-2 m reference
  error does not change OFF-versus-ON conclusions, but cm-level statements are
  impossible.

Pressure quality (device GP7): 8.5 Hz, accuracy flag 3.  High-pass noise of the
standard-atmosphere height: hand-held runs 0.12-0.30 m raw / 0.02-0.06 m after
2 s averaging; pocket runs 2.3-2.5 m raw / 0.25-0.6 m after 2 s averaging.
Pressure range within a run is up to 5 hPa (≈ 40 m) because of pocket/door
transients; the filter gates these.

## Design

* `include/libgnss++/algorithms/barometer_height.hpp`, `src/algorithms/barometer_height.cpp`
  - `pressureToStandardAtmosphereHeightM()` (ICAO troposphere), inverse,
  - `BarometerSampleBuffer` (causal windowed mean; future samples never used),
  - `BarometerHeightFilter`: 2-state KF `x = [h, b]`, baro model
    `hb = h + b + v`, GNSS model `hg = h + v`, random walks on `h` and `b`,
    4σ innovation gates, re-base of `b` after `rebase_after_rejections`
    consecutive baro rejections (pressure step), Joseph-form covariance.
* `src/algorithms/spp.cpp` (`solvePositionBaroAided`, `solvePositionLS`):
  1. unconstrained solve = exactly the OFF path;
  2. if a fresh causal pressure height exists: initialise `[h, b]` from the
     first good-geometry GNSS height (`GDOP <= 6`, `>= 6` satellites), then
     per epoch predict, baro update, capture the prior `(h, σ_h)`;
  3. update the KF with the **unconstrained** GNSS height (sigma = max(floor,
     scale × SPP vertical sigma)) - after the prior was captured, so the
     constraint never contains the epoch's own GNSS height;
  4. re-solve with the prior as one extra weighted LS row on the ellipsoidal
     height (ECEF up vector; the same row is added to the covariance), only if
     the prior sigma is ≤ 8 m; if the re-solve fails the unconstrained
     solution is returned.
* `apps/native/gnss_spp.cpp`: `--baro-height --baro-csv … [--baro-*]
  --baro-telemetry-csv`.  Pressure samples are fed incrementally, only those
  stamped at or before the epoch.
* `apps/commands/benchmarks/gnss_smartphone_mimir_adapter.py`
  (`gnss smartphone-mimir-adapter`): fail-closed Raw/PSR adapter reusing the R5
  `StreamingRinexWriter` and `validate_galileo_navigation`; every source row
  has exactly one disposition; barometer timestamps are mapped phone-clock →
  GPST through the per-run median `arrival - utcTimeMillis` (the phone clock was
  2.64-2.69 s ahead of true UTC on all runs).
* `gnss_smartphone_baro_eval.py` / `gnss_smartphone_baro_nantes_runner.py`
  (`gnss smartphone-baro-eval`, `smartphone-baro-nantes`): scoring and OFF/ON
  orchestration.

## Development tuning (S3 A2/A4/A6 only)

Common QC for **both** OFF and ON: `--max-residual-rms 50` (the unguarded
baseline contains a diverged epoch with residual RMS 2.2e5 m and 679 km error
in A2); elevation mask 15°, no C/N0 mask, GPS L1 + Galileo E1, Klobuchar +
Saastamoinen (library defaults).

Grid (3 runs pooled, ≈ 60 variants; results flat except one cliff):

* `gnss_sigma_scale` 3 → 12 m demeaned vertical RMSE (the KF underweights GNSS
  and keeps the first-epoch error), 0.5-1.5 → 3.7-4.4 m.  Chosen 1.0 (default
  changed from 3.0 to 1.0 before the freeze).
* `bias_walk` 0.02-0.15 m/√s: demeaned RMSE 3.7-5.0 m, common-offset RMSE
  5.5-6.6 m (trade-off).  Chosen 0.05.
* `baro_sigma` 0.5-2 m, window 2-5 s, `height_walk` 0.5-3: no measurable effect.
* `init_max_gdop` 2 never initialises early (11 m); 3-6 identical.

## Frozen configuration and acceptance gates (committed before S4 is opened)

Frozen = the defaults of `BaroHeightConfig`: `baro_sigma_m=1.0`,
`bias_walk=0.05 m/√s`, `height_walk=1.0 m/√s`, `gnss_sigma_scale=1.0`,
`gnss_sigma_floor_m=5`, `init_max_gdop=6`, `init_min_satellites=6`,
`innovation_gate_sigma=4`, `rebase_after_rejections=10`,
`rebase_bias_sigma_m=3`, `prior_sigma_floor_m=0.5`, `max_prior_sigma_m=8`,
`sample_window_s=2`, `max_sample_age_s=3`.  Common QC `--max-residual-rms 50`.
Reference lag 0 s; level split (informational) 3 m.  S4 is scored with exactly
the same code, flags and lag; **no tuning after the holdout is read**.

Pooled over the S4 runs, OFF → ON, all of these must hold for **Go (vertical
aid)**:

| Gate | Condition |
|---|---|
| G1 relative vertical | demeaned V RMSE_ON ≤ 0.5 × OFF and V P95_ON ≤ 0.5 × OFF |
| G2 vertical with common offset | V RMSE_ON ≤ 0.75 × OFF |
| G3 horizontal no-harm | H P50_ON ≤ 1.05 × OFF and H P95_ON ≤ 1.05 × OFF |
| G5 availability | ON ≥ OFF |
| G6 jumps | max 1-epoch horizontal and vertical step ON ≤ OFF |

**Go (horizontal aid)** additionally requires G4: H RMSE_ON or H P95_ON ≤
0.95 × OFF *and* H P50 not worse; an H-RMSE gain that comes only from fewer
gross-error epochs is reported as "outlier suppression", not geometry.

### Freeze record

* Code: commit `afcd2bb2` (`feat(spp): default-OFF barometer-aided height ...`).
  The configuration above is exactly the C++ default; a run without any
  `--baro-*` tuning flag is byte-identical to the `--baro-gnss-sigma-scale 1`
  development run.
* OFF is bit-identical to the pre-change binary (`c2eb06f7`): `md5sum` of the
  `.pos` output matches on S3 A1-A4/A6 with and without `--max-residual-rms 50`.
* Tests at freeze: C++ `run_tests` 1508 passed / 67 skipped / 0 failed
  (13 new: `Barometer*`, `SPPBarometer*`); Python 16 new tests in
  `tests/test_smartphone_mimir_adapter.py`; existing smartphone adapter /
  signoff / workflow tests pass.
* S4 (`dataset/S4/*/GP7`, `AWINDA/pos_awinda_60hz.csv`) is downloaded and
  scored **only after** the commit that contains this section.

## Development results (S3 A2/A4/A6, frozen configuration)

(pooled 1719 matched epochs; OFF / ON / Android `Fix.csv` chipset comparator)

| | H RMSE | H P50 | H P95 | V demeaned RMSE | V demeaned P95 | V common-offset RMSE | avail. |
|---|---|---|---|---|---|---|---|
| OFF | 18.9 | 12.6 | 28.7 | 20.3 | 38.6 | 20.3 | 95.7 % |
| ON | 16.9 | 12.5 | 29.6 | 3.9 | 8.9 | 6.0 | 95.7 % |
| Fix.csv | 10.0 | 7.9 | 18.2 | 5.9 | 13.2 | 16.5 | n/a |

Max one-epoch step OFF → ON: horizontal 378 → 76 m/s, vertical 315 → 26 m/s;
vertical steps > 10 m/s: 629 → 2.  Ground level segments: H RMSE 24.0 → 19.9 m,
V demeaned RMSE 27.2 → 5.7 m; upper (mezzanine) level: H RMSE 14.9 → 14.7 m,
V demeaned RMSE 14.5 → 2.2 m.

Per run (OFF → ON): A2 V 17.3 → 4.3, H 15.6 → 16.4; A4 V 17.2 → 4.8,
H 16.1 → 15.8; A6 V 24.9 → 2.6, H 23.5 → 18.1.

Reference-free start/end closure (first vs last 30 s mean, vertical): ON is not
clearly better in absolute vertical terms because the first 30 s are inside
the bias-convergence transient (A1 −7.0 → −10.2, A2 −11.4 → −10.8, A3 −9.7 →
−1.8, A4 +8.1 → −1.7, A6 +11.9 → +18.8 m).

Observations: the ON height is smooth and relative height changes (stairs,
mezzanine) are tracked to ≈ 1-2 m, but the absolute level inherits the
first-epoch SPP height error and converges only over several minutes (A2:
+15 m at start, ≈ 0 after 600 s).  Horizontal medians do not move; the RMSE
gain is outlier suppression.  The chipset `Fix.csv` is better horizontally
(median 7.9 m versus 12.5 m).

## Limits / unverified

Weak reference (piecewise-linear altitude, unknown datum, ≈ 1-2 m); one phone
model; one subject on one day; no weather record, so
the pressure bias model (random walk + re-base) is validated only through the
reference profile; the absolute vertical level is unverified; hand/pocket
differences are not separated statistically (3 scored runs).  A three-satellite
+ barometer solution (availability gain on the ≈ 25-30 epochs per pocket run with ≤ 3
usable satellites) is **not implemented**.
