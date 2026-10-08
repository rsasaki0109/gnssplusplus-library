# Smartphone barometer-aided height (Nantes feasibility study)

Status: **feasibility study with a weak reference; outcome below (vertical aid works, horizontal aid not shown, the pre-registered Go gate G3 failed by 0.07 m on the sealed holdout).**  Default OFF; with the
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
`/media/sasaki/aiueo2/datasets/nantes_mimir/MANIFEST_extracted.json`; the
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
logs L1/E1, L5/E5a, BDS B1I/B2a and GLONASS G1; the default adapter
(`--signal-set legacy-l1-e1`, the configuration of every result above) uses
only GPS L1 C/A and Galileo E1 (about 8.8 GPS + 6.2 Galileo rows per epoch),
everything else is kept in `observations.csv` with an exclusion reason.  The
multi-signal option is evaluated in the post-hoc section at the end.  Android reports the L1 carrier as
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

## Sealed holdout (S4 A1/A2, opened once after the freeze commit)

Run exactly as frozen (final binary, no `--baro-*` flags, `--max-residual-rms
50`, lag 0 s).  S4 reference: the Awinda altitude is essentially a constant
52.6-53.1 m (flat street), start/end closure 0.23 / 0.36 m horizontal; the
barometer cannot be checked against it (range 5.4 m in A1 is weather drift, 52
m in A2 pocket spikes).  So S4 vertical error means "deviation from a constant
height" and cannot reveal a real slope of the street.  A1 = left hand,
A2 = left trouser pocket; the phone clock was only 0.28-0.29 s from UTC.

Pooled (1740 matched epochs, availability 100 % for OFF and ON):

| | H RMSE | H P50 | H P95 | V demeaned RMSE | V demeaned P50 | V demeaned P95 | V common-offset RMSE | max H / V step (m/s) |
|---|---|---|---|---|---|---|---|---|
| OFF | 23.4 | 12.1 | 50.4 | 27.3 | 15.3 | 58.4 | 27.3 | 155 / 108 |
| ON | 22.6 | 12.8 | 46.7 | 8.4 | 3.4 | 21.1 | 7.9 | 120 / 51 |
| Fix.csv | 3.1 | 2.0 | 5.5 | 1.8 | 0.9 | 2.3 | 17.1 | 3 / 28 |

Per run (OFF → ON): A1 (hand) H RMSE 23.3 → 21.7, P50 10.6 → 11.4, P95 54.0 →
47.7; V demeaned RMSE 26.0 → 3.7.  A2 (pocket) H RMSE 23.5 → 23.7, P50 14.7 →
15.3, P95 47.5 → 46.2; V demeaned RMSE 28.7 → 11.9 (closure vertical 29.2 →
21.8 m, i.e. the absolute level is still wrong by ≈ 20 m at the end of the
pocket run).  Reference-free closure, hand run: vertical −7.4 → +1.3 m.

Gate evaluation (frozen, pooled):

| Gate | Result | Verdict |
|---|---|---|
| G1 relative vertical: RMSE ≤ 0.5×, P95 ≤ 0.5× | 0.31×, 0.36× | pass |
| G2 vertical with common offset: RMSE ≤ 0.75× | 0.29× | pass |
| G3 horizontal no-harm: P50 ≤ 1.05×, P95 ≤ 1.05× | P50 **1.055×** (12.75 vs 12.68 limit), P95 0.93× | **fail (by 0.07 m)** |
| G4 horizontal benefit (P50 not worse and RMSE or P95 ≤ 0.95×) | P95 0.93×, RMSE 0.97×, P50 worse | fail |
| G5 availability ON ≥ OFF | 100 % = 100 % | pass |
| G6 jumps not worse | H 155 → 120, V 108 → 51 m/s | pass |

**As pre-registered the outcome is No-Go for the combined "vertical aid" claim
because G3 failed, and No-Go for "horizontal aid".**  No parameter was changed
after reading the holdout.  For context only (not used for any decision): a
30 s block bootstrap of the paired ON-OFF differences gives, on S4, ΔH P50
+0.66 m [-0.34, +1.49], ΔH P95 -3.72 m [-6.74, +0.90], ΔH RMSE -0.77 m [-1.76,
+0.19], Δ demeaned V RMSE -18.9 m [-23.1, -14.5]; on S3 development ΔH P50
-0.10 m [-0.44, +0.15], ΔH P95 +0.83 m [-2.14, +3.32], Δ V RMSE -16.3 m
[-21.3, -12.3].  The vertical effect is significant in both sessions, the
horizontal effect is indistinguishable from zero in both; the G3 miss is a
statistically insignificant 0.67 m on a metric with ±1-2 m reference
uncertainty.

Filter behaviour on all five scored runs: constraint applied on 91-100 % of
epochs after a 2-25 epoch start-up, 1-4 baro rejections per run, no GNSS
height rejections except one, no re-base events.

Other findings: (1) the chipset `Fix.csv` solution is far better horizontally
than this code-only L1/E1 SPP on S4 (median 2.0 m versus 12 m) and in vertical
dispersion (0.9 m versus 3.4 m demeaned median), so the barometer does not
close the gap to a modern smartphone fix; (2) the vertical gain is a relative
one: the ON height tracks level changes (mezzanine ≈ 10 m) to 1-2 m but starts
from the first-epoch SPP height and converges over minutes; (3) the unguarded
OFF baseline can diverge on a single epoch (A2, 679 km) and the constrained
re-solve recovered it, which is a solver QC weakness rather than a barometer
result and is why the common residual gate is applied to both arms.

## Limits / unverified

Weak reference (piecewise-linear altitude, unknown datum, ≈ 1-2 m); one phone
model; one subject on one day; no weather record, so
the pressure bias model (random walk + re-base) is validated only through the
reference profile; the absolute vertical level is unverified; hand/pocket
differences are not separated statistically (3 scored runs).  The S4 reference altitude is
constant, so S4 vertical error is a flat-street assumption.  A three-satellite
+ barometer solution (availability gain on the ≈ 25-30 epochs per pocket run with ≤ 3
usable satellites) is **not implemented**.

## Post-hoc: multi-signal adapter and the existing R5 smartphone profile

**Status: post-hoc, non-holdout.**  S4 was the sealed holdout of the study
above and has already been opened; every S4 number in this section is
post-hoc and was used for **no** gate decision.  Nothing was tuned on S3 or S4:
the barometer configuration is the one frozen above, the SPP binary is the
frozen `gnss_spp_baro_final` (sources of `src/`/`include/` unchanged), the
common QC is `--max-residual-rms 50`, the reference lag is 0 s.  This is a
wiring task: the existing smartphone pipeline was connected to the Nantes data
as is.  Outputs and a manifest (BRDC URL + sha256, binary and input hashes)
are in `rtklib_v2_ws_output/mimir_multisignal_20261009/`.

### Why the baseline was weak

The #564 adapter used only L1/E1 code and none of the R5 features.  Mimir
`Raw.csv` has no `CodeType`/`SignalType`, so the signal has to be inferred.

### Signals in the Pixel 7 data (rows over S3 A2/A4/A6 + S4 A1/A2, 178 842 rows)

| Constellation (`ConstellationType`) | Carrier | Signal | Rows used (state usable) |
|---|---|---|---|
| GPS (1) | 1575.42 / 1176.45 MHz | L1 C/A / L5 | 29 573 / 15 882 |
| Galileo (6) | 1575.42 / 1176.45 MHz | E1 / E5a | 19 584 / 17 331 |
| GLONASS (3) | 1602 MHz + k*562.5 kHz, k = -7..+6 | G1 (FDMA) | 10 003 |
| BeiDou (5) | 1561.098 / 1176.45 MHz | B1I / B2a | 29 228 / 20 181 |
| NavIC (7) | 1176.45 MHz (S4 A1 only) | L5 | 0 (919 rows rejected, `unsupported_constellation`) |
| QZSS (4), SBAS (2) | absent from the data | - | table support for QZSS L1/L5 only, untested on data |

Total used 141 782 of 178 842 rows; 36 139 are `state_not_usable` (no code
lock / TOW or TOD unknown / ms-ambiguous), 2 `uncertain_received_sv_time`.
No `unsupported_frequency`, `no_navigation`, `invalid_svid` or
`glonass_fcn_conflict` rows occurred in the real data (they are exercised by
unit tests).

### Mapping design

* `gnss_smartphone_mimir_signals.py`: signal table `SIGNALS`, `classify_multi`
  (ConstellationType + carrier within 1 kHz of nominal; GLONASS channel from
  `1602 MHz + k*562.5 kHz`, and it must equal the channel in the broadcast
  file), RINEX 3.04 codes (`C1C`, `C5Q` for GPS L5/E5a, `C2I` BDS B1I, `C5X`
  BDS B2a, `C1C` GLONASS G1), per-constellation state rules and time bases
  (BeiDou BDT = GPST - 14 s, GLONASS time of day = GPST - 18 s + 3 h),
  `MultiSignalRinexWriter` (several signals per satellite line, GLONASS
  `SLOT / FRQ #` header), and `NavigationIndex` (row-level navigation coverage).
* The tracking attribute (`Q` / `X`) is **inferred**: Mimir does not log it.
  The native reader keys only on the band digit.
* The R5 `StreamingRinexWriter` supports only GPS L1 and Galileo E1 and fails
  closed on anything else, and GSDC output must stay unchanged, so it was
  **not** modified; the multi-signal writer reuses its helpers and its
  `HatchSmoother` unchanged.
* Every source row gets exactly one terminal disposition (`used`,
  `unsupported_constellation`, `unsupported_frequency`, `signal_not_enabled`,
  `invalid_svid`, `glonass_fcn_conflict`, `state_not_usable`,
  `uncertain_received_sv_time`, `implausible_travel_time`, `no_range_fields`,
  `no_navigation`, `excluded_by_max_epochs`); the summary lists the rejected
  rows with constellation@frequency, and rows are preserved in
  `observations.csv`.
* Navigation: the same IGS merged broadcast file as above (it already contains
  G, R, E, C, J), URL
  `ftp://igs.gnsswhu.cn/pub/gps/data/daily/2024/074/24p/BRDC00IGS_R_20240740000_01D_MN.rnx.gz`,
  sha256 (gz) `cf28abe19a018a9fcb00c87cdffced1afdd67b3395a2890c465da244d0b6c466`,
  sha256 `903986e0f1fdf8558d4af8b4ef5ccfe95e43623d33155f40b1a87ae33db8bd42`.
  A measurement without a broadcast record inside the age limit (4 h; GLONASS
  30 min) is rejected as `no_navigation`.
* **OFF contract:** the default is unchanged (`--signal-set legacy-l1-e1`);
  `observations.csv`, `rover.obs`, `baro.csv` are byte-identical to the #564
  adapter output (md5 checked on S3 A2 and A4, `summary.json` differs only in
  paths and the source-terms argument).  The GSDC/R5 adapter is not touched.
  The multi-signal set is opt-in: `--signal-set multi` (optionally
  `--enable-signals`, `--hatch-window-s`).
* Tests: `tests/test_smartphone_mimir_multisignal.py` (15) in addition to the
  16 existing Mimir tests.

### The existing best smartphone profile

`smartphone_raw_gnss.md` / `configs/benchmarks/smartphone_r5_gsdc2023.json`:
the **frozen** profile (4.93 m H median / 16.35 m P95 on GSDC development) is
single-frequency GPS L1 standalone with no solver flags.  The later
development-only promotions were GPS L1 + Galileo E1 (3.41 / 8.43 m), then
Hatch code smoothing of Galileo E1 C1C with a 30 s window (3.22 / 7.96 m), and
a truth-free Kalman/RTS smoother (2.75 / 6.17 m).  GLONASS G1 (phase 6) and GPS
L5 were No-Go in R5.  The SPP-stage best profile is therefore
**GPS L1 + Galileo E1 + R5 `HatchSmoother` window 30 s on E1 C1C, default SPP
flags**; it is applied unchanged (`--hatch-window-s 30`).  The Kalman/RTS
smoother is a separate GSDC-specific stage (needs `device_gnss.csv` epoch keys
and the GSDC profile hashes) and was **not** applied.

### Results (baro OFF / ON with the frozen #564 configuration)

Pooled over the runs, metres, same scorer and lag as above.  "V dem." is the
per-run-median-removed vertical error.  S3 = development runs A2/A4/A6; S4 =
post-hoc A1/A2 (not a holdout).  Epoch sets differ slightly between arms
(availability), so small differences are not paired comparisons.

S3 (A2/A4/A6):

| Arm | baro | H RMSE | H P50 | H P95 | V dem. RMSE | V dem. P95 | avail. |
|---|---|---|---|---|---|---|---|
| (i) #564 baseline, L1/E1 | OFF | 18.9 | 12.6 | 28.7 | 20.3 | 38.6 | 95.7 % |
| | ON | 16.9 | 12.5 | 29.6 | 3.9 | 8.9 | 95.7 % |
| (ii) multi-signal, default SPP | OFF | 25.1 | 12.7 | 28.8 | 23.0 | 32.5 | 97.1 % |
| | ON | 17.5 | 12.7 | 28.2 | 3.4 | 7.4 | 97.1 % |
| (iii) multi-signal + R5 profile (Hatch30 on E1) | OFF | 27.2 | 12.9 | 35.0 | 28.3 | 41.8 | 86.0 % |
| | ON | 19.3 | 12.9 | 34.8 | 4.0 | 6.6 | 86.0 % |
| diagnostic (i-b): L1/E1 + Hatch30 | OFF | 19.7 | 12.2 | 31.5 | 23.0 | 40.2 | 81.7 % |
| | ON | 16.9 | 12.0 | 31.1 | 5.4 | 4.6 | 81.7 % |
| exploratory (iv-x): multi + `--ionosphere-free` | OFF | 32.7 | 16.7 | 45.3 | 35.6 | 51.1 | 97.1 % |
| | ON | 23.8 | 16.5 | 44.5 | 7.4 | 15.9 | 97.1 % |
| Android `Fix.csv` | - | 10.0 | 7.9 | 18.2 | 5.9 | 13.2 | n/a |

S4 (A1/A2), **post-hoc, non-holdout**:

| Arm | baro | H RMSE | H P50 | H P95 | V dem. RMSE | V dem. P95 | avail. |
|---|---|---|---|---|---|---|---|
| (i) #564 baseline, L1/E1 | OFF | 23.4 | 12.1 | 50.4 | 27.3 | 58.4 | 100 % |
| | ON | 22.6 | 12.7 | 46.7 | 8.4 | 21.1 | 100 % |
| (ii) multi-signal, default SPP | OFF | 23.5 | 11.2 | 53.2 | 23.8 | 50.6 | 100 % |
| | ON | 22.5 | 12.1 | 48.7 | 8.9 | 22.7 | 100 % |
| (iii) multi-signal + R5 profile | OFF | 27.9 | 14.5 | 56.5 | 25.8 | 53.1 | 96.6 % |
| | ON | 26.8 | 15.1 | 55.0 | 6.7 | 17.4 | 96.6 % |
| diagnostic (i-b): L1/E1 + Hatch30 | OFF | 41.8 | 15.6 | 74.4 | 37.9 | 66.8 | 94.3 % |
| | ON | 35.6 | 16.1 | 67.5 | 14.2 | 17.6 | 94.3 % |
| exploratory (iv-x): multi + `--ionosphere-free` | OFF | 38.3 | 23.3 | 76.7 | 38.8 | 76.0 | 100 % |
| | ON | 35.8 | 23.5 | 71.6 | 10.1 | 19.5 | 100 % |
| Android `Fix.csv` | - | 3.1 | 2.0 | 5.5 | 1.8 | 2.3 | n/a |

Arm (i) reproduces the numbers recorded above for both sessions.

### Outcome (honest reading)

* **Multi-signal does not close the gap.**  Satellites per epoch go from 11.9
  to 21.4 (S3) and 8.5 to 15.8 (S4) and PDOP (A2) from 1.7 to 1.1, but the
  horizontal median stays at 11-13 m (S3 12.6 -> 12.7, S4 12.1 -> 11.2 OFF,
  12.7 -> 12.1 ON) against 2.0-7.9 m for the chipset `Fix.csv`.  Availability
  improves by 1.4 points on S3.  The OFF H RMSE is worse on S3 (25.1 vs 18.9)
  because of a few gross-error epochs that the barometer re-solve removes (ON
  17.5 vs 16.9); this is the known solver QC weakness, not a signal effect.
  The ON vertical error is 3.9 -> 3.4 m (S3) and 8.4 -> 8.9 m (S4), i.e. no
  consistent change.
* **Single-constellation checks (S3, OFF, wiring diagnostic):** GPS-only 14.1 m,
  Galileo-only 13.0 m, GLONASS-only 17.7 m (42 % availability), BeiDou-only
  17.9 m H median.  All constellations independently land at 13-18 m, so there
  is no sign of a time-base or channel error in the BeiDou/GLONASS wiring (a
  wrong time base would be kilometres), and the error floor is shared
  (multipath / phone-grade pseudoranges / weights).  Mean residual RMS rises
  from 5.1 to 7.0 m (S3) with the extra signals.
* **L5/E5a/B2a give nothing with default SPP**: the native SPP uses primary
  signals only (L1/E1/G1/B1), so these rows are written but not consumed.
  The only existing way to use them, `--ionosphere-free`, is worse (H P50
  16.7 / 23.3 m, residual RMS 13.8 / 12.0 m); no L5 inter-signal bias or
  weighting exists in the code, so this is not evidence against L5 itself.
* **The R5 profile (Hatch30 on E1) hurts on this data.**  It reduces
  availability (82-86 % S3; 94-97 % S4) and increases error.  Cause
  (verified, not tuned around): in these Pixel 7 logs `AccumulatedDeltaRangeMeters`
  advances by `c * DriftNanosPerSecond` (about 170 m/s; the median of
  (dADR - dPR)/dt over 7 900 Galileo/GPS L1 pairs equals 1.00 x c*drift in every run, 169-176 m/s)
  more than the clock-corrected pseudorange, whereas the R5 smoother uses the ADR
  delta directly.  The smoothed code drifts away from the raw code by up to
  10^6 m on affected arcs and the `--max-residual-rms 50` gate rejects the
  epochs.  A drift-compensated smoother would be a model change to the R5
  smoother and was **not** implemented or evaluated here (no tuning).
* The chipset `Fix.csv` (2.0 m on S4, 7.9 m on S3) is probably a map/filter
  aided fused fix, not raw SPP; the gap is therefore not explained by the
  adapter alone.  That is unverified.

### Limits / unverified (this section)

Post-hoc and non-holdout (S4); three plus two runs, one phone, one day; the
weak reference of the sections above still applies; L5/E5a/B2a code attributes
and the GLONASS UTC leap-second constant (18 s, valid for 2024) are inferred
or assumed; QZSS and BeiDou B1C are in the table but absent from the data;
inter-signal biases (`FullInterSignalBiasNanos`, `SatelliteInterSignalBiasNanos`)
are not applied (as in R5); the GSDC-specific Kalman/RTS smoother and IMU
process-noise stages were not applied; the C++ solver was not changed so the
C++ suite was not re-run.

