# FGO NHC/ZUPT FIX-rate audit

## Objective

Determine whether vehicle-motion constraints improve the float state presented
to ambiguity resolution without relaxing LAMBDA, adding a TDCP promotion rule,
or increasing wrong FIX.  NHC and ZUPT remain default-off until every staged
gate passes.

Tokyo run1 is the only development run.  Run2 stays sealed until a full-run1
activation passes, and run3 stays sealed until the unchanged run2 activation
passes.

## Research and code audit

- Kilic et al. add zero-velocity information to a GNSS/INS factor graph for
  wheeled robots: <https://arxiv.org/abs/2112.07176>.
- Liu et al. formulate NHC as a factor in ground-vehicle FGO and emphasize the
  no-side-slip/no-vertical-motion assumption:
  <https://doi.org/10.3390/mi13091400>.
- The local `tightly-coupled-gnss-imu-fgo` reference applies body-frame
  lateral/vertical velocity constraints and gates ZUPT with causal IMU-window
  statistics plus an optional previous-velocity check.

The current C++ NHC residual and GTSAM Jacobians use the correct body-to-ENU
rotation convention.  The ZUPT statistics do not exactly match their stated
reference, however: the reference computes the RMS vector deviation from the
window mean, while C++ computes the standard deviation of per-sample vector
norms.  The latter can hide directional vibration at nearly constant norm.
This discrepancy must be corrected and covered by a deterministic test before
any activation result is accepted.

The other central risk is observability.  Quiet constant-speed travel can look
stationary to accelerometer/gyro dispersion, while a true stop whose velocity
estimate already drifted above the 0.5 m/s gate cannot receive ZUPT.  Therefore
removing the velocity gate or tuning only IMU thresholds is out of scope.

## Frozen development protocol

### Gate 0: correctness and non-interference

- correct the IMU-window statistic to the documented vector-deviation RMS;
- unit-test directional vibration, true stationary data, ZUPT speed rejection,
  NHC speed/turn rejection, and NHC residual/Jacobians;
- with both switches off, the shipping solution remains unchanged.

### Gate 1: fixed 500-epoch development slice

Use a 500-epoch Tokyo run1 replay beginning at source epoch 5000.  It was
selected before any NHC/ZUPT replay because the corresponding preserved
full-run window contains 60 correct FLOAT opportunities and zero wrong FIX
epochs, unlike the nearly saturated opening 500 epochs.  `--start-epoch`
starts a fresh solve at epoch 5000 rather than retaining the full run's solver
history, so its baseline is frozen independently before any active constraint
replay: 333 FIX, 167 FLOAT, zero NONE/non-finite, and 0.024057 m fixed
horizontal RMS (21.1719 s solver wall time).

Compare exactly four configurations using the shipping preset:

1. baseline;
2. NHC only;
3. ZUPT only; and
4. NHC + ZUPT.

A configuration advances only with:

- zero additional wrong FIX epochs;
- zero lost correct FIX epochs;
- at least three additional correct FIX epochs;
- no additional NONE/non-finite epoch;
- fixed horizontal RMS and P95 regression no greater than 5%; and
- solver wall-time overhead no greater than 10%.

No sigma or detector threshold is tuned after this four-way replay.  If none
passes, stop activation and retain only corrected tests/telemetry.

### Gate 2: full run1

Run only Gate-1 survivors over all 11,905 epochs.  Require zero increase in
wrong-FIX distance, no decrease in correct-FIX distance, no solver/matched-
distance regression, at least +0.2 percentage points correct-FIX distance,
and no more than 5% fixed RMS/P95 regression.

### Gate 3: sealed holdouts

Freeze the configuration after run1.  Run2 and then run3 once each.  Each must
independently have zero wrong-FIX-distance increase, no correct-FIX-distance
decrease, no solver/matched-distance regression, and at most 5% fixed RMS/P95
regression.  Any failure ends activation without tuning against that run.

## Result

Gate 0 passed.  The focused deterministic tests cover directional vibration,
ZUPT application, NHC application, and turn rejection.  A 500-epoch
monitor-off/monitor-on replay had zero differences in every pre-existing CSV
field; only the ten new motion-diagnostic fields changed.

The frozen Gate-1 replay produced:

| configuration | correct FIX | wrong FIX | added correct | added wrong | fixed horizontal RMS | P95 | wall time | result |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| baseline | 333 | 0 | -- | -- | 0.024085 m | 0.0340 m | 21.1719 s | reference |
| NHC | 333 | 0 | 0 | 0 | 0.023850 m | 0.0334 m | 18.4081 s | fail: no FIX gain |
| ZUPT | 333 | 0 | 0 | 0 | 0.023802 m | 0.0330 m | 20.8266 s | fail: no FIX gain |
| NHC + ZUPT | 333 | 2 | 0 | 2 | 0.961176 m | 0.0330 m | 24.6932 s | fail: wrong FIX, RMS, runtime, and no correct-FIX gain |

The combined configuration changed two baseline FLOAT epochs into wrong FIX
at TOW 188564.0 and 188564.2; their 3-D errors were 22.399 m and 22.361 m.
The shadow observed 87 ZUPT and 316 NHC candidates in the baseline.  NHC-only
applied 316 factors; ZUPT-only applied 108 as its velocity feedback changed
later candidate decisions.  Neither changed the FIX classification.

No configuration passes Gate 1, so the audit stops without a full run1 or any
inspection of sealed run2/run3.  NHC and ZUPT remain default-off.  The accepted
deliverable is limited to the corrected vector-deviation statistic,
authority-neutral telemetry, deterministic tests, and the reproducible
offline A/B scorer.  The machine-readable result is generated with:

```powershell
python scripts/analysis/analyze_fgo_motion_constraint_ab.py `
  --baseline build-ffrt-msvc/validation/tokyo1_motion_e5000_n500_baseline.csv `
  --baseline-runtime-s 21.1719 `
  --variant nhc=build-ffrt-msvc/validation/tokyo1_motion_e5000_n500_nhc.csv `
  --runtime-s nhc=18.4081 `
  --variant zupt=build-ffrt-msvc/validation/tokyo1_motion_e5000_n500_zupt.csv `
  --runtime-s zupt=20.8266 `
  --variant both=build-ffrt-msvc/validation/tokyo1_motion_e5000_n500_both.csv `
  --runtime-s both=24.6932 `
  --json build-ffrt-msvc/validation/tokyo1_motion_e5000_n500_ab.json
```
