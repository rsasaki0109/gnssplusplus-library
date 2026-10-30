# CLAS PPP-RTK with the PPC IMU (not adopted)

Status: **not adopted**. None of this is in the code. The experiment is kept
as [a patch](ppc_clas_imu_experiment.patch) on `2ea6707`.

On the six PPC runs, the wrong CLAS fixes start after a filter reset. The float
re-initializes from an SPP seed that is metres off, settles with a
converged-looking covariance, and fixes at ratio 3–6
([Galileo note](ppc_clas_galileo_opt_in.md#results)). These experiments asked
whether the PPC IMU could help in three ways:

- a better reset position;
- a better time update;
- an independent check of the first fix after a reset.

None of them improves the default or the Galileo opt-in without a regression
elsewhere. The INS has no absolute position except the CLAS FIX itself:

- Anchored on every FIX, it follows the wrong fixes too.
- Anchored only on long-held fixes, it drifts by as much as the wrong fixes
  are off.

## Setup

- **INS.** The library's loosely coupled ESKF (`LooseCouplingProcessor`) runs
  beside the CLAS filter on the 100 Hz `imu.csv`. It uses tactical-grade noise:
  accelerometer `2.84e-4`, gyro `4.01e-5`, accelerometer bias walk `3.14e-4`,
  gyro bias walk `9.70e-6`.
  - The PPC samples are already FLU (stationary Acc Z ≈ +9.81).
  - Lever arms are in body FLU: Tokyo `0.31,0,0.55`, Nagoya
    `0.593,0.670,1.216` ([convention](ppc_rtk_vs_gnss_imu_fusion.md#ppc-lever-arm-convention)).
- **Position aiding.** Each CLAS FIX, with σ 0.1 m horizontal and 0.2 m
  vertical.
- **Velocity aiding.**
  - The first version (A1 below) used the CLAS position difference between
    consecutive epochs.
  - Later versions use the SPP Doppler velocity of the CLAS seed. Its σ is the
    seed's velocity covariance plus 0.1 m/s. The SPP is independent of the
    CLAS filter.
- **Heading.**
  - With the default heading latch, the heading latched 96° off while the car
    reversed on tokyo/run1 and nagoya/run1.
  - The Doppler versions use the direction test and consider attitude and
    biases before the latch. The INS is used only after the heading has
    converged.
- **Scoring.** As in the [validation note](ppc_clas_validation.md): six runs,
  60-epoch skip, FIX = status 6, horizontal error. "Within 0.3 m" is the share
  of all epochs with a FIX within 0.3 m.

With the INS only listening, the CLAS output is byte-identical to the base on
all six runs, with and without Galileo.

## INS accuracy at resets

At every full reset (15-epoch FLOAT and `maxdiffp`) of the Galileo opt-in
run, the Doppler-aided INS is compared with the SPP seed:

| Time since the last CLAS FIX | Resets | INS error median / p90 | SPP seed error median / p90 | INS closer |
| --- | ---: | ---: | ---: | ---: |
| 0–10 s | 552 | 0.55 / 3.37 m | 5.17 / 18.95 m | 89% |
| 10–30 s | 684 | 2.14 / 10.72 m | 7.71 / 33.79 m | 81% |
| 30–60 s | 629 | 3.55 / 12.16 m | 8.98 / 41.72 m | 74% |
| 60–120 s | 712 | 4.16 / 9.55 m | 15.00 / 58.47 m | 82% |
| over 120 s | 974 | 12.39 / 29.16 m | 10.74 / 75.24 m | 55% |

On average the INS is the better reset position for two minutes after a FIX.

## Results

Each row changes one thing against the base of its table. The variants are:

- **A1.** The INS position replaces the reset seed when the last FIX is at
  most 30 s old. The INS uses CLAS velocity.
- **Time update.** The INS displacement since the previous epoch replaces the
  filter's position and velocity prediction.
- **Velocity measurement.** A velocity row with a χ² gate (16.27) is added to
  the CLAS filter.
- **M2.** As A1, with the Doppler-aided INS and a 60 s age limit.
- **Reset variance.** The position variance at the reset replaces the filter's
  900 m², either with a fixed value or with the INS covariance.

Galileo opt-in (`GNSS_PPP_CLAS_GALILEO=1`):

| Variant | FIX | Within 0.3 m | FIX RMS2D | > 1 m | > 3 m |
| --- | ---: | ---: | ---: | ---: | ---: |
| Base | 35.87% | 24.53% | 1.072 m | 389 | 76 |
| A1 | 35.48% | 24.34% | 0.784 m | 378 | 64 |
| A1, reset variance 4 m² | 35.33% | 23.89% | 0.830 m | 433 | 118 |
| A1, reset variance 1 m² | 34.83% | 23.65% | 1.137 m | 332 | 57 |
| Time update, CLAS-velocity INS | 3.46% | 1.32% | 0.360 m | 0 | 0 |
| Time update, Doppler INS | 34.73% | 23.26% | 0.751 m | 377 | 51 |
| Velocity measurement, from the INS | 32.82% | 23.03% | 0.937 m | 254 | 78 |
| Velocity measurement, from SPP Doppler (no IMU) | 35.98% | 24.71% | 1.204 m | 357 | 120 |
| M2 | 35.86% | 24.33% | 0.816 m | 423 | 99 |
| M2, age limit 120 s | 35.70% | 24.34% | 0.716 m | 428 | 113 |
| M2, reset variance 4 m² | 35.08% | 24.08% | 0.623 m | 254 | 7 |
| M2, reset variance 1 m² | 35.68% | 24.57% | 0.622 m | 446 | 10 |
| M2, reset variance from the INS covariance | 33.98% | 24.46% | 0.630 m | 163 | 32 |
| M2, 4 m², INS anchored only on fixes held 10 s | 35.65% | 24.60% | 1.069 m | 388 | 75 |

Default (GPS+QZSS):

| Variant | FIX | Within 0.3 m | FIX RMS2D | > 1 m | > 3 m |
| --- | ---: | ---: | ---: | ---: | ---: |
| Base | 27.23% | 17.26% | 0.361 m | 179 | 0 |
| A1 | 27.21% | 17.23% | 0.361 m | 179 | 0 |
| A1, reset variance 4 m² | 26.91% | 16.90% | 0.399 m | 324 | 0 |
| Time update, CLAS-velocity INS | 3.11% | 0.95% | 0.367 m | 0 | 0 |
| Time update, Doppler INS | 26.24% | 16.49% | 0.550 m | 735 | 0 |
| Velocity measurement, from the INS | 24.46% | 14.03% | 0.757 m | 1702 | 0 |
| Velocity measurement, from SPP Doppler (no IMU) | 26.92% | 16.90% | 0.348 m | 149 | 0 |
| M2, age limit 120 s | 27.27% | 17.26% | 0.370 m | 201 | 0 |
| M2, reset variance 4 m² | 27.46% | 17.42% | 0.527 m | 719 | 0 |
| M2, reset variance from the INS covariance | 27.62% | 17.30% | 1.014 m | 843 | 583 |
| M2, 4 m², INS anchored only on fixes held 10 s | 27.31% | 17.43% | 0.367 m | 206 | 0 |

What the tables show:

- **The one row that looks like a gain is not robust.** M2 with a 4 m² reset
  variance cuts FIX above 3 m from 76 to 7 with Galileo, but the variance was
  picked on these runs. The same setting on the default profile holds a 2.3 m
  wrong fix on nagoya/run1 for 78 s, and FIX above 1 m rise from 179 to 719.
- **The time update with the CLAS-velocity INS collapses FIX.** Its heading
  was up to 30° off, and the float-derived velocity closed an unstable loop.
  The Doppler INS is stable, but its time update does not help.
- **The Doppler velocity row needs no IMU and does not help either.** On the
  default it cuts FIX above 1 m from 179 to 149, at a cost of 0.36 pp of FIX
  within 0.3 m. With Galileo, FIX above 3 m rise from 76 to 120.

## Why the IMU does not help here

- **The INS absorbs wrong fixes.**
  - Its only position aiding is the CLAS FIX at 0.1 m.
  - Wrong fixes come as runs of short segments, so by the time the filter
    resets, the INS sits on the wrong fix.
  - A reset seeded from it with a tight variance re-fixes on the same wrong
    position. On nagoya/run1, the INS-covariance seed held a 5.1 m wrong fix
    for 116 s.
- **Its covariance is too small.** At the nagoya/run1 resets of that run, the
  INS error over its own 3D sigma has a median of 2.1 and a p90 of 5.3. 25% of
  the resets are beyond 3σ.
- **With the default 900 m² the INS position carries almost no
  information.** The first code update pulls the float to the code solution.
  The INS seed only moves where the next fixes happen, and so which of them
  are wrong. For example, M2 adds a 27-epoch 6.5 m wrong fix on tokyo/run2.
- **A check of the first fix against the INS does not separate right from
  wrong.** For each FIX segment of the Galileo opt-in, the first FIX was
  compared with the INS prediction at the same epoch (horizontal distance):

| INS position aiding | Wrong first FIX (> 1 m), median distance | Correct first FIX (≤ 0.5 m), p90 distance | Rejecting a first FIX more than 3 m from the INS |
| --- | ---: | ---: | --- |
| Every CLAS FIX, INS age ≤ 60 s | 0.20 m (41 segments) | 1.24 m (449 segments) | removes 7 FIX above 3 m and 22 correct segments (460 FIX) |
| Fixes held 10 s, INS age ≤ 120 s | 4.20 m (17 segments) | 3.86 m (301 segments) | removes 2 FIX above 3 m and 45 correct segments (1,268 FIX) |

  - Anchored on every FIX, the INS has just absorbed the previous wrong
    segment, typically 0.4 s earlier.
  - Anchored only on held fixes, it drifts by metres over the longer gaps,
    which is as far as the wrong fixes are off.

An independent absolute anchor would be needed instead:

- RTK needs a base station and removes the point of PPP-RTK.
- SPP is metres off in these streets.
- A map constrains only one or two axes.

## Reproduce

Apply the patch on `2ea6707` and build `gnss_ppp`. Run the scorecard's parity
command for each run, as in the [Galileo note](ppc_clas_galileo_opt_in.md#reproduce),
with these variables:

| Variable | Use |
| --- | --- |
| `GNSS_DIAG_CLAS_IMU=<run>/imu.csv` | run the INS beside the CLAS filter |
| `GNSS_DIAG_CLAS_IMU_LEVER=x,y,z` | body-FLU lever arm (Tokyo `0.31,0,0.55`, Nagoya `0.593,0.670,1.216`) |
| `GNSS_DIAG_CLAS_IMU_DOPPLER=1` | aid the INS with the SPP Doppler velocity |
| `GNSS_DIAG_CLAS_IMU_DIRTEST=1`, `GNSS_DIAG_CLAS_IMU_CONSIDER=1` | heading-latch direction test, consider attitude and biases before the latch |
| `GNSS_DIAG_CLAS_IMU_REQUIRE_CONVERGED=1` | use the INS only after the heading has converged |
| `GNSS_DIAG_CLAS_IMU_NO_SEED=1` | do not replace the reset seed |
| `GNSS_DIAG_CLAS_IMU_MAX_AGE_S=<s>` | age limit of the INS seed since the last FIX (default 30) |
| `GNSS_DIAG_CLAS_IMU_SEED_VAR_M2=<m²>` | fixed reset position variance |
| `GNSS_DIAG_CLAS_IMU_SEED_COV=1` | reset position variance from the INS covariance |
| `GNSS_DIAG_CLAS_IMU_MIN_FIX_RUN=<epochs>` | anchor the INS only on fixes held that many epochs |
| `GNSS_DIAG_CLAS_IMU_TIME_UPDATE=1` | INS displacement as the time update |
| `GNSS_DIAG_CLAS_VEL_UPDATE_SIGMA=<m/s>` | velocity measurement in the CLAS filter (`GNSS_DIAG_CLAS_VEL_UPDATE_DOPPLER=1` for SPP Doppler instead of the INS) |
| `GNSS_DIAG_CLAS_IMU_DUMP=<csv>` | per-epoch INS prediction, for the reset and first-FIX comparisons |

Add `GNSS_PPP_CLAS_GALILEO=1` for the Galileo tables. All rows were produced
with one build of the patch. The six runs were used both to design the
variants and to measure them; no held-out CLAS kinematic data with truth is
available here.
