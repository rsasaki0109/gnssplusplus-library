# Online RTK base-epoch extrapolation results: No-Go

The frozen contract is [online_rtk_base_extrapolation_v1.md](online_rtk_base_extrapolation_v1.md).
It was frozen at `a760429f`, together with the implementation, before any
comparison replay. Contract SHA256 is
`b652802ce966253803012cb1ed9c03619a18b083e557147e1abe665abc001abb`.

The candidate stays opt-in and the default is unchanged. Nothing was tuned
after the results were seen. The machine-readable record is
[online_rtk_base_extrapolation_decision_v1.json](online_rtk_base_extrapolation_decision_v1.json).

## Decision

**No-Go.** The candidate fails 133 of 558 gates on PPC (18 run/scenarios) and
107 of 186 gates on UrbanNav (6 run/scenarios). It also fails the targeted
rotation improvement gate on UrbanNav.

- The comparison used 48 replays from one binary (SHA256 `9ac63bcb...`), run
  interleaved on the quiet host. All 48 passed, with full truth matches.
- Gate 7 holds. Candidate `none` from this tree is bit-identical to develop
  `c37979c3` on all 18 PPC runs (175,902 rows).
- Processor P95, candidate over control, is 1.07 / 1.37 / 2.0 (min / mean /
  max).

### Normal runs, control -> candidate

All values are RMSE. Position is in m, velocity in m/s, rotation in deg.

| Run | RTK position (RMSE / P95) | RTK velocity | Fused position | Rotation |
|---|---|---|---|---|
| Tokyo 1 | 32.2 / 61.7 -> 32.0 / 70.7 | 2.10 -> 12.73 | 71.7 -> 47.1 | 105.1 -> 69.9 |
| Tokyo 2 | 19.5 / 29.7 -> 39.6 / 145.9 | 2.26 -> 7.65 | 74.2 -> 43.1 | 100.3 -> 8.1 |
| Tokyo 3 | 18.6 / 31.9 -> 22.3 / 33.2 | 2.10 -> 9.89 | 41.9 -> 53.3 | 63.5 -> 25.4 |
| Nagoya 1 | 25.7 / 29.0 -> 25.5 / 29.0 | 2.51 -> 7.00 | 49.3 -> 26.4 | 88.2 -> 105.8 |
| Nagoya 2 | 37.7 / 81.8 -> 18.4 / 18.7 | 2.30 -> 4.65 | 81.0 -> 9.5 | 18.5 -> 16.2 |
| Nagoya 3 | 43.7 / 101.1 -> 53.9 / 118.0 | 2.47 -> 4.87 | 58.9 -> 68.5 | 105.0 -> 65.1 |
| Odaiba | 199.9 / 47.8 -> 199.1 / 27.1 | 3.84 -> 11.50 | 81.2 -> 128.5 | 105.6 -> 110.6 |
| Shinjuku | 27.6 / 65.6 -> 98.8 / 58.1 | 1.96 -> 6.57 | 82.8 -> 1.1e11 (diverged) | 68.7 -> 104.5 |

## What the result shows (post-hoc diagnosis, not used to change the candidate)

**What did work.** The extrapolation does what it was designed to do. RTK
status counts, control -> candidate:

| Run | SPP | FLOAT | FIXED |
|---|---|---|---|
| Tokyo 1 | 11,171 -> 1,967 | 660 -> 9,105 | 15 -> 773 |
| Nagoya 2 | 8,469 -> 162 | 908 -> 8,134 | 49 -> 1,130 |

FLOAT errors drop as well (P50 / P95):

| Run | Control | Candidate |
|---|---|---|
| Tokyo 1 | 4.5 / 141 m | 1.5 / 76 m |
| Nagoya 2 | 4.0 / 165 m | 1.3 / 15 m |

FIXED epochs keep a 0.04-0.10 m P50 on PPC.

**Why it is still No-Go.**

- **The online RTK filter's velocity breaks down.** RTK velocity RMSE is 2-4 x
  to 6 x worse on every run. The fused filter consumes that velocity, which
  explains the fused divergence on Shinjuku and the worse fused rows. Running
  the online filter's velocity states and tight external time update at 5 Hz
  differential cadence is evidently not stable as configured. It was only ever
  exercised at the 1 Hz exact-base cadence.
- **The online RTK float is still far worse than the batch product.** Tokyo 1
  shows a candidate FLOAT P50 of 1.5 m against the batch `gnss solve` low-cost
  profile (H P50 0.03 m, 82-84 % fix).
  - The online path runs a default `RTKConfig` without the product preset and
    guards. That is a larger gap than base alignment.
- **On UrbanNav every FIX is biased.** Shinjuku FIX error is P50 0.6-0.7 m in
  both arms. The base position comes from the RINEX header, which is about
  0.7 m from the surveyed position (`docs/benchmarks.md`). That affects
  control and candidate alike.

## Next step suggested by this result

Base alignment is necessary but not sufficient. The online path should first
reuse the batch product's RTK configuration (preset, guards, ambiguity
validation) and verify that its RTK velocity output is sane at full cadence.
After that, base alignment should be re-evaluated under a new contract. These
runs are now development data for that work.

## Later: the hold left out the satellite clock

Found on 2026-11-01, after this decision; it changes nothing above.

- **What was missing.** `rtk_base_alignment::holdBaseEpoch` modeled the
  satellite motion but not the satellite clock.
  - A hold keeps the residual of the base epoch, and that residual contains
    the satellite clock at the base time.
  - The held code and carrier were therefore off by the clock change,
    c × drift × age. An interpolation does not have this problem, because it
    follows the clock through the residual it interpolates.
- **Size on PPC.** One-second holds of each base epoch were compared with the
  next real epoch, with the per-epoch median removed (the base receiver
  clock). Carrier RMS per satellite, nagoya/run3 / tokyo/run3:

  | System | Before | With the satellite clock |
  | --- | --- | --- |
  | GPS | 3.8 / 4.8 mm | 2.3 / 2.9 mm |
  | BeiDou | 8.0 / 5.5 mm | 2.4 / 2.2 mm |
  | Galileo | 8.9 / 15.2 mm | 7.4 / 7.2 mm |

  - The median 3D error of correct FIX at a 0.8 s hold on tokyo/run3 fell
    from 0.057 m to 0.038 m. Interpolation gives 0.032 m there.
- **Fix.** The hold now models the broadcast satellite clock at the
  transmit time, so the clock moves to the target time with the satellite.
  `calculateModeledBaseRange` itself is unchanged.
- **Which runs change.** Only paths that use the hold: the online processor
  with `base_extrapolation_max_age_s > 0`, which is off by default, and the
  `gnss_pva_replay` candidates that set it. The results above were produced
  with the earlier model.
