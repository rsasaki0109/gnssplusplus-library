# Frozen holdout contract: default switch to `velocity_consistency_v6` (v1)

This contract is frozen in a commit before any candidate replay of the holdout
data. It decides whether the online PVA production default should become
`velocity_consistency_v6`. The development record for that candidate is
[online_pva_candidate_v7.md](online_pva_candidate_v7.md) and its
[results](online_pva_candidate_v7_results.md).

That record was built only on the six PPC-Dataset runs. Every one of those runs
was used to diagnose and design v1-v6. This contract evaluates the candidate,
unchanged, on data that played no part in its development.

Go here does **not** switch the default by itself. Go means a separate,
reviewed PR proposes the switch to the maintainer. No-Go keeps the default and
is recorded like every earlier No-Go.

## Holdout data

UrbanNav Tokyo, 2018-12-19 (IPNL-POLYU/UrbanNavDataset, `Tokyo_Data.zip` on
Dropbox), runs **Odaiba** (1,241 s) and **Shinjuku** (2,095 s).

- **Instruments:**
  - Rover receivers: Trimble NetR9 at 10 Hz and u-blox at 5 Hz.
  - Base: Trimble NetR9, 1 Hz, site CREF0001.
  - Navigation: multi-GNSS broadcast merge (`base.nav`).
  - IMU: 50 Hz.
  - Truth: Applanix POS LV, 10 Hz, with position, ENU velocity, roll, pitch
    and heading.
- **Independence from PPC 2024:**
  - It is a different year, vehicle, rover receiver and IMU, and a different
    drive.
  - It shares the base site with PPC Tokyo, and the Odaiba area is near PPC
    Tokyo 3. The data does not overlap.
- **What was looked at before this freeze, all during a format survey:**
  - Headers.
  - Rates and gaps.
  - Constellations and signals.
  - Baseline lengths from truth positions.
  - The IMU axes, from the specific force at rest and the correlation of gyro
    z with truth heading rate.
  - An IMU-versus-truth time cross-correlation.
- **What was not run before this freeze:** no estimator of this repository
  (`none` or any candidate) was run on this data, and no accuracy metric was
  computed.
- The data is not redistributed. The repository records the SHA256 of each
  raw and converted file, not the files.

## Conversion to the PPC run layout (fixed here, no tuning)

A converter script in the repository writes, per run and rover, a directory
`<root>/urbannav/<run>_<rover>/` holding the PPC file set.

1. **`rover.obs`:**
   - u-blox: the file unchanged (5 Hz).
   - Trimble: only epochs whose GPS time of week is a multiple of 0.2 s.
     Header and observation records are otherwise byte-preserved.
2. **`base.obs`:** `base_trimble.obs`, unchanged.
3. **`base.nav`:** `base.nav`, unchanged.
4. **`imu.csv`:** PPC header and units. The steps, in order:
   1. **Axes.** The UrbanNav IMU is FRD; it is mapped to FLU with
      x -> x, y -> -y, z -> -z for both the accelerometer and the gyro. The
      evidence is the specific force at rest, z = -9.78 m/s^2, and a +0.9997
      correlation of gyro z with the clockwise-from-north truth heading rate.
   2. **Units.** Gyro rad/s -> deg/s.
   3. **Wheel speed** is dropped.
   4. **Resampling.** UrbanNav IMU timestamps are not on a fixed grid. The
      replay requires an IMU sample at each rover epoch, within 1e-6 s. So the
      samples are resampled by linear interpolation onto the grid
      t = k x 0.02 s (50 Hz) of GPS time of week.
      - Only grid points strictly inside two consecutive raw samples at most
        0.1 s apart are written.
      - No extrapolation is done, and no time offset is applied.
5. **`reference.csv`:** PPC header names. `Velocity X/Y/Z` are renamed to
   `East/North/Up Velocity (m/s)`; the survey showed they are ENU. Only rows
   whose GPS time of week is a multiple of 0.2 s are kept. Everything else is
   copied unchanged.

## Replay configuration (fixed here)

`gnss_pva_replay` accepts a third layout, `urbannav/<run>`, in addition to
PPC `tokyo|nagoya/<run>`.

- **Lever arm.** For `urbannav` the antenna lever arm is **(0, 0, 0) m in FLU**.
  - UrbanNav documents no antenna-IMU lever arm for Tokyo. The maintainer chose
    zero as a declared assumption instead of a value estimated from data.
  - Control and candidate use the same zero lever arm.
- **Everything else** is the configuration the replay already uses: the base
  position from the base RINEX header, RTK settings, initialization and
  time handling.
- **PPC is unaffected.** The tokyo and nagoya branches are unchanged, and PPC
  outputs stay bit-identical (gate 7).

## Population and comparison

- **Population:** 2 runs (Odaiba, Shinjuku) x 2 rovers (u-blox, Trimble) x 3
  scenarios.
  - Normal.
  - GNSS outage, 60-70 s.
  - IMU gap, 60-64 s.
- That is 12 control replays (`--candidate none`, the current production
  default) and 12 candidate replays (`--candidate velocity_consistency_v6`).
  Both are built from the same binary and interleaved on the same quiet host,
  at most 3 at once.
- The existing comparator `scripts/analysis/compare_online_pva.py` applies to
  each rover set separately (two invocations). It is extended only as needed
  to accept the UrbanNav run names, with no change to any gate.

## Acceptance (the frozen gates, applied per run/scenario)

1. RTK and fused position, RTK and fused velocity, and full-rotation RMSE and
   P95 are each <= 1.01 x control. This holds on both the all-output and the
   common-valid cohorts.
2. Coverage loses at most 0.1 percentage point. Coverage here is the RTK,
   fused, velocity, attitude and heading availability.
3. The first fresh attitude and the first heading latch come no later. The
   scenario GNSS-update, fresh-attitude and heading recovery also come no
   later. A null value is censored, never zero.
4. Processor P95 <= 2 x control.
5. At least one normal-run full-rotation RMSE/P95 improvement.
6. Missing metrics, scenarios, truth mismatches or input-hash mismatches fail.
7. Candidate `none` built from the candidate tree is bit-identical to candidate
   `none` from the develop commit this branch starts from. This covers every
   deterministic CSV field on the 18 PPC runs.

**Go only if every gate passes on all 12 run/scenarios.** Otherwise record
No-Go with the failed gates, and keep the default. Do not change the
candidate, the conversion or the configuration after seeing any holdout
result.

## Pipeline smoke allowed before the candidate run

- **Allowed run:** one bounded control-only replay, `--candidate none
  --max-epochs 300`, on each of the four converted directories.
- **What it checks:** that the conversion and the layout run, meaning state
  `passed` and a nonzero fused availability.
- **What it does not look at:** no error metric is read. This is recorded in
  the results.
- **If the smoke fails:** if it fails for a format reason, the converter may be
  fixed to meet this contract's specification. The specification itself does
  not change. The fix and its reason are recorded.

## Reported in addition (not gates)

- Absolute errors, with the warning that the zero lever arm and the unknown
  truth reference point bias them by up to about 1 m.
- The IMU-versus-truth time offset of about 0.15 s seen in the survey. It is
  not corrected, and it affects both control and candidate.
- Per-run v6 behaviour: direction-test flips, rover-gap RTK-only resets and
  gyro-bias seeds.
