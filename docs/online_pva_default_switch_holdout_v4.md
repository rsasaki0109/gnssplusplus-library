# Frozen holdout contract: default switch to `independent_doppler_v1` (v4)

This contract is frozen in a commit before any candidate replay of the holdout
data, and before any estimator of this repository has been scored on it. It
decides whether the online PVA production default may be **proposed** to change
from `none` to `independent_doppler_v1`: the production default plus exactly one
option, `OnlineRtkImuProcessor::Config::independent_doppler_velocity = true`.

Go here does **not** switch the default by itself. It permits a separate,
reviewed PR that proposes the switch; the maintainer decides (see
[Decision](#decision)). No-Go keeps the default, is recorded, and ends the
evaluation of this candidate on this data.

## The candidate

`independent_doppler_v1` = candidate `none` (the production default) + one option:

| Option | Default | Candidate |
|---|---|---|
| `OnlineRtkImuProcessor::Config::independent_doppler_velocity` | `false` | `true` |

With the default, the RTK filter carries a velocity state that is the tight INS
prediction fed back by `reanchor()`, and that state is what the loose filter and
the next tight `reanchor()` receive as the GNSS velocity: a self-confirming
loop. With the option, the velocity fed to both is instead the Doppler
least-squares velocity at the RTK position
(`spp_velocity::solveVelocityFromObservations`, with its own covariance); when it
cannot be solved the epoch carries no GNSS velocity, never the RTK state velocity
(`src/fusion/online_rtk_imu.cpp`, `processRover`). The option has existed since
`velocity_consistency_v1` and is covered by `tests/test_online_rtk_imu.cpp`.

What it is **not**: no RTK preset, no base extrapolation, no epoch-SPP velocity
(`independent_velocity_from_epoch_spp`), no NIS gates, no float re-anchor, no
latch re-anchor, no consider update, no constant, no change of any library
source. `gnss_pva_replay --candidate independent_doppler_v1` sets that one field
after `candidate none`'s configuration and records
`"independent_doppler_velocity":true` in `replay.json` (for this candidate only;
the `replay.json` of `none` is byte-identical to develop's). The candidate block
is pinned by `tests/test_pva.py::IndependentDopplerCandidateTest`.

**Attitude is not claimed.** The default's attitude is known to be broken on
development data: on several runs 35 to 52 % of scored epochs have a
`rotation_deg` above 90 deg (table below), and this candidate does not fix it
(it feeds a velocity, not an attitude constraint). The candidate is therefore
evaluated **for position and velocity only**. Attitude enters this contract in
two ways only: the pooled H1 and H3 gates include `rotation_deg` with the same
thresholds as in holdout v2, and the relative gate H8r requires that the
candidate does not lose attitude more often than the control. An **absolute**
attitude-integrity gate (as H8 of
[holdout v3](online_pva_default_switch_holdout_v3.md)) is out of scope for this
candidate; the attitude problem of the default is left open by this contract,
whatever its result.

### Development evidence (development data only)

Recorded before this contract, by the maintainer's development work, on ten
runs that are **all development data** for this candidate: HK Deep-Urban-1 and
Harsh-Urban-1 (which the [v3 contract](online_pva_default_switch_holdout_v3.md)
scored and consumed), PPC tokyo1-3 and nagoya1-3, UrbanNav Odaiba and Shinjuku
(normal scenario, binary built from develop `adb40608`, a development
experiment hook that is **not** in this tree set `independent_doppler_velocity`
on candidate `none`; table `main_normal.md`, SHA256
`c3f0737c126089ec3993edebfbc480d195d77de7bf121b883bfc6853553f3ae8`):

| Run | Fused position RMSE, `none` -> candidate (m) | Fused velocity RMSE (m/s) | Rotation > 90 deg, scored epochs (%) |
|---|---:|---:|---:|
| HK Deep | 66.5 -> 60.1 | 8.1 -> 7.5 | 34 -> 35 |
| HK Harsh | 348.0 -> 317.5 | 12.8 -> 10.3 | 42 -> 40 |
| tokyo1 | 40.7 -> 12.0 | 5.0 -> 3.6 | 51 -> 52 |
| tokyo2 | 14.9 -> 10.5 | 1.5 -> 0.9 | 0 -> 0 |
| tokyo3 | 16.9 -> 6.4 | 3.2 -> 0.9 | 4 -> 0 |
| nagoya1 | 104.2 -> 14.1 | 5.7 -> 3.8 | 51 -> 51 |
| nagoya2 | 18.8 -> 19.1 | 1.6 -> 1.4 | 0 -> 0 |
| nagoya3 | 26.4 -> 19.9 | 2.5 -> 1.6 | 0 -> 0 |
| Odaiba | 14.3 -> 9.6 | 4.3 -> 3.7 | 52 -> 52 |
| Shinjuku | 210.4 -> 27.2 | 31.3 -> 4.0 | 43 -> 18 |

Fused position RMSE improved on 9 of 10 runs and is flat on nagoya2. Rotation
above 90 deg is not removed (35 to 52 % on several runs). Tail regressions also
recorded: HK Deep RTK-position P95 +8 %; nagoya2 IMU-gap fused-position P95
33 -> 51 m; HK Harsh IMU-gap fused-position P95 359 -> 479 m. The rotation RMSE
of the candidate is not lower than the control's on tokyo1 (104.85 -> 105.55 deg
in the check below). Tokyo1 was re-run with the candidate implemented in this
tree for this contract and reproduces the table (fused position RMSE 40.67 ->
12.03 m; see [Implementation checks](#implementation-checks-on-development-data)).

The `none` of this table is the develop `adb40608` default. It is **not** the
control of the v3 results: develop commits `3835e9e0` (kinematic epochs without
an independent SPP are skipped, SPP gated by formal sigma) and `f48d5f37` (no
INS-seeded floats without a supporting code row) changed the RTK output after
the v3 freeze, and the control of HK Deep/Harsh moved with them (v3 control
fused-position RMSE 2,246 m and 5,978 m; 66.5 m and 348.0 m here).

## Relation to earlier contracts

This contract is based on [holdout v3](online_pva_default_switch_holdout_v3.md)
(frozen `a23e8311`, result No-Go for `velocity_consistency_v10` on HK
Deep/Harsh) and keeps its H1-H7 gates, thresholds and method unchanged. Every
difference:

| Item | v3 | v4 (this contract) |
|---|---|---|
| Candidate | `velocity_consistency_v10` | `independent_doppler_v1` (see above); any other name is refused by the comparator |
| Population | HK Deep-Urban-1, Harsh-Urban-1 (2 runs) | HK **Medium-Urban-1** (1 run, `HKMediumUrban1_novatel`); the tunnel run is **not** used (see [Run decision](#run-decision-medium-urban-1-only)) |
| Replays | 6 control + 6 candidate | 3 control + 3 candidate |
| Comparator gate set | `holdout_v3`: H1-H8, 42 gates per run | `holdout_v4`: H1-H7 as v2/v3 (same thresholds) + **H8r**, 42 gates per run |
| Attitude gate | H8 absolute (candidate fraction > 90 deg <= 0.01) | **H8r relative** (candidate fraction <= control fraction + 0.01), per scenario replay; no absolute attitude gate |
| Gate 7 | binary of the freeze tree byte-identical to develop `acbea805` | `none` output of the freeze binary bit-identical to a develop `adb40608` build on the 18 PPC control replays (see [Gate 7](#gate-7)) |
| Replay binary | SHA256 `440b24ce...` | SHA256 `a6c0d7fb...` |
| Converter | `HKDeepUrban1`, `HKHarshUrban1` | one definition changed (and a comment): the run-label whitelist also accepts `HKMediumUrban1` (separate commit, with a test; see [Conversion](#conversion-to-the-ppc-run-layout-the-frozen-converter)) |
| Frame check, shared helpers, scorer | see v3 | unchanged, same SHA256 |
| `gnss_pva_evaluate.py` | `7b8f5ff6...` | only the list of accepted `--candidate` names changed (adds `independent_doppler_v1`) |
| `gnss_pva_replay.cpp` | `af4366b9...` | the `independent_doppler_v1` candidate: its block, its `replay.json` field and its name in the usage text and the argument check (17 lines added, 2 changed) |
| Decision rule | Go on Hong Kong alone permits a proposal | the same, on one run (see [Decision](#decision)) |

The v3 outcome does not bear on this contract's gates; HK Deep and Harsh are
development data for this candidate (they appear in the table above).

## Why the gates are those of holdout v2

The per-scenario gates of contracts v1-v11 (every statistic at most 1.01 x the
control on each of 3 scenarios) are below the control's own scenario-to-scenario
noise, as the [v3 contract](online_pva_default_switch_holdout_v3.md#why-the-gates-differ-from-contracts-v1-v11)
explains. The gates below pool the three scenario replays of the run and are the
H1-H7 of holdout v2 with the **same thresholds**, chosen there before any holdout
data was scored. H8r is new and is stated before any scoring.

**A property of the inherited gates that the reviewer must weigh.** H1 requires
the candidate's pooled fused-position **and rotation** RMSE and P95 to be at most
1.00 x the control's, in both cohorts. For rotation this is a strict gate on a
quantity the candidate is not designed to change: on development data its
rotation error moves by noise in either direction (tokyo1: 104.85 -> 105.55 deg
RMSE, +0.7 %). The rotation half of H1 can therefore fail on noise alone, and a
No-Go can result from it although position and velocity improve. The thresholds
are **not** relaxed for that reason: they are fixed here, before any holdout
data, as the conservative choice, and a failure on them is a No-Go.

## Holdout data

UrbanNav Hong Kong (IPNL-POLYU/UrbanNavDataset, Dropbox downloads), run
Medium-Urban-1:

| Run | Contract name | Date (GPST) | Truth span (GPS TOW, week 2158) | Converted rover epochs |
|---|---|---|---:|---:|
| Medium-Urban-1 (TST) | `HKMediumUrban1_novatel` | 2021-05-17 | 95593 - 96379 s | 704 (of 705) |

- **Instruments** (the same platform as Deep-Urban-1 and Harsh-Urban-1):
  - Rover: NovAtel Flexpak6, 1 Hz on integer GPS seconds, converted to RINEX
    3.03 by RTKCONV demo5 b33c in GPS time. GPS L1 C/A and L2 P(Y), GLONASS
    L1 C/A and L2 P, BeiDou B1I and B2I. No Galileo. Pseudorange, carrier phase,
    Doppler and C/N0 on each signal. No event records. One GLONASS slot/frequency
    entry (R18) in the header.
  - Base: HKSC (Hong Kong Lands Department, Leica GR50, 1 Hz, GPS, GLONASS,
    Galileo and BeiDou), RINEX 3.02, hourly file 02 (UTC) of day 137,
    `rinex.geodetic.gov.hk/rinex3/2021/137/HKSC/1s/`. The header position
    (-2414266.9197, 5386768.9868, 2407460.0314) m is the base position.
  - Navigation: the HKSC daily broadcast files GN, RN, EN and CN of day 137
    (`HKSC00HKG_R_20211370000_01D_{GN,RN,EN,CN}.rnx.gz`).
  - IMU: Xsens MTi-10, about 400 Hz (median step 2.5 ms, largest 10.2 ms), ROS bag
    CSV (`xsense_imu_medium_urban1.csv`) with Unix UTC `header.stamp`; the bag time
    minus the sensor stamp is 0.3 ms (median; the sensor stamp is used).
  - Truth: 1 Hz post-processed text file `UrbanNav_TST_GT_raw.txt` (NovAtel SPAN-CPT
    + IE) with UTC and GPS time, latitude and longitude as D M S, ellipsoidal
    height, body-frame velocity and acceleration, roll, pitch, heading and a
    quality flag Q.
- **Difficulty (truth and rover files only; no estimator):**
  - Baselines to HKSC from the truth positions: 4.27 / 4.54 / 4.68 km (min / median / max).
  - The vehicle is stopped (horizontal speed < 0.2 m/s) in 35 % of the converted
    rows. Maximum speed 11.5 m/s.
  - **The rover tracks few satellites.** Median satellites per rover epoch: 4
    (the satellite count of the RINEX epoch record; minimum 1, maximum 20); 346 of
    the 704 converted epochs have 5 or more. This is much thinner than Deep (median 11)
    and Harsh (median 9). Long RTK outages and low RTK availability are to be
    expected for control and candidate alike.
  - The rover file has epoch gaps (no observations): 13 gaps of more than 1.5 s,
    11 of them more than 2 s (largest 22 s, then 15 s, 12 s, 9 s). The replay's
    rover gap limit is 2 s, which those 11 exceed.
  - The two disturbance windows of the scenarios (below) hold real observations:
    10 rover epochs (at most 5 satellites, median 4) in 60-70 s and 4 epochs (at
    most 4 satellites, median 4) in 60-64 s, counted from the first converted epoch. The GNSS
    outage scenario therefore removes few epochs, and the IMU-gap scenario removes
    IMU samples over 4 s of a sparse stretch.
  - Truth quality Q (converted rows): 1/2/3 = 56/536/112. The meaning of Q is not
    documented in the files; no row is excluded by it.
- **Independence from development data, and its limit.** A different route
  (Tsim Sha Tsui) and day (2021-05-17) from every development run. But it is the
  **same platform** as HK Deep-Urban-1 and Harsh-Urban-1, which are development
  data for this candidate (they are in the table above and the v3 contract
  scored them): same vehicle, NovAtel Flexpak6 rover, Xsens IMU, HKSC base
  network and conversion. It is a different place and day, not a different
  sensor suite. It does not overlap PPC or UrbanNav Tokyo.
- **Data access:** the data has no stated licence and is not redistributed. The
  repository records SHA256 of each raw and converted file, not the files.

## Run decision: Medium-Urban-1 only

Decided here, before any estimator touched either run.

- **Used:** UrbanNav-HK-Medium-Urban-1, NovAtel Flexpak6 rover. One run.
- **Not used: UrbanNav-HK-Tunnel-1** (2021-05-18, `CHTunnel`). The tunnel rover
  file covers only 197 of the 401 truth seconds (49 %), with a longest gap of 177 s,
  and its median is 18 satellites where it observes. That is a fragmented
  record: the GNSS-outage and IMU-gap windows (60-70 s and 60-64 s of elapsed
  time) would fall in or beside its long gap, so the recovery gates H5 would not
  measure what they measure on a continuous run; its replays would be dominated by
  one 177 s reset. The full tunnel IMU CSV was not obtained (only a 3 kB head and
  tail of it were read in the earlier survey), a third base hour (day 138) and
  nav set would be needed, and the converter has not been run on it. It does not
  fit the PPC layout and the frozen converter as a comparable second run. The
  decision rests on the format-level coverage figures above only. The tunnel run
  is **not** to be added to this population later.
- **Consequence stated plainly:** the population is one run, 704 s long, with a
  thin satellite supply: 3 control and 3 candidate replays. A Go would rest on
  that alone (one city, one platform, one run, which shares its platform with the
  development data); a reviewing maintainer should weigh it on that basis. This
  is less than v3 asked for (two runs) and is decided here, not after a result.

## What was looked at before this freeze

All of this was done on the raw or converted files or on the frozen tools,
never with a scored estimator, and nothing compares an estimate to the truth.

- **Earlier format survey** (for holdout v2; not part of any scored run): the
  Medium-Urban-1 and tunnel GNSS files (coverage within the truth window:
  Medium 705 rover epochs in 787 truth seconds, 346 with 5 or more satellites,
  median 4, longest gap 22 s; tunnel 197 of 401, longest gap 177 s), the truth
  header of both, the Medium-Urban-1 IMU z axis (gravity, gyro z) and the gyro-z
  lag, the head and tail slices of the tunnel IMU CSV; also two pilot sets
  (2019, 2020) and further HKSC hours. Disclosed, and for the tunnel and the
  pilots, excluded.
- **New for v4, format level only:**
  - **Provenance re-check.** The raw files were downloaded again from the
    sources below and their SHA256 equal the survey's: truth text
    (Dropbox `UrbanNav_TST_GT_raw.txt`), IMU CSV (Dropbox
    `xsense_imu_medium_urban1.csv`), the GNSS folder zip (Dropbox), the HKSC hour
    file and the four nav files (`rinex.geodetic.gov.hk`).
  - **Conversion** with the frozen converter (one label added), and its offline
    checks (below).
  - **Truth-vs-truth and IMU-vs-truth frame checks**
    (`scripts/analysis/check_urbannav_hk_frames.py`, unchanged; no estimator):
    table below. The conventions are those of Deep and Harsh.
  - **The sensor-stamp versus bag-time difference** and the largest IMU step
    (0.3 ms and 10.2 ms).
  - **The BeiDou format check** (the library's RINEX reader run over the
    converted observation files, counting signal types only).
  - **Counts of rover epochs in the scenario windows and of the rover gaps**,
    from the converted `rover.obs` (above).
  - **Truth-only statistics** (baselines, stopped fraction, speeds, Q counts).
  - **The development evidence** above (recorded earlier; tokyo1 re-run for this
    contract, development data).
  - **The implementation checks** of the comparator and the candidate on
    development data (below). No Hong Kong data.
- **The pipeline smoke** below: one bounded control-only replay of the converted
  directory. Only the replay state, the fused and RTK availability and the match
  fraction were read.

**Not run before this freeze:** no scored estimator run on Medium-Urban-1 or on
the tunnel run. `gnss_pva_replay` with `independent_doppler_v1` was never run
on them; `gnss pva-evaluate`, `gnss solve`, the examples and `gnss_pva_metrics`
were never run on them. No accuracy metric was computed on them. The only
estimator output on Medium-Urban-1 is the control-only smoke (300 epochs), of
which only the fields listed under [Pipeline smoke](#pipeline-smoke) were read;
the tunnel run has none.

### The frames, checked again for Medium-Urban-1

`scripts/analysis/check_urbannav_hk_frames.py --truth <truth> --imu <imu>`:

| Check | Best convention | RMS best | RMS next | RMS worst |
|---|---|---:|---:|---:|
| Truth body velocity to FRD (452 moving epochs) | forward = y, right = x, down = -z | 0.116 m/s | 0.286 | 16.09 |
| Xsens accelerometer to FLU (785 s) | forward = +y, left = -x, up = +z | 0.089 m/s^2 | 0.640 (identity) | 11.32 |

- **Truth body frame:** x right, y forward, z up, as for Deep and Harsh.
- **Xsens frame:** x right, y forward, z up, the same orientation. The converter
  writes FLU = (y, -x, z), exactly as for Deep and Harsh: **no converter change is
  needed for the axes.**
- **Heading versus course over ground:** median -1.14 deg (moving rows only), a
  constant offset of the truth heading to the direction of travel, not corrected.
- **Time:** the IMU stamp minus truth time that maximises the correlation of the 1 s
  integrals of gyro z with the truth heading change is +0.01 s; every lag within
  5e-4 of the best correlation lies in [-0.04, +0.05] s; the correlation is
  0.99998. Information only. **No time offset is applied.**
- The boresight, at-rest gravity and roll/pitch checks of the v3 contract were not
  repeated here. The lever arm and the boresight are the declared zero assumptions
  of the replay, shared by control and candidate.

## Conversion to the PPC run layout (the frozen converter)

`scripts/convert_urbannav_hk_to_ppc_layout.py` is the converter of
[contract v2](online_pva_default_switch_holdout_v2.md) and v3, and the
conversion section of [v3](online_pva_default_switch_holdout_v3.md#conversion-to-the-ppc-run-layout-fixed-here-no-tuning)
applies word for word: GPS time throughout (UTC minus 315964800 plus 18 s, checked
for every truth row); a rover epoch is kept if and only if it has an exact truth
row and an exact IMU grid sample; `rover.obs` byte-preserved for the kept epochs;
`base.obs` the HKSC hour byte-preserved from the first to the last kept rover epoch
with event records dropped; `base.nav` the day's GN, RN, EN and CN merged; `imu.csv`
Xsens to FLU, gyro to deg/s, linearly interpolated onto the 100 Hz grid with no
extrapolation and no time offset; `reference.csv` with ENU velocity from the
body velocity, WGS84 ECEF, and a trailing Q column. Nothing about the conversion
changed.

**The one converter change, and its reason.** The converter whitelisted the run
label (`RUNS = ("HKDeepUrban1", "HKHarshUrban1")`) and refused
`HKMediumUrban1`. It now accepts that third label. This is a naming reason only:
the label names the output directory and the manifest, and no conversion step
depends on it. It is a separate commit with a test that converts the same synthetic
raw files under the Deep and the Medium label and requires every output file, hash
and statistic to be identical. The manifest's `contract` field still names the v2
contract, whose conversion text is unchanged.

Converter run (from the committed converter, `--run HKMediumUrban1`):

| Run | Raw rover epochs | Kept | Dropped | Rover epochs without truth row | Without IMU grid sample | Without exact base epoch | Reference rows without rover epoch |
|---|---:|---:|---:|---:|---:|---:|---:|
| `HKMediumUrban1_novatel` | 705 | 704 | 1 (95593 s: the IMU begins at 95593.55 s) | 0 | 0 | 0 | 0 |

- Truth rows: 787, of which 704 are written (83 have no rover epoch because of the
  rover gaps). Base: 786 epochs written (1,994 dropped before and 820 after the
  span, 1 event record dropped). Navigation records G/R/E/C: 208 / 411 / 1,724 / 431.
  IMU: 314,194 raw samples, 78,548 grid samples, none skipped.
- The rover is 1 Hz and the base 1 Hz on the same integer seconds: every kept rover
  epoch has an exact base epoch.

## Replay configuration (fixed here)

`gnss_pva_replay` accepts the `urbannav/<run>` layout. The only source change of
this freeze in the replay is the `independent_doppler_v1` block (see
[The candidate](#the-candidate)); the library is unchanged.

- **Lever arm.** (0, 0, 0) m in FLU, the declared assumption for UrbanNav. Control
  and candidate use the same zero.
- **Everything else** is the configuration the replay already uses: the base
  position from the base RINEX header, RTK settings, initialization and time
  handling. The processor limits that matter here: 2 s rover gap, 0.1 s IMU gap,
  16 pending base epochs, 10,000 pending IMU samples.
- **Candidate.** `independent_doppler_v1` exactly as in [The candidate](#the-candidate).
  No constant is added or changed for Hong Kong.

### Gate 7

What gate 7 checks in this contract, exactly. Two statements:

1. **`none` parity.** The replay binary built from the freeze tree, run with
   `--candidate none`, produces **output bit-identical** to the replay binary built
   from develop `adb40608` (`origin/develop` at the start of this work), run with
   the same arguments, on the **18 PPC control replays**: tokyo1-3 and nagoya1-3,
   each in the normal, GNSS-outage (`60 10`) and IMU-gap (`60 4`) scenarios, full runs
   (`MAX_EPOCHS = 0`). "Bit-identical" means: every field of every row of `pva.csv`
   except `processing_ms` is the same text, the header and row count are equal, and
   `replay.json` is byte-identical. Both binaries were built the same way
   (`cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DGNSSPP_BUILD_PYTHON_BINDINGS=OFF`,
   GCC 13.3.0, `cmake --build --target gnss_pva_replay`), in one build tree, one
   after the other. Result: **18 of 18 pairs bit-identical**: all 175,902 emitted rows (3 x 58,634: tokyo1 11,928, tokyo2 9,151, tokyo3 15,301, nagoya1 7,602, nagoya2 9,451, nagoya3 5,201 epochs per scenario) agree in all 61 fields except `processing_ms`, and the 18 `replay.json` files are byte-identical (`cmp`).
2. **Source scope.** `git diff --stat adb40608..<freeze>` touches, outside `docs/`,
   `scripts/` and `tests/`, only `apps/native/gnss_pva_replay.cpp` and
   `apps/commands/benchmarks/gnss_pva_evaluate.py`; `src/` and `include/` are
   unchanged, so the library code is that of develop `adb40608`.

What gate 7 does **not** claim: byte identity of the two executables. It cannot
hold, because the freeze commit changes `gnss_pva_replay.cpp` (usage text, accepted
names, the candidate block). The executables are:

| Build | SHA256 | Bytes |
|---|---|---:|
| develop `adb40608` (baseline) | `1db5dd3cc21b3fa54794744f021cd3504ac577afeda9d6419529a8c68cdc40a6` | 2,496,168 |
| freeze tree (this contract's binary) | `a6c0d7fba29731a643b58aadee686311ce0cfd6ac0e1f6cc1c3f164a30f747d0` | 2,496,168 |

Gate 7 is also not a re-check of any earlier contract's records. The comparator
requires every replay to record one and the same binary SHA256.

## Population and comparison

- **Population:** 1 run (`HKMediumUrban1_novatel`) x 3 scenarios.
  - Normal.
  - GNSS outage, 60-70 s (`--start-s 60 --duration-s 10`).
  - IMU gap, 60-64 s (`--start-s 60 --duration-s 4`).
- That is 3 control replays (`--candidate none`, the current production default) and
  3 candidate replays (`--candidate independent_doppler_v1`), built from the same
  binary and interleaved (control, candidate for each scenario in turn) on the same
  quiet host, at most 3 at once. Each replay is a full run (`--max-epochs 0`):

  ```bash
  # <arm> is none | independent_doppler_v1; <out> is <arm-dir>/normal/HKMediumUrban1_novatel
  # or <arm-dir>/scenarios/HKMediumUrban1_novatel-<scenario>
  gnss pva-evaluate --run-dir <root>/urbannav/HKMediumUrban1_novatel \
      --replay-binary <binary> --candidate <arm> --output-dir <out> \
      --scenario normal                                  # normal
  gnss pva-evaluate ... --scenario gnss_outage --start-s 60 --duration-s 10
  gnss pva-evaluate ... --scenario imu_gap     --start-s 60 --duration-s 4
  ```
- The comparison is one invocation of the existing comparator with the new gate set:

  ```bash
  python3 scripts/analysis/compare_online_pva.py --gate-set holdout_v4 \
      --baseline-dir <none>/normal --candidate-dir <cand>/normal \
      --baseline-scenario-dir <none>/scenarios --candidate-scenario-dir <cand>/scenarios \
      --runs HKMediumUrban1_novatel --output-dir <new>
  ```

  The scenario directories are named `HKMediumUrban1_novatel-gnss_outage` and
  `HKMediumUrban1_novatel-imu_gap`. `--gate-set default`, `holdout_v2` and
  `holdout_v3` are unchanged and give byte-identical outputs to the previous
  comparator. `holdout_v4` takes the candidate (`independent_doppler_v1`) and the
  contract (this document) as defaults, refuses any other `--candidate-name`, and
  refuses `--attitude-integrity` (the absolute gate is not part of it).
- **Pooling.** For the run and each arm, the scored epochs of the three scenario
  replays are pooled (704 x 3 epochs). Statistics are computed on the pooled set,
  on two cohorts:
  - **All-output:** every epoch where the arm has the metric.
  - **Common-valid:** the epochs where both arms have the metric, matched by
    scenario and elapsed time.

## Acceptance (the frozen gates, applied to the run)

The run has 42 gates. `x` is the control, `y` the candidate, both pooled as above
(H8r is not pooled: see its row). A tolerance of 1e-9 is added to every ratio gate.

| Gate | Statistic | Condition |
|---|---|---|
| **H1** primary, no worse | fused position and rotation: RMSE and P95, both cohorts (8 gates) | y <= 1.00 x |
| **H2** secondary non-inferiority | RTK position, RTK velocity, fused velocity: RMSE and P95, both cohorts (12 gates) | y <= 1.10 x |
| **H3** tail safety | fused position and rotation: P99, all-output cohort (2 gates) | y <= 1.25 x |
| **H4** coverage | RTK, fused, RTK velocity, fused velocity, attitude and heading availability, pooled (6 gates) | y >= x - 0.005 |
| **H5** timing | normal scenario: first fresh attitude, first heading latch. GNSS-outage and IMU-gap scenarios: GNSS-update, fresh-attitude and heading recovery (8 gates) | y <= x + 1.0 s |
| **H6** processor | processing P95 of each scenario replay (3 gates) | y <= 2 x |
| **H7** integrity | missing metrics, scenarios or replays, truth mismatches, input-hash mismatches, changed outputs, wrong candidate | any one fails the population |
| **H8r** attitude non-inferiority | per scenario replay (3 gates): fraction of scored epochs with `rotation_deg` > 90 deg | candidate fraction <= control fraction of the same replay + 0.01 (relative) |
| **Gate 7** parity | `none` output of the freeze binary bit-identical to develop `adb40608` on the 18 PPC control replays | separate check, recorded above |

Details of the rules (H1-H7 are those of [v3](online_pva_default_switch_holdout_v3.md#acceptance-the-frozen-gates-applied-per-run), restated):

- **Metrics.** RMSE, P95 and P99 are those of `gnss_pva_metrics.stats` (absolute
  values, linear interpolation between order statistics) on the pooled values.
  A cohort with no values for a metric is a missing metric and fails.
- **Rotation** is the full-rotation error and exists from the first heading latch.
  Coverage is counted over all emitted epochs.
- **H4 coverage** is pooled over the epochs of the three replays.
- **H5 nulls are censored, never zero.** A null (never recovered) for the candidate
  where the control is non-null fails. A null control passes. Both null passes. A
  control value of 0.0 s is a value. The comparison has 1e-6 s of tolerance.
- **H6** is a host-contention guard: a failure caused by a contended host is
  repeated unchanged on a quiet host and recorded as such; no input, binary or
  setting changes.
- **H8r.** A scored epoch is a row of that replay's `errors.csv` with a
  `rotation_deg` value. The comparison is strictly above 90 deg. For each of the
  three scenarios, `candidate_fraction <= control_fraction + 0.01` (plus 1e-9),
  both fractions taken over the scored epochs of that replay (the denominators may
  differ between the arms, since the rotation error exists from the first heading
  latch). It is relative: a candidate that flips as often as the control passes. It
  is not pooled over the three scenarios. It exists so that the candidate cannot lose
  attitude more often than the default does, **without** claiming that it repairs
  the default's attitude. It is computed with the function of the development
  `--attitude-integrity` gate (`rotation_flip_fraction`).
- **H7 details.** The comparator rejects the population as failed, with No-Go and no
  gate table, if:
  - a replay is missing, did not pass, is not a full run, or its `errors.csv`,
    `score.json` or native CSV hash differs from the manifest;
  - the two arms differ in raw input hashes, scenario, scenario window, epoch count,
    start time, base position, lever arm, navigation policy or emitted timestamps, or
    a match fraction is not 1;
  - the control is not candidate `none` or the candidate is not `independent_doppler_v1`
    (`--candidate-name` may only be that name);
  - the scenario of a directory is not the one its name says, or the window is not
    60 + 10 s (GNSS outage) or 60 + 4 s (IMU gap);
  - the three scenarios of the run are not on the same raw inputs;
  - not every replay records one and the same binary SHA256;
  - `errors.csv` disagrees with `score.json` on the count of any gated metric.

**Go only if every gate passes** (H1-H8r, 42 gates, and H7). Otherwise record No-Go
with the failed gates, and keep the default. Do not change the candidate, the
conversion, the configuration, the comparator or any threshold after seeing any
holdout result.

## Decision

- **Go** (every gate on the run) permits a **separate, reviewed PR that proposes
  `independent_doppler_v1` as the online PVA default**. Go does not change the
  default and does not itself authorize the change: the proposal goes through review
  and the **maintainer decides**, weighing that the evidence is one 704 s run with
  thin GNSS on a platform shared with the development data, that the default's
  attitude problem is untouched by the candidate, and the tail regressions recorded
  on development data (RTK P95, IMU-gap fused P95).
- **No-Go** keeps the default (`none`), is recorded with the failed gates, and
  **ends the evaluation of `independent_doppler_v1` on this data**. The result is
  not rerun, and no later population is run to compensate.
- **The data is consumed once scored.** After the first scored replay,
  Medium-Urban-1 is development data for any later candidate, exactly as the other
  Hong Kong runs became after v3. A new candidate needs a new contract and new data.
- **Nothing changes after seeing any result:** neither the candidate, the
  conversion, the configuration, the comparator, nor any threshold.

## Pipeline smoke

- **Allowed run:** one bounded control-only replay on the converted directory,
  `--candidate none --max-epochs 300`, normal scenario, with the binary of this
  freeze (`a6c0d7fb...`), run directly as

  ```bash
  gnss_pva_replay <root>/urbannav/HKMediumUrban1_novatel <out> 300 normal --candidate none
  ```
- **What it checks:** that the conversion and the layout run: the replay state is
  `passed` and fused and RTK availability are nonzero.
- **What was read:** only the replay state, the fused and RTK availability
  (`fused_status > 0`, `rtk_status > 0`) and the match fraction (the share of emitted
  epochs whose exact time has a truth row, by timestamps only). The scorer was not
  run, no error metric was computed, and no estimate was compared with the truth.
- **If the smoke fails for a format reason,** the converter may be fixed to meet this
  contract's specification. (No fix was needed.)

| Directory | State | Fused availability | RTK availability | Match fraction |
|---|---|---:|---:|---:|
| `HKMediumUrban1_novatel` | passed | 0.537 | 0.430 | 1.0 |

The availabilities are low because the first 300 epochs hold rover gaps of up to
22 s and a median of 4 satellites; they are information, not a gate. The smoke
outputs were not read further: `pva.csv` SHA256
`b331ea7f2caa6156ccdae9f81ff3294551f07d6b6a2ecae7d01c41c4abfccf27`, `replay.json`
`143a4c6412472f5895ed9bb476bfbe29e31c31867a3b060a37ca70d503051051` (for the record).
The tunnel run has no smoke and was not converted.

## BeiDou format check (no estimator)

The library's RINEX reader (`RINEXReader::readHeader` and `readObservationEpoch`
only, linked against the freeze-tree library; a small program that counts
`SignalType` and the RINEX codes per system and calls no estimator), run on the
converted files:

| File | Header version | BeiDou RINEX codes | Signal types read (observations) |
|---|---|---|---|
| `rover.obs` | 3.03 | `C2I/L2I`, `C7I/L7I` | `BDS_B1I` 1,984; `BDS_B2I` 843 |
| `base.obs` | 3.02 | `C1I/L1I`, `C7I/L7I` | `BDS_B1I` 11,790; `BDS_B2I` 4,904 (4,412 with phase, 492 pseudorange only) |

Rover and base BeiDou both map to `BDS_B1I` and `BDS_B2I`; no `BDS_B1C`
observation is produced. As for Deep and Harsh, this is a format-level result;
what the estimator does with BeiDou was not run and is not claimed. The rover
file holds GPS L1/L2, GLONASS L1/L2 and BeiDou B1I/B2I only; the base also holds
Galileo and GPS L5, which the rover cannot pair.

## Implementation checks on development data

Not part of any gate; the thresholds were fixed above before they ran and were not
chosen from them. No Hong Kong data.

- **The candidate reproduces the development numbers on tokyo1** (PPC, development
  data), freeze binary, normal scenario, full run (11,928 epochs, match fraction 1):
  fused-position RMSE 40.67 m (`none`) -> 12.03 m (candidate), P95 89.9 -> 22.2 m;
  fused-velocity RMSE 5.03 -> 3.61 m/s; RTK-position RMSE 28.4 -> 27.0 m; rotation
  RMSE 104.85 -> 105.55 deg. The `none` replay of the freeze binary is
  bit-identical to the `none` replay of the `adb40608` binary on this run
  (gate 7), and its `replay.json` for the candidate is
  `"candidate":"independent_doppler_v1", ... ,"independent_doppler_velocity":true`
  and nothing else added.
- **The `holdout_v4` gate set, end to end, on development data.** `gnss pva-evaluate`
  for `none` and `independent_doppler_v1`, the three scenarios, full runs, with the
  freeze binary, on PPC **tokyo1** and **nagoya2** (12 replays), and the frozen
  comparator `--gate-set holdout_v4 --runs tokyo1 nagoya2` (the contract path was a
  draft of this document; decision file SHA256
  `c52ab4a0aace6e2f25b590bf6ffb2a7839b9f4223d9123da4cb595c19468181a`, 44,406 bytes,
  not stored in the repository). It accepted the candidate name and the 12 replays
  (H7 passed), gave 42 gates per run and **a No-Go: 9 of 84 gates fail**:

  | Run | Gates passed | Failed gates |
  |---|---:|---|
  | tokyo1 | 38/42 | `H1` fused rotation RMSE and P95, both cohorts (4 gates): pooled RMSE 104.72 -> 105.46 deg (ratio 1.007), P95 171.20 -> 171.41 deg |
  | nagoya2 | 37/42 | `H1` fused position RMSE and P95, both cohorts (4 gates): RMSE 17.37 -> 19.08 m (ratio 1.098), P95 ratio 1.174; `H5` `normal.initial.first_heading_s` 41.8 -> 43.4 s (+1.6 s) |

  The worst ratios of the other gates, over both runs: H2 1.013 (limit 1.10), H3
  1.060 (1.25), H4 coverage loss 0.0008 (0.005), H6 1.15 (2.0); H8r: the largest
  excess of the candidate fraction over the control's is 0.0078 (limit 0.01) on
  tokyo1 (control and candidate fractions about 0.51 in all three scenarios).
  Fused-position pooled RMSE on tokyo1: 52.04 -> 12.46 m, ratio 0.24 (H1 passes).

  **What this shows, stated plainly.** (1) The comparator and the candidate run
  end to end. (2) With the frozen thresholds, a candidate that improves tokyo1's
  position error four-fold fails H1 on the rotation gates because its rotation
  error differs from the control's by +0.7 %, and the nagoya2 run, on which position
  is flat, fails H1 on position and one timing gate. The inherited H1 (ratio 1.00 on
  rotation, RMSE and P95) is **close to a coin flip for a candidate that does not act
  on attitude**, and H1 on position is not tolerant of a flat run. These thresholds
  are the holdout v2 thresholds that were fixed before any holdout data and this
  check; **they were not chosen from this check and are not changed because of
  it.** It is recorded so that a No-Go on Medium-Urban-1 caused by those gates alone
  can be read for what it is, and so that the maintainer who orders this contract
  knows the expected outcome on development data before the run.
- **The default, v2 and v3 comparator outputs are unchanged.** On synthetic
  three-scenario trees (candidate shifted so that gates both pass and fail: 28 and 31
  failing gates for `default` without and with `--attitude-integrity`, 9 for
  `holdout_v2`, 12 for `holdout_v3`) the decision file of the comparator of this
  tree equals that of the develop `adb40608` comparator (`git show
  adb40608:scripts/analysis/compare_online_pva.py`) in all four cases, apart from
  the pin of the comparator file itself.

## Frozen files

| File | SHA256 |
|---|---|
| `scripts/analysis/compare_online_pva.py` | `fa8376d22ec73d131e8485b2df8baf9a2009e0314b299272ef505330e8de0adc` |
| `scripts/convert_urbannav_hk_to_ppc_layout.py` (v3 plus the run-label whitelist only; separate commit) | `a943d9539fd45fe4df208609397d8e45406f148bc2cec8c3fa15dc82881c68db` |
| `scripts/analysis/check_urbannav_hk_frames.py` (same as v2/v3) | `b5af3d656197fbdc40045ec06ff66ca05b4691053bf639138b106284e94df9c4` |
| `scripts/convert_urbannav_to_ppc_layout.py` (shared helpers, unchanged) | `70b81a91205a18b8d71405b42a0270f331cdcf595eaa95b3fb2b25288850bbed` |
| `apps/commands/benchmarks/gnss_pva_metrics.py` (scorer, unchanged) | `2d767c3d6e612e83ad61dd5f4f01d524a83470b73577a1b8817cd53076fa2957` |
| `apps/commands/benchmarks/gnss_pva_evaluate.py` | `9b188cfa18c8d3010832b50f3ff8fcc5c6b9c7c1c904f79096261b21433374cf` |
| `apps/native/gnss_pva_replay.cpp` | `8ca629af6feca104ad4e603007dc9323f796e3220fa3f4e86c6f93906c1cbae4` |
| `src/fusion/online_rtk_imu.cpp` (implements the option, unchanged from develop `adb40608`) | `2f43baec1c0a0e01efb82dc78d6b9f24a9b761f5f278d37a51e46207284677c7` |
| `include/libgnss++/fusion/online_rtk_imu.hpp` (unchanged from develop `adb40608`) | `5fe512b02806e286307c9dd793d1d847e9a4860988505e782d50d52d6db5af1b` |
| `tests/test_pva.py` | `a02305e10cc490183dddafda5f09713eeb9e0eb566fe3e9fc94c90d2979553ff` |
| `tests/test_pva_comparison.py` | `387af9bbcb0982c5e120dd3058957b0ee374d4129aa4c343b53da52b8f57ca19` |
| `tests/test_urbannav_hk_conversion.py` | `44703dca3af23ed16a95d4d7317abf73a9784d58bc65b639feb8c83c94d7ab0a` |

Replay binary (`gnss_pva_replay`, Release, built from this tree, 2,496,168 B):
`a6c0d7fba29731a643b58aadee686311ce0cfd6ac0e1f6cc1c3f164a30f747d0`. It is the binary of the candidate replays, of the control replays and of
the smoke above. The tests at the freeze: `tests/test_pva.py` 12 tests (3 for this candidate), `tests/test_pva_comparison.py` 58 (10 for `holdout_v4`; the earlier 48 unchanged except that one assertion on the length of `GATE_SETS` became a prefix check), `tests/test_urbannav_hk_conversion.py` 28 (1 new), `tests/test_urbannav_conversion.py` 17, and the C++ `gnss_online_tests` (`OnlineRtkImuTest`) 37, all passing. The mkdocs strict-build test cannot run in this environment (no `mkdocs`) and is unrelated.

## Data fingerprints

Raw inputs and sources (downloaded again for this contract; the survey's SHA256 are
equal):

| File | Source | Bytes | SHA256 |
|---|---|---:|---|
| `UrbanNav-HK-Medium-Urban-1.novatel.flexpak6.obs` (in `gnss.zip`) | Dropbox GNSS folder of the dataset README | 548,442 | `e99e563b018094a4f48cf6dd2f5b77441f57df3e593809532abe6252b38c04d4` |
| `gnss.zip` (the whole folder download) | `dropbox.com/sh/2haoy68xekg95zl/...?dl=1` | 22,526,745 | `b265d9983533b2c038ba2ec46c90a4c078bef5119fef128e6b843af3a1bc5fe2` |
| `UrbanNav_TST_GT_raw.txt` (truth; the survey saved it as `gt_med.txt`) | `dropbox.com/s/twsvwftucoytfpc/` | 140,442 | `9d48bb497878aafbd17290789560394c72ecafec20c9e0eaff448418695d92bf` |
| `xsense_imu_medium_urban1.csv` (IMU; the survey saved it as `imu_med.csv`) | `dropbox.com/s/2rh1rs15ihpf63u/` | 109,736,693 | `6a3e0ab0209e032296dc750a491e27a4919473c875c5ec8f2dcd1bd3d796537d` |
| `HKSC00HKG_R_20211370200_01H_01S_MO.crx.gz` (the survey's `HKSC_137_02.crx.gz`) | `rinex.geodetic.gov.hk/rinex3/2021/137/HKSC/1s/` | 2,322,421 | `11348ef61864d74b995b4a235a5459bc9da5a0e0567343a2e7e63245328604da` |
| `HKSC00HKG_R_20211370000_01D_GN.rnx.gz` | `rinex.geodetic.gov.hk/rinex3/2021/137/HKSC/` | 29,451 | `e2d2b66142d08def1a683fb13a869cbf0c44dd84f956c7218758753a50151eb1` |
| `HKSC00HKG_R_20211370000_01D_RN.rnx.gz` | same | 29,111 | `4e2ed74803ffd23f1dfc6385f8bbe4e0eeb7b41c4400f70bc2831c6b4d8bee37` |
| `HKSC00HKG_R_20211370000_01D_EN.rnx.gz` | same | 123,183 | `807fe0f06391e9e6ffe6f8162efcd7aa6cf8869e65c3f3ce96f38329ca5c365b` |
| `HKSC00HKG_R_20211370000_01D_CN.rnx.gz` | same | 62,717 | `a78a404fe0036ba6264de8b9951e2c13a836e8a739aef0218a5235b720f8a024` |

The decompressed hourly base file (Compact RINEX expanded with the `hatanaka`
package) has SHA256 `63551020f806e1f9cbbb1b4ae8af184a5fa115810a9fab221c2497739c327bc8`
(25,674,015 B). The truth file is the one named for Medium-Urban-1 in the dataset
README (`UrbanNav_TST_GT_raw.txt`); its time span (95593 - 96379 s) matches the
rover file (`TIME OF FIRST/LAST OBS` 02:33:13 - 02:46:19 GPST on 2021-05-17), and the
file is pinned by SHA256.

Converted files, from the committed converter:

| Directory | File | Bytes | SHA256 |
|---|---|---:|---|
| `HKMediumUrban1_novatel` | `rover.obs` | 548,251 | `6b000bc7391b5495b2354fc00fdb49b0d3a1466e980e867fe928c56d189f4dc5` |
| | `base.obs` | 5,578,920 | `a7ca93213af4b938e57b1093e2c5930caf9ef47c519e9db97b1686084ced98b1` |
| | `base.nav` | 1,579,910 | `8c1f40f0a220ca76f9d969ce7f0255549e7c2d2307932b5065ed489c54b6eee3` |
| | `imu.csv` | 7,309,199 | `d019dea03b9501b97ee1645da508629573ceaf284beecc54bfb70189c8ed3576` |
| | `reference.csv` | 112,760 | `5fa03d8ec92102f7a5eb895516ed24592f89cc65f9eb0654731f62c31d968a21` |
| | manifest | 5,581 | `1f51320449025f9d252d390f88330c880c0941f92d0f382d1a409990e3f909cf` |

The converted `imu.csv` has 78,548 rows. The manifest records the converter SHA256
(equal to the table of [Frozen files](#frozen-files)).

## Known limitations of the data (stated before the run, not corrected)

These are untested by construction, since no scored estimator has been run, and they
affect control and candidate alike.

- **Population:** one run of 704 s, one city, one platform. The platform is the one of
  the development runs HK Deep and Harsh.
- **Thin GNSS:** median 4 satellites per rover epoch; the RTK solution will often be
  unavailable or float. The scenario windows hold real observations (above), so the
  GNSS-outage scenario removes little. Gates on RTK quantities (H2, H4) rest on
  few epochs.
- **Lever arm and boresight:** declared zero. The truth reference point is not
  documented. The truth heading differs from the course by about -1.1 deg.
- **Datum:** the truth frame and the HKSC header position may differ by decimetres.
  The truth ECEF is computed as WGS84.
- **BeiDou:** the rover labels B1I as `2I` (RINEX 3.03) and the base as `1I` (RINEX
  3.02); the version-aware reader maps both to B1I (format check above). Observation
  codes are not rewritten, and the use of BeiDou by the estimators is not examined.
- **GLONASS channels:** the rover header lists one `GLONASS SLOT / FRQ #` entry (R18);
  the base header lists all 24.
- **No Galileo at the rover.**
- **IMU resampling:** 400 Hz to 100 Hz by point-sampled linear interpolation.
- **Truth quality:** variable (Q 1 to 3). The truth is post-processed SPAN-CPT + IE.
- **Binary versus earlier records:** the binary of this contract contains develop's RTK
  commits `3835e9e0` and `f48d5f37`, which are after the v3 freeze (see the
  [development evidence](#development-evidence-development-data-only)). The v3
  control results are not comparable.
- **H1 rotation gates** are strict on a quantity the candidate is not designed to
  change (above).
- **Scorer:** the manifest of `gnss pva-evaluate` carries the fixed text
  `dataset_role: development/regression; no heldout claim`; it is the tool's constant
  and has no meaning for this contract.

## Reported in addition (not gates)

- Absolute errors by scenario, run and cohort, with the warning that the zero lever
  arm, the unknown truth reference point, the datum and the boresight bias them by up
  to about 1 m. The pooled statistics of H1-H4 are in the decision file for both arms.
- The H8r control and candidate fractions per scenario, and the number of scored
  epochs behind each.
- Errors split by the truth quality Q.
- The IMU-versus-truth time offset (about +0.01 s) and the heading-versus-course
  offset (about -1.1 deg). Neither is corrected.
- Per-run behaviour of the candidate: `replay.json` counters, rover-gap RTK-only
  resets (the 11 rover gaps over 2 s), RTK availability by scenario.
- Per-scenario values of every H1-H3 statistic, for comparison with the pooled ones.
- Processor P95 ratios and the host state.
