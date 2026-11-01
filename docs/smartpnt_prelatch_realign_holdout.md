# Frozen holdout: re-aligning the output filter over the stand-still before the heading latch

This contract is committed before any estimator of this repository runs on its
data. It decides, once and by the rule below, whether a PR may make the
re-alignment the behaviour of the loose ESKF's consider update before the
heading latch:

- the independent output filter that writes `--out` under `--navi776-tc`;
- `--consider-before-heading-latch`.

The decision is recorded whatever it is.

## The problem

Found on development data; see the
[Data15 note](smartpnt_reverse_start_holdout_results.md#later-a-conversion-error-behind-data13).

- **The 2 s alignment window.** The loose ESKF initializes from the first
  2 s of IMU data (`align_static_window_s`). It takes the gyro bias as their
  mean and the tilt from their mean specific force.
- **A transient in that window.** Some IMUs start with a transient. Data15
  (FSAS) has a 2 s gyro mean of (−0.038, +0.079, −0.002) °/s, against
  (−0.003, 0.000, 0.003) °/s over its 539 s stand-still.
- **Nothing corrects it before the latch.** The consider update keeps the
  attitude and both biases out of every measurement update until the heading
  latches. So the bias error integrates into the tilt for the whole
  stand-still: 46° of pitch on Data15 by the first motion, and `--out`
  hundreds of kilometres off.
- **Size on the 20 development runs.** The tilt rate from the 2 s window,
  against the mean over the stand-still before the first motion (truth speed
  above 0.3 m/s), multiplied by that stand-still:
  - Data15: 0.086 °/s over 539 s;
  - UrbanNav HK Harsh-Urban-1: 0.060 °/s over 453 s;
  - all others: 0.0046 °/s or less.

## Candidate and control

All arms share `--data-dir <run> --lever-arm <run lever arm>
--preset low-cost --ratio 2.4 --max-subset-ar-drop-steps 18
--rtk-snr-weighting --no-arfilter --navi776-tc`.

- **Control:** the shared flags.
- **Candidate:** the shared flags, run with the environment variable
  `GNSS_EXP_PRELATCH_REALIGN=1`.
- **What the candidate does.** It acts only while the consider update before
  the latch is in force, and only in that filter.
  - **Stand-still samples.** While the stationarity test of the ZUPT holds,
    it adds each raw IMU sample to sums. Any non-stationary sample clears
    them.
  - **Re-alignment.** Once the sums cover at least 2 s, and then every 1 s,
    the filter takes three values from their means: the gyro bias, the tilt
    and the accel bias, the same quantities the static alignment takes. The
    yaw is kept.
  - **What is left alone.** The covariance and the error state are not
    touched. After the latch, nothing changes.
- **Binary.** The candidate is an experiment build, not merged code.
  - **Source:** `gnss_fuse` built from `cd77ecf` (`develop`) with
    [`smartpnt_prelatch_realign_experiment.patch`](smartpnt_prelatch_realign_experiment.patch)
    (SHA256 `a7ded57ee0ad8647a844f36e27a3304e20a4338d5e68a1eaed89626967910576`).
  - **Build:** Release, GTSAM 4.3 found, Python bindings off, conda GCC 15.3.
  - **SHA256 of the binary used:**
    `d224023387848d0d78762c98eead229227828419f6993df7b5133d95ba28d3c9`.
  - **Without the variable** its output is byte-identical to a `develop`
    build. That was checked for the fused, RTK and attitude streams on full
    runs of PPC tokyo/run1 and nagoya/run1, SmartPNT Data15 and UrbanNav HK
    Harsh-Urban-1.
- **One run per arm**, with the README lever arm of the run. No setting is
  tuned on these runs.

## Development evidence

Fused `--out` 3D p50 / p90 (m), control → candidate, one run each. The
candidate was written after seeing Data15, and the rule below after seeing
this table, so these 20 runs are not evidence for the decision.

| Run | Control | Candidate | Run outcome |
| --- | ---: | ---: | --- |
| SmartPNT Data15 | 776,534 / 26,124,910 | 1.00 / 22.5 | better |
| SmartPNT Data16 | 0.191 / 76.4 | 0.175 / 76.4 | better |
| PPC nagoya/run1 | 1.458 / 52.6 | 1.464 / 50.8 | better |
| PPC tokyo/run1, run2, run3, nagoya/run2, run3; UrbanNav Odaiba; SmartPNT Data02, 11, 12, 13, 19, 20 | — | — | equal |
| UrbanNav HK Deep-Urban-1 | 28.1 / 126 | 46.6 / 133 | unusable |
| UrbanNav HK Harsh-Urban-1 | 27.2 / 252 | 78.7 / 369 | unusable |
| UrbanNav HK Medium-Urban-1 | 908 / 25,565 | 46.3 / 258 | unusable |
| SmartPNT Data01 | 545 / 186,987 | 179 / 2,241 | unusable |
| SmartPNT Data14 | 405 / 27,223 | 14,596 / 477,702 | unusable |

- **RTK streams:** byte-identical on all 20 runs.
- **The unusable runs** are unusable for other reasons, and both arms are
  unusable there:
  - HK: the RTK stream fixes few epochs, and most of those fixes are wrong.
  - Data01: no Doppler, so the heading never latches.
  - Data14: a reverse start that latches 180° off.
- **Worse on three of the five.** The candidate is worse on HK Deep, HK
  Harsh and Data14. The rule does not count them, but it is recorded here.

## Holdout data

SmartPNT-POS (Kaggle `fengzhusgg/smartpnt-pos`), the five vehicle runs that
this repository has not run an estimator on:

- **Rover:** `ROVE`.
- **IMU:** the one the run folder is named after.
- **Truth:** the Inertial Explorer export `ROVE_GroundTruth.txt`, at the
  rover antenna.

| Run | IMU (rate) | GPS week, TOW kept | Kept rover epochs | Baseline (km) | Lever arm FLU (m) | Stand-still before the first motion |
| --- | --- | --- | ---: | --- | --- | ---: |
| Data03_20150722 (Vehicle, open sky) | FSAS (200 Hz) | 1854, 271591–281181 | 9,591 of 9,593 | 0.06–2.4 | `0.573,0.016,0.010` | 474 s |
| Data04_20220315 (Vehicle) | FSAS (200 Hz) | 2201, 178060–184677 | 6,496 of 6,500 | 3.42–6.5 | `-0.282,-0.500,0.021` | 1,524 s |
| Data05_20201128 (Vehicle, open sky) | HG4930 (100 Hz) | 2133, 525600–527541 | 1,942 of 9,733 (5 Hz rover, 1 Hz truth) | 0.12–2.4 | `-0.005,-0.044,0.182` | 370 s |
| Data17_20200617 (CAR) | HG4930 (100 Hz) | 2110, 262837–268365 | 5,523 of 27,615 (5 Hz rover, 1 Hz truth) | 0.01–5.0 | `0.203,-0.105,0.441` | 377 s |
| Data18_20200618 (CAR) | HG4930 (100 Hz) | 2110, 365737–375368 | 9,620 of 48,150 (5 Hz rover, 1 Hz truth) | 4.46–7.8 | `0.203,-0.105,0.441` | 377 s |

- **All unused vehicle runs are in.**
  - **Not chosen by stand-still.** Every unused vehicle run with a base, an
    IMU and a truth file is included, whatever its stand-still.
  - **Excluded:** the UAV runs (Data06–09, 21–30) and the marine run
    (Data10). The loose ESKF's vehicle constraints (NHC, the course latch)
    do not apply to them.
- **Conversion:** `scripts/convert_smartpnt_pos_to_ppc_layout.py` of this
  tree (SHA256
  `9f9fa36ce2b892d6719a46863a3eab2a01d21c9e52f1e2fdf7343ef02fed4566`), with
  the shared helpers of `scripts/convert_urbannav_hk_to_ppc_layout.py`
  (`ab4e346103d82ca917cb43941a972563f949282569a2b94d77c5077e52bb6be3`). It
  applies the rules of the earlier holdouts, plus two that this contract adds
  before any estimator run:
  - **Repeated rover epochs are dropped.** A rover epoch whose time is not
    after the previous one is dropped with its records, and the manifest
    counts it:
    - Data04: 122 dropped;
    - Data18: 62 dropped.

    These are stale or empty records under an already used time. In Data04
    they are a 5-satellite epoch whose ranges differ from the real one by
    about 311 m. In Data18 they are 1-satellite records of zeros. This is
    the reason the [reverse-start holdout](smartpnt_reverse_start_holdout.md)
    excluded Data04 before converting it.
  - **Data03's truth point.** Data03's truth is exported 3.2 cm from the
    antenna: the export lever arm is (+0.016, 0.573, 0.010) m, against
    (−0.016, 0.573, 0.010) m to the antenna. It is accepted with a 5 cm
    tolerance and recorded in the manifest, not corrected. Both arms are
    scored against the same reference.
- **What was looked at.** Only these things; no estimator was run:
  - the truth files (the stand-still, the class of the start);
  - the README files;
  - the RINEX headers and the repeated rover epochs above;
  - what the conversion prints.

  The IMU axis check passes on all five: the gyro z correlates with the
  truth heading rate at 0.996–0.998.

Raw and converted files:

| File | Bytes | SHA256 |
| --- | ---: | --- |
| Data03 `README.xml` | 760 | `7e08c05503452bb9fbe0009fee14d940734f651104f3507eaea1c9d105baf17e` |
| Data03 `ROVE.15O` | 19,162,181 | `4ee5a756ce18e660fc3203a3a27e2c825de18a57695e34e1b5550db5e52bd276` |
| Data03 `BASE.15O` | 16,146,247 | `83c1e39fa7edca0d8f7d73340d3f2691b0d2ef5f104f2c446ef66c4c3508c6bc` |
| Data03 `brdm2030.15p` | 1,571,294 | `eb15e34b17b1d3389b8943c2bed79413d4fc2bdb67b5cc9cfc71fa63864e9211` |
| Data03 `FSAS.imr` | 77,690,112 | `8552a81c3f2b46b76342c1388ac885f1658cbaa434799803d76c276f638927b3` |
| Data03 `ROVE_GroundTruth.txt` | 6,159,796 | `b2342997ccb6aee281ebe60e7f079785dab4cadf3259446211fcdd8287bf3853` |
| Data03 converted `rover.obs` | 19,157,853 | `5327cf11ea60e92583351e6b38420e0ae29b1a56cbbd285a8bcf8f0abf76efdd` |
| Data03 converted `base.obs` | 12,339,548 | `55f67bce2b0afe45837e094b2c4ddedc920c5a4cb9d7223eeabea978a691fad8` |
| Data03 converted `base.nav` | 1,571,294 | `eb15e34b17b1d3389b8943c2bed79413d4fc2bdb67b5cc9cfc71fa63864e9211` |
| Data03 converted `imu.csv` | 114,120,682 | `4df3ccf31d30729e5dad778207bc6c20d99f89b82f15f263cf627b3109b46685` |
| Data03 converted `reference.csv` | 1,601,283 | `8e221f06896111890f0cf27e003429db5321d8efef9fa05e8cfd29de5dfabcfb` |
| Data04 `README.xml` | 854 | `abc5930d6385eb511da521e5da606c16de4d1aac62cbd284c9ec9df0156fd390` |
| Data04 `ROVE.22O` | 31,715,074 | `2a424af5f25aab5aa5e76219033c9f04dc4f5365cbd26d904e49e899fce6d327` |
| Data04 `BASE.22o` | 105,161,107 | `e5d4b157840ae0cf51a842af4f839aec3b2a8e45e9aec1be8f7ae23d6160b094` |
| Data04 `brdm0740.22p` | 7,220,323 | `3a4e2e060ae9be5a7ccfe5f7d9ce3540243dcaf62938c93c158d057c746f9351` |
| Data04 `FSAS.imr` | 42,380,960 | `45f90c11226272af237b2e40b83863314515a54453b3c8e9e5e4b2b4d27e52c0` |
| Data04 `ROVE_GroundTruth.txt` | 4,310,820 | `0adc1957d852f2844388f4dd72d034c46f9b54cda7a39d77e99d19427eee28a1` |
| Data04 converted `rover.obs` | 31,654,300 | `64eb1bc68c52b76178dffe92715abeafad20b1fcb3850431e1bbb0b5b51c757f` |
| Data04 converted `base.obs` | 97,006,594 | `e2b61ca6a656740bce5c56909b34c9fd83b148f89309db96217d071091893a34` |
| Data04 converted `base.nav` | 7,220,323 | `3a4e2e060ae9be5a7ccfe5f7d9ce3540243dcaf62938c93c158d057c746f9351` |
| Data04 converted `imu.csv` | 62,246,763 | `288029e8c0ced0d42a499d8a2cf952aae9ab898cd30b87fa8455f68ac79d015a` |
| Data04 converted `reference.csv` | 1,029,391 | `ef70c715489bdf85d0a20a38fe2368e0f364d88d3ad6e4b0b726fadd22ef2dca` |
| Data05 `README.xml` | 818 | `9a6a6035cb49b102c2e26f8c0502791510214441c8f759660083d3fb815d4c38` |
| Data05 `ROVE.20O` | 50,533,658 | `715175fc8f3e143d87c27eb26fc43889018fabe3a483d27c76529943457dc643` |
| Data05 `BASE.20O` | 28,897,434 | `cb0a5d167127a776100149b838b01e26cb0ed789b31927b97c4c3462e87c9266` |
| Data05 `brdm3330.20p` | 6,530,656 | `745ccccec55d6478394793e718f74ff4135e5235a34ddc92bdabb9bf50d15873` |
| Data05 `HG4930.imr` | 6,225,920 | `970f37438b1de4e36c86148ebbb342f1a6a35e0d1dbf83e55803651ac1d9f9af` |
| Data05 `ROVE_GroundTruth.txt` | 1,266,720 | `d54994e513a522d186208548a656340950810be318851209ae714bc7af9ebda2` |
| Data05 converted `rover.obs` | 10,085,516 | `478bf2d1c15058ed504ed76a449df7882f1fb7cda691008ffed8b5eb5b44ce2f` |
| Data05 converted `base.obs` | 6,382,794 | `144a0877e53d7ac585a9907e690914acf132c0a64212eb7a96274fb136ae990b` |
| Data05 converted `base.nav` | 6,530,656 | `745ccccec55d6478394793e718f74ff4135e5235a34ddc92bdabb9bf50d15873` |
| Data05 converted `imu.csv` | 18,292,373 | `6a3fb69585b20b92eeaba57eec04ade4b25a43c75d5a4bc2a3483a72e0245c0b` |
| Data05 converted `reference.csv` | 305,772 | `45e574cdc5599fa2ac6eb540c9d0c882ecd6c04fda174226b64a9c74b2b76d08` |
| Data17 `README.xml` | 832 | `92829c9b116978251c8ceaea251b7a1b78e5ff7f710ab032014225fc6342cc62` |
| Data17 `ROVE.20O` | 78,791,690 | `181cffd9ebbd4b613f3a9efb070edde26cf0c35ea3e9c08e1136a1c5f6ce1fd6` |
| Data17 `BASE.20o` | 422,099,602 | `ac3d43f9366ce962651144253baf5a14875110670cf9a0a1673a18e629ffbd7b` |
| Data17 `brdm1690.20p` | 6,809,746 | `90b6a4c33142f98beb7684a2434a79382d0e22dc3be16a6e0d96098b5b645a0a` |
| Data17 `HG4930.imr` | 17,694,720 | `75740a91ee10c47e4c38caa6af0839ae6e593b30bc7cf9a390cb9e56ca665379` |
| Data17 `ROVE_GroundTruth.txt` | 3,623,949 | `871bf3f908acf4ccd740f17f0cff59693a79f3b3450f89d2b86e06fbfaa54ffd` |
| Data17 converted `rover.obs` | 15,764,318 | `ace8f122902f3c35fd30b6583764cc00f5563f0a38721d2ddb060e45afb97cd3` |
| Data17 converted `base.obs` | 88,240,070 | `51ce57278de0203858cf819672283143f2f734372c541d90f7bc93207495bbe5` |
| Data17 converted `base.nav` | 6,809,746 | `90b6a4c33142f98beb7684a2434a79382d0e22dc3be16a6e0d96098b5b645a0a` |
| Data17 converted `imu.csv` | 51,996,360 | `7a1b6822ffd62b88712664688ce127ec0725025acad52287663b4dd57c007274` |
| Data17 converted `reference.csv` | 920,932 | `f563080d9356a503500b94261ff41dda646ef95770d1f2e07c212d6e9e004a8c` |
| Data18 `README.xml` | 841 | `114eeab900ca54f651d1a4c987ef31713391c89bbe2222042efaa702a11a68b2` |
| Data18 `ROVE.20O` | 179,992,031 | `c8459d3e88980a3757cd1cedf0a93716b6842669684d4f1f1e6d50c1cd3761dd` |
| Data18 `BASE.20o` | 446,490,519 | `759f57250e143ed5f1748e7d70443c54505931714948081fc1d6d7342cf484bc` |
| Data18 `brdm1700.20p` | 7,006,089 | `b996a75f928ce9d7077e5f4d71aa79bfc5839ffa81aad346e476dacf5566fbcb` |
| Data18 `HG4930.imr` | 30,834,688 | `713bde8ee73eaba79c97d2b8bf89cffa462cdbfcedfe70ca35f26fc628a44322` |
| Data18 `ROVE_GroundTruth.txt` | 6,311,415 | `ad689e4ff95ae02778477d3f90d3a27cecc39d9e9f4c3cd702b9d957b07ac1af` |
| Data18 converted `rover.obs` | 35,941,412 | `8700d7532728aa081d2f0d22489eb0dcd2791f188945806399def0c5a3ac6e6c` |
| Data18 converted `base.obs` | 153,219,664 | `a93f3b8e75ee03dc1e358177be1e426994388fe957b7c501c431b55118f8bf02` |
| Data18 converted `base.nav` | 7,006,089 | `b996a75f928ce9d7077e5f4d71aa79bfc5839ffa81aad346e476dacf5566fbcb` |
| Data18 converted `imu.csv` | 90,599,447 | `67154770591ff5113c14ecf6cc380793830551ba58a7a1585dc82b84345b6f5e` |
| Data18 converted `reference.csv` | 1,603,594 | `ece068381b5473d82843a9b9a051b9359a50c6a27372df734a2f6e6cb5657dfb` |

## Scoring and rule

Scoring is `scripts/analysis/score_fused_output.py` (SHA256
`422ff68ecb50c1624ffaef365c7a66662ccbd6dd5b312372cc59d2e94df9d3a3`), as in
the [fused-output holdout](smartpnt_fuse_output_holdout.md):

- the fused `--out` against `reference.csv`;
- 0.25 s matching;
- the first 60 matched epochs skipped;
- 3D error p50 and p90.

**Run outcome**, from `run_outcome`, candidate against control:

- **unusable** when both arms have a p50 above 10 m;
- **worse** when the candidate matches fewer than 99% of the control's
  epochs, or either quantile is above the control's ±1% band;
- **better** when at least one quantile is below the band and none is above;
- **equal** otherwise.

**Decision.**

- **Fail** if any run is worse, or if any run's RTK stream (`--rtk-pos-out`)
  is not byte-identical between the arms.
- **Pass** if at least one run is better and none fails.
- **Inconclusive** otherwise. That includes all runs equal or unusable, and
  it is not a pass.

**What a pass allows.** A PR may make the re-alignment part of the consider
update before the latch, without a new option, with tests, this record and
the development table. Merging it remains the maintainer's decision. A fail
or an inconclusive result leaves the code as it is.

## Commands

For each run, with `$D` the converted run directory and `$L` its lever arm:

```bash
COMMON="--data-dir $D --lever-arm $L --preset low-cost --ratio 2.4 \
  --max-subset-ar-drop-steps 18 --rtk-snr-weighting --no-arfilter --navi776-tc"
gnss_fuse $COMMON --out control_fused.pos --rtk-pos-out control_rtk.pos
GNSS_EXP_PRELATCH_REALIGN=1 gnss_fuse $COMMON \
  --out candidate_fused.pos --rtk-pos-out candidate_rtk.pos
```

Ten runs, each once. The results go to
`smartpnt_prelatch_realign_holdout_results.md` whatever they are. These five
runs are development data afterwards. If a run does not complete, it is
reported as such; it is not repeated or replaced.
