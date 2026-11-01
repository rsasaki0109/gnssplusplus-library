# Frozen holdout: the fused `--out` of `gnss fuse` on SmartPNT-POS

This contract is committed before any estimator of this repository runs on its
data. It decides two things, each once, by the rule below, and the decisions
are recorded whatever they are:

- **(a)** whether a PR may propose that `gnss fuse --navi776-tc` turns on
  `--independent-output-filter`;
- **(b)** whether a PR may propose making `--consider-before-heading-latch`
  the default of `gnss fuse` without tight coupling (no `--navi776-tc`, no
  `--tc-closed-loop`).

## Candidates and controls

All four arms share the RTK flags of the
[imu_fusion.md tables](imu_fusion.md#the-loose-eskf-before-the-heading-latch),
`--preset low-cost --ratio 2.4 --max-subset-ar-drop-steps 18
--rtk-snr-weighting --no-arfilter`, and the lever arm below.

| | Control | Candidate |
| --- | --- | --- |
| (a) | the shared flags plus `--navi776-tc` | the control plus `--independent-output-filter` |
| (b) | the shared flags | the control plus `--consider-before-heading-latch` |

- **Binary:** `gnss_fuse` built from `7a94af1` (the head of the PR that adds
  `--independent-output-filter`): Release, GTSAM 4.3 found, Python bindings
  off. SHA256 of the binary used:
  `4d30dbd76467b7b72079d7e0b2d6a5b67061292eab60f457d2ad898c164d2ffc`.
- **One run per arm** with the README lever arm, as in the development
  tables. No IMU noise, lever arm or other setting is tuned on these runs.

## Development evidence

All 11 runs are development data for both candidates: PPC and UrbanNav Tokyo
were used to find the mechanisms, and the tables were measured after that
([imu_fusion.md](imu_fusion.md#independent-output-filter)). The values below
are those runs rescored with `scripts/analysis/score_fused_output.py`; they
agree with the published tables. Fused `--out` 3D error p50 / p90 (m),
control → candidate. The Hong Kong error is to the raw truth, about 0.6 m
from the antenna.

| Run | (a) | (a) rule | (b) | (b) rule |
| --- | --- | --- | --- | --- |
| PPC tokyo/run1 | 6,156 / 81,696 → 0.124 / 12.0 | better | 2.43 / 60.3 → 0.123 / 9.42 | better |
| PPC tokyo/run2 | 154 / 3,447 → 0.088 / 2.58 | better | 0.117 / 1.641 → 0.105 / 1.641 | better (p90 equal) |
| PPC tokyo/run3 | 7,919 / 104,223 → 0.089 / 1.38 | better | 0.086 / 2.326 → 0.081 / 2.326 | better (p90 equal) |
| PPC nagoya/run1 | 1.56 / 84.5 → 1.46 / 52.6 | better | 2,044 / 13,274 → 1.60 / 72.5 | better |
| PPC nagoya/run2 | 34.4 / 330 → 0.488 / 19.0 | better | 0.631 / 15.38 → 0.633 / 15.22 | better (p50 equal) |
| PPC nagoya/run3 | 6.76 / 108 → 0.454 / 8.81 | better | 1.857 / 16.700 → 1.815 / 16.699 | better (p90 equal) |
| UrbanNav Odaiba | 30.3 / 135 → 0.284 / 8.72 | better | 0.393 / 17.785 → 0.273 / 17.785 | better (p90 equal) |
| UrbanNav Shinjuku | 40.6 / 548 → 0.389 / 14.6 | better | 0.531 / 19.5 → 0.321 / 15.5 | better |
| UrbanNav HK Deep-Urban-1 | 73,048 / 210,610 → 28.1 / 126 | unusable | 5.98 / 57.5 → 3.21 / 56.4 | better |
| UrbanNav HK Harsh-Urban-1 | 65.2 / 444 → 27.2 / 252 | unusable | 66.2 / 479 → 141 / 265 | unusable (p50 higher) |
| UrbanNav HK Medium-Urban-1 | 1,153 / 2,289 → 908 / 25,565 | unusable | 4,659 / 39,329 → 48.8 / 193 | unusable |

- The RTK stream is byte-identical between control and candidate on all 11
  runs, for both candidates.
- Both candidates pass the rule on the development runs. **The rule was
  written after these numbers were known** (the 1% band, the 10 m bound and
  the epoch-share guard included), so that pass is not evidence. Only the
  holdout result counts.

## Holdout data

SmartPNT-POS (Kaggle `fengzhusgg/smartpnt-pos`), two car runs in Wuhan (base at
30.53° N, 114.36° E):

- **Data11_20221230_ISA-100C_CAR_Complex** (2022-12-30);
- **Data12_20230104_ISA-100C_CAR_Complex** (2023-01-04).

- **Instruments:**
  - **Rover:** `ROVE_03`, a Septentrio receiver at 1 Hz.
  - **IMU:** the STIM300 at 200 Hz (IMR increments, GPS time). Its README
    mounting (180°, 0°, 90°) makes the body FRD, and the conversion writes
    FLU.
  - **Base:** one station with a TRM57971.00 antenna, 1 Hz. The baseline
    (truth to base) is 0.05–1.9 km on Data11 and 0.08–9.3 km on Data12, over
    1 km for 29% and 91% of the kept epochs. Beyond 1 km the `--navi776-tc`
    Doppler rows are gated off.
  - **Navigation:** the broadcast `brdm` file of the day.
- **Truth.** `Rove_03_GroundTruth.txt` is an Inertial Explorer smoothed
  tightly coupled GNSS/INS solution with the ISA-100C IMU (navigation grade).
  Its header gives the same IMU-to-antenna and IMU-to-sensor lever arms, so it
  is exported at the antenna. The README gives the ISA-100C the same lever
  arm to `ROVE_01` and to `ROVE_03`, which share that antenna. The
  scoring uses it as given, with no offset.
- **Not used before by any estimator.** Neither run, nor any other
  SmartPNT-POS run, has been used by this repository.
- **Conversion:** `scripts/convert_smartpnt_pos_to_ppc_layout.py` (SHA256
  `072ff735eef9d7c0e1aecde10b1affe45a411966ca5d86a29e2dd82386b8cc00`), its
  rules fixed here:
  - **Epochs.** A rover epoch is kept when it has an exact truth row and an
    exact 100 Hz IMU grid sample.
  - **Base.** The base file is trimmed to the kept span. Its
    `APPROX POSITION XYZ` is the README marker plus the 1.401 m antenna height
    along the ellipsoidal up (the L1 phase centre that the truth's master
    uses; README and master agree within 5 mm). `ANTENNA: DELTA H/E/N` is
    zeroed. The original header position is 1.35 m from that phase centre.
  - **IMU.** Each IMR increment is divided by its measured interval, placed
    at the interval midpoint, and resampled linearly onto the 100 Hz grid.
  - **Replay lever arm** (STIM300 to the antenna, README vehicle frame to
    FLU): `-0.282,0.500,-0.057`.

| Run | GPS week, TOW | Kept rover epochs | Dropped | Longest gap |
| --- | --- | ---: | --- | ---: |
| Data11 | 2242, 459712–468045 | 8,334 of 8,524 | 11 without a truth row, 179 before the IMU starts | 1 s |
| Data12 | 2243, 274800–295200 | 20,320 of 20,320 | none | 44 s |

- **Data11's rover file** has a blank line after its header. The converter
  drops it (`blank_lines_dropped: 1` in the manifest); the rover filter it
  shares with the UrbanNav Hong Kong converter still rejects such a line
  there.
- **What was looked at.** Only what the conversion prints,
  `gnss_rinex_info --count-records` on the converted files, and the distance
  of the truth from the base (the baseline above). No estimator was run, and
  no estimated position was compared with the truth.
  - **Every kept epoch** has a truth row, an IMU grid sample and an exact base
    epoch.
  - **Truth quality (Q 1/2/3/4):** Data11 2,596 / 5,364 / 319 / 55; Data12
    14,747 / 4,636 / 900 / 37.
  - **IMU axis check.** The converter correlates the FLU IMU, at 1 Hz and at
    speeds above 3 m/s, with three truth quantities: gyro z with the heading
    rate, accel x with the speed change, and accel y with the centripetal
    acceleration. Data11: 0.997, 0.676, 0.902 (5,815 s). Data12: 0.997, 0.959,
    0.956 (9,755 s). All three have the right sign on both runs.
  - **Parsing.** The library reads 8,334 and 20,320 rover epochs, the same
    numbers of base epochs (20,401 for Data12, which also has base epochs in
    the rover's gaps), and 128 and 130 broadcast ephemerides.

Raw inputs:

| File | Bytes | SHA256 |
| --- | ---: | --- |
| Data11 `README.xml` | 2,177 | `659de6f8545839aec87012af8e79b125022e97532d339d5746f563af49117163` |
| Data11 `ROVE_03.22O` | 70,158,914 | `60459857fcb2c7894629ddc01f2631dcafe8aa5f355ae922b5bf3376dbb6398e` |
| Data11 `Base.22O` | 174,206,646 | `01af9bcedd9a346bc4d3d2c81eb41c1ea3a0e06ccff9a0eee4245276c3230dfc` |
| Data11 `brdm3640.22p` | 6,920,928 | `54f0a5de9485d8efdae623f51b9bba84f643bb1dc6f46d345ea29e134057ffb4` |
| Data11 `STIM300.imr` | 53,334,432 | `9c52963c79596e63596d1f34ed70485118b615b9062efa683e41ee8b017e40ea` |
| Data11 `Rove_03_GroundTruth.txt` | 5,670,185 | `8f95b5c2bdcd7df213bbe1ef945f2b8f1a36cffa06758eff959e8782f43711d1` |
| Data12 `README.xml` | 2,132 | `3ea3a3d0a2773f71509e18146dc24ee78ce7d449411d03a4fa31c61f0be366a5` |
| Data12 `ROVE_03.23O` | 141,222,733 | `dde4efccdf8afc052cc0aa8f7c07c1feaa3a5cf866355dcddb1140d33577356c` |
| Data12 `BASE.23O` | 308,639,550 | `8236b6589831f757ddc6359a471f13983186ada070e5b721316bef9040f0ea93` |
| Data12 `brdm0040.23p` | 6,564,962 | `b87031fba489e1c4f8e21ef5da9972f11c1fe9ebea335b5e102326bd8a1563b4` |
| Data12 `STIM300.imr` | 130,874,752 | `94f086255d21d47ff1faa3a1dfd98289ae1606aa20c0079e2f8aef91d4f867c2` |
| Data12 `Rove_03_GroundTruth.txt` | 13,365,122 | `601e99d231a4474710de7445cd3bd6082c28b35ee825ab7a899902cd14b91346` |

Converted `smartpnt/<run>_rove03_stim300/`:

| File | Bytes | SHA256 |
| --- | ---: | --- |
| Data11 `rover.obs` | 68,211,605 | `3ad7307a5af03338a04ff79d2bf566f6b69ea6b87468b8aa534d7f7cb5d6f9cc` |
| Data11 `base.obs` | 100,722,164 | `9a7fde34bb85006244ab87c02e8f39be411c1401ed186f0ecaf46a82d821acdb` |
| Data11 `base.nav` | 6,920,928 | `54f0a5de9485d8efdae623f51b9bba84f643bb1dc6f46d345ea29e134057ffb4` |
| Data11 `imu.csv` | 78,357,548 | `1eee04ebcdc20fb37fb57ad50a7180c7eb9e5ec33349a0d1e3ed18a82ca4445e` |
| Data11 `reference.csv` | 1,395,714 | `757c0b42e8702e89c7dffdd2155589a54670edb73570ffab41a33274d464b60b` |
| Data12 `rover.obs` | 141,222,733 | `dde4efccdf8afc052cc0aa8f7c07c1feaa3a5cf866355dcddb1140d33577356c` |
| Data12 `base.obs` | 248,774,803 | `16710446f6aa139d8350ff75686fdb97257d180d4bc8d8f0352ee2b3bca27ba4` |
| Data12 `base.nav` | 6,564,962 | `b87031fba489e1c4f8e21ef5da9972f11c1fe9ebea335b5e102326bd8a1563b4` |
| Data12 `imu.csv` | 192,279,997 | `7b749e4f11104bc436235dc25a0ce5bc9dac332787b445dde0fc690ae0fec615` |
| Data12 `reference.csv` | 3,398,605 | `dffea695956fd17c383cc9c8d305923138660e85345c05d1a67ee5df0a8a90c6` |

## Replay and scoring (fixed here)

For each run `<run>` in `Data11_20221230` and `Data12_20230104`, with
`D=<root>/smartpnt/<run>_rove03_stim300` and `COMMON` the shared flags:

```sh
COMMON="--data-dir $D --lever-arm -0.282,0.500,-0.057 --preset low-cost --ratio 2.4 \
  --max-subset-ar-drop-steps 18 --rtk-snr-weighting --no-arfilter"
gnss_fuse $COMMON --navi776-tc --out a_control_fused.pos --rtk-pos-out a_control_rtk.pos
gnss_fuse $COMMON --navi776-tc --independent-output-filter \
  --out a_candidate_fused.pos --rtk-pos-out a_candidate_rtk.pos
gnss_fuse $COMMON --out b_control_fused.pos --rtk-pos-out b_control_rtk.pos
gnss_fuse $COMMON --consider-before-heading-latch \
  --out b_candidate_fused.pos --rtk-pos-out b_candidate_rtk.pos

cmp a_control_rtk.pos a_candidate_rtk.pos
cmp b_control_rtk.pos b_candidate_rtk.pos
python3 scripts/analysis/score_fused_output.py --reference $D/reference.csv \
  --control a_control_fused.pos --candidate a_candidate_fused.pos
python3 scripts/analysis/score_fused_output.py --reference $D/reference.csv \
  --control b_control_fused.pos --candidate b_candidate_fused.pos
```

- **Scoring** (`score_fused_output.py`): 3D error of `--out` against the
  truth as given. 0.25 s matching, the first 60 matched epochs skipped,
  every matched epoch counted whatever its status. Linear p50 and p90.
- **Reported, not in the rule:** each arm's RTK stream p50 / p90 (`--pos` of
  the same script) and its FIX rate.

## Rule (fixed here)

The same rule decides (a) and (b), separately.

**Per run:**

- **Quantiles.** A candidate quantile (p50 or p90) is *lower* below 0.99
  times the control's, *higher* above 1.01 times, *equal* otherwise.
- **Unusable** if both arms' p50 are above 10 m. The run cannot separate the
  arms.
- **Worse** if a quantile is higher, if the candidate matches fewer than 99%
  of the control's epochs, or if only the candidate arm exits non-zero.
  A control arm that exits non-zero makes the run unusable.
- **Better** if a quantile is lower and none is higher.
- **Equal** otherwise.

**Per candidate:**

- **Fail** if its RTK streams differ on either run (`cmp`), or either run is
  worse.
- **Pass** otherwise, if at least one run is better.
- **Inconclusive** otherwise: both runs unusable or equal.

## What follows

- **(a) passes:** a separate PR may propose that `--navi776-tc` turns on
  `--independent-output-filter` when `--base` is given. It carries this
  record, the development evidence and the usual tests. Merging it remains the
  maintainer's decision.
- **(b) passes:** a separate PR may propose the consider update as the
  default of the loose ESKF when neither `--navi776-tc` nor
  `--tc-closed-loop` is set, on the same terms. Under tight coupling it
  changes the RTK stream
  ([imu_fusion.md](imu_fusion.md#the-loose-eskf-before-the-heading-latch)),
  and this contract does not cover that.
- **Fail or inconclusive:** nothing changes, and the flag stays an opt-in.
- The runs are not repeated, and no other data is added to compensate. Once
  scored, Data11 and Data12 are development data.
- **Not in this contract:** the reverse-start heading flip (the backward-vote
  flip of the nagoya/run1 investigation). It exists only as an experiment
  outside the repository and needs its own contract and data.
