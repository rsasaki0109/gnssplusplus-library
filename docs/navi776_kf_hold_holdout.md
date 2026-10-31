# Frozen holdout: the validated-only KF hold in `--navi776-tc`

This contract is committed before any estimator of this repository runs on its
data. It decides whether a PR may propose adding
`--hold-kalman-update --hold-kalman-update-validated-only` to the
`gnss fuse --navi776-tc` preset. The decision is made once, by the rule below,
and is recorded whatever it is.

## Candidate and control

- **Control:** the PPC replay command of `scripts/analysis/ppc_fuse_spread.py`
  with `--navi776-tc` (`--preset low-cost --ratio 2.4
  --max-subset-ar-drop-steps 18 --rtk-snr-weighting --no-arfilter
  --navi776-tc`).
- **Candidate:** the control plus `--hold-kalman-update
  --hold-kalman-update-validated-only`.
- **Binary:** `gnss_fuse` built from the commit that adds this file, in the
  default configuration.

## Development evidence

All four sets are development data for this candidate
([comparison](ppc_rtk_vs_gnss_imu_fusion.md#candidates-on-urbannav-hong-kong-and-tokyo)).
Correct FIX (% of epochs) / wrong FIX (% of FIX) at 0.5 m, and correct FIX at
0.25 m, means of nine runs:

| Set | Control | Candidate | Control, 0.25 m | Candidate, 0.25 m | This rule |
| --- | ---: | ---: | ---: | ---: | --- |
| PPC, six runs | 68.09 / 7.08 | 70.57 / 5.68 | 62.93 | 68.35 | pass |
| UrbanNav Odaiba | 64.67 / 3.60 | 64.05 / 4.77 | 62.46 | 62.58 | fail (wrong FIX higher) |
| UrbanNav Shinjuku | 64.87 / 17.63 | 72.68 / 8.26 | 53.93 | 59.87 | pass |
| UrbanNav HK Deep-Urban-1 | 17.07 / 32.22 | 16.40 / 31.27 | 12.65 | 11.35 | fail (−1.3 pp at 0.25 m) |

The candidate fails this rule on two of the four development sets. A pass here
is one more data point, not proof.

## Holdout data

UrbanNav Hong Kong Medium-Urban-1 (IPNL-POLYU/UrbanNavDataset, Dropbox
downloads), 2021-05-17, Tsim Sha Tsui.

- **Instruments:** as Deep-Urban-1 and Harsh-Urban-1 in
  [holdout contract v2](online_pva_default_switch_holdout_v2.md#holdout-data):
  NovAtel Flexpak6 rover at 1 Hz, HKSC base (hourly 1 Hz file and the day's
  GN, RN, EN and CN broadcast files), Xsens IMU, SPAN-CPT truth at 1 Hz.
- **Not used before by any estimator.** Contract v2 records that an earlier
  survey read this run at format level only and excluded it from its
  population. It was not converted until this contract.
- **Conversion:** `scripts/convert_urbannav_hk_to_ppc_layout.py --run
  HKMediumUrban1`, the Deep-Urban-1 rules unchanged. 704 of 705 rover epochs
  are kept (one has no IMU sample), GPS TOW 95594–96379, week 2158, longest gap
  22 s. Every kept epoch has a truth row, an IMU grid sample and an exact base
  epoch.
- **What was looked at:** only what the conversion prints. No estimator was
  run, and no position was compared with the truth.

Raw inputs:

| File | Bytes | SHA256 |
| --- | ---: | --- |
| `UrbanNav-HK-Medium-Urban-1.novatel.flexpak6.obs` | 548,442 | `e99e563b018094a4f48cf6dd2f5b77441f57df3e593809532abe6252b38c04d4` |
| `UrbanNav_TST_GT_raw.txt` | 140,442 | `9d48bb497878aafbd17290789560394c72ecafec20c9e0eaff448418695d92bf` |
| `xsense_imu_medium_urban1.csv` | 109,736,693 | `6a3e0ab0209e032296dc750a491e27a4919473c875c5ec8f2dcd1bd3d796537d` |
| `HKSC00HKG_R_20211370200_01H_01S_MO.crx.gz` | 2,322,421 | `11348ef61864d74b995b4a235a5459bc9da5a0e0567343a2e7e63245328604da` |
| `HKSC00HKG_R_20211370000_01D_GN.rnx.gz` | 29,451 | `e2d2b66142d08def1a683fb13a869cbf0c44dd84f956c7218758753a50151eb1` |
| `HKSC00HKG_R_20211370000_01D_RN.rnx.gz` | 29,111 | `4e2ed74803ffd23f1dfc6385f8bbe4e0eeb7b41c4400f70bc2831c6b4d8bee37` |
| `HKSC00HKG_R_20211370000_01D_EN.rnx.gz` | 123,183 | `807fe0f06391e9e6ffe6f8162efcd7aa6cf8869e65c3f3ce96f38329ca5c365b` |
| `HKSC00HKG_R_20211370000_01D_CN.rnx.gz` | 62,717 | `a78a404fe0036ba6264de8b9951e2c13a836e8a739aef0218a5235b720f8a024` |

Converted `urbannav/HKMediumUrban1_novatel/`:

| File | Bytes | SHA256 |
| --- | ---: | --- |
| `rover.obs` | 548,251 | `6b000bc7391b5495b2354fc00fdb49b0d3a1466e980e867fe928c56d189f4dc5` |
| `base.obs` | 5,578,920 | `a7ca93213af4b938e57b1093e2c5930caf9ef47c519e9db97b1686084ced98b1` |
| `base.nav` | 1,579,910 | `8c1f40f0a220ca76f9d969ce7f0255549e7c2d2307932b5065ed489c54b6eee3` |
| `imu.csv` | 7,309,199 | `d019dea03b9501b97ee1645da508629573ceaf284beecc54bfb70189c8ed3576` |
| `reference.csv` | 112,760 | `5fa03d8ec92102f7a5eb895516ed24592f89cc65f9eb0654731f62c31d968a21` |

## Replay and scoring (fixed here)

- Nine runs per arm: the zero lever arm and its eight perturbations, as the
  spread script does for every run.
- The HKSC header position is the base position, as in contract v2.
- **Antenna offset.** The Hong Kong truth is about 0.6 m from the rover
  antenna. The offset measured on Deep-Urban-1 (antenna minus truth, right
  −0.014 m, forward −0.563 m, up −0.227 m) is applied to the truth, for both
  arms, as `--truth-offset-body=-0.014,-0.563,-0.227`. It is not re-estimated
  on this run.
- Scoring as the spread script: 0.25 s matching, 60-epoch skip, FIX = status 4,
  3D error after the offset.

```sh
python3 scripts/analysis/ppc_fuse_spread.py --dataset-root <root> \
  --runs urbannav/HKMediumUrban1_novatel --lever-arm 0,0,0 \
  --truth-offset-body=-0.014,-0.563,-0.227 --out-dir <out>/control
python3 scripts/analysis/ppc_fuse_spread.py --dataset-root <root> \
  --runs urbannav/HKMediumUrban1_novatel --lever-arm 0,0,0 \
  --truth-offset-body=-0.014,-0.563,-0.227 --out-dir <out>/candidate \
  -- --hold-kalman-update --hold-kalman-update-validated-only
```

The 0.25 m numbers come from `--score-only --correct-3d-m 0.25` on the same
output directories.

## Rule (fixed here)

This is the rule of the
[UrbanNav out-of-sample check](ppc_rtk_vs_gnss_imu_fusion.md#urbannav-out-of-sample-check),
unchanged.

- **Pass** if both hold for the nine-run means:
  1. the candidate's wrong-FIX rate at 0.5 m is below the control's;
  2. the candidate's correct FIX at 0.25 m is not more than 1 pp below the
     control's.
- **Inconclusive** if the control's FIX rate is below 5% of the epochs. Then
  the run cannot separate the arms, as Harsh-Urban-1 could not.
- **Fail** otherwise.

## What follows

- **Pass:** a separate PR may propose the change to `--navi776-tc`, with this
  record, the development evidence and its two failures, and the usual tests.
  Merging it remains the maintainer's decision.
- **Fail or inconclusive:** the preset is unchanged, and the candidate stays
  an opt-in.
- The run is not repeated, and no other data is added to compensate. Once
  scored, Medium-Urban-1 is development data.
