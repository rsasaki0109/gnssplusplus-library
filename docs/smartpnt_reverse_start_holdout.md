# Frozen holdout: the reverse-start check of the heading latch

This contract is committed before any estimator of this repository runs on its
data. It decides, once per mode and by the rule below, whether a PR may propose
turning on `--heading-latch-direction-regression`
([reverse starts](imu_fusion.md#reverse-starts)) by default:

- **(a)** under `--navi776-tc`, where it acts on the independent output filter;
- **(b)** without tight coupling, where it acts on the main loose ESKF.

The decisions are recorded whatever they are.

## Candidates and controls

All arms share `--data-dir <run> --lever-arm <run lever arm>` and the RTK flags
`--preset low-cost --ratio 2.4 --max-subset-ar-drop-steps 18
--rtk-snr-weighting --no-arfilter`.

| | Control | Candidate |
| --- | --- | --- |
| (a) | the shared flags plus `--navi776-tc` | the control plus `--heading-latch-direction-regression` |
| (b) | the shared flags | the control plus `--heading-latch-direction-regression` |

- **Binary:** `gnss_fuse` built from `ebb5fdb` (the head of the PR that adds
  the option): Release, GTSAM 4.3 found, Python bindings off. SHA256 of the
  binary used:
  `67b1889365c9115322cad8f9e808103d54094759f100fca09a5b1f39a73a9220`.
- **One run per arm**, with the README lever arm of the run. No setting is
  tuned on these runs.

## Development evidence

All 13 runs of the [development table](imu_fusion.md#reverse-starts) are
development data. They are PPC (6), UrbanNav (5) and SmartPNT-POS Data11 and
Data12.

- **What they show.** nagoya/run1 is the only reverse start among them. It
  flips in both modes, and its `--out` improves:
  - (a): 1.46 / 52.6 m → 0.21 / 13.8 m;
  - (b): 2,044 / 13,274 m → 0.23 / 20.2 m.
- **The other 12 runs** keep slopes of +0.49 to +2.49, and their `--out` is
  byte-identical.
- **Parameters chosen on these runs.** The 0.5 s interval and the deferred
  decision were added after a first run on these data. That first version had
  too few intervals on nagoya/run1.

## Holdout data

SmartPNT-POS (Kaggle `fengzhusgg/smartpnt-pos`), eight vehicle runs that this
repository has not used: the rover `ROVE` at 1 Hz, the IMU the run folder is
named after, and the Inertial Explorer truth `ROVE_GroundTruth.txt` exported
at the rover antenna.

| Run | IMU (rate) | GPS week, TOW kept | Kept rover epochs | Baseline (km) | Class |
| --- | --- | --- | ---: | --- | --- |
| Data01_20230102 (Vehicle) | ISA-100C (200 Hz) | 2243, 104002–112997 | 8,937 of 8,942 | 0.14–17.1 | reverse |
| Data14_20200828 (CAR) | FSAS (200 Hz) | 2120, 438667–447159 | 8,361 of 8,365 | 0.09–6.6 | reverse |
| Data02_20220309 (Vehicle) | ISA-100C (200 Hz) | 2200, 282458–288723 | 5,629 of 5,632 | 0.68–4.1 | forward |
| Data13_20211118 (CAR) | ISA-100C (200 Hz) | 2184, 362320–369426 | 6,994 of 6,995 | 5.3–13.1 | forward |
| Data15_20200922 (CAR) | FSAS (200 Hz) | 2124, 177737–189452 | 11,261 of 11,278 | 0.12–8.8 | forward |
| Data16_20211118 (CAR) | FSAS (200 Hz) | 2184, 374966–380796 | 4,859 of 4,877 | 1.1–13.1 | forward |
| Data19_20201214 (CAR, open sky) | HG4930 (100 Hz) | 2136, 95009–97012 | 2,004 of 10,084 (5 Hz rover, 1 Hz truth) | 0.12–2.4 | forward |
| Data20_20210115 (CAR, open sky) | HG4930 (100 Hz) | 2140, 440777–442373 | 1,597 of 8,030 (5 Hz rover, 1 Hz truth) | 0.12–1.0 | forward |

- **Selected on the truth only.** Before this contract, only the truth files
  of the 11 unused vehicle runs were read, to find how each run starts. The
  class of a run comes from the first truth epoch at which the speed has been
  at least 1 m/s for 3 s (the latch criterion at 1 Hz). At that epoch, course
  minus heading is:
  - **reverse class:** Data01 −179.1°, Data14 176.6°;
  - **forward class:** Data02 −0.7°, Data13 8.7°, Data15 0.1°, Data16 −0.4°,
    Data19 −0.2°, Data20 5.3°.

  Data13 and Data15 also start by reversing, but below 1 m/s. So the latch is
  expected in the forward motion that follows, and they are in the forward
  class.
- **Excluded:**
  - Data04: its rover file repeats epochs with the same time; the converter
    rejects it.
  - Data17 and Data18: bases over 400 MB, beyond this session's disk.
- **Conversion:** `scripts/convert_smartpnt_pos_to_ppc_layout.py` (SHA256
  `7ee1950e624e5e81ef32e644249ced96ef87b4e3065d494c8691c47d49f768be`). It
  applies the [rules of the earlier holdout](smartpnt_fuse_output_holdout.md#holdout-data),
  with the rover, IMU and truth file of each run.
  - **Base position.** Data02's base header position is zeros; the converter
    rewrites it, as it does for every run.
  - **Lever arm** (README vehicle frame to FLU), as in each manifest:
    - Data01, Data02, Data13: `-0.284,0.512,0.187`, `-0.140,0.450,1.160`,
      `-0.284,0.512,0.187`;
    - Data14, Data15: `-0.516,0.000,0.337`;
    - Data16: `-0.282,-0.500,0.021`;
    - Data19, Data20: `-0.005,-0.044,0.182`.
- **What was looked at.** Only these things; no estimator was run:
  - the truth files of the 11 runs (the classes above);
  - what the conversion prints;
  - `gnss_rinex_info --count-records` on the converted files;
  - the distance of the truth from the base.

  Details of each:
  - **IMU axis check.** On every run the gyro z correlates with the truth
    heading rate at 0.993–0.999, and the accel x and y correlations have the
    right sign.
  - **Every kept epoch** has a truth row, an IMU grid sample and an exact base
    epoch.

Raw inputs:

| File | Bytes | SHA256 |
| --- | ---: | --- |
| Data01 `README.xml` | 2,026 | `fdf7969aa21712b7b036489a9798b8e28d05b6cd35ac294c2cb7d485f4f9cd85` |
| Data01 `ROVE.23O` | 63,238,248 | `f4ee991aad4a0384275a5a5eb18c45b90bb2c0c701d2c3572449eb2171b45e30` |
| Data01 `BASE.23O` | 234,755,644 | `d99672f919bae83028aed8d96a7da4a8c364df0d3936a8c5b3e17733339ce1c9` |
| Data01 `brdm0020.23p` | 6,967,774 | `cd375656a4a7efe43dabaf5f2003bf4178a749347cf34dad65d36fca7ce1e2c6` |
| Data01 `ISA-100C.imr` | 57,600,224 | `aa3833d614fecbe487ef5dc9b8727e27c24c1a000f48e49d413f6b98e4d32ceb` |
| Data01 `ROVE_GroundTruth.txt` | 5,903,098 | `41257ba373ef7b924f89cd455283d64086b6c066f09eb8fa1e3ac3346fbff3d3` |
| Data02 `README.xml` | 865 | `fdaf86f73eec061399b77e7dfac028e5dcf9cc11d39f4da8b758945bf8f2000e` |
| Data02 `ROVE.22O` | 51,592,385 | `1c03551a683a697698f0b63f1f041e292a6e85be75c1783a06abe7c5c496a3f4` |
| Data02 `BASE.22o` | 159,545,050 | `fba7e7e44870da50707ee260784a9bce4d4d4f297b55fb8081e1b13fe8fcadeb` |
| Data02 `brdm0680.22p` | 7,061,180 | `26584186fd91fe6bf58c0dfa6183cc9b0155e1e1a1cce6962f9e641087a5ec99` |
| Data02 `ISA-100C.imr` | 40,141,216 | `9752bf9a54fdd9cde5a83c74b70ffe9c05cb0652bcb6be1550ca8f44edb20ea1` |
| Data02 `ROVE_GroundTruth.txt` | 4,082,969 | `c6fda1d2c31b0d4fc4a3472c55957366c1574b4a7d627eee08e34867b40f2359` |
| Data13 `README.xml` | 844 | `d2199af9f11aa60d58f40ccc461d011f6b3e834c51b8d72911c825868e252299` |
| Data13 `ROVE.21O` | 38,105,806 | `9d5a34ff20f61ee91418008d1e6692bbe094eabc3e5f034eb5d5b50cd1c78512` |
| Data13 `BASE.21o` | 94,209,332 | `4d133bd679e2291e6128ca9dac325873718a71b1c0c5a0bf9d08b9704b6e7a0a` |
| Data13 `brdm3220.21p` | 6,686,073 | `23d41549e58426ea7816748d0d547b4962b3595c96fd6a76361dd2f22bf4cddc` |
| Data13 `ISA-100C.imr` | 45,489,696 | `989c525e7c4113c15ba5724c8a36f144a5cf3124e7dee1c524004c7e13548bc3` |
| Data13 `ROVE_GroundTruth.txt` | 4,657,536 | `12287ad7629bea8db8757227688679dd6b5e85f2bb89e087671ce3e13f0c04d6` |
| Data14 `README.xml` | 834 | `62fe6a646848e8609d80eb2c121a9ea74b3f446e23acf03ef4f09d073bdd95a9` |
| Data14 `ROVE.20O` | 34,580,568 | `144be8e4f3bcf8fb73ad44204f25c1b2dc020ad6e0a41f272c09a0f7d6cb45bd` |
| Data14 `BASE.20O` | 110,279,220 | `22e03dfeed5a9f423b5f76cd7adb85ec1577b4ac0c10d6ab90dd8b69edd8fa19` |
| Data14 `brdm2410.20p` | 6,959,796 | `1d0a75914ded9a681eb3b4499287b5b92444866a40433330ba77aa95371caff6` |
| Data14 `FSAS.imr` | 54,399,200 | `db43f1610165eca78d489d62abbb9560914785567a43d4e53421172072d51a31` |
| Data14 `ROVE_GroundTruth.txt` | 5,565,370 | `a1b168edd35902725e2dc6c243d1c7adb956fe89f6e076c53941546b7e24a6e6` |
| Data15 `README.xml` | 833 | `b812f9822653ba0064d75c20b26dfeae6108e8156b4ac18843609ecf675b68ad` |
| Data15 `ROVE.20O` | 41,407,056 | `f976860663a0d1849bbc04b7510a752bf538429a4b77ac051b6a3e774a1c87e0` |
| Data15 `BASE.20O` | 114,695,284 | `cc10d5fccbc3cf7933e9ca0bc1fffe3156921dfcbbe638549223021d8527d748` |
| Data15 `brdm2660.20p` | 6,760,171 | `45e856bfb1dba612eac1749cad037d340e75063df549996fe1671e93226a98b0` |
| Data15 `FSAS.imr` | 74,996,320 | `9cd5deb3273d042fc89eb67f93f0ff3cfad408b421e6111aed25bbf25d4ce172` |
| Data15 `ROVE_GroundTruth.txt` | 7,676,435 | `b361bbd004dc6f8d5a4068d10bc81f396456662f745660e9bf94ba6c0e532920` |
| Data16 `README.xml` | 834 | `f8c920db6d2335fe26b7b5899714abecda444090a59fdf494767df26739455be` |
| Data16 `ROVE.21O` | 22,628,466 | `ecd0527fa355a6afa2dcd3a7ebf5a93c945f0789950e9388d9e76947ee210937` |
| Data16 `BASE.21o` | 78,381,898 | `249bfe4e353bcfc09e1fd0d26138747c244dfc6b5df0a7a01d968f2cac14aecd` |
| Data16 `brdm3220.21p` | 6,686,073 | `23d41549e58426ea7816748d0d547b4962b3595c96fd6a76361dd2f22bf4cddc` |
| Data16 `FSAS.imr` | 37,430,656 | `c5d3637dea425c8ed3d39e0288a91a5d49c8910b0ed48afc497064a53d00cb28` |
| Data16 `ROVE_GroundTruth.txt` | 3,821,753 | `e7b569dab9330380209e0e6cb16f515c0200de9a8fd0422e69dc5164743b6fd6` |
| Data19 `README.xml` | 792 | `8c56fcd12790d1041327c7510c1d5c1155c3cfb4eded7473437b94e7757da9f5` |
| Data19 `ROVE.20O` | 48,579,064 | `7121091dd30d40fb55463f62cc607ae99656aedef017bc9f3e6dec55f1feddec` |
| Data19 `BASE.20o` | 15,428,305 | `a8ae807a15ec84db4e9c32fceb9802abadd61d522b66897a0fd560cd6dd7e9da` |
| Data19 `brdm3490.20p` | 7,115,558 | `3f03f69998f622db5576e682523340cd5cfc85b35931ca6dc41e35c5cefc26d1` |
| Data19 `HG4930.imr` | 6,422,528 | `3ee12619d7fea78ca6ee738677c1a49125eba9febdc587cda2ff3fbd27dfb725` |
| Data19 `ROVE_GroundTruth.txt` | 1,315,079 | `3c357eeaa43059e7d11c5271e69908d810daf1af4533687de56c127120e7bf3f` |
| Data20 `README.xml` | 792 | `4847587b5f66c2b9f43ea899aaf476d44f5c76a203e7cc016a8b6db8a28def91` |
| Data20 `ROVE.21O` | 54,413,480 | `c4e8dbf751c114543d453b921fe48b20dc5556c777b108c59fc580c8157e1bdc` |
| Data20 `BASE.21O` | 19,510,028 | `f7c6f0e9640745272d234be29cd5389e27478e42476d0cc46ef1993d78e29b84` |
| Data20 `brdm0150.21p` | 6,627,727 | `817cce29fd7349584d5681c5288b66c265b7508c50ad95e7190c97bc3c1412eb` |
| Data20 `HG4930.imr` | 5,111,808 | `15c45b89cb705ed6458a7b7d5c36c6e5684fe9e09420097d85fd9e363b92a1e1` |
| Data20 `ROVE_GroundTruth.txt` | 1,048,494 | `dc504ea8a1cd4f17801c93e11bfc93b50b1e8aeb1d0bb0e45dd5eb4bc583aae8` |

Converted `smartpnt/<run>_rove_<imu>/`:

| File | Bytes | SHA256 |
| --- | ---: | --- |
| Data01 `rover.obs` | 63,198,237 | `32f034c924df27b213696c87bd8cc6c2e693b4403c8c2d9c85a83407300a3564` |
| Data01 `base.obs` | 106,720,658 | `e28d0181d2d5c3826aae89379afc241c6d6c2439dea6bb743101fbbb0e0a1f7a` |
| Data01 `base.nav` | 6,967,774 | `cd375656a4a7efe43dabaf5f2003bf4178a749347cf34dad65d36fca7ce1e2c6` |
| Data01 `imu.csv` | 84,604,711 | `5d64169e1309416b8adc643b5469d172dd8a0ce22112ddb57943dad4b7441210` |
| Data01 `reference.csv` | 1,497,938 | `a2018dacd2382d99d5f334531a5e2c59882d249c7c0af30d142c231bf8ea9fe4` |
| Data02 `rover.obs` | 51,557,990 | `5f8c34b3b5deb5343d67b5960a48c4698c96b81d48eda037f3f27a66b2c1f764` |
| Data02 `base.obs` | 34,868,293 | `6707148d3e807ab4fd65053fe1eb01dba6b5cc6af34590f9e178f7b792cc7af4` |
| Data02 `base.nav` | 7,061,180 | `26584186fd91fe6bf58c0dfa6183cc9b0155e1e1a1cce6962f9e641087a5ec99` |
| Data02 `imu.csv` | 58,992,055 | `62fd1959f215c3e4454cab8235589f44e4c2e8edd0794a0c57654af1170caa09` |
| Data02 `reference.csv` | 894,695 | `fac13b5f910ad40274a8708f2d21c85d4dd20c4d21052ebf24b3468ba8625dcb` |
| Data13 `rover.obs` | 38,105,352 | `f9d2673d4d8fb0d5a13f9139962446cdd50de09354953b01b95c816f7bb9c073` |
| Data13 `base.obs` | 94,144,904 | `4e30b13ff6f8fd8eacb8b8d2d5a102a4b159707d6f8c339f7c34407d0606be23` |
| Data13 `base.nav` | 6,686,073 | `23d41549e58426ea7816748d0d547b4962b3595c96fd6a76361dd2f22bf4cddc` |
| Data13 `imu.csv` | 66,815,010 | `e5a33c7356129faf52c330608725c32c5d468f37e78f41dd13d227dd80178ae9` |
| Data13 `reference.csv` | 1,170,559 | `6d7a4735bde6c02969b190210fcc10f704b2287e40d76beaa18aeae237cfac1c` |
| Data14 `rover.obs` | 34,558,952 | `2ddb084d3674f2e3c42ad246547c7279406b779846e8a2652957029f17bfb2b2` |
| Data14 `base.obs` | 87,596,659 | `5edc4439d24c770bf9bf6d6f843140c900167dd406049d6f7ceb857b5c3a3bda` |
| Data14 `base.nav` | 6,959,796 | `1d0a75914ded9a681eb3b4499287b5b92444866a40433330ba77aa95371caff6` |
| Data14 `imu.csv` | 79,910,444 | `0f6534a82494c045eb62ee28edba85b7ef9f015be7198b1c5d46df685e3bea33` |
| Data14 `reference.csv` | 1,401,398 | `68a3e09b591e97a4ebb28ccdfa764c0908d9a8f22184a7524da74ab6af19b73c` |
| Data15 `rover.obs` | 41,336,770 | `ee114ded37d43a411424c3fc59761cd8d3433af71d8ed91904a94b77b524fe18` |
| Data15 `base.obs` | 100,264,874 | `633e0eafcd6cfab897b045ef67410f67ad7be93d5f06f3cddaf0987ff900b34f` |
| Data15 `base.nav` | 6,760,171 | `45e856bfb1dba612eac1749cad037d340e75063df549996fe1671e93226a98b0` |
| Data15 `imu.csv` | 110,163,191 | `9b80517146f50fcf32eb022236cff7f077dfbe895fe80f81a75f629abc2ee14d` |
| Data15 `reference.csv` | 1,878,420 | `0c55a8206926a203db80e5b087e82a06e64f1d151dfdffcc8529c1acfe58523b` |
| Data16 `rover.obs` | 22,569,342 | `3180e3f883aef170b2567c1c6bcbf8450242f70bc02c47c04abba1b9353708b3` |
| Data16 `base.obs` | 78,152,966 | `e8a94ff3aedb28b2fa742d41ab7051fa6c7ba78aa553fb86499d8d3e4e544081` |
| Data16 `base.nav` | 6,686,073 | `23d41549e58426ea7816748d0d547b4962b3595c96fd6a76361dd2f22bf4cddc` |
| Data16 `imu.csv` | 54,977,252 | `6fc4f5b80783a50d2b09ab18a72aa56ab7f9edd77e4c805eb9a0640c97d4c64f` |
| Data16 `reference.csv` | 814,707 | `323cc4ae945261ee184415a6ccb408597431a59a7fe82b958ff2d129a643f66b` |
| Data19 `rover.obs` | 9,650,640 | `afb7dfcfa74ff2e75696f4d8db00ad8d597f3f1666fdb55f3b22192bb5707b84` |
| Data19 `base.obs` | 6,620,693 | `c7121ee7f91f9d263b7e5bc812b8b91519c802a0acee8206e04d672eaf3dc271` |
| Data19 `base.nav` | 7,115,558 | `3f03f69998f622db5576e682523340cd5cfc85b35931ca6dc41e35c5cefc26d1` |
| Data19 `imu.csv` | 18,671,601 | `78bf723b7e82c0a457c38fa6815e602d3eb4ca4ba7de433b1dcbf5032da13947` |
| Data19 `reference.csv` | 331,630 | `659b498b436cdba5a569f28273ef01c8cd1d0a248eb427d9c753564eff8104dd` |
| Data20 `rover.obs` | 10,822,806 | `8cfd1808bc3ec3c825981d2f6ac02579d7ffcd8ec1694cf45055433ef455e810` |
| Data20 `base.obs` | 4,480,186 | `f5cc0f09588b05fbce46fcda74203e5995740c618ceb7260a42bf22f369bf726` |
| Data20 `base.nav` | 6,627,727 | `817cce29fd7349584d5681c5288b66c265b7508c50ad95e7190c97bc3c1412eb` |
| Data20 `imu.csv` | 15,021,209 | `5adba5a298a355eda0c9937605493b36a9743280546cb4c2f4819491c0b0d893` |
| Data20 `reference.csv` | 265,818 | `79fdfedb57b0d9d19ed5d057c1f4247ebe8cc744ad510640a5f92e5524fa49a7` |

## Replay and scoring (fixed here)

For each run, with `D` its converted directory and `L` its lever arm:

```sh
COMMON="--data-dir $D --lever-arm $L --preset low-cost --ratio 2.4 \
  --max-subset-ar-drop-steps 18 --rtk-snr-weighting --no-arfilter"
gnss_fuse $COMMON --navi776-tc --out a_control_fused.pos --rtk-pos-out a_control_rtk.pos
gnss_fuse $COMMON --navi776-tc --heading-latch-direction-regression \
  --out a_candidate_fused.pos --rtk-pos-out a_candidate_rtk.pos
gnss_fuse $COMMON --out b_control_fused.pos --rtk-pos-out b_control_rtk.pos
gnss_fuse $COMMON --heading-latch-direction-regression \
  --out b_candidate_fused.pos --rtk-pos-out b_candidate_rtk.pos
python3 scripts/analysis/score_fused_output.py --reference $D/reference.csv \
  --control a_control_fused.pos --candidate a_candidate_fused.pos   # and the same for b
```

- **Flips** are the `flips:` count of the candidate's summary line
  `Heading latch direction regression: on (flips: N, last slope: S)`.
- **Run outcome** (better, worse, equal, unusable) is that of
  `score_fused_output.py`. It uses the 3D error of `--out`, 0.25 s
  matching, the first 60 matched epochs skipped, and p50 and p90 compared in a
  1% band; a run is unusable when both p50 are above 10 m.
- **Identical** means `cmp` finds no difference.

## Rule (fixed here)

The same rule decides (a) and (b), separately.

- **Fail** if any of these holds:
  - an RTK stream differs between control and candidate;
  - a forward-class run flips, or its candidate `--out` is not identical to
    the control's;
  - a reverse-class run flips more than once, or flips and is worse;
  - a reverse-class run does not flip and its `--out` is not identical.
- **Pass** otherwise, if at least one reverse-class run flips and is better.
- **Inconclusive** otherwise.

## What follows

- **(a) passes:** a separate PR may propose that `--navi776-tc` turns the
  check on.
- **(b) passes:** a separate PR may propose the check as the default of the
  loose ESKF without tight coupling.
- **On those PRs:** each carries this record, the development evidence and the
  usual tests. Merging remains the maintainer's decision.
- **Fail or inconclusive:** nothing changes, and the option stays an opt-in.
- The runs are not repeated, and no other data is added to compensate. Once
  scored, these eight runs are development data.
