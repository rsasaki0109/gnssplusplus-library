# PPC residual wrong-FIX analysis

Reference truth is used only after the run to label wrong FIX; the source and gate columns use data already present in the real-time POS/selector outputs.

Wrong-FIX threshold: **0.500 m** 3D error.

## Key findings

- `filter` residual wrong FIX is all baseline-selected and did not match a selector rule.
- Best low-collateral deployable gate for `filter` in this POS-only replay: nis_per_obs > 20 catches 37.352% of wrong FIX while touching 5.103% of good FIX.
- `smoother_v22` residual wrong FIX is all baseline-selected and did not match a selector rule.
- Best low-collateral deployable gate for `smoother_v22` in this POS-only replay: nis_per_obs > 20 catches 57.156% of wrong FIX while touching 6.040% of good FIX.

## Profile summary

| Profile | FIX epochs | Wrong FIX | Wrong/FIX | Wrong source | Rule matched |
|---|---:|---:|---:|---|---|
| filter | 44212 | 6238 | 14.109% | baseline=6238 | false=6238 |
| smoother_v22 | 42386 | 2236 | 5.275% | baseline=2236 | false=2236 |

## filter

| Run | FIX epochs | Wrong FIX | Wrong/FIX | Wrong p95 | Longest wrong span |
|---|---:|---:|---:|---:|---|
| tokyo_run1 | 9339 | 1936 | 20.730% | 93.317 m | 189314.800-189389.000 (74.400 s) |
| tokyo_run2 | 7718 | 350 | 4.535% | 45.321 m | 177731.400-177748.600 (17.400 s) |
| tokyo_run3 | 12381 | 993 | 8.020% | 18.113 m | 181699.600-181746.000 (46.600 s) |
| nagoya_run1 | 6026 | 875 | 14.520% | 8.789 m | 551153.000-551266.200 (113.400 s) |
| nagoya_run2 | 5938 | 790 | 13.304% | 130.037 m | 556155.400-556181.400 (26.200 s) |
| nagoya_run3 | 2810 | 1294 | 46.050% | 78.754 m | 554728.600-554769.600 (41.200 s) |

### Real-time discriminator percentiles

| Metric | Good FIX p50/p90/p95 | Wrong FIX p50/p90/p95 |
|---|---:|---:|
| ratio | 28.000/112.700/180.700 | 4.600/35.100/89.500 |
| baseline_m | 2869.429/9427.663/9534.781 | 4730.074/9826.517/10091.115 |
| nsat | 17.000/23.000/25.000 | 14.000/19.000/20.000 |
| prefit_rms_m | 0.544/3.450/16.366 | 2.471/17.822/38.037 |
| prefit_max_m | 5.107/31.139/68.444 | 20.047/70.043/104.680 |
| post_rms_m | 0.536/1.456/3.694 | 1.421/7.139/7.848 |
| post_max_m | 5.107/23.451/27.944 | 18.720/28.920/29.454 |
| nis_per_obs | 0.619/5.135/20.342 | 6.664/58.370/73.126 |
| outliers | 0.000/1.000/5.000 | 0.000/8.000/12.000 |

### Deployable gate simulation

| Gate | Wrong caught | Wrong caught | Good FIX harmed | Good FIX harmed |
|---|---:|---:|---:|---:|
| ratio < 6 | 3719 | 59.618% | 4449 | 11.716% |
| ratio < 8 | 4194 | 67.233% | 6637 | 17.478% |
| post_rms > 4 m | 2102 | 33.697% | 1558 | 4.103% |
| post_rms > 6 m | 1388 | 22.251% | 482 | 1.269% |
| nis_per_obs > 20 | 2330 | 37.352% | 1938 | 5.103% |
| nis_per_obs > 50 | 963 | 15.438% | 278 | 0.732% |
| prefit_rms > 5 m | 2518 | 40.366% | 3409 | 8.977% |
| baseline > 7000 m and post_rms > 4 m | 1138 | 18.243% | 613 | 1.614% |

## smoother_v22

| Run | FIX epochs | Wrong FIX | Wrong/FIX | Wrong p95 | Longest wrong span |
|---|---:|---:|---:|---:|---|
| tokyo_run1 | 8760 | 656 | 7.489% | 94.363 m | 189461.600-189500.000 (38.600 s) |
| tokyo_run2 | 7513 | 99 | 1.318% | 46.619 m | 178229.200-178241.000 (12.000 s) |
| tokyo_run3 | 12153 | 701 | 5.768% | 27.792 m | 181691.400-181744.400 (53.200 s) |
| nagoya_run1 | 6023 | 42 | 0.697% | 27.939 m | 550968.200-550971.400 (3.400 s) |
| nagoya_run2 | 5531 | 433 | 7.829% | 85.505 m | 556150.000-556180.400 (30.600 s) |
| nagoya_run3 | 2406 | 305 | 12.677% | 83.461 m | 553814.800-553831.600 (17.000 s) |

### Real-time discriminator percentiles

| Metric | Good FIX p50/p90/p95 | Wrong FIX p50/p90/p95 |
|---|---:|---:|
| ratio | 15.700/67.800/117.200 | 3.900/25.900/61.000 |
| baseline_m | 2945.070/9441.536/9632.869 | 3041.909/8961.940/9379.419 |
| nsat | 17.000/23.000/25.000 | 15.000/21.000/23.000 |
| prefit_rms_m | 0.555/3.817/16.288 | 5.408/10.861/35.272 |
| prefit_max_m | 5.510/32.038/68.619 | 23.194/51.986/95.049 |
| post_rms_m | 0.546/1.583/3.672 | 4.551/6.873/7.529 |
| post_max_m | 5.506/23.500/27.822 | 21.706/28.914/29.373 |
| nis_per_obs | 0.728/8.347/22.899 | 27.266/91.103/94.904 |
| outliers | 0.000/1.000/5.000 | 0.000/4.000/9.000 |

### Deployable gate simulation

| Gate | Wrong caught | Wrong caught | Good FIX harmed | Good FIX harmed |
|---|---:|---:|---:|---:|
| ratio < 6 | 1436 | 64.222% | 8099 | 20.172% |
| ratio < 8 | 1619 | 72.406% | 11097 | 27.639% |
| post_rms > 4 m | 1189 | 53.175% | 1607 | 4.002% |
| post_rms > 6 m | 702 | 31.395% | 507 | 1.263% |
| nis_per_obs > 20 | 1278 | 57.156% | 2425 | 6.040% |
| nis_per_obs > 50 | 817 | 36.538% | 498 | 1.240% |
| prefit_rms > 5 m | 1180 | 52.773% | 3653 | 9.098% |
| baseline > 7000 m and post_rms > 4 m | 392 | 17.531% | 650 | 1.619% |
