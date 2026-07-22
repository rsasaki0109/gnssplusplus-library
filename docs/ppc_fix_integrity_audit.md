# PPC residual wrong-FIX analysis

Reference truth is used only after the run to label wrong FIX; the source and gate columns use data already present in the real-time POS/selector outputs.

Wrong-FIX threshold: **0.500 m** 3D error.

## Key findings

- `min_sat9` wrong source: unknown=1179; rule matched: unknown=1179.
- Best low-collateral deployable gate for `min_sat9` in this POS-only replay: nsat < 8 or (nsat <= 11 and ratio <= 15) catches 25.276% of wrong FIX while touching 1.056% of good FIX.
- `integrity_gate` wrong source: unknown=927; rule matched: unknown=927.
- Best low-collateral deployable gate for `integrity_gate` in this POS-only replay: nis_per_obs > 20 catches 2.913% of wrong FIX while touching 1.591% of good FIX.
- `wrong_basin_escape` wrong source: unknown=894; rule matched: unknown=894.
- Best low-collateral deployable gate for `wrong_basin_escape` in this POS-only replay: nis_per_obs > 20 catches 2.908% of wrong FIX while touching 1.591% of good FIX.
- `consensus_escape` wrong source: unknown=869; rule matched: unknown=869.
- Best low-collateral deployable gate for `consensus_escape` in this POS-only replay: nis_per_obs > 20 catches 2.992% of wrong FIX while touching 1.591% of good FIX.
- `online_consensus_kinematic_tokyo3_fgo` wrong source: unknown=668; rule matched: unknown=668.
- Best low-collateral deployable gate for `online_consensus_kinematic_tokyo3_fgo` in this POS-only replay: nis_per_obs > 20 catches 3.293% of wrong FIX while touching 1.591% of good FIX.
- `staged_integrity` wrong source: unknown=574; rule matched: unknown=574.
- Best low-collateral deployable gate for `staged_integrity` in this POS-only replay: nis_per_obs > 20 catches 2.962% of wrong FIX while touching 1.602% of good FIX.

## Profile summary

| Profile | FIX epochs | Wrong FIX | Wrong/FIX | Wrong source | Rule matched |
|---|---:|---:|---:|---|---|
| min_sat9 | 41800 | 1179 | 2.821% | unknown=1179 | unknown=1179 |
| integrity_gate | 41220 | 927 | 2.249% | unknown=927 | unknown=927 |
| wrong_basin_escape | 41185 | 894 | 2.171% | unknown=894 | unknown=894 |
| consensus_escape | 41160 | 869 | 2.111% | unknown=869 | unknown=869 |
| online_consensus_kinematic_tokyo3_fgo | 40963 | 668 | 1.631% | unknown=668 | unknown=668 |
| staged_integrity | 40896 | 574 | 1.404% | unknown=574 | unknown=574 |

## Leave-one-run-out integrity-gate validation

six-fold leave-one-run-out; <=2% training correct-FIX harm; minimize >5m, wrong FIX, then FIX loss.

| Held-out run | Selected min sat / ceiling / ratio | min-sat9 wrong | LOO wrong | LOO >5 m |
|---|---:|---:|---:|---:|
| tokyo_run1 | 8 / 13 / 10.000 | 440 | 304 | 25 |
| tokyo_run2 | 8 / 12 / 15.000 | 95 | 46 | 13 |
| tokyo_run3 | 9 / 11 / 30.000 | 335 | 310 | 16 |
| nagoya_run1 | 8 / 12 / 15.000 | 46 | 31 | 5 |
| nagoya_run2 | 8 / 12 / 15.000 | 90 | 82 | 9 |
| nagoya_run3 | 9 / 12 / 20.000 | 90 | 76 | 12 |

## min_sat9

| Run | FIX epochs | Wrong FIX | Wrong/FIX | >5 m | >10 m | Wrong p95 | Longest wrong span |
|---|---:|---:|---:|---:|---:|---:|---|
| tokyo_run1 | 9029 | 440 | 4.873% | 93 | 29 | 17.332 m | 188219.400-188225.600 (6.400 s) |
| tokyo_run2 | 7484 | 95 | 1.269% | 19 | 10 | 12.175 m | 177925.600-177927.200 (1.800 s) |
| tokyo_run3 | 12003 | 335 | 2.791% | 17 | 8 | 4.926 m | 181708.800-181747.600 (39.000 s) |
| nagoya_run1 | 6069 | 46 | 0.758% | 12 | 7 | 34.397 m | 551591.400-551593.400 (2.200 s) |
| nagoya_run2 | 4748 | 123 | 2.591% | 43 | 37 | 28.393 m | 556737.000-556746.200 (9.400 s) |
| nagoya_run3 | 2467 | 140 | 5.675% | 65 | 61 | 105.781 m | 554493.600-554495.200 (1.800 s) |

### Real-time discriminator percentiles

| Metric | Good FIX p50/p90/p95 | Wrong FIX p50/p90/p95 |
|---|---:|---:|
| ratio | 31.700/140.600/273.000 | 7.300/172.700/554.000 |
| baseline_m | 2952.167/9441.529/9622.160 | 3040.234/9312.071/9371.640 |
| nsat | 21.000/30.000/32.000 | 13.000/26.000/27.000 |
| prefit_rms_m | 0.939/10.668/23.600 | 3.537/16.268/42.161 |
| prefit_max_m | 6.001/35.933/99.661 | 12.436/58.499/132.875 |
| post_rms_m | 0.489/0.746/0.825 | 0.681/0.940/1.093 |
| post_max_m | 2.684/2.956/2.979 | 2.739/2.973/2.986 |
| nis_per_obs | 0.346/2.074/4.806 | 0.783/4.857/7.945 |
| outliers | 4.000/18.000/26.000 | 13.000/43.000/50.000 |

### Deployable gate simulation

| Gate | Wrong caught | Wrong caught | Good FIX harmed | Good FIX harmed |
|---|---:|---:|---:|---:|
| nsat < 8 or (nsat <= 11 and ratio <= 15) | 298 | 25.276% | 429 | 1.056% |
| ratio < 6 | 410 | 34.775% | 4749 | 11.691% |
| ratio < 8 | 622 | 52.757% | 6282 | 15.465% |
| post_rms > 4 m | 0 | 0.000% | 0 | 0.000% |
| post_rms > 6 m | 0 | 0.000% | 0 | 0.000% |
| nis_per_obs > 20 | 30 | 2.545% | 646 | 1.590% |
| nis_per_obs > 50 | 12 | 1.018% | 221 | 0.544% |
| prefit_rms > 5 m | 544 | 46.141% | 6547 | 16.117% |
| baseline > 7000 m and post_rms > 4 m | 0 | 0.000% | 0 | 0.000% |

## integrity_gate

| Run | FIX epochs | Wrong FIX | Wrong/FIX | >5 m | >10 m | Wrong p95 | Longest wrong span |
|---|---:|---:|---:|---:|---:|---:|---|
| tokyo_run1 | 8802 | 291 | 3.306% | 20 | 10 | 8.915 m | 189208.800-189214.200 (5.600 s) |
| tokyo_run2 | 7349 | 46 | 0.626% | 13 | 7 | 12.175 m | 178223.400-178224.600 (1.400 s) |
| tokyo_run3 | 11938 | 315 | 2.639% | 17 | 8 | 6.047 m | 181708.800-181747.600 (39.000 s) |
| nagoya_run1 | 6036 | 33 | 0.547% | 5 | 3 | 12.017 m | 551591.400-551593.400 (2.200 s) |
| nagoya_run2 | 4723 | 119 | 2.520% | 42 | 36 | 28.390 m | 556737.000-556746.200 (9.400 s) |
| nagoya_run3 | 2372 | 123 | 5.185% | 56 | 52 | 105.794 m | 554493.600-554495.200 (1.800 s) |

### Real-time discriminator percentiles

| Metric | Good FIX p50/p90/p95 | Wrong FIX p50/p90/p95 |
|---|---:|---:|
| ratio | 32.200/143.900/281.900 | 11.800/405.400/1778.100 |
| baseline_m | 2952.169/9441.529/9631.654 | 3043.482/9371.609/9371.640 |
| nsat | 21.000/30.000/32.000 | 14.000/26.000/27.000 |
| prefit_rms_m | 0.947/10.994/23.650 | 6.695/19.429/46.124 |
| prefit_max_m | 6.090/36.157/99.775 | 19.935/69.870/133.704 |
| post_rms_m | 0.489/0.745/0.825 | 0.670/0.917/1.052 |
| post_max_m | 2.686/2.956/2.979 | 2.795/2.976/2.988 |
| nis_per_obs | 0.346/2.053/4.801 | 0.922/4.635/8.969 |
| outliers | 4.000/18.000/26.000 | 19.000/47.000/52.000 |

### Deployable gate simulation

| Gate | Wrong caught | Wrong caught | Good FIX harmed | Good FIX harmed |
|---|---:|---:|---:|---:|
| nsat < 8 or (nsat <= 11 and ratio <= 15) | 0 | 0.000% | 9 | 0.022% |
| ratio < 6 | 223 | 24.056% | 4511 | 11.195% |
| ratio < 8 | 379 | 40.885% | 5969 | 14.814% |
| post_rms > 4 m | 0 | 0.000% | 0 | 0.000% |
| post_rms > 6 m | 0 | 0.000% | 0 | 0.000% |
| nis_per_obs > 20 | 27 | 2.913% | 641 | 1.591% |
| nis_per_obs > 50 | 12 | 1.294% | 220 | 0.546% |
| prefit_rms > 5 m | 509 | 54.908% | 6526 | 16.196% |
| baseline > 7000 m and post_rms > 4 m | 0 | 0.000% | 0 | 0.000% |

## wrong_basin_escape

| Run | FIX epochs | Wrong FIX | Wrong/FIX | >5 m | >10 m | Wrong p95 | Longest wrong span |
|---|---:|---:|---:|---:|---:|---:|---|
| tokyo_run1 | 8802 | 291 | 3.306% | 20 | 10 | 8.915 m | 189208.800-189214.200 (5.600 s) |
| tokyo_run2 | 7349 | 46 | 0.626% | 13 | 7 | 12.175 m | 178223.400-178224.600 (1.400 s) |
| tokyo_run3 | 11938 | 315 | 2.639% | 17 | 8 | 6.047 m | 181708.800-181747.600 (39.000 s) |
| nagoya_run1 | 6036 | 33 | 0.547% | 5 | 3 | 12.017 m | 551591.400-551593.400 (2.200 s) |
| nagoya_run2 | 4688 | 86 | 1.834% | 10 | 4 | 8.031 m | 556737.000-556746.200 (9.400 s) |
| nagoya_run3 | 2372 | 123 | 5.185% | 56 | 52 | 105.794 m | 554493.600-554495.200 (1.800 s) |

### Real-time discriminator percentiles

| Metric | Good FIX p50/p90/p95 | Wrong FIX p50/p90/p95 |
|---|---:|---:|
| ratio | 32.200/143.900/282.000 | 11.300/476.000/1799.400 |
| baseline_m | 2952.169/9441.529/9631.654 | 2988.007/9371.611/9371.640 |
| nsat | 21.000/30.000/32.000 | 14.000/27.000/27.000 |
| prefit_rms_m | 0.947/10.953/23.649 | 6.322/19.546/46.126 |
| prefit_max_m | 6.088/36.119/99.775 | 19.274/71.166/133.707 |
| post_rms_m | 0.489/0.745/0.825 | 0.681/0.921/1.056 |
| post_max_m | 2.686/2.956/2.979 | 2.796/2.976/2.987 |
| nis_per_obs | 0.346/2.053/4.801 | 0.864/4.815/8.614 |
| outliers | 4.000/18.000/26.000 | 19.000/42.000/52.000 |

### Deployable gate simulation

| Gate | Wrong caught | Wrong caught | Good FIX harmed | Good FIX harmed |
|---|---:|---:|---:|---:|
| nsat < 8 or (nsat <= 11 and ratio <= 15) | 0 | 0.000% | 9 | 0.022% |
| ratio < 6 | 212 | 23.714% | 4511 | 11.196% |
| ratio < 8 | 367 | 41.051% | 5969 | 14.815% |
| post_rms > 4 m | 0 | 0.000% | 0 | 0.000% |
| post_rms > 6 m | 0 | 0.000% | 0 | 0.000% |
| nis_per_obs > 20 | 26 | 2.908% | 641 | 1.591% |
| nis_per_obs > 50 | 11 | 1.230% | 220 | 0.546% |
| prefit_rms > 5 m | 476 | 53.244% | 6524 | 16.192% |
| baseline > 7000 m and post_rms > 4 m | 0 | 0.000% | 0 | 0.000% |

## consensus_escape

| Run | FIX epochs | Wrong FIX | Wrong/FIX | >5 m | >10 m | Wrong p95 | Longest wrong span |
|---|---:|---:|---:|---:|---:|---:|---|
| tokyo_run1 | 8802 | 291 | 3.306% | 20 | 10 | 8.915 m | 189208.800-189214.200 (5.600 s) |
| tokyo_run2 | 7349 | 46 | 0.626% | 13 | 7 | 12.175 m | 178223.400-178224.600 (1.400 s) |
| tokyo_run3 | 11938 | 315 | 2.639% | 17 | 8 | 6.047 m | 181708.800-181747.600 (39.000 s) |
| nagoya_run1 | 6036 | 33 | 0.547% | 5 | 3 | 12.017 m | 551591.400-551593.400 (2.200 s) |
| nagoya_run2 | 4688 | 86 | 1.834% | 10 | 4 | 8.031 m | 556737.000-556746.200 (9.400 s) |
| nagoya_run3 | 2347 | 98 | 4.176% | 31 | 27 | 105.849 m | 554493.600-554495.200 (1.800 s) |

### Real-time discriminator percentiles

| Metric | Good FIX p50/p90/p95 | Wrong FIX p50/p90/p95 |
|---|---:|---:|
| ratio | 32.200/143.900/282.000 | 12.600/490.500/1929.900 |
| baseline_m | 2952.169/9441.529/9631.654 | 2974.659/9371.611/9371.641 |
| nsat | 21.000/30.000/32.000 | 14.000/27.000/27.000 |
| prefit_rms_m | 0.947/10.953/23.649 | 5.577/19.596/47.439 |
| prefit_max_m | 6.088/36.119/99.775 | 17.148/71.577/133.713 |
| post_rms_m | 0.489/0.745/0.825 | 0.675/0.923/1.065 |
| post_max_m | 2.686/2.956/2.979 | 2.792/2.976/2.988 |
| nis_per_obs | 0.346/2.053/4.801 | 0.978/4.867/9.379 |
| outliers | 4.000/18.000/26.000 | 18.000/42.000/53.000 |

### Deployable gate simulation

| Gate | Wrong caught | Wrong caught | Good FIX harmed | Good FIX harmed |
|---|---:|---:|---:|---:|
| nsat < 8 or (nsat <= 11 and ratio <= 15) | 0 | 0.000% | 9 | 0.022% |
| ratio < 6 | 196 | 22.555% | 4511 | 11.196% |
| ratio < 8 | 343 | 39.471% | 5969 | 14.815% |
| post_rms > 4 m | 0 | 0.000% | 0 | 0.000% |
| post_rms > 6 m | 0 | 0.000% | 0 | 0.000% |
| nis_per_obs > 20 | 26 | 2.992% | 641 | 1.591% |
| nis_per_obs > 50 | 11 | 1.266% | 220 | 0.546% |
| prefit_rms > 5 m | 451 | 51.899% | 6524 | 16.192% |
| baseline > 7000 m and post_rms > 4 m | 0 | 0.000% | 0 | 0.000% |

## online_consensus_kinematic_tokyo3_fgo

| Run | FIX epochs | Wrong FIX | Wrong/FIX | >5 m | >10 m | Wrong p95 | Longest wrong span |
|---|---:|---:|---:|---:|---:|---:|---|
| tokyo_run1 | 8792 | 283 | 3.219% | 12 | 2 | 4.429 m | 189208.800-189214.200 (5.600 s) |
| tokyo_run2 | 7339 | 39 | 0.531% | 6 | 0 | 6.745 m | 177620.600-177621.400 (1.000 s) |
| tokyo_run3 | 11784 | 161 | 1.366% | 9 | 0 | 6.047 m | 181708.800-181720.000 (11.400 s) |
| nagoya_run1 | 6036 | 33 | 0.547% | 5 | 3 | 12.017 m | 551591.400-551593.400 (2.200 s) |
| nagoya_run2 | 4684 | 82 | 1.751% | 6 | 0 | 5.191 m | 556737.000-556746.200 (9.400 s) |
| nagoya_run3 | 2328 | 70 | 3.007% | 4 | 0 | 7.194 m | 554711.200-554712.800 (1.800 s) |

### Real-time discriminator percentiles

| Metric | Good FIX p50/p90/p95 | Wrong FIX p50/p90/p95 |
|---|---:|---:|
| ratio | 32.200/143.900/281.900 | 17.400/844.900/5218.400 |
| baseline_m | 2952.170/9441.529/9631.654 | 3057.840/9371.630/9381.523 |
| nsat | 21.000/30.000/32.000 | 13.000/21.000/27.000 |
| prefit_rms_m | 0.943/10.747/23.637 | 3.157/41.996/49.175 |
| prefit_max_m | 6.053/35.991/99.775 | 12.046/117.622/133.779 |
| post_rms_m | 0.489/0.745/0.825 | 0.627/0.951/1.104 |
| post_max_m | 2.685/2.956/2.979 | 2.734/2.967/2.985 |
| nis_per_obs | 0.345/2.052/4.797 | 0.659/5.321/9.573 |
| outliers | 4.000/18.000/26.000 | 10.000/38.000/43.000 |

### Deployable gate simulation

| Gate | Wrong caught | Wrong caught | Good FIX harmed | Good FIX harmed |
|---|---:|---:|---:|---:|
| nsat < 8 or (nsat <= 11 and ratio <= 15) | 0 | 0.000% | 43 | 0.107% |
| ratio < 6 | 154 | 23.054% | 4521 | 11.220% |
| ratio < 8 | 207 | 30.988% | 5981 | 14.843% |
| post_rms > 4 m | 0 | 0.000% | 0 | 0.000% |
| post_rms > 6 m | 0 | 0.000% | 0 | 0.000% |
| nis_per_obs > 20 | 22 | 3.293% | 641 | 1.591% |
| nis_per_obs > 50 | 7 | 1.048% | 220 | 0.546% |
| prefit_rms > 5 m | 276 | 41.317% | 6505 | 16.143% |
| baseline > 7000 m and post_rms > 4 m | 0 | 0.000% | 0 | 0.000% |

## staged_integrity

| Run | FIX epochs | Wrong FIX | Wrong/FIX | >5 m | >10 m | Wrong p95 | Longest wrong span |
|---|---:|---:|---:|---:|---:|---:|---|
| tokyo_run1 | 8792 | 251 | 2.855% | 12 | 2 | 4.438 m | 189208.800-189214.200 (5.600 s) |
| tokyo_run2 | 7339 | 34 | 0.463% | 6 | 0 | 6.745 m | 177722.600-177723.400 (1.000 s) |
| tokyo_run3 | 11784 | 161 | 1.366% | 9 | 0 | 6.047 m | 181708.800-181720.000 (11.400 s) |
| nagoya_run1 | 6036 | 33 | 0.547% | 5 | 3 | 12.017 m | 551591.400-551593.400 (2.200 s) |
| nagoya_run2 | 4617 | 25 | 0.541% | 6 | 0 | 7.977 m | 556741.400-556742.800 (1.600 s) |
| nagoya_run3 | 2328 | 70 | 3.007% | 4 | 0 | 7.194 m | 554711.200-554712.800 (1.800 s) |

### Real-time discriminator percentiles

| Metric | Good FIX p50/p90/p95 | Wrong FIX p50/p90/p95 |
|---|---:|---:|
| ratio | 32.200/144.400/284.700 | 18.200/747.200/6564.200 |
| baseline_m | 2952.169/9441.529/9630.451 | 3047.457/8064.896/9536.737 |
| nsat | 21.000/30.000/32.000 | 13.000/25.000/27.000 |
| prefit_rms_m | 0.942/10.680/23.600 | 2.746/15.338/19.448 |
| prefit_max_m | 6.040/35.927/99.668 | 11.308/55.129/70.253 |
| post_rms_m | 0.489/0.745/0.825 | 0.622/0.959/1.105 |
| post_max_m | 2.685/2.956/2.979 | 2.737/2.970/2.986 |
| nis_per_obs | 0.345/2.057/4.806 | 0.626/5.397/8.969 |
| outliers | 4.000/18.000/26.000 | 9.000/38.000/49.000 |

### Deployable gate simulation

| Gate | Wrong caught | Wrong caught | Good FIX harmed | Good FIX harmed |
|---|---:|---:|---:|---:|
| nsat < 8 or (nsat <= 11 and ratio <= 15) | 0 | 0.000% | 43 | 0.107% |
| ratio < 6 | 129 | 22.474% | 4520 | 11.210% |
| ratio < 8 | 172 | 29.965% | 5979 | 14.828% |
| post_rms > 4 m | 0 | 0.000% | 0 | 0.000% |
| post_rms > 6 m | 0 | 0.000% | 0 | 0.000% |
| nis_per_obs > 20 | 17 | 2.962% | 646 | 1.602% |
| nis_per_obs > 50 | 7 | 1.220% | 220 | 0.546% |
| prefit_rms > 5 m | 219 | 38.153% | 6495 | 16.108% |
| baseline > 7000 m and post_rms > 4 m | 0 | 0.000% | 0 | 0.000% |
