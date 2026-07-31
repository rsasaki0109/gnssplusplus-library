# PPC kinematic integrity six-fold LOO

Reference truth is used only for offline labels and candidate scoring. Runtime inputs are consecutive POS positions and timestamps.

| Held out | Selected jump / acceleration / hold | Train >10m caught | Train correct harm | Held >10m caught | Held correct harm |
|---|---:|---:|---:|---:|---:|
| tokyo_run1 | 12 m / 200 m/s2 / 3 | 20 | 12 | 6 | 3 |
| tokyo_run2 | 12 m / 200 m/s2 / 3 | 23 | 12 | 3 | 3 |
| tokyo_run3 | 12 m / 200 m/s2 / 5 | 22 | 14 | 8 | 13 |
| nagoya_run1 | 12 m / 200 m/s2 / 3 | 26 | 15 | 0 | 0 |
| nagoya_run2 | 12 m / 200 m/s2 / 3 | 26 | 13 | 0 | 2 |
| nagoya_run3 | 12 m / 200 m/s2 / 3 | 17 | 15 | 9 | 0 |

Production: **12 m / 200 m/s2 / 3 epochs**.

It catches **26** >10 m wrong FIX and **30** total wrong FIX while harming **15** correct FIX epochs.

## Advanced extension LOO

The primary production gate is fixed. Each fold selects only plateau continuation and the telemetry-backed secondary trigger from the other five runs.

| Held out | Plateau / max age / secondary | Train added >10m | Train added correct harm | Held added >10m | Held added correct harm |
|---|---:|---:|---:|---:|---:|
| tokyo_run1 | 0.1 m / 8 / prefit>5, ratio<=15, outliers>=10, nsat<=15 | 11 | 8 | 2 | 5 |
| tokyo_run2 | 0.1 m / 8 / prefit>5, ratio<=15, outliers>=10, nsat<=15 | 9 | 8 | 4 | 5 |
| tokyo_run3 | 0.1 m / 8 / prefit>5, ratio<=15, outliers>=10, nsat<=15 | 13 | 10 | 0 | 3 |
| nagoya_run1 | 0.1 m / 8 / prefit>5, ratio<=15, outliers>=10, nsat<=15 | 13 | 13 | 0 | 0 |
| nagoya_run2 | 0.1 m / 8 / prefit>5, ratio<=8, outliers>=10, nsat<=12 | 9 | 10 | 0 | 0 |
| nagoya_run3 | 0.05 m / 8 / prefit>5, ratio<=15, outliers>=10, nsat<=15 | 10 | 13 | 0 | 0 |

Advanced production catches **39** >10 m and **43** total wrong FIX while harming **25** correct FIX epochs. The extension alone adds 13 / 13 catches at a cost of 10 correct epochs.
