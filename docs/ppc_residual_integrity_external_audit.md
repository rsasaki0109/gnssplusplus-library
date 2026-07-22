# Staged integrity fixed-policy audit

Reference truth is used only for offline labels. Runtime policy inputs are POS status and RTK telemetry.

| Run | Selected | Wrong caught | Correct harmed | >5 m caught | >10 m caught | Wrong after |
|---|---:|---:|---:|---:|---:|---:|
| urban_odaiba_trimble | 0 | 0 | 0 | 0 | 0 | 0 |
| urban_shinjuku_trimble | 22 | 22 | 0 | 0 | 0 | 27 |

Total: selected 22, caught 22 wrong FIX, harmed 0 correct FIX; wrong FIX 49 -> 27.

The frozen runtime policy produced no false demotions on these explicit holdout runs.
