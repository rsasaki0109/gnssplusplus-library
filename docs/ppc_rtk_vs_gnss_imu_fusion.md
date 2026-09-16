# PPC: RTK only vs RTK + tightly-coupled GNSS/IMU

![PPC Tokyo run1 — RTK only vs RTK + tightly-coupled GNSS/IMU](ppc_rtk_vs_gnss_imu_fusion.png)

Comparison on the public **PPC 2024 Tokyo run1** dataset (11,845 matched
epochs). The two arms share the exact same command base; the only change is
the validated `--navi776-tc` tight-coupling preset. Scoring uses the shared
PPC helpers, so the numbers agree with `apps/gnss.py ppc-coverage-matrix` and
the README tables. Per the published navi776 sign-off protocol the compared
stream is the RTK solution (`--rtk-pos-out`), which the tight loop refines.

| metric | RTK only | RTK + GNSS/IMU (tight) | delta |
|---|---:|---:|---:|
| FIX rate | 76.58 % | **78.64 %** | +2.06 pp |
| P50 horizontal | 0.031 m | **0.028 m** | −0.003 m |
| P95 horizontal | 6.869 m | **6.859 m** | −0.010 m |
| 3D < 0.5 m | 72.33 % | **73.44 %** | +1.11 pp |
| Official PPC score | 70.46 % | **74.76 %** | +4.30 pp |
| Wrong FIX / FIX | 9.60 % | **8.92 %** | −0.68 pp |
| Max horizontal | 124 m | **93 m** | −31 m |

The gain is concentrated in the FIX-decision quality and the distance-weighted
official score rather than the P50/P95 percentiles: tight coupling recovers
correct fixes and removes the largest outliers (max-epoch error).

## Reproduce

```bash
export LD_LIBRARY_PATH=$HOME/.local/lib:${LD_LIBRARY_PATH:-}

# RTK only
build/apps/gnss_fuse \
  --data-dir data/PPC-Dataset/tokyo/run1 --lever-arm 0.31,0,-0.55 \
  --preset low-cost --ratio 2.4 --max-subset-ar-drop-steps 18 \
  --rtk-snr-weighting --no-arfilter \
  --rtk-pos-out output/ppc_rtk_vs_imu_fusion/tokyo1_canon/off_rtk.pos \
  --out        output/ppc_rtk_vs_imu_fusion/tokyo1_canon/off_fused.pos

# RTK + tightly-coupled GNSS/IMU
build/apps/gnss_fuse \
  --data-dir data/PPC-Dataset/tokyo/run1 --lever-arm 0.31,0,-0.55 \
  --preset low-cost --ratio 2.4 --max-subset-ar-drop-steps 18 \
  --rtk-snr-weighting --no-arfilter --navi776-tc \
  --rtk-pos-out output/ppc_rtk_vs_imu_fusion/tokyo1_canon/on_rtk.pos \
  --out        output/ppc_rtk_vs_imu_fusion/tokyo1_canon/on_fused.pos

# Figure
python3 scripts/plot_ppc_rtk_vs_imu_fusion.py \
  --reference data/PPC-Dataset/tokyo/run1/reference.csv \
  --rtk-pos   output/ppc_rtk_vs_imu_fusion/tokyo1_canon/off_rtk.pos \
  --fusion-pos output/ppc_rtk_vs_imu_fusion/tokyo1_canon/on_rtk.pos \
  --output docs/ppc_rtk_vs_gnss_imu_fusion.png \
  --title "PPC Tokyo run1 — RTK only vs RTK + tightly-coupled GNSS/IMU"
```

## Scope

One development run, in-sample, not a held-out or leaderboard claim. The
tight-coupling preset is validated for short-baseline urban runs and stays
opt-in. Extend to Tokyo run2/run3 and the Nagoya runs for a macro comparison.
