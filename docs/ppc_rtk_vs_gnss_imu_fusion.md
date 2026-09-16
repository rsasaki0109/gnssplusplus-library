# PPC: RTK only vs RTK + tightly-coupled GNSS/IMU

![PPC Tokyo run1 — RTK only vs RTK + tightly-coupled GNSS/IMU](ppc_rtk_vs_gnss_imu_fusion.png)

Comparison on the public **PPC 2024 Tokyo run1** dataset (11,845 matched
epochs). The two arms share the exact same command base; the only change is
the validated `--navi776-tc` tight-coupling preset. Scoring uses the shared
PPC helpers, so the numbers agree with `apps/gnss.py ppc-coverage-matrix` and
the README tables. Per the published navi776 sign-off protocol the compared
stream is the RTK solution (`--rtk-pos-out`), which the tight loop refines.

FIX integrity is reported after the deployable **sigma-demote** policy from
`configs/benchmarks/ppc_sigma_demote_nis2_ratio4.toml` (demote a FIXED epoch
to FLOAT when `ratio <= 4.0` or `NIS/obs > 2.0`), applied identically to both
arms via `scripts/plot_ppc_rtk_vs_imu_fusion.py --demote-max-ratio 4
--demote-nis-per-obs 2`.

| metric | RTK only | RTK + GNSS/IMU (tight) | delta |
|---|---:|---:|---:|
| Correct FIX (3D < 0.5 m) | 64.95 % | **68.42 %** | +3.47 pp |
| Wrong FIX / FIX | 8.66 % | **4.15 %** | **−4.51 pp** |
| FIX rate (post-demote) | 71.10 % | **71.38 %** | +0.28 pp |
| Official PPC score | 70.46 % | **74.76 %** | +4.30 pp |
| P50 horizontal | 0.031 m | **0.028 m** | −0.003 m |
| P95 horizontal | 6.869 m | **6.859 m** | −0.010 m |
| 3D < 0.5 m (all epochs) | 72.33 % | **73.44 %** | +1.11 pp |
| Worst epoch (FLOAT) | 124 m | **93 m** | −31 m |

Tight coupling roughly **halves the wrong-FIX rate** and raises the
distance-weighted official score, without giving up FIX rate. The residual
tail (worst single epoch ≈ 93 m) is a FLOAT/SPP outlier; shrinking it needs
the FGO-path fix-demote/surplus guards or the offline fixed-anchor FLOAT
bridge, which are separate from this KF-path preset.

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

# Figure (with the deployable sigma-demote policy)
python3 scripts/plot_ppc_rtk_vs_imu_fusion.py \
  --reference data/PPC-Dataset/tokyo/run1/reference.csv \
  --rtk-pos   output/ppc_rtk_vs_imu_fusion/tokyo1_canon/off_rtk.pos \
  --fusion-pos output/ppc_rtk_vs_imu_fusion/tokyo1_canon/on_rtk.pos \
  --output docs/ppc_rtk_vs_gnss_imu_fusion.png \
  --demote-max-ratio 4 --demote-nis-per-obs 2 \
  --title "PPC Tokyo run1 — RTK only vs RTK + tightly-coupled GNSS/IMU"
```

## Scope

One development run, in-sample, not a held-out or leaderboard claim. The
tight-coupling preset is validated for short-baseline urban runs and stays
opt-in. Extend to Tokyo run2/run3 and the Nagoya runs for a macro comparison.
