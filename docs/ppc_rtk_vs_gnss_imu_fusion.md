# PPC: RTK only vs RTK + tightly-coupled GNSS/IMU

![PPC Tokyo run1 — RTK only vs RTK + tightly-coupled GNSS/IMU](ppc_rtk_vs_gnss_imu_fusion.png)

OpenStreetMap overlay: left is the full route, right zooms on the worst
RTK-only epoch (near Tokyo Station), where RTK-only cuts across the building
block while the fused track follows the road.

![PPC Tokyo run1 on OpenStreetMap](ppc_rtk_vs_gnss_imu_fusion_osm.png)

Comparison on the public **PPC 2024 Tokyo run1** dataset (11,845 matched
epochs). The two arms use the same command base; the fused arm adds the
validated `--navi776-tc` tight-coupling preset and the in-KF heavy-tail
robust front-end. Scoring uses the shared PPC helpers, so the numbers agree
with `apps/gnss.py ppc-coverage-matrix` and the README tables. Per the
published navi776 sign-off protocol the compared stream is the RTK solution
(`--rtk-pos-out`), which the tight loop refines.

FIX integrity is reported after the deployable **sigma-demote** policy from
`configs/benchmarks/ppc_sigma_demote_nis2_ratio4.toml` (demote a FIXED epoch
to FLOAT when `ratio <= 4.0` or `NIS/obs > 2.0`), applied identically to both
arms via `scripts/plot_ppc_rtk_vs_imu_fusion.py --demote-max-ratio 4
--demote-nis-per-obs 2`.

| metric | RTK only | RTK + GNSS/IMU (tight, robust) | delta |
|---|---:|---:|---:|
| Correct FIX (3D < 0.5 m) | 64.95 % | **73.88 %** | **+8.93 pp** |
| Wrong FIX / FIX | 8.66 % | **5.53 %** | −3.13 pp |
| FIX rate (post-demote) | 71.10 % | **78.20 %** | +7.10 pp |
| Official PPC score | 70.46 % | **77.71 %** | **+7.25 pp** |
| P50 horizontal | 0.031 m | **0.026 m** | −0.005 m |
| P95 horizontal | 6.869 m | **5.696 m** | −1.17 m |
| 3D < 0.5 m (all epochs) | 72.33 % | **78.40 %** | +6.07 pp |
| Worst epoch | 124 m | **93 m** | −31 m |

## Ablation

`+TC` is `--navi776-tc` alone; `+robust` adds the Student-t front-end
(`--integrity-student-t-all-measurements
--integrity-student-t-degrees-of-freedom 3
--integrity-heavy-tail-activation-sigma 2.0`). All rows are scored with the
same sigma-demote policy.

| arm | FIX % | Correct FIX % | Wrong FIX % | Official % | P95 m |
|---|---:|---:|---:|---:|---:|
| RTK only | 71.10 | 64.95 | 8.66 | 70.46 | 6.869 |
| + TC | 71.38 | 68.42 | **4.15** | 74.76 | 6.859 |
| + TC + robust | **78.20** | **73.88** | 5.53 | **77.71** | **5.696** |

Tight coupling alone cuts the wrong-fix rate; the robust front-end then
converts the recovered epochs into correct fixes, raising coverage and the
official score.

## Heavy-tail front-end wiring fix

The Student-t / Huber / Laplacian front-end was only assigned to the primary
RTK config inside the `--library-fix-integrity-gate` block, so
`--integrity-student-t-*` was a silent no-op on the PPC/navi776 lane
(verified byte-identical output). `apps/native/gnss_fuse.cpp` now assigns the
front-end to the primary RTK config outside that gate; default behaviour is
unchanged (the base run is byte-identical).

## Reproduce

```bash
export LD_LIBRARY_PATH=$HOME/.local/lib:${LD_LIBRARY_PATH:-}

# RTK only
build/apps/gnss_fuse \
  --data-dir data/PPC-Dataset/tokyo/run1 --lever-arm 0.31,0,-0.55 \
  --preset low-cost --ratio 2.4 --max-subset-ar-drop-steps 18 \
  --rtk-snr-weighting --no-arfilter \
  --rtk-pos-out output/ppc_rtk_vs_imu_fusion/tokyo1_canon/off_rtk.pos

# RTK + tightly-coupled GNSS/IMU + robust front-end
build/apps/gnss_fuse \
  --data-dir data/PPC-Dataset/tokyo/run1 --lever-arm 0.31,0,-0.55 \
  --preset low-cost --ratio 2.4 --max-subset-ar-drop-steps 18 \
  --rtk-snr-weighting --no-arfilter --navi776-tc \
  --integrity-student-t-all-measurements \
  --integrity-student-t-degrees-of-freedom 3 \
  --integrity-heavy-tail-activation-sigma 2.0 \
  --rtk-pos-out output/ppc_rtk_vs_imu_fusion/tokyo1_robust/student_t3.pos

# Figure
python3 scripts/plot_ppc_rtk_vs_imu_fusion.py \
  --reference data/PPC-Dataset/tokyo/run1/reference.csv \
  --rtk-pos   output/ppc_rtk_vs_imu_fusion/tokyo1_canon/off_rtk.pos \
  --fusion-pos output/ppc_rtk_vs_imu_fusion/tokyo1_robust/student_t3.pos \
  --output docs/ppc_rtk_vs_gnss_imu_fusion.png \
  --demote-max-ratio 4 --demote-nis-per-obs 2 \
  --title "PPC Tokyo run1 — RTK only vs RTK + tightly-coupled GNSS/IMU"
```

## Scope

One development run, in-sample, not a held-out or leaderboard claim. The
tight-coupling preset is validated for short-baseline urban runs and stays
opt-in. Extend to Tokyo run2/run3 and the Nagoya runs for a macro comparison.
