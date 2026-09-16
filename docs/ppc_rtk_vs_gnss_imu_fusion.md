# PPC: RTK only vs RTK + tightly-coupled GNSS/IMU

![PPC Tokyo run1 — RTK only vs RTK + tightly-coupled GNSS/IMU](ppc_rtk_vs_gnss_imu_fusion.png)

OpenStreetMap overlay: left is the full route, right zooms on the worst
RTK-only epoch (near Tokyo Station), where RTK-only cuts across the building
block while the fused track follows the road.

![PPC Tokyo run1 on OpenStreetMap](ppc_rtk_vs_gnss_imu_fusion_osm.png)

Comparison on the public **PPC 2024 Tokyo run1** dataset (11,845 matched
epochs). The two arms use the same command base; the fused arm adds the
validated `--navi776-tc` tight-coupling preset and the in-KF heavy-tail
(Student-t) robust front-end. Scoring uses the shared PPC helpers, so the
numbers agree with `apps/gnss.py ppc-coverage-matrix` and the README tables.
Per the published navi776 sign-off protocol the compared stream is the RTK
solution (`--rtk-pos-out`), which the tight loop refines.

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

## Three-run check (Tokyo run1/2/3)

Same command base, only `--navi776-tc` + Student-t changes. In-sample; not a
held-out claim.

| run | arm | Correct FIX % | Wrong FIX % | Official % | P95 m |
|---|---|---:|---:|---:|---:|
| tokyo1 | RTK | 64.95 | 8.66 | 70.46 | 6.869 |
| tokyo1 | fusion | **73.88** | **5.53** | **77.71** | **5.696** |
| tokyo2 | RTK | 68.19 | **1.74** | 83.38 | 3.398 |
| tokyo2 | fusion | **78.96** | 2.21 | **84.10** | 3.474 |
| tokyo3 | RTK | 69.18 | 3.02 | **78.21** | 8.545 |
| tokyo3 | fusion | **75.55** | **0.99** | 76.16 | **5.950** |
| **macro** | RTK | 67.44 | 4.47 | 77.35 | 6.271 |
| **macro** | fusion | **76.13** | **2.91** | **79.32** | **5.040** |

The stack raises correct-FIX on all three runs (+8.7 pp macro) and lowers
wrong-FIX and P95, but is not uniformly better per run (tokyo3 official and
tokyo2 wrong-FIX/P95 regress slightly).

## In-KF component: heavy-tail front-end

`--integrity-student-t-*` (IRLS Student-t / Huber / Laplacian) weights the RTK
measurement rows. It was previously assigned only inside the
`--library-fix-integrity-gate` block, so it was a silent no-op on this lane
(byte-identical output); it now applies to the primary RTK config whenever
selected. Default behaviour is unchanged.

## Rejected: turn-aware SPP stabilizer

`--rtk-single-stabilizer` extends the fixed-anchor stabilizer to SPP fallback
output and reduces the Tokyo run1 worst epoch (93 → 65 m), but it is **not
promoted**: on Tokyo run2 it replaced a moderate SPP error with a bad
quadratic extrapolation and raised the worst epoch from 63 m to 210 m. It
stays default-off and opt-in for research.

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

Development runs, in-sample, not a held-out or leaderboard claim. The
tight-coupling preset is validated for short-baseline urban runs and stays
opt-in.
