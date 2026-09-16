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

## Rejected: FDE with exclusion on the global NIS gate

A row-exclusion FDE (when the global normalized-innovation gate would reject
the update, exclude the worst-normalized-innovation row and retry up to N
times) was implemented on top of the robust stack and reverted after it
degraded every run:

| run | arm | Official % | P95 m | Worst epoch m |
|---|---|---:|---:|---:|
| tokyo1 | robust | 77.71 | 5.70 | 93 |
| tokyo1 | + FDE | 64.08 | 14.03 | 93 |
| tokyo2 | robust | 84.10 | 3.47 | 63 |
| tokyo2 | + FDE | 63.02 | 4.06 | **146** |
| tokyo3 | robust | 76.16 | 5.95 | 251 |

The PPC RTK update routinely exceeds NIS/obs ≈ 3 without a fault, so the gate
fires on healthy epochs; excluding rows then destroys redundancy and degrades
the solution more than the fault it removes. A viable FDE needs a calibrated
fault-free innovation distribution (or a subset-consensus test) rather than a
fixed global threshold.

## Carrier-phase FGO vs IMU-aided FGO

`gnss_fgo` now accepts `--imu <imu.csv>` and builds a tightly-coupled
double-difference-carrier + IMU factor graph (GTSAM Pose3 fixed-lag). The
flag is opt-in: without `--imu` the output is byte-identical to before.
Related knobs: `--imu-lever-arm X Y Z`, `--imu-no-mounting`,
`--imu-fixed-lag <s>` (0 = batch), `--imu-noise-scale <s>`.

![PPC Tokyo run1 — carrier-phase vs IMU-aided FGO](ppc_fgo_carrier_vs_imu.png)

| arm (Tokyo run1) | P50 | P95 | max | RMS |
|---|---:|---:|---:|---:|
| GNSS carrier FGO (DD, base) | 1.66 | 7.49 | 29.3 | 3.55 |
| Carrier + IMU FGO (fixed-lag) | 2.67 | 12.28 | 29.2 | 5.26 |
| Carrier + IMU, IMU noise x1000 | 3.10 | 13.17 | **692** | 15.4 |
| GNSS/IMU FGO (no-base TDCP) | 4.60 | 18.29 | 45.9 | 10.3 |
| GNSS code FGO (no base) | 3.62 | 20.94 | 39.6 | 9.4 |

Carrier-phase ambiguity resolution dominates: the code-only FGO is ~2x
worse than the carrier FGO. The pure GNSS carrier arm runs the Eigen
batch solver while the IMU arm runs the GTSAM fixed-lag smoother (batch
GTSAM is ~15 min for 1000 epochs), so that cross-solver gap is not a
clean IMU ablation. The clean ablation holds the solver fixed:

![IMU ablation in the same fixed-lag FGO](ppc_fgo_imu_ablation.png)

Weakening the IMU (noise x1000) inside the same fixed-lag smoother raises
P50 2.67 -> 3.10 and injects a 692 m gross error, so the IMU factors are
load-bearing; the nominal weighting bounds the worst epoch to 29 m.

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

# Carrier-phase FGO arms (gnss_fgo)
P=data/PPC-Dataset/tokyo/run1
build/apps/gnss_fgo --obs $P/rover.obs --base $P/base.obs --nav $P/base.nav \
  --preset real-data-fixed --out output/ppc_fgo/tokyo1_carrier.pos
build/apps/gnss_fgo --obs $P/rover.obs --base $P/base.obs --nav $P/base.nav \
  --preset real-data-fixed --imu $P/imu.csv \
  --out output/ppc_fgo/tokyo1_carrier_imu.pos
build/apps/gnss_fgo --obs $P/rover.obs --base $P/base.obs --nav $P/base.nav \
  --preset real-data-fixed --imu $P/imu.csv --imu-noise-scale 1000 \
  --out output/ppc_fgo/tokyo1_carrier_imu_weak.pos
build/apps/gnss_fgo_imu_no_base --obs $P/rover.obs --imu $P/imu.csv --nav $P/base.nav \
  --all-epochs --out output/ppc_fgo/tokyo1_no_base_imu.csv

# FGO figure (3 arms) and IMU ablation
python3 scripts/plot_ppc_gnss_vs_fgo.py \
  --reference $P/reference.csv \
  --gnss-pos  output/ppc_fgo/tokyo1_carrier.pos \
  --imu-csv   output/ppc_fgo/tokyo1_no_base_imu.csv \
  --extra-pos output/ppc_fgo/tokyo1_carrier_imu.pos \
  --output docs/ppc_fgo_carrier_vs_imu.png \
  --gnss-label "GNSS carrier FGO (DD, base)" \
  --imu-label "GNSS/IMU FGO (no-base TDCP)" \
  --extra-label "Carrier + IMU FGO (base)" \
  --title "PPC Tokyo run1: carrier-phase vs IMU-aided FGO"
python3 scripts/plot_ppc_gnss_vs_fgo.py \
  --reference $P/reference.csv \
  --gnss-pos  output/ppc_fgo/tokyo1_carrier_imu.pos \
  --extra-pos output/ppc_fgo/tokyo1_carrier_imu_weak.pos \
  --output docs/ppc_fgo_imu_ablation.png --no-osm \
  --gnss-label "Carrier + IMU FGO (IMU nominal)" \
  --extra-label "Carrier + IMU FGO (IMU x1000 noise)" \
  --title "PPC Tokyo run1: IMU ablation in the same fixed-lag FGO"
```

## Scope

Development runs, in-sample, not a held-out or leaderboard claim. The
tight-coupling preset is validated for short-baseline urban runs and stays
opt-in.
