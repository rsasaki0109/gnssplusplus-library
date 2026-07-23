# libgnss++

Modern C++20 GNSS toolkit for non-GUI positioning.

Native `SPP`, `RTK`, `PPP`, `CLAS/MADOCA`, `RTCM`, `UBX`, and direct `QZSS L6`
handling without an external RTKLIB runtime.

![UrbanNav Odaiba social card](docs/driving_odaiba_social_card.png)

## What You Get

- Solvers: `gnss spp`, `gnss solve`, `gnss ppp`
- Inputs/tools: RINEX, RTCM, UBX, SBF, NMEA, BINEX, QZSS L6
- Products: `SP3`, `CLK`, `IONEX`, `DCB`
- Extras: benchmarks, web dashboard, Python bindings, Docker, ROS 2 playback

![Feature overview](docs/libgnsspp_feature_overview.png)

## Results And Validation Status

| Area | Public comparison | Evidence / status |
|---|---|---|
| RTK | PPC Tokyo/Nagoya vs RTKLIB `demo5` | +17.0 pp positioning, +28.1 pp official score, -11.96 m P95 H delta |
| GNSS/IMU FGO | PPC Tokyo vs `tightly-coupled-gnss-imu-fgo` | Higher <50 cm fraction (avg +5.3 pp) and fix-rate (avg +7.9 pp) on all 3 runs; fixed-only RMS also wins 2 of 3 runs |
| CLAS PPP | Six PPC Tokyo/Nagoya runs vs MRTKLIB CLAS | 23.645% aggregate FIX, 0.593 m FIX RMS2D, 36.510 m all-solution RMS2D, and 0 FIX epochs above 3 m across 58,258 scored epochs |
| Urban RTK | UrbanNav Tokyo Odaiba vs RTKLIB `demo5` | More fixes, lower Hp95/Vp95; `--preset odaiba` closes Hmed |
| SPP | PPC SPP adaptive robust + policy gate | No P95 regression with <=1 pp positioning drop |

### PPC 2024 goal matrix vs Kaiyodai and gici-open

The audited KF/FGO selected profile clears the distance-weighted PPC public
target at **78.8455%** (published target: **78.7%**). It also exceeds the
Tokyo 1 public FIX rate (**80.861%** vs **80.8%**). A separate FIX-target
profile clears Nagoya 1 by the narrow measured margin **85.100974%** vs
**85.1%**, with 0.913% Wrong FIX/FIX and 1.460 m P95 horizontal error. The
public FIX targets come from the [Kaiyodai RTK paper](https://www.denshi.e.kaiyodai.ac.jp/wp-content/uploads/pdf/content/2024okada,sasaki,ando.pdf),
and the PPC score target from the [Turing tight-coupling slides](https://www.denshi.e.kaiyodai.ac.jp/wp-content/uploads/2025/01/Turing-Inc.-Tight-coupling-Factor-Graph-%E4%BA%95%E4%B8%8A%E6%A7%98-%E5%9C%A7%E7%B8%AE.pdf).

| Run | libgnss++ FIX | gici-open FIX | Wrong FIX/FIX | libgnss++ correct FIX/ref | gici-open correct FIX/ref | 50 cm/ref | libgnss++ official | gici-open official | P95 H |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Tokyo 1 | **80.861%** | 46.472% | 2.855% | **71.467%** | 43.528% | 80.286% | 79.458% | **80.263%** | 2.082 m |
| Tokyo 2 | **82.340%** | 76.938% | 0.463% | **79.827%** | 74.462% | 88.395% | 88.696% | **90.652%** | 1.604 m |
| Tokyo 3 | **78.461%** | 73.347% | 1.366% | **75.962%** | 71.923% | 86.295% | **85.969%** | 83.787% | 1.671 m |
| Nagoya 1 | **83.659%** | 67.812% | 0.547% | **78.460%** | 60.005% | 85.845% | 65.201% | **70.851%** | 1.332 m |
| Nagoya 2 | **55.553%** | 39.988% | 0.541% | **48.587%** | 35.330% | 60.787% | **55.529%** | 39.847% | 18.144 m |
| Nagoya 3 | **44.761%** | 21.399% | 3.007% | **43.415%** | 18.285% | 61.354% | **72.336%** | 33.495% | 1.908 m |
| **Macro mean** | **70.939%** | 54.326% | **1.463%** | **66.287%** | 50.589% | 77.160% | **74.532%** | 66.483% | 4.457 m |

![PPC libgnss++ and gici-open comparison](docs/ppc_libgnss_gici_comparison.png)

![PPC public targets](docs/ppc_public_targets.png)

The XY view below shows where each selected solution is FIXED or FLOAT. Green
points are correct FIX epochs, red crosses are FIX epochs with 3D error above
0.5 m, orange points are FLOAT, and the light-gray line is the reference
trajectory.

Before the final confidence gate, a truth-free wrong-basin escape replaces 35
Nagoya 2 FIX positions with its independent TC-FGO candidate and emits them as
FLOAT only when baseline prefit RMS exceeds 8 m and at least 45 observations
were suppressed. Nagoya 2 FIX errors above 10 m fall from 36 to 4 and those
above 5 m from 42 to 10, while its P95 horizontal error improves from 19.092 m
to 18.144 m.

Nagoya 3 now uses a causal, truth-free KF/FGO consensus state machine. The FGO
shadow never replaces the KF position: it only quarantines a FIX label while
the estimators disagree, and recovery requires consecutive agreement. A
runtime-only provisional-recovery gate retains five healthy FIX epochs without
promoting the joint anchor early. This removes 16 wrong FIX labels and 15
errors above 10 m without losing a correct FIX epoch.

The final confidence layer demotes FIX to FLOAT when fewer than 8 satellites
are used, or when at most 11 satellites are used and the AR ratio is at most
15. Strong same-epoch telemetry may exonerate the boundary case only with at
least 11 satellites, prefit RMS at most 0.5 m, and NIS/observation at most 0.2.
A causal kinematic guard then quarantines the current and next two epochs after
a jump above 12 m with acceleration above 200 m/s2. A bounded plateau extension
continues quarantine for at most eight epochs while jumps stay below 0.1 m. A
secondary trigger requires a jump above 5 m, acceleration above 100 m/s2,
prefit RMS above 5 m, ratio at most 10, at least 10 suppressed outliers, and at
most 13 satellites. It changes no positions and reads no reference truth.

Tokyo 3 additionally replays two overlapping, independently initialized
GNSS/IMU FGO shadow windows through the consensus state machine. FIX is
quarantined after sustained KF/FGO separation above 5 m when the shadow has
GDOP at most 4, DDPR RMS at most 40 m, and at least 8 satellites. The FGO
position is never substituted. Soft primary telemetry cannot start an
unrecoverable quarantine while the independent estimator is unavailable; hard
primary suspects remain fail-closed. The two-window replay removes 142 wrong
FIX labels at a cost of five correct FIX labels and finishes in NORMAL state.

The final staged residual guard adds at most seven epochs (1.4 seconds at
5 Hz) of output latency. It confirms eight consecutive FIX epochs only when
prefit RMS exceeds 40 m, ratio is at most 15, at least 12 observations are
suppressed, and suppressed observations comprise at least half of `RTKObs`.
A separate single-epoch gate requires prefit RMS at least 40 m and at most 14
satellites. These rules consume emitted RTK telemetry only and change status,
not position. They remove 57 wrong FIX labels at a cost of ten correct FIX
labels on PPC.

The official score reaches **78.845491%**. Across six runs, aggregate wrong FIX
falls from 869 to **574**, wrong FIX above 5 m from 96 to **42**, and wrong FIX
above 10 m from 59 to **5**. Correct FIX/reference remains above its 66.230%
floor and Tokyo 1 remains above its public FIX target. In six-fold leave-one-run-out selection,
five folds select the production 12 m / 3-epoch primary policy and the remaining
fold selects a 5-epoch hold. All extension folds select an eight-epoch maximum
plateau age; the secondary thresholds vary and provide held-out catches only on
two runs, so that component remains exploratory. See the
[kinematic integrity LOO report](docs/ppc_kinematic_integrity_loo.md). This is
a run-level robustness check, not an external-dataset generalization claim. The full severity audit is
in [PPC FIX integrity audit](docs/ppc_fix_integrity_audit.md). The
[wrong-FIX event ledger](docs/ppc_wrong_fix_event_ledger.md) groups all 574
residual wrong-FIX epochs into 188 contiguous events and records truth-free
runtime fingerprints for root-cause and regression work.
The [Nagoya 3 root-cause report](docs/ppc_nagoya3_wrong_fix_root_cause.md)
shows that the largest event is an overconfident float-KF basin followed by
integer certification, rather than an inherited hold-FIX failure.
The [online consensus design](docs/ppc_online_consensus_design.md) specifies the
truth-free KF/FGO quarantine and recovery state machine that follows from it.
The frozen staged policy was also replayed on full UrbanNav Odaiba/Shinjuku
Trimble rover solutions. It demoted 22 of 49 FIX errors above 2 m, harmed zero
correct FIX epochs, and consumed no reference truth at runtime; see the
[external integrity audit](docs/ppc_residual_integrity_external_audit.md).
The machine-readable requirement audit is tracked in
[PPC goal completion audit](docs/ppc_goal_completion_audit.md).

![PPC selected XY trajectories by FIX status](docs/ppc_kf_fgo_fix_status_xy.png)

`gici-open` was reproduced from commit
`e7666110a88d22e08aad24345a253564af9b8024` on its `forppc2024` branch and
evaluated from exported NMEA with the same six references and metric code.
The six-run libgnss++ FIX macro is **+16.613 pp** above that reproduction, and
its macro Wrong FIX/FIX is **1.025 pp lower**.
The GPL-3.0 program remains an external executable: no GICI source is copied,
linked, or distributed here, so libgnss++ remains MIT.

The position selectors use candidate status/ratio/satellite/residual telemetry
and candidate-to-current separation only. They preserve the baseline epoch
grid and telemetry; the wrong-basin and consensus escapes intentionally
replace position and status for their selected epochs. The final
satellite-count/ratio confidence gate changes only
the emitted status, from FIX to FLOAT; the online consensus and kinematic
guards likewise retain the primary position. Reference truth is used only after
output generation for scoring. Thresholds were tuned on this public benchmark,
so these results are an in-sample benchmark rather than a held-out
generalization claim. Definitions, commands, paths, and the machine-readable
audit are in [PPC reproduction commands](docs/ppc_reproduction.md) and
[`docs/ppc_kf_fgo_goal_metrics.json`](docs/ppc_kf_fgo_goal_metrics.json).

### PPC RTK vs RTKLIB demo5

These are public PPC Tokyo/Nagoya moving-RTK replays using the same
rover/base/nav observations for libgnss++ and RTKLIB `demo5`.

<!-- PPC_COVERAGE_MATRIX:START -->
| Run | gnssplusplus Positioning | RTKLIB Positioning | Delta | gnssplusplus Fix | RTKLIB Fix | PPC official score | RTKLIB official score | Official delta | P95 H delta |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Tokyo run1 | **90.0%** | 66.3% | **+23.7 pp** | **54.4%** | 30.5% | **34.9%** | 0.0% | **+34.9 pp** | +3.39 m |
| Tokyo run2 | **95.3%** | 84.3% | **+11.0 pp** | **64.1%** | 27.6% | **69.0%** | 16.9% | **+52.1 pp** | -18.51 m |
| Tokyo run3 | **95.7%** | 93.1% | **+2.5 pp** | **63.0%** | 40.5% | **60.6%** | 35.6% | **+25.0 pp** | -0.24 m |
| Nagoya run1 | **88.8%** | 65.8% | **+23.0 pp** | **64.5%** | 33.8% | **49.5%** | 22.4% | **+27.1 pp** | -23.78 m |
| Nagoya run2 | **85.6%** | 69.8% | **+15.8 pp** | **51.4%** | 18.8% | **20.9%** | 11.0% | **+9.9 pp** | -27.24 m |
| Nagoya run3 | **93.8%** | 67.7% | **+26.1 pp** | **27.1%** | 13.9% | **27.4%** | 7.6% | **+19.7 pp** | -5.37 m |

Across these six public runs, the coverage profile averages **+17.0 pp**
Positioning-rate lead, **+28.1 pp** PPC official-score lead, and
**-11.96 m** P95 horizontal-error delta versus RTKLIB `demo5`.
<!-- PPC_COVERAGE_MATRIX:END -->

![PPC RTK coverage scorecard](docs/ppc_rtk_demo5_scorecard.png)

### GNSS/IMU Tightly-Coupled FGO vs tightly-coupled-gnss-imu-fgo

GTSAM-based fixed-lag factor-graph backend (`FGOBackend::GTSAM`) with
tightly-coupled IMU, multi-frequency DD RTK, per-epoch partial LAMBDA,
fix-and-hold, and an urban-robustness stack (CMC multipath screening,
CP-hold recovery, DDPR-anchored resets, FDE, elevation-dependent sigma).
Public PPC Tokyo moving-RTK replays with the dataset's tactical-grade IMU,
versus [inuex35/tightly-coupled-gnss-imu-fgo](https://github.com/inuex35/tightly-coupled-gnss-imu-fgo)
(Python + GTSAM) on the same rover/base/IMU data:

| Run | libgnss++ <50cm | Reference <50cm | libgnss++ fix | Reference fix | libgnss++ fixed RMS | Reference fixed RMS |
|---|---:|---:|---:|---:|---:|---:|
| Tokyo run1 | **56.9%** | 56.7% | **50.0%** | 49.5% | **0.655 m** | 0.815 m |
| Tokyo run2 | **80.6%** | 69.9% | **71.5%** | 60.8% | **0.261 m** | 0.277 m |
| Tokyo run3 | **72.8%** | 67.9% | **71.8%** | 59.4% | 0.272 m | **0.211 m** |

![Tokyo run1 GNSS/IMU FGO](docs/gnss_imu_fgo_tokyo_run1.png)
![Tokyo run2 GNSS/IMU FGO](docs/gnss_imu_fgo_tokyo_run2.png)
![Tokyo run3 GNSS/IMU FGO](docs/gnss_imu_fgo_tokyo_run3.png)

libgnss++ beats the reference on <50 cm fraction and fix-rate on all three
runs, and on fixed-only RMS on two of three runs (run3's fixed RMS remains
behind the reference, over the largest fixed population of the three: 10973
epochs). Every feature is opt-in and the library is unchanged when built
without GTSAM. Reproduce with `gnss_fgo_parity` (requires a GTSAM build) and
the shipping preset:

```
--imu <run>/imu.csv --fixed-lag 5 --multi-freq --partial-ar --hold \
--elev-mask 25 --snr-mask 30 --imu-preset-tactical --cmc --cmc-level 0.75 \
--cp-hold --cp-hold-res 2.0 --exc-recovery --ddpr-anchor --fde --varerr \
--fix-demote --fix-demote-dist 5 --fix-demote-res 25 --fix-demote-posthold 5
```

#### Surplus-satellite rescue (opt-in)

LAMBDA candidates that fall short of the ratio gate can be rescued by an
independent integrity test: DD carrier observations that were excluded from
the fix (FDE quarantine, CMC exclusion, partial-AR drops) are re-differenced
against an alternate reference satellite at the candidate fixed position and
checked against a PDOP-scaled nearest-integer aperture, with a
GQEBR→GQEB→GQER→GQE→GQB→GQ constellation fallback. Counterfactual auditing of
the demotion guard on the same runs also moved `--fix-demote-res` from 25 to
40 for this configuration (25 demoted mostly sub-0.5 m fixes on run1/run3
while run2's genuine wrong-basin cluster sits far above 40). Tokyo PPC
full-run results with the fixed-lag-QR configuration:

```
--imu <run>/imu.csv --fixed-lag 1 --fixed-lag-qr --hold \
--imu-ratio-aperture --fixed-history-dr --cmc --cmc-level 0.75 \
--fde --fde-cp-quarantine --varerr --integ-cov 1e-6 \
--fix-demote --fix-demote-dist 5 --fix-demote-res 40 --fix-demote-posthold 5 \
--multi-freq --partial-ar --elev-mask 25 --snr-mask 30 \
--imu-preset-tactical --exc-recovery --ddpr-anchor \
--surplus-validation --surplus-validation-min-n 3 \
--surplus-validation-aperture-lt1 0.15 --surplus-validation-aperture-1to2 0.3 \
--surplus-validation-aperture-gt2 0.45
```

| Run | Fix-rate | Fixed-only RMS | Rescue contribution |
|---|---:|---:|---:|
| tokyo/run1 | 67.23% | 0.797 m | +2.99 pp, RMS improved |
| tokyo/run2 | 77.71% | 0.742 m | RMS 6.07 m → 0.74 m with res 40 |
| tokyo/run3 | 75.14% | 1.054 m | RMS 1.96 m → 1.05 m, >5 m fixes 38 → 13 |

The rescued-only population on run1 has 0.565 m RMS (n=693). The companion
`--surplus-validation-veto` mode (re-testing established fixes) is implemented
but counterfactually false-alarm dominated — leave it off. For repeated runs
on identical inputs, `--problem-cache <file>` caches the parsed observations
and built DD problem (~2× faster validation loops with an /O2 build).

### Moving CLAS PPP vs MRTKLIB

The current moving-data gate replays all six public PPC Tokyo/Nagoya runs at
5 Hz from QZSS L6 corrections with kinematic dynamics enabled. Scoring matches
solutions to the PPC reference, discards the first 60 matched epochs per run,
and defines TTFF as the first run of at least 30 consecutive FIX epochs.

MRTKLIB columns are the published v0.4.2 results from the
[CLAS benchmark article](https://zenn.dev/hatognss/articles/7a54dd82606faf).
The native results below are the complete-run outputs after the float-recovery
fix landed in #345; each run has 100% interval coverage and at least 99.92%
epoch coverage.

| Run | libgnss++ FIX | MRTKLIB FIX | FIX RMS2D* | MRTKLIB RMS2D† | All RMS2D* | FLOAT RMS2D* | SINGLE RMS2D* | max FIX* | >3 m FIX* |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Tokyo 1 | **9.702%** | 4.900% | 0.453 m | 0.747 m | 41.713 m | 16.619 m | 80.050 m | 1.533 m | **0** |
| Tokyo 2 | 19.219% | **21.700%** | 0.387 m | 0.514 m | 25.922 m | 17.896 m | 45.343 m | 1.036 m | **0** |
| Tokyo 3 | **37.387%** | 7.400% | 0.327 m | 0.801 m | 35.286 m | 19.727 m | 88.586 m | 2.826 m | **0** |
| Nagoya 1 | **36.605%** | 17.000% | 0.888 m | 1.105 m | 57.257 m | 8.064 m | 119.220 m | 1.501 m | **0** |
| Nagoya 2 | 23.117% | **23.400%** | 0.783 m | 1.119 m | 25.841 m | 16.480 m | 40.344 m | 2.486 m | **0** |
| Nagoya 3 | 4.865% | **6.300%** | 0.959 m | 0.318 m | 13.554 m | 13.795 m | 14.040 m | 1.494 m | **0** |
| **Six-run aggregate** | **23.645%** | — | **0.593 m** | — | **36.510 m** | **16.777 m** | **70.316 m** | **2.826 m** | **0** |

\* libgnss++ precision uses PPC vehicle truth transformed to the antenna phase
center. † The published MRTKLIB precision uses the unmodified PPC reference,
so cross-solver precision columns are contextual rather than reference-identical.

![PPC six-run moving CLAS metric comparison](docs/ppc_clas_full_comparison.png)

Across 58,258 scored epochs, native CLAS produced 13,775 FIX epochs. A finite
SPP candidate rejected by the chi-square/redundancy validation still remains
excluded from ordinary filter admission, cold starts, and AR. For catastrophic
FLOAT/SPP disagreement above 250 m only, it can continue the counted MRTKLIB
`maxdiffp` recovery path. On Tokyo 2 this moves the bad-seed recovery from TOW
177750.0 to 177747.4 (311.6 m to 5.4 m), 0.8 s before the MRTKLIB recovery at
177748.2. The six-run all-solution RMS2D is 36.510 m; FLOAT and SINGLE RMS2D are
16.777 m and 70.316 m respectively.

| Complete trajectories | Horizontal error and FIX epochs |
|---|---|
| ![PPC six-run CLAS trajectories](docs/ppc_clas_full_trajectories.png) | ![PPC six-run CLAS errors](docs/ppc_clas_full_errors.png) |

See the [complete table](docs/ppc_clas_full_table.md),
[machine-readable metrics](docs/ppc_clas_full_metrics.json), and
[PPC CLAS validation note](docs/ppc_clas_validation.md) for definitions and
reproduction details.

## Quick Start

Choose the entrypoint that matches your job:

- [Robotics quick start](docs/robotics_quickstart.md): RTK replay, local web
  inspection, ROS2 receiver launch, and rosbag capture.
- [Research quick start](docs/research_quickstart.md): repeatable sign-off
  runs, profile comparisons, Python inspection, and artifact layout.
- [Dataset gallery](docs/dataset_gallery.md): current public dataset lanes and
  the adapter contract for adding more.

Build:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
python3 apps/gnss.py doctor
python3 apps/gnss.py ros2-doctor --device /dev/ttyUSB0
python3 apps/gnss.py ros2-bag-doctor --bag /path/to/rosbag --summary-json output/ros2_bag_doctor_summary.json
python3 apps/gnss.py field-report --out output/field_report.md
python3 apps/gnss.py robotics-smoke --profile realtime
```

`ros2-bag-doctor` reads sqlite ROS2 bags for message-level rates/gaps. For MCAP
bags it uses the optional Python `mcap` package when available, and otherwise
falls back to MCAP `metadata.yaml` for topic presence, counts, and duration.
`gnss web` auto-discovers `output/field_report*.json` and shows the report
links, Markdown preview, setup/ROS2/bag/smoke status, and next debug commands.

Run a solution:

```bash
python3 apps/gnss.py spp \
  --obs data/rover_static.obs \
  --nav data/navigation_static.nav \
  --out output/spp_solution.pos
```

RTK example:

```bash
python3 apps/gnss.py solve \
  --rover data/rover_kinematic.obs \
  --base data/base_kinematic.obs \
  --nav data/navigation_kinematic.nav \
  --mode kinematic \
  --out output/rtk_solution.pos
```

Run the web UI:

```bash
python3 apps/gnss.py web --port 8085
```

Then open `http://127.0.0.1:8085`.

List commands:

```bash
python3 apps/gnss.py commands
python3 apps/gnss.py commands --json
python3 apps/gnss.py commands --query ppp --limit 10
```

## Docker

```bash
docker build -t libgnsspp:latest .
docker run --rm -it -p 8085:8085 -v "$PWD:/workspace" \
  libgnsspp:latest web --host 0.0.0.0 --port 8085 --root /workspace
```

## Benchmarks

- [Benchmarks](docs/benchmarks.md)
- [Validation](docs/validation.md)
- [PPC reproduction commands](docs/ppc_reproduction.md)
- [SPP accuracy notes](docs/references/spp-accuracy-improvement.md)

## Docs

- <https://rsasaki0109.github.io/gnssplusplus-library/>
- [Documentation index](docs/index.md)
- [Quick start](docs/quickstart.md)
- [Robotics quick start](docs/robotics_quickstart.md)
- [Research quick start](docs/research_quickstart.md)
- [Dataset gallery](docs/dataset_gallery.md)
- [Interfaces](docs/interfaces.md)
- [Architecture](docs/architecture.md)
- [Reference analyses](docs/references/index.md)
- [Contributing](CONTRIBUTING.md)

## Install

```bash
cmake --install build --prefix /opt/libgnsspp
/opt/libgnsspp/bin/gnss --help
```

## Tests

```bash
ctest --test-dir build --output-on-failure
```

## License

MIT License. See [LICENSE](LICENSE). Permissive third-party attributions and
the separate GPL-only competitor-benchmark boundary are documented in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
