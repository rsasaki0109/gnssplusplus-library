# Benchmarks

The current gnssplusplus `develop` branch leads RTKLIB `demo5` on PPC
positioned-epoch precision and Fix rate with no Phase 2 opt-in flags.
Positioning rate is tracked separately because no-solution gaps still matter;
the PPC coverage profile keeps valid SPP/float fallback epochs and now exceeds
RTKLIB `demo5` on Positioning rate for all six public Tokyo/Nagoya runs.
PPC is the primary public RTK benchmark here because it bundles survey-grade
receiver observations, reference-station observations, broadcast navigation
data, and reliable trajectory truth. It is not used as a proprietary
receiver-engine comparison. Treat the UrbanNav Odaiba snapshot below as a
Tier-1 public smoke/regression run.

All runs below use `--mode kinematic --preset low-cost --match-tolerance-s 0.25`.
The coverage profile additionally uses `--no-arfilter --no-kinematic-post-filter`
plus the default low-speed non-FIX drift guard, SPP height-step guard, and
FLOAT bridge-tail guard.

![PPC RTK coverage scorecard](ppc_rtk_demo5_scorecard.png)

## Public Moving-RTK Benchmark Matrix

The public-data strategy is intentionally multi-dataset. A single UrbanNav
run is useful because it exposes u-blox and Trimble rover observations with an
independent Applanix reference, but it is only one environment. Use
`gnss public-rtk-benchmarks` to keep adapter status and caveats visible:

```bash
python3 apps/gnss.py public-rtk-benchmarks --format markdown
```

| Profile | Status | Role | Reference | Receiver artifacts | Adapter | Caveat |
|---|---|---|---|---|---|---|
| [PPC-Dataset Tokyo/Nagoya](https://github.com/taroz/PPC-Dataset) | primary-public-rtk-signoff | survey-grade receiver observation sign-off | `reference.csv` trajectory truth for Tokyo/Nagoya runs | Septentrio mosaic-X5 rover RINEX plus Trimble Alloy/NetR9 base RINEX/nav | native `ppc-demo` and `ppc-rtk-signoff` layout with receiver hardware provenance | survey-grade observations and reference truth are bundled; proprietary receiver-engine solution is not treated as the benchmark target |
| [UrbanNav Tokyo Odaiba/Shinjuku](https://github.com/IPNL-POLYU/UrbanNavDataset) | wired-path-overrides | Tier-1 public smoke | Applanix POS LV620 `reference.csv` | u-blox F9P rover RINEX plus Trimble NetR9 rover/base RINEX | `ppc-rtk-signoff` path overrides with `--commercial-rover` | two Tokyo runs; Trimble observations are solved by libgnss++, not the Trimble RTK engine |
| [smartLoc urban GNSS](https://www.tu-chemnitz.de/projekt/smartLoc/gnss_dataset.html.en) | receiver-fix-signoff | urban NLOS stress | NovAtel SPAN differential RTK/IMU reference | u-blox EVK-M8T mass-market raw/fix data plus NLOS labels | `smartloc-adapter` exports receiver/raw data; `smartloc-signoff` gates receiver-fix metrics | solver sign-off still needs compatible nav/base inputs beyond the public receiver-fix path |
| [Google Smartphone Decimeter Challenge](https://www.ion.org/gnss/googlecompetition.cfm) | candidate | phone-grade stress | precise ground truth for raw GNSS and IMU traces | Android raw GNSS measurements and sensor logs | needs smartphone measurement converter; not a commercial RTK receiver path | useful stress data, but phone antenna/clock behavior is a different receiver class |
| [Ford Highway Driving RTK](https://arxiv.org/abs/2010.01774) | candidate | large-scale highway coverage | INS coupled with survey-grade GNSS receivers | production automotive GNSS over long highway drives | needs Ford log normalizer and highway-specific thresholds | excellent scale, but not an urban canyon RTK receiver comparison |
| [Oxford RobotCar RTK ground truth](https://arxiv.org/abs/2002.10152) | candidate | long-term localization coverage | post-processed raw GPS/IMU/static-base centimeter ground truth | RobotCar traversals with reference localization products | needs RobotCar reference mapper and observation availability check | strong localization benchmark, but indirect for commercial RTK receiver claims |

## PPC Tokyo Precision Profile

PPC receiver hardware provenance is emitted in every `ppc-demo` summary under
`receiver_observation_provenance`. Tokyo uses a Septentrio mosaic-X5 rover with
a Trimble AT1675 antenna and a Trimble Alloy / Zephyr Geodetic 2 reference
station. Nagoya uses a Septentrio mosaic-X5 rover with a Trimble Zephyr 3 Rover
antenna and a Trimble NetR9 / Zephyr 3 Base reference station. The field
`receiver_engine_solution_available` is deliberately `false`; this benchmark is
about solving survey-grade receiver observations against reference truth, not
about matching a proprietary receiver RTK engine.

| Run  | gnssplusplus Fix / rate | RTKLIB Fix / rate | Hmed (m)              | Vp95 (m)               |
|------|------------------------:|------------------:|:---------------------:|:----------------------:|
| run1 | **3572 / 81.26%**       | 2418 / 30.52%     | **0.037** vs 1.567 (42×) | **1.259** vs 36.703 (29×) |
| run2 | **4674 / 80.12%**       | 2127 / 27.58%     | **0.016** vs 0.835 (52×) | **0.313** vs 42.624 (136×) |
| run3 | **7516 / 86.84%**       | 5778 / 40.55%     | **0.012** vs 0.666 (56×) | **0.137** vs 24.521 (179×) |

This fixed-output table is the precision-oriented view. The coverage table
below is the sign-off view for no-solution gaps and fallback-positioned epochs.

## PPC Coverage Profile

| Run | gnssplusplus Positioning | RTKLIB Positioning | Delta | gnssplusplus Fix | RTKLIB Fix | 3D <= 50 cm / ref delta | P95 H delta |
|---|---:|---:|---:|---:|---:|---:|---:|
| Tokyo run1 | **86.2%** | 66.3% | **+19.9 pp** | **48.6%** | 30.5% | **+35.6 pp** | -6.97 m |
| Tokyo run2 | **95.3%** | 84.3% | **+11.0 pp** | **60.8%** | 27.6% | **+39.9 pp** | -18.89 m |
| Tokyo run3 | **96.0%** | 93.1% | **+2.9 pp** | **60.3%** | 40.5% | **+23.6 pp** | -0.69 m |
| Nagoya run1 | **87.9%** | 65.8% | **+22.1 pp** | **60.3%** | 33.8% | **+32.8 pp** | -22.63 m |
| Nagoya run2 | **86.2%** | 69.8% | **+16.5 pp** | **40.3%** | 18.8% | **+20.0 pp** | -27.16 m |
| Nagoya run3 | **94.6%** | 67.7% | **+26.9 pp** | **19.7%** | 13.9% | **+9.4 pp** | -5.54 m |

Across these six public runs, the coverage profile averages **+16.5 pp**
Positioning-rate lead, **+26.9 pp** 3D<=50cm/reference-score lead, and
**-13.65 m** P95 horizontal-error delta versus RTKLIB `demo5`.

### Tokyo run1 coverage-quality split

Tokyo run1's low-speed non-FIX drift guard removes 320 bounded fallback epochs,
and the SPP height-step guard removes another 30 vertical-spike fallback epochs.
The default FLOAT bridge-tail guard then removes 147 more FLOAT epochs in the
remaining low-speed tail, using horizontal FIX-anchor speed so vertical anchor
noise does not create false motion. Together these guards turn the previous
P95H regression into a **6.97 m** P95H lead, keep Positioning at **86.2%**
(**+19.9 pp** over RTKLIB), and keep the 3D<=50cm/reference score unchanged.
The full machine-readable reports are
`ppc_tokyo_run1_coverage_quality.json` and
`ppc_tokyo_run1_coverage_bad_segments.csv`; the bad-segment CSV includes
status counts, adjacent FIX-anchor gap/speed, solution path length, and
FIX-anchor bridge residuals for continued FLOAT-tail design.

| Status | Epochs | P50 H | P95 H | 3D <= 50 cm / reference | P95H exceedance share |
|---|---:|---:|---:|---:|---:|
| FIXED | 5005 | 0.03 m | 1.57 m | 33.1% | 5.2% |
| FLOAT | 5044 | 2.26 m | 26.61 m | 2.5% | 87.6% |
| SPP | 253 | 4.71 m | 32.28 m | 0.0% | 7.2% |

![PPC Tokyo run1 coverage quality by status](ppc_tokyo_run1_coverage_quality.png)

Across the six PPC Tokyo/Nagoya runs, the default FLOAT bridge-tail guard
rejects 148 epochs total: 147 on Tokyo run1, 1 on Tokyo run3, and 0 on the
other four runs. The previous 3D-speed prototype also rejected 115 Nagoya run3
FLOAT epochs with low horizontal anchor speed; the shipped guard uses
horizontal anchor speed and avoids that positioning-rate loss.

PPC Tokyo run3 is also checked visually as a 2D status-colored trajectory.
The replay uses GNSS observations only, with no IMU input. The coverage profile
retains valid SPP/float fallback epochs instead of dropping them with the
precision-oriented output filter.

PPC2024's official score is a distance ratio with 3D error <= 50 cm; the
published first-place result was 78.7% Public / 85.6% Private in
[PPC2024 results](https://taroz.net/data/PPC2024_results.pdf). The table above
is an open-run replay metric, not an official Kaggle submission.

![PPC Tokyo run3 RTK trajectory by solution status](ppc_tokyo_run3_rtk_trajectory_status.png)

## PPC Nagoya (same preset)

| Run  | Fix delta    | rate delta    | Hmed delta     |
|------|-------------:|--------------:|---------------:|
| run1 | **+1743**    | **+58.03 pp** | **9× better**  |
| run2 | **+1735**    | **+64.00 pp** | **10× better** |
| run3 | **+154**     | **+50.16 pp** | **44× better** |

## UrbanNav Tokyo Odaiba Snapshot

Dataset: [UrbanNav Tokyo Odaiba](https://github.com/IPNL-POLYU/UrbanNavDataset)  
Comparison baseline: [RTKLIB](https://github.com/tomojitakasu/RTKLIB)

Current checked-in snapshot (kinematic, low-cost preset). This validates one
public urban slice and should be read together with the matrix above:

| Config                                                                     | Fix              | Rate        | Hmed (m)              | Hp95 (m)    | Vp95 (m)    |
|----------------------------------------------------------------------------|-----------------:|------------:|:---------------------:|:-----------:|:-----------:|
| RTKLIB demo5                                                               | 595              | 7.22%       | **0.707**             | 27.878      | 45.212      |
| libgnss++ default                                                          | **1268** (+673)  | **36.98%**  | 1.707                 | **19.585**  | **25.495**  |
| libgnss++ `--enable-wide-lane-ar --wide-lane-threshold 0.10`               | 818 (+223)       | 33.65%      | **0.799** (9 cm gap)  | **19.971**  | **26.429**  |

| RTKLIB 2D | libgnss++ 2D |
|---|---|
| ![RTKLIB 2D trajectory](driving_odaiba_comparison_rtklib_2d.png) | ![libgnss++ 2D trajectory](driving_odaiba_comparison_libgnss_2d.png) |

More artifacts:

- [Odaiba social card](driving_odaiba_social_card.png)
- [Full comparison figure](driving_odaiba_comparison.png)
- [Scorecard](driving_odaiba_scorecard.png)
- Optional side-by-side PPP reference: [JAXA-SNU/MALIB](https://github.com/JAXA-SNU/MALIB)
- Additional low-cost GNSS reference: [rtklibexplorer/RTKLIB](https://github.com/rtklibexplorer/RTKLIB)

## PPC-Dataset

External dataset source: [taroz/PPC-Dataset](https://github.com/taroz/PPC-Dataset)

Example:

```bash
python3 apps/gnss.py ppc-demo \
  --dataset-root /datasets/PPC-Dataset \
  --city tokyo \
  --run run1 \
  --solver rtk \
  --require-realtime-factor-min 1.0 \
  --summary-json output/ppc_tokyo_run1_rtk_summary.json

python3 apps/gnss.py ppc-rtk-signoff \
  --dataset-root /datasets/PPC-Dataset \
  --city tokyo \
  --rtklib-bin /path/to/rnx2rtkp \
  --summary-json output/ppc_tokyo_run1_rtk_signoff.json

python3 apps/gnss.py ppc-coverage-matrix \
  --dataset-root /datasets/PPC-Dataset \
  --rtklib-root output/benchmark \
  --summary-json output/ppc_coverage_matrix/summary.json \
  --markdown-output output/ppc_coverage_matrix/table.md \
  --require-positioning-delta-min 0 \
  --require-score-3d-50cm-ref-delta-min 0 \
  --require-p95-h-delta-max 0
```

`ppc-rtk-signoff` is the fixed-threshold path for Tokyo/Nagoya quality and
runtime checks, with optional RTKLIB delta gates. Add `--commercial-pos` with
a normalized receiver CSV or `.pos` file to summarize a commercial receiver
against the same PPC `reference.csv`; `--commercial-matched-csv` writes the
per-epoch commercial receiver matches.

For the `urban-nav-tokyo` matrix row, the same path can compare low-cost and
commercial receiver observations against the independent Applanix reference.
UrbanNav-TK-20181219 includes `/Odaiba` and `/Shinjuku` runs with
`rover_ublox.obs`, `rover_trimble.obs`, `base_trimble.obs`, `base.nav`, and
`reference.csv` artifacts. Use the low-cost rover as the primary `--rover` and
solve the Trimble NetR9 rover through the localized commercial path:

```bash
python3 apps/gnss.py ppc-rtk-signoff \
  --run-dir /datasets/UrbanNav-TK-20181219/Odaiba \
  --city tokyo \
  --rover /datasets/UrbanNav-TK-20181219/Odaiba/rover_ublox.obs \
  --base /datasets/UrbanNav-TK-20181219/Odaiba/base_trimble.obs \
  --nav /datasets/UrbanNav-TK-20181219/Odaiba/base.nav \
  --reference-csv /datasets/UrbanNav-TK-20181219/Odaiba/reference.csv \
  --commercial-rover /datasets/UrbanNav-TK-20181219/Odaiba/rover_trimble.obs \
  --commercial-preset survey \
  --commercial-label trimble_net_r9 \
  --commercial-matched-csv output/urban_nav_tokyo_odaiba_trimble_matches.csv \
  --summary-json output/urban_nav_tokyo_odaiba_rtk_summary.json
```

This records `commercial_receiver.source =
libgnss_solved_receiver_observations`, so the comparison is between receiver
hardware observation streams solved by libgnss++ rather than the proprietary
Trimble RTK engine.

## smartLoc Adapter

smartLoc is the first non-UrbanNav candidate promoted into a sign-off boundary.
The source `NAV-POSLLH.csv` contains the NovAtel-derived ground truth columns
and the u-blox EVK-M8T receiver fix columns on the same time scale. Export
those into the existing comparison contracts:

```bash
python3 apps/gnss.py smartloc-adapter \
  --input-url https://www.tu-chemnitz.de/projekt/smartLoc/gnss_dataset/berlin/scenario1/berlin1_potsdamer_platz.zip \
  --reference-csv output/smartloc_berlin1_reference.csv \
  --receiver-csv output/smartloc_berlin1_ublox.csv \
  --raw-csv output/smartloc_berlin1_rawx.csv \
  --obs-rinex output/smartloc_berlin1_rover.obs \
  --summary-json output/smartloc_berlin1_adapter_summary.json
```

The resulting `reference.csv` can be read by `ppc-demo` style metric helpers,
and `receiver-csv` can be passed as `--commercial-pos --commercial-format csv`
for receiver side-by-side summaries. `raw-csv` preserves the RXM-RAWX
measurement rows with NLOS labels, and `obs-rinex` emits a minimal RINEX 3.04
rover observation file using `C1C/L1C/D1C/S1C` fields. This still is not a
complete smartLoc solver sign-off by itself because broadcast nav/base inputs
must be supplied separately.
When `--input` or direct CSV paths are omitted, `smartloc-adapter` can also
download the public zip through `--input-url` into `output/downloads`.

For the closed receiver-fix path, use the wrapper:

```bash
python3 apps/gnss.py smartloc-signoff \
  --input-url https://www.tu-chemnitz.de/projekt/smartLoc/gnss_dataset/berlin/scenario1/berlin1_potsdamer_platz.zip \
  --output-dir output/smartloc_berlin1 \
  --raw-max-epochs 200 \
  --require-matched-epochs-min 100 \
  --require-p95-h-max 10.0
```

This emits the same adapter artifacts plus `smartloc_signoff_summary.json` with
`receiver_fix` metrics and optional raw-observation provenance. It deliberately
reports `solver_signoff_available = false` until compatible broadcast
navigation/base inputs are provided for the RINEX rover observations.
If `--input` or direct CSV paths are omitted, the wrapper downloads the public
zip into `output/downloads` and records both `input_url` and `downloaded_input`
in the summary JSON.

The wrapper also emits `solver_preflight`, which inventories the public zip
before making solver claims. For the Berlin Potsdamer Platz scenario, preflight
finds the generated rover RINEX OBS and the bundled `gbm19001.sp3.Z` precise
orbit, but no broadcast navigation RINEX, base-station observations, or precise
clock product. Therefore RTK solver sign-off, SPP smoke, and PPP smoke remain
blocked by input availability instead of being silently skipped. Use
`--require-solver-inputs-available` when a dataset variant is expected to carry
all RTK solver inputs.
