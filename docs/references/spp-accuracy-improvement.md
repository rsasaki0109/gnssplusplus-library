# SPP Accuracy Improvement Plan

This note summarizes concrete ways to improve `libgnss++` single point
positioning after checking the current `SPPProcessor`, related in-tree PPP/CLAS
support, upstream OSS, and recent positioning papers.

## Current Baseline

The native SPP path already has:

- iterative weighted least squares,
- broadcast satellite clocks/orbits,
- Sagnac correction,
- Klobuchar ionosphere correction,
- Saastamoinen troposphere correction,
- elevation weighting,
- multi-GNSS clock-bias groups,
- broadcast group delay handling,
- unhealthy satellite filtering.

The weakest parts are:

- only one primary code observable per satellite is used,
- `enable_outlier_detection` is configured but not used in the solver loop,
- the weight model is only `sin(el)^2`,
- `gnss spp` exposes almost no solver/model options,
- precise products, IONEX, DCB, and SSR/OSR corrections are available elsewhere
  in the tree but are not consumable by the SPP path,
- no code-carrier smoothing or time-series filter exists in the SPP path.

## Public PPC Real-Data Baseline

The public [PPC-Dataset](https://github.com/taroz/PPC-Dataset) is now usable as
the first real driving-data SPP benchmark. The checked local copy contains all
Tokyo/Nagoya `rover.obs`, `base.nav`, `base.obs`, and `reference.csv` files.

Tokyo `run1` SPP baseline command:

```bash
./build-codex/apps/gnss_spp \
  --obs data/PPC-Dataset/tokyo/run1/rover.obs \
  --nav data/PPC-Dataset/tokyo/run1/base.nav \
  --out output/ppc_tokyo_run1_spp_full.pos \
  --summary-json output/ppc_tokyo_run1_spp_full_summary.json

python3 apps/gnss_ppc_demo.py \
  --dataset-root data/PPC-Dataset \
  --city tokyo \
  --run run1 \
  --solver spp \
  --use-existing-solution \
  --out output/ppc_tokyo_run1_spp_full.pos \
  --spp-summary-json output/ppc_tokyo_run1_spp_full_summary.json \
  --summary-json output/ppc_tokyo_run1_spp_full_ppc_metrics.json
```

Current Tokyo `run1` broadcast-SPP result:

- valid solutions: `11846 / 11928` epochs (`99.31%` SPP availability),
- PPC reference matches: `11846` epochs,
- median horizontal error: `1.796600 m`,
- p95 horizontal error: `20.468689 m`,
- max horizontal error: `92.934515 m`,
- median absolute up error: `2.084398 m`,
- p95 absolute up error: `47.970680 m`,
- mean satellites: `21.615820`,
- mean residual RMS: `3.238973 m`,
- SPP QC: `24072` robust outlier rejections and `410` RAIM/FDE rejections.

The first 1000 epochs are much cleaner (`1.407 m` horizontal RMSE and
`1.780 m` 3D RMSE), so the full-run p95/max errors are the better urban-canyon
stress target for robust weighting, NLOS rejection, and product-aided SPP.

Optional Huber-style robust WLS weighting can now be enabled without changing
the default broadcast-SPP path:

```bash
python3 apps/gnss_ppc_demo.py \
  --dataset-root data/PPC-Dataset \
  --city tokyo \
  --run run1 \
  --solver spp \
  --max-epochs -1 \
  --robust-weighting \
  --robust-threshold-sigma 2.5 \
  --robust-min-weight 0.1 \
  --out output/ppc_tokyo_run1_spp_robust_full.pos \
  --spp-summary-json output/ppc_tokyo_run1_spp_robust_full_spp_summary.json \
  --summary-json output/ppc_tokyo_run1_spp_robust_full_metrics.json
```

Tokyo `run1` robust-weighted SPP result:

- median horizontal error: `1.735529 m` (`1.796600 m` baseline),
- p95 horizontal error: `20.089085 m` (`20.468689 m` baseline),
- p95 absolute up error: `47.152582 m` (`47.970680 m` baseline),
- mean residual RMS: `2.858469 m` (`3.238973 m` baseline),
- robust downweighted measurements: `22`,
- minimum robust weight factor observed: `0.716293`.

The first full-run robust sweep showed the best balanced Huber setting at
`--robust-threshold-sigma 2.0 --robust-min-weight 0.05`: it kept all `11846`
valid epochs, reduced median horizontal error to `1.721111 m`, reduced p95
horizontal error to `20.021339 m`, and reduced mean residual RMS to
`2.787969 m`. A more aggressive `1.5` sigma threshold improved horizontal p95
to `19.797067 m`, but worsened p95 absolute up error to `49.465066 m`.

An optional kinematic position-jump gate is now available for SPP output
screening:

```bash
python3 apps/gnss_ppc_demo.py \
  --dataset-root data/PPC-Dataset \
  --city tokyo \
  --run run1 \
  --solver spp \
  --max-epochs -1 \
  --robust-weighting \
  --robust-threshold-sigma 2.0 \
  --robust-min-weight 0.05 \
  --max-position-jump-rate-mps 50 \
  --max-position-jump-min-m 30 \
  --out output/ppc_tokyo_run1_spp_robust_t2p0_jump_full.pos \
  --spp-summary-json output/ppc_tokyo_run1_spp_robust_t2p0_jump_full_spp_summary.json \
  --summary-json output/ppc_tokyo_run1_spp_robust_t2p0_jump_full_metrics.json
```

Tokyo `run1` robust plus jump-gated SPP result:

- valid solutions: `11591 / 11928` epochs (`97.17%` SPP availability),
- PPC reference matches: `11591` epochs,
- position-jump gate rejections: `255`,
- median horizontal error: `1.696003 m`,
- p95 horizontal error: `16.357185 m`,
- max horizontal error: `41.353500 m`,
- median absolute up error: `1.932640 m`,
- p95 absolute up error: `43.624871 m`,
- mean residual RMS: `2.760463 m`.

Thresholds can be explored without rerunning RINEX processing by using the
post-hoc sweep helper on an existing SPP `.pos`:

```bash
python3 apps/gnss.py ppc-spp-jump-sweep \
  --reference-csv data/PPC-Dataset/tokyo/run1/reference.csv \
  --pos output/ppc_tokyo_run1_spp_robust_t2p0_w0p05_full.pos \
  --rates-mps 25,50,75,100,150,200 \
  --min-jumps-m 10,20,30,40,50 \
  --summary-json output/ppc_tokyo_run1_spp_robust_t2p0_jump_sweep.json \
  --csv output/ppc_tokyo_run1_spp_robust_t2p0_jump_sweep.csv
```

The sweep identified `25 m/s` with a `30 m` minimum jump as a stronger PPC
Tokyo `run1` tradeoff. The full solver run with that setting produced:

- valid solutions: `11527 / 11928` epochs (`96.64%` SPP availability),
- position-jump gate rejections: `319`,
- median horizontal error: `1.692454 m`,
- p95 horizontal error: `15.043657 m`,
- max horizontal error: `41.353500 m`,
- median absolute up error: `1.922436 m`,
- p95 absolute up error: `42.249105 m`,
- mean residual RMS: `2.758781 m`.

The same sweep helper can estimate short-gap bridge recovery without rerunning
RINEX processing:

```bash
python3 apps/gnss.py ppc-spp-jump-sweep \
  --reference-csv data/PPC-Dataset/tokyo/run1/reference.csv \
  --pos output/ppc_tokyo_run1_spp_robust_t2p0_w0p05_full.pos \
  --rates-mps 25,50,75,100,150,200 \
  --min-jumps-m 10,20,30,40,50 \
  --bridge-max-gap-s 5 \
  --bridge-max-anchor-speed-mps 10 \
  --summary-json output/ppc_tokyo_run1_spp_robust_t2p0_jump_bridge_sweep.json \
  --csv output/ppc_tokyo_run1_spp_robust_t2p0_jump_bridge_sweep.csv \
  --filtered-pos-out output/ppc_tokyo_run1_spp_robust_t2p0_jump25_30_bridge_filtered.pos \
  --filtered-rate-mps 25 \
  --filtered-min-jump-m 30
```

For the `25 m/s`, `30 m` candidate, bridge recovery filled `94` of the `319`
rejected epochs using accepted anchors no more than `5 s` apart with
anchor-to-anchor speed below `10 m/s`. That raised positioning rate from
`96.452180%` to `97.239277%`, while keeping p95 horizontal error at
`15.111 m` and max horizontal error at `41.353 m`.

The filtered `.pos` can be passed back through the PPC scoring path:

```bash
python3 apps/gnss.py ppc-demo \
  --run-dir data/PPC-Dataset/tokyo/run1 \
  --solver spp \
  --use-existing-solution \
  --out output/ppc_tokyo_run1_spp_robust_t2p0_jump25_30_bridge_filtered.pos \
  --summary-json output/ppc_tokyo_run1_spp_robust_t2p0_jump25_30_bridge_filtered_metrics.json
```

That re-read produced `11621` matched epochs (`97.238725%` positioning rate),
`1.696595 m` median horizontal error, `15.111390 m` p95 horizontal error, and
`41.930951 m` p95 absolute up error.

Multiple SPP candidates can be compared with one command:

```bash
python3 apps/gnss.py ppc-spp-compare \
  --reference-csv data/PPC-Dataset/tokyo/run1/reference.csv \
  --solution baseline=output/ppc_tokyo_run1_spp_full.pos \
  --solution robust=output/ppc_tokyo_run1_spp_robust_t2p0_w0p05_full.pos \
  --solution robust_bridge=output/ppc_tokyo_run1_spp_robust_t2p0_jump25_30_bridge_filtered.pos \
  --summary-json output/ppc_tokyo_run1_spp_compare_summary.json \
  --csv output/ppc_tokyo_run1_spp_compare.csv \
  --matched-csv output/ppc_tokyo_run1_spp_compare_matches.csv \
  --png output/ppc_tokyo_run1_spp_compare.png \
  --title "PPC Tokyo run1 SPP accuracy candidates"
```

This writes a compact summary CSV, per-epoch matched error CSV, JSON payload,
and a PNG with trajectory, horizontal-error time series, CDF, and p95 bars. On
Tokyo `run1`, robust Huber weighting alone reduced p95 horizontal error from
`20.468689 m` to `20.021339 m`; the robust plus jump-gate bridge candidate
reduced it to `15.111390 m` while keeping `97.238725%` positioning rate.

The same fixed robust setting (`threshold=2.0`, `min_weight=0.05`) and
Tokyo-selected jump/bridge candidate (`25 m/s`, `30 m`, `5 s`,
`10 m/s`) were then checked on PPC Nagoya. Results are mixed:

| PPC run | candidate | positioning | median H | p95 H | delta p95 H |
| --- | ---: | ---: | ---: | ---: | ---: |
| Tokyo run1 | baseline | `99.121412%` | `1.796600 m` | `20.468689 m` | `0.000000 m` |
| Tokyo run1 | robust | `99.121412%` | `1.721111 m` | `20.021339 m` | `-0.447350 m` |
| Tokyo run1 | robust_bridge | `97.238725%` | `1.696595 m` | `15.111390 m` | `-5.357299 m` |
| Nagoya run1 | baseline | `98.653771%` | `2.800185 m` | `11.024481 m` | `0.000000 m` |
| Nagoya run1 | robust | `98.653771%` | `2.753815 m` | `10.375934 m` | `-0.648547 m` |
| Nagoya run1 | robust_bridge | `97.137629%` | `2.729376 m` | `9.609146 m` | `-1.415335 m` |
| Nagoya run2 | baseline | `99.735478%` | `1.765405 m` | `27.198508 m` | `0.000000 m` |
| Nagoya run2 | robust | `99.735478%` | `1.681284 m` | `23.374231 m` | `-3.824277 m` |
| Nagoya run2 | robust_bridge | `97.143159%` | `1.657559 m` | `22.444808 m` | `-4.753700 m` |
| Nagoya run3 | baseline | `100.000000%` | `3.590495 m` | `14.429884 m` | `0.000000 m` |
| Nagoya run3 | robust | `100.000000%` | `3.638911 m` | `14.894125 m` | `+0.464241 m` |
| Nagoya run3 | robust_bridge | `98.481061%` | `3.553679 m` | `14.838970 m` | `+0.409086 m` |

This says robust residual downweighting and jump/bridge cleanup are useful on
the outlier-heavy Tokyo `run1` and Nagoya `run2`, mildly useful on Nagoya
`run1`, and counterproductive on Nagoya `run3`. The next implementation should
not simply flip these options on by default; it needs an adaptive enablement
rule based on observed tail errors, rejected-measurement pressure, or
innovation consistency.

Adaptive Huber activation was added as `gnss_spp --adaptive-robust-weighting`.
It keeps Huber weighting disabled unless the current WLS iteration has enough
large normalized residuals:

```bash
python3 apps/gnss.py spp \
  --obs data/PPC-Dataset/nagoya/run2/rover.obs \
  --nav data/PPC-Dataset/nagoya/run2/base.nav \
  --adaptive-robust-weighting \
  --robust-threshold-sigma 2.0 \
  --robust-min-weight 0.05 \
  --adaptive-robust-activation-threshold-sigma 3.0 \
  --adaptive-robust-min-tail-measurements 2 \
  --adaptive-robust-min-tail-fraction 0.08 \
  --out output/ppc_nagoya_run2_spp_adaptive3_robust_t2p0_w0p05_full.pos \
  --summary-json output/ppc_nagoya_run2_spp_adaptive3_robust_t2p0_w0p05_full_spp_summary.json
```

With `3.0 sigma`, `2` tail residuals, and `8%` tail fraction, the PPC results
are:

| PPC run | baseline p95 H | always robust p95 H | adaptive robust p95 H | adaptive delta |
| --- | ---: | ---: | ---: | ---: |
| Tokyo run1 | `20.468689 m` | `20.021339 m` | `20.278148 m` | `-0.190541 m` |
| Nagoya run1 | `11.024481 m` | `10.375934 m` | `10.471696 m` | `-0.552785 m` |
| Nagoya run2 | `27.198508 m` | `23.374231 m` | `24.015681 m` | `-3.182827 m` |
| Nagoya run3 | `14.429884 m` | `14.894125 m` | `14.429884 m` | `0.000000 m` |

The adaptive rule gives up some improvement versus always-on robust weighting,
but it avoids the Nagoya `run3` regression. A more aggressive `2.0 sigma`
activation recovered more Nagoya `run2` improvement (`23.480 m` p95 H), but
regressed Nagoya `run3` (`14.706 m` p95 H), so `3.0 sigma` is the safer first
default for adaptive robust mode.

The post-hoc jump/bridge sweep can now enforce availability-style policy
constraints while selecting the filtered `.pos`. This keeps the p95 cleanup
from silently trading away too many matched epochs:

```bash
python3 apps/gnss.py ppc-spp-jump-sweep \
  --reference-csv data/PPC-Dataset/nagoya/run2/reference.csv \
  --pos output/ppc_nagoya_run2_spp_adaptive3_robust_t2p0_w0p05_full.pos \
  --rates-mps 25,50,75,100,150,200 \
  --min-jumps-m 10,20,30,40,50 \
  --bridge-max-gap-s 5 \
  --bridge-max-anchor-speed-mps 10 \
  --max-positioning-drop-pct 1.0 \
  --summary-json output/ppc_nagoya_run2_spp_adaptive3_jump_policy1_bridge_sweep.json \
  --csv output/ppc_nagoya_run2_spp_adaptive3_jump_policy1_bridge_sweep.csv \
  --filtered-pos-out output/ppc_nagoya_run2_spp_adaptive3_jump_policy1_bridge_filtered.pos
```

With adaptive robust input and a `1.0` percentage-point positioning drop limit,
the selected jump/bridge candidates were:

| PPC run | policy positioning | policy median H | policy p95 H | p95 delta | positioning delta |
| --- | ---: | ---: | ---: | ---: | ---: |
| Tokyo run1 | `98.150782%` | `1.745896 m` | `18.843820 m` | `-1.624869 m` | `-0.970630%` |
| Nagoya run1 | `97.686577%` | `2.777175 m` | `9.892842 m` | `-1.131639 m` | `-0.967194%` |
| Nagoya run2 | `98.783198%` | `1.681489 m` | `23.388052 m` | `-3.810456 m` | `-0.952280%` |
| Nagoya run3 | `99.134782%` | `3.529821 m` | `14.405460 m` | `-0.024424 m` | `-0.865218%` |

This combination keeps all four checked runs at or below baseline p95
horizontal error while respecting the explicit availability-drop bound.

The selected policy rows can be collapsed into a cross-run CI/report artifact
from the sweep JSONs:

```bash
python3 apps/gnss.py ppc-spp-policy-report \
  --sweep tokyo_run1=output/ppc_tokyo_run1_spp_adaptive3_jump_policy1_bridge_sweep.json \
  --sweep nagoya_run1=output/ppc_nagoya_run1_spp_adaptive3_jump_policy1_bridge_sweep.json \
  --sweep nagoya_run2=output/ppc_nagoya_run2_spp_adaptive3_jump_policy1_bridge_sweep.json \
  --sweep nagoya_run3=output/ppc_nagoya_run3_spp_adaptive3_jump_policy1_bridge_sweep.json \
  --summary-json output/ppc_spp_adaptive3_policy1_report.json \
  --csv output/ppc_spp_adaptive3_policy1_report.csv \
  --max-p95-delta-m 0 \
  --max-positioning-drop-pct 1.0
```

This report records each run's baseline row, selected policy row, p95
horizontal-error delta, positioning-rate drop, selected jump thresholds, and
bridge/rejection counts. The threshold flags make the command fail if a future
sweep no longer improves or ties baseline p95, or if it exceeds the allowed
availability drop.

For repeated PPC checks, the sweep/report/compare steps can be run as one
suite. Each `--run` uses `label=run_dir_or_reference_csv,input_pos,baseline_pos`;
the suite writes one sweep, one filtered `.pos`, and one compare bundle per run,
then writes the cross-run policy report:

```bash
python3 apps/gnss.py ppc-spp-policy-suite \
  --manifest-json configs/ppc_spp_policy_suite.adaptive3_policy1.example.json
```

The manifest can be validated without running the sweeps:

```bash
python3 apps/gnss.py ppc-spp-policy-suite \
  --manifest-json configs/ppc_spp_policy_suite.adaptive3_policy1.example.json \
  --dry-run
```

The same suite can also be spelled out directly on the command line:

```bash
python3 apps/gnss.py ppc-spp-policy-suite \
  --run tokyo_run1=data/PPC-Dataset/tokyo/run1,output/ppc_tokyo_run1_spp_adaptive3_robust_t2p0_w0p05_full.pos,output/ppc_tokyo_run1_spp_full.pos \
  --run nagoya_run1=data/PPC-Dataset/nagoya/run1,output/ppc_nagoya_run1_spp_adaptive3_robust_t2p0_w0p05_full.pos,output/ppc_nagoya_run1_spp_full.pos \
  --run nagoya_run2=data/PPC-Dataset/nagoya/run2,output/ppc_nagoya_run2_spp_adaptive3_robust_t2p0_w0p05_full.pos,output/ppc_nagoya_run2_spp_full.pos \
  --run nagoya_run3=data/PPC-Dataset/nagoya/run3,output/ppc_nagoya_run3_spp_adaptive3_robust_t2p0_w0p05_full.pos,output/ppc_nagoya_run3_spp_full.pos \
  --output-dir output/ppc_spp_adaptive3_policy1_suite \
  --prefix adaptive3_policy1 \
  --rates-mps 25,50,75,100,150,200 \
  --min-jumps-m 10,20,30,40,50 \
  --bridge-max-gap-s 5 \
  --bridge-max-anchor-speed-mps 10 \
  --max-positioning-drop-pct 1.0 \
  --max-p95-delta-m 0
```

CLI flags override manifest top-level settings such as `output_dir`, `prefix`,
thresholds, and `skip_compare` when a one-off variant is needed.
The suite summary copies the policy report's `checks`, `worst_p95_h_delta_m`,
`worst_positioning_drop_pct`, and selected per-run policy rows to top-level
fields so CI can read one JSON file.
`gnss artifact-manifest` now also picks up these `*_suite_summary.json` files
by default and records the policy report, per-run compare artifacts, worst p95
delta, and worst positioning-rate drop:

```bash
python3 apps/gnss.py artifact-manifest \
  --output output/artifact_manifest.json
```

The local web UI exposes the same bundle under the dedicated
`PPC SPP policy suites` section after the manifest has been generated:

```bash
python3 apps/gnss.py web --port 8085
```

## Immediate Fix Already Made

`models::ionoDelayKlobuchar()` used elevation as `rad / pi / pi` in the
earth-centered-angle and obliquity-factor terms. The Klobuchar model expects
elevation in semicircles, i.e. `rad / pi`.

The fixed test case now expects `7.691021650363509 m`; the old expression
produced about `12.04 m` for the same geometry.

The SPP solver now also:

- respects `ProcessorConfig::elevation_mask` instead of a hard-coded 5 degree
  mask,
- respects the ionosphere/troposphere atmosphere switches in the WLS path,
- uses `enable_outlier_detection` for robust residual outlier rejection,
- runs optional leave-one-out RAIM/FDE when enough redundant observations exist,
- computes SPP QC diagnostics for residual RMS, max residual, GDOP gate,
  reduced chi-square, rejected satellites, and FDE attempts,
- uses an explicit pseudorange variance model with code, elevation, SNR/CN0,
  ionosphere, and troposphere terms,
- can form an optional dual-frequency ionosphere-free code combination
  (`gnss_spp --ionosphere-free`) when primary and secondary code observations
  are available for the same satellite,
- can load optional SP3 orbit and RINEX CLK clock products
  (`gnss_spp --sp3 ... --clk ...`) for product-aided SPP satellite states,
- can load optional CSV SSR orbit/clock/code-bias products
  (`gnss_spp --ssr ...`) for augmented code-only SPP corrections,
- can load optional IONEX TEC maps and Bias-SINEX/DCB products
  (`gnss_spp --ionex ... --dcb ...`) for product-aided SPP corrections,
- can optionally use Huber-style robust WLS residual weighting
  (`gnss_spp --robust-weighting`) for heavy-tailed urban pseudorange errors,
- can adaptively activate Huber-style WLS residual weighting
  (`gnss_spp --adaptive-robust-weighting`) only on residual-tail epochs,
- can optionally reject kinematically implausible output jumps
  (`gnss_spp --max-position-jump-rate-mps ...`) while preserving the last
  accepted position as the next solver seed,
- exposes `gnss_spp --summary-json` plus SPP QC and variance-model options.

## OSS References

| Reference | Useful idea | Adoption target |
|---|---|---|
| [RTKLIB manual](https://sources.debian.org/src/rtklib/2.4.3%2Bdfsg1-2.1/doc/manual_2.4.2.pdf) and [`pntpos.c`](https://raw.githubusercontent.com/tomojitakasu/RTKLIB/master/src/pntpos.c) | SPP validation, chi-square/GDOP checks, RAIM FDE, SBAS/IONEX/IFLC options, variance terms for iono/trop/code-bias errors | Port the QC and variance structure first; keep C++ implementation native |
| [GNSS-SDR PVT docs](https://gnss-sdr.org/docs/sp-blocks/pvt/) | Clear SPP WLS model plus RAIM, ionosphere, troposphere, and PVT denoising concepts | Use as documentation and algorithm sanity reference |
| [goGPS](https://gogps-project.github.io/wiki/) | Combined and uncombined LS engines using multi-frequency/multi-tracking observations | Guide dual-frequency and uncombined SPP design |
| [laika](https://github.com/commaai/laika) | Product fetch/cache, satellite info, ionosphere/troposphere/DCB correction helper layer | Reuse local `fetch-products`, IONEX, and DCB support for SPP |
| [GSILIB](gsilib-notes.md) | Japanese post-processing flows with SP3/CLK/IONEX/RTCM SSR and IFB/ISB examples | Use as correction-workflow reference, not as a GUI target |

## Paper And Technical References

| Topic | Evidence | SPP implication |
|---|---|---|
| Dual-frequency ionosphere-free code | [ESA Navipedia IF combination](https://gssc.esa.int/navipedia/index.php/Ionosphere-free_Combination_for_Dual_Frequency_Receivers) and [GNSS-SDR PVT](https://gnss-sdr.org/docs/sp-blocks/pvt/) | Removes first-order ionosphere, but amplifies code noise; use as selectable `IFLC SPP` |
| Broadcast ionosphere limits | [MDPI Klobuchar/BDGIM/NTCM-BC assessment](https://www.mdpi.com/2072-4292/12/7/1215) | Klobuchar is a low-cost fallback; IONEX/GIM or regional ionosphere should be supported |
| Precise orbit/clock products | [IGS products](https://igs.org/products/) | SP3/CLK can reduce broadcast orbit/clock error even for code-only augmented SPP |
| Code-carrier smoothing | [ESA Navipedia Hatch filter](https://gssc.esa.int/navipedia/index.php/Carrier-smoothing_of_code_pseudoranges) and [improved Hatch filter paper](https://www.sciencedirect.com/science/article/abs/pii/S0263224119306591) | Smooth pseudorange noise when carrier phase is continuous; needs cycle-slip and ionosphere-divergence guards |
| Urban heavy-tailed errors | [LQLC robust estimator paper](https://arxiv.org/abs/2603.16420) | Replace plain WLS with optional IRLS robust weighting for multipath/NLOS |
| SBAS/single-frequency PPP corrections | [SBAS SF-PPP paper](https://pmc.ncbi.nlm.nih.gov/articles/PMC5017426/) | Orbit/clock/iono/code-bias corrections can make code-only positioning substantially better than broadcast SPP |
| Open high-accuracy services | [Galileo HAS](https://gssc.esa.int/navipedia/index.php/Galileo_High_Accuracy_Service_%28HAS%29) and [QZSS MADOCA-PPP](https://qzss.go.jp/en/technical/dod/madoca/index.html) | Existing SSR/PPP code can feed an augmented SPP mode before full PPP convergence is available |

## Recommended PR Order

1. **SPP QC and variance model**
   - Done: residual-based outlier rejection using the existing
     `enable_outlier_detection` config.
   - Done: reduced chi-square diagnostics, max-GDOP gate, residual RMS gate,
     and optional RAIM FDE for epochs with sufficient redundancy.
   - Done: replace raw `sin(el)^2` weighting with a first-pass explicit
     variance model: `code + elevation + SNR/CN0 + ionosphere + troposphere`.
   - Done: add precise orbit/clock, IONEX, and Bias-SINEX/OSB correction
     counters to SPP diagnostics.
   - Still needed: add product uncertainty terms to the variance model when
     SP3/CLK, IONEX, DCB/OSB, or SSR/OSR layers are active.
   - Done: emit SPP QC counters and residual/DOP aggregates from `gnss_spp`
     and its summary JSON.

2. **SPP CLI and benchmark harness**
   - Done: expose elevation mask, SNR mask, outlier/FDE/gate controls, and
     summary JSON.
   - Done: add `gnss_ppc_demo.py --solver spp` so public PPC reference
     trajectories can score SPP `.pos` output with the same PPC metrics JSON
     used by the RTK/PPP benchmark paths.
   - Done: expose optional dual-frequency IFLC SPP through
     `gnss_spp --ionosphere-free`.
   - Done: expose optional product-aided SPP through `--ionex`, `--dcb`,
     `--disable-ionex-corrections`, and `--disable-dcb-corrections`.
   - Done: expose optional SP3/CLK-aided SPP through `--sp3`, `--clk`, and
     `--disable-precise-products`.
   - Done: expose optional CSV SSR-aided SPP through `--ssr` and
     `--disable-ssr-corrections`.
   - Done: expose optional Huber residual downweighting through
     `--robust-weighting`, `--robust-threshold-sigma`, and
     `--robust-min-weight`.
   - Done: expose optional SPP position-jump gating through
     `--max-position-jump-rate-mps` and `--max-position-jump-min-m`.
   - Done: add `gnss ppc-spp-jump-sweep` for post-hoc PPC jump-gate threshold
     sweeps without rerunning RINEX processing.
   - Done: add filtered `.pos` output for selected post-hoc jump-gate and
     bridge candidates.
   - Done: add positioning-rate policy guards to `gnss ppc-spp-jump-sweep`
     for bounded availability-loss jump/bridge candidate selection.
   - Done: add `gnss ppc-spp-compare` to compare multiple SPP candidate
     `.pos` files and render PPC CSV/PNG artifacts.
   - Done: add `gnss_spp --adaptive-robust-weighting` with tail-count and
     tail-fraction activation gates, plus JSON/Python/PPC diagnostics.
   - Still needed: constellation toggles, explicit ionosphere-mode enum,
     and troposphere option.
   - Add `compare-modes` gates that report SPP horizontal/vertical RMSE, P95,
     residual RMS, satellite count, and DOP on static and Odaiba datasets.

3. **Dual-frequency code SPP**
   - Done: group observations by satellite and select primary code plus
     secondary code when available.
   - Done: form first-order ionosphere-free pseudorange with standard
     `f1^2/(f1^2-f2^2)` and `-f2^2/(f1^2-f2^2)` coefficients.
   - Done: skip broadcast ionosphere correction for IFLC measurements, combine
     broadcast group delay terms with the same coefficients, and inflate code
     variance by the coefficient-squared sum.
   - Still needed: replace the boolean option with
     `SPPConfig::IonoOpt::{BROADCAST, IONEX, IFLC, OFF}`.
   - Done: apply loaded OSB entries where available for single-frequency and
     IFLC code observations.
   - Still needed: support non-OSB differential DCB semantics explicitly and
     keep per-system signal policy configurable.

4. **IONEX/DCB and better troposphere**
   - Done: reuse in-tree `IONEXProducts` and `DCBProducts` for SPP.
   - Done: prefer IONEX-derived ionosphere delay over Klobuchar for
     single-frequency SPP when maps are loaded and interpolation succeeds.
   - Done: skip IONEX on IFLC measurements because first-order ionosphere is
     analytically canceled.
   - Use the PPP-side climatology plus Niell mapping as the SPP troposphere
     option instead of only the simple `1/cos(z)` Saastamoinen path.

5. **Augmented code-only SPP**
   - Done: add optional SP3/CLK precise product input to SPP.
   - Done: add optional CSV SSR orbit/clock/code-bias corrections to SPP,
     including RAC-to-ECEF orbit handling, satellite-clock correction, and
     SSR code-bias application ahead of DCB/OSB fallback.
   - Still needed: direct RTCM SSR, QZSS L6 CLAS/MADOCA, and Galileo HAS
     product ingestion from the SPP CLI instead of requiring pre-expanded CSV.
   - Still needed: OSR atmospheric corrections from CLAS/MADOCA/HAS for
     augmented SPP.
   - Keep this mode named separately from PPP because it remains code-only and
     should not promise carrier-phase convergence.

6. **Code-carrier smoothing and robust urban mode**
   - Add Hatch smoothing only when carrier phase is continuous and cycle-slip
     checks pass.
   - Done: add optional Huber IRLS-style weighting over code residuals.
   - Still needed: evaluate Cauchy/logistic alternatives and auto-tune robust
     thresholds per receiver class.
   - Done: add optional position-jump gating to suppress impossible 5 Hz
     urban SPP output spikes.
   - Use pseudorange acceleration and C/N0 degradation as optional multipath
     weights for urban data.

## Expected Payoff

Highest near-term payoff:

1. Fixing the Klobuchar unit bug.
2. Residual QC plus RAIM/FDE.
3. Proper measurement variance and SNR/CN0 weighting.
4. Dual-frequency IFLC SPP where measurements exist.
5. IONEX/OSB, SP3/CLK, and CSV SSR support for product-aided SPP.

Highest absolute accuracy payoff:

1. Direct RTCM/L6/HAS SSR/OSR ingestion and atmospheric corrections.
2. Product-aided single-frequency ionosphere and OSB-aware IFLC handling.
3. Code-carrier smoothing on clean carrier-phase tracks.

Highest urban robustness payoff:

1. RAIM/FDE plus robust IRLS.
2. C/N0 and pseudorange-acceleration-based down-weighting.
3. Existing visibility/fisheye tooling as an NLOS mask source when available.

## Validation Gates

Every SPP accuracy PR should include:

- static sample regression against known RINEX,
- Odaiba mixed-constellation regression,
- `gnss spp --summary-json` with residual/QC counters,
- before/after SPP RMSE/P95 when truth is available,
- no degradation to SPP seed coverage used by PPC/Taroz dogfood tests,
- a reference comparison to RTKLIB or `rtklib-py` for at least one epoch class.
