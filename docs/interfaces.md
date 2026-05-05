# Interfaces

## CLI

The primary interface is the `gnss` dispatcher plus native command binaries.

Examples:

- `gnss spp`
- `gnss solve`
- `gnss ppp` (`--nav`, `--sp3`, `--clk`, `--ionex`, `--dcb`, `--ssr`, `--ssr-rtcm`)
- `gnss clas-ppp`
- `gnss qzss-l6-info` (`--compact-flush-policy <lag-tolerant-union|orbit-or-clock-only|orbit-and-clock-only>`, `--compact-atmos-merge-policy <stec-coeff-carry|no-carry|network-locked-stec-coeff-carry>`, `--compact-atmos-subtype-merge-policy <union|gridded-priority|combined-priority>`, `--compact-phase-bias-merge-policy <latest-union|message-reset|selected-mask-prune>`, `--compact-phase-bias-source-policy <arrival-order|subtype5-priority|subtype6-priority>`, `--compact-code-bias-composition-policy <direct-values|base-plus-network|base-only-if-present>`, `--compact-phase-bias-composition-policy <direct-values|base-plus-network|base-only-if-present>`, `--compact-phase-bias-bank-policy <pending-epoch|same-30s-bank|close-30s-bank|latest-preceding-bank>`, `--compact-bias-row-materialization <overlap-only|selected-satellite-base-extend|all-base-satellite-extend>`, `--compact-row-construction-policy <independent|coupled-code-phase|row-first-value-second|network-row-driven>`)
- `gnss fetch-products`
- `gnss ppp-products-signoff`
- `gnss ionex-info`
- `gnss dcb-info`
- `gnss visibility`
- `gnss visibility-plot`
- `gnss moving-base-plot`
- `gnss moving-base-signoff`
- `gnss moving-base-prepare`
- `gnss scorpion-moving-base-signoff`
- `gnss stream`
- `gnss convert`
- `gnss live`
- `gnss rcv`
- `gnss web`

For longer workflows, `gnss web`, `gnss live-signoff`,
`gnss moving-base-signoff`, `gnss scorpion-moving-base-signoff`,
`gnss ppc-rtk-signoff`, and `gnss ppp-products-signoff`
also accept `--config-toml <file>`. Example templates live in `configs/`.

For CLAS-oriented PPP experiments, `gnss ppp` also exposes the boundary and
selector knobs used by the experiment lane:

- `--clas-epoch-policy <strict-osr|hybrid-standard-ppp>`
- `--clas-osr-application <full-osr|orbit-clock-bias|orbit-clock-only>`
- `--clas-phase-continuity <full-repair|sis-continuity-only|repair-only|raw-phase-bias|no-phase-bias>`
- `--clas-phase-bias-values <full|phase-bias-only|compensation-only>`
- `--clas-phase-bias-reference-time <phase-bias-reference|clock-reference|observation-epoch>`
- `--clas-ssr-timing <lag-tolerant|clock-bound-phase-bias|clock-bound-atmos-and-phase-bias>`
- `--clas-expanded-values <full-composed|residual-only|polynomial-only>`
- `--clas-subtype12-values <full|planar|offset-only>`
- `--clas-residual-sampling <indexed-or-mean|indexed-only|mean-only>`
- `--clas-atmos-selection <grid-first|grid-guarded|balanced|freshness-first>`
- `--clas-atmos-stale-after-seconds <seconds>`

## Docker

The repo also ships a multi-stage `Dockerfile`. The runtime image installs:

- the `gnss` dispatcher on `PATH`
- native CLI binaries under `/opt/libgnsspp/bin`
- the `libgnsspp` Python package via `PYTHONPATH`

The default entrypoint is `gnss`, so container invocations look like:

```bash
docker run --rm -it -v "$PWD:/workspace" libgnsspp:latest --help
docker run --rm -it -v "$PWD:/workspace" libgnsspp:latest ppp --help
docker run --rm -it -p 8085:8085 -v "$PWD:/workspace" libgnsspp:latest web --host 0.0.0.0 --port 8085 --root /workspace
docker compose up gnss-web
```

## Python

Python bindings expose:

- RINEX header and epoch inspection
- `.pos` loading and solution statistics
- coordinate conversion helpers
- file-based `SPP`, `PPP`, and `RTK` solve helpers

## ROS2

The ROS2 playback node publishes `.pos` files as:

- `sensor_msgs/NavSatFix`
- `geometry_msgs/PoseStamped`
- `nav_msgs/Path`
- solution status and satellite-count telemetry

## Local web UI

`gnss web` is a local visualization layer for existing artifacts, including live, moving-base, PPC, and PPP-product sign-off summaries, artifact bundles, moving-base history charts, commercial receiver side-by-side metrics, and direct links to summary, plot, product, comparison CSV/PNG, match CSV, and provenance files.

It shows:

- Odaiba metrics
- PPC summaries
- live sign-off summaries
- moving-base plots
- commercial receiver comparisons for PPC and moving-base sign-offs
- 2D trajectories
- visibility summaries and a polar visibility view
- `gnss rcv` receiver status

## Experiment Interface

`PPP-AR` experiments now use a minimal shared interface under
`experiments/ppp_ar/`.

The current experiment surface is:

- `experiments/ppp_ar/strategies.toml`
- `experiments/ppp_ar/input.example.toml`
- `experiments/ppp_ar/suite.example.toml`
- `experiments/ppp_ar/run_experiments.py`

The current CLAS PPP experiment family includes:

- `osr_float_pipeline`
- `osr_float_strict_pipeline`
- `osr_float_hybrid_pipeline`
- `osr_float_orbit_clock_bias_pipeline`
- `osr_float_orbit_clock_only_pipeline`
- `osr_float_phase_bias_only_value_pipeline`
- `osr_float_compensation_only_value_pipeline`
- `osr_float_clock_reference_time_pipeline`
- `osr_float_observation_reference_time_pipeline`
- `osr_float_sis_continuity_only_pipeline`
- `osr_float_repair_only_pipeline`
- `osr_float_raw_phase_bias_pipeline`
- `osr_float_no_phase_bias_pipeline`
- `osr_float_clock_bound_phase_bias_pipeline`
- `osr_float_clock_bound_atmos_phase_bias_pipeline`
- `osr_float_orbit_or_clock_source_pipeline`
- `osr_float_orbit_and_clock_source_pipeline`
- `osr_float_no_carry_atmos_merge_pipeline`
- `osr_float_network_locked_atmos_merge_pipeline`
- `osr_float_gridded_priority_subtype_merge_pipeline`
- `osr_float_combined_priority_subtype_merge_pipeline`
- `osr_float_message_reset_phase_bias_merge_pipeline`
- `osr_float_subtype5_priority_phase_bias_source_pipeline`
- `osr_float_subtype6_priority_phase_bias_source_pipeline`
- `osr_float_base_plus_network_code_bias_composition_pipeline`
- `osr_float_base_only_code_bias_composition_pipeline`
- `osr_float_same_30s_code_bias_bank_pipeline`
- `osr_float_close_30s_code_bias_bank_pipeline`
- `osr_float_latest_preceding_code_bias_bank_pipeline`
- `osr_float_selected_satellite_base_extend_pipeline`
- `osr_float_all_base_satellite_extend_pipeline`
- `osr_float_base_plus_network_phase_bias_composition_pipeline`
- `osr_float_base_only_phase_bias_composition_pipeline`
- `osr_float_same_30s_phase_bias_bank_pipeline`
- `osr_float_close_30s_phase_bias_bank_pipeline`
- `osr_float_latest_preceding_phase_bias_bank_pipeline`
- `osr_float_residual_only_value_pipeline`
- `osr_float_planar_subtype12_value_pipeline`
- `osr_float_offset_only_subtype12_value_pipeline`
- `osr_float_indexed_only_residual_sampling_pipeline`
- `osr_float_mean_only_residual_sampling_pipeline`
- `osr_float_selected_mask_prune_phase_bias_merge_pipeline`
- `osr_float_polynomial_only_value_pipeline`
- `osr_float_guarded_pipeline`
- `osr_float_balanced_pipeline`
- `osr_float_freshness_pipeline`
- `osr_ar_pipeline`
- `osr_ar_strict_pipeline`
- `osr_ar_hybrid_pipeline`
- `osr_ar_orbit_clock_bias_pipeline`
- `osr_ar_orbit_clock_only_pipeline`
- `osr_ar_phase_bias_only_value_pipeline`
- `osr_ar_compensation_only_value_pipeline`
- `osr_ar_clock_reference_time_pipeline`
- `osr_ar_observation_reference_time_pipeline`
- `osr_ar_sis_continuity_only_pipeline`
- `osr_ar_repair_only_pipeline`
- `osr_ar_raw_phase_bias_pipeline`
- `osr_ar_no_phase_bias_pipeline`
- `osr_ar_clock_bound_phase_bias_pipeline`
- `osr_ar_clock_bound_atmos_phase_bias_pipeline`
- `osr_ar_orbit_or_clock_source_pipeline`
- `osr_ar_orbit_and_clock_source_pipeline`
- `osr_ar_no_carry_atmos_merge_pipeline`
- `osr_ar_network_locked_atmos_merge_pipeline`
- `osr_ar_gridded_priority_subtype_merge_pipeline`
- `osr_ar_combined_priority_subtype_merge_pipeline`
- `osr_ar_message_reset_phase_bias_merge_pipeline`
- `osr_ar_subtype5_priority_phase_bias_source_pipeline`
- `osr_ar_subtype6_priority_phase_bias_source_pipeline`
- `osr_ar_base_plus_network_code_bias_composition_pipeline`
- `osr_ar_base_only_code_bias_composition_pipeline`
- `osr_ar_same_30s_code_bias_bank_pipeline`
- `osr_ar_close_30s_code_bias_bank_pipeline`
- `osr_ar_latest_preceding_code_bias_bank_pipeline`
- `osr_ar_selected_satellite_base_extend_pipeline`
- `osr_ar_all_base_satellite_extend_pipeline`
- `osr_ar_base_plus_network_phase_bias_composition_pipeline`
- `osr_ar_base_only_phase_bias_composition_pipeline`
- `osr_ar_same_30s_phase_bias_bank_pipeline`
- `osr_ar_close_30s_phase_bias_bank_pipeline`
- `osr_ar_latest_preceding_phase_bias_bank_pipeline`
- `osr_ar_residual_only_value_pipeline`
- `osr_ar_planar_subtype12_value_pipeline`
- `osr_ar_offset_only_subtype12_value_pipeline`
- `osr_ar_indexed_only_residual_sampling_pipeline`
- `osr_ar_mean_only_residual_sampling_pipeline`
- `osr_ar_selected_mask_prune_phase_bias_merge_pipeline`
- `osr_ar_polynomial_only_value_pipeline`
- `osr_ar_guarded_pipeline`
- `osr_ar_balanced_pipeline`
- `osr_ar_freshness_pipeline`

The stable rule is that every strategy arm must share:

- one `obs`
- one `nav`
- one CLAS correction source (`qzss_l6` or pre-expanded `ssr_csv`)
- one `reference_ecef`
- one `max_epochs`
- one output schema

When promotion is under evaluation, the runner can also take a suite config and
compare the same strategy set across multiple CLAS cases before changing the
default solver path.

If `reference_ecef` is omitted for a case, the runner falls back to the RINEX
header `APPROX POSITION XYZ`. That keeps new cases easy to add while still
using one shared metric schema.

The current output schema for comparisons is:

- `wall_time_s`
- `ppp_solution_rate_pct`
- `ppp_fixed_solutions`
- `fallback_solutions`
- `clas_hybrid_fallback_epochs`
- `mean_3d_error_m`
- `median_3d_error_m`
- `p95_3d_error_m`
- `max_3d_error_m`
- `readability_score`
- `extensibility_score`

This keeps the stable core separate from the experiment lane while still making
results directly comparable.

For compact-SSR ingestion experiments, the shared decode boundary also exposes:

- `gnss qzss-l6-info --compact-flush-policy <lag-tolerant-union|orbit-or-clock-only|orbit-and-clock-only>`
- `gnss qzss-l6-info --compact-atmos-merge-policy <stec-coeff-carry|no-carry|network-locked-stec-coeff-carry>`
- `gnss qzss-l6-info --compact-atmos-subtype-merge-policy <union|gridded-priority|combined-priority>`
- `gnss qzss-l6-info --compact-phase-bias-merge-policy <latest-union|message-reset|selected-mask-prune>`
- `gnss qzss-l6-info --compact-phase-bias-source-policy <arrival-order|subtype5-priority|subtype6-priority>`
- `gnss qzss-l6-info --compact-code-bias-composition-policy <direct-values|base-plus-network|base-only-if-present>`
- `gnss qzss-l6-info --compact-code-bias-bank-policy <pending-epoch|same-30s-bank|close-30s-bank|latest-preceding-bank>`
- `gnss qzss-l6-info --compact-bias-row-materialization <overlap-only|selected-satellite-base-extend|all-base-satellite-extend>`
- `gnss qzss-l6-info --compact-phase-bias-composition-policy <direct-values|base-plus-network|base-only-if-present>`
- `gnss qzss-l6-info --compact-phase-bias-bank-policy <pending-epoch|same-30s-bank|close-30s-bank|latest-preceding-bank>`
- `gnss clas-ppp --compact-atmos-merge-policy <stec-coeff-carry|no-carry|network-locked-stec-coeff-carry>`
- `gnss clas-ppp --compact-atmos-subtype-merge-policy <union|gridded-priority|combined-priority>`
- `gnss clas-ppp --compact-phase-bias-merge-policy <latest-union|message-reset|selected-mask-prune>`
- `gnss clas-ppp --compact-phase-bias-source-policy <arrival-order|subtype5-priority|subtype6-priority>`
- `gnss clas-ppp --compact-code-bias-composition-policy <direct-values|base-plus-network|base-only-if-present>`
- `gnss clas-ppp --compact-code-bias-bank-policy <pending-epoch|same-30s-bank|close-30s-bank|latest-preceding-bank>`
- `gnss clas-ppp --compact-bias-row-materialization <overlap-only|selected-satellite-base-extend|all-base-satellite-extend>`
- `gnss clas-ppp --compact-phase-bias-composition-policy <direct-values|base-plus-network|base-only-if-present>`
- `gnss clas-ppp --compact-phase-bias-bank-policy <pending-epoch|same-30s-bank|close-30s-bank|latest-preceding-bank>`
- `gnss clas-ppp --expanded-ssr <sampled.csv>`
- `gnss clas-ppp --qzss-expanded-cache <sampled.csv>`
- `gnss clas-ppp --qzss-window-margin-seconds <seconds>`
- `gnss clas-ppp --filter-iterations <n>`
- `gnss clas-ppp --ppp-filter-log <csv>`
- `gnss clas-ppp --ppp-residual-log <csv>`
- `gnss clas-ppp --ppp-state-log <stat>`

For `gnss clas-ppp --profile clas`, the wrapper defaults native PPP filter
iterations to 1. Direct `gnss ppp` keeps the native default. When `--qzss-l6`
is used with `--max-epochs`, the wrapper limits expanded SSR rows to the
observed TOW window plus `--qzss-window-margin-seconds`; `--qzss-expanded-cache`
reuses an existing expanded CSV when present.

When a suite config is used, the output tree is partitioned by case:

- `output/.../<case_key>/compact_ssr.csv`
- `output/.../<case_key>/expanded_ssr.csv`
- `output/.../<case_key>/<strategy>/<strategy>.pos`
- `output/.../<case_key>/<strategy>/<strategy>_summary.json`

## Moving-base

`gnss solve`, `gnss replay`, and `gnss live` accept `--mode moving-base`.

`gnss moving-base-prepare` extracts rover/base UBX streams plus a per-epoch reference CSV from a ROS2 moving-base bag or Zenodo zip. It can also export the rover receiver `NAV-PVT` solution as a normalized commercial receiver CSV. `gnss moving-base-signoff` is the validation entrypoint for replay/live runs against those references. `gnss scorpion-moving-base-signoff` wraps the public SCORPION bag flow by chaining prepare, BRDC nav fetch, replay validation, and receiver side-by-side matching. Together they cover:

- ROS2 bag or zip ingestion with u-blox `NAV-PVT` / `RXM-RAWX` / `NAV-RELPOSNED`
- replay inputs via `--rover-ubx` and `--base-ubx`
- public receiver side-by-side summaries via `--commercial-csv` / `--commercial-pos`
- fix rate
- baseline error percentiles
- heading error percentiles
- realtime factor
- decoder error counts
