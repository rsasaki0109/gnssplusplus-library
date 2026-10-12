# Interfaces

See [received-event RTK/IMU](online_rtk_imu.md) for the sequential native API
and stdin executable, including input availability and timestamp contracts.

See [runtime FIX recovery](fix_recovery_guard.md) for optional residual
containment, clean-candidate recovery and the off/on evaluation contract.

[Python satellite observation analysis](python_observation_analysis.md) provides
code residual, SNR and carrier-continuity diagnostics from the public bindings.

## CLI

The primary interface is the `gnss` dispatcher plus native command binaries.

Examples:

- `gnss doctor`
- `gnss ros2-doctor`
- `gnss ros2-bag-doctor`
- `gnss field-report`
- `gnss robotics-smoke`
- `gnss spp`
- `gnss solve` (`--pos-uncertainty` appends per-epoch sigma columns, see below)
- `gnss uncertainty-eval` (truth-based honesty check of those sigma columns)
- `gnss ppp` (`--nav`, `--sp3`, `--clk`, `--ionex`, `--dcb`, `--ssr`, `--ssr-rtcm`)
- `gnss clas-ppp`
- `gnss qzss-l6-info` (`--compact-flush-policy <lag-tolerant-union|orbit-or-clock-only|orbit-and-clock-only>`, `--compact-atmos-merge-policy <stec-coeff-carry|no-carry|network-locked-stec-coeff-carry>`, `--compact-atmos-subtype-merge-policy <union|gridded-priority|combined-priority>`, `--compact-phase-bias-merge-policy <latest-union|message-reset|selected-mask-prune>`, `--compact-phase-bias-source-policy <arrival-order|subtype5-priority|subtype6-priority>`, `--compact-code-bias-composition-policy <direct-values|base-plus-network|base-only-if-present>`, `--compact-code-bias-bank-policy <pending-epoch|same-30s-bank|close-30s-bank|latest-preceding-bank|delayed-15s-bank>`, `--compact-phase-bias-composition-policy <direct-values|base-plus-network|base-only-if-present>`, `--compact-phase-bias-bank-policy <pending-epoch|same-30s-bank|close-30s-bank|latest-preceding-bank>`, `--compact-bias-row-materialization <overlap-only|selected-satellite-base-extend|all-base-satellite-extend>`, `--compact-row-construction-policy <independent|coupled-code-phase|row-first-value-second|network-row-driven>`)
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
- `gnss realtime-stress` (recorded-data real-time stress harness, see [Real-time stress](realtime_stress.md))
- `gnss rtcm-encode`
- `gnss station` (`check|start|status|stop|restart`, with a station TOML)
- `gnss rcv`
- `gnss web`

For longer workflows, `gnss web`, `gnss live-signoff`,
`gnss moving-base-signoff`, `gnss scorpion-moving-base-signoff`,
`gnss ppc-rtk-signoff`, `gnss ppc-coverage-matrix`, and
`gnss ppp-products-signoff`
also accept `--config-toml <file>`. Templates are indexed in
`configs/README.md` and grouped by purpose.

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

### Multi-constellation RTCM epoch assembly (`--no-rtcm-epoch-assembly`)

A receiver sends one MSM message per constellation with the same time tag and
sets the RTCM "multiple message" bit (legacy 1003/1004: synchronous GNSS flag)
on every message of the epoch except the last. `gnss live` (rover and base RTCM),
`gnss replay` (base RTCM) and `gnss convert` (RTCM to RINEX) merge those parts
into one epoch with one shared implementation
(`io::ObservationEpochAssembler`, fed by `RTCMStreamDecoder::Event::
observation_more_follows` or `RTCMProcessor::lastObservationMoreFollows()`):

- a part whose time differs from the pending epoch by more than 1 ms emits the
  pending epoch first (a lost last message cannot merge two epochs);
- otherwise it is merged; a (satellite, signal) already present is kept once;
- bit 0 emits the epoch immediately, so no latency is added;
- the pending epoch is flushed at end of stream.

A stream whose messages all carry bit 0 (every GPS-only stream) is unchanged.
It is on by default; `--no-rtcm-epoch-assembly` (in all three tools) restores
one epoch per observation message. `gnss rtcm-encode` sets the bit on every
observation message of an epoch except the last one emitted
(`RTCMProcessor::encodeObservations(..., more_follows)` defaults to `false`).
`gnss stream` only prints one diagnostic line per message and does not build
epochs, so it is not affected. Time tags are only comparable if their weeks
agree: GLONASS MSM epochs are resolved against the host clock when the decoder
has no reference time (see Limits found 4 in [Real-time
stress](realtime_stress.md)).

### Causal output guard for `gnss live` (`--kinematic-post-filter`)

Opt-in (default off; without it `gnss live` output is byte-identical to
earlier releases) and adds no output delay. Once a FIX has been written, an
epoch whose height differs from the last *written* epoch by more than
`max(1 m, 4 m/s * dt)` (`dt <= 0` counts as 1 s) is not written, and a dropped
epoch does not move the anchor. This is the height-step rule of `gnss solve`'s
kinematic post-filter; both tools share one implementation
(`rtk_validation::KinematicPostFilter`). With the flag the `summary:` line
gains `post_filter_dropped_epochs=`; dropped epochs are missing rows in the
`.pos`. Measured effect and its limits: [Real-time stress
harness](realtime_stress.md#wrong-fix-on-the-real-time-path).

### Per-epoch uncertainty in `.pos` (`--pos-uncertainty`)

`gnss solve`, `gnss replay` and `gnss live` accept `--pos-uncertainty` (default
off; with it off the `.pos` is byte-identical to earlier releases, and it
requires `--format pos`). It appends seven columns after
`RTKUpdateNISRejected` and declares them in the `% GPS_Week ...` header:

| Column | Meaning |
| --- | --- |
| `sdn(m) sde(m) sdu(m)` | 1-sigma north/east/up standard deviation in metres, from `PositionSolution::positionCovarianceEnu()` |
| `sdne(m) sdeu(m) sdun(m)` | covariance cross terms, RTKLIB signed square root `sign(c)*sqrt(abs(c))` |
| `DiffAge(s)` | rover epoch time minus the base observation time used (`0` for an exactly matched base epoch; for an interpolated or held base epoch, the age of the older sample) |

Unavailable values are written as `nan`. Existing column positions do not move,
so index-based readers keep working; `Solution::loadFromFile` also rebuilds the
ECEF covariance from these columns. The sigma is a model output, not a
certified bound: the default RTK reported covariance can be a constant (see
`--rtk-reported-covariance`). `gnss solve` has no NMEA output, so no GST
sentence is produced.

`gnss uncertainty-eval --pos run.pos --reference-csv reference.csv
[--out-json s.json --out-csv epochs.csv --out-stanford-csv bins.csv]` matches
the epochs to a PPC / UrbanNav `reference.csv` (`--match-tolerance-s`, default
0.25 s) and reports, for FIXED, FLOAT and all epochs: normalized errors
`e_H/sigma_H` (`sigma_H = sqrt(sdn^2+sde^2)`) and `|e_U|/sdu` (median, p95,
p99), exceedance fractions `P(|e| > k*sigma)` for k = 1, 2, 3, 5 beside the
Gaussian expectation, and Stanford-diagram bin counts of error against a
`K*sigma` bound (`--k-h`/`--k-v`, default 5.33) split into nominal, misleading
(error above the bound) and, with `--alert-limit-h/-v`, unavailable (bound above
the alert limit). `K*sigma` is a diagnostic proxy and is **not** a certified
protection level. A PNG is written with `--out-png` only when matplotlib is
installed.

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
- read-only artifact helpers under `libgnsspp.artifacts`:
  `load_summary()`, `summary_schema()`, `load_pos()`, and `pos_stats()`

## ROS2

The ROS2 playback node publishes `.pos` files as:

- `sensor_msgs/NavSatFix`
- `geometry_msgs/PoseStamped`
- `nav_msgs/Path`
- solution status and satellite-count telemetry

## Local web UI

`gnss web` is a local visualization layer for existing artifacts, including field report summaries and Markdown previews, live, moving-base, PPC, and PPP-product sign-off summaries, artifact bundles, moving-base history charts, commercial receiver side-by-side metrics, next debug commands, and direct links to summary, plot, product, comparison CSV/PNG, match CSV, and provenance files.

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
`scripts/experiments/ppp_ar/`.

The current experiment surface is:

- `scripts/experiments/ppp_ar/strategies.toml`
- `scripts/experiments/ppp_ar/input.example.toml`
- `scripts/experiments/ppp_ar/suite.example.toml`
- `scripts/experiments/ppp_ar/run_experiments.py`

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
- `gnss qzss-l6-info --compact-code-bias-bank-policy <pending-epoch|same-30s-bank|close-30s-bank|latest-preceding-bank|delayed-15s-bank>`
- `gnss qzss-l6-info --compact-bias-row-materialization <overlap-only|selected-satellite-base-extend|all-base-satellite-extend>`
- `gnss qzss-l6-info --compact-phase-bias-composition-policy <direct-values|base-plus-network|base-only-if-present>`
- `gnss qzss-l6-info --compact-phase-bias-bank-policy <pending-epoch|same-30s-bank|close-30s-bank|latest-preceding-bank>`
- `gnss ppp --receiver-antenna-type <type>`
- `gnss clas-ppp --compact-atmos-merge-policy <stec-coeff-carry|no-carry|network-locked-stec-coeff-carry>`
- `gnss clas-ppp --compact-atmos-subtype-merge-policy <union|gridded-priority|combined-priority>`
- `gnss clas-ppp --compact-phase-bias-merge-policy <latest-union|message-reset|selected-mask-prune>`
- `gnss clas-ppp --compact-phase-bias-source-policy <arrival-order|subtype5-priority|subtype6-priority>`
- `gnss clas-ppp --compact-code-bias-composition-policy <direct-values|base-plus-network|base-only-if-present>`
- `gnss clas-ppp --compact-code-bias-bank-policy <pending-epoch|same-30s-bank|close-30s-bank|latest-preceding-bank|delayed-15s-bank>`
- `gnss clas-ppp --antex <antennas.atx>`
- `gnss clas-ppp --receiver-antenna-type <type>`
- `gnss clas-ppp --clas-atmos-selection <grid-first|grid-guarded|balanced|freshness-first>`
- `gnss clas-ppp --clas-atmos-stale-after-seconds <seconds>`
- `gnss clas-ppp --compact-bias-row-materialization <overlap-only|selected-satellite-base-extend|all-base-satellite-extend>`
- `gnss clas-ppp --compact-phase-bias-composition-policy <direct-values|base-plus-network|base-only-if-present>`
- `gnss clas-ppp --compact-phase-bias-bank-policy <pending-epoch|same-30s-bank|close-30s-bank|latest-preceding-bank>`

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
