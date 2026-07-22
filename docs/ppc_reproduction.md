# PPC Reproduction Commands

`PPC-Dataset` can be verified directly from an extracted dataset tree:

```bash
python3 apps/gnss.py ppc-demo \
  --dataset-root /datasets/PPC-Dataset \
  --city tokyo \
  --run run1 \
  --solver rtk \
  --require-realtime-factor-min 1.0 \
  --summary-json output/ppc_tokyo_run1_rtk_summary.json
```

## Current Sign-Off Profile

Use `ppc-rtk-signoff` for the deployable status profile:

```bash
python3 apps/gnss.py ppc-rtk-signoff \
  --dataset-root /datasets/PPC-Dataset \
  --city tokyo \
  --realtime-profile sigma-demote \
  --rtklib-bin /path/to/rnx2rtkp \
  --summary-json output/ppc_tokyo_run1_rtk_signoff.json
```

Use `ppc-coverage-matrix` to replay the full six-run `sigma-demote nis2-ratio4`
profile:

```bash
python3 apps/gnss.py ppc-coverage-matrix \
  --config-toml configs/benchmarks/ppc_sigma_demote_nis2_ratio4.toml
```

Override `--dataset-root` after `--config-toml` when your PPC checkout is not
under `data/PPC-Dataset`.

## Local MADOCALIB RTKLIB Smoke

If a standard RTKLIB `rnx2rtkp` is not installed but `external/madocalib` is
available locally, build its console app and use the PPC RTK config for the
fork-specific option enum:

```bash
make -C external/madocalib/app/consapp/rnx2rtkp/gcc

python3 apps/gnss.py ppc-coverage-matrix \
  --config-toml configs/benchmarks/ppc_sigma_demote_nis2_ratio4.toml \
  --max-epochs 20 \
  --rtklib-bin external/madocalib/app/consapp/rnx2rtkp/gcc/rnx2rtkp \
  --rtklib-config scripts/madocalib_ppc_rtk.conf \
  --output-dir output/ppc_rtklib_baseline_smoke \
  --summary-json output/ppc_rtklib_baseline_smoke/summary.json \
  --markdown-output output/ppc_rtklib_baseline_smoke/table.md
```

`ppc-demo` passes `-p 2` to `rnx2rtkp` for RTK comparisons. The MADOCALIB
config intentionally omits `pos1-posmode` because that fork's config enum does
not accept relative `kinematic`, while the CLI option does.

## Coverage Matrix

Use this for the README RTKLIB `demo5` comparison table:

```bash
python3 apps/gnss.py ppc-coverage-matrix \
  --dataset-root /datasets/PPC-Dataset \
  --rtklib-root output/benchmark \
  --ratio 2.4 \
  --summary-json output/ppc_coverage_matrix/summary.json \
  --markdown-output output/ppc_coverage_matrix/table.md

python3 scripts/update_ppc_coverage_readme.py \
  --summary-json output/ppc_coverage_matrix/summary.json
```

## Full Moving CLAS Comparison

Replay the complete available interval of all six Tokyo/Nagoya runs with the
same kinematic CLAS parity profile. The first command downloads/decodes QZSS L6
corrections, expands the dense SSR input, and runs `gnss_ppp`; allow substantial
disk space and runtime for the six full histories.

```bash
python3 scripts/generate_ppc_clas_scorecard.py \
  --dataset-root data/PPC-Dataset \
  --work-dir output/ppc_clas_full \
  --l6-cache output/ppc_clas_full/l6_cache \
  --configs parity \
  --report output/ppc_clas_full/scorecard.md

python3 scripts/generate_ppc_clas_full_comparison.py \
  --dataset-root data/PPC-Dataset \
  --results-dir output/ppc_clas_full \
  --metrics docs/ppc_clas_full_metrics.json \
  --markdown docs/ppc_clas_full_table.md \
  --trajectory-figure docs/ppc_clas_full_trajectories.png \
  --error-figure docs/ppc_clas_full_errors.png \
  --metric-figure docs/ppc_clas_full_comparison.png
```

Scoring rotates the dataset's city-specific body-frame lever arm through each
PPC attitude sample to compare at the antenna phase center: Tokyo uses
`[0.31, 0.0, -0.55]` m and Nagoya uses
`[0.593, -0.670, -1.216]` m in the PPC vehicle FRD convention (x forward,
y right, z down). The first 60
matched epochs are discarded independently per run, status 6 is FIX, and TTFF
starts at the first 30 consecutive FIX epochs. MRTKLIB's published v0.4.2
figures use the unmodified reference point, so README comparisons label that
reference-definition difference explicitly.

## Audited KF/FGO Goal Matrix And gici-open

The README goal matrix uses all six public Tokyo/Nagoya runs. The selected POS
paths and complete per-run metrics are recorded in
[`ppc_kf_fgo_goal_metrics.json`](ppc_kf_fgo_goal_metrics.json). Its final
Tokyo 1 tier is a position-only, reference-free choice between the preceding
selected trajectory and an independently generated tightly-coupled RTK
trajectory:

```bash
python3 scripts/select_pos_candidate_quality.py \
  --baseline-pos output/tokyo1_selected_quality_rtkbaseline_tier2_truthfree.pos \
  --candidate-pos output/tc_m3_full_t1_on/rtk.pos \
  --output-pos output/tokyo1_selected_quality_tcm3_tier3_truthfree.pos \
  --summary-json output/tokyo1_selected_quality_tcm3_tier3_truthfree.json \
  --candidate-status 4 --candidate-min-ratio 2 \
  --candidate-min-satellites 8 \
  --candidate-max-post-rms-m 0 \
  --candidate-max-nis-per-observation 0 \
  --min-position-separation-m 2
```

This selects 141 candidate positions. It preserves the baseline epoch grid,
status labels, and telemetry; it neither reads nor accepts a reference path.
The preceding tiers use the same invariant. Apply the final truth-free
confidence gate to all six selected paths, then score the resulting outputs
with the common metric implementation:

```bash
python3 scripts/apply_ppc_status_demotion.py \
  --pos tokyo_run1=output/tokyo1_selected_quality_tcm3_tier3_truthfree.pos \
  --pos tokyo_run2=output/tokyo2_selected_quality_rtkbaseline_tier2_truthfree.pos \
  --pos tokyo_run3=output/tokyo3_selected_quality_rtkbaseline_tier2_truthfree.pos \
  --pos nagoya_run1=output/hybrid_nagoya1_multistage_m4_fixedpos_bridge05_vertical025_veld_vertical10_truthfree.pos \
  --pos nagoya_run2=output/nagoya2_selected_quality_rtkbaseline_tier2_truthfree.pos \
  --pos nagoya_run3=output/nagoya3_selected_quality_rtkbaseline_truthfree.pos \
  --output-dir output/ppc_kf_fgo_min_sat9 \
  --min-satellites 9

python3 scripts/summarize_fgo_ppc_matrix.py \
  --dataset-root data/PPC-Dataset \
  --pos tokyo/run1=output/ppc_kf_fgo_min_sat9/tokyo_run1.pos \
  --pos tokyo/run2=output/ppc_kf_fgo_min_sat9/tokyo_run2.pos \
  --pos tokyo/run3=output/ppc_kf_fgo_min_sat9/tokyo_run3.pos \
  --pos nagoya/run1=output/ppc_kf_fgo_min_sat9/nagoya_run1.pos \
  --pos nagoya/run2=output/ppc_kf_fgo_min_sat9/nagoya_run2.pos \
  --pos nagoya/run3=output/ppc_kf_fgo_min_sat9/nagoya_run3.pos \
  --output-json output/kf_fgo_min_sat9_ppc_matrix.json \
  --markdown-output output/kf_fgo_min_sat9_ppc_matrix.md
```

The resulting distance-weighted official score is **78.716546%**. Tokyo 1 is
**83.040559% FIX**. The minimum-satellite gate demotes 374 FIX labels without
changing any position, removing 206 of 1,385 post-run wrong-FIX labels while
retaining the Tokyo 1 public 80.8% FIX target. The separate Nagoya 1 FIX-target
profile is generated by `gnss ppc-demo` with the
`goal_kf_current_r2_min8_rate20_rescue29_8` settings recorded in its summary
JSON; it measures **85.100974% FIX**, **0.912712% Wrong FIX/FIX**, and
**1.459572 m P95 H**. It is intentionally not substituted into the six-run
official-score matrix.

For the competitor reproduction, check out
[`inuex35/gici-open`](https://github.com/inuex35/gici-open) at
`e7666110a88d22e08aad24345a253564af9b8024` (`forppc2024`), run its three
`option/tcN.yaml` configurations against the corresponding Tokyo and Nagoya
inputs, and export each NMEA trajectory. GICI remains a separate GPL-3.0
program; only its NMEA output crosses the benchmark boundary. Convert and score
each output with:

```bash
python3 scripts/convert_gici_nmea_to_pos.py \
  ../gici-open/reproduction_outputs/tokyo1.nmea \
  output/gici/tokyo1.pos --libgnss-status
```

Repeat the conversion for all six runs, then pass the six POS paths to
`summarize_fgo_ppc_matrix.py` with the same `--dataset-root` and `--pos`
layout shown above and `--match-tolerance-s 0.11`. The reproduced GICI macro FIX
rate is **54.325998%**, versus **73.176798%** for the selected libgnss++
matrix. GICI's local Windows/WSL replay required portability and RINEX-reader
fixes in its own GPL worktree; those changes are not copied into this MIT
repository. This is therefore a same-machine reproduction, not a claim that
the pristine upstream checkout runs the PPC data without adaptation.

Regenerate the machine-readable audit and both README figures from the scored
artifacts:

```bash
MPLBACKEND=Agg python3 scripts/generate_ppc_goal_scorecard.py \
  --lib-matrix output/kf_fgo_min_sat9_ppc_matrix.json \
  --gici-matrix output/gici_reproduction_ppc_matrix.json \
  --nagoya1-public-summary output/goal_kf_current_r2_min8_rate20_rescue29_8/summary.json \
  --summary-json docs/ppc_kf_fgo_goal_metrics.json \
  --comparison-png docs/ppc_libgnss_gici_comparison.png \
  --targets-png docs/ppc_public_targets.png \
  --status-demotion-min-satellites 9

MPLBACKEND=Agg python3 scripts/plot_ppc_status_trajectories.py \
  --dataset-root data/PPC-Dataset \
  --metrics-json docs/ppc_kf_fgo_goal_metrics.json \
  --output docs/ppc_kf_fgo_fix_status_xy.png \
  --title "PPC 2024 selected trajectories by solution status" \
  --subtitle "Min 9 satellites: correct FIX, wrong FIX (>0.5 m 3D), FLOAT, and other states"
```

The selector thresholds were tuned on this public benchmark. The audit proves
that truth is not consumed during selection and that FIX labels are not
inflated; it does not turn the public runs into a held-out validation set.

## Historical Diagnostics

Older selector, residual-reset, IMU bridge, and tail-cleanup sweeps are kept as
benchmark diagnostics rather than README sign-off commands. See
[Benchmarks](benchmarks.md) for the scorecards and reproduction commands, and
[PPC realtime gate outputs](ppc_realtime_gate_existing_outputs.md) for the
accepted/rejected runtime status-demotion profiles.

## Provenance

The PPC summary records `receiver_observation_provenance` for the bundled
survey-grade rover/base RINEX streams. Proprietary receiver-engine solutions are
not assumed to be part of the PPC benchmark target.

RTK ionosphere sweeps can be run through `ppc-demo`, `ppc-rtk-signoff`, or
`ppc-coverage-matrix` with `--iono auto|off|iflc|est`; PPC summaries record the
requested value as `rtk_iono`. Ambiguity-ratio sweeps use `--ratio <value>`;
PPC summaries record the requested value as `rtk_ratio_threshold`.

Dataset source: [taroz/PPC-Dataset](https://github.com/taroz/PPC-Dataset)
