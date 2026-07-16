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
