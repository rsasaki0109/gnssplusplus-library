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

Use `ppc-coverage-matrix` to replay the full six-run `sigma-demote nis2`
profile:

```bash
python3 apps/gnss.py ppc-coverage-matrix \
  --dataset-root /datasets/PPC-Dataset \
  --preset low-cost \
  --ratio 2.8 \
  --carrier-phase-sigma 0.001 \
  --max-postfix-rms 0.2 \
  --max-consec-float-reset 10 \
  --max-subset-ar-drop-steps 18 \
  --adaptive-dynamic-slip-thresholds \
  --adaptive-dynamic-slip-nonfix-count 25 \
  --max-pos-jump 5.0 \
  --max-pos-jump-min 5.0 \
  --max-pos-jump-rate 25.0 \
  --demote-fixed-status-nis-per-obs 2 \
  --output-dir output/ppc_sigma_profile_runtime_demote_nis2 \
  --summary-json output/ppc_sigma_profile_runtime_demote_nis2/summary.json \
  --markdown-output output/ppc_sigma_profile_runtime_demote_nis2/table.md
```

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
