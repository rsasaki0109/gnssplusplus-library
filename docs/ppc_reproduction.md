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
  --config-toml configs/ppc_sigma_demote_nis2_ratio4.toml
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
  --config-toml configs/ppc_sigma_demote_nis2_ratio4.toml \
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
