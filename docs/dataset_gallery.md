# Dataset Gallery

This page is the adoption map for public and internal datasets. A dataset is
useful to libgnss++ when it has a repeatable command, a clear reference source,
and an artifact contract.

## Current lanes

| Dataset | Status | Best entrypoint | Reference | Main use |
|---|---|---|---|---|
| PPC-Dataset Tokyo/Nagoya | bundled primary RTK sign-off | `gnss ppc-rtk-signoff` | `reference.csv` trajectory truth | Moving RTK coverage and fix quality |
| UrbanNav Tokyo Odaiba/Shinjuku | path-override public smoke | `gnss ppc-rtk-signoff --run-dir ...` | Applanix reference export | Urban canyon RTK stress |
| smartLoc urban GNSS | receiver-fix lane | `gnss smartloc-adapter`, `gnss smartloc-signoff` | NovAtel SPAN differential RTK/IMU | NLOS receiver-fix analysis |
| Google Smartphone Decimeter Challenge | candidate | new adapter needed | competition ground truth | Smartphone raw GNSS and IMU stress |
| Ford Highway Driving RTK | candidate | new adapter needed | INS plus survey-grade GNSS | Highway-scale coverage |
| Oxford RobotCar RTK ground truth | candidate | new adapter needed | post-processed GPS/IMU reference | Long-term localization reference |

Use:

```bash
python3 apps/gnss.py public-rtk-benchmarks --format markdown
```

to print the current public benchmark inventory and caveats.

## Bundled PPC-Dataset

The repo includes a PPC-Dataset checkout under `data/PPC-Dataset`. Each run has
the same structure:

```text
data/PPC-Dataset/<city>/<run>/
  rover.obs
  base.obs
  base.nav
  reference.csv
  imu.csv
  trajectory.kml
```

Run a short smoke:

```bash
python3 apps/gnss.py ppc-rtk-signoff \
  --dataset-root data/PPC-Dataset \
  --city nagoya \
  --run run1 \
  --max-epochs 200 \
  --summary-json output/adoption/ppc_nagoya_run1_smoke.json
```

Run the six-run matrix when evaluating a profile:

```bash
python3 apps/gnss.py ppc-coverage-matrix \
  --dataset-root data/PPC-Dataset \
  --summary-json output/adoption/ppc_coverage_matrix.json
```

## External RTK dataset pattern

When a dataset can be mapped to rover/base/nav/reference files, keep the
adapter thin and run the normal sign-off path:

```bash
python3 apps/gnss.py ppc-rtk-signoff \
  --run-dir /datasets/my_rtk_dataset/run1 \
  --rover /datasets/my_rtk_dataset/run1/rover.obs \
  --base /datasets/my_rtk_dataset/run1/base.obs \
  --nav /datasets/my_rtk_dataset/run1/base.nav \
  --reference-csv /datasets/my_rtk_dataset/run1/reference.csv \
  --out output/my_rtk_dataset/run1.pos \
  --summary-json output/my_rtk_dataset/run1.json
```

Only write a new adapter when the dataset layout, timestamp convention, or
reference format cannot be expressed with path overrides.

## Dataset adapter contract

A new adapter should produce a normalized run directory with:

| File | Required | Notes |
|---|---|---|
| `rover.obs` | yes | RINEX observation file for the rover |
| `base.obs` | RTK only | RINEX observation file for the base station |
| `base.nav` or `navigation.nav` | yes | Broadcast navigation file |
| `reference.csv` | sign-off runs | Time-aligned truth or reference trajectory |
| `receiver_solution.csv` | optional | Commercial or receiver-engine comparison |
| `provenance.json` | recommended | Receiver, antenna, source URL, conversion command |

The adapter should also document:

- source dataset version or download date,
- coordinate frame and time scale,
- antenna and receiver metadata if available,
- any filtering or interpolation applied to reference truth,
- the exact command that produces the normalized directory.

## Promotion rule

Treat a dataset lane as stable only when it has:

1. one smoke command with `--max-epochs`,
2. one full sign-off command,
3. at least one checked `summary.json`,
4. docs that explain the reference source and caveats.

This keeps public claims tied to commands instead of screenshots or ad hoc
logs.
