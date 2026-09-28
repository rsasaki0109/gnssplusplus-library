# Reproduce README Results

`gnss reproduce` reruns the numbers in the README
[Results And Validation Status](https://github.com/rsasaki0109/gnssplusplus-library#results-and-validation-status)
table from tracked lane manifests. One command per README row downloads or
reads the public inputs, runs the solvers, scores the output, and (with
`--check`) compares every metric with the README value and tolerance.

```bash
python3 apps/gnss.py reproduce list
python3 apps/gnss.py reproduce spp-policy --ppc-root /datasets/PPC-Dataset --check
```

Each lane is described by `configs/reproduce/<lane>.toml`: required dataset
files, the exact argv of every step, external tool pins, and the expected
metrics. The steps call existing repository commands and scripts; the lane
runner adds no scoring logic of its own.

## Lane status

| README row | Lane | Status | Runtime (local) | Local result (2026-09-28) |
|---|---|---|---:|---|
| RTK: PPC Tokyo/Nagoya vs RTKLIB `demo5` | `rtk-demo5` | ready | ~13 min | **Drift.** Numbers differ from the README; see [Known discrepancies](#known-discrepancies) |
| CLAS PPP: six PPC runs vs MRTKLIB CLAS | `clas-ppc` | ready | CLAS_RUNTIME | CLAS_RESULT |
| Urban RTK: UrbanNav Odaiba vs RTKLIB `demo5` | `odaiba` | ready | ~4 min | 3 of 4 claims pass; the `--preset odaiba` Hmed claim fails |
| SPP: PPC adaptive robust + policy gate | `spp-policy` | ready | ~4 min | **Pass.** No P95 regression on 4/4 runs; drop <= 0.98 pp |
| GNSS/IMU FGO: PPC Tokyo vs `tightly-coupled-gnss-imu-fgo` | `fgo-tokyo` | planned | - | Needs a GTSAM build and IMU replay lane |
| PPC 2024 goal matrix vs Kaiyodai and gici-open | `ppc-goal` | planned | - | See [PPC reproduction](ppc_reproduction.md) |
| Smartphone dev routes (base-surveyed) | `gsdc-dev-routes` | planned | - | - |
| Smartphone GSDC official submission | `gsdc-official` | planned | - | The score comes from Kaggle and cannot be recomputed locally |

Runtimes were measured on a 12-thread Windows 11 workstation with an MSVC
Release build. Lanes that run in parallel slow each other down.

## Datasets

| Dataset | Used by | Get it | Expected layout |
|---|---|---|---|
| [PPC-Dataset](https://github.com/taroz/PPC-Dataset) | `rtk-demo5`, `clas-ppc`, `spp-policy` | `git clone https://github.com/taroz/PPC-Dataset` | `<ppc-root>/{tokyo,nagoya}/run{1,2,3}/{rover.obs,base.obs,base.nav,reference.csv}` |
| [UrbanNav Tokyo Odaiba](https://github.com/IPNL-POLYU/UrbanNavDataset) | `odaiba` | UrbanNav Tokyo data release (Trimble rover/base RINEX + Applanix reference) | `<urbannav-root>/Odaiba/{rover_trimble.obs,base_trimble.obs,base.nav,reference.csv}` |
| QZSS L6 CLAS archive | `clas-ppc` | Downloaded automatically from `https://sys.qzss.go.jp/archives/l6` | Cached under `<work-dir>/inputs/l6_cache` (about 1.7 GB of expanded SSR CSV per run) |

Dataset roots are resolved in this order:

1. `--ppc-root` / `--urbannav-root`
2. `GNSSPP_PPC_DATASET_ROOT` / `GNSSPP_URBANNAV_ROOT`
3. `--data-root <dir>` expands to `<dir>/PPC-Dataset` and `<dir>/driving/Tokyo_Data`
4. `data/PPC-Dataset` and `data/driving/Tokyo_Data` inside the repository

Before running, each lane checks that the files it needs exist. `--dry-run`
reports missing files as warnings.

## Tools

**libgnss++ build.** The lanes use a non-GTSAM Release build and need only
`gnss_spp`, `gnss_solve`, and `gnss_ppp`:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DGNSSPP_BUILD_PYTHON_BINDINGS=OFF
cmake --build build --target gnss_spp gnss_solve gnss_ppp --parallel
```

For an out-of-tree build, pass `--build-dir <dir>` or set `GNSSPP_BUILD_DIR`.
The dispatcher also looks up `gnss spp` / `gnss solve` binaries through
`GNSSPP_BUILD_DIR`. On Windows, configure from a `vcvars64` shell with the
vcpkg toolchain (`-DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake
-DVCPKG_TARGET_TRIPLET=x64-windows`).

**RTKLIB demo5.** The `rtk-demo5` and `odaiba` lanes compare against
[rtklibexplorer/RTKLIB](https://github.com/rtklibexplorer/RTKLIB) pinned to
tag `b34k`, commit `55a0f2c742a605a3d27a90b17556b2cfdf98ad50`:

```bash
curl -L -o b34k.tar.gz https://github.com/rtklibexplorer/RTKLIB/archive/refs/tags/b34k.tar.gz
tar xzf b34k.tar.gz
make -C RTKLIB-b34k/app/consapp/rnx2rtkp/gcc
export RTKLIB_RNX2RTKP=$PWD/RTKLIB-b34k/app/consapp/rnx2rtkp/gcc/rnx2rtkp
```

On Windows without gcc, compile the same sources with MSVC. Use the makefile
defines `-DTRACE -DENAGLO -DENAQZS -DENAGAL -DENACMP -DENAIRN -DNFREQ=3
-DNEXOBS=3` and link `winmm.lib ws2_32.lib`. The b34k binary identifies itself
as `demo5 b34j` in `.pos` headers. The solver options are tracked in
`configs/reproduce/rtklib_demo5_ppc.conf` (PPC) and `scripts/rtklib_odaiba.conf`
(Odaiba).

## One command per README result

```bash
# RTK coverage profile vs RTKLIB demo5 (six PPC runs)
python3 apps/gnss.py reproduce rtk-demo5 --ppc-root /datasets/PPC-Dataset \
  --rtklib-bin "$RTKLIB_RNX2RTKP" --check

# CLAS PPP-RTK vs MRTKLIB v0.4.2 (downloads QZSS L6 automatically)
python3 apps/gnss.py reproduce clas-ppc --ppc-root /datasets/PPC-Dataset --check

# UrbanNav Odaiba RTK vs RTKLIB demo5, default and --preset odaiba
python3 apps/gnss.py reproduce odaiba --urbannav-root /datasets/UrbanNav/Tokyo_Data \
  --rtklib-bin "$RTKLIB_RNX2RTKP" --check

# SPP adaptive robust weighting + 1 pp policy gate
python3 apps/gnss.py reproduce spp-policy --ppc-root /datasets/PPC-Dataset --check
```

Common options:

| Option | Meaning |
|---|---|
| `--work-dir` | Output directory (default `output/reproduce/<lane>`) |
| `--build-dir` | Build tree that holds `apps/gnss_*` (default `$GNSSPP_BUILD_DIR`, then `build*/`) |
| `--rtklib-bin` | `rnx2rtkp` for the demo5 lanes (default `$RTKLIB_RNX2RTKP`) |
| `--dry-run` | Print the rendered commands and dataset warnings without running anything |
| `--check` | Exit with status 3 when a gated metric drifts from the manifest expectation |
| `--check-only` | Skip the steps and re-check the metrics already in `--work-dir` |
| `--update-docs` | Also regenerate the tracked docs artifacts the lane owns, such as the `docs/benchmarks.md` coverage block, `docs/ppc_rtk_demo5_scorecard.png`, `docs/ppc_clas_full_*`, and the Odaiba figures |

Every run writes `<work-dir>/reproduce_result.json` with per-step wall times,
observed values, and the pass or fail state of each metric. It also writes
one log per step under `<work-dir>/logs/`.

## Manifest format

```toml
[lane]
name = "spp-policy"            # CLI lane name
status = "ready"               # or "planned"
title = "..."
readme_row = "..."
runtime_estimate = "~5 min"

[datasets.ppc]                 # ppc -> --ppc-root, urbannav -> --urbannav-root
required = ["tokyo/run1/rover.obs"]

[[steps]]                      # run in order, cwd = repository root
name = "spp-{city}_{run}"
foreach = [{ city = "tokyo", run = "run1" }]
argv = ["{gnss}", "spp", "--obs", "{ppc_root}/{city}/{run}/rover.obs", "--out", "{work_dir}/x.pos"]
env = { GNSS_PPP_CLAS_SIS_BOUNDARY = "1" }

[[docs_steps]]                 # only with --update-docs
name = "..."
argv = ["{python}", "scripts/..."]

[[metrics]]
name = "{label} p95 delta"
source = "{work_dir}/report.json"
path = "runs[label={label}].p95_h_delta_m"   # dotted keys, [index], [field=value]
max = 0.0                                     # or expected + abs_tol/rel_tol, min, expected = true
minus_path = "..."                            # optional: gate on path - minus_path
gate = true                                   # false = report only
foreach = [{ label = "tokyo_run1" }]
```

Placeholders: `{gnss}` (Python + `apps/gnss.py`), `{python}`, `{work_dir}`,
`{ppc_root}`, `{urbannav_root}`, `{rtklib_bin}`, `{build_dir}`, and
`{bin:NAME}` (a built binary from `--build-dir`). A `foreach` row defines
additional placeholders for its step or metric.

## Known discrepancies

These are the latest local results (2026-09-28, develop `bcb1aac` + this lane
framework, MSVC Release build, demo5 b34k). The README text has not been
changed; the lanes keep the README values as their expectations, so
`--check` fails for these rows until the README is updated or the historical
inputs are recovered.

**`rtk-demo5`.**

| Metric | README | Reproduced |
|---|---:|---:|
| Avg Positioning delta vs demo5 | +17.0 pp | **-9.47 pp** |
| Avg PPC official-score delta | +28.1 pp | **+45.42 pp** |
| Avg P95 H delta | -11.96 m | **-11.44 m** |

The historical RTKLIB solutions (`output/benchmark/<run>/rtklib.pos`) were made
outside the repository with an unrecorded configuration. With the tracked b34k
config, demo5 outputs a float or single solution for 98.6-100% of epochs,
against 65.8-93.1% in the historical table, but it fixes rarely (5.6-43.6%).
The Positioning lead therefore flips sign, while the official-score lead grows.
Current gnssplusplus also differs from the historical table: for example,
Tokyo run1 Fix is 83.7% against 54.4%, and Nagoya run3 Positioning is 83.0%
against 93.8%. Variants tried without matching the historical demo5 columns
include GPS-only, GPS+GAL+QZS, fix-and-hold, a 10 degree mask, and base time
interpolation off.

**`odaiba`.** The README claims are gated on all matched epochs:

| Claim | demo5 b34k | libgnss++ default | libgnss++ `--preset odaiba` | Result |
|---|---:|---:|---:|---|
| More fixes | 209 | **922** | 54 | pass (default) |
| Lower Hp95 | 26.26 m | **5.10 m** | 5.11 m | pass |
| Lower Vp95 | 43.29 m | **15.10 m** | 15.18 m | pass |
| `--preset odaiba` closes Hmed | **0.684 m** | 0.696 m | 0.709 m | **fail** (+0.024 m) |

On common epochs, the preset does beat demo5 on Hmed (0.632 m vs 0.673 m).
The docs/benchmarks.md snapshot table (demo5 595 fixes, default 1268, preset
735) does not reproduce with the current solver and demo5 b34k.

**`spp-policy`.** The README claim reproduces. The historical per-run policy
P95 H values in `docs/references/spp-accuracy-improvement.md` are within
0.07 m and are reported without gating.
