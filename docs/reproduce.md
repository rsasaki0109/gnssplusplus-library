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
| RTK: PPC Tokyo/Nagoya vs RTKLIB `demo5` | `rtk-demo5` | ready | ~13 min | **Pass.** README refreshed 2026-09-28; see [README refresh](#readme-refresh-2026-09-28) |
| CLAS PPP: six PPC runs vs MRTKLIB CLAS | `clas-ppc` | ready | ~45 min | **Pass.** Reproduces 25.121% FIX, 0.359 m FIX RMS2D, 0 FIX > 3 m, 58,259 epochs, all hard gates (L6/SSR expansion ~14 min + six `gnss_ppp` runs ~30 min) |
| Urban RTK: UrbanNav Odaiba vs RTKLIB `demo5` | `odaiba` | ready | ~4 min | **Pass.** README refreshed 2026-09-28; see [README refresh](#readme-refresh-2026-09-28) |
| SPP: PPC adaptive robust + policy gate | `spp-policy` | ready | ~4 min | **Pass.** No P95 regression on 4/4 runs; drop <= 0.98 pp |
| GNSS/IMU FGO: PPC Tokyo vs `tightly-coupled-gnss-imu-fgo` | `fgo-tokyo` | ready | ~35 min | **Pass.** Comparison table and GF-reset column reproduce exactly; the GF-reset baseline Tokyo run3 row does not (reported, not gated); see [fgo-tokyo result](#fgo-tokyo-local-result-2026-09-28) |
| PPC 2024 goal matrix vs Kaiyodai and gici-open | `ppc-goal` | planned | - | See [PPC reproduction](ppc_reproduction.md) |
| Smartphone dev routes (base-surveyed) | `gsdc-dev-routes` | planned | - | - |
| Smartphone GSDC official submission | `gsdc-official` | planned | - | The score comes from Kaggle and cannot be recomputed locally |

Runtimes were measured on a 12-thread Windows 11 workstation with an MSVC
Release build. Lanes that run in parallel slow each other down.

## Datasets

| Dataset | Used by | Get it | Expected layout |
|---|---|---|---|
| [PPC-Dataset](https://github.com/taroz/PPC-Dataset) | `rtk-demo5`, `clas-ppc`, `spp-policy`, `fgo-tokyo` | `git clone https://github.com/taroz/PPC-Dataset` | `<ppc-root>/{tokyo,nagoya}/run{1,2,3}/{rover.obs,base.obs,base.nav,reference.csv}`; `fgo-tokyo` also reads `tokyo/run{1,2,3}/imu.csv` |
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

**GTSAM build (`fgo-tokyo` only).** `gnss_fgo_parity` needs GTSAM 4.3.x
(see `AGENTS.md`). Build it in a separate tree and pass that tree with
`--build-dir`:

```bash
cmake -S . -B build-gtsam -DCMAKE_BUILD_TYPE=Release -DGNSSPP_BUILD_PYTHON_BINDINGS=OFF   -DGTSAM_DIR=<gtsam-prefix>/lib/cmake/GTSAM
cmake --build build-gtsam --target gnss_fgo_parity --parallel 2
```

At runtime the GTSAM shared libraries must be found: on Linux append their
directory to `LD_LIBRARY_PATH`; on Windows set `GTSAM_BIN_DIR` to the directory
holding the GTSAM DLLs (the lane's run wrapper prepends it to `PATH`). Each
replay peaks at 1.8-3.1 GB resident memory, and the lane runs them one at a
time.

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

# GNSS/IMU tightly-coupled FGO vs tightly-coupled-gnss-imu-fgo (GTSAM build)
python3 apps/gnss.py reproduce fgo-tokyo --ppc-root /datasets/PPC-Dataset   --build-dir build-gtsam --check
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
| `--update-docs` | Also regenerate the tracked docs artifacts the lane owns, such as the `docs/benchmarks.md` coverage block, `docs/ppc_rtk_demo5_scorecard.png`, `docs/ppc_clas_full_*`, the Odaiba figures, and `docs/gnss_imu_fgo_tokyo_run{1,2,3}.png` |

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

## README refresh (2026-09-28)

The first local run (develop `bcb1aac`, MSVC Release, demo5 b34k) did not
reproduce two README rows, because their RTKLIB baselines had been produced
outside the repository with an unrecorded configuration. The README rows and
`docs/benchmarks.md` now show the reproduced values, and the lanes gate on them.

**`rtk-demo5`.**

| Metric | Previous README | Reproduced (now in README) |
|---|---:|---:|
| Avg Fix-rate delta vs demo5 | - | **+56.8 pp** |
| Avg PPC official-score delta | +28.1 pp | **+45.4 pp** |
| Avg P95 H delta | -11.96 m | **-11.44 m** |
| Avg Positioning delta | +17.0 pp | **-9.5 pp** |

With the tracked b34k config, demo5 publishes a FLOAT or SINGLE solution for
98.6-100% of epochs (historical table: 65.8-93.1%) but fixes only 5.6-43.6%.
The Positioning comparison therefore flips sign while the Fix-rate and
official-score leads grow. Variants tried to recover the historical demo5
columns (GPS-only, GPS+GAL+QZS, fix-and-hold, 10 degree mask, base time
interpolation off) did not match.

**`odaiba`.**

| Claim | demo5 b34k | libgnss++ default | Result |
|---|---:|---:|---|
| More fixes | 209 | **922** | pass |
| Lower Hp95 | 26.26 m | **5.10 m** | pass |
| Lower Vp95 | 43.29 m | **15.10 m** | pass |
| Lower Hmed on common epochs (7,996) | 0.671 m | **0.659 m** | pass |

The previous claim "`--preset odaiba` closes Hmed" no longer holds on all
matched epochs (0.709 m vs 0.684 m) and the preset now yields only 54 fixes,
so the README no longer cites it; the lane still runs it as a reported
diagnostic. The previous snapshot table (demo5 595 fixes, default 1268, preset
735) was likewise produced with an unrecorded RTKLIB build.

**`spp-policy`.** The README claim reproduces. The historical per-run policy
P95 H values in `docs/references/spp-accuracy-improvement.md` are within
0.07 m and are reported without gating.

## fgo-tokyo local result (2026-09-28)

First local run of the `fgo-tokyo` lane: develop `46c9af27` plus this lane,
MSVC Release, GTSAM 4.3, Windows 11. The lane replays the README preset with
`--gf-slip-reset` (`gf_reset`) and without it (`baseline`) on Tokyo runs 1-3.
Replays are deterministic: a second baseline run3 wrote a byte-identical
`--dump-csv`.

**README comparison table** (`gf_reset`; the reference columns are the
published tightly-coupled-gnss-imu-fgo values):

| Run | <50 cm README / local | Fix README / local | Fixed RMS README / local | Wall time (Windows) |
|---|---:|---:|---:|---:|
| Tokyo run1 | 54.9% / 54.89% | 53.8% / 53.83% | 1.180 / 1.1806 m | 325 s |
| Tokyo run2 | 85.7% / 85.70% | 78.6% / 78.61% | 0.109 / 0.1092 m | 241 s |
| Tokyo run3 | 77.5% / 77.53% | 69.3% / 69.30% | 0.125 / 0.1252 m | 277 s |

The claims (higher FIX rate on 3/3 runs, avg +10.7 pp; <50 cm on 2/3, avg
+7.9 pp; fixed RMS on 2/3) reproduce. The README wall times were 463.5,
584.6, and 844.9 s on the Linux validation host.

**GF-reset table.** Every `gf_reset` cell, the GF guard demotions (14/0/17),
the aggregate Wrong FIX/FIX after the reset (11.759%), and the matched
distance (99.682%) reproduce to three decimals. The baseline rows for runs 1-2
also match. The baseline Tokyo run3 row did not, so the README row and the
aggregates it feeds were refreshed on 2026-09-29 and are now gated:

| Baseline Tokyo run3 | Previous README | Reproduced (now in README) |
|---|---:|---:|
| Correct FIX distance | 59.175% | 59.712% |
| Wrong FIX distance | 7.918% | 7.726% |
| Official score | 64.081% | 65.446% |
| Fixed-only horizontal RMS | 0.257 m | 1.517 m |

Aggregate baseline: 49.181 / 13.741 / 54.178% -> 49.440 / 13.649 / 54.837%;
baseline Wrong FIX/FIX: 21.839% -> 21.634%. The previous baseline was produced
before the GF-reset commit (`9b058572`) with an unrecorded tree. The lane also
gates the GF-reset improvement: aggregate official score +8.855 pp and
wrong-FIX distance -5.999 pp.

**Surplus-satellite rescue table: not reproduced.** It was measured in commit
`6ede956c` with an earlier preset (`--imu-preset-tactical --cp-hold-res 2.0
--fix-demote-dist 5`). The "before" configuration was not recorded, and the
solver has changed since then.

`docs/gnss_imu_fgo_tokyo_run{1,2,3}.png` were regenerated from this run with
`--update-docs`. The previously tracked figures predated the GF reset (for
example, run1 showed fix 50.0%, <50 cm 56.9%, and fixed RMS 0.66 m).
