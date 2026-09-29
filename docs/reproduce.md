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
| PPC 2024 goal matrix vs Kaiyodai and gici-open | `ppc-goal` | ready (score-only) | ~1 min | **Pass.** Replays the truth-free post-processing chain from 26 SHA-256-pinned tier inputs and reproduces every README number exactly (78.845491%, the six-run libgnss++/gici-open table, Nagoya 1 85.100974%); the solver outputs at the bottom of the chain are frozen, not regenerated; see [ppc-goal result](#ppc-goal-local-result-2026-09-29) |
| Smartphone dev routes (base-surveyed) | `gsdc-dev-routes` | ready | ~25 min | **Pass.** README refreshed 2026-09-29 to the reproduced H 0.576 / U 0.740 / A 0.303 / LAX-T 0.716 m (previously 0.577 / 0.738 / 0.302 / 0.712); see [gsdc-dev-routes result](#gsdc-dev-routes-local-result-2026-09-29) |
| Smartphone GSDC official submission | `gsdc-official` | ready | ~30 min per drive; full run ~12-18 h | **Subset verified (by design).** Gate: the rebuilt `submission.csv` is byte-identical to Kaggle ref 56625084 (`cbd1fde1...`), and the Kaggle score is readback-only. 6 of 40 final drives and 2 of 25 stage-0 drives were rerun, and all were byte-identical; see [gsdc-official](#gsdc-official-rebuilding-the-kaggle-submission) |

Runtimes were measured on a 12-thread Windows 11 workstation with an MSVC
Release build. Lanes that run in parallel slow each other down.

## Datasets

| Dataset | Used by | Get it | Expected layout |
|---|---|---|---|
| [PPC-Dataset](https://github.com/taroz/PPC-Dataset) | `rtk-demo5`, `clas-ppc`, `spp-policy`, `fgo-tokyo`, `ppc-goal` | `git clone https://github.com/taroz/PPC-Dataset` | `<ppc-root>/{tokyo,nagoya}/run{1,2,3}/{rover.obs,base.obs,base.nav,reference.csv}`; `fgo-tokyo` also reads `tokyo/run{1,2,3}/imu.csv` |
| [UrbanNav Tokyo Odaiba](https://github.com/IPNL-POLYU/UrbanNavDataset) | `odaiba` | UrbanNav Tokyo data release (Trimble rover/base RINEX + Applanix reference) | `<urbannav-root>/Odaiba/{rover_trimble.obs,base_trimble.obs,base.nav,reference.csv}` |
| [GSDC 2023 `dataset_2023`](https://github.com/taroz/gsdc2023) (Kaggle Google Smartphone Decimeter Challenge 2023 train and test sets with CORS base RINEX and `brdc.nav`) | `gsdc-dev-routes`, `gsdc-official` | Kaggle GSDC 2023 data as repackaged by taroz/gsdc2023 (`dataset_2023.zip`, SHA-256 `bda30ab4...`) | `<gsdc-root>/train/<drive>/{brdc.nav,<BASE>_rnx2.obs,pixel5/{device_gnss.csv,device_imu.csv,ground_truth.csv}}`, or the zip itself; see [gsdc-dev-routes inputs](#gsdc-dev-routes-inputs-and-base-provenance). `gsdc-official` reads `<gsdc-root>/test/<drive>/{brdc.nav,<BASE>_rnx2.obs,<phone>/{device_gnss.csv,device_imu.csv}}` |
| Kaggle GSDC 2023 train `ground_truth.csv` | `gsdc-official` (height maps) | Kaggle `smartphone-decimeter-2023` competition data (156 train files, SHA-256 pinned in the recipe) | `--gsdc-truth-root` holding `train/<course>/<phone>/ground_truth.csv` or flat `<course>__<phone>__ground_truth.csv` |
| PPC goal-matrix frozen tier inputs | `ppc-goal` | Not published; the 26 files (~33 MB) exist only in the `output/` tree of the checkout that produced the README. See [ppc-goal inputs](#ppc-goal-frozen-inputs) | `<ppc-goal-inputs>/` in the historical `output/` layout (`tokyo1_selected_quality_rtkbaseline_tier2_truthfree.pos`, `gici_common/tokyo1.pos`, ...); SHA-256 pinned in `scripts/experiments/ppc/stage_ppc_goal_inputs.py` |
| QZSS L6 CLAS archive | `clas-ppc` | Downloaded automatically from `https://sys.qzss.go.jp/archives/l6` | Cached under `<work-dir>/inputs/l6_cache` (about 1.7 GB of expanded SSR CSV per run) |

Dataset roots are resolved in this order:

1. `--ppc-root` / `--urbannav-root` / `--gsdc-root` / `--gsdc-truth-root` / `--ppc-goal-inputs`
2. `GNSSPP_PPC_DATASET_ROOT` / `GNSSPP_URBANNAV_ROOT` / `GNSSPP_GSDC_ROOT` / `GNSSPP_GSDC_TRUTH_ROOT` / `GNSSPP_PPC_GOAL_INPUTS`
3. `--data-root <dir>` expands to `<dir>/PPC-Dataset`, `<dir>/driving/Tokyo_Data`, `<dir>/gsdc2023/dataset_2023` and `<dir>/ppc_goal_inputs`
4. `data/PPC-Dataset`, `data/driving/Tokyo_Data`, `data/gsdc2023/dataset_2023` and `data/ppc_goal_inputs` inside the repository

The GSDC ground-truth root falls back to the GSDC root when neither
`--gsdc-truth-root` nor `GNSSPP_GSDC_TRUTH_ROOT` is set.

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

**GTSAM build (`fgo-tokyo`, `gsdc-dev-routes`, `gsdc-official`).** `gnss_fgo_parity` and
`gnss_fgo_imu_no_base` (build that target for the `gsdc-*` lanes) need GTSAM 4.3.x
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

# PPC 2024 goal matrix vs gici-open (score-only; frozen tier inputs, no solver)
python3 apps/gnss.py reproduce ppc-goal --ppc-root /datasets/PPC-Dataset   --ppc-goal-inputs /archive/ppc_goal_inputs --check

# GSDC Pixel5 base-surveyed dev routes H/U/A/LAX-T (GTSAM build)
python3 apps/gnss.py reproduce gsdc-dev-routes --gsdc-root /datasets/gsdc2023/dataset_2023 \
  --build-dir build-gtsam --check

# GSDC 2023-2024 Kaggle submission rebuild, SHA-256 gate (GTSAM build; hours)
python3 apps/gnss.py reproduce gsdc-official --gsdc-root /datasets/gsdc2023/dataset_2023 \
  --gsdc-truth-root /datasets/gsdc2023/kaggle_train_gt --build-dir build-gtsam --check
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
| `--update-docs` | Also regenerate the tracked docs artifacts the lane owns, such as the `docs/benchmarks.md` coverage block, `docs/ppc_rtk_demo5_scorecard.png`, `docs/ppc_clas_full_*`, the Odaiba figures, `docs/gnss_imu_fgo_tokyo_run{1,2,3}.png`, `docs/ppc_kf_fgo_goal_metrics.json` with `docs/ppc_libgnss_gici_comparison.png`, `docs/ppc_public_targets.png` and `docs/ppc_kf_fgo_fix_status_xy.png`, and `docs/gsdc_base_surveyed_osm.png` (downloads OpenStreetMap tiles) |

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

[datasets.ppc]                 # ppc, urbannav, gsdc, gsdc_truth -> --<name>-root
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
`{ppc_root}`, `{urbannav_root}`, `{gsdc_root}`, `{gsdc_truth_root}`, `{rtklib_bin}`, `{build_dir}`, and
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

The lane also runs `--preset odaiba` (the `low-cost` profile plus
arc-smoothed wide-lane AR) and gates it against the default arm:

| Claim | libgnss++ default | `--preset odaiba` | Result |
|---|---:|---:|---|
| More fixes | 922 | **6086** | pass |
| Lower Hp95 | 5.10 m | **4.87 m** | pass |
| Lower Vp95 | 15.10 m | **13.49 m** | pass |
| Hmed (reported, not gated) | **0.696 m** | 0.716 m | - |

Hmed is reported only. All solvers' fixed epochs sit about 0.70 m from
`reference.csv`, so Hmed is floored by that common offset and drops as FLOAT
epochs are added. The README's earlier claim that "`--preset odaiba` closes
Hmed" is not restored.

Before 2026-09-29 the preset produced 54 fixes. It was a stale copy of the
early `low-cost` values, and it fixed wide-lane integers from a single noisy
MW epoch. See the [Odaiba snapshot](benchmarks.md#urbannav-tokyo-odaiba-snapshot).
The previous snapshot table (demo5 595 fixes, default 1268, preset 735) was
produced with an unrecorded RTKLIB build.

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

## gsdc-dev-routes inputs and base provenance

The README base-surveyed table was measured with research harnesses that
were removed in PR #510 (phase37 / phase25 / phase63; they remain in the tag
`archive/research-phase-2026-09-28`). Those harnesses did not transform any
input. They extracted members byte-for-byte from the taroz `dataset_2023`
archive (`dataset_2023.zip`, SHA-256
`bda30ab456e6fd6f83550c246e8dbd287306d5385f1f1069c99c16298e647408`).
`scripts/experiments/gsdc/stage_gsdc_dev_route_inputs.py` replaces them. It
copies or hard-links each member into `<work-dir>/inputs/<route>/` and checks
it against the SHA-256 pinned in the research records. The stager reads an
extracted `dataset_2023` tree or the zip itself.

| Route | Drive (Pixel5) | Base RINEX | Surveyed base ECEF (m) |
|---|---|---|---|
| H | `2021-08-24-20-32-us-ca-mtv-h` | `P221_rnx2.obs` (`4d3e37cb...`) | P221 2021: -2698117.9416 -4301326.2649 3847286.2750 |
| U | `2023-03-08-21-34-us-ca-mtv-u` | `P221_rnx2.obs` (`aedb7a39...`) | P221 2023: -2698117.9861 -4301326.2071 3847286.2977 |
| A | `2021-03-16-18-59-us-ca-mtv-a` | `SLAC_rnx2.obs` (`380b8ff9...`) | SLAC 2021: -2703116.3177 -4291766.7551 3854248.0736 |
| LAX-T | `2022-04-01-18-22-us-ca-lax-t` | `LBCH_rnx2.obs` (`d731e0e8...`) | LBCH 2022: -2507799.2243 -4676369.3031 3526891.0358 |

The base RINEX files are the CORS stations that ship with `dataset_2023`,
the same files the phase63 harness passed to `--native-base-rinex`. The
surveyed coordinates are the year-matched rows of `base/base_position.csv` in
the same dataset ([taroz/gsdc2023](https://github.com/taroz/gsdc2023)). When
that file is present, the stager cross-checks the values it passes to
`--native-base-position-ecef`. Ground truth (`pixel5/ground_truth.csv`, used
only for scoring) is read from the GSDC root. If the tree has none, it is read
from `--gsdc-truth-root`, either nested or as flat
`<drive>__pixel5__ground_truth.csv` files.

Recipes: H, U, and A use the flag set printed in the
[record](use_cases/records/smartphone_base_surveyed_route_results_v1.md).
LAX-T uses the phase476 LAX-T argv (`--android-include-first-native-epoch
--native-sparse-p-staging`) with the phase538 `--native-joint-ionosphere 3
0.02 1.5`, plus the record's base-compensation flags. Both sources were
recovered from the archive tag. The score is the horizontal Haversine error
(R = 6371008.8 m) on an exact `UnixTimeMillis` inner join, reported as
`(P50 + P95) / 2` with linearly interpolated percentiles.

## gsdc-dev-routes local result (2026-09-29)

This was the first local run of the lane: develop `4b22fe43` plus this lane,
built with MSVC Release and GTSAM 4.3 on Windows 11, one solver process at a
time.

| Route | Previous README | Record | Reproduced (now in README) | P50 | P95 | Mean | Wall time | Peak RSS |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| H | 0.577 | 0.57738 | **0.57623** | 0.3671 | 0.7854 | 0.4245 | 293 s | 0.99 GB |
| U | 0.738 | 0.73751 | **0.74045** | 0.6260 | 0.8549 | 0.6194 | 151 s | 0.33 GB |
| A | 0.302 | 0.30169 | **0.30265** | 0.2142 | 0.3911 | 0.2179 | 290 s | 0.65 GB |
| LAX-T | 0.712 | 0.71207 | **0.71586** | 0.5795 | 0.8522 | 0.7092 | 655 s | 1.03 GB |

Every route reproduces the README value to within 4 mm. None reproduces the
record exactly. The replay is deterministic: two H runs wrote byte-identical
solutions. A Windows build of `274ab819`, the tree that recorded the table,
scores H at 0.57643 m. About 1 mm of the gap therefore comes from the build
(MSVC and the Windows GTSAM build, versus the Linux GCC build used for the
record), and about 0.2 mm from later solver changes. The README table now
shows the reproduced values and the lane gates them with a 2 mm
cross-toolchain tolerance; the 5-decimal record values are reported without
gating. U and A have no solution for
their first truth epoch because the H/U/A recipe omits
`--android-include-first-native-epoch`, so the join scores 1101/1102 and
2158/2159 epochs.

`--update-docs` redraws `docs/gsdc_base_surveyed_osm.png` from the reproduced
tracks. The tracks and worst-epoch insets match the tracked figure (H 0.81 m,
U 0.95 m, A 0.52 m, LAX-T 5.74 m vs 5.72 m). The panel titles show the
reproduced scores; the tracked figure was replaced with this redraw so it
matches the refreshed README table.

## ppc-goal frozen inputs

The README goal matrix is the end of a long chain of truth-free selectors
built on July 2026 solver outputs. The final steps of that chain are
documented in [PPC reproduction](ppc_reproduction.md), but the commands that
produced its bottom layer were never recorded. That layer includes the tier-2
selected KF trajectories, the tightly-coupled candidates, the FGO shadow
windows, the gici-open runs and the Nagoya 1 FIX-target solution. The lane
therefore starts from 26 frozen files and pins each with SHA-256 in
`scripts/experiments/ppc/stage_ppc_goal_inputs.py`:

| Group | Files | Size |
|---|---|---:|
| Tier-2 / selected KF trajectories | `{tokyo1,tokyo2,tokyo3,nagoya2}_selected_quality_rtkbaseline_tier2_truthfree.pos`, `nagoya3_selected_quality_rtkbaseline_truthfree.pos`, `hybrid_nagoya1_multistage_m4_fixedpos_bridge05_vertical025_veld_vertical10_truthfree.pos` | 9.4 MB |
| Tightly-coupled candidates | `tc_m3_full_t1_on/rtk.pos` (Tokyo 1), `probe_fuse_nagoya2_full_tc_m4.pos` (Nagoya 2) | 2.7 MB |
| FGO shadow windows | Nagoya 3 `fgo_partial_noreset_ddpranchor_*` (2), Tokyo 3 `fgo_shipping_tokyo3_start*` (2), Tokyo 1 `fgo_shipping_{,nhc_}tokyo1_*` (5), Tokyo 2 `fgo_shipping_{,nhc_}tokyo2_*` (2) | 13.3 MB |
| gici-open `e7666110` trajectories | `gici_common/{tokyo,nagoya}{1,2,3}.pos` | 5.7 MB |
| Nagoya 1 FIX-target profile | `goal_kf_current_r2_min8_rate20_rescue29_8/solution.pos` | 1.2 MB |

`--ppc-goal-inputs` takes a directory in the historical `output/` layout, so
the `output/` directory of the checkout that produced the README can be passed
as is. `stage_ppc_goal_inputs.py --inputs-root <output> --out <dir>` copies
the verified set into a standalone directory for archiving. The files are not
committed and not published. They are
larger than the repository's figure assets, and the gici-open trajectories are
the output of a GPL-3.0 program.

The lane then replays, in order, the Tokyo 1 tier-3 selection, the Nagoya 2
wrong-basin escape, the Nagoya 3 causal consensus, the kinematic status
demotion, the Tokyo 3 two-window FGO consensus, the Tokyo 1 / Tokyo 2
multi-shadow position consensus, and the staged residual policy. It then scores
both matrices, rescores the Nagoya 1 profile with `gnss ppc-demo
--use-existing-solution`, and draws the figures, the wrong-FIX ledger, and the
goal contract. Every step is a Python post-processing script, so no solver runs.

## ppc-goal local result (2026-09-29)

First local run: develop `2f919306` plus this lane, Windows 11, inputs from
the original checkout's `output/` tree. The whole lane took 51 s.

- All 26 inputs matched their pinned SHA-256.
- Every replayed intermediate was byte-identical to the historical file. This
  covers the tier-3 Tokyo 1, the Nagoya 2 escape, the Nagoya 3 consensus, the
  six kinematic-advanced POS files, the Tokyo 3 consensus, the Tokyo 1 / 2
  position consensus, and the three final Nagoya POS files that survived
  locally. The final Tokyo POS files no longer exist locally.
- Both scored matrices match `output/kf_fgo_staged_integrity_full_matrix.json`
  and `output/gici_reproduction_ppc_matrix.json` field for field.
- All 78 gated metrics pass. They cover 78.845491%, every libgnss++ and
  gici-open cell of the README table, the macro row, +16.613 / 1.025 pp,
  Nagoya 1 85.100974% / 0.913% / 1.460 m, 574 wrong FIX (42 > 5 m, 5 > 10 m,
  188 events), and the goal contract.

`--update-docs` regenerated byte-identical copies of the three PNGs.
`docs/ppc_kf_fgo_goal_metrics.json` changed only in its provenance paths.
Those were Windows paths such as `output\kf_fgo_...json`, with gici-open run
paths left empty. They are now portable paths under `output/reproduce/ppc-goal/`,
and the metric values are unchanged. The README was not modified.

The Tokyo 1 / Tokyo 2 multi-shadow position consensus argv was not recorded.
It was reconstructed from the thresholds, primary POS, and shadow list stored
in the historical summary JSONs, and the replay matched byte for byte. The
lane does not regenerate any solver output below the frozen layer. See
"Tier provenance" in [PPC reproduction](ppc_reproduction.md#tier-provenance)
for what is and is not recorded there.

## gsdc-official: rebuilding the Kaggle submission

The README "Smartphone GNSS/IMU" official row is Kaggle submission 56625084
(Private 0.984 m / Public 0.915 m, 40 test drives, 71,936 rows; readback
record
[gsdc2023_heading_modern_official_readback_20260928.json](use_cases/records/gsdc2023_heading_modern_official_readback_20260928.json)).
Kaggle scores it against hidden ground truth, so no local run can recompute
the score. The lane therefore gates on the file itself: the rebuilt
`submission.csv` must have the SHA-256 of the submitted file,
`cbd1fde10f0f317f7803871b7d00b64a73398498f967b86f2130d8ba8acc26f8`. The lane
never submits to Kaggle.

**Recipe.** `configs/reproduce/gsdc_official_recipe.json` unrolls the chain
that the research assembler
(`scripts/analysis/assemble_gsdc_heading_modern_test.py`) consumed. It pins,
per drive, the input files relative to `--gsdc-root` with their SHA-256, the
exact `gnss_fgo_imu_no_base` argv (placeholders `{bin}`, `{gsdc_root}`,
`{work_dir}`), and the SHA-256 of the solution the research run wrote:

| Stage | Drives | What it is | Research runtime |
|---|---:|---|---:|
| `stage0` | 25 | Offset + extra-band replays (`test40_offset_extra_bands_v1`). Their trajectories select the height-map points | ~9.1 h |
| height maps | 25 | Kaggle train `ground_truth.csv` points (156 files) within 30 m of the stage-0 trajectory, written as `lat_deg,lon_deg,height_m` | ~1 min |
| `final` | 17 | Pixel5 with epoch heading seeds (new in 56625084) | ~3.8 h |
| `final` | 9 | Modern phones with the continuous raw-clock recipe (new in 56625084) | ~1.9 h |
| `final` | 14 | Height-map or relative-height replays retained from submission 56536540 | ~2.8 h |
| assemble | 40 | Official key list (Kaggle `sample_submission.csv` order, stored run-length encoded in the recipe); each coordinate is copied verbatim from the native row with the same `UnixTimeMillis`, and extra native rows are dropped | seconds |

The research runtimes were measured with two replays in parallel. The
research used four frozen MSVC builds (named with their SHA-256 in the
recipe). The lane instead runs one build of the current tree for every
drive.

**Steps.** `scripts/experiments/gsdc/gsdc_official_reproduce.py` runs
`verify-inputs` (SHA-256 of 160 raw inputs and 156 truth files),
`run --stage stage0`, `height-maps`, `run --stage final` (one solver process
at a time) and `assemble`. Both `run` steps resume: a drive whose `run.json`
records exit 0 and whose `solution.csv` still has the recorded SHA-256 is
skipped. Stage 0 is also skipped for a drive whose height map is already in
`<work-dir>/height_maps/` with the pinned SHA-256. The height-map step needs
numpy, pandas and scipy. Use `--drives <tripId> ...` on the script for a
subset. `assemble --reference-submission <csv>` (or
`$GSDC_OFFICIAL_REFERENCE_SUBMISSION`) adds per-drive row differences
against the submitted file, and `--allow-partial` fills drives that have not
run yet from that file, so a subset can be checked end to end.

**Gates.** The `submission.csv` SHA-256 match, 71,936 rows, and 40/40 drives
byte-identical to the research solutions. The per-group identity counts and
the final wall time are reported without gating.

### gsdc-official local result (2026-09-29)

Develop `2f919306` (native sources unchanged since `4b22fe43`) was built with
MSVC Release and GTSAM 4.3 on Windows 11, and the solver ran one process at a
time.

| Check | Result |
|---|---|
| `verify-inputs` | 160 raw inputs and 156 truth files match their SHA-256 (12 s) |
| Recipe against the research outputs | With the 40 research `solution.csv` files and the 25 research stage-0 trajectories placed in the work-dir layout, `height-maps` rebuilds 25/25 maps byte-identically and `assemble` writes `cbd1fde1...` (71,936 rows), which **matches the submitted file** |
| Stage 0, current build | `sjc-r/sm-a205u` 238 s and `mtv-ie2/pixel6pro` 311 s: both byte-identical to the research output of `source_bias_difference_sigma_admission_fixed.exe`. Their rebuilt height maps match the pinned SHA-256 |
| Final, current build | `mtv-pe1/samsunga325g` (retained, map) 260 s, `sjc-he2/pixel5` (Pixel5 heading) 711 s, `mtv-e/sm-g988b` (modern) 375 s, `mtv-ie2/pixel6pro` (modern) 607 s, `sjc-r/sm-a205u` (retained, map) 287 s, `lax-hh/samsunga325g` (retained, relative height) 254 s: 6/6 byte-identical to the research outputs of three frozen binaries |
| Partial assembly | Three drives rebuilt through the lane driver plus 37 drives filled from the submitted file (`--allow-partial`) give `cbd1fde1...` |

The current tree therefore reproduces all four frozen research binaries
byte for byte on every drive tried, and this subset showed no MSVC-vs-GCC
drift of the kind seen in `gsdc-dev-routes`, because every research binary
was also an MSVC build. The research runtimes sum to about 9.1 h (stage 0)
plus 8.4 h (final), measured two replays at a time. The replays here ran at
0.9-2.4x their research wall time. A full single-process run is therefore
estimated at 12-18 h, or about 8-10 h when the height maps are already
pinned in the work dir. Verification is intentionally limited to this
subset (per-drive byte identity plus `--allow-partial` assembly); the full
40-drive run is available through the same command but is not part of the
routine check.
