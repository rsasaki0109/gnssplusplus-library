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
| RTK: PPC Tokyo/Nagoya vs RTKLIB `demo5` | `rtk-demo5` | ready | ~13 min | **Pass.** README refreshed 2026-09-28 and again 2026-10-10 (Linux gcc 13.3); and 2026-10-11 (Doppler/phase slip-test interval fix: +56.9 pp fix rate, weighted official score 68.108% to 68.120%); see [README refresh](#readme-refresh-2026-09-28), [2026-10-10 refresh](#readme-refresh-2026-10-10) and [2026-10-11 refresh](#readme-refresh-2026-10-11) |
| CLAS PPP: six PPC runs vs MRTKLIB CLAS | `clas-ppc` | ready | ~45 min | **Pass.** README refreshed 2026-10-29: reproduces 27.232% FIX, 0.361 m FIX RMS2D, 0 FIX > 3 m, 58,259 epochs, all hard gates (L6/SSR expansion ~14 min + six `gnss_ppp` runs ~30 min) |
| Urban RTK: UrbanNav Odaiba vs RTKLIB `demo5` | `odaiba` | ready | ~7 min | **Pass.** 2026-10-26: demo5 runs multi-GNSS (`pos1-navsys = 61`) and the demo5 claim moved to `--preset urban` (more fixes, no more wrong fixes, lower Hmed/Hp95/Vp95); see [2026-10-26 refresh](#readme-refresh-2026-10-26). Before that: README refreshed 2026-09-29 after both solvers switched to the surveyed base position (removing a 0.70 m west bias in every fixed epoch); see [odaiba base position](#odaiba-base-position-2026-09-29). Re-run 2026-10-10 on the original 10 Hz rover: all gates pass; the `--preset odaiba` claim is restated (more fixes, lower Hmed; all-epoch Hp95/Vp95 reported, not gated); re-run 2026-10-11 after the Doppler/phase slip-test interval fix: default arm bit-identical, preset 7961 fixes, Hp95 5.28 m, Vp95 13.89 m; see [2026-10-10 refresh](#readme-refresh-2026-10-10) and [2026-10-11 refresh](#readme-refresh-2026-10-11) |
| SPP: PPC adaptive robust + policy gate | `spp-policy` | ready | ~4 min | **Pass.** No P95 regression on 4/4 runs; drop <= 0.98 pp |
| GNSS/IMU FGO: PPC Tokyo vs `tightly-coupled-gnss-imu-fgo` | `fgo-tokyo` | ready | ~35 min | **Pass.** Comparison table and GF-reset column reproduce exactly; the GF-reset baseline Tokyo run3 row does not (reported, not gated); see [fgo-tokyo result](#fgo-tokyo-local-result-2026-09-28) |
| PPC 2024 goal matrix vs Kaiyodai and gici-open | `ppc-goal` | ready (score-only) | ~1 min | **Pass.** Replays the truth-free post-processing chain from 26 SHA-256-pinned tier inputs and reproduces every README number exactly (78.845491%, the six-run libgnss++/gici-open table, Nagoya 1 85.100974%); the solver outputs at the bottom of the chain are frozen, not regenerated; see [ppc-goal result](#ppc-goal-local-result-2026-09-29) |
| Smartphone dev routes (base-surveyed) | `gsdc-dev-routes` | ready | ~25 min | **Pass.** README refreshed 2026-09-29 to the reproduced H 0.576 / U 0.740 / A 0.303 / LAX-T 0.716 m (previously 0.577 / 0.738 / 0.302 / 0.712); see [gsdc-dev-routes result](#gsdc-dev-routes-local-result-2026-09-29) |
| Smartphone GSDC official submission | `gsdc-official` | ready | Linux GCC: full run ~3 h with `--jobs 4` on 4 cores (the third pass doubles the long drives) | **Pass on one build.** Gate: the rebuilt `submission.csv` is byte-identical to Kaggle ref 57742915 (`12d39d57...`), and the Kaggle score is readback-only. Every drive is pinned to one Linux GCC 15 build; MSVC outputs differ by up to 0.08 m and do not pass. On 2026-10-25 the full lane (25 stage-0 runs, 25 height maps, 40 final drives) ran on that build; the 9 final drives and 8 stage-0 runs already pinned to GCC came out byte-identical, and the assembled file was submitted as 57619467. 57626255 / 57629197 then solved ebf-x Mi8 from the taroz IdxStart epoch, 57664241 / 57678818 added the upstream third pass, 57717122 tuned 18 drives on Kaggle probes, 57733487 picked 22 drives by a rule fixed on train and snapped toward a train-truth road map, and 57742915 moves that road map into the factor graph; see [gsdc-official](#gsdc-official-rebuilding-the-kaggle-submission) |
| (docs, not a README row) Galileo HAS float PPP via IDD | `has-idd-ppp` | ready | ~1 min | **Pass.** Static and kinematic OBE4 hour; see [has-idd-ppp](#has-idd-ppp-galileo-has-float-ppp-2026-09-29) and [Galileo HAS support](galileo_has.md) |
| (docs, not a README row) Galileo HAS float PPP via SIS (E6-B pages) | `has-sis-ppp` | ready | ~1 min | **Pass.** Decoder (432 MT1 messages per hour, no CRC failure) and static Kamakura 2025-02-15 hour; indicative only (outside the HAS service area); see [has-sis-ppp](#has-sis-ppp-galileo-has-sis-float-ppp-2026-09-30) and [Galileo HAS support](galileo_has.md) |
| (docs, not a README row) Precise-product PPP with IGS final SP3/CLK | `igs-final-ppp` | ready | ~2 min | **Pass.** Static Kamakura 2025-02-15 / 2025-08-21 hours and OBE4 2023-08-17 hour within 0.32 m horizontal / 0.27 m vertical after one hour, level with RTKLIB demo5 on the same inputs; see [igs-final-ppp](#igs-final-ppp-precise-product-ppp-2026-10-01) |

Runtimes were measured on a 12-thread Windows 11 workstation with an MSVC
Release build. Lanes that run in parallel slow each other down.

## Datasets

| Dataset | Used by | Get it | Expected layout |
|---|---|---|---|
| [PPC-Dataset](https://github.com/taroz/PPC-Dataset) | `rtk-demo5`, `clas-ppc`, `spp-policy`, `fgo-tokyo`, `ppc-goal` | `git clone https://github.com/taroz/PPC-Dataset` | `<ppc-root>/{tokyo,nagoya}/run{1,2,3}/{rover.obs,base.obs,base.nav,reference.csv}`; `fgo-tokyo` also reads `tokyo/run{1,2,3}/imu.csv` |
| [UrbanNav Tokyo Odaiba](https://github.com/IPNL-POLYU/UrbanNavDataset) | `odaiba` | UrbanNav Tokyo data release (Trimble rover/base RINEX + Applanix reference) | `<urbannav-root>/Odaiba/{rover_trimble.obs,base_trimble.obs,base.nav,reference.csv}`; the base coordinate is the providers' surveyed position, not the RINEX header (see [odaiba base position](#odaiba-base-position-2026-09-29)) |
| [GSDC 2023 `dataset_2023`](https://github.com/taroz/gsdc2023) (Kaggle Google Smartphone Decimeter Challenge 2023 train and test sets with CORS base RINEX and `brdc.nav`) | `gsdc-dev-routes`, `gsdc-official` | Kaggle GSDC 2023 data as repackaged by taroz/gsdc2023 (`dataset_2023.zip`, SHA-256 `bda30ab4...`) | `<gsdc-root>/train/<drive>/{brdc.nav,<BASE>_rnx2.obs,pixel5/{device_gnss.csv,device_imu.csv,ground_truth.csv}}`, or the zip itself; see [gsdc-dev-routes inputs](#gsdc-dev-routes-inputs-and-base-provenance). `gsdc-official` reads `<gsdc-root>/test/<drive>/{brdc.nav,<BASE>_rnx2.obs,<phone>/{device_gnss.csv,device_imu.csv}}` |
| Kaggle GSDC 2023 train `ground_truth.csv` | `gsdc-official` (height maps) | Kaggle `smartphone-decimeter-2023` competition data (156 train files, SHA-256 pinned in the recipe) | `--gsdc-truth-root` holding `train/<course>/<phone>/ground_truth.csv` or flat `<course>__<phone>__ground_truth.csv` |
| PPC goal-matrix frozen tier inputs | `ppc-goal` | Not published; the 26 files (~33 MB) exist only in the `output/` tree of the checkout that produced the README. See [ppc-goal inputs](#ppc-goal-frozen-inputs) | `<ppc-goal-inputs>/` in the historical `output/` layout (`tokyo1_selected_quality_rtkbaseline_tier2_truthfree.pos`, `gici_common/tokyo1.pos`, ...); SHA-256 pinned in `scripts/experiments/ppc/stage_ppc_goal_inputs.py` |
| Galileo HAS IDD sample ([hirokawa/cssrlib-data](https://github.com/hirokawa/cssrlib-data) `data/doy2023-229`) | `has-idd-ppp` | Download `OBE42023229c.obs`, `OBE42023229c.nav` and `idd2023229c.rtc` from that directory. The upstream license is not stated, so the files are not redistributed | `<has-data-root>/{OBE42023229c.obs,OBE42023229c.nav,idd2023229c.rtc}` |
| Galileo HAS SIS samples ([hirokawa/cssrlib-data](https://github.com/hirokawa/cssrlib-data) `data/doy2025-046`, `data/doy2025-233`) plus IGS BRDC navigation | `has-sis-ppp` | Download `046r_rnx.obs`, `046r_gale6.txt`, `233h_rnx.obs`, `233h_gale6.txt` from those directories and the IGS merged navigation files `BRDC00WRD_S_2025{046,233}0000_01D_MN.rnx.gz` from `https://igs.bkg.bund.de/root_ftp/IGS/BRDC/2025/{046,233}/` (gunzip next to them). The cssrlib-data license is not stated, so nothing is redistributed | `<has-sis-data-root>/doy2025-046/{046r_rnx.obs,046r_gale6.txt,BRDC00WRD_S_20250460000_01D_MN.rnx}`, same for `doy2025-233/233h_*` |
| IGS final orbits/clocks and `igs20.atx` (public, IGS) on top of the two cssrlib-data sets above | `igs-final-ppp` | `IGS0OPSFIN_{20250460000,20252330000,20232290000}_01D_{15M_ORB.SP3,30S_CLK.CLK}.gz` from `https://igs.bkg.bund.de/root_ftp/IGS/products/{2353,2380,2275}/` (gunzip next to the observations of that day) and `igs20.atx` from the immutable IGS archive copy `https://files.igs.org/pub/station/general/pcv_archive/igs20_2425.atx.gz` (gunzip; SHA-256 `8715268e...`) | `<has-sis-data-root>/igs20.atx`, `<has-sis-data-root>/doy2025-{046,233}/IGS0OPSFIN_2025{046,233}0000_01D_*` next to `{046r,233h}_rnx.obs`, and `<has-data-root>/IGS0OPSFIN_20232290000_01D_*` next to `OBE42023229c.obs` |
| QZSS L6 CLAS archive | `clas-ppc` | Downloaded automatically from `https://sys.qzss.go.jp/archives/l6` | Cached under `<work-dir>/inputs/l6_cache` (about 1.7 GB of expanded SSR CSV per run) |

Dataset roots are resolved in this order:

1. `--ppc-root` / `--urbannav-root` / `--gsdc-root` / `--gsdc-truth-root` / `--ppc-goal-inputs` / `--has-data-root` / `--has-sis-data-root`
2. `GNSSPP_PPC_DATASET_ROOT` / `GNSSPP_URBANNAV_ROOT` / `GNSSPP_GSDC_ROOT` / `GNSSPP_GSDC_TRUTH_ROOT` / `GNSSPP_PPC_GOAL_INPUTS` / `GNSSPP_HAS_DATA_ROOT` / `GNSSPP_HAS_SIS_DATA_ROOT`
3. `--data-root <dir>` expands to `<dir>/PPC-Dataset`, `<dir>/driving/Tokyo_Data`, `<dir>/gsdc2023/dataset_2023`, `<dir>/ppc_goal_inputs`, `<dir>/galileo_has/doy2023-229` and `<dir>/galileo_has/cssrlib-data`
4. `data/PPC-Dataset`, `data/driving/Tokyo_Data`, `data/gsdc2023/dataset_2023`, `data/ppc_goal_inputs`, `data/galileo_has/doy2023-229` and `data/galileo_has/cssrlib-data` inside the repository

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

# UrbanNav Odaiba RTK vs RTKLIB demo5, default, --preset urban and --preset odaiba
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

# Galileo HAS float PPP from the HAS IDD sample (docs lane, not a README row)
python3 apps/gnss.py reproduce has-idd-ppp --has-data-root /datasets/cssrlib-data/data/doy2023-229 --check

# Galileo HAS float PPP from decoded E6-B signal-in-space pages (docs lane)
python3 apps/gnss.py reproduce has-sis-ppp --has-sis-data-root /datasets/cssrlib-data/data --check

# Static float PPP with IGS final orbits/clocks on the same hours (docs lane)
python3 apps/gnss.py reproduce igs-final-ppp --has-sis-data-root /datasets/cssrlib-data/data \
  --has-data-root /datasets/cssrlib-data/data/doy2023-229 --check
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
`{ppc_root}`, `{urbannav_root}`, `{gsdc_root}`, `{gsdc_truth_root}`, `{has_root}`, `{rtklib_bin}`, `{build_dir}`, and
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

Hmed is reported only. All solvers' fixed epochs sat about 0.70 m from
`reference.csv`, so Hmed was floored by that common offset and dropped as FLOAT
epochs were added. The cause was the base coordinate, fixed on 2026-09-29; see
[odaiba base position](#odaiba-base-position-2026-09-29), which supersedes the
Odaiba numbers in this section.

Before 2026-09-29 the preset produced 54 fixes. It was a stale copy of the
early `low-cost` values, and it fixed wide-lane integers from a single noisy
MW epoch. See the [Odaiba snapshot](benchmarks.md#urbannav-tokyo-odaiba-snapshot).
The previous snapshot table (demo5 595 fixes, default 1268, preset 735) was
produced with an unrecorded RTKLIB build.

**`spp-policy`.** The README claim reproduces. The historical per-run policy
P95 H values in `docs/references/spp-accuracy-improvement.md` are within
0.07 m and are reported without gating.

## odaiba base position (2026-09-29)

`gnss odaiba-benchmark` now passes the surveyed TUMSAT base position
(-3961904.9530, 3348993.7578, 3698211.7553) to libgnss++ (`--base-ecef`) and
to RTKLIB (`-r`). The data providers publish this position for the same
`base_trimble.obs` station
([MeijoMeguroLab/Open_data](https://github.com/MeijoMeguroLab/Open_data/blob/main/docs/2019_dataset.md)).
The previous runs used the `base_trimble.obs` header APPROX POSITION, which is
0.723 m west of it. That offset was the whole 0.70 m west error of every fixed
epoch. The error was constant in ENU across all headings and speeds, so a
lever arm and a time-tag offset were ruled out. The evidence is in the
[Odaiba snapshot](benchmarks.md#base-station-position).
`--base-position rinex-header` reproduces the old numbers. The solver defaults
are unchanged.

Local run (2026-09-29): MSVC Release build of develop, demo5 b34k.

| Claim | demo5 b34k | libgnss++ default | Result |
|---|---:|---:|---|
| More fixes | 205 | **922** | pass |
| Lower Hp95 | 25.62 m | **5.87 m** | pass |
| Lower Vp95 | 43.37 m | **16.42 m** | pass |
| Lower Hmed on common epochs (7,987) | 0.548 m | **0.294 m** | pass |

| Claim | libgnss++ default | `--preset odaiba` | Result |
|---|---:|---:|---|
| More fixes | 922 | **6115** | pass |
| Lower Hp95 | 5.87 m | **5.13 m** | pass |
| Lower Vp95 | 16.42 m | **9.74 m** | pass |
| Hmed (reported, not gated) | 0.347 m | **0.068 m** | - |

Against the header-base run, Hmed falls from 0.684 to 0.567 m for demo5, from
0.696 to 0.347 m for the default profile and from 0.716 to 0.068 m for the
preset. The default arm's Hp95 rises from 5.10 to 5.87 m and its Vp95 from 15.10
to 16.42 m. The p95 tail is FLOAT and SPP epochs whose median east error is
about +7 m, and the old 0.7 m west bias partly cancelled it. Shifting the old
solution 0.723 m east gives an Hp95 of 5.72 m. The remaining difference comes
from the two runs publishing different epochs (11,027 versus 10,956).

## README refresh (2026-10-10)

The `rtk-demo5`, `clas-ppc` and `odaiba` lanes were rerun on develop `31d7177f`
(Linux, gcc 13.3 Release, no GTSAM, demo5 b34k from the pinned commit, PPC
dataset from `taroz/PPC-Dataset`, the original UrbanNav `Tokyo_Data.zip`
Odaiba files: `rover_trimble.obs` 71,151,487 B `3ebb5c4f...`,
`base_trimble.obs` `5cc6c8d2...`, `base.nav` `c58bd7ec...`, `reference.csv`
`fbf3f005...`). The README rows, lane expectations, generated tables and
figures now show these values. The RTK numbers are platform-sensitive (the
2026-09-28 values came from an MSVC build); the lane tolerances are unchanged.

**`rtk-demo5`** (six-run averages; demo5 fix rates are unchanged to 0.01 pp):

| Metric | 2026-09-28 (MSVC) | 2026-10-10 (gcc 13.3) |
|---|---:|---:|
| Avg Fix-rate delta vs demo5 | +56.8 pp | **+57.0 pp** |
| Avg PPC official-score delta | +45.4 pp | **+44.9 pp** |
| Avg P95 H delta | -11.44 m | **-11.33 m** |
| Avg Positioning delta | -9.5 pp | **-9.7 pp** |

The per-run table in `docs/benchmarks.md` is regenerated. The largest moves are
Nagoya run2 official score 32.4% to 30.0% (delta +16.9 to +14.5 pp) and Nagoya
run3 P95 H delta +4.68 to +5.60 m. The same values were obtained at the
2026-09-28 README commit on this platform, so the difference is platform and
compiler, not recent solver changes.

**`clas-ppc`:**

| Metric | Previous | 2026-10-10 |
|---|---:|---:|
| Aggregate FIX | 25.121% (14,635 epochs) | **25.117%** (14,633 epochs) |
| Aggregate FIX RMS2D | 0.359365 m | **0.359287 m** (0.359 m) |
| All / FLOAT / SINGLE RMS2D | 36.522 / 16.885 / 70.361 m | **36.631 / 16.964 / 70.596 m** |
| Nagoya 2 FIX | 23.969% (2,250 epochs) | **23.948%** (2,248 epochs) |

Only Nagoya 2 changes its FIX count (-2 epochs); Tokyo 1-3 solutions are
unchanged and Nagoya 1-3 differ only in FLOAT/SINGLE epochs. The first merge
that changes it is #582 (order-independent tracking-code choice). Zero FIX
epochs exceed 3 m and all six MRTKLIB v0.4.2 hard gates still pass.

**`odaiba`** (original 10 Hz Trimble rover; demo5 b34k reproduces
exactly, 205 fixes, Hp95 25.622 m, Vp95 43.375 m):

| Claim | demo5 b34k | libgnss++ default | Result |
|---|---:|---:|---|
| More fixes | 205 | **1298** | pass |
| Lower Hp95 | 25.62 m | **4.77 m** | pass |
| Lower Vp95 | 43.37 m | **11.54 m** | pass |
| Lower Hmed on common epochs (8,027) | 0.553 m | **0.298 m** | pass |

| Claim | libgnss++ default | `--preset odaiba` | Result |
|---|---:|---:|---|
| More fixes | 1298 | **8002** | pass |
| Lower Hmed | 0.359 m | **0.043 m** | pass |
| Hp95, all epochs (reported, not gated) | **4.77 m** | 5.29 m | - |
| Vp95, all epochs (reported, not gated) | **11.54 m** | 15.31 m | - |

The 2026-09-29 values (default 922 fixes, Hp95 5.87 m, Vp95 16.42 m, Hmed
0.294 m on 7,987 common epochs; preset 6115 fixes, Hp95 5.13 m, Vp95 9.74 m)
predate the BeiDou B1I mapping (#577) and later develop changes. The default
arm gained fixes and improved Hp95/Vp95 (Hmed moved by +0.012 m), while the
preset gained fixes but its all-epoch Hp95/Vp95 rose above the default arm.
A bisection on the 10 Hz data showed this is not a solver regression: the old
preset Vp95 held only because two FLOAT post-filters blanked two stationary
deep-canyon windows, and #577/#588 changed the fix pattern so they no longer
fire (with the filters off the pre-#577 preset Vp95 was already 14.74 m; on
the epochs both arms publish the preset Vp95 is lower, 7.38 vs 9.41 m). The
preset claim is therefore restated as more fixes and lower Hmed, gated, with
its all-epoch Hp95/Vp95 reported but not gated; see
[benchmarks](benchmarks.md#urbannav-tokyo-odaiba-snapshot).

## README refresh (2026-10-11)

The `rtk-demo5` and `odaiba` lanes were rerun on develop `ced3d057` plus the
Doppler/phase slip-test interval fix (`updateBias` used the time since the last
remembered solution while the Doppler phase history is refreshed on every call,
so after a rejected epoch the receiver-clock-drift term of the single-difference
range rate produced false slips on every satellite; it now uses the Doppler
history interval; regression test `tests/test_rtk_doppler_slip_dt.cpp`). Same
host, toolchain, demo5 b34k pin and Odaiba input checksums as the 2026-10-10
refresh; the original `Tokyo_Data.zip` Odaiba members were re-fetched and their
SHA-256 values match. `clas-ppc` was not rerun: `updateBias` lives in the RTK
filter (`src/algorithms/rtk_bias.cpp`), which the CLAS PPP path (`gnss_ppp`) does
not call, so its numbers are unaffected by construction.

**`rtk-demo5`** (demo5 fix rates unchanged to 0.01 pp):

| Metric | 2026-10-10 | 2026-10-11 |
|---|---:|---:|
| Avg Fix-rate delta vs demo5 | +57.0 pp | **+56.9 pp** |
| Avg PPC official-score delta | +44.9 pp | **+44.9 pp** (44.914 to 44.920) |
| Avg P95 H delta | -11.33 m | **-11.33 m** |
| Avg Positioning delta | -9.7 pp | **-9.7 pp** (-9.653 to -9.667) |
| Weighted PPC official score | 68.108% | **68.120%** |

| Run | Fix rate | PPC official score |
|---|---:|---:|
| Tokyo run1 | 83.95% to 83.85% | 69.62% (unchanged) |
| Tokyo run2 | 84.27% to 84.48% | 83.22% to 83.32% |
| Tokyo run3 | 80.56% to 79.89% | 79.01% (unchanged) |
| Nagoya run1 | 79.39% (unchanged) | 52.84% (unchanged) |
| Nagoya run2 | 63.59% (unchanged) | 30.02% (unchanged) |
| Nagoya run3 | 63.85% to 63.84% | 52.19% to 52.12% |

Positioning rates move by at most 0.05 pp (Tokyo run3 93.75% to 93.69%) and the
per-run P95 H by at most 0.006 m; Tokyo run1 max horizontal error falls from
29.7 to 25.8 m. The README row (+57.0 to +56.9 pp fix rate), the lane
expectations, and the generated coverage block and scorecard in
`docs/benchmarks.md` / `docs/ppc_rtk_demo5_scorecard.png` follow; tolerances are
unchanged.

**`odaiba`** (default arm bit-identical, 1298 fixes, Hp95 4.773 m, Vp95 11.537 m,
Hmed 0.298 vs 0.553 m on 8,027 common epochs; demo5 unchanged):

| `--preset odaiba` | 2026-10-10 | 2026-10-11 |
|---|---:|---:|
| Fix epochs | 8002 | **7961** |
| Fix rate | 78.92% | **79.00%** |
| Hmed | 0.043 m | **0.043 m** |
| Hp95, all epochs | 5.293 m | **5.284 m** |
| Vp95, all epochs | 15.307 m | **13.892 m** |
| Epochs published | 10,140 | **10,077** |

The gates (more fixes than the default, lower Hmed) still pass. The preset's
all-epoch Hp95/Vp95 remain above the default arm's (+0.51 m and +2.35 m,
previously +0.52 m and +3.77 m). Two statements added on 2026-10-10 (#593) are
withdrawn. "The preset Vp95 is lower on the epochs both arms publish" (7.38 vs
9.41 m on 9,615 epochs) came from a different epoch matching and is not
comparable; on the 9,906 common epochs measured now it is 11.81 m against
10.52 m for the default (Hp95 5.11 vs 4.57 m, Hmed 0.042 vs 0.318 m). "About
830 more FLOAT epochs" was wrong: the preset publishes fewer epochs and far
fewer FLOAT epochs than the default (1,935 against 9,577). 604 of the preset's 629 epochs with more than 10 m
vertical error lie in the two stationary deep-canyon windows. See
[benchmarks](benchmarks.md#urbannav-tokyo-odaiba-snapshot).

## README refresh (2026-10-26)

**`odaiba`.** Two changes on this date:

- `scripts/rtklib_odaiba.conf` now runs demo5 with every constellation
  (`pos1-navsys = 61`), as libgnss++ does. Until then demo5 ran GPS-only and
  had 205 fixes; it now has 5839.
- The demo5 claim moved from `--preset odaiba` to `--preset urban`, the CDME
  fix-recovery recipe of
  [rtk_doppler_consensus_age.md](rtk_doppler_consensus_age.md).

The lane's summaries now count wrong fixes: FIXED epochs whose 3D error from
`reference.csv` exceeds 0.5, 2 and 3 m. The lane gates `--preset urban`
against demo5 on all of the claims below. It still gates `--preset odaiba`
against the default arm only.

| Claim | demo5 b34k | `--preset urban` | Result |
|---|---:|---:|---|
| More fixes | 5839 | **7467** | pass |
| No more wrong fixes >0.5 / 2 / 3 m | 19 / 4 / 4 | **9 / 0 / 0** | pass |
| Lower Hmed | 0.190 m | **0.046 m** | pass |
| Lower Hp95 | 7.856 m | **4.860 m** | pass |
| Lower Vp95 | 15.722 m | **10.770 m** | pass |

The default profile keeps a lower Hp95/Vp95 than demo5 (4.777/11.537 m),
with 1298 fixes and no wrong fix. `--preset odaiba` has 7961 fixes but 109 /
5 / 5 wrong fixes, so it is no longer claimed against demo5.

These numbers were measured with `gnss odaiba-benchmark` on develop plus the
preset, reusing the 2026-10-26 demo5 POS. The `--preset urban` POS is
byte-identical to the CDME POS of the evaluation, and the default POS is
unchanged.

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

For fresh solver regeneration from raw PPC data, use
[PPC native replay](ppc_native_replay.md). It builds RTK and RTK/IMU, scores each
stream separately and records complete source/input/binary provenance. Its
results are a new native baseline, separate from the historical selected tiers
below.

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

The README "Smartphone GNSS/IMU" official row is Kaggle submission 57742915
(Private 0.714 m / Public 0.732 m, 40 test drives, 71,936 rows; readback
record
[gsdc2023_official_readback_20261024.json](use_cases/records/gsdc2023_official_readback_20261024.json)).
It is submission 56625084 (Private 0.984 m / Public 0.915 m; record
[gsdc2023_heading_modern_official_readback_20260928.json](use_cases/records/gsdc2023_heading_modern_official_readback_20260928.json))
with ten drives changed by per-drive flags, rebuilt on one Linux GCC
build. The flags are listed under `research_source.added_flags` /
`removed_flags` in the recipe. One of them, ebf-x Mi8, is also solved from a
later epoch (`input_window`; see the 2026-10-25 ebf-x subsection below).
Thirty drives then take the upstream third pass (see the 2026-10-26
subsections below). 57717122 was tuned on the test set: 18 drives used the
setting that scored best in single-drive Kaggle probes. 57733487 instead
picked 22 drives by a rule fixed on train and then moved epochs across track
toward a road map built from train ground truth. 57742915 keeps those picks
and moves the road map into the factor graph as a cross-track prior on 32
drives (the 2026-10-28 subsections below).
Kaggle scores it against hidden ground truth, so no local run can recompute
the score. The lane therefore gates on the file itself: the rebuilt
`submission.csv` must have the SHA-256 of the submitted file,
`12d39d57a8ff3d4deb520e87355873a194ae86e635a70afb9f54e5f40d9ba12f`. The lane
never submits to Kaggle.

**One platform.** Every final replay, stage-0 run and height map is pinned
to one Linux GCC 15 Release build (GTSAM 4.3.0, conda-forge;
`gnss_fgo_imu_no_base` SHA-256 `a29694ad...`, develop `ed5f8d0`; develop
`86d1e81` links the same binary). Submissions up to 57600830 kept the 31
drives retained from 56625084 at their MSVC research outputs. The recipe
keeps those outputs and the frozen MSVC binaries per drive under
`research_source.msvc_research`. GCC and MSVC outputs of the same drive
differ by up to 0.08 m and are not byte identical, so an MSVC build does not
pass the SHA-256 gate. The 9 drives changed since 56625084 are:

- the 6 per-phone main Doppler drives (sjc-r A505U also with
  `--native-course-attitude-seeds-mount-deg 178.9`);
- sjc-he2 Pixel5 with `--android-hardware-clock-reset-clock-jumps`;
- sjc-ce1 Pixel7Pro with `--native-source-imu-initialization`;
- ebf-z Pixel5 without `--native-epoch-heading-attitude-seeds`.

**Recipe.** `configs/reproduce/gsdc_official_recipe.json` unrolls the chain
that the research assembler
(`scripts/analysis/assemble_gsdc_heading_modern_test.py`) consumed. It pins,
per drive, the input files relative to `--gsdc-root` with their SHA-256, the
exact `gnss_fgo_imu_no_base` argv (placeholders `{bin}`, `{gsdc_root}`,
`{work_dir}`), and the SHA-256 of the solution the research run wrote:

| Stage | Drives | What it is | GCC runtime |
|---|---:|---|---:|
| `stage0` | 25 | Offset + extra-band replays (`test40_offset_extra_bands_v1`). Their trajectories select the height-map points | ~2.6 h |
| height maps | 25 | Kaggle train `ground_truth.csv` points (156 files) within 30 m of the stage-0 trajectory, written as `lat_deg,lon_deg,height_m` | ~1 min |
| `final` | 17 | Pixel5 with epoch heading seeds (new in 56625084) | ~1.9 h |
| `final` | 9 | Modern phones with the continuous raw-clock recipe (new in 56625084) | ~0.9 h |
| `final` | 14 | Height-map or relative-height replays retained from submission 56536540 | ~1.1 h |
| assemble | 40 | Official key list (Kaggle `sample_submission.csv` order, stored run-length encoded in the recipe); each coordinate is copied verbatim from the native row with the same `UnixTimeMillis`, and extra native rows are dropped | seconds |

The runtimes are summed per-drive walls of the 2026-10-25 GCC rebuild,
which ran four replays at a time on a 4-core Linux container (about 1.5 h
end to end). The research runs behind 56625084 used four frozen MSVC builds
(named with their SHA-256 in the recipe). The lane runs one build of the
current tree for every drive.

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
run yet from that file, so a subset can be checked end to end. For a drive
with an `input_window`, `run` first writes its Android GNSS rows from the
window epoch and its IMU rows from `imu_lead_ms` earlier into
`<work-dir>/inputs/` (raw lines, SHA-256 pinned). `assemble` then fills the
official keys before its first native row with that row.

**Gates.** The `submission.csv` SHA-256 match, 71,936 rows, and 40/40 drives
byte-identical to the research solutions. The per-group identity counts and
the final wall time are reported without gating.

### gsdc-official recipe update (2026-10-24)

The recipe moved from 56625084 to 57590194. The 8 replaced drives were
rerun with a Linux GCC 15 Release build of develop `5c0481e` (GTSAM 4.3.0,
conda-forge). All 8 `solution.csv` files are byte-identical to the
submitted solutions. The 7 GCC stage-0 trajectories and height maps behind
them come from the same lane on develop `a40998e`. The 156 truth files
matched their SHA-256. The rebuilt maps differ from the MSVC maps only by
boundary points. Running `height-maps` on those 7 GCC stage-0 trajectories
rebuilds all 7 maps to their newly pinned SHA-256. The submitted
`submission.csv` is 57586667 with the sjc-ce1 rows copied verbatim from its
GCC solution. `assemble --allow-partial` runs on the 8 GCC solutions, with
the other 32 drives filled from the submitted file. It writes `f9526888...`
(71,936 rows) and reports 8/8 drives identical.

The recipe then moved to 57594243. That submission is 57590194 with the
ebf-z Pixel5 rows replaced by a run without epoch heading seeds. With the
seeds, the optimized body yaw slips one full turn against the GNSS course
between minutes 3 and 13. The ebf-z stage 0, height map and final run come
from the lane on the same build. `height-maps` rebuilds its map to the
pinned SHA-256. `assemble --allow-partial` on the 9 GCC solutions writes
`2344a34f...` and reports 9/9 drives identical.

The recipe then moved to 57600830. That submission is 57594243 with the
sjc-r A505U rows replaced by a run with
`--native-course-attitude-seeds-mount-deg 178.9`. With the default seeds,
the IMU-stage yaw slips one full turn between epochs 800 and 1400. The
new run is byte-identical to the experimental build that produced the
submission, and without the flag the build reproduces the previous A505U
output. `assemble --allow-partial` writes `a7242b56...` and reports 9/9
drives identical.

### gsdc-official one-build rebuild (2026-10-25)

The recipe then moved to 57619467. In 57600830, 31 drives were still at
their MSVC research outputs, so no single build could pass the gate. The
lane reran, with the same argv and the GCC build above, the 17 stage-0 runs
pinned to MSVC, rebuilt their height maps, and reran all 40 final drives.
Four replays ran at a time, about 1.5 h on 4 cores. The 9 final drives
already pinned to GCC came out byte-identical. So did the 8 stage-0 runs
pinned from develop `a40998e`, when rerun on this build.

The 31 rerun drives differ from their MSVC outputs as follows:

- median 0.001 m and P95 0.008 m over all rows;
- at most 0.079 m, on mtv-m Mi8;
- at most 0.025 m on every other drive.

Kaggle scores the assembled file (`c46ca65f...`) Private 0.844 m / Public
0.885 m, against 0.845 m / 0.884 m for 57600830. With the updated recipe,
`height-maps` matches 25/25 maps, and `assemble` writes `c46ca65f...` with
40/40 drives identical.

### gsdc-official ebf-x input window (2026-10-25)

The recipe then moved to 57629197. Its ebf-x Mi8 rows replace those of
57619467. In 57619467, the first 11 static minutes of that
drive sit about 2 m from the taroz test output, and they are 38% of its
1,897 scored rows. A truth-free check found why. Epoch-to-epoch carrier
phase displacements from the raw ADR (robust least squares with a free
clock term, about 22 satellites and 1 cm residual RMS per epoch) disagree
with the native trajectory by 1.2 m over the 40 s parking-lot exit
(epochs 680-720). The disagreement is already in the GNSS-first stage.
The taroz trajectory agrees within 0.2 m. Among the 40 test drives, this is
the only one where the native trajectory disagrees with the carrier phase
and taroz does not.

Two changes fix it:

- Taroz `settings_test.csv` sets IdxStart=600 for this drive. Its first 599
  epochs are static, yet the carrier phase moves 0.9 m, as if the phone were
  handled. The drive is now solved from epoch 599 (`input_window`), and
  `assemble` fills the 599 leading keys with the first solved row, as taroz
  `submission.m` does.
- The drive gets `--native-source-tdcp-meter-sigma`,
  `--native-phase184-source-tdcp-huber-k`, `--native-source-route-type
  Highway` and `--native-source-l5 1`. With them, the exit agrees with the
  carrier phase within 0.3 m.

Distance to the taroz output, over all scored rows:

| Run | P50 | P95 |
|---|---:|---:|
| 57619467 | 0.44 m | 2.81 m |
| Window only | 0.44 m | 1.75 m |
| Window and source TDCP sigma | 0.31 m | 0.71 m |
| Window, source TDCP sigma and Phase184 Huber (recipe) | 0.31 m | 0.64 m |

Kaggle scores the last two Public 0.864 m (57626257) and 0.850 m (57626255),
with Private unchanged at 0.844 m. On train, the same TDCP flags on every
drive are worse: 4 of the first 6 drives regress, mtv-b Pixel4 by
+0.106 m. No train drive shows the ebf-x symptom, so they stay on this
drive only. Through the lane driver, `run --stage final` on ebf-x writes the
pinned windowed inputs and the research solution byte for byte. `assemble`
writes `f19acb58...` with 40/40 drives identical. The research file
57626255 has the same coordinates, but its fill step dropped trailing zeros
in 284 rows. The lane-built file was therefore submitted as 57629197, and Kaggle
scores it the same, Private 0.844 m / Public 0.850 m.

### gsdc-official third pass (2026-10-26)

The recipe then moved to 57664241. Taroz `run_fgo.m` solves each drive with
`fgo_gnss` and then `fgo_gnss_imu` twice. The last pass re-screens P and D
around the first GNSS/IMU trajectory with the tighter final thresholds and
solves again. Until now the native recipes ran one IMU solve. Thirty drives
now add `--native-imu-refinement-pass`. They also add the flags it requires
where the recipe lacked them: `--native-export-gnss-stage`,
`--native-export-imu-stage`, `--native-source-imu-initialization` and
`--native-phase213-main-doppler`. These are listed per drive under
`research_source.added_flags`; the previous provenance is under
`research_source.previous_research_source`.

Drives without the pass:

- Mi8 (5 drives): worse on train, +0.061 m mean.
- Pixel4 XL: the prerequisites cost +1.0 m on train.
- A325G (2 drives): neutral on train.
- sjc-r A505U: its course seeds exclude source IMU initialization.
- ebf-z Pixel5: Kaggle Public was worse with it (0.836 m vs 0.821 m).

On the 35 train drives with taroz output, the pass moves the mean from
0.7358 m to 0.7187 m (taroz 0.7123 m). Kaggle scores the three steps as
follows:

| Ref | Change | Public | Private |
|---|---|---:|---:|
| 57663786 | third pass on 30 drives, with ebf-z and without lax-p | 0.836 m | 0.811 m |
| 57663818 | ebf-z back to 57629197 | 0.821 m | 0.811 m |
| 57664241 | plus lax-p (recipe) | 0.817 m | 0.811 m |

lax-p Pixel5 crashes in its refinement solve in about two runs out of three
(SIGSEGV after 25-30 minutes): a TBB worker thread inside GTSAM overflows its
stack. Lifting the stack limit (`ulimit -s unlimited`) does not help, because
TBB sizes its worker stacks itself. Pinned to one CPU, TBB starts no workers
and the drive finishes in 1,670 s with the same solution. That holds with an
8 MB or an unlimited stack. The recipe gives that drive `single_cpu: true`,
and `run` pins the child to the first CPU it may use. Through the lane
driver, `run --stage final` on the 30 changed drives writes the research
solutions byte for byte. `assemble`, with the other 10 drives from the
57629197 rebuild, writes `ea13dc36...` with 40/40 drives identical.

### gsdc-official course seeds with the third pass (2026-10-26)

The recipe then moved to 57678818. sjc-r A505U is mounted backwards, so it
runs with `--native-course-attitude-seeds-mount-deg 178.9`. The third pass
needs `--native-source-imu-initialization`, and that flag turns on the epoch
heading seeds, which course seeds cannot be combined with. The solver now
accepts both flags together:

- The initial IMU pass takes the course seeds instead of the source heading
  seeds.
- The refinement pass takes the attitude that the initial pass optimized.

Every other flag combination builds the same configuration as before. With
the new build, the A505U recipe argv without the pass reproduces its pinned
output byte for byte. With the pass, the drive moves from 0.61 m to 0.56 m
from the taroz output.

Kaggle readback:

| Ref | Change | Public | Private |
|---|---|---:|---:|
| 57677499 | 57664241 plus sjc-r A505U | 0.817 m | 0.806 m |
| 57677502 | probe: plus mtv-e Pixel4 XL | 0.817 m | 0.806 m |
| 57677505 | probe: plus mtv-pe1 A325G | 0.821 m | 0.806 m |
| 57678818 | plus lax-hh A325G (recipe) | 0.816 m | 0.806 m |

mtv-pe1 A325G moved 0.16 m toward taroz and still made Public worse, so it
keeps its recipe. Pixel4 XL made no difference at three decimals.

The binary changes, so the whole lane was rerun from scratch on the new
build (`gnss_fgo_imu_no_base` SHA-256 `cccb04cc...`), four replays at a time
on 4 cores. It took 2 h 21 min:

- the 25 stage-0 runs and 25 height maps match their pinned SHA-256;
- the 40 final drives are identical to their research outputs (lax-p with
  `single_cpu`);
- `assemble` writes `2e0deb2c...`.

### gsdc-official test-tuned per-drive settings (2026-10-28)

The recipe then moved to 57717122, and **this step is tuned on the test
set**. Starting from 57678818, each candidate setting was rerun on its drive
alone and submitted with only that drive replaced. Each drive then takes the
setting with the largest strict improvement at three decimals in its split.
Every row is still native output. The candidates were:

- the third pass removed (57629197 output);
- main-graph XXVV motion (`--native-phase217-main-pose3-motion`, sigma 0.01 m
  Highway, 0.05 m Street, 0.1 m Mi8);
- source TDCP metre sigma with Phase184 Huber;
- no height map or relative-height constraint;
- RPYReset, and the source stop/observation stages.

RPYReset and the stages left drives within 0.06 m and were not probed.
18 drives changed:

- XXVV motion on 9 drives;
- source TDCP on the 4 Mi8 drives;
- no height constraint on 3 drives;
- no third pass on mtv-g Pixel5 and mtv-pe1 Pixel5 (2022-03-22).

The combined file scores **Private 0.767 m / Public 0.782 m**, from 0.806 /
0.816 m. That is below the 0.782 / 0.796 m post-deadline score of the taroz
code. The choices do not carry to train: XXVV motion on the 36 train drives
moves the mean from 0.7134 m to 0.7213 m, with 18 drives better and 18
worse. Each drive's probe ref and score are under
`research_source.test_tuned`.

The native app now gives TBB worker threads a 64 MiB stack. Three
concurrent lax-p runs that are not pinned to one CPU finish with the pinned
solution, so the drive no longer needs `single_cpu`. `run` also takes
`--jobs N`, and the lane manifest passes `--jobs 4`.

The binary changes, so the whole lane was rerun from scratch through the
lane driver on the new build (`gnss_fgo_imu_no_base` SHA-256 `bcd8f7c3...`)
with `--jobs 4`. It took 2 h 51 min on 4 cores:

- the 25 stage-0 runs and 25 height maps match their pinned SHA-256;
- the 40 final drives are identical to their research outputs (lax-p
  unpinned);
- `assemble` writes `02208f4f...`.

### gsdc-official truth-free selection and road snap (2026-10-28)

The recipe then moved to 57733487, which **uses no Kaggle feedback**. Every
rule below was fixed on the 36 train drives before it touched test, and no
Kaggle score chose anything. Starting again from 57678818:

1. **Height map.** A drive drops its height map when the map points within
   15 m of its 57678818 trajectory span a median height of 0.3 m or more.
   On train, with maps built without the drive's own course and trip, the map hurts
   more as that spread grows. 10 drives drop it.
2. **XXVV motion.** A325F, A325G and A205U always take it (all 4 train
   drives of these phones gain). Other drives take it when it lowers the
   median pseudorange residual against the trajectory (10 of 13 right on
   train).
3. **Source TDCP.** It competes on the same residual (3 of 3 right on train)
   on drives that keep their map.
4. **Road snap.** The new `road_snap` step in `assemble` builds a road map
   from the 156 train ground-truth files. Each moving epoch moves across
   track by half the median offset to map points within 6 m whose heading
   is within 25 degrees, using only offsets under 1 m. On train, with the
   map built without the drive's own course and trip, this gives
   -0.0130 m mean (P50+P95)/2, with 18 drives better and 6 worse.

22 drives change: XXVV motion on 11, no height map plus XXVV motion on 7, no
height map on 3, and source TDCP on ebf-ww Mi8. Their research outputs come
from the 57678818 argv with only those flags changed, under
`research_source.truth_free`. The refinement-pass toggle and relative-height
pairs were also checked on train and left out.

| Submission | Step | Private | Public |
|---|---|---:|---:|
| 57726666 | residual rule, motion and TDCP only | 0.792 | 0.817 |
| 57732745 | rules 1-3 | 0.784 | 0.816 |
| 57733311 | + nearest-point snap, map from the 40 taroz train drives | 0.777 | 0.809 |
| 57733407 | same snap, map from all 156 train files | 0.777 | 0.807 |
| 57733487 | median-offset snap (rule 4) | **0.775** | **0.804** |

Two caveats come with the "no Kaggle feedback" label. The variant types
(motion, TDCP, no height map) were chosen after the per-drive Kaggle probes
of 57717122. Also, 57733487 is the best of these five submissions. The
test-tuned 57717122 (0.767 / 0.782 m) stays in git history (develop
`e9a1f20`) and in the readback record.

`scripts/experiments/gsdc/gsdc_truthfree_select.py` recomputes the picks
from the 57678818 outputs, the variant outputs and the 25 height maps of
57678818 (`--variants-dir <dir>/<variant>/<course>/<phone>/solution.csv`,
`--maps-dir`, `--gsdc-root`). With the research outputs it picks the same 22
drives and gives every `prres_m` recorded under `research_source.truth_free`;
it exits non-zero when a pick differs from the recipe. The lane itself runs
the pinned argv and does not rerun the candidates.

`assemble` writes the 57732745 file (`9045daf6...`) first and then snaps it.
`--gsdc-truth-root` is therefore needed even when the height maps are
already built. The rewritten coordinates use `%.10f`.

The lane driver reran the new recipe from scratch on the same build
(`bcd8f7c3...`) with `--jobs 4`. It took 2 h 27 min on 4 cores:

- the 15 stage-0 runs and 15 height maps match their pinned SHA-256;
- the 40 final drives are identical to their research outputs;
- `assemble` writes `9045daf6...` before the snap and `a81ceb0b...` after
  it (13,998 epochs moved on 26 drives, at most 0.50 m).

### gsdc-official road prior in the factor graph (2026-10-28)

The recipe then moved to 57742915. It keeps the 22 picks of 57733487 and
drops the post-hoc snap. Instead, the native app takes the road map as a
cross-track prior in the IMU main graph (`--native-road-map`, the horizontal
counterpart of `--native-height-map`):

- For a moving seed, map points within 6 m whose heading is within 25
  degrees give the seed's offset along their left normals.
- Offsets under 1 m are kept, and the position is held toward the seed
  minus the median offset along their mean normal (sigma 0.5 m, Huber 0.5).

The new `road-maps` step builds each drive's map from the 156 train truth
files: moving points with their local heading, within 30 m of the drive's raw
WLS track. 32 drives get a non-empty map and run with the prior; 26 of them
change.

On train, each map was built without the drive's own course and trip:

- the mean (P50+P95)/2 drops by -0.106 m over 36 drives, with 22 better and
  1 worse (mtv-h Pixel5 +0.098 m);
- sigma 0.5 m was the default set before any run, not tuned;
- the post-hoc snap on top of the prior loses (-0.106 -> -0.099 m).

| Submission | Step | Private | Public |
|---|---|---:|---:|
| 57733487 | post-hoc snap | 0.775 | 0.804 |
| 57742915 | road prior in the factor graph | **0.714** | **0.732** |

The caveats of 57733487 still apply: the variant types came from the
57717122 probes, and 57742915 is the sixth no-feedback submission.

The lane driver reran the new recipe from scratch on the build that adds the
prior (`gnss_fgo_imu_no_base` SHA-256 `7a9b7264...`; with the prior off it
reproduces the earlier outputs) with `--jobs 4`. It took 2 h 23 min on 4
cores:

- the 15 stage-0 runs, 15 height maps and 32 road maps match their pinned
  SHA-256;
- the 40 final drives are identical to their research outputs;
- `assemble` writes `12d39d57...`.

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

## has-idd-ppp: Galileo HAS float PPP (2026-09-29)

`has-idd-ppp` is a docs lane, not a README row. It runs `gnss_ppp` with
`--ssr-rtcm-profile has-idd` on the public Galileo HAS Internet Data
Distribution sample of [hirokawa/cssrlib-data](https://github.com/hirokawa/cssrlib-data)
(`data/doy2023-229`: station OBE4, 2023-08-17 01:59-03:00 GPST, RTCM 3 SSR
1060 / 1059 / 1243 / 1242 plus 1019 / 1046 ephemerides), static and kinematic,
plus the `legacy` RTCM SSR conversion of the same stream for comparison, and
scores each `.pos` against the cssrlib reference coordinate with
`scripts/experiments/has/has_idd_ppp_reproduce.py`.

```bash
python3 apps/gnss.py reproduce has-idd-ppp --has-data-root /datasets/cssrlib-data/data/doy2023-229 --check
```

The cssrlib-data license is not stated upstream, so the three input files are
not redistributed; download them into the `--has-data-root` directory.

Local result (MSVC Release, 44 s): every metric passes. The static run is at
0.014 m horizontal / +0.050 m vertical after one hour and converges below
0.20 m horizontal after 0.8 min and below 0.40 m vertical after 5.8 min
(0.095 / -0.135 m, 4.2 / 21.0 min before the static filter committed one
measurement update per epoch, see [igs-final-ppp](#igs-final-ppp-precise-product-ppp-2026-10-01));
the gates are H <= 0.20 m and |U| <= 0.40 m at 60 min, and convergence within
10 min (H) and 30 min (U). The kinematic run is at 0.059 / +0.015 m after
30 min and 0.115 / +0.059 m after one hour, gated at H <= 0.30 m and
|U| <= 0.60 m at 30 and 60 min (before the one-measurement-update-per-epoch
fix of the kinematic PPP filter it stayed at about 1.9 m / -5.6 m; before the
kinematic post-fit residual screening it read 0.136 / +0.058 m and
0.018 / -0.390 m, with an hour RMS of 0.156 / 0.206 m instead of
0.086 / 0.099 m). The legacy
conversion is reported, not gated. See [Galileo HAS support](galileo_has.md)
for the full table and the cssrlib comparison.

## has-sis-ppp: Galileo HAS SIS float PPP (2026-09-30)

`has-sis-ppp` is a docs lane, not a README row. It decodes the HAS E6-B pages
of two public Kamakura hours of [hirokawa/cssrlib-data](https://github.com/hirokawa/cssrlib-data)
(`data/doy2025-046`, 2025-02-15 17h, and `data/doy2025-233`, 2025-08-21 07h)
with `gnss_has_info`, runs `gnss_ppp --has-pages` static and kinematic on both
with the IGS merged BRDC navigation of the day, and scores each `.pos` against
the cssrlib reference coordinate.

```bash
python3 apps/gnss.py reproduce has-sis-ppp --has-sis-data-root /datasets/cssrlib-data/data --check
```

Local result (MSVC Release, 2026-10-01): every metric passes. Each hour
yields 432 MT1 messages without a CRC failure. The 2025-02-15 static run is at
0.031 / +0.126 m after 30 min and 0.184 / -0.123 m after one hour (gates:
H <= 0.20 m, |U| <= 0.40 m at 30 and 60 min; before the static
one-update-per-epoch fix 0.026 / +0.184 m and 0.010 / +0.041 m; the kinematic
run, unchanged, ends at 0.184 / -0.256 m). The 2025-08-21 hour is reported
only: libgnss++ ends 0.1-0.4 m low there depending on the correction source
(0.7-0.8 m before the static fix, about 1 m before the solid-earth-tide fix). Kamakura
is outside the HAS service area, so the numbers are indicative. See
[Galileo HAS support](galileo_has.md) for the decoder parity with cssrlib and
the full comparison.

## igs-final-ppp: precise-product PPP (2026-10-01)

`igs-final-ppp` is a docs lane, not a README row. It runs `gnss_ppp --sp3
--clk --antex --static --elevation-mask 10` with IGS final orbits and clocks
(`IGS0OPSFIN`, 15 min SP3, 30 s CLK) on the two Kamakura hours of
`has-sis-ppp` and the OBE4 hour of `has-idd-ppp`, and scores each `.pos`
against the same reference coordinates.

```bash
python3 apps/gnss.py reproduce igs-final-ppp --has-sis-data-root /datasets/cssrlib-data/data \
  --has-data-root /datasets/cssrlib-data/data/doy2023-229 --check
```

The IGS products and `igs20.atx` are public but large, so they are not
committed: download them as listed in [Datasets](#datasets) and gunzip them
next to the observations. Take `igs20.atx` from the IGS PCV archive
(`pcv_archive/igs20_2425.atx.gz`, gunzipped SHA-256
`8715268e17e09e5447f4949d67cbd067e7f0f33d48dd698aafe14f5cffb26de2`): the
live `general/igs20.atx` is updated in place, so its content changes over
time. IGS0OPSFIN carries GPS only, so the other
constellations of the multi-GNSS observation files are dropped (satellites
without a precise orbit and clock are excluded rather than mixed in on
broadcast clocks, as RTKLIB does with `sateph = precise`).

Local result (MSVC Release, 2026-10-01, 168 s), H / U in metres after
10 / 30 / 60 min, with RTKLIB demo5 b34j PPP-static on the same observations,
products and ANTEX (GPS, L1+L2 ionosphere-free, estimated ZTD, 10 degrees,
tides):

| Run | libgnss++ | RTKLIB demo5 | before the static one-update fix | before the precise-product fix |
|---|---|---|---|---|
| Kamakura 2025-08-21 07h | 0.455 / -0.630, 0.182 / -0.241, **0.099 / -0.223** | 0.434 / -0.488, 0.154 / -0.179, 0.099 / -0.212 | 0.490 / -0.646, 0.168 / -0.222, 0.120 / -0.265 | 1.391 / +0.870, 1.453 / +1.384, 1.502 / +0.889 |
| Kamakura 2025-02-15 17h | 0.576 / -0.164, 0.297 / -0.204, **0.151 / -0.007** | 0.717 / -0.147, 0.418 / -0.202, 0.267 / -0.017 | 0.611 / -0.007, 0.454 / -0.205, 0.315 / +0.027 | 1.459 / +0.502, 1.226 / +1.164, 1.466 / +0.747 |
| OBE4 2023-08-17 02h | 0.172 / -0.192, 0.172 / -0.126, **0.135 / -0.053** | 0.426 / -0.245, 0.315 / -0.127, 0.280 / -0.054 | 0.171 / -0.341, 0.209 / -0.165, 0.199 / -0.057 | 2.773 / -2.974, 2.353 / -0.779, 1.561 / -0.623 |

Convergence (H < 0.20 m / |U| < 0.40 m, staying there): Kamakura 08-21
29.1 / 21.0 min, 02-15 51.4 / 7.7 min, OBE4 28.2 / 0.8 min.

The gates are H <= 0.40 m and |U| <= 0.40 m at 60 min for the Kamakura hours
and 0.30 m for OBE4. Before the fix, precise-product PPP omitted the periodic
relativistic satellite clock term (-2 r.v/c^2, which IGS SP3/CLK clocks
exclude), fell back to broadcast orbits and clocks for satellites missing
from the SP3, and did not apply the satellite antenna PCO to the
centre-of-mass SP3 orbits; with GPS-only observations the 2025-08-21 hour
ended at E -5.5 / N -3.3 / U +4.3 m.

### Static filter: one measurement update per epoch (2026-10-01)

Until 2026-10-01 the static and `--low-dynamics` PPP filter applied each
epoch's measurement update up to eight times (three with SP3/CLK products).
`applyPreciseCorrections()` evaluates the observation geometry once, at the
prior position, so every pass after the first read the position innovation
the state had already absorbed and pushed the position again on an
already-shrunk covariance; the clock, troposphere and ambiguity terms were
re-evaluated each pass and took up the overshoot. On the first epochs, where
the prior is the SPP seed metres away, this moved the solution tens to
hundreds of metres (the first broadcast epoch of the Kamakura hour moved the
position 70 m cumulatively and the zenith delay to 4.66 m) before the phase
rows pulled it back, and with 30 s data the filter wandered for the first
hour. RTKLIB and MADOCALIB commit one update per epoch (`pppos()` restarts
every residual-screening pass from the prior state), and a single update
linearized at the prior is exact to well below a millimetre for GNSS ranges.
Kinematic PPP was moved to one update per epoch earlier; static and
`--low-dynamics` PPP now do the same. The coherent MADOCA static
ionosphere-free profile followed the same day, once its SPP anchor blend was
removed (see `docs/madocalib_native_migration.md`).

Two static-only gaps showed up once the start-up push was gone, and are fixed
with it: static ionosphere-free PPP without SSR ran without geometry-free /
Melbourne-Wubbena slip detection (RTKLIB `detslp_gf` / `detslp_mw`), relying
on the LLI flag alone; and with SP3/CLK products a satellite whose second
frequency dropped out stayed in the filter on raw L1 code (ionosphere
uncorrected) with its L1 phase tied to the ionosphere-free ambiguity (RTKLIB
skips such a satellite). On TSK2 G22 lost L2 for four epochs at 21:52 GPST;
when it came back with a new L2 ambiguity the IF phase residual was -7.6 m and
the position ended the day 0.96 m east.

Static PPP, IGS finals, 30 s data (H / U in metres at 10 / 60 min; end of
day E / N / U for the 24 h runs; max 3D over the run):

| Run | before | after | RTKLIB demo5 |
|---|---|---|---|
| Kamakura 2025-08-21 07h decimated to 30 s, GPS only | 19.18 / -42.74, 3.99 / +9.96; max 252 m | 0.321 / -0.615, **0.046 / -0.154**; max 7.6 m (first epoch) | 0.204 / -0.322, 0.088 / -0.142 |
| TSK2 2024-01-01 24 h (multi-GNSS file, GPS used) | 31.3 / +135.5, 4.16 / +5.88; end -0.972 / +0.216 / +0.238; max 931 m | 0.763 / +0.567, 0.072 / +0.013; end **+0.007 / +0.004 / -0.042**; max 1.7 m | 0.569 / +0.291, 0.095 / +0.054; end -0.010 / +0.003 / +0.002 |
| TSK2 2024-01-01 24 h, GPS-only file | 22.0 / -92.7, 2.57 / -4.21; end -0.865 / +0.137 / -0.146 | 0.749 / +0.550, 0.072 / +0.013; end +0.007 / +0.004 / -0.042 | (as above) |
| TSKB 2025-08-21 24 h vs CODE daily SINEX | 145.6 / -78.1, 4.24 / -8.87; end +0.122 / -0.092 / -0.115; max 438 m | 1.106 / -0.987, 0.054 / -0.018; end **-0.006 / +0.012 / +0.001**; max 2.5 m | - |

Convergence (H < 0.20 m / |U| < 0.40 m): TSK2 27.0 / 19.5 min (before: never /
1,362 min; RTKLIB 31.0 / 2.0 min), TSKB 31.0 / 22.5 min (before: 492 /
438 min). Of the three parts, the single update removes the start-up
excursion and the 30 s divergence; the slip detection removes the TSK2
end-of-day 0.96 m; dropping single-frequency satellites removes a 7.9 m
excursion at 30 min on TSKB.

Broadcast-only static PPP on the Kamakura hours (1 Hz, H / U at 60 min):

| Run | before | after |
|---|---|---|
| 2025-08-21 GPS | 0.388 / -0.531 | 0.411 / -0.889 (RTKLIB demo5 broadcast PPP-static: 0.523 / -0.784) |
| 2025-08-21 GPS + Galileo | 0.430 / -0.655 | 0.651 / -0.246 |
| 2025-02-15 GPS + Galileo | 0.212 / +0.252 | 0.319 / +0.123 |
| 2025-08-21 all systems | 0.631 / -5.109 | 1.079 / -9.259 |
| 2025-02-15 all systems | 1.081 / +1.804 | 0.489 / +1.505 |
| 2025-08-21 all systems, decimated to 30 s | 1.043 / -2.979 (10 min: 16.2 / -16.9) | 0.424 / -0.191 (10 min: 0.957 / -0.678) |
| 2025-08-21 all systems, `--kinematic --low-dynamics` | 0.620 / -3.516 | 0.071 / -0.600 |
| OBE4 2023-08-17 GPS + Galileo | 0.364 / -0.195 | 0.187 / -0.102 |

The all-system broadcast rows are dominated by BeiDou-3, not by the update
count: GPS + BDS-3 alone ends 5.0 m low before and 13.2 m low after, GPS +
BDS-2 1.46 m low before and 0.00 m after, while GPS + GLONASS, GPS + QZSS and
GPS + Galileo + QZSS end within 0.52 m in up with one update. The
multi-pass filter damped that BeiDou-3 broadcast bias; its cause is the
BeiDou signal pairing and receiver clock fixed in the next section.

### BeiDou with broadcast ephemerides (2026-10-01)

Two modelling errors made broadcast-only PPP with BeiDou-3 metres off; the
broadcast BeiDou orbits and clocks themselves are fine. Against the WUM0MGXFIN
multi-GNSS finals of both days (5-min samples over the hour, broadcast clocks
moved from B3I to the B1I/B3I ionosphere-free reference with TGD1), every
BDS-3 satellite has a mean radial difference of -0.8 to -1.8 m (GPS -0.3 to
-2.3 m, the broadcast antenna-phase-centre vs SP3 centre-of-mass offset) and
a clock within 1.0 m of its system median. So the BDT time tag, the GEO
rotation and the CGCS2000 frame are not the cause.

1. **B2b used as B2I.** BDS-3 satellites (C19 and above) do not transmit B2I;
   the band-7 code a receiver logs for them (`C7D` / `C7P` / `C7Z`) is B2b,
   whose group delay is only broadcast in B-CNAV3. The RINEX reader keeps one
   secondary observation per satellite with band 7 ahead of band 6, so the
   Kamakura BDS-3 satellites (`C2I C5P C6I C7D`) were processed as B1I / "B2I"
   with TGD1 / TGD2 removed, and their D1 TGD2 field simply repeats TGD1 (all
   BDS-3 satellites in both IGS merged BRDC files). Broadcast BDS-3 now pairs
   B1I with B3I (`C6I` / `C6Q` / `C6X`, taken from the per-tracking-code
   observations when the reader selected B2b) and removes TGD1 only; the
   geometry-free / Melbourne-Wubbena slip test uses the same pair and the
   B3I loss-of-lock flag. BDS-2 keeps B1I / B2I with TGD1 / TGD2.
2. **No BeiDou receiver clock.** Galileo / QZSS / BeiDou shared the GPS
   receiver clock. Relative to the WUM finals the broadcast clocks of each
   system have their own datum (per-system median, metres, GPS / BDS-3 /
   BDS-2: 2025-08-21 +4.0 / +1.8 / +10.0, 2025-02-15 +0.1 / +4.9 / +11.2), and
   with the GPS clock the prefit code residuals of BDS-3 sit about 2 m and
   those of BDS-2 about 5 m above the GPS ones on the 2025-08-21 hour. With
   broadcast ephemerides only, PPP now estimates one receiver clock for BDS-3
   and one for BDS-2, as MADOCALIB does (RTKLIB estimates one clock per
   system). The per-epoch re-seeding of the GPS clock from the SPP keeps the
   Galileo / QZSS / BeiDou inter-system biases and their covariance instead of
   re-initializing those clocks; GLONASS keeps its re-initialized clock.
   On the PPC drives (Septentrio, B1I / B3I for BDS-3) the estimated biases
   are steady at about +7.7 m (BDS-3) and +3.6 m (BDS-2).

Precise-product, SSR (CLAS, MADOCA, HAS) and DCB runs are unchanged: both
changes apply only to the broadcast ionosphere-free path.

Broadcast-only static PPP on the Kamakura hours (1 Hz, H / U in metres at 10 /
60 min):

| Run | before | after |
|---|---|---|
| 2025-08-21 GPS + BDS-3 | 2.866 / -5.862, 3.784 / -13.183 | 0.598 / -0.705, **0.273 / -0.200** |
| 2025-02-15 GPS + BDS-3 | 0.166 / +0.855, 0.259 / +0.014 | 0.556 / +0.154, 0.141 / -0.061 |
| 2025-08-21 GPS + BDS-2 | 2.124 / -0.336, 1.133 / +0.004 | 0.669 / -1.829, 0.458 / -0.971 |
| 2025-02-15 GPS + BDS-2 | 1.474 / +1.438, 1.080 / +0.223 | 0.891 / -0.564, 0.150 / -0.315 |
| 2025-08-21 GPS + BeiDou | - | 0.650 / -0.961, 0.349 / -0.245 |
| 2025-08-21 GPS + Galileo + QZSS + BeiDou | - | 0.689 / -0.063, 0.186 / -0.355 |
| 2025-08-21 all systems | 0.905 / -4.052, 1.079 / -9.259 | 0.425 / +0.492, **0.154 / -0.264** |
| 2025-02-15 all systems | 0.965 / +0.732, 0.451 / +1.626 | 0.274 / -0.307, 0.202 / +1.589 |
| 2025-08-21 GPS (unchanged) | 0.524 / -1.528, 0.411 / -0.889 | same |

The remaining +1.6 m up of the 2025-02-15 all-system run comes from GLONASS
(GPS + GLONASS alone ends at +2.0 m; one GLONASS clock, no inter-frequency
code biases). RTKLIB demo5 b34 PPP-static with `pos1-navsys=33` (GPS +
BeiDou) on the same files is not a usable reference: it reproduces its
GPS-only result to the millimetre on the GPS + BDS-3 files and ends 2.5-3.8 m
off with the BDS-2 satellites (GEO included).

