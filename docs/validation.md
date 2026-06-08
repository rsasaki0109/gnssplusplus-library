# Validation

## Sign-off commands

Sign-off commands are quality gates around real solver commands. They emit
summary JSON and fail when thresholds are violated.

Main sign-off entrypoints:

- `gnss short-baseline-signoff`
- `gnss rtk-kinematic-signoff`
- `gnss ppp-static-signoff`
- `gnss ppp-kinematic-signoff`
- `gnss ppp-products-signoff`
- `gnss live-signoff`
- `gnss moving-base-signoff`
- `gnss ppc-rtk-signoff`
- `gnss ppc-coverage-matrix`
- `gnss ppc-taroz-amb-pdc-smoke`
- `gnss taroz-observable-dogfood`
- `gnss taroz-oracle-suite`
- `gnss odaiba-benchmark --require-*`
- `gnss public-rtk-benchmarks`
- `gnss smartloc-adapter`
- `gnss smartloc-signoff`

Example:

```bash
python3 apps/gnss.py ppp-static-signoff \
  --fetch-products \
  --product-date 2024-01-02 \
  --product sp3=https://cddis.nasa.gov/archive/gnss/products/{gps_week}/COD0OPSFIN_{yyyy}{doy}0000_01D_05M_ORB.SP3.gz \
  --product clk=https://cddis.nasa.gov/archive/gnss/products/{gps_week}/COD0OPSFIN_{yyyy}{doy}0000_01D_30S_CLK.CLK.gz \
  --product ionex=https://cddis.nasa.gov/archive/gnss/products/ionex/{yyyy}/{doy}/COD0OPSFIN_{yyyy}{doy}0000_01D_01H_GIM.INX.gz \
  --product dcb=https://cddis.nasa.gov/archive/gnss/products/bias/{yyyy}/CAS0MGXRAP_{yyyy}{doy}0000_01D_01D_DCB.BSX.gz \
  --require-ppp-solution-rate-min 100

python3 apps/gnss.py ppp-kinematic-signoff \
  --max-epochs 120 \
  --require-common-epoch-pairs-min 120 \
  --require-reference-fix-rate-min 95 \
  --require-converged \
  --require-convergence-time-max 300 \
  --require-mean-error-max 7 \
  --require-p95-error-max 7 \
  --require-max-error-max 7 \
  --require-mean-sats-min 18 \
  --require-ppp-solution-rate-min 100

python3 apps/gnss.py ppp-products-signoff \
  --profile static \
  --require-converged \
  --require-ionex-corrections-min 1 \
  --require-dcb-corrections-min 1

python3 apps/gnss.py ppp-products-signoff \
  --profile ppc \
  --run-dir /datasets/PPC-Dataset/tokyo/run1 \
  --require-converged \
  --require-ppp-solution-rate-min 95 \
  --require-lib-mean-error-vs-malib-max-delta 0.25
```

For the `ppc` profile with `--malib-pos` or `--malib-bin`, the command also emits paired comparison artifacts (`comparison_csv`, `comparison_png`) and `common_epoch_pairs`, so `gnss web` and CI artifact bundles can show the same side-by-side error slices.

`gnss live-signoff`, `gnss ppp-products-signoff`, `gnss moving-base-signoff`,
`gnss scorpion-moving-base-signoff`, and `gnss ppc-rtk-signoff`
also accept `--config-toml`, so you can pin long threshold sets and artifact paths in a file instead of repeating them on the command line.

For public benchmark rows that can be expressed as rover/base/nav plus an
independent `reference.csv`, `gnss ppc-demo` and `gnss ppc-rtk-signoff` also
accept `--commercial-pos` for an existing receiver solution or
`--commercial-rover` for a commercial receiver rover RINEX solved through
libgnss++ against the same base/nav/reference. The receiver summary is stored
under `commercial_receiver`, and `delta_vs_commercial_receiver` reports
libgnss++ minus receiver deltas for positioning rate, fix rate, PPC official
distance-ratio score, 3D<=50cm reference-epoch score, and horizontal/up error metrics.
Run `gnss public-rtk-benchmarks` before treating any single public dataset as
representative coverage; UrbanNav Tokyo is a Tier-1 smoke/regression row, not
the final commercial RTK receiver proof.

PPC-Dataset is the primary public moving-RTK sign-off. It carries survey-grade
receiver observations, reference-station observations, broadcast nav, and
reference trajectory truth. `gnss ppc-demo` records the rover/base receiver and
antenna provenance under `receiver_observation_provenance`; proprietary
receiver-engine solutions are intentionally not the benchmark target. It also
reports `positioning_rate_pct` separately from `fix_rate_pct`, so no-solution
gaps cannot be hidden by a high fixed-solution ratio over only positioned
epochs. Use `--no-arfilter --no-kinematic-post-filter` when validating the RTK
coverage profile; that keeps valid SPP/float fallback epochs and records
`rtk_output_profile: coverage` in the summary JSON. The default low-speed
non-FIX drift guard remains active in that profile; it rejects bounded
FLOAT/SPP segments whose surrounding FIX anchors are nearly stationary but whose
fallback positions drift more than 30 m from the anchor bridge. Use
`--no-nonfix-drift-guard` only when reproducing the unguarded fallback stream.
The default SPP height-step guard then removes SPP-only vertical spikes above
the `--spp-height-step-min` / `--spp-height-step-rate` envelope; use
`--no-spp-height-step-guard` only when reproducing the raw SPP fallback stream.
The FLOAT bridge-tail guard is also default-on after six-run PPC sign-off: it
rejects FLOAT epochs in slow bounded FIX-to-FIX segments when their position
diverges from the anchor bridge. Its speed gate uses horizontal FIX-anchor
speed so vertical anchor noise does not create false motion; use
`--no-float-bridge-tail-guard` only when reproducing the pre-bridge-tail
coverage stream.
For isolated false-fix spikes, `--fixed-bridge-burst-guard` is available as a
default-off PPC tuning gate. It inspects short FIX segments bounded by nearby
FIX anchors and rejects only the burst epochs whose bridge residual exceeds
`--fixed-bridge-burst-max-residual` (20 m by default). On Tokyo run1 ratio 2.4
coverage it rejects 12 epochs, lowering max H by 4.33 m and P95H by 0.12 m
while costing 0.10 pp Positioning and 0.03 pp PPC official score, so keep it
explicit when evaluating precision-tail tradeoffs.
Use `--nonfix-drift-max-residual 4 --nonfix-drift-min-horizontal-residual 6`
only for an explicit P95-cleanup profile: combined with the fixed-burst guard it
rejects 226 non-FIX drift epochs on Tokyo run1, improves P95H from 34.53 m to
30.61 m, and keeps PPC official nearly flat, but costs 1.47 pp Positioning rate.
`ppc-coverage-matrix` accepts these non-FIX, SPP height-step, FLOAT bridge-tail,
and fixed-burst tuning flags so the same profile can be swept across all six PPC
runs. The full six-run sweep keeps a +15.7 pp average Positioning lead over
RTKLIB and a +28.1 pp PPC official lead, but costs 1.33 pp average Positioning
versus the coverage profile and only improves P95H on 3/6 runs; the horizontal
residual floor reduces Nagoya run3 over-pruning from 13.90 pp to 3.48 pp
Positioning cost. Treat the profile as a tail-diagnostic lens, not as the
Positioning-rate sign-off.
Use `scripts/analyze_ppc_coverage_quality.py` with the PPC solution, RTKLIB
solution, and `reference.csv` when a coverage run improves Positioning rate but
regresses P95 horizontal error; the report separates FIXED/FLOAT/SPP quality and
bad continuous drift segments. The segment CSV also records adjacent FIX-anchor
gap/speed, solution path length, and bridge residuals so FLOAT-tail guards can
be designed from bounded segment evidence instead of status-only ratios.
Add `--official-segments-csv` when tuning PPC official score directly; it writes
one row per reference distance segment with gnssplusplus and RTKLIB score state,
3D error, status, and score-delta distance.
Use `--iono auto|off|iflc|est` on `ppc-demo`, `ppc-rtk-signoff`, or
`ppc-coverage-matrix` when sweeping RTK ionosphere handling; the PPC summary
records the requested mode as `rtk_iono`.
Use `--ratio <value>` on the same commands when sweeping ambiguity-ratio
thresholds; the summary records the requested value as `rtk_ratio_threshold`.
Use `--max-hold-div`, `--max-pos-jump`, `--max-pos-jump-min`, and
`--max-pos-jump-rate` for explicit fixed-solution validation sweeps; summaries
record the jump settings as `rtk_max_position_jump_m`,
`rtk_max_position_jump_min_m`, and `rtk_max_position_jump_rate_mps`.

## Taroz ambiguity PDC FGO beta validation

`gnss ppc-taroz-amb-pdc-smoke` is the validation harness for the beta C++ port
of taroz's PPC `amb-pdc` FGO path. It is separate from the production RTK
sign-off commands above: its job is to dogfood the taroz-style FGO runner on
PPC-Dataset and to keep the port's intermediate diagnostics inspectable.

Use the short command in CI or quick local checks:

```bash
python3 apps/gnss.py ppc-taroz-amb-pdc-smoke \
  --dataset-root /datasets/PPC-Dataset \
  --max-epochs 20 \
  --summary-json output/dogfood/ppc_taroz_amb_pdc_smoke/summary.json
```

Use longer local sign-offs before treating the beta path as healthy:

```bash
python3 apps/gnss.py ppc-taroz-amb-pdc-smoke \
  --dataset-root /datasets/PPC-Dataset \
  --max-epochs 200 \
  --generate-spp-seed \
  --require-valid-p95-3d-max 2.0 \
  --summary-json output/dogfood/ppc_taroz_amb_pdc_smoke_200/summary.json

python3 apps/gnss.py ppc-taroz-amb-pdc-smoke \
  --dataset-root /datasets/PPC-Dataset \
  --run nagoya/run3 \
  --skip-epochs 400 \
  --max-epochs 200 \
  --generate-spp-seed \
  --require-valid-p95-3d-max 2.0 \
  --summary-json output/dogfood/ppc_taroz_amb_pdc_nagoya_run3_shifted/summary.json

python3 apps/gnss.py ppc-taroz-amb-pdc-smoke \
  --dataset-root /datasets/PPC-Dataset \
  --run nagoya/run3 \
  --max-epochs 1000 \
  --generate-spp-seed \
  --require-valid-p95-3d-max 1.1 \
  --require-fixed-p95-3d-max 0.2 \
  --summary-json output/dogfood/ppc_taroz_amb_pdc_nagoya_run3_1000_seed_current/summary.json

python3 apps/gnss.py ppc-taroz-amb-pdc-smoke \
  --dataset-root /datasets/PPC-Dataset \
  --run nagoya/run2 \
  --max-epochs 1000 \
  --generate-spp-seed \
  --require-valid-p95-3d-max 0.25 \
  --require-fixed-p95-3d-max 0.22 \
  --summary-json output/dogfood/ppc_taroz_amb_pdc_nagoya_run2_1000_seed_current/summary.json

python3 apps/gnss.py ppc-taroz-amb-pdc-smoke \
  --dataset-root /datasets/PPC-Dataset \
  --run tokyo/run2 \
  --max-epochs 1000 \
  --generate-spp-seed \
  --require-valid-p95-3d-max 1.6 \
  --require-fixed-p95-3d-max 0.1 \
  --summary-json output/dogfood/ppc_taroz_amb_pdc_tokyo_run2_1000_seed_current/summary.json
```

The dataset-gated optional CI sign-off runner also includes the
`nagoya/run3` 1000-epoch generated-seed check when `GNSSPP_PPC_DATASET_ROOT`
is configured.

The `taroz-amb-pdc` preset enables two truth-free FLOAT output guards by
default: `--max-float-seed-divergence 100` and
`--max-float-position-jump 100`. Rejected FLOAT epochs are emitted as
no-solution and counted in the summary JSON as
`float_rejected_seed_position_divergence` and
`float_rejected_position_jump`. The PPC harness also records 3D error tail
counts and the worst epoch in each reference-summary bucket, so long runs show
both p95 behavior and isolated fixed/FLOAT/no-solution outliers.

The beta scope is intentionally narrow. The C++ path covers generated SPP
seeding, double-difference FGO, FLOAT/FIXED output, and diagnostic summaries for
the PPC `amb-pdc` workflow. The MATLAB-oracle dogfood harnesses cover the
ported taroz modes below. Their final-output contract tests pin the emitted
epoch count, GPS time sequence, summary/graph counts, optimizer cost fields, and
finite state columns so output regressions are caught even when full MATLAB
parity artifacts are not checked into the repository.

| Mode | Dogfood entrypoint | Main output contract |
|---|---|---|
| P | `taroz-p-dogfood` | `fgo_taroz_p.pos`, epoch debug CSV, and summary counts |
| D | `taroz-observable-dogfood --mode d` | `per_epoch_vel.csv`, `graph_detail.csv`, and summary counts/cost |
| PD | `taroz-pd-dogfood` | `per_epoch_state.csv`, `graph_detail.csv`, and summary counts/cost |
| position PD | `taroz-observable-dogfood --mode pos-pd` | position/clock `per_epoch_state.csv` and graph/summary counts |
| position PDC | `taroz-observable-dogfood --mode pos-pdc` | position/clock `per_epoch_state.csv` and graph/summary counts |
| position/velocity PDC | `taroz-observable-dogfood --mode pos-vel-pdc` | position/velocity/clock `per_epoch_state.csv` and graph/summary counts |
| PC | `taroz-pc-dogfood` | final `.pos`, epoch debug, factor debug, LAMBDA debug, and summary counts |
| position/velocity ambiguity PDC | `taroz-pos-vel-amb-pdc-dogfood` | epoch debug, factor debug, SD factor debug, LAMBDA debug, optimizer cost trace, and summary counts |

Use the suite entrypoint for a broad local gate:

```bash
python3 apps/gnss.py taroz-oracle-suite \
  --native-bin-dir build/apps \
  --out-root output/dogfood/taroz_oracle_suite_current
```

Add `--generate-matlab-dump --taroz-root /path/to/taroz_gtsam_gnss` when
MATLAB and the taroz checkout are available. The taroz MATLAB example directory
must have the dependencies expected by taroz itself, including the GTSAM MATLAB
toolbox and `readobs` path. Use `--taroz-example-dir` when those example files
do not live under `<taroz-root>/examples`.

Run a focused mode when isolating a regression:

```bash
python3 apps/gnss.py taroz-p-dogfood --generate-matlab-dump
python3 apps/gnss.py taroz-pd-dogfood --generate-matlab-dump
python3 apps/gnss.py taroz-pc-dogfood --generate-matlab-dump
python3 apps/gnss.py taroz-observable-dogfood --mode pos-pdc --generate-matlab-dump
python3 apps/gnss.py taroz-pos-vel-amb-pdc-dogfood --generate-matlab-dump
```

The ambiguity PDC dogfood also has non-default expectation profiles for
option-level sign-offs that should not be judged with the default FIX/FLOAT
counts. The harness always checks the native cost trace shape against the
summary, so these profiles still pin the C++ solver trajectory contract when
`--skip-parity` is used. The default seed profile expects 1141 matched seed
epochs with 34 interpolated epochs; the `no-seed-interpolation` profile pins
the same run with interpolation disabled at 1107 matched seed epochs and zero
interpolated epochs.

```bash
python3 apps/gnss.py taroz-pos-vel-amb-pdc-dogfood \
  --fgo-extra-arg=--no-epoch-lambda-fixed-output \
  --expectation-profile no-epoch-lambda-fixed-output \
  --skip-parity

python3 apps/gnss.py taroz-pos-vel-amb-pdc-dogfood \
  --fgo-extra-arg=--lambda-ratio-threshold \
  --fgo-extra-arg=100 \
  --expectation-profile strict-lambda-ratio \
  --skip-parity

python3 apps/gnss.py taroz-pos-vel-amb-pdc-dogfood \
  --fgo-extra-arg=--seed-interpolation-max-gap \
  --fgo-extra-arg=0 \
  --expectation-profile no-seed-interpolation \
  --skip-parity

python3 apps/gnss.py taroz-pos-vel-amb-pdc-dogfood \
  --fgo-extra-arg=--max-epochs \
  --fgo-extra-arg=120 \
  --expectation-profile first-120-window \
  --skip-parity

python3 apps/gnss.py taroz-pos-vel-amb-pdc-dogfood \
  --fgo-extra-arg=--max-epochs \
  --fgo-extra-arg=120 \
  --fgo-extra-arg=--lambda-ratio-threshold \
  --fgo-extra-arg=100 \
  --expectation-profile first-120-strict-lambda-ratio \
  --skip-parity

python3 apps/gnss.py taroz-pos-vel-amb-pdc-dogfood \
  --fgo-extra-arg=--skip-epochs \
  --fgo-extra-arg=400 \
  --fgo-extra-arg=--max-epochs \
  --fgo-extra-arg=120 \
  --expectation-profile shifted-120-window \
  --skip-parity

python3 apps/gnss.py taroz-pos-vel-amb-pdc-dogfood \
  --fgo-extra-arg=--skip-epochs \
  --fgo-extra-arg=400 \
  --fgo-extra-arg=--max-epochs \
  --fgo-extra-arg=120 \
  --fgo-extra-arg=--lambda-ratio-threshold \
  --fgo-extra-arg=100 \
  --expectation-profile shifted-120-strict-lambda-ratio \
  --skip-parity

python3 apps/gnss.py taroz-pos-vel-amb-pdc-dogfood \
  --fgo-extra-arg=--max-epochs \
  --fgo-extra-arg=120 \
  --fgo-extra-arg=--max-float-seed-divergence \
  --fgo-extra-arg=1 \
  --expectation-profile first-120-seed-divergence-guard \
  --skip-parity
```

For windowed MATLAB internal checks on non-default windows, regenerate a
windowed MATLAB oracle and run the window parity test. The test compares the
GPS time sequence, per-epoch fixed/float status, ambiguity candidate/fixed
counts, seed/SPP position, ratio differences, fixed/float position, velocity,
optimizer cost trace shape, and final-cost scale. The shifted C++ window uses
`--skip-epochs 400`; the MATLAB dump profile maps that to 412 fixed-interval
taroz epochs so the GPS time sequence matches C++ exactly.

```bash
python3 apps/gnss.py taroz-pos-vel-amb-pdc-dogfood \
  --generate-matlab-dump \
  --taroz-root /tmp/taroz_gtsam_gnss \
  --expectation-profile first-120-window \
  --fgo-extra-arg=--max-epochs \
  --fgo-extra-arg=120 \
  --out-dir output/dogfood/taroz_pos_vel_amb_pdc_first_120_profile_current \
  --matlab-dir output/dogfood/taroz_matlab_pos_vel_amb_pdc_first_120_debug \
  --skip-parity

python3 tests/test_taroz_pos_vel_amb_pdc_window_cost_parity.py -v

python3 apps/gnss.py taroz-pos-vel-amb-pdc-dogfood \
  --generate-matlab-dump \
  --taroz-root /tmp/taroz_gtsam_gnss \
  --expectation-profile shifted-120-window \
  --fgo-extra-arg=--skip-epochs \
  --fgo-extra-arg=400 \
  --fgo-extra-arg=--max-epochs \
  --fgo-extra-arg=120 \
  --out-dir output/dogfood/taroz_pos_vel_amb_pdc_shifted_120_current \
  --matlab-dir output/dogfood/taroz_matlab_pos_vel_amb_pdc_shifted_120_debug \
  --skip-parity

GNSSPP_TAROZ_POS_VEL_AMB_PDC_WINDOW_CPP_DIR=output/dogfood/taroz_pos_vel_amb_pdc_shifted_120_current \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_WINDOW_CPP_SUMMARY=output/dogfood/taroz_pos_vel_amb_pdc_shifted_120_current/summary.json \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_WINDOW_CPP_COST_TRACE=output/dogfood/taroz_pos_vel_amb_pdc_shifted_120_current/cost_trace.csv \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_WINDOW_CPP_EPOCH_DEBUG=output/dogfood/taroz_pos_vel_amb_pdc_shifted_120_current/epoch_debug.csv \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_WINDOW_MATLAB_DIR=output/dogfood/taroz_matlab_pos_vel_amb_pdc_shifted_120_debug \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_WINDOW_MATLAB_GRAPH=output/dogfood/taroz_matlab_pos_vel_amb_pdc_shifted_120_debug/graph_detail.csv \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_WINDOW_MATLAB_COST_TRACE=output/dogfood/taroz_matlab_pos_vel_amb_pdc_shifted_120_debug/optimizer_cost_trace.csv \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_WINDOW_MATLAB_EPOCH_STATE=output/dogfood/taroz_matlab_pos_vel_amb_pdc_shifted_120_debug/per_epoch_state.csv \
python3 tests/test_taroz_pos_vel_amb_pdc_window_cost_parity.py -v
```

For public PPC-Dataset MATLAB parity, prepare a taroz-compatible MATLAB data
directory from the generated SPP seed, regenerate the MATLAB dump with that
data directory, then run the optional public-window parity test. This first
public window is intentionally looser than the taroz example window, but the
RINEX 3 reader now keeps code/carrier/Doppler/SNR fields grouped by tracking
code and the test pins GPS time sequence, FLOAT status, seed positions,
position/velocity drift, ratio scale, candidate-count drift, and cost-trace
sanity:

```bash
python3 apps/gnss.py ppc-taroz-amb-pdc-smoke \
  --dataset-root /datasets/PPC-Dataset \
  --run nagoya/run3 \
  --max-epochs 120 \
  --generate-spp-seed \
  --taroz-matlab-data-dir output/dogfood/ppc_nagoya_run3_taroz_matlab_data_current \
  --summary-json output/dogfood/ppc_taroz_amb_pdc_nagoya_run3_first_120_seed_current/summary.json

GNSSPP_TAROZ_ROOT=/path/to/gtsam_gnss \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_EXAMPLE_DIR=/path/to/gtsam_gnss/examples \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_DATA_DIR=output/dogfood/ppc_nagoya_run3_taroz_matlab_data_current \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_OUT_DIR=output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_nagoya_run3_first_120_debug \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_MAX_EPOCHS=120 \
matlab -batch "run('scripts/dump_taroz_pos_vel_ambiguity_pdc_debug.m')"

python3 apps/gnss.py ppc-taroz-amb-pdc-smoke \
  --dataset-root /datasets/PPC-Dataset \
  --run nagoya/run3 \
  --skip-epochs 400 \
  --max-epochs 120 \
  --generate-spp-seed \
  --taroz-matlab-data-dir output/dogfood/ppc_nagoya_run3_shifted_120_taroz_matlab_data_current \
  --summary-json output/dogfood/ppc_taroz_amb_pdc_nagoya_run3_shifted_120_seed_current/summary.json

GNSSPP_TAROZ_ROOT=/path/to/gtsam_gnss \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_EXAMPLE_DIR=/path/to/gtsam_gnss/examples \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_DATA_DIR=output/dogfood/ppc_nagoya_run3_shifted_120_taroz_matlab_data_current \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_OUT_DIR=output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_nagoya_run3_shifted_120_debug \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_SKIP_EPOCHS=400 \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_MAX_EPOCHS=120 \
matlab -batch "run('scripts/dump_taroz_pos_vel_ambiguity_pdc_debug.m')"

python3 apps/gnss.py ppc-taroz-amb-pdc-smoke \
  --dataset-root /datasets/PPC-Dataset \
  --run nagoya/run3 \
  --skip-epochs 800 \
  --max-epochs 120 \
  --generate-spp-seed \
  --taroz-matlab-data-dir output/dogfood/ppc_nagoya_run3_shifted2_120_taroz_matlab_data_current \
  --summary-json output/dogfood/ppc_taroz_amb_pdc_nagoya_run3_shifted2_120_seed_current/summary.json

GNSSPP_TAROZ_ROOT=/path/to/gtsam_gnss \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_EXAMPLE_DIR=/path/to/gtsam_gnss/examples \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_DATA_DIR=output/dogfood/ppc_nagoya_run3_shifted2_120_taroz_matlab_data_current \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_OUT_DIR=output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_nagoya_run3_shifted2_120_debug \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_SKIP_EPOCHS=800 \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_MAX_EPOCHS=120 \
matlab -batch "run('scripts/dump_taroz_pos_vel_ambiguity_pdc_debug.m')"

python3 apps/gnss.py ppc-taroz-amb-pdc-smoke \
  --dataset-root /datasets/PPC-Dataset \
  --run tokyo/run1 \
  --skip-epochs 400 \
  --max-epochs 120 \
  --generate-spp-seed \
  --taroz-matlab-data-dir output/dogfood/ppc_tokyo_run1_shifted_120_taroz_matlab_data_current \
  --summary-json output/dogfood/ppc_taroz_amb_pdc_tokyo_run1_shifted_120_seed_current/summary.json

GNSSPP_TAROZ_ROOT=/path/to/gtsam_gnss \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_EXAMPLE_DIR=/path/to/gtsam_gnss/examples \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_DATA_DIR=output/dogfood/ppc_tokyo_run1_shifted_120_taroz_matlab_data_current \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_OUT_DIR=output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_tokyo_run1_shifted_120_debug \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_SKIP_EPOCHS=400 \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_MAX_EPOCHS=120 \
matlab -batch "run('scripts/dump_taroz_pos_vel_ambiguity_pdc_debug.m')"

python3 apps/gnss.py ppc-taroz-amb-pdc-smoke \
  --dataset-root /datasets/PPC-Dataset \
  --run tokyo/run2 \
  --skip-epochs 400 \
  --max-epochs 120 \
  --generate-spp-seed \
  --taroz-matlab-data-dir output/dogfood/ppc_tokyo_run2_shifted_120_taroz_matlab_data_current \
  --summary-json output/dogfood/ppc_taroz_amb_pdc_tokyo_run2_shifted_120_seed_current/summary.json

GNSSPP_TAROZ_ROOT=/path/to/gtsam_gnss \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_EXAMPLE_DIR=/path/to/gtsam_gnss/examples \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_DATA_DIR=output/dogfood/ppc_tokyo_run2_shifted_120_taroz_matlab_data_current \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_OUT_DIR=output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_tokyo_run2_shifted_120_debug \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_SKIP_EPOCHS=400 \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_MAX_EPOCHS=120 \
matlab -batch "run('scripts/dump_taroz_pos_vel_ambiguity_pdc_debug.m')"

python3 apps/gnss.py ppc-taroz-amb-pdc-smoke \
  --dataset-root /datasets/PPC-Dataset \
  --run tokyo/run3 \
  --skip-epochs 400 \
  --max-epochs 120 \
  --generate-spp-seed \
  --taroz-matlab-data-dir output/dogfood/ppc_tokyo_run3_shifted_120_taroz_matlab_data_current \
  --summary-json output/dogfood/ppc_taroz_amb_pdc_tokyo_run3_shifted_120_seed_current/summary.json

GNSSPP_TAROZ_ROOT=/path/to/gtsam_gnss \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_EXAMPLE_DIR=/path/to/gtsam_gnss/examples \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_DATA_DIR=output/dogfood/ppc_tokyo_run3_shifted_120_taroz_matlab_data_current \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_OUT_DIR=output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_tokyo_run3_shifted_120_debug \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_SKIP_EPOCHS=400 \
GNSSPP_TAROZ_POS_VEL_AMB_PDC_MAX_EPOCHS=120 \
matlab -batch "run('scripts/dump_taroz_pos_vel_ambiguity_pdc_debug.m')"

python3 tests/test_taroz_pos_vel_amb_pdc_ppc_window_parity.py -v
```

The same optional public PPC parity test consumes shifted-120 artifacts for all
three Nagoya runs and all three Tokyo runs at `--skip-epochs 400`; it also
consumes a second Nagoya run3 window at `--skip-epochs 800`:
`output/dogfood/ppc_taroz_amb_pdc_nagoya_run1_shifted_120_seed_current` and
`output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_nagoya_run1_shifted_120_debug`,
`output/dogfood/ppc_taroz_amb_pdc_nagoya_run2_shifted_120_seed_current` and
`output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_nagoya_run2_shifted_120_debug`,
`output/dogfood/ppc_taroz_amb_pdc_nagoya_run3_shifted_120_seed_current` and
`output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_nagoya_run3_shifted_120_debug`,
and `output/dogfood/ppc_taroz_amb_pdc_nagoya_run3_shifted2_120_seed_current`
with
`output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_nagoya_run3_shifted2_120_debug`,
plus `output/dogfood/ppc_taroz_amb_pdc_tokyo_run1_shifted_120_seed_current`
with
`output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_tokyo_run1_shifted_120_debug`,
and `output/dogfood/ppc_taroz_amb_pdc_tokyo_run2_shifted_120_seed_current`
with
`output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_tokyo_run2_shifted_120_debug`,
and `output/dogfood/ppc_taroz_amb_pdc_tokyo_run3_shifted_120_seed_current`
with
`output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_tokyo_run3_shifted_120_debug`.
Generate `nagoya/run1` and `nagoya/run2` with the same C++ and MATLAB
skip-400 command shape shown for `nagoya/run3`, replacing the run name and
output roots with the matching `nagoya_run1` or `nagoya_run2` paths listed
above.
For each shifted public window, generate the taroz MATLAB data directory with
the same `--skip-epochs ... --max-epochs 120` runner settings so the SPP seed
file covers the skipped epochs. The MATLAB dump aligns the 1 Hz base
observations to the 5 Hz rover timeline before slicing each shifted window, and
the parity test pins matching Nagoya skip-400 (116/120, 39/120, and 112/120
FIX), Tokyo skip-400 (108/120, 115/120, and 113/120 FIX), and lower-FIX
Nagoya skip-800 (29/120 FIX) status, seeds, candidate counts, ratio scale,
fixed/float position, velocity, LAMBDA matrix inputs, and optimizer-cost
trajectory scale.

The matching CTest parity tests are optional-artifact tests: they skip cleanly
when the local `output/dogfood/...` oracle files are absent and become strict
regressions after a dogfood run has generated them. The window test pins
per-epoch fixed status, seed/SPP position, ambiguity counts, ratio, fixed/float
position, velocity, and cost trace shape against the MATLAB dump.
For position/velocity ambiguity PDC, the parity gate also checks that the C++
LAMBDA debug stream forms a complete square candidate matrix per attempted
epoch, with stable satellite row/column mapping, symmetric covariance, and
epoch-debug status/ratio consistency. Its seed-state check ties
`seed_matched_epochs` and the 34 interpolated seed epochs to the MATLAB SPP
per-epoch dump, rather than only checking final `.pos` rows.

```bash
ctest --test-dir build-codex -R 'python_taroz_.*(internal_parity|factor_parity|window_cost_parity|ppc_window_parity)_tests' --output-on-failure
```

The remaining beta-hardening work is broader than final-output shape. Current
parity covers the listed modes, default ambiguity PDC internals, LAMBDA matrix
contracts, seed interpolation counts, first/shifted window option contracts,
and first/shifted window per-epoch plus cost-trajectory contracts. It is still
not a complete bit-for-bit port of every
taroz MATLAB branch: longer PPC-Dataset windows beyond the current public
first/shifted slices, additional MATLAB internal dumps, and broader option
combinations are required before calling the taroz port complete.

The current taroz parity gap table is:

| Surface | Current status | Remaining before complete-port claim |
|---|---|---|
| Example modes in taroz `examples/*.m` | P, D, PD, position PD/PDC, position/velocity PDC, PC, and position/velocity ambiguity PDC have C++ dogfood entrypoints and artifact contracts | Paper examples and standalone MATLAB helper/demo scripts are not claimed as ported workflows |
| PPC `amb-pdc` default run | C++ summary, final `.pos`, DD/SD factor debug, LAMBDA debug, seed counts, and cost trace are pinned against generated MATLAB dumps | Broader per-factor numeric parity across more public PPC runs remains local-only |
| Windowed ambiguity PDC | First 120 and shifted 120 taroz-example windows compare status, position, velocity, ambiguity counts, ratios, seeds, and cost shape against MATLAB; public `nagoya/run3` first 120 pins status, seeds, position, velocity, ratio scale, candidate-count drift, and cost sanity; public shifted 120 windows cover all six Nagoya/Tokyo public runs at skip 400 plus `nagoya/run3` skip 800, pinning status, seeds, candidate counts, ratio, fixed/float position, velocity, LAMBDA matrix inputs, and cost trajectory scale after base-time alignment | Add longer/full windows and tighten public-run factor parity |
| Public PPC-Dataset dogfood | Six-run 200 epoch generated-seed local gate plus `nagoya/run2`, `nagoya/run3`, and `tokyo/run2` 1000 epoch optional artifact tests; long summaries include tail counts and worst epochs | Full-window PPC runs and more strict tail thresholds are still research gates, not CI defaults |
| Seed handling | Generated SPP seed path is part of the PPC runner; generated-seed summaries require exact seed match count, zero interpolation, seed-row coverage, and shifted-window seed/epoch sequence alignment | Seed-gap interpolation and intentionally shifted external seed files need more dedicated PPC fixtures |
| Option profiles | Ratio, epoch lambda output, seed interpolation, shifted/first windows, first/shifted-window strict ratio, and seed divergence guard have expectation profiles | More cross-products of ratio, guards, skip/max, and interpolation should be added only when their counts are stable |
| Solver trajectory | C++ cost trace shape is always checked; default, taroz-example windows, and public shifted PPC windows compare MATLAB/C++ cost scale where dumps exist | More windows/runs should pin MATLAB/C++ iteration and cost trajectory behavior |

When `--generate-spp-seed` is used, the PPC harness treats the generated seed as
part of the sign-off. The native summary must report a seed path,
`seed_matched_epochs` must equal the optimized epoch count, and
`seed_interpolated_epochs` must remain zero. The generated-seed audit also
checks the seed row count, missing optimized epochs, duplicate/non-sorted epoch
keys, and, for shifted windows, that the seed sequence starting at
`skip_epochs` exactly matches the optimized epoch sequence.

`gnss smartloc-adapter` widens the public matrix beyond UrbanNav by exporting
smartLoc `NAV-POSLLH.csv` into a `reference.csv` plus a normalized u-blox
receiver CSV, and by exporting `RXM-RAWX.csv` into a normalized raw observation
CSV plus minimal RINEX 3.04 rover observations. `gnss smartloc-signoff` wraps
that adapter and gates the receiver-fix metrics against the smartLoc ground
truth. When local inputs are omitted, it can download the public scenario zip
through `--input-url` into `--download-cache-dir` and records that provenance in
the summary JSON. That closes the public receiver-fix path, but not the whole
solver sign-off: smartLoc solver runs still need compatible broadcast
navigation and base/reference inputs. The summary includes `solver_preflight`
so CI can distinguish generated rover RINEX, bundled precise-orbit artifacts,
missing broadcast nav, missing base observations, and missing precise clocks.
Add `--require-solver-inputs-available` to fail the sign-off when the RTK solver
input set is expected to be complete.

If you already have a MALIB `.pos` file, you can gate the delta directly:

```bash
python3 apps/gnss.py ppp-products-signoff \
  --profile static \
  --obs data/rover_static.obs \
  --nav data/navigation_static.nav \
  --malib-pos output/malib_ppp_static_solution.pos \
  --use-existing-malib \
  --require-lib-mean-error-vs-malib-max-delta 0.25 \
  --require-lib-max-error-vs-malib-max-delta 0.50
```

## Realtime validation

`gnss live-signoff` verifies:

- termination mode,
- aligned epochs,
- written and fixed solutions,
- decoder errors,
- solver wall time,
- realtime factor,
- effective epoch rate.

`gnss moving-base-signoff` verifies real moving-base datasets with a reference CSV carrying per-epoch base/rover ECEF coordinates. It can gate:

- valid and matched epochs,
- fix rate,
- baseline error percentiles,
- heading error percentiles,
- solver wall time / realtime factor / effective epoch rate,
- `live` termination and decoder errors.

Use `gnss moving-base-prepare` first when the source dataset is a ROS2 bag or Zenodo zip carrying u-blox `NAV-PVT`, `RXM-RAWX`, and `NAV-RELPOSNED` topics. For replay mode, pair the exported `rover.ubx` / `base.ubx` files with a fetched BRDC navigation file from `gnss fetch-products --preset brdc-nav`. Add `--commercial-csv` to export the rover receiver's `NAV-PVT` solution as a normalized commercial receiver CSV.

For commercial receiver side-by-side evaluation, add `--commercial-pos` with a normalized receiver
solution CSV or `.pos` file. The commercial solution is matched to the same reference CSV and stored
under `commercial_receiver` in the summary JSON. Treat RTKLIB/demo5 as a public reproducible
baseline; moving-RTK quality claims should use independent reference data plus commercial receiver
output when available.

For the public SCORPION dataset, `gnss scorpion-moving-base-signoff` wraps the same flow into one command and defaults to the public Zenodo zip when no `--input` is supplied. It emits the same summary JSON fields plus prepare/fetch provenance, libgnss matched CSV, commercial receiver CSV, commercial receiver matched CSV, and a plot preview that `gnss web` can render directly. The SCORPION receiver CSV comes from rover `NAV-PVT`; it is a public receiver side-by-side baseline, not an independent survey truth source.

## Test layers

The repo validates functionality through multiple layers:

- C++ unit and solver tests
- CLI regressions
- benchmark/image-generation tests
- packaging and installed-prefix smoke tests
- Python bindings tests
- ROS2 playback tests
- browser smoke tests for `gnss web`

Main command:

```bash
ctest --test-dir build --output-on-failure
```

## Dogfooding

Installed-prefix smoke tests verify that:

- installed `gnss` commands run correctly,
- sign-off scripts behave after `cmake --install`,
- generated assets still render correctly outside the source tree.
