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
Use `--nonfix-drift-max-residual 4` only for an explicit P95-cleanup profile:
combined with the fixed-burst guard it rejects 337 non-FIX drift epochs on
Tokyo run1, improves P95H from 34.53 m to 26.61 m, and keeps PPC official nearly
flat, but costs 2.40 pp Positioning rate. `ppc-coverage-matrix` accepts these
non-FIX, SPP height-step, FLOAT bridge-tail, and fixed-burst tuning flags so the
same profile can be swept across all six PPC runs. The full six-run sweep keeps
a +13.3 pp average Positioning lead over RTKLIB and a +28.1 pp PPC official
lead, but costs 3.67 pp average Positioning versus the coverage profile and
only improves P95H on 3/6 runs; Nagoya run3 loses 13.90 pp Positioning with no
useful P95 gain. Treat the profile as a tail-diagnostic lens, not as the
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
