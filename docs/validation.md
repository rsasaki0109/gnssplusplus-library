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
- `gnss odaiba-benchmark --require-*`

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

For PPC-Dataset RTK runs, `gnss ppc-demo` and `gnss ppc-rtk-signoff` also accept `--commercial-pos`. This is the public-data path for comparing libgnss++ against a commercial receiver solution when the dataset has an independent `reference.csv`; the receiver summary is stored under `commercial_receiver`, and `delta_vs_commercial_receiver` reports libgnss++ minus receiver deltas for fix rate and horizontal/up error metrics.

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
