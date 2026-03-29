# Validation

## Sign-off commands

Sign-off commands are quality gates around real solver commands. They emit
summary JSON and fail when thresholds are violated.

Main sign-off entrypoints:

- `gnss short-baseline-signoff`
- `gnss rtk-kinematic-signoff`
- `gnss ppp-static-signoff`
- `gnss ppp-kinematic-signoff`
- `gnss live-signoff`
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
