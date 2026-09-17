# Smartphone base-surveyed correction: dev-route results (v1)

The GSDC base RINEX header position differs from the surveyed base position
by up to ~2 m, which leaves a base-side bias in the pseudorange
compensation. `--native-base-position-ecef X Y Z` (merged in #501) replaces
the header position with the surveyed value.

Horizontal accuracy (P50+P95)/2 m, exact `(phone, UnixTimeMillis)` join,
spherical Haversine R=6371008.8. Dev routes, Pixel5:

| route | date | base | no-base | base-surveyed |
| --- | --- | --- | ---: | ---: |
| H | 2021-08-24 MTV | P221 | 1.0769 | **0.57738** |
| U | 2023-03-08 MTV | P221 | 1.3060 | **0.73751** |
| A | 2021-03-16 MTV | SLAC | 1.3613 | **0.30169** |
| LAX-T | 2022-04-01 LAX | LBCH | 3.1026 | **0.71207** |

H and U reproduce the #501 result; **A is a new, large improvement**
(1.36 -> 0.30 m). The per-route base station must be matched: A uses SLAC,
not P221 (using P221 gives a ~7 km failure).

Surveyed base ECEF used:

- P221 2021: `-2698117.9416 -4301326.2649 3847286.2750`
- P221 2023: `-2698117.9861 -4301326.2071 3847286.2977`
- SLAC 2021: `-2703116.3177 -4291766.7551 3854248.0736`
- LBCH 2022: `-2507799.2243 -4676369.3031 3526891.0358`

## Reproduce (H)

```bash
export LD_LIBRARY_PATH=$HOME/.local/lib:${LD_LIBRARY_PATH:-}
P=output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2021-08-24-20-32-us-ca-mtv-h/pixel5/inputs
B=output/smartphone-r5/phase63-settings-integrity-recovery-v1/routes/2021-08-24-20-32-us-ca-mtv-h__pixel5/base.obs
build/apps/gnss_fgo_imu_no_base \
  --dataset-id 2021-08-24-20-32-us-ca-mtv-h/pixel5 \
  --android-gnss $P/device_gnss.csv --android-imu $P/device_imu.csv --nav $P/brdc.nav \
  --all-epochs --android-raw-utc-keys --android-raw-clock-only \
  --android-utc-wall-clock-fallback --native-pdc-imu-tdcp-no-bridge \
  --native-source-direct-observable-quality --native-source-clock-c0d-factor \
  --native-source-clock-c0d-meter-state-parity \
  --native-source-clock-c0d-active-solve-diagnostic \
  --native-source-clock-c0d-gnss-first-meter-state-handoff \
  --native-source-clock-c0d-epoch-vector-parity \
  --native-source-clock-c0d-phase99-main-multifrontal-qr-solver \
  --native-phase157-raw-p-bootstrap --native-phase165-raw-p-no-doppler-graph \
  --native-phase167-raw-p-no-doppler-lm-termination-budget \
  --native-phase171-raw-p-no-doppler-imu-main \
  --native-phase171-raw-p-ecef-doppler-gnss-first \
  --native-phase197-source-utc-fallback-imu-offset --native-phase217-main-pose3-motion \
  --native-upstream-stop-constraints --native-phase213-main-doppler \
  --native-source-tdcp-meter-sigma --native-lm-lambda-floor \
  --native-base-pseudorange-compensation --native-base-pseudorange-source-miss-mask \
  --native-base-rinex $B \
  --native-base-rinex-sha256 4d3e37cbe0347fa56216db54ede9e0f30731885f337f1653ab5a86afb2bb2150 \
  --native-base-position-ecef -2698117.9416 -4301326.2649 3847286.2750 \
  --out out.csv --summary-json out.json
```

A uses the same flag set with the accepted A-transfer inputs
(`phase25-raw-clock-eval-v1/raw/...`), the A base (`380b8ff9...`) and
`--native-base-position-ecef -2703116.3177 -4291766.7551 3854248.0736`.
U uses the phase37 U inputs, the U base (`aedb7a39...`) and
`--native-base-position-ecef -2698117.9861 -4301326.2071 3847286.2977`.

## LAX-T

LAX-T needs `--native-sparse-p-staging` to initialise and the
`--native-joint-ionosphere 3 0.02 1.5` recipe (best no-base 2.9065 m,
phase538). The app originally forbade combining sparse-p staging with the
base compensation; that exclusion is dropped in
`feat/smartphone-lax-sparse-staging-base`, so LAX-T now runs base-surveyed
and scores **0.71207 m**.

Command additions on top of the LAX-T phase538 recipe:

```bash
--native-base-pseudorange-compensation --native-base-pseudorange-source-miss-mask \
--native-base-rinex <lax base.obs> \
--native-base-rinex-sha256 d731e0e8a7ba4396d62340c85b6238e66c50c6a349621f9dcea0ea6885fd4cfe \
--native-base-position-ecef -2507799.2243 -4676369.3031 3526891.0358
```

