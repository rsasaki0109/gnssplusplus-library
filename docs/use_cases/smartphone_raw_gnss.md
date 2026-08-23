# Smartphone raw GNSS (R5)

Status: adapter and native GPS L1 standalone lane implemented; the full
development route passes its frozen profile. The holdout remains sealed until
the implementation and thresholds are committed. Results from the small public
parser fixture remain wiring evidence only, not an accuracy claim.

## Application boundary

This workflow targets Google Smartphone Decimeter Challenge (GSDC) 2023
`device_gnss.csv` and `ground_truth.csv` pairs. It preserves every source
column and value, inventories handset clock discontinuities and signal
availability, exports the device WLS baseline, and scores it against the
independent reference. Phone-grade antenna, clock, duty-cycle, and optional
measurement fields are not treated as equivalent to a survey receiver.

The original GSDC data is subject to the competition terms. A public
preprocessed archive is available from the open-source
[`taroz/gsdc2023`](https://github.com/taroz/gsdc2023) reproduction project,
but local availability does not grant permission to redistribute the dataset.
Keep downloaded data under ignored `data/` or `output/` directories and record
the exact source terms in every run.

## Adapter smoke

Run a bounded development slice after materialising one phone directory:

```bash
python3 apps/gnss.py smartphone-gnss-adapter \
  --device-gnss data/gsdc2023/train/<route>/<phone>/device_gnss.csv \
  --ground-truth data/gsdc2023/train/<route>/<phone>/ground_truth.csv \
  --output-dir output/smartphone-r5/smoke \
  --dataset-id <route>/<phone> \
  --device-model <phone> \
  --source-url https://www.kaggle.com/competitions/smartphone-decimeter-2023 \
  --source-terms "GSDC competition terms; local evaluation only" \
  --role development \
  --skip-epochs 1 \
  --max-epochs 600
```

The command fails on malformed or backwards timestamps, inconsistent or
backwards hardware-clock discontinuity counts, malformed WLS ECEF positions,
and missing truth. The frozen development route has one receiver epoch before
truth begins, so the command excludes it explicitly with `--skip-epochs 1`
and records that choice in the summary. Rows without a derived `SignalType` are preserved in
`observations.csv`, counted as `<unmapped>`, and explicitly excluded from any
future solver input instead of being silently assigned a signal code.

Artifacts:

- `observations.csv`: all original fields and selected raw rows;
- `receiver_wls.csv`: the handset-provided WLS ECEF result converted to WGS84;
- `reference.csv`: timestamp-aligned independent reference positions;
- `summary.json`: source hashes, role, device, clock transitions, signal and
  constellation populations, truth coverage, and baseline error metrics.
- `rover.obs`: GPS L1 C/A mapped to RINEX 3.04 using GPST receiver arrival
  time and the device WLS ECEF position only as the solver initial value.

`device_wls_score.accuracy_claim` is deliberately `baseline-only`. Run the
native standalone and frozen sign-off lanes with:

```bash
python3 apps/gnss.py spp \
  --obs output/smartphone-r5/development-full/rover.obs \
  --nav data/gsdc2023/materialized/dataset_2023/train/2023-05-24-20-26-us-ca-sjc-ge2/brdc.nav \
  --out output/smartphone-r5/development-full/libgnsspp_spp.pos \
  --summary-json output/smartphone-r5/development-full/libgnsspp_spp_summary.json

python3 apps/gnss.py smartphone-gnss-signoff \
  --position output/smartphone-r5/development-full/libgnsspp_spp.pos \
  --ground-truth data/gsdc2023/materialized/dataset_2023/train/2023-05-24-20-26-us-ca-sjc-ge2/pixel7pro/ground_truth.csv \
  --adapter-summary output/smartphone-r5/development-full/summary.json \
  --solver-summary output/smartphone-r5/development-full/libgnsspp_spp_summary.json \
  --output-dir output/smartphone-r5/development-full \
  --profile configs/benchmarks/smartphone_r5_gsdc2023.json
```

The profile prevents command-line threshold overrides and verifies the adapter
dataset role, ID, and source-file hashes before scoring.

## Frozen development and holdout contract

| Role | Dataset | Device/raw SHA-256 | Truth SHA-256 |
|---|---|---|---|
| Development | `2023-05-24-20-26-us-ca-sjc-ge2/pixel7pro` | `c5af6cc3...ff10e42` | `d32d0108...981638a` |
| Holdout | `2023-09-06-22-49-us-ca-routebb1/pixel7pro` | `f6c05015...aba245b` | `abb40762...58879a7` |

The archive SHA-256 is
`bda30ab456e6fd6f83550c246e8dbd287306d5385f1f1069c99c16298e647408`.
The complete hashes and exact signal/time mapping are versioned in
`configs/benchmarks/smartphone_r5_gsdc2023.json`.

The 1,383-epoch development run produced 1,376 truth-matched solutions:
99.49% availability, 4.93 m horizontal median, 16.35 m horizontal p95,
28.38 m absolute vertical p95, and an 8 s maximum solution gap. The frozen
standalone thresholds are respectively 98%, 7 m, 25 m, 45 m, and 10 s. These
are phone-grade standalone bounds, not decimetre or survey claims.

## Release sequence

1. Freeze a development phone/route and an untouched truth-bearing holdout.
2. Publish archive and extracted-file hashes plus the clean-room command.
3. Run the 600-epoch adapter smoke and inspect excluded signal populations.
4. Map only supported signals into the native observation interface. GPS L1
   standalone is complete; other signals remain explicitly unsupported.
5. Produce and truth-score standalone and precise-product POS outputs. The
   standalone lane is complete; a precise-product lane remains pending.
6. Freeze thresholds and invocation before opening the holdout.
7. Run the holdout once and publish Go/No-Go without post-failure tuning.

Reject the workflow when truth is absent, source terms are unknown, clock
state cannot be interpreted, supported observations are empty, or the selected
route cannot meet its frozen coverage and error gates.
