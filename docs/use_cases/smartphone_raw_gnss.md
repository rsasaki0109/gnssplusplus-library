# Smartphone raw GNSS (R5)

Status: adapter and native GPS L1 standalone lane implemented. The full
development route and untouched holdout run1 both pass the frozen profile.
Results from the small public parser fixture remain wiring evidence only, not
an accuracy claim.

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

The adapter reads `device_gnss.csv` one epoch at a time and publishes the five
artifacts only after all validation and truth checks pass. This keeps the
lossless selected-row output, source hashes, and fail-closed behavior unchanged
without retaining the full device CSV in memory; the small truth timestamp
index is retained for alignment and scoring.

## Kaggle token setup

Configure the official Kaggle access token locally without placing it in shell
history or process arguments:

```bash
scripts/configure_kaggle_token.sh
scripts/configure_kaggle_token.sh --check
```

For a non-interactive job, stdin must be an explicit opt-in and the token must
come from an already protected environment—not a literal command-line value:

```bash
printf '%s\n' "$KAGGLE_API_TOKEN" | scripts/configure_kaggle_token.sh --stdin
```

The helper writes `${KAGGLE_CONFIG_DIR:-$HOME/.kaggle}/access_token` with
directory/file modes `700/600`, rejects whitespace and extra lines, and does
not make a network request. Use `--force` only when deliberately replacing an
existing token. See `scripts/README.md` for the overwrite and fixture checks.

## Adapter smoke

The preferred clean-room command verifies the 2.76 GB archive hash, extracts
only the frozen inputs, runs the adapter, native solver, sign-off, KML and PNG
steps, and hashes the resulting bundle:

```bash
python3 apps/gnss.py smartphone-gnss-workflow \
  --archive data/gsdc2023/cache/dataset_2023.zip \
  --role development \
  --output-dir output/smartphone-r5/clean-room-smoke \
  --max-epochs 600
```

For a reproducible Release performance run, add `--performance`. The native
SPP binary then emits one timing row per selected epoch and the workflow adds
GPST-anchored interval p50/p95, effective epoch rate, realtime factor, valid
rate, and stage wall-time measurements to its manifest:

```bash
python3 apps/gnss.py smartphone-gnss-workflow \
  --archive data/gsdc2023/cache/dataset_2023.zip \
  --role development \
  --output-dir output/smartphone-r5/development-performance \
  --max-epochs 600 \
  --performance --performance-interval-s 60
```

The generic `performance-baseline` command also records the Release CMake
configuration, source revision/status, solver binary hash, input hashes, and
exact command for SPP or RTK. Its interval report measures only native
`processEpoch`/`processRTKEpoch` calls; workflow stage timings remain separate
so archive extraction, adapter, sign-off, and visualization costs are not
mistaken for solver cost. The repository does not contain Kaggle's hidden-test
service or private leaderboard oracle. The local evaluator described below is
therefore explicitly named `public-spec-compatible-distance-undetermined` and
never labels a local diagnostic value as an official score.

After a truth-scored development run, the quality report joins the existing
sign-off `matches.csv` to the corresponding raw device epochs and emits fixed
60-second segment and quality-bucket CSVs. C/N0, raw pseudorange uncertainty,
received-SV-time uncertainty, ADR validity/state, clock discontinuity count,
raw epoch gaps, and unsupported-signal fraction are assigned without using
truth errors; errors are attached only for post-hoc analysis:

```bash
python3 apps/gnss.py smartphone-quality-report \
  --device-gnss data/gsdc2023/materialized/dataset_2023/train/2023-05-24-20-26-us-ca-sjc-ge2/pixel7pro/device_gnss.csv \
  --adapter-summary output/smartphone-r5/development-full/summary.json \
  --signoff-summary output/smartphone-r5/development-full/signoff_summary.json \
  --matches output/smartphone-r5/development-full/matches.csv \
  --output-json output/smartphone-r5/development-full/quality_report.json \
  --output-segments-csv output/smartphone-r5/development-full/quality_segments.csv \
  --output-buckets-csv output/smartphone-r5/development-full/quality_buckets.csv
```

The optional `--candidate-signoff-summary` records a predeclared solver
configuration comparison. The current SPP already receives RINEX `S1C` C/N0,
so an SNR variance-model option can be evaluated without adding a new raw-data
path. A candidate is promoted only when all frozen sign-off metrics do not
regress and at least one improves; holdout is not used for tuning.

On the frozen development route, the first fixed candidate (`--snr-reference-dbhz
50`) was not promoted: horizontal median/P95 changed from 4.928/16.353 m to
4.991/17.603 m and vertical P95 from 28.376 m to 29.069 m, with availability
unchanged at 99.49%. No holdout run was spent on this rejected candidate.

## GSDC 2023 public metric and submission contract

The local archive README and frozen profile identify the 2023 target as the
Kaggle [Google Smartphone Decimeter Challenge 2023-2024](https://www.kaggle.com/competitions/smartphone-decimeter-2023).
The public page describes a per-phone horizontal-distance P50/P95 score.  The
authenticated official sample retrieved in phase 29 is authoritative for the
submission schema and uses
`tripId,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees`; the older local
truth-free artifacts and evaluator use `phone` internally and therefore are
not interchangeable without the strict promotion check below.

Generate a submission from a solver POS file without reading truth:

```bash
python3 apps/gnss.py smartphone-kaggle-submit \
  --position output/smartphone-r5/development-full/libgnsspp_spp.pos \
  --device-gnss data/gsdc2023/materialized/dataset_2023/train/2023-05-24-20-26-us-ca-sjc-ge2/pixel7pro/device_gnss.csv \
  --profile configs/benchmarks/smartphone_r5_gsdc2023.json \
  --role development \
  --dataset-id 2023-05-24-20-26-us-ca-sjc-ge2/pixel7pro \
  --phone 2023-05-24-20-26-us-ca-sjc-ge2_pixel7pro \
  --output output/smartphone-r5/development-full/kaggle_submission.csv
```

This publishes the CSV and an atomic `.manifest.json` sidecar. The manifest
hashes the position/device/profile inputs and the submission, records the
truth-free contract, and never includes a ground-truth path or hash. GPST
week/TOW is converted with exact Decimal arithmetic and floors to integer
milliseconds; device epoch keys are checked exactly when `--device-gnss` is
provided. Duplicate, non-finite, out-of-range, malformed, or extra CSV fields
fail closed.

Evaluate against a local truth file as a reproducible public-spec audit:

```bash
python3 apps/gnss.py smartphone-kaggle-evaluate \
  --submission output/smartphone-r5/development-full/kaggle_submission.csv \
  --submission-manifest output/smartphone-r5/development-full/kaggle_submission.csv.manifest.json \
  --ground-truth data/gsdc2023/materialized/dataset_2023/train/2023-05-24-20-26-us-ca-sjc-ge2/pixel7pro/ground_truth.csv \
  --profile configs/benchmarks/smartphone_r5_gsdc2023.json \
  --role development \
  --dataset-id 2023-05-24-20-26-us-ca-sjc-ge2/pixel7pro \
  --phone 2023-05-24-20-26-us-ca-sjc-ge2_pixel7pro \
  --allow-gaps \
  --output-json output/smartphone-r5/development-full/kaggle_metric_report.json
```

The evaluator rejects extra keys and, by default, missing
`(phone,UnixTimeMillis)` keys; `--allow-gaps` is an explicit sparse-prediction
override for the public page's documented gap allowance and reports coverage.
The official [2023 Evaluation page](https://www.kaggle.com/competitions/smartphone-decimeter-2023)
defines only the horizontal-distance P50/P95 aggregation. It does not publish
whether the distance is Vincenty/WGS84 or Haversine/spherical, the spherical
Earth radius (if any), or the percentile interpolation. Consequently the
report's `primary_score_m` is always `null` with status
`undetermined-from-public-primary-sources`. It emits four explicitly local
diagnostics (two distance models crossed with two percentile conventions):

The audit used only the official Kaggle Evaluation/Submission File material
and official Google/Kaggle artifacts. No official public evaluator
implementation was found that resolves the omitted details; participant
notebooks or third-party repositories are intentionally not treated as primary
evidence.

- `wgs84_vincenty__linear_n_minus_1`: WGS84 ellipsoid, Vincenty inverse, and
  linear interpolation at `(n - 1) * q`;
- `haversine_sphere__linear_n_minus_1`: spherical Haversine with the declared
  local radius `6371008.8 m`, and the same interpolation;
- `wgs84_vincenty__nearest_rank_ceiling`: WGS84/Vincenty with
  `max(1, ceil(n * q))` nearest rank;
- `haversine_sphere__nearest_rank_ceiling`: spherical/Haversine with
  `6371008.8 m` and the nearest-rank convention.

These values are sensitivity diagnostics, not a reproduction of Kaggle's
private evaluator. The JSON records the source URL, all assumptions, and the
absence of public confirmation for each formula. The truth-free submission
manifest contract and exact key validation remain unchanged apart from the
audited schema version.

For adapter-only diagnosis, run a bounded development slice after
materialising one phone directory:

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

The untouched holdout was opened only after commit `98b15f4` froze the route,
hashes, mapping, invocation, and thresholds. Holdout run1 produced 1,596
truth-matched solutions from 1,601 selected epochs: 99.69% availability,
4.13 m horizontal median, 14.91 m horizontal p95, 29.89 m absolute vertical
p95, and a 4 s maximum gap. All six frozen gates passed. The immutable result
is recorded in `docs/use_cases/records/smartphone_r5_holdout_run1.json`.

The development-only precise-product variant applied CODE final SP3/CLK to
8,125 measurements, but horizontal median error regressed from 4.93 m to
11.82 m. It is recorded as `evaluated-not-promoted` in
`docs/use_cases/records/smartphone_r5_precise_development.json`; no holdout run
was spent on that rejected variant. Standalone GPS L1 remains the promoted R5
lane.

## Galileo E1 development candidate (phase 4)

The raw Pixel development route was audited before enabling another signal.
Galileo E1 (`SignalType=GAL_E1_C_P`, `ConstellationType=6`,
`CarrierFrequencyHz=1575420000`, empty `CodeType`) is the safest candidate:
libgnss++ already has Galileo E1 as the primary Galileo signal, fixed-frequency
RINEX `E` observation mapping, Galileo broadcast-navigation parsing, a
separate Galileo clock-bias group, and the existing SPP CN0/elevation variance
model. GLONASS G1 was evaluated separately in phase 6; its per-satellite
FDMA/`GLONASS SLOT / FRQ #` contract was auditable, but the candidate did not
meet the accuracy promotion gate. GPS L5 was
not selected because it is a secondary signal in the current SPP policy and
would require a different single-frequency/dual-frequency policy.

The candidate accepts only the exact Galileo source contract above and maps it
to `E C1C/L1C/D1C/S1C`; non-empty or unknown `CodeType`, constellation, or
frequency values fail closed. A mixed RINEX navigation file is mandatory, and
every selected Galileo E1 observation must have a native Galileo broadcast
record within four hours. The native SPP summary now records
`config.model_intersystem_bias=true`; Galileo and GPS use separate clock-bias
groups, while the existing CN0/elevation variance weighting is unchanged.
The opt-in is development-only:

```bash
python3 apps/gnss.py smartphone-gnss-workflow \
  --archive data/gsdc2023/cache/dataset_2023.zip \
  --role development \
  --output-dir output/smartphone-r5/development-galileo-e1 \
  --experimental-galileo-e1 --performance
```

The default GPS-only adapter output remained byte-identical to the frozen
development `rover.obs`. On the same Pixel development route, the measured
comparison was:

| Slice | Lane | RINEX rows (Galileo E1) | Solutions / availability | H median / P95 (m) | V P95 (m) | Adapter / solver wall (s) | Adapter / solver peak RSS (KB) |
|---|---|---:|---:|---:|---:|---:|---:|
| 600 epochs | GPS L1 baseline | 4,474 (0) | 593 / 98.83% | 5.622 / 18.081 | 33.585 | 0.70 / 0.31 | 23,136 / 14,688 |
| 600 epochs | GPS + Galileo E1 | 8,111 (3,637) | 600 / 100.00% | 3.376 / 8.921 | 25.530 | 1.04 / 0.67 | 24,040 / 15,592 |
| Full 1,383 epochs | GPS L1 baseline | 11,000 (0) | 1,376 / 99.49% | 4.928 / 16.353 | 28.376 | 1.11 / 0.53 | 23,348 / 17,140 |
| Full 1,383 epochs | GPS + Galileo E1 | 19,743 (8,743) | 1,383 / 100.00% | 3.407 / 8.428 | 19.502 | 1.89 / 1.24 | 24,800 / 19,760 |

The four local public-spec diagnostics (official distance formula and
percentile convention remain unpublished) changed as follows for the full
route: Haversine/linear `10.629 -> 5.911 m`, Haversine/nearest-rank
`10.644 -> 5.912 m`, WGS84/Vincenty/linear `10.626 -> 5.921 m`, and
WGS84/Vincenty/nearest-rank `10.638 -> 5.922 m`. The candidate passed all
frozen accuracy/availability/gap gates and is therefore promoted as an
explicit development-only lane. It costs wall time and a small amount of
peak RSS because it parses the mixed navigation contract and processes 8,743
additional observations; this is recorded as a follow-up optimization target.
No holdout data was run or used for this decision. The full machine-readable
record is in `docs/use_cases/records/smartphone_r5_galileo_e1_development.json`;
the release profile records the decision under
`galileo_e1_development_lane`.

## Galileo E1 Hatch code smoothing (phase 7, promoted development-only)

The promoted Galileo E1 lane was used to test a truth-free Hatch candidate on
`RINEX E C1C`. The adapter audit confirmed that Android
`AccumulatedDeltaRangeMeters` is a range-scale value in metres and that the
existing `L1C` conversion is metres divided by the E1 wavelength. The candidate
therefore updates only the code field with
`S_t=((N-1)*(S_(t-1)+ADR_t-ADR_(t-1))+P_t)/N`; carrier phase, Doppler, C/N0,
GPS observations, and all source rows remain unchanged.

The state key is `(RINEX system, PRN, SignalType)`. A row with ADR valid bit
`1` starts or updates an arc. ADR reset bit `2`, cycle-slip bit `4`, an invalid
state, missing/out-of-range ADR, a hardware-clock discontinuity, or a gap over
1,500 ms clears the arc and emits that row's raw pseudorange. The following
valid row starts a new arc. The development route is one hertz, so the fixed
candidate windows are 10, 30, and 60 samples/seconds. No truth is read by the
adapter or smoother.

The 1,383 selected epochs were split chronologically into 830 train and 553
validation epochs. Separate adapter/solver processes enforced a reset at the
boundary. The window was selected using train H median, then H P95, V P95, and
the four local public-spec diagnostics lexicographically, with availability
non-regression required. The 30-second window won; validation then passed with
all three sign-off accuracy values and all four diagnostics lower than the
baseline, equal 100% availability, and no holdout access:

| Split / lane | Solutions / availability | H median / P95 (m) | V P95 (m) | Haversine-linear / WGS84-linear (m) |
|---|---:|---:|---:|---:|
| Train baseline | 823 / 99.16% | 5.467 / 18.910 | 32.418 | 12.175 / 12.183 |
| Train Hatch 10 s | 830 / 100.00% | 3.124 / 7.878 | 22.717 | 5.495 / 5.501 |
| Train Hatch 30 s (selected) | 830 / 100.00% | 3.060 / 7.799 | 22.356 | 5.423 / 5.421 |
| Train Hatch 60 s | 830 / 100.00% | 3.080 / 7.864 | 22.499 | 5.466 / 5.465 |
| Validation baseline | 553 / 100.00% | 4.349 / 11.723 | 21.392 | 8.027 / 8.040 |
| Validation Hatch 30 s | 553 / 100.00% | 3.447 / 8.237 | 16.661 | 5.835 / 5.847 |

On the full development route, Hatch 30 s retained 1,383/1,383 solutions and
improved H median/P95 from `3.407/8.428 m` to `3.215/7.957 m`, V P95 from
`19.502 m` to `20.192 m` (the latter is a small regression, so this lane is
not a strict all-major-metric full-route promotion claim; the promotion gate
was the frozen train/validation gate). The four diagnostics changed from
`5.910908/5.911589/5.921347/5.921857 m` to
`5.579926/5.580951/5.578909/5.579290 m`. Full-route adapter/solver wall time
was `2.13/1.52 s` for the E1 baseline and `2.16/1.58 s` with Hatch; peak RSS
was `25,120/19,812 KB` and `25,136/19,744 KB`, respectively. Native SPP epoch
time changed from `0.887 ms` mean to `0.907 ms` mean. These are single Release
observations, not a throughput claim.

The opt-in is available through both the adapter and workflow, and remains
development-only:

```bash
python3 apps/gnss.py smartphone-gnss-workflow \
  --archive data/gsdc2023/cache/dataset_2023.zip \
  --role development \
  --output-dir output/smartphone-r5/development-galileo-e1-hatch30 \
  --experimental-galileo-e1 \
  --experimental-galileo-e1-hatch-window-s 30 \
  --performance
```

The full record, source hashes, reset counts, split commands, and artifact
hashes are in
`docs/use_cases/records/smartphone_r5_galileo_e1_hatch_development.json`.
The release profile stores the lane under
`galileo_e1_development_lane.galileo_e1_hatch_development_lane`. The official
Kaggle primary score remains undetermined; values above are the existing four
explicit local diagnostics. Holdout is sealed until a new release freeze.

## Truth-free trajectory smoother (phase 8, promoted development-only)

The Hatch30 POS is now consumable by a separate, explicitly experimental
truth-free trajectory command. It transforms WGS84 ECEF to a numerically
guarded local ENU frame, runs a constant-velocity Kalman filter followed by a
per-segment RTS pass, and reconstructs WGS84 ECEF/LLH output. The measurement
covariance uses only solver PDOP/satellite-count quality as a declared proxy and
floor; it does not read labels or ground truth. A Mahalanobis innovation gate
marks rejected measurements as `PROPAGATED`, while long device gaps and the
fixed train/validation boundary reset all filter and RTS state.

The raw `device_gnss.csv` epoch keys are the output population. Thus an input
POS gap is not silently dropped: internal missing keys are emitted as
`interpolated`, with the source and covariance recorded in `trajectory.csv`.
The smoother itself never accepts a ground-truth argument. The optional
submission path calls the existing truth-free generator and publishes the
submission and manifests atomically:

```bash
python3 apps/gnss.py smartphone-trajectory-smoother \
  --experimental-truth-free-kalman-rts \
  --position output/smartphone-r5/hatch-full-w30/libgnsspp_spp.pos \
  --device-gnss data/gsdc2023/materialized/dataset_2023/train/2023-05-24-20-26-us-ca-sjc-ge2/pixel7pro/device_gnss.csv \
  --profile configs/benchmarks/smartphone_r5_gsdc2023.json \
  --role development \
  --output-dir output/smartphone-r5/trajectory-smoother-development/selected \
  --process-noise 1.0 --measurement-floor-m 1.0 \
  --submission-output output/smartphone-r5/trajectory-smoother-development/submission.csv \
  --phone pixel7pro \
  --dataset-id 2023-05-24-20-26-us-ca-sjc-ge2/pixel7pro
```

Candidate selection is a separate development-only command. It evaluates the
predeclared `(process-noise, measurement-floor-m)` pairs
`(0.01,0.5)`, `(0.01,1.0)`, `(0.10,0.5)`, `(0.10,1.0)`, and `(1.0,1.0)` on
the fixed 830/553 chronological split. Every candidate resets at index 830;
the winner is selected on train and must be non-inferior on validation for
availability, WGS84 horizontal median/P95, vertical P95, and all four local
distance/percentile diagnostics, with at least one strict improvement. The
evaluator reads development truth only for this score gate and has no holdout
mode:

```bash
python3 apps/gnss.py smartphone-trajectory-smoother-eval \
  --experimental-trajectory-selection --role development \
  --position output/smartphone-r5/hatch-full-w30/libgnsspp_spp.pos \
  --device-gnss data/gsdc2023/materialized/dataset_2023/train/2023-05-24-20-26-us-ca-sjc-ge2/pixel7pro/device_gnss.csv \
  --ground-truth data/gsdc2023/materialized/dataset_2023/train/2023-05-24-20-26-us-ca-sjc-ge2/pixel7pro/ground_truth.csv \
  --profile configs/benchmarks/smartphone_r5_gsdc2023.json \
  --output-dir output/smartphone-r5/trajectory-smoother-development \
  --phone pixel7pro --dataset-id 2023-05-24-20-26-us-ca-sjc-ge2/pixel7pro
```

The selected `q1p00_floor1p0` candidate passed the validation gate. Values are
metres; `H` is WGS84 horizontal error and `K` is the mean of the four explicit
local diagnostics (the official Kaggle distance/percentile details remain
undetermined):

| Split / lane | Epochs / availability | H median / P95 | V P95 | K |
|---|---:|---:|---:|---:|
| Train baseline | 830 / 100.00% | 3.053 / 7.790 | 22.356 | 5.432 |
| Train selected | 830 / 100.00% | 2.529 / 6.062 | 23.382 | 4.296 |
| Validation baseline | 553 / 100.00% | 3.449 / 8.245 | 16.354 | 5.851 |
| Validation selected | 553 / 100.00% | 3.166 / 6.573 | 14.393 | 4.879 |
| Full development baseline | 1,383 / 100.00% | 3.214 / 7.944 | 20.192 | 5.580 |
| Full development selected | 1,383 / 100.00% | 2.748 / 6.173 | 20.096 | 4.458 |

The selected full route contains 1,015 measured and 368 gated/rejected rows,
two segments (the explicit split reset), zero numerical fallbacks, and
1,383/1,383 output keys. The selection run took `2.52 s` wall time and
`44,120 KB` peak RSS; smoothing itself took about `0.122 s`. A separate
truth-free gap-contract smoke on the pre-Hatch 1,376-row POS reproduced the
7 missing device epochs as 7 `interpolated` rows, with a maximum input POS gap
of 8 s and 1,383/1,383 output keys. This smoke is not an accuracy result.

The machine-readable evaluation, hashes, and gap smoke record are in
`docs/use_cases/records/smartphone_r5_trajectory_smoother_development.json`.
Artifacts are under
`output/smartphone-r5/trajectory-smoother-development/`; the smoother and
submission manifests explicitly state `truth_used=false`. This is a
development-only promotion; no holdout data was opened.

## Causal IMU motion-adaptive process noise (phase 9, promoted development-only)

The Pixel development `device_imu.csv` was audited before using it. The schema
has `MessageType`, UTC milliseconds, `elapsedRealtimeNanos`, three measurement
axes, and three bias axes. It contains `UncalGyro` 166,369 rows and
`UncalAccel` 166,305 rows (approximately 125 Hz each), plus `UncalMag` 69,161
rows (approximately 50 Hz). Gyro and acceleration biases are finite and zero
on this route; magnetometer bias is recorded but the magnetometer is not used
for motion adaptation. There are no non-finite or parse-error rows, no UTC or
elapsed-clock reversals, and no observed clock discontinuities. The CSV hash is
`8f1c582bdad545d11e68349b221001b69e2a9d43f9257e60038c193276fa1804`.

The candidate uses only a one-second causal window of gyro vector norm and
robust acceleration-norm variation. It does not estimate attitude and never
integrates device-frame acceleration. When dynamics exceed fixed thresholds,
the baseline `q=1.0` process noise is multiplied by a fixed gain. A stale
window, IMU sample gap over 250 ms, non-finite value, or elapsed/UTC clock
discontinuity forces multiplier `1.0`. The motion profile is reset at the
830/553 filter boundary, and future IMU samples are excluded. The explicit
development-only selector is:

```bash
python3 apps/gnss.py smartphone-trajectory-imu-eval \
  --experimental-motion-adaptive-q-selection --role development \
  --position output/smartphone-r5/hatch-full-w30/libgnsspp_spp.pos \
  --device-gnss data/gsdc2023/materialized/dataset_2023/train/2023-05-24-20-26-us-ca-sjc-ge2/pixel7pro/device_gnss.csv \
  --device-imu data/gsdc2023/materialized/dataset_2023/train/2023-05-24-20-26-us-ca-sjc-ge2/pixel7pro/device_imu.csv \
  --ground-truth data/gsdc2023/materialized/dataset_2023/train/2023-05-24-20-26-us-ca-sjc-ge2/pixel7pro/ground_truth.csv \
  --profile configs/benchmarks/smartphone_r5_gsdc2023.json \
  --output-dir output/smartphone-r5/trajectory-imu-development \
  --phone pixel7pro --dataset-id 2023-05-24-20-26-us-ca-sjc-ge2/pixel7pro
```

Five predeclared one-second candidates were ranked on train by H median, H
P95, V P95, and the mean of the four local diagnostics. Validation required
availability, all three sign-off error values, and each diagnostic to be
non-inferior, with one strict improvement. The selected candidate was
`gyro0p25_accel1p0_gain2` (gyro threshold `0.25`, acceleration-norm dynamic
threshold `1.0`, gain `2.0`). It marked 424/1,383 epochs dynamic and safely
fell back at 2 boundary epochs; no holdout was read.

| Split / lane | Epochs / availability | H median / P95 (m) | V P95 (m) | Four-diagnostic mean (m) |
|---|---:|---:|---:|---:|
| Train baseline | 830 / 100.00% | 3.053 / 7.790 | 22.356 | 5.432 |
| Train IMU selected | 830 / 100.00% | 2.530 / 6.041 | 23.382 | 4.286 |
| Validation baseline | 553 / 100.00% | 3.449 / 8.245 | 16.354 | 5.851 |
| Validation IMU selected | 553 / 100.00% | 3.113 / 6.630 | 13.271 | 4.871 |
| Full development baseline | 1,383 / 100.00% | 3.214 / 7.944 | 20.192 | 5.580 |
| Full development IMU selected | 1,383 / 100.00% | 2.742 / 6.271 | 19.285 | 4.507 |

The full selected route has 1,047 measured and 336 rejected/predicted output
rows, two reset segments, and zero numerical fallbacks. The evaluation took
`6.33 s` wall time with `109,680 KB` peak RSS; compact IMU buffers keep the
35-MB CSV audit bounded (a direct audit measured about `104,292 KB` peak RSS).
The selected truth-free manifest also contains `motion_profile.csv`, the IMU
audit/hash, and the atomic submission generator manifest. Full artifacts and
all candidate metrics are recorded in
`docs/use_cases/records/smartphone_r5_trajectory_imu_development.json`.
The official Kaggle primary score remains undetermined; the values above use
the existing four local diagnostics. This phase is development-only.

## SPP broadcast lookup cache (phase 5, development-only)

Release timing isolated the Galileo cost to the repeated
`solvePositionLS`/`buildMeasurements` stage. `NavigationData` already caches a
state for an exact satellite/time key, but each SPP position iteration still
repeated the state calls and the `getEphemeris` map scan. The optimized
ordinary broadcast path now keeps an epoch-local copy of the corrected
satellite state, corrected transmit time, and selected ephemeris pointer. The
first iteration retains the old call order; matrix construction, outlier
rejection, and RAIM decisions are unchanged. Precise products, SSR
corrections, and the literal MRTKLIB IFLC path are deliberately left uncached
because their selection can depend on receiver/correction state.

On the same Release build and 1,383-epoch Pixel development route:

| Lane | Solver wall (s) | Peak RSS (KB) | Elapsed sum (ms) | Epoch mean / P50 / P95 (ms) | POS bytes |
|---|---:|---:|---:|---:|---|
| Galileo E1 before cache | 1.24 | 19,760 | 1,052.644 | 0.761 / 0.936 / 1.206 | reference |
| Galileo E1 with cache | 1.06 | 19,804 | 847.806 | 0.613 / 0.748 / 0.974 | byte-identical |

The solver wall time decreased by 14.5% and the timed epoch work by 19.5%; a
44-KB RSS increase is the bounded epoch-local cache. The GPS-only control POS
was also byte-identical. The optimized run retained 1,383/1,383 solutions,
the four local Kaggle diagnostics (`5.910908`, `5.911589`, `5.921347`,
`5.921857` m), and all sign-off metrics. The official primary score remains
undetermined because the public specification omits distance and percentile
details. The exact hashes and timing CSV are recorded in
`docs/use_cases/records/smartphone_r5_spp_broadcast_cache_optimization.json`;
the release profile records the stage, safety gates, and measurements under
`galileo_e1_development_lane.spp_broadcast_cache_optimization`.

The same gating is safe for RTK's SPP fallback, while the primary RTK
double-difference path in `rtk_satellites.cpp` keeps its own rover/base state
and geometry calls. A 120-epoch PPC-Dataset Tokyo run1 smoke benchmark with
the optimized build produced 120 valid / 116 fixed solutions, 96.67% fix
rate, 0.046 m horizontal median, 0.129 m horizontal P95, 0.322 m maximum
horizontal error, 0.037 m vertical P95, and 1.374 s solver wall time
(17.32× realtime). No pre-optimization RTK artifact was available, so this is
a safety/throughput benchmark rather than a separate RTK promotion claim.
No holdout data was opened.

### RTK fixed pre/post verification (same cache, Release)

The missing RTK pre/post comparison was subsequently run on the same 120
epochs using separate temporary source/build trees. Each variant had one
warmup followed by ten measured runs pinned to CPU 0; five pairs ran
baseline→candidate and five ran candidate→baseline. The input, command,
compiler flags, and base interpolation policy were fixed. The complete
hash manifest and every run are in
`docs/use_cases/records/rtk_ppc_tokyo_spp_cache_prepost_release.json`;
generated aggregate artifacts are under `output/rtk-cache-audit/`.

| Variant | Solver wall median / P95 across runs (s) | Epoch mean / P50 / P95 (ms) | Realtime factor | Peak RSS (KB) | Valid / fixed |
|---|---:|---:|---:|---:|---:|
| Baseline (cache off) | 1.409847 / 1.616603 | 11.7487 / 11.7282 / 13.2042 | 16.8839× | 30,444 | 120 / 116 |
| Candidate (cache on) | 1.321774 / 1.738514 | 11.0148 / 11.0309 / 12.3483 | 18.0061× | 30,448 | 120 / 116 |

Both variants produced the same POS bytes (`bae93b2c…b4b6e2bdd`), 96.6667%
fix rate, zero wrong-fix epochs, H median/P95 `0.045861/0.128737 m`, V P95
`0.037212 m`, ratio P50/P95 `21.875584/63.085008`, and local PPC score
`0.286866%`. The median solver wall improved by 6.25%; the candidate P95
across-run wall is higher because of a scheduling/thermal outlier, while its
per-epoch P95 is lower.

Stage telemetry identifies SPP as the dominant measured stage: its mean was
`9.4148→8.6279 ms/epoch`; the inclusive `resetPositionToSPP` stage was
`4.6339→4.2398 ms`, and independent KF update was `1.2217→1.2380 ms`.
These stage values overlap because reset calls SPP. RTK invokes SPP for the
normal current-SPP/gating seed and again in dynamic reset, so the cache is not
fallback-only. The SPP-result and cross-call lookup reuse experiments below
were both closed as No-Go; the next independent bottleneck is KF update. The
official Kaggle primary metric is not used for this PPC RTK signoff. The
current `gnss-performance-report.v1` schema keeps generic solver and RTK stage
metrics only; removed experiment counters appear only in the historical
No-Go records/artifacts.

### Same-epoch SPP result reuse (historical No-Go; removed)

The next RTK experiment enabled `--reuse-rtk-spp-epoch` for the same
PPC-Dataset Tokyo run1 120-epoch route. The cache is a stack object owned by
one `processRTKEpoch()` call; it is never retained by `RTKProcessor`, shared
between threads, or carried to the next epoch. Both `initializeFilter()` and
the dynamic `resetPositionToSPP()` consume it only after a successful current
SPP result has been produced.

Reuse is accepted only when all of the following match: rover object identity,
exact epoch time and a deterministic fingerprint of every rover observation
and RINEX tracking side collection; navigation object identity, supported
mutation revision, satellite/ephemeris counts, and broadcast atmospheric
model; the complete `SPPConfig`; and precise/IONEX/DCB/SSR loaded flags and
population counts. An invalid SPP, revision-zero navigation fixture, changed
input, changed option, or changed correction state fails closed and executes a
fresh SPP call. This historical experiment was removed after the byte-identity
gate failed; no production or holdout evaluation is claimed. The recorded
timing CSVs exposed `spp_epoch_reuse_attempts/hits/misses`; the current
performance reporter intentionally omits those removed-experiment counters.

Five measured runs per setting (one warmup per setting, CPU 0 pinned, Release
`-O3 -DNDEBUG`) showed a substantial throughput reduction in the duplicate
SPP work:

| Variant | Solver wall median / P95 across runs (s) | Epoch mean / P50 / P95 (ms) | SPP mean (ms) | Reset mean (ms) | Peak RSS (KB) | Valid / fixed | Reuse attempts / hits / misses |
|---|---:|---:|---:|---:|---:|---:|---:|
| Feature off | 1.289214 / 1.291719 | 10.7434 / 10.7482 / 11.9379 | 8.4548 | 4.1573 | 30,524 | 120 / 116 | 0 / 0 / 0 |
| Feature on | 0.798002 / 0.799453 | 6.6500 / 6.5864 / 7.4856 | 4.2857 | 0.0858 | 30,468 | 120 / 116 | 121 / 121 / 0 |

The candidate reduced median solver wall by 38.10% and did not change valid,
FIX, wrong-FIX, horizontal/vertical error, or the four local PPC diagnostics
at the recorded precision (`H median/P95 0.045861/0.128737 m`, `V P95
0.037212 m`, zero wrong-FIX, local distance score `0.286866%`). However,
it is not byte-identical: three POS rows differ only in the rounded RTK update
NIS field (`9.5017→9.5018`, `62.2881→62.2880`, `21.2628→21.2627`), and the
unrounded ratio stream has small last-digit differences. This is consistent
with stateful iterative SPP convergence: skipping the second call also skips
its internal state update. Therefore the strict POS byte-identity gate is not
met and the feature was removed; no production promotion or holdout evaluation
is claimed. See
`docs/use_cases/records/rtk_ppc_tokyo_spp_epoch_reuse_release.json` and
`output/rtk-cache-reuse/` for hashes, per-run timing, and stage summaries.

### RTK same-epoch broadcast lookup cache (historical No-Go; removed)

Because full SPP-result reuse skipped the second `SPPProcessor::processEpoch()`
state transition, the strict alternative keeps every SPP call, iteration, QC
decision, and persistent SPP/RTK side effect. The historical opt-in
`--reuse-rtk-broadcast-state-epoch` shares only ordinary broadcast ephemeris
selection and satellite-state lookup results between those calls. Its cache is
a stack object scoped to one `processRTKEpochInternal()` call and is guarded by
rover object/time/content, navigation object/revision/shape, complete SPP and
processor options, product populations, and the Galileo I/NAV override. It is
fail-closed for revision-zero navigation, precise products, SSR, and the
literal MRTKLIB IFLC path; standalone smartphone SPP does not pass a cache.

The three earlier NIS-only POS differences are therefore explained by the
full-result experiment, not by this lookup-only candidate: the skipped second
SPP call had changed the persistent `estimated_position_`, receiver clock,
system-bias and last-valid state after its own iterative/QC path. RTK NIS is
then computed from the seeded filter state/covariance and its innovation, so
the changed seed propagated into three rounded NIS fields. The lookup-only
candidate executes that second call and preserves the state/iteration order.

Five measured Release runs per variant (PPC-Dataset Tokyo run1, 120 epochs,
CPU 0, `-O3 -DNDEBUG`, no holdout) produced byte-identical POS files and
identical 120 valid / 116 fixed solutions:

| Variant | External wall median / P95 (s) | Solver sum median / P95 (s) | Epoch mean median (ms) | Epoch P50 / P95 median (ms) | SPP / reset mean (ms) | Peak RSS median (KB) |
|---|---:|---:|---:|---:|---:|---:|
| Cache off | 1.310 / 1.334 | 1.243631 / 1.266250 | 10.3636 | 10.3732 / 11.7443 | 8.1504 / 4.0277 | 30,484 |
| Cache on | 1.310 / 1.394 | 1.245755 / 1.321013 | 10.3813 | 10.3504 / 11.6876 | 8.1660 / 3.9935 | 30,476 |

The candidate had 240,049 ephemeris-cache hits and 8,302 misses per run; the
state-cache counters remained zero because this route's existing navigation
state cache and receiver-position initialization avoid the eligible state
lookup. Median external wall time was unchanged and solver work increased by
0.171%; the higher candidate wall P95 is a scheduling outlier. H/V error,
ratio, status, and the local PPC diagnostics were unchanged
(`H median/P95 0.045861/0.128737 m`, `V P95 0.037212 m`, ratio P50/P95
`21.875584/63.085008`, local score `0.286866%`). This does not meet the
performance-improvement gate, so the candidate is No-Go and was removed (no
production promotion and no holdout evaluation).
Detailed hashes, stage telemetry, cache counters, and commands are recorded
in `docs/use_cases/records/rtk_ppc_tokyo_spp_broadcast_lookup_cache_release.json`;
per-run artifacts are under `output/rtk-broadcast-cache/`.

## GLONASS G1 development candidate (phase 6, No-Go)

The Pixel development route contains a usable GLONASS G1 population, so the
FDMA contract was audited in a bounded development-only experiment. The source
has 10,210 selected `GLO_G1_CA` rows over 1,383 epochs, `ConstellationType=3`,
empty `CodeType`, and nine stable PRN-to-FCN mappings (3/4/5/9/10/16/18/19/20
to +5/+6/+1/-2/-7/-1/-3/+3/+2). The Android carrier values are checked against
`1602.000 MHz + FCN * 0.5625 MHz` with a 200-Hz source tolerance; the largest
observed residual is 100 Hz. The experimental RINEX writer emitted `R`
`C1C/L1C/D1C/S1C` plus a standard `GLONASS SLOT / FRQ #` header. All 10,210
selected observations matched one of 1,199 R navigation records, with a
maximum nearest-record age of 899.999 seconds. Native GLONASS state propagation,
UTC-to-GPST conversion, separate GLONASS clock-bias grouping, and
`model_intersystem_bias=true` were verified from the existing solver code; no
native solver change was needed.

Despite complete availability and navigation coverage, the candidate failed
the promotion gate against the promoted Galileo E1 development lane. Full-route
H median/P95/V P95 were `4.502/11.710/24.000 m` versus `3.407/8.428/19.502 m`,
and the four local public-spec Kaggle diagnostics were
`8.096934/8.097086/8.106009/8.106757 m` versus
`5.910908/5.911589/5.921347/5.921857 m`. The 600-epoch slice showed the same
direction (`4.907/13.247/27.313 m` versus `3.376/8.921/25.530 m`). The
candidate was therefore not promoted, its experimental adapter code was
removed, and no holdout data was opened. The complete audit and retained
development artifacts are recorded in
`docs/use_cases/records/smartphone_r5_glonass_g1_development.json`.

## GPS L5_Q development candidate (phase 6, No-Go before solver run)

The same Pixel development route contains 4,005 `GPS_L5_Q` rows (4,003 after
the frozen first-epoch skip) over 1,383 selected epochs. The source declares
`ConstellationType=1`, an empty `CodeType`, and an exact 1,176.45 MHz carrier;
the four visible L5 PRNs are 18, 25, 26, and 28. L5 has raw pseudorange and
quality fields on every row. 3,875 selected L5 satellite/epoch keys also have
GPS L1 C/A, while 128 L5 keys do not. Epoch cadence is 1 s with no selected
gap. The full quality and pairing audit is recorded in
`docs/use_cases/records/smartphone_r5_gps_l5_q_development.json`.

The local GPS broadcast file has 449 records covering PRNs 1--32, so selected
L5 observations have 100% nearest-ephemeris coverage (maximum age 3,000.999 s).
Native signal policy and RINEX parsing already recognize GPS band 5 as
`GPS_L5` and the standard `C5Q/L5Q/D5Q/S5Q` family. The adapter does not emit
these fields today because its safe contract is one selected frequency per
satellite and GPS L1 remains the frozen primary lane.

This candidate is blocked before a solver experiment. In the current SPP,
GPS L5 is a secondary signal: default SPP selects GPS L1 only, while IFLC
groups L1/L5 into one `SPPObservation`. The GPS RINEX nav parser retains only
the legacy `eph.tgd` delay for GPS and sets `eph.tgd_secondary=0`; the SPP
group-delay path applies that same value for GPS L1 and L5 and has no
GPS-L5-Q-specific ISC. Although the generic bias-code table names L5 as
`C5Q`, this route has no GPS L5 OSB/DCB product. Treating L5 as an additional
independent code would therefore change the measurement model, not merely
add observations, and could introduce metre-level signal bias. No
experimental flag, candidate output, or holdout run was created.

The next safe step is to add and validate GPS L5-Q signal-bias provenance
(CNAV or an appropriate DCB/OSB product), a per-frequency observation/bias/
ionosphere model, and a paired L1/L5 RINEX writer contract. Only then should a
development 600/full comparison be run against the promoted Galileo E1 lane;
all accuracy, availability, and four local Kaggle diagnostic gates must
improve before promotion.

## Train-route generalization inventory (phase 7)

The local `dataset_2023.zip` is inventoried with ZIP central-directory
metadata before any route is selected or extracted.  The reproducible command
below verifies the frozen archive SHA-256, materializes only the selected
route/phone `device_gnss.csv`, `device_imu.csv`, `ground_truth.csv`, and
route-level `brdc.nav`, and then runs the fixed Galileo E1 + Hatch30 lane with
the promoted constant-velocity smoother and the already frozen causal IMU
process-noise candidate:

```bash
python3 apps/gnss.py smartphone-generalization \
  --archive data/gsdc2023/cache/dataset_2023.zip \
  --profile configs/benchmarks/smartphone_r5_gsdc2023.json \
  --output-dir output/smartphone-r5/generalization
```

The command has no holdout mode.  The profile's designated holdout ID is
excluded using metadata only; its content and truth are never opened.  The
selection record is frozen in
`docs/use_cases/records/smartphone_r5_generalization_candidates.json`.
Central inventory found 1,048 archive entries and 547 train entries, with 41
route/phone pairs containing the three required CSVs and 40 of those having a
unique route broadcast file.  The final train-only set spans 2021/E1-highway/mi8,
2022/SJC/pixel5, and 2023/MTV/pixel6pro.  A first signal-only screen rejected
the otherwise structurally complete 2022 Samsung route because it had no
`GAL_E1_C_P`; this was recorded before the final set was frozen.  A 1-ms
week/TOW canonicalization is applied only when native POS rounding differs
from the Android epoch key; coordinates and quality tokens are unchanged and
larger mismatches fail closed.

For each selected phone, the report emits exact matched truth coverage,
availability, WGS84 horizontal P50/P95, vertical P95, four local distance /
percentile diagnostics, per-phone `(P50+P95)/2`, and the arithmetic mean over
phones.  It also records adapter/SPP/smoother wall time and peak RSS.  The
official Kaggle primary value remains unset because the public specification
does not resolve distance/percentile implementation details.  In the measured
three-phone development set, fixed-q and IMU-adaptive accuracy was identical:
mean H median/P95 `2.631/194.731 m`, mean V P95 `28.737 m`, availability
`100%`, and mean WGS84 linear diagnostic `98.681 m`.  The IMU stage cost more
(mean `2.237 s`, peak RSS `45,533 KB`) than fixed-q (mean `1.181 s`, peak RSS
`43,037 KB`), so this cross-route run gives no strict accuracy improvement
(`no-go-no-strict-improvement`).  Full JSON artifacts are under
`output/smartphone-r5/generalization-v6/` for the recorded run.

## Bounded smoother reacquisition experiment (phase 8 follow-up)

The generalization diagnostics show that the raw Galileo-E1/Hatch30 SPP POS
stays close to truth while the existing smoother drifts after long Mahalanobis
gate-reject runs: Mi8 has raw/existing H P95 `5.400/368.633 m` with a 130-epoch
(129 s) maximum reject run, Pixel5 has `9.947/191.539 m` with 87 epochs (86 s),
and Pixel6Pro has `5.817/24.022 m` with 51 epochs (50 s).  The largest existing
smoother errors are rejected/predicted rows and are hundreds of metres from
the simultaneous raw observation, supporting prediction drift as the cause.
Per-epoch raw/smoother error, speed, innovation, and reject-run data are in
`output/smartphone-r5/reacquisition-v2/routes/*/trajectory_diagnostics.csv` and
the corresponding JSON summaries.

The development-only experiment is invoked with:

```bash
python3 apps/gnss.py smartphone-reacquisition-eval \
  --generalization-root output/smartphone-r5/generalization-v6 \
  --output-dir output/smartphone-r5/reacquisition-v2
```

The selection record freezes Mi8 and Pixel5 as candidate-train routes and
Pixel6Pro as validation before scoring.  It compares only three candidates:
bounded reacquisition after 2/2 s, 3/3 s, or 5/5 s of consecutive rejects /
prediction.  A trigger resets the truth-free CV state to the current finite
observation, starts a new RTS segment, and records reacquisition count and
maximum reject run in each smoother manifest.  `reacq_r2_d2` wins the train
aggregate and passes validation against both raw and existing smoother:
validation H median/P95 is `2.210/5.606 m`, V P95 `14.032 m`, all four local
diagnostics and availability are non-regressive, and 48 reacquisitions are
recorded.  The independent development-main route regresses (`2.748/6.173 m`
existing versus `2.901/7.510 m` candidate H median/P95), so the promotion gate
is `no-go-development-main-regression`; the existing smoother remains retained.
IMU adaptive remains No-Go.  The complete decision and hashes are recorded in
`docs/use_cases/records/smartphone_r5_reacquisition_candidates.json`.

### Conservative reacquisition follow-up

Because the main Pixel7Pro route has only an 11-epoch maximum reject run, a
second development-only experiment froze substantially more conservative
15/15, 20/20, and 30/30 epoch/second bounds.  The prior Pixel6Pro validation
route was not reused.  The first unused, distinct E1-capable train pair was
fixed from the central-directory inventory and signal-only screen as
`2021-03-16-20-40-us-ca-mtv-b/pixel4xl` (8,819 valid `GAL_E1_C_P` rows, 1,476
epochs); the route's truth was not read during selection or pipeline
generation.

Run it with:

```bash
python3 apps/gnss.py smartphone-reacquisition-conservative-eval \
  --output-dir output/smartphone-r5/reacquisition-conservative-v3
```

The train aggregate selected `reacq_r15_d15`.  On the new validation route it
reduced existing-smoother H P95 from `2301.470 m` to `118.139 m`, but still
regressed the raw SPP/Hatch reference (`6.482 m`), V P95 (`14.941 m` to
`38.207 m`), and all four local distance/percentile diagnostics.  Therefore
the validation gate is No-Go.  All three candidates produced byte-identical
main Pixel7Pro POS output with zero reacquisition events, but this does not
override the new-validation failure.  The existing smoother and default
pipeline contract remain unchanged; IMU adaptive remains No-Go.

The report, manifests, materialization record, and route outputs are under
`output/smartphone-r5/reacquisition-conservative-v3/`; the frozen selection
and final decision are recorded in
`docs/use_cases/records/smartphone_r5_reacquisition_conservative_candidates.json`.

### Segment stability gate (phase 9, promoted development-only)

The existing truth-free RTS smoother is now followed by a segment stability
gate.  For every smoother segment it records maximum consecutive rejects,
reject fraction, maximum normalized innovation, and maximum prediction
duration.  The reject fraction remains an audit field rather than a tuned
gate.  An unstable segment is replaced in full by the canonical raw/Hatch POS;
if a device epoch key is missing, the replacement is explicitly bracketed raw
interpolation (or an explicit nearest raw row at an edge).  Stable segments
retain the existing RTS coordinates.  This keeps truth out of pipeline
generation and records the raw position hash with each decision.

The selection was frozen before scoring to three small candidates: 15/15,
20/20, and 30/30 maximum rejects/prediction seconds.  Mi8, Pixel5, and the
development-main Pixel7Pro were the train roles.  A new, distinct
E1-capable validation pair, `2021-07-14-20-50-us-ca-mtv-e/pixel4`, was selected
from the central-directory inventory and signal-only screened before its
required files were materialized.  The designated holdout was not opened.

Run the reproducible evaluation with:

```bash
python3 apps/gnss.py smartphone-segment-stability-eval \
  --output-dir output/smartphone-r5/segment-stability-v1
```

Candidate `segment_r15_d15` was selected (all three tied on this data and the
fixed candidate ID tie-break selected the smallest ID) and promoted only to
the development-only recommended pipeline.  Both failure train routes were
classified unstable and fell back exactly to raw/Hatch for all 2,001 Mi8 and
1,449 Pixel5 epochs.  The main Pixel7Pro segment was stable (11 maximum
rejects, 11 s maximum prediction, 26.68% reject fraction), retained the
smoother's improvement over raw, produced zero fallback epochs, and its POS
was byte-identical to the existing smoother for every candidate.

The new validation segment was intentionally unstable (204 maximum rejects,
204 s maximum prediction, 85.68% reject fraction), so all 1,187 candidate
epochs used raw fallback.  This was non-regressive versus raw/Hatch:
availability `100%`, horizontal WGS84 median/P95 `2.684/7.338 m`, and vertical
P95 `13.885 m`; the existing smoother was `361.428/1379.047 m` horizontal and
`47.276 m` vertical P95.  The stable-segment validation gate was not applicable
because that route had no stable segment; the main-route byte-identity and
all four local diagnostic gates passed.  The end-to-end measured wall time was
`7.479 s` (new-validation adapter `1.734 s`, peak RSS `24,964 KB`; SPP
`1.177 s`, peak RSS `19,412 KB`).

The report and decision are recorded in
`output/smartphone-r5/segment-stability-v1/segment_stability_report.json` and
`docs/use_cases/records/smartphone_r5_segment_stability_candidates.json`.
The production/default smoother contract remains unchanged; no holdout tuning
was performed.

### Android handset WLS position lane (phase 10, evaluated—not promoted)

The handset-provided `WlsPositionXEcefMeters`, `WlsPositionYEcefMeters`, and
`WlsPositionZEcefMeters` fields are now available through a separate,
truth-free extractor:

```bash
python3 apps/gnss.py smartphone-wls-position \
  --device-gnss <device_gnss.csv> \
  --output-dir output/smartphone-r5/wls-position-v1/one-route \
  --skip-epochs 1
```

Each epoch is accepted only when every source row has finite WLS coordinates
within the earth-scale ECEF range and all rows agree within `0.001 m` per
axis.  Timestamps must be monotonic; hardware-clock discontinuity counts
must be constant within an epoch and monotonic between epochs.  Missing,
partial, non-finite, out-of-range, or inconsistent WLS values fail closed and
are classified in the error/manifest contract.  Positive epoch gaps and clock
transitions are recorded rather than silently repaired.  Successful runs
atomically publish `receiver_wls.csv`, `wls.pos`, `wls_summary.json`, and
`wls_manifest.json`; the POS uses WGS84 geodetic conversion and can feed the
truth-free submission generator.

The fixed six-route audit found complete WLS fields on every row: zero missing
or invalid epochs, zero within-epoch coordinate disagreements, one-second
maximum timestamp gaps, and zero clock-discontinuity transitions.  WLS raw
horizontal P95 beat the native Galileo E1/Hatch raw lane on five routes but
lost on the development-main Pixel7Pro, so route behavior is mixed.  Selected
WLS raw scores were:

| Route | WLS H median / P95 (m) | WLS V P95 (m) | Availability |
|---|---:|---:|---:|
| Mi8 | 1.234 / 3.613 | 4.880 | 100% |
| Pixel5 | 2.420 / 4.634 | 5.765 | 100% |
| Pixel6Pro validation | 2.418 / 5.163 | 14.782 | 100% |
| Pixel4XL audit | 2.390 / 4.715 | 11.427 | 100% |
| Pixel4 audit | 4.213 / 6.174 | 7.281 | 100% |
| Pixel7Pro main | 3.676 / 8.489 | 14.218 | 100% |

The shared trajectory smoother plus segment-stability candidates (15/15,
20/20, and 30/30) were generated before truth scoring.  None passed the
train non-regression gate against WLS raw: all improved aggregate H median and
vertical P95 but worsened aggregate H P95 and all four local distance /
percentile diagnostics.  Therefore no candidate was selected; the fixed
Pixel6Pro validation and Pixel7Pro main gates were recorded as not applicable
for promotion.  Pixel4XL and Pixel4 remained post-selection audits only.

Because WLS wins and loses by route, the report also records whether its
truth-free integrity features (finite/consistent coordinates, epoch coverage,
gap, clock transitions, and satellite population) could support a selector.
They are currently suitable only for fail-closed integrity rejection, not
accuracy selection; no truth-dependent runtime selector was implemented.
The complete report, per-route WLS manifests, and frozen role/candidate record
are under `output/smartphone-r5/wls-position-v1/` and
`docs/use_cases/records/smartphone_r5_wls_position_candidates.json`.
The designated holdout was not opened and the production/default lane is
unchanged.

### WLS/native device-family generalization (phase 11, No-Go)

The Pixel7Pro route frozen from the ZIP central directory was evaluated once
after both truth-free lanes had published their artifacts:
`2022-11-15-00-53-us-ca-mtv-a/pixel7pro`.  The WLS lane extracted the handset
ECEF fields and the native lane used Galileo E1, C1C Hatch smoothing over 30 s,
and the existing segment-stability bounds.  The designated holdout
`2023-09-06-22-49-us-ca-routebb1/pixel7pro` was not opened, materialized, or
scored.  Truth was parsed once, only after adapter/SPP, WLS, both smoothers,
and their manifests were complete.

The new route did not reproduce the development-main Pixel7Pro native
advantage under the frozen all-metric gate.  Native segment-stability had
horizontal median/P95 `2.550/7.018 m`, vertical P95 `12.752 m`, and local
diagnostic mean `4.782 m`; raw WLS had `2.918/5.646 m`, `14.195 m`, and
`4.284 m`, respectively.  Native improved the median and vertical P95 but
regressed horizontal P95 and all four local diagnostics, so the route-level
reproduction gate failed.

The combined seven-route selector comparison also failed non-regression
(`H-P95 5.356 m` for the device-profile selector versus `5.491 m` for WLS,
but vertical P95 `10.999 m` versus `10.364 m`).  The existing six-route gate
likewise failed on vertical P95.  Therefore no device-family selector profile
was written or promoted; the runtime/default lane is unchanged.  This is a
recorded No-Go, not a truth-dependent route selector.

The complete truth-free artifacts, route-level scores, timing, and hashes are
in `output/smartphone-r5/wls-device-family-v1/`; the frozen selection and
post-evaluation decision are in
`docs/use_cases/records/smartphone_r5_wls_device_family_selection.json`.

### Truth-free native/WLS stability selector (phase 12, development-only recommended)

The next candidate uses only truth-free lane evidence at runtime.  Both the
native Galileo E1/Hatch30 segment report and the raw handset WLS manifest are
validated first.  When every native segment is marked `stable`, the native
stable POS is selected; one or more unstable segments selects raw WLS.  A
malformed WLS integrity manifest, non-finite value, or device-key mismatch
fails closed.  The selected POS, Kaggle-compatible submission, submission
manifest, and selector manifest are atomically published.  The selector does
not accept a device model or truth input.

The new route was frozen from central-directory metadata after excluding the
holdout and every previously used route/device:
`2023-05-16-19-55-us-ca-mtv-xe1/pixel7pro`.  Its device-only capability screen
found `11,185` valid Galileo E1 rows across `2,323` epochs and `63,320` finite
WLS rows.  Native raw SPP had one missing device epoch; the segment-stability
lane synthesized the full device-key output, found the single native segment
unstable, and selected raw WLS.  Only after both lanes and the selector
submission were published was the new route truth parsed once.

The new-route selected WLS lane scored H median/P95 `2.344/4.496 m`, V P95
`6.541 m`, and local diagnostic mean `3.420 m` at `100%` availability; the
native stable alternative was `2.743/6.399 m`, V P95 `13.503 m`, and `4.569 m`.
On the known seven routes, the stability selector (main Pixel7Pro native,
other routes raw WLS) achieved H median/P95 `2.620/5.160 m`, V P95 `11.205 m`,
and diagnostic mean `3.890 m`, versus native-only `2.647/6.882 m` and WLS-only
`2.753/5.491 m`.  All four local distance/percentile diagnostics improved over
both single lanes, availability remained `100%`, and V P95 stayed below the
frozen `45 m` sign-off threshold.

The recommended profile is recorded as development-only in
`configs/benchmarks/smartphone_r5_gsdc2023.json` and exposed in the
development workflow manifest.  It is not the production RTK/SPP default and
the designated holdout remains unopened.  Reproducible outputs, atomic
manifests, and the metadata-first selection record are under
`output/smartphone-r5/wls-stability-selector-v1/` and
`docs/use_cases/records/smartphone_r5_wls_stability_selector_selection.json`.

### Stability-selector release freeze and sealed holdout attempt (phase 13)

The recommended lane was frozen before any holdout payload access.  The freeze
record fixes the source-file and Release `gnss_spp` hashes, archive/profile and
navigation hashes, all Galileo E1/Hatch30, trajectory, WLS-integrity, segment
stability, selector, Kaggle-diagnostic, and sign-off parameters, plus the
truth-free/atomic and no-post-holdout-tuning contracts.  Its separate hash
manifest records freeze-record SHA256
`49ec6a07ca770aac764f562abfa2d4823cd74aabbb948163250f7c3b5656783f`.

The single permitted run-2 attempt reached the archive central-directory
metadata check, then stopped on a pre-materialization profile/freeze device
hash-key mismatch.  No device GNSS, navigation, or truth payload was opened;
the truth-free lanes and selector were therefore not generated and truth was
read zero times.  This was not an environment/IO failure, so the frozen policy
forbids adjustment or rerun.  The sealed failure record is
`docs/use_cases/records/smartphone_r5_wls_stability_selector_holdout_run2.json`
and the raw run artifact is under
`output/smartphone-r5/wls-stability-selector-holdout-run2-v1/`.  The existing
run-1 holdout result remains the only scored holdout result; production RTK/SPP
defaults are unchanged.

The v2 preflight-only retry froze the same algorithm/parameter projection
(`c82fba6130c7d16c207213ab1e2ac542e02280db194949a300c68bebd412ca7e` on both
v1 and v2) and the same Release/archive/source contracts.  Its single attempt
materialized only the required device GNSS and route navigation members, then
stopped before the native adapter because the adapter authorization expected a
basename source key while the v2 freeze stores full relative source paths.  No
truth was materialized or opened (`truth_open_count=0`), and no WLS/selector
artifact or score exists.  This preflight mismatch is sealed as
`docs/use_cases/records/smartphone_r5_wls_stability_selector_holdout_run2_v2.json`;
the one-shot policy forbids another retry or tuning.

### v3 preflight repair and sealed holdout result (phase 15)

The v3 freeze retained the same measurement, selector, and numeric parameter
projection (`c82fba6130c7d16c207213ab1e2ac542e02280db194949a300c68bebd412ca7e`)
and changed only preflight orchestration.  The adapter and WLS authorization
guards now explicitly map the full repository-relative freeze source paths to
their required basenames; basename-only keys are rejected.  A no-archive
fixture atomically materialized a tiny device file, passed both authorization
guards, and started the truth-free adapter smoke.  The v3 freeze record and
separate manifest are
`docs/use_cases/records/smartphone_r5_wls_stability_selector_holdout_freeze_v3.json`
(`62ffc9e45049dba60cd4bbf5c6b6c4f3a064c81e3c8ddfd425fc4a916e661bff`) and
`docs/use_cases/records/smartphone_r5_wls_stability_selector_holdout_freeze_v3_manifest.json`
(`4d0de39e3dbc823be932ea843051d69afc7573a553fd808de0c1639b75bd28d9`).

The one permitted v3 holdout run then materialized only device GNSS and
broadcast navigation, completed both truth-free lanes and the atomic selector
submission, and opened/parses the designated truth exactly once.  The native
Galileo E1/Hatch30 segment was unstable (`1/1` segment; 48 consecutive rejects,
48 s prediction duration), so the selector truth-free decision was `wls_raw`.
Selected H P50/P95 was `2.639/5.086 m`, V P95 `5.361 m`, availability `100%`,
and the four local Kaggle diagnostics were `3.869/3.869/3.863/3.863 m`.
Native-only was `2.129/5.031 m`, V P95 `10.215 m`, availability `100%`, while
raw WLS matched the selected lane.  The selected lane improved on the existing
run-1 baseline (`4.127/14.910 m`, V P95 `29.890 m`, availability `99.688%`)
without post-holdout tuning; wall/RSS were `15.362 s`/`125476 kB`.

All frozen sign-off thresholds passed.  The development-only Kaggle
submission lane remains recommended and the production RTK/SPP default is
unchanged.  The sealed report, run manifest, and record are under
`output/smartphone-r5/wls-stability-selector-holdout-run2-v3/` and
`docs/use_cases/records/smartphone_r5_wls_stability_selector_holdout_run2_v3.json`.
No further holdout rerun or parameter/code tuning is permitted.

## Release sequence

### WLS residual release research (No-Go, phase 16)

The next release study first selected one validation route and one future
holdout from ZIP central-directory metadata only.  The validation was
`2023-05-25-20-11-us-ca-sjc-he2/pixel7pro`; the future holdout is
`2023-05-25-19-10-us-ca-sjc-be2/sm-s908b`.  The latter was not materialized and
its truth was not opened.  The old holdout
`2023-09-06-22-49-us-ca-routebb1/pixel7pro` was excluded from all train and
candidate ranking.  The selection record is
`docs/use_cases/records/smartphone_r5_wls_residual_release_candidate_selection.json`
(`e7c51c056c105a123d394403a3e0d33709e4e22cea411b809358f00d5ba596b8`).

Already-opened development data was decomposed into local along/cross/up WLS
residuals by phone, speed, and turn-rate groups.  Bias signs varied by route
and device, so no fixed shift was justified.  Nine truth-free candidates were
predeclared: centered component-wise median windows 3/5 (plus raw), each with
along-track shifts -1/0/+1 m.  The sealed run's selected `median5_shift_0`
improved H P95 (5.109 vs 5.160 m) and V P95 (10.977 vs 11.205 m) against the
v3 selector aggregate, but regressed H P50 by 0.000085 m (2.620236 vs
2.620151 m).  The corrected gate therefore rejects promotion.  The new
validation truth was opened exactly once; native stability selected the native
lane and all its sign-off/non-regression checks passed, but this cannot repair
the known-train H P50 failure.  The implementation remains experimental and
the production RTK/SPP defaults are unchanged.  Details and hashes are in
`docs/use_cases/records/smartphone_r5_wls_residual_release_evaluation.json`.

### WLS residual v2 audit (No-Go before truth, phase 17)

The v2 audit first corrected and fixture-tested the aggregate comparator
mapping: route-level metric paths must map to the `mean_*` fields in the
known-route aggregate.  The candidate and gate were then frozen without
search: component-wise centered `median5_shift_0`, zero along-track shift,
four diagnostics non-regressing within `1e-6 m`, strict H-P95 improvement,
H-P50 regression at most `0.01 m`, non-regressed availability/coverage, and
V-P95 within `45 m` with no more than `5 m` major regression.

Metadata-only inventory screening selected the new non-Pixel route
`2023-05-23-19-56-us-ca-mtv-ie2/sm-a505g`; the next holdout
`2023-05-25-19-10-us-ca-sjc-be2/sm-s908b` remained unmaterialized and its truth
was not opened.  The selected route's device GNSS/IMU/navigation members were
materialized, and the truth-free adapter/SPP plus native stability lane ran.
The native segment was unstable, but the WLS manifest reported 115 positive
`timestamp_gap` classifications (maximum gap `1.001 s`).  The selector's
fail-closed integrity contract therefore rejected WLS before publishing the
selector/candidate/submission; no truth was materialized or opened
(`truth_open_count=0`).

This is a sealed No-Go, with no selector or parameter tuning and no production
RTK/SPP change.  The failure report/manifest are under
`output/smartphone-r5/wls-residual-v2/`; the sealed record is
`docs/use_cases/records/smartphone_r5_wls_residual_release_evaluation_v2.json`.

### WLS residual v2.1 schema-consistency correction (development-only promotion, phase 18)

The v2 failure record remains sealed with zero truth reads.  Its cause was a
schema inconsistency: WLS classified ordinary `1001 ms` Android epoch spacing
as a gap although the existing adapter/Hatch contract treats elapsed times up
to and including `1500 ms` as continuous.  The WLS boundary is now exactly
`<=1500 ms` continuous and `>1500 ms` a gap/reset boundary; non-increasing
timestamps remain fail-closed.  Boundary fixtures cover `1001` (pass), `1500`
(pass), and `1501 ms` (gap/reset and selector rejection).  No candidate,
selector, gate, or numeric algorithm parameter changed.

Using the same metadata-frozen `sm-a505g` validation, the truth-free E1/
Hatch30, WLS, native stability, selector, median5, and submission artifacts
were sealed before opening truth once.  Native stability was unstable, so the
selector chose `wls_raw`.  Against the v3 selector baseline, the selected lane
achieved H P50/P95 `3.539/8.060 m` versus `3.702/8.690 m`, V P95 `16.267 m`
versus `18.187 m`, availability/coverage `100%/100%`, and all four diagnostic
variants improved (`5.806/5.814/5.800/5.811 m` versus
`6.200/6.219/6.196/6.212 m`).  The frozen gate passed.

The WLS-branch residual postprocess is promoted for development-only Kaggle
submission workflow use.  Production RTK/SPP defaults remain unchanged, and
the next holdout `2023-05-25-19-10-us-ca-sjc-be2/sm-s908b` remains
unmaterialized and unread.  The sealed record is
`docs/use_cases/records/smartphone_r5_wls_residual_release_evaluation_v2_1.json`;
the report and manifest are under `output/smartphone-r5/wls-residual-v2.1/`.

### Reserved next-holdout v4 freeze and one-shot result (No-Go, phase 19)

The reserved next holdout `2023-05-25-19-10-us-ca-sjc-be2/sm-s908b` was fixed
in a v4 freeze before payload access.  The freeze pins the Release binary,
source files, profile, archive, and central-directory metadata; its v3
pipeline parameter hash remains `c82fba…`, with WLS residual candidate
`median5_shift_0`, zero along-track shift, and the existing `<=1500 ms`
continuous timestamp-gap contract.  Fixture and actual central-metadata
preflight both passed with zero payload materializations and zero truth reads.
The freeze record and separate hash manifest are
`docs/use_cases/records/smartphone_r5_wls_stability_selector_holdout_freeze_v4.json`
(`ee1e3f6a…`) and
`docs/use_cases/records/smartphone_r5_wls_stability_selector_holdout_freeze_v4_manifest.json`.

The single sealed run materialized only the frozen device GNSS and broadcast
navigation members, completed the native Galileo E1/Hatch30 lane and handset
WLS lane, selected WLS because its one native segment was unstable, applied
the frozen median5 postprocess, hashed the atomic submission, and opened truth
exactly once.  Native stability had 384 rejected epochs, 28 consecutive
rejects, and 28.001 s maximum prediction duration.  Final median5 metrics were
H P50/P95 `2.330/7.642 m`, V P95 `23.958 m`, availability/coverage `100%/100%`,
and diagnostics `4.981/5.000/4.986/4.993 m`.  The same sealed artifacts' v3
counterfactual raw-WLS metrics were H P50/P95 `2.177/5.063 m`, V P95 `26.340 m`,
and diagnostics `3.616/3.616/3.620/3.623 m`.  Thus median5 improved V P95 by
`2.382 m` but regressed all four diagnostics and horizontal P50/P95; the
frozen v2.1 gate failed and the v4 WLS-branch promotion is No-Go.

The sealed run record is
`docs/use_cases/records/smartphone_r5_wls_stability_selector_holdout_run2_v4.json`;
report and manifest are under
`output/smartphone-r5/wls-stability-selector-holdout-run2-v4/`.  No post-truth
code or parameter tuning, rerun, or production RTK/SPP default change is
permitted.  The prior development-only v2.1 lane remains recorded separately;
this v4 holdout result does not promote median5 for a new release.

### Multi-phone WLS ensemble validation (development-only promotion, phase 20)

The multi-phone phase selected two previously unused two-phone train routes,
one two-phone validation route, and a three-phone next holdout from ZIP
central-directory metadata only.  The selected methods were frozen before
payload access: coordinate-wise ECEF median, Weiszfeld geometric median, and
20% symmetric trimmed mean (arithmetic-mean fallback for two phones).  Every
route manifest records UnixTimeMillis alignment within 1 ms, phone count,
missing observations, and ECEF spread.  The next holdout
`2022-10-06-21-51-us-ca-mtv-n` remains unmaterialized and unread; prior routes
and v3/v4 holdouts were excluded from ranking.

The first train attempt was sealed as a pre-validation failure because the
two train truth logs used stable 432--433 ms logger offsets.  The audit was
corrected without changing WLS, fusion, or numeric candidate parameters: it
estimates a constant pairwise truth-clock offset and applies the frozen 100 ms
tolerance to the residual.  Both train audits then passed (coverage `99.84%`
and `99.94%`, horizontal P95 `14.776/14.289 m`, vertical P95
`0.557/0.492 m`).

The coordinate-wise median won the train-only fixed-method ranking.  On the
new validation route, after all truth-free lane artifacts were hashed, truth
was opened once per phone for one evaluation pass.  The fused lane improved
over the single-phone WLS/v3-branch-equivalent baseline from H P50/P95
`3.307/5.860 m` to `3.155/5.120 m`, V P95 `7.290` to `5.777 m`, with
availability unchanged at `100%` and coverage `99.15%`; all four official-like
diagnostic variants improved by about `0.446 m`.  The frozen gate passed, so
the lane is recorded as development-only recommended.  Production RTK/SPP
defaults remain unchanged.

The sealed record and self-hash manifest are
`docs/use_cases/records/smartphone_r5_wls_multi_phone_ensemble_evaluation.json`
(`edab0c8b…`) and
`docs/use_cases/records/smartphone_r5_wls_multi_phone_ensemble_evaluation_manifest.json`
(`cda3a002…`).  The report and truth-free validation artifacts are under
`output/smartphone-r5/wls-multi-phone-ensemble-v1-retry/`; validation truth
open count is `2`, evaluation pass count is `1`, and no holdout payload was
read.

### Multi-phone ensemble holdout freeze and sealed failure (phase 21)

The reserved three-phone route
`2022-10-06-21-51-us-ca-mtv-n` was frozen before payload access.  The freeze
pinned the coordinate-wise ECEF median, 1 ms timestamp alignment, one-phone
raw-WLS fallback, WLS `<=1500 ms` continuous gap contract, atomic submission
key contract, four diagnostics, sign-off thresholds, Release binaries,
profile, archive, and source hashes.  Central-directory and fixture preflight
passed with zero holdout materializations and zero truth reads.  The freeze
record and manifest are
`docs/use_cases/records/smartphone_r5_wls_multi_phone_ensemble_holdout_freeze.json`
(`3b12264c…`) and
`docs/use_cases/records/smartphone_r5_wls_multi_phone_ensemble_holdout_freeze_manifest.json`
(`bd2bf1cc…`).

The single post-freeze attempt materialized only the three device GNSS
members.  It stopped before WLS extraction because the existing WLS helper
correctly rejects a `holdout` role while the multi-phone holdout orchestration
did not provide its authorized execution mode.  No WLS, median, submission,
or truth artifact was generated; truth open count is `0`.  Per the sealed
one-shot contract, no code/parameter adjustment or rerun was made.  The
failure is recorded as No-Go for full-submission integration, while the
development-only validation lane and production defaults are unchanged.

The sealed failure record and manifest are
`docs/use_cases/records/smartphone_r5_wls_multi_phone_ensemble_holdout_evaluation_failure.json`
(`73a3be41…`) and
`docs/use_cases/records/smartphone_r5_wls_multi_phone_ensemble_holdout_evaluation_failure_manifest.json`.

### Multi-phone holdout authorization-only recovery (phase 22, No-Go)

The v1.1 freeze added an explicit holdout authorization contract to the WLS
helper: the record and manifest schema/hash, exact route and three-phone
allowlist, algorithm-parameter hash
(`203cea0d8354c9b70515134642837269f26514346404514ee059a26e4a4076fb`), and
truth-free phase must all match.  Missing, malformed, mismatched, or
truth-bearing invocations remain fail-closed.  Fixture tests cover those
boundaries and an archive-free authorized WLS smoke.  The candidate
coordinate-wise median, 1 ms alignment, one-phone fallback, and all numeric
parameters were unchanged.  The v1.1 freeze record/manifest are
`docs/use_cases/records/smartphone_r5_wls_multi_phone_ensemble_holdout_freeze_v1_1.json`
(`af172d4c…`) and
`docs/use_cases/records/smartphone_r5_wls_multi_phone_ensemble_holdout_freeze_v1_1_manifest.json`
(`256c8b21…`); the previous failure is retained with truth-open count `0`.

The one permitted v1.1 holdout attempt reused only the already sealed
device-only materializations (all three input hashes were rechecked), then
stopped at the first WLS integrity check: `sm-a205u` row 2 had no complete
WLS ECEF (`missing_wls_epoch`).  No WLS, median, submission, or truth artifact
was published/opened (`truth_open_count=0`), so phone/aggregate metrics and
wall/RSS success timing are not applicable.  The failure artifact is
`output/smartphone-r5/wls-multi-phone-ensemble-holdout-v1.1/` and the sealed
record/manifest are
`docs/use_cases/records/smartphone_r5_wls_multi_phone_ensemble_holdout_evaluation_v1_1_failure.json`
(`0d33cafe…`) and
`docs/use_cases/records/smartphone_r5_wls_multi_phone_ensemble_holdout_evaluation_v1_1_failure_manifest.json`.
This is a No-Go for full-submission integration; no rerun, tuning, profile
promotion, or production RTK/SPP default change is permitted.

### Multi-phone v1.2 sparse-WLS recovery (phase 23, No-Go)

The v1.2 change retained the single-phone WLS default fail-closed and added
an `--allow-missing-wls-epochs` path only after the sealed multi-phone
authorization contract.  An epoch with at least one complete finite WLS row
is retained after the existing agreement checks; blank rows are counted, an
all-blank epoch is omitted with timestamp/reason, and partial triplets,
non-finite values, inconsistent coordinates, and unresolved all-phone
timestamps remain fail-closed.  The target timestamp set is taken from each
phone's selected device epochs; the coordinate-wise median records used-phone
count, fallback, spread, and source per epoch.  Fixture coverage includes
leading/internal omissions, all-phone missing timestamps, and partial/nonfinite/
inconsistent rejection.  The fusion/alignment/tolerance/fallback algorithm
core hash stayed identical (`203cea0d…`).

The v1.2 freeze was sealed before holdout payload access; its record and
manifest are
`docs/use_cases/records/smartphone_r5_wls_multi_phone_ensemble_holdout_freeze_v1_2.json`
(`7541b248…`) and
`docs/use_cases/records/smartphone_r5_wls_multi_phone_ensemble_holdout_freeze_v1_2_manifest.json`
(`94434131…`).  The one permitted run reused the previously sealed three
device-only inputs and generated sparse truth-free WLS artifacts for all
three phones.  It then stopped at the unchanged `>1500 ms` timestamp-gap
integrity gate: `sm-a205u` had one gap (`2.001 s`).  No median, submission, or
ground truth was generated/opened (`truth_open_count=0`); the generated WLS
summaries still record the blank-row and omitted-epoch statistics.  The
sealed failure record/manifest are
`docs/use_cases/records/smartphone_r5_wls_multi_phone_ensemble_holdout_evaluation_v1_2_failure.json`
(`5b90f47e…`) and
`docs/use_cases/records/smartphone_r5_wls_multi_phone_ensemble_holdout_evaluation_v1_2_failure_manifest.json`.
This remains No-Go for submission integration; no gap-policy relaxation,
rerun, tuning, profile promotion, or production RTK/SPP default change is
permitted.

### Multi-phone v1.3 sparse-gap contract consistency (phase 24, No-Go)

The v1.3 contract keeps the ordinary/single-phone WLS path fail-closed for
gaps over 1500 ms.  Only a v1.3 sealed multi-phone sparse invocation may
retain such a valid-WLS gap as a diagnostic (`gap count`, maximum gap, and
omitted timestamps).  The coordinate-wise ECEF median still uses only finite
WLS rows within the fixed 1 ms alignment tolerance: every target device
timestamp must have at least one source phone, peer recovery is allowed, and
interpolation/extrapolation is forbidden.  Shared gaps and peer timestamps
outside tolerance fail closed.  Fixtures cover one-phone 2.001 s gaps with
peer coverage, all-phone shared gaps, and out-of-tolerance peers.  The
candidate method, tolerance, fallback, and numeric algorithm core hash remain
unchanged (`203cea0d8354c9b70515134642837269f26514346404514ee059a26e4a4076fb`).

The v1.3 freeze record and hash manifest were sealed before the one-shot
attempt:
`docs/use_cases/records/smartphone_r5_wls_multi_phone_ensemble_holdout_freeze_v1_3.json`
(`ec4dafb2…`) and
`docs/use_cases/records/smartphone_r5_wls_multi_phone_ensemble_holdout_freeze_v1_3_manifest.json`
(`c5392c1b…`).  Existing device-only materializations were hash-verified;
the archive was inspected through central metadata only before execution.

The sole v1.3 attempt generated and hashed all three truth-free WLS lanes and
the single-phone baseline lane, then stopped at the frozen coverage guard:
`sm-a205u/1665093125997` had no finite source within 1 ms on any phone.  No
median lane, selected submission, or truth was opened (`truth_open_count=0`),
and no metrics or timing score is claimed.  The sealed failure record and
manifest are
`docs/use_cases/records/smartphone_r5_wls_multi_phone_ensemble_holdout_evaluation_v1_3_failure.json`
(`77ca0f74…`) and
`docs/use_cases/records/smartphone_r5_wls_multi_phone_ensemble_holdout_evaluation_v1_3_failure_manifest.json`.
This is No-Go for development submission integration; no rerun, adjustment,
profile promotion, or production RTK/SPP default change is permitted.

### Multi-phone alignment tolerance v1.4 audit (phase 25, No-Go)

The v1.4 research candidate changes only the optional nearest-source
alignment tolerance from 1 ms to 10 ms for general Android epoch jitter:
absolute offsets through 10 ms pass, 11 ms fails, an equal-distance tie picks
the earlier source timestamp deterministically, and interpolation/extrapolation
remain forbidden.  The coordinate-wise median, geometric median, trimmed-mean
candidate set, sparse fallback, and all algorithm parameters are unchanged;
the ordinary default remains 1 ms.  A fixture covers the `10 ms` acceptance,
`11 ms` rejection, and deterministic tie boundary.

The already sealed two-route train and one-route validation artifacts were
replayed truth-free at both tolerances.  All route submissions had identical
SHA-256 hashes and all aggregate horizontal/vertical, availability/coverage,
and four official-like diagnostic values were identical; no extrapolation was
used.  This confirms non-regression of the existing evidence, but it is not a
new validation selection.

Before any new payload was read, central-directory metadata was audited after
excluding every previously used multi-phone route and the reserved v1.3/v1.4
holdout routes.  The archive contains eight eligible multi-phone groups and
all eight are excluded by prior-use or holdout policy, leaving no fresh
validation and no permissible next holdout.  Therefore a
fresh-validation-driven v1.4 freeze was not authorized in that metadata-only
audit, no new truth was opened, and no profile or production RTK/SPP default
changed at that point.  The metadata selection and existing-artifact
comparison are sealed in
`docs/use_cases/records/smartphone_r5_wls_multi_phone_alignment_v1_4_selection.json`
and
`docs/use_cases/records/smartphone_r5_wls_multi_phone_alignment_v1_4_existing_comparison.json`;
the corresponding hash manifest records the No-Go decision and preserves the
fresh-validation holdout truth-open count at zero.  A later explicit release
owner decision permitted a separate existing-evidence freeze and reserved
holdout one-shot; that sealed result is recorded below and does not change the
fresh-route inventory conclusion.

### Multi-phone v1.4 existing-evidence freeze and one-shot holdout (phase 26)

Because the inventory contained no unused multi-phone validation route, the
release-owner freeze explicitly records `fresh_unused_multi_phone_route_count:
0` and uses the already selected three-phone route only as the reserved
holdout.  The freeze keeps the coordinate-wise ECEF median, sparse-WLS
integrity, fallback, and every numeric parameter unchanged.  The sole change
is truth-free cross-phone input alignment: nearest finite WLS rows within
`10 ms` are accepted, `11 ms` is rejected, equal-distance ties choose the
earlier source timestamp, and interpolation/extrapolation remain forbidden.
The existing 1 ms versus 10 ms train/validation comparison had identical
submissions, metrics, and all four diagnostics.  The freeze and its manifest
were sealed before the reused device inputs or any truth access:
`docs/use_cases/records/smartphone_r5_wls_multi_phone_ensemble_holdout_freeze_v1_4.json`
and
`docs/use_cases/records/smartphone_r5_wls_multi_phone_ensemble_holdout_freeze_v1_4_manifest.json`.

The one permitted run reused only the previously hash-sealed three device
GNSS inputs, generated sparse WLS, 10 ms coordinate-wise median, and each
phone's atomic submission, then sealed those artifacts before materializing
the three truth files once.  The selected lane used 3,618 three-phone, 29
two-phone, and 15 one-phone fallback target observations; 59 phone
observations were missing, the maximum source offset was 6 ms, and ECEF
spread P50/P95/max was `11.761/28.413/136.584 m`.  Aggregate availability and
truth coverage were `100.000%/99.973%` for both lanes.  The selected median
improved over single-phone WLS from H P50/P95 `3.159/8.542 m` to
`2.757/7.067 m`, V P95 `15.481` to `10.556 m`, and the four diagnostics
`5.850/5.853/5.850/5.857` to `4.905/4.912/4.912/4.918 m` (ordering follows
the manifest); all per-phone and aggregate sign-off gates passed.

The sealed run record and manifest are
`docs/use_cases/records/smartphone_r5_wls_multi_phone_ensemble_holdout_evaluation_v1_4.json`
and
`docs/use_cases/records/smartphone_r5_wls_multi_phone_ensemble_holdout_evaluation_v1_4_manifest.json`;
truth open count is `3` with one evaluation pass, wall time was `8.291 s`,
and peak RSS was `198,092 KB`.  The lane is integrated as a
development-only full-submission candidate in the profile.  Production
RTK/SPP defaults remain unchanged, and no rerun or post-holdout tuning is
permitted.

### Full-test submission batch integration (phase 27, sealed truth-free preflight)

The test archive was audited through its ZIP central directory before any
test member was materialized.  It contains 40 route/phone records; all 40
have exactly one `device_gnss.csv` and one route `brdc.nav`.  The inventory
records possible truth members as metadata only and keeps their materialization
forbidden.  Neither the archive nor the local dataset tree contains the
authoritative `sample_submission.csv`, so the batch correctly stops before
device/nav payload access and emits a sealed failure rather than inferring a
key order.  The inventory and failure artifacts are
`output/smartphone-r5/wls-test-batch-v1/test_inventory.json` and
`output/smartphone-r5/wls-test-batch-v1/test_batch_failure.json`.
The sealed record is
`docs/use_cases/records/smartphone_r5_wls_test_batch_v1.json`
(`df5145baaa2f75c25061f453abb581b20a884f79d1b1fe18535adf014c73307b`).

The official-source recovery audit is sealed in
`docs/use_cases/records/smartphone_r5_wls_test_batch_v1_kaggle_source_audit.json`
(`2f808f3edf0301c83450279db389394adbf87de61895e612c1da34da4b1050c3`) with
manifest
`docs/use_cases/records/smartphone_r5_wls_test_batch_v1_kaggle_source_audit_manifest.json`
(`5ace7ca89d2f95bf70dd620db0eaab6ea438e6bf8dee3a95488a26b8a1793e09`).
The Kaggle CLI, Python package, common credential files, and matching
environment-variable names were absent; the official files-list API returned
HTTP 401, so no sample was downloaded.  The official competition page says to
predict each `phone`/`UnixTimeMillis` in the sample and gives the four-column
format, but does not state that row order is irrelevant or that the sample's
once-per-second key subset can be derived from raw test epochs.  Therefore no
derived-key-order artifact was generated and the existing failure remains
sealed with zero truth access.

When a sample is supplied, the test-only authorization pins the v1.4 freeze,
source/binary/profile/archive hashes, 10 ms nearest-source coordinate-wise
ECEF median, sparse WLS integrity, and the route allowlist before payload
access.  A route with one phone emits raw WLS; a route with multiple phones
uses the frozen median; an all-phone-unresolved key may use only an exact-key
native Galileo E1/Hatch30+segment-stability artifact.  Missing fallback,
duplicate/missing/extra sample keys, non-finite coordinates, and any truth
access fail closed.  Publication is atomic and cache keys include all input
and contract hashes.  The command is:

```text
python3 apps/gnss.py smartphone-wls-test-batch \
  --archive data/gsdc2023/cache/dataset_2023.zip \
  --inventory output/smartphone-r5/wls-test-batch-v1/test_inventory.json \
  --sample-submission data/gsdc2023/sample_submission.csv \
  --output-dir output/smartphone-r5/wls-test-batch-v1
```

No external Kaggle submission is performed and production RTK/SPP defaults
remain unchanged.

### Derived-key full-test attempt (phase 28, explicit unverified opt-in)

Because an official sample was still unavailable, the batch was resumed only
with the explicit
`--allow-derived-unverified-key-order` opt-in into
`output/smartphone-r5/wls-test-batch-derived-unverified-v1/`.  The command
materialized only the 40 inventory-authorized `device_gnss.csv`/`brdc.nav`
pairs and validated `utcTimeMillis` Raw epochs (frozen `skip_epochs=1`) as
finite, strictly increasing per-phone keys.  The provisional convention is
printable ASCII `route/phone`, ordered bytewise by phone and numerically by
`UnixTimeMillis`; it is explicitly not an official-sample-verified submission.

All 40 route attempts completed truth-free: 37 routes published raw-WLS route
artifacts and content-addressed caches, while 3 routes failed closed.  The
failures were the out-of-earth-range WLS row in
`2021-11-30-20-59-us-ca-mtv-m/mi8`, and unresolved sparse epochs without an
existing exact-key native stable fallback in
`2022-06-22-20-12-us-ca-lax-hh/samsunga325g` and
`2022-10-06-20-46-us-ca-sjc-r/sm-a205u`.  Since an incomplete 37/40 key set
cannot be emitted, no `submission_derived_unverified.csv` was published;
partial route artifacts and the sealed failure/run manifests were retained.
No new native lane, interpolation, candidate, or numeric parameter was added.

The run manifest is
`output/smartphone-r5/wls-test-batch-derived-unverified-v1/test_batch_derived_run_manifest.json`
(`a89c5c3ac816289b6c6e5958d77fbc034f601869c3e64bd5191d87808fe74822`), and
the failure artifact/manifest hashes are
`479185f27eca9e49a4060c2730c231a10ed8fbb43cfe9dfe717a1db436a1eeae` and
`296b430ec01d88fdc417d3930b26ea366fccf2654936fc8cb029a90f10f3a9eb`.
When an official sample becomes available, the only permitted promotion path
is:

```text
python3 apps/gnss.py smartphone-wls-test-batch \
  --output-dir output/smartphone-r5/wls-test-batch-derived-unverified-v1 \
  --sample-submission <official-sample.csv> \
  --promote-derived-with-sample
```

That path performs strict key-set validation and sample-order reordering only;
it does not rerun WLS, native fallback, or any positioning algorithm.  It
publishes an official `submission.csv` only after the sample key set exactly
matches the provisional set.  The provisional lane and promotion remain
development-only and never claim Kaggle submission eligibility before this
check.

### Test completeness fallback v1 (truth-free, development-only)

Before extending the test lane, the three prior failures were audited using
only their materialized device/nav inputs and sealed failure artifacts.  The
audit is `docs/use_cases/records/smartphone_r5_wls_test_completeness_fallback_v1_failure_audit.json`.
It records one finite out-of-earth-range WLS epoch with a valid two-second
bracket, one internal all-blank epoch with a valid two-second bracket, and two
leading blank epochs with no valid selected-source bracket.  No native stable
artifact was available for these three inputs and test truth access remained
zero.

The fixed candidate was then truth-free tested on three already materialized
development routes with 1/2/5/10-second artificial masks and one out-of-range
omission per route.  All 18 artifacts were hashed before reading development
truth; availability and truth coverage stayed at 1.0 and the fixed horizontal,
vertical, and four diagnostic upper-bound gates passed.  The evaluation and
truth-free artifact manifest are under
`output/smartphone-r5/wls-test-completeness-fallback-v1/evaluation/`.

The freeze is
`docs/use_cases/records/smartphone_r5_wls_test_completeness_fallback_freeze_v1.json`
with manifest
`docs/use_cases/records/smartphone_r5_wls_test_completeness_fallback_freeze_v1_manifest.json`.
Only an explicit sealed test authorization enables this extension:

- finite out-of-earth-range epochs may be omitted as recorded sparse inputs;
  partial coordinate triplets, non-finite values, and inconsistent epochs
  remain fail-closed;
- native Galileo E1/Hatch30 stable fallback uses the nearest position within
  10 ms, with the earlier timestamp winning ties;
- an unresolved internal key may use ECEF linear interpolation only when both
  valid selected-source brackets exist and their gap is at most 10 seconds;
  edge extrapolation and longer gaps fail closed.

The resumed run reused and hash-verified all 37 prior successful route outputs
byte-for-byte, then processed only the three failures.  The first two were
resolved by interpolation (one out-of-range omission and one blank omission),
while `2022-10-06-20-46-us-ca-sjc-r/sm-a205u` remained a deliberate No-Go:
its leading blank keys have neither a native <=10 ms source nor a legal
selected-source bracket.  The sealed result is
`output/smartphone-r5/wls-test-batch-completeness-fallback-v1/test_batch_derived_run_manifest.json`;
it has 39 completed routes, one fail-closed route, zero test truth access,
and no provisional submission.  The sealed run record is
`docs/use_cases/records/smartphone_r5_wls_test_completeness_fallback_v1_run.json`
(SHA-256
`79d1be0d00b76a1abed081f276e849ba8fe17560cd0e04a4c19037cdd684a754`).
The candidate remains development-only, official-sample-unverified, and
never changes the production RTK/SPP default.

### Test edge completeness fallback v2 (No-Go)

An edge-only constant-hold candidate was evaluated on the same three
truth-bearing development routes.  Leading and trailing masks of 1, 2, 5,
and 10 seconds were generated truth-free; each masked key used the nearest
finite selected-source ECEF only when the `HardwareClockDiscontinuityCount`
segment stayed continuous.  Constant hold never extrapolates velocity, and a
missing source, clock break, or gap over 10 seconds remains fail-closed.

The 27 truth-free artifacts were hashed before reading development truth.  The
gate failed only for `sm-a505g` leading 10 seconds: fixed H P50 regressed by
0.018637 m against the 0.01 m signoff limit.  The candidate was not tuned or
relaxed; no v2 freeze was created, the 39-route v1 cache was not reprocessed,
`sm-a205u` was not resumed, and no 40/40 provisional CSV or external Kaggle
submission was produced.  The sealed No-Go record is
`docs/use_cases/records/smartphone_r5_wls_test_edge_completeness_fallback_v2_failure.json`.
Its SHA-256 is
`117fcff02089664863d1fe1b3be8537f569e52ea09f4c4fe63f43a1a9c334056`.

### Test edge completeness fallback v2.1 (frozen, truth-free complete)

The ten-second edge candidate remains a No-Go.  Based on the predeclared
development mask, where the one-second leading/trailing cases passed all three
routes and all signoff checks, v2.1 froze the narrower constant-hold contract:
only leading or trailing unresolved keys may use the nearest finite selected
WLS ECEF, the target and source must remain in the same
`HardwareClockDiscontinuityCount` segment, the maximum distance is 1,000 ms,
and velocity extrapolation is forbidden.  A 1,001 ms edge is still
fail-closed.  No candidate or numeric parameter search was added.

The freeze record and manifest were sealed before test payload access:

`docs/use_cases/records/smartphone_r5_wls_test_edge_completeness_fallback_freeze_v2_1.json`
(`823ad164fe4c4e85d0964e737d418aa85e36117104d67246cd4c9dddccf68c7c`)

`docs/use_cases/records/smartphone_r5_wls_test_edge_completeness_fallback_freeze_v2_1_manifest.json`
(`2b8849e6655f025cdb492f712b53d8501719a2effeba817dc78e20a9f701e3e1`).

The sealed truth-free test run reused all 39 prior successful route outputs
byte-for-byte and processed only
`2022-10-06-20-46-us-ca-sjc-r/sm-a205u`.  Its single leading unresolved key
was 999 ms from the nearest selected WLS source and was resolved by constant
hold within the same clock segment.  The resulting 40/40 provisional CSV has
72,010 self-derived keys with zero missing, extra, duplicate, or non-finite
coordinates.  The run manifest is
`output/smartphone-r5/wls-test-batch-edge-completeness-fallback-v2-1/test_batch_derived_run_manifest.json`
(`726a9c0b2141c567607701c97a544935f6500f0c59dd8054d06d08093ae37d35`), and
the sealed run record is
`docs/use_cases/records/smartphone_r5_wls_test_edge_completeness_fallback_v2_1_run.json`
(`f569c530d88e0c0e3aaf93282dd0b9eae395638b9729afc88024b0c2a7fa6998`).

The CSV is explicitly derived-key-order and not official-sample verified;
the official sample was absent, so external Kaggle submission remains
forbidden.  Test truth was never materialized or opened.  Production RTK/SPP
defaults remain unchanged; this edge lane is development-only.

1. Freeze a development phone/route and an untouched truth-bearing holdout.
2. Publish archive and extracted-file hashes plus the clean-room command.
3. Run the 600-epoch adapter smoke and inspect excluded signal populations.
4. Map only supported signals into the native observation interface. GPS L1
   remains the frozen baseline; Galileo E1 is available only through the
   development opt-in after its exact source/RINEX/nav audit.
5. Produce and truth-score standalone and precise-product POS outputs. The
   standalone lane is complete; a precise-product lane remains pending.
6. Freeze thresholds and invocation before opening the holdout.
7. Run the holdout once and publish Go/No-Go without post-failure tuning.

Reject the workflow when truth is absent, source terms are unknown, clock
state cannot be interpreted, supported observations are empty, or the selected
route cannot meet its frozen coverage and error gates.

### TDCP/ADR trajectory constraint experiment (No-Go)

The next candidate was frozen before any new truth evaluation in
`docs/use_cases/records/smartphone_r5_tdcp_trajectory_selection.json`.  The
truth-free command
`python3 apps/gnss.py smartphone-tdcp-trajectory --base-position <native-hatch30.pos> --device-gnss <device_gnss.csv> --output-dir <output>`
uses only finite Android GPS L1 C/A and Galileo E1 ADR, satellite ECEF, and
the frozen base POS.  It solves the declared four-state time-differenced
carrier-phase equation, applies the frozen robust/RMS/displacement gates, and
falls back to the exact base position on any failure.  `--ground-truth` is not
accepted by this lane; position, trajectory, summary, and manifest outputs are
published atomically.

The fixed train-only evaluator was run with
`python3 apps/gnss.py smartphone-tdcp-trajectory-eval --selection-record docs/use_cases/records/smartphone_r5_tdcp_trajectory_selection.json --output-dir output/smartphone-r5/tdcp-trajectory-evaluation`.
The sealed result is
`docs/use_cases/records/smartphone_r5_tdcp_trajectory_evaluation.json` (SHA-256
`0b405b314cbd70b2de15297b71fe58524f292aca132e117806d56bd7f143d9bf`) and the
machine-readable run evidence is
`output/smartphone-r5/tdcp-trajectory-evaluation/tdcp_train_evaluation.json`.
The blend failed the fixed train gate: it worsened horizontal P50 on all three
routes, worsened all four local diagnostics on the mi8 and Pixel7Pro routes,
and did not improve aggregate horizontal P95 or diagnostic mean.  The fresh
validation and metadata-selected unused holdout were therefore not
materialized or opened.  The implementation remains an experimental
development artifact only; production SPP/RTK defaults and the recommended
submission lane are unchanged.  Public/private Kaggle scores were not used
for candidate selection or tuning.

## Completion audit and release verification

The cross-project completion audit is the machine-readable record
`docs/use_cases/records/rtk_smartphone_performance_release_audit.json`
(SHA-256
`1745954e0998f0a9ee66af125c05be1acb97866f31725c78114fd12e20e0c33c`).  It
maps the RTK pre/post byte-identity evidence, smartphone adapter measurements,
development-only lanes and No-Go experiments, leakage controls, the 40/40
truth-free test artifacts, the authenticated official sample reconciliation,
the single scored external Kaggle submission, build/test results, and the
unresolved leaderboard-rank/metric blockers to their sealed records and
artifacts.  This
audit does not regenerate the provisional CSV or open test/holdout truth.

To reproduce the read-only audit checks and build verification:

```bash
python3 -m json.tool docs/use_cases/records/rtk_smartphone_performance_release_audit.json >/dev/null
sha256sum docs/use_cases/records/rtk_smartphone_performance_release_audit.json
sha256sum output/smartphone-r5/wls-test-batch-edge-completeness-fallback-v2-1/{test_batch_derived_run_manifest.json,provisional_key_manifest.json,submission_derived_unverified.csv}
sha256sum output/smartphone-r5/wls-test-batch-edge-completeness-fallback-v2-1/{submission.csv,submission.manifest.json,test_batch_reconciliation_v2_manifest.json}
python3 -m json.tool docs/use_cases/records/smartphone_r5_kaggle_official_sample_reconciliation_v2.json >/dev/null
cmake --build build --config Release -j$(nproc)
LD_LIBRARY_PATH=/home/sasaki/.local/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH} ctest --test-dir build -j1 --output-on-failure
git diff --check
```

The original provisional test CSV remains explicitly
official-sample-unverified and non-submittable.  The new sample-reconciled CSV
was submitted once and scored as ref `55886678` (public `4.018`, private
`4.873`), but the leaderboard response did not expose team `sasaki`; it must
not be called rank-one evidence.  The audit therefore intentionally records
`goal_complete=false`.

### Historical official-sample blocker audit (superseded by phase 29)

The earlier anonymous-access check is sealed in
`docs/use_cases/records/smartphone_r5_kaggle_official_sample_blocker_audit_v2.json`
(SHA-256
`7646fc89ac331907763a50630d3939424c453fa75e6f0653bbcf57c34345dc86`) with
manifest
`docs/use_cases/records/smartphone_r5_kaggle_official_sample_blocker_audit_v2_manifest.json`.
The official Kaggle files-list API and the direct
`sample_submission.csv` download both returned HTTP 401
`Unauthenticated`; the HTML competition/data pages do not provide sample
bytes.  No non-official copy was used.  Strict key-set/order promotion was
therefore not run, and the existing derived-key artifact remains
non-submittable.  The minimum unblock is an authorized Kaggle account with
rules accepted and an API token configured locally; because the official page
lists the competition close date as 2024-05-23, account-level submission
availability must also be confirmed before attempting any upload.

The earlier follow-up re-detection of modern token locations is sealed in
`docs/use_cases/records/smartphone_r5_kaggle_official_sample_blocker_audit_v3.json`
(SHA-256
`a839481bfa00eee85b31703e50dd7c8bb798ca789c0464fdde5d79b46447754c`) with
manifest SHA-256
`bcfdb70b5f9892b77d6e8f16fcbc4821c5f7a7f391d03f6d7a9a9fe53bee5ea4`.
The token was not visible in the active process or accessible same-user
process environment; `KAGGLE_API_TOKEN`, legacy variables, standard modern
token files, alternate config directories, workspace `.env`/direnv files,
and Kaggle client packages were absent.  Keyring tooling existed, but
attribute-only probes exposed no usable Kaggle token.  No sample download or
strict promotion was attempted.  Configure authentication in this same
process, then run only the official files-list and single-file download
commands recorded in the v3 JSON.

### Authenticated official sample and strict promotion (phase 29, No-Go)

After the token was configured locally with mode `600`, the official Kaggle
files-list API was queried with an Authorization header that was never written
to an argument, log, or artifact.  It identified exactly
`sdc2023/sample_submission.csv` with 5,286,788 bytes.  The official download
returned a one-member ZIP; the member was extracted only after size and
path-traversal checks and published atomically to
`data/gsdc2023/sample_submission.csv`.  Its SHA-256 is
`b0c4853076f715d6bdca46e5c8c99e575f7982d8ee4fc9c0fe417507badcb780`.
The source, response headers, ZIP hash, extraction guards, and no-token
contract are sealed in
`data/gsdc2023/sample_submission.manifest.json` (SHA-256
`c7d74f59ce936d4141da16a7492b20e08f0a379788b83d9f9694c50170a30c49`).

The frozen 40/40 truth-free artifact was compared without rerunning WLS or
opening test truth.  The official sample has header
`tripId,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees` and 71,936 unique
keys; the provisional artifact has header
`phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees` and 72,010 unique
keys.  Both have zero duplicate keys, but 24 official keys are absent from
the provisional set and 98 provisional keys are not in the official set.
Coordinates in the official sample were not used.  Because both the header
and key set fail exact comparison, no formal `submission.csv` or promotion
manifest was published; no inferred timestamp repair or schema rewrite is
permitted.  The complete comparison and failure decision are sealed in
`docs/use_cases/records/smartphone_r5_kaggle_official_sample_promotion_v1.json`
(SHA-256
`df48c38c184bc4d7b53880195834960c486d8fa549423262eb36b428eb6b6d16`).

Read-only competition metadata reports competition id `60095`, deadline
`2024-05-23T23:59:00Z`, `submissionsDisabled=false`, and a five-submission
daily limit.  The deadline has passed, while late-submission eligibility was
not established: the authenticated submission-list GET succeeded, but the
submission-limits probe returned HTTP 404.  No submission or leaderboard
mutation was attempted.  The promotion remains No-Go and the provisional CSV
remains explicitly non-submittable until a separately reviewed schema/key
contract resolves the `tripId`/`phone` and coverage differences.

### Official sample reconciliation v2 (phase 30)

The strict phase-29 comparison was not relaxed implicitly.  A separately
reviewed reconciliation path now treats the authenticated official sample as
the sole schema, key, and row-order authority.  It consumes the already sealed
truth-free 40/40 provisional CSV; it does not rerun WLS, open truth, synthesize
keys, nearest-match timestamps, remap, or interpolate.  Exact provisional keys
keep their coordinates, the 24 official keys absent from that CSV use only the
official sample's finite in-range baseline coordinates, and the 98 provisional
extra keys are dropped.

The Release command was:

```text
python3 apps/gnss.py smartphone-wls-test-batch --archive data/gsdc2023/cache/dataset_2023.zip --inventory output/smartphone-r5/wls-test-batch-v1/test_inventory.json --sample-submission data/gsdc2023/sample_submission.csv --profile configs/benchmarks/smartphone_r5_gsdc2023.json --output-dir output/smartphone-r5/wls-test-batch-edge-completeness-fallback-v2-1 --reconcile-derived-with-sample-v2
```

It atomically produced the local official-schema artifact
`output/smartphone-r5/wls-test-batch-edge-completeness-fallback-v2-1/submission.csv`:
71,936 rows with exact header
`tripId,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees`, official sample key
order preserved byte-for-byte, and zero duplicate, missing, extra, or
non-finite output keys/coordinates.  Source counts are exact provisional
71,912, official-sample baseline fallback 24, and dropped provisional extras
98.  Per-route counts and the zero timestamp-offset analysis are in the
hash-checked run manifest.  The submission SHA-256 is
`3da0009093cf0f53a4bb9919b5c5d3402a9f8c540a5eaa1300bd147ecdc2a803`; its
reconciliation manifest SHA-256 is
`50624f6647919409a0d523635daad0e45699aab2e64cb869627bafb0272ffff6`, and the
run-manifest SHA-256 is
`9b611f1920bb48597eac518cf9f4a8e26f54f3719598ff9aaebea4839caa703a`.

The machine-readable reconciliation evidence is sealed in
`docs/use_cases/records/smartphone_r5_kaggle_official_sample_reconciliation_v2.json`
(SHA-256
`2f5350cc1746e59fc88577a141496d4e2c279493b256a63d52186f5a5f2a928a`).
The official sample's baseline coordinates are a schema reconciliation fallback
only and are not ground truth or an evaluation result.

### Single authorized Kaggle submission (phase 31)

After the local preflight rechecked the official header, 71,936 rows, exact
sample key order, duplicate/extra/missing/non-finite counts, and the fixed
SHA-256, the user explicitly authorized one external submission.  The official
Kaggle upload/finalize protocol was called exactly once with description
`libgnss++ truth-free WLS v2.1`; no retry or parameter change was made.  Kaggle
returned submission ref `55886678`, status `COMPLETE`, public score `4.018`,
and private score `4.873`.  A read-only download of that ref was 6,150,020
bytes and had the same SHA-256 as the local CSV, confirming the uploaded
payload.  The read-only submission list matched ref, timestamp, filename,
description, and byte count.

The authenticated leaderboard endpoints were read-only and returned 280
entries, but did not contain team `sasaki`; therefore no rank is asserted and
rank-one was not achieved.  Submission/score evidence, including the
no-token/no-resubmission contract, is sealed in
`docs/use_cases/records/smartphone_r5_kaggle_submission_v1.json` (SHA-256
`87738e6410dfaac1363d26c656a9a627015292c6a211a0de4c01eebb221bb92e`).  The
competition's close date had passed, so the API's acceptance and scoring are
recorded factually without claiming ongoing eligibility or a leaderboard rank.

For the read-only score comparison, lower scores are better.  The private
280-entry snapshot had rank-one score `0.883` and a top-ten range of
`0.883`–`1.832`; inserting the submitted private score `4.873` by score order
would give hypothetical position 234 (no tie).  The public snapshot had
rank-one score `0.789` and a top-ten range of `0.789`–`1.453`; inserting public
score `4.018` would give hypothetical position 238 (no tie).  These positions
are explicitly hypothetical and are not official ranks because this was a late
submission and the submitting team was absent from the returned leaderboard.

### Single authorized native FGO v5 submission (phase 32)

The frozen native-FGO v5 source-seam-bridge artifact was rechecked locally
before submission: exact header
`tripId,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees`, 71,936 rows, 71,936
unique keys, zero duplicate/missing/extra/non-finite keys, 6,150,020 bytes,
and SHA-256
`89de0e03ff8eb687c9228ac19a1e62d160400b317352652cf8665b22591eef26`.
The user explicitly authorized this one submission.  The official
`kagglesdk` `CompetitionApiService` protocol made exactly one
`StartSubmissionUpload`, one signed file PUT, and one `CreateSubmission` call
with description `libgnss++ native FGO v5`; no retry or second file was sent.

Kaggle returned ref `55900482`, status `COMPLETE`, public score `3.952`, and
private score `4.276`.  A read-only `DownloadSubmission` response was HTTP
200, 6,150,020 bytes, and matched the local SHA-256.  Read-only submission
list metadata matched the ref, filename, description, and byte count.  The
authenticated public/private leaderboard snapshots each contained 280
entries; score-order insertion gives hypothetical positions 172 (public) and
167 (private), not official ranks.  Team names were not recorded and rank one
was not achieved.

Compared with the earlier ref `55886678` (public/private `4.018`/`4.873`),
v5 improved the returned scores by `0.066`/`0.597`; these scores were observed
after the frozen submission and were not used for tuning.  The complete
token-safe one-shot protocol, payload hash, read-only polling/download/list,
leaderboard snapshot, and prior comparison are sealed in
`docs/use_cases/records/smartphone_r5_native_fgo_v5_kaggle_submission_v1.json`
(SHA-256 `8bf5079e82b0ec00022a765dc6039fc9beb20bc9c8c4ff8d78c47ae0af1c4722`)
and its companion manifest (SHA-256
`391dffa3f88cebd0d884508b0a0f58a2a146cdb6b19996c0e49b97d97061b942`).
No token value was printed or persisted, no truth was used, the v5 artifact
and production defaults remain unchanged, and no further submission is
authorized by this record.

### Observable-feature error correction phase (local-only No-Go)

The next independent candidate audit is sealed in
`docs/use_cases/records/smartphone_r5_observable_error_correction_selection.json`
and its train result in
`docs/use_cases/records/smartphone_r5_observable_error_correction_evaluation.json`.
Road-network/map matching was first rejected because this workspace has no
local OSM, road graph, map tile, or route-geometry asset and external
acquisition was out of scope.  The remaining candidate was a fixed,
truth-free three-axis ridge residual correction over handset WLS ECEF using
only raw observable quality, satellite geometry, ADR/clock, timing, WLS ECEF,
and previous-epoch displacement features.  Its alpha (`100.0`), feature order,
route split, and gates were frozen before any new truth access.

The truth-free feature phase produced seven atomic route artifacts.  Only the
seven fixed train truths were then opened for leave-one-route-out fitting and
scoring; fresh validation
`2023-09-07-18-59-us-ca/pixel5` and future holdout
`2023-09-06-00-01-us-ca-routen/pixel6pro` remained unopened.  The candidate
failed every held-out route's horizontal non-regression checks and the strict
aggregate gate: H P95 changed from `5.490768` to `10.833844` m and the four-way
diagnostic mean from `4.121835` to `9.449923` m.  Availability and truth
coverage were unchanged.  No model or parameter was changed after scoring;
there is no production integration, holdout result, or Kaggle submission for
this phase.

To reproduce the local result (no Kaggle/token access and no holdout access):

```bash
PYTHONPATH=apps/commands:apps/commands/benchmarks \
  python3 apps/commands/benchmarks/gnss_smartphone_observable_error_correction_eval.py
PYTHONPATH=apps/commands:apps/commands/benchmarks \
  python3 tests/test_smartphone_observable_error_correction.py
```

The truth-free and evaluation artifact hashes, the selection hash, and the
test/source hashes are fixed in
`docs/use_cases/records/smartphone_r5_observable_error_correction_evaluation_manifest.json`.

### GSDC 2023 top-solution feasibility and Doppler phase (train No-Go)

The primary-source feasibility audit is sealed in
`docs/use_cases/records/smartphone_r5_gsdc2023_top_solution_feasibility_audit.json`.
The official ION abstracts for the first-place and third-place work
([tightly coupled GNSS/INS and timing/weight adaptation](https://www.ion.org/publications/abstract.cfm?articleID=19924)
and
[Doppler/TDCP-assisted estimation](https://www.ion.org/publications/abstract.cfm?articleID=19922))
support the physical direction, but neither abstract publishes a complete
reproducible 2023 implementation.  The official workshop material
([Smartphone Decimeter Challenge 2023-2024](https://www.ion.org/gnss/upload/Smartphone-Decimeter-Challenge-2023-2024.pdf))
and the author repository
([taroz/gsdc2023](https://github.com/taroz/gsdc2023))
likewise document factor-graph/measurement-quality ingredients and external
toolchain or preprocessed-data requirements.  These sources were used for
feasibility ranking only; no reported leaderboard value selected a parameter.

The highest-upside locally reproducible small component was a weighted
pseudorange-rate receiver-velocity estimate followed by a bounded one-step
ECEF prediction/update around the handset WLS position.  It uses only
`device_gnss.csv` satellite ECEF position/velocity, pseudorange rate and its
uncertainty, WLS ECEF, and hardware-clock state.  Its equation, fixed robust
screen, innovation gate, reset behavior, atomic artifacts, and fail-closed
rules are in the machine-readable record
`docs/use_cases/records/smartphone_r5_gsdc2023_doppler_position_evaluation.json`
(manifest:
`docs/use_cases/records/smartphone_r5_gsdc2023_doppler_position_evaluation_manifest.json`).
The production RTK/SPP defaults and the recommended smartphone lane were not
changed.

The three route-disjoint train run failed before fresh validation was eligible:
aggregate WGS84 H P50/P95 changed from `2.230/6.164 m` to `2.108/7.447 m`,
vertical P95 from `12.544 m` to `11.392 m`, and the four-diagnostic mean from
`4.197 m` to `4.776 m`; availability and truth coverage were unchanged.  The
mi8 route improved, while Pixel 5 regressed in H P50/H P95 and Pixel 7 Pro
regressed in H P50 and all four diagnostics.  Exactly the three frozen train
truth files were read by the evaluator.  Fresh validation
`2023-09-07-18-59-us-ca/pixel5` and future holdout
`2023-09-06-00-01-us-ca-routen/pixel6pro` were not materialized or opened.

After that score was sealed, a synthetic unit fixture exposed a one-gross-row
robust-fit edge case.  A deterministic leave-one-out consensus recovery was
added with the same numeric parameters; it was not selected from truth or
leaderboard data and the train truth was not reread.  A separate post-fix
truth-free run is retained under
`output/smartphone-r5/doppler-position-v1-post-consensus-fix/train/`.
Its three position and trajectory files are byte-identical to the scored
artifacts; only summaries gained observable invalid/duplicate row counts.
Because the correction was not rescored, it is evidence of test robustness,
not promotion evidence.

The next-ranked tightly coupled GNSS/IMU timing candidate remains deferred:
the archive has IMU samples, but this workspace lacks a validated truth-free
orientation/lever-arm/timestamp-offset estimator and a reproducible coupled
solver.  Map matching remains blocked because no versioned local road graph or
route geometry is present; fetching a live OSM graph would break the sealed
offline contract.  The learned observable-residual candidate is already a
separate No-Go.  Any future candidate must use a new frozen split and gate;
there is no fresh-validation or holdout result for this phase.

Reproduce the Doppler truth-free artifacts and unit contract with:

```bash
python3 apps/gnss.py smartphone-doppler-position \
  --position <wls.pos> --device-gnss <device_gnss.csv> \
  --output-dir <output> --dataset-id <route/phone> --phone <phone>
PYTHONPATH=apps/commands python3 tests/test_smartphone_doppler_position.py -v
```

The train truth score is intentionally a sealed historical audit, not a
command to rerun it.  Full Release/CTest and `git diff --check` verification
are required before any future development-only integration.

### Causal IMU motion-Q alternative (route-disjoint train No-Go)

Because the Doppler candidate failed its train gate, the least-cost second
physical alternative was frozen and tested: the existing causal IMU feature
builder adjusts only the process-noise multiplier of the truth-free
constant-velocity Kalman/RTS smoother.  The selection record
`docs/use_cases/records/smartphone_r5_gsdc2023_imu_motion_q_selection.json`
fixes all parameters and reuses the same route-disjoint train identities, with
fresh validation and future holdout still sealed.  Its evaluator and atomic
run evidence are recorded in
`docs/use_cases/records/smartphone_r5_gsdc2023_imu_motion_q_evaluation.json`
and its manifest.

The candidate was truth-free generated for all three train routes before
scoring.  Aggregate WGS84 H P50/P95 changed from `2.230/6.164 m` to
`2.269/6.189 m`; vertical P95 improved from `12.544 m` to `12.089 m`, but
the four-diagnostic mean changed from `4.197 m` to `4.230 m`.  The Pixel 5
route regressed in both horizontal percentiles and all four diagnostics, and
mi8 regressed in H P95 and all four diagnostics.  Thus the frozen gate failed;
no fresh-validation or holdout truth was opened and no lane was promoted.

This run also exposed a data-availability limitation rather than silently
inventing motion: the older mi8 and Pixel 5 IMU files have incomplete elapsed
clock rows, so their motion profiles safely used baseline process noise on all
epochs.  Pixel 7 Pro supplied finite IMU samples and 424 dynamic epochs, but
that did not generalize across the route split.  This small fallback is not a
replacement for the published tightly coupled GNSS/INS estimator, and the
production defaults remain unchanged.

### Raw-measurement quality control and robust SPP (train No-Go)

The next independent candidate was frozen before implementation in
`docs/use_cases/records/smartphone_r5_raw_quality_control_selection.json`.
It audits only fields observable before truth: GPS L1/Galileo E1 raw
pseudorange and uncertainty, C/N0, ADR state and uncertainty, clock
discontinuities, constellation/frequency, satellite ECEF geometry, and epoch
gaps.  The pseudorange-minus-WLS range is reported as a clock-centered
innovation proxy; it is diagnostic and is never used to train a threshold.
Android sentinel values are treated as unavailable.

The candidate enabled the existing native SPP uncertainty-weighted iterative
Huber path with fixed 3-sigma threshold and 0.05 weight floor, while retaining
the existing residual rejection and RAIM/FDE.  If its POS is malformed,
nonfinite, incomplete, or the solver fails, the command publishes the exact
handset-WLS POS as a truth-free fallback.  Reports and POS files are staged and
atomically published with source hashes:

```bash
python3 apps/gnss.py smartphone-raw-quality-control \
  --device-gnss <device_gnss.csv> --output-dir <route-output> \
  --obs <rover.obs> --nav <brdc.nav> --fallback-position <wls.pos> \
  --dataset-id <route/phone>
python3 apps/gnss.py smartphone-raw-quality-control-eval \
  --selection-record docs/use_cases/records/smartphone_r5_raw_quality_control_selection.json \
  --output-dir output/smartphone-r5/raw-quality-control-v1 --role train
```

The three frozen train routes were generated truth-free before exactly three
train truth files were opened for scoring.  The candidate failed the frozen
route-level and aggregate gates: mean H P50/P95 changed from
`2.443/5.579 m` to `2.864/7.764 m`, mean V P95 from `8.288 m` to `16.036 m`,
and the four-diagnostic mean from `4.009 m` to `5.314 m`; availability and
truth coverage were unchanged.  The native robust weighting counter remained
zero on these inputs because existing pre-QC rejection resolved the residual
tails, and that behavior was recorded rather than tuned.  Fresh validation
`2023-09-07-18-59-us-ca/pixel5` and future holdout
`2023-09-06-00-01-us-ca-routen/pixel6pro` were not materialized or opened.
The result is a No-Go with no production/default change.
The final sealed report was produced after reporting-only import/serialization
repairs; the frozen candidate, numeric parameters, route roles, and inputs did
not change, and no validation/holdout access occurred during those repairs.

The bounded alternative inspection is sealed in
`docs/use_cases/records/smartphone_r5_gsdc2023_fgo_wiring_feasibility.json`
and its output evidence
`output/smartphone-r5/raw-quality-control-v1/fgo_wiring_feasibility.json`.
Although the repository builds `gnss_fgo`, its current input contract is an
RTK/factor-graph path and the smartphone adapter exposes a single handset
RINEX stream, not the independent base/double-difference model it expects.
An explicit two-epoch, truth-free Eigen debug smoke accepted the handset RINEX
and formed 22 pseudorange factors, but no Doppler/TDCP or double-difference
carrier factors were available.  Wiring a coupled Android pseudorange/clock
factor model therefore requires a new estimator contract, not a bounded
adapter change; it remains deferred.  The optional GTSAM binary also requires
the documented `/home/sasaki/.local/lib` runtime path.

The train report, truth-free seal, route manifests, and FGO feasibility record
are under `output/smartphone-r5/raw-quality-control-v1/`.  No validation,
holdout, token, leaderboard, or external submission was used in this phase.


### Published-source GNSS specification audit (MAT-free, 2026-08-30)

The pinned MIT source tree
[taroz/gsdc2023](https://github.com/taroz/gsdc2023) at commit
`29923f9f370f09ebc00f96d8cca375007a18e7d5` was read only as an algorithm
specification.  No MATLAB data file, saved result, sample coordinate, or
MATLAB-generated artifact is an input to the native lane.  Exact source
hashes, line ranges, and the current native symbols are in
`docs/use_cases/records/smartphone_r5_gsdc2023_native_gnss_pdc_gap_matrix_v1.json`.

The audit covers `gnsslog2obs.m`, `sysfreq2sigtype.m`, `exobs.m`,
`exobs_residuals.m`, `correct_pseudorange.m`, `obserrmodel.m`,
`fgo_gnss.m`, `parameters.m`, `write_results.m`, `submission.m`, and
`score.m`.  The useful raw contracts—Android clock reconstruction,
GPS/GLONASS/Galileo/BeiDou signal mapping, ADR/wavelength conversion, and
negative pseudorange-rate Doppler mapping—are ported to
`src/io/android_raw_gnss.cpp`.  The dedicated executable consumes raw
`device_gnss.csv` and broadcast navigation directly:

~~~bash
LD_LIBRARY_PATH=/home/sasaki/.local/lib:$LD_LIBRARY_PATH \
  build/apps/gnss_pos_vel_pdc \
  --android-raw <route>/device_gnss.csv --nav <route>/brdc.nav \
  --device-model <phone> --trip-id <route>/<phone> \
  --keyed-out-csv <output>.csv --summary-json <summary>.json
~~~

The current native graph is a deterministic Eigen P+D+ordinary-TDCP/float
carrier route with native SPP initialization.  It is intentionally not
described as an exact taroz/GTSAM reproduction: upstream base-station
pseudorange compensation is unavailable in a raw/nav-only contract, the
published SNR/error and residual masks are incomplete, and the state/backend
contracts differ.  These are the first high-impact missing components and
are recorded as gaps rather than hidden behind a heuristic.

The structural raw run and parser fixtures are truth-free.  A declared
development score may read only `ground_truth.csv` after the raw output is
sealed; validation, holdout, test truth, credentials, and all `.mat` paths
remain forbidden.  Production RTK/SPP defaults are unchanged.

### Native Android-RINEX FGO train gate (frozen, truth-free first)

The existing native factor-graph executable is evaluated as a separate,
development-only candidate in
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_selection.json`.
The record freezes the route-disjoint train split, source and input hashes,
and the complete Eigen recipe before any new truth read. The recipe uses
Android-derived rover RINEX, broadcast navigation, and the existing WLS POS
as initialization, with pseudorange, ordinary TDCP, position/clock motion,
and the existing default Huber loss. It explicitly disables
single-difference Doppler/TDCP, all double-difference factors, carrier-phase
ambiguity factors, and velocity states because there is no handset base
stream. A per-route 900-second/8-GiB safety bound, finite-output checks, and
same-filesystem atomic publish are enforced by
`apps/commands/benchmarks/gnss_smartphone_native_fgo_eval.py`; an invalid FGO
run is retained as a failure artifact and never silently substituted with
WLS.

Run the first truth-free route, then the remaining frozen train routes with
the exact route strings from the selection record:

~~~bash
PYTHONPATH=apps/commands:apps/commands/benchmarks \
  python3 apps/commands/benchmarks/gnss_smartphone_native_fgo_eval.py run \
  --selection-record docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_selection.json \
  --route '<dataset_id>|<obs>|<nav>|<seed_pos>|<device_gnss>|<truth>' \
  --output-root output/smartphone-r5/native-fgo-v1 \
  --binary build/apps/gnss_fgo
PYTHONPATH=apps/commands:apps/commands/benchmarks \
  python3 apps/commands/benchmarks/gnss_smartphone_native_fgo_eval.py train-score \
  --selection-record docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_selection.json \
  --output-root output/smartphone-r5/native-fgo-v1
~~~

`run` opens no truth. `train-score` first verifies all three sealed route
manifests and only then reads each train truth file once. It requires route
level non-regression against the exact WLS seed and strict aggregate H-P95
and four-diagnostic improvement. A failed train gate leaves the previously
frozen fresh validation and future holdout sealed; no public/private Kaggle
score is used for tuning and production RTK/SPP defaults are unchanged.

The frozen train gate passed. Across the three routes, exact-WLS versus native
FGO was H-P50 2.929 -> 1.981 m, H-P95 7.927 -> 4.160 m, V-P95 15.806 ->
7.871 m, and the mean of four local diagnostic variants 5.427 -> 3.069 m;
availability stayed 1.0 and route-level non-regression passed. The sealed
report and per-route run-manifest hashes are recorded in
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_evaluation.json`.

Because every train gate passed, the previously metadata-frozen fresh
validation `2023-09-07-18-59-us-ca/pixel5` was materialized in two phases:
central-directory-verified device/nav members first, then Galileo-E1
truth-free adapter and SPP seed, then native FGO. Only after the validation
FGO manifest was sealed was its ground truth materialized and read once. The
validation comparison was WLS versus native FGO: H-P50 3.509 -> 2.673 m,
H-P95 10.051 -> 3.650 m, V-P95 14.821 -> 10.526 m, and four-diagnostic mean
6.789 -> 3.164 m, with availability and truth coverage both 1.0. This is a
development-only recommendation; the future holdout remains unmaterialized
and unopened, and no post-validation tuning or Kaggle mutation was performed.
The validation authorization, truth-free materialization, and sealed report
are cross-referenced by the evaluation manifest.

Focused regression and final checks are:

~~~bash
PYTHONPATH=apps/commands:apps/commands/benchmarks \
  python3 tests/test_smartphone_native_fgo_eval.py -v
cmake --build build --config Release -j$(nproc)
LD_LIBRARY_PATH=/home/sasaki/.local/lib:$LD_LIBRARY_PATH \
  ctest --test-dir build -j1 --output-on-failure
git diff --check
~~~

### Native FGO future holdout and truth-free test batch

The future holdout `2023-09-06-00-01-us-ca-routen/pixel6pro` was evaluated
exactly once, using the recovery-v2 freeze and its central-directory member
metadata. Device/navigation payloads and adapter/SPP/native-FGO outputs were
sealed first; only then was holdout truth materialized and read once. The
candidate passed the frozen route gate: WLS versus native FGO was H-P50
3.467 -> 2.125 m, H-P95 8.340 -> 4.696 m, V-P95 17.248 -> 8.036 m, and the
four-diagnostic mean 5.908 -> 3.410 m. Availability and truth coverage were
1.0. This is development-only evidence; no source, numeric parameter, or
production default was changed after the truth read. The machine-readable
record and manifest are
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_holdout_evaluation.json`
and its `_manifest.json` companion.

Because that gate passed, the test authorization froze a 40-route allowlist,
the archive/inventory/sample hashes, the native-FGO recipe, Release binaries,
and an explicit WLS fallback. Run the truth-free batch with:

~~~bash
PYTHONPATH=apps/commands:apps/commands/benchmarks \
  python3 apps/commands/benchmarks/gnss_smartphone_native_fgo_test_batch.py \
  --authorization docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_test_batch_freeze_recovery_v2.json \
  --authorization-manifest docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_test_batch_freeze_recovery_v2_manifest.json \
  --output-dir output/smartphone-r5/native-fgo-test-v2
~~~

The completed batch processed all 40 routes and published an exact official
sample-key/order CSV with 71,936 rows, zero duplicate/missing/extra/nonfinite
keys, and no test truth access. Native FGO supplied 35 routes; five adapter
failures used the explicitly sealed sparse-WLS fallback. 69,848 rows came
from those truth-free lanes and 2,088 unresolved keys used the finite official
sample coordinate fallback; those sample coordinates are not truth and are
reported separately. The submission is a local development artifact only:
`output/smartphone-r5/native-fgo-test-v2/submission.csv`. Route/cache hashes,
wall time, and peak RSS are in the run manifest. The batch and result records
are `docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_test_batch.json`
and its `_manifest.json` companion. An initial SPP-directory orchestration
failure is retained under `native-fgo-test-v1`; recovery-v2 changed only that
directory setup and reused the test inputs without truth.

No Kaggle submission or other external mutation was made. The production
RTK/SPP defaults remain unchanged.

### Native FGO test coordinate recovery v3

The v2 test CSV is superseded for completeness purposes because its 2,088
unresolved rows used the official sample's known dummy coordinate.  The
sealed v3 recovery reads `sample_submission.csv` only for its exact header,
key set, and row order.  It never reads or copies the sample latitude or
longitude cells, and it starts no FGO/WLS process.  It reuses the hash-pinned
v2 native-FGO/WLS route artifacts plus the prior truth-free WLS route output
with this fixed priority:

1. exact native-FGO position;
2. exact v2/prior WLS position;
3. same-trip ECEF linear interpolation;
4. same-trip constant edge hold, with no velocity extrapolation.

The interpolation and edge-hold bound is 2,500 ms.  This was fixed before
generation because the sealed source audit showed a 2,004 ms leading deficit;
it is not truth-derived tuning.  Cross-trip interpolation, unresolved keys,
non-finite/out-of-Earth positions, and the known sample dummy
`34.640195,-120.589642` are rejected fail-closed.  The old 24 WLS sample
fallback keys are recorded and reconstructed from finite sealed route
positions; none uses the old formal dummy.

Reproduce the truth-free recovery and independent key/hash checks with:

~~~bash
PYTHONPATH=apps/commands:apps/commands/benchmarks \
  python3 apps/commands/benchmarks/gnss_smartphone_native_fgo_test_submission_recovery.py
PYTHONPATH=apps/commands:apps/commands/benchmarks \
  python3 tests/test_smartphone_native_fgo_test_submission_recovery.py -v
~~~

The resulting local development artifact is
`output/smartphone-r5/native-fgo-test-v3-recovered/submission.csv` (71,936
rows, SHA-256
`8581cf40a4e8ae81448df4e87b381431dce4c290b6b738fc5e6b9c03c3c41a29`).  Source
counts are native-FGO exact 61,261, v2 WLS exact 8,587, prior WLS exact 2,062,
same-trip interpolation 1, and edge hold 25.  Verification reports exact
header/key order, zero missing/extra/duplicate/non-finite keys, and zero
known-dummy rows.  The freeze, route/source manifest, and run record are
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_test_submission_recovery_freeze_v3.json`,
its `_manifest.json` companion, and
`output/smartphone-r5/native-fgo-test-v3-recovered/recovery_run_manifest.json`.
Truth access and Kaggle submission remain forbidden; production RTK/SPP
defaults are unchanged.

The sealed result record and content-address manifest are
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_test_submission_recovery_v3.json`
and its `_manifest.json` companion.  An independent post-generation quality
audit found zero out-of-Earth source positions and zero known-dummy rows.  It
flagged one same-trip consecutive step above the diagnostic 300 m/s physical
jump threshold (607.406 m over 1 s); this is retained as an audit signal only,
with no post-generation coordinate selection or parameter change.

### Native FGO test physical-continuity recovery v1

The v3 output was audited for a truth-free physical continuity issue using the
existing 70 m/s receiver-speed bound.  The rule was frozen before generation:
only an interior same-trip point whose two adjacent gaps are at most the v3
2,500 ms window, whose two incident ECEF speeds are both strictly above
70 m/s, and whose direct neighbor-to-neighbor ECEF speed is at most 70 m/s is
replaced.  Replacement is timestamp-weighted ECEF interpolation followed by
WGS84 conversion.  Route boundaries, one-sided events, plausible motion,
source seams, and adjacent candidate spikes are preserved or fail closed.

The observed 607 m/s event at
`2022-02-23-22-35-us-ca-lax-m/pixel5/1645656530438` is a prior-WLS to native-FGO
source seam, not an isolated two-sided spike: its incident ECEF speeds are
606.035 and 37.353 m/s and the direct-neighbor speed is 308.948 m/s.  It was
therefore left unchanged.  The pass repaired nine other isolated points and
left five adjacent/sequential candidates reported rather than repaired.  The
result has zero isolated two-sided spikes, zero sample-coordinate fallback,
zero known-dummy rows, and exact 71,936-row key order.  The output is a new
development-only artifact, not a replacement of v3 or a production default.

Reproduce from the sealed v3 artifact with:

~~~bash
PYTHONPATH=apps/commands:apps/commands/benchmarks \
  python3 apps/commands/benchmarks/gnss_smartphone_native_fgo_test_submission_continuity.py
PYTHONPATH=apps/commands:apps/commands/benchmarks \
  python3 -m unittest -v tests/test_smartphone_native_fgo_test_submission_continuity.py
~~~

The freeze, result record, and hash manifests are
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_test_submission_continuity_freeze_v1.json`,
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_test_submission_continuity_v1.json`,
and their `_manifest.json` companions.  The output is
`output/smartphone-r5/native-fgo-test-v4-continuity-recovered/submission.csv`
with SHA-256
`64b85bd5946ad33d77e522bf7456d5dc0c4e4870cd7b65756faedd9f18ca1af8`.
No truth member, Kaggle token, or external submission was accessed.

### Native FGO test source-seam bridge audit v1

The v4 artifact was then audited without opening test truth, rerunning FGO/WLS,
or reading sample coordinates.  A generic bridge contract was frozen first:
same trip only; source-seam/gap points only; both interpolation endpoints must
be from the same trusted native-FGO or WLS lane; endpoint separation must be
at most 10,000 ms; endpoint ECEF speed must be at most 70 m/s; all coordinates
must be finite and within the Earth ECEF norm; no sample/dummy coordinates or
velocity extrapolation; and unresolved/invalid cases fail closed.  An eligible
interior seam would be replaced in one atomic output by timestamp-weighted ECEF
interpolation.  To avoid changing already-physical points, at least one seam
boundary edge must exceed 70 m/s; otherwise the entire seam is preserved
byte-for-byte.  The contract does not change estimator parameters.

The audit enumerated all 19 v4 transitions above 70 m/s.  There were zero
isolated two-sided candidates after v4.  The highlighted
`2022-02-23-22-35-us-ca-lax-m/pixel5/1645656530438` event remains a prior-WLS
to native-FGO seam: its incident speeds are 606.035177 and 37.352942 m/s.
The native-FGO same-lane bracket is 58,000 ms (outside the 10-second bound),
and the prior/current-WLS byte-identical same-lane bracket is 2,000 ms but has
312.017226 m/s endpoint speed.  The remaining transitions are same-lane
high-speed or sequential events and are reported unchanged.  Four other
prior-WLS runs were eligible: 3, 1, 1, and 2 interior rows respectively, with
native-FGO endpoint brackets of 2–4 seconds and 16.118–37.643 m/s.  Those
seven rows were replaced by same-lane timestamp-weighted ECEF interpolation;
the highlighted 607 m/s seam was not changed.

The resulting v5 artifact has 14 (down from 19) >70 m/s edges and 18 (down
from 28) one-sided violation points, with zero isolated two-sided spikes.  It
has exact 71,936-row key order, zero dummy/sample fallback, and zero
non-finite/out-of-Earth coordinates.  It is a development-only candidate;
v4 remains retained as the sealed input baseline.

The target ±10-second inventory (12 official keys, chosen source and ECEF /
geodetic coordinates, and all native-FGO/prior-WLS alternatives) plus the
19-transition list is sealed in
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_test_submission_source_seam_bridge_v1.json`
and its `_manifest.json` companion.  The pre-decision contract and input
hashes are sealed in
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_test_submission_source_seam_bridge_freeze_v1.json`
and its `_manifest.json` companion.  The freeze record SHA-256 is
`eef25620633cdd7086dbc78b22a6705f4e5043b81301ef7eea03528f8f794f48`; its
manifest SHA-256 is
`a8de1dd300597465151cabfc6f35a9aa7eeff048a65d37eba6a39dc4cf9864c8`.
The source-seam bridge result record SHA-256 is
`ff90e3f9e2164765503a7a5c4ea2a794851db532e9e89a257b40e9dd6840ca8e`.
The v5 output SHA-256 is
`89de0e03ff8eb687c9228ac19a1e62d160400b317352652cf8665b22591eef26`.
Its output and run manifest hashes are
`dbba0ebe190d99aaeb1163405bd49f19bd7dc449be76241880f0d11f7e43c519` and
`89330f31ec5bde1575c664f108c4a1c71744f24810c937051cdf440a4f768d1b`.
The retained v4 submission SHA-256 remains
`64b85bd5946ad33d77e522bf7456d5dc0c4e4870cd7b65756faedd9f18ca1af8`.
Truth access, token access, Kaggle mutation, and production-default changes
remain disabled.

### Native FGO v2 IMU/preintegration feasibility smoke

An independent v2 lane was frozen after a central-directory-only inventory of
the archive.  The conservative truth-use audit excludes every route/device
with prior truth evidence.  The new train identities are
`2021-03-16-18-59-us-ca-mtv-a/pixel5`,
`2021-07-27-19-49-us-ca-mtv-b/pixel4`, and
`2022-02-24-18-29-us-ca-lax-o/pixel5`; fresh validation is
`2023-05-09-21-32-us-ca-mtv-pe1/pixel5`, and the future holdout is
`2023-05-16-19-54-us-ca-mtv-xe1/pixel5`.  The split and central metadata were
frozen before any selected payload was opened in
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_v2_split_inventory_freeze.json`.
The candidate recipe and its SHA-pinned manifest are
`smartphone_r5_gsdc2023_native_fgo_v2_candidate_freeze.json` and its
`_manifest.json` companion.

The bounded command materializes only train `device_gnss.csv`,
`device_imu.csv`, route `brdc.nav`, and route reference RINEX.  It converts the
Android `UncalAccel`/`UncalGyro` schema to the existing C++ IMU loader using a
fixed nearest-acceleration 25 ms bound (gyro timestamps anchor; no
interpolation or extrapolation), then runs the existing
`gnss_fgo_parity --float-only --max-epochs 2` GTSAM Pose3 path.  This is actual
preintegration/`CombinedImuFactor` wiring, not the earlier motion-Q smoother.
The fixed truth-free offset diagnostic searches -100…+100 ms in 10 ms steps
using raw WLS speed/IMU activity correlation.  Because parity exposes no safe
offset option, the estimate is never silently applied; invalid or insufficient
observability records the frozen native-FGO-v1 fallback.

Reproduce the sealed truth-free train smoke with:

~~~bash
python3 apps/commands/benchmarks/gnss_smartphone_native_fgo_v2_smoke.py \\
  --output-root output/smartphone-r5/native-fgo-v2-processed
python3 tests/test_smartphone_native_fgo_v2_smoke.py -v
~~~

All three new-train two-epoch runs passed the structural gate: adapter truth
use was false, finite IMU samples were loaded, one IMU interval was inserted,
GTSAM graph values/factors were positive, convergence was reported, and no
non-finite solution occurred.  The exact route metrics, offset diagnostics,
conversion hashes, wall/RSS, and route logs are in
`output/smartphone-r5/native-fgo-v2-processed/train_smoke_manifest.json` and
the three route manifests.  The train smoke is not an accuracy/promotion
gate: the current parity binary is base-assisted, has no offset-application
option, and emits no reusable smartphone position stream.  Therefore fresh
validation remains unopened, the future holdout remains untouched, no FGO
parameter was tuned, and production RTK/SPP defaults are unchanged.  The
machine-readable blocker/result record is
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_v2_smoke.json`.
The raw archive also does not independently validate each handset's IMU
mounting axes against body-FLU; the frozen identity convention is therefore an
explicit assumption, not evidence of physical inertial correctness, and remains
a promotion blocker.

### Native FGO P/D/C bridge (development No-Go)

The existing native no-base graph already had pseudorange, ordinary TDCP,
float carrier-arc, position/clock motion, and ambiguity states.  The smallest
missing published-factor bridge was an undifferenced receiver Doppler factor:
its residual is the measured range-rate minus satellite range-rate and clock
drift, with the line-of-sight velocity Jacobian.  This is implemented behind
the default-off `--undifferenced-doppler-factors` switch; base, SD/DD, integer
ambiguity, production RTK, and production SPP defaults are unchanged.  The
bridge rejects the GTSAM backend for this switch rather than silently ignoring
it, and falls back only when a caller supplies an explicitly hash-authorized
artifact.

The upstream comparison is pinned to MIT `taroz/gsdc2023` commit
`29923f9f370f09ebc00f96d8cca375007a18e7d5` (repository MIT license SHA
`f5b3bcb0ce8267e076b4dad74c67c02efab4161efd6c5ba208402a0f51fbea1b`).
The relevant upstream files are `fgo_gnss.m` (SHA
`5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3`),
`fgo_gnss_imu.m` (SHA
`c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3`), and
`parameters.m` (SHA
`518925e9c75c7a14fceb5cd99432fe883311e0d64c72b94396744ac765120f52`).
Their PR/Doppler/TDCP/motion contracts were used as evidence; no MATLAB or
truth-derived parameter was copied into the production path.

Before scoring, the route-disjoint development recipe was frozen in
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_pdc_bridge_freeze_v1.json`
and its manifest.  The three development identities were
`2021-03-16-18-59-us-ca-mtv-a/pixel5`,
`2021-07-27-19-49-us-ca-mtv-b/pixel4`, and
`2022-02-24-18-29-us-ca-lax-o/pixel5`.  Their truth files had already been
read by an earlier development experiment; that historical reuse is recorded
in the corrected truth-use inventory and is not represented as a fresh split.
All truth-free candidate and exact P/TDCP/motion baseline artifacts were
sealed before the score.  Reproduce the fixed comparison with:

~~~bash
python3 apps/commands/benchmarks/gnss_smartphone_native_fgo_pdc_bridge_train_score.py \
  --gate-record docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_pdc_bridge_train_gate_v1.json
~~~

The resulting report is
`output/smartphone-r5/native-fgo-pdc-bridge-v1/train_evaluation.json` (SHA
`b48a9c2b750f5f57955f8bc7ca648a7664095b3078437e11ce77be6c7a87b1af`).  The
candidate was finite and complete, but did not converge within the frozen
eight iterations and failed every route and aggregate promotion gate.  The
aggregate baseline versus candidate was H P50 `2.347` m versus `46985.036` m,
H P95 `5.582` m versus `129232.654` m, V P95 `19.369` m versus `121009.307`
m, with the four diagnostic means `3.975` m versus `88069.002` m.  Candidate
availability and truth coverage were unchanged, but it introduced additional
over-70 m/s transitions.  This is a sealed No-Go; fresh validation, holdout,
test, token, leaderboard tuning, and external mutation remain unopened.
The machine-readable result and artifact references are in
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_pdc_bridge_result_v1.json`.

### Native FGO PDC correctness recovery (truth-free structural No-Go)

The PDC v1 failure was audited against Android's primary API contract before
changing code. `getPseudorangeRateMetersPerSecond` is an uncorrected m/s
quantity, positive when the satellite is moving away, and its uncertainty is
one sigma in m/s; the API also defines pseudorange rate as `-k*doppler_shift`.
The adapter therefore keeps the exact mapping
`D=-PseudorangeRateMetersPerSecond/wavelength`, while libgnss++ stores RINEX
`D` in cycles/s (Hz). The local reference implementation in
`src/algorithms/spp_velocity.cpp` independently uses `-D*c/frequency`, solves
receiver ECEF velocity plus clock drift in m/s, subtracts satellite clock
drift, and rotates satellite position and velocity consistently for signal
travel time. The source URLs, pinned taroz commit and pre-change hashes are
in `docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_pdc_correctness_authorization_v1.json`.

The demonstrated old-graph defect was the absence of an explicit receiver
clock-drift term in the no-base undifferenced Doppler row, together with the
legacy unrotated/Sagnac approximation. The opt-in correction adds the receiver
clock-bias finite difference `(clock[k]-clock[k-1])/dt` and rotates satellite
position and velocity together. It is behind
`--corrected-undifferenced-doppler-factors`; the default and all production
RTK/SPP defaults remain unchanged. Contract tests cover approaching and
receding signs, wavelength conversion, stationary receiver clock drift,
receiver velocity/clock Jacobians by finite differences, and paired Earth
rotation. Focused execution passed 6/6.

Reproduce the truth-free route artifacts with the frozen wrapper:

~~~bash
python3 apps/commands/benchmarks/gnss_smartphone_native_fgo_pdc_bridge.py \
  --freeze-record docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_pdc_corrected_freeze_v1.json \
  --corrected-undifferenced-doppler \
  --candidate-id native-fgo-pdc-corrected-v1 \
  --route <fixed-train-route> --max-epochs 0 \
  --output-root output/smartphone-r5/native-fgo-pdc-corrected-v1
~~~

All three fixed train routes produced finite, complete, truth-free outputs and
monotonically decreasing costs, with 32,715/26,252/29,011 corrected Doppler
factors. They did not converge within the frozen eight iterations and still
showed 49.763/7,268.358/138.842 m/s Doppler residual RMS, maximum transition
speeds of 1,929.737/303,760.093/6,233.486 m/s, and 40/396/63 transitions over
the fixed 70 m/s physical bound. The corrected lane is consequently a sealed
structural No-Go; no development truth score was opened, and no validation,
holdout, test, leaderboard or parameter-tuning step is authorized. Full route
hashes and the exact failure gate are in
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_pdc_corrected_structural_result_v1.json`,
with the companion hash manifest
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_pdc_corrected_structural_result_v1_manifest.json`.

The truth-free real-data witness can be reproduced without a truth file:

~~~bash
python3 apps/commands/benchmarks/gnss_smartphone_native_fgo_pdc_witness.py \
  --device-gnss <development-device_gnss.csv> \
  --route-id <route/device> --output-json <witness.json>
~~~

It uses only raw handset WLS ECEF and broadcast-state-derived satellite
position/velocity/clock fields, removes a per-epoch common clock term for this
diagnostic, and never selects a mapping. Across the three routes the correct
Android mapping and its RINEX round-trip had clock-centered P95 residuals of
1.365/1.670/3.436 m/s; a sign inversion gave
1,379/1,338/1,405 m/s and treating Hz as m/s gave
4,092/4,050/4,347 m/s. The formal v5 submission has keyed latitude/longitude
only and no per-route ECEF state, so an exact “v5 ECEF plus ephemeris” witness
is explicitly blocked rather than inferred. The witness outputs are marked
diagnostic-only and are not an accuracy score.

### Native FGO Doppler-velocity WLS initializer (development No-Go)

After the corrected-Doppler structural audit, a separate opt-in candidate used
the exact corrected receiver-only rows already present in the graph,
`r_i = los_i · v_ecef + clock_rate`, as a per-epoch weighted linear solve. It
used uncertainty weights, deterministic three-step Huber IRLS, SVD rank and
condition gates, diagonal covariance, a 4-sigma robust RMS gate, a 25-sigma
absolute residual gate, and a 70 m/s receiver-speed gate. Short missing spans
were completed only by same-series bounded interpolation/constant hold; a
clock discontinuity or long gap failed closed. Valid covariance initialized the
velocity states and supplied velocity/clock-rate priors. The switch is
`--doppler-velocity-wls-initialization`; it is default-off and requires the
corrected undifferenced-Doppler and Eigen paths. No production RTK/SPP default
was changed.

The freeze and exact input/source/binary hashes are in
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_pdc_wls_freeze_v1.json`.
Reproduce a truth-free short smoke with:

~~~bash
python3 apps/commands/benchmarks/gnss_smartphone_native_fgo_pdc_wls.py \
  --freeze-record docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_pdc_wls_freeze_v1.json \
  --route <frozen-train-route> --max-epochs 30
~~~

The three 30-epoch structural smokes passed with WLS valid/rejected counts
30/0, 30/0, and 30/0; max condition numbers were 8.48, 9.01, and 8.42; max
robust RMS values were 0.856, 3.928, and 1.383; and max speeds were 0.497,
8.005, and 1.122 m/s. Full truth-free runs also produced 2,159/1,678/2,439
complete keyed epochs with no rejected WLS epoch. After those artifacts were
sealed, the development-only score gate read the three already authorized
development truth files once each. The candidate regressed badly against the
sealed native-FGO baseline: aggregate H P50 `2.347` -> `22,254.393` m, H P95
`5.582` -> `41,536.823` m, V P95 `19.369` -> `46,533.605` m, and diagnostic
mean `3.975` -> `31,892.948` m. Route-level continuity also introduced
over-70 m/s transitions. This is a sealed train No-Go; the fresh validation,
future holdout, test, Kaggle, and token paths remained unopened.

The score gate and sealed report are
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_pdc_wls_train_gate_v1.json`
and `output/smartphone-r5/native-fgo-pdc-wls-v1/train_evaluation.json` (SHA256
`2ebb511ca4cb0092cc3c2f15f05a6239a03b0d993ea3530c6537db0caf8f9421`). The
candidate remains development-only and is not promoted or used for submission.
The machine-readable sealed decision is
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_pdc_wls_train_result_v1.json`
(SHA256 `696fd5ae82d7d890529672f1d14116a3f9089d90a3bd58f699a4b6c2a9888fb2`);
its score manifest is
`output/smartphone-r5/native-fgo-pdc-wls-v1/train_evaluation.manifest.json`
(SHA256 `dcf8a1003b9030f4ca3167be588a332c4a920c20e047bf204f4365d669e82963`).

### PDC factor-by-factor structural audit (truth-free)

The PDC graph was audited without reopening truth or rerunning any score. The
pre-audit freeze is
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_pdc_factor_audit_freeze_v1.json`
(SHA256 `a1d9a6f9d7018cb0bdf9736e9886d19865d3476ae1547281632833da90a3eac2`).
It pins the already materialized development route
`2021-03-16-18-59-us-ca-mtv-a/pixel5`, its route manifest and observation/nav
hashes, the existing Release `gnss_fgo` binary (SHA256
`a10d13da4614f7cdc8e24556f5a32f870aee6c7aa94c5d3e890429996d583b0c`), and a
fixed A--F matrix: P only; P+D with velocity/clock priors and no temporal
factors; P+motion; P+D+WLS velocity/clock initialization+motion; then TDCP;
then float carrier/ambiguity-between. A is run at one epoch, B--F at two
epochs, and C--F are repeated at 30 epochs. All rows use the existing
corrected no-base Doppler contract and `--no-dd-factors`; no candidate
parameter was searched or changed.

The sealed truth-free output is
`output/smartphone-r5/native-fgo-pdc-factor-audit-v1/factor_audit.json`
(SHA256 `b0b18311ef253010240b3cfc7df4854b4f5ca81d79f217ee47522f02b63b0847`),
with manifest SHA256 `257744c4cde33612d5d0574531662bb81b95c28749a77b24ddc4d24a15324f8e`.
All ten stages were finite and complete, with monotonically decreasing native
cost traces and no >70 m/s neighboring-position transition. The first
velocity/clock-enabled stage D converged in 7 iterations at two epochs; D,
E, and F all converged in 8 or fewer iterations at 30 epochs. The 30-epoch
factor coverage was respectively:

* C: 445 P + 29 motion rows;
* D: 445 P + 430 corrected D + 29 motion rows, WLS valid 30/30, max speed
  `0.482` m/s and D residual RMS `0.137` m/s;
* E: the same plus 337 TDCP rows, TDCP RMS `0.027` m and D RMS `0.381` m/s;
* F: the same plus 356 float-carrier rows and 14 ambiguity states, carrier RMS
  `0.021` m and D RMS `0.410` m/s.

The audit records initial/final global robust graph cost, factor counts and
family RMS/quadratic-equivalent costs, termination/cost trace, ECEF seed
delta/position and velocity magnitudes, and the exact weighted P-row
Jacobian rank/condition/column norms. For example, the 30-epoch P geometry
has 445 rows, 122 reconstructed columns, rank 121, and condition number
`34.6704`; the 2-epoch P geometry has rank 9 of 10 columns because the
clock/bias gauge is visible. Clock bias/drift and final ambiguity values are
intentionally recorded as unavailable: the frozen `gnss_fgo` epoch/POS output
contract exposes neither private state, and the audit does not infer them.
Likewise, the checkout has no sealed taroz dogfood artifact, so exact numeric
factor/Jacobian parity is reported unavailable rather than replaced with a
surrogate. The result is structural evidence only: no new truth score,
validation, holdout, test, Kaggle access, or production/default change is
authorized by this audit.

Reproduce the sealed audit with:

~~~bash
python3 apps/commands/benchmarks/gnss_smartphone_native_fgo_pdc_factor_audit.py \
  --freeze-record docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_pdc_factor_audit_freeze_v1.json \
  --output-root output/smartphone-r5/native-fgo-pdc-factor-audit-v1 \
  --binary build/apps/gnss_fgo
~~~

The follow-up hypothesis audit is sealed at
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_pdc_factor_hypothesis_audit_v1.json`
(SHA256
`6ae38af2440b2040144a241fb47715bcc5df8acae5cad9e8b0228c1dd619d698`),
with manifest SHA256
`f1560078082e590325320213e3512ac48db045d526c29f2a357499a5502f015f`.
It records the source-line hypotheses tested from the same sealed artifacts:
absolute-ECEF conditioning, clock metres-versus-seconds, clock-rate `dt`,
velocity/LOS frame, satellite/Sagnac term, motion sigma/gauge, and duplicate
TDCP/carrier constraints. No source or parameter was changed. The P geometry
remains finite (30-epoch rank 121/122, condition `34.6704`), D converges with
30/30 valid WLS epochs and RMS `0.137 m/s`, and the first observed degradation
is the finite, convergent TDCP addition (D RMS `0.137` to `0.381 m/s`). Since
the taroz dogfood directories are absent, exact real-row parity remains
unavailable and is not replaced by a surrogate.

### Published/windowed PDC bridge (truth-free, opt-in)

The published checkout was audited before this lane was run. `run_fgo.m` makes
independent `fgo_gnss_imu` calls; the pinned MATLAB graph uses a batch
Levenberg--Marquardt optimizer with `setMaxIterations(1000)` and exposes no
stride, overlap, state handoff, or marginalization contract. The local
`IncrementalFixedLagSmoother` is intentionally not used: its dispatch requires
the Pose3+IMU/DD graph (`src/algorithms/fgo_gtsam_backend.cpp:38-43`), whereas
this candidate is the no-base Eigen P/D/TDCP/carrier graph. Exact source lines
and hashes are recorded in
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_pdc_windowed_freeze_v1.json`.

The development-only wrapper
`apps/commands/benchmarks/gnss_smartphone_native_fgo_pdc_windowed.py` therefore
solves independent 120-epoch windows with 60-epoch overlap and deterministic
interior-row stitching. Each invocation resets clock, velocity initialization,
TDCP history, and float ambiguity arcs. It uses the existing corrected native
P+D+TDCP+float-carrier recipe, max 1000 iterations, relative/absolute cost
stops `1e-6/1e-9`, Huber thresholds 4 sigma, and no base/DD factors. A failed
window or unsafe >70 m/s stitch falls back atomically to the already sealed
native-FGO route positions; sample coordinates and truth are forbidden.

Run the structural lane only with the explicit guard:

~~~bash
GNSSPP_WINDOWED_PDC_NO_TRUTH=1 \
python3 apps/commands/benchmarks/gnss_smartphone_native_fgo_pdc_windowed.py \
  --no-truth
~~~

The freeze hash manifest is
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_pdc_windowed_freeze_v1_manifest.json`.
The generated truth-free output is resume-safe at
`output/smartphone-r5/native-fgo-pdc-windowed-v1/`; its manifest records every
window command, summary/factor coverage, source hash, fallback decision,
continuity result, wall time, and peak child RSS. No validation, holdout, test,
Kaggle, or production-default path is opened or changed by this lane. The
windowed parity test remains optional and skips when the upstream MATLAB/C++
dogfood directories are absent.

On the sealed route, the run produced 35 windows: 33 complete/converged
windows and two native `FGO produced no valid epochs` windows (starts 1140 and
1200). The first overlap seam also reached 218.551 m/s between window 12 and
13, exceeding the frozen 70 m/s physical gate. The lane is therefore a sealed
No-Go and published `windowed.pos` is byte-identical to the sealed fallback;
no truth score was attempted. The result record is
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_pdc_windowed_result_v1.json`
(SHA256 `8dc912a06b839ffcc091b54100f1ba40c869d2948aa7ec07f7b2b7e1a8349bb7`;
windowed manifest SHA256 `48b019d36e1211caf6040e21b5d10bcf5352742170e7623712b153bb2d0d772f`).
An orchestration-only path-reference repair is separately recorded in
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_pdc_windowed_manifest_repair_v1.json`;
it did not rerun the solver or change any artifact bytes (repair record SHA256
`e51cb485dd5dbe306cf4d91aeeff4fa4cc81793bf148de9fd90a41c9e32c0b75`).

### Truth-free ECEF overlap stitch recovery (sealed No-Go)

The next opt-in post-processing candidate reused the sealed v1 window files;
it did not invoke `gnss_fgo`.  The freeze record
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_pdc_windowed_stitch_freeze_v1.json`
(SHA256 `40ef733d28cebd7d72710a122a2ec5cfe72a6c97a93480eb41a582b4f990dac6`)
pins the source, v1 manifest, fallback, window geometry, and the generic
contract.  For each timestamp, structurally valid windows contribute a
discrete triangular weight (edge floor 1, maximum at the center); finite ECEF
coordinates are blended and converted to WGS84 only for export.  A window is
rejected wholesale for an internal gate failure or overlap disagreement above
`3 * 50 m + 70 m/s * timestamp_span`; incompatible clock segments are never
blended.  Missing/unsafe rows fall back to the exact sealed v5 route position.
No translation, truth, sample coordinate, extrapolation, or parameter search is
permitted.

The truth-free artifact is atomically sealed at
`output/smartphone-r5/native-fgo-pdc-windowed-v2-stitch/`:
`stitch_manifest.json` SHA256
`39e4c99681c604e37e41c0443f397a451875245156f42e10c95e60f369e6dbdb`, and
`windowed_v2.pos` SHA256
`e9be773d6c7d10323f10c3ece4d4e1e5ed1edc7568c4b9a5221a6cb856a8105d` (2,159
rows).  The structural gate passed: exact keys, finite/in-Earth coordinates,
zero dummy/sample rows, zero unsafe transitions, and maximum speed `44.306`
m/s.  Source counts were 1,680 triangular blends, 359 unchanged
single-window rows, and 120 exact v5 fallback rows; windows 12 and 13 were
rejected after a `192.845 m` overlap disagreement exceeded the frozen `150 m`
bound.

After that structural pass, the fixed existing development-train route was
scored exactly once against its sealed v5 route position using all four
official diagnostic variants.  The score authorization and evaluator are
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_pdc_windowed_stitch_train_score_authorization_v1.json`
and
`apps/commands/benchmarks/gnss_smartphone_native_fgo_pdc_windowed_stitch_eval.py`.
The score read the already-authorized truth file once (SHA256
`7c84ed6a80b1bbb08c0ffad57493513833b9d5474e22a43c5a44da82824ee22d`) and never
opened validation, holdout, or test truth.  v5 → stitch was H P50
`1.979 -> 3.044 m`, H P95 `2.977 -> 8.127 m`, V P95 `6.311 -> 14.189 m`, and
four-diagnostic mean `2.485 -> 5.591 m`; availability and truth coverage both
remained `1.0`.  Every diagnostic regressed, so the result is a sealed No-Go:
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_pdc_windowed_stitch_result_v1.json`
(SHA256 `3dedbd06513cb6db3631809ac06e014579ce69084ce637478c94334554c52d71`).
The lane is not promoted, production defaults and v5 are unchanged, and no
validation/holdout/test batch is authorized.

### Historical precomputed-result experiment rejected from native lane

An earlier branch contained a converter, fallback, and one external submission
based on saved upstream result artifacts.  That experiment is not a
libgnss++ inference result and cannot establish the native 0.782-class target.
Its implementation, test registrations, and active command aliases have been
removed from this branch.  Historical records may remain in repository history
for auditability, but they are not runtime dependencies, are not native
evidence, and must not be used as an inference input.  The native contract
rejects any `.mat` path before opening it; no conversion, oracle comparison,
test batch, or resubmission is permitted.

The current native best remains the separately recorded v5 result
(public/private `3.952/4.276`), and the native 0.782-class target remains
unachieved.  Continue from the raw-only bridge and gap matrix above; do not
reinterpret the historical imported score as a native improvement.
