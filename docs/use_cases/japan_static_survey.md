# Japan static survey: IGS Tsukuba relative static + PPP

This route is a reproducible, auditable demonstration for the IGS Tsukuba
stations `TSK200JPN` (rover) and `TSKB00JPN` (base), using the 30-second
observation window beginning at 2024-01-01 00:00:00 GPS time. A full day has
2,880 nominal observation epochs; the smoke mode processes the first 20
epochs. The command produces two independent lanes:

1. a relative static RTK solution (`gnss solve`); and
2. a final-product static PPP solution (`gnss ppp`) using SP3/CLK and the
   receiver ANTEX model.

The route is currently a demonstration and evidence-collection surface. It
is not a control-point certification or a claim of survey accuracy.

## One command

From the repository root, materialize the pinned inputs and run the 20-epoch
smoke test:

```bash
python3 apps/gnss.py japan-static-survey \
  --output-dir output/use_cases/japan_static_survey/smoke \
  --mode smoke
```

Use `--mode full` for the one-day, 2,880-epoch lane, or
`--materialize-only` to fetch/verify/convert inputs without invoking either
solver:

```bash
python3 apps/gnss.py japan-static-survey \
  --output-dir output/use_cases/japan_static_survey/full \
  --mode full

python3 apps/gnss.py japan-static-survey \
  --output-dir output/use_cases/japan_static_survey/materialized \
  --materialize-only
```

`--offline` makes a cache miss or any size/SHA-256 mismatch fail closed. The
default cache is `~/.cache/libgnsspp/japan-static-survey`; use `--cache-dir`
to make it a reviewable project artifact. A failed fetch, conversion, or
child command leaves `bundle.log` and `manifest.json` and exits nonzero.

The default `--dataset development` is the 2024-01-01 fixture described below.
The command also has a separately pinned `--dataset holdout` for 2024-01-02:

```bash
python3 apps/gnss.py japan-static-survey \
  --dataset holdout \
  --output-dir output/use_cases/japan_static_survey/holdout \
  --mode full
```

Run the holdout once, only after the development-day candidate and thresholds
are frozen. Do not tune or rerun against it; its manifest records the sealed
holdout role.

Each successful bundle contains:

```text
relative_static.pos       relative_static.kml       relative_static_trajectory.png
ppp_static.pos            ppp_static.kml            ppp_static_trajectory.png
relative_summary.json     ppp_summary.json          bundle.log
manifest.json
```

The manifest records the exact child argv, source URLs and pinned hashes,
materialized hashes, receiver antenna type, truth solution IDs/epochs/frame,
height and point-reference contracts, and every step exit status.

## Input and provenance contract

The default sources are fixed to this R2 fixture:

| Input | Source | Purpose |
|---|---|---|
| `TSK200JPN...MO.crx.gz`, `TSKB00JPN...MO.crx.gz` | BKG IGS archive | rover/base observations |
| `BRDC00IGS...MN.rnx.gz` | BKG IGS archive | broadcast navigation |
| `IGS0OPSFIN...ORB.SP3.gz`, `...CLK.CLK.gz` | BKG IGS final products, GPS week 2295 | PPP precise orbit/clock |
| `IGS0OPSSNX_2023365...SOL.SNX.gz` | SOPAC/Garner mirror of the IGS weekly combination | independent station truth |
| station logs | `files.igs.org/pub/station/log/` | station/receiver history audit |
| `IGS20.ssc`, `igs20.atx` | `files.igs.org` | reference-frame and antenna-model provenance |

The command verifies the pinned byte count and SHA-256 before using every
source. Compact RINEX is converted with a system `CRX2RNX` when available;
on Linux x86-64 it otherwise downloads the official GSI RNXCMP 4.2.0 binary,
verifies its pinned archive hash, safely extracts the `CRX2RNX` member, and
sets the cached copy executable. Other platforms fail closed unless a system
converter is installed. There is no silent text-file or header fallback.

IGS products are openly distributed best-effort data; the application should
acknowledge the International GNSS Service and the BKG product source. The
CompactRINEX conversion is the Geospatial Information Authority of Japan
(GSI) RNXCMP tool. Product availability, mirror freshness, and license terms
remain external operational dependencies and are recorded in the manifest,
not inferred as a warranty by this repository.

## Independent truth and Japanese coordinate contract

Accuracy comparisons must never use `APPROX POSITION XYZ` from a RINEX
header. The independent truth reader selects exactly one station solution
from `SOLUTION/EPOCHS` whose validity contains the observation date, then
reads only the matching `STAX`, `STAY`, and `STAZ` rows from
`SOLUTION/ESTIMATE`. It never reads `SOLUTION/APRIORI`; missing, ambiguous,
duplicate, or out-of-window records are errors.

The pinned weekly solution contains:

| Station / solution | Validity selected | Estimate epoch | ECEF X (m) | ECEF Y (m) | ECEF Z (m) |
|---|---|---|---:|---:|---:|
| `TSK2` / soln 11 | 2024-01-01 through 2024-01-07 | 2024-01-03 12:00:00Z | -3,957,184.96826790 | 3,310,231.00877522 | 3,737,703.81925572 |
| `TSKB` / soln 12 | 2024-01-01 through 2024-01-06 23:59:59Z | 2024-01-03 12:00:00Z | -3,957,200.08745802 | 3,310,199.00772871 | 3,737,711.49330339 |

The comparison frame is IGS20/weekly SINEX. ECEF-derived ellipsoidal heights
are approximately 70.0107 m (`TSK2`) and 67.3134 m (`TSKB`); these are not
orthometric heights and no geoid transformation is applied. The coordinates
are station solution points, not an unqualified ground-control monument.

The pinned weekly file has XYZ estimates at its mean epoch but no `VELX`,
`VELY`, or `VELZ` rows for these stations. Therefore this implementation
records the estimate epoch explicitly and does not claim a dynamic
position-plus-velocity propagation from 2024-01-03 to 2024-01-01. A
velocity-capable weekly truth product or an independently approved
propagation source is required before freezing a time-propagated accuracy
threshold or using this route as a control-point sign-off. This boundary is
intentional: a static coordinate at the wrong epoch must not be presented as
survey truth.

`IGS20.ssc` is retained for provenance only. It is not substituted as truth:
the selected station/solution validity and independent weekly estimate are
the authoritative contract for this fixture.

## ARP, marker, PCO/PCV, and lane boundaries

The materialized rover RINEX identifies `TRM159900.00    NONE`; the base
identifies `AOAD/M_T        DOME`. The PPP child receives the rover's exact
`ANT # / TYPE` value together with `--antex .../igs20.atx`, so receiver PCO/PCV
corrections are selected by the solver's ANTEX lookup. The station coordinate
and the RINEX antenna reference are not silently converted into a monument
coordinate, and the antenna delta is not treated as an orthometric-height
correction.

The relative static lane does not pass ANTEX to `gnss solve`. It is therefore
useful as a short-baseline demonstration, but it is **not** a certified
control-point result. Only the PPP lane has the explicit receiver ANTEX
contract, and even that lane remains subject to the truth-epoch limitation
above and local survey authority review.

## Output metrics and gates

Both summaries include observation count, solution state, satellite count,
ECEF/ENU error against the independent truth, horizontal/vertical/3D error,
and empirical epoch repeatability covariance. The covariance is explicitly
labelled empirical coordinate repeatability, not the solver's posterior
covariance; the `.pos` format does not serialize the full posterior matrix.

The smoke gate is intentionally a wiring check: relative fixed rate at least
70%, PPP solution rate at least 90%, and the stated minimum satellite counts.
The full candidate adds horizontal/vertical limits (relative 0.20/0.40 m,
PPP 2.0/4.0 m) and requires all 2,880 nominal epochs. These are provisional
review thresholds, not frozen survey limits, because the pinned truth file
does not provide velocities for epoch propagation. Any nonzero gate exit,
missing artifact, failed converter, or truth metadata error is a failed
result rather than an inferred pass.

The frozen executions on 2026-08-24 produced:

| Dataset | Lane | State rate | Horizontal / vertical error | Convergence / initial risk | Decision |
|---|---|---:|---:|---|---|
| 2024-01-01 development | Relative | FIX 99.861% | 0.0089 / 0.0111 m | max epoch 3D 0.0298 m | demonstration only: RTK ANTEX is not applied |
| 2024-01-01 development | PPP | solution 100% | 1.345 / 0.168 m | converged after 5,940 s; max epoch 3D 44.3 m | candidate after convergence; authority review required |
| 2024-01-02 sealed holdout | Relative | FIX 99.861% | 0.0084 / 0.0106 m | max epoch 3D 0.0250 m | demonstration only: RTK ANTEX is not applied |
| 2024-01-02 sealed holdout | PPP | solution 100% | 1.945 / 0.409 m | converged after 3,780 s; max epoch 3D 625 m | candidate after convergence; authority review required |

The holdout was run once with unchanged gates and was not used for tuning.
The large pre-convergence PPP errors prohibit treating the first valid
solution as a measured control point. Neither lane is promoted here as a
Japanese legal/survey control-point workflow.

For a future release, freeze thresholds only after:

1. an independent truth product with a valid velocity/epoch propagation is
   supplied;
2. the full 2,880-epoch output is reviewed in both lanes;
3. the receiver antenna PCO/PCV lookup is confirmed in the PPP summary/log;
4. Japanese ellipsoidal-versus-orthometric height handling is agreed with the
   survey authority; and
5. one separately authorized time window is held out without tuning against
   it.
