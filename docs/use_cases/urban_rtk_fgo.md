# Urban driving: bridge RTK outages with IMU, NHC, and ZUPT

Use this route for a road vehicle that needs a continuous trajectory through
urban canyons, under trees, or in short tunnels. The accuracy contract has two
distinct states:

| State | Position source | Practical interpretation |
|---|---|---|
| RTK `FIXED` | Carrier-phase RTK | Centimetre-class is plausible after it has been checked against truth |
| RTK degraded or absent | IMU propagation, optionally constrained by NHC/ZUPT | Continuity bridge; uncertainty and drift grow with time |

IMU, NHC, and ZUPT do **not** turn an RTK outage into centimetre-accurate FIX.
They keep the trajectory continuous until trustworthy GNSS can anchor it
again. This distinction is the core of the recipe.

## Inputs and first artifact

The public PPC Tokyo runs already have the required files:

```text
data/PPC-Dataset/tokyo/run1/
  rover.obs  base.obs  base.nav  imu.csv  reference.csv
```

Build the existing fusion command and check the dataset before starting:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target gnss_fuse --parallel 2

test -s data/PPC-Dataset/tokyo/run1/rover.obs
test -s data/PPC-Dataset/tokyo/run1/base.obs
test -s data/PPC-Dataset/tokyo/run1/base.nav
test -s data/PPC-Dataset/tokyo/run1/imu.csv
mkdir -p output/use_cases/urban_fusion
```

Run a 200-epoch wiring check. It writes the pre-fusion RTK stream and the
continuous fused stream from the same observations in one pass:

```bash
python3 apps/gnss.py fuse \
  --data-dir data/PPC-Dataset/tokyo/run1 \
  --lever-arm 0.31,0,-0.55 \
  --preset low-cost \
  --zupt --no-nhc \
  --max-epochs 200 \
  --rtk-pos-out output/use_cases/urban_fusion/rtk.pos \
  --out output/use_cases/urban_fusion/fused_zupt.pos \
  2>&1 | tee output/use_cases/urban_fusion/fused_zupt.log

python3 apps/gnss.py pos2kml \
  output/use_cases/urban_fusion/fused_zupt.pos \
  output/use_cases/urban_fusion/fused_zupt.kml \
  --status all
```

`--lever-arm x,y,z` is the IMU-to-antenna offset in the PPC vehicle coordinates,
in metres. Tokyo's published PPC value is `0.31,0,-0.55`; keep its signed Z
component (do not replace it with `+0.55`). Measure it for a different vehicle.
The complete IMU CSV and axis contract is in
[GNSS/IMU fusion](../imu_fusion.md).

Frame contract: the ESKF's internal `toPositionSolution()` is IMU-origin
position/velocity and is retained for propagation and RTK coupling. The
external fused `.pos` and KML written by this guide are antenna-frame outputs
from `toAntennaPositionSolution()`, using `p + R*r` and
`v + R*(omega x r)` plus the corresponding `H P H^T` covariance projection.
The RTK `.pos` and PPC `reference.csv` are also antenna-frame, so the bridge
scorer compares like frames. Keep this distinction in any archived summary
or manifest. The `urban-bridge-score` summary records this as
`coordinate_frame_contract`; a raw internal ESKF solution must not be scored
as antenna data.

Keep `--navi776-tc` out of the first loose-coupling baseline. It enables a
separate RTK-hosted tight-coupling combination and must be scored as its own
variant rather than silently changing the RTK stream under this recipe.

## The constraint recipe

Start with ZUPT enabled and NHC disabled, as in the command above.

- ZUPT constrains velocity to zero only during a detected stop. It limits
  stationary drift and helps bias estimation, but cannot help while moving.
- NHC constrains lateral and vertical body-frame velocity. Enable it only for
  a wheeled vehicle with little side slip; sharp turns, ice, banked roads, and
  aggressive manoeuvres violate the model.
- Neither constraint should be judged by RTK FIX rate. Judge the fused path
  during RTK-degraded spans and check that GNSS reacquisition has no jump.

For a new vehicle, replay the same precomputed RTK stream so the only changed
variables are the motion constraints:

```bash
python3 apps/gnss.py fuse \
  --data-dir data/PPC-Dataset/tokyo/run1 \
  --gnss-pos output/use_cases/urban_fusion/rtk.pos \
  --lever-arm 0.31,0,-0.55 --no-zupt --no-nhc \
  --out output/use_cases/urban_fusion/fused_imu_only.pos

python3 apps/gnss.py fuse \
  --data-dir data/PPC-Dataset/tokyo/run1 \
  --gnss-pos output/use_cases/urban_fusion/rtk.pos \
  --lever-arm 0.31,0,-0.55 --zupt --nhc \
  --out output/use_cases/urban_fusion/fused_zupt_nhc.pos
```

Keep NHC only if truth-based errors improve in the RTK-degraded segments and
turns do not regress. The separate fixed-lag FGO audit found no FIX gain from
NHC-only or ZUPT-only on its frozen Tokyo slice, while the combined setting
introduced two wrong fixes. Consequently the FGO switches remain default-off;
see the [NHC/ZUPT FIX-rate audit](../fgo_nhc_zupt_fix_rate_audit.md). That FGO
result is a warning against treating a motion constraint as ambiguity-fixing
evidence; it is not a claim that the ESKF continuity bridge is unusable.

## Inspect the hand-off

Render RTK and fused tracks with the same status-coloured plotter. The
`trackplot` command's optional second input is specifically an RTKLIB-format
file, so two libgnss++ `.pos` files are plotted separately:

```bash
python3 apps/gnss.py trackplot \
  output/use_cases/urban_fusion/fused_zupt.pos
python3 apps/gnss.py trackplot \
  output/use_cases/urban_fusion/rtk.pos
```

`gnss stats` is useful for epoch/status counts, but its spread around a single
mean position is **not an accuracy metric for a moving vehicle**. Use
`reference.csv` and the full-run validation workflow for accuracy. The
published PPC snapshot in [GNSS/IMU fusion](../imu_fusion.md) reports about
99% fused availability versus 69--92% for RTK alone, but also shows that long
urban dead-reckoning gaps can increase horizontal RMSE.

For each RTK-degraded interval, inspect:

1. error at the last trustworthy RTK anchor;
2. maximum horizontal error and drift rate during the bridge;
3. the position jump when trustworthy RTK returns; and
4. whether ZUPT/NHC was applied only when its vehicle-motion assumption held.

Do that inspection mechanically on a full run with `urban-bridge-score`:

```bash
python3 apps/gnss.py urban-bridge-score \
  --rtk-pos output/use_cases/urban_fusion/rtk.pos \
  --fused-pos output/use_cases/urban_fusion/fused_zupt.pos \
  --reference-csv data/PPC-Dataset/tokyo/run1/reference.csv \
  --summary-json output/use_cases/urban_fusion/bridge_score.json \
  --segments-csv output/use_cases/urban_fusion/bridge_segments.csv
```

The scorer limits the evaluation window to the RTK file's time span. Its JSON
records SHA-256 provenance, RTK/fused availability, fixed-epoch P95 error,
bridge coverage, and optional pass/fail thresholds. Each CSV row records one
RTK-degraded segment, including whether it has FIX anchors on both sides,
maximum horizontal error, error-growth rate, and reacquisition discontinuity.

### R1 Tokyo run1 frozen candidate

The R1 candidate keeps the PPC lever-arm contract (`0.31,0,-0.55`) and adds
the deterministic Doppler recovery policy. A velocity update is gated at
`--max-velocity-nis 25`; after three consecutive trusted, finite/PSD Doppler
rejections, only the lever-arm-compensated velocity is re-anchored, with a
20 m/s correction bound. A position update is gated at the application default
and a position-only recovery requires 30 consecutive `FIXED` rejections;
`FLOAT` epochs reset that patience. The position recovery has no arbitrary
distance cap by default, because a returning accurate FIX can be far from the
IMU after a long outage. Position, attitude, and bias states are not overwritten
by the velocity recovery.

Use these explicit options for the frozen run1 replay (do not open run2/run3
until this run1 artifact has passed the project's accuracy review):

```bash
python3 apps/gnss.py fuse \
  --data-dir data/PPC-Dataset/tokyo/run1 \
  --lever-arm 0.31,0,-0.55 \
  --preset low-cost \
  --zupt --no-nhc \
  --max-velocity-nis 25 \
  --max-consecutive-velocity-gate-rejections 3 \
  --max-gnss-velocity-reanchor-mps 20 \
  --rtk-pos-out output/use_cases/urban_fusion_full/r1_frozen.rtk.pos \
  --out output/use_cases/urban_fusion_full/r1_frozen.fused.pos \
  2>&1 | tee output/use_cases/urban_fusion_full/r1_frozen.log

python3 apps/gnss.py urban-bridge-score \
  --rtk-pos output/use_cases/urban_fusion_full/r1_frozen.rtk.pos \
  --fused-pos output/use_cases/urban_fusion_full/r1_frozen.fused.pos \
  --reference-csv data/PPC-Dataset/tokyo/run1/reference.csv \
  --summary-json output/use_cases/urban_fusion_full/r1_frozen_score.json \
  --segments-csv output/use_cases/urban_fusion_full/r1_frozen_segments.csv \
  --require-fused-bridge-coverage-min 99 \
  --require-max-bridge-error-max 75 \
  --require-max-reacquisition-jump-max 15 \
  --require-fixed-p95-regression-max 5 \
  --require-fused-availability-at-least-rtk \
  --require-no-nonfinite
```

For a reproducible hand-off, the same frozen recipe can be materialized as a
single bundle command. It reuses `fuse`, `pos2kml`, `trackplot`, and
`urban-bridge-score`; no separate shell `tee` or manual conversion is needed:

```bash
python3 apps/gnss.py urban-continuity-bundle \
  --data-dir data/PPC-Dataset/tokyo/run1 \
  --reference-csv data/PPC-Dataset/tokyo/run1/reference.csv \
  --output-dir output/use_cases/urban_fusion_full/r1_bundle
```

The bundle emits `rtk.pos`, `fused.pos`, `fused.kml`, `fused_trajectory.png`,
`bundle.log`, `score.json`, `segments.csv`, and `manifest.json`. The manifest
records the exact dispatcher and child argv, input/output byte counts and
SHA-256 values, binary path/hash and version-probe availability, frame and
lever-arm contracts, frozen gate results, each step exit status, and measured
maximum propagation age. Its operational age recommendation is 60 s; the
Tokyo run1 validation observed 87.2 s, so a bridge over the recommendation is
reported as operationally unusable even when the score gate passes. The
manifest is written incrementally so a failed conversion, plot, or score still
leaves the log and failure evidence.

The checked artifacts used the same options with the names
`negative_z_velocity_reanchor_3500*` (first 3,500 epochs) and
`negative_z_velocity_reanchor_unbounded_run1*` (full run1):

| R1 artifact | Fused availability | Bridge coverage | Max bridge H error | Max reacquisition step | Fixed-P95 regression vs RTK |
|---|---:|---:|---:|---:|---:|
| 3,500-epoch gate | 99.600% | 99.334% | 55.580 m | 10.154 m | 2.068 m |
| Full run1 | 99.724% | 99.233% | 64.898 m | 10.154 m | 2.718 m |

Both artifacts contain no non-finite fused epochs. The full-run score passes
the frozen gates (coverage >=99%, max bridge <=75 m, reacquisition <=15 m,
fixed-P95 regression <=5 m, and fused availability >= RTK availability). Its
gated score and per-bridge ledger are archived at
`output/use_cases/urban_fusion_full/negative_z_velocity_reanchor_unbounded_run1_gated_score.json`
and
`output/use_cases/urban_fusion_full/negative_z_velocity_reanchor_unbounded_run1_gated_segments.csv`.
These numbers are a run1 candidate result, not permission to tune or evaluate
the run2/run3 holdouts.

Continuity is not the same as an operationally usable bridge. In this
low-cost Tokyo run1, the largest `propagation_age_s` in the segment ledger is
87.2 s (segment 169). A deployment profile should set a vehicle- and
environment-specific maximum bridge age; 60 s is a conservative starting
recommendation for this profile. Mark any segment over that limit as
**unusable** for the application, even when the fused file remains continuous,
and surface the condition to the integrity/mission layer rather than hiding it
behind the availability percentage.

### R1 frozen release decision and holdout evidence

The run1 candidate was frozen before evaluating the holdouts. The score gates
are the coverage, bridge-error, reacquisition, fixed-P95-regression,
fused-availability, and non-finite limits shown above. The following table is
the immutable post-freeze record; percentages are reported by the bundle
scorer and are not rounded for the gate decision:

| Bundle | Reference epochs | RTK fixed | Complete bridges | Fused availability vs RTK | Coverage | Max bridge H | Reacquisition | Fixed-P95 regression | Max propagation age | Score gate |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| `r1_frozen_bundle` (run1) | 11,951 | 8,430 | 205 | 99.723872% vs 99.113045% | 99.233172% | 64.899308 m | 10.154390 m | 2.686921 m | 87.2 s | PASS |
| `r1_holdout_run2` | 9,151 | 7,230 | 131 | 99.890722% vs 99.595673% | 99.791775% | 14.504349 m | 1.620029 m | 0.358979 m | 96.6 s | PASS |
| `r1_holdout_run3` | 15,301 | 11,071 | 234 | 99.934645% vs 99.993464% | 99.905437% | 26.238413 m | 10.523244 m | 0.422489 m | 63.4 s | **FAIL** (availability only) |

All three bundles have `nonfinite=0`. The run2 and run3 artifacts are
archived at `output/use_cases/urban_fusion_full/r1_holdout_run2/` and
`output/use_cases/urban_fusion_full/r1_holdout_run3/`; run1 is archived at
`output/use_cases/urban_fusion_full/r1_frozen_bundle/` and its exact repeat
is byte-for-byte deterministic. Run3 was not rerun, retuned, or downgraded to
development data. Its single fused-availability failure keeps the R1 profile
offered for field use **on hold**; do not change the frozen settings using
these holdout observations. A future release decision must either accept the
explicit hold or produce a separately approved candidate on newly authorized
development data.

The 60 s operational bridge-age recommendation is a separate integrity gate:
run1 (87.2 s), run2 (96.6 s), and run3 (63.4 s) all exceed it. Thus, even the
run2 score pass is not a field-usable result under the current checklist.
See the [R1-09 field checklist](urban_rtk_imu_field_checklist.md) for the
usable/degraded/unusable classification and the one-command hand-off.

R1-05 also replayed the same saved RTK stream with the frozen lever arm and
recovery settings while changing only the motion constraints. The replay
comparison is useful for rejecting NHC, but its saved-RTK timestamp population
is not the in-process full-run denominator above:

| Replay profile | Bridge coverage | Max bridge H error | Max reacquisition step |
|---|---:|---:|---:|
| IMU only | 96.876% | 33.679 m | 1.024 m |
| ZUPT | 96.876% | 84.230 m | 3.375 m |
| ZUPT + NHC | 96.876% | 166.179 m | 3.706 m |

The ZUPT and ZUPT+NHC regressions support keeping NHC out of the frozen
run1 recipe. Artifacts are under
`output/use_cases/urban_fusion_full/r1_profile_compare/`; do not compare
their availability directly with the in-process R1 candidate.

### Historical pre-R1 full-run evidence

The pre-R1 Tokyo run1 replay (11,951 reference epochs) shows why the bridge
scorer is mandatory:

| Fusion profile | Fused availability | Bridge coverage | Max bridge H error | Max reacquisition step | Fixed-P95 regression vs RTK |
|---|---:|---:|---:|---:|---:|
| IMU only | 99.03% | 96.88% | 1,396.9 km | 297.85 m | +1,322.1 km |
| ZUPT, no NHC | 99.72% | 99.23% | 1,321.30 m | 62.20 m | +58.54 m |
| ZUPT + NHC | 99.03% | 96.88% | 3,047.73 m | 6.82 m | +2,998.37 m |

These historical profiles increase output continuity but fail an
application-accuracy gate. The IMU-only and ZUPT+NHC rows reuse the saved RTK
`.pos`; that replay surface outputs fused epochs at saved GNSS timestamps and
therefore has a slightly different availability denominator from the
in-process ZUPT row. Compare accuracy only after preserving that provenance
distinction.

## Exit criteria

The short wiring check passes when all artifacts exist and the fused stream
covers more than a trivial interval:

```bash
test -s output/use_cases/urban_fusion/rtk.pos
test -s output/use_cases/urban_fusion/fused_zupt.pos
test -s output/use_cases/urban_fusion/fused_zupt.kml
test -s output/use_cases/urban_fusion/fused_zupt.log
test "$(grep -vc '^%' output/use_cases/urban_fusion/fused_zupt.pos)" -ge 100
grep -q '<coordinates>' output/use_cases/urban_fusion/fused_zupt.kml
```

A vehicle profile is ready for a longer evaluation only when the full run,
scored against `reference.csv`, has no unacceptable bridge drift or
reacquisition jump. Archive the command line, lever arm, IMU axis convention,
RTK and fused `.pos` files, KML, and log together.

## Boundary and next step

This offline route demonstrates RTK-to-IMU continuity using existing commands
and public data. It does not establish lane-level safety, integrity risk,
sensor time synchronisation, or universal centimetre accuracy. Production
work also needs calibrated latency and lever arm, IMU thermal/bias testing,
fault detection, and a maximum allowed dead-reckoning time.

Next step: preserve the frozen run1 and holdout bundles as release evidence,
keep the R1 field profile on hold because run3 misses the availability gate,
and do not tune or rerun either holdout. The general artifact, validation, and
benchmark contracts are documented in
[interfaces](../interfaces.md),
[validation](../validation.md), and [benchmarks](../benchmarks.md).
