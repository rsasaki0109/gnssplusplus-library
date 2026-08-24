# Integrity-aware geofence decisions

R8 turns a truth-qualified urban trajectory into `inside`, `outside`, or
`unknown` geofence decisions. It deliberately does **not** infer integrity from
RTK `FIXED`, and it does not call its bounds aviation HPL/VPL. The implemented
empirical protection envelope (EPE) is a conservative finite-dataset guard,
not a certified integrity-risk probability or safety-of-life authorization.

The R8 release is a **No-Go** for cross-city promotion. Tokyo development
passed, but the untouched Nagoya holdout failed the upstream positioning
quality contract before any geofence decision was issued.

## One-command workflow

A bounded development smoke uses the first 3,500 GNSS epochs:

```bash
python3 apps/gnss.py integrity-geofence-workflow \
  --phase development --mode smoke \
  --data-dir data/PPC-Dataset/tokyo/run1 \
  --output-dir output/integrity_r8/smoke
```

The full development replay is:

```bash
python3 apps/gnss.py integrity-geofence-workflow \
  --phase development --mode full \
  --data-dir data/PPC-Dataset/tokyo/run1 \
  --output-dir output/integrity_r8/development-replay
```

The Nagoya holdout is closed as `closed_no_go`; do not change the state or
rerun it. A future R8 candidate needs a newly frozen profile and a different
unopened city/run.

## Decision contract

The workflow first runs the existing urban continuity bundle. If its quality
gate fails, all downstream geofence decisions are withheld. If it passes, the
decision stage uses RTK coordinates at FIX epochs and the fused coordinate
between FIX anchors. It separates these populations:

| Population | Frozen EPE |
|---|---:|
| RTK FIX | 24.0 m |
| Bridge age 0–5 s | 71.8 m |
| Bridge age 5–15 s | 71.9 m |
| Bridge age 15–30 s | 71.9 m |
| Bridge age 30–60 s | 71.9 m |

Bridge envelopes are forced to be non-decreasing with age. A missing or
non-finite solution, missing FIX anchor, age above 60 s, failed upstream
quality contract, or missing envelope produces `unknown`. These states are
never silently counted as inside or outside.

The demonstrator uses a 500 m circular operating zone centred on the first
RTK FIX. For each epoch it computes signed distance to the boundary:

- `inside` only when the estimated point is more than its EPE inside;
- `outside` only when it is more than its EPE outside;
- `unknown` when the EPE intersects the boundary.

Independent PPC Applanix truth is used only to score the released evaluation.
An operational deployment can emit decisions without online truth, but it
cannot claim that its EPE remains valid without separate monitoring evidence.

## Development and holdout

Tokyo run1 calibrated five separate error populations. Its maximum observed
horizontal errors were 21.36 m for FIX and 47.37–64.90 m across bridge bins.
After a 10% plus 0.5 m margin and monotonic-age adjustment, the frozen EPEs
above produced:

- 11,951 truth epochs and 11,741 quality-qualified epochs;
- 96.812% decisive availability;
- zero misleading inside/outside decisions;
- zero EPE exceedances;
- 381 unknown epochs: 171 boundary intersections, 27 missing solutions, and
  183 epochs over 60 s since FIX.

Nagoya run1 was selected and hash-sealed before its solution was generated.
The one-shot run stopped because fused bridge coverage was 97.891% versus the
99% gate and maximum bridge horizontal error was 100.191 m versus the 75 m
gate. The decision stage did not run, which is the intended fail-safe outcome.
See the [development record](records/integrity_r8_development.json) and
[holdout run1 record](records/integrity_r8_holdout_run1.json).

## Outputs and field boundary

On a qualified run, `decision/` contains `summary.json`, `decisions.csv`,
`geofence.geojson`, `decision_scorecard.png`, and `manifest.json`; the upstream
directory contains `.pos`, KML, PNG, score, segments, log, and manifest. Every
source and artifact is SHA-256 recorded.

This is suitable for testing conservative application logic, unknown-state
handling, and regression behavior. Before using it for people, vehicles,
machinery, drones, or restricted areas, add an application hazard analysis,
independent fault modes, integrity-risk allocation, authenticated/cyber-safe
inputs, map/frame assurance, latency monitoring, alert time, redundancy, and
a representative multi-city/multi-environment validation campaign.
