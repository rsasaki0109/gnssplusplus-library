# Application use-case development roadmap

Date: 2026-08-23

## Goal

Turn the positioning features already present in libgnss++ into reproducible
application workflows. Each workflow must start from obtainable data, produce
artifacts a downstream user understands, state an accuracy boundary, and have
a machine-readable acceptance gate.

The development order is:

1. urban RTK continuity with IMU/ZUPT and optional NHC;
2. static survey and control-point measurement with GEONET/IGS data;
3. reference-trajectory production for HD maps and SLAM evaluation;
4. sub-metre wide-area positioning with CLAS/MADOCA;
5. discovery spikes for smartphone GNSS and UAV navigation.

Structural monitoring, GNSS time service, and integrity/geofencing remain
parked until the first four workflows share a stable quality contract.

## Common product contract

Every use case ships the same five layers:

| Layer | Required deliverable |
|---|---|
| Acquire | Public-data source, licence/provenance, checksum, and expected directory layout |
| Run | One bounded smoke command and one full evaluation command |
| Inspect | `.pos`, KML/plot, log, and a versioned summary JSON |
| Gate | Explicit thresholds plus a non-zero exit code when they fail |
| Explain | Application claim, known failure modes, and a next-field-test checklist |

No workflow may use “centimetre” or “sub-metre” as an unqualified whole-route
claim. Accuracy, availability, status population, and truth coverage are
reported separately. A run without independent truth is a wiring or
repeatability result, not accuracy evidence.

## Release train

### R1 — Urban continuity recipe (weeks 1–2)

Status: implementation and frozen run1/run2/run3 evaluation complete. Run3
failed the unchanged availability gate, so vehicle-profile promotion remains
blocked; the negative holdout is preserved.

Scope:

- keep [the current urban recipe](use_cases/urban_rtk_fgo.md) as the operator
  entry point;
- maintain the `urban-bridge-score` scorer that aligns `rtk.pos`, fused
  `.pos`, and PPC `reference.csv`;
- segment the result into RTK FIX, RTK-degraded bridge, and reacquisition;
- compare IMU-only, ZUPT, and ZUPT+NHC on an identical precomputed RTK stream;
- emit summary JSON, bridge-segment CSV, and a scorecard plot;
- add a maximum bridge-age recommendation instead of implying indefinite
  dead reckoning;
- either make `fuse --kml` include propagated fusion epochs or document and
  test the accepted `pos2kml --status all` route.

Acceptance gate:

- all output epochs match truth within the declared time tolerance;
- no non-finite fused epoch;
- fused availability exceeds RTK availability;
- fixed-epoch accuracy does not regress beyond a frozen tolerance;
- every bridge reports duration, maximum horizontal error, drift rate, and
  reacquisition jump;
- NHC is promoted for a vehicle profile only if it wins on full Tokyo run1
  and unchanged Tokyo run2/run3 holdouts without a turn-segment regression.

R1 is complete when one command produces the full artifact bundle and CI can
fail a regression from its summary JSON.

### R2 — Japanese static survey route (weeks 3–5)

Status: public-data adapter, guide, development-day full replay, and sealed
next-day holdout are complete. Relative RTK remains demonstrative without
ANTEX; PPP is a post-convergence candidate requiring survey-authority review.

Scope:

- define one immutable GEONET/IGS example by station IDs, UTC/GPST date,
  RINEX interval, monument coordinates, and source URLs;
- add a small fetch/materialisation adapter for the selected public files,
  preserving checksums and source metadata;
- run two lanes:
  - short-baseline static relative positioning with
    `gnss short-baseline-signoff`;
  - base-free static PPP with `gnss ppp-static-signoff` and IGS products;
- document Japanese coordinate concerns: ellipsoidal versus orthometric
  height, antenna reference point/phase centre, epoch and reference frame;
- emit estimated coordinate, covariance, ENU delta from published truth,
  observation duration, convergence/solution status, and provenance JSON;
- provide a 10-minute smoke slice and a survey-duration full run.

Acceptance gate:

- clean-room fetch and replay succeed from documented public inputs;
- repeated runs are deterministic within the declared numeric tolerance;
- horizontal/vertical errors are scored against published station truth;
- missing products, station metadata, or antenna information fail closed;
- both relative-static and PPP lanes state which result is fit for a control
  point and which is only demonstrative.

R2 is complete when a Japanese user can select the documented station/date,
run the guide without hand-renaming files, and receive a pass/fail report.

### R3 — Reference trajectory package (weeks 6–7)

Status: versioned bundle, three consumer profiles, independent PPC scoring,
non-solving validator, Python loader, and ROS2 consumption smoke are complete.

Scope:

- turn one solve into a versioned trajectory bundle: raw `.pos`, accepted
  subset, KML, plot, ROS2 metadata, summary JSON, and manifest;
- score against PPC Applanix truth before calling the result a reference;
- report route-distance coverage rather than epoch FIX rate alone;
- detect time gaps, position jumps, status transitions, and rejected spans;
- record antenna-to-camera/LiDAR lever arm and target coordinate frame;
- define consumer profiles for SLAM evaluation, map prototyping, and
  visualisation, each with different acceptance thresholds;
- add a command that validates a bundle without rerunning the solver.

Acceptance gate:

- manifest contains input hashes, command/config, software version, frames,
  lever arm, time system, and quality thresholds;
- accepted trajectory has no undeclared time gaps or frame discontinuities;
- error and coverage meet the selected consumer profile;
- ROS2 replay smoke and Python `load_pos()` contract both pass;
- “ground truth” is never emitted as a label when independent truth is absent.

R3 is complete when a perception developer can consume the bundle without
learning GNSS status-column conventions.

### R4 — CLAS/MADOCA sub-metre decision workflow (week 8)

Status: public QZSS L6 acquisition, decode inventory, PPC truth score,
application gate, variant table, and correction-loss negative test are
complete. The frozen run is degraded rather than promoted for continuous
sub-metre use.

Scope:

- select one redistributable L6/RINEX example or provide a deterministic
  download/materialisation path;
- connect L6 provenance, decode diagnostics, positioning summary, and truth
  score into one manifest;
- distinguish FIX accuracy, whole-route accuracy, solution availability, and
  L6 correction availability;
- publish a clear Japan/coverage/receiver-capability decision table;
- retain MADOCA and RTCM SSR as separate correction-source variants.

Acceptance gate:

- first correction epoch and first valid position are both reported;
- fallback epochs and unsupported message types are visible;
- sub-metre claims are truth-scored and population-qualified;
- loss of L6 produces an explicit degraded state rather than a silent success.

## Discovery queue

Run these as time-boxed design spikes only after R2 is usable:

| Candidate | Spike | Go condition |
|---|---|---|
| Smartphone raw GNSS | Inventory Android/Google challenge formats and map them to existing observation interfaces | One public train/holdout pair can be converted without lossy manual steps |
| UAV navigation | Audit dynamics, attitude, lever arm, NHC assumptions, output rates, and landing-path metrics | Existing fusion supports the dynamics without enabling vehicle-only constraints |
| Structural monitoring | Replay static PPP as a time series and estimate noise floor before trend detection | Stable coordinates and covariance are available across multi-day data |
| GNSS clock/NTP | Audit clock-state observability and timestamp APIs | Clock estimate has a documented uncertainty and holdover model |
| Integrity/geofencing | Reuse quality reason codes and propagated-state age | R1–R4 summaries share the same usable/degraded/unusable vocabulary |

## Work rules

- Use one development dataset and keep holdouts sealed until the development
  gate passes.
- Default-off experiments remain bit-identical when disabled.
- Commit negative results and do not tune on a failed holdout.
- Never make documentation tests assert only that a command string exists;
  at least one bounded real-data smoke must produce and validate artifacts.
- Generated artifacts stay under `output/`; tracked benchmark summaries must
  include provenance and a regeneration command.
- A use case is done only when a new user can answer: what data do I need,
  what do I run, what did I get, how accurate is it, and when must I reject it?

## Immediate backlog

The next implementation slice is R1 fusion root-cause correction using the
scorer and artifact manifest now in place:

1. identify the first divergence in the full-run bridge CSV;
2. distinguish propagation, heading-alignment, velocity-update, and
   re-anchoring failures with existing telemetry;
3. correct and test the Tokyo run1 root cause without inspecting holdouts;
4. freeze thresholds after a passing full run1;
5. evaluate run2 and run3 once with unchanged settings;
6. promote a vehicle profile only if both holdouts pass.

In parallel only with non-overlapping work, R2 may inventory the exact
GEONET/IGS station/date and download terms. No solver tuning starts in R2
until the public-data fixture and truth contract are frozen.
