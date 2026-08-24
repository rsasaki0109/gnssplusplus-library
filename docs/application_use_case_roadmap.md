# Application use-case development roadmap

Date: 2026-08-23

Reorganised: 2026-08-24

## Priority policy

This is the primary product roadmap. User-facing application workflows take
priority over station-management infrastructure and broad solver research.
Infrastructure work may enter a release only when it is required by the next
application acceptance gate. The operational station plan in
[Development backlog](development_backlog.md) and the RINEX/MADOCA plans are
supporting tracks, not competing top-level product queues.

## Goal

Turn the positioning features already present in libgnss++ into reproducible
application workflows. Each workflow must start from obtainable data, produce
artifacts a downstream user understands, state an accuracy boundary, and have
a machine-readable acceptance gate.

The completed first wave is:

1. urban RTK continuity with IMU/ZUPT and optional NHC;
2. static survey and control-point measurement with GEONET/IGS data;
3. reference-trajectory production for HD maps and SLAM evaluation;
4. sub-metre wide-area positioning with CLAS/MADOCA;
The next application wave is:

5. smartphone raw-GNSS positioning;
6. UAV survey/navigation and reference-trajectory delivery;
7. structural displacement monitoring;
8. integrity/geofence decisions over qualified solutions;
9. GNSS timing and holdover assessment.

R1--R4 now provide the first common quality contract. R5 starts immediately;
R6--R9 remain ordered by dataset readiness and dependency, not by low-level
solver convenience.

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

## Next application release train

| Order | Release | User outcome | Primary reuse | Exit decision |
|---:|---|---|---|---|
| Complete | R5 Smartphone raw GNSS | An Android dataset becomes a truth-scored `.pos` bundle without manual conversion | R2 acquisition/provenance, R3 bundle/gate | Development and untouched holdout passed the frozen standalone profile; precise variant was not promoted |
| Complete (No-Go) | R6 UAV navigation | A UAV operator receives a lever-arm/attitude-qualified flight and landing trajectory | R1 fusion, R3 consumer profiles | Development navigation/visualization passed, mapping was No-Go, and untouched holdout run1 failed closed on an invalid MCAP schema; no promotion or post-failure tuning |
| Now | R7 Structural monitoring | A maintainer receives multi-day displacement/noise reports rather than isolated PPP points | R2 static PPP and covariance | Go only when multi-day repeatability separates motion from solver/environment noise |
| Later | R8 Integrity/geofencing | An application consumes qualified positions and explicit alert/reject reasons | R1--R4 state vocabulary | Go only with measured false-alarm, missed-alert, stale-data and alert-latency populations |
| Later | R9 GNSS timing | An operator receives clock bias/drift uncertainty and tested holdover limits | PPP clock state and station replay | Go only when UTC/GPST/leap-second and uncertainty contracts are explicit |

### R5 — Smartphone raw-GNSS workflow (next, four-week target)

Status: complete. Source archive and Pixel 7 Pro development/holdout routes frozen;
lossless row preservation, fail-closed adapter, GPS L1 RINEX mapping, native
standalone SPP, truth sign-off, and development thresholds are complete. The
untouched holdout run1 passed every frozen standalone gate. The one-command
workflow emits POS, KML, PNG, log, summaries, and a hash manifest. The
development precise-product variant regressed and was explicitly not promoted.

Scope:

- freeze one public Android train route and one sealed holdout route;
- implement a deterministic source-format-to-observation adapter with no
  spreadsheet or hand-renaming step;
- inventory device capabilities, signals, duty cycling, clock discontinuities,
  antenna metadata, and missing observables in the manifest;
- run standalone and precise-product lanes supported by the actual fields;
- emit POS, KML/PNG, log, summary, rejected spans, and provenance manifest;
- score accuracy, solution availability, truth coverage, time gaps, and device
  exclusions separately.

Acceptance gate:

- clean-room materialisation and bounded smoke succeed;
- malformed timestamps, clock resets, missing truth, and unsupported signals
  fail closed;
- development thresholds are frozen before the holdout is opened;
- one untouched holdout run is preserved without post-failure tuning;
- the guide names supported device/data classes and never generalises one
  handset result to all Android devices.

### R6 — UAV survey/navigation workflow (four-week target)

Status: complete with an R6 release No-Go. MARS-LVIG development acquisition,
raw-GNSS/IMU/attitude/truth adaptation, native SPP, five motion populations,
28 frozen gates, separate consumer decisions, one-command workflow, negative
tests, and documentation are complete. Development navigation and visualization
passed; mapping was No-Go because only four-DoF truth and no resolved
LiDAR-to-body rotation were available. The untouched MCAP holdout run1 stopped
fail-closed on an empty embedded schema encoding before solution generation.
The failure is preserved without tuning or rerun. See
`docs/use_cases/uav_navigation.md` and the two R6 records under
`docs/use_cases/records/`.

Scope and gate:

- select a public flight with raw GNSS, IMU/attitude, antenna/body lever arm,
  timestamps, and independent reference;
- define cruise, turn, climb/descent, hover, and landing populations;
- disable vehicle-only NHC and reject undefined frame/lever-arm inputs;
- gate horizontal/vertical error, trajectory coverage, maximum gap/jump,
  attitude alignment, and landing-path continuity;
- publish separate navigation, mapping, and visualisation decisions.

### R7 — Structural displacement monitoring (four-week target)

Scope and gate:

- extend the R2 public station route to multiple days without using daily
  RINEX header positions as truth;
- report daily/interval coordinates, empirical covariance, environmental and
  product provenance, gaps, steps, and long-term drift;
- estimate the stable-site noise floor before selecting displacement limits;
- require an injected/surveyed displacement witness before claiming detection;
- label a result `unusable` when reference frame, antenna history, monument
  change, or observation continuity is ambiguous.

### R8/R9 — Safety and timing applications

R8 consumes the common quality summaries; it does not invent integrity from a
FIX flag. R9 begins with an observability audit and must publish uncertainty
and holdover evidence before exposing an NTP/PTP-style service. Neither may
pre-empt R5--R7 merely because its infrastructure is convenient to build.

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

R5 and R6 items 1--10 are complete; R6 concluded No-Go and its opened holdout
must not be rerun. The active implementation slice is R7 structural monitoring:

1. freeze at least three public observation days for one demonstrably stable
   station and keep a later day sealed;
2. freeze station reference frame, antenna/receiver history, monument event
   evidence, source/product hashes, and missing-data rules;
3. compute daily and fixed-interval coordinates without using each day's RINEX
   approximate header position as truth;
4. publish empirical covariance, gaps, steps, long-term drift, and product/
   environment provenance;
5. estimate the stable-site horizontal/vertical noise floor on development days;
6. inject a known displacement witness after the noise floor and verify
   detection plus direction/magnitude reporting;
7. freeze repeatability, continuity, false-alert, and witness-detection gates;
8. run the sealed day once without tuning;
9. publish R7 usable/degraded/unusable decisions and the one-command report;
10. complete R7 docs and tests before opening R8 implementation.

R1 and R4 negative results remain maintenance inputs, not permission to tune
on their sealed runs. Station-service, RINEX 4, MADOCA parity, and upstream
adoption work continue only as bounded enablers for an active application
release.
