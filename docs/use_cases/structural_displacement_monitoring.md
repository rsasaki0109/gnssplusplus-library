# Structural displacement monitoring: IGS Tsukuba multi-day route

This R7 workflow extends the public [Japan static survey](japan_static_survey.md)
route into a four-day displacement-monitoring qualification. It estimates the
stable-site noise floor from 2024-01-01 through 2024-01-03, freezes alert gates,
and then scores the untouched 2024-01-04 holdout once. The released decision is
`usable` for this regression and synthetic-alert qualification only. It is not
a claim that a real structure moved, nor a field-deployment approval.

## Commands

Use a separate output directory for the bounded 20-epoch smoke:

```bash
python3 apps/gnss.py structural-displacement-workflow \
  --phase development --mode smoke \
  --output-dir output/structural_r7/smoke \
  --cache-dir data/structural_r7/cache
```

Reproduce the full three-day development result with:

```bash
python3 apps/gnss.py structural-displacement-workflow \
  --phase development --mode full \
  --output-dir output/structural_r7/development-replay \
  --cache-dir data/structural_r7/cache
```

The holdout has already been opened and closed. The tracked profile now says
`closed_passed`, and the original output directory also blocks a second run.
Do not change it back to `sealed` or tune against 2024-01-04. To evaluate a
future release, select and hash a new later day before opening it.

## Data, frame, and station contract

The command downloads hash-pinned TSK2/TSKB 30-second observations, broadcast
navigation, IGS final SP3/CLK products, the IGS weekly combined SINEX, IGS20
SSC/ANTEX, and both official station logs. BKG, SOPAC/Garner, IGS Central
Bureau, and GSI RNXCMP provenance is inherited from the R2 route and recorded
per day. The precise products are retained in each bundle even though the
released displacement series uses the relative-static lane.

Daily `APPROX POSITION XYZ` values are never truth. Every daily and six-hour
coordinate is compared in the frozen IGS20 weekly-SINEX frame, using the
selected `SOLUTION/EPOCHS` plus `SOLUTION/ESTIMATE` rows. The relative lane
uses the independent TSKB SINEX coordinate as its base position. The weekly
solution lacks station velocity, so this four-day route must not be extended
to long epochs without a velocity-capable frame realization.

The logs and all four RINEX headers agree on the monitored equipment:

| Site | Monument | Receiver during fixture | Antenna during fixture |
|---|---|---|---|
| TSK2 | `21730S010`, pillar on concrete block | `6042R40142`, TRIMBLE ALLOY 6.15 | `5938338714`, TRM159900.00 NONE |
| TSKB | `21730S005`, pillar on concrete block | TRIMBLE ALLOY 6.15 | AOAD/M_T DOME |

The TSK2 antenna has been logged since 2021-05-26 and the receiver firmware
entry since 2022-09-12; TSKB's active receiver entry starts 2022-10-18 and its
active antenna entry 2017-05-17. Both logs have open-ended removal fields and
the RINEX identities are unchanged on all four days. Nevertheless, the pinned
log files are 2022 revisions. An operator must obtain a newer station log or
site inspection before a field claim. An unknown monument, receiver, antenna,
radome, cable, earthquake, maintenance, or local-tie change makes the result
`unusable`, not a displacement alert.

Weather observations are not bundled and local environmental/loading models
are not applied. This absence is emitted explicitly in `signoff.json` and is
another reason the three-day noise floor cannot be generalized seasonally.

Optional camera snapshots add visual context. Pass
`--snapshot-dir` (workflow or sign-off) with files named `YYYYMMDD*` or
`YYYY-MM-DD*` (UTC). Each snapshot is SHA-256 hashed into
`signoff.json` under `visual_evidence`; a day without a snapshot is declared
absent, and when the channel is provided an absent day or an unmatched file
fails the contract (`visual_evidence_incomplete`,
`visual_evidence_unmatched_files`). Snapshots are site context, never a
displacement measurement.

## Artifacts and metrics

Each upstream day contains `.pos`, KML, PNG, summaries, a log, and a manifest.
The R7 `signoff.json` adds:

- one daily coordinate and four six-hour coordinates per full day;
- empirical epoch covariance and daily stable-site noise floor;
- missing-epoch gaps, daily ENU steps, and linear ENU drift per day;
- station, frame, product, environment, and source-hash provenance;
- false-alert count plus injected-witness magnitude, direction, and detection;
- `usable`, `degraded`, or `unusable` with fail-closed contract reasons.

Development measured horizontal/vertical 1-sigma daily noise of
0.000358/0.000662 m. Because three consecutive days are a weak environmental
sample, the gates were floored at 0.020 m horizontal and 0.040 m vertical,
with at least 99% FIX, no gap above 60 s, and zero stable-site alerts. Only
after those values existed, the development workflow selected and froze the
synthetic witness `(E, N, U) = (0.040, -0.020, 0.080) m`.

The sealed day passed unchanged: 99.861111% FIX, no missing epoch gap, no
stable-site alert, and an offset of approximately `(-0.000307, +0.000058,
+0.000567) m` from the frozen baseline. The synthetic witness was detected
with its recorded magnitude and azimuth. Full evidence is preserved in the
[development record](records/structural_r7_development.json) and
[holdout run1 record](records/structural_r7_holdout_run1.json).

## Decision rules and field checklist

`usable` means frame, equipment identity, continuity, false-alert, and witness
gates all pass. `degraded` means the data contract is intact but a numeric gate
fails. `unusable` means frame, monument/equipment history, truth independence,
or continuity is missing or ambiguous.

Before attaching the workflow to a bridge, dam, slope, or building:

1. survey the actual monument/structure tie and antenna phase-centre contract;
2. collect representative seasonal, thermal, multipath, loading, and weather
   evidence before selecting millimetre/centimetre alert levels;
3. use a current reference-frame solution with velocity and event handling;
4. record every receiver, antenna, radome, cable, firmware, and maintenance
   change in a machine-readable registry;
5. validate with a surveyed physical displacement witness, redundancy, and an
   operational false-alarm/missed-detection review.
