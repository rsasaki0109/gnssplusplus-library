# Phase211: separate IMU factors do not improve H

Primary-agent work only. Freeze commit `bca899a` precedes the single
authorized evaluation. Metadata/authorization and unchanged synthetic metric
tests passed 13/13; the raw solver was not rerun for evaluation.

H development score: **1.2755873347358355 m**.
P50: 0.7559659728170619 m; P95: 1.795208696654609 m.
Phase199 best: 1.2751561666667786 m.
Delta: **+0.00043116806905696414 m**, worse.
All 3139 keys matched; finite/Earth-range checks passed. No interpolation,
edge hold, offset reapplication or over-70-m/s segment was reported.
Candidate and truth were each read once; one score was calculated.

Do not promote Phase209. Retain its default-off implementation and tests.
Phase198/199 remains best. H is repeatedly used development/train, not heldout
or leaderboard proof; the 0.782-class objective remains unachieved.

The tested density-only and separate-factor variants both stay near 1.276 m.
This does not prove all IMU modeling irrelevant, but neither tested change
explains the approximately 0.493 m gap to the target. Avoid a numerical sweep
of these options on H. Next inspect the remaining source/native initial-prior
and observation-model differences with code and synthetic observability
controls before selecting another raw experiment. A source-like removal of
finite priors must not silently introduce an unobservable graph or be called
full source parity. Preserve the best recipe for broader route-grouped
validation; do not label historically scored routes as fresh heldout data.

Known telemetry issue: Phase210's top-level summary status retains the old
`imu-combined-factor` string despite the verified separate factor branch.
Correct future summary labeling with a focused test; do not rewrite the sealed
candidate or summary, and do not rerun Phase210 just to change this label.
