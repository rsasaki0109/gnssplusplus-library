# Phase270: relative-height observation audit

Primary-agent source inspection only. No candidate or truth coordinates,
MAT payloads, or native reruns were used. Phase269 established only a
negligible velocity-prior delta; it does not explain the accuracy gap.

## Concrete missing observation

Cached `output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m:238-246`
selects nearby same-trajectory positions and inserts a zero-displacement
BetweenFactorVector with horizontal sigmas infinite and vertical sigma
0.1 m. Parameters at `parameters.m:163-168` specify spatial distance below
15 m, accumulated travel difference above 100, and Huber k=0.5. Both
endpoints must be non-stop. This is an equal-local-up constraint, not an
absolute altitude observation. Native backend inspection found no matching
relative-height insertion; existing motion and stop factors are not this
observation family.

The source's alternate `posgt` branch at lines 230-235 depends on externally
loaded `ref_hight.mat` (lines 26-28). It is excluded entirely. Inspection of
these code lines is not access to that file, and does not establish which
branch produced any historical leaderboard result.

## Important non-parity trap

Line 121 defines `cumdist = cumsum(velini.v3)`, without multiplying by time
interval. Therefore its threshold is not generally a distance in metres.
At constant speed 10 m/s with 0.5-second samples, indices separated by
11 intervals differ by 110 in that source quantity but only 55 m of
integrated travel. Conversely, sparse epochs undercount physical travel.
A direct port must explicitly label this sample-index convention; a
time-integrated selector is a different algorithm, not source parity.
Do not assume 1 Hz from route naming or inspect saved trajectories to
choose a favorable threshold.

## Next bounded implementation and validation

Implement a pure, opt-in pair selector with synthetic tests before any
real accuracy experiment. Inputs must be same-run native in-memory ECEF
positions, velocities, exact epoch identities, and stop masks. It must
have no paths or coordinate-file reader. Start with the explicit source
sample-index convention; reject nonfinite vectors, mismatched lengths,
invalid timestamps, and inconsistent ordering. Cover strict threshold
boundaries, stopped endpoints, duplicate prevention, irregular sampling,
and deterministic ordering. Report selected pair count and the convention.

Then implement a scalar local-up difference factor, with analytic and
finite-difference Pose3 Jacobian tests, including a rotated ENU frame and
nonzero lever arm. Horizontal displacement must not affect its residual.
Verify the scalar robust loss against the source's zero-horizontal-weight
vector factor. Freeze candidates from GNSS-first before the main solve;
do not select pairs from the evolving main estimate or saved outputs.

Only after those tests and an actual insertion/no-fallback integration
test should a single raw run be frozen. Nearby trajectories can be on
different road levels; robust loss is not proof that a pair is valid.
This remains a hypothesis, not evidence of improvement. H is development
data; broader leakage-audited validation and leaderboard evidence remain
outstanding, as does the 0.782-class objective.
