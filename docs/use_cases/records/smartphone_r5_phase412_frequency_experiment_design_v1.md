# Phase412 — prospective frequency-state experiment design

Design frozen before any enabled raw run or corresponding truth evaluation.
Not an execution manifest; CLI admission and raw-time checks remain required.

Candidate: native IMU main only, already-admitted exact adjacent GPS L1/L5 and
Galileo E1/E5a TDCP pairs; one shared residual slant-change key and one zero-mean
Gaussian prior per pair. Original scalar Huber 4 and source-metre signal sigma
remain unchanged. Unmatched observations are retained. GNSS-first remains OFF.
Atmosphere corrections remain applied once; no P-only residual-ionosphere flag.

Use one explicitly experimental prior sigma of **0.01 m per matched interval**,
the existing backend synthetic-test setting, identically on H and LAX-T. This
is a modelling hypothesis, not an established physical ionosphere calibration,
not inferred from ground truth, and not a recommendation for all environments.
No prior/noise/Huber sweep or route-specific switching in this comparison.
The nuisance may absorb other frequency-dependent error; label it accordingly.

Proceed only after Phase410/411 OFF replay confirms original candidate hashes
and zero new-state/factor counts. Enabled execution must pin the current binary,
source, raw GNSS/IMU/nav hashes, full argv, and exclusive output directory.
Recheck raw timing: no missing/nonzero TimeOffsetNanos and no cross-band clock
field mismatch on in-scope raw GPS/Galileo rows. Reject a changed input contract.
No saved coordinate/trajectory/sidecar may feed native inference.

Before opening truth, require finite converged stages, exact original output
keys/counts, matched-pair state/factor/prior counts, complete in-memory correction
identity, and agreement of corrected app/backend TDCP RMS. Freeze scoring
separately, retaining the historical LAX-T first-epoch evaluation domain.
Do not call these reused routes heldout. Report each route and regressions;
do not promote globally from one improved score. Independent generalization
and valid leaderboard evidence remain required for the original goal.
