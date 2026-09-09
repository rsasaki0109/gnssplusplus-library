# Phase331: raw stationary-base residuals have a position-like component

Inspected base and rover equations: both apply satellite clock in metres,
subtract modeled ionosphere/troposphere, and paired mode has zero explicit
group delay. Base residual additionally subtracts its geometric range; rover
then subtracts the resulting correction. No obvious sign/unit inconsistency
was found in these expressions, but this is not numerical source parity.

Compiled and froze a raw-only stationary-base diagnostic before one execution.
All source/binary/base/nav hashes verified. No phone, truth, candidate, station
table, MAT or saved-positioning input was used. The dense source base model
was built from 3500 raw epochs. At each epoch, healthy GPS L1 corrections
above 15 degrees elevation were fit by unweighted rank-4 QR to `[-LOS, 1]`.
Only aggregate norms and residual magnitudes were emitted; estimates are never
fed back into the base reference or smartphone inference.

All 3500 epoch fits succeeded, with 24212 admitted satellite rows. Median
apparent displacement norm was 2.764934258432287 m, median absolute fitted
clock 1.3798841277800709 m, and median post-fit RMS 0.9922099606100028 m.
The difference between first- and second-half mean displacement vectors had
norm 1.071621332450647 m. This shows a geometry-correlated correction component
at metre scale, not a certified station position error. The variation and
remaining residuals prevent treating it as a known constant reference shift.

The design is a first-order diagnostic, not a full static PPP estimator:
orbit/atmosphere errors, tracking biases, smoothing and multipath can all
project into its fitted displacement. GPS-only, a fixed elevation cutoff and
unweighted least squares further limit interpretation. No uncertainty bound,
survey accuracy, source residual parity, or smartphone accuracy improvement
has been established. No new test suite was run; standalone compilation and
the frozen raw diagnostic succeeded.

Next test this diagnostic against synthetic known clock/reference errors and
compare frequency/atmospheric contributions with preregistered raw-only fits.
Do not shift the base coordinate by this fitted vector or fit against H truth.
Operational baseline remains unchanged; the 0.782/LB objective is still open.
