# Phase204: IMU bias audit

Status: source audit and synthetic verification complete. Work was handed
from Luna Max to the primary agent at the user's request. No production solver
changes, raw-data runs, or accuracy evaluations are part of this audit.

## Evidence

- Cached `fgo_gnss_imu.m:294` uses `ImuFactor` with the ending bias B2;
  native `CombinedImuFactor` corrects the motion residual using starting B1.
- Source lines 155 and 298 give the separate bias-between factor covariance
  `N * sigma²`. Native passes `sigma²` as continuous-time bias random-walk
  covariance. An actual GTSAM preintegration test with 53 samples spanning one
  second verifies native bias marginal covariance `T * sigma²`, a ratio of 53.
  This is a synthetic example, not a measured uniform H sampling rate.
- With equal biases, the unwhitened nine motion residuals and their Jacobians
  agree. Changing endpoint bias makes the source and native motion residuals
  differ. This does not establish equal likelihoods or equal graph optima.
- Combined's bias residual is B1 minus B2, whereas the separate between factor
  uses B2 minus B1. Their Jacobian signs reverse. Initial test expectations were
  corrected after observing this in the linked GTSAM implementation; production
  code was not changed.
- Installed GTSAM 4.3 headers mark `biasAccOmegaInt` unused. An old-version
  assumed identity contribution must not be used to explain this build.
- Source line 186 uses infinite-sigma bias priors; native puts a finite prior
  on its first bias (accelerometer sigma 0.1, gyro sigma 0.01). This is a separate
  difference from random-walk scaling and endpoint selection.

All four synthetic tests passed after a fresh build. The fourth verifies nonzero
motion/bias cross covariance and a motion covariance differing from the separate
preintegrator. Reversing bias residual sign also reverses cross covariance; it
does not remove that correlation. These tests use controlled synthetic noise
values, not an estimate of H sensor noise.

Verification: `cmake --build build --target gnss_run_tests -j2` completed with
exit 0. Filter `FGOGtsamPhase204ImuBiasAuditTest.*:GtsamImuTimeIndexingTest.*:*Phase171*`
passed 18/18 tests; `git diff --check` passed. This is not a full CTest run.

## Next experiment

The best H development recipe remains Phase198/199 (1.2751561666667786 m).
Phase201's integration schedule worsened it to 1.2978799269779575 m in Phase203;
leave that selector off. Keep the -20 ms correction on and Phase194 noise off.

First isolate bias random-walk scaling while retaining CombinedImuFactor,
starting-bias correction, initial priors, and the legacy integration schedule.
An experimental per-interval density `Q = (N / T) * sigma²` would match the
source bias marginal `N * sigma²`, but still changes native motion/bias cross
covariance and is NOT equivalent to the source's separate-factor model.
Define N explicitly before any run; do not substitute a tuned constant sample
rate. Review and synthetic validation must precede the separately frozen raw
comparison. Endpoint selection and initial priors remain later independent
ablation candidates.

None of these findings explains the H error causally or proves improvement.
Broader route evaluation is required; H is development data, not held-out or
leaderboard evidence. No MAT file or saved trajectory is a solver input.
