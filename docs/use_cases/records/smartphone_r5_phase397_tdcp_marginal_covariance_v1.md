# Phase397 — C++ frequency-correlated Gaussian primitive

Implemented `tdcp_frequency_covariance.hpp`, currently not called by a
production solver. Inputs are two residual standard deviations [m], signed
frequency coefficients [m/m], and an explicit finite nonnegative variance
for a zero-mean slant-change nuisance [m²]. No default tuning parameter,
raw-file I/O, saved-state input, inferred ionosphere value or truth fitting.

For r=a*dI+epsilon with independent Gaussian epsilon and dI~N(0,q), returns
C=diag(sigma²)+q*a*a' and Cholesky-based whitening W=L^-1. Invalid, overflowing
or non-positive-definite inputs fail instead of silently changing the model.

Fresh standalone C++ test binary passed 8/8: four prior observability controls
plus zero-q diagonal recovery, agreement with explicit nuisance least-squares
elimination, band-permutation cost invariance, whitening identity, correlated
covariance, and invalid/overflow rejection. Existing test source registration
covers these additions, but full CTest and production integration are unrun.

The equality is a Gaussian quadratic-cost equality for fixed covariance.
It is not equality to independent per-band Huber losses after marginalization.
If covariance parameters are optimized, the Gaussian normalization/logdet
also matters; this primitive does not estimate covariance parameters.
Do not append whitened/transformed observations to the original pair as
independent information. No precision or accuracy improvement claimed.

Before graph integration: decide explicit robust nuisance state versus a
joint robust factor, trace the already-applied carrier atmospheric sign,
define exact same-satellite/endpoint pairing, handle unmatched observations,
and freeze a physically justified prior without development-score sweeping.
This header supplies the covariance algebra, not those missing contracts.
The native-only 0.782-class / leaderboard objective remains active.
