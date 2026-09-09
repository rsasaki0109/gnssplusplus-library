# Phase464 — held-out clock negative controls

Added `RawPSeedTest.HeldOutClockCannotCertifyCorrelatedSupportOrMissingGroup`
using the same actual-native synthetic fixture and settings as Phase463.
Target Galileo satellite is entirely excluded from the position/clock solve.
The three remaining Galileo observations each receive +100 m code error;
GPS support remains clean. RAIM and iterative outlier detection remain ON.

Results:

- Estimated Galileo relative bias is approximately 117 m instead of the
  physical synthetic 17 m: the support error is absorbed by the free clock.
- Clean held-out target: residual -100 m, rejected by the 20 m predicate.
- Target with the same +100 m error: residual approximately -0.000107 m,
  accepted. The test explicitly asserts this counterexample.
- Removing all remaining Galileo observations leaves its bias unobserved,
  unavailable and nonfinite, even though the GPS-only solve succeeds. No
  zero-clock fallback or prediction is used.

All 30 RawPSeedTest tests pass; standalone build and `git diff --check` pass.
No production changes, Kaggle truth, real raw replay or submission.

Decision: leave-one-satellite-out prevents direct self-fitting but is not an
independent certificate against correlated contamination among the support
observations. Do not use it alone to re-admit rejected native P factors.
Within one free system-clock group, a common code offset and clock offset
are observationally indistinguishable. This is not necessarily position
damage when a nuisance clock absorbs the common mode, and these tests do
not prove any specific LAX error source. The mixed clean/biased target
counterexample does disprove interpreting a small held-out residual as a
general code-quality certificate.

This closes the simple clock-aware hard re-admission candidate: Phase463's
positive example is insufficient. Further progress should use the native
graph's temporal constraints and uncertainty, or model these rows robustly
without treating a fitted clock as a clean-observation label. Repeated
threshold tests on the two sparse epochs are not an accuracy strategy.
Any new candidate needs an explicit full-route, multi-route development
comparison and a separate untouched evaluation boundary before scoring.

The 0.782-class and leaderboard goals remain active and unmet.
