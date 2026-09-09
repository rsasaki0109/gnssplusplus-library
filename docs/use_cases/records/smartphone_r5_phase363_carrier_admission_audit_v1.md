# Phase363 — remaining carrier admission differences

Primary-agent code inspection only; no truth, MAT, positioning payload,
native run or submission. Phase362 rejected the gyro initialization change.

The historical Phase120 statement that native lacks dDL masking is no
longer current: `observable_upstream_preprocessing.hpp::applyAdjacentMasks`
implements the carrier-Doppler residual and two-sided rejection, called by
`src/algorithms/fgo_problems.cpp`. Do not implement another dDL gate on the
assumption that it is missing.

Two remaining differences warrant a raw support audit, not immediate tuning:

1. Source `functions/exobs.m` first masks carrier by tracking/SNR/multipath,
   then computes a dense adjacent-epoch diff in cycles and rejects only the
   current endpoint when abs(diff(L)) > 20000. No equivalent explicit cycle
   threshold was located in the inspected Android parser or shared native
   preprocessing helper. A code-phase jump gate is a different equation.
2. Native applyAdjacentMasks stores the previous observation by satellite
   and signal, requiring only positive dt <= 1.5 s. It does not require
   prior.epoch_index + 1 == current_index. A missing middle epoch at a
   subsecond cadence can therefore be bridged; the source dense diff sees
   NaN instead. This cannot be assumed to affect H's roughly 1 Hz data.

Next count the raw H candidate pairs satisfying the source 20000-cycle
predicate, distinguish already rejected carriers from newly affected
support, and count nonadjacent pairs accepted by the native time bound.
Preserve source order: carrier quality masks precede the cycle jump mask,
which precedes residual masking. No raw threshold sweep, new score or
solver change is justified until the audit shows affected retained support.
These code differences are not evidence of an accuracy improvement.
