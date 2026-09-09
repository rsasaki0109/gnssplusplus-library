# Phase271: in-memory relative-height pair selector

Implemented `selectSourceSampleSumRelativeHeightPairs` in
`include/libgnss++/fusion/relative_height_pairs.hpp`. Inputs are a single
vector of epoch/position/velocity/stop records, so parallel-array length
mismatches are unrepresentable. There is no file reader. Same-run provenance
must still be enforced by the future native integration caller.

The explicitly named convention is cumulative speed samples, not integrated
distance. Defaults are strict distance <15 m and speed-sample difference
>100, excluding stopped endpoints. Pairs have i<j and deterministic order.
Finite vectors, normalized finite time-of-week, strictly increasing epochs,
finite accumulated speeds, and valid thresholds are checked.

Build session 76408 completed with exit 0. Two new selector tests and twelve
existing FusionInitializationTest tests passed (14 total). Coverage includes
strict proximity and travel boundaries, both stopped endpoints, irregular
sampling invariance, duplicate epochs, nonfinite velocity/position, empty
input, invalid proximity, and unique pair ordering. No full CTest claim.

This helper is not yet connected to the graph. No production recipe change,
native raw run, truth read, MAT access, saved coordinate input, or accuracy
claim. Next implement/test the scalar local-up factor and integration guards.
The selector is quadratic, matching the source enumeration; assess cost
before broader batch use without silently capping or changing pair selection.
