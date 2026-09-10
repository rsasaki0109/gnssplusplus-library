# Phase311 raw correction support attribution

Frozen at e59b7b66. All input/source/binary hashes verified. One raw hash scan
and native pass per base/nav/phone. Same-process correction only; no truth,
MAT, saved positioning, station table, network or Kaggle/token access.

Aggregates exactly match Phase310; each signal count conserves.

| Signal | P rows | Missing stream | Unavailable lookup | Finite |
|---|---:|---:|---:|---:|
| GPS L1CA | 25666 | 0 | 473 | 25193 |
| GPS L5 | 19962 | 0 | 0 | 19962 |
| GLONASS L1CA | 11908 | 549 | 0 | 11359 |
| Galileo E5a | 15474 | 0 | 595 | 14879 |
| Galileo E1 | 16089 | 0 | 751 | 15338 |
| BeiDou B1I | 19623 | 19623 | 0 | 0 |

Phase298 raw-base header declares no BeiDou observations. Thus all B1I
stream misses are consistent with absent source observations, not a reason
to alias B1I to another constellation or generate zero corrections.
GLONASS satellite identity and remaining lookup failure reasons need further
attribution. Do not claim every unavailable lookup is temporal extrapolation
without inspecting the model's domain checks.

Next port the source finite-correction miss admission into the current rover
recipe, preserving absence explicitly, after paired satellite/code-bias
preparation. These are raw pre-FGO counts, not final factor support or scores.
Missing corrections cannot authorize carrying uncorrected P rows under a
source-equivalent label. Existing operational baseline remains unchanged.
Diagnostic rebuilt successfully; no tests/solver/accuracy evaluation this turn.
