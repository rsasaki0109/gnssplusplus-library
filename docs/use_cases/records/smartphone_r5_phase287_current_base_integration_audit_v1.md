# Phase287 current baseline / raw-base integration audit

Inspected at clean HEAD `69e18ec`, directly by the root agent; no delegation.
This is source and historical metadata evidence, not a new accuracy run.

## Findings that change the next action

- Phase126's design audit is not the current implementation status. The
  native base implementation already consumes its source-complete station
  reference validator and source geometry/residual functions. Reimplementing
  that design from scratch would duplicate existing work.
- Current CLI admission (`gnss_fgo_imu_no_base.cpp:2081`) restricts the
  Phase126 source-complete selector to MTV-A/LAX-T and the Phase118 recipe.
  It requires Phase118 TDCP Huber and rejects upstream-quality selection.
  The operational Phase234 H manifest explicitly records `base: false`.
  Thus Phase126 cannot simply be enabled on that baseline; removing the
  route guard alone would not establish a compatible measurement contract.
- A raw H base member is recorded in Phase147: 11,854,125 bytes, SHA-256
  `4d3e37cbe0347fa56216db54ede9e0f30731885f337f1653ab5a86afb2bb2150`,
  under the Phase63 recovery route directory. This turn did not verify the
  current file or read its header; historical presence is not current proof.
- Phase147 H used the older compatibility base selector, not Phase126.
  Its return code was -6 at main-optimizer admission. Phase148 explains the
  missing retained Doppler/state admission problem. That failure is neither
  a score nor evidence that base correction degrades the current H graph.
- The source `correct_pseudorange.m` computes base residuals before calling
  `posbase.addOffset`. It reads station tables. This does not prove that
  applying a RINEX antenna offset reproduces its measurement reference;
  do not substitute station tables or guess a coordinate correction.

## Next implementation boundary

Trace current stage/main P preparation and base-application ordering,
including the Phase165 same-run seed, the upstream residual mask, code-bias
convention and correction misses. Then define a default-off integration
using the existing raw-base primitives with a proven station reference and
synthetic stage/main count and measurement tests. Preserve old Phase126
admission and historical manifests. Do not run a compound legacy recipe or
relax its guards merely to get a new H score.

Raw base header/identity validation remains necessary before a frozen raw
candidate. No new solver, evaluator, truth, candidate coordinate, MAT,
station-table payload, raw payload, Kaggle or token access occurred here.
Historical aggregate records were read; no metric was recomputed. The
0.782-class and leaderboard objectives remain unproven.
