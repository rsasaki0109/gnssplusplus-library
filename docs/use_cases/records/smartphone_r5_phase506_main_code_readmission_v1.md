# Phase506 — default-off main code readmission implementation

FGOProblem now retains a separate complete P-factor pool for labelled masked
rows. Builder reuses ordinary factor construction (satellite state, corrected
code, signal/clock group, sigma, GLONASS provenance), while excluding masked
pool rows from ordinary epoch factors, GPS clock-jump tracking and baseline
residual medians. After ordinary residual filtering it maps retained input
epochs exactly and stores only baseline-residual-passing pool rows.

CLI --native-main-code-edge-readmission is default off, raw all-epoch Phase171
ECEF-D only, forbids NHC/frequency states/code-floor combinations. Inserts the
pool only after GNSS-first and main preparation, before main preflight/solve.
Duplicate identities, invalid epoch indices, nonfinite satellite/code and
invalid sigma fail closed before insertion. No serialized inference inputs.
Preserves original normalized robust P noise; no new multiplier or threshold.

243 H insertions expected from the diagnostic, not yet measured from the full
pool. Builder counters for original admitted P remain baseline counters; new
CLI insertion marker and actual graph count must verify extra factors.
GNSS-first, original masks/medians and epoch admission must remain unchanged.
Candidate is a hypothesis: temporal self-consistency does not prove no bias.

Build session 87110 started; git diff --check clean. FGOProblem layout changed,
so dependent binaries must finish rebuilding before running. No candidate
native run, invariant verification or accuracy scoring yet. Goal unmet.
