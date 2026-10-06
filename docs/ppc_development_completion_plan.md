# PPC reproduction integrity online fusion and Python analysis

The active development goal covers four deliverables: regenerate PPC evaluations
from raw inputs, improve wrong-FIX detection and recovery, provide causal RTK/IMU
processing, and expose useful satellite observation analysis through Python.
Implementation, real-data evidence, regression checks and operating instructions
are required for each deliverable. Publishing or merging changes is outside scope.

## Raw input reproduction

Add a source-tree command that builds explicitly selected native executables,
runs RTK and RTK/IMU on all six existing PPC development runs, and scores their
outputs separately. Record input hashes, source contents, exact argument arrays,
build configuration, executable hashes, logs, timing and artifact hashes. A
bounded smoke is labelled as such; the full evaluation has no epoch cap. Repeat
the same recipe and verify position and status reproducibility.

Historical selected-tier outputs have incomplete solver provenance. Preserve
their existing score-only reproduction and report the fresh native results as
a separate baseline. Do not imply that new outputs reconstruct historical tiers.

## Wrong FIX detection and recovery

Use fresh baseline outputs and runtime diagnostics to classify wrong-FIX events.
Offline reference labels must never enter the detector. Record both 0.5 m and
2 m 3D error populations, correct-FIX loss, missing outputs, recovery delays,
unrecovered events, horizontal P95 and runtime. Implement the selected detector
and recovery state machine behind explicit options. Verify disabled behavior,
interruptions, repeated suspect evidence and clean-evidence recovery. Evaluate
with the same binary and input population and publish the adoption decision.

## Causal RTK and IMU processing

Audit base alignment, secondary-code preprocessing, IMU initialization and output
timing. Add a sequential-input mode that cannot consume later observations,
including during initialization and reacquisition. Verify prefix invariance by
comparing outputs from identical input prefixes followed by different suffixes,
and check late/missing base data, IMU gaps, reinitialization and declared latency.
Distinguish causal solutions from delayed or batch products in output metadata.

## Python satellite observation analysis

Build the public bindings and add a runnable real-data analysis route using
satellite identity, corrected pseudorange, SNR, carrier phase, Doppler and
satellite motion. Export per-satellite residual and observation time series,
slip indications with their actual evidence, summary JSON and plots. Document
units, receiver-clock removal, missing observables and the limits of heuristic
slip detection. Verify results against independent synthetic examples and a
bounded PPC replay.

## Evaluation scope

The six PPC runs are existing development data. Their results are regression
and development evidence, not new held-out generalization evidence. Closed
application holdouts remain closed. A failed adoption gate is retained honestly;
it is not permission to promote a default or claim a measured improvement.

## Current state

2026-10-06: audited the existing replay manifests and source entry points.
The `ppc-goal` lane consumes 26 historical frozen tier files; complete solver
regeneration is absent. The fixed-lag covariance report identifies whole-file
secondary-code preprocessing and future base interpolation as causality gaps.
Python corrected measurements expose identity and raw observables but do not
yet provide the requested satellite analysis workflow. Native replay work is
in progress on `feat/ppc-reproduction-integrity-online-tools`.

The raw-replay slice is a sign-off improvement: its user-visible value is a
fresh RTK/IMU baseline with reconstructable provenance. It changes the benchmark
wrapper, dispatcher, Python test registration and replay guides; solver behavior
and historical benchmark claims remain outside this slice. It requires local
PPC input data and an explicitly configured CMake build, with no account or
credential dependency. Acceptance requires successful full six-run generation,
same-condition repeated POS data, and negative tests for stale output and
provenance mismatches. The focused ten tests and the existing 32 reproduction
tests passed. Native rebuilding and real-data validation are pending.

The native executables now build successfully. A 120-epoch Tokyo run1 smoke
regenerated and scored three streams successfully. The uncapped six-run replay
is running from frozen commit `b34cf3e7`; it must finish and then be repeated
before raw-replay acceptance. Tokyo run1 currently has 784 wrong FIX epochs
above 0.5 m and 217 above 2 m. The offline audit records 1,702 missing or
unmatched epochs relative to the 11,928 admitted rover inputs. These figures
describe the new native baseline, not historical selected tiers.

Further implementation is isolated in `feat/ppc-integrity-online-analysis` at
`E:/gnsspp-goal-development`, so evaluation source contents remain stable.
The satellite analysis API and CLI have fourteen passing independent tests.
The public extension builds and a bounded PPC example exports 9,432 rows and
33 satellite plots, with G05/C11 visually reviewed. The broader regression
checks and final integration remain pending. The
received-event RTK/IMU processor and stdin RTCM/IMU executable compile. Its
nine queue/reset tests pass. Numerical prefix
parity and late/missing-input real-data evidence remain required.

The new offline native integrity audit has four passing tests for two error
thresholds, missing-output costs, status demotion versus actual accuracy, and
right-censored recovery. On Tokyo run1, 247 of the 784 wrong FIX epochs have
none of the audit's named runtime warnings. This limits simple threshold
detectors. `FixRecoveryGuard` and the optional native `--fix-recovery` path
are now implemented, with immediate hard-residual containment, repeated joint
prefit evidence, one reset on quarantine entry, and consecutive clean-candidate
recovery. Its seven native tests pass, and the disabled path preserves every
POS numeric field on the 120-epoch PPC smoke. Six-run comparison and adoption
decision remain pending. No deliverable is signed off
solely from the bounded smoke or these partial-run observations.
