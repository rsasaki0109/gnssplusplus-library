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

2026-10-06 checkpoint: the raw command regenerated and scored all six runs
and 18 RTK/fused/coupled-RTK streams from frozen commit `b34cf3e7`. Its same
recipe repeat is still running; exact repeated POS data is required before
acceptance. The 26 historical tiers remain a separate score-only lane.
The replay's twelve focused tests and the existing 32 reproduction tests pass.

Implementation continues in `feat/ppc-integrity-online-analysis` at
`E:/gnsspp-goal-development`, keeping the first evaluation source frozen.
The optional default-off FIX guard has seven native mechanism tests and a
120-epoch disabled-path POS parity check. The offline native integrity audit
has six tests and labels all six fresh baseline runs. Its strict paired
provenance check rejects differing source, binary, input, settings, environment
or effective arguments. Full six-run off/on comparison and the declared
adoption gate in [fix_recovery_guard.md](fix_recovery_guard.md) remain pending.

The received-event RTK/IMU processor and stdin RTCM/IMU executable build.
The raw Tokyo run1 verifier passed 600 epochs with five typed-API scenarios
and two streamed CLI executions. Numerical/metadata prefixes match exactly
for 300 epochs, excluding measured wall time. Normal typed processing produced
600 valid RTK positions, 590 fresh fused positions, and 103 tight time updates.
Missing/late base data, a four-second IMU outage with fresh reinitialization,
and 0.15 s delayed rover delivery are verified. A 1 Hz base/5 Hz rover interval
bug found by this real test was corrected. Four historical RTCM context tests
are also registered; full native regression checks remain required.
This is a declared received-event simulation, not a live PPC reception trace
or a new full-run accuracy claim.

The public Python satellite analysis API and CLI have fourteen independent
passing tests. The built extension's bounded 300-epoch PPC example exports
9,432 rows and 33 satellite plots. G05/C11 plots were visually reviewed,
including gaps and unavailable carrier observations. Python binding smoke
tests pass with eight historical-data cases skipped. The broader benchmark
runner passed 181 cases (one skipped); full native/CLI/packaging checks and
final integration are ongoing. ROS2 dependencies are unavailable on this
Windows build, so its runtime node test cannot be performed here.
