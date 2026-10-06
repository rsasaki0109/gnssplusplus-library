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

2026-10-06: all four implementation/evaluation deliverables are complete.
[ppc_development_results.md](ppc_development_results.md) is the final evidence
ledger with reproducible commands, machine-readable results and regression
limitations. This completion does not approve default adoption of the FIX
guard: the frozen six-run comparison returns NO_GO and the default stays off.

The native full replay and repeat both score all six runs and 18 streams;
every repeated POS numeric field matches. The integrated default-off RTK path
also matches the original baseline on all six runs. The received-event path
passes real-position prefix invariance for 300 of 600 Tokyo 1 epochs, five
typed scenarios and two streaming executions, including actual tight-filter
updates, late/missing base input and fresh initialization after an IMU gap.
The public Python example exports 9,432 rows and 33 satellite plots from 300
epochs. Frozen historical tiers, closed holdouts and the paused smartphone
development lane are unchanged.

All default native targets build, all C++ CTest suites pass, and the CLI,
benchmark, binding and install/package checks pass in the Windows environment.
The full 137-lane CTest run retains nine unrelated smartphone environment/
fixture failures, documented individually in the final ledger; it is not
reported as a completely green suite. ROS2 is unavailable and unverified.
