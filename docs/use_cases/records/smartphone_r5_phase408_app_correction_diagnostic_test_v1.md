# Phase408 — actual app correction-diagnostic failure controls

Added `tests/test_native_tdcp_correction_diagnostics.py`. It extracts the
actual current TdcpRuntimeReport/evaluateTdcpRuntime/Huber helper and constants,
compiles them against the real FGO header, then executes synthetic in-memory
assertions. No solver/raw data, truth, MAT or saved positioning input used.

Test passed (one unittest containing multiple C++ checks):

- Baseline synthetic residual RMS .02 m; with .01 m exported correction the
  actual app report produces .01 m, preserving one finite factor/group.
- Enabled mode without export fails.
- Wrong endpoint, satellite or signal identity fails.
- Nonfinite correction fails.
- Disabled mode with an unexpected correction export fails.

This complements Phase407's full-backend nonzero-state/RMS test; it does not
constitute a raw CLI integration test or broad numerical accuracy proof.
The compiled fixture code is piped to the compiler, not a measurement or
positioning intermediate. Temporary executable is scoped to the test.
`git diff --check` passed. CLI still unexposed and defaults OFF.

Next establish native epoch-time versus factor-dt consistency and raw
cross-band measurement-time provenance before exposing the opt-in experiment.
Prior-scale choice and disabled raw replay also remain. Goal remains unmet.
