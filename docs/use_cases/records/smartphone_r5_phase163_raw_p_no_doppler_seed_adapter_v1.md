# Phase163 same-run raw-P no-Doppler seed adapter

This record describes the bounded Phase163 implementation. It is a
preparatory handoff lane, not a no-Doppler FGO result and not an accuracy
evaluation.

## Contract

- `--native-phase163-raw-p-no-doppler-seed-stage` runs the existing native
  raw-P SPP stage in the same process and exits before FGO.
- The private SPP copy clears Doppler. Positions and velocities therefore
  come only from accepted same-run raw-P epochs and the existing explicit
  one-sided-endpoint/centered-interior timestamp policy.
- D is copied only from the exact original in-memory
  `ObservationData::receiver_clock_drift_mps` at the matching epoch identity
  (week/tow, raw source index, and raw UTC key). Missing, nonfinite, or
  misaligned values fail closed; no finite-difference, hold, interpolation,
  or zero fallback supplies D.
- C7 availability is explicit. Native GPS-reference-only output certifies
  C[0]; the remaining six frequency slots are unavailable (`NaN`/false).
  A non-GPS reference or any observed non-GPS constellation group disables
  graph compatibility because the constellation-level SPP ISB has not been
  proven to map to the source frequency slots. No ISB is manufactured.
- The app writes aggregate/finite-status metadata only, sets
  `fgo_entered=false`, and never reads a saved seed or solution file. No
  graph topology or existing default recipe is changed.

## Source-backed boundary and limitations

The native source clock state is the seven-component C vector with C[0] as
the shared base/GPS-L1 component. The existing SPP result exposes a
constellation-level reference clock and ISBs, but does not expose enough
frequency-slot provenance to certify C[1..6]. Accordingly, this change stops
at a typed adapter. The official XXVV/CCDD graph, no-Doppler factor admission,
and stage/main C/D/V export remain unimplemented and must not be inferred
from this record.

## Validation

The focused synthetic suite passed 29/29 tests: the existing 26 raw-P tests
plus three adapter tests covering finite same-run GPS C[0] handoff, missing /
nonfinite / identity-misaligned D rejection, and mixed-clock C7 graph
disabling. The affected native executable
`gnss_fgo_imu_no_base` and aggregate C++ test target both built successfully.
No raw route, truth, MAT, Kaggle, or real FGO run was performed for Phase163.
