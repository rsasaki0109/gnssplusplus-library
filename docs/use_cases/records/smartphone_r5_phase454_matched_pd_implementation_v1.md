# Phase454 — identity-matched P-D diagnostic implementation

Added `matched_pseudorange_doppler.hpp` and native builder aggregate logging.
The builder retains an explicit problem-epoch to input-epoch mapping when
each seed is admitted. Every pre-residual-mask P factor is matched on exact
satellite and signal in that input epoch and its immediate neighbors.
Statistics are separated by retained/rejected population and incoming/outgoing
direction. Only epochs with fewer than eight final P factors are logged,
using the existing diagnostic selection, not a new measurement gate.

Missing, ambiguous, nonfinite and out-of-gap pairs are counted as missing;
they are not zero-filled. Available pairs use the existing native P-D
expression, current signal wavelength and a finite 0 < dt <= 1.5 seconds.
This is explicitly an immediate-neighbor diagnostic, not a replacement for
the upstream mask's last-seen-observation traversal. No position series or
satellite coordinates are emitted. Model, thresholds and factor membership
are unchanged by design, but output identity still needs a fresh raw replay.

Verification completed:

- Standalone C++ tests: 2 passed (identity, duplicate identity on either side,
  missing Doppler, invalid intervals and values, missing versus zero, code
  jump signs and consistent nonzero range rate).
- `cmake --build build --target gnss_fgo_imu_no_base -j1`: exit 0.
- `git diff --check`: passed before the final test/document additions.

No raw replay, truth read, accuracy score or submission in this phase.
Next: freeze new source/test/binary pins and a new output directory before
one LAX-T replay; require baseline output identity and per-population count
conservation in both directions before interpreting any aggregate.
Do not reuse Phase451 pins: its verified historical binary/source predate
this change. The Phase451 result remains recorded in Phase453.

The diagnostic primitive tests do not establish end-to-end input mapping
correctness or performance improvement. Full CTest/PPC verification remains
outstanding. No build or native process from this phase remains running.
