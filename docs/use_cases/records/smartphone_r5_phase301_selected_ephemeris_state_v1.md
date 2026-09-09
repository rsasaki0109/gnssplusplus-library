# Phase301 propagation from a selected broadcast message

Starting HEAD 7d34e07f, root-only.
Pinned MALIB ephemeris.c ephclk/ephpos were read by HTTPS. Both select using
teph while evaluating at the transmission time. ephpos uses a 0.001 s
forward difference for velocity and clock drift. The native existing
Ephemeris API uses 0.5 s central differences for Keplerian states instead.

Added stateFromSelectedEphemeris: caller-selected message -> transmission
time -> native propagation at t and t+0.001 -> finite-checked position,
velocity, clock and drift. The same Ephemeris object serves both evaluations;
no hidden nav lookup can change it. It retains native orbital equations and
does not claim source ephemeris selection, Galileo constants, full orbit
parity, source zero-clock fallback, health admission or receiver-frame/Sagnac
parity. Production builders do not call it yet.

Synthetic accelerating SBAS state demonstrates the forward-difference term
(2.001 m/s rather than instantaneous 2 m/s), position and zero clock, with
nonfinite-state rejection. This is not an H accuracy result.

Next: supply an explicitly qualified selection-time policy from NavigationData,
test an ephemeris-switch boundary, then connect the epoch/satellite grouping.
No raw, truth, candidate, MAT, station-table or Kaggle/token input. Only source
code fetched; existing tests use synthetic fixtures. No trajectory or score.

Validation: gnss_run_tests target built successfully; all 28 base-compensation
tests passed. Not full CTest or real-data/source-orbit parity.
