# Phase402 — backend prototype, verification pending

Added default-false `use_native_tdcp_frequency_residual_states` and an explicit
prior sigma (default zero, invalid when enabled). Constructor rejects wrong
backend/main selector and missing positive prior. Backend additionally requires
native IMU C7 main, ordinary nonlinear corrected-carrier TDCP, matching clock
metadata; raw staging, affine and atmosphere-bypass variants are rejected.

Pairs use the Phase401 helper. Each pair gets one zero-initialized `u` key and
one Gaussian prior. Existing native TDCP factors are replaced in-place by the
Phase399 wrapper, retaining original scalar noise. Unmatched rows are unchanged.
Counts require exactly two wrapped factors per state. Backend TDCP residual
RMS includes the optimized residual-state contribution.

Added a paired-graph branch in the existing native Phase171/213/217 synthetic
handoff test, checking OFF counters, one state/two wrapped factors, convergence,
finite residual RMS and invalid zero prior rejection. Not executed yet.

At this checkpoint app/library build session 3784 and backend-test object
compilation session 51343 are live. Object target `/tmp/gnss_phase402_backend_tests.o`.
Do not claim successful build/tests yet; poll these exact handles first.
After completion link against the rebuilt libraries and run the focused
Phase171 main test. `git diff --check` passed.

No CLI selector is exposed yet. App postfit TDCP reconstruction still lacks
the new state contribution, so do not expose/raw-run this option before fixing
that output contract. GNSS-first must have the flag explicitly OFF when main
configuration is copied. Raw timing provenance, physical prior-scale selection,
default replay and real-data evaluation remain required. No accuracy gain,
truth reads, MAT inputs or saved positioning inference this phase.
