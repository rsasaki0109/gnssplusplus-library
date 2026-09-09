# Phase65 native base pseudorange compensation

Phase65 froze the source-supported `taroz/gsdc2023` base-station pseudorange
compensation as an opt-in native FGO candidate.  The implementation is
restricted to adopted undifferenced pseudorange factors:

`P_rover_corrected = P_rover - pc(t)`

where `pc(t)` is the same-satellite/same-signal base residual after the native
broadcast satellite-clock, Klobuchar, Saastamoinen, and group-delay model,
centered `movmean` (151 samples at observed 1 Hz or 11 at observed 15 s), and
in-domain linear interpolation.  No new rows or factors are created, and SPP,
TDCP, Doppler, IMU, base-position CSV, and base-offset CSV paths are untouched.
The candidate is enabled only with
`--native-base-pseudorange-compensation --native-base-rinex <member> --native-base-rinex-sha256 <digest>`;
the flag-off path retains the Phase43 recipe.

The implementation source/test stage was sealed in `f8cdf3b`; the structural
runner and manifest were sealed in `b3a02ef`.  The one attempted matrix stopped
after the first MTV-a flag-off control because the runner had a nested-hash
presentation bug while comparing its Phase43 reference.  The native control
artifact itself is byte-identical to Phase43 (submission SHA
`d4d7652e…c5426c`, summary SHA `d4980260…2303fe`).  No candidate invocation,
base parse, accuracy score, truth read, MAT read, or 0.782 claim resulted.

This is recorded as an evaluator-integrity fail-closed result in
`smartphone_r5_phase65_native_base_pseudorange_compensation_structural_failure_v1.json`.
The remaining structural matrix and any accuracy evaluation require a new
immutable recovery freeze; Phase43 remains the champion and Phase51/58 remain
experimental.
