# Phase318: paired epoch states in the IMU CLI

Added default-off `--native-paired-epoch-states` to `gnss_fgo_imu_no_base`.
It selects source header tracking and preserved base bands, source per-epoch
base states with FGO slot filtering, and shared rover states with symmetric
zero explicit group delay. Base reference requires RINEX3, finite approximate
position, and present zero antenna delta. It uses the existing in-memory
finite-correction application before GNSS-first and IMU/main construction.
The option is written into both selector and top-level summary provenance.

This is separate from the historical Phase126 CLI compound selector, whose
route/recipe restrictions are unchanged. Initial admission requires raw H
Pixel5 all-epoch UTC/meter-handoff with base and miss-mask flags. Legacy
Phase126-131, external extra-band flag and signal-specific TGD combinations
are rejected. Existing base hash syntax, MAT and other recipe checks remain.
The library's older zero-code-bias field is reused internally; this does not
claim the historical Phase126 CLI compound admission was executed.

Validation: built `gnss_fgo_imu_no_base`; five executable CLI tests passed.
The Phase234 argument list plus paired/base/miss-mask arguments reaches raw
ingress using deliberately absent paths. Missing mask, other route, legacy
Phase126 mix, and MAT base are rejected. No raw payload was opened, no output
was produced, and no optimization ran. An initial diagnostic invocation lacked
GTSAM runtime library search paths (exit127); after appending the documented
library path the parser tests passed. This was not a restarted solver run.

These tests prove argument admission and binary build, not full downstream
graph admission, successful IMU optimization, or output/score parity. This is
not full CTest or a real-data byte-identity check. Native geometry/orbit and
raw header station reference limitations from Phase317 still apply.

Next freeze the complete paired H raw-only run (including IMU and all binary,
source and input hashes), run once into new output paths, inspect graph and
solver diagnostics, then freeze a one-shot evaluator. No truth or saved
positioning input was accessed in this phase; the performance goal is open.
