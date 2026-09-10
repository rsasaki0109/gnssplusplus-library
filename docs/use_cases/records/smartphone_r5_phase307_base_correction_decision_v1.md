# Phase307 raw correction build rejected

Initial manifest e18c7963 was accidentally generated from truncated tool
output. Corrected and JSON-validated at 158167f1 BEFORE any real input read.
No initial raw execution occurred. One attempted combined delete/add patch
was rejected without mutation; subsequent replacement restored the manifest.

Source/binary/base/nav hashes verified. One hash scan and one native read per
real input. Diagnostic exit 0 reports model failure, NOT successful build.
All 3500 epochs loaded; 68697 states and 186827 signal residuals processed.
Failure: source-complete correction stream has duplicate/non-monotonic time.
The recorded 39 streams is progress telemetry, not published usable output.
The model's transaction does not expose a successful correction map.

Next investigate code-to-stream aliasing: native SignalType can represent
both Galileo C7X and C8X, both present in H (Phase299). This is a hypothesis
supported by code mapping, not yet a per-stream forensic attribution. Do not
remove duplicate protection or arbitrarily discard a code to force success.
Resolve physical-frequency/correction identity and intended source frequency
admission before another frozen run.

No phone, truth, candidate, MAT, station table or saved-state input. No
positioning solver, accuracy calculation, network or Kaggle/token access.
Raw header reference and zero antenna delta are the explicit station
convention, not proof of equivalence to source station tables. Goal incomplete.
