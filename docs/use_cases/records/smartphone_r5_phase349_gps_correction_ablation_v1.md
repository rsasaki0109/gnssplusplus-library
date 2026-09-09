# Phase349: GPS-values-only base correction ablation

Added default-off `--native-base-gps-values-only-ablation`. It requires the
existing paired H raw epoch-state recipe and dense base smoothing, rejects
the mutually exclusive mask-only mode, and inherits all existing raw/MAT
and paired-route guards. Both summary selector and top-level provenance
report the new flag. Nonzero correction enabled remains true because GPS
values are still applied; the new flag is necessary to identify their scope.

`gpsValuesOnlyAblation` queries the full base model first, preserving false
and nonfinite results, and changes available finite non-GPS values to zero.
Thus this is not GPS-only observation selection: other-system P/D/TDCP
remain according to the existing graph/support recipe. Full-subtraction
support is independently checked in a private in-process factor copy before
the ablated transaction; retained epoch/satellite/signal identities must
match, exactly as in the earlier zero-values experiment. No serialized
factor/position input, no correction applied twice, no global default change.

New C++ test covers GPS/Galileo/GLONASS finite/missing/unavailable/NaN cases,
retained identities, unchanged sigma and GPS-versus-zero values. Fresh focused
test executable passed all 41 base-compensation tests. The ordinary native
CLI target build completed successfully with tmpfs compiler scratch; 14
CLI preflight tests passed, including the new accepted recipe and invalid
combinations. No full CTest/real-data byte-identity gate claim.

No raw performance experiment or score yet. The next experiment must freeze
the Phase326 dense full-correction recipe plus only this flag, with current
binary/source/raw pins, before running or opening truth. Compare primarily
with Phase329 zero-values support and Phase326 full values; all are H
development diagnostics, not heldout performance or leaderboard evidence.
An improvement must not be presumed from this implementation.
