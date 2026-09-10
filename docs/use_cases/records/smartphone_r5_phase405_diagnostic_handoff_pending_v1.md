# Phase405 — same-run diagnostic correction handoff, verification pending

Added FGOResult in-memory correction records for every admitted TDCP factor
when enabled: endpoint indices, satellite, signal, and alpha*optimized residual
slant change [m]. Unpaired corrections are zero; disabled export is empty.
Values are built from the same optimizer instance and never serialized as
inference inputs. Nonfinite exports fail.

App TDCP diagnostics validate length and each identity/value, then subtract
the correction from prediction-minus-measurement residual before RMS/Huber
aggregation. Backend RMS uses the opposite residual sign with the matching
addition. Summary exposes only state/factor counts, not this correction series.

Added native paired-graph test assertions for empty OFF export, ON identities,
finite corrections and agreement between exported-state residual reconstruction
and backend RMS (1e-7 m tolerance). Not yet executed at this checkpoint.

Live handles: app/library build session 14860; test object compilation 95010,
target `/tmp/gnss_phase405_backend_tests.o`. Poll rather than restart. After
library build finishes, link the new object and run focused Phase171 main
tests. App edits were made before its compilation stage was reached.

CLI remains unexposed. Prior scale, raw timing provenance and default-mode
replay are still required. No raw run, truth read, MAT, saved-position input
or accuracy claim. Goal remains active.
