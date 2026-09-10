# Phase409 — native pair epoch-time validation

Added `pairAdmittedFactorsAtEpochs` and connected the frequency-state backend
to it. After exact endpoint/satellite/band pairing, selected pair duration
must agree with actual native epoch-time subtraction within 1 ns, be finite,
positive and <=1.5 s. Clock metadata and epoch domain sizes must match.
The tolerance is arithmetic consistency, not a fitted synchronization offset.

Fresh standalone pairing suite passes 6/6, including real-time mismatch,
reversed/nonfinite time and domain mismatch controls. App/library build passed.
Relinked the unchanged Phase407 backend test object against rebuilt libraries;
native Phase171 main suite passes 2/2 including nonzero residual-state/RMS test.
Full CTest not run. All sessions completed.

Read-only raw GPS/Galileo timing audit on Phase37 H (77748 rows) and LAX-T
(42370 rows): zero missing or nonzero TimeOffsetNanos; zero mismatches of
(TimeNanos, FullBiasNanos, BiasNanos, HardwareClockDiscontinuityCount) within
each (UTC,constellation,SVID). This includes all raw GPS/Galileo rows rather
than only ADR-valid dual pairs. No coordinate/enriched measurement fields
were interpreted and no truth, MAT or saved-state input used.

These raw facts apply only to those inspected pinned inputs. General raw
pair-time provenance still needs an entry guard or preserved metadata; this
aggregate audit alone must not authorize arbitrary future datasets. No raw
solver or accuracy score this phase. CLI remains unexposed. Next freeze the
opt-in experiment's prior and input contract and verify disabled-mode replay;
no positioning improvement has yet been demonstrated.
