# Phase297 opt-in RINEX3 header tracking filter

Starting HEAD `4bd54fe`. Root-only implementation; prior turn made progress.

Added default-off RINEXReader::setSourceHeaderTrackingFilter. For RINEX3,
the reader builds source default code selections from all header types and
filters fields before native selection/assembly. It never selects a lower
priority code based on that epoch's nonzero values. Unsupported versions
return false from readObservationEpoch with the flag enabled.

This is a field-admission feature, not full source RINEX parity. Existing
supported-band emission and exact-tracking metadata policies remain; RINEX2
translation, extended observations, source option overrides and a complete
obs2code validity table are not implemented by this switch. CLI and base
solver do not enable it yet. Native defaults are unchanged.

New actual-reader synthetic test declares C1C/C1P, leaves C1C empty, and
supplies C1P. It compares default reader admission against no fallback with
the new filter. No real data or truth is involved. Production reader code
changed only behind the new flag; this is not a behavior-preserving solver
refactor or a claim of real-data byte parity.

No real raw, solution, truth, MAT, station-table, network or Kaggle/token
access. No score or trajectory run. Synthetic test fixture I/O is permitted.

Validation: gnss_run_tests target built successfully (-j4). The filter
BasePseudorangeCompensationTest.*:*Rinex*:*RINEX* passed all 59 tests in
8 suites, including the new actual-reader missing-code case. Not full CTest
or real-data byte parity. The native FGO app binary was not rebuilt here.
