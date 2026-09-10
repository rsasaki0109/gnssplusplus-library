# Phase338: raw H base low-elevation support

Added `scripts/native_base_elevation_audit.cpp`, a read-only native diagnostic.
It loads raw H base observations and navigation, applies source header tracking
selection and source L1/L5 storage-slot selection, constructs source shared
satellite states, and evaluates geometry at the raw RINEX header reference.
It counts all health values as a conservative support superset; it does not
apply corrections or feed positions into any estimator. Header coordinates
are neither exported nor adjusted. No phone data, MAT, candidates or truth.

Before running, verified SHA256:

- source: `8a04c795082b5202c6512e9b352f62ada80cf84f02f3f2d7102f589d1430b084`
- raw base: `4d3e37cbe0347fa56216db54ede9e0f30731885f337f1653ab5a86afb2bb2150`
- raw nav: `147d948f0eba3bf09e295e7f67fbe8db60c25e236bc3c7d958dc933483f10909`

Built with C++17/O1 against existing native libraries, using tmpfs compiler
scratch and executable `/dev/shm/gnss-test-build-recovery.BJvQt9/base_elevation_audit`.
One raw diagnostic pass exited 0:

- epochs: 3500; source-frequency rows: 112050
- below 5 degrees: 3766; nonpositive elevation: 0
- minimum elevation: 0.054473843872041579 degrees
- GPS rows: 54714; GPS below 5 degrees: 1096

Thus the low-elevation atmosphere discrepancy found synthetically in Phase337
has actual base-observation exposure, including GPS. The nonpositive-elevation
ionosphere guard discrepancy has no exposure in this support set. Do not
equate the below-5 count with the exact number affected by the troposphere
clamp (whose threshold is lower), nor with retained rover rows.

Next: quantify unclamped source-like versus native troposphere differences
through the same dense 151-epoch smoothing and actual correction query
support before deciding on an opt-in source-parity implementation. A change
to global atmosphere helpers would alter unrelated solvers and is not
authorized by this evidence. No new accuracy result or promotion.
