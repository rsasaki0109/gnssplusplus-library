# Phase283: main-only CLI P re-masking

Added --native-pseudorange-remasking, default OFF. The selected main builder
retains pre-mask P rows; the GNSS-first builder retention flag is OFF.
After successful GNSS-first, exact Phase91 alignment and complete copied
position/clock counts are required. Pure selection runs once; nonempty
selected rows are copied from the immutable pool into a temporary vector
before swapping the main P vector. Failure returns without starting main.
There is no input path for saved positioning or MAT data.

This preserves corrected P, satellite geometry and sigma. It changes only
admission using updated range/clock residuals; it is not a full geometry,
atmosphere, elevation, or outer-iteration source port. GNSS-first selection
is unchanged. Summary reports requested/applied, pool, old/new, recovered,
removed/unchanged counts, and convention. Historical builder rejection
diagnostics still describe the original construction, not the new selection.

CLI admission requires Pixel5 Phase171 ECEF-D and UTC fallback, and rejects
base compensation/Phase126/Phase135. Builds 55004 and 99081 completed with
exit 0. Six new CLI admission tests plus eight metre-sigma and six relative
height admission tests passed (20 total). No full CTest or real-data claim.

Remaining before raw accuracy experiment: audit interactions with other
post-builder row mutations and provenance guards; ensure old/new counter
conservation and actual final inserted P count are checked. Tests so far
cover the pure selector and CLI rejection, not full production raw recovery.
No raw run, truth read, MAT payload access or submission occurred here.
