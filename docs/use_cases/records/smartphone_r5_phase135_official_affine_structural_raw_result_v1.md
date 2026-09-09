# Phase135 structural raw result: fail-closed

The independent authorization was sealed in
`15ea056c9604b4e53a58d85e23d435e97936d8fa`.  After that commit, the pinned
freeze, manifest, pre-raw record, runner, binary, and canonical Doppler
correction object were revalidated.  The authorized wrapper then hashed the
four MTV-A raw inputs once each; all sealed sizes and digests matched.

The run did not reach the native process.  Its raw-only command preflight
incorrectly treated the required recipe selector
`--native-pdc-imu-tdcp-no-bridge` as a forbidden input because the generic
substring check matched `pdc`.  This is a wrapper bug, not an algorithm or
data result.  The wrapper failed closed before launch, did not generate or
open a solution/summary, and aborted the fixed route sequence before reading
LAX-T.  No fix or rerun is permitted by this authorization.

Accounting is therefore MTV-A raw GNSS/IMU/nav/base = 1 each, LAX-T = 0 each,
native solver invocations = 0, truth/accuracy/MAT/PDC/precomputed/Kaggle = 0,
and rerun/fallback/repair/sweep = 0.  No structural GO is claimed; all
factor, bridge, C7/D, geometry, raw-base, solver-progress, cost, coverage, and
Pixel5 gates are recorded as not reached.

The result is intentionally a NO-GO sealed artifact.  A future attempt would
need a new wrapper correction/qualification commit and an independent new raw
authorization; this result cannot be repaired or rerun in place.
