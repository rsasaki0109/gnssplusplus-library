# Phase280: ephemeral pre-mask P pool

Added default-off retain_native_pseudorange_remasking_pool. The builder
requires upstream observable quality and P factors, and copies the P vector
immediately before grouped residual filtering into FGOProblem's ephemeral
native_pseudorange_remasking_pool. There is no file interface, serialization,
re-admission or optimizer consumption yet. Default admission is unchanged
by the added branch; real-data byte parity has not been measured.

Build 62401 completed with exit 0. Twelve selected tests passed: two pool
tests, one remasking predicate test, and nine upstream preprocessing tests.
The actual synthetic navigation/observation builder produces a nonempty
pool, whose size equals the candidate count and accepted-plus-rejected
count. Accepted row identity, measurement, sigma, residual and satellite
position agree with retention OFF. Invalid configuration is rejected.
This fixture does not assert a positive rejection count; it does not yet
prove restoration of a discarded row through the full builder-to-solver
path. No full CTest or real-data accuracy claim.

Next implement a transactional grouped residual reselector over this pool,
with exact identities, finite states and old/new admission accounting.
Keep P measurement/sigma fixed; do not replace GNSS-first observations.
Only a validated same-run handoff may invoke the future CLI path. The pool
alone cannot authenticate arbitrary public-API caller provenance.
