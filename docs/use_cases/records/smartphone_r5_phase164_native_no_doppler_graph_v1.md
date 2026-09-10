# Phase164 native raw-P no-Doppler graph

Status: synthetic-library validation complete; real-route execution is not authorized by this record.

## Scope

Phase164 adds one default-off GTSAM recipe for a complete same-run
`raw_p_seed::RawPNoDopplerSeed` handoff.  It is a Point3 position, Vector3
velocity, epoch-local C7, and D graph.  It is separate from the generic
Doppler admission path and does not remove the Phase135/143 guards.

Admission is fail-closed unless the problem has exact epoch/source identity,
finite accepted raw-P position/velocity/C0/D values, GPS-L1 pseudorange rows
with at least four rows per epoch, GPS as the certified C7 reference, no
generic Doppler/carrier/DD rows, and source C0D-eligible positive intervals.
Seeds are in-memory only; no serialized seed or coordinate file is read.

## Source-backed topology

The source C7 order remains `[C0, GLO-L1, GAL-L1, BDS-L1, GPS-L5,
GAL-L5, BDS-L5]`.  The adapter certifies only native GPS reference C0.
Unavailable C1--C6 values begin at explicit numerical zero initial guesses,
not measured ISBs.  Components without a retained pseudorange row receive a
configured `native_raw_p_no_doppler_unobserved_clock_gauge_sigma_m` (the
synthetic test uses 1e6 m) diagonal gauge prior so they are not silently
unconstrained; this prior is a numerical gauge and provides no
observational-rank claim.  C0 remains constrained by source pseudorange rows
and CCDD.  The selected sigma and number of gauged components are exported in
the result diagnostics.

The graph reuses the source C7 pseudorange factor and CCDD factor.  Its
dedicated motion factor is the source XXVV equation

`(x[k+1]-x[k]) - dt*(v[k]+v[k+1])/2`.

Point3 source-clock TDCP is supported when TDCP rows are supplied.  Empty
Doppler is permitted only under this dedicated selector; D is initialized
from the exact same-run raw epoch value and is otherwise estimated/validated
through the existing CCDD state path.  Existing legacy and ordinary-Doppler
recipes are unchanged.

The dedicated XXVV state is ECEF to match the backend's Point3 ECEF position
keys; legacy Doppler/gnss-first velocity states remain ENU and keep their
existing export conversion.  This frame split is selector-local and is
covered by the synthetic exact-velocity assertion.

The existing velocity-state backend also retains its documented 1e6 m/s broad
zero numerical priors on V and D.  These are gauge/numerical safeguards, not
Doppler measurements.  XXVV constrains velocity sums; by itself it does not
prove a unique velocity trajectory, so finite synthetic convergence is not an
observability or accuracy claim.

## Validation

During development, the first exact-velocity assertion failed at
1.5451245 m/s for both synthetic epochs.  The cause was a frame mismatch:
the new factor coupled ECEF Point3 positions to the existing legacy ENU
velocity initialization/export.  The dedicated selector now keeps raw-P V in
ECEF, while ordinary Doppler paths retain ENU; the factor key order was also
aligned to the source X1/X2/V1/V2 contract.  This correction was validated by
the post-fix exact assertions below.

Build:

```text
cmake --build build --target gnss_lib_solvers -j2       # passed
cmake --build build --target gnss_run_tests -j2         # passed
cmake --build build --target gnss_fgo_imu_no_base -j2   # passed
```

Focused execution with `LD_LIBRARY_PATH=/home/sasaki/.local/lib`:

```text
FGOGtsamPhase164RawNoDopplerGraphTest.*                 # 3/3 passed
FGOGtsamPhase101SourceClockVectorTest.*                 # 6/6 passed
FGOGtsamSourceClockC0DPhase93Test.*                     # 2/2 passed
FGOGtsamPhase135AffineFamilyTest.SelectorRejectsEigenAndNeverFallsBack  # passed
RawPSeedTest.* and RawPSeedAdapterTest.*                 # 29/29 passed
```

The combined Phase164 plus Phase93/101/default-selector slice was 12/12
passed after the frame correction.

The synthetic moving GPS-L1, two-epoch graph converged with an empty Doppler
family and exported finite position-derived velocity, C7, and D vectors.
Synthetic insufficient-row, source-time-gap, and rejected-seed cases failed
closed.  The XXVV residual and Jacobians were checked directly.  No raw,
truth, MAT, Kaggle, or real-route solver input was accessed.

## Limits and next boundary

This proves only library wiring and synthetic graph behavior.  It does not
prove H/U route observability, accuracy, or convergence on raw data.  The
adapter still cannot certify non-GPS native SPP clock groups as the official
C7 frequency slots.  The Phase163 prep application is not changed to launch
this graph; a future raw execution must explicitly construct the in-memory
problem and opt in after a separate authorization record.
