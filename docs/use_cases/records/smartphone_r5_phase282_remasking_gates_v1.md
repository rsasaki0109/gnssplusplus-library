# Phase282: reselector median and admission gates

Added synthetic L1/L5 group separation with even/odd medians, exact L1
20 m boundary acceptance and L5 16 m rejection. Added tests rejecting
non-metre clock state, non-normalized time of week, out-of-range epoch
index, zero sigma, an old row absent from the pool, and duplicate epoch
times. Selector now rejects non-normalized TOW and nonfinite medians;
even median uses half-sums to avoid intermediate overflow.

Build 60625 failed because the test used nonexistent GPS_L5I. Corrected
to the repository's GPS_L5 without changing expected results. Build 21065
completed with exit 0. All five selected tests passed (two reselector,
two pool, one admission-predicate). No full CTest or real-data claim.

Still not connected to CLI; no raw or truth reads, MAT use, saved positioning
input, or production recipe changes. Integration must preserve exact raw
epoch ownership, reject incompatible correction modes, and publish actual
old/new/recovered/removed counts. Test coverage here does not authenticate
arbitrary public FGOProblem contents or establish full source parity.
