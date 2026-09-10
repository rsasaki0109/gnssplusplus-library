# Phase340: raw phone overlap with smoothed base troposphere differences

Extended `native_base_elevation_audit.cpp` with an optional fourth argument,
the raw Android GNSS CSV. The native loader disables enriched-pseudorange
verification, as in prior raw diagnostics. Valid rows with pseudorange are
queried by exact satellite/native signal against the Phase339 smoothed
troposphere-difference streams. Base signal-to-slot aliases are checked for
conflicts. Queries linearly interpolate adjacent finite grid entries, return
exact endpoint values at exact times, and never extrapolate. Missing streams
and nonfinite/out-of-domain values are counted separately. No trajectory,
truth, MAT or saved correction series is read or written.

One pass after successful C++17/O1 compilation exited 0; Phase339 base
statistics repeated exactly. Raw-phone results:

- valid pseudorange candidates: 108722
- finite difference queries: 86947
- changed queries (>1e-6 m): 676
- missing stream: 20172
- unavailable/nonfinite/out-of-domain: 1603
- maximum absolute queried difference: 78.756597197666323 m

This demonstrates overlap in the raw phone admission set, not in the final
FGO factors. The previous absence of changes at observed high-elevation base
points therefore cannot alone dismiss this issue. Next check must use
same-process raw-built FGO admission and its quality/elevation filters before
claiming actual solver exposure. This diagnostic's all-health base support
and interpolation are not a complete production Model equivalence test.
Production recipe and accuracy unchanged; no correction was applied.

Raw inputs are the same H base/nav as Phase338 and the raw GNSS path under
Phase37's `2021-08-24-20-32-us-ca-mtv-h/pixel5/inputs`. This is an exploratory
read-only diagnostic, not a newly hash-frozen accuracy evaluation.
Temporary executable: `/dev/shm/gnss-test-build-recovery.BJvQt9/phone_atmosphere_overlap_audit`.
