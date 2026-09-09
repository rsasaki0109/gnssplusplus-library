# Smartphone R5 Phase161 native rank/provenance diagnosis

Status: bounded source-and-sealed-metadata diagnosis.  No Phase160 raw,
navigation, solver, truth, MAT, Kaggle, or coordinate payload was read or
rerun for this record.

## Sealed observation

The Phase160 result (`docs/use_cases/records/smartphone_r5_phase160_h_raw_p_collect_all_result_v1.json`)
reported 3,140 native SPP statuses of `spp`, while the raw-P diagnostic
reported 1,199 accepted epochs, 1,931 rank-deficient epochs (mostly reported
rank 4/5 against required rank 7), and 10 missing native clock-bias estimates.
Its first accepted-row view had 22 corrected rows; the native summary's final
quality-controlled view had 20 satellites used.  The sealed result did not
carry native row identities, final weights, or final clock-column diagnostics,
so those counts cannot prove a single cause for all 1,931 epochs.

## Source comparison

Native `SPPProcessor::solvePositionLS()` constructs a zero-initialized design
matrix at `src/algorithms/spp.cpp:1347-1350`, maps the reference clock and
separate inter-system-bias columns at `:1321-1345`, and solves a
sqrt(weight)-scaled matrix at `:1421-1439`.  Its final measurement vector is
rebuilt after the position iterations at `:1496-1522`; residual QC may return a
strict subset through outlier/FDE branches at `:1649-1754`.  The final mapping
is GPS/QZSS to the GPS reference group and separate GLONASS, Galileo, and
BeiDou groups through `clockBiasGroup()` at `:125-142` (QZSS is separate only
in the literal MRTKLIB IFLC path).

Before this patch, raw-P rank preflight constructed its own unweighted design
from all corrected preprocessing rows and did not initialize inactive clock
columns.  The latter is a proven defect: `Eigen::MatrixXd` coefficients are
not initialized, so sparse clock-group rows could contain stale values and
manufacture rank.  The helper now zeroes every accepted row before assigning
its active clock column (`src/algorithms/raw_p_seed.cpp:352-365`) and accepts
the same base row weights used by native SPP (`:263-390`).

Native final rows now carry `SolutionMeasurementIdentity` (satellite, signal,
primary/secondary source row indexes, IFLC marker, native clock group, and
base weight) from `src/algorithms/spp.cpp:1835-1850`; preprocessing carries the
same identity at `:2759-2785`.  Raw-P selection in
`src/algorithms/raw_p_seed.cpp:408-473` requires an exact identity match,
including repeated-satellite source indexes, then uses the native clock group
and base weight.  If a valid native solution has no complete ledger, the
diagnostic rejects it closed rather than reintroducing all corrected rows.

## Limits and interpretation

The source proves that the old preflight was not the native final weighted,
QC-filtered design.  It does not prove that this semantic mismatch explains
all Phase160 rank-deficient epochs.  At fixed columns, adding rows cannot
lower exact mathematical rank; a changed observed clock-group set changes the
columns, and weighting changes numerical rank/conditioning.  Native status
`spp` only establishes the native status path, not that the old reconstructed
matrix had full rank or that every returned coordinate was finite.

The 10 missing clock estimates are consistent with a native QC subset losing a
non-reference group, but Phase160 did not serialize native identities, so
that explanation remains unproven.  A coherent constellation-wide clock
offset is a legitimate ISB, not a valid outlier fixture; the synthetic QC
case therefore uses one per-observation outlier and proves exact subset
selection while retaining the legitimate group.  Whole-group removal remains
an unproven case and is not claimed here.

## Synthetic and build evidence

`tests/test_raw_p_seed.cpp` covers:

- genuinely collinear geometry still rejected;
- sparse clock columns zero-initialized and a near-collinear weighted design
  following native sqrt(weight) scaling;
- exact native QC row identity (including a reduced row set);
- a per-observation multi-constellation QC reduction with the surviving
  inter-system bias group retained;
- existing single/multi-clock acceptance, absent-group reporting, and
  invalid-geometry rejection regressions.

Results: `RawPSeedTest.*` 26/26 passed.  The affected targets
`gnss_run_tests` and `gnss_fgo_imu_no_base` both built successfully.  No real
route was run.  The next permitted diagnostic may use the newly built native
summary fields, but should still report any remaining rank cause as
unresolved unless native final identities and weights directly support it.
