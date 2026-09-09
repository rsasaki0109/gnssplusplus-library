# Smartphone R5 Phase155 preprocessing rejection diagnostics

Status: implemented and synthetically verified.  This is a reporting-only
extension to the default-off Phase149 raw-P preparatory lane.  It does not
rerun or reinterpret the sealed Phase154 H result.

## Source-backed behavior

`SPPProcessor::validateObservations` in `src/algorithms/spp.cpp:1935-2149`
now receives an optional diagnostics sink.  It preserves the existing branch
order and numerical decisions while assigning each source-order raw-P row a
single terminal reason.  The reasons cover unsupported system/signal,
invalid pseudorange, BeiDou GEO exclusion, missing/invalid-at-query-time
ephemeris, unhealthy satellite, SNR mask, and rows not selected by the native
ionosphere-free construction.  `SPPProcessor::preprocessEpoch` at
`src/algorithms/spp.cpp:2500-2770` adds terminal reasons for unavailable
satellite state, elevation mask, invalid correction variance, and unsupported
output system; successfully emitted corrected rows are marked `accepted`.

The navigation API exposes one null result for both absent records and no
valid record at the requested transmit time, so those cases intentionally
share `missing-ephemeris`; no unsupported inference is added.  Rows that pass
validation but are not selected into an IFLC observation use the explicit
`not-used-by-spp-preprocess` reason.  IFLC primary/secondary rows retain their
source indices, with `ionosphere_free_rows_expanded` identifying the one-to-
many preprocessing relationship.

`raw_p_seed::solve` copies the diagnostics immediately after the same native
`preprocessEpoch` call, before corrected-row geometry/rank failure handling.
Consequently a post-SPP-preprocess failure such as the sealed Phase154
first-epoch geometry failure can expose row accounting.  Guards before native
preprocessing (for example too few raw satellites or unsupported raw clock
groups) keep diagnostics unavailable rather than fabricating counts.  The
native JSON serializer emits availability, input/accepted/rejected totals,
reason totals, and source row identities; it emits no measurement or
coordinate values.

The implementation does not alter SPP filters, correction values, state
initialization, solver limits, outlier handling, or default recipes.  In
particular, Phase154's 31 raw-P / 22-satellite / 2-corrected first-epoch
observation remains historical evidence only; Phase155 makes no claim about
which source gate caused those counts until a separately authorized run.
The cold-start position used by the existing SPP correction pass may affect
elevation/state outcomes, but this record does not infer that it did.

## Synthetic verification

`RawPSeedTest.*` passes 15/15.  The added tests cover conservation and unique
source-row accounting for mixed valid/invalid P rows, all rows rejected by
missing navigation, and all rows rejected by the unchanged elevation mask.
The prior stationary/moving, Doppler-clearing, multi-clock, rank, geometry,
and time-policy tests remain passing.  Targets `gnss_run_tests` and
`gnss_fgo_imu_no_base` build successfully.

No real GNSS/IMU/navigation input, solver/FGO run, truth, MAT, base, Kaggle,
coordinate payload, or Phase154 artifact rewrite was performed.

## Next boundary

Run the already-pinned H raw-P prep diagnostic only in a separate authorization
and result record if row-level causes are required.  Do not relax any native
gate based solely on the resulting counts.
