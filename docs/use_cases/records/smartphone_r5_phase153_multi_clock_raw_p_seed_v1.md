# Smartphone R5 Phase153 multi-clock raw-P seed support

Status: implemented as a default-off preparatory-lane extension.  This record
contains no real-route execution and does not enable the H no-Doppler graph.

## Source-backed contract

Native `SPPProcessor::solvePositionLS()` in `src/algorithms/spp.cpp` chooses
GPS as the reference clock when present; otherwise it chooses the
highest-count observed group with the existing `std::map` tie order.  With
`model_intersystem_bias`, it adds one column for every observed non-reference
group for which native SPP uses a separate clock bias.  The new
`CorrectedMeasurement::clock_group` in `include/libgnss++/algorithms/spp.hpp`
reports that same post-filter mapping, including the MRTKLIB QZSS exception.

`libgnss::raw_p_seed::solve()` now computes its rank from the native
post-filter corrected rows: four ECEF/shared-clock columns plus the observed
native ISB columns.  It reads the native `SPPProcessor::getSystemBiases()` map
and the returned receiver clock, and rejects missing/nonfinite expected
estimates.  Every known output group is serialized; groups removed by native
filtering are `observed=false`, `estimate_available=false`, and `bias_m=null`,
never a fabricated zero.  Unknown groups and known enum groups without a
native corrected-measurement mapping (currently NavIC) remain fail-closed.

The opt-in application path and raw-P/Doppler-clearing behavior are otherwise
unchanged.  The JSON boundary adds `corrected_clock_groups`,
`reference_clock_group`, and typed `clock_group_biases` entries to the existing
Phase149 structural schema.

## Synthetic verification

`RawPSeedTest.*` passes 12/12.  It covers the historical single-group solve,
moving/endpoint velocity and Doppler-clearing regression; exact synthetic
GPS+Galileo observations with a distinct Galileo clock bias and known motion;
post-filter removal of a raw group with explicit absent output; unknown and
unsupported clock groups; dynamic five-column rank deficiency; nonfinite
geometry; and timestamp/quantity guards.  No raw GNSS/IMU/nav/base input,
truth, MAT, Kaggle, imported coordinate/seed file, or real solver/FGO run was
performed.

The affected native targets `gnss_run_tests` and `gnss_fgo` build.  Existing
default recipes remain unchanged; this implementation only makes known
multi-clock epochs eligible for the raw-P preparatory API and does not claim
H-route coverage, no-Doppler graph observability, accuracy, or leaderboard
performance.
