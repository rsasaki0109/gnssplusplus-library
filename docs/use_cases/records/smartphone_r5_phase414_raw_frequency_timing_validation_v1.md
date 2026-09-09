# Phase414 — native raw frequency timing guard

Added default-OFF `AndroidRawGnssConfig::require_frequency_pair_timing`.
When enabled, all parsed GPS/Galileo rows require explicit finite zero
TimeOffsetNanos and identical TimeNanos, FullBiasNanos, BiasNanos and hardware
clock discontinuity count for each (raw UTC, constellation, SVID) key.
Checks precede observation quality masks. No coordinate or truth input is used.
This is the restricted Phase412 experiment contract, not a general assertion
that nonzero Android time offsets are invalid.

Verified current native app build (`cmake --build build --target
gnss_fgo_imu_no_base -j1`) and fresh standalone AndroidRawGnssTest suite:
19/19 passed. The new test covers GPS and Galileo independently with matched
clocks, 1 ns nonzero offset, 1 ns FullBias mismatch, and missing TimeOffset.
All cases load with the guard OFF; only matched explicit-zero cases load ON.
Standalone executable: `/tmp/gnss_phase414_raw_timing_tests`.

The public experimental CLI and raw-loader wiring remain pending. This is
input-contract validation, not an enabled raw experiment or accuracy result.
Full CTest and the PPC refactor gate were not run. Goal remains unmet.
