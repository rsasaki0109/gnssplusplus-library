#pragma once

/**
 * @file android_raw_gnss.hpp
 * @brief Strict conversion of raw Android GNSSLogger rows to observations.
 *
 * The conversion is intentionally an input boundary, not an estimator.  It
 * ports the observable/time and P/L/D mapping in taroz/gsdc2023's
 * ``functions/gnsslog2obs.m`` (pinned in the smartphone native-only audit)
 * into a reusable C++ API.  Published result MAT files and truth are not
 * accepted by this API.
 */

#include <cstddef>
#include <cstdint>
#include <string>

#include <libgnss++/core/observation.hpp>

namespace libgnss::io {

struct AndroidRawGnssConfig {
    /// Optional device name used only for the published ADR sign correction.
    std::string device_model;
    /// Retain GPS L1 C/A and Galileo E1 C observations when true.
    bool include_galileo_e1 = true;
    /// Require Android TimeNanos/FullBiasNanos/ReceivedSvTimeNanos timing.
    /// The enriched ArrivalTimeNanosSinceGpsEpoch column is diagnostic only.
    bool require_raw_android_clock = true;
    /// When an enriched RawPseudorangeMeters column is present, compare it to
    /// the raw-clock reconstruction and reject a large disagreement.
    bool verify_enriched_pseudorange = true;
    double enriched_pseudorange_tolerance_m = 0.05;
};

struct AndroidRawGnssDiagnostics {
    std::size_t input_rows = 0;
    std::size_t raw_rows = 0;
    std::size_t skipped_non_raw_rows = 0;
    std::size_t skipped_unsupported_signal_rows = 0;
    std::size_t skipped_invalid_timing_rows = 0;
    std::size_t selected_rows = 0;
    std::size_t selected_epochs = 0;
    std::size_t carrier_rows = 0;
    std::size_t doppler_rows = 0;
    std::size_t carrier_loss_rows = 0;
    std::size_t clock_discontinuities = 0;
    std::size_t enriched_pseudorange_checks = 0;
    std::size_t enriched_pseudorange_mismatches = 0;
    std::size_t receiver_position_rows = 0;
    std::size_t gps_rows = 0;
    std::size_t galileo_e1_rows = 0;
    int first_gps_week = 0;
    double first_gps_tow = 0.0;
    double last_gps_tow = 0.0;
    std::string timing_formula;
    std::string carrier_formula;
    std::string doppler_formula;
    std::string adr_sign_policy;
};

struct AndroidRawGnssResult {
    ObservationSeries observations;
    AndroidRawGnssDiagnostics diagnostics;
};

/**
 * @brief Load raw Android ``device_gnss.csv`` rows.
 *
 * Supported rows are GPS L1 C/A and, optionally, Galileo E1 C.  The raw
 * Android clock equation is reconstructed with integer nanosecond fields:
 * ``week=floor((TimeNanos-baseFullBias)/1e9/604800)`` and
 * ``tow_rx=(TimeNanos-baseFullBias-week*604800e9)/1e9-BiasNanos/1e9``;
 * pseudorange is ``(tow_rx-tow_tx)*c`` after a nearest-week unwrap.  Carrier
 * phase is ADR in metres divided by wavelength, with the exact five-device
 * sign correction published by taroz's ``gnsslog2obs.m``.  Android
 * pseudorange-rate is converted to RINEX Doppler as ``-rate/lambda``.
 *
 * Duplicate satellite/signal rows in one epoch, malformed finite values,
 * unsupported raw timing, and large disagreement with an enriched
 * pseudorange column fail closed.  Result/truth/submission coordinates and
 * imported outputs are never used.  A finite device-provided WLS ECEF column
 * is accepted only as the approximate receiver seed already supported by
 * ``ObservationData``; it is not a result-MAT or submission coordinate.
 */
bool loadAndroidRawGnssCsv(const std::string& path,
                           const AndroidRawGnssConfig& config,
                           AndroidRawGnssResult& result,
                           std::string& error);

}  // namespace libgnss::io
