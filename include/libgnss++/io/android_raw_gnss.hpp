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
    /// Retain the L1 observations used by taroz's FGO mapping when true.
    bool include_galileo_e1 = true;
    /// Retain the L5-family observations used by taroz's L1/L5 FGO path.
    /// This includes GPS L5, Galileo E5a, and BeiDou B2a.
    bool include_l5 = true;
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
    std::size_t skipped_invalid_quality_rows = 0;
    /// Rows masked by the upstream exobs code/doppler/carrier status rules.
    std::size_t masked_code_rows = 0;
    std::size_t masked_doppler_rows = 0;
    std::size_t masked_carrier_rows = 0;
    std::size_t deduplicated_rows = 0;
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
    std::size_t gps_l1_rows = 0;
    std::size_t gps_l5_rows = 0;
    std::size_t glonass_l1_rows = 0;
    std::size_t galileo_e1_rows = 0;
    std::size_t galileo_e5_rows = 0;
    std::size_t beidou_l1_rows = 0;
    std::size_t beidou_l5_rows = 0;
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
 * Supported rows are GPS L1 C/A, GPS L5, GLONASS G1, Galileo E1/E5a, and
 * BeiDou B1/B2a when their corresponding frequency is present.  The raw
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
 * is retained in ``ObservationData`` for adapter compatibility and
 * diagnostics, but the dedicated raw PDC executable explicitly ignores it
 * and obtains its seed from native SPP.
 */
bool loadAndroidRawGnssCsv(const std::string& path,
                           const AndroidRawGnssConfig& config,
                           AndroidRawGnssResult& result,
                           std::string& error);

}  // namespace libgnss::io
