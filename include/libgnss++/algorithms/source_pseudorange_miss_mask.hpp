#pragma once

/**
 * @file source_pseudorange_miss_mask.hpp
 * @brief Source-exact finite-base-correction filtering for native FGO.
 *
 * The official taroz graph only inserts an undifferenced pseudorange factor
 * when its interpolated base correction is finite.  This small helper keeps
 * that operation explicit and testable without performing any I/O or
 * changing TDCP, Doppler, IMU, or epoch state vectors.
 */

#include <libgnss++/algorithms/fgo.hpp>

#include <functional>
#include <cmath>
#include <map>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace libgnss::source_pseudorange_miss_mask {

struct SignalCounts {
    std::size_t original_adopted_rows = 0U;
    std::size_t retained_finite_pc_rows = 0U;
    std::size_t corrected_rows = 0U;
    std::size_t matched_exact_stream_rows = 0U;
    std::size_t finite_correction_rows_among_matched = 0U;
    std::size_t dropped_missing_exact_stream_rows = 0U;
    std::size_t dropped_out_of_domain_rows = 0U;
    std::size_t dropped_nonfinite_correction_rows = 0U;
    bool factor_count_consistent = false;
};

struct Report {
    bool callback_contract_valid = false;
    bool factor_count_consistent = false;
    bool signal_count_consistent = false;
    bool correction_already_applied = false;
    bool canonical_key_mode = false;
    std::size_t original_adopted_rows = 0U;
    std::size_t retained_finite_pc_rows = 0U;
    std::size_t corrected_rows = 0U;
    std::size_t matched_exact_stream_rows = 0U;
    std::size_t finite_correction_rows_among_matched = 0U;
    std::size_t dropped_missing_exact_stream_rows = 0U;
    std::size_t dropped_out_of_domain_rows = 0U;
    std::size_t dropped_nonfinite_correction_rows = 0U;
    double retained_finite_pc_fraction = 0.0;
    double retained_over_original_fraction = 0.0;
    double correction_abs_p50_m = 0.0;
    double correction_abs_p95_m = 0.0;
    double correction_abs_max_m = 0.0;
    // SignalType includes the constellation/frequency class used by the
    // native exact stream key.  Keeping this map typed avoids inventing a
    // second signal alias solely for telemetry; the application serializes
    // the existing names and frequency bands.
    std::map<SignalType, SignalCounts> signal_counts;
    // A successful non-empty call performs exactly one in-place correction
    // pass.  A repeated call is rejected before mutation.
    std::size_t correction_application_passes = 0U;
    std::string failure;
};

using HasStream = std::function<bool(const SatelliteId&, SignalType)>;
using CorrectionAt = std::function<bool(const GNSSTime&,
                                         const SatelliteId&,
                                         SignalType,
                                         double&)>;

// Diagnostic ablation only: preserve actual correction availability/NaNs,
// but subtract zero on retained rows. Not a positioning correction model.
inline CorrectionAt finiteMaskOnlyAblation(const CorrectionAt& actual) {
    if (!actual) return {};
    return [actual](const GNSSTime& time, const SatelliteId& satellite,
                    SignalType signal, double& correction) {
        const bool available = actual(time, satellite, signal, correction);
        if (available && std::isfinite(correction)) correction = 0.0;
        return available;
    };
}

// Diagnostic only: retain full-model support, apply numerical values to GPS
// alone. Non-GPS unavailable/NaN corrections remain misses, not zero fills.
inline CorrectionAt gpsValuesOnlyAblation(const CorrectionAt& actual) {
    if (!actual) return {};
    return [actual](const GNSSTime& time, const SatelliteId& satellite,
                    SignalType signal, double& correction) {
        const bool available = actual(time, satellite, signal, correction);
        if (available && std::isfinite(correction) && satellite.system != GNSSSystem::GPS)
            correction = 0.0;
        return available;
    };
}

using StreamCenter = std::function<bool(const SatelliteId&, SignalType, double&)>;

// Experimental GPS temporal-component ablation. The caller supplies centers
// from this run's raw-built base model, never saved calibration/position data.
// Missing/invalid centers are configuration failures, not new observation
// misses: preserve actual support or fail the enclosing transaction.
inline CorrectionAt gpsCenteredValuesAblation(const CorrectionAt& actual,
                                              const StreamCenter& center_at) {
    if (!actual || !center_at) return {};
    return [actual,center_at](const GNSSTime& time,const SatelliteId& satellite,
                             SignalType signal,double& correction) {
        const bool available=actual(time,satellite,signal,correction);
        if (!available || !std::isfinite(correction)) return available;
        if (satellite.system!=GNSSSystem::GPS) {correction=0;return true;}
        double center=std::numeric_limits<double>::quiet_NaN();
        if (!center_at(satellite,signal,center) || !std::isfinite(center))
            throw std::invalid_argument("missing finite raw-built GPS correction center");
        const double centered=correction-center;
        if (!std::isfinite(centered))
            throw std::invalid_argument("nonfinite centered GPS correction");
        correction=centered;
        return true;
    };
}

using CanonicalHasStream = std::function<bool(const SatelliteId&,
                                              SignalType,
                                              bool,
                                              int)>;
using CanonicalCorrectionAt = std::function<bool(const GNSSTime&,
                                                  const SatelliteId&,
                                                  SignalType,
                                                  bool,
                                                  int,
                                                  double&)>;

/**
 * Apply the official finite-pc miss mask in-place.
 *
 * A factor is retained only if its exact stream exists, its epoch index and
 * timestamp are valid, the callback supplies an in-domain correction, and
 * subtraction remains finite.  The retained vector preserves factor order
 * and each factor's original epoch_index; callers therefore do not need to
 * renumber epochs or touch any non-pseudorange factor collection.
 *
 * The callbacks are supplied by the already-built base model.  This function
 * performs no file or navigation access, and an invalid callback contract
 * leaves the input vector unchanged and returns false.
 */
bool apply(std::vector<FGOProcessor::PseudorangeFactor>& factors,
           const std::vector<FGOProcessor::EpochSeed>& epochs,
           const HasStream& has_stream,
           const CorrectionAt& correction_at,
           Report& report);

/**
 * Apply the same finite-pc transaction through the Phase131 canonical
 * physical-frequency correction key.  The original typed SignalType and
 * factor vector order remain untouched; certified GLONASS FCN provenance is
 * passed explicitly to both callbacks.
 */
bool applyCanonical(std::vector<FGOProcessor::PseudorangeFactor>& factors,
                    const std::vector<FGOProcessor::EpochSeed>& epochs,
                    const CanonicalHasStream& has_stream,
                    const CanonicalCorrectionAt& correction_at,
                    Report& report);

}  // namespace libgnss::source_pseudorange_miss_mask
