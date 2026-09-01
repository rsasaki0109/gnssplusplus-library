#pragma once

/**
 * @file base_pseudorange_compensation.hpp
 * @brief Truth-free native port of the published base pseudorange correction.
 *
 * The implementation owns only an in-memory copy of one already selected base
 * observation stream.  It does not know about files, truth, WLS solutions, or
 * an optimizer.  Callers can therefore apply the resulting correction only
 * to the adopted undifferenced pseudorange factors, leaving the SPP seed and
 * all carrier/Doppler factors untouched.
 */

#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>

#include <cstddef>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace libgnss::base_pseudorange_compensation {

using ObservationKey = std::pair<SatelliteId, SignalType>;

struct Config {
    Vector3d base_position_ecef = Vector3d::Zero();
    double expected_interval_s = 0.0;
    std::size_t moving_mean_samples = 0U;
    // Keep the base residual model identical to the ordinary native FGO
    // pseudorange model.  These are explicit instead of implicit so the
    // caller can pin the candidate's correction contract in telemetry.
    bool use_ionosphere_model = true;
    bool use_troposphere_model = true;
    bool use_signal_specific_galileo_group_delay = false;
};

struct Diagnostics {
    bool enabled = false;
    bool built = false;
    std::string failure;
    double base_interval_s = std::numeric_limits<double>::quiet_NaN();
    std::size_t base_epochs = 0U;
    std::size_t base_observation_rows = 0U;
    std::size_t matching_streams = 0U;
    std::size_t matched_base_rows = 0U;
    std::size_t finite_base_residual_rows = 0U;
    std::size_t smoothed_rows = 0U;
    std::size_t interpolated_rows = 0U;
    std::size_t interpolation_misses = 0U;
    std::size_t adopted_pseudorange_rows = 0U;
    std::size_t adopted_rows_corrected = 0U;
    double correction_abs_p50_m = std::numeric_limits<double>::quiet_NaN();
    double correction_abs_p95_m = std::numeric_limits<double>::quiet_NaN();
    double correction_abs_max_m = std::numeric_limits<double>::quiet_NaN();
    std::size_t moving_mean_samples = 0U;
};

/**
 * @brief In-memory same-satellite/same-signal base correction stream.
 */
class Model {
public:
    /**
     * Build finite base residual streams from raw RINEX observations and
     * broadcast navigation.  `base_epochs` must have been read by the caller
     * from the exact Phase64-pinned member; this method performs no I/O.
     */
    bool build(const ObservationSeries& base_epochs,
               const NavigationData& nav,
               const Config& config);

    /**
     * Interpolate one correction in-domain.  Returns false for a missing
     * stream, an out-of-domain time, or a non-finite result; no extrapolation
     * or endpoint hold is performed.
     */
    bool correctionAt(const GNSSTime& time,
                      const SatelliteId& satellite,
                      SignalType signal,
                      double& correction_m) const;

    const Diagnostics& diagnostics() const { return diagnostics_; }

private:
    struct Sample {
        GNSSTime time;
        double residual_m = 0.0;
    };

    std::map<ObservationKey, std::vector<Sample>> streams_;
    Diagnostics diagnostics_;
};

/**
 * MATLAB smoothdata(...,"movmean",N) uses a centered window and shrinks the
 * window at either endpoint.  This helper is public so the exact edge policy
 * can be covered independently of a navigation file in focused tests.
 */
std::vector<double> centeredMovingMean(const std::vector<double>& values,
                                       std::size_t window_samples);

/** Exact source sign: positive base pc is subtracted from rover P. */
inline double subtractCorrection(double pseudorange_m, double correction_m) {
    return pseudorange_m - correction_m;
}

}  // namespace libgnss::base_pseudorange_compensation
