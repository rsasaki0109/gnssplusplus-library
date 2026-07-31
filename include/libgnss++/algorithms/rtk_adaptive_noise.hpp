#pragma once

#include <libgnss++/algorithms/rtk_measurement.hpp>

#include <cstddef>
#include <unordered_map>

namespace libgnss {
namespace rtk_adaptive_noise {

// Innovation-based adaptive measurement variance (Motooka 2026, NAVIGATION
// navi.776): R_{k+1} = alpha*R_k + (1-alpha)*(v*v' - H*P-*H'), diagonal only.
// Applied at the single-difference level: for DD row i the innovation
// variance decomposes as E[v_i^2] = (H P- H')_ii + ref_var + sat_var_i, so the
// per-satellite sample is v_i^2 - hph_ii - ref_var. Adapting the SD
// satellite_variance inputs of buildDoubleDifferenceCovariance (instead of
// the assembled DD diagonal) keeps every DD block positive definite by
// construction and keys the memory per satellite, so it survives reference
// switches.
struct AdaptiveNoiseConfig {
    double alpha_phase = 0.9;
    double alpha_code = 0.5;
    double alpha_doppler = 0.5;
    // Clamp window relative to the current epoch's model variance
    // (varerr * nlos factor), so elevation/SNR trends stay the backbone and
    // stale absolute variances cannot survive geometry changes.
    double min_variance_scale = 0.25;
    double max_variance_scale = 25.0;
    // Entries not updated for this long are dropped (signal outage).
    double reset_gap_s = 5.0;
};

class AdaptiveNoiseTracker {
public:
    // Returns the adapted variance for this key/kind clamped to
    // [min_scale, max_scale] * model_variance, or model_variance when the key
    // has no history yet.
    double adaptedVariance(int key,
                           rtk_measurement::MeasurementKind kind,
                           double model_variance,
                           const AdaptiveNoiseConfig& config) const;

    // Feeds one innovation sample. innovation is the prefit residual v_i of
    // the DD row, hph_diagonal the matching diagonal entry of H*P-*H'
    // (prior covariance), reference_variance the model variance assigned to
    // the reference satellite side of the DD row.
    void update(int key,
                rtk_measurement::MeasurementKind kind,
                double innovation,
                double hph_diagonal,
                double reference_variance,
                double model_variance,
                double tow,
                const AdaptiveNoiseConfig& config);

    // Drops the entry for one key/kind (cycle slip on that satellite).
    void resetKey(int key, rtk_measurement::MeasurementKind kind);

    // Drops entries whose last update is further than reset_gap_s from tow.
    // A GPS week rollover makes every entry look stale, which degrades to a
    // conservative full reset.
    void pruneStale(double tow, double reset_gap_s);

    void clear();

    std::size_t size() const { return entries_.size(); }

    // Mean of adapted/model variance ratios currently tracked for a kind
    // (telemetry). Returns 1.0 when no entries exist.
    double meanVarianceScale(rtk_measurement::MeasurementKind kind) const;

private:
    struct Entry {
        double variance = 0.0;
        double model_variance = 0.0;
        double last_tow = 0.0;
    };

    static long long combinedKey(int key, rtk_measurement::MeasurementKind kind);
    static double alphaForKind(rtk_measurement::MeasurementKind kind,
                               const AdaptiveNoiseConfig& config);

    std::unordered_map<long long, Entry> entries_;
};

}  // namespace rtk_adaptive_noise
}  // namespace libgnss
