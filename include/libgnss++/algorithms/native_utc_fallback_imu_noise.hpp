#pragma once

#include <libgnss++/algorithms/fgo.hpp>

namespace libgnss::native_utc_fallback_imu_noise {

// The cached source uses coefficient 1.0 when the Android elapsed-time field
// is absent.  The native alignment loader deliberately keeps its existing
// 0.5 coefficient; this policy changes only the CombinedImuFactor white-noise
// densities after the loader reports that UTC fallback was actually applied.
constexpr double kDefaultAccelNoiseSigma = 0.025;
constexpr double kDefaultGyroNoiseSigma = 0.0005;
constexpr double kSourceFallbackAccelNoiseSigma = 0.05;
constexpr double kSourceFallbackGyroNoiseSigma = 0.001;

struct Selection {
    bool requested = false;
    bool fallback_applied = false;
    bool applied = false;
    double accel_noise_sigma = kDefaultAccelNoiseSigma;
    double gyro_noise_sigma = kDefaultGyroNoiseSigma;
    double measurement_sync_coefficient = 0.5;
    const char* source = "native-pixel5-coefficient-0.5";
};

inline Selection select(bool requested, bool fallback_applied) {
    Selection selection;
    selection.requested = requested;
    selection.fallback_applied = fallback_applied;
    if (requested && fallback_applied) {
        selection.applied = true;
        selection.accel_noise_sigma = kSourceFallbackAccelNoiseSigma;
        selection.gyro_noise_sigma = kSourceFallbackGyroNoiseSigma;
        selection.measurement_sync_coefficient = 1.0;
        selection.source = "source-utc-wall-clock-fallback-coefficient-1.0";
    } else if (requested) {
        selection.source =
            "native-pixel5-coefficient-0.5-fallback-not-applied";
    }
    return selection;
}

inline Selection apply(FGOProcessor::ImuNoiseParams& noise,
                       bool requested,
                       bool fallback_applied) {
    const Selection selection = select(requested, fallback_applied);
    noise.accel_noise_sigma = selection.accel_noise_sigma;
    noise.gyro_noise_sigma = selection.gyro_noise_sigma;
    return selection;
}

}  // namespace libgnss::native_utc_fallback_imu_noise
