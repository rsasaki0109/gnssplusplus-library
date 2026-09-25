#pragma once

#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace libgnss {

// Pinned fgo_gnss_imu.m: zero-velocity priors are final-pass-only,
// but stationary pose factors occur in both passes when dtgps < 1.5 s.
// Legacy preserves the existing native graph, including its gap policy.
enum class NativeImuStopPhase { Legacy, Initialization, Final };

inline void validateNativeImuStopPhase(NativeImuStopPhase phase) {
    switch (phase) {
        case NativeImuStopPhase::Legacy:
        case NativeImuStopPhase::Initialization:
        case NativeImuStopPhase::Final: return;
    }
    throw std::invalid_argument("Invalid native IMU stop phase");
}

inline bool nativeImuStopVelocityPrior(NativeImuStopPhase phase) {
    validateNativeImuStopPhase(phase);
    return phase != NativeImuStopPhase::Initialization;
}

inline bool nativeImuStopPoseInterval(NativeImuStopPhase phase, double dt_s) {
    validateNativeImuStopPhase(phase);
    return phase == NativeImuStopPhase::Legacy ||
           (std::isfinite(dt_s) && dt_s > 0.0 && dt_s < 1.5);
}

inline bool nativeImuStopPoseUtcInterval(NativeImuStopPhase phase,
                                       std::int64_t before_ms, std::int64_t after_ms) {
    validateNativeImuStopPhase(phase);
    if (phase == NativeImuStopPhase::Legacy) return true;
    // Source dtgps is named after GPS but computed from obs.utcms.
    // Positive, increasing integers avoid both missing-time fallback and overflow.
    return before_ms > 0 && after_ms > before_ms && after_ms - before_ms < 1500;
}

}  // namespace libgnss
