#pragma once

#include <stdexcept>

namespace libgnss {
enum class NativeImuObservationPhase { Legacy, Initialization, Final };

inline void validateNativeImuObservationPhase(NativeImuObservationPhase phase) {
    switch (phase) {
        case NativeImuObservationPhase::Legacy:
        case NativeImuObservationPhase::Initialization:
        case NativeImuObservationPhase::Final: return;
    }
    throw std::invalid_argument("Invalid native IMU observation phase");
}

inline double nativeImuCodeResidualThreshold(NativeImuObservationPhase phase, bool l1) {
    validateNativeImuObservationPhase(phase);
    return phase == NativeImuObservationPhase::Initialization
        ? (l1 ? 50.0 : 30.0) : (l1 ? 20.0 : 15.0);
}

inline double nativeImuDopplerResidualThreshold(NativeImuObservationPhase phase) {
    validateNativeImuObservationPhase(phase);
    return phase == NativeImuObservationPhase::Initialization ? 20.0 : 3.0;
}
}  // namespace libgnss
