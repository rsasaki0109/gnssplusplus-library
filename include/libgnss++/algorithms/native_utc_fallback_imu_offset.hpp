#pragma once

#include <cstdint>

namespace libgnss::native_utc_fallback_imu_offset {

// Source parameters.m uses -20 ms for both accelerometer and gyro UTC
// offsets when GNSS elapsed-time anchors are absent.  The loader applies this
// only to the mapped UTC value in the explicit wall-clock fallback branch;
// IMU elapsedRealtimeNanos may still be present in a fallback input.
constexpr std::int64_t kSourceUtcWallClockOffsetMs = -20;

struct Selection {
    bool requested = false;
    bool fallback_applied = false;
    bool applied = false;
    std::int64_t offset_ms = 0;
    const char* source = "native-no-utc-fallback-offset";
};

inline Selection select(bool requested, bool fallback_applied,
                        bool override_set = false,
                        std::int64_t override_ms = 0) {
    Selection selection;
    selection.requested = requested || override_set;
    selection.fallback_applied = fallback_applied;
    if (override_set && fallback_applied) {
        selection.applied = true;
        selection.offset_ms = override_ms;
        selection.source = "native-explicit-imu-time-offset";
        return selection;
    }
    if (requested && fallback_applied) {
        selection.applied = true;
        selection.offset_ms = kSourceUtcWallClockOffsetMs;
        selection.source = "source-utc-wall-clock-offset-minus-20ms";
    } else if (requested || override_set) {
        selection.source = "native-no-offset-fallback-not-applied";
    }
    return selection;
}

}  // namespace libgnss::native_utc_fallback_imu_offset
