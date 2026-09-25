#pragma once

#include <libgnss++/algorithms/native_imu_refinement_handoff.hpp>
#include <libgnss++/fusion/fusion_initialization.hpp>

namespace libgnss::native_imu_refinement {

struct AttitudeResetHandoff {
    Handoff states;
    std::size_t low_speed_count = 0;
    std::size_t nearest_fill_count = 0;
};

// Source setting.RPYReset selects vel2rpy of the PREVIOUS IMU pass. Smoothing
// supplies attitude only: keep the optimized position, raw velocity, C7 and D
// intact. The ordinary handoff validates the same-run frame and exact epochs.
inline AttitudeResetHandoff fromResultWithAttitudeReset(
    const std::vector<ObservationData>& raw,
    const FGOProcessor::FGOProblem& source,
    const FGOProcessor::FGOResult& result) {
    AttitudeResetHandoff out;
    out.states = fromResult(raw, source, result);
    fusion_initialization::VelocityHeadingConfig config;
    config.nearest_fill_interior = true;
    const auto heading = fusion_initialization::velocityToRpy(
        result.epoch_velocity_nav_mps, config);
    if (!heading.ok || heading.rpy_rad.size() != raw.size() ||
        heading.linear_fill_count != 0)
        throw std::invalid_argument("Refinement attitude reset has no complete velocity-derived heading");
    for (std::size_t i = 0; i < raw.size(); ++i) {
        const auto& rpy = heading.rpy_rad[i];
        if (!rpy.allFinite())
            throw std::invalid_argument("Refinement attitude reset contains a nonfinite heading");
        out.states.attitude_body_to_nav[i] = (
            Eigen::AngleAxisd(rpy.z(), Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rpy.y(), Vector3d::UnitY()) *
            Eigen::AngleAxisd(rpy.x(), Vector3d::UnitX())).toRotationMatrix();
    }
    out.low_speed_count = heading.low_speed_count;
    out.nearest_fill_count = heading.nearest_fill_count;
    return out;
}
}  // namespace libgnss::native_imu_refinement
