#pragma once

#include <libgnss++/algorithms/fgo.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace libgnss::native_imu_refinement {

// Input for a future observation rebuild, not a second solve of unchanged rows.
// Keep C7 in metres separately from ObservationData's legacy seconds field.
struct Handoff {
    std::vector<ObservationData> observations;
    std::vector<Vector3d> velocity_ecef_mps;
    std::vector<Matrix3d> attitude_body_to_nav;
    std::vector<FGOProcessor::EpochClockBiasComponentsM> clock_components_m;
};

// This boundary accepts only complete, converged, same-run IMU output. It has
// no file input, nearest-time join, missing-state fill, or position-offset step.
// The caller must retain the original nav frame and rebuild with SPP disabled.
inline Handoff fromResult(const std::vector<ObservationData>& raw,
                          const FGOProcessor::FGOProblem& source,
                          const FGOProcessor::FGOResult& result) {
    const std::size_t n = raw.size();
    if (!n || !result.diagnostics.converged || !source.imu.valid ||
        source.epochs.size() != n || result.solution.solutions.size() != n ||
        result.epoch_velocity_nav_mps.size() != n ||
        result.epoch_attitude_rpy_rad.size() != n ||
        result.epoch_clock_drift_mps.size() != n ||
        result.epoch_clock_bias_components_m.size() != n ||
        !source.imu.nav_origin_ecef.allFinite() ||
        source.imu.nav_origin_ecef.norm() < 1e6 ||
        !std::isfinite(source.imu.nav_origin_lat_rad) ||
        !std::isfinite(source.imu.nav_origin_lon_rad) ||
        std::abs(source.imu.nav_origin_lat_rad) > 1.5707963267948966 ||
        std::abs(source.imu.nav_origin_lon_rad) > 3.1415926535897932) {
        throw std::invalid_argument("IMU refinement requires complete converged native states and nav frame");
    }
    Handoff out;
    out.observations = raw;
    out.clock_components_m = result.epoch_clock_bias_components_m;
    out.velocity_ecef_mps.reserve(n);
    out.attitude_body_to_nav.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        const auto& epoch = raw[i];
        const auto& seed = source.epochs[i];
        const auto& solution = result.solution.solutions[i];
        const auto& velocity = result.epoch_velocity_nav_mps[i];
        const auto& rpy = result.epoch_attitude_rpy_rad[i];
        const auto& clock = out.clock_components_m[i];
        if (epoch.raw_source_index != i || seed.raw_source_index != i ||
            epoch.raw_utc_time_millis <= 0 ||
            seed.raw_utc_time_millis != epoch.raw_utc_time_millis ||
            (seed.time - epoch.time) != 0.0 ||
            (solution.time - epoch.time) != 0.0 ||
            (i && (epoch.raw_utc_time_millis <= raw[i-1].raw_utc_time_millis ||
                   !(epoch.time - raw[i-1].time > 0.0)))) {
            throw std::invalid_argument("IMU refinement raw epoch identity mismatch");
        }
        if (!solution.position_ecef.allFinite() ||
            solution.position_ecef.norm() < 1e6 ||
            solution.position_ecef.norm() > 1e8 || !velocity.allFinite() ||
            !rpy.allFinite() || !std::isfinite(result.epoch_clock_drift_mps[i]) ||
            !std::all_of(clock.begin(), clock.end(), [](double v) { return std::isfinite(v); })) {
            throw std::invalid_argument("IMU refinement nonfinite or invalid optimized state");
        }
        auto& rebuilt = out.observations[i];
        rebuilt.receiver_position = solution.position_ecef;
        rebuilt.receiver_clock_bias = clock[0] / constants::SPEED_OF_LIGHT;
        rebuilt.receiver_clock_drift_mps = result.epoch_clock_drift_mps[i];
        out.velocity_ecef_mps.push_back(enu2ecef(
            velocity, source.imu.nav_origin_lat_rad, source.imu.nav_origin_lon_rad));
        out.attitude_body_to_nav.push_back((
            Eigen::AngleAxisd(rpy.z(), Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rpy.y(), Vector3d::UnitY()) *
            Eigen::AngleAxisd(rpy.x(), Vector3d::UnitX())).toRotationMatrix());
    }
    return out;
}
}  // namespace libgnss::native_imu_refinement
