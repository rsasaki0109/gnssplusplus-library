#pragma once

#include <libgnss++/algorithms/native_imu_refinement_handoff.hpp>

namespace libgnss::native_imu_refinement {

// Boundary for the planned initial-IMU observation refresh. Unlike fromResult,
// positions/velocities/clocks come from a converged GNSS-only graph, while body
// attitude comes from the existing source IMU initializer. No synthetic IMU
// result is created to reuse the refinement boundary. This helper does not
// enable a graph, change masks, apply base corrections, or apply phone offsets.
inline Handoff fromGnssResult(const std::vector<ObservationData>& raw,
                             const FGOProcessor::FGOProblem& initialized_imu,
                             const FGOProcessor::FGOResult& gnss) {
    const auto n = raw.size();
    const auto& imu = initialized_imu.imu;
    if (!n || !gnss.diagnostics.converged || gnss.diagnostics.imu_intervals != 0 ||
        !gnss.epoch_attitude_rpy_rad.empty() || !imu.valid ||
        initialized_imu.epochs.size() != n || gnss.solution.solutions.size() != n ||
        gnss.epoch_velocities_ecef_mps.size() != n ||
        gnss.epoch_clock_bias_components_m.size() != n || gnss.epoch_clock_drift_mps.size() != n ||
        initialized_imu.native_source_clock_c0d_gnss_first_c_handoff_m.size() != n ||
        initialized_imu.native_source_clock_c0d_gnss_first_d_handoff_mps.size() != n ||
        imu.epoch_heading_attitudes_body_to_nav.size() != n ||
        imu.epoch_heading_attitude_times.size() != n || imu.stop_velocity_seeds_nav.size() != n ||
        !imu.nav_origin_ecef.allFinite() || imu.nav_origin_ecef.norm() < 1e6 ||
        imu.nav_origin_ecef.norm() > 1e8 || !std::isfinite(imu.nav_origin_lat_rad) ||
        !std::isfinite(imu.nav_origin_lon_rad) ||
        std::abs(imu.nav_origin_lat_rad) > 1.5707963267948966 ||
        std::abs(imu.nav_origin_lon_rad) > 3.1415926535897932) {
        throw std::invalid_argument("Initial IMU refresh requires complete GNSS-only states and initialized IMU frame");
    }
    Handoff out;
    out.observations = raw;
    out.velocity_ecef_mps = gnss.epoch_velocities_ecef_mps;
    out.clock_components_m = gnss.epoch_clock_bias_components_m;
    out.attitude_body_to_nav = imu.epoch_heading_attitudes_body_to_nav;
    for (std::size_t i = 0; i < n; ++i) {
        const auto& epoch = raw[i];
        const auto& seed = initialized_imu.epochs[i];
        const auto& solution = gnss.solution.solutions[i];
        const auto& velocity = out.velocity_ecef_mps[i];
        const auto& rotation = out.attitude_body_to_nav[i];
        const auto& clock = out.clock_components_m[i];
        if (epoch.raw_source_index != i || seed.raw_source_index != i ||
            epoch.raw_utc_time_millis <= 0 || seed.raw_utc_time_millis != epoch.raw_utc_time_millis ||
            (seed.time - epoch.time) != 0.0 || (solution.time - epoch.time) != 0.0 ||
            (imu.epoch_heading_attitude_times[i] - epoch.time) != 0.0 ||
            (i && (epoch.raw_utc_time_millis <= raw[i - 1].raw_utc_time_millis ||
                   !(epoch.time - raw[i - 1].time > 0.0)))) {
            throw std::invalid_argument("Initial IMU refresh raw epoch identity mismatch");
        }
        if (!solution.position_ecef.allFinite() || solution.position_ecef.norm() < 1e6 ||
            solution.position_ecef.norm() > 1e8 || !seed.position_ecef.allFinite() ||
            (seed.position_ecef - solution.position_ecef).norm() != 0.0 ||
            !velocity.allFinite() || !imu.stop_velocity_seeds_nav[i].allFinite() ||
            !rotation.allFinite() || (rotation.transpose() * rotation - Matrix3d::Identity()).norm() > 1e-9 ||
            std::abs(rotation.determinant() - 1.0) > 1e-9 ||
            !std::isfinite(gnss.epoch_clock_drift_mps[i]) ||
            !std::all_of(clock.begin(), clock.end(), [](double v) { return std::isfinite(v); }) ||
            clock != initialized_imu.native_source_clock_c0d_gnss_first_c_handoff_m[i] ||
            gnss.epoch_clock_drift_mps[i] != initialized_imu.native_source_clock_c0d_gnss_first_d_handoff_mps[i]) {
            throw std::invalid_argument("Initial IMU refresh invalid or inconsistent GNSS handoff state");
        }
        const auto velocity_nav = ecef2enu(velocity, imu.nav_origin_lat_rad, imu.nav_origin_lon_rad);
        if ((velocity_nav - imu.stop_velocity_seeds_nav[i]).norm() > 1e-7)
            throw std::invalid_argument("Initial IMU refresh velocity frame mismatch");
        auto& observation = out.observations[i];
        observation.receiver_position = solution.position_ecef;
        observation.receiver_clock_bias = clock[0] / constants::SPEED_OF_LIGHT;
        observation.receiver_clock_drift_mps = gnss.epoch_clock_drift_mps[i];
    }
    return out;
}

}  // namespace libgnss::native_imu_refinement
