#pragma once

#include <Eigen/Core>
#include <cmath>

namespace libgnss::safe_float_continuity {

struct Config {
    bool enabled = false;
    double maximum_anchor_age_s = 6.0;
    double maximum_velocity_age_s = 6.0;
    // A last solver output may bridge only an isolated sub-second gap.  A
    // generated continuity output is never remembered as another anchor.
    double maximum_solver_gap_anchor_age_s = 0.5;
    double maximum_speed_mps = 80.0;
    double velocity_sigma_mps = 3.0;
};

struct Result {
    bool valid = false;
    Eigen::Vector3d position_ecef = Eigen::Vector3d::Zero();
    double position_variance_m2 = 0.0;
};

inline Result propagate(
    const Config& config,
    const Eigen::Vector3d& anchor_position_ecef,
    double anchor_age_s,
    const Eigen::Vector3d& velocity_ecef,
    double velocity_age_s) {
    Result result;
    if (
        !config.enabled ||
        !anchor_position_ecef.allFinite() ||
        !velocity_ecef.allFinite() ||
        !std::isfinite(anchor_age_s) ||
        anchor_age_s <= 0.0 ||
        anchor_age_s > config.maximum_anchor_age_s ||
        !std::isfinite(velocity_age_s) ||
        velocity_age_s < 0.0 ||
        velocity_age_s > config.maximum_velocity_age_s ||
        !std::isfinite(config.maximum_speed_mps) ||
        config.maximum_speed_mps <= 0.0 ||
        velocity_ecef.norm() > config.maximum_speed_mps ||
        !std::isfinite(config.velocity_sigma_mps) ||
        config.velocity_sigma_mps <= 0.0) {
        return result;
    }
    result.position_ecef =
        anchor_position_ecef + velocity_ecef * anchor_age_s;
    result.position_variance_m2 =
        25.0 + std::pow(config.velocity_sigma_mps * anchor_age_s, 2.0);
    result.valid =
        result.position_ecef.allFinite() &&
        std::isfinite(result.position_variance_m2);
    return result;
}

}  // namespace libgnss::safe_float_continuity
