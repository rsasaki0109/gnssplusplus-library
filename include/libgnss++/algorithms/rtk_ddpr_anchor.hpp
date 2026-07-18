#pragma once

#include <cstddef>
#include <limits>
#include <vector>

#include <Eigen/Dense>

namespace libgnss::rtk_ddpr_anchor {

struct Observation {
    Eigen::Vector3d reference_satellite_rover_ecef = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_satellite_rover_ecef = Eigen::Vector3d::Zero();
    Eigen::Vector3d reference_satellite_base_ecef = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_satellite_base_ecef = Eigen::Vector3d::Zero();
    double dd_pseudorange_m = 0.0;
};

struct Config {
    std::size_t min_observations = 4;
    std::size_t max_iterations = 8;
    std::size_t max_fde_removals = 3;
    double convergence_m = 1e-4;
    double fde_threshold_m = 10.0;
};

enum class Status {
    ACCEPTED,
    INVALID_CONFIG,
    INVALID_INPUT,
    INSUFFICIENT_OBSERVATIONS,
    SINGULAR_GEOMETRY,
    DID_NOT_CONVERGE,
};

struct Result {
    Status status = Status::INVALID_INPUT;
    bool valid = false;
    Eigen::Vector3d position_ecef = Eigen::Vector3d::Zero();
    Eigen::Matrix3d covariance_ecef = Eigen::Matrix3d::Zero();
    std::size_t observations_used = 0;
    std::size_t observations_rejected = 0;
    double residual_rms_m = std::numeric_limits<double>::quiet_NaN();
    double residual_max_m = std::numeric_limits<double>::quiet_NaN();
};

/** Solve a rover ECEF anchor from code double differences only. */
Result solve(const std::vector<Observation>& observations,
             const Eigen::Vector3d& base_position_ecef,
             const Eigen::Vector3d& initial_rover_position_ecef,
             const Config& config = {});

}  // namespace libgnss::rtk_ddpr_anchor
