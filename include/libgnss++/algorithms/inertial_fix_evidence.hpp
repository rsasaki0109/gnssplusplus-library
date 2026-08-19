#pragma once

#include <cmath>
#include <limits>

#include <Eigen/Dense>

namespace libgnss::inertial_fix_evidence {

struct Config {
    double maximum_time_error_s = 0.05;
    double covariance_scale = 16.0;
    // chi-square(3 dof, 0.999) / 3.
    double maximum_nis_per_dimension = 5.422;
    double maximum_position_delta_m = 2.0;
};

struct Evidence {
    bool available = false;
    bool healthy_independent_anchor = false;
    double time_error_s = std::numeric_limits<double>::quiet_NaN();
    Eigen::Vector3d predicted_position_ecef = Eigen::Vector3d::Zero();
    Eigen::Matrix3d predicted_position_covariance_ecef =
        Eigen::Matrix3d::Zero();
    Eigen::Vector3d primary_candidate_ecef = Eigen::Vector3d::Zero();
};

struct Decision {
    bool passed = false;
    double position_delta_m = std::numeric_limits<double>::quiet_NaN();
    double nis_per_dimension = std::numeric_limits<double>::quiet_NaN();
};

inline Decision evaluate(const Config& config, const Evidence& evidence) {
    Decision decision;
    if (!evidence.available ||
        !evidence.healthy_independent_anchor ||
        !std::isfinite(evidence.time_error_s) ||
        !evidence.predicted_position_ecef.allFinite() ||
        !evidence.predicted_position_covariance_ecef.allFinite() ||
        !evidence.primary_candidate_ecef.allFinite() ||
        !std::isfinite(config.covariance_scale) ||
        config.covariance_scale <= 0.0) {
        return decision;
    }

    const Eigen::Vector3d separation =
        evidence.primary_candidate_ecef -
        evidence.predicted_position_ecef;
    decision.position_delta_m = separation.norm();
    Eigen::Matrix3d covariance =
        config.covariance_scale * 0.5 *
            (evidence.predicted_position_covariance_ecef +
             evidence.predicted_position_covariance_ecef.transpose()) +
        Eigen::Matrix3d::Identity() * 0.01;
    Eigen::LDLT<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success || !solver.isPositive()) {
        return decision;
    }
    const Eigen::Vector3d solved = solver.solve(separation);
    decision.nis_per_dimension = separation.dot(solved) / 3.0;
    decision.passed =
        evidence.time_error_s <= config.maximum_time_error_s &&
        std::isfinite(decision.position_delta_m) &&
        decision.position_delta_m <=
            config.maximum_position_delta_m &&
        std::isfinite(decision.nis_per_dimension) &&
        decision.nis_per_dimension >= 0.0 &&
        decision.nis_per_dimension <=
            config.maximum_nis_per_dimension;
    return decision;
}

}  // namespace libgnss::inertial_fix_evidence
