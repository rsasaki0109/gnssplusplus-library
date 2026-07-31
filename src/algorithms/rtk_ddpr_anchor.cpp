#include <libgnss++/algorithms/rtk_ddpr_anchor.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>

#include <libgnss++/core/coordinates.hpp>

namespace libgnss::rtk_ddpr_anchor {
namespace {

bool validConfig(const Config& config) {
    return config.min_observations >= 3 && config.max_iterations > 0 &&
           std::isfinite(config.convergence_m) && config.convergence_m > 0.0 &&
           std::isfinite(config.fde_threshold_m) && config.fde_threshold_m >= 0.0;
}

bool validObservation(const Observation& observation) {
    return observation.reference_satellite_rover_ecef.allFinite() &&
           observation.target_satellite_rover_ecef.allFinite() &&
           observation.reference_satellite_base_ecef.allFinite() &&
           observation.target_satellite_base_ecef.allFinite() &&
           std::isfinite(observation.dd_pseudorange_m);
}

double modeledDoubleDifference(const Observation& observation,
                               const Eigen::Vector3d& rover,
                               const Eigen::Vector3d& base) {
    const double rover_reference = geodist(observation.reference_satellite_rover_ecef, rover);
    const double rover_target = geodist(observation.target_satellite_rover_ecef, rover);
    const double base_reference = geodist(observation.reference_satellite_base_ecef, base);
    const double base_target = geodist(observation.target_satellite_base_ecef, base);
    return (rover_reference - base_reference) - (rover_target - base_target);
}

bool linearize(const std::vector<Observation>& observations,
               const std::vector<std::size_t>& active,
               const Eigen::Vector3d& rover,
               const Eigen::Vector3d& base,
               Eigen::MatrixXd& design,
               Eigen::VectorXd& residuals) {
    design.resize(active.size(), 3);
    residuals.resize(active.size());
    for (std::size_t row = 0; row < active.size(); ++row) {
        const auto& observation = observations[active[row]];
        const Eigen::Vector3d ref_delta =
            observation.reference_satellite_rover_ecef - rover;
        const Eigen::Vector3d sat_delta = observation.target_satellite_rover_ecef - rover;
        const double ref_range = ref_delta.norm();
        const double sat_range = sat_delta.norm();
        if (!(ref_range > 1.0) || !(sat_range > 1.0)) {
            return false;
        }
        design.row(row) = (-ref_delta / ref_range + sat_delta / sat_range).transpose();
        residuals(row) = observation.dd_pseudorange_m -
                         modeledDoubleDifference(observation, rover, base);
    }
    return design.allFinite() && residuals.allFinite();
}

}  // namespace

Result solve(const std::vector<Observation>& observations,
             const Eigen::Vector3d& base_position_ecef,
             const Eigen::Vector3d& initial_rover_position_ecef,
             const Config& config) {
    Result result;
    if (!validConfig(config)) {
        result.status = Status::INVALID_CONFIG;
        return result;
    }
    if (!base_position_ecef.allFinite() || !initial_rover_position_ecef.allFinite()) {
        result.status = Status::INVALID_INPUT;
        return result;
    }

    std::vector<std::size_t> active;
    active.reserve(observations.size());
    for (std::size_t i = 0; i < observations.size(); ++i) {
        if (validObservation(observations[i])) {
            active.push_back(i);
        }
    }
    if (active.size() < config.min_observations) {
        result.status = Status::INSUFFICIENT_OBSERVATIONS;
        return result;
    }

    Eigen::Vector3d rover = initial_rover_position_ecef;
    Eigen::MatrixXd design;
    Eigen::VectorXd residuals;
    for (std::size_t removal = 0;; ++removal) {
        bool converged = false;
        for (std::size_t iteration = 0; iteration < config.max_iterations; ++iteration) {
            if (!linearize(observations, active, rover, base_position_ecef, design, residuals)) {
                result.status = Status::INVALID_INPUT;
                return result;
            }
            Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(design);
            if (qr.rank() < 3) {
                result.status = Status::SINGULAR_GEOMETRY;
                return result;
            }
            const Eigen::Vector3d correction = qr.solve(residuals);
            if (!correction.allFinite()) {
                result.status = Status::SINGULAR_GEOMETRY;
                return result;
            }
            rover += correction;
            if (correction.norm() <= config.convergence_m) {
                converged = true;
                break;
            }
        }
        if (!converged) {
            result.status = Status::DID_NOT_CONVERGE;
            return result;
        }
        if (!linearize(observations, active, rover, base_position_ecef, design, residuals)) {
            result.status = Status::INVALID_INPUT;
            return result;
        }

        Eigen::Index worst_index = 0;
        const double worst = residuals.cwiseAbs().maxCoeff(&worst_index);
        const bool can_remove = config.fde_threshold_m > 0.0 &&
            worst > config.fde_threshold_m && removal < config.max_fde_removals &&
            active.size() > config.min_observations;
        if (can_remove) {
            // A gross code outlier can bias the three-position fit enough
            // that a good row owns the largest residual. Evaluate every
            // one-row exclusion and retain the hypothesis with the lowest
            // post-fit RMS instead of blindly deleting that row.
            double best_rms = std::numeric_limits<double>::infinity();
            std::vector<std::size_t> best_active;
            Eigen::Vector3d best_rover = rover;
            for (std::size_t excluded = 0; excluded < active.size(); ++excluded) {
                std::vector<std::size_t> trial_active = active;
                trial_active.erase(trial_active.begin() + excluded);
                Eigen::Vector3d trial_rover = rover;
                bool trial_converged = false;
                Eigen::MatrixXd trial_design;
                Eigen::VectorXd trial_residuals;
                for (std::size_t iteration = 0; iteration < config.max_iterations; ++iteration) {
                    if (!linearize(observations, trial_active, trial_rover,
                                   base_position_ecef, trial_design, trial_residuals)) {
                        break;
                    }
                    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> trial_qr(trial_design);
                    if (trial_qr.rank() < 3) {
                        break;
                    }
                    const Eigen::Vector3d correction = trial_qr.solve(trial_residuals);
                    if (!correction.allFinite()) {
                        break;
                    }
                    trial_rover += correction;
                    if (correction.norm() <= config.convergence_m) {
                        trial_converged = true;
                        break;
                    }
                }
                if (!trial_converged ||
                    !linearize(observations, trial_active, trial_rover,
                               base_position_ecef, trial_design, trial_residuals)) {
                    continue;
                }
                const double trial_rms =
                    std::sqrt(trial_residuals.squaredNorm() / trial_active.size());
                if (trial_rms < best_rms) {
                    best_rms = trial_rms;
                    best_active = std::move(trial_active);
                    best_rover = trial_rover;
                }
            }
            if (best_active.empty()) {
                result.status = Status::SINGULAR_GEOMETRY;
                return result;
            }
            active = std::move(best_active);
            rover = best_rover;
            continue;
        }

        const double squared_norm = residuals.squaredNorm();
        const double variance = squared_norm /
            std::max<double>(1.0, static_cast<double>(active.size()) - 3.0);
        const Eigen::Matrix3d normal = design.transpose() * design;
        Eigen::LDLT<Eigen::Matrix3d> ldlt(normal);
        if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
            result.status = Status::SINGULAR_GEOMETRY;
            return result;
        }
        result.status = Status::ACCEPTED;
        result.valid = true;
        result.position_ecef = rover;
        result.covariance_ecef = variance * ldlt.solve(Eigen::Matrix3d::Identity());
        result.observations_used = active.size();
        result.observations_rejected = observations.size() - active.size();
        result.residual_rms_m = std::sqrt(squared_norm / active.size());
        result.residual_max_m = worst;
        return result;
    }
}

}  // namespace libgnss::rtk_ddpr_anchor
