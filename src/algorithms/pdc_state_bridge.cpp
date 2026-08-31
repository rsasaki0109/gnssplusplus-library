#include <libgnss++/algorithms/pdc_state_bridge.hpp>

#include <Eigen/Sparse>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace libgnss::pdc_state_bridge {
namespace {

constexpr int kStateStride = 12;
constexpr int kPositionOffset = 0;
constexpr int kClockOffset = 3;
constexpr int kVelocityOffset = 8;
constexpr int kClockRateOffset = 11;
constexpr double kMaxTemporalGapSeconds = 1.5;

int positionColumn(std::size_t epoch, int axis) {
    return static_cast<int>(kStateStride * epoch + kPositionOffset + axis);
}

int clockColumn(std::size_t epoch, int group) {
    return static_cast<int>(kStateStride * epoch + kClockOffset + group);
}

int velocityColumn(std::size_t epoch, int axis) {
    return static_cast<int>(kStateStride * epoch + kVelocityOffset + axis);
}

int clockRateColumn(std::size_t epoch) {
    return static_cast<int>(kStateStride * epoch + kClockRateOffset);
}

double huberLoss(double normalized, double threshold) {
    const double magnitude = std::abs(normalized);
    if (!std::isfinite(magnitude)) return std::numeric_limits<double>::infinity();
    if (magnitude <= threshold) return 0.5 * normalized * normalized;
    return threshold * (magnitude - 0.5 * threshold);
}

double huberWeight(double normalized, double threshold) {
    const double magnitude = std::abs(normalized);
    if (!std::isfinite(magnitude) || magnitude <= threshold) return 1.0;
    return threshold / magnitude;
}

struct NormalEquation {
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd rhs;
};

struct IndexedRows {
    std::vector<std::vector<const PseudorangeRow*>> pseudorange;
    std::vector<std::vector<const DopplerRow*>> doppler;
};

bool validOptions(const Options& options) {
    return options.max_iterations > 0 && options.min_pseudorange_rows >= 4 &&
           options.min_doppler_rows >= 4 &&
           options.pseudorange_huber_threshold_sigma > 0.0 &&
           options.doppler_huber_threshold_sigma > 0.0 &&
           options.position_prior_sigma_m > 0.0 &&
           options.clock_prior_sigma_m > 0.0 &&
           options.velocity_prior_sigma_mps > 0.0 &&
           options.clock_rate_prior_sigma_mps > 0.0 &&
           options.motion_sigma_m > 0.0 && options.clock_motion_sigma_m > 0.0 &&
           options.clock_jump_sigma_m > 0.0 &&
           options.inter_system_clock_motion_sigma_m > 0.0 &&
           options.clock_rate_between_sigma_mps > 0.0 &&
           options.max_velocity_mps > 0.0 && options.max_clock_rate_mps > 0.0 &&
           options.max_normalized_rms > 0.0 && options.max_position_norm_m > 1.0e6;
}

double interval(const std::vector<EpochInput>& epochs, std::size_t current) {
    if (current == 0 || current >= epochs.size()) return std::numeric_limits<double>::quiet_NaN();
    const double dt = epochs[current].time - epochs[current - 1].time;
    return std::isfinite(dt) && dt > 0.0 && dt <= kMaxTemporalGapSeconds
               ? dt
               : std::numeric_limits<double>::quiet_NaN();
}

double computeCost(const std::vector<EpochInput>& epochs,
                   const IndexedRows& rows,
                   const Eigen::VectorXd& state,
                   const Options& options) {
    double cost = 0.0;
    for (std::size_t i = 0; i < epochs.size(); ++i) {
        for (int axis = 0; axis < 3; ++axis) {
            const double normalized =
                (state(positionColumn(i, axis)) - epochs[i].seed_position_ecef(axis)) /
                options.position_prior_sigma_m;
            cost += 0.5 * normalized * normalized;
        }
        for (int group = 0; group < 5; ++group) {
            const double normalized =
                (state(clockColumn(i, group)) - epochs[i].seed_clock_bias_m) /
                options.clock_prior_sigma_m;
            cost += 0.5 * normalized * normalized;
        }
        for (int axis = 0; axis < 3; ++axis) {
            cost += 0.5 * std::pow(
                state(velocityColumn(i, axis)) / options.velocity_prior_sigma_mps,
                2.0);
        }
        cost += 0.5 * std::pow(
            state(clockRateColumn(i)) / options.clock_rate_prior_sigma_mps, 2.0);

        const double dt = interval(epochs, i);
        if (std::isfinite(dt)) {
            for (int group = 0; group < 5; ++group) {
                const double sigma = group == 0
                    ? (epochs[i].clock_jump ? options.clock_jump_sigma_m
                                             : options.clock_motion_sigma_m)
                    : options.inter_system_clock_motion_sigma_m;
                double residual = state(clockColumn(i, group)) -
                                  state(clockColumn(i - 1, group));
                if (group == 0) {
                    residual -= 0.5 * dt *
                        (state(clockRateColumn(i - 1)) + state(clockRateColumn(i)));
                }
                cost += 0.5 * std::pow(residual / sigma, 2.0);
            }
            for (int axis = 0; axis < 3; ++axis) {
                const double residual =
                    state(positionColumn(i, axis)) -
                    state(positionColumn(i - 1, axis)) -
                    0.5 * dt * (state(velocityColumn(i - 1, axis)) +
                                state(velocityColumn(i, axis)));
                cost += 0.5 * std::pow(residual / options.motion_sigma_m, 2.0);
            }
            cost += 0.5 * std::pow(
                (state(clockRateColumn(i)) - state(clockRateColumn(i - 1))) /
                    options.clock_rate_between_sigma_mps,
                2.0);
        }
    }
    for (std::size_t i = 0; i < rows.pseudorange.size(); ++i) {
        for (const auto* row : rows.pseudorange[i]) {
            const Vector3d delta =
                row->satellite_position_ecef - state.segment<3>(positionColumn(i, 0));
            const double range = delta.norm();
            if (!(range > 0.0) || !std::isfinite(range)) return std::numeric_limits<double>::infinity();
            const double predicted = range +
                state(clockColumn(i, clockGroupIndex(row->clock_group)));
            const double normalized =
                (predicted - row->corrected_pseudorange_m) / row->sigma_m;
            cost += huberLoss(normalized, options.pseudorange_huber_threshold_sigma);
        }
    }
    for (std::size_t i = 0; i < rows.doppler.size(); ++i) {
        for (const auto* row : rows.doppler[i]) {
            const double predicted =
                row->los.dot(state.segment<3>(velocityColumn(i, 0))) +
                state(clockRateColumn(i));
            const double normalized =
                (predicted - row->residual_mps) / row->sigma_mps;
            cost += huberLoss(normalized, options.doppler_huber_threshold_sigma);
        }
    }
    return cost;
}

NormalEquation buildNormalEquation(const std::vector<EpochInput>& epochs,
                                   const IndexedRows& rows,
                                   const Eigen::VectorXd& state,
                                   const Options& options) {
    const int state_size = static_cast<int>(epochs.size()) * kStateStride;
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve((rows.pseudorange.size() + rows.doppler.size()) * 80U +
                     epochs.size() * 180U);
    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(state_size);
    auto addRow = [&](const std::vector<int>& columns,
                      const std::vector<double>& coefficients,
                      double residual,
                      double sigma,
                      double robust_threshold) {
        if (columns.empty() || columns.size() != coefficients.size() ||
            !(sigma > 0.0) || !std::isfinite(residual)) return;
        const double normalized = residual / sigma;
        const double weight = huberWeight(normalized, robust_threshold) /
                              (sigma * sigma);
        if (!std::isfinite(weight)) return;
        for (std::size_t a = 0; a < columns.size(); ++a) {
            rhs(columns[a]) += weight * coefficients[a] * residual;
            for (std::size_t b = 0; b < columns.size(); ++b) {
                triplets.emplace_back(columns[a], columns[b],
                                      weight * coefficients[a] * coefficients[b]);
            }
        }
    };

    for (std::size_t i = 0; i < epochs.size(); ++i) {
        for (int axis = 0; axis < 3; ++axis) {
            addRow({positionColumn(i, axis)}, {1.0},
                   epochs[i].seed_position_ecef(axis) -
                       state(positionColumn(i, axis)),
                   options.position_prior_sigma_m,
                   std::numeric_limits<double>::infinity());
        }
        for (int group = 0; group < 5; ++group) {
            addRow({clockColumn(i, group)}, {1.0},
                   epochs[i].seed_clock_bias_m - state(clockColumn(i, group)),
                   options.clock_prior_sigma_m,
                   std::numeric_limits<double>::infinity());
        }
        for (int axis = 0; axis < 3; ++axis) {
            addRow({velocityColumn(i, axis)}, {1.0},
                   -state(velocityColumn(i, axis)),
                   options.velocity_prior_sigma_mps,
                   std::numeric_limits<double>::infinity());
        }
        addRow({clockRateColumn(i)}, {1.0}, -state(clockRateColumn(i)),
               options.clock_rate_prior_sigma_mps,
               std::numeric_limits<double>::infinity());

        const double dt = interval(epochs, i);
        if (std::isfinite(dt)) {
            for (int group = 0; group < 5; ++group) {
                const double sigma = group == 0
                    ? (epochs[i].clock_jump ? options.clock_jump_sigma_m
                                             : options.clock_motion_sigma_m)
                    : options.inter_system_clock_motion_sigma_m;
                std::vector<int> columns = {clockColumn(i - 1, group),
                                            clockColumn(i, group)};
                std::vector<double> coefficients = {-1.0, 1.0};
                double model = state(clockColumn(i, group)) -
                               state(clockColumn(i - 1, group));
                if (group == 0) {
                    columns.push_back(clockRateColumn(i - 1));
                    columns.push_back(clockRateColumn(i));
                    coefficients.push_back(-0.5 * dt);
                    coefficients.push_back(-0.5 * dt);
                    model -= 0.5 * dt *
                        (state(clockRateColumn(i - 1)) + state(clockRateColumn(i)));
                }
                addRow(columns, coefficients, -model, sigma,
                       std::numeric_limits<double>::infinity());
            }
            for (int axis = 0; axis < 3; ++axis) {
                const double model =
                    state(positionColumn(i, axis)) -
                    state(positionColumn(i - 1, axis)) -
                    0.5 * dt * (state(velocityColumn(i - 1, axis)) +
                                state(velocityColumn(i, axis)));
                addRow({positionColumn(i - 1, axis), positionColumn(i, axis),
                        velocityColumn(i - 1, axis), velocityColumn(i, axis)},
                       {-1.0, 1.0, -0.5 * dt, -0.5 * dt}, -model,
                       options.motion_sigma_m,
                       std::numeric_limits<double>::infinity());
            }
            const double model =
                state(clockRateColumn(i)) - state(clockRateColumn(i - 1));
            addRow({clockRateColumn(i - 1), clockRateColumn(i)}, {-1.0, 1.0},
                   -model, options.clock_rate_between_sigma_mps,
                   std::numeric_limits<double>::infinity());
        }
    }

    for (std::size_t i = 0; i < rows.pseudorange.size(); ++i) {
        for (const auto* row : rows.pseudorange[i]) {
            const Vector3d delta =
                row->satellite_position_ecef - state.segment<3>(positionColumn(i, 0));
            const double range = delta.norm();
            if (!(range > 0.0) || !std::isfinite(range)) continue;
            const Vector3d los = delta / range;
            const int group = clockGroupIndex(row->clock_group);
            const double predicted = range + state(clockColumn(i, group));
            addRow({positionColumn(i, 0), positionColumn(i, 1), positionColumn(i, 2),
                    clockColumn(i, group)},
                   {-los.x(), -los.y(), -los.z(), 1.0},
                   row->corrected_pseudorange_m - predicted, row->sigma_m,
                   options.pseudorange_huber_threshold_sigma);
        }
    }
    for (std::size_t i = 0; i < rows.doppler.size(); ++i) {
        for (const auto* row : rows.doppler[i]) {
            const double predicted =
                row->los.dot(state.segment<3>(velocityColumn(i, 0))) +
                state(clockRateColumn(i));
            addRow({velocityColumn(i, 0), velocityColumn(i, 1),
                    velocityColumn(i, 2), clockRateColumn(i)},
                   {row->los.x(), row->los.y(), row->los.z(), 1.0},
                   row->residual_mps - predicted, row->sigma_mps,
                   options.doppler_huber_threshold_sigma);
        }
    }
    NormalEquation result;
    result.hessian.resize(state_size, state_size);
    result.hessian.setFromTriplets(triplets.begin(), triplets.end());
    result.hessian.makeCompressed();
    result.rhs = std::move(rhs);
    return result;
}

}  // namespace

int clockGroupIndex(GNSSSystem group) {
    switch (group) {
        case GNSSSystem::GPS:
        case GNSSSystem::QZSS:
        case GNSSSystem::UNKNOWN:
            return 0;
        case GNSSSystem::Galileo:
            return 1;
        case GNSSSystem::BeiDou:
            return 2;
        case GNSSSystem::GLONASS:
            return 3;
        case GNSSSystem::NavIC:
            return 4;
        default:
            return 0;
    }
}

IntegratedPositionSeeds integratePositionSeeds(
    const std::vector<EpochInput>& epochs,
    const std::vector<PositionSeedVelocity>& velocities,
    const std::vector<bool>& clock_jumps,
    double max_gap_s) {
    IntegratedPositionSeeds result;
    result.positions.resize(epochs.size());
    if (epochs.empty() || velocities.size() != epochs.size() ||
        !(max_gap_s > 0.0) || !std::isfinite(max_gap_s)) {
        return result;
    }

    std::size_t anchor = epochs.size();
    for (std::size_t i = 0; i < epochs.size(); ++i) {
        const auto& position = epochs[i].seed_position_ecef;
        if (position.allFinite() && std::isfinite(position.norm()) &&
            position.norm() > 1.0e6 && position.norm() < 1.0e7) {
            anchor = i;
            break;
        }
    }
    if (anchor == epochs.size()) return result;

    for (std::size_t i = 0; i < epochs.size(); ++i) {
        result.positions[i] = epochs[i].seed_position_ecef;
    }
    result.anchor_index = anchor;
    result.valid = true;

    Vector3d previous_velocity = Vector3d::Zero();
    bool have_previous_velocity = false;
    if (velocities[anchor].valid &&
        velocities[anchor].velocity_ecef_mps.allFinite()) {
        previous_velocity = velocities[anchor].velocity_ecef_mps;
        have_previous_velocity = true;
    }

    for (std::size_t i = anchor + 1U; i < epochs.size(); ++i) {
        const double dt = epochs[i].time - epochs[i - 1U].time;
        const bool clock_jump = i < clock_jumps.size() && clock_jumps[i];
        const bool continuous = std::isfinite(dt) && dt > 0.0 &&
                                dt <= max_gap_s && !clock_jump;
        const bool current_velocity_valid =
            velocities[i].valid && velocities[i].velocity_ecef_mps.allFinite();
        bool used_integral = false;
        if (continuous && have_previous_velocity) {
            Vector3d displacement = dt * previous_velocity;
            if (current_velocity_valid) {
                displacement = 0.5 * dt *
                               (previous_velocity +
                                velocities[i].velocity_ecef_mps);
            }
            const Vector3d candidate = result.positions[i - 1U] + displacement;
            if (candidate.allFinite() && std::isfinite(candidate.norm()) &&
                candidate.norm() > 1.0e6 && candidate.norm() < 1.0e7) {
                result.positions[i] = candidate;
                ++result.integrated_epochs;
                used_integral = true;
                if (!current_velocity_valid) {
                    ++result.held_velocity_epochs;
                }
            }
        }

        if (!used_integral) {
            const auto& spp_position = epochs[i].seed_position_ecef;
            if (!spp_position.allFinite() ||
                !std::isfinite(spp_position.norm()) ||
                spp_position.norm() <= 1.0e6 || spp_position.norm() >= 1.0e7) {
                result.valid = false;
                return result;
            }
            result.positions[i] = spp_position;
            ++result.per_epoch_spp_fallback_epochs;
        }
        const double displacement =
            (result.positions[i] - epochs[i].seed_position_ecef).norm();
        if (std::isfinite(displacement)) {
            result.max_displacement_from_spp_m = std::max(
                result.max_displacement_from_spp_m, displacement);
        }
        if (used_integral) {
            const double step_speed =
                (result.positions[i] - result.positions[i - 1U]).norm() / dt;
            if (std::isfinite(step_speed)) {
                result.max_integrated_step_speed_mps = std::max(
                    result.max_integrated_step_speed_mps, step_speed);
            }
        }

        if (!continuous || !used_integral) {
            ++result.reset_intervals;
            have_previous_velocity = false;
        }
        if (current_velocity_valid) {
            previous_velocity = velocities[i].velocity_ecef_mps;
            have_previous_velocity = true;
        }
    }
    return result;
}

SolveResult solve(const std::vector<EpochInput>& epochs,
                  const std::vector<PseudorangeRow>& pseudorange_rows,
                  const std::vector<DopplerRow>& doppler_rows,
                  const Options& options) {
    SolveResult result;
    result.pseudorange_rows = pseudorange_rows.size();
    result.doppler_rows = doppler_rows.size();
    result.epochs.resize(epochs.size());
    if (epochs.empty() || !validOptions(options)) {
        result.reason = "invalid-options-or-empty-epochs";
        return result;
    }
    for (const auto& epoch : epochs) {
        if (!epoch.seed_position_ecef.allFinite() ||
            epoch.seed_position_ecef.norm() < 1.0e6 ||
            !std::isfinite(epoch.seed_clock_bias_m)) {
            result.reason = "invalid-epoch-seed";
            return result;
        }
        if (epoch.has_seed_velocity &&
            (!epoch.seed_velocity_ecef_mps.allFinite() ||
             !std::isfinite(epoch.seed_velocity_ecef_mps.norm()))) {
            result.reason = "invalid-velocity-seed";
            return result;
        }
        if (epoch.has_seed_clock_rate &&
            !std::isfinite(epoch.seed_clock_rate_mps)) {
            result.reason = "invalid-clock-rate-seed";
            return result;
        }
    }
    IndexedRows rows;
    rows.pseudorange.resize(epochs.size());
    rows.doppler.resize(epochs.size());
    for (const auto& row : pseudorange_rows) {
        if (row.epoch_index >= epochs.size() ||
            !row.satellite_position_ecef.allFinite() ||
            !std::isfinite(row.corrected_pseudorange_m) ||
            !(row.corrected_pseudorange_m > 0.0) ||
            !std::isfinite(row.sigma_m) || !(row.sigma_m > 0.0)) {
            result.reason = "invalid-pseudorange-row";
            return result;
        }
        rows.pseudorange[row.epoch_index].push_back(&row);
    }
    for (const auto& row : doppler_rows) {
        if (row.epoch_index >= epochs.size() || !row.los.allFinite() ||
            !(row.los.norm() > 0.9 && row.los.norm() < 1.1) ||
            !std::isfinite(row.residual_mps) ||
            !std::isfinite(row.sigma_mps) || !(row.sigma_mps > 0.0)) {
            result.reason = "invalid-doppler-row";
            return result;
        }
        rows.doppler[row.epoch_index].push_back(&row);
    }

    const int state_size = static_cast<int>(epochs.size()) * kStateStride;
    Eigen::VectorXd state = Eigen::VectorXd::Zero(state_size);
    for (std::size_t i = 0; i < epochs.size(); ++i) {
        state.segment<3>(positionColumn(i, 0)) = epochs[i].seed_position_ecef;
        for (int group = 0; group < 5; ++group) {
            state(clockColumn(i, group)) = epochs[i].seed_clock_bias_m;
        }
        if (epochs[i].has_seed_velocity) {
            state.segment<3>(velocityColumn(i, 0)) =
                epochs[i].seed_velocity_ecef_mps;
        }
        if (epochs[i].has_seed_clock_rate) {
            state(clockRateColumn(i)) = epochs[i].seed_clock_rate_mps;
        }
    }
    result.initial_cost = computeCost(epochs, rows, state, options);
    if (!std::isfinite(result.initial_cost)) {
        result.reason = "nonfinite-initial-cost";
        return result;
    }

    double previous_cost = result.initial_cost;
    // Keep the same deterministic LM schedule as the dedicated native PDC
    // solver.  In particular, retrying a factorization with a larger diagonal
    // is a numerical conditioning safeguard, not a route/truth-derived knob.
    double damping = 1.0e-3;
    for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
        const NormalEquation normal = buildNormalEquation(epochs, rows, state, options);
        bool accepted = false;
        double candidate_cost = previous_cost;
        Eigen::VectorXd candidate = state;
        Eigen::VectorXd accepted_delta = Eigen::VectorXd::Zero(state_size);
        const Eigen::VectorXd diagonal =
            normal.hessian.diagonal().cwiseAbs().cwiseMax(1.0);
        for (int damping_attempt = 0; damping_attempt < 18; ++damping_attempt) {
            Eigen::SparseMatrix<double> damped = normal.hessian;
            for (int i = 0; i < state_size; ++i) {
                damped.coeffRef(i, i) += damping * diagonal(i);
            }
            damped.makeCompressed();
            Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
            solver.compute(damped);
            if (solver.info() != Eigen::Success) {
                damping *= 10.0;
                continue;
            }
            const Eigen::VectorXd delta = solver.solve(normal.rhs);
            if (solver.info() != Eigen::Success || !delta.allFinite()) {
                damping *= 10.0;
                continue;
            }
            double step_scale = 1.0;
            for (int line_search_attempt = 0; line_search_attempt < 12;
                 ++line_search_attempt) {
                candidate = state + step_scale * delta;
                candidate_cost = computeCost(epochs, rows, candidate, options);
                if (std::isfinite(candidate_cost) &&
                    candidate_cost <= previous_cost) {
                    accepted_delta = step_scale * delta;
                    accepted = true;
                    break;
                }
                step_scale *= 0.5;
            }
            if (accepted) {
                damping = std::max(1.0e-12, damping * 0.3);
                break;
            }
            damping *= 10.0;
        }
        if (!accepted) {
            result.reason = "line-search-failed";
            return result;
        }
        state = std::move(candidate);
        result.iterations = iteration + 1;
        const double decrease = previous_cost - candidate_cost;
        const double relative_decrease = decrease /
            std::max(1.0, std::abs(previous_cost));
        previous_cost = candidate_cost;
        if (accepted_delta.norm() < 1.0e-10 ||
            (iteration > 0 && decrease >= 0.0 && relative_decrease < 1.0e-5)) {
            result.converged = true;
            break;
        }
    }
    result.final_cost = computeCost(epochs, rows, state, options);
    if (!std::isfinite(result.final_cost)) {
        result.reason = "nonfinite-final-cost";
        return result;
    }
    result.motion_intervals = 0;
    for (std::size_t i = 1; i < epochs.size(); ++i) {
        if (std::isfinite(interval(epochs, i))) ++result.motion_intervals;
    }

    for (std::size_t i = 0; i < epochs.size(); ++i) {
        EpochEstimate& estimate = result.epochs[i];
        estimate.pseudorange_rows = static_cast<int>(rows.pseudorange[i].size());
        estimate.doppler_rows = static_cast<int>(rows.doppler[i].size());
        estimate.state.position_ecef = state.segment<3>(positionColumn(i, 0));
        for (int group = 0; group < 5; ++group) {
            estimate.state.clock_bias_m[static_cast<std::size_t>(group)] =
                state(clockColumn(i, group));
        }
        estimate.state.velocity_ecef_mps = state.segment<3>(velocityColumn(i, 0));
        estimate.state.clock_rate_mps = state(clockRateColumn(i));
        double sum_squared = 0.0;
        double max_abs = 0.0;
        int count = 0;
        int inliers = 0;
        for (const auto* row : rows.pseudorange[i]) {
            const double range =
                (row->satellite_position_ecef - estimate.state.position_ecef).norm();
            const double normalized =
                (range + estimate.state.clock_bias_m[static_cast<std::size_t>(
                    clockGroupIndex(row->clock_group))] - row->corrected_pseudorange_m) /
                row->sigma_m;
            if (!std::isfinite(normalized)) continue;
            sum_squared += normalized * normalized;
            max_abs = std::max(max_abs, std::abs(normalized));
            ++count;
            if (std::abs(normalized) <= options.pseudorange_huber_threshold_sigma) ++inliers;
        }
        for (const auto* row : rows.doppler[i]) {
            const double normalized =
                (row->los.dot(estimate.state.velocity_ecef_mps) +
                 estimate.state.clock_rate_mps - row->residual_mps) /
                row->sigma_mps;
            if (!std::isfinite(normalized)) continue;
            sum_squared += normalized * normalized;
            max_abs = std::max(max_abs, std::abs(normalized));
            ++count;
            if (std::abs(normalized) <= options.doppler_huber_threshold_sigma) ++inliers;
        }
        estimate.inlier_rows = inliers;
        estimate.normalized_rms = count > 0
            ? std::sqrt(sum_squared / static_cast<double>(count))
            : std::numeric_limits<double>::infinity();
        estimate.max_abs_normalized_residual = max_abs;
        estimate.valid = estimate.pseudorange_rows >= options.min_pseudorange_rows &&
                         estimate.state.position_ecef.allFinite() &&
                         estimate.state.position_ecef.norm() <= options.max_position_norm_m &&
                         estimate.state.velocity_ecef_mps.allFinite() &&
                         estimate.state.velocity_ecef_mps.norm() <= options.max_velocity_mps &&
                         std::isfinite(estimate.state.clock_rate_mps) &&
                         std::abs(estimate.state.clock_rate_mps) <= options.max_clock_rate_mps &&
                         std::isfinite(estimate.normalized_rms) &&
                         estimate.normalized_rms <= options.max_normalized_rms &&
                         std::isfinite(estimate.max_abs_normalized_residual);
        estimate.reason = estimate.valid ? "valid" :
                          (estimate.pseudorange_rows < options.min_pseudorange_rows
                               ? "insufficient-pseudorange-rows"
                               : "quality-gate-rejected");
        if (estimate.valid) {
            ++result.valid_epochs;
            result.max_velocity_norm_mps = std::max(
                result.max_velocity_norm_mps, estimate.state.velocity_ecef_mps.norm());
            result.max_clock_rate_abs_mps = std::max(
                result.max_clock_rate_abs_mps, std::abs(estimate.state.clock_rate_mps));
        }
    }
    result.valid = result.converged && result.valid_epochs > 0;
    result.reason = result.valid ? "valid" :
                    (result.converged ? "epoch-quality-gate-rejected"
                                      : "optimizer-did-not-converge");
    return result;
}

}  // namespace libgnss::pdc_state_bridge
