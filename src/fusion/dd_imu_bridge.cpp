#include <libgnss++/fusion/dd_imu_bridge.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>

#include <libgnss++/algorithms/lambda.hpp>
#include <libgnss++/fusion/attitude.hpp>

namespace libgnss {
namespace dd_imu_bridge {

DDIMUBridge::DDIMUBridge(const FusionState& initial, BridgeConfig config)
    : config_(config) {
    state_.eskf = initial;
    state_.augmented_covariance = initial.covariance;
}

int DDIMUBridge::findAmbiguity(const AmbiguityKey& key) const {
    for (std::size_t i = 0; i < state_.ambiguities.size(); ++i) {
        if (state_.ambiguities[i].key == key) return static_cast<int>(i);
    }
    return -1;
}

int DDIMUBridge::findSignalAmbiguity(const AmbiguityKey& query) const {
    int best = -1;
    for (std::size_t i = 0; i < state_.ambiguities.size(); ++i) {
        const auto& key = state_.ambiguities[i].key;
        if (key.satellite_prn == query.satellite_prn &&
            key.frequency_index == query.frequency_index &&
            key.satellite_system == query.satellite_system &&
            key.reference_satellite_prn == query.reference_satellite_prn &&
            key.reference_satellite_system == query.reference_satellite_system &&
            key.reference_generation == query.reference_generation &&
            key.signal_type == query.signal_type &&
            (best < 0 || key.generation > state_.ambiguities[static_cast<std::size_t>(best)].key.generation))
            best = static_cast<int>(i);
    }
    return best;
}

void DDIMUBridge::removeAmbiguity(int index) {
    if (index < 0 || index >= static_cast<int>(state_.ambiguities.size())) return;
    const Eigen::Index remove = fusion_index::SIZE + index;
    const Eigen::Index old_n = state_.augmented_covariance.rows();
    Eigen::MatrixXd reduced(old_n - 1, old_n - 1);
    if (remove > 0)
        reduced.topLeftCorner(remove, remove) =
            state_.augmented_covariance.topLeftCorner(remove, remove);
    if (remove + 1 < old_n) {
        reduced.topRightCorner(remove, old_n - remove - 1) =
            state_.augmented_covariance.block(0, remove + 1, remove, old_n - remove - 1);
        reduced.bottomLeftCorner(old_n - remove - 1, remove) =
            state_.augmented_covariance.block(remove + 1, 0, old_n - remove - 1, remove);
        reduced.bottomRightCorner(old_n - remove - 1, old_n - remove - 1) =
            state_.augmented_covariance.block(remove + 1, remove + 1,
                                               old_n - remove - 1, old_n - remove - 1);
    }
    state_.augmented_covariance = std::move(reduced);
    state_.ambiguities.erase(state_.ambiguities.begin() + index);
}

int DDIMUBridge::ensureAmbiguity(const DDObservation& observation) {
    const int existing = findAmbiguity(observation.key);
    if (existing >= 0 && !observation.cycle_slip) return existing;

    AmbiguityKey key = observation.key;
    const int latest = findSignalAmbiguity(key);
    if (!observation.cycle_slip && latest >= 0) return latest;
    if (observation.cycle_slip && latest >= 0) {
        key.generation = std::max(
            key.generation, state_.ambiguities[static_cast<std::size_t>(latest)].key.generation + 1);
        removeAmbiguity(latest);
    }
    DDObservation seeded = observation;
    seeded.key = key;
    const int regenerated = findAmbiguity(key);
    if (regenerated >= 0) return regenerated;

    AmbiguityErrorState ambiguity;
    ambiguity.key = key;
    if (std::isfinite(seeded.carrier_residual_m) && seeded.wavelength_m > 0.0)
        ambiguity.float_value_cycles = seeded.carrier_residual_m / seeded.wavelength_m;
    ambiguity.variance_cycles2 = std::max(
        seeded.carrier_variance_m2 / std::max(seeded.wavelength_m * seeded.wavelength_m, 1e-8), 1.0);
    const Eigen::Index old_n = state_.augmented_covariance.rows();
    state_.augmented_covariance.conservativeResize(old_n + 1, old_n + 1);
    state_.augmented_covariance.row(old_n).setZero();
    state_.augmented_covariance.col(old_n).setZero();
    state_.augmented_covariance(old_n, old_n) = ambiguity.variance_cycles2;
    state_.ambiguities.push_back(ambiguity);
    return static_cast<int>(state_.ambiguities.size() - 1);
}

void DDIMUBridge::acceptPropagatedINS(
    const FusionState& propagated, const Eigen::Matrix<double, 15, 15>& transition) {
    const Eigen::Index n = state_.augmented_covariance.rows();
    if (n > fusion_index::SIZE) {
        state_.augmented_covariance.block(0, fusion_index::SIZE, fusion_index::SIZE,
                                          n - fusion_index::SIZE) =
            transition * state_.augmented_covariance.block(
                0, fusion_index::SIZE, fusion_index::SIZE, n - fusion_index::SIZE);
        state_.augmented_covariance.block(fusion_index::SIZE, 0,
                                          n - fusion_index::SIZE, fusion_index::SIZE) =
            state_.augmented_covariance.block(0, fusion_index::SIZE, fusion_index::SIZE,
                                              n - fusion_index::SIZE).transpose();
    }
    state_.eskf = propagated;
    state_.augmented_covariance.topLeftCorner(fusion_index::SIZE, fusion_index::SIZE) =
        propagated.covariance;
}

UpdateResult DDIMUBridge::update(const std::vector<DDObservation>& observations) {
    last_joint_posterior_valid_ = false;
    // A DD ambiguity is only meaningful for its current satellite/reference
    // arc. Retire states that disappeared or whose reference changed before
    // augmenting the current epoch. Keeping them forever makes the covariance
    // grow without bound and preserves invalid cross-correlations.
    for (int index = static_cast<int>(state_.ambiguities.size()) - 1;
         index >= 0; --index) {
        const auto& key = state_.ambiguities[static_cast<std::size_t>(index)].key;
        const bool active = std::any_of(
            observations.begin(), observations.end(), [&](const DDObservation& observation) {
                const auto& candidate = observation.key;
                return key.satellite_prn == candidate.satellite_prn &&
                       key.frequency_index == candidate.frequency_index &&
                       key.satellite_system == candidate.satellite_system &&
                       key.reference_satellite_prn == candidate.reference_satellite_prn &&
                       key.reference_satellite_system == candidate.reference_satellite_system &&
                       key.reference_generation == candidate.reference_generation &&
                       key.signal_type == candidate.signal_type;
            });
        if (!active) removeAmbiguity(index);
    }
    std::vector<int> ambiguity_indices(observations.size(), -1);
    for (std::size_t i = 0; i < observations.size(); ++i) {
        const auto& observation = observations[i];
        if (observation.carrier_variance_m2 > 0.0 && observation.wavelength_m > 0.0)
            ambiguity_indices[i] = ensureAmbiguity(observation);
    }
    int rows = 0;
    int code_rows = 0;
    int carrier_rows = 0;
    for (std::size_t observation_index = 0; observation_index < observations.size(); ++observation_index) {
        const auto& observation = observations[observation_index];
        code_rows += observation.code_variance_m2 > 0.0 ? 1 : 0;
        carrier_rows += observation.carrier_variance_m2 > 0.0 &&
                        observation.wavelength_m > 0.0 ? 1 : 0;
    }
    rows = code_rows + carrier_rows;
    const Eigen::Index n = state_.augmented_covariance.rows();
    Eigen::MatrixXd h = Eigen::MatrixXd::Zero(rows, n);
    Eigen::VectorXd residual(rows);
    Eigen::MatrixXd r = Eigen::MatrixXd::Zero(rows, rows);
    int row = 0;
    for (std::size_t observation_index = 0; observation_index < observations.size();
         ++observation_index) {
        const auto& observation = observations[observation_index];
        if (observation.code_variance_m2 > 0.0) {
            h.block<1, 3>(row, fusion_index::POSITION) = observation.geometry_enu;
            residual[row] = observation.code_residual_m;
            r(row, row) = observation.code_variance_m2;
            ++row;
        }
        if (observation.carrier_variance_m2 > 0.0 && observation.wavelength_m > 0.0) {
            const int ambiguity = ambiguity_indices[observation_index];
            if (ambiguity < 0) continue;
            h.block<1, 3>(row, fusion_index::POSITION) = observation.geometry_enu;
            h(row, fusion_index::SIZE + ambiguity) = observation.wavelength_m;
            residual[row] = observation.carrier_residual_m - observation.wavelength_m *
                state_.ambiguities[static_cast<std::size_t>(ambiguity)].float_value_cycles;
            r(row, row) = observation.carrier_variance_m2;
            ++row;
        }
    }
    const TightlyCoupledState pre_joint_state = state_;
    auto result = applyLinearUpdate(h.topRows(row), residual.head(row),
                                    r.topLeftCorner(row, row),
                                    config_.max_nis_per_observation);
    result.joint_nis_per_observation = result.nis_per_observation;
    result.carrier_update_accepted = result.ok && carrier_rows > 0;
    if (result.ok && carrier_rows > 0) {
        last_ins_prediction_ = pre_joint_state;
        last_joint_posterior_ = state_;
        last_joint_posterior_valid_ = true;
    }

    // A bad carrier arc must not discard an otherwise healthy code-DD update.
    // First attempt the genuinely joint update; if it fails, retain the live
    // ambiguity state but retry this epoch with code rows only. Partial AR is
    // suppressed by carrier_update_accepted=false in that case.
    if (carrier_rows > 0 && code_rows > 0 &&
        (!result.ok || !config_.commit_carrier_updates)) {
        state_ = pre_joint_state;
        Eigen::MatrixXd code_h = Eigen::MatrixXd::Zero(code_rows, n);
        Eigen::VectorXd code_residual(code_rows);
        Eigen::MatrixXd code_r = Eigen::MatrixXd::Zero(code_rows, code_rows);
        int code_row = 0;
        for (const auto& observation : observations) {
            if (observation.code_variance_m2 <= 0.0) continue;
            code_h.block<1, 3>(code_row, fusion_index::POSITION) =
                observation.geometry_enu;
            code_residual[code_row] = observation.code_residual_m;
            code_r(code_row, code_row) = observation.code_variance_m2;
            ++code_row;
        }
        const double joint_nis = result.nis_per_observation;
        result = applyLinearUpdate(code_h, code_residual, code_r,
                                   config_.max_nis_per_observation);
        result.carrier_fallback_used = true;
        result.carrier_update_accepted = false;
        result.joint_nis_per_observation = joint_nis;
    }
    if (result.ok) consecutive_innovation_rejections_ = 0;
    return result;
}

SSEPartialARResult DDIMUBridge::evaluateSSEPartialAmbiguities(
    const std::vector<DDObservation>& observations,
    bool external_solution_healthy) const {
    SSEPartialARResult result;
    if (!external_solution_healthy ||
        !last_joint_posterior_valid_ ||
        last_joint_posterior_.augmented_covariance.rows() <=
            fusion_index::SIZE) {
        return result;
    }
    result.available = true;
    std::vector<int> indices;
    for (const auto& observation : observations) {
        const auto it = std::find_if(
            last_joint_posterior_.ambiguities.begin(),
            last_joint_posterior_.ambiguities.end(),
            [&](const AmbiguityErrorState& ambiguity) {
                return ambiguity.key == observation.key;
            });
        if (it == last_joint_posterior_.ambiguities.end() ||
            observation.cycle_slip ||
            observation.lock_count < config_.partial_ar_min_lock_count) {
            continue;
        }
        indices.push_back(static_cast<int>(
            std::distance(
                last_joint_posterior_.ambiguities.begin(), it)));
    }
    std::sort(indices.begin(), indices.end(), [&](int a, int b) {
        const auto quality = [&](int index) {
            const auto& key =
                last_joint_posterior_.ambiguities[index].key;
            const auto it = std::find_if(
                observations.begin(), observations.end(),
                [&](const DDObservation& observation) {
                    return observation.key == key;
                });
            if (it == observations.end()) {
                return -std::numeric_limits<double>::infinity();
            }
            const double variance =
                std::max(1e-9, it->carrier_variance_m2);
            return 2.0 * it->elevation_rad +
                   0.02 * it->lock_count -
                   std::abs(it->posterior_abs_residual_m) /
                       std::sqrt(variance);
        };
        return quality(a) > quality(b);
    });
    indices.erase(
        std::unique(indices.begin(), indices.end()), indices.end());
    result.attempted = static_cast<int>(indices.size());

    const int minimum_sse_ambiguities = std::max(
        config_.partial_ar_min_ambiguities,
        config_.sse_min_fixed_ambiguities);
    for (int count = static_cast<int>(indices.size());
         count >= minimum_sse_ambiguities; --count) {
        ++result.subsets_evaluated;
        Eigen::VectorXd float_ambiguities(count);
        Eigen::MatrixXd Qaa(count, count);
        Eigen::MatrixXd Pxa(3, count);
        for (int row = 0; row < count; ++row) {
            const int ambiguity = indices[row];
            float_ambiguities[row] =
                last_joint_posterior_.ambiguities[ambiguity]
                    .float_value_cycles;
            Pxa.col(row) =
                last_joint_posterior_.augmented_covariance.block(
                    fusion_index::POSITION,
                    fusion_index::SIZE + ambiguity, 3, 1);
            for (int column = 0; column < count; ++column) {
                Qaa(row, column) =
                    last_joint_posterior_.augmented_covariance(
                        fusion_index::SIZE + ambiguity,
                        fusion_index::SIZE + indices[column]);
            }
        }
        Qaa = (Qaa + Qaa.transpose()) * 0.5;
        Eigen::LDLT<Eigen::MatrixXd> ambiguity_decomposition(Qaa);
        if (ambiguity_decomposition.info() != Eigen::Success ||
            (ambiguity_decomposition.vectorD().array() <= 1e-12).any()) {
            continue;
        }
        LambdaCandidateDiagnostics search;
        if (!lambdaSearchTopK(
                float_ambiguities, Qaa, 2, search) ||
            search.candidates.cols() < 1 ||
            search.squared_residuals.size() < 2) {
            continue;
        }
        const double ratio =
            search.squared_residuals(0) > 0.0
                ? search.squared_residuals(1) /
                      search.squared_residuals(0)
                : 0.0;
        const double bsr = bootstrappedSuccessRate(
            search.conditional_variances,
            config_.sse_ffrt_covariance_scale);
        FixedFailureRateRatioThreshold ffrt;
        if (!fixedFailureRateRatioThreshold(
                count, bsr, 0.001, ffrt)) {
            continue;
        }
        result.ratio = std::max(result.ratio, ratio);
        result.bootstrapped_success_rate =
            std::max(result.bootstrapped_success_rate, bsr);
        if (std::isfinite(ffrt.minimum_second_to_best_ratio)) {
            result.ffrt_minimum_ratio =
                ffrt.minimum_second_to_best_ratio;
        }
        if (!ffrt.accepts_any_candidate ||
            !std::isfinite(ratio) ||
            ratio < ffrt.minimum_second_to_best_ratio) {
            continue;
        }
        ++result.ratio_passed_subsets;
        const Eigen::VectorXd fixed =
            search.candidates.col(0);
        const Eigen::VectorXd ambiguity_innovation =
            fixed - float_ambiguities;
        const Eigen::VectorXd solved_innovation =
            ambiguity_decomposition.solve(ambiguity_innovation);
        if (ambiguity_decomposition.info() != Eigen::Success ||
            !solved_innovation.allFinite()) {
            continue;
        }
        const Eigen::Vector3d fixed_position =
            last_joint_posterior_.eskf.nominal.position_enu +
            Pxa * solved_innovation;
        const Eigen::Vector3d separation =
            fixed_position -
            last_ins_prediction_.eskf.nominal.position_enu;
        const Eigen::Matrix3d fixed_position_covariance =
            last_joint_posterior_.augmented_covariance.block<3, 3>(
                fusion_index::POSITION,
                fusion_index::POSITION) -
            Pxa * ambiguity_decomposition.solve(Pxa.transpose());
        // The carrier posterior and external INS prediction are not
        // independent: the posterior was formed from that INS prior. For a
        // linear Kalman update Cov(x_post, x_prior) = P_post, so the
        // solution-separation covariance is P_prior - P_post. Treating them
        // as independent and adding the covariances makes the gate
        // dangerously permissive.
        Eigen::Matrix3d separation_covariance =
            last_ins_prediction_.augmented_covariance.block<3, 3>(
                fusion_index::POSITION,
                fusion_index::POSITION) -
            fixed_position_covariance;
        separation_covariance =
            (separation_covariance +
             separation_covariance.transpose()) *
            0.5;
        Eigen::LDLT<Eigen::Matrix3d> separation_decomposition(
            separation_covariance);
        if (separation_decomposition.info() != Eigen::Success ||
            (separation_decomposition.vectorD().array() <= 1e-12).any()) {
            continue;
        }
        const Eigen::Vector3d normalized =
            separation_decomposition.solve(separation);
        if (separation_decomposition.info() != Eigen::Success ||
            !normalized.allFinite()) {
            continue;
        }
        const double statistic_per_dof =
            separation.dot(normalized) / 3.0;
        if (!std::isfinite(statistic_per_dof)) {
            continue;
        }
        result.statistic_per_dof = statistic_per_dof;
        result.position_separation_m = separation.norm();
        result.fixed_position_enu = fixed_position;
        if (statistic_per_dof >
            config_.sse_max_statistic_per_dof) {
            ++result.separation_rejected_subsets;
            continue;
        }
        result.passed = true;
        result.fixed_count = count;
        result.dropped_count =
            static_cast<int>(indices.size()) - count;
        return result;
    }
    return result;
}

UpdateResult DDIMUBridge::applyLinearUpdate(const Eigen::MatrixXd& h,
                                             const Eigen::VectorXd& residual,
                                             const Eigen::MatrixXd& covariance,
                                             double max_nis) {
    UpdateResult result;
    result.observation_count = static_cast<int>(residual.size());
    if (residual.size() == 0) return result;
    Eigen::MatrixXd s = h * state_.augmented_covariance * h.transpose() + covariance;
    s = 0.5 * (s + s.transpose());
    Eigen::LDLT<Eigen::MatrixXd> ldlt(s);
    if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
        result.rejected_by_innovation_gate = true;
        return result;
    }
    const Eigen::VectorXd weighted = ldlt.solve(residual);
    if (!weighted.allFinite()) return result;
    result.nis_per_observation = residual.dot(weighted) / residual.size();
    if (!std::isfinite(result.nis_per_observation) ||
        result.nis_per_observation < 0.0) {
        result.rejected_by_innovation_gate = true;
        return result;
    }
    if (max_nis > 0.0 && result.nis_per_observation > max_nis) {
        result.rejected_by_innovation_gate = true;
        return result;
    }
    const Eigen::MatrixXd pht = state_.augmented_covariance * h.transpose();
    const Eigen::MatrixXd k = ldlt.solve(pht.transpose()).transpose();
    const Eigen::VectorXd correction = k * residual;
    const Eigen::MatrixXd ikh = Eigen::MatrixXd::Identity(h.cols(), h.cols()) - k * h;
    state_.augmented_covariance = ikh * state_.augmented_covariance * ikh.transpose() +
                                  k * covariance * k.transpose();
    state_.augmented_covariance =
        0.5 * (state_.augmented_covariance + state_.augmented_covariance.transpose());
    inject(correction);
    result.ok = true;
    return result;
}

void DDIMUBridge::inject(const Eigen::VectorXd& correction) {
    auto& nominal = state_.eskf.nominal;
    nominal.position_enu += correction.segment<3>(fusion_index::POSITION);
    nominal.velocity_enu += correction.segment<3>(fusion_index::VELOCITY);
    nominal.attitude_body_to_enu =
        (nominal.attitude_body_to_enu *
         attitude::smallAngleQuaternion(correction.segment<3>(fusion_index::ATTITUDE))).normalized();
    nominal.accel_bias += correction.segment<3>(fusion_index::ACCEL_BIAS);
    nominal.gyro_bias += correction.segment<3>(fusion_index::GYRO_BIAS);
    for (std::size_t i = 0; i < state_.ambiguities.size(); ++i)
        state_.ambiguities[i].float_value_cycles += correction[fusion_index::SIZE + i];
    state_.eskf.covariance = state_.augmented_covariance.topLeftCorner<15, 15>();
}

PartialARResult DDIMUBridge::resolvePartialAmbiguities(
    const std::vector<DDObservation>& observations) {
    PartialARResult result;
    std::vector<int> indices;
    for (const auto& observation : observations) {
        int index = findAmbiguity(observation.key);
        if (index < 0)
            index = findSignalAmbiguity(observation.key);
        if (index >= 0 && !observation.cycle_slip &&
            observation.lock_count >= config_.partial_ar_min_lock_count &&
            !state_.ambiguities[index].held)
            indices.push_back(index);
    }
    std::sort(indices.begin(), indices.end(), [&](int a, int b) {
        const auto quality = [&](int index) {
            const auto it = std::find_if(observations.begin(), observations.end(), [&](const DDObservation& o) {
                int candidate = findAmbiguity(o.key);
                if (candidate < 0)
                    candidate = findSignalAmbiguity(o.key);
                return candidate == index;
            });
            if (it == observations.end()) return -1e9;
            return 2.0 * it->elevation_rad + 0.02 * it->lock_count -
                   3.0 * it->posterior_abs_residual_m + 0.05 * std::cos(it->body_azimuth_rad);
        };
        return quality(a) > quality(b);
    });
    indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
    result.attempted = static_cast<int>(indices.size());
    for (int count = static_cast<int>(indices.size());
         count >= config_.partial_ar_min_ambiguities; --count) {
        Eigen::VectorXd float_amb(count);
        Eigen::MatrixXd q(count, count);
        for (int i = 0; i < count; ++i) {
            float_amb[i] = state_.ambiguities[indices[i]].float_value_cycles;
            for (int j = 0; j < count; ++j)
                q(i, j) = state_.augmented_covariance(
                    fusion_index::SIZE + indices[i], fusion_index::SIZE + indices[j]);
        }
        Eigen::VectorXd fixed;
        double ratio = 0.0;
        if (!lambdaSearch(float_amb, q, fixed, ratio)) continue;
        result.ratio = std::max(result.ratio, ratio);
        if (!std::isfinite(ratio) || ratio < config_.lambda_ratio_threshold) continue;
        Eigen::MatrixXd h = Eigen::MatrixXd::Zero(count, state_.augmented_covariance.rows());
        Eigen::VectorXd residual(count);
        for (int i = 0; i < count; ++i) {
            h(i, fusion_index::SIZE + indices[i]) = 1.0;
            residual[i] = fixed[i] - float_amb[i];
        }
        const Eigen::MatrixXd r = Eigen::MatrixXd::Identity(count, count) *
                                  std::pow(config_.fixed_ambiguity_sigma_cycles, 2);
        if (!applyLinearUpdate(h, residual, r, 0.0).ok) continue;
        for (int i = 0; i < count; ++i) state_.ambiguities[indices[i]].held = true;
        result.fixed = true;
        result.fixed_count = count;
        return result;
    }
    return result;
}

SoftResetAction DDIMUBridge::softResetPosition(const Eigen::Vector3d& spp, bool valid) {
    if (!spp.allFinite()) return SoftResetAction::REJECTED;
    ++consecutive_innovation_rejections_;
    if (consecutive_innovation_rejections_ <
        std::max(1, config_.soft_reset_rejection_patience)) {
        return SoftResetAction::REJECTED;
    }
    consecutive_innovation_rejections_ = 0;
    const Eigen::Vector3d innovation = spp - state_.eskf.nominal.position_enu;
    if (valid && innovation.norm() > config_.soft_reset_max_innovation_m) {
        // Add only non-negative diagonal uncertainty and saturate it. The old
        // implementation multiplied the whole position block on every rejected
        // epoch, causing exponential growth and kilometre-scale corrections
        // after a later accepted DD update.
        for (int axis = 0; axis < 3; ++axis) {
            const double current = state_.augmented_covariance(axis, axis);
            const double scaled = current *
                std::max(1.0, config_.rejected_reset_covariance_scale);
            const double target = std::min(
                scaled, std::max(current, config_.soft_reset_max_position_variance_m2));
            state_.augmented_covariance(axis, axis) = target;
        }
        state_.eskf.covariance = state_.augmented_covariance.topLeftCorner<15, 15>();
        return SoftResetAction::COVARIANCE_INFLATION;
    }
    Eigen::MatrixXd h = Eigen::MatrixXd::Zero(3, state_.augmented_covariance.rows());
    h.leftCols<3>().setIdentity();
    const Eigen::Matrix3d r = Eigen::Matrix3d::Identity() *
                              std::pow(config_.soft_reset_position_sigma_m, 2);
    return applyLinearUpdate(h, innovation, r, config_.max_nis_per_observation).ok
               ? SoftResetAction::MEASUREMENT_UPDATE : SoftResetAction::REJECTED;
}

}  // namespace dd_imu_bridge
}  // namespace libgnss
