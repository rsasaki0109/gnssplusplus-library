#include "libgnss++/algorithms/integrity_consensus.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

namespace libgnss {

IntegrityConsensusManager::IntegrityConsensusManager()
    : IntegrityConsensusManager(Config{}) {}

IntegrityConsensusManager::IntegrityConsensusManager(Config config)
    : config_(config) {}

void IntegrityConsensusManager::reset() {
    state_ = State::NORMAL;
    suspect_count_ = 0;
    disagreement_count_ = 0;
    recovery_count_ = 0;
    has_independent_reset_generation_ = false;
}

IntegrityConsensusManager::Decision IntegrityConsensusManager::update(const Input& input) {
    Decision decision;
    const bool primary_healthy = input.primary.valid &&
        input.primary.position_ecef.allFinite() &&
        std::isfinite(input.primary.covariance_trace_m2) &&
        input.primary.covariance_trace_m2 <= config_.max_primary_covariance_trace_m2;
    const bool independent_available = input.independent.valid &&
        input.independent.position_ecef.allFinite() &&
        std::isfinite(input.independent_age_s) && input.independent_age_s >= 0.0 &&
        input.independent_age_s <= config_.max_independent_age_s;
    const bool independent_healthy = independent_available &&
        std::isfinite(input.independent.covariance_trace_m2) &&
        input.independent.covariance_trace_m2 <=
            config_.max_independent_covariance_trace_m2;

    if (!independent_available) {
        decision.reasons |= INDEPENDENT_UNAVAILABLE;
    } else if (!independent_healthy) {
        decision.reasons |= INDEPENDENT_UNCERTAIN;
    }

    if (primary_healthy && independent_healthy) {
        decision.disagreement_m =
            (input.primary.position_ecef - input.independent.position_ecef).norm();
        const double combined_trace = input.primary.covariance_trace_m2 +
            input.independent.covariance_trace_m2;
        decision.aperture_m = std::clamp(
            config_.aperture_sigma_multiplier * std::sqrt(std::max(0.0, combined_trace)),
            config_.aperture_min_m,
            config_.aperture_max_m);
        decision.estimators_agree = decision.disagreement_m <= decision.aperture_m;
        if (!decision.estimators_agree) decision.reasons |= ESTIMATOR_DISAGREEMENT;
    }

    if (input.primary_suspect || input.hard_primary_suspect) {
        decision.reasons |= PRIMARY_SUSPECT;
    }
    const bool soft_suspect_evidence_available = independent_healthy ||
        !config_.soft_suspect_requires_independent;
    const bool effective_primary_suspect = input.primary_suspect &&
        soft_suspect_evidence_available &&
        (!independent_healthy || !decision.estimators_agree);

    const bool generation_changed = has_independent_reset_generation_ &&
        input.independent_reset_generation != independent_reset_generation_;
    if (generation_changed) decision.reasons |= RESET_GENERATION_CHANGED;
    if (independent_available) {
        independent_reset_generation_ = input.independent_reset_generation;
        has_independent_reset_generation_ = true;
    }

    const bool disagreement = input.fixed_candidate && independent_healthy && primary_healthy &&
        !decision.estimators_agree &&
        (!config_.disagreement_requires_primary_suspect || input.primary_suspect);
    suspect_count_ = effective_primary_suspect ? suspect_count_ + 1 : 0;
    disagreement_count_ = disagreement ? disagreement_count_ + 1 : 0;

    switch (state_) {
        case State::NORMAL:
            if (input.hard_primary_suspect) {
                state_ = State::QUARANTINE;
                recovery_count_ = 0;
                decision.request_primary_reset = true;
            } else if (suspect_count_ >= std::max(1, config_.suspect_streak) ||
                       disagreement_count_ >= std::max(1, config_.suspect_streak)) {
                state_ = State::SUSPECT;
            }
            break;
        case State::SUSPECT:
            if (input.hard_primary_suspect ||
                suspect_count_ >= std::max(1, config_.quarantine_streak) ||
                disagreement_count_ >= std::max(1, config_.quarantine_streak)) {
                state_ = State::QUARANTINE;
                recovery_count_ = 0;
                decision.request_primary_reset = true;
            } else if (!effective_primary_suspect && !disagreement) {
                state_ = State::NORMAL;
            }
            break;
        case State::QUARANTINE:
            if (input.fixed_candidate && decision.estimators_agree &&
                !effective_primary_suspect && !generation_changed) {
                state_ = State::RECOVERY;
                recovery_count_ = 1;
            }
            break;
        case State::RECOVERY:
            if (generation_changed || effective_primary_suspect ||
                (input.fixed_candidate && !decision.estimators_agree)) {
                state_ = State::QUARANTINE;
                recovery_count_ = 0;
            } else if (input.fixed_candidate && decision.estimators_agree) {
                ++recovery_count_;
                if (recovery_count_ >= std::max(1, config_.recovery_streak)) {
                    state_ = State::NORMAL;
                    recovery_count_ = 0;
                    suspect_count_ = 0;
                    disagreement_count_ = 0;
                    decision.promote_joint_anchor = true;
                }
            }
            break;
    }

    decision.state = state_;
    decision.recovery_streak = recovery_count_;
    const bool allow_provisional_recovery_fixed =
        state_ == State::RECOVERY && input.fixed_candidate &&
        decision.estimators_agree && input.recovery_candidate_healthy;
    decision.allow_fixed = !input.fixed_candidate || state_ == State::NORMAL ||
        allow_provisional_recovery_fixed;
    return decision;
}

MultiShadowPositionConsensus::MultiShadowPositionConsensus()
    : MultiShadowPositionConsensus(Config{}) {}

MultiShadowPositionConsensus::MultiShadowPositionConsensus(Config config)
    : config_(config) {
    if (config_.min_independent_shadows < 2) {
        throw std::invalid_argument("min_independent_shadows must be at least two");
    }
    if (!(config_.shadow_agreement_aperture_m > 0.0) ||
        config_.primary_separation_min_m < 0.0 ||
        config_.candidate_max_prediction_error_m < 0.0) {
        throw std::invalid_argument("invalid multi-shadow consensus distance gate");
    }
}

MultiShadowPositionConsensus::Decision MultiShadowPositionConsensus::evaluate(
    const Input& input) const {
    Decision decision;
    if (!input.primary_valid || !input.primary_fixed ||
        !input.primary_position_ecef.allFinite()) {
        decision.reasons |= PRIMARY_NOT_FIXED;
        return decision;
    }

    std::vector<const Shadow*> eligible;
    std::set<std::uint64_t> source_ids;
    for (const auto& shadow : input.shadows) {
        if (!shadow.valid || !shadow.fixed || !shadow.healthy ||
            !shadow.position_ecef.allFinite()) {
            continue;
        }
        if (!source_ids.insert(shadow.source_id).second) {
            decision.reasons |= DUPLICATE_SHADOW_SOURCE;
            return decision;
        }
        eligible.push_back(&shadow);
    }
    if (eligible.size() < static_cast<std::size_t>(config_.min_independent_shadows)) {
        decision.reasons |= INSUFFICIENT_SHADOWS;
        return decision;
    }

    auto diameter = [](const std::vector<const Shadow*>& cluster) {
        double value = 0.0;
        for (std::size_t i = 0; i < cluster.size(); ++i) {
            for (std::size_t j = i + 1; j < cluster.size(); ++j) {
                value = std::max(
                    value,
                    (cluster[i]->position_ecef - cluster[j]->position_ecef).norm());
            }
        }
        return value;
    };

    std::vector<const Shadow*> best;
    double best_diameter = std::numeric_limits<double>::infinity();
    bool ambiguous = false;
    for (std::size_t target_size = eligible.size();
         target_size >= static_cast<std::size_t>(config_.min_independent_shadows);
         --target_size) {
        std::vector<const Shadow*> candidate;
        std::function<void(std::size_t)> visit = [&](std::size_t begin) {
            if (candidate.size() == target_size) {
                const bool fresh = config_.fresh_shadow_max_age_epochs == 0 ||
                    std::any_of(candidate.begin(), candidate.end(), [&](const Shadow* shadow) {
                        return shadow->age_epochs < config_.fresh_shadow_max_age_epochs;
                    });
                const double candidate_diameter = diameter(candidate);
                if (!fresh || candidate_diameter > config_.shadow_agreement_aperture_m) {
                    return;
                }
                if (best.empty()) {
                    best = candidate;
                    best_diameter = candidate_diameter;
                } else {
                    ambiguous = true;
                    if (candidate_diameter < best_diameter) {
                        best = candidate;
                        best_diameter = candidate_diameter;
                    }
                }
                return;
            }
            const std::size_t needed = target_size - candidate.size();
            for (std::size_t i = begin; i + needed <= eligible.size(); ++i) {
                candidate.push_back(eligible[i]);
                visit(i + 1);
                candidate.pop_back();
            }
        };
        visit(0);
        if (!best.empty()) break;
        if (target_size == static_cast<std::size_t>(config_.min_independent_shadows)) break;
    }
    if (best.empty()) {
        decision.reasons |= NO_UNIQUE_CLUSTER;
        return decision;
    }
    decision.cluster_diameter_m = best_diameter;
    if (ambiguous) {
        decision.reasons |= AMBIGUOUS_CLUSTERS;
        return decision;
    }

    for (Eigen::Index axis = 0; axis < 3; ++axis) {
        std::vector<double> values;
        values.reserve(best.size());
        for (const auto* shadow : best) values.push_back(shadow->position_ecef[axis]);
        std::sort(values.begin(), values.end());
        const std::size_t middle = values.size() / 2;
        decision.consensus_position_ecef[axis] = values.size() % 2 == 0
            ? 0.5 * (values[middle - 1] + values[middle])
            : values[middle];
    }
    decision.primary_separation_m =
        (decision.consensus_position_ecef - input.primary_position_ecef).norm();
    if (decision.primary_separation_m < config_.primary_separation_min_m) {
        decision.reasons |= PRIMARY_TOO_CLOSE;
        return decision;
    }
    if (config_.candidate_max_prediction_error_m > 0.0) {
        if (!input.prediction_valid || !input.predicted_position_ecef.allFinite()) {
            decision.reasons |= PREDICTION_UNAVAILABLE;
            return decision;
        }
        decision.prediction_error_m =
            (decision.consensus_position_ecef - input.predicted_position_ecef).norm();
        if (decision.prediction_error_m > config_.candidate_max_prediction_error_m) {
            decision.reasons |= PREDICTION_DISAGREEMENT;
            return decision;
        }
    }
    for (const auto* shadow : best) decision.source_ids.push_back(shadow->source_id);
    std::sort(decision.source_ids.begin(), decision.source_ids.end());
    decision.replace_primary_position = true;
    return decision;
}

RealtimeFixIntegrityGate::RealtimeFixIntegrityGate()
    : RealtimeFixIntegrityGate(Config{}) {}

RealtimeFixIntegrityGate::RealtimeFixIntegrityGate(Config config)
    : config_(std::move(config)), consensus_(config_.consensus) {
    if (config_.primary_max_prefit_rms_m <= 0.0 ||
        config_.primary_min_suppressed_outliers < 1 ||
        config_.primary_max_covariance_trace_m2 <= 0.0 ||
        config_.recovery_min_ratio <= 0.0 ||
        config_.recovery_max_separation_m <= 0.0 ||
        config_.recovery_max_prefit_rms_m <= 0.0 ||
        config_.recovery_max_suppressed_outliers < 0 ||
        config_.residual_streak_min_prefit_rms_m <= 0.0 ||
        config_.residual_streak_max_ratio <= 0.0 ||
        config_.residual_streak_min_suppressed_outliers < 1 ||
        config_.residual_streak_min_outlier_fraction < 0.0 ||
        config_.residual_streak_min_outlier_fraction > 1.0 ||
        config_.residual_streak_epochs < 1 ||
        config_.residual_spike_min_prefit_rms_m <= 0.0 ||
        config_.residual_spike_max_satellites < 1 ||
        config_.base_confidence_min_satellites < 1 ||
        config_.base_confidence_low_satellite_ceiling < 0 ||
        config_.base_confidence_low_satellite_max_ratio <= 0.0 ||
        config_.base_confidence_exonerate_min_satellites < 0 ||
        config_.base_confidence_exonerate_max_prefit_rms_m <= 0.0 ||
        config_.base_confidence_exonerate_max_nis_per_obs <= 0.0) {
        throw std::invalid_argument("invalid realtime FIX integrity configuration");
    }
}

int RealtimeFixIntegrityGate::maxOutputLatencyEpochs() const {
    return config_.enable_residual_policy
        ? std::max(0, config_.residual_streak_epochs - 1)
        : 0;
}

bool RealtimeFixIntegrityGate::residualStreakMatches(
    const PositionSolution& solution) const {
    return solution.isFixed() &&
        std::isfinite(solution.rtk_update_prefit_residual_rms_m) &&
        solution.rtk_update_prefit_residual_rms_m >
            config_.residual_streak_min_prefit_rms_m &&
        std::isfinite(solution.ratio) &&
        solution.ratio <= config_.residual_streak_max_ratio &&
        solution.rtk_update_suppressed_outliers >=
            config_.residual_streak_min_suppressed_outliers &&
        solution.rtk_update_observations > 0 &&
        static_cast<double>(solution.rtk_update_suppressed_outliers) /
                static_cast<double>(solution.rtk_update_observations) >=
            config_.residual_streak_min_outlier_fraction;
}

bool RealtimeFixIntegrityGate::residualSpikeMatches(
    const PositionSolution& solution) const {
    return solution.isFixed() &&
        std::isfinite(solution.rtk_update_prefit_residual_rms_m) &&
        solution.rtk_update_prefit_residual_rms_m >=
            config_.residual_spike_min_prefit_rms_m &&
        solution.num_satellites <= config_.residual_spike_max_satellites;
}

bool RealtimeFixIntegrityGate::independentHealthy(
    const IndependentEstimate& independent) const {
    return independent.present && independent.estimate.valid &&
        independent.estimate.position_ecef.allFinite() &&
        std::isfinite(independent.age_s) && independent.age_s >= 0.0 &&
        independent.age_s <= config_.consensus.max_independent_age_s &&
        std::isfinite(independent.estimate.covariance_trace_m2) &&
        independent.estimate.covariance_trace_m2 <=
            config_.consensus.max_independent_covariance_trace_m2;
}

bool RealtimeFixIntegrityGate::baseConfidenceMatches(
    const PositionSolution& solution) const {
    if (!solution.isFixed()) return false;
    // Field-for-field port of should_demote()'s satellite/ratio gate in
    // scripts/apply_ppc_status_demotion.py: demote when the satellite count
    // is below the hard floor, or the satellite count is at/below the
    // low-satellite ceiling with a weak ambiguity ratio.
    const bool low_satellites =
        solution.num_satellites < config_.base_confidence_min_satellites;
    const bool low_satellite_low_ratio =
        solution.num_satellites <= config_.base_confidence_low_satellite_ceiling &&
        std::isfinite(solution.ratio) &&
        solution.ratio <= config_.base_confidence_low_satellite_max_ratio;
    if (!low_satellites && !low_satellite_low_ratio) return false;

    // Field-for-field port of should_exonerate(): strong runtime telemetry
    // (high satellite count, tight prefit RMS, tight NIS/observation) may
    // override the base gate even though the low-satellite/ratio confidence
    // check fired.
    const bool exonerated =
        solution.num_satellites >= config_.base_confidence_exonerate_min_satellites &&
        std::isfinite(solution.rtk_update_prefit_residual_rms_m) &&
        solution.rtk_update_prefit_residual_rms_m <=
            config_.base_confidence_exonerate_max_prefit_rms_m &&
        std::isfinite(
            solution.rtk_update_normalized_innovation_squared_per_observation) &&
        solution.rtk_update_normalized_innovation_squared_per_observation <=
            config_.base_confidence_exonerate_max_nis_per_obs;
    return !exonerated;
}

void RealtimeFixIntegrityGate::demote(
    PendingEpoch& epoch, bool Telemetry::*reason) {
    epoch.telemetry.*reason = true;
    if (epoch.solution.isFixed()) {
        epoch.solution.status = SolutionStatus::FLOAT;
        epoch.telemetry.output_demoted = true;
    }
}

RealtimeFixIntegrityGate::Emission RealtimeFixIntegrityGate::emitFront(
    std::uint64_t current_index) {
    PendingEpoch epoch = std::move(pending_.front());
    pending_.pop_front();
    const std::uint64_t latency = current_index >= epoch.arrival_index
        ? current_index - epoch.arrival_index
        : 0;
    epoch.telemetry.output_latency_epochs = static_cast<int>(latency);
    return Emission{std::move(epoch.solution), std::move(epoch.telemetry)};
}

RealtimeFixIntegrityGate::Update RealtimeFixIntegrityGate::push(EpochInput input) {
    PendingEpoch pending;
    pending.solution = std::move(input.primary);
    pending.arrival_index = epoch_index_;
    pending.telemetry.fixed_candidate = pending.solution.isFixed();

    const double primary_covariance_trace = pending.solution.position_covariance.trace();
    pending.telemetry.primary_covariance_trace_m2 = primary_covariance_trace;
    pending.telemetry.independent_present = input.independent.present;
    pending.telemetry.independent_valid =
        input.independent.present && input.independent.estimate.valid;
    pending.telemetry.independent_covariance_trace_m2 =
        input.independent.estimate.covariance_trace_m2;
    pending.telemetry.independent_age_s = input.independent.age_s;
    pending.telemetry.independent_reset_generation =
        input.independent.reset_generation;
    const double prefit = pending.solution.rtk_update_prefit_residual_rms_m;
    const int outliers = pending.solution.rtk_update_suppressed_outliers;
    pending.telemetry.primary_suspect = pending.telemetry.fixed_candidate &&
        std::isfinite(prefit) && prefit > config_.primary_max_prefit_rms_m &&
        outliers >= config_.primary_min_suppressed_outliers;
    // hard_primary_suspect must not be corroborated by a present-but-
    // unhealthy independent estimate: an unhealthy independent (e.g. a
    // shadow whose covariance trace is unpopulated/placeholder, per
    // ShadowEstimateHealthGate) carries no real accuracy evidence and must
    // behave exactly like an absent one here, otherwise a residual storm on
    // the primary can lock QUARANTINE forever (RECOVERY requires
    // estimators_agree, which needs a healthy independent that can never
    // arrive). See docs/ppc_online_consensus_design.md.
    pending.telemetry.hard_primary_suspect = pending.telemetry.primary_suspect &&
        independentHealthy(input.independent) &&
        std::isfinite(primary_covariance_trace) &&
        primary_covariance_trace <= config_.primary_max_covariance_trace_m2;

    if (config_.enable_consensus) {
        IntegrityConsensusManager::Input manager_input;
        manager_input.primary.valid = pending.solution.isValid();
        manager_input.primary.position_ecef = pending.solution.position_ecef;
        manager_input.primary.covariance_trace_m2 = primary_covariance_trace;
        if (input.independent.present) {
            manager_input.independent = input.independent.estimate;
            manager_input.independent_age_s = input.independent.age_s;
            manager_input.independent_reset_generation =
                input.independent.reset_generation;
        }
        manager_input.fixed_candidate = pending.telemetry.fixed_candidate;
        manager_input.primary_suspect = pending.telemetry.primary_suspect;
        manager_input.hard_primary_suspect = pending.telemetry.hard_primary_suspect;

        const double recovery_separation_m =
            input.independent.present && input.independent.estimate.valid
            ? (pending.solution.position_ecef -
               input.independent.estimate.position_ecef).norm()
            : std::numeric_limits<double>::infinity();
        const bool recovery_telemetry_healthy =
            pending.telemetry.fixed_candidate &&
            std::isfinite(recovery_separation_m) &&
            recovery_separation_m <= config_.recovery_max_separation_m &&
            std::isfinite(pending.solution.ratio) &&
            pending.solution.ratio >= config_.recovery_min_ratio &&
            std::isfinite(prefit) && prefit <= config_.recovery_max_prefit_rms_m &&
            outliers <= config_.recovery_max_suppressed_outliers;
        manager_input.recovery_candidate_healthy =
            config_.allow_provisional_recovery_fixed &&
            recovery_telemetry_healthy;
        pending.telemetry.consensus = consensus_.update(manager_input);
        if (pending.telemetry.fixed_candidate &&
            !pending.telemetry.consensus.allow_fixed) {
            demote(pending, &Telemetry::consensus_demoted);
        }
    }

    if (config_.enable_residual_policy) {
        pending.telemetry.residual_streak_match =
            residualStreakMatches(pending.solution) ||
            (pending.telemetry.fixed_candidate &&
             std::isfinite(prefit) &&
             prefit > config_.residual_streak_min_prefit_rms_m &&
             std::isfinite(pending.solution.ratio) &&
             pending.solution.ratio <= config_.residual_streak_max_ratio &&
             outliers >= config_.residual_streak_min_suppressed_outliers &&
             pending.solution.rtk_update_observations > 0 &&
             static_cast<double>(outliers) /
                     static_cast<double>(pending.solution.rtk_update_observations) >=
                 config_.residual_streak_min_outlier_fraction);
        residual_streak_count_ = pending.telemetry.residual_streak_match
            ? residual_streak_count_ + 1
            : 0;
        const bool spike = residualSpikeMatches(pending.solution) ||
            (pending.telemetry.fixed_candidate &&
             std::isfinite(prefit) &&
             prefit >= config_.residual_spike_min_prefit_rms_m &&
             pending.solution.num_satellites <=
                 config_.residual_spike_max_satellites);
        if (spike) demote(pending, &Telemetry::residual_spike_demoted);
    } else {
        residual_streak_count_ = 0;
    }

    // Frozen offline "base" confidence gate: per-epoch, no streak
    // confirmation needed, so it demotes the current candidate immediately
    // (same immediate-demote path as the spike rule above) before the epoch
    // is pushed into the retroactive-demotion buffer.
    if (config_.enable_base_confidence_policy && baseConfidenceMatches(pending.solution)) {
        demote(pending, &Telemetry::base_confidence_demoted);
    }

    pending_.push_back(std::move(pending));
    if (config_.enable_residual_policy &&
        residual_streak_count_ >= config_.residual_streak_epochs) {
        const std::size_t count = std::min<std::size_t>(
            static_cast<std::size_t>(config_.residual_streak_epochs),
            pending_.size());
        for (std::size_t offset = 0; offset < count; ++offset) {
            auto& epoch = pending_[pending_.size() - 1 - offset];
            if (epoch.telemetry.residual_streak_match) {
                demote(epoch, &Telemetry::residual_streak_demoted);
            }
        }
    }

    Update result;
    result.current = pending_.back().telemetry;
    const std::size_t max_pending =
        static_cast<std::size_t>(maxOutputLatencyEpochs());
    while (pending_.size() > max_pending) {
        result.emitted.push_back(emitFront(epoch_index_));
    }
    ++epoch_index_;
    return result;
}

std::vector<RealtimeFixIntegrityGate::Emission> RealtimeFixIntegrityGate::flush() {
    std::vector<Emission> output;
    const std::uint64_t current_index = epoch_index_ == 0 ? 0 : epoch_index_ - 1;
    while (!pending_.empty()) output.push_back(emitFront(current_index));
    return output;
}

void RealtimeFixIntegrityGate::reset() {
    consensus_.reset();
    pending_.clear();
    residual_streak_count_ = 0;
    epoch_index_ = 0;
}

ShadowEstimateHealthGate::ShadowEstimateHealthGate()
    : ShadowEstimateHealthGate(Config{}) {}

ShadowEstimateHealthGate::ShadowEstimateHealthGate(Config config)
    : config_(std::move(config)) {}

ShadowEstimateHealthGate::Result ShadowEstimateHealthGate::evaluate(
    const Sample& sample) const {
    Result result;
    const bool status_ok = sample.status_present &&
        (!config_.require_fixed_status || sample.status_fixed);
    const bool telemetry_complete = sample.gdop.has_value() &&
        sample.ddpr_rms_m.has_value() && sample.num_satellites.has_value();
    const bool telemetry_healthy = telemetry_complete &&
        *sample.gdop <= config_.max_gdop &&
        *sample.ddpr_rms_m <= config_.max_ddpr_rms_m &&
        *sample.num_satellites >= config_.min_satellites &&
        std::isfinite(sample.age_s) && sample.age_s >= 0.0 &&
        sample.age_s <= config_.max_age_s;

    // A covariance trace must be present AND strictly positive (a reported
    // exact zero is never a physically meaningful position uncertainty for
    // any real KF/FGO estimator; it indicates the shadow source did not
    // actually populate the field) to count as real accuracy evidence.
    // Absence of that evidence disables demotion authority unless the
    // caller explicitly opts in to a substituted default.
    const bool trace_present = sample.covariance_trace_m2.has_value() &&
        std::isfinite(*sample.covariance_trace_m2) &&
        *sample.covariance_trace_m2 > 0.0;
    double effective_trace = std::numeric_limits<double>::quiet_NaN();
    bool trace_healthy = false;
    if (trace_present) {
        effective_trace = *sample.covariance_trace_m2;
        trace_healthy = effective_trace <= config_.max_covariance_trace_m2;
    } else if (config_.assume_default_covariance_trace) {
        effective_trace = config_.default_covariance_trace_m2;
        trace_healthy = effective_trace <= config_.max_covariance_trace_m2;
    }

    result.healthy = status_ok && telemetry_healthy && trace_healthy;
    result.covariance_trace_m2 = result.healthy
        ? effective_trace
        : std::numeric_limits<double>::infinity();
    return result;
}

}  // namespace libgnss
