#include <libgnss++/algorithms/fgo.hpp>

#include <libgnss++/algorithms/lambda.hpp>
#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signal_policy.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/models/ionosphere.hpp>
#include <libgnss++/models/troposphere.hpp>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#ifdef GNSSPP_HAS_CHOLMOD
#include <Eigen/CholmodSupport>
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <string>
#include <tuple>


#include "fgo_internal.hpp"

namespace libgnss {

using namespace fgo_internal;

std::vector<FGOProcessor::GeometryFreeSlipShadowEpoch>
FGOProcessor::analyzeGeometryFreeSlipShadow(const FGOProblem& problem,
                                            double threshold_m,
                                            double max_gap_s) {
    struct PhaseSample {
        double single_difference_phase_m = 0.0;
        bool has_single_difference_doppler = false;
        double single_difference_doppler_mps = 0.0;
        std::set<std::size_t> ambiguity_indices;
    };
    struct History {
        GNSSTime time;
        double geometry_free_m = 0.0;
        std::set<std::size_t> first_ambiguity_indices;
        std::set<std::size_t> second_ambiguity_indices;
    };
    struct DopplerHistory {
        GNSSTime time;
        double single_difference_phase_m = 0.0;
        double single_difference_doppler_mps = 0.0;
        std::set<std::size_t> ambiguity_indices;
    };

    using CarrierKey = std::pair<SatelliteId, SignalType>;
    using PairKey = std::tuple<SatelliteId, SignalType, SignalType>;
    std::vector<std::map<CarrierKey, PhaseSample>> phases_by_epoch(
        problem.epochs.size());
    for (const auto& factor : problem.double_difference_carrier_factors) {
        if (factor.epoch_index >= phases_by_epoch.size()) {
            continue;
        }
        auto& phases = phases_by_epoch[factor.epoch_index];
        const auto add_phase = [&](const CarrierKey& key, double phase_m,
                                   const ObservationModelDebug& rover_model,
                                   const ObservationModelDebug& base_model) {
            auto& sample = phases[key];
            sample.single_difference_phase_m = phase_m;
            if (rover_model.has_doppler_residual &&
                base_model.has_doppler_residual) {
                sample.has_single_difference_doppler = true;
                sample.single_difference_doppler_mps =
                    rover_model.doppler_residual_mps -
                    base_model.doppler_residual_mps;
            }
            sample.ambiguity_indices.insert(factor.ambiguity_index);
        };
        add_phase({factor.satellite, factor.signal},
                  factor.rover_satellite_model.raw_carrier_m -
                      factor.base_satellite_model.raw_carrier_m,
                  factor.rover_satellite_model,
                  factor.base_satellite_model);
        // A slip on the DD reference contaminates every target ambiguity in
        // that group, hence the same factor ambiguity is attached here too.
        add_phase({factor.reference_satellite, factor.signal},
                  factor.rover_reference_model.raw_carrier_m -
                      factor.base_reference_model.raw_carrier_m,
                  factor.rover_reference_model,
                  factor.base_reference_model);
    }

    std::vector<GeometryFreeSlipShadowEpoch> shadow(problem.epochs.size());
    std::map<PairKey, History> history;
    std::map<CarrierKey, DopplerHistory> doppler_history;
    std::set<std::size_t> tainted_ambiguities;
    const double safe_threshold = std::max(0.0, threshold_m);
    const double safe_max_gap = std::max(0.0, max_gap_s);
    for (std::size_t epoch_index = 0; epoch_index < phases_by_epoch.size();
         ++epoch_index) {
        std::map<SatelliteId, std::vector<std::pair<SignalType, PhaseSample>>>
            by_satellite;
        for (const auto& [key, sample] : phases_by_epoch[epoch_index]) {
            by_satellite[key.first].push_back({key.second, sample});
        }
        const GNSSTime time = problem.epochs[epoch_index].time;
        std::set<CarrierKey> doppler_events;
        for (const auto& [key, sample] : phases_by_epoch[epoch_index]) {
            const auto previous = doppler_history.find(key);
            if (sample.has_single_difference_doppler &&
                previous != doppler_history.end()) {
                const double dt = time - previous->second.time;
                const bool same_arcs =
                    previous->second.ambiguity_indices ==
                    sample.ambiguity_indices;
                if (same_arcs && dt > 0.0 &&
                    (safe_max_gap <= 0.0 || dt <= safe_max_gap)) {
                    const double predicted_delta_m =
                        0.5 * (previous->second.single_difference_doppler_mps +
                               sample.single_difference_doppler_mps) * dt;
                    const double innovation_m = std::abs(
                        sample.single_difference_phase_m -
                        previous->second.single_difference_phase_m -
                        predicted_delta_m);
                    if (innovation_m > 0.20) {
                        doppler_events.insert(key);
                        shadow[epoch_index].doppler_max_innovation_m =
                            std::max(
                                shadow[epoch_index].doppler_max_innovation_m,
                                innovation_m);
                    }
                }
            }
            if (sample.has_single_difference_doppler) {
                doppler_history[key] = {
                    time, sample.single_difference_phase_m,
                    sample.single_difference_doppler_mps,
                    sample.ambiguity_indices};
            } else {
                doppler_history.erase(key);
            }
        }
        shadow[epoch_index].doppler_event_signals =
            static_cast<int>(doppler_events.size());
        for (const auto& [satellite, samples] : by_satellite) {
            for (std::size_t first = 0; first < samples.size(); ++first) {
                for (std::size_t second = first + 1; second < samples.size();
                     ++second) {
                    const auto& first_sample = samples[first].second;
                    const auto& second_sample = samples[second].second;
                    const double geometry_free =
                        first_sample.single_difference_phase_m -
                        second_sample.single_difference_phase_m;
                    const PairKey key{satellite, samples[first].first,
                                      samples[second].first};
                    const auto previous = history.find(key);
                    if (previous != history.end()) {
                        const double dt = time - previous->second.time;
                        const bool same_arcs =
                            previous->second.first_ambiguity_indices ==
                                first_sample.ambiguity_indices &&
                            previous->second.second_ambiguity_indices ==
                                second_sample.ambiguity_indices;
                        if (same_arcs && dt > 0.0 &&
                            (safe_max_gap <= 0.0 || dt <= safe_max_gap)) {
                            const double jump = std::abs(
                                geometry_free - previous->second.geometry_free_m);
                            if (jump > safe_threshold) {
                                ++shadow[epoch_index].event_pairs;
                                shadow[epoch_index].max_jump_m = std::max(
                                    shadow[epoch_index].max_jump_m, jump);
                                shadow[epoch_index].event_satellite_signals.insert(
                                    {satellite, samples[first].first});
                                shadow[epoch_index].event_satellite_signals.insert(
                                    {satellite, samples[second].first});
                                tainted_ambiguities.insert(
                                    first_sample.ambiguity_indices.begin(),
                                    first_sample.ambiguity_indices.end());
                                tainted_ambiguities.insert(
                                    second_sample.ambiguity_indices.begin(),
                                    second_sample.ambiguity_indices.end());
                                const int doppler_matches =
                                    static_cast<int>(doppler_events.count(
                                        {satellite, samples[first].first})) +
                                    static_cast<int>(doppler_events.count(
                                        {satellite, samples[second].first}));
                                if (doppler_matches == 1) {
                                    ++shadow[epoch_index]
                                          .doppler_isolated_event_pairs;
                                }
                            }
                        }
                    }
                    history[key] = {time, geometry_free,
                                    first_sample.ambiguity_indices,
                                    second_sample.ambiguity_indices};
                }
            }
        }

        std::set<std::size_t> tainted_present;
        for (const auto& [key, sample] : phases_by_epoch[epoch_index]) {
            (void)key;
            for (const std::size_t ambiguity_index : sample.ambiguity_indices) {
                if (tainted_ambiguities.count(ambiguity_index) != 0) {
                    tainted_present.insert(ambiguity_index);
                }
            }
        }
        shadow[epoch_index].tainted_ambiguities =
            static_cast<int>(tainted_present.size());
    }
    return shadow;
}

std::vector<FGOProcessor::SingleDifferenceTdcpFactor>
FGOProcessor::buildClockResilientTemporalCarrierShadow(
    const FGOProblem& problem, double sigma_m, double max_gap_s) {
    using GroupKey = std::pair<GNSSSystem, SignalType>;
    using ObservationKey = std::pair<SatelliteId, SignalType>;
    using HistoryKey = std::tuple<SatelliteId, SatelliteId, SignalType>;

    struct HistorySample {
        std::size_t epoch_index = 0;
        double residual_m = 0.0;
        Vector3d los = Vector3d::Zero();
        int arc_length_epochs = 1;
        bool has_sd_doppler = false;
        double sd_doppler_mps = 0.0;
        double sd_doppler_sigma_mps = 0.0;
    };

    const auto& observations = problem.carrier_observations.empty()
        ? problem.double_difference_pseudorange_observations
        : problem.carrier_observations;
    std::vector<std::map<ObservationKey, const CarrierPhaseFactor*>> by_epoch(
        problem.epochs.size());
    for (const auto& observation : observations) {
        if (observation.epoch_index >= by_epoch.size() ||
            !observation.has_carrier_phase || observation.loss_of_lock ||
            !std::isfinite(observation.corrected_carrier_m) ||
            !std::isfinite(observation.model_debug.geometric_range_m) ||
            !observation.los.allFinite()) {
            continue;
        }
        by_epoch[observation.epoch_index]
                [{observation.satellite, observation.signal}] = &observation;
    }

    std::vector<SingleDifferenceTdcpFactor> factors;
    std::map<GroupKey, SatelliteId> causal_references;
    std::map<HistoryKey, HistorySample> previous;
    const double safe_sigma = std::max(1e-4, sigma_m);
    const double safe_max_gap = std::max(0.0, max_gap_s);

    for (std::size_t epoch_index = 0; epoch_index < by_epoch.size();
         ++epoch_index) {
        std::map<GroupKey, std::vector<const CarrierPhaseFactor*>> groups;
        for (const auto& [key, observation] : by_epoch[epoch_index]) {
            (void)key;
            groups[{observation->satellite.system, observation->signal}]
                .push_back(observation);
        }

        std::map<HistoryKey, HistorySample> current;
        for (const auto& [group_key, group] : groups) {
            if (group.size() < 2) {
                causal_references.erase(group_key);
                continue;
            }

            const CarrierPhaseFactor* reference = nullptr;
            const auto prior_reference = causal_references.find(group_key);
            if (prior_reference != causal_references.end()) {
                for (const auto* candidate : group) {
                    if (candidate->satellite == prior_reference->second) {
                        reference = candidate;
                        break;
                    }
                }
            }
            if (reference == nullptr) {
                reference = *std::max_element(
                    group.begin(), group.end(),
                    [](const auto* lhs, const auto* rhs) {
                        if (lhs->elevation_rad != rhs->elevation_rad) {
                            return lhs->elevation_rad < rhs->elevation_rad;
                        }
                        return rhs->satellite < lhs->satellite;
                    });
            }
            causal_references[group_key] = reference->satellite;

            const double reference_residual =
                reference->corrected_carrier_m -
                reference->model_debug.geometric_range_m;
            for (const auto* observation : group) {
                if (observation->satellite == reference->satellite) {
                    continue;
                }
                const double residual =
                    observation->corrected_carrier_m -
                    observation->model_debug.geometric_range_m -
                    reference_residual;
                const Vector3d los = observation->los - reference->los;
                const HistoryKey history_key{observation->satellite,
                                             reference->satellite,
                                             observation->signal};
                const bool has_sd_doppler =
                    observation->has_doppler_residual &&
                    reference->has_doppler_residual;
                const double sd_doppler_mps = has_sd_doppler
                    ? observation->doppler_residual_mps -
                          reference->doppler_residual_mps
                    : 0.0;
                const double sd_doppler_sigma_mps = has_sd_doppler
                    ? std::hypot(observation->doppler_sigma_mps,
                                 reference->doppler_sigma_mps)
                    : 0.0;
                current[history_key] = {epoch_index, residual, los, 1,
                                        has_sd_doppler, sd_doppler_mps,
                                        sd_doppler_sigma_mps};

                const auto prior = previous.find(history_key);
                if (prior == previous.end()) {
                    continue;
                }
                const double dt = problem.epochs[epoch_index].time -
                                  problem.epochs[prior->second.epoch_index].time;
                if (prior->second.epoch_index + 1 != epoch_index || dt <= 0.0 ||
                    (safe_max_gap > 0.0 && dt > safe_max_gap)) {
                    continue;
                }

                const double delta = residual - prior->second.residual_m;
                if (!std::isfinite(delta)) {
                    continue;
                }
                SingleDifferenceTdcpFactor factor;
                factor.previous_epoch_index = prior->second.epoch_index;
                factor.current_epoch_index = epoch_index;
                factor.satellite = observation->satellite;
                factor.reference_satellite = reference->satellite;
                factor.signal = observation->signal;
                factor.previous_los = prior->second.los;
                factor.los = los;
                factor.delta_carrier_m = delta;
                factor.target_ambiguity_index = observation->ambiguity_index;
                factor.reference_ambiguity_index = reference->ambiguity_index;
                factor.arc_length_epochs = prior->second.arc_length_epochs + 1;
                factor.dt_s = dt;
                factor.has_doppler_witness =
                    prior->second.has_sd_doppler && has_sd_doppler;
                factor.previous_sd_doppler_mps =
                    prior->second.sd_doppler_mps;
                factor.current_sd_doppler_mps = sd_doppler_mps;
                factor.previous_sd_doppler_sigma_mps =
                    prior->second.sd_doppler_sigma_mps;
                factor.current_sd_doppler_sigma_mps =
                    sd_doppler_sigma_mps;
                current[history_key].arc_length_epochs = factor.arc_length_epochs;
                const double sin_el = std::max(
                    0.1, std::sin(std::min(observation->elevation_rad,
                                           reference->elevation_rad)));
                factor.sigma_m = safe_sigma / std::sqrt(sin_el);
                factor.elevation_rad =
                    std::min(observation->elevation_rad,
                             reference->elevation_rad);
                factors.push_back(factor);
            }
        }
        previous = std::move(current);
    }
    return factors;
}

std::vector<FGOProcessor::TemporalCarrierShadowFactorDiagnostics>
FGOProcessor::classifyClockResilientTemporalCarrierShadow(
    const FGOProblem& problem,
    const std::vector<SingleDifferenceTdcpFactor>& factors,
    const std::vector<Vector3d>& epoch_positions_ecef,
    std::vector<FGOEpochDiagnostics>& epoch_diagnostics,
    const std::vector<GeometryFreeSlipShadowEpoch>* geometry_free_shadow,
    const std::vector<std::set<std::size_t>>*
        fde_rejected_ambiguities_by_epoch) {
    const std::size_t num_epochs = problem.epochs.size();
    if (epoch_diagnostics.size() < num_epochs) {
        epoch_diagnostics.resize(num_epochs);
    }
    if (epoch_positions_ecef.size() < num_epochs) {
        return {};
    }

    using DopplerKey =
        std::tuple<std::size_t, SatelliteId, SatelliteId, SignalType>;
    std::map<DopplerKey, SingleDifferenceDopplerFactor> sd_doppler_by_factor;
    for (const auto& doppler : problem.single_difference_doppler_factors) {
        sd_doppler_by_factor[{doppler.epoch_index, doppler.satellite,
                              doppler.reference_satellite, doppler.signal}] =
            doppler;
    }

    constexpr double kResidualOutlierThresholdM = 0.10;
    constexpr double kDopplerNormalizedInnovationThreshold = 5.0;
    constexpr std::size_t kDopplerCalibrationWarmup = 10;
    constexpr std::size_t kDopplerCalibrationWindow = 30;
    constexpr double kDopplerCalibratedScoreThreshold = 5.0;
    using DopplerCalibrationKey =
        std::tuple<std::size_t, SatelliteId, SatelliteId, SignalType>;
    std::map<DopplerCalibrationKey, std::vector<double>>
        doppler_innovation_history;
    const auto median = [](std::vector<double> values) {
        const std::size_t middle = values.size() / 2;
        std::nth_element(values.begin(), values.begin() + middle, values.end());
        double value = values[middle];
        if (values.size() % 2 == 0) {
            const auto lower =
                std::max_element(values.begin(), values.begin() + middle);
            value = 0.5 * (value + *lower);
        }
        return value;
    };

    std::vector<double> sum_sq(num_epochs, 0.0);
    std::vector<TemporalCarrierShadowFactorDiagnostics> classified_factors;
    classified_factors.reserve(factors.size());
    for (const auto& factor : factors) {
        if (factor.previous_epoch_index >= num_epochs ||
            factor.current_epoch_index >= num_epochs) {
            continue;
        }
        const Vector3d previous_delta =
            epoch_positions_ecef[factor.previous_epoch_index] -
            problem.epochs[factor.previous_epoch_index].position_ecef;
        const Vector3d current_delta =
            epoch_positions_ecef[factor.current_epoch_index] -
            problem.epochs[factor.current_epoch_index].position_ecef;
        const double predicted = factor.los.dot(current_delta) -
                                 factor.previous_los.dot(previous_delta);
        const double residual = factor.delta_carrier_m - predicted;
        if (!std::isfinite(residual)) {
            continue;
        }
        auto& diagnostics = epoch_diagnostics[factor.current_epoch_index];
        ++diagnostics.clock_resilient_tdcp_factors;
        sum_sq[factor.current_epoch_index] += residual * residual;
        diagnostics.clock_resilient_tdcp_max_abs_m =
            std::max(diagnostics.clock_resilient_tdcp_max_abs_m,
                     std::abs(residual));

        TemporalCarrierShadowFactorDiagnostics classified;
        classified.factor = factor;
        classified.residual_m = residual;
        classified.normalized_residual =
            residual / std::max(1e-4, factor.sigma_m);
        classified.residual_outlier =
            std::abs(residual) > kResidualOutlierThresholdM;

        const auto evaluate_doppler = [&](double previous_mps,
                                          double current_mps,
                                          double previous_sigma_mps,
                                          double current_sigma_mps,
                                          double dt) {
            if (dt <= 0.0 || !std::isfinite(dt) ||
                previous_sigma_mps <= 0.0 || current_sigma_mps <= 0.0) {
                return;
            }
            const double predicted_delta =
                0.5 * (previous_mps + current_mps) * dt;
            classified.doppler_innovation_signed_m =
                factor.delta_carrier_m - predicted_delta;
            classified.doppler_innovation_m =
                std::abs(classified.doppler_innovation_signed_m);
            const double integrated_doppler_sigma_m =
                0.5 * dt *
                std::hypot(previous_sigma_mps, current_sigma_mps);
            classified.doppler_innovation_sigma_m =
                std::hypot(factor.sigma_m, integrated_doppler_sigma_m);
            classified.normalized_doppler_innovation =
                classified.doppler_innovation_m /
                std::max(1e-4, classified.doppler_innovation_sigma_m);
            classified.doppler_evaluated =
                std::isfinite(classified.doppler_innovation_m) &&
                std::isfinite(classified.normalized_doppler_innovation);
            classified.doppler_outlier =
                classified.doppler_evaluated &&
                classified.normalized_doppler_innovation >
                    kDopplerNormalizedInnovationThreshold;
        };
        if (factor.has_doppler_witness) {
            evaluate_doppler(factor.previous_sd_doppler_mps,
                             factor.current_sd_doppler_mps,
                             factor.previous_sd_doppler_sigma_mps,
                             factor.current_sd_doppler_sigma_mps, factor.dt_s);
        } else {
            const DopplerKey previous_key{
                factor.previous_epoch_index, factor.satellite,
                factor.reference_satellite, factor.signal};
            const DopplerKey current_key{
                factor.current_epoch_index, factor.satellite,
                factor.reference_satellite, factor.signal};
            const auto previous_doppler = sd_doppler_by_factor.find(previous_key);
            const auto current_doppler = sd_doppler_by_factor.find(current_key);
            if (previous_doppler != sd_doppler_by_factor.end() &&
                current_doppler != sd_doppler_by_factor.end()) {
                const double dt =
                    problem.epochs[factor.current_epoch_index].time -
                    problem.epochs[factor.previous_epoch_index].time;
                evaluate_doppler(previous_doppler->second.residual_mps,
                                 current_doppler->second.residual_mps,
                                 previous_doppler->second.sigma_mps,
                                 current_doppler->second.sigma_mps, dt);
            }
        }

        if (classified.doppler_evaluated && factor.arc_length_epochs >= 2) {
            const std::size_t arc_start_epoch =
                factor.current_epoch_index + 1 -
                static_cast<std::size_t>(factor.arc_length_epochs);
            const DopplerCalibrationKey key{
                arc_start_epoch, factor.satellite,
                factor.reference_satellite, factor.signal};
            auto& history = doppler_innovation_history[key];
            if (history.size() >= kDopplerCalibrationWarmup) {
                classified.doppler_bias_m = median(history);
                std::vector<double> deviations;
                deviations.reserve(history.size());
                for (const double sample : history) {
                    deviations.push_back(
                        std::abs(sample - classified.doppler_bias_m));
                }
                const double robust_sigma_m = 1.4826 * median(deviations);
                classified.doppler_calibrated_scale_m = std::max(
                    {1e-4, classified.doppler_innovation_sigma_m,
                     robust_sigma_m});
                classified.doppler_centered_innovation_m =
                    classified.doppler_innovation_signed_m -
                    classified.doppler_bias_m;
                classified.doppler_calibrated_score =
                    std::abs(classified.doppler_centered_innovation_m) /
                    classified.doppler_calibrated_scale_m;
                classified.doppler_calibration_evaluated =
                    std::isfinite(classified.doppler_calibrated_score);
                classified.doppler_calibrated_outlier =
                    classified.doppler_calibration_evaluated &&
                    classified.doppler_calibrated_score >
                        kDopplerCalibratedScoreThreshold;
            }
            history.push_back(classified.doppler_innovation_signed_m);
            if (history.size() > kDopplerCalibrationWindow) {
                history.erase(history.begin());
            }
        }

        if (geometry_free_shadow != nullptr &&
            factor.current_epoch_index < geometry_free_shadow->size()) {
            const auto& events =
                (*geometry_free_shadow)[factor.current_epoch_index]
                    .event_satellite_signals;
            classified.geometry_free_witness =
                events.count({factor.satellite, factor.signal}) != 0 ||
                events.count({factor.reference_satellite, factor.signal}) != 0;
        }
        classified.carrier_hold_witness = diagnostics.carrier_hold_active;
        if (fde_rejected_ambiguities_by_epoch != nullptr &&
            factor.current_epoch_index <
                fde_rejected_ambiguities_by_epoch->size()) {
            const auto& rejected = (*fde_rejected_ambiguities_by_epoch)
                [factor.current_epoch_index];
            classified.carrier_fde_witness =
                rejected.count(factor.target_ambiguity_index) != 0 ||
                rejected.count(factor.reference_ambiguity_index) != 0;
        }

        const bool witnessed = classified.doppler_calibrated_outlier ||
            classified.geometry_free_witness ||
            classified.carrier_hold_witness ||
            classified.carrier_fde_witness;
        if (!classified.residual_outlier) {
            classified.classification =
                TemporalCarrierShadowClassification::Clean;
            ++diagnostics.clock_resilient_tdcp_clean;
        } else if (witnessed) {
            classified.classification =
                TemporalCarrierShadowClassification::WitnessedOutlier;
            ++diagnostics.clock_resilient_tdcp_witnessed_outliers;
        } else {
            classified.classification =
                TemporalCarrierShadowClassification::UnexplainedOutlier;
            ++diagnostics.clock_resilient_tdcp_unexplained_outliers;
        }
        classified_factors.push_back(std::move(classified));
    }
    for (std::size_t i = 0; i < num_epochs; ++i) {
        const int count = epoch_diagnostics[i].clock_resilient_tdcp_factors;
        if (count > 0) {
            epoch_diagnostics[i].clock_resilient_tdcp_rms_m =
                std::sqrt(sum_sq[i] / static_cast<double>(count));
        }
    }
    return classified_factors;
}

std::vector<FGOProcessor::PredictedDdprQualityFactorDiagnostics>
FGOProcessor::analyzePredictedDdprQualityShadow(
    const FGOProblem& problem,
    const std::vector<Vector3d>& previous_solution_positions_ecef,
    const std::vector<Vector3d>& predicted_positions_ecef,
    double doppler_sigma_mps,
    double normalized_outlier_threshold,
    double max_gap_s) {
    using Key = std::tuple<SatelliteId, SatelliteId, SignalType>;
    struct History {
        const DoubleDifferencePseudorangeFactor* factor = nullptr;
        std::size_t epoch_index = 0;
        double dd_doppler_mps = 0.0;
        bool has_dd_doppler = false;
        int pair_age_epochs = 0;
    };

    const auto finite_position = [](const Vector3d& position) {
        return position.allFinite() && position.norm() > 0.0;
    };
    const auto dd_range = [](const DoubleDifferencePseudorangeFactor& factor,
                             const Vector3d& rover_position_ecef) {
        const double rover_target =
            (factor.rover_satellite_position_ecef - rover_position_ecef).norm();
        const double rover_reference =
            (factor.rover_reference_position_ecef - rover_position_ecef).norm();
        const double base_target =
            (factor.base_satellite_position_ecef - factor.base_position_ecef)
                .norm();
        const double base_reference =
            (factor.base_reference_position_ecef - factor.base_position_ecef)
                .norm();
        return (rover_target - base_target) -
               (rover_reference - base_reference);
    };
    const auto dd_doppler = [](const DoubleDifferencePseudorangeFactor& factor,
                               double& value) {
        const auto& rt = factor.rover_satellite_model;
        const auto& rr = factor.rover_reference_model;
        const auto& bt = factor.base_satellite_model;
        const auto& br = factor.base_reference_model;
        if (!rt.has_doppler_residual || !rr.has_doppler_residual ||
            !bt.has_doppler_residual || !br.has_doppler_residual) {
            return false;
        }
        value = (rt.doppler_residual_mps - bt.doppler_residual_mps) -
                (rr.doppler_residual_mps - br.doppler_residual_mps);
        return std::isfinite(value);
    };

    const std::size_t num_epochs = problem.epochs.size();
    std::vector<std::vector<const DoubleDifferencePseudorangeFactor*>>
        factors_by_epoch(num_epochs);
    for (const auto& factor : problem.double_difference_pseudorange_factors) {
        if (factor.epoch_index < num_epochs) {
            factors_by_epoch[factor.epoch_index].push_back(&factor);
        }
    }

    const double safe_doppler_sigma = std::max(1e-6, doppler_sigma_mps);
    const double safe_threshold = std::max(0.0, normalized_outlier_threshold);
    const double safe_max_gap = std::max(0.0, max_gap_s);
    std::map<Key, History> history;
    std::vector<PredictedDdprQualityFactorDiagnostics> result;
    result.reserve(problem.double_difference_pseudorange_factors.size());
    for (std::size_t epoch_index = 0; epoch_index < num_epochs; ++epoch_index) {
        std::map<Key, History> current;
        for (const auto* factor : factors_by_epoch[epoch_index]) {
            const Key key{factor->satellite, factor->reference_satellite,
                          factor->signal};
            double current_dd_doppler = 0.0;
            const bool current_has_doppler =
                dd_doppler(*factor, current_dd_doppler);
            History current_history{factor, epoch_index, current_dd_doppler,
                                    current_has_doppler, 1};
            const auto prior = history.find(key);
            if (prior == history.end()) {
                current[key] = current_history;
                continue;
            }

            const std::size_t previous_epoch_index = prior->second.epoch_index;
            const double dt = problem.epochs[epoch_index].time -
                              problem.epochs[previous_epoch_index].time;
            if (previous_epoch_index + 1 != epoch_index || dt <= 0.0 ||
                (safe_max_gap > 0.0 && dt > safe_max_gap)) {
                current[key] = current_history;
                continue;
            }

            current_history.pair_age_epochs = prior->second.pair_age_epochs + 1;
            current[key] = current_history;
            PredictedDdprQualityFactorDiagnostics diagnostic;
            diagnostic.previous_epoch_index = previous_epoch_index;
            diagnostic.current_epoch_index = epoch_index;
            diagnostic.satellite = factor->satellite;
            diagnostic.reference_satellite = factor->reference_satellite;
            diagnostic.signal = factor->signal;
            diagnostic.dt_s = dt;
            diagnostic.pair_age_epochs = current_history.pair_age_epochs;
            diagnostic.measured_ddpr_change_m =
                factor->observed_dd_pseudorange_m -
                prior->second.factor->observed_dd_pseudorange_m;
            diagnostic.elevation_rad = std::min(
                factor->rover_satellite_model.elevation_rad,
                factor->rover_reference_model.elevation_rad);
            diagnostic.target_snr_dbhz =
                factor->rover_satellite_model.snr_dbhz;
            diagnostic.reference_snr_dbhz =
                factor->rover_reference_model.snr_dbhz;

            if (current_has_doppler && prior->second.has_dd_doppler) {
                diagnostic.doppler_evaluated = true;
                diagnostic.doppler_predicted_change_m =
                    0.5 * (prior->second.dd_doppler_mps +
                           current_dd_doppler) *
                    dt;
                diagnostic.doppler_innovation_m =
                    diagnostic.measured_ddpr_change_m -
                    diagnostic.doppler_predicted_change_m;
                const double measured_sigma = std::hypot(
                    prior->second.factor->sigma_m, factor->sigma_m);
                // Four receiver/satellite Doppler terms form each DD. The
                // trapezoidal difference uses two epochs, yielding
                // sqrt(2)*dt*sigma for equal independent link sigmas.
                const double integrated_doppler_sigma =
                    std::sqrt(2.0) * dt * safe_doppler_sigma;
                diagnostic.doppler_innovation_sigma_m =
                    std::hypot(measured_sigma, integrated_doppler_sigma);
                diagnostic.normalized_doppler_innovation =
                    std::abs(diagnostic.doppler_innovation_m) /
                    diagnostic.doppler_innovation_sigma_m;
            }

            if (previous_epoch_index <
                    previous_solution_positions_ecef.size() &&
                epoch_index < predicted_positions_ecef.size() &&
                finite_position(previous_solution_positions_ecef[
                    previous_epoch_index]) &&
                finite_position(predicted_positions_ecef[epoch_index])) {
                diagnostic.imu_geometry_evaluated = true;
                diagnostic.imu_predicted_change_m =
                    dd_range(*factor, predicted_positions_ecef[epoch_index]) -
                    dd_range(*prior->second.factor,
                             previous_solution_positions_ecef[
                                 previous_epoch_index]);
                diagnostic.previous_predicted_ddpr_residual_m =
                    prior->second.factor->observed_dd_pseudorange_m -
                    dd_range(*prior->second.factor,
                             previous_solution_positions_ecef[
                                 previous_epoch_index]);
                diagnostic.current_predicted_ddpr_residual_m =
                    factor->observed_dd_pseudorange_m -
                    dd_range(*factor,
                             predicted_positions_ecef[epoch_index]);
                diagnostic.imu_innovation_m =
                    diagnostic.measured_ddpr_change_m -
                    diagnostic.imu_predicted_change_m;
                diagnostic.imu_innovation_sigma_m = std::hypot(
                    prior->second.factor->sigma_m, factor->sigma_m);
                diagnostic.normalized_imu_innovation =
                    std::abs(diagnostic.imu_innovation_m) /
                    std::max(1e-6, diagnostic.imu_innovation_sigma_m);
            }

            if (!diagnostic.doppler_evaluated &&
                !diagnostic.imu_geometry_evaluated) {
                diagnostic.proposed_action =
                    PredictedDdprQualityAction::Unavailable;
            } else {
                diagnostic.proposed_action = PredictedDdprQualityAction::Keep;
                if (diagnostic.doppler_evaluated &&
                    diagnostic.imu_geometry_evaluated &&
                    diagnostic.normalized_doppler_innovation > safe_threshold &&
                    diagnostic.normalized_imu_innovation > safe_threshold) {
                    diagnostic.proposed_action =
                        PredictedDdprQualityAction::Downweight;
                }
            }
            result.push_back(diagnostic);
        }
        history = std::move(current);
    }
    return result;
}

std::vector<FGOProcessor::PredictedDdprBiasStateDiagnostics>
FGOProcessor::analyzePredictedDdprBiasStateShadow(
    const std::vector<PredictedDdprQualityFactorDiagnostics>& quality_rows,
    double process_noise_m_sqrt_s,
    double initial_sigma_m,
    double min_measurement_sigma_m,
    double robust_update_sigma,
    int min_prior_updates) {
    using Key = std::tuple<SatelliteId, SatelliteId, SignalType>;
    struct State {
        double mean_m = 0.0;
        double variance_m2 = 0.0;
        int updates = 0;
        std::size_t last_epoch_index = 0;
        bool initialized = false;
    };

    const double q = std::max(0.0, process_noise_m_sqrt_s);
    const double initial_sigma = std::max(1e-6, initial_sigma_m);
    const double min_measurement_sigma =
        std::max(1e-6, min_measurement_sigma_m);
    const double clip_sigma = std::max(0.0, robust_update_sigma);
    const int burn_in = std::max(0, min_prior_updates);
    std::map<Key, State> states;
    std::vector<PredictedDdprBiasStateDiagnostics> result;
    result.reserve(quality_rows.size());

    for (const auto& row : quality_rows) {
        PredictedDdprBiasStateDiagnostics diagnostic;
        diagnostic.previous_epoch_index = row.previous_epoch_index;
        diagnostic.current_epoch_index = row.current_epoch_index;
        diagnostic.satellite = row.satellite;
        diagnostic.reference_satellite = row.reference_satellite;
        diagnostic.signal = row.signal;
        diagnostic.dt_s = row.dt_s;
        diagnostic.pair_age_epochs = row.pair_age_epochs;
        diagnostic.raw_residual_m =
            row.current_predicted_ddpr_residual_m;

        const Key key{row.satellite, row.reference_satellite, row.signal};
        auto& state = states[key];
        const bool continuous = state.initialized &&
                                state.last_epoch_index ==
                                    row.previous_epoch_index &&
                                row.current_epoch_index ==
                                    row.previous_epoch_index + 1 &&
                                std::isfinite(row.dt_s) && row.dt_s > 0.0;
        if (!continuous) {
            state = State{};
            state.variance_m2 = initial_sigma * initial_sigma;
            diagnostic.continuity_reset = true;
        } else {
            state.variance_m2 += q * q * row.dt_s;
        }

        diagnostic.prior_updates = state.updates;
        diagnostic.prior_bias_m = state.mean_m;
        diagnostic.prior_sigma_m =
            std::sqrt(std::max(0.0, state.variance_m2));
        diagnostic.corrected_residual_m =
            diagnostic.raw_residual_m - diagnostic.prior_bias_m;
        diagnostic.prediction_usable =
            continuous && state.updates >= burn_in &&
            row.imu_geometry_evaluated &&
            std::isfinite(diagnostic.raw_residual_m);

        // The quality monitor propagates the two adjacent DDPR sigmas with
        // hypot(). Divide by sqrt(2) to obtain an equal-row approximation;
        // the floor deliberately prevents an overconfident shadow state.
        const double approximated_row_sigma =
            row.imu_innovation_sigma_m / std::sqrt(2.0);
        diagnostic.measurement_sigma_m = std::max(
            min_measurement_sigma,
            std::isfinite(approximated_row_sigma)
                ? approximated_row_sigma
                : min_measurement_sigma);

        const bool update_valid =
            row.imu_geometry_evaluated &&
            std::isfinite(diagnostic.raw_residual_m) &&
            std::isfinite(state.mean_m) &&
            std::isfinite(state.variance_m2);
        if (update_valid) {
            const double measurement_variance =
                diagnostic.measurement_sigma_m *
                diagnostic.measurement_sigma_m;
            const double innovation_m =
                diagnostic.raw_residual_m - state.mean_m;
            diagnostic.innovation_sigma_m = std::sqrt(std::max(
                1e-12, state.variance_m2 + measurement_variance));
            diagnostic.normalized_innovation =
                std::abs(innovation_m) / diagnostic.innovation_sigma_m;
            diagnostic.applied_innovation_m = innovation_m;
            if (clip_sigma > 0.0) {
                const double limit =
                    clip_sigma * diagnostic.innovation_sigma_m;
                if (std::abs(diagnostic.applied_innovation_m) > limit) {
                    diagnostic.applied_innovation_m =
                        std::copysign(limit,
                                      diagnostic.applied_innovation_m);
                    diagnostic.update_clipped = true;
                }
            }
            const double kalman_gain =
                state.variance_m2 /
                (state.variance_m2 + measurement_variance);
            state.mean_m +=
                kalman_gain * diagnostic.applied_innovation_m;
            state.variance_m2 =
                std::max(0.0, (1.0 - kalman_gain) * state.variance_m2);
            ++state.updates;
            state.last_epoch_index = row.current_epoch_index;
            state.initialized = true;
            diagnostic.update_applied = true;
        } else {
            // Invalid geometry breaks the causal chain. A later valid row
            // starts from the configured prior instead of stale state.
            states.erase(key);
        }

        if (diagnostic.update_applied) {
            diagnostic.posterior_bias_m = state.mean_m;
            diagnostic.posterior_sigma_m =
                std::sqrt(std::max(0.0, state.variance_m2));
        }
        result.push_back(diagnostic);
    }
    return result;
}

}  // namespace libgnss
