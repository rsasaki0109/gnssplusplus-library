#include <libgnss++/algorithms/ppp_ar.hpp>

#include <libgnss++/algorithms/lambda.hpp>
#include <libgnss++/core/coordinates.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <utility>
#include <vector>

namespace libgnss::ppp_ar {

constexpr std::array<double, 60> kClaslibRatioThresholdsAlpha10 = {
    39.86,9.00,5.39,4.11,3.45,3.05,2.78,2.59,2.44,2.32,
    2.23,2.15,2.08,2.02,1.97,1.93,1.89,1.85,1.82,1.79,
    1.77,1.74,1.72,1.70,1.68,1.67,1.65,1.63,1.62,1.61,
    1.59,1.58,1.57,1.56,1.55,1.54,1.53,1.52,1.51,1.51,
    1.50,1.49,1.48,1.48,1.47,1.46,1.46,1.45,1.45,1.44,
    1.44,1.43,1.43,1.42,1.42,1.41,1.41,1.40,1.40,1.40
};

SatelliteId clasRealSatellite(const SatelliteId& satellite) {
    if (satellite.prn > 100) {
        return SatelliteId(
            satellite.system,
            static_cast<uint8_t>(std::max(1, static_cast<int>(satellite.prn) - 100)));
    }
    return satellite;
}

std::pair<GNSSSystem, int> ambiguityDdGroup(const SatelliteId& satellite) {
    return {satellite.system, satellite.prn > 100 ? 1 : 0};
}

int receiverClockStateIndex(
    const ppp_shared::PPPState& filter_state,
    const SatelliteId& satellite) {
    switch (satellite.system) {
        case GNSSSystem::GLONASS:
            return filter_state.glo_clock_index;
        case GNSSSystem::Galileo:
            return filter_state.gal_clock_index >= 0
                ? filter_state.gal_clock_index : filter_state.clock_index;
        case GNSSSystem::QZSS:
            return filter_state.qzs_clock_index >= 0
                ? filter_state.qzs_clock_index : filter_state.clock_index;
        case GNSSSystem::BeiDou:
            return filter_state.bds_clock_index >= 0
                ? filter_state.bds_clock_index : filter_state.clock_index;
        default:
            return filter_state.clock_index;
    }
}

double claslibRatioThresholdForNb(int nb) {
    if (nb <= 0) {
        return std::numeric_limits<double>::infinity();
    }
    const size_t index =
        static_cast<size_t>(std::min(nb, static_cast<int>(kClaslibRatioThresholdsAlpha10.size())) - 1);
    return kClaslibRatioThresholdsAlpha10[index];
}

double safeVarianceFloor(double variance, double floor_value) {
    if (!std::isfinite(variance) || variance < floor_value) {
        return floor_value;
    }
    return variance;
}

double safeCorrelation(const MatrixXd& covariance, int i, int j) {
    if (i < 0 || j < 0 ||
        i >= covariance.rows() || j >= covariance.cols()) {
        return 0.0;
    }
    const double var_i = safeVarianceFloor(covariance(i, i), 1e-12);
    const double var_j = safeVarianceFloor(covariance(j, j), 1e-12);
    const double denom = std::sqrt(var_i * var_j);
    if (!std::isfinite(denom) || denom <= 0.0) {
        return 0.0;
    }
    return covariance(i, j) / denom;
}

double positionCorrelationNorm(
    const MatrixXd& covariance,
    const ppp_shared::PPPState& filter_state,
    int ambiguity_index) {
    if (ambiguity_index < 0 ||
        ambiguity_index >= covariance.rows() ||
        filter_state.pos_index < 0 ||
        filter_state.pos_index + 2 >= covariance.rows()) {
        return 0.0;
    }
    double corr_sq_sum = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
        const double corr =
            safeCorrelation(covariance, ambiguity_index, filter_state.pos_index + axis);
        corr_sq_sum += corr * corr;
    }
    return std::sqrt(corr_sq_sum);
}

struct DdPair {
    int ref_idx = -1;
    int sat_idx = -1;
};

bool shouldDumpStrictFirstAr(
    const ppp_shared::PPPConfig& config) {
    return config.wlnl_strict_claslib_parity &&
           config.ar_method == ppp_shared::PPPConfig::ARMethod::DD_PER_FREQ &&
           !config.strict_first_ar_dump_path.empty() &&
           !std::filesystem::exists(config.strict_first_ar_dump_path);
}

void writeStrictFirstArDump(
    const ppp_shared::PPPConfig& config,
    const ppp_shared::PPPState& filter_state,
    const MatrixXd& pre_anchor_covariance,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const std::vector<SatelliteId>& satellites,
    const std::vector<int>& state_indices,
    const std::vector<double>& scales,
    const std::vector<int>& active_indices,
    const std::map<std::pair<GNSSSystem, int>, int>& system_ref_map,
    const std::vector<DdPair>& dd_pairs,
    const VectorXd& dd_float,
    const MatrixXd& dd_cov,
    const VectorXd& dd_fixed,
    bool lambda_ok,
    double ratio,
    double required_ratio,
    const GNSSTime* observation_time) {
    std::ofstream output(config.strict_first_ar_dump_path, std::ios::trunc);
    if (!output) {
        std::cerr << "[PPP-AR-DUMP] failed path=" << config.strict_first_ar_dump_path << "\n";
        return;
    }

    auto covarianceCyclesFromMatrix = [&](const MatrixXd& covariance, int a, int b) {
        return covariance(
            state_indices[static_cast<size_t>(a)],
            state_indices[static_cast<size_t>(b)]) /
            (scales[static_cast<size_t>(a)] * scales[static_cast<size_t>(b)]);
    };

    struct CorrelationEntry {
        int lhs = -1;
        int rhs = -1;
        double corr = 0.0;
    };

    std::vector<CorrelationEntry> top_correlations;
    const Eigen::Index correlation_capacity =
        std::max<Eigen::Index>(0, dd_cov.rows() * (dd_cov.rows() - 1) / 2);
    top_correlations.reserve(static_cast<size_t>(correlation_capacity));
    for (int i = 0; i < dd_cov.rows(); ++i) {
        for (int j = i + 1; j < dd_cov.cols(); ++j) {
            top_correlations.push_back({i, j, safeCorrelation(dd_cov, i, j)});
        }
    }
    std::sort(top_correlations.begin(),
              top_correlations.end(),
              [](const CorrelationEntry& lhs, const CorrelationEntry& rhs) {
                  return std::abs(lhs.corr) > std::abs(rhs.corr);
              });
    if (top_correlations.size() > 12) {
        top_correlations.resize(12);
    }

    output << std::fixed << std::setprecision(12);
    output << "strict_first_ar_dump=1\n";
    if (observation_time != nullptr) {
        output << "time_week=" << observation_time->week << "\n";
        output << "time_tow=" << observation_time->tow << "\n";
    }
    output << "candidate_count=" << active_indices.size() << "\n";
    output << "nb=" << dd_pairs.size() << "\n";
    output << "lambda_ok=" << (lambda_ok ? 1 : 0) << "\n";
    output << "ratio=" << ratio << "\n";
    output << "required_ratio=" << required_ratio << "\n";
    if (lambda_ok && dd_fixed.size() == dd_float.size() &&
        dd_cov.rows() == dd_float.size() && dd_cov.cols() == dd_float.size()) {
        Eigen::LDLT<MatrixXd> dd_ldlt(dd_cov);
        if (dd_ldlt.info() == Eigen::Success) {
            const VectorXd residual = dd_float - dd_fixed;
            const VectorXd weighted = dd_ldlt.solve(residual);
            if (weighted.allFinite()) {
                const double norm0 = residual.dot(weighted);
                const double norm1 = ratio * norm0;
                output << "lambda_norm0=" << norm0 << "\n";
                output << "lambda_norm1=" << norm1 << "\n";
            }
        }
    }
    output << "[phase_states]\n";
    for (size_t order = 0; order < active_indices.size(); ++order) {
        const int candidate_index = active_indices[order];
        const auto ambiguity_satellite =
            satellites[static_cast<size_t>(candidate_index)];
        const auto ambiguity_it = ambiguity_states.find(ambiguity_satellite);
        const int lock_count =
            ambiguity_it != ambiguity_states.end() ? ambiguity_it->second.lock_count : -1;
        const int state_index = state_indices[static_cast<size_t>(candidate_index)];
        output << "i=" << order
               << " sat=" << ambiguity_satellite.toString()
               << " real=" << clasRealSatellite(ambiguity_satellite).toString()
               << " freq_group=" << ambiguityDdGroup(ambiguity_satellite).second
               << " state_idx=" << state_index
               << " x=" << filter_state.state(state_index)
               << " y=" << filter_state.state(state_index) /
                    scales[static_cast<size_t>(candidate_index)]
               << " pdiag=" << filter_state.covariance(state_index, state_index)
               << " qdiag_cycles=" << covarianceCyclesFromMatrix(
                       filter_state.covariance, candidate_index, candidate_index)
               << " lock=" << lock_count
               << "\n";
    }
    if (!filter_state.ionosphere_indices.empty()) {
        output << "[iono_states]\n";
        std::vector<std::pair<SatelliteId, int>> iono_states(
            filter_state.ionosphere_indices.begin(),
            filter_state.ionosphere_indices.end());
        std::sort(iono_states.begin(), iono_states.end(),
                  [](const auto& lhs, const auto& rhs) {
                      if (lhs.first.system != rhs.first.system) {
                          return lhs.first.system < rhs.first.system;
                      }
                      return lhs.first.prn < rhs.first.prn;
                  });
        for (const auto& [satellite, state_index] : iono_states) {
            if (state_index < 0 || state_index >= filter_state.total_states) {
                continue;
            }
            output << "sat=" << satellite.toString()
                   << " state_idx=" << state_index
                   << " x=" << filter_state.state(state_index)
                   << " pdiag=" << filter_state.covariance(state_index, state_index)
                   << "\n";
        }
    }
    output << "[eligible_y_na_cycles]\n";
    for (size_t order = 0; order < active_indices.size(); ++order) {
        const int candidate_index = active_indices[order];
        const auto ambiguity_it =
            ambiguity_states.find(satellites[static_cast<size_t>(candidate_index)]);
        const int lock_count =
            ambiguity_it != ambiguity_states.end() ? ambiguity_it->second.lock_count : -1;
        output << "i=" << order
               << " sat=" << satellites[static_cast<size_t>(candidate_index)].toString()
               << " real=" << clasRealSatellite(satellites[static_cast<size_t>(candidate_index)]).toString()
               << " state_idx=" << state_indices[static_cast<size_t>(candidate_index)]
               << " y=" << filter_state.state(state_indices[static_cast<size_t>(candidate_index)]) /
                    scales[static_cast<size_t>(candidate_index)]
               << " scale=" << scales[static_cast<size_t>(candidate_index)]
               << " lock=" << lock_count
               << " group_sys=" << static_cast<int>(
                      ambiguityDdGroup(satellites[static_cast<size_t>(candidate_index)]).first)
               << " group_freq=" << ambiguityDdGroup(satellites[static_cast<size_t>(candidate_index)]).second
               << "\n";
    }
    output << "[references]\n";
    for (const auto& [group, ref_idx] : system_ref_map) {
        output << "sys=" << static_cast<int>(group.first)
               << " freq_group=" << group.second
               << " ref=" << satellites[static_cast<size_t>(ref_idx)].toString()
               << "\n";
    }
    output << "[dd_pairs]\n";
    for (size_t k = 0; k < dd_pairs.size(); ++k) {
        const int ri = dd_pairs[k].ref_idx;
        const int si = dd_pairs[k].sat_idx;
        double qdiag_pre = dd_cov(static_cast<int>(k), static_cast<int>(k));
        if (pre_anchor_covariance.rows() == filter_state.covariance.rows() &&
            pre_anchor_covariance.cols() == filter_state.covariance.cols()) {
            qdiag_pre =
                covarianceCyclesFromMatrix(pre_anchor_covariance, ri, ri) -
                covarianceCyclesFromMatrix(pre_anchor_covariance, ri, si) -
                covarianceCyclesFromMatrix(pre_anchor_covariance, si, ri) +
                covarianceCyclesFromMatrix(pre_anchor_covariance, si, si);
        }
        output << "k=" << k
               << " ref=" << satellites[static_cast<size_t>(ri)].toString()
               << " sat=" << satellites[static_cast<size_t>(si)].toString()
               << " dd_float=" << dd_float(static_cast<int>(k))
               << " dd_fixed=" << (lambda_ok ? dd_fixed(static_cast<int>(k)) : 0.0)
               << " frac=" << dd_float(static_cast<int>(k)) - std::round(dd_float(static_cast<int>(k)))
               << " qdiag=" << dd_cov(static_cast<int>(k), static_cast<int>(k))
               << " qdiag_pre=" << qdiag_pre
               << "\n";
    }
    output << "[lambda_Qa]\n";
    for (int row = 0; row < dd_cov.rows(); ++row) {
        output << "row=" << row;
        for (int col = 0; col < dd_cov.cols(); ++col) {
            output << " q" << col << "=" << dd_cov(row, col);
        }
        output << "\n";
    }
    output << "[dd_cov_top_correlations]\n";
    for (const auto& entry : top_correlations) {
        output << "i=" << entry.lhs
               << " j=" << entry.rhs
               << " corr=" << entry.corr
               << " pair_i=" << satellites[static_cast<size_t>(dd_pairs[static_cast<size_t>(entry.lhs)].ref_idx)].toString()
               << "->" << satellites[static_cast<size_t>(dd_pairs[static_cast<size_t>(entry.lhs)].sat_idx)].toString()
               << " pair_j=" << satellites[static_cast<size_t>(dd_pairs[static_cast<size_t>(entry.rhs)].ref_idx)].toString()
               << "->" << satellites[static_cast<size_t>(dd_pairs[static_cast<size_t>(entry.rhs)].sat_idx)].toString()
               << "\n";
    }
}

EligibleAmbiguities collectEligibleAmbiguities(
    const ppp_shared::PPPState& filter_state,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    int min_lock_count,
    const GNSSTime* observation_time) {
    EligibleAmbiguities eligible;
    for (const auto& [satellite, state_index] : filter_state.ambiguity_indices) {
        ++eligible.total_ambiguities;
        const auto ambiguity_it = ambiguity_states.find(satellite);
        if (ambiguity_it == ambiguity_states.end()) {
            continue;
        }
        const auto& ambiguity = ambiguity_it->second;
        if (ambiguity.needs_reinitialization || ambiguity.lock_count < min_lock_count) {
            if (ambiguity.needs_reinitialization) {
                ++eligible.skipped_reinitialization;
            } else {
                ++eligible.skipped_lock;
            }
            continue;
        }
        if (observation_time != nullptr &&
            std::abs(ambiguity.last_time - *observation_time) > 1e-3) {
            ++eligible.skipped_stale;
            continue;
        }
        if (!std::isfinite(ambiguity.ambiguity_scale_m) || ambiguity.ambiguity_scale_m <= 0.0) {
            ++eligible.skipped_scale;
            continue;
        }
        if (state_index < filter_state.amb_index || state_index >= filter_state.total_states) {
            ++eligible.skipped_index;
            continue;
        }
        eligible.satellites.push_back(satellite);
        eligible.state_indices.push_back(state_index);
        eligible.scales.push_back(ambiguity.ambiguity_scale_m);
    }
    return eligible;
}

WlnlWideLaneFixSummary applyWideLaneFixes(
    const ppp_shared::PPPConfig& config,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const std::vector<SatelliteId>& satellites,
    bool debug_enabled) {
    WlnlWideLaneFixSummary summary;
    constexpr double kWideLaneFractionThreshold = 0.30;

    struct WlCandidate {
        SatelliteId satellite;
        double mw_mean_cycles = 0.0;
        int mw_count = 0;
        int rounded_integer = 0;
        double fractional_cycles = 0.0;
    };

    std::map<GNSSSystem, std::vector<WlCandidate>> candidates_by_system;

    for (const auto& satellite : satellites) {
        auto ambiguity_it = ambiguity_states.find(satellite);
        if (ambiguity_it == ambiguity_states.end()) {
            continue;
        }
        auto& ambiguity = ambiguity_it->second;
        summary.max_mw_count = std::max(summary.max_mw_count, ambiguity.mw_count);
        if (ambiguity.mw_count < config.wl_min_averaging_epochs ||
            !std::isfinite(ambiguity.mw_mean_cycles)) {
            if (ambiguity.wl_is_fixed) {
                ++summary.fixed_count;
            }
            continue;
        }

        const double mw_mean = ambiguity.mw_mean_cycles;
        const int wl_int = static_cast<int>(std::round(mw_mean));
        const double frac = mw_mean - wl_int;
        candidates_by_system[satellite.system].push_back(
            WlCandidate{satellite, mw_mean, ambiguity.mw_count, wl_int, frac});

        if (ambiguity.wl_is_fixed) {
            ++summary.fixed_count;
            continue;
        }
        if (std::abs(frac) >= kWideLaneFractionThreshold) {
            if (debug_enabled) {
                std::cerr << "[PPP-WLNL] WL hold "
                          << satellite.toString()
                          << " mw_mean=" << mw_mean
                          << " int=" << wl_int
                          << " frac=" << frac
                          << " count=" << ambiguity.mw_count << "\n";
            }
            continue;
        }

        ambiguity.wl_fixed_integer = wl_int;
        ambiguity.wl_is_fixed = true;
        ++summary.fixed_count;
        if (debug_enabled) {
            std::cerr << "[PPP-WLNL] WL fix "
                      << satellite.toString()
                      << " mw_mean=" << mw_mean
                      << " int=" << wl_int
                      << " frac=" << frac
                      << " count=" << ambiguity.mw_count << "\n";
        }
    }

    for (const auto& [system, candidates] : candidates_by_system) {
        if (candidates.size() < 2) {
            continue;
        }

        std::vector<std::vector<std::pair<int, int>>> adjacency(candidates.size());
        for (size_t i = 0; i < candidates.size(); ++i) {
            for (size_t j = i + 1; j < candidates.size(); ++j) {
                const double dd_wl = candidates[i].mw_mean_cycles - candidates[j].mw_mean_cycles;
                const int dd_int = static_cast<int>(std::round(dd_wl));
                const double dd_frac = dd_wl - dd_int;
                if (std::abs(dd_frac) >= kWideLaneFractionThreshold) {
                    continue;
                }
                adjacency[i].push_back({static_cast<int>(j), dd_int});
                adjacency[j].push_back({static_cast<int>(i), -dd_int});
            }
        }

        std::vector<bool> component_seen(candidates.size(), false);
        for (size_t start = 0; start < candidates.size(); ++start) {
            if (component_seen[start] || adjacency[start].empty()) {
                continue;
            }

            std::vector<int> component;
            std::vector<int> stack{static_cast<int>(start)};
            component_seen[start] = true;
            while (!stack.empty()) {
                const int current = stack.back();
                stack.pop_back();
                component.push_back(current);
                for (const auto& [neighbor, _] : adjacency[static_cast<size_t>(current)]) {
                    if (component_seen[static_cast<size_t>(neighbor)]) {
                        continue;
                    }
                    component_seen[static_cast<size_t>(neighbor)] = true;
                    stack.push_back(neighbor);
                }
            }
            if (component.size() < 2) {
                continue;
            }

            int root = component.front();
            bool root_is_existing_fix = false;
            for (const int index : component) {
                const auto& ambiguity =
                    ambiguity_states.at(candidates[static_cast<size_t>(index)].satellite);
                if (!ambiguity.wl_is_fixed) {
                    continue;
                }
                if (!root_is_existing_fix) {
                    root = index;
                    root_is_existing_fix = true;
                    continue;
                }
                const auto& current_root =
                    ambiguity_states.at(candidates[static_cast<size_t>(root)].satellite);
                if (ambiguity.lock_count > current_root.lock_count) {
                    root = index;
                }
            }
            if (!root_is_existing_fix) {
                root = *std::min_element(
                    component.begin(),
                    component.end(),
                    [&](int lhs, int rhs) {
                        const auto& left = candidates[static_cast<size_t>(lhs)];
                        const auto& right = candidates[static_cast<size_t>(rhs)];
                        const double left_frac = std::abs(left.fractional_cycles);
                        const double right_frac = std::abs(right.fractional_cycles);
                        if (left_frac != right_frac) {
                            return left_frac < right_frac;
                        }
                        return left.mw_count > right.mw_count;
                    });
            }

            std::map<int, int> assigned_integers;
            auto& root_ambiguity = ambiguity_states[candidates[static_cast<size_t>(root)].satellite];
            const int root_integer = root_ambiguity.wl_is_fixed
                ? root_ambiguity.wl_fixed_integer
                : candidates[static_cast<size_t>(root)].rounded_integer;
            assigned_integers[root] = root_integer;
            if (!root_ambiguity.wl_is_fixed) {
                root_ambiguity.wl_fixed_integer = root_integer;
                root_ambiguity.wl_is_fixed = true;
                ++summary.fixed_count;
                if (debug_enabled) {
                    std::cerr << "[PPP-WLNL] WL pair anchor "
                              << candidates[static_cast<size_t>(root)].satellite.toString()
                              << " int=" << root_integer
                              << " frac=" << candidates[static_cast<size_t>(root)].fractional_cycles
                              << " count=" << candidates[static_cast<size_t>(root)].mw_count << "\n";
                }
            }

            std::vector<int> queue{root};
            size_t queue_index = 0;
            while (queue_index < queue.size()) {
                const int current = queue[queue_index++];
                const int current_integer = assigned_integers[current];
                for (const auto& [neighbor, dd_int] : adjacency[static_cast<size_t>(current)]) {
                    if (assigned_integers.find(neighbor) != assigned_integers.end()) {
                        continue;
                    }
                    const int neighbor_integer = current_integer - dd_int;
                    assigned_integers[neighbor] = neighbor_integer;
                    queue.push_back(neighbor);

                    auto& neighbor_ambiguity =
                        ambiguity_states[candidates[static_cast<size_t>(neighbor)].satellite];
                    const bool was_fixed = neighbor_ambiguity.wl_is_fixed;
                    neighbor_ambiguity.wl_fixed_integer = neighbor_integer;
                    neighbor_ambiguity.wl_is_fixed = true;
                    if (!was_fixed) {
                        ++summary.fixed_count;
                    }
                    if (debug_enabled && !was_fixed) {
                        const double dd_wl =
                            candidates[static_cast<size_t>(current)].mw_mean_cycles -
                            candidates[static_cast<size_t>(neighbor)].mw_mean_cycles;
                        const double dd_frac = dd_wl - dd_int;
                        std::cerr << "[PPP-WLNL] WL pair fix "
                                  << candidates[static_cast<size_t>(neighbor)].satellite.toString()
                                  << " ref="
                                  << candidates[static_cast<size_t>(current)].satellite.toString()
                                  << " dd=" << dd_wl
                                  << " dd_int=" << dd_int
                                  << " dd_frac=" << dd_frac
                                  << " int=" << neighbor_integer
                                  << " count=" << candidates[static_cast<size_t>(neighbor)].mw_count
                                  << "\n";
                    }
                }
            }
        }
        (void)system;
    }
    return summary;
}

WlnlPreparation prepareWlnlCandidates(
    const ppp_shared::PPPConfig& config,
    const ppp_shared::PPPState& filter_state,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    bool use_ssr_products,
    bool debug_enabled) {
    WlnlPreparation preparation;
    preparation.min_lock_count = use_ssr_products
        ? std::max(1, config.wl_min_averaging_epochs)
        : config.convergence_min_epochs;
    preparation.eligible_ambiguities = collectEligibleAmbiguities(
        filter_state, ambiguity_states, preparation.min_lock_count);
    preparation.wl_summary = applyWideLaneFixes(
        config, ambiguity_states, preparation.eligible_ambiguities.satellites, debug_enabled);
    return preparation;
}

DdFixAttempt tryDirectDdFix(
    const ppp_shared::PPPConfig& config,
    const ppp_shared::PPPState& filter_state,
    const MatrixXd& pre_anchor_covariance,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const std::vector<SatelliteId>& satellites,
    const std::vector<int>& state_indices,
    const std::vector<double>& scales,
    const std::set<SatelliteId>& excluded_real_satellites,
    bool debug_enabled,
    const GNSSTime* observation_time) {
    DdFixAttempt attempt;

    if (debug_enabled) {
        for (size_t i = 0; i < satellites.size(); ++i) {
            const auto ambiguity_it =
                ambiguity_states.find(satellites[static_cast<size_t>(i)]);
            const int lock_count =
                ambiguity_it != ambiguity_states.end() ? ambiguity_it->second.lock_count : -1;
            std::cerr << "[PPP-AR-CAND] sat=" << satellites[static_cast<size_t>(i)].toString()
                      << " real=" << clasRealSatellite(satellites[static_cast<size_t>(i)]).toString()
                      << " state_idx=" << state_indices[static_cast<size_t>(i)]
                      << " scale=" << scales[static_cast<size_t>(i)]
                      << " lock=" << lock_count
                      << " group_sys="
                      << static_cast<int>(
                             ambiguityDdGroup(satellites[static_cast<size_t>(i)]).first)
                      << " group_freq="
                      << ambiguityDdGroup(satellites[static_cast<size_t>(i)]).second
                      << "\n";
        }
    }

    std::vector<int> active_indices;
    active_indices.reserve(satellites.size());
    for (int i = 0; i < static_cast<int>(satellites.size()); ++i) {
        if (excluded_real_satellites.count(clasRealSatellite(satellites[static_cast<size_t>(i)])) == 0) {
            active_indices.push_back(i);
        }
    }

    if (static_cast<int>(active_indices.size()) < config.min_satellites_for_ar) {
        return attempt;
    }

    std::vector<DdPair> dd_pairs;
    std::map<std::pair<GNSSSystem, int>, int> system_ref_map;
    for (const int candidate_index : active_indices) {
        const auto group = ambiguityDdGroup(satellites[static_cast<size_t>(candidate_index)]);
        if (system_ref_map.find(group) == system_ref_map.end()) {
            system_ref_map[group] = candidate_index;
        }
    }

    for (const int candidate_index : active_indices) {
        const auto group = ambiguityDdGroup(satellites[static_cast<size_t>(candidate_index)]);
        const int ref_index = system_ref_map[group];
        if (ref_index == candidate_index) {
            continue;
        }
        dd_pairs.push_back({ref_index, candidate_index});
    }

    if (debug_enabled) {
        for (const auto& [group, ref_idx] : system_ref_map) {
            std::cerr << "[PPP-AR-REF] sys=" << static_cast<int>(group.first)
                      << " freq_group=" << group.second
                      << " ref=" << satellites[static_cast<size_t>(ref_idx)].toString()
                      << " lock="
                      << ambiguity_states.at(satellites[static_cast<size_t>(ref_idx)]).lock_count
                      << "\n";
        }
    }

    attempt.nb = static_cast<int>(dd_pairs.size());
    if (debug_enabled) {
        std::cerr << "[PPP-AR-DDSET] candidates=" << active_indices.size()
                  << " nb=" << attempt.nb
                  << " excluded_real=" << excluded_real_satellites.size()
                  << "\n";
    }
    if (attempt.nb < config.min_satellites_for_ar) {
        return attempt;
    }

    VectorXd dd_float = VectorXd::Zero(attempt.nb);
    MatrixXd dd_cov = MatrixXd::Zero(attempt.nb, attempt.nb);

    for (int k = 0; k < attempt.nb; ++k) {
        const int ri = dd_pairs[static_cast<size_t>(k)].ref_idx;
        const int si = dd_pairs[static_cast<size_t>(k)].sat_idx;
        const double ref_cycles =
            filter_state.state(state_indices[static_cast<size_t>(ri)]) /
            scales[static_cast<size_t>(ri)];
        const double sat_cycles =
            filter_state.state(state_indices[static_cast<size_t>(si)]) /
            scales[static_cast<size_t>(si)];
        dd_float(k) = ref_cycles - sat_cycles;

        for (int l = 0; l < attempt.nb; ++l) {
            const int rj = dd_pairs[static_cast<size_t>(l)].ref_idx;
            const int sj = dd_pairs[static_cast<size_t>(l)].sat_idx;
            auto covarianceCycles = [&](int a, int b) {
                return filter_state.covariance(
                    state_indices[static_cast<size_t>(a)],
                    state_indices[static_cast<size_t>(b)]) /
                    (scales[static_cast<size_t>(a)] * scales[static_cast<size_t>(b)]);
            };
            dd_cov(k, l) =
                covarianceCycles(ri, rj) - covarianceCycles(ri, sj) -
                covarianceCycles(si, rj) + covarianceCycles(si, sj);
        }
    }

    if (debug_enabled) {
        const bool have_pre_anchor =
            pre_anchor_covariance.rows() == filter_state.covariance.rows() &&
            pre_anchor_covariance.cols() == filter_state.covariance.cols();
        const MatrixXd& debug_cross_cov =
            have_pre_anchor ? pre_anchor_covariance : filter_state.covariance;
        auto covarianceCyclesFromMatrix = [&](const MatrixXd& covariance, int a, int b) {
            return covariance(
                state_indices[static_cast<size_t>(a)],
                state_indices[static_cast<size_t>(b)]) /
                (scales[static_cast<size_t>(a)] * scales[static_cast<size_t>(b)]);
        };
        const int dump_count = std::min(attempt.nb, 8);
        for (int k = 0; k < dump_count; ++k) {
            const int ri = dd_pairs[static_cast<size_t>(k)].ref_idx;
            const int si = dd_pairs[static_cast<size_t>(k)].sat_idx;
            const double frac = dd_float(k) - std::round(dd_float(k));
            const double sigma = std::sqrt(safeVarianceFloor(dd_cov(k, k), 1e-12));
            const double var_ref = covarianceCyclesFromMatrix(filter_state.covariance, ri, ri);
            const double var_sat = covarianceCyclesFromMatrix(filter_state.covariance, si, si);
            const double cov_ref_sat =
                covarianceCyclesFromMatrix(filter_state.covariance, ri, si);
            const double corr_ref_sat =
                cov_ref_sat /
                std::sqrt(
                    safeVarianceFloor(var_ref, 1e-12) *
                    safeVarianceFloor(var_sat, 1e-12));
            double qdiag_pre = dd_cov(k, k);
            if (have_pre_anchor) {
                qdiag_pre =
                    covarianceCyclesFromMatrix(pre_anchor_covariance, ri, ri) -
                    covarianceCyclesFromMatrix(pre_anchor_covariance, ri, si) -
                    covarianceCyclesFromMatrix(pre_anchor_covariance, si, ri) +
                    covarianceCyclesFromMatrix(pre_anchor_covariance, si, si);
            }
            const int ref_state_index = state_indices[static_cast<size_t>(ri)];
            const int sat_state_index = state_indices[static_cast<size_t>(si)];
            const int ref_clock_index = receiverClockStateIndex(
                filter_state, satellites[static_cast<size_t>(ri)]);
            const int sat_clock_index = receiverClockStateIndex(
                filter_state, satellites[static_cast<size_t>(si)]);
            std::cerr << "[PPP-AR-DDMAT] k=" << k
                      << " ref=" << satellites[static_cast<size_t>(ri)].toString()
                      << " sat=" << satellites[static_cast<size_t>(si)].toString()
                      << " dd_float=" << dd_float(k)
                      << " frac=" << frac
                      << " qdiag=" << dd_cov(k, k)
                      << " qdiag_pre=" << qdiag_pre
                      << " sigma=" << sigma
                      << "\n";
            if (config.wlnl_strict_claslib_parity) {
                std::cerr << "[PPP-AR-COV] k=" << k
                          << " ref=" << satellites[static_cast<size_t>(ri)].toString()
                          << " sat=" << satellites[static_cast<size_t>(si)].toString()
                          << " var_ref=" << var_ref
                          << " var_sat=" << var_sat
                          << " cov_refsat=" << cov_ref_sat
                          << " corr_refsat=" << corr_ref_sat
                          << " ref_clk_corr="
                          << safeCorrelation(debug_cross_cov, ref_state_index, ref_clock_index)
                          << " sat_clk_corr="
                          << safeCorrelation(debug_cross_cov, sat_state_index, sat_clock_index)
                          << " ref_trop_corr="
                          << safeCorrelation(debug_cross_cov, ref_state_index, filter_state.trop_index)
                          << " sat_trop_corr="
                          << safeCorrelation(debug_cross_cov, sat_state_index, filter_state.trop_index)
                          << " ref_pos_corr="
                          << positionCorrelationNorm(debug_cross_cov, filter_state, ref_state_index)
                          << " sat_pos_corr="
                          << positionCorrelationNorm(debug_cross_cov, filter_state, sat_state_index)
                          << "\n";
            }
        }
    }

    attempt.required_ratio = config.ar_ratio_threshold;
    if (config.use_clas_osr_filter) {
        attempt.required_ratio =
            std::max(attempt.required_ratio, claslibRatioThresholdForNb(attempt.nb));
    }
    VectorXd dd_fixed = VectorXd::Zero(attempt.nb);
    const bool lambda_ok = lambdaSearch(dd_float, dd_cov, dd_fixed, attempt.ratio);
    if (shouldDumpStrictFirstAr(config)) {
        writeStrictFirstArDump(
            config,
            filter_state,
            pre_anchor_covariance,
            ambiguity_states,
            satellites,
            state_indices,
            scales,
            active_indices,
            system_ref_map,
            dd_pairs,
            dd_float,
            dd_cov,
            dd_fixed,
            lambda_ok,
            attempt.ratio,
            attempt.required_ratio,
            observation_time);
    }
    if (!lambda_ok) {
        return attempt;
    }

    if (debug_enabled) {
        std::cerr << "[PPP-AR-LAMBDA] nb=" << attempt.nb
                  << " ratio=" << attempt.ratio
                  << " base_threshold=" << config.ar_ratio_threshold << "\n";
    }

    if (!std::isfinite(attempt.ratio) || attempt.ratio < attempt.required_ratio) {
        return attempt;
    }

    attempt.state = filter_state;
    attempt.ambiguities = ambiguity_states;

    const int na = filter_state.amb_index;
    const VectorXd dd_residual = dd_float - dd_fixed;
    MatrixXd dd_cov_copy = dd_cov;
    Eigen::LDLT<MatrixXd> ldlt(dd_cov_copy);
    if (ldlt.info() != Eigen::Success) {
        return DdFixAttempt{};
    }

    const VectorXd db = ldlt.solve(dd_residual);
    if (!db.allFinite()) {
        return DdFixAttempt{};
    }

    const MatrixXd& Pcross =
        pre_anchor_covariance.rows() > 0 ? pre_anchor_covariance : filter_state.covariance;
    MatrixXd Qab = MatrixXd::Zero(na, attempt.nb);
    for (int k = 0; k < attempt.nb; ++k) {
        const int ri = dd_pairs[static_cast<size_t>(k)].ref_idx;
        const int si = dd_pairs[static_cast<size_t>(k)].sat_idx;
        for (int i = 0; i < na; ++i) {
            Qab(i, k) =
                (Pcross(i, state_indices[static_cast<size_t>(ri)]) /
                 scales[static_cast<size_t>(ri)]) -
                (Pcross(i, state_indices[static_cast<size_t>(si)]) /
                 scales[static_cast<size_t>(si)]);
        }
    }

    const VectorXd delta = Qab * db;
    attempt.state.state.head(na) -= delta;

    if (config.wlnl_strict_claslib_parity) {
        constexpr double kClaslibHoldAmbVarianceCycles2 = 0.001;
        MatrixXd hold_H = MatrixXd::Zero(attempt.nb, filter_state.total_states);
        VectorXd hold_v = VectorXd::Zero(attempt.nb);
        MatrixXd hold_R =
            MatrixXd::Identity(attempt.nb, attempt.nb) *
            kClaslibHoldAmbVarianceCycles2;
        for (int k = 0; k < attempt.nb; ++k) {
            const int ri = dd_pairs[static_cast<size_t>(k)].ref_idx;
            const int si = dd_pairs[static_cast<size_t>(k)].sat_idx;
            hold_H(k, state_indices[static_cast<size_t>(ri)]) =
                1.0 / scales[static_cast<size_t>(ri)];
            hold_H(k, state_indices[static_cast<size_t>(si)]) =
                -1.0 / scales[static_cast<size_t>(si)];
            hold_v(k) = dd_fixed(k) - dd_float(k);
        }

        attempt.hold_state = filter_state;
        const MatrixXd hold_covariance_before = attempt.hold_state.covariance;
        const MatrixXd hold_PHt = hold_covariance_before * hold_H.transpose();
        const MatrixXd hold_S = hold_H * hold_PHt + hold_R;
        Eigen::LDLT<MatrixXd> hold_ldlt(hold_S);
        if (hold_ldlt.info() == Eigen::Success) {
            const MatrixXd hold_K =
                hold_PHt * hold_ldlt.solve(
                    MatrixXd::Identity(attempt.nb, attempt.nb));
            const MatrixXd hold_I_KH =
                MatrixXd::Identity(filter_state.total_states, filter_state.total_states) -
                hold_K * hold_H;
            attempt.hold_state.state += hold_K * hold_v;
            attempt.hold_state.covariance = hold_I_KH * hold_covariance_before;
            attempt.hold_state.covariance =
                0.5 * (attempt.hold_state.covariance +
                       attempt.hold_state.covariance.transpose());
            attempt.has_hold_state = true;
        }
    }

    if (debug_enabled && excluded_real_satellites.empty()) {
        double qab_pos_norm = 0.0;
        for (int axis = 0; axis < 3; ++axis) {
            qab_pos_norm += Qab.row(filter_state.pos_index + axis).squaredNorm();
        }
        qab_pos_norm = std::sqrt(qab_pos_norm);
        std::cerr << "[PPP-AR] dd_resid=" << dd_residual.norm()
                  << " db=" << db.norm()
                  << " Qab=" << Qab.norm()
                  << " Qab_pos=" << qab_pos_norm
                  << " delta_all=" << delta.norm()
                  << " delta_pos=" << delta.segment(filter_state.pos_index, 3).norm()
                  << " delta_clk=" << std::abs(delta(filter_state.clock_index))
                  << "\n";
    }

    for (const auto& [group, ref_idx] : system_ref_map) {
        (void)group;
        const int ref_state = state_indices[static_cast<size_t>(ref_idx)];
        auto& ref_ambiguity = attempt.ambiguities[satellites[static_cast<size_t>(ref_idx)]];
        ref_ambiguity.is_fixed = true;
        ref_ambiguity.fixed_value = attempt.state.state(ref_state);
    }

    for (int k = 0; k < attempt.nb; ++k) {
        const int ri = dd_pairs[static_cast<size_t>(k)].ref_idx;
        const int si = dd_pairs[static_cast<size_t>(k)].sat_idx;
        const double ref_value_m =
            attempt.state.state(state_indices[static_cast<size_t>(ri)]);
        const double scale_si = scales[static_cast<size_t>(si)];
        const double ref_cycles = ref_value_m / scales[static_cast<size_t>(ri)];
        const double fixed_sat_cycles = ref_cycles - dd_fixed(k);
        const double fixed_sat_m = fixed_sat_cycles * scale_si;

        const int sat_state = state_indices[static_cast<size_t>(si)];
        attempt.state.state(sat_state) = fixed_sat_m;
        attempt.state.covariance.row(sat_state).setZero();
        attempt.state.covariance.col(sat_state).setZero();
        attempt.state.covariance(sat_state, sat_state) = 1e-6;

        auto& ambiguity_state = attempt.ambiguities[satellites[static_cast<size_t>(si)]];
        ambiguity_state.float_value = fixed_sat_m;
        ambiguity_state.fixed_value = fixed_sat_m;
        ambiguity_state.is_fixed = true;
    }

    attempt.fixed = true;
    return attempt;
}

DdFixAttempt tryDirectDdFixWithPar(
    const ppp_shared::PPPConfig& config,
    const ppp_shared::PPPState& filter_state,
    const MatrixXd& pre_anchor_covariance,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const EligibleAmbiguities& eligible_ambiguities,
    const std::map<SatelliteId, double>& real_satellite_elevations,
    bool debug_enabled,
    const GNSSTime* observation_time) {
    auto try_dd_fix = [&](const std::set<SatelliteId>& excluded_real_satellites) {
        return tryDirectDdFix(
            config,
            filter_state,
            pre_anchor_covariance,
            ambiguity_states,
            eligible_ambiguities.satellites,
            eligible_ambiguities.state_indices,
            eligible_ambiguities.scales,
            excluded_real_satellites,
            debug_enabled,
            observation_time);
    };

    DdFixAttempt best_attempt = try_dd_fix({});
    if (best_attempt.fixed || !config.use_clas_osr_filter || real_satellite_elevations.empty()) {
        return best_attempt;
    }

    std::vector<SatelliteId> par_candidates;
    par_candidates.reserve(real_satellite_elevations.size());
    for (const auto& [satellite, _] : real_satellite_elevations) {
        par_candidates.push_back(satellite);
    }
    std::sort(par_candidates.begin(), par_candidates.end(),
              [&](const SatelliteId& lhs, const SatelliteId& rhs) {
                  return real_satellite_elevations.at(lhs) < real_satellite_elevations.at(rhs);
              });

    std::set<SatelliteId> excluded_real_satellites;
    const int max_exclusions = std::min(
        4,
        std::max(0, static_cast<int>(par_candidates.size()) - config.min_satellites_for_ar));

    for (int iteration = 0; iteration < max_exclusions && !best_attempt.fixed; ++iteration) {
        SatelliteId best_exclusion;
        bool has_best_exclusion = false;
        double best_ratio = best_attempt.ratio;
        double best_required_ratio = best_attempt.required_ratio;

        for (const auto& real_satellite : par_candidates) {
            if (excluded_real_satellites.count(real_satellite) != 0) {
                continue;
            }

            auto trial_exclusions = excluded_real_satellites;
            trial_exclusions.insert(real_satellite);
            DdFixAttempt trial_attempt = try_dd_fix(trial_exclusions);
            if (trial_attempt.fixed) {
                best_attempt = std::move(trial_attempt);
                if (debug_enabled) {
                    std::cerr << "[PPP-AR] PAR fixed: excluded="
                              << real_satellite.toString()
                              << " nb=" << best_attempt.nb
                              << " ratio=" << best_attempt.ratio
                              << " threshold=" << best_attempt.required_ratio << "\n";
                }
                break;
            }

            if (trial_attempt.ratio > best_ratio) {
                best_ratio = trial_attempt.ratio;
                best_required_ratio = trial_attempt.required_ratio;
                best_exclusion = real_satellite;
                has_best_exclusion = true;
            }
        }

        if (best_attempt.fixed || !has_best_exclusion) {
            break;
        }

        excluded_real_satellites.insert(best_exclusion);
        if (debug_enabled) {
            std::cerr << "[PPP-AR] PAR exclude candidate: "
                      << best_exclusion.toString()
                      << " ratio=" << best_ratio
                      << " threshold=" << best_required_ratio << "\n";
        }
    }

    return best_attempt;
}

WlnlFixAttempt tryWlnlFix(
    const ppp_shared::PPPConfig& config,
    ppp_shared::PPPState& filter_state,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const std::vector<SatelliteId>& satellites,
    const std::vector<int>& state_indices,
    const std::map<SatelliteId, WlnlNlInfo>& nl_info,
    bool debug_enabled) {
    WlnlFixAttempt attempt;

    struct WlnlDdPair {
        int ref_idx = -1;
        int sat_idx = -1;
    };

    std::vector<int> wl_fixed_indices;
    wl_fixed_indices.reserve(satellites.size());
    for (int i = 0; i < static_cast<int>(satellites.size()); ++i) {
        const auto ambiguity_it = ambiguity_states.find(satellites[static_cast<size_t>(i)]);
        if (ambiguity_it == ambiguity_states.end() || !ambiguity_it->second.wl_is_fixed) {
            continue;
        }
        wl_fixed_indices.push_back(i);
    }

    std::map<WlnlGroupKey, int> system_ref_map;
    for (const int idx : wl_fixed_indices) {
        const auto nl_it = nl_info.find(satellites[static_cast<size_t>(idx)]);
        if (nl_it == nl_info.end() || !nl_it->second.valid) {
            continue;
        }
        const auto& group = nl_it->second.group;
        if (system_ref_map.find(group) == system_ref_map.end()) {
            system_ref_map[group] = idx;
        } else {
            const auto& current_ref =
                ambiguity_states.at(satellites[static_cast<size_t>(system_ref_map[group])]);
            const auto& candidate = ambiguity_states.at(satellites[static_cast<size_t>(idx)]);
            if (candidate.lock_count > current_ref.lock_count) {
                system_ref_map[group] = idx;
            }
        }
    }
    if (debug_enabled) {
        for (const auto& [group, ref_idx] : system_ref_map) {
            std::cerr << "[PPP-WLNL-REF] sys=" << static_cast<int>(group.first)
                      << " bandpair=" << group.second.first << "," << group.second.second
                      << " ref=" << satellites[static_cast<size_t>(ref_idx)].toString()
                      << " lock=" << ambiguity_states.at(satellites[static_cast<size_t>(ref_idx)]).lock_count
                      << "\n";
        }
    }

    std::vector<WlnlDdPair> dd_pairs;
    for (const int idx : wl_fixed_indices) {
        const auto nl_it = nl_info.find(satellites[static_cast<size_t>(idx)]);
        if (nl_it == nl_info.end() || !nl_it->second.valid) {
            continue;
        }
        const auto ref_it = system_ref_map.find(nl_it->second.group);
        if (ref_it == system_ref_map.end() || ref_it->second == idx) {
            continue;
        }
        dd_pairs.push_back({ref_it->second, idx});
    }

    const bool strict_claslib_parity = config.wlnl_strict_claslib_parity;
    constexpr double kNlFractionThreshold = 0.25;
    std::vector<WlnlDdPair> active_dd_pairs;
    active_dd_pairs.reserve(dd_pairs.size());
    for (size_t pair_index = 0; pair_index < dd_pairs.size(); ++pair_index) {
        const int ri = dd_pairs[pair_index].ref_idx;
        const int si = dd_pairs[pair_index].sat_idx;
        const auto ref_it = nl_info.find(satellites[static_cast<size_t>(ri)]);
        const auto sat_it = nl_info.find(satellites[static_cast<size_t>(si)]);
        if (ref_it == nl_info.end() || !ref_it->second.valid ||
            sat_it == nl_info.end() || !sat_it->second.valid) {
            continue;
        }
        const double dd_nl =
            ref_it->second.nl_ambiguity_cycles - sat_it->second.nl_ambiguity_cycles;
        const double frac = dd_nl - std::round(dd_nl);
        if (debug_enabled && pair_index < 8) {
            std::cerr << "[PPP-WLNL] DD NL pair " << pair_index
                      << " ref=" << satellites[static_cast<size_t>(ri)].toString()
                      << " sat=" << satellites[static_cast<size_t>(si)].toString()
                      << " nl=" << dd_nl
                      << " frac=" << frac << "\n";
        }
        if (!strict_claslib_parity && std::abs(frac) > kNlFractionThreshold) {
            continue;
        }
        active_dd_pairs.push_back(dd_pairs[pair_index]);
    }

    int existing_nl_fixes = 0;
    for (const auto& [satellite, ambiguity] : ambiguity_states) {
        (void)satellite;
        if (ambiguity.is_fixed && ambiguity.wl_is_fixed && ambiguity.nl_is_fixed) {
            ++existing_nl_fixes;
        }
    }
    if (!strict_claslib_parity && existing_nl_fixes == 0) {
        std::vector<WlnlDdPair> gps_only_dd_pairs;
        gps_only_dd_pairs.reserve(active_dd_pairs.size());
        for (const auto& dd_pair : active_dd_pairs) {
            const auto& ref_sat = satellites[static_cast<size_t>(dd_pair.ref_idx)];
            const auto& sat_sat = satellites[static_cast<size_t>(dd_pair.sat_idx)];
            if (ref_sat.system == GNSSSystem::GPS && sat_sat.system == GNSSSystem::GPS) {
                gps_only_dd_pairs.push_back(dd_pair);
            }
        }
        if (gps_only_dd_pairs.size() >= 2 && gps_only_dd_pairs.size() < active_dd_pairs.size()) {
            if (debug_enabled) {
                std::cerr << "[PPP-WLNL] startup GPS-only DD retry: kept="
                          << gps_only_dd_pairs.size()
                          << " dropped=" << (active_dd_pairs.size() - gps_only_dd_pairs.size())
                          << "\n";
            }
            active_dd_pairs = std::move(gps_only_dd_pairs);
        }
    }

    bool startup_gps_only_subset = false;
    if (existing_nl_fixes == 0 && !active_dd_pairs.empty()) {
        startup_gps_only_subset = std::all_of(
            active_dd_pairs.begin(),
            active_dd_pairs.end(),
            [&](const WlnlDdPair& dd_pair) {
                const auto& ref_sat = satellites[static_cast<size_t>(dd_pair.ref_idx)];
                const auto& sat_sat = satellites[static_cast<size_t>(dd_pair.sat_idx)];
                return ref_sat.system == GNSSSystem::GPS &&
                       sat_sat.system == GNSSSystem::GPS;
            });
    }

    const int minimum_dd_pairs =
        (existing_nl_fixes == 0 && !strict_claslib_parity) ? 2 : 3;
    attempt.nb = static_cast<int>(active_dd_pairs.size());
    if (debug_enabled && existing_nl_fixes == 0) {
        std::cerr << "[PPP-WLNL-DDSET] total=" << dd_pairs.size()
                  << " active=" << active_dd_pairs.size()
                  << " strict=" << (strict_claslib_parity ? 1 : 0)
                  << " startup_gps_only=" << (startup_gps_only_subset ? 1 : 0)
                  << " min_dd=" << minimum_dd_pairs << "\n";
    }
    if (attempt.nb < minimum_dd_pairs) {
        if (debug_enabled) {
            std::cerr << "[PPP-WLNL] insufficient DD NL pairs: " << attempt.nb << "\n";
        }
        return attempt;
    }

    VectorXd dd_nl_float = VectorXd::Zero(attempt.nb);
    MatrixXd dd_transform = MatrixXd::Zero(attempt.nb, filter_state.total_states);
    MatrixXd dd_state_transform = MatrixXd::Zero(attempt.nb, filter_state.total_states);
    std::vector<bool> dd_valid(static_cast<size_t>(attempt.nb), false);

    int valid_dd = 0;
    for (int k = 0; k < attempt.nb; ++k) {
        const int ri = active_dd_pairs[static_cast<size_t>(k)].ref_idx;
        const int si = active_dd_pairs[static_cast<size_t>(k)].sat_idx;
        const auto ref_it = nl_info.find(satellites[static_cast<size_t>(ri)]);
        const auto sat_it = nl_info.find(satellites[static_cast<size_t>(si)]);
        const SatelliteId ref_l1_sat = satellites[static_cast<size_t>(ri)];
        const SatelliteId sat_l1_sat = satellites[static_cast<size_t>(si)];
        const SatelliteId ref_l2_sat(
            ref_l1_sat.system,
            static_cast<uint8_t>(std::min(255, static_cast<int>(ref_l1_sat.prn) + 100)));
        const SatelliteId sat_l2_sat(
            sat_l1_sat.system,
            static_cast<uint8_t>(std::min(255, static_cast<int>(sat_l1_sat.prn) + 100)));
        const int ref_l1_state = state_indices[static_cast<size_t>(ri)];
        const int sat_l1_state = state_indices[static_cast<size_t>(si)];
        const auto ref_l2_state_it = filter_state.ambiguity_indices.find(ref_l2_sat);
        const auto sat_l2_state_it = filter_state.ambiguity_indices.find(sat_l2_sat);
        if (ref_it == nl_info.end() || !ref_it->second.valid ||
            sat_it == nl_info.end() || !sat_it->second.valid ||
            ref_l1_state < 0 || ref_l1_state >= filter_state.total_states ||
            sat_l1_state < 0 || sat_l1_state >= filter_state.total_states ||
            ref_l2_state_it == filter_state.ambiguity_indices.end() ||
            sat_l2_state_it == filter_state.ambiguity_indices.end() ||
            ref_l2_state_it->second < 0 || ref_l2_state_it->second >= filter_state.total_states ||
            sat_l2_state_it->second < 0 || sat_l2_state_it->second >= filter_state.total_states ||
            sat_it->second.lambda_nl_m <= 0.0 || !std::isfinite(sat_it->second.lambda_nl_m) ||
            sat_it->second.lambda_wl_m <= 0.0 || !std::isfinite(sat_it->second.lambda_wl_m)) {
            dd_nl_float(k) = 0.0;
            continue;
        }
        const double coeff_l1 =
            0.5 * (1.0 / sat_it->second.lambda_nl_m + 1.0 / sat_it->second.lambda_wl_m);
        const double coeff_l2 =
            0.5 * (1.0 / sat_it->second.lambda_nl_m - 1.0 / sat_it->second.lambda_wl_m);
        const double ref_coeff_l1 =
            0.5 * (1.0 / ref_it->second.lambda_nl_m + 1.0 / ref_it->second.lambda_wl_m);
        const double ref_coeff_l2 =
            0.5 * (1.0 / ref_it->second.lambda_nl_m - 1.0 / ref_it->second.lambda_wl_m);
        dd_nl_float(k) = ref_it->second.nl_ambiguity_cycles - sat_it->second.nl_ambiguity_cycles;
        dd_transform(k, ref_l1_state) = ref_coeff_l1;
        dd_transform(k, sat_l1_state) = -coeff_l1;
        dd_transform(k, ref_l2_state_it->second) = ref_coeff_l2;
        dd_transform(k, sat_l2_state_it->second) = -coeff_l2;
        dd_state_transform(k, ref_l1_state) = ref_coeff_l1;
        dd_state_transform(k, sat_l1_state) = -coeff_l1;
        dd_state_transform(k, ref_l2_state_it->second) = ref_coeff_l2;
        dd_state_transform(k, sat_l2_state_it->second) = -coeff_l2;
        const auto ref_iono_state_it = filter_state.ionosphere_indices.find(ref_l1_sat);
        const auto sat_iono_state_it = filter_state.ionosphere_indices.find(sat_l1_sat);
        if (config.estimate_ionosphere &&
            ref_iono_state_it != filter_state.ionosphere_indices.end() &&
            sat_iono_state_it != filter_state.ionosphere_indices.end() &&
            ref_iono_state_it->second >= 0 &&
            ref_iono_state_it->second < filter_state.total_states &&
            sat_iono_state_it->second >= 0 &&
            sat_iono_state_it->second < filter_state.total_states &&
            std::abs(ref_coeff_l2) > 1e-12 &&
            std::abs(coeff_l2) > 1e-12) {
            const double ref_nl_iono_coeff =
                ref_coeff_l1 + (ref_coeff_l1 * ref_coeff_l1) / ref_coeff_l2;
            const double sat_nl_iono_coeff =
                coeff_l1 + (coeff_l1 * coeff_l1) / coeff_l2;
            dd_state_transform(k, ref_iono_state_it->second) = -ref_nl_iono_coeff;
            dd_state_transform(k, sat_iono_state_it->second) = sat_nl_iono_coeff;
        }
        dd_valid[static_cast<size_t>(k)] = true;
        ++valid_dd;
    }

    if (valid_dd < minimum_dd_pairs) {
        if (debug_enabled) {
            std::cerr << "[PPP-WLNL] insufficient valid DD NL: " << valid_dd << "\n";
        }
        return attempt;
    }

    MatrixXd dd_nl_cov = dd_transform * filter_state.covariance * dd_transform.transpose();
    MatrixXd dd_state_cov =
        dd_state_transform * filter_state.covariance * dd_state_transform.transpose();
    for (int k = 0; k < attempt.nb; ++k) {
        if (!dd_valid[static_cast<size_t>(k)]) {
            dd_nl_cov.row(k).setZero();
            dd_nl_cov.col(k).setZero();
            dd_nl_cov(k, k) = 1e10;
            dd_state_cov.row(k).setZero();
            dd_state_cov.col(k).setZero();
            dd_state_cov(k, k) = 1e10;
            continue;
        }
        dd_nl_cov(k, k) = safeVarianceFloor(dd_nl_cov(k, k), 1e-6);
        dd_state_cov(k, k) = safeVarianceFloor(dd_state_cov(k, k), 1e-6);
    }

    VectorXd dd_nl_fixed = VectorXd::Zero(attempt.nb);
    if (!lambdaSearch(dd_nl_float, dd_nl_cov, dd_nl_fixed, attempt.ratio)) {
        if (debug_enabled) {
            std::cerr << "[PPP-WLNL] NL lambda search failed, nb=" << attempt.nb << "\n";
        }
        return attempt;
    }
    const VectorXd dd_state_model_cycles = dd_state_transform * filter_state.state;
    double required_ratio = config.ar_ratio_threshold;
    if (!strict_claslib_parity &&
        existing_nl_fixes == 0 &&
        startup_gps_only_subset &&
        attempt.nb == 2) {
        required_ratio = 6.5;
    }
    if (debug_enabled && existing_nl_fixes == 0) {
        std::cerr << "[PPP-WLNL-LAMBDA] nb=" << attempt.nb
                  << " valid_dd=" << valid_dd
                  << " startup_gps_only=" << (startup_gps_only_subset ? 1 : 0)
                  << " ratio=" << attempt.ratio
                  << " threshold=" << required_ratio << "\n";
        for (int k = 0; k < attempt.nb && k < 8; ++k) {
            const int ri = active_dd_pairs[static_cast<size_t>(k)].ref_idx;
            const int si = active_dd_pairs[static_cast<size_t>(k)].sat_idx;
            std::cerr << "[PPP-WLNL-DDMAT] k=" << k
                      << " ref=" << satellites[static_cast<size_t>(ri)].toString()
                      << " sat=" << satellites[static_cast<size_t>(si)].toString()
                      << " obs_nl=" << dd_nl_float(k)
                      << " state_nl=" << dd_state_model_cycles(k)
                      << " fixed_nl=" << dd_nl_fixed(k)
                      << " q_obs=" << dd_nl_cov(k, k)
                      << " q_state=" << dd_state_cov(k, k)
                      << "\n";
        }
    }
    if (!std::isfinite(attempt.ratio) || attempt.ratio < required_ratio) {
        if (debug_enabled) {
            std::cerr << "[PPP-WLNL] NL ratio reject: nb=" << attempt.nb
                      << " ratio=" << attempt.ratio
                      << " threshold=" << required_ratio << "\n";
        }
        return attempt;
    }

    const auto build_fixed_state_candidate =
        [&](const VectorXd& fixed_dd_nl_cycles, ppp_shared::PPPState& fixed_state_out) {
            const int ambiguity_block_start =
                state_indices.empty() ? filter_state.total_states
                                      : *std::min_element(state_indices.begin(), state_indices.end());
            if (ambiguity_block_start <= 0) {
                return false;
            }
            Eigen::LDLT<MatrixXd> dd_ldlt(dd_state_cov);
            if (dd_ldlt.info() != Eigen::Success) {
                return false;
            }
            fixed_state_out = filter_state;
            const MatrixXd q_ab =
                filter_state.covariance.topRows(ambiguity_block_start) *
                dd_state_transform.transpose();
            const VectorXd dd_model_float_cycles = dd_state_transform * filter_state.state;
            attempt.fixed_state_dd_gap_cycles =
                (dd_nl_float - dd_model_float_cycles).norm();
            if (ppp_shared::pppDebugEnabled() &&
                attempt.fixed_state_dd_gap_cycles > 0.2) {
                for (int i = 0; i < attempt.nb && i < 4; ++i) {
                    const int ri = active_dd_pairs[static_cast<size_t>(i)].ref_idx;
                    const int si = active_dd_pairs[static_cast<size_t>(i)].sat_idx;
                    const auto& ref_satellite = satellites[static_cast<size_t>(ri)];
                    const auto& sat_satellite = satellites[static_cast<size_t>(si)];
                    const auto is_focus_gps = [](const SatelliteId& sat) {
                        return sat.system == GNSSSystem::GPS &&
                               (sat.prn == 25 || sat.prn == 26 ||
                                sat.prn == 29 || sat.prn == 31);
                    };
                    const bool focus_pair =
                        is_focus_gps(ref_satellite) || is_focus_gps(sat_satellite);
                    std::cerr << "[PPP-WLNL-STATE] ref="
                              << ref_satellite.toString()
                              << " sat="
                              << sat_satellite.toString()
                              << " obs_nl=" << dd_nl_float(i)
                              << " state_nl=" << dd_model_float_cycles(i)
                              << " gap=" << (dd_nl_float(i) - dd_model_float_cycles(i))
                              << "\n";
                    if (!focus_pair) {
                        continue;
                    }
                    const auto ref_it = nl_info.find(ref_satellite);
                    const auto sat_it = nl_info.find(sat_satellite);
                    const SatelliteId ref_l2_sat(
                        ref_satellite.system,
                        static_cast<uint8_t>(std::min(255, static_cast<int>(ref_satellite.prn) + 100)));
                    const SatelliteId sat_l2_sat(
                        sat_satellite.system,
                        static_cast<uint8_t>(std::min(255, static_cast<int>(sat_satellite.prn) + 100)));
                    const int ref_l1_state = state_indices[static_cast<size_t>(ri)];
                    const int sat_l1_state = state_indices[static_cast<size_t>(si)];
                    const auto ref_l2_state_it = filter_state.ambiguity_indices.find(ref_l2_sat);
                    const auto sat_l2_state_it = filter_state.ambiguity_indices.find(sat_l2_sat);
                    const auto ref_iono_state_it = filter_state.ionosphere_indices.find(ref_satellite);
                    const auto sat_iono_state_it = filter_state.ionosphere_indices.find(sat_satellite);
                    if (ref_it == nl_info.end() || sat_it == nl_info.end() ||
                        ref_l2_state_it == filter_state.ambiguity_indices.end() ||
                        sat_l2_state_it == filter_state.ambiguity_indices.end()) {
                        continue;
                    }
                    const double ref_coeff_l1 =
                        0.5 * (1.0 / ref_it->second.lambda_nl_m + 1.0 / ref_it->second.lambda_wl_m);
                    const double ref_coeff_l2 =
                        0.5 * (1.0 / ref_it->second.lambda_nl_m - 1.0 / ref_it->second.lambda_wl_m);
                    const double sat_coeff_l1 =
                        0.5 * (1.0 / sat_it->second.lambda_nl_m + 1.0 / sat_it->second.lambda_wl_m);
                    const double sat_coeff_l2 =
                        0.5 * (1.0 / sat_it->second.lambda_nl_m - 1.0 / sat_it->second.lambda_wl_m);
                    const double ref_state_l1 =
                        ref_coeff_l1 * filter_state.state(ref_l1_state);
                    const double ref_state_l2 =
                        ref_coeff_l2 * filter_state.state(ref_l2_state_it->second);
                    const double sat_state_l1 =
                        sat_coeff_l1 * filter_state.state(sat_l1_state);
                    const double sat_state_l2 =
                        sat_coeff_l2 * filter_state.state(sat_l2_state_it->second);
                    double ref_state_iono = 0.0;
                    double sat_state_iono = 0.0;
                    if (config.estimate_ionosphere &&
                        ref_iono_state_it != filter_state.ionosphere_indices.end() &&
                        sat_iono_state_it != filter_state.ionosphere_indices.end() &&
                        ref_iono_state_it->second >= 0 &&
                        ref_iono_state_it->second < filter_state.total_states &&
                        sat_iono_state_it->second >= 0 &&
                        sat_iono_state_it->second < filter_state.total_states &&
                        std::abs(ref_coeff_l2) > 1e-12 &&
                        std::abs(sat_coeff_l2) > 1e-12) {
                        const double ref_nl_iono_coeff =
                            ref_coeff_l1 + (ref_coeff_l1 * ref_coeff_l1) / ref_coeff_l2;
                        const double sat_nl_iono_coeff =
                            sat_coeff_l1 + (sat_coeff_l1 * sat_coeff_l1) / sat_coeff_l2;
                        ref_state_iono =
                            -ref_nl_iono_coeff * filter_state.state(ref_iono_state_it->second);
                        sat_state_iono =
                            -sat_nl_iono_coeff * filter_state.state(sat_iono_state_it->second);
                    }
                    const double obs_ref_phase_cycles =
                        ref_it->second.nl_phase_m / ref_it->second.lambda_nl_m;
                    const double obs_sat_phase_cycles =
                        sat_it->second.nl_phase_m / sat_it->second.lambda_nl_m;
                    const double obs_ref_pred_cycles =
                        ref_it->second.predicted_m / ref_it->second.lambda_nl_m;
                    const double obs_sat_pred_cycles =
                        sat_it->second.predicted_m / sat_it->second.lambda_nl_m;
                    const double obs_pair_phase_cycles =
                        obs_ref_phase_cycles - obs_sat_phase_cycles;
                    const double obs_pair_pred_cycles =
                        obs_ref_pred_cycles - obs_sat_pred_cycles;
                    const double obs_ref_total = obs_ref_phase_cycles - obs_ref_pred_cycles;
                    const double obs_sat_total = obs_sat_phase_cycles - obs_sat_pred_cycles;
                    const double ref_state_total =
                        ref_state_l1 + ref_state_l2 + ref_state_iono;
                    const double sat_state_total =
                        sat_state_l1 + sat_state_l2 + sat_state_iono;
                    const double state_pair_l1 = ref_state_l1 - sat_state_l1;
                    const double state_pair_l2 = ref_state_l2 - sat_state_l2;
                    const double state_pair_iono = ref_state_iono - sat_state_iono;
                    const double state_pair_total =
                        state_pair_l1 + state_pair_l2 + state_pair_iono;
                    std::cerr << "[PPP-WLNL-COMP] ref=" << ref_satellite.toString()
                              << " sat=" << sat_satellite.toString()
                              << " ref_obs_phase=" << obs_ref_phase_cycles
                              << " ref_obs_pred=" << obs_ref_pred_cycles
                              << " ref_obs_total=" << obs_ref_total
                              << " ref_state_l1=" << ref_state_l1
                              << " ref_state_l2=" << ref_state_l2
                              << " ref_state_iono=" << ref_state_iono
                              << " ref_state_total=" << ref_state_total
                              << " sat_obs_phase=" << obs_sat_phase_cycles
                              << " sat_obs_pred=" << obs_sat_pred_cycles
                              << " sat_obs_total=" << obs_sat_total
                              << " sat_state_l1=" << sat_state_l1
                              << " sat_state_l2=" << sat_state_l2
                              << " sat_state_iono=" << sat_state_iono
                              << " sat_state_total=" << sat_state_total
                              << " obs_phase=" << obs_pair_phase_cycles
                              << " obs_pred=" << obs_pair_pred_cycles
                              << " obs_total=" << (obs_pair_phase_cycles - obs_pair_pred_cycles)
                              << " state_l1=" << state_pair_l1
                              << " state_l2=" << state_pair_l2
                              << " state_iono=" << state_pair_iono
                              << " state_total=" << state_pair_total
                              << " gap="
                              << ((obs_pair_phase_cycles - obs_pair_pred_cycles) - state_pair_total)
                              << "\n";
                }
            }
            const VectorXd dd_residual_cycles = dd_nl_float - fixed_dd_nl_cycles;
            const VectorXd fixed_state_update = q_ab * dd_ldlt.solve(dd_residual_cycles);
            if (!fixed_state_update.allFinite()) {
                return false;
            }
            fixed_state_out.state.head(ambiguity_block_start) -= fixed_state_update;

            const MatrixXd fixed_covariance_reduction =
                q_ab * dd_ldlt.solve(q_ab.transpose());
            if (!fixed_covariance_reduction.allFinite()) {
                return false;
            }
            fixed_state_out.covariance.topLeftCorner(
                ambiguity_block_start,
                ambiguity_block_start) -= fixed_covariance_reduction;
            fixed_state_out.covariance.topLeftCorner(
                ambiguity_block_start,
                ambiguity_block_start) =
                0.5 * (fixed_state_out.covariance.topLeftCorner(
                           ambiguity_block_start,
                           ambiguity_block_start) +
                       fixed_state_out.covariance.topLeftCorner(
                           ambiguity_block_start,
                           ambiguity_block_start).transpose());
            if (ppp_shared::pppDebugEnabled()) {
                const Vector3d fixed_pos =
                    fixed_state_out.state.segment(fixed_state_out.pos_index, 3);
                const Vector3d float_pos =
                    filter_state.state.segment(filter_state.pos_index, 3);
                std::cerr << "[PPP-WLNL-FIXSTATE] pos_shift="
                          << (fixed_pos - float_pos).norm()
                          << " clock_shift="
                          << std::abs(fixed_state_out.state(filter_state.clock_index) -
                                      filter_state.state(filter_state.clock_index))
                          << " trop_shift="
                          << std::abs(fixed_state_out.state(filter_state.trop_index) -
                                      filter_state.state(filter_state.trop_index))
                          << " dd_float_gap="
                          << (dd_nl_float - dd_model_float_cycles).norm()
                          << "\n";
            }
            return true;
        };

    constexpr double hold_variance = 0.001 * 0.001;
    const int nx = filter_state.total_states;
    int nv = 0;
    MatrixXd H = MatrixXd::Zero(attempt.nb, nx);
    VectorXd v = VectorXd::Zero(attempt.nb);
    MatrixXd R = MatrixXd::Zero(attempt.nb, attempt.nb);

    for (int k = 0; k < attempt.nb; ++k) {
        const int ri = active_dd_pairs[static_cast<size_t>(k)].ref_idx;
        const int si = active_dd_pairs[static_cast<size_t>(k)].sat_idx;
        const int ref_state = state_indices[static_cast<size_t>(ri)];
        const int sat_state = state_indices[static_cast<size_t>(si)];
        const double float_dd_m = filter_state.state(ref_state) - filter_state.state(sat_state);

        const auto ref_nl_it = nl_info.find(satellites[static_cast<size_t>(ri)]);
        const auto sat_nl_it = nl_info.find(satellites[static_cast<size_t>(si)]);
        if (ref_nl_it == nl_info.end() || sat_nl_it == nl_info.end() ||
            !ref_nl_it->second.valid || !sat_nl_it->second.valid) {
            continue;
        }
        const auto& ref_amb = ambiguity_states.at(satellites[static_cast<size_t>(ri)]);
        const auto& sat_amb = ambiguity_states.at(satellites[static_cast<size_t>(si)]);
        const double dd_wl_integer =
            static_cast<double>(ref_amb.wl_fixed_integer - sat_amb.wl_fixed_integer);
        const double fixed_dd_m =
            dd_nl_fixed(k) * sat_nl_it->second.lambda_nl_m +
            dd_wl_integer *
                sat_nl_it->second.beta * sat_nl_it->second.lambda_wl_m;
        const double fixed_dd_l2_m =
            dd_nl_fixed(k) * sat_nl_it->second.lambda_nl_m -
            dd_wl_integer *
                sat_nl_it->second.beta * sat_nl_it->second.lambda_wl_m;

        attempt.dd_constraints.push_back({
            satellites[static_cast<size_t>(ri)],
            satellites[static_cast<size_t>(si)],
            fixed_dd_m,
            fixed_dd_l2_m});

        v(nv) = fixed_dd_m - float_dd_m;
        H(nv, ref_state) = 1.0;
        H(nv, sat_state) = -1.0;
        R(nv, nv) = hold_variance;
        ++nv;

        if (debug_enabled && k < 3) {
            std::cerr << "[PPP-WLNL] hold " << k
                      << " float_dd=" << float_dd_m
                      << " fixed_dd=" << fixed_dd_m
                      << " v=" << v(nv - 1) << "\n";
        }
    }

    // Do NOT apply holdamb KF update.  The NL integer constraints are
    // iono-free but the L1 ambiguity states are per-frequency (include
    // iono residual).  Applying holdamb would corrupt the filter.
    // Instead, fixed integers are stored in ambiguity_states and used
    // by solveFixedPosition() for a clean WLS fixed solution.
    (void)H;
    (void)v;
    (void)R;
    (void)nv;

    for (const auto& [group, ref_idx] : system_ref_map) {
        (void)group;
        const auto ref_nl_it = nl_info.find(satellites[static_cast<size_t>(ref_idx)]);
        if (ref_nl_it == nl_info.end() || !ref_nl_it->second.valid) {
            continue;
        }
        auto& ambiguity = ambiguity_states[satellites[static_cast<size_t>(ref_idx)]];
        ambiguity.is_fixed = true;
        ambiguity.nl_is_fixed = true;
        ambiguity.nl_fixed_cycles = std::round(ref_nl_it->second.nl_ambiguity_cycles);
    }

    for (int k = 0; k < attempt.nb; ++k) {
        const int ri = active_dd_pairs[static_cast<size_t>(k)].ref_idx;
        const int si = active_dd_pairs[static_cast<size_t>(k)].sat_idx;
        const auto& ref_ambiguity = ambiguity_states.at(satellites[static_cast<size_t>(ri)]);
        if (!ref_ambiguity.nl_is_fixed) {
            continue;
        }
        auto& sat_ambiguity = ambiguity_states[satellites[static_cast<size_t>(si)]];
        sat_ambiguity.is_fixed = true;
        sat_ambiguity.nl_is_fixed = true;
        sat_ambiguity.nl_fixed_cycles = ref_ambiguity.nl_fixed_cycles - dd_nl_fixed(k);
    }

    attempt.has_fixed_state = build_fixed_state_candidate(dd_nl_fixed, attempt.fixed_state);
    attempt.fixed = true;
    return attempt;
}

std::map<SatelliteId, WlnlNlInfo> buildWlnlNlInfoMap(
    const std::vector<SatelliteId>& satellites,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const WlnlNlInfoProvider& provider) {
    std::map<SatelliteId, WlnlNlInfo> nl_info;
    for (const auto& satellite : satellites) {
        const auto ambiguity_it = ambiguity_states.find(satellite);
        if (ambiguity_it == ambiguity_states.end() || !ambiguity_it->second.wl_is_fixed) {
            continue;
        }
        WlnlNlInfo info;
        if (!provider || !provider(satellite, info) || !info.valid) {
            continue;
        }
        nl_info[satellite] = std::move(info);
    }
    return nl_info;
}

WlnlFixAttempt resolveWlnlFix(
    const ppp_shared::PPPConfig& config,
    ppp_shared::PPPState& filter_state,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const EligibleAmbiguities& eligible_ambiguities,
    const WlnlNlInfoProvider& provider,
    bool debug_enabled) {
    const auto nl_info = buildWlnlNlInfoMap(
        eligible_ambiguities.satellites,
        ambiguity_states,
        provider);
    return tryWlnlFix(
        config,
        filter_state,
        ambiguity_states,
        eligible_ambiguities.satellites,
        eligible_ambiguities.state_indices,
        nl_info,
        debug_enabled);
}

std::vector<FixedNlObservation> buildFixedNlObservations(
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const FixedNlObservationProvider& provider) {
    std::vector<FixedNlObservation> observations;
    for (const auto& [satellite, ambiguity] : ambiguity_states) {
        if (!ambiguity.is_fixed || !ambiguity.wl_is_fixed || !ambiguity.nl_is_fixed) {
            continue;
        }
        FixedNlObservation observation;
        if (!provider || !provider(satellite, ambiguity, observation)) {
            continue;
        }
        observations.push_back(std::move(observation));
    }
    return observations;
}

std::vector<FixedCarrierObservation> buildFixedCarrierObservations(
    size_t candidate_count,
    const FixedCarrierObservationProvider& provider) {
    std::vector<FixedCarrierObservation> observations;
    observations.reserve(candidate_count);
    for (size_t index = 0; index < candidate_count; ++index) {
        FixedCarrierObservation observation;
        if (!provider || !provider(index, observation)) {
            continue;
        }
        observations.push_back(std::move(observation));
    }
    return observations;
}

bool solveFixedNlPosition(
    const std::vector<FixedNlObservation>& fixed_observations,
    const Vector3d& initial_position,
    double initial_clock_m,
    double trop_zenith,
    const GNSSTime& time,
    const TropMappingFunction& trop_mapping_function,
    Vector3d& fixed_position,
    double* position_shift_norm_m) {
    if (fixed_observations.size() < 4) {
        return false;
    }

    Vector3d position = initial_position;
    double clock_m = initial_clock_m;

    for (int iter = 0; iter < 5; ++iter) {
        const int nobs = static_cast<int>(fixed_observations.size());
        MatrixXd H = MatrixXd::Zero(nobs, 4);
        VectorXd residuals = VectorXd::Zero(nobs);

        for (int i = 0; i < nobs; ++i) {
            const auto& fixed_observation = fixed_observations[static_cast<size_t>(i)];
            const double geo = geodist(fixed_observation.sat_pos, position);
            const Vector3d los = (fixed_observation.sat_pos - position).normalized();
            const double elevation = std::asin(los.dot(position.normalized()));
            const double trop_delay =
                fixed_observation.use_trop_model && trop_mapping_function
                    ? trop_mapping_function(position, elevation, time) * trop_zenith
                    : 0.0;

            const double predicted = geo + clock_m
                                     + fixed_observation.system_clock_offset_m
                                     - constants::SPEED_OF_LIGHT * fixed_observation.sat_clk
                                     + trop_delay
                                     + fixed_observation.fixed_nl_cycles *
                                           fixed_observation.lambda_nl_m;
            residuals(i) = fixed_observation.nl_phase_m - predicted;
            H(i, 0) = -los.x();
            H(i, 1) = -los.y();
            H(i, 2) = -los.z();
            H(i, 3) = 1.0;
        }

        const MatrixXd HTH = H.transpose() * H;
        const VectorXd dx = HTH.ldlt().solve(H.transpose() * residuals);
        if (!dx.allFinite()) {
            return false;
        }
        position += dx.head(3);
        clock_m += dx(3);

        if (dx.head(3).norm() < 1e-4) {
            break;
        }
    }

    fixed_position = position;
    if (position_shift_norm_m != nullptr) {
        *position_shift_norm_m = (position - initial_position).norm();
    }
    return true;
}

bool solveFixedCarrierPosition(
    const std::vector<FixedCarrierObservation>& fixed_observations,
    const Vector3d& initial_position,
    double initial_clock_m,
    double trop_zenith,
    bool estimate_troposphere,
    Vector3d& fixed_position) {
    if (fixed_observations.size() < 4) {
        return false;
    }

    Vector3d position = initial_position;
    double clock_m = initial_clock_m;

    for (int iter = 0; iter < 6; ++iter) {
        const int nobs = static_cast<int>(fixed_observations.size());
        MatrixXd H = MatrixXd::Zero(nobs, 4);
        MatrixXd W = MatrixXd::Zero(nobs, nobs);
        VectorXd residuals = VectorXd::Zero(nobs);

        for (int i = 0; i < nobs; ++i) {
            const auto& fixed_observation = fixed_observations[static_cast<size_t>(i)];
            const Vector3d range_vector = fixed_observation.satellite_position - position;
            const double geometric_range = range_vector.norm();
            if (!std::isfinite(geometric_range) || geometric_range <= 1.0) {
                return false;
            }
            const Vector3d line_of_sight = range_vector / geometric_range;
            const double trop_delay =
                estimate_troposphere
                    ? fixed_observation.trop_mapping * trop_zenith
                    : fixed_observation.modeled_trop_delay_m;
            const double predicted =
                geometric_range + clock_m + fixed_observation.system_clock_offset_m
                - constants::SPEED_OF_LIGHT * fixed_observation.satellite_clock_bias_s
                + trop_delay - fixed_observation.ionosphere_m + fixed_observation.ambiguity_m;
            residuals(i) = fixed_observation.carrier_phase_if - predicted;
            H(i, 0) = -line_of_sight.x();
            H(i, 1) = -line_of_sight.y();
            H(i, 2) = -line_of_sight.z();
            H(i, 3) = 1.0;
            W(i, i) = 1.0 / safeVarianceFloor(fixed_observation.variance_cp, 1e-8);
        }

        const MatrixXd normal = H.transpose() * W * H;
        const VectorXd rhs = H.transpose() * W * residuals;
        const VectorXd dx = normal.ldlt().solve(rhs);
        if (!dx.allFinite()) {
            return false;
        }
        position += dx.head(3);
        clock_m += dx(3);
        if (dx.head(3).norm() < 1e-4) {
            break;
        }
    }

    fixed_position = position;
    return true;
}

}  // namespace libgnss::ppp_ar
