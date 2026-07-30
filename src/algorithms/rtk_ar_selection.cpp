#include <libgnss++/algorithms/rtk_ar_selection.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace libgnss {
namespace rtk_ar_selection {
namespace {

std::vector<int> buildSubsetExcluding(const std::vector<PairDescriptor>& pairs,
                                      std::initializer_list<GNSSSystem> excluded) {
    std::vector<int> subset;
    subset.reserve(pairs.size());
    for (int index = 0; index < static_cast<int>(pairs.size()); ++index) {
        bool keep = true;
        for (GNSSSystem system : excluded) {
            if (pairs[index].system == system) {
                keep = false;
                break;
            }
        }
        if (keep) {
            subset.push_back(index);
        }
    }
    return subset;
}

}  // namespace

bool conditionNarrowLaneOnFixedWideLane(
    const Eigen::VectorXd& head_state,
    const Eigen::MatrixXd& Q_head_n1,
    const Eigen::VectorXd& n1_float,
    const Eigen::MatrixXd& Q_n1,
    const Eigen::VectorXd& wide_lane_float,
    const Eigen::MatrixXd& Q_wide_lane,
    const Eigen::VectorXd& fixed_wide_lane,
    Eigen::VectorXd& conditioned_head_state,
    Eigen::MatrixXd& conditioned_Q_head_n1,
    Eigen::VectorXd& conditioned_n1_float,
    Eigen::MatrixXd& conditioned_Q_n1) {
    const Eigen::Index n = n1_float.size();
    if (n == 0 || wide_lane_float.size() != n ||
        fixed_wide_lane.size() != n || Q_n1.rows() != n ||
        Q_n1.cols() != n || Q_wide_lane.rows() != n ||
        Q_wide_lane.cols() != n || Q_head_n1.rows() != head_state.size() ||
        Q_head_n1.cols() != n || !head_state.allFinite() ||
        !Q_head_n1.allFinite() || !n1_float.allFinite() ||
        !Q_n1.allFinite() || !wide_lane_float.allFinite() ||
        !Q_wide_lane.allFinite() || !fixed_wide_lane.allFinite()) {
        return false;
    }
    const Eigen::MatrixXd symmetric_Qw =
        (Q_wide_lane + Q_wide_lane.transpose()) * 0.5;
    Eigen::LDLT<Eigen::MatrixXd> decomposition(symmetric_Qw);
    if (decomposition.info() != Eigen::Success ||
        (decomposition.vectorD().array() <= 1e-12).any()) {
        return false;
    }
    const Eigen::MatrixXd inverse_Qw =
        decomposition.solve(Eigen::MatrixXd::Identity(n, n));
    if (decomposition.info() != Eigen::Success ||
        !inverse_Qw.allFinite()) {
        return false;
    }
    const Eigen::MatrixXd n1_gain = Q_n1 * inverse_Qw;
    const Eigen::MatrixXd head_gain = Q_head_n1 * inverse_Qw;
    const Eigen::VectorXd innovation =
        fixed_wide_lane - wide_lane_float;
    conditioned_head_state = head_state + head_gain * innovation;
    conditioned_n1_float = n1_float + n1_gain * innovation;
    conditioned_Q_n1 = Q_n1 - n1_gain * Q_n1;
    conditioned_Q_n1 =
        (conditioned_Q_n1 + conditioned_Q_n1.transpose()) * 0.5;
    conditioned_Q_head_n1 =
        Q_head_n1 - head_gain * Q_n1;
    return conditioned_head_state.allFinite() &&
           conditioned_n1_float.allFinite() &&
           conditioned_Q_n1.allFinite() &&
           conditioned_Q_head_n1.allFinite();
}

std::vector<int> filterPairsByRelativeVariance(const std::vector<PairDescriptor>& pairs,
                                               double multiplier,
                                               double minimum_threshold,
                                               int minimum_pairs) {
    if (pairs.empty()) {
        return {};
    }

    std::vector<double> variances;
    variances.reserve(pairs.size());
    for (const auto& pair : pairs) {
        variances.push_back(pair.variance);
    }
    std::sort(variances.begin(), variances.end());
    const double median_variance = variances[variances.size() / 2];
    const double threshold = std::max(median_variance * multiplier, minimum_threshold);

    std::vector<int> subset;
    subset.reserve(pairs.size());
    for (int index = 0; index < static_cast<int>(pairs.size()); ++index) {
        if (pairs[index].variance <= threshold) {
            subset.push_back(index);
        }
    }
    if (static_cast<int>(subset.size()) < minimum_pairs) {
        return {};
    }
    return subset;
}

std::vector<std::vector<int>> buildPreferredSubsets(const std::vector<PairDescriptor>& pairs) {
    return {
        buildSubsetExcluding(pairs, {GNSSSystem::GLONASS, GNSSSystem::BeiDou}),
        buildSubsetExcluding(pairs, {GNSSSystem::GLONASS}),
        buildSubsetExcluding(pairs, {GNSSSystem::BeiDou}),
    };
}

std::vector<std::vector<int>> buildPaperConstellationFallbackSubsets(
    const std::vector<PairDescriptor>& pairs) {
    return {
        buildSubsetExcluding(pairs, {GNSSSystem::GLONASS}),
        buildSubsetExcluding(pairs, {GNSSSystem::BeiDou}),
        buildSubsetExcluding(pairs, {GNSSSystem::GLONASS, GNSSSystem::BeiDou}),
        buildSubsetExcluding(pairs, {GNSSSystem::GLONASS, GNSSSystem::Galileo}),
        buildSubsetExcluding(
            pairs, {GNSSSystem::GLONASS, GNSSSystem::Galileo, GNSSSystem::BeiDou}),
    };
}

std::vector<std::vector<int>> buildBSRGuidedDropSubsets(
    const std::vector<PairDescriptor>& pairs,
    const Eigen::MatrixXd& Qb,
    int minimum_pairs,
    int max_drop_steps,
    int worst_axes) {
    const int n = static_cast<int>(pairs.size());
    if (n == 0) return {};
    if (Qb.rows() != n || Qb.cols() != n) return {};
    if (worst_axes < 1) return {};

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(Qb);
    if (eig.info() != Eigen::Success) return {};

    const Eigen::VectorXd& eigvals = eig.eigenvalues();
    const Eigen::MatrixXd& eigvecs = eig.eigenvectors();
    if (eigvals.size() == 0 || eigvals.minCoeff() < -1e-10) {
        return {};
    }

    // Identify the K worst axes (largest eigenvalues = least informative).
    std::vector<std::pair<double, int>> ranked;
    ranked.reserve(n);
    for (int k = 0; k < n; ++k) {
        ranked.emplace_back(eigvals(k), k);
    }
    std::sort(ranked.begin(), ranked.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });

    const int K = std::min(worst_axes, n);
    std::vector<int> worst_axis_indices;
    std::vector<double> worst_axis_eigvals;
    worst_axis_indices.reserve(K);
    worst_axis_eigvals.reserve(K);
    for (int i = 0; i < K; ++i) {
        worst_axis_indices.push_back(ranked[i].second);
        worst_axis_eigvals.push_back(std::max(0.0, ranked[i].first));
    }

    // Per-pair loading on the worst axes.
    std::vector<double> loading(n, 0.0);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < K; ++j) {
            const int k = worst_axis_indices[j];
            loading[i] += std::abs(eigvecs(i, k)) * worst_axis_eigvals[j];
        }
    }

    std::vector<std::vector<int>> subsets;
    std::vector<int> current_subset;
    current_subset.reserve(n);
    for (int i = 0; i < n; ++i) current_subset.push_back(i);

    const int max_steps = std::min(n - minimum_pairs, max_drop_steps);
    for (int step = 0; step < max_steps; ++step) {
        int worst_index = -1;
        double worst_load = -1.0;
        for (int i : current_subset) {
            if (loading[i] > worst_load) {
                worst_load = loading[i];
                worst_index = i;
            }
        }
        if (worst_index < 0) break;

        std::vector<int> subset;
        subset.reserve(current_subset.size());
        for (int i : current_subset) {
            if (i != worst_index) subset.push_back(i);
        }
        if (static_cast<int>(subset.size()) < minimum_pairs) break;
        subsets.push_back(subset);
        current_subset = subset;
        loading[worst_index] = -1.0;  // exclude from future picks
    }
    return subsets;
}

std::vector<std::vector<int>> buildProgressiveVarianceDropSubsets(
    const std::vector<PairDescriptor>& pairs,
    int minimum_pairs,
    int max_drop_steps) {
    std::vector<std::vector<int>> subsets;
    std::vector<int> current_subset;
    current_subset.reserve(pairs.size());
    for (int index = 0; index < static_cast<int>(pairs.size()); ++index) {
        current_subset.push_back(index);
    }

    const int max_steps = std::min(static_cast<int>(pairs.size()) - minimum_pairs, max_drop_steps);
    for (int step = 0; step < max_steps; ++step) {
        int worst_index = -1;
        double worst_variance = -1.0;
        for (int index : current_subset) {
            if (pairs[index].variance > worst_variance) {
                worst_variance = pairs[index].variance;
                worst_index = index;
            }
        }
        if (worst_index < 0) {
            break;
        }

        std::vector<int> subset;
        subset.reserve(current_subset.size());
        for (int index : current_subset) {
            if (index != worst_index) {
                subset.push_back(index);
            }
        }
        if (static_cast<int>(subset.size()) < minimum_pairs) {
            break;
        }
        subsets.push_back(subset);
        current_subset = subset;
    }
    return subsets;
}

std::vector<std::vector<int>> buildSatelliteQualityDropSubsets(
    const std::vector<PairDescriptor>& pairs,
    int minimum_pairs,
    int max_drop_steps) {
    struct SatelliteQuality {
        SatelliteId satellite;
        std::vector<int> indices;
        double variance = -1.0;
        double fractional_distance = -1.0;
        double posterior_abs_residual = -1.0;
        double elevation = std::numeric_limits<double>::infinity();
        double snr = std::numeric_limits<double>::infinity();
        double azimuth = std::numeric_limits<double>::quiet_NaN();
        double azimuth_crowding =
            std::numeric_limits<double>::infinity();
        bool has_fractional_distance = false;
        bool has_posterior_abs_residual = false;
        bool has_elevation = false;
        bool has_snr = false;
        bool has_azimuth = false;
        double score = 0.0;
    };

    if (minimum_pairs < 1 || max_drop_steps < 1) return {};
    std::map<SatelliteId, SatelliteQuality> grouped;
    for (int index = 0; index < static_cast<int>(pairs.size()); ++index) {
        const auto& pair = pairs[index];
        auto [it, inserted] = grouped.try_emplace(pair.satellite);
        auto& quality = it->second;
        if (inserted) quality.satellite = pair.satellite;
        quality.indices.push_back(index);
        if (std::isfinite(pair.variance)) {
            quality.variance = std::max(quality.variance, pair.variance);
        }
        if (std::isfinite(pair.fractional_distance_cycles)) {
            quality.fractional_distance =
                std::max(quality.fractional_distance,
                         pair.fractional_distance_cycles);
            quality.has_fractional_distance = true;
        }
        if (std::isfinite(pair.posterior_abs_residual_m)) {
            quality.posterior_abs_residual =
                std::max(
                    quality.posterior_abs_residual,
                    pair.posterior_abs_residual_m);
            quality.has_posterior_abs_residual = true;
        }
        if (std::isfinite(pair.elevation_rad)) {
            quality.elevation = std::min(quality.elevation, pair.elevation_rad);
            quality.has_elevation = true;
        }
        if (std::isfinite(pair.snr_dbhz)) {
            quality.snr = std::min(quality.snr, pair.snr_dbhz);
            quality.has_snr = true;
        }
        if (std::isfinite(pair.azimuth_rad)) {
            quality.azimuth = pair.azimuth_rad;
            quality.has_azimuth = true;
        }
    }
    if (grouped.size() < 2 || static_cast<int>(pairs.size()) <= minimum_pairs) {
        return {};
    }

    std::vector<SatelliteQuality> ranked;
    ranked.reserve(grouped.size());
    for (const auto& [satellite, quality] : grouped) {
        (void)satellite;
        ranked.push_back(quality);
    }
    constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
    for (auto& quality : ranked) {
        if (!quality.has_azimuth) continue;
        for (const auto& other : ranked) {
            if (!other.has_azimuth ||
                other.satellite == quality.satellite) {
                continue;
            }
            double separation =
                std::abs(quality.azimuth - other.azimuth);
            separation = std::min(separation, kTwoPi - separation);
            quality.azimuth_crowding =
                std::min(quality.azimuth_crowding, separation);
        }
    }
    const double denominator =
        static_cast<double>(std::max<std::size_t>(1, ranked.size() - 1));
    for (auto& quality : ranked) {
        double rank_sum = 0.0;
        int rank_count = 0;
        auto add_rank = [&](auto worse_than, bool available) {
            if (!available) return;
            int worse_rank = 0;
            for (const auto& other : ranked) {
                if (worse_than(quality, other)) ++worse_rank;
            }
            rank_sum += static_cast<double>(worse_rank) / denominator;
            ++rank_count;
        };
        add_rank(
            [](const auto& value, const auto& other) {
                return value.variance > other.variance;
            },
            std::isfinite(quality.variance));
        add_rank(
            [](const auto& value, const auto& other) {
                return value.fractional_distance >
                       other.fractional_distance;
            },
            quality.has_fractional_distance);
        add_rank(
            [](const auto& value, const auto& other) {
                return value.posterior_abs_residual >
                       other.posterior_abs_residual;
            },
            quality.has_posterior_abs_residual);
        add_rank(
            [](const auto& value, const auto& other) {
                return value.elevation < other.elevation;
            },
            quality.has_elevation);
        add_rank(
            [](const auto& value, const auto& other) {
                return value.snr < other.snr;
            },
            quality.has_snr);
        add_rank(
            [](const auto& value, const auto& other) {
                return value.azimuth_crowding <
                       other.azimuth_crowding;
            },
            quality.has_azimuth &&
                std::isfinite(quality.azimuth_crowding));
        quality.score = rank_count > 0 ? rank_sum / rank_count : 0.0;
    }
    std::stable_sort(
        ranked.begin(), ranked.end(),
        [](const SatelliteQuality& left, const SatelliteQuality& right) {
            if (left.score != right.score) return left.score > right.score;
            if (left.variance != right.variance) {
                return left.variance > right.variance;
            }
            return left.satellite < right.satellite;
        });

    std::vector<int> current;
    current.reserve(pairs.size());
    for (int index = 0; index < static_cast<int>(pairs.size()); ++index) {
        current.push_back(index);
    }
    std::vector<std::vector<int>> subsets;
    for (const auto& quality : ranked) {
        if (static_cast<int>(subsets.size()) >= max_drop_steps) break;
        std::vector<int> candidate;
        candidate.reserve(current.size());
        for (int index : current) {
            if (pairs[index].satellite != quality.satellite) {
                candidate.push_back(index);
            }
        }
        if (candidate.size() == current.size()) continue;
        if (static_cast<int>(candidate.size()) < minimum_pairs) continue;
        subsets.push_back(candidate);
        current = std::move(candidate);
    }
    return subsets;
}

std::vector<std::vector<int>> buildSatelliteQualityDiverseDropSubsets(
    const std::vector<PairDescriptor>& pairs,
    int minimum_pairs,
    int max_drop_steps,
    int maximum_subsets) {
    if (maximum_subsets < 1) {
        return {};
    }
    std::vector<std::vector<int>> result;
    std::set<std::vector<int>> seen;
    const auto append_unique =
        [&](const std::vector<std::vector<int>>& candidates) {
            for (const auto& candidate : candidates) {
                if (static_cast<int>(result.size()) >= maximum_subsets) {
                    break;
                }
                if (seen.insert(candidate).second) {
                    result.push_back(candidate);
                }
            }
        };
    append_unique(buildSatelliteQualityDropSubsets(
        pairs, minimum_pairs, max_drop_steps));

    enum class Metric {
        VARIANCE,
        POSTERIOR_RESIDUAL,
        FRACTIONAL_DISTANCE,
        ELEVATION,
        SNR,
        AZIMUTH
    };
    constexpr Metric metrics[] = {
        Metric::VARIANCE,
        Metric::POSTERIOR_RESIDUAL,
        Metric::FRACTIONAL_DISTANCE,
        Metric::ELEVATION,
        Metric::SNR,
        Metric::AZIMUTH,
    };
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (const Metric metric : metrics) {
        if (static_cast<int>(result.size()) >= maximum_subsets) {
            break;
        }
        const bool metric_available = std::any_of(
            pairs.begin(), pairs.end(),
            [metric](const PairDescriptor& pair) {
                switch (metric) {
                    case Metric::VARIANCE:
                        return std::isfinite(pair.variance);
                    case Metric::POSTERIOR_RESIDUAL:
                        return std::isfinite(
                            pair.posterior_abs_residual_m);
                    case Metric::FRACTIONAL_DISTANCE:
                        return std::isfinite(
                            pair.fractional_distance_cycles);
                    case Metric::ELEVATION:
                        return std::isfinite(pair.elevation_rad);
                    case Metric::SNR:
                        return std::isfinite(pair.snr_dbhz);
                    case Metric::AZIMUTH:
                        return std::isfinite(pair.azimuth_rad);
                }
                return false;
            });
        if (!metric_available) {
            continue;
        }
        auto metric_pairs = pairs;
        for (auto& pair : metric_pairs) {
            if (metric != Metric::VARIANCE) {
                pair.variance = nan;
            }
            if (metric != Metric::POSTERIOR_RESIDUAL) {
                pair.posterior_abs_residual_m = nan;
            }
            if (metric != Metric::FRACTIONAL_DISTANCE) {
                pair.fractional_distance_cycles = nan;
            }
            if (metric != Metric::ELEVATION) {
                pair.elevation_rad = nan;
            }
            if (metric != Metric::SNR) {
                pair.snr_dbhz = nan;
            }
            if (metric != Metric::AZIMUTH) {
                pair.azimuth_rad = nan;
            }
        }
        append_unique(buildSatelliteQualityDropSubsets(
            metric_pairs, minimum_pairs, max_drop_steps));
    }
    return result;
}

}  // namespace rtk_ar_selection
}  // namespace libgnss
