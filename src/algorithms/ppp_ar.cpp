#include <libgnss++/algorithms/ppp_ar.hpp>

#include <libgnss++/algorithms/lambda.hpp>
#include <libgnss++/core/coordinates.hpp>

#include <algorithm>
#include <array>
#include <cmath>
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

int madocaFrequencyAmbiguityStateIndex(
    const ppp_shared::PPPState& filter_state,
    const SatelliteId& satellite,
    int frequency_index) {
    if (frequency_index < 0) {
        return -1;
    }
    const auto index_it = filter_state.frequency_ambiguity_indices.find(
        ppp_shared::frequencyAmbiguityKey(satellite, frequency_index));
    return index_it == filter_state.frequency_ambiguity_indices.end() ? -1 : index_it->second;
}

AmbiguityStateIndexProvider makeMadocaFrequencyAmbiguityStateIndexProvider(
    const ppp_shared::PPPState& filter_state) {
    return [&filter_state](const SatelliteId& satellite, int frequency_index) {
        return madocaFrequencyAmbiguityStateIndex(filter_state, satellite, frequency_index);
    };
}

MadocaArDesignRow buildMadocaArDesignRow(
    const SatelliteId& satellite,
    const SatelliteId& reference_satellite,
    int first_frequency_index,
    int second_frequency_index,
    const AmbiguityStateIndexProvider& state_index_provider,
    const AmbiguityWavelengthProvider& wavelength_provider) {
    MadocaArDesignRow row;
    if (!state_index_provider || !wavelength_provider || first_frequency_index < 0) {
        return row;
    }

    auto append_term = [&](const SatelliteId& term_satellite,
                           int frequency_index,
                           double sign) {
        const int state_index = state_index_provider(term_satellite, frequency_index);
        const double wavelength_m = wavelength_provider(term_satellite, frequency_index);
        if (state_index < 0 || !std::isfinite(wavelength_m) || wavelength_m <= 0.0) {
            row.valid = false;
            row.terms.clear();
            return false;
        }

        MadocaArDesignTerm term;
        term.satellite = term_satellite;
        term.frequency_index = frequency_index;
        term.state_index = state_index;
        term.coefficient = sign / wavelength_m;
        row.terms.push_back(term);
        return true;
    };

    row.valid = true;
    if (!append_term(satellite, first_frequency_index, 1.0) ||
        !append_term(reference_satellite, first_frequency_index, -1.0)) {
        return row;
    }

    if (second_frequency_index >= 0) {
        const MadocaArDesignTerm reference_first = row.terms.back();
        row.terms.pop_back();
        if (!append_term(satellite, second_frequency_index, -1.0)) {
            return row;
        }
        row.terms.push_back(reference_first);
        if (!append_term(reference_satellite, second_frequency_index, 1.0)) {
            return row;
        }
    }

    return row;
}

double evaluateMadocaArDesignRow(
    const MadocaArDesignRow& row,
    const VectorXd& state) {
    if (!row.valid) {
        return 0.0;
    }

    double value = 0.0;
    for (const auto& term : row.terms) {
        if (term.state_index < 0 || term.state_index >= state.size()) {
            return 0.0;
        }
        value += term.coefficient * state(term.state_index);
    }
    return value;
}

double madocaArDesignRowVariance(
    const MadocaArDesignRow& row,
    const MatrixXd& covariance) {
    if (!row.valid) {
        return 0.0;
    }

    double variance = 0.0;
    for (const auto& lhs : row.terms) {
        if (lhs.state_index < 0 || lhs.state_index >= covariance.rows()) {
            return 0.0;
        }
        for (const auto& rhs : row.terms) {
            if (rhs.state_index < 0 || rhs.state_index >= covariance.cols()) {
                return 0.0;
            }
            variance += lhs.coefficient * covariance(lhs.state_index, rhs.state_index) *
                rhs.coefficient;
        }
    }
    return variance;
}

EligibleAmbiguities collectEligibleAmbiguities(
    const ppp_shared::PPPState& filter_state,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    int min_lock_count) {
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

namespace {

// Apply a scalar Kalman pseudo-observation y=innovation onto filter_state with
// sparse design row `row` and scalar variance R (cycles^2).  Returns false on
// numerical failure so the caller can count skips and skip side effects.
bool applyScalarMadocaStateConstraint(
    ppp_shared::PPPState& filter_state,
    const MadocaArDesignRow& row,
    double innovation,
    double observation_variance_cycles_sq) {
    const int n = static_cast<int>(filter_state.state.size());
    if (filter_state.covariance.rows() != n ||
        filter_state.covariance.cols() != n) {
        return false;
    }

    VectorXd h = VectorXd::Zero(n);
    for (const auto& term : row.terms) {
        if (term.state_index < 0 || term.state_index >= n) {
            return false;
        }
        h(term.state_index) = term.coefficient;
    }
    if (h.isZero(0.0)) {
        return false;
    }

    const VectorXd Ph = filter_state.covariance * h;
    const double HPHt = h.dot(Ph);
    const double innovation_cov = HPHt + observation_variance_cycles_sq;
    if (!(innovation_cov > 0.0) || !std::isfinite(innovation_cov)) {
        return false;
    }

    const VectorXd K = Ph / innovation_cov;
    filter_state.state += K * innovation;
    filter_state.covariance.noalias() -= K * Ph.transpose();
    filter_state.covariance =
        0.5 * (filter_state.covariance + filter_state.covariance.transpose());
    return true;
}

}  // namespace

MadocaEwlConstraintSummary applyMadocaExtraWideLaneStateConstraint(
    ppp_shared::PPPState& filter_state,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const AmbiguityStateIndexProvider& state_index_provider,
    const AmbiguityWavelengthProvider& wavelength_provider,
    double observation_variance_cycles_sq,
    double frac_gate_cycles,
    double sigma_gate_cycles,
    bool debug_enabled) {
    MadocaEwlConstraintSummary summary;
    if (!state_index_provider || !wavelength_provider ||
        !(observation_variance_cycles_sq > 0.0) ||
        !std::isfinite(observation_variance_cycles_sq) ||
        !(frac_gate_cycles > 0.0) || !(sigma_gate_cycles > 0.0)) {
        return summary;
    }

    struct ReferenceCandidate {
        SatelliteId satellite;
        int lock_count = -1;
    };
    std::map<std::pair<GNSSSystem, int>, ReferenceCandidate> references;

    // EWL requires per-frequency states at freq 1 AND 2.  Pick the highest
    // lock_count sat per system group as reference.
    for (const auto& [satellite, ambiguity] : ambiguity_states) {
        if (state_index_provider(satellite, 1) < 0 ||
            state_index_provider(satellite, 2) < 0) {
            continue;
        }
        const auto group = ambiguityDdGroup(satellite);
        auto& ref = references[group];
        if (ambiguity.lock_count > ref.lock_count) {
            ref.satellite = satellite;
            ref.lock_count = ambiguity.lock_count;
        }
    }

    for (const auto& [satellite, ambiguity] : ambiguity_states) {
        if (state_index_provider(satellite, 1) < 0 ||
            state_index_provider(satellite, 2) < 0) {
            continue;
        }
        const auto group = ambiguityDdGroup(satellite);
        const auto ref_it = references.find(group);
        if (ref_it == references.end() || ref_it->second.lock_count < 0) {
            continue;
        }
        const SatelliteId& reference = ref_it->second.satellite;
        if (reference == satellite) {
            continue;
        }

        ++summary.candidates;

        const MadocaArDesignRow row = buildMadocaArDesignRow(
            satellite, reference, 1, 2,
            state_index_provider, wavelength_provider);
        if (!row.valid) {
            ++summary.skipped_invalid_row;
            continue;
        }

        const double sd_ewl_float =
            evaluateMadocaArDesignRow(row, filter_state.state);
        const double sd_ewl_var =
            madocaArDesignRowVariance(row, filter_state.covariance);
        const double sd_ewl_sigma =
            (std::isfinite(sd_ewl_var) && sd_ewl_var > 0.0)
                ? std::sqrt(sd_ewl_var)
                : std::numeric_limits<double>::infinity();

        if (!std::isfinite(sd_ewl_float)) {
            ++summary.skipped_invalid_row;
            continue;
        }

        const long long integer = static_cast<long long>(
            std::llround(sd_ewl_float));
        const double frac = sd_ewl_float - static_cast<double>(integer);

        if (sd_ewl_sigma > sigma_gate_cycles) {
            ++summary.skipped_large_sigma;
            continue;
        }
        if (std::abs(frac) > frac_gate_cycles) {
            ++summary.skipped_large_frac;
            continue;
        }

        const double innovation =
            static_cast<double>(integer) - sd_ewl_float;
        if (!applyScalarMadocaStateConstraint(
                filter_state, row, innovation, observation_variance_cycles_sq)) {
            ++summary.skipped_no_covariance;
            continue;
        }

        ++summary.applied_constraints;

        if (debug_enabled) {
            std::cerr << "[PPP-MADOCA-EWL] ref=" << reference.toString()
                      << " sat=" << satellite.toString()
                      << " ewl_int=" << integer
                      << " ewl_float=" << sd_ewl_float
                      << " frac=" << frac
                      << " sigma=" << sd_ewl_sigma << "\n";
        }
    }

    return summary;
}

MadocaWlConstraintSummary applyMadocaWideLaneStateConstraint(
    ppp_shared::PPPState& filter_state,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const AmbiguityStateIndexProvider& state_index_provider,
    const AmbiguityWavelengthProvider& wavelength_provider,
    double observation_variance_cycles_sq,
    bool debug_enabled) {
    MadocaWlConstraintSummary summary;
    if (!state_index_provider || !wavelength_provider ||
        !(observation_variance_cycles_sq > 0.0) ||
        !std::isfinite(observation_variance_cycles_sq)) {
        return summary;
    }

    struct ReferenceCandidate {
        SatelliteId satellite;
        int lock_count = -1;
    };
    std::map<std::pair<GNSSSystem, int>, ReferenceCandidate> references;

    for (const auto& [satellite, ambiguity] : ambiguity_states) {
        if (!ambiguity.wl_is_fixed) {
            continue;
        }
        if (state_index_provider(satellite, 0) < 0 ||
            state_index_provider(satellite, 1) < 0) {
            continue;
        }
        const auto group = ambiguityDdGroup(satellite);
        auto& ref = references[group];
        if (ambiguity.lock_count > ref.lock_count) {
            ref.satellite = satellite;
            ref.lock_count = ambiguity.lock_count;
        }
    }

    for (const auto& [satellite, ambiguity] : ambiguity_states) {
        if (!ambiguity.wl_is_fixed) {
            continue;
        }
        const auto group = ambiguityDdGroup(satellite);
        const auto ref_it = references.find(group);
        if (ref_it == references.end() || ref_it->second.lock_count < 0) {
            continue;
        }
        const SatelliteId& reference = ref_it->second.satellite;
        if (reference == satellite) {
            continue;
        }
        const auto ref_amb_it = ambiguity_states.find(reference);
        if (ref_amb_it == ambiguity_states.end() ||
            !ref_amb_it->second.wl_is_fixed) {
            continue;
        }

        ++summary.attempted_pairs;

        const MadocaArDesignRow row = buildMadocaArDesignRow(
            satellite, reference, 0, 1,
            state_index_provider, wavelength_provider);
        if (!row.valid) {
            ++summary.skipped_invalid_row;
            continue;
        }

        const int sd_fixed_cycles =
            ambiguity.wl_fixed_integer - ref_amb_it->second.wl_fixed_integer;
        const double sd_float_cycles =
            evaluateMadocaArDesignRow(row, filter_state.state);
        const double innovation =
            static_cast<double>(sd_fixed_cycles) - sd_float_cycles;

        // Reject pseudo-observations whose innovation is implausibly large
        // (> 5 cycles SD frac) — this suggests the per-frequency state is
        // not yet consistent with the MW-averaged WL integer.  Applying a
        // tight constraint in that regime poisons the filter.
        if (!std::isfinite(innovation) || std::abs(innovation) > 5.0) {
            ++summary.skipped_large_innovation;
            if (debug_enabled) {
                std::cerr << "[PPP-MADOCA-WL] skip large innovation ref="
                          << reference.toString()
                          << " sat=" << satellite.toString()
                          << " innov=" << innovation << "\n";
            }
            continue;
        }

        const int n = static_cast<int>(filter_state.state.size());
        if (filter_state.covariance.rows() != n ||
            filter_state.covariance.cols() != n) {
            ++summary.skipped_no_covariance;
            continue;
        }

        VectorXd h = VectorXd::Zero(n);
        for (const auto& term : row.terms) {
            if (term.state_index < 0 || term.state_index >= n) {
                h.setZero();
                break;
            }
            h(term.state_index) = term.coefficient;
        }
        if (h.isZero(0.0)) {
            ++summary.skipped_invalid_row;
            continue;
        }

        const VectorXd Ph = filter_state.covariance * h;
        const double HPHt = h.dot(Ph);
        const double innovation_cov = HPHt + observation_variance_cycles_sq;
        if (!(innovation_cov > 0.0) || !std::isfinite(innovation_cov)) {
            ++summary.skipped_no_covariance;
            continue;
        }

        const VectorXd K = Ph / innovation_cov;
        filter_state.state += K * innovation;
        filter_state.covariance.noalias() -= K * Ph.transpose();
        filter_state.covariance =
            0.5 * (filter_state.covariance + filter_state.covariance.transpose());

        ++summary.applied_constraints;

        if (debug_enabled) {
            std::cerr << "[PPP-MADOCA-WL] ref=" << reference.toString()
                      << " sat=" << satellite.toString()
                      << " sd_fix=" << sd_fixed_cycles
                      << " sd_float=" << sd_float_cycles
                      << " innov=" << innovation
                      << " HPHt=" << HPHt << "\n";
        }
    }

    return summary;
}

WlnlWideLaneFixSummary applyWideLaneFixes(
    const ppp_shared::PPPConfig& config,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const std::vector<SatelliteId>& satellites,
    bool debug_enabled) {
    WlnlWideLaneFixSummary summary;
    for (const auto& satellite : satellites) {
        auto ambiguity_it = ambiguity_states.find(satellite);
        if (ambiguity_it == ambiguity_states.end()) {
            continue;
        }
        auto& ambiguity = ambiguity_it->second;
        summary.max_mw_count = std::max(summary.max_mw_count, ambiguity.mw_count);
        if (ambiguity.wl_is_fixed) {
            ++summary.fixed_count;
            continue;
        }
        if (ambiguity.mw_count < config.wl_min_averaging_epochs) {
            continue;
        }
        const double mw_mean = ambiguity.mw_mean_cycles;
        const int wl_int = static_cast<int>(std::round(mw_mean));
        const double frac = mw_mean - wl_int;
        if (std::abs(frac) >= 0.25) {
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
    bool debug_enabled) {
    DdFixAttempt attempt;

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

    struct DdPair {
        int ref_idx = -1;
        int sat_idx = -1;
    };

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

    attempt.nb = static_cast<int>(dd_pairs.size());
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

    VectorXd dd_fixed = VectorXd::Zero(attempt.nb);
    if (!lambdaSearch(dd_float, dd_cov, dd_fixed, attempt.ratio)) {
        return attempt;
    }

    if (debug_enabled && excluded_real_satellites.empty()) {
        // Summarize DD float/frac/diag to understand LAMBDA ratio behavior.
        // Large mean_abs_frac indicates per-sat state bias that DD does not
        // cancel — blocks ratio>1.5 even with tight covariance.
        double sum_abs_frac = 0.0;
        double max_abs_frac = 0.0;
        for (int k = 0; k < attempt.nb; ++k) {
            const double frac = dd_float(k) - std::round(dd_float(k));
            sum_abs_frac += std::abs(frac);
            max_abs_frac = std::max(max_abs_frac, std::abs(frac));
        }
        std::cerr << "[PPP-AR-DD] nb=" << attempt.nb
                  << " ratio=" << attempt.ratio
                  << " mean_abs_frac=" << (sum_abs_frac / std::max(1, attempt.nb))
                  << " max_abs_frac=" << max_abs_frac << "\n";
    }

    attempt.required_ratio = config.ar_ratio_threshold;
    if (config.use_clas_osr_filter) {
        attempt.required_ratio =
            std::max(attempt.required_ratio, claslibRatioThresholdForNb(attempt.nb));
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

    // Holdamb for DD_IFLC / DD_PER_FREQ / DD_MADOCA_CASCADED: apply tight Kalman
    // pseudo-observation update on the DD ambiguity constraint instead of zeroing
    // the per-satellite cross-cov. Preserves amb-pos cross-cov so subsequent
    // epochs propagate the integer lock through the KF.
    const bool use_holdamb_iflc =
        config.enable_ppp_holdamb &&
        config.ar_method != ppp_shared::PPPConfig::ARMethod::DD_WLNL;

    if (use_holdamb_iflc) {
        const int nx = filter_state.total_states;
        constexpr double kHoldSigmaCycles = 1e-3;
        constexpr double kHoldVar = kHoldSigmaCycles * kHoldSigmaCycles;
        const double inn_gate_m = config.ppp_holdamb_innovation_gate_m > 0.0
            ? config.ppp_holdamb_innovation_gate_m
            : std::numeric_limits<double>::infinity();
        MatrixXd H = MatrixXd::Zero(attempt.nb, nx);
        VectorXd v = VectorXd::Zero(attempt.nb);
        int nv = 0;
        int gated = 0;
        for (int k = 0; k < attempt.nb; ++k) {
            const int ri = dd_pairs[static_cast<size_t>(k)].ref_idx;
            const int si = dd_pairs[static_cast<size_t>(k)].sat_idx;
            const int ref_state = state_indices[static_cast<size_t>(ri)];
            const int sat_state = state_indices[static_cast<size_t>(si)];
            const double scale_ri = scales[static_cast<size_t>(ri)];
            const double scale_si = scales[static_cast<size_t>(si)];
            if (scale_ri <= 0.0 || scale_si <= 0.0) continue;
            const double current_dd_cyc =
                attempt.state.state(ref_state) / scale_ri -
                attempt.state.state(sat_state) / scale_si;
            const double innovation = dd_fixed(k) - current_dd_cyc;
            const double inn_m = innovation * 0.5 * (scale_ri + scale_si);
            if (std::abs(inn_m) > inn_gate_m) { ++gated; continue; }
            H(nv, ref_state) = 1.0 / scale_ri;
            H(nv, sat_state) = -1.0 / scale_si;
            v(nv) = innovation;
            ++nv;

            auto& sat_amb = attempt.ambiguities[satellites[static_cast<size_t>(si)]];
            const double ref_cycles = attempt.state.state(ref_state) / scale_ri;
            const double fixed_sat_cycles = ref_cycles - dd_fixed(k);
            const double fixed_sat_m = fixed_sat_cycles * scale_si;
            sat_amb.float_value = fixed_sat_m;
            sat_amb.fixed_value = fixed_sat_m;
            sat_amb.is_fixed = true;
        }
        if (nv > 0) {
            const MatrixXd Hcrop = H.topRows(nv);
            const VectorXd vcrop = v.head(nv);
            const MatrixXd Rcrop = MatrixXd::Identity(nv, nv) * kHoldVar;
            const MatrixXd PHt = attempt.state.covariance * Hcrop.transpose();
            const MatrixXd S = Hcrop * PHt + Rcrop;
            Eigen::LLT<MatrixXd> llt(S);
            if (llt.info() == Eigen::Success) {
                const MatrixXd K = PHt * llt.solve(MatrixXd::Identity(nv, nv));
                attempt.state.state += K * vcrop;
                attempt.state.covariance.noalias() -= K * PHt.transpose();
                attempt.state.covariance =
                    0.5 * (attempt.state.covariance + attempt.state.covariance.transpose());
                if (debug_enabled) {
                    std::cerr << "[PPP-AR-HOLDAMB] DD_IFLC update nv=" << nv
                              << " gated=" << gated
                              << " ||v||=" << vcrop.norm()
                              << " ||K*v||=" << (K * vcrop).norm() << "\n";
                }
            } else if (debug_enabled) {
                std::cerr << "[PPP-AR-HOLDAMB] LLT failure, skipping update\n";
            }
        }
    } else {
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
    bool debug_enabled) {
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
            debug_enabled);
    };

    DdFixAttempt best_attempt = try_dd_fix({});
    if (best_attempt.fixed || real_satellite_elevations.empty()) {
        return best_attempt;
    }
    const bool par_allowed =
        config.use_clas_osr_filter ||
        config.ar_method == ppp_shared::PPPConfig::ARMethod::DD_MADOCA_CASCADED;
    if (!par_allowed) {
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

    attempt.nb = static_cast<int>(dd_pairs.size());
    if (attempt.nb < 4) {
        if (debug_enabled) {
            std::cerr << "[PPP-WLNL] insufficient DD NL pairs: " << attempt.nb << "\n";
        }
        return attempt;
    }

    VectorXd dd_nl_float = VectorXd::Zero(attempt.nb);
    MatrixXd dd_nl_cov = MatrixXd::Identity(attempt.nb, attempt.nb) * 1.0;

    int valid_dd = 0;
    for (int k = 0; k < attempt.nb; ++k) {
        const int ri = dd_pairs[static_cast<size_t>(k)].ref_idx;
        const int si = dd_pairs[static_cast<size_t>(k)].sat_idx;
        const auto ref_it = nl_info.find(satellites[static_cast<size_t>(ri)]);
        const auto sat_it = nl_info.find(satellites[static_cast<size_t>(si)]);
        if (ref_it == nl_info.end() || !ref_it->second.valid ||
            sat_it == nl_info.end() || !sat_it->second.valid) {
            dd_nl_float(k) = 0.0;
            dd_nl_cov(k, k) = 1e10;
            continue;
        }
        dd_nl_float(k) = ref_it->second.nl_ambiguity_cycles - sat_it->second.nl_ambiguity_cycles;
        ++valid_dd;

        if (debug_enabled && k < 5) {
            const double frac = dd_nl_float(k) - std::round(dd_nl_float(k));
            std::cerr << "[PPP-WLNL] DD NL pair " << k
                      << " ref=" << satellites[static_cast<size_t>(ri)].toString()
                      << " sat=" << satellites[static_cast<size_t>(si)].toString()
                      << " nl=" << dd_nl_float(k) << " frac=" << frac << "\n";
        }
    }

    if (valid_dd < 4) {
        if (debug_enabled) {
            std::cerr << "[PPP-WLNL] insufficient valid DD NL: " << valid_dd << "\n";
        }
        return attempt;
    }

    // PAR-aware LAMBDA: greedily exclude DD pairs with worst |frac| when the
    // initial ratio fails. The excluded vector tracks which pair indices are
    // dropped; downstream holdamb/fix-marking loops skip them.
    std::vector<bool> dd_pair_excluded(attempt.nb, false);
    VectorXd dd_nl_fixed = VectorXd::Zero(attempt.nb);
    int active_nb = attempt.nb;

    auto run_subset_lambda = [&](double& out_ratio) -> bool {
        std::vector<int> kept;
        kept.reserve(attempt.nb);
        for (int k = 0; k < attempt.nb; ++k) {
            if (!dd_pair_excluded[static_cast<size_t>(k)]) kept.push_back(k);
        }
        if (static_cast<int>(kept.size()) < 4) return false;
        const int sn = static_cast<int>(kept.size());
        VectorXd sub_float(sn);
        MatrixXd sub_cov = MatrixXd::Identity(sn, sn);
        for (int i = 0; i < sn; ++i) sub_float(i) = dd_nl_float(kept[static_cast<size_t>(i)]);
        VectorXd sub_fixed(sn);
        if (!lambdaSearch(sub_float, sub_cov, sub_fixed, out_ratio)) return false;
        for (int i = 0; i < sn; ++i) dd_nl_fixed(kept[static_cast<size_t>(i)]) = sub_fixed(i);
        active_nb = sn;
        return true;
    };

    if (!run_subset_lambda(attempt.ratio)) {
        if (debug_enabled) {
            std::cerr << "[PPP-WLNL] NL lambda search failed, nb=" << attempt.nb << "\n";
        }
        return attempt;
    }
    if (config.enable_wlnl_par &&
        (!std::isfinite(attempt.ratio) || attempt.ratio < config.ar_ratio_threshold)) {
        const int max_excl = std::max(0, std::min(config.wlnl_par_max_exclusions, attempt.nb - 4));
        const double frac_thr = config.wlnl_par_exclude_frac_threshold;
        for (int iter = 0; iter < max_excl; ++iter) {
            int worst_k = -1;
            double worst_frac = frac_thr;
            for (int k = 0; k < attempt.nb; ++k) {
                if (dd_pair_excluded[static_cast<size_t>(k)]) continue;
                const double frac = std::abs(dd_nl_float(k) - std::round(dd_nl_float(k)));
                if (frac > worst_frac) { worst_frac = frac; worst_k = k; }
            }
            if (worst_k < 0) break;
            dd_pair_excluded[static_cast<size_t>(worst_k)] = true;
            double new_ratio = 0.0;
            if (!run_subset_lambda(new_ratio)) {
                dd_pair_excluded[static_cast<size_t>(worst_k)] = false;
                break;
            }
            attempt.ratio = new_ratio;
            if (debug_enabled) {
                std::cerr << "[PPP-WLNL-PAR] excluded pair " << worst_k
                          << " worst_frac=" << worst_frac
                          << " new_ratio=" << new_ratio
                          << " active_nb=" << active_nb << "\n";
            }
            if (std::isfinite(new_ratio) && new_ratio >= config.ar_ratio_threshold) break;
        }
    }
    if (!std::isfinite(attempt.ratio) || attempt.ratio < config.ar_ratio_threshold) {
        if (debug_enabled) {
            std::cerr << "[PPP-WLNL] NL ratio reject: nb=" << attempt.nb
                      << " ratio=" << attempt.ratio
                      << " threshold=" << config.ar_ratio_threshold << "\n";
        }
        return attempt;
    }

    constexpr double hold_variance = 0.001 * 0.001;
    const int nx = filter_state.total_states;
    int nv = 0;
    MatrixXd H = MatrixXd::Zero(attempt.nb, nx);
    VectorXd v = VectorXd::Zero(attempt.nb);
    MatrixXd R = MatrixXd::Zero(attempt.nb, attempt.nb);

    for (int k = 0; k < attempt.nb; ++k) {
        if (dd_pair_excluded[static_cast<size_t>(k)]) continue;
        const int ri = dd_pairs[static_cast<size_t>(k)].ref_idx;
        const int si = dd_pairs[static_cast<size_t>(k)].sat_idx;
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
        const double fixed_dd_m =
            dd_nl_fixed(k) * sat_nl_it->second.lambda_nl_m +
            (ref_amb.wl_fixed_integer - sat_amb.wl_fixed_integer) *
                sat_nl_it->second.beta * sat_nl_it->second.lambda_wl_m;

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
        if (dd_pair_excluded[static_cast<size_t>(k)]) continue;
        const int ri = dd_pairs[static_cast<size_t>(k)].ref_idx;
        const int si = dd_pairs[static_cast<size_t>(k)].sat_idx;
        const auto& ref_ambiguity = ambiguity_states.at(satellites[static_cast<size_t>(ri)]);
        if (!ref_ambiguity.nl_is_fixed) {
            continue;
        }
        auto& sat_ambiguity = ambiguity_states[satellites[static_cast<size_t>(si)]];
        sat_ambiguity.is_fixed = true;
        sat_ambiguity.nl_is_fixed = true;
        sat_ambiguity.nl_fixed_cycles = ref_ambiguity.nl_fixed_cycles - dd_nl_fixed(k);
    }
    attempt.nb = active_nb;  // report only the kept pair count

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
            double trop_delay = fixed_observation.modeled_trop_delay_m;
            if (estimate_troposphere) {
                if (std::isfinite(fixed_observation.modeled_trop_delay_m) &&
                    fixed_observation.modeled_trop_delay_m > 0.0 &&
                    std::isfinite(fixed_observation.modeled_zenith_trop_delay_m) &&
                    fixed_observation.modeled_zenith_trop_delay_m > 0.0 &&
                    std::isfinite(fixed_observation.trop_mapping) &&
                    fixed_observation.trop_mapping > 0.0) {
                    trop_delay = fixed_observation.modeled_trop_delay_m +
                        fixed_observation.trop_mapping *
                            (trop_zenith - fixed_observation.modeled_zenith_trop_delay_m);
                } else {
                    trop_delay = fixed_observation.trop_mapping * trop_zenith;
                }
            }
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
