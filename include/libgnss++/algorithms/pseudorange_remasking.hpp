#pragma once
#include <libgnss++/algorithms/fgo.hpp>
#include <libgnss++/algorithms/observable_upstream_preprocessing.hpp>
#include <algorithm>
#include <map>
#include <set>
#include <stdexcept>
#include <tuple>

namespace libgnss::pseudorange_remasking {
struct Selection {
    std::vector<std::size_t> pool_indices;
    std::size_t recovered = 0, removed = 0, unchanged = 0;
};

// Pure in-memory selection: no mutation until the caller accepts the complete
// result. Measurements, satellite geometry and sigmas remain untouched.
inline Selection select(const FGOProcessor::FGOProblem& problem) {
    namespace upstream = observable_upstream;
    using Key = std::tuple<std::size_t, SatelliteId, SignalType>;
    using Group = std::pair<GNSSSystem, upstream::ObservationBand>;
    const auto& pool = problem.native_pseudorange_remasking_pool;
    if (pool.empty() || problem.epochs.empty())
        throw std::invalid_argument("empty pseudorange re-masking input");
    for (std::size_t i=0; i<problem.epochs.size(); ++i) {
        const auto& epoch = problem.epochs[i];
        if (!epoch.position_ecef.allFinite() ||
            !epoch.receiver_clock_bias_is_meters ||
            !std::isfinite(epoch.receiver_clock_bias_m) ||
            !std::isfinite(epoch.time.tow) ||
            epoch.time.tow < 0.0 || epoch.time.tow >= 604800.0 ||
            (i && !(epoch.time - problem.epochs[i-1].time > 0)))
            throw std::invalid_argument("invalid re-masking epoch state");
    }
    const auto key = [](const auto& row) {
        return Key{row.epoch_index,row.satellite,row.signal};
    };
    std::set<Key> pool_keys, old_keys;
    std::map<Key,const FGOProcessor::PseudorangeFactor*> pool_rows;
    std::map<Group,std::vector<double>> groups;
    std::vector<double> residuals;
    for (const auto& row : pool) {
        const auto band = upstream::bandForSignal(row.signal);
        if (row.epoch_index >= problem.epochs.size() ||
            !pool_keys.insert(key(row)).second ||
            band == upstream::ObservationBand::Unknown ||
            !row.satellite_position_ecef.allFinite() ||
            !std::isfinite(row.corrected_pseudorange_m) ||
            !upstream::finitePositive(row.sigma_m))
            throw std::invalid_argument("invalid or duplicate re-masking pool row");
        const auto& epoch = problem.epochs[row.epoch_index];
        const double range = (row.satellite_position_ecef-epoch.position_ecef).norm();
        const double residual = row.corrected_pseudorange_m-range-epoch.receiver_clock_bias_m;
        if (!(range > 0) || !std::isfinite(residual))
            throw std::invalid_argument("invalid re-masking range/residual");
        residuals.push_back(residual);
        pool_rows.emplace(key(row), &row);
        groups[{row.clock_group,band}].push_back(residual);
    }
    for (const auto& row : problem.pseudorange_factors) {
        if (!pool_keys.count(key(row)) || !old_keys.insert(key(row)).second)
            throw std::invalid_argument("old P selection is not a unique pool subset");
        const auto& original = *pool_rows.at(key(row));
        if (row.corrected_pseudorange_m != original.corrected_pseudorange_m ||
            row.sigma_m != original.sigma_m || row.clock_group != original.clock_group ||
            row.satellite_position_ecef != original.satellite_position_ecef)
            throw std::invalid_argument("P row changed after pre-mask pool capture");
    }
    std::map<Group,double> medians;
    for (auto& [group,values] : groups) {
        std::sort(values.begin(),values.end());
        const std::size_t n=values.size();
        const double median=n%2 ? values[n/2] : values[n/2-1]*0.5+values[n/2]*0.5;
        if (!std::isfinite(median))
            throw std::invalid_argument("nonfinite re-masking median");
        medians[group]=median;
    }
    Selection result;
    for (std::size_t i=0; i<pool.size(); ++i) {
        const auto& row=pool[i];
        const auto band=upstream::bandForSignal(row.signal);
        const bool accepted=upstream::acceptsCenteredPseudorangeResidual(
            residuals[i],medians.at({row.clock_group,band}),upstream::residualThreshold(band,'P'));
        const bool old=old_keys.count(key(row)) != 0;
        if (accepted) {
            result.pool_indices.push_back(i);
            if (old) ++result.unchanged; else ++result.recovered;
        } else if (old) ++result.removed;
    }
    return result;
}
}  // namespace libgnss::pseudorange_remasking
