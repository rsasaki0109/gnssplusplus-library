#pragma once
#include <libgnss++/algorithms/doppler_contract.hpp>
#include <libgnss++/algorithms/raw_p_seed.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace libgnss::smartphone_temporal {
inline bool samsungClockRecipe(const std::string& phone) {
    return phone == "sm-a226b" || phone == "sm-a505g" || phone == "sm-a600t" ||
           phone == "sm-a325f" || phone == "sm-a217m" || phone == "sm-a205u" ||
           phone == "samsunga325g" || phone == "samsunga32" || phone == "sm-a505u";
}

struct SamsungClockDrift {
    bool ok = false;
    std::vector<double> medians, values;
    std::vector<std::size_t> rows;
    // Provenance for the shared cleanup step (also used for raw phone drift).
    std::vector<std::size_t> left_sources, right_sources;
    std::vector<double> right_weights;
    std::size_t jump_masks = 0, filled_values = 0;
};

// preprocessing.m: mask both sides of adjacent jumps >50; when any jump
// exists, fill all missing values linearly (including endpoint extrapolation).
// Otherwise only nearest-fill missing values. A tie selects the next sample.
inline SamsungClockDrift finishSamsungDrift(std::vector<double> medians) {
    SamsungClockDrift r;
    r.medians = medians;
    r.values = std::move(medians);
    auto& d = r.values;
    r.left_sources.resize(d.size());
    r.right_sources.resize(d.size());
    r.right_weights.assign(d.size(), 0.0);
    for (std::size_t i = 0; i < d.size(); ++i)
        r.left_sources[i] = r.right_sources[i] = i;
    std::vector<bool> masks(d.size(), false);
    for (std::size_t i = 1; i < d.size(); ++i)
        if (std::isfinite(d[i]) && std::isfinite(d[i - 1]) &&
            std::abs(d[i] - d[i - 1]) > 50.0) masks[i - 1] = masks[i] = true;
    std::vector<std::size_t> known;
    for (std::size_t i = 0; i < d.size(); ++i) {
        if (masks[i]) {
            d[i] = std::numeric_limits<double>::quiet_NaN();
            ++r.jump_masks;
        }
        if (std::isfinite(d[i])) known.push_back(i);
    }
    if (known.empty()) return r;
    for (std::size_t i = 0; i < d.size(); ++i) {
        if (std::isfinite(d[i])) continue;
        const auto upper = std::lower_bound(known.begin(), known.end(), i);
        if (r.jump_masks && known.size() >= 2) {
            const auto right = upper == known.begin() ? known[1]
                             : upper == known.end() ? known.back() : *upper;
            const auto left = upper == known.begin() ? known.front()
                            : upper == known.end() ? known[known.size() - 2] : *(upper - 1);
            r.left_sources[i] = left;
            r.right_sources[i] = right;
            r.right_weights[i] = (static_cast<double>(i) - left) / (right - left);
            d[i] = d[left] + (static_cast<double>(i) - left) / (right - left) *
                               (d[right] - d[left]);
        } else {
            const auto source = upper == known.begin() ? known.front()
                              : upper == known.end() ? known.back()
                              : i - *(upper - 1) < *upper - i ? *(upper - 1) : *upper;
            r.left_sources[i] = r.right_sources[i] = source;
            d[i] = d[source];
        }
        ++r.filled_values;
    }
    r.ok = std::all_of(d.begin(), d.end(), [](double v) { return std::isfinite(v); });
    return r;
}

// Source-style GPS L1 median using only the same-invocation native trajectory.
// Satellite modeling deliberately uses the existing native Doppler convention;
// this is not a claim of paired upstream satellite-state equivalence.
inline SamsungClockDrift samsungClockDrift(
    const std::vector<ObservationData>& epochs, const NavigationData& nav,
    const std::vector<raw_p_seed::RawPNoDopplerSeed>& seeds) {
    if (epochs.empty() || epochs.size() != seeds.size()) return {};
    std::vector<double> medians;
    std::vector<std::size_t> counts;
    for (std::size_t i = 0; i < epochs.size(); ++i) {
        const auto& e = epochs[i];
        const auto& s = seeds[i];
        if (s.epoch_index != i || s.raw_utc_time_millis != e.raw_utc_time_millis ||
            (s.time - e.time) != 0.0 || !s.position_ecef.allFinite() ||
            !s.velocity_ecef_mps.allFinite()) return {};
        std::vector<double> residuals;
        for (const auto& o : e.observations) {
            if (o.satellite.system != GNSSSystem::GPS || o.signal != SignalType::GPS_L1CA ||
                !o.valid || !o.has_pseudorange || !o.has_doppler ||
                !o.has_pseudorange_rate_mps || !std::isfinite(o.pseudorange) ||
                o.pseudorange < 1e7 || o.pseudorange > 4e7 ||
                !std::isfinite(o.snr) || o.snr < 15) continue;
            auto tx = e.time - o.pseudorange / constants::SPEED_OF_LIGHT;
            Vector3d sp, sv, los;
            double cb = 0, cd = 0, sat_rate = 0;
            if (!nav.calculateSatelliteState(o.satellite, tx, sp, sv, cb, cd)) continue;
            tx = tx - cb;
            if (!nav.calculateSatelliteState(o.satellite, tx, sp, sv, cb, cd) ||
                !doppler_contract::knownSatelliteRangeRate(sp, sv, s.position_ecef,
                                                           false, los, sat_rate)) continue;
            const double residual = doppler_contract::receiverOnlyResidual(
                o.pseudorange_rate_mps, sat_rate, cd) + los.dot(s.velocity_ecef_mps);
            if (std::isfinite(residual)) residuals.push_back(residual);
        }
        counts.push_back(residuals.size());
        double median = std::numeric_limits<double>::quiet_NaN();
        if (!residuals.empty()) {
            std::sort(residuals.begin(), residuals.end());
            median = residuals[residuals.size() / 2];
            if (residuals.size() % 2 == 0)
                median = 0.5 * (median + residuals[residuals.size() / 2 - 1]);
        }
        medians.push_back(median);
    }
    auto result = finishSamsungDrift(std::move(medians));
    result.rows = std::move(counts);
    return result;
}
}  // namespace libgnss::smartphone_temporal
