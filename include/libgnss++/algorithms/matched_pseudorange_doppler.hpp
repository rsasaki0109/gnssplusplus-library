#pragma once
#include <libgnss++/algorithms/observable_upstream_preprocessing.hpp>
#include <libgnss++/algorithms/rejected_residual_summary.hpp>
#include <optional>

namespace libgnss {
// Diagnostic only. Ambiguous identities and absent pairs are never zero-filled.
inline std::optional<double> matchedPseudorangeDoppler(
    const ObservationData& previous, const ObservationData& current,
    const SatelliteId& satellite, SignalType signal, double dt_s) {
    if (!std::isfinite(dt_s) || dt_s <= 0 || dt_s > 1.5) return std::nullopt;
    auto unique = [&](const ObservationData& epoch) -> const Observation* {
        const Observation* found = nullptr;
        for (const auto& row : epoch.observations) {
            if (row.satellite == satellite && row.signal == signal) {
                if (found) return nullptr;
                found = &row;
            }
        }
        return found;
    };
    const auto* old = unique(previous);
    const auto* now = unique(current);
    if (!old || !now) return std::nullopt;
    for (const auto* row : {old, now}) {
        if (!row->has_pseudorange || !row->has_doppler ||
            !std::isfinite(row->pseudorange) || row->pseudorange <= 0 ||
            !std::isfinite(row->doppler)) return std::nullopt;
    }
    const double wavelength = signalWavelengthMeters(*now);
    if (!observable_upstream::finitePositive(wavelength)) return std::nullopt;
    const double value = observable_upstream::pseudorangeDopplerDifference(
        old->pseudorange, now->pseudorange, old->doppler, now->doppler,
        wavelength, dt_s);
    return std::isfinite(value) ? std::optional<double>(value) : std::nullopt;
}

struct MatchedPseudorangeDopplerSummary {
    std::size_t missing = 0;
    RejectedResidualSummary values;
    void observe(std::optional<double> value) {
        if (value) values.observe(*value);
        else ++missing;
    }
};
}
