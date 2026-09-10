#pragma once
#include <libgnss++/algorithms/matched_pseudorange_doppler.hpp>
#include <stdexcept>

namespace libgnss {
// Input-epoch ordinal, satellite, signal. Caller must map this exact ordinal
// to immutable raw identity before crossing any epoch filtering boundary.
using CodeEdgeIdentity = std::tuple<std::size_t, SatelliteId, SignalType>;

// Diagnostic candidates only, not a clean-code classifier or admission rule.
inline std::set<CodeEdgeIdentity> codeEdgeCandidates(
    const std::vector<ObservationData>& epochs, const std::vector<int>& clocks) {
    namespace up = observable_upstream;
    if (clocks.size() != epochs.size())
        throw std::invalid_argument("Code edge candidates require clock coverage");
    std::set<CodeEdgeIdentity> centers, neighbors, supported;
    auto edge = [&](std::size_t a, std::size_t b, SatelliteId sat, SignalType sig) {
        return matchedPseudorangeDoppler(epochs[a], epochs[b], sat, sig,
                                        epochs[b].time - epochs[a].time);
    };
    for (std::size_t i = 1; i + 1 < epochs.size(); ++i) {
        if (clocks[i-1] != clocks[i] || clocks[i] != clocks[i+1]) continue;
        for (const auto& row : epochs[i].observations) {
            const auto band = up::bandForSignal(row.signal);
            if (band == up::ObservationBand::Unknown) continue;
            const auto before = edge(i-1, i, row.satellite, row.signal);
            const auto after = edge(i, i+1, row.satellite, row.signal);
            const double threshold = up::pairThreshold(band, 'P');
            if (!before || !after || std::abs(*before) <= threshold ||
                std::abs(*after) <= threshold || std::signbit(*before) == std::signbit(*after)) continue;
            centers.emplace(i, row.satellite, row.signal);
            neighbors.emplace(i-1, row.satellite, row.signal);
            neighbors.emplace(i+1, row.satellite, row.signal);
        }
    }
    for (const auto& identity : neighbors) {
        if (centers.count(identity)) continue;
        const auto& [i, sat, sig] = identity;
        bool valid = true;
        std::size_t witnesses = 0;
        for (int direction : {-1, 1}) {
            if ((direction < 0 && i == 0) || (direction > 0 && i+1 >= epochs.size())) {
                valid = false; continue;
            }
            const std::size_t j = direction < 0 ? i-1 : i+1;
            if (centers.count(CodeEdgeIdentity{j, sat, sig})) continue;
            if (clocks[i] != clocks[j]) { valid = false; continue; }
            const auto value = edge(std::min(i,j), std::max(i,j), sat, sig);
            if (!value || std::abs(*value) > up::pairThreshold(up::bandForSignal(sig), 'P'))
                valid = false;
            else ++witnesses;
        }
        if (valid && witnesses > 0) supported.insert(identity);
    }
    return supported;
}
}
