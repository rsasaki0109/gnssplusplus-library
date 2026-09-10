#include <libgnss++/io/android_raw_gnss.hpp>
#include <libgnss++/algorithms/matched_pseudorange_doppler.hpp>
#include <libgnss++/algorithms/code_edge_candidates.hpp>
#include <iostream>
#include <map>

int main(int argc, char** argv) {
    if (argc != 2) return 2;
    libgnss::io::AndroidRawGnssConfig config;
    config.verify_enriched_pseudorange = false;
    config.device_model = "pixel5";
    libgnss::io::AndroidRawGnssResult raw;
    std::string error;
    if (!libgnss::io::loadAndroidRawGnssCsv(argv[1], config, raw, error)) return 3;
    namespace up = libgnss::observable_upstream;
    const auto& epochs = raw.observations.epochs;
    const auto& clocks = raw.epoch_hardware_clock_discontinuity_count;
    if (clocks.size() != epochs.size()) return 4;
    std::vector<up::EpochMask> masks;
    std::size_t p = 0, l = 0;
    up::applyAdjacentMasks(epochs, "pixel5", masks, p, l);
    std::size_t triples = 0, double_bad = 0, reversing = 0, masked_neighbors = 0;
    using Identity = std::tuple<std::size_t, libgnss::SatelliteId, libgnss::SignalType>;
    std::set<Identity> centers, neighbors;
    std::size_t clock_discontinuous_windows = 0;
    for (std::size_t i = 1; i + 1 < epochs.size(); ++i) {
        if (clocks[i-1] != clocks[i] || clocks[i] != clocks[i+1]) {
            ++clock_discontinuous_windows;
            continue;
        }
        for (const auto& row : epochs[i].observations) {
            const auto incoming = libgnss::matchedPseudorangeDoppler(
                epochs[i-1], epochs[i], row.satellite, row.signal,
                epochs[i].time - epochs[i-1].time);
            const auto outgoing = libgnss::matchedPseudorangeDoppler(
                epochs[i], epochs[i+1], row.satellite, row.signal,
                epochs[i+1].time - epochs[i].time);
            if (!incoming || !outgoing) continue;
            ++triples;
            const double threshold = up::pairThreshold(up::bandForSignal(row.signal), 'P');
            if (std::abs(*incoming) <= threshold || std::abs(*outgoing) <= threshold) continue;
            ++double_bad;
            if (std::signbit(*incoming) == std::signbit(*outgoing)) continue;
            ++reversing;
            centers.emplace(i, row.satellite, row.signal);
            neighbors.emplace(i - 1, row.satellite, row.signal);
            neighbors.emplace(i + 1, row.satellite, row.signal);
            const up::ObservationKey key{row.satellite, row.signal};
            if (masks[i-1].pseudorange.count(key) && masks[i+1].pseudorange.count(key))
                ++masked_neighbors;
        }
    }
    std::size_t also_centers = 0, outside_edge_bad = 0, outside_edge_missing = 0;
    std::size_t supported_disjoint = 0, no_outside_edge = 0;
    for (const auto& identity : neighbors) {
        if (centers.count(identity)) ++also_centers;
        const auto& [i, satellite, signal] = identity;
        bool bad = false, missing = false;
        std::size_t supported_edges = 0;
        const double threshold = up::pairThreshold(up::bandForSignal(signal), 'P');
        for (int direction : {-1, 1}) {
            if ((direction < 0 && i == 0) || (direction > 0 && i + 1 >= epochs.size())) {
                missing = true;
                continue;
            }
            const std::size_t j = direction < 0 ? i - 1 : i + 1;
            if (centers.count(Identity{j, satellite, signal})) continue;
            if (clocks[i] != clocks[j]) { missing = true; continue; }
            const std::size_t a = std::min(i, j), b = std::max(i, j);
            const auto edge = libgnss::matchedPseudorangeDoppler(
                epochs[a], epochs[b], satellite, signal, epochs[b].time - epochs[a].time);
            if (!edge) missing = true;
            else if (std::abs(*edge) > threshold) bad = true;
            else ++supported_edges;
        }
        outside_edge_bad += bad;
        outside_edge_missing += missing;
        if (supported_edges == 0) ++no_outside_edge;
        if (!centers.count(identity) && !bad && !missing && supported_edges > 0)
            ++supported_disjoint;
    }
    std::cout << "unique_centers=" << centers.size() << " unique_neighbors=" << neighbors.size()
              << " neighbors_also_centers=" << also_centers
              << " neighbors_with_bad_noncenter_edge=" << outside_edge_bad
              << " neighbors_with_missing_noncenter_edge=" << outside_edge_missing << '\n';
    const auto reusable = libgnss::codeEdgeCandidates(epochs, clocks);
    if (reusable.size() != supported_disjoint) return 5;
    std::cout << "reusable_candidate_count=" << reusable.size() << '\n';
    std::cout << "disjoint_neighbors_with_present_consistent_outside_edges=" << supported_disjoint
              << " neighbors_without_consistent_outside_edge=" << no_outside_edge
              << " clock_discontinuous_windows=" << clock_discontinuous_windows << '\n';
    std::cout << "epochs=" << epochs.size() << " pd_masked_rows=" << p
              << " valid_triples=" << triples << " both_edges_exceed=" << double_bad
              << " opposite_sign=" << reversing
              << " opposite_sign_with_both_neighbors_masked=" << masked_neighbors << '\n';
}
