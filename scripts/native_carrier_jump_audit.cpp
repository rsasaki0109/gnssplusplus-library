#include <libgnss++/io/android_raw_gnss.hpp>
#include <libgnss++/algorithms/observable_upstream_preprocessing.hpp>
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
    std::vector<up::EpochMask> masks;
    std::size_t p_rejected = 0, l_rejected = 0;
    up::applyAdjacentMasks(epochs, "pixel5", masks, p_rejected, l_rejected);
    using Key = std::pair<libgnss::SatelliteId, libgnss::SignalType>;
    struct Prior { std::size_t epoch; const libgnss::Observation* row; };
    std::map<Key, Prior> previous;
    std::size_t pairs = 0, jumps = 0, already_masked = 0, bridges = 0;
    std::size_t raw_cmc_jumps = 0, cmc_with_doppler = 0, cmc_ld_consistent = 0;
    double cmc_ld_max = 0.0;
    for (std::size_t i = 0; i < epochs.size(); ++i) {
        for (const auto& row : epochs[i].observations) {
            const Key key{row.satellite, row.signal};
            auto it = previous.find(key);
            if (it != previous.end()) {
                const auto j = it->second.epoch;
                const auto& old = *it->second.row;
                const double dt = epochs[i].time - epochs[j].time;
                if (j + 1 < i && dt > 0 && dt <= 1.5) ++bridges;
                if (j + 1 == i && old.has_carrier_phase && row.has_carrier_phase &&
                    std::isfinite(old.carrier_phase) && std::isfinite(row.carrier_phase)) {
                    ++pairs;
                    // Raw proxy only: final FGO uses corrected P and L and
                    // further geometry/quality admission. Do not equate counts.
                    const double wavelength = libgnss::signalWavelengthMeters(row);
                    if (dt > 0 && dt <= 1.5 && old.has_pseudorange && row.has_pseudorange &&
                        std::isfinite(old.pseudorange) && std::isfinite(row.pseudorange) &&
                        std::isfinite(wavelength) && wavelength > 0 &&
                        std::abs((row.pseudorange-old.pseudorange) -
                                 (row.carrier_phase-old.carrier_phase)*wavelength) > 10.0) {
                        ++raw_cmc_jumps;
                        if (old.has_doppler && row.has_doppler &&
                            std::isfinite(old.doppler) && std::isfinite(row.doppler)) {
                            const double ld = std::abs(up::carrierDopplerDifference(
                                old.carrier_phase, row.carrier_phase,
                                old.doppler, row.doppler, wavelength, dt));
                            if (std::isfinite(ld)) {
                                ++cmc_with_doppler;
                                if (ld <= 1.5) ++cmc_ld_consistent;
                                cmc_ld_max = std::max(cmc_ld_max, ld);
                            }
                        }
                    }
                    if (std::abs(row.carrier_phase - old.carrier_phase) > 20000) {
                        ++jumps;
                        if (masks[i].carrier.count(key)) ++already_masked;
                    }
                }
            }
            previous[key] = {i, &row};
        }
    }
    std::cout << "{\"epochs\":" << epochs.size()
              << ",\"quality_admitted_adjacent_carrier_pairs\":" << pairs
              << ",\"cycle_jump_current_endpoints\":" << jumps
              << ",\"jump_current_endpoints_already_ld_masked\":" << already_masked
              << ",\"nonadjacent_identity_pairs_within_1_5s\":" << bridges
              << ",\"ld_masked_endpoints\":" << l_rejected << "}\n";
    std::cout << "{\"raw_cmc_over_10m_pairs\":" << raw_cmc_jumps
              << ",\"with_finite_doppler_witness\":" << cmc_with_doppler
              << ",\"ld_within_1_5m\":" << cmc_ld_consistent
              << ",\"max_abs_ld_m\":" << cmc_ld_max << "}\n";
}
