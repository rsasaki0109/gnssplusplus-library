// Actual upstream mask, synthetic raw observations; no route or truth input.
#include <libgnss++/algorithms/observable_upstream_preprocessing.hpp>
#include <iostream>

std::size_t masked(const std::vector<double>& errors) {
    std::vector<libgnss::ObservationData> epochs(errors.size());
    for (std::size_t i = 0; i < errors.size(); ++i) {
        epochs[i].time.week = 2200;
        epochs[i].time.tow = 100. + i;
        libgnss::Observation row;
        row.satellite = libgnss::SatelliteId(libgnss::GNSSSystem::GPS, 1);
        row.signal = libgnss::SignalType::GPS_L1CA;
        row.has_pseudorange = row.has_doppler = true;
        row.pseudorange = 20000000. + errors[i];
        row.doppler = 0.;
        epochs[i].observations.push_back(row);
    }
    std::vector<libgnss::observable_upstream::EpochMask> masks;
    std::size_t p = 0, l = 0;
    libgnss::observable_upstream::applyAdjacentMasks(epochs, "pixel5", masks, p, l);
    return p;
}

int main() {
    const auto impulse = masked({0., 100., 0.});
    const auto plateau = masked({100., 100., 100.});
    const auto clean = masked({0., 0., 0.});
    std::cout << "isolated_bad_code_rows=1 masked_rows=" << impulse
              << " constant_bad_code_rows=3 masked_rows=" << plateau
              << " clean_masked_rows=" << clean << '\n';
    return impulse == 3 && plateau == 0 && clean == 0 ? 0 : 1;
}
