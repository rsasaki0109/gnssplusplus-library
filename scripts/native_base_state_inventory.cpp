#include <libgnss++/io/rinex.hpp>
#include <libgnss++/algorithms/source_epoch_states.hpp>
#include <iostream>
#include <map>
int main(int argc, char** argv) {
    if (argc != 3) return 2;
    libgnss::io::RINEXReader base, nav_reader;
    base.setSourceHeaderTrackingFilter(true);
    base.setPreserveAdditionalFrequencyBands(true);
    if (!base.open(argv[1]) || !nav_reader.open(argv[2])) return 3;
    libgnss::io::RINEXReader::RINEXHeader header;
    if (!base.readHeader(header) || header.version < 3 || header.version >= 4) return 4;
    libgnss::NavigationData nav;
    if (!nav_reader.readNavigationData(nav)) return 5;
    std::size_t epochs=0, successful=0, states=0, unhealthy=0;
    std::map<std::string,std::size_t> failures;
    libgnss::ObservationData epoch;
    while (base.readObservationEpoch(epoch)) {
        ++epochs;
        try {
            const auto result = libgnss::source_transmission_clock::buildEpochStates(
                epoch.time, epoch.observations, nav);
            ++successful;
            states += result.size();
            for (const auto& [satellite,state] : result)
                if (state.ephemeris_health != 0) ++unhealthy;
        } catch (const std::invalid_argument& error) {
            ++failures[error.what()]; // Diagnostic only: no fallback state.
        }
    }
    std::cout << "{\"epochs\":" << epochs << ",\"successful_epochs\":" << successful
              << ",\"states_in_successful_epochs\":" << states
              << ",\"nonzero_health_states\":" << unhealthy << ",\"failures\":{";
    bool first=true;
    for (const auto& [reason,count] : failures) {
        if (!first) std::cout << ',';
        first=false;
        std::cout << '\"' << reason << "\":" << count;
    }
    std::cout << "}}\n";
    return epochs ? 0 : 6;
}
