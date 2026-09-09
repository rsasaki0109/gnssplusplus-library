#include <libgnss++/io/rinex.hpp>
#include <iostream>
#include <map>
#include <string>

// Read-only aggregate diagnostic. No navigation, positioning or truth input.
int main(int argc, char** argv) {
    if (argc != 3 || (std::string(argv[2]) != "source" &&
                      std::string(argv[2]) != "default")) return 2;
    libgnss::io::RINEXReader reader;
    reader.setSourceHeaderTrackingFilter(std::string(argv[2]) == "source");
    reader.setPreserveAdditionalFrequencyBands(true);
    if (!reader.open(argv[1])) return 3;
    libgnss::io::RINEXReader::RINEXHeader header;
    if (!reader.readHeader(header) || header.version < 3.0 || header.version >= 4.0) return 4;
    std::size_t epochs = 0, rows = 0, p = 0;
    std::map<std::string, std::size_t> counts;
    libgnss::ObservationData epoch;
    while (reader.readObservationEpoch(epoch)) {
        ++epochs;
        rows += epoch.observations.size();
        for (const auto& row : epoch.observations) {
            if (!row.has_pseudorange) continue;
            ++p;
            ++counts[std::to_string(static_cast<int>(row.satellite.system)) +
                     ":" + row.pseudorange_observation_type];
        }
    }
    std::cout << "{\"epochs\":" << epochs << ",\"rows\":" << rows
              << ",\"p_rows\":" << p << ",\"codes\":{";
    bool first = true;
    for (const auto& [code, count] : counts) {
        if (!first) std::cout << ',';
        first = false;
        std::cout << '\"' << code << "\":" << count;
    }
    std::cout << "}}\n";
    return epochs ? 0 : 5;
}
