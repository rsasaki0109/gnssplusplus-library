#include "rtklib2/io/rinex_reader.hpp"

#include <algorithm>
#include <string>
#include <utility>

#include "libgnss++/io/rinex.hpp"

namespace rtklib2::io::rinex {

namespace {
[[nodiscard]] std::string trim(std::string value) {
    const auto first = value.find_first_not_of(' ');
    if (first == std::string::npos) {
        return {};
    }
    const auto last = value.find_last_not_of(' ');
    return value.substr(first, last - first + 1);
}
}  // namespace

bool read_observation_file(const std::string& file_path,
                           std::vector<libgnss::ObservationData>& epochs,
                           ObservationHeader* header) {
    libgnss::io::RINEXReader reader;
    if (!reader.open(file_path)) {
        return false;
    }

    libgnss::io::RINEXReader::RINEXHeader legacy_header;
    if (!reader.readHeader(legacy_header)) {
        reader.close();
        return false;
    }

    if (header != nullptr) {
        header->version = legacy_header.version;
        header->marker_name = trim(legacy_header.marker_name);
        header->approximate_position = legacy_header.approximate_position;
    }

    epochs.clear();
    libgnss::ObservationData epoch;
    while (reader.readObservationEpoch(epoch)) {
        epochs.push_back(epoch);
    }

    reader.close();
    return !epochs.empty();
}

bool read_navigation_file(const std::string& file_path,
                          libgnss::NavigationData& navigation_data) {
    libgnss::io::RINEXReader reader;
    if (!reader.open(file_path)) {
        return false;
    }

    libgnss::io::RINEXReader::RINEXHeader legacy_header;
    if (!reader.readHeader(legacy_header)) {
        reader.close();
        return false;
    }

    const bool success = reader.readNavigationData(navigation_data);
    reader.close();
    return success && !navigation_data.isEmpty();
}

}  // namespace rtklib2::io::rinex
