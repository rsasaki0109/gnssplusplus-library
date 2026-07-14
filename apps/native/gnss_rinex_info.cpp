#include <algorithm>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <libgnss++/io/rinex.hpp>

namespace {

struct Options {
    bool count_records = false;
    std::vector<std::string> paths;
};

std::string fileTypeString(libgnss::io::RINEXReader::FileType type) {
    switch (type) {
        case libgnss::io::RINEXReader::FileType::OBSERVATION:
            return "observation";
        case libgnss::io::RINEXReader::FileType::NAVIGATION:
            return "navigation";
        case libgnss::io::RINEXReader::FileType::METEOROLOGICAL:
            return "meteorological";
        case libgnss::io::RINEXReader::FileType::CLOCK:
            return "clock";
        case libgnss::io::RINEXReader::FileType::UNKNOWN:
        default:
            return "unknown";
    }
}

std::string formatTime(const libgnss::GNSSTime& time) {
    if (time.week == 0 && std::abs(time.tow) < 1e-9) {
        return "n/a";
    }
    std::ostringstream oss;
    oss << "week " << time.week << ", tow " << std::fixed << std::setprecision(3) << time.tow;
    return oss.str();
}

std::string joinStrings(const std::vector<std::string>& values, std::size_t limit = 12) {
    if (values.empty()) {
        return "n/a";
    }

    std::ostringstream oss;
    const std::size_t shown = std::min(values.size(), limit);
    for (std::size_t i = 0; i < shown; ++i) {
        if (i > 0) oss << ", ";
        oss << values[i];
    }
    if (values.size() > shown) {
        oss << ", ... (" << values.size() << " total)";
    }
    return oss.str();
}

void printUsage(const char* program_name) {
    std::cout
        << "Usage: " << program_name << " [options] <rinex-file> [more-files...]\n"
        << "  --count-records   Count observation epochs or navigation ephemerides\n"
        << "  -h, --help        Show this help\n";
}

[[noreturn]] void argumentError(const std::string& message, const char* program_name) {
    std::cerr << "Argument error: " << message << "\n\n";
    printUsage(program_name);
    std::exit(1);
}

Options parseArguments(int argc, char* argv[]) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            std::exit(0);
        }
        if (arg == "--count-records") {
            options.count_records = true;
            continue;
        }
        if (!arg.empty() && arg[0] == '-') {
            argumentError("unknown option: " + arg, argv[0]);
        }
        options.paths.push_back(arg);
    }

    if (options.paths.empty()) {
        argumentError("at least one RINEX file is required", argv[0]);
    }
    return options;
}

void printHeaderSummary(const std::string& path, const libgnss::io::RINEXReader::RINEXHeader& header) {
    std::cout << "File: " << path << "\n";
    std::cout << "  type: " << fileTypeString(header.file_type) << "\n";
    std::cout << "  version: " << std::fixed << std::setprecision(2) << header.version << "\n";
    std::cout << "  satellite system: "
              << (header.satellite_system.empty() ? "n/a" : header.satellite_system) << "\n";
    std::cout << "  marker: " << (header.marker_name.empty() ? "n/a" : header.marker_name) << "\n";
    std::cout << "  receiver: "
              << (header.receiver_type.empty() ? "n/a" : header.receiver_type) << "\n";
    std::cout << "  antenna: " << (header.antenna_type.empty() ? "n/a" : header.antenna_type) << "\n";
    std::cout << "  approximate position (ECEF m): "
              << std::setprecision(4)
              << header.approximate_position.transpose() << "\n";
    std::cout << "  interval: " << header.interval << " s\n";
    std::cout << "  first obs: " << formatTime(header.first_obs) << "\n";
    std::cout << "  last obs: " << formatTime(header.last_obs) << "\n";

    if (!header.observation_types.empty()) {
        std::cout << "  observation types: " << joinStrings(header.observation_types) << "\n";
    }
    if (!header.system_obs_types.empty()) {
        std::cout << "  system observation types:\n";
        for (const auto& [system, obs_types] : header.system_obs_types) {
            std::cout << "    " << system << ": " << joinStrings(obs_types) << "\n";
        }
    }
}

int countObservationEpochs(libgnss::io::RINEXReader& reader,
                           libgnss::GNSSTime& first_epoch,
                           libgnss::GNSSTime& last_epoch,
                           std::size_t& total_observations) {
    int epoch_count = 0;
    libgnss::ObservationData obs;
    while (reader.readObservationEpoch(obs)) {
        if (epoch_count == 0) {
            first_epoch = obs.time;
        }
        last_epoch = obs.time;
        total_observations += obs.observations.size();
        epoch_count++;
    }
    return epoch_count;
}

void printCounts(libgnss::io::RINEXReader& reader,
                 libgnss::io::RINEXReader::FileType file_type) {
    if (file_type == libgnss::io::RINEXReader::FileType::OBSERVATION) {
        libgnss::GNSSTime first_epoch;
        libgnss::GNSSTime last_epoch;
        std::size_t total_observations = 0;
        const int epochs = countObservationEpochs(reader, first_epoch, last_epoch, total_observations);
        std::cout << "  epoch count: " << epochs << "\n";
        std::cout << "  total observation records: " << total_observations << "\n";
        if (epochs > 0) {
            std::cout << "  first epoch from data: " << formatTime(first_epoch) << "\n";
            std::cout << "  last epoch from data: " << formatTime(last_epoch) << "\n";
        }
        return;
    }

    if (file_type == libgnss::io::RINEXReader::FileType::NAVIGATION) {
        libgnss::NavigationData nav_data;
        if (!reader.readNavigationData(nav_data)) {
            std::cout << "  ephemeris count: unavailable\n";
            return;
        }
        std::cout << "  ephemeris count: " << nav_data.ephemeris_data.size() << "\n";
    }
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        const Options options = parseArguments(argc, argv);
        bool first_file = true;
        for (const auto& path : options.paths) {
            if (!first_file) {
                std::cout << "\n";
            }
            first_file = false;

            libgnss::io::RINEXReader reader;
            if (!reader.open(path)) {
                std::cerr << "Error: cannot open " << path << "\n";
                return 1;
            }

            libgnss::io::RINEXReader::RINEXHeader header;
            if (!reader.readHeader(header)) {
                std::cerr << "Error: failed to read RINEX header: " << path << "\n";
                return 1;
            }

            printHeaderSummary(path, header);
            if (options.count_records) {
                printCounts(reader, header.file_type);
            }
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
