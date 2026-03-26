#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <fstream>
#include <iostream>
#include <set>
#include <string>

#include <libgnss++/core/navigation.hpp>
#include <libgnss++/io/rinex.hpp>

namespace {

struct Options {
    std::string obs_path;
    std::string nav_path;
    std::string sp3_out;
    std::string clk_out;
    int max_epochs = 0;
    int epoch_step = 1;
    bool quiet = false;
};

void printUsage(const char* program_name) {
    std::cout
        << "Usage: " << program_name << " --obs <rover.obs> --nav <nav.rnx> "
        << "--sp3-out <products.sp3> --clk-out <products.clk> [options]\n"
        << "Options:\n"
        << "  --max-epochs <count>   Limit processed epochs (default: all)\n"
        << "  --epoch-step <count>   Keep every Nth observation epoch (default: 1)\n"
        << "  --quiet                Suppress summary output\n"
        << "  -h, --help             Show this help\n";
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
        } else if (arg == "--obs" && i + 1 < argc) {
            options.obs_path = argv[++i];
        } else if (arg == "--nav" && i + 1 < argc) {
            options.nav_path = argv[++i];
        } else if (arg == "--sp3-out" && i + 1 < argc) {
            options.sp3_out = argv[++i];
        } else if (arg == "--clk-out" && i + 1 < argc) {
            options.clk_out = argv[++i];
        } else if (arg == "--max-epochs" && i + 1 < argc) {
            options.max_epochs = std::stoi(argv[++i]);
        } else if (arg == "--epoch-step" && i + 1 < argc) {
            options.epoch_step = std::stoi(argv[++i]);
        } else if (arg == "--quiet") {
            options.quiet = true;
        } else {
            argumentError("unknown or incomplete argument: " + arg, argv[0]);
        }
    }

    if (options.obs_path.empty()) {
        argumentError("--obs is required", argv[0]);
    }
    if (options.nav_path.empty()) {
        argumentError("--nav is required", argv[0]);
    }
    if (options.sp3_out.empty()) {
        argumentError("--sp3-out is required", argv[0]);
    }
    if (options.clk_out.empty()) {
        argumentError("--clk-out is required", argv[0]);
    }
    if (options.max_epochs < 0) {
        argumentError("--max-epochs must be non-negative", argv[0]);
    }
    if (options.epoch_step <= 0) {
        argumentError("--epoch-step must be positive", argv[0]);
    }
    return options;
}

std::string formatEpochLine(const libgnss::GNSSTime& time) {
    const auto tp = time.toSystemTime();
    const std::time_t unix_time = std::chrono::system_clock::to_time_t(tp);
    std::tm utc_tm{};
    gmtime_r(&unix_time, &utc_tm);
    const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
        tp.time_since_epoch()).count() % 1'000'000;
    char buffer[64];
    std::snprintf(
        buffer,
        sizeof(buffer),
        "*  %04d %02d %02d %02d %02d %011.8f\n",
        utc_tm.tm_year + 1900,
        utc_tm.tm_mon + 1,
        utc_tm.tm_mday,
        utc_tm.tm_hour,
        utc_tm.tm_min,
        static_cast<double>(utc_tm.tm_sec) + static_cast<double>(micros) * 1e-6);
    return std::string(buffer);
}

std::string formatClockEpochFields(const libgnss::GNSSTime& time) {
    const auto tp = time.toSystemTime();
    const std::time_t unix_time = std::chrono::system_clock::to_time_t(tp);
    std::tm utc_tm{};
    gmtime_r(&unix_time, &utc_tm);
    const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
        tp.time_since_epoch()).count() % 1'000'000;
    char buffer[64];
    std::snprintf(
        buffer,
        sizeof(buffer),
        "%04d %02d %02d %02d %02d %011.8f",
        utc_tm.tm_year + 1900,
        utc_tm.tm_mon + 1,
        utc_tm.tm_mday,
        utc_tm.tm_hour,
        utc_tm.tm_min,
        static_cast<double>(utc_tm.tm_sec) + static_cast<double>(micros) * 1e-6);
    return std::string(buffer);
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        const Options options = parseArguments(argc, argv);

        libgnss::io::RINEXReader obs_reader;
        if (!obs_reader.open(options.obs_path)) {
            std::cerr << "Error: failed to open observation file: " << options.obs_path << "\n";
            return 1;
        }
        libgnss::io::RINEXReader::RINEXHeader obs_header;
        if (!obs_reader.readHeader(obs_header)) {
            std::cerr << "Error: failed to read observation header: " << options.obs_path << "\n";
            return 1;
        }

        libgnss::io::RINEXReader nav_reader;
        if (!nav_reader.open(options.nav_path)) {
            std::cerr << "Error: failed to open navigation file: " << options.nav_path << "\n";
            return 1;
        }
        libgnss::NavigationData nav_data;
        if (!nav_reader.readNavigationData(nav_data)) {
            std::cerr << "Error: failed to read navigation data: " << options.nav_path << "\n";
            return 1;
        }

        std::ofstream sp3_output(options.sp3_out);
        if (!sp3_output.is_open()) {
            std::cerr << "Error: failed to open SP3 output: " << options.sp3_out << "\n";
            return 1;
        }
        std::ofstream clk_output(options.clk_out);
        if (!clk_output.is_open()) {
            std::cerr << "Error: failed to open CLK output: " << options.clk_out << "\n";
            return 1;
        }

        clk_output << "     3.00           C                   RINEX VERSION / TYPE\n";
        clk_output << "END OF HEADER\n";

        libgnss::ObservationData epoch;
        int seen_epochs = 0;
        int written_epochs = 0;
        int written_satellite_samples = 0;
        while ((options.max_epochs == 0 || written_epochs < options.max_epochs) &&
               obs_reader.readObservationEpoch(epoch)) {
            ++seen_epochs;
            if (((seen_epochs - 1) % options.epoch_step) != 0) {
                continue;
            }

            const auto epoch_satellites = epoch.getSatellites();
            std::set<libgnss::SatelliteId> satellites(epoch_satellites.begin(), epoch_satellites.end());
            if (satellites.empty()) {
                continue;
            }

            sp3_output << formatEpochLine(epoch.time);
            int epoch_samples = 0;
            for (const auto& satellite : satellites) {
                libgnss::Vector3d position = libgnss::Vector3d::Zero();
                libgnss::Vector3d velocity = libgnss::Vector3d::Zero();
                double clock_bias = 0.0;
                double clock_drift = 0.0;
                if (!nav_data.calculateSatelliteState(
                        satellite, epoch.time, position, velocity, clock_bias, clock_drift)) {
                    continue;
                }

                char sp3_line[160];
                std::snprintf(
                    sp3_line,
                    sizeof(sp3_line),
                    "P%s %14.6f %14.6f %14.6f %14.6f\n",
                    satellite.toString().c_str(),
                    position.x() / 1000.0,
                    position.y() / 1000.0,
                    position.z() / 1000.0,
                    clock_bias * 1e6);
                sp3_output << sp3_line;

                clk_output << "AS " << satellite.toString() << " "
                           << formatClockEpochFields(epoch.time)
                           << "  2 ";
                char clk_values[64];
                std::snprintf(clk_values, sizeof(clk_values), "% .12E  % .12E\n", clock_bias, 1.0e-12);
                clk_output << clk_values;

                ++epoch_samples;
                ++written_satellite_samples;
            }

            if (epoch_samples > 0) {
                ++written_epochs;
            }
        }

        if (written_epochs == 0 || written_satellite_samples == 0) {
            std::cerr << "Error: no SP3/CLK samples could be generated from the provided data\n";
            return 1;
        }

        if (!options.quiet) {
            std::cout << "Navigation products summary:\n";
            std::cout << "  observation epochs seen: " << seen_epochs << "\n";
            std::cout << "  epochs written: " << written_epochs << "\n";
            std::cout << "  satellite samples written: " << written_satellite_samples << "\n";
            std::cout << "  SP3 output: " << options.sp3_out << "\n";
            std::cout << "  CLK output: " << options.clk_out << "\n";
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
