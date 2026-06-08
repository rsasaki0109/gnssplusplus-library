#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/signal_policy.hpp>
#include <libgnss++/io/rinex.hpp>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;

struct Options {
    std::string obs_path;
    std::string nav_path;
    std::string csv_path;
    std::string summary_json_path;
    int max_epochs = 0;
    double min_elevation_deg = 0.0;
    bool quiet = false;
};

struct SummaryStats {
    int epochs_processed = 0;
    int epochs_with_rows = 0;
    int rows_written = 0;
    int skipped_without_nav = 0;
    int skipped_invalid_observation = 0;
    int snr_row_count = 0;
    int max_satellites_per_epoch = 0;
    double sum_satellites_per_epoch = 0.0;
    double sum_elevation_deg = 0.0;
    double sum_snr_dbhz = 0.0;
    libgnss::GNSSTime first_epoch;
    libgnss::GNSSTime last_epoch;
    bool has_first_epoch = false;
    std::set<std::string> unique_satellites;
    std::map<std::string, int> rows_per_system;
};

void printUsage(const char* program_name) {
    std::cout
        << "Usage: " << program_name << " --obs <rover.obs> --nav <nav.rnx> [options]\n"
        << "Options:\n"
        << "  --csv <visibility.csv>           Write per-satellite visibility rows to CSV\n"
        << "  --summary-json <summary.json>    Write run summary to JSON\n"
        << "  --max-epochs <count>             Limit processed epochs (default: all)\n"
        << "  --min-elevation-deg <deg>        Filter rows below this elevation (default: 0)\n"
        << "  --quiet                          Suppress stdout summary\n"
        << "  -h, --help                       Show this help\n";
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
        } else if (arg == "--csv" && i + 1 < argc) {
            options.csv_path = argv[++i];
        } else if (arg == "--summary-json" && i + 1 < argc) {
            options.summary_json_path = argv[++i];
        } else if (arg == "--max-epochs" && i + 1 < argc) {
            options.max_epochs = std::stoi(argv[++i]);
        } else if (arg == "--min-elevation-deg" && i + 1 < argc) {
            options.min_elevation_deg = std::stod(argv[++i]);
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
    if (options.max_epochs < 0) {
        argumentError("--max-epochs must be non-negative", argv[0]);
    }
    if (options.min_elevation_deg < -90.0 || options.min_elevation_deg > 90.0) {
        argumentError("--min-elevation-deg must be within [-90, 90]", argv[0]);
    }
    return options;
}

std::string jsonEscape(const std::string& value) {
    std::ostringstream escaped;
    for (char ch : value) {
        switch (ch) {
            case '\\': escaped << "\\\\"; break;
            case '"': escaped << "\\\""; break;
            case '\n': escaped << "\\n"; break;
            case '\r': escaped << "\\r"; break;
            case '\t': escaped << "\\t"; break;
            default: escaped << ch; break;
        }
    }
    return escaped.str();
}

std::string systemName(libgnss::GNSSSystem system) {
    switch (system) {
        case libgnss::GNSSSystem::GPS: return "GPS";
        case libgnss::GNSSSystem::GLONASS: return "GLONASS";
        case libgnss::GNSSSystem::Galileo: return "Galileo";
        case libgnss::GNSSSystem::BeiDou: return "BeiDou";
        case libgnss::GNSSSystem::QZSS: return "QZSS";
        case libgnss::GNSSSystem::SBAS: return "SBAS";
        case libgnss::GNSSSystem::NavIC: return "NavIC";
        default: return "UNKNOWN";
    }
}

std::string signalName(libgnss::SignalType signal) {
    switch (signal) {
        case libgnss::SignalType::GPS_L1CA: return "GPS_L1CA";
        case libgnss::SignalType::GPS_L1P: return "GPS_L1P";
        case libgnss::SignalType::GPS_L2P: return "GPS_L2P";
        case libgnss::SignalType::GPS_L2C: return "GPS_L2C";
        case libgnss::SignalType::GPS_L5: return "GPS_L5";
        case libgnss::SignalType::GLO_L1CA: return "GLO_L1CA";
        case libgnss::SignalType::GLO_L1P: return "GLO_L1P";
        case libgnss::SignalType::GLO_L2CA: return "GLO_L2CA";
        case libgnss::SignalType::GLO_L2P: return "GLO_L2P";
        case libgnss::SignalType::GAL_E1: return "GAL_E1";
        case libgnss::SignalType::GAL_E5A: return "GAL_E5A";
        case libgnss::SignalType::GAL_E5B: return "GAL_E5B";
        case libgnss::SignalType::GAL_E6: return "GAL_E6";
        case libgnss::SignalType::BDS_B1I: return "BDS_B1I";
        case libgnss::SignalType::BDS_B2I: return "BDS_B2I";
        case libgnss::SignalType::BDS_B3I: return "BDS_B3I";
        case libgnss::SignalType::BDS_B1C: return "BDS_B1C";
        case libgnss::SignalType::BDS_B2A: return "BDS_B2A";
        case libgnss::SignalType::QZS_L1CA: return "QZS_L1CA";
        case libgnss::SignalType::QZS_L2C: return "QZS_L2C";
        case libgnss::SignalType::QZS_L5: return "QZS_L5";
        default: return "UNKNOWN";
    }
}

int observationRank(const libgnss::Observation& obs) {
    const bool primary = libgnss::signal_policy::isPrimarySignal(obs.satellite.system, obs.signal);
    const bool secondary = libgnss::signal_policy::isSecondarySignal(obs.satellite.system, obs.signal);
    if (primary) {
        return libgnss::signal_policy::signalPriority(obs.satellite.system, obs.signal, true);
    }
    if (secondary) {
        return 10 + libgnss::signal_policy::signalPriority(obs.satellite.system, obs.signal, false);
    }
    return 100;
}

const libgnss::Observation* selectRepresentativeObservation(
    const std::vector<libgnss::Observation>& observations) {
    const libgnss::Observation* best = nullptr;
    std::tuple<int, int, int, int, double> best_key;
    for (const auto& observation : observations) {
        const auto key = std::make_tuple(
            observation.valid ? 0 : 1,
            observation.has_pseudorange ? 0 : 1,
            observationRank(observation),
            observation.has_carrier_phase ? 0 : 1,
            -observation.snr);
        if (best == nullptr || key < best_key) {
            best = &observation;
            best_key = key;
        }
    }
    return best;
}

void writeSummaryJson(const Options& options,
                      const libgnss::Vector3d& receiver_position,
                      const SummaryStats& stats) {
    if (options.summary_json_path.empty()) {
        return;
    }

    std::ofstream output(options.summary_json_path);
    if (!output.is_open()) {
        throw std::runtime_error("failed to open summary JSON for writing: " + options.summary_json_path);
    }

    output << "{\n";
    output << "  \"obs\": \"" << jsonEscape(options.obs_path) << "\",\n";
    output << "  \"nav\": \"" << jsonEscape(options.nav_path) << "\",\n";
    if (!options.csv_path.empty()) {
        output << "  \"csv\": \"" << jsonEscape(options.csv_path) << "\",\n";
    }
    output << "  \"epochs_processed\": " << stats.epochs_processed << ",\n";
    output << "  \"epochs_with_rows\": " << stats.epochs_with_rows << ",\n";
    output << "  \"rows_written\": " << stats.rows_written << ",\n";
    output << "  \"unique_satellites\": " << stats.unique_satellites.size() << ",\n";
    output << "  \"skipped_without_nav\": " << stats.skipped_without_nav << ",\n";
    output << "  \"skipped_invalid_observation\": " << stats.skipped_invalid_observation << ",\n";
    output << "  \"min_elevation_deg\": " << std::fixed << std::setprecision(3)
           << options.min_elevation_deg << ",\n";
    output << "  \"receiver_position_ecef_m\": ["
           << std::setprecision(4) << receiver_position.x() << ", "
           << receiver_position.y() << ", "
           << receiver_position.z() << "],\n";
    if (stats.has_first_epoch) {
        output << "  \"first_epoch\": {\"week\": " << stats.first_epoch.week
               << ", \"tow\": " << std::setprecision(3) << stats.first_epoch.tow << "},\n";
        output << "  \"last_epoch\": {\"week\": " << stats.last_epoch.week
               << ", \"tow\": " << std::setprecision(3) << stats.last_epoch.tow << "},\n";
    }
    const double mean_satellites = stats.epochs_processed > 0
        ? stats.sum_satellites_per_epoch / static_cast<double>(stats.epochs_processed)
        : 0.0;
    const double mean_elevation = stats.rows_written > 0
        ? stats.sum_elevation_deg / static_cast<double>(stats.rows_written)
        : 0.0;
    const double mean_snr = stats.snr_row_count > 0
        ? stats.sum_snr_dbhz / static_cast<double>(stats.snr_row_count)
        : 0.0;
    output << "  \"mean_satellites_per_epoch\": " << std::setprecision(4) << mean_satellites << ",\n";
    output << "  \"max_satellites_per_epoch\": " << stats.max_satellites_per_epoch << ",\n";
    output << "  \"mean_elevation_deg\": " << std::setprecision(4) << mean_elevation << ",\n";
    output << "  \"mean_snr_dbhz\": " << std::setprecision(4) << mean_snr << ",\n";
    output << "  \"rows_per_system\": {\n";
    bool first = true;
    for (const auto& [system, count] : stats.rows_per_system) {
        if (!first) {
            output << ",\n";
        }
        output << "    \"" << jsonEscape(system) << "\": " << count;
        first = false;
    }
    output << "\n  }\n";
    output << "}\n";
}

void printSummary(const Options& options,
                  const libgnss::Vector3d& receiver_position,
                  const SummaryStats& stats) {
    if (options.quiet) {
        return;
    }
    const double mean_satellites = stats.epochs_processed > 0
        ? stats.sum_satellites_per_epoch / static_cast<double>(stats.epochs_processed)
        : 0.0;
    const double mean_elevation = stats.rows_written > 0
        ? stats.sum_elevation_deg / static_cast<double>(stats.rows_written)
        : 0.0;
    const double mean_snr = stats.snr_row_count > 0
        ? stats.sum_snr_dbhz / static_cast<double>(stats.snr_row_count)
        : 0.0;

    std::cout << "Visibility summary:\n";
    std::cout << "  observation file: " << options.obs_path << "\n";
    std::cout << "  navigation file: " << options.nav_path << "\n";
    std::cout << "  receiver ECEF (m): "
              << std::fixed << std::setprecision(3)
              << receiver_position.x() << " "
              << receiver_position.y() << " "
              << receiver_position.z() << "\n";
    std::cout << "  epochs processed: " << stats.epochs_processed << "\n";
    std::cout << "  epochs with rows: " << stats.epochs_with_rows << "\n";
    std::cout << "  rows written: " << stats.rows_written << "\n";
    std::cout << "  unique satellites: " << stats.unique_satellites.size() << "\n";
    std::cout << "  mean satellites/epoch: " << std::setprecision(3) << mean_satellites << "\n";
    std::cout << "  max satellites/epoch: " << stats.max_satellites_per_epoch << "\n";
    std::cout << "  mean elevation (deg): " << std::setprecision(3) << mean_elevation << "\n";
    if (stats.snr_row_count > 0) {
        std::cout << "  mean SNR (dB-Hz): " << std::setprecision(3) << mean_snr << "\n";
    }
    std::cout << "  skipped without nav: " << stats.skipped_without_nav << "\n";
    std::cout << "  skipped invalid observation: " << stats.skipped_invalid_observation << "\n";
    if (!options.csv_path.empty()) {
        std::cout << "  CSV output: " << options.csv_path << "\n";
    }
    if (!options.summary_json_path.empty()) {
        std::cout << "  summary JSON: " << options.summary_json_path << "\n";
    }
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

        libgnss::Vector3d receiver_position = obs_header.approximate_position;
        if (receiver_position.norm() <= 0.0) {
            std::cerr << "Error: observation header does not contain an approximate receiver position\n";
            return 1;
        }

        std::ofstream csv_output;
        if (!options.csv_path.empty()) {
            csv_output.open(options.csv_path);
            if (!csv_output.is_open()) {
                std::cerr << "Error: failed to open CSV output: " << options.csv_path << "\n";
                return 1;
            }
            csv_output << "epoch_index,week,tow,satellite,system,signal,azimuth_deg,elevation_deg,snr_dbhz,"
                          "has_pseudorange,has_carrier_phase,has_doppler\n";
        }

        const double min_elevation_rad = options.min_elevation_deg / kRadToDeg;
        libgnss::ObservationData epoch;
        SummaryStats stats;

        while ((options.max_epochs == 0 || stats.epochs_processed < options.max_epochs) &&
               obs_reader.readObservationEpoch(epoch)) {
            ++stats.epochs_processed;
            if (!stats.has_first_epoch) {
                stats.first_epoch = epoch.time;
                stats.has_first_epoch = true;
            }
            stats.last_epoch = epoch.time;

            int epoch_rows = 0;
            for (const auto& satellite : epoch.getSatellites()) {
                const auto observations = epoch.getObservations(satellite);
                const libgnss::Observation* chosen = selectRepresentativeObservation(observations);
                if (chosen == nullptr || !chosen->valid) {
                    ++stats.skipped_invalid_observation;
                    continue;
                }

                libgnss::Vector3d satellite_position = libgnss::Vector3d::Zero();
                libgnss::Vector3d satellite_velocity = libgnss::Vector3d::Zero();
                double clock_bias = 0.0;
                double clock_drift = 0.0;
                libgnss::GNSSTime tx_time = epoch.time;
                if (chosen->has_pseudorange && chosen->pseudorange > 0.0) {
                    tx_time = epoch.time - (chosen->pseudorange / libgnss::constants::SPEED_OF_LIGHT);
                }
                if (!nav_data.calculateSatelliteState(
                        satellite, tx_time, satellite_position, satellite_velocity, clock_bias, clock_drift) &&
                    !nav_data.calculateSatelliteState(
                        satellite, epoch.time, satellite_position, satellite_velocity, clock_bias, clock_drift)) {
                    ++stats.skipped_without_nav;
                    continue;
                }

                const auto geometry = nav_data.calculateGeometry(receiver_position, satellite_position);
                if (geometry.elevation < min_elevation_rad) {
                    continue;
                }

                ++epoch_rows;
                ++stats.rows_written;
                stats.unique_satellites.insert(satellite.toString());
                stats.rows_per_system[systemName(satellite.system)] += 1;
                stats.sum_elevation_deg += geometry.elevation * kRadToDeg;
                if (chosen->snr > 0.0) {
                    stats.sum_snr_dbhz += chosen->snr;
                    ++stats.snr_row_count;
                }

                if (csv_output.is_open()) {
                    csv_output << stats.epochs_processed << ","
                               << epoch.time.week << ","
                               << std::fixed << std::setprecision(3) << epoch.time.tow << ","
                               << satellite.toString() << ","
                               << systemName(satellite.system) << ","
                               << signalName(chosen->signal) << ","
                               << std::setprecision(6) << geometry.azimuth * kRadToDeg << ","
                               << geometry.elevation * kRadToDeg << ","
                               << chosen->snr << ","
                               << (chosen->has_pseudorange ? 1 : 0) << ","
                               << (chosen->has_carrier_phase ? 1 : 0) << ","
                               << (chosen->has_doppler ? 1 : 0) << "\n";
                }
            }

            if (epoch_rows > 0) {
                ++stats.epochs_with_rows;
            }
            stats.sum_satellites_per_epoch += static_cast<double>(epoch_rows);
            stats.max_satellites_per_epoch = std::max(stats.max_satellites_per_epoch, epoch_rows);
        }

        if (stats.epochs_processed == 0) {
            std::cerr << "Error: no observation epochs were read from " << options.obs_path << "\n";
            return 1;
        }
        if (stats.rows_written == 0) {
            std::cerr << "Error: no visibility rows were produced from the provided data\n";
            return 1;
        }

        writeSummaryJson(options, receiver_position, stats);
        printSummary(options, receiver_position, stats);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
