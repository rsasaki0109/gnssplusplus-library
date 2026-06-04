#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/io/rinex.hpp>
#include <libgnss++/models/ionosphere.hpp>
#include <libgnss++/models/troposphere.hpp>

#include <Eigen/Sparse>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
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

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kRadiansToDegrees = 180.0 / kPi;
constexpr std::size_t kClockGroups = 5;
constexpr std::size_t kStateStride = 12;

struct Options {
    std::string obs_path;
    std::string nav_path;
    std::string seed_pos_path;
    std::string out_csv_path;
    std::string factor_debug_csv_path;
    std::string graph_csv_path;
    std::string summary_json_path;
    int skip_epochs = 0;
    int max_epochs = 0;
    int max_iterations = 1000;
    double seed_match_tolerance_s = 0.51;
    double seed_interpolation_max_gap_s = 60.0;
    double min_snr_dbhz = 35.0;
    double min_elevation_deg = 15.0;
    double pseudorange_sigma_zenith_m = 3.0;
    double doppler_sigma_zenith_mps = 0.2;
    double tdcp_sigma_zenith_m = 0.05;
    double position_prior_sigma_m = 1000.0;
    double clock_prior_sigma_m = 1e6;
    double velocity_prior_sigma_mps = 1000.0;
    double clock_drift_prior_sigma_mps = 1000.0;
    double motion_sigma_m = 0.1;
    double clock_motion_sigma_m = 0.1;
    double clock_jump_sigma_m = 1e6;
    double inter_system_clock_motion_sigma_m = 1e-6;
    double clock_drift_between_sigma_mps = 0.1;
    double huber_threshold_sigma = 1.234;
    bool debug_problem_only = false;
    bool quiet = false;
};

struct SeedPosition {
    libgnss::GNSSTime time;
    libgnss::Vector3d position_ecef = libgnss::Vector3d::Zero();
};

struct DopplerFactor {
    std::size_t epoch_index = 0;
    libgnss::GNSSTime time;
    libgnss::SatelliteId satellite;
    libgnss::SignalType signal = libgnss::SignalType::GPS_L1CA;
    double snr_dbhz = 0.0;
    double elevation_rad = 0.0;
    double sigma_mps = 1.0;
    double residual_mps = 0.0;
    double measured_range_rate_mps = 0.0;
    double modeled_range_rate_mps = 0.0;
    double satellite_clock_drift_mps = 0.0;
    double wavelength_m = 0.0;
    libgnss::Vector3d los = libgnss::Vector3d::Zero();
};

struct PseudorangeFactor {
    std::size_t epoch_index = 0;
    libgnss::GNSSTime time;
    libgnss::SatelliteId satellite;
    libgnss::SignalType signal = libgnss::SignalType::GPS_L1CA;
    std::size_t clock_group = 0;
    double snr_dbhz = 0.0;
    double elevation_rad = 0.0;
    double sigma_m = 1.0;
    double residual_m = 0.0;
    double corrected_pseudorange_m = 0.0;
    double modeled_range_m = 0.0;
    double ionosphere_delay_m = 0.0;
    double troposphere_delay_m = 0.0;
    double satellite_clock_m = 0.0;
    double group_delay_m = 0.0;
    libgnss::Vector3d los = libgnss::Vector3d::Zero();
};

using ObservationKey = std::tuple<libgnss::SatelliteId, libgnss::SignalType>;

struct CarrierResidual {
    std::size_t epoch_index = 0;
    libgnss::GNSSTime time;
    libgnss::SatelliteId satellite;
    libgnss::SignalType signal = libgnss::SignalType::GPS_L1CA;
    double snr_dbhz = 0.0;
    double elevation_rad = 0.0;
    double residual_m = 0.0;
    double corrected_carrier_m = 0.0;
    double modeled_range_m = 0.0;
    double raw_carrier_cycles = 0.0;
    double wavelength_m = 0.0;
    double ionosphere_delay_m = 0.0;
    double troposphere_delay_m = 0.0;
    double satellite_clock_m = 0.0;
    std::uint8_t lli = 0;
    bool loss_of_lock = false;
    libgnss::Vector3d los = libgnss::Vector3d::Zero();
};

struct TdcpFactor {
    std::size_t epoch_index = 0;
    std::size_t previous_epoch_index = 0;
    libgnss::GNSSTime time;
    libgnss::SatelliteId satellite;
    libgnss::SignalType signal = libgnss::SignalType::GPS_L1CA;
    double snr_dbhz = 0.0;
    double elevation_rad = 0.0;
    double previous_elevation_rad = 0.0;
    double sigma_m = 1.0;
    double residual_m = 0.0;
    double previous_carrier_residual_m = 0.0;
    double current_carrier_residual_m = 0.0;
    double previous_raw_carrier_cycles = 0.0;
    double current_raw_carrier_cycles = 0.0;
    double wavelength_m = 0.0;
    libgnss::Vector3d los = libgnss::Vector3d::Zero();
};

struct Problem {
    std::vector<libgnss::ObservationData> epochs;
    std::vector<libgnss::Vector3d> seed_positions;
    std::vector<PseudorangeFactor> pseudorange_factors;
    std::vector<DopplerFactor> factors;
    std::vector<TdcpFactor> tdcp_factors;
    std::vector<bool> clock_jumps;
    std::size_t nsat = 0;
    std::size_t seed_matched_epochs = 0;
    std::size_t seed_interpolated_epochs = 0;
};

struct SolveResult {
    Eigen::VectorXd state;
    double initial_cost = 0.0;
    double final_cost = 0.0;
    double residual_rms_mps = 0.0;
    int iterations = 0;
    bool converged = false;
};

[[noreturn]] void usageError(const std::string& message, const char* argv0) {
    std::cerr << "Error: " << message << "\n\n"
              << "Usage: " << argv0 << " --obs rover.obs --nav base.nav "
              << "--seed-pos rover_1Hz_spp.pos [options]\n\n"
              << "Options:\n"
              << "  --out-csv PATH                 Write per-epoch position/velocity CSV.\n"
              << "  --factor-debug-csv PATH        Write per-factor debug CSV.\n"
              << "  --graph-csv PATH               Write taroz-like graph detail CSV.\n"
              << "  --summary-json PATH            Write JSON summary.\n"
              << "  --max-epochs N                 Limit processed epochs; 0 means all.\n"
              << "  --skip-epochs N                Skip leading observation epochs.\n"
              << "  --max-iterations N             IRLS iteration limit.\n"
              << "  --tdcp-sigma-zenith M          TDCP zenith sigma in meters.\n"
              << "  --debug-problem-only           Build factors and skip optimization.\n"
              << "  --quiet                        Suppress human-readable summary.\n";
    throw std::invalid_argument(message);
}

int parseIntArg(const std::string& value, const std::string& name, const char* argv0) {
    try {
        std::size_t consumed = 0;
        const int parsed = std::stoi(value, &consumed);
        if (consumed == value.size()) {
            return parsed;
        }
    } catch (const std::exception&) {
    }
    usageError("invalid integer for " + name + ": " + value, argv0);
}

double parseDoubleArg(const std::string& value, const std::string& name, const char* argv0) {
    try {
        std::size_t consumed = 0;
        const double parsed = std::stod(value, &consumed);
        if (consumed == value.size() && std::isfinite(parsed)) {
            return parsed;
        }
    } catch (const std::exception&) {
    }
    usageError("invalid number for " + name + ": " + value, argv0);
}

Options parseArguments(int argc, char* argv[]) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto require_value = [&](const std::string& name) -> std::string {
            if (i + 1 >= argc) {
                usageError("missing value for " + name, argv[0]);
            }
            return argv[++i];
        };

        if (arg == "--obs") {
            options.obs_path = require_value(arg);
        } else if (arg == "--nav") {
            options.nav_path = require_value(arg);
        } else if (arg == "--seed-pos") {
            options.seed_pos_path = require_value(arg);
        } else if (arg == "--out-csv") {
            options.out_csv_path = require_value(arg);
        } else if (arg == "--factor-debug-csv") {
            options.factor_debug_csv_path = require_value(arg);
        } else if (arg == "--graph-csv") {
            options.graph_csv_path = require_value(arg);
        } else if (arg == "--summary-json") {
            options.summary_json_path = require_value(arg);
        } else if (arg == "--skip-epochs") {
            options.skip_epochs = parseIntArg(require_value(arg), arg, argv[0]);
        } else if (arg == "--max-epochs") {
            options.max_epochs = parseIntArg(require_value(arg), arg, argv[0]);
        } else if (arg == "--max-iterations") {
            options.max_iterations = parseIntArg(require_value(arg), arg, argv[0]);
        } else if (arg == "--seed-match-tolerance") {
            options.seed_match_tolerance_s = parseDoubleArg(require_value(arg), arg, argv[0]);
        } else if (arg == "--seed-interpolation-max-gap") {
            options.seed_interpolation_max_gap_s = parseDoubleArg(require_value(arg), arg, argv[0]);
        } else if (arg == "--min-snr") {
            options.min_snr_dbhz = parseDoubleArg(require_value(arg), arg, argv[0]);
        } else if (arg == "--min-elevation") {
            options.min_elevation_deg = parseDoubleArg(require_value(arg), arg, argv[0]);
        } else if (arg == "--pseudorange-sigma-zenith") {
            options.pseudorange_sigma_zenith_m = parseDoubleArg(require_value(arg), arg, argv[0]);
        } else if (arg == "--doppler-sigma-zenith") {
            options.doppler_sigma_zenith_mps = parseDoubleArg(require_value(arg), arg, argv[0]);
        } else if (arg == "--tdcp-sigma-zenith") {
            options.tdcp_sigma_zenith_m = parseDoubleArg(require_value(arg), arg, argv[0]);
        } else if (arg == "--position-prior-sigma") {
            options.position_prior_sigma_m = parseDoubleArg(require_value(arg), arg, argv[0]);
        } else if (arg == "--clock-prior-sigma") {
            options.clock_prior_sigma_m = parseDoubleArg(require_value(arg), arg, argv[0]);
        } else if (arg == "--velocity-prior-sigma") {
            options.velocity_prior_sigma_mps = parseDoubleArg(require_value(arg), arg, argv[0]);
        } else if (arg == "--clock-drift-prior-sigma") {
            options.clock_drift_prior_sigma_mps = parseDoubleArg(require_value(arg), arg, argv[0]);
        } else if (arg == "--motion-sigma") {
            options.motion_sigma_m = parseDoubleArg(require_value(arg), arg, argv[0]);
        } else if (arg == "--clock-motion-sigma") {
            options.clock_motion_sigma_m = parseDoubleArg(require_value(arg), arg, argv[0]);
        } else if (arg == "--clock-jump-sigma") {
            options.clock_jump_sigma_m = parseDoubleArg(require_value(arg), arg, argv[0]);
        } else if (arg == "--isb-motion-sigma") {
            options.inter_system_clock_motion_sigma_m =
                parseDoubleArg(require_value(arg), arg, argv[0]);
        } else if (arg == "--clock-drift-between-sigma") {
            options.clock_drift_between_sigma_mps = parseDoubleArg(require_value(arg), arg, argv[0]);
        } else if (arg == "--huber-threshold") {
            options.huber_threshold_sigma = parseDoubleArg(require_value(arg), arg, argv[0]);
        } else if (arg == "--debug-problem-only") {
            options.debug_problem_only = true;
        } else if (arg == "--quiet") {
            options.quiet = true;
        } else if (arg == "--help" || arg == "-h") {
            usageError("help requested", argv[0]);
        } else {
            usageError("unknown argument: " + arg, argv[0]);
        }
    }

    if (options.obs_path.empty()) {
        usageError("--obs is required", argv[0]);
    }
    if (options.nav_path.empty()) {
        usageError("--nav is required", argv[0]);
    }
    if (options.seed_pos_path.empty()) {
        usageError("--seed-pos is required", argv[0]);
    }
    if (options.skip_epochs < 0 || options.max_epochs < 0 ||
        options.max_iterations < 0) {
        usageError("epoch and iteration limits must be non-negative", argv[0]);
    }
    if (options.seed_match_tolerance_s < 0.0 ||
        options.seed_interpolation_max_gap_s < 0.0 ||
        options.pseudorange_sigma_zenith_m <= 0.0 ||
        options.doppler_sigma_zenith_mps <= 0.0 ||
        options.tdcp_sigma_zenith_m <= 0.0 ||
        options.position_prior_sigma_m <= 0.0 ||
        options.clock_prior_sigma_m <= 0.0 ||
        options.velocity_prior_sigma_mps <= 0.0 ||
        options.clock_drift_prior_sigma_mps <= 0.0 ||
        options.motion_sigma_m <= 0.0 ||
        options.clock_motion_sigma_m <= 0.0 ||
        options.clock_jump_sigma_m <= 0.0 ||
        options.inter_system_clock_motion_sigma_m <= 0.0 ||
        options.clock_drift_between_sigma_mps <= 0.0 ||
        options.huber_threshold_sigma <= 0.0) {
        usageError("sigma, threshold, and tolerance values must be positive", argv[0]);
    }
    return options;
}

bool isPrimaryPdSignal(libgnss::SignalType signal) {
    switch (signal) {
        case libgnss::SignalType::GPS_L1CA:
        case libgnss::SignalType::GLO_L1CA:
        case libgnss::SignalType::GAL_E1:
        case libgnss::SignalType::BDS_B1I:
        case libgnss::SignalType::BDS_B1C:
        case libgnss::SignalType::QZS_L1CA:
            return true;
        default:
            return false;
    }
}

std::string signalName(libgnss::SignalType signal) {
    switch (signal) {
        case libgnss::SignalType::GPS_L1CA: return "GPS_L1CA";
        case libgnss::SignalType::GLO_L1CA: return "GLO_L1CA";
        case libgnss::SignalType::GAL_E1: return "GAL_E1";
        case libgnss::SignalType::BDS_B1I: return "BDS_B1I";
        case libgnss::SignalType::BDS_B1C: return "BDS_B1C";
        case libgnss::SignalType::QZS_L1CA: return "QZS_L1CA";
        default: return "UNKNOWN";
    }
}

bool isHealthyForPositioning(const libgnss::Observation& observation,
                             const libgnss::Ephemeris& eph) {
    int sv_health = static_cast<int>(eph.health);
    if (observation.satellite.system == libgnss::GNSSSystem::QZSS) {
        sv_health &= 0xFE;
    }
    return sv_health == 0;
}

std::size_t clockGroup(libgnss::GNSSSystem system) {
    switch (system) {
        case libgnss::GNSSSystem::GPS:
            return 0U;
        case libgnss::GNSSSystem::GLONASS:
            return 1U;
        case libgnss::GNSSSystem::Galileo:
            return 2U;
        case libgnss::GNSSSystem::QZSS:
            return 3U;
        case libgnss::GNSSSystem::BeiDou:
            return 4U;
        default:
            return 0U;
    }
}

double groupDelayCorrectionMeters(const libgnss::Observation& observation,
                                  const libgnss::Ephemeris& eph) {
    switch (observation.satellite.system) {
        case libgnss::GNSSSystem::GPS:
        case libgnss::GNSSSystem::QZSS:
        case libgnss::GNSSSystem::Galileo:
            return eph.tgd * libgnss::constants::SPEED_OF_LIGHT;
        case libgnss::GNSSSystem::BeiDou:
            switch (observation.signal) {
                case libgnss::SignalType::BDS_B1I:
                case libgnss::SignalType::BDS_B1C:
                    return eph.tgd * libgnss::constants::SPEED_OF_LIGHT;
                case libgnss::SignalType::BDS_B2I:
                case libgnss::SignalType::BDS_B2A:
                    return eph.tgd_secondary * libgnss::constants::SPEED_OF_LIGHT;
                default:
                    return 0.0;
            }
        default:
            return 0.0;
    }
}

double sagnacRangeCorrection(const libgnss::Vector3d& satellite_position,
                             const libgnss::Vector3d& receiver_position) {
    return libgnss::constants::OMEGA_E / libgnss::constants::SPEED_OF_LIGHT *
           (satellite_position(0) * receiver_position(1) -
            satellite_position(1) * receiver_position(0));
}

bool parseSeparatedIntegers(std::string value,
                            char separator,
                            int& first,
                            int& second,
                            int& third) {
    std::replace(value.begin(), value.end(), separator, ' ');
    std::istringstream input(value);
    return static_cast<bool>(input >> first >> second >> third);
}

bool parseRtklibTime(std::string value,
                     int& hour,
                     int& minute,
                     double& second) {
    std::replace(value.begin(), value.end(), ':', ' ');
    std::istringstream input(value);
    return static_cast<bool>(input >> hour >> minute >> second);
}

bool isLeapYear(int year) {
    return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

libgnss::GNSSTime gpstCalendarToTime(int year,
                                     int month,
                                     int day,
                                     int hour,
                                     int minute,
                                     double second) {
    int days_since_gps_epoch = 0;
    for (int current_year = 1980; current_year < year; ++current_year) {
        days_since_gps_epoch += isLeapYear(current_year) ? 366 : 365;
    }

    int days_in_month[] = {
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (isLeapYear(year)) {
        days_in_month[1] = 29;
    }
    for (int current_month = 1; current_month < month; ++current_month) {
        days_since_gps_epoch += days_in_month[current_month - 1];
    }
    days_since_gps_epoch += day;
    days_since_gps_epoch -= 6;

    const int gps_week = days_since_gps_epoch / 7;
    const int day_of_week = days_since_gps_epoch % 7;
    const double tow = static_cast<double>(day_of_week) * 86400.0 +
                       static_cast<double>(hour) * 3600.0 +
                       static_cast<double>(minute) * 60.0 + second;
    return libgnss::GNSSTime(gps_week, tow);
}

bool parseRtklibSeedPositionLine(const std::string& line, SeedPosition& seed) {
    std::istringstream input(line);
    std::string date_token;
    std::string time_token;
    if (!(input >> date_token >> time_token)) {
        return false;
    }

    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    double second = 0.0;
    if (!parseSeparatedIntegers(date_token, '/', year, month, day) ||
        !parseRtklibTime(time_token, hour, minute, second)) {
        return false;
    }

    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double height_m = 0.0;
    if (!(input >> latitude_deg >> longitude_deg >> height_m)) {
        return false;
    }
    if (!std::isfinite(latitude_deg) || !std::isfinite(longitude_deg) ||
        !std::isfinite(height_m)) {
        return false;
    }

    seed.time = gpstCalendarToTime(year, month, day, hour, minute, second);
    seed.position_ecef = libgnss::geodetic2ecef(latitude_deg * kDegreesToRadians,
                                                longitude_deg * kDegreesToRadians,
                                                height_m);
    return seed.position_ecef.norm() > 1e6;
}

bool parseSeedPositionLine(const std::string& line, SeedPosition& seed) {
    std::istringstream input(line);
    std::string first_token;
    if (!(input >> first_token)) {
        return false;
    }
    if (first_token[0] == '%' || first_token[0] == '#') {
        return false;
    }
    if (first_token.find('/') != std::string::npos) {
        return parseRtklibSeedPositionLine(line, seed);
    }

    int week = 0;
    try {
        std::size_t consumed = 0;
        week = std::stoi(first_token, &consumed);
        if (consumed != first_token.size()) {
            return false;
        }
    } catch (const std::exception&) {
        return false;
    }

    double tow = 0.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    if (!(input >> tow >> x >> y >> z)) {
        return false;
    }
    if (!std::isfinite(tow) || !std::isfinite(x) ||
        !std::isfinite(y) || !std::isfinite(z)) {
        return false;
    }

    seed.time = libgnss::GNSSTime(week, tow);
    seed.position_ecef = libgnss::Vector3d(x, y, z);
    return seed.position_ecef.norm() > 1e6;
}

std::vector<SeedPosition> readSeedPositions(const std::string& path) {
    std::ifstream input(path);
    if (!input.is_open()) {
        throw std::runtime_error("failed to open seed POS file: " + path);
    }

    std::vector<SeedPosition> seeds;
    std::string line;
    while (std::getline(input, line)) {
        SeedPosition seed;
        if (parseSeedPositionLine(line, seed)) {
            seeds.push_back(seed);
        }
    }
    if (seeds.empty()) {
        throw std::runtime_error(
            "seed POS file has no readable LibGNSS++ or RTKLIB POS rows: " + path);
    }

    std::sort(seeds.begin(),
              seeds.end(),
              [](const SeedPosition& lhs, const SeedPosition& rhs) {
                  return lhs.time < rhs.time;
              });
    return seeds;
}

bool findSeedPosition(const std::vector<SeedPosition>& seeds,
                      const libgnss::GNSSTime& time,
                      double tolerance_s,
                      double interpolation_max_gap_s,
                      std::size_t& cursor,
                      libgnss::Vector3d& position_ecef,
                      bool& interpolated) {
    interpolated = false;
    while (cursor < seeds.size() && seeds[cursor].time - time < -tolerance_s) {
        ++cursor;
    }

    std::size_t best_index = seeds.size();
    double best_abs_dt = tolerance_s;
    if (cursor < seeds.size()) {
        const double abs_dt = std::abs(seeds[cursor].time - time);
        if (abs_dt <= best_abs_dt) {
            best_index = cursor;
            best_abs_dt = abs_dt;
        }
    }
    if (cursor > 0) {
        const std::size_t previous_index = cursor - 1;
        const double abs_dt = std::abs(seeds[previous_index].time - time);
        if (abs_dt <= best_abs_dt) {
            best_index = previous_index;
        }
    }
    if (best_index == seeds.size()) {
        if (interpolation_max_gap_s <= 0.0 || cursor == 0 ||
            cursor >= seeds.size()) {
            return false;
        }
        const SeedPosition& lower = seeds[cursor - 1];
        const SeedPosition& upper = seeds[cursor];
        const double gap_s = upper.time - lower.time;
        const double lower_dt_s = time - lower.time;
        if (gap_s <= 0.0 || gap_s > interpolation_max_gap_s ||
            lower_dt_s < 0.0 || lower_dt_s > gap_s) {
            return false;
        }
        const double alpha = lower_dt_s / gap_s;
        position_ecef =
            lower.position_ecef +
            alpha * (upper.position_ecef - lower.position_ecef);
        interpolated = true;
        return position_ecef.norm() > 1e6;
    }

    position_ecef = seeds[best_index].position_ecef;
    return true;
}

void writeCsvDouble(std::ostream& output, double value) {
    if (std::isfinite(value)) {
        output << value;
    } else {
        output << "NaN";
    }
}

double robustHuberLoss(double whitened_error, double threshold) {
    const double abs_error = std::abs(whitened_error);
    if (abs_error <= threshold) {
        return 0.5 * whitened_error * whitened_error;
    }
    return threshold * (abs_error - 0.5 * threshold);
}

double robustHuberWeight(double whitened_error, double threshold) {
    const double abs_error = std::abs(whitened_error);
    if (abs_error <= threshold || abs_error <= 0.0) {
        return 1.0;
    }
    return threshold / abs_error;
}

int positionColumn(std::size_t epoch_index, int component) {
    return static_cast<int>(kStateStride * epoch_index +
                            static_cast<std::size_t>(component));
}

int clockColumn(std::size_t epoch_index, std::size_t group) {
    return static_cast<int>(kStateStride * epoch_index + 3U + group);
}

int velocityColumn(std::size_t epoch_index, int component) {
    return static_cast<int>(kStateStride * epoch_index + 8U +
                            static_cast<std::size_t>(component));
}

int driftColumn(std::size_t epoch_index) {
    return static_cast<int>(kStateStride * epoch_index + 11U);
}

double pseudorangePrediction(const Eigen::VectorXd& state,
                             const PseudorangeFactor& factor) {
    const std::size_t i = factor.epoch_index;
    double prediction =
        factor.los(0) * state(positionColumn(i, 0)) +
        factor.los(1) * state(positionColumn(i, 1)) +
        factor.los(2) * state(positionColumn(i, 2)) +
        state(clockColumn(i, 0));
    if (factor.clock_group != 0U) {
        prediction += state(clockColumn(i, factor.clock_group));
    }
    return prediction;
}

double dopplerPrediction(const Eigen::VectorXd& state, const DopplerFactor& factor) {
    const std::size_t i = factor.epoch_index;
    return factor.los(0) * state(velocityColumn(i, 0)) +
           factor.los(1) * state(velocityColumn(i, 1)) +
           factor.los(2) * state(velocityColumn(i, 2)) +
           state(driftColumn(i));
}

double tdcpPrediction(const Eigen::VectorXd& state, const TdcpFactor& factor) {
    const std::size_t i = factor.epoch_index;
    const std::size_t previous = factor.previous_epoch_index;
    return factor.los(0) *
               (state(positionColumn(i, 0)) -
                state(positionColumn(previous, 0))) +
           factor.los(1) *
               (state(positionColumn(i, 1)) -
                state(positionColumn(previous, 1))) +
           factor.los(2) *
               (state(positionColumn(i, 2)) -
                state(positionColumn(previous, 2))) +
           state(clockColumn(i, 0)) -
           state(clockColumn(previous, 0));
}

double computeCost(const Problem& problem,
                   const Options& options,
                   const Eigen::VectorXd& state) {
    double cost = 0.0;
    for (std::size_t i = 0; i < problem.epochs.size(); ++i) {
        for (int component = 0; component < 3; ++component) {
            const double e =
                state(positionColumn(i, component)) /
                options.position_prior_sigma_m;
            cost += 0.5 * e * e;
        }
        for (std::size_t group = 0; group < kClockGroups; ++group) {
            const double e =
                state(clockColumn(i, group)) / options.clock_prior_sigma_m;
            cost += 0.5 * e * e;
        }
        for (int component = 0; component < 3; ++component) {
            const double e =
                state(velocityColumn(i, component)) /
                options.velocity_prior_sigma_mps;
            cost += 0.5 * e * e;
        }
        const double d_prior =
            state(driftColumn(i)) / options.clock_drift_prior_sigma_mps;
        cost += 0.5 * d_prior * d_prior;
        if (i > 0) {
            for (std::size_t group = 0; group < kClockGroups; ++group) {
                const double sigma =
                    group == 0U
                        ? (problem.clock_jumps[i]
                               ? options.clock_jump_sigma_m
                               : options.clock_motion_sigma_m)
                        : options.inter_system_clock_motion_sigma_m;
                double prediction =
                    state(clockColumn(i, group)) -
                    state(clockColumn(i - 1U, group));
                if (group == 0U) {
                    prediction -= 0.5 *
                                  (state(driftColumn(i - 1U)) +
                                   state(driftColumn(i)));
                }
                const double e = prediction / sigma;
                cost += 0.5 * e * e;
            }

            for (int component = 0; component < 3; ++component) {
                const double seed_delta =
                    problem.seed_positions[i](component) -
                    problem.seed_positions[i - 1U](component);
                const double prediction =
                    seed_delta +
                    state(positionColumn(i, component)) -
                    state(positionColumn(i - 1U, component)) -
                    0.5 * (state(velocityColumn(i - 1U, component)) +
                           state(velocityColumn(i, component)));
                const double e = prediction / options.motion_sigma_m;
                cost += 0.5 * e * e;
            }

            const double between =
                (state(driftColumn(i)) - state(driftColumn(i - 1U))) /
                options.clock_drift_between_sigma_mps;
            cost += 0.5 * between * between;
        }
    }

    for (const PseudorangeFactor& factor : problem.pseudorange_factors) {
        const double error =
            (pseudorangePrediction(state, factor) - factor.residual_m) /
            factor.sigma_m;
        cost += robustHuberLoss(error, options.huber_threshold_sigma);
    }

    for (const DopplerFactor& factor : problem.factors) {
        const double error =
            (dopplerPrediction(state, factor) - factor.residual_mps) /
            factor.sigma_mps;
        cost += robustHuberLoss(error, options.huber_threshold_sigma);
    }
    for (const TdcpFactor& factor : problem.tdcp_factors) {
        const double error =
            (tdcpPrediction(state, factor) - factor.residual_m) /
            factor.sigma_m;
        cost += robustHuberLoss(error, options.huber_threshold_sigma);
    }
    return cost;
}

struct NormalEquation {
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd rhs;
};

void addWeightedRow(std::vector<Eigen::Triplet<double>>& triplets,
                    Eigen::VectorXd& rhs,
                    const std::vector<int>& columns,
                    const std::vector<double>& coefficients,
                    double residual,
                    double sigma,
                    double robust_weight) {
    const double inv_variance = robust_weight / (sigma * sigma);
    for (std::size_t a = 0; a < columns.size(); ++a) {
        const double weighted_a = inv_variance * coefficients[a];
        rhs(columns[a]) += weighted_a * residual;
        for (std::size_t b = 0; b < columns.size(); ++b) {
            triplets.emplace_back(columns[a],
                                  columns[b],
                                  weighted_a * coefficients[b]);
        }
    }
}

NormalEquation buildNormalEquation(const Problem& problem,
                                   const Options& options,
                                   const Eigen::VectorXd& state) {
    const int state_size = static_cast<int>(kStateStride * problem.epochs.size());
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve((problem.pseudorange_factors.size() +
                      problem.factors.size() +
                      problem.tdcp_factors.size()) * 16U +
                     problem.epochs.size() * 80U);
    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(state_size);

    for (std::size_t i = 0; i < problem.epochs.size(); ++i) {
        for (int component = 0; component < 3; ++component) {
            const int col = positionColumn(i, component);
            addWeightedRow(triplets,
                           rhs,
                           {col},
                           {1.0},
                           -state(col),
                           options.position_prior_sigma_m,
                           1.0);
        }
        for (std::size_t group = 0; group < kClockGroups; ++group) {
            const int col = clockColumn(i, group);
            addWeightedRow(triplets,
                           rhs,
                           {col},
                           {1.0},
                           -state(col),
                           options.clock_prior_sigma_m,
                           1.0);
        }
        for (int component = 0; component < 3; ++component) {
            const int col = velocityColumn(i, component);
            addWeightedRow(triplets,
                           rhs,
                           {col},
                           {1.0},
                           -state(col),
                           options.velocity_prior_sigma_mps,
                           1.0);
        }

        const int d_col = driftColumn(i);
        addWeightedRow(triplets,
                       rhs,
                       {d_col},
                       {1.0},
                       -state(d_col),
                       options.clock_drift_prior_sigma_mps,
                       1.0);

        if (i > 0) {
            for (std::size_t group = 0; group < kClockGroups; ++group) {
                const int previous = clockColumn(i - 1U, group);
                const int current = clockColumn(i, group);
                const double sigma =
                    group == 0U
                        ? (problem.clock_jumps[i]
                               ? options.clock_jump_sigma_m
                               : options.clock_motion_sigma_m)
                        : options.inter_system_clock_motion_sigma_m;
                std::vector<int> columns{previous, current};
                std::vector<double> coeffs{-1.0, 1.0};
                double prediction = state(current) - state(previous);
                if (group == 0U) {
                    const int previous_drift = driftColumn(i - 1U);
                    const int current_drift = driftColumn(i);
                    columns.push_back(previous_drift);
                    columns.push_back(current_drift);
                    coeffs.push_back(-0.5);
                    coeffs.push_back(-0.5);
                    prediction -= 0.5 *
                                  (state(previous_drift) + state(current_drift));
                }
                addWeightedRow(triplets,
                               rhs,
                               columns,
                               coeffs,
                               -prediction,
                               sigma,
                               1.0);
            }

            for (int component = 0; component < 3; ++component) {
                const int previous_position = positionColumn(i - 1U, component);
                const int current_position = positionColumn(i, component);
                const int previous_velocity = velocityColumn(i - 1U, component);
                const int current_velocity = velocityColumn(i, component);
                const double seed_delta =
                    problem.seed_positions[i](component) -
                    problem.seed_positions[i - 1U](component);
                const double prediction =
                    seed_delta +
                    state(current_position) -
                    state(previous_position) -
                    0.5 * (state(previous_velocity) + state(current_velocity));
                addWeightedRow(triplets,
                               rhs,
                               {previous_position,
                                current_position,
                                previous_velocity,
                                current_velocity},
                               {-1.0, 1.0, -0.5, -0.5},
                               -prediction,
                               options.motion_sigma_m,
                               1.0);
            }

            const int previous = driftColumn(i - 1U);
            const int current = driftColumn(i);
            const double prediction = state(current) - state(previous);
            addWeightedRow(triplets,
                           rhs,
                           {previous, current},
                           {-1.0, 1.0},
                           -prediction,
                           options.clock_drift_between_sigma_mps,
                           1.0);
        }
    }

    for (const PseudorangeFactor& factor : problem.pseudorange_factors) {
        const std::size_t i = factor.epoch_index;
        const double prediction = pseudorangePrediction(state, factor);
        const double residual = factor.residual_m - prediction;
        const double whitened_error = -residual / factor.sigma_m;
        const double robust_weight =
            robustHuberWeight(whitened_error, options.huber_threshold_sigma);
        std::vector<int> columns{
            positionColumn(i, 0),
            positionColumn(i, 1),
            positionColumn(i, 2),
            clockColumn(i, 0),
        };
        std::vector<double> coeffs{
            factor.los(0),
            factor.los(1),
            factor.los(2),
            1.0,
        };
        if (factor.clock_group != 0U) {
            columns.push_back(clockColumn(i, factor.clock_group));
            coeffs.push_back(1.0);
        }
        addWeightedRow(triplets,
                       rhs,
                       columns,
                       coeffs,
                       residual,
                       factor.sigma_m,
                       robust_weight);
    }

    for (const DopplerFactor& factor : problem.factors) {
        const std::size_t i = factor.epoch_index;
        const double prediction = dopplerPrediction(state, factor);
        const double residual = factor.residual_mps - prediction;
        const double whitened_error = -residual / factor.sigma_mps;
        const double robust_weight =
            robustHuberWeight(whitened_error, options.huber_threshold_sigma);
        addWeightedRow(triplets,
                       rhs,
                       {velocityColumn(i, 0),
                        velocityColumn(i, 1),
                        velocityColumn(i, 2),
                        driftColumn(i)},
                       {factor.los(0), factor.los(1), factor.los(2), 1.0},
                       residual,
                       factor.sigma_mps,
                       robust_weight);
    }

    for (const TdcpFactor& factor : problem.tdcp_factors) {
        const std::size_t i = factor.epoch_index;
        const std::size_t previous = factor.previous_epoch_index;
        const double prediction = tdcpPrediction(state, factor);
        const double residual = factor.residual_m - prediction;
        const double whitened_error = -residual / factor.sigma_m;
        const double robust_weight =
            robustHuberWeight(whitened_error, options.huber_threshold_sigma);
        addWeightedRow(triplets,
                       rhs,
                       {positionColumn(previous, 0),
                        positionColumn(i, 0),
                        positionColumn(previous, 1),
                        positionColumn(i, 1),
                        positionColumn(previous, 2),
                        positionColumn(i, 2),
                        clockColumn(previous, 0),
                        clockColumn(i, 0)},
                       {-factor.los(0),
                        factor.los(0),
                        -factor.los(1),
                        factor.los(1),
                        -factor.los(2),
                        factor.los(2),
                        -1.0,
                        1.0},
                       residual,
                       factor.sigma_m,
                       robust_weight);
    }

    NormalEquation normal;
    normal.hessian.resize(state_size, state_size);
    normal.hessian.setFromTriplets(triplets.begin(), triplets.end());
    normal.rhs = std::move(rhs);
    return normal;
}

double residualRms(const Problem& problem, const Eigen::VectorXd& state) {
    const std::size_t count =
        problem.pseudorange_factors.size() +
        problem.factors.size() +
        problem.tdcp_factors.size();
    if (count == 0U) {
        return 0.0;
    }
    double sum_sq = 0.0;
    for (const PseudorangeFactor& factor : problem.pseudorange_factors) {
        const double residual =
            pseudorangePrediction(state, factor) - factor.residual_m;
        sum_sq += residual * residual;
    }
    for (const DopplerFactor& factor : problem.factors) {
        const double residual =
            dopplerPrediction(state, factor) - factor.residual_mps;
        sum_sq += residual * residual;
    }
    for (const TdcpFactor& factor : problem.tdcp_factors) {
        const double residual =
            tdcpPrediction(state, factor) - factor.residual_m;
        sum_sq += residual * residual;
    }
    return std::sqrt(sum_sq / static_cast<double>(count));
}

SolveResult solveProblem(const Problem& problem, const Options& options) {
    SolveResult result;
    result.state =
        Eigen::VectorXd::Zero(static_cast<int>(kStateStride * problem.epochs.size()));
    result.initial_cost = computeCost(problem, options, result.state);

    if (options.debug_problem_only || options.max_iterations == 0 ||
        problem.epochs.empty()) {
        result.final_cost = result.initial_cost;
        result.residual_rms_mps = residualRms(problem, result.state);
        return result;
    }

    double previous_cost = result.initial_cost;
    double damping = 1e-3;
    for (int iteration = 1; iteration <= options.max_iterations; ++iteration) {
        NormalEquation normal = buildNormalEquation(problem, options, result.state);
        bool accepted = false;
        double cost = previous_cost;
        Eigen::VectorXd candidate_state = result.state;
        Eigen::VectorXd accepted_delta =
            Eigen::VectorXd::Zero(result.state.size());
        const Eigen::VectorXd diagonal =
            normal.hessian.diagonal().cwiseAbs().cwiseMax(1.0);
        for (int damping_attempt = 0; damping_attempt < 18; ++damping_attempt) {
            Eigen::SparseMatrix<double> damped_hessian = normal.hessian;
            for (int col = 0; col < damped_hessian.cols(); ++col) {
                damped_hessian.coeffRef(col, col) += damping * diagonal(col);
            }
            damped_hessian.makeCompressed();

            Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
            solver.compute(damped_hessian);
            if (solver.info() != Eigen::Success) {
                damping *= 10.0;
                continue;
            }
            const Eigen::VectorXd delta = solver.solve(normal.rhs);
            if (solver.info() != Eigen::Success || !delta.allFinite()) {
                damping *= 10.0;
                continue;
            }

            double step_scale = 1.0;
            for (int line_search_attempt = 0; line_search_attempt < 12;
                 ++line_search_attempt) {
                candidate_state = result.state + step_scale * delta;
                cost = computeCost(problem, options, candidate_state);
                if (std::isfinite(cost) && cost <= previous_cost) {
                    accepted_delta = step_scale * delta;
                    accepted = true;
                    break;
                }
                step_scale *= 0.5;
            }
            if (accepted) {
                damping = std::max(1e-12, damping * 0.3);
                break;
            }
            damping *= 10.0;
        }
        if (!accepted) {
            break;
        }

        result.state = std::move(candidate_state);
        result.iterations = iteration;

        const double absolute_decrease = previous_cost - cost;
        const double relative_decrease =
            absolute_decrease / std::max(1.0, std::abs(previous_cost));
        previous_cost = cost;

        if (accepted_delta.norm() < 1e-10 ||
            (iteration > 1 && absolute_decrease >= 0.0 &&
             relative_decrease < 1e-5)) {
            result.converged = true;
            break;
        }
    }

    result.final_cost = computeCost(problem, options, result.state);
    result.residual_rms_mps = residualRms(problem, result.state);
    return result;
}

bool calculateObservationModel(const libgnss::ObservationData& epoch,
                               const libgnss::Observation& observation,
                               const libgnss::NavigationData& nav,
                               const libgnss::Vector3d& receiver_position,
                               const Options& options,
                               libgnss::Vector3d& satellite_position,
                               libgnss::Vector3d& satellite_velocity,
                               double& satellite_clock_bias,
                               double& satellite_clock_drift,
                               const libgnss::Ephemeris*& eph,
                               libgnss::NavigationData::SatelliteGeometry& geometry,
                               libgnss::Vector3d& ex,
                               double& range_m) {
    if (!isPrimaryPdSignal(observation.signal) ||
        !observation.valid ||
        !observation.has_pseudorange ||
        observation.pseudorange <= 0.0 ||
        observation.snr < options.min_snr_dbhz) {
        return false;
    }

    libgnss::GNSSTime transmit_time =
        epoch.time - observation.pseudorange / libgnss::constants::SPEED_OF_LIGHT;
    if (!nav.calculateSatelliteState(observation.satellite,
                                     transmit_time,
                                     satellite_position,
                                     satellite_velocity,
                                     satellite_clock_bias,
                                     satellite_clock_drift)) {
        return false;
    }

    transmit_time = transmit_time - satellite_clock_bias;
    if (!nav.calculateSatelliteState(observation.satellite,
                                     transmit_time,
                                     satellite_position,
                                     satellite_velocity,
                                     satellite_clock_bias,
                                     satellite_clock_drift)) {
        return false;
    }

    eph = nav.getEphemeris(observation.satellite, transmit_time);
    if (!eph || !isHealthyForPositioning(observation, *eph)) {
        return false;
    }

    const libgnss::Vector3d delta = satellite_position - receiver_position;
    range_m = delta.norm();
    if (range_m <= 0.0) {
        return false;
    }
    ex = delta / range_m;
    geometry = nav.calculateGeometry(receiver_position, satellite_position);
    return geometry.elevation >= options.min_elevation_deg * kDegreesToRadians;
}

bool preparePseudorangeFactor(const libgnss::ObservationData& epoch,
                              std::size_t epoch_index,
                              const libgnss::Observation& observation,
                              const libgnss::NavigationData& nav,
                              const libgnss::Vector3d& receiver_position,
                              const Options& options,
                              PseudorangeFactor& factor) {
    libgnss::Vector3d satellite_position;
    libgnss::Vector3d satellite_velocity;
    double satellite_clock_bias = 0.0;
    double satellite_clock_drift = 0.0;
    const libgnss::Ephemeris* eph = nullptr;
    libgnss::NavigationData::SatelliteGeometry geometry;
    libgnss::Vector3d ex = libgnss::Vector3d::Zero();
    double geometric_range = 0.0;
    if (!calculateObservationModel(epoch,
                                   observation,
                                   nav,
                                   receiver_position,
                                   options,
                                   satellite_position,
                                   satellite_velocity,
                                   satellite_clock_bias,
                                   satellite_clock_drift,
                                   eph,
                                   geometry,
                                   ex,
                                   geometric_range)) {
        return false;
    }

    double receiver_lat = 0.0;
    double receiver_lon = 0.0;
    double receiver_height = 0.0;
    libgnss::ecef2geodetic(receiver_position,
                           receiver_lat,
                           receiver_lon,
                           receiver_height);
    double ionosphere_delay = 0.0;
    if (nav.ionosphere_model.valid) {
        ionosphere_delay = libgnss::models::ionoDelayKlobuchar(
            receiver_lat,
            receiver_lon,
            geometry.azimuth,
            geometry.elevation,
            epoch.time.tow,
            nav.ionosphere_model.alpha,
            nav.ionosphere_model.beta);
        const double frequency_hz =
            libgnss::signalFrequencyHz(observation.signal, eph);
        if (frequency_hz > 0.0) {
            const double scale = libgnss::constants::GPS_L1_FREQ / frequency_hz;
            ionosphere_delay *= scale * scale;
        }
    }
    const double troposphere_delay =
        libgnss::models::tropDelaySaastamoinen(receiver_position,
                                               geometry.elevation);
    const double satellite_clock_m =
        satellite_clock_bias * libgnss::constants::SPEED_OF_LIGHT;
    const double group_delay_m = groupDelayCorrectionMeters(observation, *eph);
    const double modeled_range =
        geometric_range + sagnacRangeCorrection(satellite_position, receiver_position);
    const double corrected_pseudorange =
        observation.pseudorange +
        satellite_clock_m -
        ionosphere_delay -
        troposphere_delay -
        group_delay_m;
    const double residual = corrected_pseudorange - modeled_range;
    const double sin_el = std::sin(geometry.elevation);
    if (sin_el <= 0.0) {
        return false;
    }

    factor.epoch_index = epoch_index;
    factor.time = epoch.time;
    factor.satellite = observation.satellite;
    factor.signal = observation.signal;
    factor.clock_group = clockGroup(observation.satellite.system);
    factor.snr_dbhz = observation.snr;
    factor.elevation_rad = geometry.elevation;
    factor.sigma_m = options.pseudorange_sigma_zenith_m / std::sqrt(sin_el);
    factor.residual_m = residual;
    factor.corrected_pseudorange_m = corrected_pseudorange;
    factor.modeled_range_m = modeled_range;
    factor.ionosphere_delay_m = ionosphere_delay;
    factor.troposphere_delay_m = troposphere_delay;
    factor.satellite_clock_m = satellite_clock_m;
    factor.group_delay_m = group_delay_m;
    factor.los = -ex;
    return std::isfinite(factor.residual_m) &&
           std::isfinite(factor.sigma_m) &&
           factor.sigma_m > 0.0;
}

bool prepareCarrierResidual(const libgnss::ObservationData& epoch,
                            std::size_t epoch_index,
                            const libgnss::Observation& observation,
                            const libgnss::NavigationData& nav,
                            const libgnss::Vector3d& receiver_position,
                            const Options& options,
                            CarrierResidual& residual_out) {
    if (!observation.has_carrier_phase ||
        observation.carrier_phase == 0.0 ||
        observation.loss_of_lock ||
        ((observation.lli & 0x01U) != 0U)) {
        return false;
    }

    libgnss::Vector3d satellite_position;
    libgnss::Vector3d satellite_velocity;
    double satellite_clock_bias = 0.0;
    double satellite_clock_drift = 0.0;
    const libgnss::Ephemeris* eph = nullptr;
    libgnss::NavigationData::SatelliteGeometry geometry;
    libgnss::Vector3d ex = libgnss::Vector3d::Zero();
    double geometric_range = 0.0;
    if (!calculateObservationModel(epoch,
                                   observation,
                                   nav,
                                   receiver_position,
                                   options,
                                   satellite_position,
                                   satellite_velocity,
                                   satellite_clock_bias,
                                   satellite_clock_drift,
                                   eph,
                                   geometry,
                                   ex,
                                   geometric_range)) {
        return false;
    }

    double wavelength = libgnss::signalWavelengthMeters(observation.signal, eph);
    if (wavelength <= 0.0) {
        wavelength = libgnss::signalWavelengthMeters(observation);
    }
    if (wavelength <= 0.0) {
        return false;
    }

    double receiver_lat = 0.0;
    double receiver_lon = 0.0;
    double receiver_height = 0.0;
    libgnss::ecef2geodetic(receiver_position,
                           receiver_lat,
                           receiver_lon,
                           receiver_height);
    double ionosphere_delay = 0.0;
    if (nav.ionosphere_model.valid) {
        ionosphere_delay = libgnss::models::ionoDelayKlobuchar(
            receiver_lat,
            receiver_lon,
            geometry.azimuth,
            geometry.elevation,
            epoch.time.tow,
            nav.ionosphere_model.alpha,
            nav.ionosphere_model.beta);
        const double frequency_hz =
            libgnss::signalFrequencyHz(observation.signal, eph);
        if (frequency_hz > 0.0) {
            const double scale = libgnss::constants::GPS_L1_FREQ / frequency_hz;
            ionosphere_delay *= scale * scale;
        }
    }
    const double troposphere_delay =
        libgnss::models::tropDelaySaastamoinen(receiver_position,
                                               geometry.elevation);
    const double satellite_clock_m =
        satellite_clock_bias * libgnss::constants::SPEED_OF_LIGHT;
    const double modeled_range =
        geometric_range + sagnacRangeCorrection(satellite_position, receiver_position);
    const double corrected_carrier =
        observation.carrier_phase * wavelength +
        satellite_clock_m -
        troposphere_delay +
        ionosphere_delay;
    const double residual = corrected_carrier - modeled_range;

    residual_out.epoch_index = epoch_index;
    residual_out.time = epoch.time;
    residual_out.satellite = observation.satellite;
    residual_out.signal = observation.signal;
    residual_out.snr_dbhz = observation.snr;
    residual_out.elevation_rad = geometry.elevation;
    residual_out.residual_m = residual;
    residual_out.corrected_carrier_m = corrected_carrier;
    residual_out.modeled_range_m = modeled_range;
    residual_out.raw_carrier_cycles = observation.carrier_phase;
    residual_out.wavelength_m = wavelength;
    residual_out.ionosphere_delay_m = ionosphere_delay;
    residual_out.troposphere_delay_m = troposphere_delay;
    residual_out.satellite_clock_m = satellite_clock_m;
    residual_out.lli = observation.lli;
    residual_out.loss_of_lock = observation.loss_of_lock;
    residual_out.los = -ex;
    return std::isfinite(residual_out.residual_m) &&
           std::isfinite(residual_out.wavelength_m) &&
           residual_out.wavelength_m > 0.0;
}

bool prepareDopplerFactor(const libgnss::ObservationData& epoch,
                          std::size_t epoch_index,
                          const libgnss::Observation& observation,
                          const libgnss::NavigationData& nav,
                          const libgnss::Vector3d& receiver_position,
                          const Options& options,
                          DopplerFactor& factor) {
    if (!observation.has_doppler) {
        return false;
    }

    libgnss::Vector3d satellite_position;
    libgnss::Vector3d satellite_velocity;
    double satellite_clock_bias = 0.0;
    double satellite_clock_drift = 0.0;
    const libgnss::Ephemeris* eph = nullptr;
    libgnss::NavigationData::SatelliteGeometry geometry;
    libgnss::Vector3d ex = libgnss::Vector3d::Zero();
    double range = 0.0;
    if (!calculateObservationModel(epoch,
                                   observation,
                                   nav,
                                   receiver_position,
                                   options,
                                   satellite_position,
                                   satellite_velocity,
                                   satellite_clock_bias,
                                   satellite_clock_drift,
                                   eph,
                                   geometry,
                                   ex,
                                   range)) {
        return false;
    }

    double wavelength = libgnss::signalWavelengthMeters(observation.signal, eph);
    if (wavelength <= 0.0) {
        wavelength = libgnss::signalWavelengthMeters(observation);
    }
    if (wavelength <= 0.0) {
        return false;
    }

    const double sagnac_rate =
        libgnss::constants::OMEGA_E / libgnss::constants::SPEED_OF_LIGHT *
        (satellite_velocity(1) * receiver_position(0) -
         satellite_velocity(0) * receiver_position(1));
    const double modeled_range_rate = satellite_velocity.dot(ex) + sagnac_rate;
    const double satellite_clock_drift_mps =
        satellite_clock_drift * libgnss::constants::SPEED_OF_LIGHT;
    const double measured_range_rate = -observation.doppler * wavelength;
    const double residual =
        measured_range_rate - (modeled_range_rate - satellite_clock_drift_mps);
    const double sin_el = std::sin(geometry.elevation);
    if (sin_el <= 0.0) {
        return false;
    }

    factor.epoch_index = epoch_index;
    factor.time = epoch.time;
    factor.satellite = observation.satellite;
    factor.signal = observation.signal;
    factor.snr_dbhz = observation.snr;
    factor.elevation_rad = geometry.elevation;
    factor.sigma_mps = options.doppler_sigma_zenith_mps / std::sqrt(sin_el);
    factor.residual_mps = residual;
    factor.measured_range_rate_mps = measured_range_rate;
    factor.modeled_range_rate_mps = modeled_range_rate;
    factor.satellite_clock_drift_mps = satellite_clock_drift_mps;
    factor.wavelength_m = wavelength;
    factor.los = -ex;
    return std::isfinite(factor.residual_mps) &&
           std::isfinite(factor.sigma_mps) &&
           factor.sigma_mps > 0.0;
}

Problem buildProblem(const Options& options) {
    libgnss::io::RINEXReader obs_reader;
    if (!obs_reader.open(options.obs_path)) {
        throw std::runtime_error("failed to open observation file: " + options.obs_path);
    }
    libgnss::io::RINEXReader::RINEXHeader obs_header;
    if (!obs_reader.readHeader(obs_header)) {
        throw std::runtime_error("failed to read observation header: " + options.obs_path);
    }

    libgnss::io::RINEXReader nav_reader;
    if (!nav_reader.open(options.nav_path)) {
        throw std::runtime_error("failed to open navigation file: " + options.nav_path);
    }
    libgnss::NavigationData nav_data;
    if (!nav_reader.readNavigationData(nav_data)) {
        throw std::runtime_error("failed to read navigation data: " + options.nav_path);
    }

    const std::vector<SeedPosition> seed_positions =
        readSeedPositions(options.seed_pos_path);
    std::size_t seed_cursor = 0;

    Problem problem;
    std::set<libgnss::SatelliteId> unique_satellites;
    const double fixed_interval_s =
        obs_header.interval > 0.0 ? obs_header.interval : 1.0;
    const double gap_tolerance_s = std::max(0.01, 0.5 * fixed_interval_s);
    std::map<libgnss::SatelliteId, double> previous_gps_pseudorange_by_satellite;
    bool have_previous_gps_pseudorange = false;
    std::vector<std::map<ObservationKey, CarrierResidual>> carrier_residuals_by_epoch;

    auto append_epoch = [&](libgnss::ObservationData epoch,
                            bool build_factors) -> bool {
        if (options.max_epochs > 0 &&
            static_cast<int>(problem.epochs.size()) >= options.max_epochs) {
            return false;
        }

        libgnss::Vector3d seed_position = libgnss::Vector3d::Zero();
        bool interpolated = false;
        if (findSeedPosition(seed_positions,
                             epoch.time,
                             options.seed_match_tolerance_s,
                             options.seed_interpolation_max_gap_s,
                             seed_cursor,
                             seed_position,
                             interpolated)) {
            epoch.receiver_position = seed_position;
            ++problem.seed_matched_epochs;
            if (interpolated) {
                ++problem.seed_interpolated_epochs;
            }
        } else if (obs_header.approximate_position.norm() > 1e6) {
            seed_position = obs_header.approximate_position;
            epoch.receiver_position = seed_position;
        } else {
            return true;
        }

        const std::size_t epoch_index = problem.epochs.size();
        std::map<libgnss::SatelliteId, double> gps_pseudorange_by_satellite;
        std::map<ObservationKey, CarrierResidual> carrier_residuals;
        if (build_factors) {
            for (const auto& observation : epoch.observations) {
                unique_satellites.insert(observation.satellite);
                PseudorangeFactor pseudorange_factor;
                if (preparePseudorangeFactor(epoch,
                                             epoch_index,
                                             observation,
                                             nav_data,
                                             seed_position,
                                             options,
                                             pseudorange_factor)) {
                    problem.pseudorange_factors.push_back(pseudorange_factor);
                    if (observation.satellite.system == libgnss::GNSSSystem::GPS) {
                        gps_pseudorange_by_satellite[observation.satellite] =
                            observation.pseudorange;
                    }
                }

                DopplerFactor factor;
                if (prepareDopplerFactor(epoch,
                                         epoch_index,
                                         observation,
                                         nav_data,
                                         seed_position,
                                         options,
                                         factor)) {
                    problem.factors.push_back(factor);
                }

                CarrierResidual carrier_residual;
                if (prepareCarrierResidual(epoch,
                                           epoch_index,
                                           observation,
                                           nav_data,
                                           seed_position,
                                           options,
                                           carrier_residual)) {
                    carrier_residuals[{observation.satellite, observation.signal}] =
                        carrier_residual;
                }
            }
        }

        bool clock_jump = false;
        if (have_previous_gps_pseudorange && !gps_pseudorange_by_satellite.empty()) {
            double sum_delta = 0.0;
            std::size_t count = 0;
            for (const auto& [satellite, pseudorange] : gps_pseudorange_by_satellite) {
                const auto previous_it =
                    previous_gps_pseudorange_by_satellite.find(satellite);
                if (previous_it == previous_gps_pseudorange_by_satellite.end()) {
                    continue;
                }
                sum_delta += pseudorange - previous_it->second;
                ++count;
            }
            if (count > 0U) {
                clock_jump = sum_delta / static_cast<double>(count) > 1e5;
            }
        }
        problem.clock_jumps.push_back(clock_jump);
        previous_gps_pseudorange_by_satellite = gps_pseudorange_by_satellite;
        have_previous_gps_pseudorange = !gps_pseudorange_by_satellite.empty();

        problem.seed_positions.push_back(seed_position);
        problem.epochs.push_back(epoch);
        carrier_residuals_by_epoch.push_back(std::move(carrier_residuals));
        return true;
    };

    libgnss::ObservationData epoch;
    int raw_epoch_index = 0;
    bool have_last_output_time = false;
    libgnss::GNSSTime last_output_time;
    while (obs_reader.readObservationEpoch(epoch)) {
        if (raw_epoch_index++ < options.skip_epochs) {
            continue;
        }

        if (have_last_output_time) {
            libgnss::GNSSTime expected_time =
                last_output_time + fixed_interval_s;
            while (epoch.time - expected_time > gap_tolerance_s) {
                libgnss::ObservationData empty_epoch(expected_time);
                if (!append_epoch(empty_epoch, false)) {
                    break;
                }
                last_output_time = expected_time;
                expected_time = last_output_time + fixed_interval_s;
                if (options.max_epochs > 0 &&
                    static_cast<int>(problem.epochs.size()) >=
                        options.max_epochs) {
                    break;
                }
            }
            if (options.max_epochs > 0 &&
                static_cast<int>(problem.epochs.size()) >= options.max_epochs) {
                break;
            }
        }

        if (!append_epoch(epoch, true)) {
            break;
        }
        last_output_time = epoch.time;
        have_last_output_time = true;
    }
    problem.nsat = unique_satellites.size();

    for (std::size_t i = 1; i < problem.epochs.size(); ++i) {
        if (problem.clock_jumps[i]) {
            continue;
        }
        const auto& previous_carriers = carrier_residuals_by_epoch[i - 1U];
        const auto& current_carriers = carrier_residuals_by_epoch[i];
        for (const auto& [key, current] : current_carriers) {
            const auto previous_it = previous_carriers.find(key);
            if (previous_it == previous_carriers.end()) {
                continue;
            }
            const CarrierResidual& previous = previous_it->second;
            const double sin_el = std::sin(current.elevation_rad);
            if (sin_el <= 0.0) {
                continue;
            }

            TdcpFactor factor;
            factor.epoch_index = i;
            factor.previous_epoch_index = i - 1U;
            factor.time = current.time;
            factor.satellite = current.satellite;
            factor.signal = current.signal;
            factor.snr_dbhz = current.snr_dbhz;
            factor.elevation_rad = current.elevation_rad;
            factor.previous_elevation_rad = previous.elevation_rad;
            factor.sigma_m = options.tdcp_sigma_zenith_m / std::sqrt(sin_el);
            factor.residual_m = current.residual_m - previous.residual_m;
            factor.previous_carrier_residual_m = previous.residual_m;
            factor.current_carrier_residual_m = current.residual_m;
            factor.previous_raw_carrier_cycles = previous.raw_carrier_cycles;
            factor.current_raw_carrier_cycles = current.raw_carrier_cycles;
            factor.wavelength_m = current.wavelength_m;
            factor.los = previous.los;
            if (std::isfinite(factor.residual_m) &&
                std::isfinite(factor.sigma_m) &&
                factor.sigma_m > 0.0) {
                problem.tdcp_factors.push_back(factor);
                unique_satellites.insert(factor.satellite);
            }
        }
    }
    problem.nsat = unique_satellites.size();
    return problem;
}

bool writePerEpochCsv(const std::string& path,
                      const Problem& problem,
                      const SolveResult& result) {
    if (path.empty()) {
        return true;
    }
    std::ofstream output(path);
    if (!output.is_open()) {
        return false;
    }
    output << "epoch,gps_week,gps_tow,spp_x_m,spp_y_m,spp_z_m,"
              "fgo_x_m,fgo_y_m,fgo_z_m,"
              "fgo_vx_mps,fgo_vy_mps,fgo_vz_mps,"
              "fgo_c_gps_m,fgo_c_glo_m,fgo_c_gal_m,fgo_c_qzs_m,"
              "fgo_c_bds_m,fgo_clock_drift_mps,clock_jump\n";
    output << std::fixed << std::setprecision(15);
    for (std::size_t i = 0; i < problem.epochs.size(); ++i) {
        const auto& epoch = problem.epochs[i];
        const auto& seed = problem.seed_positions[i];
        output << (i + 1U) << ','
               << epoch.time.week << ','
               << epoch.time.tow << ','
               << seed(0) << ','
               << seed(1) << ','
               << seed(2) << ','
               << seed(0) + result.state(positionColumn(i, 0)) << ','
               << seed(1) + result.state(positionColumn(i, 1)) << ','
               << seed(2) + result.state(positionColumn(i, 2)) << ','
               << result.state(velocityColumn(i, 0)) << ','
               << result.state(velocityColumn(i, 1)) << ','
               << result.state(velocityColumn(i, 2)) << ','
               << result.state(clockColumn(i, 0)) << ','
               << result.state(clockColumn(i, 1)) << ','
               << result.state(clockColumn(i, 2)) << ','
               << result.state(clockColumn(i, 3)) << ','
               << result.state(clockColumn(i, 4)) << ','
               << result.state(driftColumn(i)) << ','
               << (problem.clock_jumps[i] ? 1 : 0) << '\n';
    }
    return true;
}

bool writeFactorDebugCsv(const std::string& path,
                         const Problem& problem,
                         const SolveResult& result) {
    if (path.empty()) {
        return true;
    }
    std::ofstream output(path);
    if (!output.is_open()) {
        return false;
    }
    output << "factor_type,epoch_index,gps_week,gps_tow,satellite,signal,"
              "previous_epoch_index,clock_group,snr_dbhz,elevation_deg,"
              "previous_elevation_deg,sigma_p_m,sigma_d_mps,sigma_c_m,"
              "res_pc_m,res_d_mps,res_tdcp_m,measured_range_rate_mps,"
              "modeled_range_rate_mps,satellite_clock_drift_mps,"
              "previous_carrier_residual_m,current_carrier_residual_m,"
              "previous_raw_carrier_cycles,current_raw_carrier_cycles,"
              "wavelength_m,final_residual_m,los_x,los_y,los_z\n";
    output << std::fixed << std::setprecision(15);
    for (const PseudorangeFactor& factor : problem.pseudorange_factors) {
        const double final_residual =
            pseudorangePrediction(result.state, factor) - factor.residual_m;
        output << "P,"
               << factor.epoch_index << ','
               << factor.time.week << ','
               << factor.time.tow << ','
               << factor.satellite.toString() << ','
               << signalName(factor.signal) << ','
               << "NaN,"
               << factor.clock_group << ','
               << factor.snr_dbhz << ','
               << factor.elevation_rad * kRadiansToDegrees << ','
               << "NaN,"
               << factor.sigma_m << ",NaN,NaN,"
               << factor.residual_m << ",NaN,NaN,NaN,NaN,NaN,"
               << "NaN,NaN,NaN,NaN,NaN,"
               << final_residual << ','
               << factor.los(0) << ','
               << factor.los(1) << ','
               << factor.los(2) << '\n';
    }
    for (const DopplerFactor& factor : problem.factors) {
        const double final_residual =
            dopplerPrediction(result.state, factor) - factor.residual_mps;
        output << "D,"
               << factor.epoch_index << ','
               << factor.time.week << ','
               << factor.time.tow << ','
               << factor.satellite.toString() << ','
               << signalName(factor.signal) << ','
               << "NaN,"
               << clockGroup(factor.satellite.system) << ','
               << factor.snr_dbhz << ','
               << factor.elevation_rad * kRadiansToDegrees << ','
               << "NaN,"
               << "NaN,"
               << factor.sigma_mps << ",NaN,"
               << "NaN,"
               << factor.residual_mps << ','
               << "NaN,"
               << factor.measured_range_rate_mps << ','
               << factor.modeled_range_rate_mps << ','
               << factor.satellite_clock_drift_mps << ','
               << "NaN,NaN,NaN,NaN,"
               << factor.wavelength_m << ','
               << final_residual << ','
               << factor.los(0) << ','
               << factor.los(1) << ','
               << factor.los(2) << '\n';
    }
    for (const TdcpFactor& factor : problem.tdcp_factors) {
        const double final_residual =
            tdcpPrediction(result.state, factor) - factor.residual_m;
        output << "C,"
               << factor.epoch_index << ','
               << factor.time.week << ','
               << factor.time.tow << ','
               << factor.satellite.toString() << ','
               << signalName(factor.signal) << ','
               << factor.previous_epoch_index << ','
               << 0 << ','
               << factor.snr_dbhz << ','
               << factor.elevation_rad * kRadiansToDegrees << ','
               << factor.previous_elevation_rad * kRadiansToDegrees << ','
               << "NaN,NaN,"
               << factor.sigma_m << ','
               << "NaN,NaN,"
               << factor.residual_m << ','
               << "NaN,NaN,NaN,"
               << factor.previous_carrier_residual_m << ','
               << factor.current_carrier_residual_m << ','
               << factor.previous_raw_carrier_cycles << ','
               << factor.current_raw_carrier_cycles << ','
               << factor.wavelength_m << ','
               << final_residual << ','
               << factor.los(0) << ','
               << factor.los(1) << ','
               << factor.los(2) << '\n';
    }
    return true;
}

std::size_t graphFactorCount(const Problem& problem) {
    if (problem.epochs.empty()) {
        return problem.pseudorange_factors.size() +
               problem.factors.size() +
               problem.tdcp_factors.size();
    }
    return problem.pseudorange_factors.size() +
           problem.factors.size() +
           problem.tdcp_factors.size() +
           4U * problem.epochs.size() +
           3U * (problem.epochs.size() - 1U);
}

bool writeGraphCsv(const std::string& path,
                   const Problem& problem,
                   const SolveResult& result) {
    if (path.empty()) {
        return true;
    }
    std::ofstream output(path);
    if (!output.is_open()) {
        return false;
    }
    output << "n,nsat,graph_factors,graph_values,initial_cost,final_cost,"
              "optimizer_error,iterations,valid_position_epochs,"
              "valid_velocity_epochs\n";
    output << std::fixed << std::setprecision(15)
           << problem.epochs.size() << ','
           << problem.nsat << ','
           << graphFactorCount(problem) << ','
           << 4U * problem.epochs.size() << ','
           << result.initial_cost << ','
           << result.final_cost << ','
           << result.final_cost << ','
           << result.iterations << ','
           << problem.epochs.size() << ','
           << problem.epochs.size() << '\n';
    return true;
}

std::string jsonBool(bool value) {
    return value ? "true" : "false";
}

bool writeSummaryJson(const std::string& path,
                      const Options& options,
                      const Problem& problem,
                      const SolveResult& result) {
    if (path.empty()) {
        return true;
    }
    std::ofstream output(path);
    if (!output.is_open()) {
        return false;
    }
    const std::size_t epoch_count = problem.epochs.size();
    output << std::fixed << std::setprecision(15)
           << "{\n"
           << "  \"preset\": \"taroz-pdc\",\n"
           << "  \"backend\": \"eigen\",\n"
           << "  \"debug_problem_only\": " << jsonBool(options.debug_problem_only) << ",\n"
           << "  \"optimized_epochs\": " << epoch_count << ",\n"
           << "  \"valid_position_epochs\": " << epoch_count << ",\n"
           << "  \"valid_velocity_epochs\": " << epoch_count << ",\n"
           << "  \"seed_matched_epochs\": " << problem.seed_matched_epochs << ",\n"
           << "  \"seed_interpolated_epochs\": " << problem.seed_interpolated_epochs << ",\n"
           << "  \"nsat\": " << problem.nsat << ",\n"
           << "  \"pseudorange_factors\": " << problem.pseudorange_factors.size() << ",\n"
           << "  \"doppler_factors\": " << problem.factors.size() << ",\n"
           << "  \"tdcp_factors\": " << problem.tdcp_factors.size() << ",\n"
           << "  \"position_prior_factors\": " << epoch_count << ",\n"
           << "  \"clock_prior_factors\": " << epoch_count << ",\n"
           << "  \"velocity_prior_factors\": " << epoch_count << ",\n"
           << "  \"clock_drift_prior_factors\": " << epoch_count << ",\n"
           << "  \"motion_factors\": "
           << (epoch_count > 0 ? epoch_count - 1U : 0U) << ",\n"
           << "  \"clock_motion_factors\": "
           << (epoch_count > 0 ? epoch_count - 1U : 0U) << ",\n"
           << "  \"clock_drift_between_factors\": "
           << (epoch_count > 0 ? epoch_count - 1U : 0U) << ",\n"
           << "  \"graph_factors\": " << graphFactorCount(problem) << ",\n"
           << "  \"graph_values\": " << 4U * epoch_count << ",\n"
           << "  \"min_snr_dbhz\": " << options.min_snr_dbhz << ",\n"
           << "  \"min_elevation_deg\": " << options.min_elevation_deg << ",\n"
           << "  \"pseudorange_sigma_zenith_m\": "
           << options.pseudorange_sigma_zenith_m << ",\n"
           << "  \"doppler_sigma_zenith_mps\": "
           << options.doppler_sigma_zenith_mps << ",\n"
           << "  \"tdcp_sigma_zenith_m\": "
           << options.tdcp_sigma_zenith_m << ",\n"
           << "  \"position_prior_sigma_m\": "
           << options.position_prior_sigma_m << ",\n"
           << "  \"clock_prior_sigma_m\": "
           << options.clock_prior_sigma_m << ",\n"
           << "  \"velocity_prior_sigma_mps\": "
           << options.velocity_prior_sigma_mps << ",\n"
           << "  \"clock_drift_prior_sigma_mps\": "
           << options.clock_drift_prior_sigma_mps << ",\n"
           << "  \"motion_sigma_m\": " << options.motion_sigma_m << ",\n"
           << "  \"clock_motion_sigma_m\": "
           << options.clock_motion_sigma_m << ",\n"
           << "  \"clock_jump_sigma_m\": " << options.clock_jump_sigma_m << ",\n"
           << "  \"clock_drift_between_sigma_mps\": "
           << options.clock_drift_between_sigma_mps << ",\n"
           << "  \"huber_threshold_sigma\": "
           << options.huber_threshold_sigma << ",\n"
           << "  \"iterations\": " << result.iterations << ",\n"
           << "  \"converged\": " << jsonBool(result.converged) << ",\n"
           << "  \"initial_cost\": " << result.initial_cost << ",\n"
           << "  \"final_cost\": " << result.final_cost << ",\n"
           << "  \"residual_rms_mps\": " << result.residual_rms_mps << "\n"
           << "}\n";
    return true;
}

void printSummary(const Problem& problem, const SolveResult& result) {
    std::cout << "Taroz PDC position/velocity batch complete\n"
              << "  epochs: " << problem.epochs.size() << "\n"
              << "  satellites: " << problem.nsat << "\n"
              << "  pseudorange_factors: "
              << problem.pseudorange_factors.size() << "\n"
              << "  doppler_factors: " << problem.factors.size() << "\n"
              << "  tdcp_factors: " << problem.tdcp_factors.size() << "\n"
              << "  graph_factors: " << graphFactorCount(problem) << "\n"
              << "  iterations: " << result.iterations << "\n"
              << "  converged: " << (result.converged ? "true" : "false") << "\n"
              << std::fixed << std::setprecision(6)
              << "  initial_cost: " << result.initial_cost << "\n"
              << "  final_cost: " << result.final_cost << "\n"
              << "  residual_rms_mps: " << result.residual_rms_mps << "\n";
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        const Options options = parseArguments(argc, argv);
        const Problem problem = buildProblem(options);
        const SolveResult result = solveProblem(problem, options);

        if (!writePerEpochCsv(options.out_csv_path, problem, result)) {
            std::cerr << "Error: failed to write velocity CSV: "
                      << options.out_csv_path << "\n";
            return 1;
        }
        if (!writeFactorDebugCsv(options.factor_debug_csv_path, problem, result)) {
            std::cerr << "Error: failed to write factor debug CSV: "
                      << options.factor_debug_csv_path << "\n";
            return 1;
        }
        if (!writeGraphCsv(options.graph_csv_path, problem, result)) {
            std::cerr << "Error: failed to write graph CSV: "
                      << options.graph_csv_path << "\n";
            return 1;
        }
        if (!writeSummaryJson(options.summary_json_path, options, problem, result)) {
            std::cerr << "Error: failed to write summary JSON: "
                      << options.summary_json_path << "\n";
            return 1;
        }
        if (!options.quiet) {
            printSummary(problem, result);
        }
        return 0;
    } catch (const std::invalid_argument&) {
        return 2;
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }
}
