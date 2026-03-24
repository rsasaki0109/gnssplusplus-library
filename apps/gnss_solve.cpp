#include <Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/gnss.hpp>
#include <libgnss++/io/rinex.hpp>
#include <libgnss++/io/solution_writer.hpp>
#include <libgnss++/models/troposphere.hpp>

namespace {

constexpr double kExactTimeToleranceSeconds = 1e-6;
constexpr double kDefaultFloatVsSppGuardMeters = 30.0;

enum class ModeChoice {
    AUTO,
    KINEMATIC,
    STATIC
};

enum class IonoChoice {
    AUTO,
    OFF,
    IFLC
};

struct SolveConfig {
    std::string data_dir;
    std::string rover_obs_path;
    std::string base_obs_path;
    std::string nav_path;
    std::string output_pos_path = "output/rtk_solution.pos";
    std::string output_kml_path = "output/rtk_solution.kml";
    bool write_kml = true;
    bool enable_base_interpolation = true;
    bool verbose = false;
    bool base_position_override = false;
    Eigen::Vector3d base_position_ecef = Eigen::Vector3d::Zero();
    ModeChoice mode = ModeChoice::AUTO;
    IonoChoice iono = IonoChoice::AUTO;
    libgnss::io::SolutionWriter::Format output_format = libgnss::io::SolutionWriter::Format::POS;
    double max_baseline_length_m = 20000.0;
    double ratio_threshold = 3.0;
    int min_satellites_for_ar = 5;
    double elevation_mask_deg = 15.0;
    int max_epochs = -1;
};

double timeDiffSeconds(const libgnss::GNSSTime& a, const libgnss::GNSSTime& b) {
    return a - b;
}

struct ObservationKey {
    libgnss::SatelliteId satellite;
    libgnss::SignalType signal;

    bool operator<(const ObservationKey& other) const {
        if (satellite < other.satellite) return true;
        if (other.satellite < satellite) return false;
        return signal < other.signal;
    }
};

std::map<ObservationKey, const libgnss::Observation*> indexObservations(
    const libgnss::ObservationData& epoch) {
    std::map<ObservationKey, const libgnss::Observation*> indexed;
    for (const auto& obs : epoch.observations) {
        indexed[{obs.satellite, obs.signal}] = &obs;
    }
    return indexed;
}

double signalWavelength(libgnss::SignalType signal) {
    switch (signal) {
        case libgnss::SignalType::GPS_L1CA:
            return libgnss::constants::GPS_L1_WAVELENGTH;
        case libgnss::SignalType::GPS_L2C:
            return libgnss::constants::GPS_L2_WAVELENGTH;
        default:
            return 0.0;
    }
}

bool calculateModeledBaseRange(const libgnss::SatelliteId& satellite,
                               const libgnss::GNSSTime& time,
                               double approx_pseudorange,
                               const libgnss::Vector3d& base_position,
                               const libgnss::NavigationData& nav,
                               double& modeled_range) {
    libgnss::Vector3d sat_pos;
    libgnss::Vector3d sat_vel;
    double sat_clk = 0.0;
    double sat_clk_drift = 0.0;

    const double travel_time = approx_pseudorange > 1.0
        ? approx_pseudorange / libgnss::constants::SPEED_OF_LIGHT
        : 0.075;
    libgnss::GNSSTime tx_time = time - travel_time;
    if (!nav.calculateSatelliteState(satellite, tx_time, sat_pos, sat_vel, sat_clk, sat_clk_drift)) {
        return false;
    }

    tx_time = tx_time - sat_clk;
    if (!nav.calculateSatelliteState(satellite, tx_time, sat_pos, sat_vel, sat_clk, sat_clk_drift)) {
        return false;
    }

    const auto geom = nav.calculateGeometry(base_position, sat_pos);
    if (geom.elevation <= 0.05) {
        return false;
    }

    modeled_range = libgnss::geodist(sat_pos, base_position) +
        libgnss::models::tropDelaySaastamoinen(base_position, geom.elevation);
    return std::isfinite(modeled_range);
}

bool interpolateBaseEpoch(const libgnss::ObservationData& before,
                          const libgnss::ObservationData& after,
                          const libgnss::GNSSTime& target_time,
                          const libgnss::Vector3d& base_position,
                          const libgnss::NavigationData& nav,
                          libgnss::ObservationData& interpolated_epoch) {
    constexpr double kMaxInterpolationGapSeconds = 2.0;
    const double total_dt = timeDiffSeconds(after.time, before.time);
    if (!std::isfinite(total_dt) || total_dt <= 1e-6 || total_dt > kMaxInterpolationGapSeconds) {
        return false;
    }

    const double alpha = timeDiffSeconds(target_time, before.time) / total_dt;
    if (!std::isfinite(alpha) || alpha < -1e-6 || alpha > 1.0 + 1e-6) {
        return false;
    }

    interpolated_epoch = libgnss::ObservationData(target_time);
    interpolated_epoch.receiver_position =
        before.receiver_position.norm() > 0.0 ? before.receiver_position : after.receiver_position;
    interpolated_epoch.receiver_clock_bias =
        (1.0 - alpha) * before.receiver_clock_bias + alpha * after.receiver_clock_bias;

    const auto before_obs = indexObservations(before);
    const auto after_obs = indexObservations(after);

    for (const auto& [key, obs_before_ptr] : before_obs) {
        const auto after_it = after_obs.find(key);
        if (after_it == after_obs.end()) continue;

        const auto& obs_before = *obs_before_ptr;
        const auto& obs_after = *after_it->second;
        const double wavelength = signalWavelength(key.signal);
        if (wavelength <= 0.0 || !obs_before.has_pseudorange || !obs_after.has_pseudorange) {
            continue;
        }

        const double approx_target_code =
            obs_before.pseudorange + alpha * (obs_after.pseudorange - obs_before.pseudorange);

        double modeled_before = 0.0;
        double modeled_after = 0.0;
        double modeled_target = 0.0;
        if (!calculateModeledBaseRange(key.satellite, before.time, obs_before.pseudorange,
                                       base_position, nav, modeled_before) ||
            !calculateModeledBaseRange(key.satellite, after.time, obs_after.pseudorange,
                                       base_position, nav, modeled_after) ||
            !calculateModeledBaseRange(key.satellite, target_time, approx_target_code,
                                       base_position, nav, modeled_target)) {
            continue;
        }

        libgnss::Observation obs(key.satellite, key.signal);
        obs.valid = obs_before.valid && obs_after.valid;
        obs.lli = alpha < 0.5 ? obs_before.lli : obs_after.lli;
        obs.code = alpha < 0.5 ? obs_before.code : obs_after.code;
        obs.signal_strength = std::max(obs_before.signal_strength, obs_after.signal_strength);
        obs.snr = (1.0 - alpha) * obs_before.snr + alpha * obs_after.snr;
        obs.loss_of_lock = obs_before.loss_of_lock || obs_after.loss_of_lock;

        const double code_residual_before = obs_before.pseudorange - modeled_before;
        const double code_residual_after = obs_after.pseudorange - modeled_after;
        obs.pseudorange = modeled_target +
            code_residual_before + alpha * (code_residual_after - code_residual_before);
        obs.has_pseudorange = std::isfinite(obs.pseudorange);

        if (obs_before.has_carrier_phase && obs_after.has_carrier_phase &&
            (obs_before.lli & 0x01) == 0 && (obs_after.lli & 0x01) == 0 && !obs.loss_of_lock) {
            const double phase_residual_before = obs_before.carrier_phase * wavelength - modeled_before;
            const double phase_residual_after = obs_after.carrier_phase * wavelength - modeled_after;
            const double phase_residual_target =
                phase_residual_before + alpha * (phase_residual_after - phase_residual_before);
            obs.carrier_phase = (modeled_target + phase_residual_target) / wavelength;
            obs.has_carrier_phase = std::isfinite(obs.carrier_phase);
        }

        if (obs_before.has_doppler && obs_after.has_doppler) {
            obs.doppler = obs_before.doppler + alpha * (obs_after.doppler - obs_before.doppler);
            obs.has_doppler = true;
        }

        if (obs.has_pseudorange || obs.has_carrier_phase || obs.has_doppler) {
            interpolated_epoch.addObservation(obs);
        }
    }

    return !interpolated_epoch.observations.empty();
}

std::string modeChoiceString(ModeChoice mode) {
    switch (mode) {
        case ModeChoice::AUTO:
            return "auto";
        case ModeChoice::KINEMATIC:
            return "kinematic";
        case ModeChoice::STATIC:
            return "static";
    }
    return "unknown";
}

std::string ionoChoiceString(IonoChoice iono) {
    switch (iono) {
        case IonoChoice::AUTO:
            return "auto";
        case IonoChoice::OFF:
            return "off";
        case IonoChoice::IFLC:
            return "iflc";
    }
    return "unknown";
}

std::string outputFormatString(libgnss::io::SolutionWriter::Format format) {
    switch (format) {
        case libgnss::io::SolutionWriter::Format::POS:
            return "pos";
        case libgnss::io::SolutionWriter::Format::LLH:
            return "llh";
        case libgnss::io::SolutionWriter::Format::XYZ:
            return "xyz";
    }
    return "pos";
}

void printUsage(const char* program_name) {
    std::cout
        << "Usage: " << program_name << " [options]\n"
        << "  --data-dir <dir>           Use <dir>/rover.obs, base.obs, navigation.nav\n"
        << "  --rover <file>             Rover RINEX observation file\n"
        << "  --base <file>              Base RINEX observation file\n"
        << "  --nav <file>               Navigation RINEX file\n"
        << "  --out <file>               Output solution file (default: output/rtk_solution.pos)\n"
        << "  --kml <file>               Write KML output (default: output/rtk_solution.kml)\n"
        << "  --no-kml                   Disable KML output\n"
        << "  --format <pos|llh|xyz>     Output text format (default: pos)\n"
        << "  --mode <auto|kinematic|static>\n"
        << "                             Position mode (default: auto)\n"
        << "  --iono <auto|off|iflc>     Ionosphere option (default: auto)\n"
        << "  --ratio <value>            Ambiguity ratio threshold (default: 3.0)\n"
        << "  --min-ar-sats <n>          Minimum satellites for AR (default: 5)\n"
        << "  --elevation-mask-deg <v>   Elevation mask in degrees (default: 15)\n"
        << "  --max-baseline-m <v>       Max baseline length in meters (default: 20000)\n"
        << "  --base-ecef <x> <y> <z>    Override base ECEF position in meters\n"
        << "  --max-epochs <n>           Stop after n rover epochs\n"
        << "  --no-base-interp           Require exact rover/base epoch alignment\n"
        << "  --verbose                  Print per-epoch progress summary\n"
        << "  -h, --help                 Show this help\n";
}

[[noreturn]] void argumentError(const std::string& message, const char* program_name) {
    std::cerr << "Argument error: " << message << "\n\n";
    printUsage(program_name);
    std::exit(1);
}

ModeChoice parseModeChoice(const std::string& value, const char* program_name) {
    if (value == "auto") return ModeChoice::AUTO;
    if (value == "kinematic") return ModeChoice::KINEMATIC;
    if (value == "static") return ModeChoice::STATIC;
    argumentError("unsupported --mode value: " + value, program_name);
}

IonoChoice parseIonoChoice(const std::string& value, const char* program_name) {
    if (value == "auto") return IonoChoice::AUTO;
    if (value == "off") return IonoChoice::OFF;
    if (value == "iflc") return IonoChoice::IFLC;
    argumentError("unsupported --iono value: " + value, program_name);
}

libgnss::io::SolutionWriter::Format parseOutputFormat(const std::string& value,
                                                      const char* program_name) {
    if (value == "pos") return libgnss::io::SolutionWriter::Format::POS;
    if (value == "llh") return libgnss::io::SolutionWriter::Format::LLH;
    if (value == "xyz") return libgnss::io::SolutionWriter::Format::XYZ;
    argumentError("unsupported --format value: " + value, program_name);
}

SolveConfig parseArguments(int argc, char* argv[]) {
    SolveConfig config;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];

        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            std::exit(0);
        }
        if (arg == "--data-dir" && i + 1 < argc) {
            config.data_dir = argv[++i];
        } else if (arg == "--rover" && i + 1 < argc) {
            config.rover_obs_path = argv[++i];
        } else if (arg == "--base" && i + 1 < argc) {
            config.base_obs_path = argv[++i];
        } else if (arg == "--nav" && i + 1 < argc) {
            config.nav_path = argv[++i];
        } else if (arg == "--out" && i + 1 < argc) {
            config.output_pos_path = argv[++i];
        } else if (arg == "--kml" && i + 1 < argc) {
            config.output_kml_path = argv[++i];
            config.write_kml = true;
        } else if (arg == "--no-kml") {
            config.write_kml = false;
        } else if (arg == "--format" && i + 1 < argc) {
            config.output_format = parseOutputFormat(argv[++i], argv[0]);
        } else if (arg == "--mode" && i + 1 < argc) {
            config.mode = parseModeChoice(argv[++i], argv[0]);
        } else if (arg == "--iono" && i + 1 < argc) {
            config.iono = parseIonoChoice(argv[++i], argv[0]);
        } else if (arg == "--ratio" && i + 1 < argc) {
            config.ratio_threshold = std::stod(argv[++i]);
        } else if (arg == "--min-ar-sats" && i + 1 < argc) {
            config.min_satellites_for_ar = std::stoi(argv[++i]);
        } else if (arg == "--elevation-mask-deg" && i + 1 < argc) {
            config.elevation_mask_deg = std::stod(argv[++i]);
        } else if (arg == "--max-baseline-m" && i + 1 < argc) {
            config.max_baseline_length_m = std::stod(argv[++i]);
        } else if (arg == "--base-ecef" && i + 3 < argc) {
            config.base_position_ecef =
                Eigen::Vector3d(std::stod(argv[++i]), std::stod(argv[++i]), std::stod(argv[++i]));
            config.base_position_override = true;
        } else if (arg == "--max-epochs" && i + 1 < argc) {
            config.max_epochs = std::stoi(argv[++i]);
        } else if (arg == "--no-base-interp") {
            config.enable_base_interpolation = false;
        } else if (arg == "--verbose") {
            config.verbose = true;
        } else {
            argumentError("unknown or incomplete argument: " + arg, argv[0]);
        }
    }

    if (!config.data_dir.empty()) {
        if (config.rover_obs_path.empty()) config.rover_obs_path = config.data_dir + "/rover.obs";
        if (config.base_obs_path.empty()) config.base_obs_path = config.data_dir + "/base.obs";
        if (config.nav_path.empty()) config.nav_path = config.data_dir + "/navigation.nav";
    }

    if (config.rover_obs_path.empty() || config.base_obs_path.empty() || config.nav_path.empty()) {
        argumentError("provide --data-dir or all of --rover, --base, and --nav", argv[0]);
    }
    if (config.min_satellites_for_ar < 4) {
        argumentError("--min-ar-sats must be >= 4", argv[0]);
    }
    if (config.elevation_mask_deg < 0.0 || config.elevation_mask_deg >= 90.0) {
        argumentError("--elevation-mask-deg must be in [0, 90)", argv[0]);
    }
    if (config.max_baseline_length_m <= 0.0) {
        argumentError("--max-baseline-m must be > 0", argv[0]);
    }

    return config;
}

bool pathLooksStatic(const std::string& text) {
    return text.find("short_baseline") != std::string::npos ||
        text.find("static") != std::string::npos;
}

bool pathLooksShortBaseline(const std::string& text) {
    return text.find("short_baseline") != std::string::npos;
}

libgnss::RTKProcessor::RTKConfig::PositionMode resolvePositionMode(const SolveConfig& config) {
    if (config.mode == ModeChoice::KINEMATIC) {
        return libgnss::RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
    }
    if (config.mode == ModeChoice::STATIC) {
        return libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC;
    }

    const std::string hint = config.data_dir + " " + config.rover_obs_path + " " + config.base_obs_path;
    if (pathLooksStatic(hint)) {
        return libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC;
    }
    return libgnss::RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
}

libgnss::RTKProcessor::RTKConfig::IonoOpt resolveIonoOpt(const SolveConfig& config,
                                                         libgnss::RTKProcessor::RTKConfig::PositionMode mode) {
    if (config.iono == IonoChoice::OFF) {
        return libgnss::RTKProcessor::RTKConfig::IonoOpt::OFF;
    }
    if (config.iono == IonoChoice::IFLC) {
        return libgnss::RTKProcessor::RTKConfig::IonoOpt::IFLC;
    }

    const std::string hint = config.data_dir + " " + config.rover_obs_path + " " + config.base_obs_path;
    const bool short_baseline = pathLooksShortBaseline(hint);
    if (mode == libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC && !short_baseline) {
        return libgnss::RTKProcessor::RTKConfig::IonoOpt::IFLC;
    }
    return libgnss::RTKProcessor::RTKConfig::IonoOpt::OFF;
}

std::string positionModeString(libgnss::RTKProcessor::RTKConfig::PositionMode mode) {
    return mode == libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC ? "static" : "kinematic";
}

std::string ionoOptString(libgnss::RTKProcessor::RTKConfig::IonoOpt iono) {
    return iono == libgnss::RTKProcessor::RTKConfig::IonoOpt::IFLC ? "iflc" : "off";
}

bool writeSolutions(const SolveConfig& config, const libgnss::Solution& solution) {
    libgnss::io::SolutionWriter writer;
    if (!writer.open(config.output_pos_path, config.output_format)) {
        std::cerr << "Error: failed to open output file: " << config.output_pos_path << std::endl;
        return false;
    }

    for (const auto& epoch_solution : solution.solutions) {
        writer.writeEpoch(epoch_solution);
    }
    writer.close();

    if (config.write_kml && !solution.writeKML(config.output_kml_path)) {
        std::cerr << "Error: failed to write KML output: " << config.output_kml_path << std::endl;
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        const SolveConfig config = parseArguments(argc, argv);

        libgnss::RTKProcessor rtk_processor;
        libgnss::SPPProcessor spp_processor;
        libgnss::RTKProcessor::RTKConfig rtk_config;
        rtk_config.max_baseline_length = config.max_baseline_length_m;
        rtk_config.ar_mode = libgnss::RTKProcessor::RTKConfig::AmbiguityResolutionMode::CONTINUOUS;
        rtk_config.ratio_threshold = config.ratio_threshold;
        rtk_config.ambiguity_ratio_threshold = config.ratio_threshold;
        rtk_config.min_satellites_for_ar = config.min_satellites_for_ar;
        rtk_config.elevation_mask = config.elevation_mask_deg * M_PI / 180.0;
        rtk_config.position_mode = resolvePositionMode(config);
        rtk_config.ionoopt = resolveIonoOpt(config, rtk_config.position_mode);
        rtk_processor.setRTKConfig(rtk_config);

        std::cout << "libgnss++ post-process solver" << std::endl;
        std::cout << "  rover: " << config.rover_obs_path << std::endl;
        std::cout << "  base: " << config.base_obs_path << std::endl;
        std::cout << "  nav: " << config.nav_path << std::endl;
        std::cout << "  out: " << config.output_pos_path << " (" << outputFormatString(config.output_format)
                  << ")" << std::endl;
        if (config.write_kml) {
            std::cout << "  kml: " << config.output_kml_path << std::endl;
        }
        std::cout << "  mode: " << positionModeString(rtk_config.position_mode)
                  << " (requested " << modeChoiceString(config.mode) << ")" << std::endl;
        std::cout << "  iono: " << ionoOptString(rtk_config.ionoopt)
                  << " (requested " << ionoChoiceString(config.iono) << ")" << std::endl;
        std::cout << "  base interpolation: "
                  << (config.enable_base_interpolation ? "enabled" : "disabled") << std::endl;

        libgnss::io::RINEXReader rover_reader;
        libgnss::io::RINEXReader base_reader;
        libgnss::io::RINEXReader nav_reader;

        if (!rover_reader.open(config.rover_obs_path)) {
            std::cerr << "Error: cannot open rover observation file" << std::endl;
            return 1;
        }
        libgnss::io::RINEXReader::RINEXHeader rover_header;
        if (!rover_reader.readHeader(rover_header)) {
            std::cerr << "Error: failed to read rover observation header" << std::endl;
            return 1;
        }

        if (!base_reader.open(config.base_obs_path)) {
            std::cerr << "Error: cannot open base observation file" << std::endl;
            return 1;
        }
        libgnss::io::RINEXReader::RINEXHeader base_header;
        if (!base_reader.readHeader(base_header)) {
            std::cerr << "Error: failed to read base observation header" << std::endl;
            return 1;
        }

        if (!nav_reader.open(config.nav_path)) {
            std::cerr << "Error: cannot open navigation file" << std::endl;
            return 1;
        }
        libgnss::NavigationData nav_data;
        if (!nav_reader.readNavigationData(nav_data)) {
            std::cerr << "Error: failed to read navigation data" << std::endl;
            return 1;
        }

        Eigen::Vector3d base_position = Eigen::Vector3d::Zero();
        if (config.base_position_override) {
            base_position = config.base_position_ecef;
        } else if (base_header.approximate_position.norm() > 0.0) {
            base_position = base_header.approximate_position;
        } else {
            std::cerr << "Error: base position unavailable. Use --base-ecef to override." << std::endl;
            return 1;
        }
        rtk_processor.setBasePosition(base_position);

        libgnss::Solution solution;
        libgnss::ObservationData rover_obs;
        libgnss::ObservationData base_obs;
        libgnss::ObservationData previous_base_obs;
        bool has_previous_base = false;
        bool rover_ok = rover_reader.readObservationEpoch(rover_obs);
        bool base_ok = base_reader.readObservationEpoch(base_obs);

        if (!rover_ok) {
            std::cerr << "Error: no rover epochs available" << std::endl;
            return 1;
        }
        if (!base_ok) {
            std::cerr << "Error: no base epochs available" << std::endl;
            return 1;
        }

        if (rover_header.approximate_position.norm() > 0.0) {
            rover_obs.receiver_position = rover_header.approximate_position;
        } else {
            rover_obs.receiver_position = base_position + Eigen::Vector3d(3000.0, 0.0, 0.0);
        }

        int processed_rover_epochs = 0;
        int valid_solution_count = 0;
        int fixed_solution_count = 0;
        int exact_base_epochs = 0;
        int interpolated_base_epochs = 0;
        int skipped_rover_epochs = 0;

        while (rover_ok) {
            if (config.max_epochs > 0 && processed_rover_epochs >= config.max_epochs) {
                break;
            }

            libgnss::ObservationData lower_base_obs = previous_base_obs;
            bool have_lower_base = has_previous_base;

            while (base_ok && timeDiffSeconds(base_obs.time, rover_obs.time) < -kExactTimeToleranceSeconds) {
                lower_base_obs = base_obs;
                have_lower_base = true;
                previous_base_obs = base_obs;
                has_previous_base = true;

                libgnss::ObservationData next_base_obs;
                base_ok = base_reader.readObservationEpoch(next_base_obs);
                if (base_ok) {
                    base_obs = std::move(next_base_obs);
                }
            }

            libgnss::ObservationData aligned_base_obs;
            const double exact_dt = base_ok ? std::abs(timeDiffSeconds(base_obs.time, rover_obs.time))
                                            : std::numeric_limits<double>::infinity();
            bool have_aligned_base = false;
            bool used_interpolated_base = false;

            if (base_ok && exact_dt <= kExactTimeToleranceSeconds) {
                aligned_base_obs = base_obs;
                exact_base_epochs++;
                have_aligned_base = true;
            } else if (config.enable_base_interpolation && base_ok && have_lower_base &&
                       timeDiffSeconds(rover_obs.time, lower_base_obs.time) >= -kExactTimeToleranceSeconds &&
                       timeDiffSeconds(base_obs.time, rover_obs.time) >= -kExactTimeToleranceSeconds &&
                       interpolateBaseEpoch(lower_base_obs, base_obs, rover_obs.time,
                                            base_position, nav_data, aligned_base_obs)) {
                interpolated_base_epochs++;
                have_aligned_base = true;
                used_interpolated_base = true;
            }

            if (!have_aligned_base) {
                skipped_rover_epochs++;
                const Eigen::Vector3d saved_rover_pos = rover_obs.receiver_position;
                rover_ok = rover_reader.readObservationEpoch(rover_obs);
                if (rover_ok) {
                    rover_obs.receiver_position = saved_rover_pos;
                }
                processed_rover_epochs++;
                continue;
            }

            auto pos_solution = rtk_processor.processRTKEpoch(rover_obs, aligned_base_obs, nav_data);
            if (used_interpolated_base && pos_solution.status == libgnss::SolutionStatus::FLOAT) {
                auto spp_solution = spp_processor.processEpoch(rover_obs, nav_data);
                if (spp_solution.isValid()) {
                    const double float_vs_spp = (pos_solution.position_ecef - spp_solution.position_ecef).norm();
                    if (!std::isfinite(float_vs_spp) || float_vs_spp > kDefaultFloatVsSppGuardMeters) {
                        spp_solution.status = libgnss::SolutionStatus::SPP;
                        pos_solution = spp_solution;
                    }
                }
            }

            if (pos_solution.isValid()) {
                solution.addSolution(pos_solution);
                valid_solution_count++;
                if (pos_solution.isFixed()) {
                    fixed_solution_count++;
                }

                if (config.verbose && (valid_solution_count <= 5 || valid_solution_count % 100 == 0)) {
                    std::cout << "epoch " << valid_solution_count
                              << " tow=" << std::fixed << std::setprecision(3) << pos_solution.time.tow
                              << " status=" << static_cast<int>(pos_solution.status)
                              << " sats=" << pos_solution.num_satellites
                              << " ratio=" << std::setprecision(2) << pos_solution.ratio
                              << std::endl;
                }
            }

            const Eigen::Vector3d saved_rover_pos = rover_obs.receiver_position;
            rover_ok = rover_reader.readObservationEpoch(rover_obs);
            if (rover_ok) {
                if (pos_solution.isFixed() || pos_solution.status == libgnss::SolutionStatus::SPP) {
                    rover_obs.receiver_position = pos_solution.position_ecef;
                } else {
                    rover_obs.receiver_position = saved_rover_pos;
                }
            }
            processed_rover_epochs++;
        }

        if (solution.isEmpty()) {
            std::cerr << "Error: no valid solutions were produced" << std::endl;
            return 1;
        }
        if (!writeSolutions(config, solution)) {
            return 1;
        }

        Eigen::Vector3d mean_pos = Eigen::Vector3d::Zero();
        int mean_count = 0;
        for (const auto& epoch_solution : solution.solutions) {
            if (epoch_solution.isValid()) {
                mean_pos += epoch_solution.position_ecef;
                mean_count++;
            }
        }
        if (mean_count > 0) {
            mean_pos /= static_cast<double>(mean_count);
        }

        const auto stats = solution.calculateStatistics(mean_pos);
        std::cout << "\nSummary" << std::endl;
        std::cout << "  total solutions: " << solution.size() << std::endl;
        std::cout << "  valid solutions: " << stats.valid_solutions << std::endl;
        std::cout << "  fixed solutions: " << stats.fixed_solutions << std::endl;
        std::cout << "  fix rate: " << std::fixed << std::setprecision(2)
                  << stats.fix_rate * 100.0 << "%" << std::endl;
        std::cout << "  RMS horizontal (self-consistency): " << stats.rms_horizontal << " m" << std::endl;
        std::cout << "  RMS vertical (self-consistency): " << stats.rms_vertical << " m" << std::endl;
        std::cout << "  exact base epochs: " << exact_base_epochs << std::endl;
        std::cout << "  interpolated base epochs: " << interpolated_base_epochs << std::endl;
        std::cout << "  skipped rover epochs: " << skipped_rover_epochs << std::endl;
        if (rover_header.approximate_position.norm() > 0.0 && mean_count > 0) {
            std::cout << "  header vs mean diff: "
                      << (mean_pos - rover_header.approximate_position).norm() << " m" << std::endl;
        }
        std::cout << "  output written: " << config.output_pos_path << std::endl;
        if (config.write_kml) {
            std::cout << "  KML written: " << config.output_kml_path << std::endl;
        }

        return 0;
    } catch (const std::invalid_argument& e) {
        std::cerr << "Error (invalid_argument): " << e.what() << std::endl;
        return 1;
    } catch (const std::out_of_range& e) {
        std::cerr << "Error (out_of_range): " << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
