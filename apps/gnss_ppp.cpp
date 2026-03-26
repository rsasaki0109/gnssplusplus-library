#include <algorithm>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>

#include <libgnss++/algorithms/ppp.hpp>
#include <libgnss++/core/solution.hpp>
#include <libgnss++/io/rinex.hpp>

namespace {

struct Options {
    std::string obs_path;
    std::string nav_path;
    std::string sp3_path;
    std::string clk_path;
    std::string ssr_path;
    std::string ssr_rtcm_path;
    std::string antex_path;
    std::string blq_path;
    std::string ocean_loading_station_name;
    std::string out_path;
    std::string kml_path;
    int max_epochs = 0;
    int convergence_min_epochs = 20;
    double ssr_step_seconds = 1.0;
    bool estimate_troposphere = true;
    bool kinematic_mode = false;
    bool low_dynamics_mode = false;
    bool enable_ar = false;
    double ar_ratio_threshold = 3.0;
    bool quiet = false;
};

void printUsage(const char* program_name) {
    std::cout
        << "Usage: " << program_name << " --obs <rover.obs> [options]\n"
        << "Options:\n"
        << "  --nav <nav.rnx>           Optional broadcast navigation file\n"
        << "  --sp3 <orbit.sp3>        Precise orbit file\n"
        << "  --clk <clock.clk>        Precise clock file\n"
        << "  --ssr <corrections.csv>  Simple SSR orbit/clock corrections CSV\n"
        << "  --ssr-rtcm <file|ntrip://...|serial://...|tcp://...>\n"
        << "                          RTCM SSR source converted/read for PPP use\n"
        << "  --antex <antennas.atx>   Optional ANTEX file for receiver antenna PCO\n"
        << "  --blq <station.blq>      Optional BLQ ocean loading coefficient file\n"
        << "  --ocean-loading-station <name>\n"
        << "                          Station name to select from the BLQ file\n"
        << "  --ssr-step-seconds <seconds>\n"
        << "                          Sampling step for RTCM SSR conversion (default: 1.0)\n"
        << "  --out <solution.pos>     Output position file (required)\n"
        << "  --kml <solution.kml>     Optional KML output\n"
        << "  --max-epochs <count>     Limit processed epochs (default: all)\n"
        << "  --convergence-min-epochs <count>\n"
        << "                          Minimum epochs before PPP convergence/AR checks (default: 20)\n"
        << "  --no-estimate-troposphere\n"
        << "                          Disable zenith troposphere estimation\n"
        << "  --estimate-troposphere  Enable zenith troposphere estimation (default)\n"
        << "  --static                Use a static PPP motion model (default)\n"
        << "  --kinematic             Use a kinematic PPP motion model\n"
        << "  --low-dynamics          Keep kinematic PPP anchored for quasi-static motion\n"
        << "  --no-low-dynamics       Disable quasi-static anchoring (default)\n"
        << "  --enable-ar             Enable PPP ambiguity fixing when supported\n"
        << "  --disable-ar            Disable PPP ambiguity fixing (default)\n"
        << "  --ar-ratio-threshold <value>\n"
        << "                          Ratio threshold for PPP ambiguity fixing (default: 3.0)\n"
        << "  --quiet                  Suppress per-run summary output\n"
        << "  -h, --help               Show this help\n";
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
        } else if (arg == "--sp3" && i + 1 < argc) {
            options.sp3_path = argv[++i];
        } else if (arg == "--clk" && i + 1 < argc) {
            options.clk_path = argv[++i];
        } else if (arg == "--ssr" && i + 1 < argc) {
            options.ssr_path = argv[++i];
        } else if (arg == "--ssr-rtcm" && i + 1 < argc) {
            options.ssr_rtcm_path = argv[++i];
        } else if (arg == "--antex" && i + 1 < argc) {
            options.antex_path = argv[++i];
        } else if (arg == "--blq" && i + 1 < argc) {
            options.blq_path = argv[++i];
        } else if (arg == "--ocean-loading-station" && i + 1 < argc) {
            options.ocean_loading_station_name = argv[++i];
        } else if (arg == "--out" && i + 1 < argc) {
            options.out_path = argv[++i];
        } else if (arg == "--kml" && i + 1 < argc) {
            options.kml_path = argv[++i];
        } else if (arg == "--max-epochs" && i + 1 < argc) {
            options.max_epochs = std::stoi(argv[++i]);
        } else if (arg == "--convergence-min-epochs" && i + 1 < argc) {
            options.convergence_min_epochs = std::stoi(argv[++i]);
        } else if (arg == "--ssr-step-seconds" && i + 1 < argc) {
            options.ssr_step_seconds = std::stod(argv[++i]);
        } else if (arg == "--no-estimate-troposphere") {
            options.estimate_troposphere = false;
        } else if (arg == "--estimate-troposphere") {
            options.estimate_troposphere = true;
        } else if (arg == "--static") {
            options.kinematic_mode = false;
        } else if (arg == "--kinematic") {
            options.kinematic_mode = true;
        } else if (arg == "--low-dynamics") {
            options.low_dynamics_mode = true;
        } else if (arg == "--no-low-dynamics") {
            options.low_dynamics_mode = false;
        } else if (arg == "--enable-ar") {
            options.enable_ar = true;
        } else if (arg == "--disable-ar") {
            options.enable_ar = false;
        } else if (arg == "--ar-ratio-threshold" && i + 1 < argc) {
            options.ar_ratio_threshold = std::stod(argv[++i]);
        } else if (arg == "--quiet") {
            options.quiet = true;
        } else {
            argumentError("unknown or incomplete argument: " + arg, argv[0]);
        }
    }

    if (options.obs_path.empty()) {
        argumentError("--obs is required", argv[0]);
    }
    if (options.out_path.empty()) {
        argumentError("--out is required", argv[0]);
    }
    if (options.nav_path.empty() && options.sp3_path.empty()) {
        argumentError("provide at least one of --nav or --sp3", argv[0]);
    }
    if (!options.ssr_rtcm_path.empty() && options.nav_path.empty()) {
        argumentError("--ssr-rtcm requires --nav", argv[0]);
    }
    if (options.max_epochs < 0) {
        argumentError("--max-epochs must be non-negative", argv[0]);
    }
    if (options.convergence_min_epochs <= 0) {
        argumentError("--convergence-min-epochs must be positive", argv[0]);
    }
    if (options.ar_ratio_threshold <= 0.0) {
        argumentError("--ar-ratio-threshold must be positive", argv[0]);
    }
    if (options.ssr_step_seconds <= 0.0) {
        argumentError("--ssr-step-seconds must be positive", argv[0]);
    }
    return options;
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
        if (!options.blq_path.empty() &&
            options.ocean_loading_station_name.empty() &&
            obs_header.marker_name.find_first_not_of(' ') == std::string::npos) {
            std::cerr << "Error: --blq requires a marker name in the observation header or "
                         "--ocean-loading-station\n";
            return 1;
        }

        libgnss::NavigationData nav_data;
        if (!options.nav_path.empty()) {
            libgnss::io::RINEXReader nav_reader;
            if (!nav_reader.open(options.nav_path)) {
                std::cerr << "Error: failed to open navigation file: " << options.nav_path << "\n";
                return 1;
            }
            if (!nav_reader.readNavigationData(nav_data)) {
                std::cerr << "Error: failed to read navigation data: " << options.nav_path << "\n";
                return 1;
            }
        }

        libgnss::PPPProcessor::PPPConfig ppp_config;
        ppp_config.orbit_file_path = options.sp3_path;
        ppp_config.clock_file_path = options.clk_path;
        ppp_config.use_precise_orbits = !options.sp3_path.empty();
        ppp_config.use_precise_clocks = !options.clk_path.empty();
        ppp_config.ssr_file_path = options.ssr_path;
        ppp_config.use_ssr_corrections =
            !options.ssr_path.empty() || !options.ssr_rtcm_path.empty();
        ppp_config.antex_file_path = options.antex_path;
        ppp_config.ocean_loading_file_path = options.blq_path;
        ppp_config.estimate_troposphere = options.estimate_troposphere;
        ppp_config.kinematic_mode = options.kinematic_mode;
        ppp_config.low_dynamics_mode = options.low_dynamics_mode;
        ppp_config.enable_ambiguity_resolution = options.enable_ar;
        ppp_config.convergence_min_epochs = options.convergence_min_epochs;
        ppp_config.ar_ratio_threshold = options.ar_ratio_threshold;
        if (options.low_dynamics_mode) {
            ppp_config.reset_clock_to_spp_each_epoch = false;
            ppp_config.reset_kinematic_position_to_spp_each_epoch = false;
            ppp_config.use_dynamics_model = false;
            ppp_config.process_noise_position = 0.0;
            ppp_config.process_noise_velocity = 1e-8;
        }
        ppp_config.receiver_antenna_type = obs_header.antenna_type;
        ppp_config.receiver_antenna_delta_enu = obs_header.antenna_delta;
        ppp_config.ocean_loading_station_name =
            options.ocean_loading_station_name.empty() ?
                obs_header.marker_name :
                options.ocean_loading_station_name;
        ppp_config.apply_ocean_loading = !options.blq_path.empty();

        libgnss::ProcessorConfig processor_config;
        processor_config.mode = libgnss::PositioningMode::PPP;
        processor_config.use_precise_orbits = ppp_config.use_precise_orbits;
        processor_config.use_precise_clocks = ppp_config.use_precise_clocks;
        processor_config.orbit_file_path = options.sp3_path;
        processor_config.clock_file_path = options.clk_path;

        libgnss::PPPProcessor processor(ppp_config);
        if (!processor.initialize(processor_config)) {
            std::cerr << "Error: failed to initialize PPP processor\n";
            return 1;
        }
        if (!options.ssr_rtcm_path.empty() &&
            !processor.loadRTCMSSRProducts(
                options.ssr_rtcm_path, nav_data, options.ssr_step_seconds)) {
            std::cerr << "Error: failed to load RTCM SSR corrections: "
                      << options.ssr_rtcm_path << "\n";
            return 1;
        }

        libgnss::Solution solutions;
        libgnss::ObservationData observation_data;
        int processed_epochs = 0;
        int valid_solutions = 0;
        int ppp_float_solutions = 0;
        int ppp_fixed_solutions = 0;
        int fallback_solutions = 0;
        int atmospheric_trop_corrections = 0;
        int atmospheric_iono_corrections = 0;
        double atmospheric_trop_meters = 0.0;
        double atmospheric_iono_meters = 0.0;
        while ((options.max_epochs == 0 || processed_epochs < options.max_epochs) &&
               obs_reader.readObservationEpoch(observation_data)) {
            if (obs_header.approximate_position.norm() > 0.0) {
                observation_data.receiver_position = obs_header.approximate_position;
            }

            const auto solution = processor.processEpoch(observation_data, nav_data);
            atmospheric_trop_corrections +=
                processor.getLastAppliedAtmosphericTroposphereCorrections();
            atmospheric_iono_corrections +=
                processor.getLastAppliedAtmosphericIonosphereCorrections();
            atmospheric_trop_meters +=
                processor.getLastAppliedAtmosphericTroposphereMeters();
            atmospheric_iono_meters +=
                processor.getLastAppliedAtmosphericIonosphereMeters();
            processed_epochs++;
            if (solution.isValid()) {
                solutions.addSolution(solution);
                valid_solutions++;
                if (solution.status == libgnss::SolutionStatus::PPP_FLOAT) {
                    ppp_float_solutions++;
                } else if (solution.status == libgnss::SolutionStatus::PPP_FIXED) {
                    ppp_fixed_solutions++;
                } else {
                    fallback_solutions++;
                }
            }
        }

        if (solutions.isEmpty()) {
            std::cerr << "Error: PPP processing produced no valid solutions\n";
            return 1;
        }

        if (!solutions.writeToFile(options.out_path)) {
            std::cerr << "Error: failed to write solution file: " << options.out_path << "\n";
            return 1;
        }
        if (!options.kml_path.empty() && !solutions.writeKML(options.kml_path)) {
            std::cerr << "Error: failed to write KML file: " << options.kml_path << "\n";
            return 1;
        }

        if (!options.quiet) {
            const auto stats = processor.getStats();
            std::cout << "PPP summary:\n";
            std::cout << "  processed epochs: " << processed_epochs << "\n";
            std::cout << "  valid solutions: " << valid_solutions << "\n";
            std::cout << "  PPP float solutions: " << ppp_float_solutions << "\n";
            std::cout << "  PPP fixed solutions: " << ppp_fixed_solutions << "\n";
            std::cout << "  fallback solutions: " << fallback_solutions << "\n";
            std::cout << "  mode: " << (options.kinematic_mode ? "kinematic" : "static") << "\n";
            std::cout << "  low dynamics: " << (options.low_dynamics_mode ? "on" : "off") << "\n";
            std::cout << "  ambiguity resolution: " << (options.enable_ar ? "on" : "off") << "\n";
            std::cout << "  SSR corrections: "
                      << ((options.ssr_path.empty() && options.ssr_rtcm_path.empty()) ? "off" : "on")
                      << "\n";
            if (atmospheric_trop_corrections > 0 || atmospheric_iono_corrections > 0) {
                std::cout << "  atmospheric trop corrections: "
                          << atmospheric_trop_corrections << "\n";
                std::cout << "  atmospheric trop meters: "
                          << atmospheric_trop_meters << "\n";
                std::cout << "  atmospheric ionosphere corrections: "
                          << atmospheric_iono_corrections << "\n";
                std::cout << "  atmospheric ionosphere meters: "
                          << atmospheric_iono_meters << "\n";
            }
            if (options.enable_ar) {
                std::cout << "  AR ratio threshold: " << options.ar_ratio_threshold << "\n";
            }
            if (valid_solutions > 0) {
                const double ppp_solution_rate =
                    100.0 * static_cast<double>(ppp_float_solutions + ppp_fixed_solutions) /
                    static_cast<double>(valid_solutions);
                std::cout << "  PPP solution rate (%): " << ppp_solution_rate << "\n";
            }
            std::cout << "  converged: " << (processor.hasConverged() ? "yes" : "no") << "\n";
            if (processor.hasConverged()) {
                std::cout << "  convergence time (s): " << processor.getConvergenceTime() << "\n";
            }
            std::cout << "  average processing time (ms): " << stats.average_processing_time_ms << "\n";
            std::cout << "  output: " << options.out_path << "\n";
        }

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
