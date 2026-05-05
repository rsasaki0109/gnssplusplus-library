#include <cstdlib>
#include <iostream>
#include <string>

#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/core/solution.hpp>
#include <libgnss++/io/rinex.hpp>

namespace {

struct Options {
    std::string obs_path;
    std::string nav_path;
    std::string out_path;
    int max_epochs = 0;
    double elevation_mask_deg = 15.0;
    bool ionosphere_free_code = false;
    bool quiet = false;
    int navsys_mask = 0;  ///< RTKLIB-style mask (0=all, 1=GPS,4=GLO,8=GAL,16=QZS,32=BDS)
};

void printUsage(const char* program_name) {
    std::cout
        << "Usage: " << program_name << " --obs <rover.obs> --nav <nav.rnx> --out <solution.pos>\n"
        << "Options:\n"
        << "  --obs <rover.obs>   RINEX observation file\n"
        << "  --nav <nav.rnx>     RINEX navigation file\n"
        << "  --out <solution.pos> Output position file\n"
        << "  --max-epochs <n>    Stop after n epochs\n"
        << "  --elevation-mask <deg> Elevation mask in degrees (default: 15)\n"
        << "  --ionosphere-free-code Use dual-frequency ionosphere-free code when available\n"
        << "  --navsys <mask>     RTKLIB navsys mask (0=all, 1=GPS, 4=GLO, 8=GAL, 16=QZS, 32=BDS; OR-combine)\n"
        << "  --quiet             Suppress run summary\n"
        << "  -h, --help          Show this help\n";
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
        } else if (arg == "--out" && i + 1 < argc) {
            options.out_path = argv[++i];
        } else if (arg == "--max-epochs" && i + 1 < argc) {
            options.max_epochs = std::stoi(argv[++i]);
        } else if (arg == "--elevation-mask" && i + 1 < argc) {
            options.elevation_mask_deg = std::stod(argv[++i]);
        } else if (arg == "--ionosphere-free-code") {
            options.ionosphere_free_code = true;
        } else if (arg == "--navsys" && i + 1 < argc) {
            options.navsys_mask = std::stoi(argv[++i]);
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
    if (options.out_path.empty()) {
        argumentError("--out is required", argv[0]);
    }
    if (options.max_epochs < 0) {
        argumentError("--max-epochs must be non-negative", argv[0]);
    }
    if (options.elevation_mask_deg < 0.0 || options.elevation_mask_deg >= 90.0) {
        argumentError("--elevation-mask must be in [0, 90)", argv[0]);
    }
    if (options.navsys_mask < 0 || options.navsys_mask > 127) {
        argumentError("--navsys must be a RTKLIB navsys mask from 0 to 127", argv[0]);
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

        libgnss::ProcessorConfig config;
        config.mode = libgnss::PositioningMode::SPP;
        config.snr_mask = 0.0;
        config.elevation_mask = options.elevation_mask_deg;

        libgnss::SPPProcessor processor;
        libgnss::SPPProcessor::SPPConfig spp_config;
        spp_config.use_ionosphere_free_code = options.ionosphere_free_code;
        if (options.navsys_mask != 0) {
            spp_config.enable_gps     = (options.navsys_mask & 1)  != 0;
            spp_config.enable_glonass = (options.navsys_mask & 4)  != 0;
            spp_config.enable_galileo = (options.navsys_mask & 8)  != 0;
            spp_config.enable_qzss    = (options.navsys_mask & 16) != 0;
            spp_config.enable_beidou  = (options.navsys_mask & 32) != 0;
        }
        processor.setSPPConfig(spp_config);
        if (!processor.initialize(config)) {
            std::cerr << "Error: failed to initialize SPP processor\n";
            return 1;
        }

        libgnss::Solution solution;
        libgnss::ObservationData obs;
        int processed_epochs = 0;
        while (obs_reader.readObservationEpoch(obs)) {
            if (options.max_epochs > 0 && processed_epochs >= options.max_epochs) {
                break;
            }
            if (obs_header.approximate_position.norm() > 0.0) {
                obs.receiver_position = obs_header.approximate_position;
            }
            const auto epoch_solution = processor.processEpoch(obs, nav_data);
            if (epoch_solution.isValid()) {
                solution.addSolution(epoch_solution);
            }
            ++processed_epochs;
        }

        if (!solution.writeToFile(options.out_path)) {
            std::cerr << "Error: failed to write solution file: " << options.out_path << "\n";
            return 1;
        }

        if (!options.quiet) {
            const auto stats = solution.calculateStatistics();
            std::cout << "Processed epochs: " << processed_epochs << "\n";
            std::cout << "Valid solutions: " << stats.valid_solutions << "\n";
            std::cout << "Output: " << options.out_path << "\n";
        }

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
