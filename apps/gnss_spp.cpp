#include <cstdlib>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/core/solution.hpp>
#include <libgnss++/io/rinex.hpp>

namespace {

struct Options {
    std::string obs_path;
    std::string nav_path;
    std::string out_path;
    std::string summary_json_path;
    std::string sp3_path;
    std::string clk_path;
    std::string ssr_path;
    std::string ionex_path;
    std::string dcb_path;
    int max_epochs = 0;
    double elevation_mask_deg = 15.0;
    double snr_mask = 0.0;
    libgnss::SPPProcessor::SPPConfig spp_config;
    bool quiet = false;
};

struct RunSummary {
    int processed_epochs = 0;
    int valid_solutions = 0;
    int spp_pre_qc_measurements = 0;
    int spp_outlier_rejections = 0;
    int spp_raim_fde_attempts = 0;
    int spp_raim_fde_rejections = 0;
    int spp_gdop_gate_rejections = 0;
    int spp_residual_gate_rejections = 0;
    int spp_chi_square_gate_rejections = 0;
    int spp_position_jump_gate_rejections = 0;
    int spp_robust_weighted_measurements = 0;
    int spp_adaptive_robust_activations = 0;
    int spp_adaptive_robust_tail_measurements = 0;
    int spp_ionosphere_free_measurements = 0;
    int spp_precise_orbit_clock_measurements = 0;
    int spp_ssr_orbit_clock_corrections = 0;
    int spp_ssr_code_bias_corrections = 0;
    int spp_ionex_corrections = 0;
    int spp_dcb_corrections = 0;
    double spp_ssr_orbit_meters = 0.0;
    double spp_ssr_clock_meters = 0.0;
    double spp_ssr_code_bias_meters = 0.0;
    double spp_ionex_meters = 0.0;
    double spp_dcb_meters = 0.0;
    double spp_min_robust_weight_factor = 1.0;
    bool precise_loaded = false;
    bool ssr_loaded = false;
    bool ionex_loaded = false;
    bool dcb_loaded = false;
    size_t precise_satellites = 0;
    size_t ssr_satellites = 0;
    size_t ionex_maps = 0;
    size_t dcb_entries = 0;
    double sum_satellites = 0.0;
    double sum_pdop = 0.0;
    double sum_gdop = 0.0;
    double sum_residual_rms_m = 0.0;
    double max_residual_rms_m = 0.0;
    std::map<std::string, int> rejected_satellites;

    void addSolution(const libgnss::PositionSolution& solution) {
        spp_pre_qc_measurements += solution.spp_pre_qc_measurements;
        spp_outlier_rejections += solution.spp_outlier_rejections;
        spp_raim_fde_attempts += solution.spp_raim_fde_attempts;
        spp_raim_fde_rejections += solution.spp_raim_fde_rejections;
        spp_gdop_gate_rejections += solution.spp_gdop_gate_rejections;
        spp_residual_gate_rejections += solution.spp_residual_gate_rejections;
        spp_chi_square_gate_rejections += solution.spp_chi_square_gate_rejections;
        spp_position_jump_gate_rejections += solution.spp_position_jump_gate_rejections;
        spp_robust_weighted_measurements += solution.spp_robust_weighted_measurements;
        spp_adaptive_robust_activations += solution.spp_adaptive_robust_activations;
        spp_adaptive_robust_tail_measurements +=
            solution.spp_adaptive_robust_tail_measurements;
        spp_ionosphere_free_measurements += solution.spp_ionosphere_free_measurements;
        spp_precise_orbit_clock_measurements +=
            solution.spp_precise_orbit_clock_measurements;
        spp_ssr_orbit_clock_corrections += solution.spp_ssr_orbit_clock_corrections;
        spp_ssr_code_bias_corrections += solution.spp_ssr_code_bias_corrections;
        spp_ionex_corrections += solution.spp_ionex_corrections;
        spp_dcb_corrections += solution.spp_dcb_corrections;
        spp_ssr_orbit_meters += solution.spp_ssr_orbit_meters;
        spp_ssr_clock_meters += solution.spp_ssr_clock_meters;
        spp_ssr_code_bias_meters += solution.spp_ssr_code_bias_meters;
        spp_ionex_meters += solution.spp_ionex_meters;
        spp_dcb_meters += solution.spp_dcb_meters;
        if (solution.spp_robust_weighted_measurements > 0) {
            spp_min_robust_weight_factor =
                std::min(spp_min_robust_weight_factor,
                         solution.spp_min_robust_weight_factor);
        }

        for (const auto& satellite : solution.spp_rejected_satellites) {
            rejected_satellites[satellite.toString()]++;
        }

        if (!solution.isValid()) {
            return;
        }
        ++valid_solutions;
        sum_satellites += solution.num_satellites;
        sum_pdop += solution.pdop;
        sum_gdop += solution.gdop;
        sum_residual_rms_m += solution.residual_rms;
        if (solution.residual_rms > max_residual_rms_m) {
            max_residual_rms_m = solution.residual_rms;
        }
    }

    double availabilityRate() const {
        return processed_epochs > 0
                   ? static_cast<double>(valid_solutions) /
                         static_cast<double>(processed_epochs)
                   : 0.0;
    }

    double meanSatellites() const {
        return valid_solutions > 0 ? sum_satellites / valid_solutions : 0.0;
    }

    double meanPdop() const {
        return valid_solutions > 0 ? sum_pdop / valid_solutions : 0.0;
    }

    double meanGdop() const {
        return valid_solutions > 0 ? sum_gdop / valid_solutions : 0.0;
    }

    double meanResidualRms() const {
        return valid_solutions > 0 ? sum_residual_rms_m / valid_solutions : 0.0;
    }
};

std::string requireValue(const std::string& arg, int& i, int argc, char* argv[]) {
    if (i + 1 >= argc) {
        throw std::invalid_argument("missing value for " + arg);
    }
    return argv[++i];
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

void printUsage(const char* program_name) {
    std::cout
        << "Usage: " << program_name << " --obs <rover.obs> --nav <nav.rnx> --out <solution.pos>\n"
        << "Options:\n"
        << "  --obs <rover.obs>                 RINEX observation file\n"
        << "  --nav <nav.rnx>                   RINEX navigation file\n"
        << "  --out <solution.pos>              Output position file\n"
        << "  --summary-json <summary.json>     Write machine-readable run summary\n"
        << "  --max-epochs <n>                  Stop after n epochs\n"
        << "  --elevation-mask-deg <deg>        Elevation mask in degrees (default: 15)\n"
        << "  --snr-mask <dbhz>                 Minimum SNR/CN0 threshold (default: 0)\n"
        << "  --disable-outlier-detection       Disable robust residual outlier rejection\n"
        << "  --outlier-threshold-sigma <n>     Residual outlier threshold (default: 3)\n"
        << "  --disable-raim-fde                Disable leave-one-out RAIM/FDE\n"
        << "  --raim-fde-min-improvement-ratio <r>\n"
        << "                                    Minimum fractional RMS improvement (default: 0.25)\n"
        << "  --raim-fde-min-improvement-m <m>  Minimum absolute RMS improvement (default: 1)\n"
        << "  --max-gdop <v>                    GDOP rejection threshold, <=0 disables (default: 50)\n"
        << "  --max-residual-rms <m>            Residual RMS rejection threshold, <=0 disables\n"
        << "  --max-chi-square-per-dof <v>      Reduced chi-square rejection threshold, <=0 disables\n"
        << "  --disable-variance-model          Use legacy elevation-only relative weighting\n"
        << "  --snr-reference-dbhz <dbhz>       CN0 where no SNR penalty is applied (default: 45)\n"
        << "  --robust-weighting                Downweight large residuals inside WLS iterations\n"
        << "  --robust-threshold-sigma <n>      Huber robust threshold (default: 3)\n"
        << "  --robust-min-weight <f>           Minimum robust weight factor in [0,1] (default: 0.05)\n"
        << "  --adaptive-robust-weighting       Enable Huber weights only for residual-tail epochs\n"
        << "  --adaptive-robust-activation-threshold-sigma <n>\n"
        << "                                    Normalized residual threshold to activate adaptive robust weighting (default: 3)\n"
        << "  --adaptive-robust-min-tail-measurements <n>\n"
        << "                                    Minimum tail residual count to activate adaptive robust weighting (default: 2)\n"
        << "  --adaptive-robust-min-tail-fraction <f>\n"
        << "                                    Minimum tail residual fraction to activate adaptive robust weighting (default: 0.08)\n"
        << "  --max-position-jump-rate-mps <v>  Reject SPP jumps above this rate, <=0 disables\n"
        << "  --max-position-jump-min-m <m>     Minimum allowed SPP jump before rate gate applies\n"
        << "  --ionosphere-free                 Use dual-frequency code IFLC when available\n"
        << "  --sp3 <orbits.sp3>                Optional SP3 precise orbit product\n"
        << "  --clk <clocks.clk>                Optional RINEX CLK precise clock product\n"
        << "  --ssr <corrections.csv>           Optional CSV SSR orbit/clock/code-bias product\n"
        << "  --ionex <maps.ionex>              Optional IONEX TEC map product\n"
        << "  --dcb <bias.bsx>                  Optional DCB / Bias-SINEX product\n"
        << "  --disable-precise-products        Load SP3/CLK only for inspection, do not apply\n"
        << "  --disable-ssr-corrections         Load SSR only for inspection, do not apply\n"
        << "  --disable-ionex-corrections       Load IONEX only for inspection, do not apply\n"
        << "  --disable-dcb-corrections         Load DCB/OSB only for inspection, do not apply\n"
        << "  --quiet                           Suppress run summary\n"
        << "  -h, --help                        Show this help\n";
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
        } else if (arg == "--obs") {
            options.obs_path = requireValue(arg, i, argc, argv);
        } else if (arg == "--nav") {
            options.nav_path = requireValue(arg, i, argc, argv);
        } else if (arg == "--out") {
            options.out_path = requireValue(arg, i, argc, argv);
        } else if (arg == "--summary-json") {
            options.summary_json_path = requireValue(arg, i, argc, argv);
        } else if (arg == "--max-epochs") {
            options.max_epochs = std::stoi(requireValue(arg, i, argc, argv));
        } else if (arg == "--elevation-mask-deg" || arg == "--elevation-mask") {
            options.elevation_mask_deg = std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--snr-mask") {
            options.snr_mask = std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--disable-outlier-detection") {
            options.spp_config.enable_outlier_detection = false;
        } else if (arg == "--outlier-threshold-sigma") {
            options.spp_config.outlier_threshold_sigma =
                std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--disable-raim-fde") {
            options.spp_config.enable_raim_fde = false;
        } else if (arg == "--raim-fde-min-improvement-ratio") {
            options.spp_config.raim_fde_min_rms_improvement_ratio =
                std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--raim-fde-min-improvement-m") {
            options.spp_config.raim_fde_min_rms_improvement_m =
                std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--max-gdop") {
            options.spp_config.max_gdop = std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--max-residual-rms") {
            options.spp_config.max_residual_rms =
                std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--max-chi-square-per-dof") {
            options.spp_config.max_chi_square_per_dof =
                std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--disable-variance-model") {
            options.spp_config.use_variance_model = false;
        } else if (arg == "--snr-reference-dbhz") {
            options.spp_config.snr_reference_dbhz =
                std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--robust-weighting") {
            options.spp_config.enable_robust_weighting = true;
        } else if (arg == "--robust-threshold-sigma") {
            options.spp_config.robust_weight_threshold_sigma =
                std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--robust-min-weight") {
            options.spp_config.robust_weight_min_factor =
                std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--adaptive-robust-weighting") {
            options.spp_config.enable_robust_weighting = true;
            options.spp_config.enable_adaptive_robust_weighting = true;
        } else if (arg == "--adaptive-robust-activation-threshold-sigma") {
            options.spp_config.adaptive_robust_activation_threshold_sigma =
                std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--adaptive-robust-min-tail-measurements") {
            options.spp_config.adaptive_robust_min_tail_measurements =
                std::stoi(requireValue(arg, i, argc, argv));
        } else if (arg == "--adaptive-robust-min-tail-fraction") {
            options.spp_config.adaptive_robust_min_tail_fraction =
                std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--max-position-jump-rate-mps") {
            options.spp_config.max_position_jump_rate_mps =
                std::stod(requireValue(arg, i, argc, argv));
            options.spp_config.enable_position_jump_gate =
                options.spp_config.max_position_jump_rate_mps > 0.0;
        } else if (arg == "--max-position-jump-min-m") {
            options.spp_config.max_position_jump_min_m =
                std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--ionosphere-free" || arg == "--iflc") {
            options.spp_config.use_ionosphere_free_combination = true;
        } else if (arg == "--sp3") {
            options.sp3_path = requireValue(arg, i, argc, argv);
        } else if (arg == "--clk") {
            options.clk_path = requireValue(arg, i, argc, argv);
        } else if (arg == "--ssr") {
            options.ssr_path = requireValue(arg, i, argc, argv);
        } else if (arg == "--ionex") {
            options.ionex_path = requireValue(arg, i, argc, argv);
        } else if (arg == "--dcb") {
            options.dcb_path = requireValue(arg, i, argc, argv);
        } else if (arg == "--disable-precise-products") {
            options.spp_config.use_precise_products = false;
        } else if (arg == "--disable-ssr-corrections") {
            options.spp_config.use_ssr_corrections = false;
        } else if (arg == "--disable-ionex-corrections") {
            options.spp_config.use_ionex_corrections = false;
        } else if (arg == "--disable-dcb-corrections") {
            options.spp_config.use_dcb_corrections = false;
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
        argumentError("--elevation-mask-deg must be in [0, 90)", argv[0]);
    }
    if (options.snr_mask < 0.0) {
        argumentError("--snr-mask must be non-negative", argv[0]);
    }
    if (options.spp_config.outlier_threshold_sigma <= 0.0) {
        argumentError("--outlier-threshold-sigma must be > 0", argv[0]);
    }
    if (options.spp_config.raim_fde_min_rms_improvement_ratio < 0.0) {
        argumentError("--raim-fde-min-improvement-ratio must be >= 0", argv[0]);
    }
    if (options.spp_config.raim_fde_min_rms_improvement_m < 0.0) {
        argumentError("--raim-fde-min-improvement-m must be >= 0", argv[0]);
    }
    if (options.spp_config.snr_reference_dbhz <= 0.0) {
        argumentError("--snr-reference-dbhz must be > 0", argv[0]);
    }
    if (options.spp_config.robust_weight_threshold_sigma <= 0.0) {
        argumentError("--robust-threshold-sigma must be > 0", argv[0]);
    }
    if (options.spp_config.robust_weight_min_factor < 0.0 ||
        options.spp_config.robust_weight_min_factor > 1.0) {
        argumentError("--robust-min-weight must be in [0, 1]", argv[0]);
    }
    if (options.spp_config.adaptive_robust_activation_threshold_sigma <= 0.0) {
        argumentError("--adaptive-robust-activation-threshold-sigma must be > 0", argv[0]);
    }
    if (options.spp_config.adaptive_robust_min_tail_measurements < 1) {
        argumentError("--adaptive-robust-min-tail-measurements must be >= 1", argv[0]);
    }
    if (options.spp_config.adaptive_robust_min_tail_fraction < 0.0 ||
        options.spp_config.adaptive_robust_min_tail_fraction > 1.0) {
        argumentError("--adaptive-robust-min-tail-fraction must be in [0, 1]", argv[0]);
    }
    if (options.spp_config.max_position_jump_rate_mps < 0.0) {
        argumentError("--max-position-jump-rate-mps must be >= 0", argv[0]);
    }
    if (options.spp_config.max_position_jump_min_m < 0.0) {
        argumentError("--max-position-jump-min-m must be >= 0", argv[0]);
    }
    return options;
}

bool writeSummaryJson(const std::string& path,
                      const Options& options,
                      const RunSummary& summary) {
    std::ofstream output(path);
    if (!output.is_open()) {
        return false;
    }

    output << std::fixed << std::setprecision(6);
    output << "{\n";
    output << "  \"obs\": \"" << jsonEscape(options.obs_path) << "\",\n";
    output << "  \"nav\": \"" << jsonEscape(options.nav_path) << "\",\n";
    output << "  \"out\": \"" << jsonEscape(options.out_path) << "\",\n";
    output << "  \"sp3\": "
           << (options.sp3_path.empty()
                   ? "null"
                   : ("\"" + jsonEscape(options.sp3_path) + "\""))
           << ",\n";
    output << "  \"clk\": "
           << (options.clk_path.empty()
                   ? "null"
                   : ("\"" + jsonEscape(options.clk_path) + "\""))
           << ",\n";
    output << "  \"ssr\": "
           << (options.ssr_path.empty()
                   ? "null"
                   : ("\"" + jsonEscape(options.ssr_path) + "\""))
           << ",\n";
    output << "  \"ionex\": "
           << (options.ionex_path.empty()
                   ? "null"
                   : ("\"" + jsonEscape(options.ionex_path) + "\""))
           << ",\n";
    output << "  \"dcb\": "
           << (options.dcb_path.empty()
                   ? "null"
                   : ("\"" + jsonEscape(options.dcb_path) + "\""))
           << ",\n";
    output << "  \"processed_epochs\": " << summary.processed_epochs << ",\n";
    output << "  \"valid_solutions\": " << summary.valid_solutions << ",\n";
    output << "  \"availability_rate\": " << summary.availabilityRate() << ",\n";
    output << "  \"mean_satellites\": " << summary.meanSatellites() << ",\n";
    output << "  \"mean_pdop\": " << summary.meanPdop() << ",\n";
    output << "  \"mean_gdop\": " << summary.meanGdop() << ",\n";
    output << "  \"mean_residual_rms_m\": " << summary.meanResidualRms() << ",\n";
    output << "  \"max_residual_rms_m\": " << summary.max_residual_rms_m << ",\n";
    output << "  \"precise_loaded\": " << (summary.precise_loaded ? "true" : "false") << ",\n";
    output << "  \"precise_satellites\": " << summary.precise_satellites << ",\n";
    output << "  \"ssr_loaded\": " << (summary.ssr_loaded ? "true" : "false") << ",\n";
    output << "  \"ssr_satellites\": " << summary.ssr_satellites << ",\n";
    output << "  \"ionex_loaded\": " << (summary.ionex_loaded ? "true" : "false") << ",\n";
    output << "  \"ionex_maps\": " << summary.ionex_maps << ",\n";
    output << "  \"dcb_loaded\": " << (summary.dcb_loaded ? "true" : "false") << ",\n";
    output << "  \"dcb_entries\": " << summary.dcb_entries << ",\n";
    output << "  \"config\": {\n";
    output << "    \"elevation_mask_deg\": " << options.elevation_mask_deg << ",\n";
    output << "    \"snr_mask\": " << options.snr_mask << ",\n";
    output << "    \"outlier_detection\": "
           << (options.spp_config.enable_outlier_detection ? "true" : "false") << ",\n";
    output << "    \"outlier_threshold_sigma\": "
           << options.spp_config.outlier_threshold_sigma << ",\n";
    output << "    \"raim_fde\": "
           << (options.spp_config.enable_raim_fde ? "true" : "false") << ",\n";
    output << "    \"raim_fde_min_improvement_ratio\": "
           << options.spp_config.raim_fde_min_rms_improvement_ratio << ",\n";
    output << "    \"raim_fde_min_improvement_m\": "
           << options.spp_config.raim_fde_min_rms_improvement_m << ",\n";
    output << "    \"max_gdop\": " << options.spp_config.max_gdop << ",\n";
    output << "    \"max_residual_rms\": "
           << options.spp_config.max_residual_rms << ",\n";
    output << "    \"max_chi_square_per_dof\": "
           << options.spp_config.max_chi_square_per_dof << ",\n";
    output << "    \"variance_model\": "
           << (options.spp_config.use_variance_model ? "true" : "false") << ",\n";
    output << "    \"snr_reference_dbhz\": "
           << options.spp_config.snr_reference_dbhz << ",\n";
    output << "    \"robust_weighting\": "
           << (options.spp_config.enable_robust_weighting ? "true" : "false") << ",\n";
    output << "    \"robust_threshold_sigma\": "
           << options.spp_config.robust_weight_threshold_sigma << ",\n";
    output << "    \"robust_min_weight\": "
           << options.spp_config.robust_weight_min_factor << ",\n";
    output << "    \"adaptive_robust_weighting\": "
           << (options.spp_config.enable_adaptive_robust_weighting ? "true" : "false") << ",\n";
    output << "    \"adaptive_robust_activation_threshold_sigma\": "
           << options.spp_config.adaptive_robust_activation_threshold_sigma << ",\n";
    output << "    \"adaptive_robust_min_tail_measurements\": "
           << options.spp_config.adaptive_robust_min_tail_measurements << ",\n";
    output << "    \"adaptive_robust_min_tail_fraction\": "
           << options.spp_config.adaptive_robust_min_tail_fraction << ",\n";
    output << "    \"position_jump_gate\": "
           << (options.spp_config.enable_position_jump_gate ? "true" : "false") << ",\n";
    output << "    \"max_position_jump_rate_mps\": "
           << options.spp_config.max_position_jump_rate_mps << ",\n";
    output << "    \"max_position_jump_min_m\": "
           << options.spp_config.max_position_jump_min_m << ",\n";
    output << "    \"ionosphere_free\": "
           << (options.spp_config.use_ionosphere_free_combination ? "true" : "false") << ",\n";
    output << "    \"precise_products\": "
           << (options.spp_config.use_precise_products ? "true" : "false") << ",\n";
    output << "    \"ssr_corrections\": "
           << (options.spp_config.use_ssr_corrections ? "true" : "false") << ",\n";
    output << "    \"ionex_corrections\": "
           << (options.spp_config.use_ionex_corrections ? "true" : "false") << ",\n";
    output << "    \"dcb_corrections\": "
           << (options.spp_config.use_dcb_corrections ? "true" : "false") << "\n";
    output << "  },\n";
    output << "  \"spp_qc\": {\n";
    output << "    \"pre_qc_measurements\": " << summary.spp_pre_qc_measurements << ",\n";
    output << "    \"outlier_rejections\": " << summary.spp_outlier_rejections << ",\n";
    output << "    \"raim_fde_attempts\": " << summary.spp_raim_fde_attempts << ",\n";
    output << "    \"raim_fde_rejections\": " << summary.spp_raim_fde_rejections << ",\n";
    output << "    \"gdop_gate_rejections\": " << summary.spp_gdop_gate_rejections << ",\n";
    output << "    \"residual_gate_rejections\": "
           << summary.spp_residual_gate_rejections << ",\n";
    output << "    \"chi_square_gate_rejections\": "
           << summary.spp_chi_square_gate_rejections << ",\n";
    output << "    \"position_jump_gate_rejections\": "
           << summary.spp_position_jump_gate_rejections << ",\n";
    output << "    \"robust_weighted_measurements\": "
           << summary.spp_robust_weighted_measurements << ",\n";
    output << "    \"adaptive_robust_activations\": "
           << summary.spp_adaptive_robust_activations << ",\n";
    output << "    \"adaptive_robust_tail_measurements\": "
           << summary.spp_adaptive_robust_tail_measurements << ",\n";
    output << "    \"min_robust_weight_factor\": "
           << summary.spp_min_robust_weight_factor << ",\n";
    output << "    \"ionosphere_free_measurements\": "
           << summary.spp_ionosphere_free_measurements << ",\n";
    output << "    \"precise_orbit_clock_measurements\": "
           << summary.spp_precise_orbit_clock_measurements << ",\n";
    output << "    \"ssr_orbit_clock_corrections\": "
           << summary.spp_ssr_orbit_clock_corrections << ",\n";
    output << "    \"ssr_orbit_meters\": " << summary.spp_ssr_orbit_meters << ",\n";
    output << "    \"ssr_clock_meters\": " << summary.spp_ssr_clock_meters << ",\n";
    output << "    \"ssr_code_bias_corrections\": "
           << summary.spp_ssr_code_bias_corrections << ",\n";
    output << "    \"ssr_code_bias_meters\": "
           << summary.spp_ssr_code_bias_meters << ",\n";
    output << "    \"ionex_corrections\": " << summary.spp_ionex_corrections << ",\n";
    output << "    \"ionex_meters\": " << summary.spp_ionex_meters << ",\n";
    output << "    \"dcb_corrections\": " << summary.spp_dcb_corrections << ",\n";
    output << "    \"dcb_meters\": " << summary.spp_dcb_meters << ",\n";
    output << "    \"rejected_satellites\": {";
    bool first_satellite = true;
    for (const auto& [satellite, count] : summary.rejected_satellites) {
        output << (first_satellite ? "\n" : ",\n");
        output << "      \"" << jsonEscape(satellite) << "\": " << count;
        first_satellite = false;
    }
    if (!summary.rejected_satellites.empty()) {
        output << "\n    ";
    }
    output << "}\n";
    output << "  }\n";
    output << "}\n";
    return true;
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
        config.snr_mask = options.snr_mask;
        config.elevation_mask = options.elevation_mask_deg;

        libgnss::SPPProcessor processor;
        processor.setSPPConfig(options.spp_config);
        if (!processor.initialize(config)) {
            std::cerr << "Error: failed to initialize SPP processor\n";
            return 1;
        }
        if ((!options.sp3_path.empty() || !options.clk_path.empty()) &&
            !processor.loadPreciseProducts(options.sp3_path, options.clk_path)) {
            std::cerr << "Error: failed to load SP3/CLK products"
                      << " sp3=" << options.sp3_path
                      << " clk=" << options.clk_path << "\n";
            return 1;
        }
        if (!options.ssr_path.empty() && !processor.loadSSRProducts(options.ssr_path)) {
            std::cerr << "Error: failed to load SSR product: "
                      << options.ssr_path << "\n";
            return 1;
        }
        if (!options.ionex_path.empty() && !processor.loadIONEXProducts(options.ionex_path)) {
            std::cerr << "Error: failed to load IONEX product: "
                      << options.ionex_path << "\n";
            return 1;
        }
        if (!options.dcb_path.empty() && !processor.loadDCBProducts(options.dcb_path)) {
            std::cerr << "Error: failed to load DCB product: "
                      << options.dcb_path << "\n";
            return 1;
        }

        libgnss::Solution solution;
        RunSummary summary;
        summary.precise_loaded = processor.hasLoadedPreciseProducts();
        summary.ssr_loaded = processor.hasLoadedSSRProducts();
        summary.ionex_loaded = processor.hasLoadedIONEXProducts();
        summary.dcb_loaded = processor.hasLoadedDCBProducts();
        summary.precise_satellites = processor.getLoadedPreciseSatelliteCount();
        summary.ssr_satellites = processor.getLoadedSSRSatelliteCount();
        summary.ionex_maps = processor.getLoadedIONEXMapCount();
        summary.dcb_entries = processor.getLoadedDCBEntryCount();
        libgnss::ObservationData obs;
        while (obs_reader.readObservationEpoch(obs)) {
            if (options.max_epochs > 0 && summary.processed_epochs >= options.max_epochs) {
                break;
            }
            if (obs_header.approximate_position.norm() > 0.0) {
                obs.receiver_position = obs_header.approximate_position;
            }
            const auto epoch_solution = processor.processEpoch(obs, nav_data);
            summary.addSolution(epoch_solution);
            if (epoch_solution.isValid()) {
                solution.addSolution(epoch_solution);
            }
            ++summary.processed_epochs;
        }

        if (!solution.writeToFile(options.out_path)) {
            std::cerr << "Error: failed to write solution file: " << options.out_path << "\n";
            return 1;
        }

        if (!options.summary_json_path.empty() &&
            !writeSummaryJson(options.summary_json_path, options, summary)) {
            std::cerr << "Error: failed to write summary JSON: "
                      << options.summary_json_path << "\n";
            return 1;
        }

        if (!options.quiet) {
            std::cout << "Processed epochs: " << summary.processed_epochs << "\n";
            std::cout << "Valid solutions: " << summary.valid_solutions << "\n";
            std::cout << "Availability: " << std::fixed << std::setprecision(2)
                      << summary.availabilityRate() * 100.0 << "%\n";
            std::cout << "Mean satellites: " << std::setprecision(2)
                      << summary.meanSatellites() << "\n";
            std::cout << "Mean GDOP: " << std::setprecision(2)
                      << summary.meanGdop() << "\n";
            std::cout << "Mean residual RMS [m]: " << std::setprecision(3)
                      << summary.meanResidualRms() << "\n";
            std::cout << "SPP QC: outliers=" << summary.spp_outlier_rejections
                      << " raim_fde=" << summary.spp_raim_fde_rejections
                      << " robust_weighted=" << summary.spp_robust_weighted_measurements
                      << " adaptive_robust=" << summary.spp_adaptive_robust_activations
                      << " iflc=" << summary.spp_ionosphere_free_measurements
                      << " precise=" << summary.spp_precise_orbit_clock_measurements
                      << " ssr_orbclk=" << summary.spp_ssr_orbit_clock_corrections
                      << " ssr_cbias=" << summary.spp_ssr_code_bias_corrections
                      << " ionex=" << summary.spp_ionex_corrections
                      << " dcb=" << summary.spp_dcb_corrections
                      << " gdop_gate=" << summary.spp_gdop_gate_rejections
                      << " residual_gate=" << summary.spp_residual_gate_rejections
                      << " chi_square_gate=" << summary.spp_chi_square_gate_rejections
                      << " jump_gate=" << summary.spp_position_jump_gate_rejections
                      << "\n";
            if (processor.hasLoadedPreciseProducts()) {
                std::cout << "Precise products satellites: "
                          << processor.getLoadedPreciseSatelliteCount()
                          << " measurements="
                          << summary.spp_precise_orbit_clock_measurements << "\n";
            }
            if (processor.hasLoadedSSRProducts()) {
                std::cout << "SSR satellites: "
                          << processor.getLoadedSSRSatelliteCount()
                          << " orbit_clock_corrections="
                          << summary.spp_ssr_orbit_clock_corrections
                          << " code_bias_corrections="
                          << summary.spp_ssr_code_bias_corrections
                          << " orbit_meters=" << summary.spp_ssr_orbit_meters
                          << " clock_meters=" << summary.spp_ssr_clock_meters
                          << " code_bias_meters="
                          << summary.spp_ssr_code_bias_meters << "\n";
            }
            if (processor.hasLoadedIONEXProducts()) {
                std::cout << "IONEX maps: " << processor.getLoadedIONEXMapCount()
                          << " corrections=" << summary.spp_ionex_corrections
                          << " meters=" << summary.spp_ionex_meters << "\n";
            }
            if (processor.hasLoadedDCBProducts()) {
                std::cout << "DCB entries: " << processor.getLoadedDCBEntryCount()
                          << " corrections=" << summary.spp_dcb_corrections
                          << " meters=" << summary.spp_dcb_meters << "\n";
            }
            std::cout << "Output: " << options.out_path << "\n";
            if (!options.summary_json_path.empty()) {
                std::cout << "Summary JSON: " << options.summary_json_path << "\n";
            }
        }

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
