#include <algorithm>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
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
    std::string ionex_path;
    std::string dcb_path;
    std::string antex_path;
    std::string blq_path;
    std::string ocean_loading_station_name;
    std::string out_path;
    std::string summary_json_path;
    std::string kml_path;
    int max_epochs = 0;
    int convergence_min_epochs = 20;
    double ssr_step_seconds = 1.0;
    bool estimate_troposphere = true;
    bool estimate_ionosphere = false;
    bool use_ionosphere_free = true;
    bool use_clas_osr_filter = false;
    std::string clas_epoch_policy = "strict-osr";
    std::string clas_osr_application = "full-osr";
    std::string clas_phase_continuity = "full-repair";
    std::string clas_phase_bias_values = "full";
    std::string clas_phase_bias_reference_time = "phase-bias-reference";
    std::string clas_ssr_timing = "lag-tolerant";
    std::string clas_expanded_values = "full-composed";
    std::string clas_subtype12_values = "full";
    std::string clas_residual_sampling = "indexed-or-mean";
    std::string clas_atmos_selection = "grid-first";
    double clas_atmos_stale_after_seconds = 15.0;
    bool kinematic_mode = false;
    bool low_dynamics_mode = false;
    bool enable_ar = false;
    double ar_ratio_threshold = 3.0;
    bool use_iers_solid_tide = false;
    std::string eop_c04_file;
    bool use_iers_pole_tide = false;
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
        << "  --ionex <maps.ionex>     Optional IONEX TEC map product\n"
        << "  --dcb <bias.bsx>         Optional DCB / Bias-SINEX product\n"
        << "  --antex <antennas.atx>   Optional ANTEX file for receiver antenna PCO\n"
        << "  --blq <station.blq>      Optional BLQ ocean loading coefficient file\n"
        << "  --ocean-loading-station <name>\n"
        << "                          Station name to select from the BLQ file\n"
        << "  --ssr-step-seconds <seconds>\n"
        << "                          Sampling step for RTCM SSR conversion (default: 1.0)\n"
        << "  --out <solution.pos>     Output position file (required)\n"
        << "  --summary-json <summary.json>\n"
        << "                          Optional machine-readable run summary\n"
        << "  --kml <solution.kml>     Optional KML output\n"
        << "  --max-epochs <count>     Limit processed epochs (default: all)\n"
        << "  --convergence-min-epochs <count>\n"
        << "                          Minimum epochs before PPP convergence/AR checks (default: 20)\n"
        << "  --no-estimate-troposphere\n"
        << "                          Disable zenith troposphere estimation\n"
        << "  --estimate-troposphere  Enable zenith troposphere estimation (default)\n"
        << "  --clas-osr              Use the CLAS OSR-based PPP-RTK filter path\n"
        << "  --clas-epoch-policy <strict-osr|hybrid-standard-ppp>\n"
        << "                          CLAS epoch boundary policy (default: strict-osr)\n"
        << "  --clas-osr-application <full-osr|orbit-clock-bias|orbit-clock-only>\n"
        << "                          CLAS accepted-update correction semantics (default: full-osr)\n"
        << "  --clas-phase-continuity <full-repair|sis-continuity-only|repair-only|raw-phase-bias|no-phase-bias>\n"
        << "                          CLAS phase-bias / continuity semantics (default: full-repair)\n"
        << "  --clas-phase-bias-values <full|phase-bias-only|compensation-only>\n"
        << "                          CLAS phase-bias value-construction policy before SIS/repair injection (default: full)\n"
        << "  --clas-phase-bias-reference-time <phase-bias-reference|clock-reference|observation-epoch>\n"
        << "                          CLAS phase-bias reference-time semantics before SIS/repair injection (default: phase-bias-reference)\n"
        << "  --clas-ssr-timing <lag-tolerant|clock-bound-phase-bias|clock-bound-atmos-and-phase-bias>\n"
        << "                          CLAS expanded-SSR source timing policy (default: lag-tolerant)\n"
        << "  --clas-expanded-values <full-composed|residual-only|polynomial-only>\n"
        << "                          CLAS expanded atmosphere-value construction policy (default: full-composed)\n"
        << "  --clas-subtype12-values <full|planar|offset-only>\n"
        << "                          CLAS subtype-12 surface-term policy before residual formation (default: full)\n"
        << "  --clas-residual-sampling <indexed-or-mean|indexed-only|mean-only>\n"
        << "                          CLAS residual-list sampling policy inside expanded atmosphere rows (default: indexed-or-mean)\n"
        << "  --clas-atmos-selection <grid-first|grid-guarded|balanced|freshness-first>\n"
        << "                          CLAS atmosphere token selection policy (default: grid-first)\n"
        << "  --clas-atmos-stale-after-seconds <seconds>\n"
        << "                          Balanced policy stale threshold (default: 15.0)\n"
        << "  --static                Use a static PPP motion model (default)\n"
        << "  --kinematic             Use a kinematic PPP motion model\n"
        << "  --low-dynamics          Keep kinematic PPP anchored for quasi-static motion\n"
        << "  --no-low-dynamics       Disable quasi-static anchoring (default)\n"
        << "  --enable-ar             Enable PPP ambiguity fixing when supported\n"
        << "  --disable-ar            Disable PPP ambiguity fixing (default)\n"
        << "  --ar-ratio-threshold <value>\n"
        << "                          Ratio threshold for PPP ambiguity fixing (default: 3.0)\n"
        << "  --use-iers-solid-tide   Use the IERS Conventions 2010 (Dehant) Step-1+Step-2\n"
        << "                          solid-earth-tide model instead of the simplified\n"
        << "                          Step-1-only built-in approximation. Opt-in pending\n"
        << "                          truth-bench validation. See docs/iers-integration-plan.md\n"
        << "  --no-iers-solid-tide    Use the built-in Step-1-only solid-earth-tide model (default)\n"
        << "  --eop-c04 <eopc04.txt>   IERS 20 C04 Earth Orientation Parameter file (daily samples).\n"
        << "                          Loaded into PPPProcessor for downstream IERS Phase D consumers\n"
        << "                          (pole tide, sub-daily EOP). Phase D-0 scaffolding only — does\n"
        << "                          not change PPP output until D-1+ feature flags are enabled.\n"
        << "                          Source: https://hpiers.obspm.fr/iers/eop/eopc04/eopc04.1962-now\n"
        << "  --use-iers-pole-tide     Apply the IERS Conventions 2010 §7.1.4 pole-tide station\n"
        << "                          displacement (Phase D-1). Requires --eop-c04 to be set;\n"
        << "                          otherwise the displacement is silently skipped because\n"
        << "                          xp/yp are unknown. Opt-in pending truth-bench validation.\n"
        << "  --no-iers-pole-tide      Skip the pole-tide model (default)\n"
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
        } else if (arg == "--ionex" && i + 1 < argc) {
            options.ionex_path = argv[++i];
        } else if (arg == "--dcb" && i + 1 < argc) {
            options.dcb_path = argv[++i];
        } else if (arg == "--antex" && i + 1 < argc) {
            options.antex_path = argv[++i];
        } else if (arg == "--blq" && i + 1 < argc) {
            options.blq_path = argv[++i];
        } else if (arg == "--ocean-loading-station" && i + 1 < argc) {
            options.ocean_loading_station_name = argv[++i];
        } else if (arg == "--out" && i + 1 < argc) {
            options.out_path = argv[++i];
        } else if (arg == "--summary-json" && i + 1 < argc) {
            options.summary_json_path = argv[++i];
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
        } else if (arg == "--clas-osr") {
            options.use_clas_osr_filter = true;
        } else if (arg == "--clas-epoch-policy" && i + 1 < argc) {
            options.clas_epoch_policy = argv[++i];
        } else if (arg == "--clas-osr-application" && i + 1 < argc) {
            options.clas_osr_application = argv[++i];
        } else if (arg == "--clas-phase-continuity" && i + 1 < argc) {
            options.clas_phase_continuity = argv[++i];
        } else if (arg == "--clas-phase-bias-values" && i + 1 < argc) {
            options.clas_phase_bias_values = argv[++i];
        } else if (arg == "--clas-phase-bias-reference-time" && i + 1 < argc) {
            options.clas_phase_bias_reference_time = argv[++i];
        } else if (arg == "--clas-ssr-timing" && i + 1 < argc) {
            options.clas_ssr_timing = argv[++i];
        } else if (arg == "--clas-expanded-values" && i + 1 < argc) {
            options.clas_expanded_values = argv[++i];
        } else if (arg == "--clas-subtype12-values" && i + 1 < argc) {
            options.clas_subtype12_values = argv[++i];
        } else if (arg == "--clas-residual-sampling" && i + 1 < argc) {
            options.clas_residual_sampling = argv[++i];
        } else if (arg == "--clas-atmos-selection" && i + 1 < argc) {
            options.clas_atmos_selection = argv[++i];
        } else if (arg == "--clas-atmos-stale-after-seconds" && i + 1 < argc) {
            options.clas_atmos_stale_after_seconds = std::stod(argv[++i]);
        } else if (arg == "--no-ionosphere-free") {
            options.use_ionosphere_free = false;
        } else if (arg == "--ionosphere-free") {
            options.use_ionosphere_free = true;
        } else if (arg == "--estimate-ionosphere") {
            options.estimate_ionosphere = true;
        } else if (arg == "--no-estimate-ionosphere") {
            options.estimate_ionosphere = false;
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
        } else if (arg == "--use-iers-solid-tide") {
            options.use_iers_solid_tide = true;
        } else if (arg == "--no-iers-solid-tide") {
            options.use_iers_solid_tide = false;
        } else if (arg == "--eop-c04" && i + 1 < argc) {
            options.eop_c04_file = argv[++i];
        } else if (arg == "--use-iers-pole-tide") {
            options.use_iers_pole_tide = true;
        } else if (arg == "--no-iers-pole-tide") {
            options.use_iers_pole_tide = false;
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
    if (options.clas_atmos_stale_after_seconds <= 0.0) {
        argumentError("--clas-atmos-stale-after-seconds must be positive", argv[0]);
    }
    if (options.ssr_step_seconds <= 0.0) {
        argumentError("--ssr-step-seconds must be positive", argv[0]);
    }
    if (options.clas_epoch_policy != "strict-osr" &&
        options.clas_epoch_policy != "hybrid-standard-ppp") {
        argumentError(
            "--clas-epoch-policy must be one of: strict-osr, hybrid-standard-ppp",
            argv[0]);
    }
    if (options.clas_osr_application != "full-osr" &&
        options.clas_osr_application != "orbit-clock-bias" &&
        options.clas_osr_application != "orbit-clock-only") {
        argumentError(
            "--clas-osr-application must be one of: full-osr, orbit-clock-bias, orbit-clock-only",
            argv[0]);
    }
    if (options.clas_phase_continuity != "full-repair" &&
        options.clas_phase_continuity != "sis-continuity-only" &&
        options.clas_phase_continuity != "repair-only" &&
        options.clas_phase_continuity != "raw-phase-bias" &&
        options.clas_phase_continuity != "no-phase-bias") {
        argumentError(
            "--clas-phase-continuity must be one of: full-repair, sis-continuity-only, repair-only, raw-phase-bias, no-phase-bias",
            argv[0]);
    }
    if (options.clas_phase_bias_values != "full" &&
        options.clas_phase_bias_values != "phase-bias-only" &&
        options.clas_phase_bias_values != "compensation-only") {
        argumentError(
            "--clas-phase-bias-values must be one of: full, phase-bias-only, compensation-only",
            argv[0]);
    }
    if (options.clas_phase_bias_reference_time != "phase-bias-reference" &&
        options.clas_phase_bias_reference_time != "clock-reference" &&
        options.clas_phase_bias_reference_time != "observation-epoch") {
        argumentError(
            "--clas-phase-bias-reference-time must be one of: phase-bias-reference, clock-reference, observation-epoch",
            argv[0]);
    }
    if (options.clas_ssr_timing != "lag-tolerant" &&
        options.clas_ssr_timing != "clock-bound-phase-bias" &&
        options.clas_ssr_timing != "clock-bound-atmos-and-phase-bias") {
        argumentError(
            "--clas-ssr-timing must be one of: lag-tolerant, clock-bound-phase-bias, clock-bound-atmos-and-phase-bias",
            argv[0]);
    }
    if (options.clas_expanded_values != "full-composed" &&
        options.clas_expanded_values != "residual-only" &&
        options.clas_expanded_values != "polynomial-only") {
        argumentError(
            "--clas-expanded-values must be one of: full-composed, residual-only, polynomial-only",
            argv[0]);
    }
    if (options.clas_subtype12_values != "full" &&
        options.clas_subtype12_values != "planar" &&
        options.clas_subtype12_values != "offset-only") {
        argumentError(
            "--clas-subtype12-values must be one of: full, planar, offset-only",
            argv[0]);
    }
    if (options.clas_residual_sampling != "indexed-or-mean" &&
        options.clas_residual_sampling != "indexed-only" &&
        options.clas_residual_sampling != "mean-only") {
        argumentError(
            "--clas-residual-sampling must be one of: indexed-or-mean, indexed-only, mean-only",
            argv[0]);
    }
    if (options.clas_atmos_selection != "grid-first" &&
        options.clas_atmos_selection != "grid-guarded" &&
        options.clas_atmos_selection != "balanced" &&
        options.clas_atmos_selection != "freshness-first") {
        argumentError(
            "--clas-atmos-selection must be one of: grid-first, grid-guarded, balanced, freshness-first",
            argv[0]);
    }
    return options;
}

std::string jsonEscape(const std::string& value) {
    std::ostringstream escaped;
    for (const char ch : value) {
        switch (ch) {
        case '\\':
            escaped << "\\\\";
            break;
        case '"':
            escaped << "\\\"";
            break;
        case '\n':
            escaped << "\\n";
            break;
        case '\r':
            escaped << "\\r";
            break;
        case '\t':
            escaped << "\\t";
            break;
        default:
            escaped << ch;
            break;
        }
    }
    return escaped.str();
}

}  // namespace

namespace {

libgnss::PPPProcessor::PPPConfig::ClasEpochPolicy parseClasEpochPolicy(
    const std::string& value) {
    using Policy = libgnss::PPPProcessor::PPPConfig::ClasEpochPolicy;
    if (value == "hybrid-standard-ppp") {
        return Policy::HYBRID_STANDARD_PPP_FALLBACK;
    }
    return Policy::STRICT_OSR;
}

libgnss::PPPProcessor::PPPConfig::ClasAtmosSelectionPolicy parseClasAtmosSelectionPolicy(
    const std::string& value) {
    using Policy = libgnss::PPPProcessor::PPPConfig::ClasAtmosSelectionPolicy;
    if (value == "grid-first") {
        return Policy::GRID_FIRST;
    }
    if (value == "grid-guarded") {
        return Policy::GRID_GUARDED;
    }
    if (value == "balanced") {
        return Policy::BALANCED;
    }
    return Policy::FRESHNESS_FIRST;
}

libgnss::PPPProcessor::PPPConfig::ClasCorrectionApplicationPolicy parseClasCorrectionApplicationPolicy(
    const std::string& value) {
    using Policy = libgnss::PPPProcessor::PPPConfig::ClasCorrectionApplicationPolicy;
    if (value == "orbit-clock-bias") {
        return Policy::ORBIT_CLOCK_BIAS;
    }
    if (value == "orbit-clock-only") {
        return Policy::ORBIT_CLOCK_ONLY;
    }
    return Policy::FULL_OSR;
}

libgnss::PPPProcessor::PPPConfig::ClasPhaseContinuityPolicy parseClasPhaseContinuityPolicy(
    const std::string& value) {
    using Policy = libgnss::PPPProcessor::PPPConfig::ClasPhaseContinuityPolicy;
    if (value == "sis-continuity-only") {
        return Policy::SIS_CONTINUITY_ONLY;
    }
    if (value == "repair-only") {
        return Policy::REPAIR_ONLY;
    }
    if (value == "raw-phase-bias") {
        return Policy::RAW_PHASE_BIAS;
    }
    if (value == "no-phase-bias") {
        return Policy::NO_PHASE_BIAS;
    }
    return Policy::FULL_REPAIR;
}

libgnss::PPPProcessor::PPPConfig::ClasPhaseBiasValuePolicy parseClasPhaseBiasValuePolicy(
    const std::string& value) {
    using Policy = libgnss::PPPProcessor::PPPConfig::ClasPhaseBiasValuePolicy;
    if (value == "phase-bias-only") {
        return Policy::PHASE_BIAS_ONLY;
    }
    if (value == "compensation-only") {
        return Policy::COMPENSATION_ONLY;
    }
    return Policy::FULL;
}

libgnss::PPPProcessor::PPPConfig::ClasPhaseBiasReferenceTimePolicy
parseClasPhaseBiasReferenceTimePolicy(const std::string& value) {
    using Policy = libgnss::PPPProcessor::PPPConfig::ClasPhaseBiasReferenceTimePolicy;
    if (value == "clock-reference") {
        return Policy::CLOCK_REFERENCE;
    }
    if (value == "observation-epoch") {
        return Policy::OBSERVATION_EPOCH;
    }
    return Policy::PHASE_BIAS_REFERENCE;
}

libgnss::PPPProcessor::PPPConfig::ClasSsrTimingPolicy parseClasSsrTimingPolicy(
    const std::string& value) {
    using Policy = libgnss::PPPProcessor::PPPConfig::ClasSsrTimingPolicy;
    if (value == "clock-bound-phase-bias") {
        return Policy::CLOCK_BOUND_PHASE_BIAS;
    }
    if (value == "clock-bound-atmos-and-phase-bias") {
        return Policy::CLOCK_BOUND_ATMOS_AND_PHASE_BIAS;
    }
    return Policy::LAG_TOLERANT;
}

libgnss::PPPProcessor::PPPConfig::ClasExpandedValueConstructionPolicy parseClasExpandedValueConstructionPolicy(
    const std::string& value) {
    using Policy = libgnss::PPPProcessor::PPPConfig::ClasExpandedValueConstructionPolicy;
    if (value == "residual-only") {
        return Policy::RESIDUAL_ONLY;
    }
    if (value == "polynomial-only") {
        return Policy::POLYNOMIAL_ONLY;
    }
    return Policy::FULL_COMPOSED;
}

libgnss::PPPProcessor::PPPConfig::ClasExpandedResidualSamplingPolicy parseClasExpandedResidualSamplingPolicy(
    const std::string& value) {
    using Policy = libgnss::PPPProcessor::PPPConfig::ClasExpandedResidualSamplingPolicy;
    if (value == "indexed-only") {
        return Policy::INDEXED_ONLY;
    }
    if (value == "mean-only") {
        return Policy::MEAN_ONLY;
    }
    return Policy::INDEXED_OR_MEAN;
}

libgnss::PPPProcessor::PPPConfig::ClasSubtype12ValueConstructionPolicy
parseClasSubtype12ValueConstructionPolicy(const std::string& value) {
    using Policy = libgnss::PPPProcessor::PPPConfig::ClasSubtype12ValueConstructionPolicy;
    if (value == "planar") {
        return Policy::PLANAR;
    }
    if (value == "offset-only") {
        return Policy::OFFSET_ONLY;
    }
    return Policy::FULL;
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
        ppp_config.ionex_file_path = options.ionex_path;
        ppp_config.dcb_file_path = options.dcb_path;
        ppp_config.antex_file_path = options.antex_path;
        ppp_config.ocean_loading_file_path = options.blq_path;
        ppp_config.estimate_troposphere = options.estimate_troposphere;
        ppp_config.estimate_ionosphere = options.estimate_ionosphere;
        ppp_config.use_ionosphere_free = options.use_ionosphere_free;
        ppp_config.use_clas_osr_filter = options.use_clas_osr_filter;
        ppp_config.clas_epoch_policy =
            parseClasEpochPolicy(options.clas_epoch_policy);
        ppp_config.clas_correction_application_policy =
            parseClasCorrectionApplicationPolicy(options.clas_osr_application);
        ppp_config.clas_phase_continuity_policy =
            parseClasPhaseContinuityPolicy(options.clas_phase_continuity);
        ppp_config.clas_phase_bias_value_policy =
            parseClasPhaseBiasValuePolicy(options.clas_phase_bias_values);
        ppp_config.clas_phase_bias_reference_time_policy =
            parseClasPhaseBiasReferenceTimePolicy(options.clas_phase_bias_reference_time);
        ppp_config.clas_ssr_timing_policy =
            parseClasSsrTimingPolicy(options.clas_ssr_timing);
        ppp_config.clas_expanded_value_construction_policy =
            parseClasExpandedValueConstructionPolicy(options.clas_expanded_values);
        ppp_config.clas_subtype12_value_construction_policy =
            parseClasSubtype12ValueConstructionPolicy(options.clas_subtype12_values);
        ppp_config.clas_expanded_residual_sampling_policy =
            parseClasExpandedResidualSamplingPolicy(options.clas_residual_sampling);
        ppp_config.clas_atmos_selection_policy =
            parseClasAtmosSelectionPolicy(options.clas_atmos_selection);
        ppp_config.clas_atmos_stale_after_seconds =
            options.clas_atmos_stale_after_seconds;
        ppp_config.kinematic_mode = options.kinematic_mode;
        ppp_config.low_dynamics_mode = options.low_dynamics_mode;
        ppp_config.enable_ambiguity_resolution = options.enable_ar;
        ppp_config.convergence_min_epochs = options.convergence_min_epochs;
        ppp_config.ar_ratio_threshold = options.ar_ratio_threshold;
        ppp_config.use_iers_solid_tide = options.use_iers_solid_tide;
        ppp_config.eop_path = options.eop_c04_file;
        ppp_config.use_iers_pole_tide = options.use_iers_pole_tide;
        if (options.low_dynamics_mode) {
            ppp_config.reset_clock_to_spp_each_epoch = false;
            ppp_config.reset_kinematic_position_to_spp_each_epoch = false;
            ppp_config.use_dynamics_model = false;
            ppp_config.process_noise_position = 0.0;
            ppp_config.process_noise_velocity = 1e-8;
        }
        // GPS week for L6 binary decode (only needed when SSR is L6 format)
        if (obs_header.first_obs.week > 0) {
            ppp_config.l6_gps_week = obs_header.first_obs.week;
        }
        ppp_config.approximate_position = obs_header.approximate_position;
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
        int ionex_corrections = 0;
        int dcb_corrections = 0;
        int clas_hybrid_fallback_epochs = 0;
        std::map<std::string, int> clas_hybrid_fallback_reasons;
        double atmospheric_trop_meters = 0.0;
        double atmospheric_iono_meters = 0.0;
        double ionex_meters = 0.0;
        double dcb_meters = 0.0;
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
            ionex_corrections += processor.getLastAppliedIonexCorrections();
            dcb_corrections += processor.getLastAppliedDcbCorrections();
            atmospheric_trop_meters +=
                processor.getLastAppliedAtmosphericTroposphereMeters();
            atmospheric_iono_meters +=
                processor.getLastAppliedAtmosphericIonosphereMeters();
            ionex_meters += processor.getLastAppliedIonexMeters();
            dcb_meters += processor.getLastAppliedDcbMeters();
            if (processor.getLastClasHybridFallbackUsed()) {
                ++clas_hybrid_fallback_epochs;
                ++clas_hybrid_fallback_reasons[processor.getLastClasHybridFallbackReason()];
            }
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

        const auto stats = processor.getStats();
        const double ppp_solution_rate =
            valid_solutions > 0 ?
                100.0 * static_cast<double>(ppp_float_solutions + ppp_fixed_solutions) /
                    static_cast<double>(valid_solutions) :
                0.0;
        if (!options.summary_json_path.empty()) {
            const std::filesystem::path summary_path(options.summary_json_path);
            if (!summary_path.parent_path().empty()) {
                std::filesystem::create_directories(summary_path.parent_path());
            }
            std::ofstream summary(summary_path);
            if (!summary.is_open()) {
                std::cerr << "Error: failed to write summary JSON: "
                          << options.summary_json_path << "\n";
                return 1;
            }
            summary << "{\n"
                    << "  \"obs\": \"" << jsonEscape(options.obs_path) << "\",\n"
                    << "  \"nav\": "
                    << (options.nav_path.empty() ? "null" : ("\"" + jsonEscape(options.nav_path) + "\""))
                    << ",\n"
                    << "  \"sp3\": "
                    << (options.sp3_path.empty() ? "null" : ("\"" + jsonEscape(options.sp3_path) + "\""))
                    << ",\n"
                    << "  \"clk\": "
                    << (options.clk_path.empty() ? "null" : ("\"" + jsonEscape(options.clk_path) + "\""))
                    << ",\n"
                    << "  \"ionex\": "
                    << (options.ionex_path.empty() ? "null" : ("\"" + jsonEscape(options.ionex_path) + "\""))
                    << ",\n"
                    << "  \"dcb\": "
                    << (options.dcb_path.empty() ? "null" : ("\"" + jsonEscape(options.dcb_path) + "\""))
                    << ",\n"
                    << "  \"out\": \"" << jsonEscape(options.out_path) << "\",\n"
                    << "  \"mode\": \"" << (options.kinematic_mode ? "kinematic" : "static") << "\",\n"
                    << "  \"low_dynamics\": " << (options.low_dynamics_mode ? "true" : "false") << ",\n"
                    << "  \"clas_epoch_policy\": \"" << jsonEscape(options.clas_epoch_policy) << "\",\n"
                    << "  \"clas_osr_application\": \"" << jsonEscape(options.clas_osr_application) << "\",\n"
                    << "  \"clas_phase_continuity\": \"" << jsonEscape(options.clas_phase_continuity) << "\",\n"
                    << "  \"clas_phase_bias_values\": \"" << jsonEscape(options.clas_phase_bias_values) << "\",\n"
                    << "  \"clas_phase_bias_reference_time\": \"" << jsonEscape(options.clas_phase_bias_reference_time) << "\",\n"
                    << "  \"clas_ssr_timing\": \"" << jsonEscape(options.clas_ssr_timing) << "\",\n"
                    << "  \"clas_expanded_values\": \"" << jsonEscape(options.clas_expanded_values) << "\",\n"
                    << "  \"clas_subtype12_values\": \"" << jsonEscape(options.clas_subtype12_values) << "\",\n"
                    << "  \"clas_residual_sampling\": \"" << jsonEscape(options.clas_residual_sampling) << "\",\n"
                    << "  \"clas_atmos_selection\": \"" << jsonEscape(options.clas_atmos_selection) << "\",\n"
                    << "  \"clas_atmos_stale_after_seconds\": " << options.clas_atmos_stale_after_seconds << ",\n"
                    << "  \"ambiguity_resolution_enabled\": " << (options.enable_ar ? "true" : "false") << ",\n"
                    << "  \"ar_ratio_threshold\": " << options.ar_ratio_threshold << ",\n"
                    << "  \"processed_epochs\": " << processed_epochs << ",\n"
                    << "  \"valid_solutions\": " << valid_solutions << ",\n"
                    << "  \"ppp_float_solutions\": " << ppp_float_solutions << ",\n"
                    << "  \"ppp_fixed_solutions\": " << ppp_fixed_solutions << ",\n"
                    << "  \"fallback_solutions\": " << fallback_solutions << ",\n"
                    << "  \"clas_hybrid_fallback_epochs\": " << clas_hybrid_fallback_epochs << ",\n"
                    << "  \"ppp_solution_rate_pct\": " << ppp_solution_rate << ",\n"
                    << "  \"ssr_corrections_enabled\": "
                    << ((!options.ssr_path.empty() || !options.ssr_rtcm_path.empty()) ? "true" : "false") << ",\n"
                    << "  \"atmospheric_trop_corrections\": " << atmospheric_trop_corrections << ",\n"
                    << "  \"atmospheric_trop_meters\": " << atmospheric_trop_meters << ",\n"
                    << "  \"atmospheric_iono_corrections\": " << atmospheric_iono_corrections << ",\n"
                    << "  \"atmospheric_iono_meters\": " << atmospheric_iono_meters << ",\n"
                    << "  \"ionex_loaded\": " << (processor.hasLoadedIONEXProducts() ? "true" : "false") << ",\n"
                    << "  \"ionex_maps\": " << processor.getLoadedIONEXMapCount() << ",\n"
                    << "  \"ionex_corrections\": " << ionex_corrections << ",\n"
                    << "  \"ionex_meters\": " << ionex_meters << ",\n"
                    << "  \"dcb_loaded\": " << (processor.hasLoadedDCBProducts() ? "true" : "false") << ",\n"
                    << "  \"dcb_entries\": " << processor.getLoadedDCBEntryCount() << ",\n"
                    << "  \"dcb_corrections\": " << dcb_corrections << ",\n"
                    << "  \"dcb_meters\": " << dcb_meters << ",\n"
                    << "  \"clas_hybrid_fallback_reasons\": {";
            bool first_reason = true;
            for (const auto& [reason, count] : clas_hybrid_fallback_reasons) {
                if (!first_reason) {
                    summary << ", ";
                }
                summary << "\"" << jsonEscape(reason) << "\": " << count;
                first_reason = false;
            }
            summary << "},\n"
                    << "  \"converged\": " << (processor.hasConverged() ? "true" : "false") << ",\n"
                    << "  \"convergence_time_s\": " << processor.getConvergenceTime() << ",\n"
                    << "  \"average_processing_time_ms\": " << stats.average_processing_time_ms << "\n"
                    << "}\n";
        }

        if (!options.quiet) {
            std::cout << "PPP summary:\n";
            std::cout << "  processed epochs: " << processed_epochs << "\n";
            std::cout << "  valid solutions: " << valid_solutions << "\n";
            std::cout << "  PPP float solutions: " << ppp_float_solutions << "\n";
            std::cout << "  PPP fixed solutions: " << ppp_fixed_solutions << "\n";
            std::cout << "  fallback solutions: " << fallback_solutions << "\n";
            std::cout << "  CLAS epoch policy: " << options.clas_epoch_policy << "\n";
            std::cout << "  CLAS OSR application: " << options.clas_osr_application << "\n";
            std::cout << "  CLAS phase continuity: " << options.clas_phase_continuity << "\n";
            std::cout << "  CLAS phase-bias values: " << options.clas_phase_bias_values << "\n";
            std::cout << "  CLAS phase-bias reference time: "
                      << options.clas_phase_bias_reference_time << "\n";
            std::cout << "  CLAS SSR timing: " << options.clas_ssr_timing << "\n";
            std::cout << "  CLAS expanded values: " << options.clas_expanded_values << "\n";
            std::cout << "  CLAS subtype-12 values: " << options.clas_subtype12_values << "\n";
            std::cout << "  CLAS residual sampling: " << options.clas_residual_sampling << "\n";
            std::cout << "  mode: " << (options.kinematic_mode ? "kinematic" : "static") << "\n";
            std::cout << "  low dynamics: " << (options.low_dynamics_mode ? "on" : "off") << "\n";
            std::cout << "  CLAS atmosphere selection: " << options.clas_atmos_selection << "\n";
            std::cout << "  CLAS stale-after (s): " << options.clas_atmos_stale_after_seconds << "\n";
            if (clas_hybrid_fallback_epochs > 0) {
                std::cout << "  CLAS hybrid fallback epochs: "
                          << clas_hybrid_fallback_epochs << "\n";
            }
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
            if (processor.hasLoadedIONEXProducts()) {
                std::cout << "  ionex maps: "
                          << processor.getLoadedIONEXMapCount() << "\n";
                std::cout << "  ionex corrections: "
                          << ionex_corrections << "\n";
                std::cout << "  ionex meters: "
                          << ionex_meters << "\n";
            }
            if (processor.hasLoadedDCBProducts()) {
                std::cout << "  dcb entries: "
                          << processor.getLoadedDCBEntryCount() << "\n";
                std::cout << "  dcb corrections: "
                          << dcb_corrections << "\n";
                std::cout << "  dcb meters: "
                          << dcb_meters << "\n";
            }
            if (options.enable_ar) {
                std::cout << "  AR ratio threshold: " << options.ar_ratio_threshold << "\n";
            }
            if (valid_solutions > 0) {
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
