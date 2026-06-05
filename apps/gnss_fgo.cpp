#include <libgnss++/algorithms/fgo.hpp>
#include <libgnss++/algorithms/lambda.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/io/rinex.hpp>
#include <libgnss++/models/ionosphere.hpp>
#include <libgnss++/models/troposphere.hpp>

#ifdef GNSS_WITH_GTSAM
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/dataset.h>
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace {

struct Options {
    std::string obs_path;
    std::string base_path;
    std::string nav_path;
    std::string out_path;
    std::string summary_json_path;
    std::string epoch_debug_csv_path;
    std::string factor_debug_csv_path;
    std::string sd_factor_debug_csv_path;
    std::string lambda_debug_csv_path;
    std::string cost_trace_csv_path;
    std::string seed_pos_path;
    std::string preset = "default";
    std::string backend = "eigen";
    int max_epochs = 0;
    int skip_epochs = 0;
    int max_iterations = 8;
    double relative_cost_convergence_threshold = 0.0;
    double absolute_cost_convergence_threshold = 0.0;
    double pseudorange_sigma_m = 3.0;
    double pseudorange_elevation_sigma_power = 1.0;
    double motion_sigma_m = 50.0;
    double clock_motion_sigma_m = 300.0;
    double velocity_prior_sigma_mps = 100.0;
    double velocity_motion_sigma_m = 0.01;
    double ambiguity_between_sigma_cycles = 0.001;
    double position_prior_sigma_m = 0.0;
    double clock_prior_sigma_m = 0.0;
    double tdcp_sigma_m = 0.03;
    double carrier_phase_sigma_m = 0.01;
    double double_difference_pseudorange_sigma_m = 1.0;
    double double_difference_carrier_sigma_m = 0.02;
    double ambiguity_prior_sigma_m = 1000.0;
    double pseudorange_huber_threshold_sigma = 4.0;
    double carrier_phase_huber_threshold_sigma = 4.0;
    double tdcp_huber_threshold_sigma = 4.0;
    double fixed_ambiguity_sigma_m = 0.003;
    double ambiguity_fix_threshold_cycles = 0.15;
    double lambda_ratio_threshold = 3.0;
    double max_tdcp_gap_s = 2.0;
    double base_match_tolerance_s = 0.02;
    double base_interpolation_max_gap_s = 1.2;
    double seed_match_tolerance_s = 0.01;
    double seed_interpolation_max_gap_s = 0.0;
    double tdcp_slip_threshold_m = 10.0;
    double min_elevation_deg = 10.0;
    double min_snr_dbhz = 0.0;
    double double_difference_reference_min_snr_dbhz = -1.0;
    double double_difference_base_min_snr_dbhz = -1.0;
    int min_satellites_per_epoch = 4;
    int min_output_dd_carrier_factors_per_epoch = 0;
    double max_float_seed_position_divergence_m = 0.0;
    double max_float_position_jump_m = 0.0;
    int min_fixed_ambiguities = 4;
    int max_lambda_ambiguities = 12;
    bool use_carrier_phase_factors = false;
    bool fix_ambiguities = false;
    bool fix_all_ambiguities = false;
    bool use_lambda_ambiguity_fix = true;
    bool use_epoch_lambda_fixed_output = false;
    bool use_partial_lambda_ambiguity_fix = true;
    bool use_robust_loss = true;
    bool no_pseudorange_factors = false;
    bool no_double_difference_factors = false;
    bool use_single_difference_doppler_factors = false;
    bool use_single_difference_tdcp_factors = false;
    bool use_velocity_states = false;
    bool use_velocity_motion_factors = false;
    bool use_ambiguity_between_factors = false;
    bool linearize_double_difference_factors_at_seed = false;
    bool dd_ambiguity_per_epoch = false;
    bool no_ambiguity_priors = false;
    bool no_spp_seed = false;
    bool no_motion_factors = false;
    bool clock_only_motion_factors = false;
    bool no_tdcp_factors = false;
    bool no_tdcp_slip_reject = false;
    bool reject_rover_carrier_lli = false;
    bool exclude_glonass_qzss_sbas = false;
    bool insert_fixed_interval_gaps = false;
    bool use_ionosphere_model = true;
    bool use_troposphere_model = true;
    bool debug_problem_only = false;
    bool quiet = false;
};

void printUsage(const char* program_name) {
    std::cout
        << "Usage: " << program_name << " --obs <rover.obs> --nav <nav.rnx> --out <solution.pos>\n"
        << "Options:\n"
        << "  --obs <rover.obs>             RINEX observation file\n"
        << "  --base <base.obs>             Optional base RINEX for double-difference carrier factors\n"
        << "  --nav <nav.rnx>               RINEX navigation file\n"
        << "  --out <solution.pos>          Output position file\n"
        << "  --summary-json <summary.json> Write machine-readable run summary\n"
        << "  --epoch-debug-csv <debug.csv> Write per-epoch FGO diagnostics\n"
        << "  --factor-debug-csv <debug.csv>\n"
        << "                                Write per-DD-factor residual diagnostics\n"
        << "  --sd-factor-debug-csv <debug.csv>\n"
        << "                                Write taroz SD Doppler/TDCP diagnostics\n"
        << "  --lambda-debug-csv <debug.csv>\n"
        << "                                Write per-LAMBDA ambiguity/covariance diagnostics\n"
        << "  --cost-trace-csv <trace.csv>  Write optimizer cost by iteration\n"
        << "  --debug-problem-only          Build factors and debug outputs without optimizing\n"
        << "  --seed-pos <solution.pos>     Use LibGNSS++/RTKLIB POS rows as epoch initial positions\n"
        << "  --backend <name>              Optimizer backend: eigen, gtsam-pc (if built with GTSAM)\n"
        << "  --preset <name>               Defaults: default, real-data, real-data-float,\n"
        << "                                real-data-fixed, tdcp-only, taroz-p,\n"
        << "                                taroz-pc, taroz-amb-pdc\n"
        << "  --skip-epochs <n>             Skip the first n rover epochs before solving\n"
        << "  --max-epochs <n>              Stop after n epochs\n"
        << "  --max-iterations <n>          Batch Gauss-Newton iterations (default: 8)\n"
        << "  --relative-cost-threshold <v> Stop when relative cost decrease is below v\n"
        << "  --absolute-cost-threshold <v> Stop when absolute cost decrease is below v\n"
        << "  --pseudorange-sigma <m>       Code factor sigma in meters (default: 3)\n"
        << "  --pseudorange-elevation-power <p>\n"
        << "                                Code sigma elevation exponent (default: 1)\n"
        << "  --motion-sigma <m>            Adjacent position smoothness sigma (default: 50)\n"
        << "  --clock-motion-sigma <m>      Adjacent clock smoothness sigma (default: 300)\n"
        << "  --velocity-prior-sigma <m/s>  Velocity prior sigma (default: 100)\n"
        << "  --velocity-motion-sigma <m>   Taroz XXVV motion factor sigma (default: 0.01)\n"
        << "  --ambiguity-between-sigma <cycles>\n"
        << "                                Taroz ambiguity between sigma (default: 0.001)\n"
        << "  --position-prior-sigma <m>    Weak position prior sigma; 0 disables (default: 0)\n"
        << "  --clock-prior-sigma <m>       Weak clock prior sigma; 0 disables (default: 0)\n"
        << "  --tdcp-sigma <m>              TDCP factor sigma in meters (default: 0.03)\n"
        << "  --carrier-phase-sigma <m>     Carrier phase sigma in meters (default: 0.01)\n"
        << "  --dd-pseudorange-sigma <m>    Double-difference code sigma in meters (default: 1)\n"
        << "  --dd-carrier-sigma <m>        Double-difference carrier sigma in meters (default: 0.02)\n"
        << "  --ambiguity-prior-sigma <m>   Weak float ambiguity prior sigma (default: 1000)\n"
        << "  --pseudorange-huber-threshold <s>\n"
        << "                                Code Huber threshold in sigma units (default: 4)\n"
        << "  --carrier-phase-huber-threshold <s>\n"
        << "                                Carrier Huber threshold in sigma units (default: 4)\n"
        << "  --tdcp-huber-threshold <s>    TDCP Huber threshold in sigma units (default: 4)\n"
        << "  --fixed-ambiguity-sigma <m>   Fixed ambiguity constraint sigma (default: 0.003)\n"
        << "  --ambiguity-fix-threshold <c> Max fractional cycles for nearest-int fix (default: 0.15)\n"
        << "  --min-fixed-ambiguities <n>   Min accepted ambiguities before fixed pass (default: 4)\n"
        << "  --lambda-ratio-threshold <v>  LAMBDA ratio threshold (default: 3)\n"
        << "  --max-lambda-ambiguities <n>  Max ambiguity states passed to LAMBDA (default: 12)\n"
        << "  --max-tdcp-gap <s>            Max adjacent epoch gap for TDCP (default: 2)\n"
        << "  --base-match-tolerance <s>    Max rover/base epoch gap for DD factors (default: 0.02)\n"
        << "  --base-interpolation-max-gap <s>\n"
        << "                                Max bracketing base epoch gap for interpolation (default: 1.2)\n"
        << "  --seed-match-tolerance <s>    Max rover/seed POS epoch gap (default: 0.01)\n"
        << "  --seed-interpolation-max-gap <s>\n"
        << "                                Interpolate seed POS across gaps up to this length (default: 0)\n"
        << "  --tdcp-slip-threshold <m>     Max TDCP/code delta mismatch (default: 10)\n"
        << "  --min-elevation <deg>         Elevation mask in degrees (default: 10)\n"
        << "  --min-snr <dbhz>              SNR mask in dB-Hz (default: 0)\n"
        << "  --min-satellites-per-epoch <n>\n"
        << "                                Minimum usable measurements before keeping epoch (default: 4)\n"
        << "  --min-output-dd-carrier-factors-per-epoch <n>\n"
        << "                                Mark DD-only output epochs NONE below this carrier factor count\n"
        << "  --max-float-seed-divergence <m>\n"
        << "                                Mark FLOAT outputs NONE when farther than this from seed; 0 disables\n"
        << "  --max-float-position-jump <m>\n"
        << "                                Mark jumping FLOAT outputs NONE until the next FIXED; 0 disables\n"
        << "  --dd-reference-min-snr <dbhz> Rover reference SNR mask for DD (-1 inherits --min-snr)\n"
        << "  --dd-base-min-snr <dbhz>      Base SNR mask for DD factors (-1 inherits --min-snr)\n"
        << "  --carrier-phase-factors       Enable float ambiguity carrier phase factors\n"
        << "  --fix-ambiguities             Enable ambiguity fixed pass\n"
        << "  --fix-all-ambiguities         Include non-DD carrier ambiguities in fixed pass\n"
        << "  --no-lambda-ambiguity-fix     Use nearest-integer fixed pass instead of LAMBDA\n"
        << "  --epoch-lambda-fixed-output   Apply per-epoch LAMBDA fixed position output\n"
        << "  --no-epoch-lambda-fixed-output\n"
        << "                                Keep optimized float positions in output\n"
        << "  --no-partial-lambda-ambiguity-fix\n"
        << "                                Disable partial LAMBDA retry with fewer candidates\n"
        << "  --no-pseudorange-factors      Do not add raw pseudorange factors\n"
        << "  --no-dd-factors               Ignore --base for FGO double-difference factors\n"
        << "  --sd-doppler-factors          Add taroz-style SD Doppler factors\n"
        << "  --sd-tdcp-factors             Add taroz-style SD TDCP factors\n"
        << "  --velocity-states             Add per-epoch 3D velocity states\n"
        << "  --velocity-motion-factors     Add taroz XXVV position/velocity motion factors\n"
        << "  --ambiguity-between-factors   Add taroz epoch-to-epoch ambiguity constraints\n"
        << "  --dd-ambiguity-per-epoch      Do not carry DD ambiguity states across epochs\n"
        << "  --no-ambiguity-priors         Disable weak ambiguity prior factors\n"
        << "  --reject-rover-carrier-lli    Drop rover carrier observations with LLI=1\n"
        << "  --no-spp-seed                 Use receiver/seed POS initial positions instead of per-epoch SPP\n"
        << "  --no-robust-loss              Disable Huber robust weighting\n"
        << "  --no-motion-factors           Disable adjacent epoch smoothness factors\n"
        << "  --clock-only-motion-factors   Keep clock smoothness but disable position smoothness\n"
        << "  --no-tdcp-factors             Disable time-differenced carrier phase factors\n"
        << "  --no-tdcp-slip-reject         Keep TDCP factors even when code/phase jump check fails\n"
        << "  --ionosphere-model            Enable broadcast ionosphere correction\n"
        << "  --no-ionosphere-model         Disable broadcast ionosphere correction\n"
        << "  --troposphere-model           Enable Saastamoinen troposphere correction\n"
        << "  --no-troposphere-model        Disable Saastamoinen troposphere correction\n"
        << "  --exclude-glo-qzs-sbs         Exclude GLONASS, QZSS, and SBAS observations\n"
        << "  --insert-fixed-interval-gaps  Insert empty fixed-interval epochs before building factors\n"
        << "  --quiet                       Suppress run summary\n"
        << "  -h, --help                    Show this help\n";
}

[[noreturn]] void argumentError(const std::string& message, const char* program_name) {
    std::cerr << "Argument error: " << message << "\n\n";
    printUsage(program_name);
    std::exit(1);
}

void applyRealDataFloatPreset(Options& options) {
    options.max_iterations = 12;
    options.pseudorange_sigma_m = 4.0;
    options.motion_sigma_m = 100.0;
    options.clock_motion_sigma_m = 600.0;
    options.tdcp_sigma_m = 0.05;
    options.carrier_phase_sigma_m = 0.02;
    options.double_difference_pseudorange_sigma_m = 1.0;
    options.double_difference_carrier_sigma_m = 0.02;
    options.ambiguity_prior_sigma_m = 2000.0;
    options.pseudorange_huber_threshold_sigma = 3.0;
    options.carrier_phase_huber_threshold_sigma = 3.0;
    options.tdcp_huber_threshold_sigma = 3.0;
    options.max_tdcp_gap_s = 1.5;
    options.base_match_tolerance_s = 0.02;
    options.base_interpolation_max_gap_s = 1.2;
    options.tdcp_slip_threshold_m = 6.0;
    options.min_elevation_deg = 15.0;
    options.use_carrier_phase_factors = false;
    options.fix_ambiguities = false;
    options.use_lambda_ambiguity_fix = true;
    options.use_partial_lambda_ambiguity_fix = true;
    options.use_robust_loss = true;
    options.no_motion_factors = false;
    options.no_tdcp_factors = false;
    options.no_tdcp_slip_reject = false;
}

void applyTarozPcPreset(Options& options) {
    options.backend = "gtsam-pc";
    options.max_iterations = 1000;
    options.double_difference_pseudorange_sigma_m = 1.0;
    options.double_difference_carrier_sigma_m = 0.003;
    options.ambiguity_prior_sigma_m = 1e6;
    options.pseudorange_huber_threshold_sigma = 1.234;
    options.carrier_phase_huber_threshold_sigma = 1.234;
    options.lambda_ratio_threshold = 2.0;
    options.min_fixed_ambiguities = 5;
    options.min_elevation_deg = 15.0;
    options.min_snr_dbhz = 35.0;
    options.double_difference_reference_min_snr_dbhz = 0.0;
    options.double_difference_base_min_snr_dbhz = 0.0;
    options.seed_interpolation_max_gap_s = 30.0;
    options.no_spp_seed = true;
    options.use_carrier_phase_factors = false;
    options.fix_ambiguities = false;
    options.use_lambda_ambiguity_fix = true;
    options.use_partial_lambda_ambiguity_fix = true;
    options.use_robust_loss = true;
    options.no_pseudorange_factors = true;
    options.no_motion_factors = true;
    options.no_tdcp_factors = true;
    options.dd_ambiguity_per_epoch = true;
    options.no_ambiguity_priors = true;
    options.reject_rover_carrier_lli = true;
    options.exclude_glonass_qzss_sbas = true;
    options.use_ionosphere_model = false;
    options.use_troposphere_model = false;
}

void applyTarozAmbPdcPreset(Options& options) {
    applyTarozPcPreset(options);
    options.backend = "eigen";
    options.min_satellites_per_epoch = 0;
    options.min_output_dd_carrier_factors_per_epoch = 6;
    options.max_float_seed_position_divergence_m = 100.0;
    options.max_float_position_jump_m = 100.0;
    options.insert_fixed_interval_gaps = true;
    options.fix_ambiguities = false;
    options.no_motion_factors = true;
    options.no_tdcp_factors = true;
    options.use_single_difference_doppler_factors = true;
    options.use_single_difference_tdcp_factors = true;
    options.tdcp_huber_threshold_sigma = 1.234;
    options.use_velocity_states = true;
    options.use_velocity_motion_factors = true;
    options.velocity_prior_sigma_mps = 100.0;
    options.velocity_motion_sigma_m = 0.01;
    options.use_ambiguity_between_factors = true;
    options.linearize_double_difference_factors_at_seed = true;
    options.ambiguity_between_sigma_cycles = 0.001;
    options.use_epoch_lambda_fixed_output = true;
    options.relative_cost_convergence_threshold = 1e-5;
    options.absolute_cost_convergence_threshold = 1e-5;
    options.no_ambiguity_priors = true;
}

void applyTarozPPreset(Options& options) {
    options.backend = "eigen";
    options.max_iterations = 1000;
    options.pseudorange_sigma_m = 3.0;
    options.pseudorange_elevation_sigma_power = 0.5;
    options.position_prior_sigma_m = 1e3;
    options.clock_prior_sigma_m = 1e6;
    options.clock_motion_sigma_m = 100.0;
    options.pseudorange_huber_threshold_sigma = 1.234;
    options.min_elevation_deg = 15.0;
    options.min_snr_dbhz = 35.0;
    options.seed_interpolation_max_gap_s = 30.0;
    options.no_spp_seed = true;
    options.use_carrier_phase_factors = false;
    options.fix_ambiguities = false;
    options.use_lambda_ambiguity_fix = true;
    options.use_partial_lambda_ambiguity_fix = true;
    options.use_robust_loss = true;
    options.no_pseudorange_factors = false;
    options.no_double_difference_factors = true;
    options.no_motion_factors = false;
    options.clock_only_motion_factors = true;
    options.no_tdcp_factors = true;
    options.no_ambiguity_priors = true;
    options.reject_rover_carrier_lli = false;
    options.exclude_glonass_qzss_sbas = false;
    options.use_ionosphere_model = true;
    options.use_troposphere_model = true;
}

void applyPresetDefaults(Options& options, const char* program_name) {
    if (options.preset == "default") {
        return;
    }
    if (options.preset == "real-data" ||
        options.preset == "real-data-float") {
        applyRealDataFloatPreset(options);
        return;
    }
    if (options.preset == "real-data-fixed") {
        applyRealDataFloatPreset(options);
        options.fix_ambiguities = true;
        options.lambda_ratio_threshold = 1.5;
        options.min_fixed_ambiguities = 6;
        options.max_lambda_ambiguities = 16;
        options.fixed_ambiguity_sigma_m = 0.005;
        return;
    }
    if (options.preset == "tdcp-only") {
        options.max_iterations = 12;
        options.pseudorange_sigma_m = 4.0;
        options.motion_sigma_m = 100.0;
        options.clock_motion_sigma_m = 600.0;
        options.tdcp_sigma_m = 0.05;
        options.double_difference_pseudorange_sigma_m = 1.0;
        options.double_difference_carrier_sigma_m = 0.02;
        options.pseudorange_huber_threshold_sigma = 3.0;
        options.tdcp_huber_threshold_sigma = 3.0;
        options.max_tdcp_gap_s = 1.5;
        options.base_match_tolerance_s = 0.02;
        options.base_interpolation_max_gap_s = 1.2;
        options.tdcp_slip_threshold_m = 6.0;
        options.min_elevation_deg = 15.0;
        options.use_carrier_phase_factors = false;
        options.fix_ambiguities = false;
        options.use_robust_loss = true;
        options.no_motion_factors = false;
        options.no_tdcp_factors = false;
        options.no_tdcp_slip_reject = false;
        return;
    }
    if (options.preset == "taroz-p") {
        applyTarozPPreset(options);
        return;
    }
    if (options.preset == "taroz-pc") {
        applyTarozPcPreset(options);
        return;
    }
    if (options.preset == "taroz-amb-pdc") {
        applyTarozAmbPdcPreset(options);
        return;
    }
    argumentError("unsupported --preset value: " + options.preset, program_name);
}

Options parseArguments(int argc, char* argv[]) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            std::exit(0);
        }
        if (arg == "--preset" && i + 1 < argc) {
            options.preset = argv[++i];
        } else if (arg == "--preset") {
            argumentError("unknown or incomplete argument: " + arg, argv[0]);
        }
    }
    applyPresetDefaults(options, argv[0]);

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            std::exit(0);
        } else if (arg == "--obs" && i + 1 < argc) {
            options.obs_path = argv[++i];
        } else if (arg == "--base" && i + 1 < argc) {
            options.base_path = argv[++i];
        } else if (arg == "--nav" && i + 1 < argc) {
            options.nav_path = argv[++i];
        } else if (arg == "--out" && i + 1 < argc) {
            options.out_path = argv[++i];
        } else if (arg == "--summary-json" && i + 1 < argc) {
            options.summary_json_path = argv[++i];
        } else if (arg == "--epoch-debug-csv" && i + 1 < argc) {
            options.epoch_debug_csv_path = argv[++i];
        } else if (arg == "--factor-debug-csv" && i + 1 < argc) {
            options.factor_debug_csv_path = argv[++i];
        } else if (arg == "--sd-factor-debug-csv" && i + 1 < argc) {
            options.sd_factor_debug_csv_path = argv[++i];
        } else if (arg == "--lambda-debug-csv" && i + 1 < argc) {
            options.lambda_debug_csv_path = argv[++i];
        } else if (arg == "--cost-trace-csv" && i + 1 < argc) {
            options.cost_trace_csv_path = argv[++i];
        } else if (arg == "--seed-pos" && i + 1 < argc) {
            options.seed_pos_path = argv[++i];
        } else if (arg == "--backend" && i + 1 < argc) {
            options.backend = argv[++i];
        } else if (arg == "--preset" && i + 1 < argc) {
            ++i;
        } else if (arg == "--skip-epochs" && i + 1 < argc) {
            options.skip_epochs = std::stoi(argv[++i]);
        } else if (arg == "--max-epochs" && i + 1 < argc) {
            options.max_epochs = std::stoi(argv[++i]);
        } else if (arg == "--max-iterations" && i + 1 < argc) {
            options.max_iterations = std::stoi(argv[++i]);
        } else if (arg == "--relative-cost-threshold" && i + 1 < argc) {
            options.relative_cost_convergence_threshold = std::stod(argv[++i]);
        } else if (arg == "--absolute-cost-threshold" && i + 1 < argc) {
            options.absolute_cost_convergence_threshold = std::stod(argv[++i]);
        } else if (arg == "--pseudorange-sigma" && i + 1 < argc) {
            options.pseudorange_sigma_m = std::stod(argv[++i]);
        } else if (arg == "--pseudorange-elevation-power" && i + 1 < argc) {
            options.pseudorange_elevation_sigma_power = std::stod(argv[++i]);
        } else if (arg == "--motion-sigma" && i + 1 < argc) {
            options.motion_sigma_m = std::stod(argv[++i]);
        } else if (arg == "--clock-motion-sigma" && i + 1 < argc) {
            options.clock_motion_sigma_m = std::stod(argv[++i]);
        } else if (arg == "--velocity-prior-sigma" && i + 1 < argc) {
            options.velocity_prior_sigma_mps = std::stod(argv[++i]);
        } else if (arg == "--velocity-motion-sigma" && i + 1 < argc) {
            options.velocity_motion_sigma_m = std::stod(argv[++i]);
        } else if (arg == "--ambiguity-between-sigma" && i + 1 < argc) {
            options.ambiguity_between_sigma_cycles = std::stod(argv[++i]);
        } else if (arg == "--position-prior-sigma" && i + 1 < argc) {
            options.position_prior_sigma_m = std::stod(argv[++i]);
        } else if (arg == "--clock-prior-sigma" && i + 1 < argc) {
            options.clock_prior_sigma_m = std::stod(argv[++i]);
        } else if (arg == "--tdcp-sigma" && i + 1 < argc) {
            options.tdcp_sigma_m = std::stod(argv[++i]);
        } else if (arg == "--carrier-phase-sigma" && i + 1 < argc) {
            options.carrier_phase_sigma_m = std::stod(argv[++i]);
        } else if (arg == "--dd-pseudorange-sigma" && i + 1 < argc) {
            options.double_difference_pseudorange_sigma_m = std::stod(argv[++i]);
        } else if (arg == "--dd-carrier-sigma" && i + 1 < argc) {
            options.double_difference_carrier_sigma_m = std::stod(argv[++i]);
        } else if (arg == "--ambiguity-prior-sigma" && i + 1 < argc) {
            options.ambiguity_prior_sigma_m = std::stod(argv[++i]);
        } else if (arg == "--pseudorange-huber-threshold" && i + 1 < argc) {
            options.pseudorange_huber_threshold_sigma = std::stod(argv[++i]);
        } else if (arg == "--carrier-phase-huber-threshold" && i + 1 < argc) {
            options.carrier_phase_huber_threshold_sigma = std::stod(argv[++i]);
        } else if (arg == "--tdcp-huber-threshold" && i + 1 < argc) {
            options.tdcp_huber_threshold_sigma = std::stod(argv[++i]);
        } else if (arg == "--fixed-ambiguity-sigma" && i + 1 < argc) {
            options.fixed_ambiguity_sigma_m = std::stod(argv[++i]);
        } else if (arg == "--ambiguity-fix-threshold" && i + 1 < argc) {
            options.ambiguity_fix_threshold_cycles = std::stod(argv[++i]);
        } else if (arg == "--min-fixed-ambiguities" && i + 1 < argc) {
            options.min_fixed_ambiguities = std::stoi(argv[++i]);
        } else if (arg == "--lambda-ratio-threshold" && i + 1 < argc) {
            options.lambda_ratio_threshold = std::stod(argv[++i]);
        } else if (arg == "--max-lambda-ambiguities" && i + 1 < argc) {
            options.max_lambda_ambiguities = std::stoi(argv[++i]);
        } else if (arg == "--max-tdcp-gap" && i + 1 < argc) {
            options.max_tdcp_gap_s = std::stod(argv[++i]);
        } else if (arg == "--base-match-tolerance" && i + 1 < argc) {
            options.base_match_tolerance_s = std::stod(argv[++i]);
        } else if (arg == "--base-interpolation-max-gap" && i + 1 < argc) {
            options.base_interpolation_max_gap_s = std::stod(argv[++i]);
        } else if (arg == "--seed-match-tolerance" && i + 1 < argc) {
            options.seed_match_tolerance_s = std::stod(argv[++i]);
        } else if (arg == "--seed-interpolation-max-gap" && i + 1 < argc) {
            options.seed_interpolation_max_gap_s = std::stod(argv[++i]);
        } else if (arg == "--tdcp-slip-threshold" && i + 1 < argc) {
            options.tdcp_slip_threshold_m = std::stod(argv[++i]);
        } else if (arg == "--min-elevation" && i + 1 < argc) {
            options.min_elevation_deg = std::stod(argv[++i]);
        } else if (arg == "--min-snr" && i + 1 < argc) {
            options.min_snr_dbhz = std::stod(argv[++i]);
        } else if (arg == "--min-satellites-per-epoch" && i + 1 < argc) {
            options.min_satellites_per_epoch = std::stoi(argv[++i]);
        } else if (arg == "--min-output-dd-carrier-factors-per-epoch" &&
                   i + 1 < argc) {
            options.min_output_dd_carrier_factors_per_epoch =
                std::stoi(argv[++i]);
        } else if (arg == "--max-float-seed-divergence" && i + 1 < argc) {
            options.max_float_seed_position_divergence_m =
                std::stod(argv[++i]);
        } else if (arg == "--max-float-position-jump" && i + 1 < argc) {
            options.max_float_position_jump_m = std::stod(argv[++i]);
        } else if (arg == "--dd-reference-min-snr" && i + 1 < argc) {
            options.double_difference_reference_min_snr_dbhz = std::stod(argv[++i]);
        } else if (arg == "--dd-base-min-snr" && i + 1 < argc) {
            options.double_difference_base_min_snr_dbhz = std::stod(argv[++i]);
        } else if (arg == "--carrier-phase-factors") {
            options.use_carrier_phase_factors = true;
        } else if (arg == "--fix-ambiguities") {
            options.fix_ambiguities = true;
        } else if (arg == "--fix-all-ambiguities") {
            options.fix_all_ambiguities = true;
        } else if (arg == "--no-lambda-ambiguity-fix") {
            options.use_lambda_ambiguity_fix = false;
        } else if (arg == "--epoch-lambda-fixed-output") {
            options.use_epoch_lambda_fixed_output = true;
        } else if (arg == "--no-epoch-lambda-fixed-output") {
            options.use_epoch_lambda_fixed_output = false;
        } else if (arg == "--no-partial-lambda-ambiguity-fix") {
            options.use_partial_lambda_ambiguity_fix = false;
        } else if (arg == "--no-pseudorange-factors") {
            options.no_pseudorange_factors = true;
        } else if (arg == "--no-dd-factors") {
            options.no_double_difference_factors = true;
        } else if (arg == "--sd-doppler-factors") {
            options.use_single_difference_doppler_factors = true;
        } else if (arg == "--sd-tdcp-factors") {
            options.use_single_difference_tdcp_factors = true;
        } else if (arg == "--velocity-states") {
            options.use_velocity_states = true;
        } else if (arg == "--velocity-motion-factors") {
            options.use_velocity_motion_factors = true;
            options.use_velocity_states = true;
        } else if (arg == "--ambiguity-between-factors") {
            options.use_ambiguity_between_factors = true;
        } else if (arg == "--dd-ambiguity-per-epoch") {
            options.dd_ambiguity_per_epoch = true;
        } else if (arg == "--no-ambiguity-priors") {
            options.no_ambiguity_priors = true;
        } else if (arg == "--reject-rover-carrier-lli") {
            options.reject_rover_carrier_lli = true;
        } else if (arg == "--no-spp-seed") {
            options.no_spp_seed = true;
        } else if (arg == "--no-robust-loss") {
            options.use_robust_loss = false;
        } else if (arg == "--no-motion-factors") {
            options.no_motion_factors = true;
        } else if (arg == "--clock-only-motion-factors") {
            options.clock_only_motion_factors = true;
        } else if (arg == "--no-tdcp-factors") {
            options.no_tdcp_factors = true;
        } else if (arg == "--no-tdcp-slip-reject") {
            options.no_tdcp_slip_reject = true;
        } else if (arg == "--ionosphere-model") {
            options.use_ionosphere_model = true;
        } else if (arg == "--no-ionosphere-model") {
            options.use_ionosphere_model = false;
        } else if (arg == "--troposphere-model") {
            options.use_troposphere_model = true;
        } else if (arg == "--no-troposphere-model") {
            options.use_troposphere_model = false;
        } else if (arg == "--debug-problem-only") {
            options.debug_problem_only = true;
        } else if (arg == "--exclude-glo-qzs-sbs") {
            options.exclude_glonass_qzss_sbas = true;
        } else if (arg == "--insert-fixed-interval-gaps") {
            options.insert_fixed_interval_gaps = true;
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
    if (options.backend != "eigen" && options.backend != "gtsam-pc") {
        argumentError("--backend must be eigen or gtsam-pc", argv[0]);
    }
    if (options.max_epochs < 0) {
        argumentError("--max-epochs must be non-negative", argv[0]);
    }
    if (options.skip_epochs < 0) {
        argumentError("--skip-epochs must be non-negative", argv[0]);
    }
    if (options.max_iterations <= 0) {
        argumentError("--max-iterations must be positive", argv[0]);
    }
    if (options.relative_cost_convergence_threshold < 0.0 ||
        options.absolute_cost_convergence_threshold < 0.0) {
        argumentError("cost convergence thresholds must be non-negative", argv[0]);
    }
    if (options.pseudorange_sigma_m <= 0.0 ||
        options.motion_sigma_m <= 0.0 ||
        options.clock_motion_sigma_m <= 0.0 ||
        options.velocity_prior_sigma_mps <= 0.0 ||
        options.velocity_motion_sigma_m <= 0.0 ||
        options.ambiguity_between_sigma_cycles <= 0.0 ||
        options.tdcp_sigma_m <= 0.0 ||
        options.carrier_phase_sigma_m <= 0.0 ||
        options.double_difference_pseudorange_sigma_m <= 0.0 ||
        options.double_difference_carrier_sigma_m <= 0.0 ||
        options.ambiguity_prior_sigma_m <= 0.0 ||
        options.fixed_ambiguity_sigma_m <= 0.0) {
        argumentError("sigma values must be positive", argv[0]);
    }
    if (options.pseudorange_elevation_sigma_power < 0.0) {
        argumentError("--pseudorange-elevation-power must be non-negative", argv[0]);
    }
    if (options.position_prior_sigma_m < 0.0 ||
        options.clock_prior_sigma_m < 0.0) {
        argumentError("prior sigma values must be non-negative", argv[0]);
    }
    if (options.ambiguity_fix_threshold_cycles < 0.0) {
        argumentError("--ambiguity-fix-threshold must be non-negative", argv[0]);
    }
    if (options.pseudorange_huber_threshold_sigma < 0.0 ||
        options.carrier_phase_huber_threshold_sigma < 0.0 ||
        options.tdcp_huber_threshold_sigma < 0.0) {
        argumentError("Huber thresholds must be non-negative", argv[0]);
    }
    if (options.lambda_ratio_threshold < 0.0) {
        argumentError("--lambda-ratio-threshold must be non-negative", argv[0]);
    }
    if (options.min_fixed_ambiguities <= 0) {
        argumentError("--min-fixed-ambiguities must be positive", argv[0]);
    }
    if (options.min_satellites_per_epoch < 0) {
        argumentError("--min-satellites-per-epoch must be non-negative", argv[0]);
    }
    if (options.min_output_dd_carrier_factors_per_epoch < 0) {
        argumentError(
            "--min-output-dd-carrier-factors-per-epoch must be non-negative",
            argv[0]);
    }
    if (!std::isfinite(options.max_float_seed_position_divergence_m) ||
        options.max_float_seed_position_divergence_m < 0.0) {
        argumentError("--max-float-seed-divergence must be non-negative",
                      argv[0]);
    }
    if (!std::isfinite(options.max_float_position_jump_m) ||
        options.max_float_position_jump_m < 0.0) {
        argumentError("--max-float-position-jump must be non-negative",
                      argv[0]);
    }
    if (options.max_lambda_ambiguities < 0) {
        argumentError("--max-lambda-ambiguities must be non-negative", argv[0]);
    }
    if (options.max_tdcp_gap_s < 0.0) {
        argumentError("--max-tdcp-gap must be non-negative", argv[0]);
    }
    if (options.base_match_tolerance_s < 0.0) {
        argumentError("--base-match-tolerance must be non-negative", argv[0]);
    }
    if (options.base_interpolation_max_gap_s < 0.0) {
        argumentError("--base-interpolation-max-gap must be non-negative", argv[0]);
    }
    if (options.seed_match_tolerance_s < 0.0) {
        argumentError("--seed-match-tolerance must be non-negative", argv[0]);
    }
    if (options.seed_interpolation_max_gap_s < 0.0) {
        argumentError("--seed-interpolation-max-gap must be non-negative", argv[0]);
    }
    if (options.tdcp_slip_threshold_m < 0.0) {
        argumentError("--tdcp-slip-threshold must be non-negative", argv[0]);
    }
    if (options.double_difference_base_min_snr_dbhz < -1.0) {
        argumentError("--dd-base-min-snr must be >= -1", argv[0]);
    }
    if (options.double_difference_reference_min_snr_dbhz < -1.0) {
        argumentError("--dd-reference-min-snr must be >= -1", argv[0]);
    }
    return options;
}

double ratePercent(std::size_t numerator, std::size_t denominator) {
    if (denominator == 0) {
        return 0.0;
    }
    return 100.0 * static_cast<double>(numerator) /
           static_cast<double>(denominator);
}

std::string formatPercent(double value) {
    std::ostringstream formatted;
    formatted << std::fixed << std::setprecision(2) << value;
    return formatted.str();
}

bool isTarozExcludedSystem(libgnss::GNSSSystem system) {
    return system == libgnss::GNSSSystem::GLONASS ||
           system == libgnss::GNSSSystem::QZSS ||
           system == libgnss::GNSSSystem::SBAS;
}

void applySystemFilter(libgnss::ObservationData& epoch, const Options& options) {
    if (!options.exclude_glonass_qzss_sbas) {
        return;
    }
    epoch.observations.erase(
        std::remove_if(epoch.observations.begin(),
                       epoch.observations.end(),
                       [](const libgnss::Observation& observation) {
                           return isTarozExcludedSystem(
                               observation.satellite.system);
                       }),
        epoch.observations.end());
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

const char* jsonBool(bool value) {
    return value ? "true" : "false";
}

struct SeedPosition {
    libgnss::GNSSTime time;
    libgnss::Vector3d position_ecef = libgnss::Vector3d::Zero();
};

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

    constexpr double kDegreesToRadians = 0.017453292519943295769;
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

#ifndef GNSS_WITH_GTSAM
double ddGeometry(const libgnss::Vector3d& rover_position,
                  const libgnss::Vector3d& base_position,
                  const libgnss::Vector3d& rover_satellite_position,
                  const libgnss::Vector3d& rover_reference_position,
                  const libgnss::Vector3d& base_satellite_position,
                  const libgnss::Vector3d& base_reference_position) {
    return (rover_satellite_position - rover_position).norm() -
           (base_satellite_position - base_position).norm() -
           ((rover_reference_position - rover_position).norm() -
            (base_reference_position - base_position).norm());
}
#endif

#ifdef GNSS_WITH_GTSAM
class GtsamPseudorangeFactorX
    : public gtsam::NoiseModelFactorN<gtsam::Vector> {
public:
    GtsamPseudorangeFactorX(gtsam::Key key_x,
                            const gtsam::Vector& los,
                            double residual_m,
                            const gtsam::Vector& initial_x,
                            const gtsam::SharedNoiseModel& model)
        : gtsam::NoiseModelFactorN<gtsam::Vector>(model, key_x),
          los_(los),
          residual_m_(residual_m),
          initial_x_(initial_x) {}

    gtsam::Vector evaluateError(const gtsam::Vector& x,
                                gtsam::OptionalMatrixType hx) const override {
        gtsam::Vector error(1);
        error(0) = los_.dot(x - initial_x_) - residual_m_;
        if (hx) {
            *hx = los_.transpose();
        }
        return error;
    }

private:
    gtsam::Vector los_;
    double residual_m_ = 0.0;
    gtsam::Vector initial_x_;
};

class GtsamCarrierPhaseFactorXB
    : public gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector> {
public:
    GtsamCarrierPhaseFactorXB(gtsam::Key key_x,
                              gtsam::Key key_b,
                              const gtsam::Vector& los,
                              double residual_m,
                              int bias_index,
                              double wavelength_m,
                              const gtsam::Vector& initial_x,
                              const gtsam::SharedNoiseModel& model)
        : gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector>(
              model, key_x, key_b),
          los_(los),
          residual_m_(residual_m),
          bias_index_(bias_index),
          wavelength_m_(wavelength_m),
          initial_x_(initial_x) {}

    gtsam::Vector evaluateError(const gtsam::Vector& x,
                                const gtsam::Vector& b,
                                gtsam::OptionalMatrixType hx,
                                gtsam::OptionalMatrixType hb) const override {
        gtsam::Vector error(1);
        error(0) = los_.dot(x - initial_x_) -
                   (residual_m_ - wavelength_m_ * b(bias_index_));
        if (hx) {
            *hx = los_.transpose();
        }
        if (hb) {
            *hb = gtsam::Matrix::Zero(1, b.size());
            (*hb)(0, bias_index_) = wavelength_m_;
        }
        return error;
    }

private:
    gtsam::Vector los_;
    double residual_m_ = 0.0;
    int bias_index_ = 0;
    double wavelength_m_ = 0.0;
    gtsam::Vector initial_x_;
};

gtsam::Vector vector3ToGtsam(const libgnss::Vector3d& value) {
    gtsam::Vector converted(3);
    converted << value(0), value(1), value(2);
    return converted;
}

gtsam::Vector scalarSigma(double sigma) {
    gtsam::Vector sigmas(1);
    sigmas(0) = sigma;
    return sigmas;
}

gtsam::SharedNoiseModel robustScalarNoise(double sigma,
                                          double huber_threshold_sigma,
                                          bool use_robust_loss) {
    auto gaussian =
        gtsam::noiseModel::Diagonal::Sigmas(scalarSigma(std::max(1e-6, sigma)));
    if (!use_robust_loss || huber_threshold_sigma <= 0.0) {
        return gaussian;
    }
    return gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(huber_threshold_sigma),
        gaussian);
}

libgnss::Vector3d ddPositionJacobian(
    const libgnss::Vector3d& rover_position,
    const libgnss::Vector3d& satellite_position,
    const libgnss::Vector3d& reference_position) {
    const libgnss::Vector3d satellite_delta =
        satellite_position - rover_position;
    const libgnss::Vector3d reference_delta =
        reference_position - rover_position;
    return -satellite_delta.normalized() + reference_delta.normalized();
}

double ddGeometry(const libgnss::Vector3d& rover_position,
                  const libgnss::Vector3d& base_position,
                  const libgnss::Vector3d& rover_satellite_position,
                  const libgnss::Vector3d& rover_reference_position,
                  const libgnss::Vector3d& base_satellite_position,
                  const libgnss::Vector3d& base_reference_position) {
    return (rover_satellite_position - rover_position).norm() -
           (base_satellite_position - base_position).norm() -
           ((rover_reference_position - rover_position).norm() -
            (base_reference_position - base_position).norm());
}

double elevationRad(const libgnss::Vector3d& receiver_position,
                    const libgnss::Vector3d& satellite_position) {
    double lat = 0.0;
    double lon = 0.0;
    double height = 0.0;
    libgnss::ecef2geodetic(receiver_position, lat, lon, height);
    const libgnss::Vector3d enu =
        libgnss::ecef2enu(satellite_position - receiver_position, lat, lon);
    const double range = enu.norm();
    if (range <= 0.0) {
        return 0.0;
    }
    return std::asin(std::max(-1.0, std::min(1.0, enu(2) / range)));
}

double seedPositionDivergenceMeters(
    const libgnss::Vector3d& position_ecef,
    const libgnss::FGOProcessor::EpochSeed& seed) {
    if (!position_ecef.allFinite() || !seed.position_ecef.allFinite() ||
        seed.position_ecef.norm() <= 1e6) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return (position_ecef - seed.position_ecef).norm();
}

bool applyFloatSeedPositionDivergenceGate(
    libgnss::PositionSolution& solution,
    const libgnss::FGOProcessor::FGOProblem& problem,
    std::size_t epoch_index,
    double max_divergence_m,
    std::size_t& rejected_count) {
    if (solution.status != libgnss::SolutionStatus::FLOAT ||
        !std::isfinite(max_divergence_m) || max_divergence_m <= 0.0) {
        return false;
    }
    if (!solution.position_ecef.allFinite()) {
        solution.status = libgnss::SolutionStatus::NONE;
        ++rejected_count;
        return true;
    }
    if (epoch_index >= problem.epochs.size()) {
        return false;
    }

    const double divergence_m =
        seedPositionDivergenceMeters(solution.position_ecef,
                                     problem.epochs[epoch_index]);
    if (std::isfinite(divergence_m) && divergence_m > max_divergence_m) {
        solution.status = libgnss::SolutionStatus::NONE;
        ++rejected_count;
        return true;
    }
    return false;
}

bool applyFloatPositionJumpGate(
    libgnss::PositionSolution& solution,
    const libgnss::Vector3d& previous_output_position,
    bool has_previous_output_position,
    double max_jump_m,
    bool& block_float_until_fixed,
    std::size_t& rejected_count) {
    if (!std::isfinite(max_jump_m) || max_jump_m <= 0.0) {
        return false;
    }
    if (solution.status == libgnss::SolutionStatus::FIXED) {
        block_float_until_fixed = false;
        return false;
    }
    if (solution.status != libgnss::SolutionStatus::FLOAT) {
        return false;
    }

    bool reject = block_float_until_fixed;
    if (has_previous_output_position &&
        solution.position_ecef.allFinite() &&
        previous_output_position.allFinite()) {
        const double jump_m =
            (solution.position_ecef - previous_output_position).norm();
        if (std::isfinite(jump_m) && jump_m > max_jump_m) {
            reject = true;
            block_float_until_fixed = true;
        }
    }
    if (reject) {
        solution.status = libgnss::SolutionStatus::NONE;
        ++rejected_count;
        return true;
    }
    return false;
}

libgnss::FGOProcessor::FGOResult optimizeGtsamPcProblem(
    const libgnss::FGOProcessor::FGOProblem& problem,
    const libgnss::FGOProcessor::FGOConfig& config) {
    using FGOProcessor = libgnss::FGOProcessor;

    struct LocalBiasInfo {
        int local_index = 0;
        double initial_cycles = 0.0;
        double wavelength_m = 0.0;
        double prior_sigma_cycles = 1e6;
    };

    FGOProcessor::FGOResult result;
    result.diagnostics.epochs = problem.epochs.size();
    result.diagnostics.pseudorange_factors = problem.pseudorange_factors.size();
    result.diagnostics.tdcp_factors = problem.tdcp_factors.size();
    result.diagnostics.carrier_phase_factors =
        problem.carrier_phase_factors.size();
    result.diagnostics.double_difference_pseudorange_factors =
        problem.double_difference_pseudorange_factors.size();
    result.diagnostics.double_difference_carrier_factors =
        problem.double_difference_carrier_factors.size();
    result.diagnostics.ambiguity_states = problem.ambiguity_states.size();
    result.diagnostics.double_difference_matched_base_epochs =
        problem.diagnostics.double_difference_matched_base_epochs;
    result.diagnostics.double_difference_interpolated_base_epochs =
        problem.diagnostics.double_difference_interpolated_base_epochs;
    result.diagnostics.double_difference_candidate_pairs =
        problem.diagnostics.double_difference_candidate_pairs;
    result.diagnostics.double_difference_rejected_no_base_epoch =
        problem.diagnostics.double_difference_rejected_no_base_epoch;
    result.diagnostics.double_difference_rejected_no_reference =
        problem.diagnostics.double_difference_rejected_no_reference;

    if (problem.epochs.empty() ||
        problem.double_difference_pseudorange_factors.empty()) {
        return result;
    }

    std::map<std::size_t, std::map<libgnss::SatelliteId, LocalBiasInfo>>
        local_bias_by_epoch;
    std::map<std::size_t, std::set<libgnss::SatelliteId>>
        dd_satellites_by_epoch;
    std::map<std::size_t, std::map<libgnss::SatelliteId, libgnss::SatelliteId>>
        reference_by_epoch_satellite;

    const auto ensure_bias =
        [&local_bias_by_epoch](std::size_t epoch_index,
                               const libgnss::SatelliteId& satellite,
                               double initial_cycles,
                               double wavelength_m,
                               double prior_sigma_cycles) {
            auto& local_biases = local_bias_by_epoch[epoch_index];
            auto existing_it = local_biases.find(satellite);
            if (existing_it != local_biases.end()) {
                existing_it->second.prior_sigma_cycles =
                    std::min(existing_it->second.prior_sigma_cycles,
                             prior_sigma_cycles);
                return;
            }
            const int local_index = static_cast<int>(local_biases.size());
            local_biases.emplace(
                satellite,
                LocalBiasInfo{local_index,
                              initial_cycles,
                              wavelength_m,
                              prior_sigma_cycles});
        };

    for (const auto& factor : problem.double_difference_carrier_factors) {
        double initial_cycles = 0.0;
        double wavelength_m = 0.0;
        if (factor.ambiguity_index < problem.ambiguity_states.size()) {
            const auto& ambiguity =
                problem.ambiguity_states[factor.ambiguity_index];
            wavelength_m = ambiguity.wavelength_m;
            if (ambiguity.wavelength_m > 0.0) {
                initial_cycles =
                    ambiguity.initial_ambiguity_m / ambiguity.wavelength_m;
            }
        }
        ensure_bias(factor.epoch_index,
                    factor.satellite,
                    initial_cycles,
                    wavelength_m,
                    std::max(1e6, config.ambiguity_prior_sigma_m));
        ensure_bias(factor.epoch_index,
                    factor.reference_satellite,
                    0.0,
                    wavelength_m,
                    1.0);
        reference_by_epoch_satellite[factor.epoch_index][factor.satellite] =
            factor.reference_satellite;
        dd_satellites_by_epoch[factor.epoch_index].insert(factor.satellite);
        dd_satellites_by_epoch[factor.epoch_index].insert(
            factor.reference_satellite);
    }
    for (const auto& factor : problem.double_difference_pseudorange_factors) {
        dd_satellites_by_epoch[factor.epoch_index].insert(factor.satellite);
        dd_satellites_by_epoch[factor.epoch_index].insert(
            factor.reference_satellite);
    }

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initials;
    const gtsam::Vector x_prior_sigmas =
        (gtsam::Vector(3) << 100.0, 100.0, 100.0).finished();
    const auto x_prior_noise =
        gtsam::noiseModel::Diagonal::Sigmas(x_prior_sigmas);

    for (std::size_t epoch_index = 0; epoch_index < problem.epochs.size();
         ++epoch_index) {
        const gtsam::Key key_x = gtsam::symbol('x', epoch_index);
        const gtsam::Vector initial_x =
            vector3ToGtsam(problem.epochs[epoch_index].position_ecef);
        initials.insert(key_x, initial_x);
        graph.push_back(std::make_shared<gtsam::PriorFactor<gtsam::Vector>>(
            key_x, initial_x, x_prior_noise));

        const auto local_it = local_bias_by_epoch.find(epoch_index);
        if (local_it != local_bias_by_epoch.end()) {
            gtsam::Vector initial_b =
                gtsam::Vector::Zero(local_it->second.size());
            gtsam::Vector ambiguity_prior_sigmas =
                gtsam::Vector::Ones(local_it->second.size());
            for (const auto& [satellite, bias] : local_it->second) {
                initial_b(bias.local_index) = bias.initial_cycles;
                ambiguity_prior_sigmas(bias.local_index) =
                    std::max(1.0, bias.prior_sigma_cycles);
            }
            const gtsam::Key key_b = gtsam::symbol('b', epoch_index);
            initials.insert(key_b, initial_b);
            graph.push_back(std::make_shared<gtsam::PriorFactor<gtsam::Vector>>(
                key_b,
                initial_b,
                gtsam::noiseModel::Diagonal::Sigmas(ambiguity_prior_sigmas)));
        }
    }

    for (const auto& factor : problem.double_difference_pseudorange_factors) {
        if (factor.epoch_index >= problem.epochs.size()) {
            continue;
        }
        const libgnss::Vector3d seed_position =
            problem.epochs[factor.epoch_index].position_ecef;
        const double geometry = ddGeometry(seed_position,
                                           factor.base_position_ecef,
                                           factor.rover_satellite_position_ecef,
                                           factor.rover_reference_position_ecef,
                                           factor.base_satellite_position_ecef,
                                           factor.base_reference_position_ecef);
        const double residual = factor.observed_dd_pseudorange_m - geometry;
        const libgnss::Vector3d jacobian =
            ddPositionJacobian(seed_position,
                               factor.rover_satellite_position_ecef,
                               factor.rover_reference_position_ecef);
        graph.push_back(std::make_shared<GtsamPseudorangeFactorX>(
            gtsam::symbol('x', factor.epoch_index),
            vector3ToGtsam(jacobian),
            residual,
            vector3ToGtsam(seed_position),
            robustScalarNoise(factor.sigma_m,
                              config.pseudorange_huber_threshold_sigma,
                              config.use_robust_loss)));
    }

    for (const auto& factor : problem.double_difference_carrier_factors) {
        if (factor.epoch_index >= problem.epochs.size() ||
            factor.ambiguity_index >= problem.ambiguity_states.size()) {
            continue;
        }
        const auto local_epoch_it =
            local_bias_by_epoch.find(factor.epoch_index);
        if (local_epoch_it == local_bias_by_epoch.end()) {
            continue;
        }
        const auto local_it =
            local_epoch_it->second.find(factor.satellite);
        if (local_it == local_epoch_it->second.end()) {
            continue;
        }
        const auto reference_bias_it =
            local_epoch_it->second.find(factor.reference_satellite);
        if (reference_bias_it == local_epoch_it->second.end()) {
            continue;
        }
        const auto& ambiguity =
            problem.ambiguity_states[factor.ambiguity_index];
        if (ambiguity.wavelength_m <= 0.0) {
            continue;
        }

        const libgnss::Vector3d seed_position =
            problem.epochs[factor.epoch_index].position_ecef;
        const double geometry = ddGeometry(seed_position,
                                           factor.base_position_ecef,
                                           factor.rover_satellite_position_ecef,
                                           factor.rover_reference_position_ecef,
                                           factor.base_satellite_position_ecef,
                                           factor.base_reference_position_ecef);
        const double residual = factor.observed_dd_carrier_m - geometry;
        const libgnss::Vector3d jacobian =
            ddPositionJacobian(seed_position,
                               factor.rover_satellite_position_ecef,
                               factor.rover_reference_position_ecef);
        graph.push_back(std::make_shared<GtsamCarrierPhaseFactorXB>(
            gtsam::symbol('x', factor.epoch_index),
            gtsam::symbol('b', factor.epoch_index),
            vector3ToGtsam(jacobian),
            residual,
            local_it->second.local_index,
            ambiguity.wavelength_m,
            vector3ToGtsam(seed_position),
            robustScalarNoise(factor.sigma_m,
                              config.carrier_phase_huber_threshold_sigma,
                              config.use_robust_loss)));
    }

    auto start_time = std::chrono::high_resolution_clock::now();
    result.diagnostics.graph_factors = graph.size();
    result.diagnostics.graph_values = initials.size();
    result.diagnostics.initial_cost = graph.error(initials);
    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(std::max(1, config.max_iterations));
    params.setVerbosity("SILENT");
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initials, params);
    const gtsam::Values optimized = optimizer.optimize();
    auto end_time = std::chrono::high_resolution_clock::now();

    result.diagnostics.iterations = optimizer.iterations();
    result.diagnostics.final_cost = graph.error(optimized);
    result.diagnostics.converged = true;
    result.diagnostics.processing_time_ms =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            end_time - start_time)
            .count();

    std::unique_ptr<gtsam::Marginals> marginals;
    if (config.fix_ambiguities) {
        marginals = std::make_unique<gtsam::Marginals>(graph, optimized);
    }
    result.ambiguity_candidate_satellites_by_epoch.resize(problem.epochs.size());
    result.ambiguity_reference_satellites_by_epoch.resize(problem.epochs.size());
    result.ambiguity_estimate_cycles_by_epoch.resize(problem.epochs.size());

    libgnss::Vector3d previous_output_position = libgnss::Vector3d::Zero();
    bool has_previous_output_position = false;
    bool block_float_until_fixed = false;
    for (std::size_t epoch_index = 0; epoch_index < problem.epochs.size();
         ++epoch_index) {
        const gtsam::Vector x =
            optimized.at<gtsam::Vector>(gtsam::symbol('x', epoch_index));
        libgnss::Vector3d position_ecef(x(0), x(1), x(2));
        double ratio = 0.0;
        int fixed_ambiguities = 0;
        bool fixed_epoch = false;

        const auto local_epoch_it =
            local_bias_by_epoch.find(epoch_index);
        gtsam::Vector float_ambiguities;
        std::vector<int> candidate_indices;
        std::vector<libgnss::SatelliteId> satellite_by_local;
        bool has_float_ambiguities = false;
        if (local_epoch_it != local_bias_by_epoch.end()) {
            const gtsam::Key key_b = gtsam::symbol('b', epoch_index);
            float_ambiguities = optimized.at<gtsam::Vector>(key_b);
            if (float_ambiguities.size() ==
                static_cast<int>(local_epoch_it->second.size())) {
                has_float_ambiguities = true;
                candidate_indices.reserve(
                    static_cast<std::size_t>(float_ambiguities.size()));
                satellite_by_local.assign(
                    static_cast<std::size_t>(float_ambiguities.size()),
                    libgnss::SatelliteId());
                for (const auto& [satellite, bias] : local_epoch_it->second) {
                    if (bias.local_index >= 0 &&
                        bias.local_index < float_ambiguities.size()) {
                        satellite_by_local[static_cast<std::size_t>(
                            bias.local_index)] = satellite;
                        if (std::isfinite(float_ambiguities(bias.local_index))) {
                            result.ambiguity_estimate_cycles_by_epoch[epoch_index]
                                [satellite] = float_ambiguities(bias.local_index);
                        }
                    }
                }
                const auto reference_it =
                    reference_by_epoch_satellite.find(epoch_index);
                for (int i = 0; i < float_ambiguities.size(); ++i) {
                    if (std::isfinite(float_ambiguities(i)) &&
                        std::abs(float_ambiguities(i)) > 1e-9) {
                        candidate_indices.push_back(i);
                        const auto& satellite =
                            satellite_by_local[static_cast<std::size_t>(i)];
                        result.ambiguity_candidate_satellites_by_epoch[epoch_index]
                            .insert(satellite);
                        if (reference_it != reference_by_epoch_satellite.end()) {
                            const auto sat_ref_it =
                                reference_it->second.find(satellite);
                            if (sat_ref_it != reference_it->second.end()) {
                                result
                                    .ambiguity_reference_satellites_by_epoch
                                        [epoch_index]
                                    .insert(sat_ref_it->second);
                            }
                        }
                    }
                }
            }
        }
        if (config.fix_ambiguities && marginals &&
            local_epoch_it != local_bias_by_epoch.end() &&
            has_float_ambiguities &&
            static_cast<int>(candidate_indices.size()) >
                config.min_fixed_ambiguities) {
            const gtsam::Key key_b = gtsam::symbol('b', epoch_index);
            const gtsam::Key key_x = gtsam::symbol('x', epoch_index);
                try {
                    gtsam::KeyVector keys;
                    keys.push_back(key_b);
                    keys.push_back(key_x);
                    const gtsam::Matrix covariance =
                        marginals->jointMarginalCovariance(keys).fullMatrix();
                    const int bias_count =
                        static_cast<int>(float_ambiguities.size());
                    if (covariance.rows() == bias_count + 3 &&
                        covariance.cols() == bias_count + 3) {
                        const int ambiguity_count =
                            static_cast<int>(candidate_indices.size());
                        if (ambiguity_count > config.min_fixed_ambiguities) {
                            Eigen::VectorXd ambiguity_float =
                                Eigen::VectorXd::Zero(ambiguity_count);
                            Eigen::MatrixXd ambiguity_covariance =
                                Eigen::MatrixXd::Zero(ambiguity_count, ambiguity_count);
                            for (int row = 0; row < ambiguity_count; ++row) {
                                const int source_row = candidate_indices[row];
                                ambiguity_float(row) = float_ambiguities(source_row);
                                for (int col = 0; col < ambiguity_count; ++col) {
                                    const int source_col = candidate_indices[col];
                                    ambiguity_covariance(row, col) =
                                        covariance(source_row, source_col);
                                }
                            }
                            ambiguity_covariance =
                                0.5 * (ambiguity_covariance +
                                       ambiguity_covariance.transpose());
                            for (int i = 0; i < ambiguity_count; ++i) {
                                const double diagonal =
                                    std::abs(ambiguity_covariance(i, i));
                                ambiguity_covariance(i, i) +=
                                    std::max(1e-12, diagonal * 1e-9);
                            }

                            Eigen::VectorXd fixed_ambiguities_vector;
                            ++result.diagnostics.lambda_ambiguity_attempts;
                            const bool solved =
                                libgnss::lambdaSearch(ambiguity_float,
                                                       ambiguity_covariance,
                                                       fixed_ambiguities_vector,
                                                       ratio);
                            const bool accepted =
                                solved && std::isfinite(ratio) &&
                                ratio > config.lambda_ratio_threshold;
                            if (config.collect_lambda_debug) {
                                result.lambda_debug_entries.reserve(
                                    result.lambda_debug_entries.size() +
                                    static_cast<std::size_t>(ambiguity_count *
                                                             ambiguity_count));
                                for (int row = 0; row < ambiguity_count; ++row) {
                                    const int source_row = candidate_indices[row];
                                    for (int col = 0; col < ambiguity_count; ++col) {
                                        const int source_col = candidate_indices[col];
                                        libgnss::FGOProcessor::LambdaDebugEntry entry;
                                        entry.epoch_index = epoch_index;
                                        entry.time = problem.epochs[epoch_index].time;
                                        entry.solved = solved;
                                        entry.fixed_epoch = accepted;
                                        entry.ratio = ratio;
                                        entry.candidate_count = ambiguity_count;
                                        entry.row = row;
                                        entry.col = col;
                                        entry.local_index = source_row;
                                        entry.other_local_index = source_col;
                                        entry.satellite =
                                            satellite_by_local[static_cast<std::size_t>(
                                                source_row)];
                                        entry.other_satellite =
                                            satellite_by_local[static_cast<std::size_t>(
                                                source_col)];
                                        entry.ambiguity_float =
                                            ambiguity_float(row);
                                        entry.fixed_ambiguity =
                                            solved &&
                                                    fixed_ambiguities_vector.size() >
                                                        row
                                                ? fixed_ambiguities_vector(row)
                                                : std::numeric_limits<double>::quiet_NaN();
                                        entry.covariance =
                                            ambiguity_covariance(row, col);
                                        entry.position_covariance_x =
                                            covariance(bias_count, source_row);
                                        entry.position_covariance_y =
                                            covariance(bias_count + 1, source_row);
                                        entry.position_covariance_z =
                                            covariance(bias_count + 2, source_row);
                                        result.lambda_debug_entries.push_back(entry);
                                    }
                                }
                            }
                            if (solved) {
                                result.diagnostics.lambda_ambiguity_fix_solved = true;
                            }
                            if (solved && std::isfinite(ratio)) {
                                result.diagnostics.lambda_ambiguity_ratio =
                                    std::max(result.diagnostics.lambda_ambiguity_ratio,
                                             ratio);
                            }
                            result.diagnostics.lambda_ambiguity_candidates +=
                                static_cast<std::size_t>(ambiguity_count);
                            result.diagnostics.ambiguity_fix_candidates +=
                                static_cast<std::size_t>(ambiguity_count);
                            if (accepted) {
                                const Eigen::VectorXd ambiguity_delta =
                                    ambiguity_float - fixed_ambiguities_vector;
                                const Eigen::MatrixXd position_ambiguity_covariance =
                                    [&]() {
                                        Eigen::MatrixXd covariance_block =
                                            Eigen::MatrixXd::Zero(3, ambiguity_count);
                                        for (int col = 0; col < ambiguity_count; ++col) {
                                            covariance_block.col(col) =
                                                covariance.block(bias_count,
                                                                 candidate_indices[col],
                                                                 3,
                                                                 1);
                                        }
                                        return covariance_block;
                                    }();
                                const Eigen::Vector3d position_update =
                                    position_ambiguity_covariance *
                                    ambiguity_covariance.ldlt().solve(
                                        ambiguity_delta);
                                position_ecef -= position_update;
                                fixed_epoch = true;
                                fixed_ambiguities = ambiguity_count;
                                result.diagnostics.fixed_solution = true;
                                result.diagnostics.lambda_ambiguity_fix_used = true;
                                result.diagnostics.lambda_ambiguity_used_candidates +=
                                    static_cast<std::size_t>(ambiguity_count);
                                result.diagnostics.fixed_ambiguities +=
                                    static_cast<std::size_t>(ambiguity_count);
                            }
                        }
                    }
                } catch (const std::exception&) {
                    // Keep the float solution when covariance extraction fails.
                }
        }

        libgnss::PositionSolution solution;
        solution.time = problem.epochs[epoch_index].time;
        solution.status =
            fixed_epoch ? libgnss::SolutionStatus::FIXED
                        : libgnss::SolutionStatus::FLOAT;
        solution.position_ecef = position_ecef;
        solution.num_frequencies = 1;
        solution.ratio = ratio;
        solution.num_fixed_ambiguities = fixed_ambiguities;
        solution.processing_time_ms =
            result.diagnostics.processing_time_ms /
            static_cast<double>(std::max<std::size_t>(1, problem.epochs.size()));
        solution.iterations = result.diagnostics.iterations;
        solution.position_covariance = libgnss::Matrix3d::Identity() * 9999.0;

        double lat = 0.0;
        double lon = 0.0;
        double height = 0.0;
        libgnss::ecef2geodetic(solution.position_ecef, lat, lon, height);
        solution.position_geodetic = libgnss::GeodeticCoord(lat, lon, height);

        const auto dd_satellite_it = dd_satellites_by_epoch.find(epoch_index);
        if (dd_satellite_it != dd_satellites_by_epoch.end()) {
            solution.num_satellites =
                static_cast<int>(dd_satellite_it->second.size());
            solution.satellites_used.assign(dd_satellite_it->second.begin(),
                                            dd_satellite_it->second.end());
        }
        applyFloatPositionJumpGate(
            solution,
            previous_output_position,
            has_previous_output_position,
            config.max_float_position_jump_m,
            block_float_until_fixed,
            result.diagnostics.float_rejected_position_jump);
        applyFloatSeedPositionDivergenceGate(
            solution,
            problem,
            epoch_index,
            config.max_float_seed_position_divergence_m,
            result.diagnostics.float_rejected_seed_position_divergence);
        result.solution.addSolution(solution);
        previous_output_position = solution.position_ecef;
        has_previous_output_position = true;
    }

    return result;
}
#endif

void writeSummaryJson(const Options& options,
                      const libgnss::FGOProcessor::FGOResult& result,
                      const libgnss::Solution::SolutionStatistics& stats,
                      std::size_t input_epochs,
                      std::size_t seed_matched_epochs,
                      std::size_t seed_interpolated_epochs,
                      double availability_rate_percent,
                      double fix_rate_percent,
                      double float_rate_percent) {
    if (options.summary_json_path.empty()) {
        return;
    }

    std::ofstream output(options.summary_json_path);
    if (!output.is_open()) {
        throw std::runtime_error(
            "failed to open summary JSON for writing: " +
            options.summary_json_path);
    }

    const auto& diagnostics = result.diagnostics;
    output << std::setprecision(std::numeric_limits<double>::max_digits10);
    output << "{\n";
    output << "  \"obs\": \"" << jsonEscape(options.obs_path) << "\",\n";
    output << "  \"base\": \"" << jsonEscape(options.base_path) << "\",\n";
    output << "  \"nav\": \"" << jsonEscape(options.nav_path) << "\",\n";
    output << "  \"out\": \"" << jsonEscape(options.out_path) << "\",\n";
    output << "  \"preset\": \"" << jsonEscape(options.preset) << "\",\n";
    output << "  \"backend\": \"" << jsonEscape(options.backend) << "\",\n";
    output << "  \"skip_epochs\": " << options.skip_epochs << ",\n";
    output << "  \"max_epochs\": " << options.max_epochs << ",\n";
    output << "  \"max_iterations\": " << options.max_iterations << ",\n";
    output << "  \"relative_cost_convergence_threshold\": "
           << options.relative_cost_convergence_threshold << ",\n";
    output << "  \"absolute_cost_convergence_threshold\": "
           << options.absolute_cost_convergence_threshold << ",\n";
    output << "  \"seed_pos\": \"" << jsonEscape(options.seed_pos_path) << "\",\n";
    output << "  \"epoch_debug_csv\": \""
           << jsonEscape(options.epoch_debug_csv_path) << "\",\n";
    output << "  \"factor_debug_csv\": \""
           << jsonEscape(options.factor_debug_csv_path) << "\",\n";
    output << "  \"sd_factor_debug_csv\": \""
           << jsonEscape(options.sd_factor_debug_csv_path) << "\",\n";
    output << "  \"lambda_debug_csv\": \""
           << jsonEscape(options.lambda_debug_csv_path) << "\",\n";
    output << "  \"cost_trace_csv\": \""
           << jsonEscape(options.cost_trace_csv_path) << "\",\n";
    output << "  \"seed_match_tolerance_s\": "
           << options.seed_match_tolerance_s << ",\n";
    output << "  \"seed_interpolation_max_gap_s\": "
           << options.seed_interpolation_max_gap_s << ",\n";
    output << "  \"use_spp_seed\": " << jsonBool(!options.no_spp_seed) << ",\n";
    output << "  \"use_pseudorange_factors\": "
           << jsonBool(!options.no_pseudorange_factors) << ",\n";
    output << "  \"use_single_difference_doppler_factors\": "
           << jsonBool(options.use_single_difference_doppler_factors) << ",\n";
    output << "  \"use_single_difference_tdcp_factors\": "
           << jsonBool(options.use_single_difference_tdcp_factors) << ",\n";
    output << "  \"use_velocity_states\": "
           << jsonBool(options.use_velocity_states) << ",\n";
    output << "  \"use_velocity_motion_factors\": "
           << jsonBool(options.use_velocity_motion_factors) << ",\n";
    output << "  \"use_ambiguity_between_factors\": "
           << jsonBool(options.use_ambiguity_between_factors) << ",\n";
    output << "  \"linearize_double_difference_factors_at_seed\": "
           << jsonBool(options.linearize_double_difference_factors_at_seed)
           << ",\n";
    output << "  \"use_epoch_lambda_fixed_output\": "
           << jsonBool(options.use_epoch_lambda_fixed_output) << ",\n";
    output << "  \"pseudorange_sigma_m\": "
           << options.pseudorange_sigma_m << ",\n";
    output << "  \"pseudorange_elevation_sigma_power\": "
           << options.pseudorange_elevation_sigma_power << ",\n";
    output << "  \"pseudorange_huber_threshold_sigma\": "
           << options.pseudorange_huber_threshold_sigma << ",\n";
    output << "  \"carrier_phase_huber_threshold_sigma\": "
           << options.carrier_phase_huber_threshold_sigma << ",\n";
    output << "  \"tdcp_huber_threshold_sigma\": "
           << options.tdcp_huber_threshold_sigma << ",\n";
    output << "  \"lambda_ratio_threshold\": "
           << options.lambda_ratio_threshold << ",\n";
    output << "  \"position_prior_sigma_m\": "
           << options.position_prior_sigma_m << ",\n";
    output << "  \"clock_prior_sigma_m\": "
           << options.clock_prior_sigma_m << ",\n";
    output << "  \"motion_sigma_m\": " << options.motion_sigma_m << ",\n";
    output << "  \"clock_motion_sigma_m\": "
           << options.clock_motion_sigma_m << ",\n";
    output << "  \"velocity_prior_sigma_mps\": "
           << options.velocity_prior_sigma_mps << ",\n";
    output << "  \"velocity_motion_sigma_m\": "
           << options.velocity_motion_sigma_m << ",\n";
    output << "  \"ambiguity_between_sigma_cycles\": "
           << options.ambiguity_between_sigma_cycles << ",\n";
    output << "  \"use_position_motion_factors\": "
           << jsonBool(!options.no_motion_factors &&
                       !options.clock_only_motion_factors) << ",\n";
    output << "  \"use_clock_motion_factors\": "
           << jsonBool(!options.no_motion_factors) << ",\n";
    output << "  \"dd_ambiguity_per_epoch\": "
           << jsonBool(options.dd_ambiguity_per_epoch) << ",\n";
    output << "  \"use_ambiguity_priors\": "
           << jsonBool(!options.no_ambiguity_priors) << ",\n";
    output << "  \"reject_rover_carrier_lli\": "
           << jsonBool(options.reject_rover_carrier_lli) << ",\n";
    output << "  \"min_snr_dbhz\": " << options.min_snr_dbhz << ",\n";
    output << "  \"min_satellites_per_epoch\": "
           << options.min_satellites_per_epoch << ",\n";
    output << "  \"min_output_dd_carrier_factors_per_epoch\": "
           << options.min_output_dd_carrier_factors_per_epoch << ",\n";
    output << "  \"max_float_seed_position_divergence_m\": "
           << options.max_float_seed_position_divergence_m << ",\n";
    output << "  \"max_float_position_jump_m\": "
           << options.max_float_position_jump_m << ",\n";
    output << "  \"insert_fixed_interval_gaps\": "
           << jsonBool(options.insert_fixed_interval_gaps) << ",\n";
    output << "  \"double_difference_reference_min_snr_dbhz\": "
           << options.double_difference_reference_min_snr_dbhz << ",\n";
    output << "  \"double_difference_base_min_snr_dbhz\": "
           << options.double_difference_base_min_snr_dbhz << ",\n";
    output << "  \"exclude_glonass_qzss_sbas\": "
           << jsonBool(options.exclude_glonass_qzss_sbas) << ",\n";
    output << "  \"use_ionosphere_model\": "
           << jsonBool(options.use_ionosphere_model) << ",\n";
    output << "  \"use_troposphere_model\": "
           << jsonBool(options.use_troposphere_model) << ",\n";
    output << "  \"debug_problem_only\": "
           << jsonBool(options.debug_problem_only) << ",\n";
    output << "  \"seed_matched_epochs\": " << seed_matched_epochs << ",\n";
    output << "  \"seed_interpolated_epochs\": "
           << seed_interpolated_epochs << ",\n";
    output << "  \"input_epochs\": " << input_epochs << ",\n";
    output << "  \"optimized_epochs\": " << diagnostics.epochs << ",\n";
    output << "  \"valid_solutions\": " << stats.valid_solutions << ",\n";
    output << "  \"fixed_solutions\": " << stats.fixed_solutions << ",\n";
    output << "  \"float_solutions\": " << stats.float_solutions << ",\n";
    output << "  \"availability_rate_percent\": " << availability_rate_percent << ",\n";
    output << "  \"fix_rate_percent\": " << fix_rate_percent << ",\n";
    output << "  \"float_rate_percent\": " << float_rate_percent << ",\n";
    output << "  \"pseudorange_factors\": " << diagnostics.pseudorange_factors << ",\n";
    output << "  \"tdcp_factors\": " << diagnostics.tdcp_factors << ",\n";
    output << "  \"single_difference_doppler_factors\": "
           << diagnostics.single_difference_doppler_factors << ",\n";
    output << "  \"single_difference_tdcp_factors\": "
           << diagnostics.single_difference_tdcp_factors << ",\n";
    output << "  \"carrier_phase_factors\": " << diagnostics.carrier_phase_factors << ",\n";
    output << "  \"double_difference_pseudorange_factors\": "
           << diagnostics.double_difference_pseudorange_factors << ",\n";
    output << "  \"double_difference_carrier_factors\": "
           << diagnostics.double_difference_carrier_factors << ",\n";
    output << "  \"ambiguity_states\": " << diagnostics.ambiguity_states << ",\n";
    output << "  \"ambiguity_fix_candidates\": " << diagnostics.ambiguity_fix_candidates << ",\n";
    output << "  \"lambda_ambiguity_candidates\": "
           << diagnostics.lambda_ambiguity_candidates << ",\n";
    output << "  \"lambda_ambiguity_used_candidates\": "
           << diagnostics.lambda_ambiguity_used_candidates << ",\n";
    output << "  \"lambda_ambiguity_attempts\": "
           << diagnostics.lambda_ambiguity_attempts << ",\n";
    output << "  \"lambda_ambiguity_fix_solved\": "
           << jsonBool(diagnostics.lambda_ambiguity_fix_solved) << ",\n";
    output << "  \"lambda_ambiguity_fix_used\": "
           << jsonBool(diagnostics.lambda_ambiguity_fix_used) << ",\n";
    output << "  \"partial_lambda_ambiguity_fix_used\": "
           << jsonBool(diagnostics.partial_lambda_ambiguity_fix_used) << ",\n";
    output << "  \"lambda_ambiguity_ratio\": "
           << diagnostics.lambda_ambiguity_ratio << ",\n";
    output << "  \"fixed_ambiguities\": " << diagnostics.fixed_ambiguities << ",\n";
    output << "  \"fixed_solution\": " << jsonBool(diagnostics.fixed_solution) << ",\n";
    output << "  \"tdcp_candidate_pairs\": " << diagnostics.tdcp_candidate_pairs << ",\n";
    output << "  \"tdcp_rejected_gap\": " << diagnostics.tdcp_rejected_gap << ",\n";
    output << "  \"tdcp_rejected_missing_previous\": "
           << diagnostics.tdcp_rejected_missing_previous << ",\n";
    output << "  \"tdcp_rejected_loss_of_lock\": "
           << diagnostics.tdcp_rejected_loss_of_lock << ",\n";
    output << "  \"tdcp_rejected_code_phase_jump\": "
           << diagnostics.tdcp_rejected_code_phase_jump << ",\n";
    output << "  \"double_difference_matched_base_epochs\": "
           << diagnostics.double_difference_matched_base_epochs << ",\n";
    output << "  \"double_difference_interpolated_base_epochs\": "
           << diagnostics.double_difference_interpolated_base_epochs << ",\n";
    output << "  \"double_difference_candidate_pairs\": "
           << diagnostics.double_difference_candidate_pairs << ",\n";
    output << "  \"double_difference_rejected_no_base_epoch\": "
           << diagnostics.double_difference_rejected_no_base_epoch << ",\n";
    output << "  \"double_difference_rejected_no_reference\": "
           << diagnostics.double_difference_rejected_no_reference << ",\n";
    output << "  \"motion_factors\": " << diagnostics.motion_factors << ",\n";
    output << "  \"ambiguity_between_factors\": "
           << diagnostics.ambiguity_between_factors << ",\n";
    output << "  \"robust_pseudorange_factors\": "
           << diagnostics.robust_pseudorange_factors << ",\n";
    output << "  \"robust_carrier_phase_factors\": "
           << diagnostics.robust_carrier_phase_factors << ",\n";
    output << "  \"robust_double_difference_pseudorange_factors\": "
           << diagnostics.robust_double_difference_pseudorange_factors << ",\n";
    output << "  \"robust_double_difference_carrier_factors\": "
           << diagnostics.robust_double_difference_carrier_factors << ",\n";
    output << "  \"robust_tdcp_factors\": " << diagnostics.robust_tdcp_factors << ",\n";
    output << "  \"graph_factors\": " << diagnostics.graph_factors << ",\n";
    output << "  \"graph_values\": " << diagnostics.graph_values << ",\n";
    output << "  \"float_rejected_seed_position_divergence\": "
           << diagnostics.float_rejected_seed_position_divergence << ",\n";
    output << "  \"float_rejected_position_jump\": "
           << diagnostics.float_rejected_position_jump << ",\n";
    output << "  \"iterations\": " << diagnostics.iterations << ",\n";
    output << "  \"converged\": " << jsonBool(diagnostics.converged) << ",\n";
    output << "  \"initial_cost\": " << diagnostics.initial_cost << ",\n";
    output << "  \"final_cost\": " << diagnostics.final_cost << ",\n";
    output << "  \"processing_time_ms\": " << diagnostics.processing_time_ms << ",\n";
    output << "  \"epoch_lambda_processing_time_ms\": "
           << diagnostics.epoch_lambda_processing_time_ms << ",\n";
    output << "  \"epoch_lambda_setup_time_ms\": "
           << diagnostics.epoch_lambda_setup_time_ms << ",\n";
    output << "  \"epoch_lambda_factorization_time_ms\": "
           << diagnostics.epoch_lambda_factorization_time_ms << ",\n";
    output << "  \"epoch_lambda_covariance_solve_time_ms\": "
           << diagnostics.epoch_lambda_covariance_solve_time_ms << ",\n";
    output << "  \"epoch_lambda_search_time_ms\": "
           << diagnostics.epoch_lambda_search_time_ms << ",\n";
    output << "  \"epoch_lambda_fixed_output_time_ms\": "
           << diagnostics.epoch_lambda_fixed_output_time_ms << ",\n";
    output << "  \"epoch_lambda_debug_record_time_ms\": "
           << diagnostics.epoch_lambda_debug_record_time_ms << ",\n";
    output << "  \"postprocessing_time_ms\": "
           << diagnostics.postprocessing_time_ms << ",\n";
    output << "  \"total_processing_time_ms\": "
           << diagnostics.total_processing_time_ms << ",\n";
    output << "  \"last_update_norm_m\": " << diagnostics.last_update_norm_m << ",\n";
    output << "  \"residual_rms_m\": " << diagnostics.residual_rms_m << ",\n";
    output << "  \"tdcp_residual_rms_m\": " << diagnostics.tdcp_residual_rms_m << ",\n";
    output << "  \"single_difference_doppler_residual_rms_mps\": "
           << diagnostics.single_difference_doppler_residual_rms_mps << ",\n";
    output << "  \"single_difference_tdcp_residual_rms_m\": "
           << diagnostics.single_difference_tdcp_residual_rms_m << ",\n";
    output << "  \"carrier_phase_residual_rms_m\": "
           << diagnostics.carrier_phase_residual_rms_m << ",\n";
    output << "  \"double_difference_pseudorange_residual_rms_m\": "
           << diagnostics.double_difference_pseudorange_residual_rms_m << ",\n";
    output << "  \"double_difference_carrier_residual_rms_m\": "
           << diagnostics.double_difference_carrier_residual_rms_m << ",\n";
    output << "  \"fixed_ambiguity_residual_rms_cycles\": "
           << diagnostics.fixed_ambiguity_residual_rms_cycles << "\n";
    output << "}\n";
}

libgnss::FGOProcessor::FGOResult makeProblemOnlyResult(
    const libgnss::FGOProcessor::FGOProblem& problem,
    const libgnss::FGOProcessor::FGOConfig& config) {
    libgnss::FGOProcessor::FGOResult result;
    auto& diagnostics = result.diagnostics;
    diagnostics.epochs = problem.epochs.size();
    diagnostics.pseudorange_factors = problem.pseudorange_factors.size();
    diagnostics.tdcp_factors = problem.tdcp_factors.size();
    diagnostics.single_difference_doppler_factors =
        problem.single_difference_doppler_factors.size();
    diagnostics.single_difference_tdcp_factors =
        problem.single_difference_tdcp_factors.size();
    diagnostics.carrier_phase_factors = problem.carrier_phase_factors.size();
    diagnostics.double_difference_pseudorange_factors =
        problem.double_difference_pseudorange_factors.size();
    diagnostics.double_difference_carrier_factors =
        problem.double_difference_carrier_factors.size();
    diagnostics.ambiguity_between_factors =
        problem.ambiguity_between_factors.size();
    diagnostics.ambiguity_states = problem.ambiguity_states.size();
    diagnostics.tdcp_candidate_pairs =
        problem.diagnostics.tdcp_candidate_pairs;
    diagnostics.tdcp_rejected_gap = problem.diagnostics.tdcp_rejected_gap;
    diagnostics.tdcp_rejected_missing_previous =
        problem.diagnostics.tdcp_rejected_missing_previous;
    diagnostics.tdcp_rejected_loss_of_lock =
        problem.diagnostics.tdcp_rejected_loss_of_lock;
    diagnostics.tdcp_rejected_code_phase_jump =
        problem.diagnostics.tdcp_rejected_code_phase_jump;
    diagnostics.double_difference_matched_base_epochs =
        problem.diagnostics.double_difference_matched_base_epochs;
    diagnostics.double_difference_interpolated_base_epochs =
        problem.diagnostics.double_difference_interpolated_base_epochs;
    diagnostics.double_difference_candidate_pairs =
        problem.diagnostics.double_difference_candidate_pairs;
    diagnostics.double_difference_rejected_no_base_epoch =
        problem.diagnostics.double_difference_rejected_no_base_epoch;
    diagnostics.double_difference_rejected_no_reference =
        problem.diagnostics.double_difference_rejected_no_reference;
    diagnostics.motion_factors =
        (config.use_motion_factors || config.use_velocity_motion_factors) &&
                problem.epochs.size() >= 2
            ? problem.epochs.size() - 1
            : 0;

    const std::size_t position_prior_factors =
        config.position_prior_sigma_m > 0.0 ? problem.epochs.size() : 0;
    const std::size_t clock_prior_factors =
        config.clock_prior_sigma_m > 0.0 ? problem.epochs.size() : 0;
    const std::size_t velocity_prior_factors =
        config.use_velocity_states && config.velocity_prior_sigma_mps > 0.0
            ? problem.epochs.size()
            : 0;
    const std::size_t ambiguity_prior_factors =
        config.use_ambiguity_priors && config.ambiguity_prior_sigma_m > 0.0
            ? problem.ambiguity_states.size()
            : 0;
    diagnostics.graph_factors =
        diagnostics.pseudorange_factors +
        diagnostics.tdcp_factors +
        diagnostics.single_difference_doppler_factors +
        diagnostics.single_difference_tdcp_factors +
        diagnostics.carrier_phase_factors +
        diagnostics.double_difference_pseudorange_factors +
        diagnostics.double_difference_carrier_factors +
        diagnostics.ambiguity_between_factors +
        diagnostics.motion_factors +
        position_prior_factors +
        clock_prior_factors +
        velocity_prior_factors +
        ambiguity_prior_factors;
    diagnostics.graph_values =
        problem.epochs.size() * (config.use_velocity_states ? 3 : 2) +
        problem.ambiguity_states.size();

    std::map<std::size_t, std::vector<const libgnss::FGOProcessor::PseudorangeFactor*>>
        pseudorange_factors_by_epoch;
    for (const auto& factor : problem.pseudorange_factors) {
        pseudorange_factors_by_epoch[factor.epoch_index].push_back(&factor);
    }
    std::map<std::size_t, std::set<libgnss::SatelliteId>>
        dd_satellites_by_epoch;
    for (const auto& factor : problem.double_difference_pseudorange_factors) {
        dd_satellites_by_epoch[factor.epoch_index].insert(factor.satellite);
        dd_satellites_by_epoch[factor.epoch_index].insert(
            factor.reference_satellite);
    }
    for (const auto& factor : problem.double_difference_carrier_factors) {
        dd_satellites_by_epoch[factor.epoch_index].insert(factor.satellite);
        dd_satellites_by_epoch[factor.epoch_index].insert(
            factor.reference_satellite);
    }
    std::map<std::size_t, std::size_t> dd_carrier_factors_by_epoch;
    for (const auto& factor : problem.double_difference_carrier_factors) {
        ++dd_carrier_factors_by_epoch[factor.epoch_index];
    }

    result.solution.solutions.reserve(problem.epochs.size());
    for (std::size_t epoch_index = 0; epoch_index < problem.epochs.size();
         ++epoch_index) {
        const auto& epoch = problem.epochs[epoch_index];
        libgnss::PositionSolution solution;
        solution.time = epoch.time;
        solution.status = libgnss::SolutionStatus::SPP;
        solution.position_ecef = epoch.position_ecef;
        solution.receiver_clock_bias =
            epoch.receiver_clock_bias_m / libgnss::constants::SPEED_OF_LIGHT;
        solution.position_covariance =
            libgnss::Matrix3d::Identity() * 9999.0;
        solution.num_frequencies = 1;

        double lat = 0.0;
        double lon = 0.0;
        double height = 0.0;
        libgnss::ecef2geodetic(solution.position_ecef, lat, lon, height);
        solution.position_geodetic = libgnss::GeodeticCoord(lat, lon, height);

        const auto pseudorange_it =
            pseudorange_factors_by_epoch.find(epoch_index);
        bool has_epoch_pseudorange_solution = false;
        if (pseudorange_it != pseudorange_factors_by_epoch.end()) {
            has_epoch_pseudorange_solution = true;
            solution.num_satellites =
                static_cast<int>(pseudorange_it->second.size());
            solution.satellites_used.reserve(pseudorange_it->second.size());
            solution.satellite_elevations.reserve(pseudorange_it->second.size());
            solution.satellite_residuals.reserve(pseudorange_it->second.size());
            for (const auto* factor : pseudorange_it->second) {
                solution.satellites_used.push_back(factor->satellite);
                solution.satellite_elevations.push_back(factor->elevation_rad);
                solution.satellite_residuals.push_back(
                    std::numeric_limits<double>::quiet_NaN());
            }
        } else {
            const auto dd_it = dd_satellites_by_epoch.find(epoch_index);
            if (dd_it != dd_satellites_by_epoch.end()) {
                solution.num_satellites = static_cast<int>(dd_it->second.size());
                solution.satellites_used.assign(dd_it->second.begin(),
                                                dd_it->second.end());
            }
        }
        if (!has_epoch_pseudorange_solution &&
            config.min_output_double_difference_carrier_factors_per_epoch > 0) {
            const auto dd_carrier_count_it =
                dd_carrier_factors_by_epoch.find(epoch_index);
            const std::size_t dd_carrier_count =
                dd_carrier_count_it == dd_carrier_factors_by_epoch.end()
                    ? 0
                    : dd_carrier_count_it->second;
            if (dd_carrier_count <
                static_cast<std::size_t>(
                    config
                        .min_output_double_difference_carrier_factors_per_epoch)) {
                solution.status = libgnss::SolutionStatus::NONE;
            }
        }
        result.solution.addSolution(solution);
    }

    return result;
}

std::string joinSatellites(const std::set<libgnss::SatelliteId>& satellites) {
    std::ostringstream joined;
    bool first = true;
    for (const auto& satellite : satellites) {
        if (!first) {
            joined << ';';
        }
        joined << satellite.toString();
        first = false;
    }
    return joined.str();
}

bool writeEpochDebugCsv(
    const std::string& path,
    const libgnss::FGOProcessor::FGOProblem& problem,
    const libgnss::FGOProcessor::FGOResult& result) {
    if (path.empty()) {
        return true;
    }

    std::map<std::size_t, std::set<std::size_t>> ambiguity_indices_by_epoch;
    std::map<std::size_t, std::set<libgnss::SatelliteId>> dd_satellites_by_epoch;
    std::map<std::size_t, std::set<libgnss::SatelliteId>> reference_satellites_by_epoch;

    for (const auto& factor : problem.double_difference_carrier_factors) {
        ambiguity_indices_by_epoch[factor.epoch_index].insert(
            factor.ambiguity_index);
        dd_satellites_by_epoch[factor.epoch_index].insert(factor.satellite);
        dd_satellites_by_epoch[factor.epoch_index].insert(
            factor.reference_satellite);
        reference_satellites_by_epoch[factor.epoch_index].insert(
            factor.reference_satellite);
    }
    for (const auto& factor : problem.double_difference_pseudorange_factors) {
        dd_satellites_by_epoch[factor.epoch_index].insert(factor.satellite);
        dd_satellites_by_epoch[factor.epoch_index].insert(
            factor.reference_satellite);
        reference_satellites_by_epoch[factor.epoch_index].insert(
            factor.reference_satellite);
    }

    std::ofstream output(path);
    if (!output.is_open()) {
        return false;
    }

    output << "epoch_index,gps_week,gps_tow,status,ratio,num_fixed_ambiguities,"
              "ambiguity_candidates,dd_satellites,reference_satellites,"
              "position_x_m,position_y_m,position_z_m,"
              "seed_position_x_m,seed_position_y_m,seed_position_z_m,"
              "seed_position_divergence_m,"
              "velocity_x_mps,velocity_y_mps,velocity_z_mps\n";
    output << std::fixed << std::setprecision(6);
    const auto& solution = result.solution;
    const std::size_t rows =
        std::min(problem.epochs.size(), solution.solutions.size());
    const bool has_result_candidates =
        result.ambiguity_candidate_satellites_by_epoch.size() >= rows &&
        result.ambiguity_reference_satellites_by_epoch.size() >= rows;
    for (std::size_t epoch_index = 0; epoch_index < rows; ++epoch_index) {
        const auto& epoch = problem.epochs[epoch_index];
        const auto& sol = solution.solutions[epoch_index];
        const auto ambiguity_it = ambiguity_indices_by_epoch.find(epoch_index);
        std::size_t ambiguity_candidates =
            ambiguity_it == ambiguity_indices_by_epoch.end()
                ? 0
                : ambiguity_it->second.size();
        const auto satellites_it = dd_satellites_by_epoch.find(epoch_index);
        const auto references_it =
            reference_satellites_by_epoch.find(epoch_index);
        const std::set<libgnss::SatelliteId> empty_satellites;
        std::set<libgnss::SatelliteId> result_satellites;
        const std::set<libgnss::SatelliteId>* output_satellites =
            satellites_it == dd_satellites_by_epoch.end()
                ? &empty_satellites
                : &satellites_it->second;
        const std::set<libgnss::SatelliteId>* output_references =
            references_it == reference_satellites_by_epoch.end()
                ? &empty_satellites
                : &references_it->second;
        if (has_result_candidates) {
            const auto& candidates =
                result.ambiguity_candidate_satellites_by_epoch[epoch_index];
            const auto& references =
                result.ambiguity_reference_satellites_by_epoch[epoch_index];
            ambiguity_candidates = candidates.size();
            result_satellites = candidates;
            result_satellites.insert(references.begin(), references.end());
            output_satellites = &result_satellites;
            output_references = &references;
        }

        const double nan = std::numeric_limits<double>::quiet_NaN();
        libgnss::Vector3d velocity_mps =
            libgnss::Vector3d::Constant(nan);
        if (result.epoch_velocities_ecef_mps.size() > epoch_index) {
            velocity_mps = result.epoch_velocities_ecef_mps[epoch_index];
        }
        auto velocityBetween = [&](std::size_t begin_index,
                                   std::size_t end_index) -> libgnss::Vector3d {
            const double dt =
                solution.solutions[end_index].time -
                solution.solutions[begin_index].time;
            if (dt <= 0.0) {
                return libgnss::Vector3d::Constant(nan);
            }
            return (solution.solutions[end_index].position_ecef -
                    solution.solutions[begin_index].position_ecef) /
                   dt;
        };
        if (rows >= 2 && !velocity_mps.allFinite()) {
            if (epoch_index > 0 && epoch_index + 1 < rows) {
                velocity_mps = velocityBetween(epoch_index - 1, epoch_index + 1);
            } else if (epoch_index + 1 < rows) {
                velocity_mps = velocityBetween(epoch_index, epoch_index + 1);
            } else {
                velocity_mps = velocityBetween(epoch_index - 1, epoch_index);
            }
        }
        double seed_position_divergence_m =
            std::numeric_limits<double>::quiet_NaN();
        if (sol.position_ecef.allFinite() &&
            epoch.position_ecef.allFinite() &&
            epoch.position_ecef.norm() > 1e6) {
            seed_position_divergence_m =
                (sol.position_ecef - epoch.position_ecef).norm();
        }

        output << epoch_index << ','
               << epoch.time.week << ','
               << epoch.time.tow << ','
               << static_cast<int>(sol.status) << ','
               << sol.ratio << ','
               << sol.num_fixed_ambiguities << ','
               << ambiguity_candidates << ','
               << joinSatellites(*output_satellites)
               << ','
               << joinSatellites(*output_references)
               << ','
               << sol.position_ecef(0) << ','
               << sol.position_ecef(1) << ','
               << sol.position_ecef(2) << ','
               << epoch.position_ecef(0) << ','
               << epoch.position_ecef(1) << ','
               << epoch.position_ecef(2) << ','
               << seed_position_divergence_m << ','
               << velocity_mps(0) << ','
               << velocity_mps(1) << ','
               << velocity_mps(2)
               << '\n';
    }
    return true;
}

void writeCsvDouble(std::ostream& output, double value);

bool writeLambdaDebugCsv(
    const std::string& path,
    const std::vector<libgnss::FGOProcessor::LambdaDebugEntry>& entries) {
    if (path.empty()) {
        return true;
    }

    std::ofstream output(path);
    if (!output.is_open()) {
        return false;
    }

    output << "epoch_index,gps_week,gps_tow,solved,fixed_epoch,ratio,"
              "candidate_count,row,col,satellite,other_satellite,local_index,"
              "other_local_index,ambiguity_float,fixed_ambiguity,covariance,"
              "position_covariance_x,position_covariance_y,position_covariance_z\n";
    output << std::fixed << std::setprecision(15);
    for (const auto& entry : entries) {
        output << entry.epoch_index << ','
               << entry.time.week << ','
               << entry.time.tow << ','
               << (entry.solved ? 1 : 0) << ','
               << (entry.fixed_epoch ? 1 : 0) << ',';
        writeCsvDouble(output, entry.ratio);
        output << ','
               << entry.candidate_count << ','
               << entry.row << ','
               << entry.col << ','
               << entry.satellite.toString() << ','
               << entry.other_satellite.toString() << ','
               << entry.local_index << ','
               << entry.other_local_index << ',';
        writeCsvDouble(output, entry.ambiguity_float);
        output << ',';
        writeCsvDouble(output, entry.fixed_ambiguity);
        output << ',';
        writeCsvDouble(output, entry.covariance);
        output << ',';
        writeCsvDouble(output, entry.position_covariance_x);
        output << ',';
        writeCsvDouble(output, entry.position_covariance_y);
        output << ',';
        writeCsvDouble(output, entry.position_covariance_z);
        output << '\n';
    }
    return true;
}

bool writeCostTraceCsv(
    const std::string& path,
    const std::vector<libgnss::FGOProcessor::CostTraceEntry>& entries) {
    if (path.empty()) {
        return true;
    }

    std::ofstream output(path);
    if (!output.is_open()) {
        return false;
    }

    output << "phase,local_iteration,global_iteration,cost,"
              "absolute_decrease,relative_decrease,update_norm,converged\n";
    output << std::fixed << std::setprecision(15);
    for (const auto& entry : entries) {
        output << entry.phase << ','
               << entry.local_iteration << ','
               << entry.global_iteration << ',';
        writeCsvDouble(output, entry.cost);
        output << ',';
        writeCsvDouble(output, entry.absolute_decrease);
        output << ',';
        writeCsvDouble(output, entry.relative_decrease);
        output << ',';
        writeCsvDouble(output, entry.update_norm);
        output << ','
               << (entry.converged ? 1 : 0)
               << '\n';
    }
    return true;
}

struct FactorDebugKey {
    std::size_t epoch_index = 0;
    libgnss::SatelliteId satellite;
    libgnss::SatelliteId reference_satellite;
    libgnss::SignalType signal = libgnss::SignalType::GPS_L1CA;

    bool operator<(const FactorDebugKey& other) const {
        return std::tie(epoch_index,
                        satellite,
                        reference_satellite,
                        signal) <
               std::tie(other.epoch_index,
                        other.satellite,
                        other.reference_satellite,
                        other.signal);
    }
};

void writeCsvDouble(std::ostream& output, double value) {
    if (std::isfinite(value)) {
        output << value;
    } else {
        output << "NaN";
    }
}

double ddDebugTerm(double rover_satellite,
                   double base_satellite,
                   double rover_reference,
                   double base_reference) {
    return (rover_satellite - base_satellite) -
           (rover_reference - base_reference);
}

bool isPrimaryFgoDebugSignal(libgnss::SignalType signal) {
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

bool isHealthyForFgoDebug(const libgnss::Observation& observation,
                          const libgnss::Ephemeris& eph) {
    int sv_health = static_cast<int>(eph.health);
    if (observation.satellite.system == libgnss::GNSSSystem::QZSS) {
        sv_health &= 0xFE;
    }
    return sv_health == 0;
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
                    return eph.tgd_secondary *
                           libgnss::constants::SPEED_OF_LIGHT;
                default:
                    return 0.0;
            }
        default:
            return 0.0;
    }
}

libgnss::Vector3d earthRotationCorrected(
    const libgnss::Vector3d& satellite_position,
    const libgnss::Vector3d& receiver_position) {
    const double signal_travel_time =
        (satellite_position - receiver_position).norm() /
        libgnss::constants::SPEED_OF_LIGHT;
    const double angle = libgnss::constants::OMEGA_E * signal_travel_time;

    Eigen::Matrix3d earth_rotation;
    earth_rotation << std::cos(angle),  std::sin(angle), 0.0,
                     -std::sin(angle),  std::cos(angle), 0.0,
                      0.0,              0.0,             1.0;
    return earth_rotation * satellite_position;
}

struct SdReferenceKey {
    std::size_t epoch_index = 0;
    libgnss::GNSSSystem system = libgnss::GNSSSystem::UNKNOWN;
    libgnss::SignalType signal = libgnss::SignalType::GPS_L1CA;

    bool operator<(const SdReferenceKey& other) const {
        return std::tie(epoch_index, system, signal) <
               std::tie(other.epoch_index, other.system, other.signal);
    }
};

struct SdDefaultReferenceKey {
    libgnss::GNSSSystem system = libgnss::GNSSSystem::UNKNOWN;
    libgnss::SignalType signal = libgnss::SignalType::GPS_L1CA;

    bool operator<(const SdDefaultReferenceKey& other) const {
        return std::tie(system, signal) <
               std::tie(other.system, other.signal);
    }
};

using SdObservationKey =
    std::pair<libgnss::SatelliteId, libgnss::SignalType>;

struct SdObservationModel {
    libgnss::SatelliteId satellite;
    libgnss::SignalType signal = libgnss::SignalType::GPS_L1CA;
    double snr_dbhz = 0.0;
    double elevation_rad = 0.0;
    double raw_doppler_hz = std::numeric_limits<double>::quiet_NaN();
    double raw_carrier_cycles = std::numeric_limits<double>::quiet_NaN();
    double wavelength_m = std::numeric_limits<double>::quiet_NaN();
    double doppler_residual_mps = std::numeric_limits<double>::quiet_NaN();
    double carrier_residual_m = std::numeric_limits<double>::quiet_NaN();
    double sigma_d_mps = std::numeric_limits<double>::quiet_NaN();
    double sigma_l_m = std::numeric_limits<double>::quiet_NaN();
    bool has_doppler_residual = false;
    bool has_carrier_residual = false;
    libgnss::Vector3d los = libgnss::Vector3d::Zero();
};

bool prepareSdObservationModel(const libgnss::ObservationData& epoch,
                               const libgnss::Observation& observation,
                               const libgnss::NavigationData& nav,
                               const libgnss::Vector3d& receiver_position,
                               const Options& options,
                               SdObservationModel& model) {
    if (receiver_position.norm() <= 1e6 ||
        !isPrimaryFgoDebugSignal(observation.signal) ||
        !observation.valid ||
        !observation.has_pseudorange ||
        observation.pseudorange <= 0.0 ||
        observation.snr < options.min_snr_dbhz) {
        return false;
    }

    libgnss::Vector3d satellite_position;
    libgnss::Vector3d satellite_velocity;
    double satellite_clock_bias = 0.0;
    double satellite_clock_drift = 0.0;

    libgnss::GNSSTime transmit_time =
        epoch.time -
        observation.pseudorange / libgnss::constants::SPEED_OF_LIGHT;
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

    const libgnss::Ephemeris* eph =
        nav.getEphemeris(observation.satellite, transmit_time);
    if (!eph || !isHealthyForFgoDebug(observation, *eph)) {
        return false;
    }

    const libgnss::Vector3d corrected_satellite_position =
        earthRotationCorrected(satellite_position, receiver_position);
    const auto geometry =
        nav.calculateGeometry(receiver_position, corrected_satellite_position);
    const double min_elevation_rad = options.min_elevation_deg * M_PI / 180.0;
    if (geometry.elevation < min_elevation_rad) {
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
    if (options.use_ionosphere_model && nav.ionosphere_model.valid) {
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
            const double scale =
                libgnss::constants::GPS_L1_FREQ / frequency_hz;
            ionosphere_delay *= scale * scale;
        }
    }

    double troposphere_delay = 0.0;
    if (options.use_troposphere_model) {
        troposphere_delay =
            libgnss::models::tropDelaySaastamoinen(receiver_position,
                                                   geometry.elevation);
    }

    double wavelength = libgnss::signalWavelengthMeters(observation);
    if (wavelength <= 0.0) {
        wavelength = libgnss::signalWavelengthMeters(observation.signal, eph);
    }
    if (wavelength <= 0.0) {
        return false;
    }

    const double sin_el = std::sin(geometry.elevation);
    if (sin_el <= 0.0) {
        return false;
    }
    const double sqrt_sin_el = std::sqrt(sin_el);
    const double satellite_clock_m =
        satellite_clock_bias * libgnss::constants::SPEED_OF_LIGHT;
    const double corrected_range =
        (corrected_satellite_position - receiver_position).norm();
    const libgnss::Vector3d corrected_delta =
        corrected_satellite_position - receiver_position;
    if (corrected_range <= 0.0) {
        return false;
    }

    model = SdObservationModel{};
    model.satellite = observation.satellite;
    model.signal = observation.signal;
    model.snr_dbhz = observation.snr;
    model.elevation_rad = geometry.elevation;
    model.wavelength_m = wavelength;
    model.sigma_d_mps = 0.2 / sqrt_sin_el;
    model.sigma_l_m =
        options.double_difference_carrier_sigma_m / sqrt_sin_el;
    model.los = -corrected_delta / corrected_range;

    if (observation.has_doppler) {
        const libgnss::Vector3d doppler_delta =
            satellite_position - receiver_position;
        const double doppler_range = doppler_delta.norm();
        if (doppler_range > 0.0) {
            const libgnss::Vector3d ex = doppler_delta / doppler_range;
            const double sagnac_rate =
                libgnss::constants::OMEGA_E /
                libgnss::constants::SPEED_OF_LIGHT *
                (satellite_velocity(1) * receiver_position(0) -
                 satellite_velocity(0) * receiver_position(1));
            const double modeled_range_rate =
                satellite_velocity.dot(ex) + sagnac_rate;
            const double satellite_clock_drift_mps =
                satellite_clock_drift *
                libgnss::constants::SPEED_OF_LIGHT;
            const double measured_range_rate =
                -observation.doppler * wavelength;
            model.raw_doppler_hz = observation.doppler;
            model.doppler_residual_mps =
                measured_range_rate -
                (modeled_range_rate - satellite_clock_drift_mps);
            model.has_doppler_residual =
                std::isfinite(model.doppler_residual_mps);
        }
    }

    const bool has_carrier =
        observation.has_carrier_phase && observation.carrier_phase != 0.0;
    const bool loss_of_lock =
        observation.loss_of_lock || ((observation.lli & 0x01U) != 0U);
    if (has_carrier && !loss_of_lock) {
        const double corrected_carrier =
            observation.carrier_phase * wavelength +
            satellite_clock_m -
            troposphere_delay +
            ionosphere_delay;
        model.raw_carrier_cycles = observation.carrier_phase;
        model.carrier_residual_m = corrected_carrier - corrected_range;
        model.has_carrier_residual =
            std::isfinite(model.carrier_residual_m);
    }

    return model.has_doppler_residual || model.has_carrier_residual;
}

bool writeSdFactorDebugCsv(
    const std::string& path,
    const std::vector<libgnss::ObservationData>& rover_epochs,
    const libgnss::NavigationData& nav,
    const libgnss::FGOProcessor::FGOProblem& problem,
    const Options& options) {
    if (path.empty()) {
        return true;
    }

    std::map<SdReferenceKey, libgnss::SatelliteId> reference_by_group;
    std::map<SdDefaultReferenceKey,
             std::map<libgnss::SatelliteId, std::size_t>>
        default_reference_counts;
    auto remember_reference =
        [&reference_by_group](std::size_t epoch_index,
                              const libgnss::SatelliteId& satellite,
                              const libgnss::SatelliteId& reference,
                              libgnss::SignalType signal) {
            reference_by_group[SdReferenceKey{
                epoch_index,
                satellite.system,
                signal,
            }] = reference;
        };
    for (const auto& factor : problem.double_difference_pseudorange_factors) {
        remember_reference(factor.epoch_index,
                           factor.satellite,
                           factor.reference_satellite,
                           factor.signal);
        ++default_reference_counts[SdDefaultReferenceKey{
            factor.satellite.system,
            factor.signal,
        }][factor.reference_satellite];
    }
    for (const auto& factor : problem.double_difference_carrier_factors) {
        remember_reference(factor.epoch_index,
                           factor.satellite,
                           factor.reference_satellite,
                           factor.signal);
        ++default_reference_counts[SdDefaultReferenceKey{
            factor.satellite.system,
            factor.signal,
        }][factor.reference_satellite];
    }
    std::map<SdDefaultReferenceKey, libgnss::SatelliteId>
        default_reference_by_group;
    for (const auto& [group_key, counts] : default_reference_counts) {
        const auto best_it =
            std::max_element(counts.begin(),
                             counts.end(),
                             [](const auto& lhs, const auto& rhs) {
                                 return lhs.second < rhs.second;
                             });
        if (best_it != counts.end()) {
            default_reference_by_group[group_key] = best_it->first;
        }
    }

    std::ofstream output(path);
    if (!output.is_open()) {
        return false;
    }
    output << "epoch_index,gps_week,gps_tow,satellite,reference,signal,"
              "elevation_deg,reference_elevation_deg,sigma_dsd_mps,"
              "sigma_tdcp_m,res_dsd_mps,res_lsd_m,tdcp_sd_m,"
              "raw_d_rover_hz,raw_l_rover_cycles,wavelength_m,"
              "valid_doppler_sd,valid_tdcp_sd,los_x,los_y,los_z\n";
    output << std::fixed << std::setprecision(12);

    std::map<SdObservationKey, double> previous_res_lsd_by_key;
    const std::size_t rows = std::min(problem.epochs.size(), rover_epochs.size());
    for (std::size_t epoch_index = 0; epoch_index < rows; ++epoch_index) {
        const auto& problem_epoch = problem.epochs[epoch_index];
        const auto& rover_epoch = rover_epochs[epoch_index];
        std::map<SdObservationKey, SdObservationModel> models_by_key;
        for (const auto& observation : rover_epoch.observations) {
            SdObservationModel model;
            if (prepareSdObservationModel(rover_epoch,
                                          observation,
                                          nav,
                                          problem_epoch.position_ecef,
                                          options,
                                          model)) {
                models_by_key[{model.satellite, model.signal}] = model;
            }
        }

        std::map<SdObservationKey, double> current_res_lsd_by_key;
        for (const auto& [key, model] : models_by_key) {
            const auto reference_it = reference_by_group.find(SdReferenceKey{
                epoch_index,
                model.satellite.system,
                model.signal,
            });
            libgnss::SatelliteId reference_satellite;
            if (reference_it != reference_by_group.end()) {
                reference_satellite = reference_it->second;
            } else {
                const auto default_reference_it =
                    default_reference_by_group.find(SdDefaultReferenceKey{
                        model.satellite.system,
                        model.signal,
                    });
                if (default_reference_it ==
                        default_reference_by_group.end() ||
                    !(model.satellite == default_reference_it->second)) {
                    continue;
                }
                reference_satellite = default_reference_it->second;
            }
            const auto reference_model_it =
                models_by_key.find({reference_satellite, model.signal});
            if (reference_model_it == models_by_key.end()) {
                continue;
            }
            const SdObservationModel& reference_model =
                reference_model_it->second;

            double res_dsd =
                std::numeric_limits<double>::quiet_NaN();
            const bool valid_doppler =
                model.has_doppler_residual &&
                reference_model.has_doppler_residual;
            if (valid_doppler) {
                res_dsd =
                    model.doppler_residual_mps -
                    reference_model.doppler_residual_mps;
            }

            double res_lsd =
                std::numeric_limits<double>::quiet_NaN();
            double tdcp_sd =
                std::numeric_limits<double>::quiet_NaN();
            bool valid_tdcp = false;
            if (model.has_carrier_residual &&
                reference_model.has_carrier_residual) {
                res_lsd =
                    model.carrier_residual_m -
                    reference_model.carrier_residual_m;
                current_res_lsd_by_key[key] = res_lsd;
                const auto previous_it =
                    previous_res_lsd_by_key.find(key);
                if (previous_it != previous_res_lsd_by_key.end()) {
                    tdcp_sd = res_lsd - previous_it->second;
                    valid_tdcp =
                        std::isfinite(tdcp_sd) && tdcp_sd != 0.0;
                }
            }

            if (!valid_doppler &&
                !std::isfinite(res_lsd) &&
                !valid_tdcp) {
                continue;
            }

            const libgnss::Vector3d dd_los =
                model.los - reference_model.los;
            output << epoch_index << ','
                   << problem_epoch.time.week << ','
                   << problem_epoch.time.tow << ','
                   << model.satellite.toString() << ','
                   << reference_satellite.toString() << ','
                   << static_cast<int>(model.signal) << ','
                   << model.elevation_rad * 180.0 / M_PI << ','
                   << reference_model.elevation_rad * 180.0 / M_PI << ',';
            writeCsvDouble(output, model.sigma_d_mps);
            output << ',';
            writeCsvDouble(output, model.sigma_l_m);
            output << ',';
            writeCsvDouble(output, res_dsd);
            output << ',';
            writeCsvDouble(output, res_lsd);
            output << ',';
            writeCsvDouble(output, tdcp_sd);
            output << ',';
            writeCsvDouble(output, model.raw_doppler_hz);
            output << ',';
            writeCsvDouble(output, model.raw_carrier_cycles);
            output << ',';
            writeCsvDouble(output, model.wavelength_m);
            output << ','
                   << (valid_doppler ? 1 : 0) << ','
                   << (valid_tdcp ? 1 : 0) << ',';
            writeCsvDouble(output, dd_los(0));
            output << ',';
            writeCsvDouble(output, dd_los(1));
            output << ',';
            writeCsvDouble(output, dd_los(2));
            output << '\n';
        }

        previous_res_lsd_by_key = std::move(current_res_lsd_by_key);
    }

    return true;
}

bool writeFactorDebugCsv(
    const std::string& path,
    const libgnss::FGOProcessor::FGOProblem& problem,
    const libgnss::FGOProcessor::FGOResult& result) {
    if (path.empty()) {
        return true;
    }

    std::map<FactorDebugKey,
             const libgnss::FGOProcessor::DoubleDifferenceCarrierFactor*>
        carrier_factor_by_key;
    for (const auto& factor : problem.double_difference_carrier_factors) {
        carrier_factor_by_key[FactorDebugKey{
            factor.epoch_index,
            factor.satellite,
            factor.reference_satellite,
            factor.signal,
        }] = &factor;
    }

    std::ofstream output(path);
    if (!output.is_open()) {
        return false;
    }
    if (problem.double_difference_pseudorange_factors.empty() &&
        !problem.pseudorange_factors.empty()) {
        output << "epoch_index,gps_week,gps_tow,satellite,clock_group,"
                  "elevation_deg,sigma_p_m,corrected_pseudorange_m,"
                  "seed_range_m,initial_res_pc_m,final_residual_m,"
                  "los_x,los_y,los_z\n";
        output << std::fixed << std::setprecision(12);

        for (const auto& factor : problem.pseudorange_factors) {
            if (factor.epoch_index >= problem.epochs.size()) {
                continue;
            }
            const auto& epoch = problem.epochs[factor.epoch_index];
            const libgnss::Vector3d delta =
                factor.satellite_position_ecef - epoch.position_ecef;
            const double range = delta.norm();
            if (range <= 0.0) {
                continue;
            }
            const libgnss::Vector3d los = delta / range;
            const double initial_residual =
                factor.corrected_pseudorange_m - range -
                epoch.receiver_clock_bias_m;
            double final_residual =
                std::numeric_limits<double>::quiet_NaN();
            if (result.solution.solutions.size() > factor.epoch_index) {
                const auto& solution =
                    result.solution.solutions[factor.epoch_index];
                for (std::size_t i = 0;
                     i < solution.satellites_used.size() &&
                     i < solution.satellite_residuals.size();
                     ++i) {
                    if (solution.satellites_used[i] == factor.satellite) {
                        final_residual = solution.satellite_residuals[i];
                        break;
                    }
                }
            }

            output << factor.epoch_index << ','
                   << epoch.time.week << ','
                   << epoch.time.tow << ','
                   << factor.satellite.toString() << ','
                   << static_cast<int>(factor.clock_group) << ','
                   << factor.elevation_rad * 180.0 / M_PI << ',';
            writeCsvDouble(output, factor.sigma_m);
            output << ',';
            writeCsvDouble(output, factor.corrected_pseudorange_m);
            output << ',';
            writeCsvDouble(output, range);
            output << ',';
            writeCsvDouble(output, initial_residual);
            output << ',';
            writeCsvDouble(output, final_residual);
            output << ',';
            writeCsvDouble(output, -los(0));
            output << ',';
            writeCsvDouble(output, -los(1));
            output << ',';
            writeCsvDouble(output, -los(2));
            output << '\n';
        }
        return true;
    }

    output << "epoch_index,gps_week,gps_tow,satellite,reference,signal,"
              "elevation_deg,sigma_pdd_m,sigma_ldd_m,res_pdd,res_ldd,ambiguity_initial,"
              "ambiguity_estimate,geometry_m,rover_satellite_range_m,"
              "base_satellite_range_m,rover_reference_range_m,"
              "base_reference_range_m,observed_dd_pseudorange_m,"
              "observed_dd_carrier_m,dd_raw_pseudorange_m,"
              "dd_raw_carrier_m,dd_code_correction_m,"
              "dd_carrier_correction_m,dd_satellite_clock_m,"
              "dd_ionosphere_delay_m,dd_troposphere_delay_m,"
              "dd_group_delay_m\n";
    output << std::fixed << std::setprecision(12);

    for (const auto& factor : problem.double_difference_pseudorange_factors) {
        if (factor.epoch_index >= problem.epochs.size()) {
            continue;
        }
        const auto& epoch = problem.epochs[factor.epoch_index];
        const double geometry = ddGeometry(epoch.position_ecef,
                                           factor.base_position_ecef,
                                           factor.rover_satellite_position_ecef,
                                           factor.rover_reference_position_ecef,
                                           factor.base_satellite_position_ecef,
                                           factor.base_reference_position_ecef);
        const FactorDebugKey key{
            factor.epoch_index,
            factor.satellite,
            factor.reference_satellite,
            factor.signal,
        };
        const double pseudorange_residual =
            factor.observed_dd_pseudorange_m - geometry;
        double carrier_residual = std::numeric_limits<double>::quiet_NaN();
        double initial_cycles = std::numeric_limits<double>::quiet_NaN();
        double estimated_cycles = std::numeric_limits<double>::quiet_NaN();
        double observed_dd_carrier =
            std::numeric_limits<double>::quiet_NaN();
        double carrier_sigma = std::numeric_limits<double>::quiet_NaN();
        double raw_carrier_dd = std::numeric_limits<double>::quiet_NaN();
        double carrier_correction_dd =
            std::numeric_limits<double>::quiet_NaN();

        const double rover_satellite_range =
            factor.rover_satellite_model.geometric_range_m;
        const double base_satellite_range =
            factor.base_satellite_model.geometric_range_m;
        const double rover_reference_range =
            factor.rover_reference_model.geometric_range_m;
        const double base_reference_range =
            factor.base_reference_model.geometric_range_m;
        const double raw_pseudorange_dd =
            ddDebugTerm(factor.rover_satellite_model.raw_pseudorange_m,
                        factor.base_satellite_model.raw_pseudorange_m,
                        factor.rover_reference_model.raw_pseudorange_m,
                        factor.base_reference_model.raw_pseudorange_m);
        const double code_correction_dd =
            factor.observed_dd_pseudorange_m - raw_pseudorange_dd;
        const double satellite_clock_dd =
            ddDebugTerm(factor.rover_satellite_model.satellite_clock_m,
                        factor.base_satellite_model.satellite_clock_m,
                        factor.rover_reference_model.satellite_clock_m,
                        factor.base_reference_model.satellite_clock_m);
        const double ionosphere_dd =
            ddDebugTerm(factor.rover_satellite_model.ionosphere_delay_m,
                        factor.base_satellite_model.ionosphere_delay_m,
                        factor.rover_reference_model.ionosphere_delay_m,
                        factor.base_reference_model.ionosphere_delay_m);
        const double troposphere_dd =
            ddDebugTerm(factor.rover_satellite_model.troposphere_delay_m,
                        factor.base_satellite_model.troposphere_delay_m,
                        factor.rover_reference_model.troposphere_delay_m,
                        factor.base_reference_model.troposphere_delay_m);
        const double group_delay_dd =
            ddDebugTerm(factor.rover_satellite_model.group_delay_m,
                        factor.base_satellite_model.group_delay_m,
                        factor.rover_reference_model.group_delay_m,
                        factor.base_reference_model.group_delay_m);

        const auto carrier_it = carrier_factor_by_key.find(key);
        if (carrier_it != carrier_factor_by_key.end()) {
            const auto& carrier_factor = *carrier_it->second;
            if (carrier_factor.ambiguity_index < problem.ambiguity_states.size()) {
                const auto& ambiguity =
                    problem.ambiguity_states[carrier_factor.ambiguity_index];
                observed_dd_carrier = carrier_factor.observed_dd_carrier_m;
                carrier_sigma = carrier_factor.sigma_m;
                carrier_residual =
                    carrier_factor.observed_dd_carrier_m - geometry;
                raw_carrier_dd =
                    ddDebugTerm(
                        carrier_factor.rover_satellite_model.raw_carrier_m,
                        carrier_factor.base_satellite_model.raw_carrier_m,
                        carrier_factor.rover_reference_model.raw_carrier_m,
                        carrier_factor.base_reference_model.raw_carrier_m);
                carrier_correction_dd =
                    carrier_factor.observed_dd_carrier_m - raw_carrier_dd;
                initial_cycles =
                    ambiguity.wavelength_m > 0.0
                        ? ambiguity.initial_ambiguity_m / ambiguity.wavelength_m
                        : std::numeric_limits<double>::quiet_NaN();
                if (result.ambiguity_estimate_cycles_by_epoch.size() >
                    factor.epoch_index) {
                    const auto estimate_it =
                        result
                            .ambiguity_estimate_cycles_by_epoch[factor.epoch_index]
                            .find(factor.satellite);
                    if (estimate_it !=
                        result
                            .ambiguity_estimate_cycles_by_epoch[factor.epoch_index]
                            .end()) {
                        estimated_cycles = estimate_it->second;
                    }
                }
            }
        }

        output << factor.epoch_index << ','
               << epoch.time.week << ','
               << epoch.time.tow << ','
               << factor.satellite.toString() << ','
               << factor.reference_satellite.toString() << ','
               << static_cast<int>(factor.signal) << ','
               << factor.elevation_rad * 180.0 / M_PI << ',';
        writeCsvDouble(output, factor.sigma_m);
        output << ',';
        writeCsvDouble(output, carrier_sigma);
        output << ',';
        writeCsvDouble(output, pseudorange_residual);
        output << ',';
        writeCsvDouble(output, carrier_residual);
        output << ',';
        writeCsvDouble(output, initial_cycles);
        output << ',';
        writeCsvDouble(output, estimated_cycles);
        output << ',';
        writeCsvDouble(output, geometry);
        output << ',';
        writeCsvDouble(output, rover_satellite_range);
        output << ',';
        writeCsvDouble(output, base_satellite_range);
        output << ',';
        writeCsvDouble(output, rover_reference_range);
        output << ',';
        writeCsvDouble(output, base_reference_range);
        output << ',';
        writeCsvDouble(output, factor.observed_dd_pseudorange_m);
        output << ',';
        writeCsvDouble(output, observed_dd_carrier);
        output << ',';
        writeCsvDouble(output, raw_pseudorange_dd);
        output << ',';
        writeCsvDouble(output, raw_carrier_dd);
        output << ',';
        writeCsvDouble(output, code_correction_dd);
        output << ',';
        writeCsvDouble(output, carrier_correction_dd);
        output << ',';
        writeCsvDouble(output, satellite_clock_dd);
        output << ',';
        writeCsvDouble(output, ionosphere_dd);
        output << ',';
        writeCsvDouble(output, troposphere_dd);
        output << ',';
        writeCsvDouble(output, group_delay_dd);
        output << '\n';
    }

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

        const std::vector<SeedPosition> seed_positions =
            options.seed_pos_path.empty()
                ? std::vector<SeedPosition>()
                : readSeedPositions(options.seed_pos_path);
        std::size_t seed_cursor = 0;
        std::size_t seed_matched_epochs = 0;
        std::size_t seed_interpolated_epochs = 0;

        std::vector<libgnss::ObservationData> epochs;
        const double fixed_interval_s =
            obs_header.interval > 0.0 ? obs_header.interval : 1.0;
        const double fixed_gap_tolerance_s =
            std::max(0.01, 0.5 * fixed_interval_s);
        bool have_last_output_time = false;
        libgnss::GNSSTime last_output_time;

        auto append_rover_epoch =
            [&](libgnss::ObservationData epoch_to_append) -> bool {
            if (options.max_epochs > 0 &&
                static_cast<int>(epochs.size()) >= options.max_epochs) {
                return false;
            }
            libgnss::Vector3d seed_position = libgnss::Vector3d::Zero();
            bool seed_interpolated = false;
            if (!seed_positions.empty() &&
                findSeedPosition(seed_positions,
                                 epoch_to_append.time,
                                 options.seed_match_tolerance_s,
                                 options.seed_interpolation_max_gap_s,
                                 seed_cursor,
                                 seed_position,
                                 seed_interpolated)) {
                epoch_to_append.receiver_position = seed_position;
                epoch_to_append.receiver_clock_bias = 0.0;
                ++seed_matched_epochs;
                if (seed_interpolated) {
                    ++seed_interpolated_epochs;
                }
            } else if (obs_header.approximate_position.norm() > 0.0) {
                epoch_to_append.receiver_position = obs_header.approximate_position;
            }
            applySystemFilter(epoch_to_append, options);
            epochs.push_back(std::move(epoch_to_append));
            return true;
        };

        libgnss::ObservationData epoch;
        int rover_epoch_index = 0;
        while (obs_reader.readObservationEpoch(epoch)) {
            if (rover_epoch_index++ < options.skip_epochs) {
                continue;
            }

            if (options.insert_fixed_interval_gaps && have_last_output_time) {
                libgnss::GNSSTime expected_time =
                    last_output_time + fixed_interval_s;
                while (epoch.time - expected_time > fixed_gap_tolerance_s) {
                    libgnss::ObservationData empty_epoch(expected_time);
                    if (!append_rover_epoch(empty_epoch)) {
                        break;
                    }
                    last_output_time = expected_time;
                    expected_time = last_output_time + fixed_interval_s;
                    if (options.max_epochs > 0 &&
                        static_cast<int>(epochs.size()) >= options.max_epochs) {
                        break;
                    }
                }
                if (options.max_epochs > 0 &&
                    static_cast<int>(epochs.size()) >= options.max_epochs) {
                    break;
                }
            }

            if (!append_rover_epoch(epoch)) {
                break;
            }
            last_output_time = epoch.time;
            have_last_output_time = true;
        }

        std::vector<libgnss::ObservationData> base_epochs;
        libgnss::Vector3d base_position = libgnss::Vector3d::Zero();
        if (!options.base_path.empty() && !options.no_double_difference_factors) {
            libgnss::io::RINEXReader base_reader;
            if (!base_reader.open(options.base_path)) {
                std::cerr << "Error: failed to open base observation file: "
                          << options.base_path << "\n";
                return 1;
            }
            libgnss::io::RINEXReader::RINEXHeader base_header;
            if (!base_reader.readHeader(base_header)) {
                std::cerr << "Error: failed to read base observation header: "
                          << options.base_path << "\n";
                return 1;
            }
            if (base_header.approximate_position.norm() <= 1e6) {
                std::cerr << "Error: base approximate position is unavailable in: "
                          << options.base_path << "\n";
                return 1;
            }
            base_position = base_header.approximate_position;

            libgnss::ObservationData base_epoch;
            while (base_reader.readObservationEpoch(base_epoch)) {
                base_epoch.receiver_position = base_position;
                applySystemFilter(base_epoch, options);
                base_epochs.push_back(base_epoch);
            }
            if (base_epochs.empty()) {
                std::cerr << "Error: base observation file has no epochs: "
                          << options.base_path << "\n";
                return 1;
            }
        }

        libgnss::FGOProcessor::FGOConfig config;
        config.max_iterations = options.max_iterations;
        config.relative_cost_convergence_threshold =
            options.relative_cost_convergence_threshold;
        config.absolute_cost_convergence_threshold =
            options.absolute_cost_convergence_threshold;
        config.pseudorange_sigma_m = options.pseudorange_sigma_m;
        config.pseudorange_elevation_sigma_power =
            options.pseudorange_elevation_sigma_power;
        config.motion_sigma_m = options.motion_sigma_m;
        config.clock_motion_sigma_m = options.clock_motion_sigma_m;
        config.velocity_prior_sigma_mps = options.velocity_prior_sigma_mps;
        config.velocity_motion_sigma_m = options.velocity_motion_sigma_m;
        config.ambiguity_between_sigma_cycles =
            options.ambiguity_between_sigma_cycles;
        config.position_prior_sigma_m = options.position_prior_sigma_m;
        config.clock_prior_sigma_m = options.clock_prior_sigma_m;
        config.tdcp_sigma_m = options.tdcp_sigma_m;
        config.carrier_phase_sigma_m = options.carrier_phase_sigma_m;
        config.double_difference_pseudorange_sigma_m =
            options.double_difference_pseudorange_sigma_m;
        config.double_difference_carrier_sigma_m =
            options.double_difference_carrier_sigma_m;
        config.ambiguity_prior_sigma_m = options.ambiguity_prior_sigma_m;
        config.pseudorange_huber_threshold_sigma =
            options.pseudorange_huber_threshold_sigma;
        config.carrier_phase_huber_threshold_sigma =
            options.carrier_phase_huber_threshold_sigma;
        config.tdcp_huber_threshold_sigma = options.tdcp_huber_threshold_sigma;
        config.fixed_ambiguity_sigma_m = options.fixed_ambiguity_sigma_m;
        config.ambiguity_fix_max_fractional_cycles =
            options.ambiguity_fix_threshold_cycles;
        config.lambda_ratio_threshold = options.lambda_ratio_threshold;
        config.use_epoch_lambda_fixed_output =
            options.use_epoch_lambda_fixed_output;
        config.min_fixed_ambiguities = options.min_fixed_ambiguities;
        config.max_lambda_ambiguities = options.max_lambda_ambiguities;
        config.max_tdcp_gap_s = options.max_tdcp_gap_s;
        config.base_epoch_match_tolerance_s = options.base_match_tolerance_s;
        config.base_interpolation_max_gap_s =
            options.base_interpolation_max_gap_s;
        config.tdcp_code_phase_jump_threshold_m = options.tdcp_slip_threshold_m;
        config.min_elevation_deg = options.min_elevation_deg;
        config.min_snr_dbhz = options.min_snr_dbhz;
        config.min_satellites_per_epoch = options.min_satellites_per_epoch;
        config.min_output_double_difference_carrier_factors_per_epoch =
            options.min_output_dd_carrier_factors_per_epoch;
        config.max_float_seed_position_divergence_m =
            options.max_float_seed_position_divergence_m;
        config.max_float_position_jump_m = options.max_float_position_jump_m;
        config.double_difference_reference_min_snr_dbhz =
            options.double_difference_reference_min_snr_dbhz;
        config.double_difference_base_min_snr_dbhz =
            options.double_difference_base_min_snr_dbhz;
        config.use_spp_seed = !options.no_spp_seed;
        config.use_pseudorange_factors = !options.no_pseudorange_factors;
        config.use_motion_factors = !options.no_motion_factors;
        config.use_position_motion_factors =
            config.use_motion_factors && !options.clock_only_motion_factors;
        config.use_clock_motion_factors = config.use_motion_factors;
        config.use_tdcp_factors = !options.no_tdcp_factors;
        config.use_double_difference_factors =
            !options.base_path.empty() && !options.no_double_difference_factors;
        config.use_single_difference_doppler_factors =
            options.use_single_difference_doppler_factors;
        config.use_single_difference_tdcp_factors =
            options.use_single_difference_tdcp_factors;
        config.use_velocity_states = options.use_velocity_states;
        config.use_velocity_motion_factors =
            options.use_velocity_motion_factors && options.use_velocity_states;
        config.use_ambiguity_between_factors =
            options.use_ambiguity_between_factors;
        config.linearize_double_difference_factors_at_seed =
            options.linearize_double_difference_factors_at_seed;
        config.reset_double_difference_ambiguities_each_epoch =
            options.dd_ambiguity_per_epoch;
        config.use_carrier_phase_factors =
            options.use_carrier_phase_factors ||
            (options.fix_ambiguities && !config.use_double_difference_factors);
        config.fix_ambiguities = options.fix_ambiguities;
        config.prefer_double_difference_ambiguity_fixing =
            !options.fix_all_ambiguities;
        config.use_lambda_ambiguity_fix = options.use_lambda_ambiguity_fix;
        config.use_partial_lambda_ambiguity_fix =
            options.use_partial_lambda_ambiguity_fix;
        config.use_robust_loss = options.use_robust_loss;
        config.use_ambiguity_priors = !options.no_ambiguity_priors;
        config.reject_rover_carrier_loss_of_lock =
            options.reject_rover_carrier_lli;
        config.reject_tdcp_code_phase_jump = !options.no_tdcp_slip_reject;
        config.use_ionosphere_model = options.use_ionosphere_model;
        config.use_troposphere_model = options.use_troposphere_model;
        config.collect_lambda_debug = !options.lambda_debug_csv_path.empty();

        const libgnss::FGOProcessor processor(config);
        const libgnss::FGOProcessor::FGOProblem problem =
            config.use_double_difference_factors
                ? processor.buildDoubleDifferenceProblem(
                      epochs, base_epochs, nav_data, base_position)
                : processor.buildPseudorangeProblem(epochs, nav_data);
        libgnss::FGOProcessor::FGOResult result;
        if (options.debug_problem_only) {
            result = makeProblemOnlyResult(problem, config);
        } else if (options.backend == "gtsam-pc") {
#ifdef GNSS_WITH_GTSAM
            if (!config.use_double_difference_factors) {
                std::cerr << "Error: --backend gtsam-pc requires --base and DD factors\n";
                return 1;
            }
            result = optimizeGtsamPcProblem(problem, config);
#else
            std::cerr << "Error: --backend gtsam-pc is not available in this build\n";
            return 1;
#endif
        } else {
            result = processor.optimizeProblem(problem);
        }

        if (result.solution.isEmpty()) {
            std::cerr << "Error: FGO produced no valid epochs\n";
            return 1;
        }
        if (!result.solution.writeToFile(options.out_path)) {
            std::cerr << "Error: failed to write solution file: " << options.out_path << "\n";
            return 1;
        }
        if (!writeEpochDebugCsv(options.epoch_debug_csv_path, problem, result)) {
            std::cerr << "Error: failed to write epoch debug CSV: "
                      << options.epoch_debug_csv_path << "\n";
            return 1;
        }
        if (!writeFactorDebugCsv(options.factor_debug_csv_path, problem, result)) {
            std::cerr << "Error: failed to write factor debug CSV: "
                      << options.factor_debug_csv_path << "\n";
            return 1;
        }
        if (!writeSdFactorDebugCsv(options.sd_factor_debug_csv_path,
                                   epochs,
                                   nav_data,
                                   problem,
                                   options)) {
            std::cerr << "Error: failed to write SD factor debug CSV: "
                      << options.sd_factor_debug_csv_path << "\n";
            return 1;
        }
        if (!writeLambdaDebugCsv(options.lambda_debug_csv_path,
                                 result.lambda_debug_entries)) {
            std::cerr << "Error: failed to write lambda debug CSV: "
                      << options.lambda_debug_csv_path << "\n";
            return 1;
        }
        if (!writeCostTraceCsv(options.cost_trace_csv_path,
                               result.cost_trace_entries)) {
            std::cerr << "Error: failed to write cost trace CSV: "
                      << options.cost_trace_csv_path << "\n";
            return 1;
        }

        const auto stats = result.solution.calculateStatistics();
        const double availability_rate =
            ratePercent(stats.valid_solutions, epochs.size());
        const double fix_rate =
            ratePercent(stats.fixed_solutions, stats.valid_solutions);
        const double float_rate =
            ratePercent(stats.float_solutions, stats.valid_solutions);
        writeSummaryJson(options,
                         result,
                         stats,
                         epochs.size(),
                         seed_matched_epochs,
                         seed_interpolated_epochs,
                         availability_rate,
                         fix_rate,
                         float_rate);

        if (!options.quiet) {
            std::cout << "Preset: " << options.preset << "\n";
            std::cout << "Backend: " << options.backend << "\n";
            std::cout << "Input epochs: " << epochs.size() << "\n";
            if (!options.seed_pos_path.empty()) {
                std::cout << "Seed POS matched epochs: "
                          << seed_matched_epochs << "\n";
                std::cout << "Seed POS interpolated epochs: "
                          << seed_interpolated_epochs << "\n";
                std::cout << "SPP seed: "
                          << (config.use_spp_seed ? "enabled" : "disabled")
                          << "\n";
            }
            std::cout << "Optimized epochs: " << result.diagnostics.epochs << "\n";
            std::cout << "Valid solutions: " << stats.valid_solutions << "\n";
            std::cout << "Fixed solutions: " << stats.fixed_solutions << "\n";
            std::cout << "Float solutions: " << stats.float_solutions << "\n";
            std::cout << "Availability rate [%]: "
                      << formatPercent(availability_rate) << "\n";
            std::cout << "Fix rate [%]: "
                      << formatPercent(fix_rate) << "\n";
            std::cout << "Float rate [%]: "
                      << formatPercent(float_rate) << "\n";
            std::cout << "Pseudorange factors: " << result.diagnostics.pseudorange_factors << "\n";
            std::cout << "TDCP factors: " << result.diagnostics.tdcp_factors << "\n";
            std::cout << "SD Doppler factors: "
                      << result.diagnostics.single_difference_doppler_factors
                      << "\n";
            std::cout << "SD TDCP factors: "
                      << result.diagnostics.single_difference_tdcp_factors
                      << "\n";
            std::cout << "Carrier phase factors: "
                      << result.diagnostics.carrier_phase_factors << "\n";
            std::cout << "Double-difference pseudorange factors: "
                      << result.diagnostics.double_difference_pseudorange_factors
                      << "\n";
            std::cout << "Double-difference carrier factors: "
                      << result.diagnostics.double_difference_carrier_factors << "\n";
            std::cout << "Ambiguity states: " << result.diagnostics.ambiguity_states << "\n";
            std::cout << "Ambiguity fix candidates: "
                      << result.diagnostics.ambiguity_fix_candidates << "\n";
            std::cout << "LAMBDA ambiguity candidates: "
                      << result.diagnostics.lambda_ambiguity_candidates << "\n";
            std::cout << "LAMBDA used candidates: "
                      << result.diagnostics.lambda_ambiguity_used_candidates << "\n";
            std::cout << "LAMBDA attempts: "
                      << result.diagnostics.lambda_ambiguity_attempts << "\n";
            std::cout << "LAMBDA solved: "
                      << (result.diagnostics.lambda_ambiguity_fix_solved ? "yes" : "no")
                      << "\n";
            std::cout << "LAMBDA used: "
                      << (result.diagnostics.lambda_ambiguity_fix_used ? "yes" : "no")
                      << "\n";
            std::cout << "Partial LAMBDA used: "
                      << (result.diagnostics.partial_lambda_ambiguity_fix_used ? "yes" : "no")
                      << "\n";
            std::cout << "LAMBDA ratio: "
                      << result.diagnostics.lambda_ambiguity_ratio << "\n";
            std::cout << "Fixed ambiguities: "
                      << result.diagnostics.fixed_ambiguities << "\n";
            std::cout << "Fixed solution: "
                      << (result.diagnostics.fixed_solution ? "yes" : "no") << "\n";
            std::cout << "TDCP candidate pairs: " << result.diagnostics.tdcp_candidate_pairs << "\n";
            std::cout << "TDCP rejected by gap: " << result.diagnostics.tdcp_rejected_gap << "\n";
            std::cout << "TDCP rejected missing previous: "
                      << result.diagnostics.tdcp_rejected_missing_previous << "\n";
            std::cout << "TDCP rejected by loss of lock: "
                      << result.diagnostics.tdcp_rejected_loss_of_lock << "\n";
            std::cout << "TDCP rejected by code/phase jump: "
                      << result.diagnostics.tdcp_rejected_code_phase_jump << "\n";
            std::cout << "DD matched base epochs: "
                      << result.diagnostics.double_difference_matched_base_epochs << "\n";
            std::cout << "DD interpolated base epochs: "
                      << result.diagnostics.double_difference_interpolated_base_epochs << "\n";
            std::cout << "DD candidate pairs: "
                      << result.diagnostics.double_difference_candidate_pairs << "\n";
            std::cout << "DD rejected no base epoch: "
                      << result.diagnostics.double_difference_rejected_no_base_epoch << "\n";
            std::cout << "DD rejected no reference: "
                      << result.diagnostics.double_difference_rejected_no_reference << "\n";
            std::cout << "FLOAT rejected by seed divergence: "
                      << result.diagnostics.float_rejected_seed_position_divergence
                      << "\n";
            std::cout << "FLOAT rejected by position jump: "
                      << result.diagnostics.float_rejected_position_jump << "\n";
            std::cout << "Motion factors: " << result.diagnostics.motion_factors << "\n";
            std::cout << "Ambiguity between factors: "
                      << result.diagnostics.ambiguity_between_factors << "\n";
            std::cout << "Robust pseudorange factors: "
                      << result.diagnostics.robust_pseudorange_factors << "\n";
            std::cout << "Robust carrier phase factors: "
                      << result.diagnostics.robust_carrier_phase_factors << "\n";
            std::cout << "Robust DD pseudorange factors: "
                      << result.diagnostics.robust_double_difference_pseudorange_factors
                      << "\n";
            std::cout << "Robust DD carrier factors: "
                      << result.diagnostics.robust_double_difference_carrier_factors << "\n";
            std::cout << "Robust TDCP factors: "
                      << result.diagnostics.robust_tdcp_factors << "\n";
            std::cout << "Iterations: " << result.diagnostics.iterations << "\n";
            std::cout << "Converged: " << (result.diagnostics.converged ? "yes" : "no") << "\n";
            std::cout << "Processing time [ms]: "
                      << result.diagnostics.processing_time_ms << "\n";
            std::cout << "Last update norm [m]: "
                      << result.diagnostics.last_update_norm_m << "\n";
            std::cout << "Residual RMS [m]: " << result.diagnostics.residual_rms_m << "\n";
            std::cout << "TDCP residual RMS [m]: " << result.diagnostics.tdcp_residual_rms_m << "\n";
            std::cout << "SD Doppler residual RMS [m/s]: "
                      << result.diagnostics.single_difference_doppler_residual_rms_mps
                      << "\n";
            std::cout << "SD TDCP residual RMS [m]: "
                      << result.diagnostics.single_difference_tdcp_residual_rms_m
                      << "\n";
            std::cout << "Carrier phase residual RMS [m]: "
                      << result.diagnostics.carrier_phase_residual_rms_m << "\n";
            std::cout << "DD pseudorange residual RMS [m]: "
                      << result.diagnostics.double_difference_pseudorange_residual_rms_m
                      << "\n";
            std::cout << "DD carrier residual RMS [m]: "
                      << result.diagnostics.double_difference_carrier_residual_rms_m << "\n";
            std::cout << "Fixed ambiguity residual RMS [cycles]: "
                      << result.diagnostics.fixed_ambiguity_residual_rms_cycles << "\n";
            if (!options.summary_json_path.empty()) {
                std::cout << "Summary JSON: " << options.summary_json_path << "\n";
            }
            if (!options.epoch_debug_csv_path.empty()) {
                std::cout << "Epoch debug CSV: "
                          << options.epoch_debug_csv_path << "\n";
            }
            if (!options.factor_debug_csv_path.empty()) {
                std::cout << "Factor debug CSV: "
                          << options.factor_debug_csv_path << "\n";
            }
            if (!options.lambda_debug_csv_path.empty()) {
                std::cout << "LAMBDA debug CSV: "
                          << options.lambda_debug_csv_path << "\n";
            }
            std::cout << "Output: " << options.out_path << "\n";
        }

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
