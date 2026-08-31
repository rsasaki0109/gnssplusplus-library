#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/io/android_raw_gnss.hpp>
#include <libgnss++/io/rinex.hpp>
#include <libgnss++/models/ionosphere.hpp>
#include <libgnss++/models/troposphere.hpp>

#include <Eigen/Sparse>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
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
#include <unistd.h>
#include <vector>

#include "observable_measurement_helpers.hpp"
#include "observable_upstream_preprocessing.hpp"
#include "observable_robust_loss.hpp"
#include "observable_seed_positions.hpp"

namespace {

using libgnss_apps::SeedPosition;
using libgnss_apps::clockGroup;
using libgnss_apps::findSeedPosition;
using libgnss_apps::groupDelayCorrectionMeters;
using libgnss_apps::isHealthyForPositioning;
using libgnss_apps::isPrimaryPdSignal;
using libgnss_apps::readSeedPositions;
using libgnss_apps::robustHuberLoss;
using libgnss_apps::robustHuberWeight;
using libgnss_apps::sagnacRangeCorrection;
using libgnss_apps::signalName;
namespace upstream = libgnss_apps::upstream;

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kRadiansToDegrees = 180.0 / kPi;
constexpr std::size_t kClockGroups = 5;
constexpr std::size_t kStateStride = 12;

struct Options {
    std::string obs_path;
    std::string android_raw_path;
    std::string nav_path;
    std::string seed_pos_path;
    std::string out_csv_path;
    std::string keyed_out_csv_path;
    std::string trip_id = "native/phone";
    std::string device_model;
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
    bool include_galileo_e1 = true;
    bool include_l5 = true;
    // Experimental, raw-only port of taroz exobs_residuals/obserrmodel.
    // The default remains false so existing RINEX and raw artifacts retain
    // their byte-compatible behavior.
    bool upstream_residual_snr = false;
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
    std::size_t native_spp_seed_epochs = 0;
    std::size_t native_spp_reinitializations = 0;
    std::size_t native_spp_attempts = 0;
    std::size_t native_spp_valid_solutions = 0;
    int native_spp_last_satellites = 0;
    int native_spp_last_iterations = 0;
    double native_spp_last_position_norm_m = 0.0;
    double native_spp_last_gdop = 0.0;
    double native_spp_last_residual_rms_m = 0.0;
    int native_spp_last_gdop_rejections = 0;
    int native_spp_last_residual_rejections = 0;
    int native_spp_last_chi_rejections = 0;
    bool native_spp_first_failure_recorded = false;
    int native_spp_first_failure_status = 0;
    int native_spp_first_failure_satellites = 0;
    double native_spp_first_failure_position_norm_m = 0.0;
    double native_spp_first_failure_gdop = 0.0;
    double native_spp_first_failure_residual_rms_m = 0.0;
    int native_spp_last_status = 0;
    bool have_native_spp_last_position = false;
    libgnss::Vector3d native_spp_last_position = libgnss::Vector3d::Zero();
    std::size_t native_spp_propagated_seed_epochs = 0;
    std::size_t seed_failures = 0;
    bool android_raw = false;
    libgnss::io::AndroidRawGnssDiagnostics android_raw_diagnostics;
    upstream::SnrPercentiles upstream_snr_percentiles;
    std::size_t upstream_pd_pair_rejections = 0;
    std::size_t upstream_ld_pair_rejections = 0;
    std::size_t upstream_pseudorange_residual_rejections = 0;
    std::size_t upstream_doppler_residual_rejections = 0;
    std::size_t upstream_pseudorange_candidates = 0;
    std::size_t upstream_doppler_candidates = 0;
    std::size_t upstream_pseudorange_global_groups = 0;
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
              // Keep the original RINEX usage text byte-for-byte.  The raw
              // Android frontend is an additive mode, so existing scripts
              // and the observable CLI contract retain their diagnostics.
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
        } else if (arg == "--android-raw") {
            options.android_raw_path = require_value(arg);
        } else if (arg == "--nav") {
            options.nav_path = require_value(arg);
        } else if (arg == "--seed-pos") {
            options.seed_pos_path = require_value(arg);
        } else if (arg == "--out-csv") {
            options.out_csv_path = require_value(arg);
        } else if (arg == "--keyed-out-csv") {
            options.keyed_out_csv_path = require_value(arg);
        } else if (arg == "--trip-id") {
            options.trip_id = require_value(arg);
        } else if (arg == "--device-model") {
            options.device_model = require_value(arg);
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
        } else if (arg == "--no-galileo-e1") {
            options.include_galileo_e1 = false;
        } else if (arg == "--no-l5") {
            options.include_l5 = false;
        } else if (arg == "--upstream-residual-snr") {
            options.upstream_residual_snr = true;
        } else if (arg == "--quiet") {
            options.quiet = true;
        } else if (arg == "--help" || arg == "-h") {
            usageError("help requested", argv[0]);
        } else {
            usageError("unknown argument: " + arg, argv[0]);
        }
    }

    if (options.obs_path.empty() && options.android_raw_path.empty()) {
        usageError("--obs is required", argv[0]);
    }
    if (options.nav_path.empty()) {
        usageError("--nav is required", argv[0]);
    }
    if (options.obs_path.empty() == options.android_raw_path.empty()) {
        usageError("choose exactly one of --obs or --android-raw", argv[0]);
    }
    if (!options.android_raw_path.empty() && !options.seed_pos_path.empty()) {
        usageError("--seed-pos is not accepted with --android-raw", argv[0]);
    }
    if (options.android_raw_path.empty() && options.seed_pos_path.empty()) {
        usageError("--seed-pos is required", argv[0]);
    }
    if (options.upstream_residual_snr && options.android_raw_path.empty()) {
        usageError("--upstream-residual-snr requires --android-raw", argv[0]);
    }
    if (options.obs_path.empty() && options.keyed_out_csv_path.empty()) {
        usageError("--keyed-out-csv is required with --android-raw", argv[0]);
    }
    if (!options.android_raw_path.empty() && options.trip_id.empty()) {
        usageError("--trip-id must not be empty with --android-raw", argv[0]);
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

void writeCsvDouble(std::ostream& output, double value) {
    if (std::isfinite(value)) {
        output << value;
    } else {
        output << "NaN";
    }
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

struct UpstreamEpochMask {
    std::set<ObservationKey> pseudorange;
    std::set<ObservationKey> carrier;
};

double finiteMedian(std::vector<double> values) {
    values.erase(std::remove_if(values.begin(), values.end(),
                                [](double value) {
                                    return !std::isfinite(value);
                                }),
                  values.end());
    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2U;
    if ((values.size() & 1U) != 0U) return values[middle];
    return 0.5 * (values[middle - 1U] + values[middle]);
}

bool upstreamCarrierSignDevice(const std::string& device_model) {
    return device_model == "sm-a205u" || device_model == "sm-a217m" ||
           device_model == "sm-a505g" || device_model == "sm-a600t" ||
           device_model == "sm-a505u";
}

bool upstreamPseudorangePairSkipDevice(const std::string& device_model) {
    // exobs_residuals.m skips only the two phones whose pseudorange/Doppler
    // timing is known to make this particular diagnostic non-actionable.
    return device_model == "sm-a205u" || device_model == "sm-a505u";
}

double effectiveMinimumSnr(const Options& options) {
    return options.upstream_residual_snr ? 20.0 : options.min_snr_dbhz;
}

double effectiveMinimumElevationDegrees(const Options& options) {
    // parameters.m uses 5 degrees whenever the L5 setting is enabled.  The
    // raw frontend exposes L5 by default, so the opt-in residual lane uses
    // that fixed published mask; the legacy lane retains its 15 degree mask.
    return options.upstream_residual_snr ? 5.0 : options.min_elevation_deg;
}

upstream::SnrPercentiles collectUpstreamSnrPercentiles(
    const libgnss::ObservationSeries& observations) {
    std::vector<double> l1;
    std::vector<double> l5;
    for (const auto& epoch : observations.epochs) {
        for (const auto& observation : epoch.observations) {
            if (upstream::bandForSignal(observation.signal) ==
                upstream::ObservationBand::L1) {
                l1.push_back(observation.snr);
            } else if (upstream::bandForSignal(observation.signal) ==
                       upstream::ObservationBand::L5) {
                l5.push_back(observation.snr);
            }
        }
    }
    upstream::SnrPercentiles percentiles;
    percentiles.l1_dbhz = upstream::linearPercentile(std::move(l1), 85.0);
    percentiles.l5_dbhz = upstream::linearPercentile(std::move(l5), 85.0);
    return percentiles;
}

void applyUpstreamAdjacentMasks(
    const std::vector<libgnss::ObservationData>& epochs,
    const std::string& device_model,
    std::vector<UpstreamEpochMask>& masks,
    std::size_t& pseudorange_rejections,
    std::size_t& carrier_rejections) {
    masks.assign(epochs.size(), UpstreamEpochMask{});
    std::map<ObservationKey, std::tuple<std::size_t, const libgnss::Observation*,
                                        libgnss::GNSSTime>> previous;
    for (std::size_t current_index = 0; current_index < epochs.size();
         ++current_index) {
        const auto& epoch = epochs[current_index];
        for (const auto& observation : epoch.observations) {
            const ObservationKey key{observation.satellite, observation.signal};
            const auto previous_it = previous.find(key);
            if (previous_it != previous.end()) {
                const auto [previous_index, previous_observation, previous_time] =
                    previous_it->second;
                const double dt = epoch.time - previous_time;
                const upstream::ObservationBand band =
                    upstream::bandForSignal(observation.signal);
                const double wavelength =
                    libgnss::signalWavelengthMeters(observation);
                if (dt > 0.0 && dt <= 1.5 &&
                    band != upstream::ObservationBand::Unknown &&
                    upstream::finitePositive(wavelength)) {
                    if (previous_observation->has_pseudorange &&
                        observation.has_pseudorange &&
                        previous_observation->pseudorange > 0.0 &&
                        observation.pseudorange > 0.0 &&
                        previous_observation->has_doppler &&
                        observation.has_doppler &&
                        std::isfinite(previous_observation->doppler) &&
                        std::isfinite(observation.doppler)) {
                        const double difference =
                            upstream::pseudorangeDopplerDifference(
                                previous_observation->pseudorange,
                                observation.pseudorange,
                                previous_observation->doppler,
                                observation.doppler,
                                wavelength,
                                dt);
                        if (std::isfinite(difference) &&
                            std::abs(difference) >
                                upstream::pairThreshold(band, 'P') &&
                            !upstreamPseudorangePairSkipDevice(device_model)) {
                            const bool inserted_previous =
                                masks[previous_index].pseudorange.insert(key).second;
                            const bool inserted_current =
                                masks[current_index].pseudorange.insert(key).second;
                            if (inserted_previous) ++pseudorange_rejections;
                            if (inserted_current) ++pseudorange_rejections;
                        }
                    }
                    if (previous_observation->has_carrier_phase &&
                        observation.has_carrier_phase &&
                        std::isfinite(previous_observation->carrier_phase) &&
                        std::isfinite(observation.carrier_phase) &&
                        previous_observation->has_doppler &&
                        observation.has_doppler &&
                        std::isfinite(previous_observation->doppler) &&
                        std::isfinite(observation.doppler)) {
                        const double offset = upstreamCarrierSignDevice(device_model)
                                                   ? 1.117
                                                   : 0.0;
                        const double difference =
                            upstream::carrierDopplerDifference(
                                previous_observation->carrier_phase,
                                observation.carrier_phase,
                                previous_observation->doppler,
                                observation.doppler,
                                wavelength,
                                dt,
                                offset);
                        if (std::isfinite(difference) &&
                            std::abs(difference) >
                                upstream::pairThreshold(band, 'L')) {
                            const bool inserted_previous =
                                masks[previous_index].carrier.insert(key).second;
                            const bool inserted_current =
                                masks[current_index].carrier.insert(key).second;
                            if (inserted_previous) ++carrier_rejections;
                            if (inserted_current) ++carrier_rejections;
                        }
                    }
                }
            }
            previous[key] = {current_index, &observation, epoch.time};
        }
    }
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
        observation.snr < effectiveMinimumSnr(options)) {
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
    return geometry.elevation >=
           effectiveMinimumElevationDegrees(options) * kDegreesToRadians;
}

bool preparePseudorangeFactor(const libgnss::ObservationData& epoch,
                              std::size_t epoch_index,
                              const libgnss::Observation& observation,
                              const libgnss::NavigationData& nav,
                              const libgnss::Vector3d& receiver_position,
                              const Options& options,
                              const upstream::SnrPercentiles& snr_percentiles,
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
    factor.sigma_m = options.upstream_residual_snr
                         ? upstream::snrPercentileSigma(
                               observation.signal, observation.snr,
                               snr_percentiles, 'P')
                         : options.pseudorange_sigma_zenith_m /
                               std::sqrt(sin_el);
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
                          const upstream::SnrPercentiles& snr_percentiles,
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
    factor.sigma_mps = options.upstream_residual_snr
                           ? upstream::snrPercentileSigma(
                                 observation.signal, observation.snr,
                                 snr_percentiles, 'D')
                           : options.doppler_sigma_zenith_mps /
                                 std::sqrt(sin_el);
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
    libgnss::io::RINEXReader::RINEXHeader obs_header;
    libgnss::io::RINEXReader obs_reader;
    libgnss::io::AndroidRawGnssResult android_raw;
    if (!options.android_raw_path.empty()) {
        libgnss::io::AndroidRawGnssConfig converter_config;
        converter_config.device_model = options.device_model;
        converter_config.include_galileo_e1 = options.include_galileo_e1;
        converter_config.include_l5 = options.include_l5;
        std::string conversion_error;
        if (!libgnss::io::loadAndroidRawGnssCsv(
                options.android_raw_path, converter_config, android_raw,
                conversion_error)) {
            throw std::runtime_error("failed to convert raw Android GNSS: " +
                                     conversion_error);
        }
        // Android Raw epochs are formed by utcTimeMillis, and the converter
        // already reproduces the upstream receiver-time equation.  The
        // dedicated PDC path retains those timestamps and does not synthesize
        // a Python/RINEX intermediate.
        obs_header.interval = 1.0;
    } else {
        if (!obs_reader.open(options.obs_path)) {
            throw std::runtime_error("failed to open observation file: " + options.obs_path);
        }
        if (!obs_reader.readHeader(obs_header)) {
            throw std::runtime_error("failed to read observation header: " + options.obs_path);
        }
    }

    libgnss::io::RINEXReader nav_reader;
    if (!nav_reader.open(options.nav_path)) {
        throw std::runtime_error("failed to open navigation file: " + options.nav_path);
    }
    libgnss::NavigationData nav_data;
    if (!nav_reader.readNavigationData(nav_data)) {
        throw std::runtime_error("failed to read navigation data: " + options.nav_path);
    }

    const std::vector<SeedPosition> seed_positions = options.android_raw_path.empty()
                                                          ? readSeedPositions(options.seed_pos_path)
                                                          : std::vector<SeedPosition>{};
    std::size_t seed_cursor = 0;

    // Raw Android mode must not consume the optional WlsPosition* columns.
    // They are useful adapter diagnostics, but are generated/enriched
    // coordinates rather than an inference input.  Use the existing native
    // SPP implementation for a truth-free receiver seed instead.  RINEX mode
    // retains its historical explicit --seed-pos/header contract.
    libgnss::SPPProcessor native_spp;
    if (!options.android_raw_path.empty()) {
        libgnss::ProcessorConfig spp_processor_config;
        spp_processor_config.elevation_mask = 0.0;
        spp_processor_config.snr_mask = 0.0;
        spp_processor_config.use_ionosphere_model = true;
        spp_processor_config.use_troposphere_model = true;
        libgnss::SPPProcessor::SPPConfig spp_config;
        spp_config.use_precise_products = false;
        spp_config.use_ssr_corrections = false;
        spp_config.use_ionex_corrections = false;
        spp_config.use_dcb_corrections = false;
        spp_config.enable_position_jump_gate = false;
        spp_config.enable_robust_weighting = false;
        spp_config.enable_outlier_detection = false;
        spp_config.enable_raim_fde = false;
        spp_config.max_gdop = 0.0;
        // The seed is a receiver-only frame initializer.  Do not introduce
        // unconstrained per-constellation clock columns when a sparse first
        // epoch has fewer than one independent observation per system; the
        // downstream PDC graph retains its full clock-group model.
        spp_config.model_intersystem_bias = false;
        native_spp.setSPPConfig(spp_config);
        if (!native_spp.initialize(spp_processor_config)) {
            throw std::runtime_error("failed to initialize native raw SPP seed");
        }
    }

    Problem problem;
    problem.android_raw = !options.android_raw_path.empty();
    if (problem.android_raw) {
        problem.android_raw_diagnostics = android_raw.diagnostics;
    }
    std::set<libgnss::SatelliteId> unique_satellites;
    const double fixed_interval_s =
        obs_header.interval > 0.0 ? obs_header.interval : 1.0;
    const double gap_tolerance_s = std::max(0.01, 0.5 * fixed_interval_s);
    std::map<libgnss::SatelliteId, double> previous_gps_pseudorange_by_satellite;
    bool have_previous_gps_pseudorange = false;
    std::vector<std::map<ObservationKey, CarrierResidual>> carrier_residuals_by_epoch;
    std::vector<UpstreamEpochMask> upstream_epoch_masks;
    if (problem.android_raw && options.upstream_residual_snr) {
        problem.upstream_snr_percentiles =
            collectUpstreamSnrPercentiles(android_raw.observations);
        applyUpstreamAdjacentMasks(android_raw.observations.epochs,
                                   options.device_model,
                                   upstream_epoch_masks,
                                   problem.upstream_pd_pair_rejections,
                                   problem.upstream_ld_pair_rejections);
    }

    auto append_epoch = [&](libgnss::ObservationData epoch,
                            bool build_factors,
                            const UpstreamEpochMask* upstream_mask) -> bool {
        if (options.max_epochs > 0 &&
            static_cast<int>(problem.epochs.size()) >= options.max_epochs) {
            return false;
        }

        libgnss::Vector3d seed_position = libgnss::Vector3d::Zero();
        bool interpolated = false;
        if (problem.android_raw) {
            // Clear the optional device WLS coordinates before invoking the
            // library SPP seed so a CSV enriched by a Python adapter cannot
            // affect native inference, even indirectly through initialization.
            libgnss::ObservationData spp_epoch = epoch;
            // A deterministic Earth-surface point only supplies a bounded
            // coordinate frame for the first native SPP iteration.  It is not
            // a route/device coordinate and is never emitted; subsequent
            // epochs reuse the last finite estimate produced by native SPP.
            spp_epoch.receiver_position = problem.have_native_spp_last_position
                                              ? problem.native_spp_last_position
                                              : libgnss::Vector3d(6378137.0, 0.0, 0.0);
            ++problem.native_spp_attempts;
            libgnss::PositionSolution spp_solution =
                native_spp.processEpoch(spp_epoch, nav_data);
            problem.native_spp_last_satellites = spp_solution.num_satellites;
            problem.native_spp_last_iterations = spp_solution.iterations;
            problem.native_spp_last_position_norm_m = spp_solution.position_ecef.norm();
            problem.native_spp_last_status = static_cast<int>(spp_solution.status);
            problem.native_spp_last_gdop = spp_solution.gdop;
            problem.native_spp_last_residual_rms_m = spp_solution.residual_rms;
            problem.native_spp_last_gdop_rejections = spp_solution.spp_gdop_gate_rejections;
            problem.native_spp_last_residual_rejections = spp_solution.spp_residual_gate_rejections;
            problem.native_spp_last_chi_rejections = spp_solution.spp_chi_square_gate_rejections;
            if (!spp_solution.isValid() || !spp_solution.position_ecef.allFinite() ||
                spp_solution.position_ecef.norm() < 6.0e6 ||
                spp_solution.position_ecef.norm() > 7.0e6) {
                // A bad raw epoch must not poison the stateful SPP seed for
                // subsequent epochs.  Reinitialize once from the last finite
                // native estimate (or the same fixed Earth-surface frame for
                // the first epoch); this is solver-state recovery, not a
                // device/result/truth coordinate fallback.
                native_spp.reset();
                ++problem.native_spp_reinitializations;
                ++problem.native_spp_attempts;
                spp_solution = native_spp.processEpoch(spp_epoch, nav_data);
                problem.native_spp_last_satellites = spp_solution.num_satellites;
                problem.native_spp_last_iterations = spp_solution.iterations;
                problem.native_spp_last_position_norm_m = spp_solution.position_ecef.norm();
                problem.native_spp_last_status = static_cast<int>(spp_solution.status);
                problem.native_spp_last_gdop = spp_solution.gdop;
                problem.native_spp_last_residual_rms_m = spp_solution.residual_rms;
                problem.native_spp_last_gdop_rejections = spp_solution.spp_gdop_gate_rejections;
                problem.native_spp_last_residual_rejections = spp_solution.spp_residual_gate_rejections;
                problem.native_spp_last_chi_rejections = spp_solution.spp_chi_square_gate_rejections;
            }
            if (spp_solution.isValid() && spp_solution.position_ecef.allFinite() &&
                spp_solution.position_ecef.norm() >= 6.0e6 &&
                spp_solution.position_ecef.norm() <= 7.0e6) {
                seed_position = spp_solution.position_ecef;
                ++problem.seed_matched_epochs;
                ++problem.native_spp_seed_epochs;
                ++problem.native_spp_valid_solutions;
                problem.native_spp_last_position = spp_solution.position_ecef;
                problem.have_native_spp_last_position = true;
            } else if (problem.have_native_spp_last_position) {
                // Keep the previous native SPP fix as an initial value for a
                // single raw epoch that has no standalone SPP solution.  It
                // is never updated by this branch and is visible in the
                // manifest; downstream PDC factors remain the only source of
                // the optimized position.
                seed_position = problem.native_spp_last_position;
                ++problem.seed_matched_epochs;
                ++problem.native_spp_propagated_seed_epochs;
            } else {
                if (!problem.native_spp_first_failure_recorded) {
                    problem.native_spp_first_failure_recorded = true;
                    problem.native_spp_first_failure_status =
                        static_cast<int>(spp_solution.status);
                    problem.native_spp_first_failure_satellites =
                        spp_solution.num_satellites;
                    problem.native_spp_first_failure_position_norm_m =
                        spp_solution.position_ecef.norm();
                    problem.native_spp_first_failure_gdop = spp_solution.gdop;
                    problem.native_spp_first_failure_residual_rms_m =
                        spp_solution.residual_rms;
                }
                ++problem.seed_failures;
                return true;
            }
        } else if (!problem.android_raw &&
                   findSeedPosition(seed_positions,
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
        } else if (!problem.android_raw && obs_header.approximate_position.norm() > 1e6) {
            seed_position = obs_header.approximate_position;
            epoch.receiver_position = seed_position;
        } else {
            return true;
        }

        const std::size_t epoch_index = problem.epochs.size();
        std::map<libgnss::SatelliteId, double> gps_pseudorange_by_satellite;
        std::map<ObservationKey, CarrierResidual> carrier_residuals;
        if (build_factors) {
            // Apply residual-stage masks only to the factor view.  Native SPP
            // above intentionally sees the complete raw epoch, matching the
            // upstream ordering where initialization precedes exobs_residuals.
            libgnss::ObservationData factor_epoch = epoch;
            if (upstream_mask != nullptr) {
                for (auto& observation : factor_epoch.observations) {
                    const ObservationKey key{observation.satellite,
                                             observation.signal};
                    if (upstream_mask->pseudorange.find(key) !=
                        upstream_mask->pseudorange.end()) {
                        observation.has_pseudorange = false;
                    }
                    if (upstream_mask->carrier.find(key) !=
                        upstream_mask->carrier.end()) {
                        observation.has_carrier_phase = false;
                    }
                }
            }
            std::vector<PseudorangeFactor> epoch_pseudorange;
            std::vector<DopplerFactor> epoch_doppler;
            for (const auto& observation : factor_epoch.observations) {
                unique_satellites.insert(observation.satellite);
                PseudorangeFactor pseudorange_factor;
                if (preparePseudorangeFactor(epoch,
                                             epoch_index,
                                             observation,
                                             nav_data,
                                             seed_position,
                                             options,
                                             problem.upstream_snr_percentiles,
                                             pseudorange_factor)) {
                    epoch_pseudorange.push_back(pseudorange_factor);
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
                                         problem.upstream_snr_percentiles,
                                         factor)) {
                    epoch_doppler.push_back(factor);
                }

                CarrierResidual carrier_residual;
                if (prepareCarrierResidual(factor_epoch,
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

            if (options.upstream_residual_snr) {
                problem.upstream_pseudorange_candidates +=
                    epoch_pseudorange.size();
                problem.upstream_doppler_candidates += epoch_doppler.size();

                // exobs_residuals.m computes ``isb`` with
                // splitapply(medianall, resPc-clk, findgroups(obs.sys)).
                // resPc is an epoch-by-satellite matrix and obs.sys is a row
                // vector, so MATLAB groups columns: each system's median is
                // taken over all epochs in this frequency loop and reused at
                // every epoch.  Preserve all finite candidates here and
                // apply the global system/band pass after the raw stream has
                // been consumed (one band corresponds to one frequency loop).
                problem.pseudorange_factors.insert(
                    problem.pseudorange_factors.end(), epoch_pseudorange.begin(),
                    epoch_pseudorange.end());

                // exobs_residuals.m subtracts the receiver clock drift from
                // Doppler residuals before applying its 3 m/s gate.  In the
                // raw-only lane the finite, per-epoch median of the modeled
                // residuals is the observable clock-drift center; no truth or
                // device-provided coordinate is used.
                std::vector<double> doppler_residuals;
                doppler_residuals.reserve(epoch_doppler.size());
                for (const auto& factor : epoch_doppler) {
                    doppler_residuals.push_back(factor.residual_mps);
                }
                const double doppler_median = finiteMedian(doppler_residuals);
                for (const auto& factor : epoch_doppler) {
                    const double threshold = upstream::residualThreshold(
                        upstream::bandForSignal(factor.signal), 'D');
                    if (std::isfinite(doppler_median) &&
                        std::isfinite(threshold) &&
                        std::abs(factor.residual_mps - doppler_median) <= threshold) {
                        problem.factors.push_back(factor);
                    } else {
                        ++problem.upstream_doppler_residual_rejections;
                    }
                }
            } else {
                problem.pseudorange_factors.insert(problem.pseudorange_factors.end(),
                                                   epoch_pseudorange.begin(),
                                                   epoch_pseudorange.end());
                problem.factors.insert(problem.factors.end(),
                                       epoch_doppler.begin(), epoch_doppler.end());
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
    auto consume_epoch = [&](libgnss::ObservationData next_epoch) -> bool {
        epoch = std::move(next_epoch);
        if (raw_epoch_index++ < options.skip_epochs) {
            return true;
        }

        if (have_last_output_time) {
            libgnss::GNSSTime expected_time =
                last_output_time + fixed_interval_s;
            while (epoch.time - expected_time > gap_tolerance_s) {
                libgnss::ObservationData empty_epoch(expected_time);
                if (!append_epoch(empty_epoch, false, nullptr)) {
                    break;
                }
                last_output_time = expected_time;
                expected_time = last_output_time + fixed_interval_s;
                if (options.max_epochs > 0 &&
                    static_cast<int>(problem.epochs.size()) >=
                        options.max_epochs) {
                    return false;
                }
            }
            if (options.max_epochs > 0 &&
                static_cast<int>(problem.epochs.size()) >= options.max_epochs) {
                return false;
            }
        }

        const UpstreamEpochMask* upstream_mask = nullptr;
        if (problem.android_raw && options.upstream_residual_snr &&
            raw_epoch_index > 0 &&
            static_cast<std::size_t>(raw_epoch_index - 1) <
                upstream_epoch_masks.size()) {
            upstream_mask = &upstream_epoch_masks[
                static_cast<std::size_t>(raw_epoch_index - 1)];
        }
        if (!append_epoch(epoch, true, upstream_mask)) {
            return false;
        }
        last_output_time = epoch.time;
        have_last_output_time = true;
        return true;
    };
    if (problem.android_raw) {
        for (auto& raw_epoch : android_raw.observations.epochs) {
            if (!consume_epoch(std::move(raw_epoch))) break;
        }
    } else {
        while (obs_reader.readObservationEpoch(epoch)) {
            if (!consume_epoch(std::move(epoch))) break;
        }
    }

    if (problem.android_raw && options.upstream_residual_snr) {
        // This is the second pass corresponding to
        // ``isb=splitapply(medianall,resPc-clk,findgroups(obs.sys))`` in
        // exobs_residuals.m.  The source matrix is epoch-by-satellite and
        // the row grouping vector is satellite/system, hence the center is
        // global over all retained epochs of each system and frequency.  A
        // per-epoch center would silently turn the published ISB exclusion
        // into a different algorithm.
        std::map<std::pair<std::size_t, upstream::ObservationBand>,
                 std::vector<double>> pseudorange_residuals;
        for (const auto& factor : problem.pseudorange_factors) {
            // The upstream ``clk`` argument is the smartphone receiver clock
            // estimate in meters.  AndroidRawGnss stores the same estimate in
            // seconds, so remove it for the ISB center while retaining the
            // original residual in the graph (the graph clock state carries
            // that bias, just as the upstream factor does).
            const double receiver_clock_m =
                problem.epochs[factor.epoch_index].receiver_clock_bias *
                libgnss::constants::SPEED_OF_LIGHT;
            pseudorange_residuals[{factor.clock_group,
                                   upstream::bandForSignal(factor.signal)}]
                .push_back(factor.residual_m - receiver_clock_m);
        }
        std::map<std::pair<std::size_t, upstream::ObservationBand>, double>
            pseudorange_medians;
        for (const auto& [key, residuals] : pseudorange_residuals) {
            pseudorange_medians[key] = finiteMedian(residuals);
        }
        problem.upstream_pseudorange_global_groups = pseudorange_medians.size();
        std::vector<PseudorangeFactor> filtered;
        filtered.reserve(problem.pseudorange_factors.size());
        for (const auto& factor : problem.pseudorange_factors) {
            const auto key = std::make_pair(
                factor.clock_group, upstream::bandForSignal(factor.signal));
            const auto median_it = pseudorange_medians.find(key);
            const double median = median_it == pseudorange_medians.end()
                                      ? std::numeric_limits<double>::quiet_NaN()
                                      : median_it->second;
            const double threshold = upstream::residualThreshold(
                key.second, 'P');
            const double receiver_clock_m =
                problem.epochs[factor.epoch_index].receiver_clock_bias *
                libgnss::constants::SPEED_OF_LIGHT;
            const double residual_without_receiver_clock =
                factor.residual_m - receiver_clock_m;
            if (std::isfinite(median) && std::isfinite(threshold) &&
                std::abs(residual_without_receiver_clock - median) <= threshold) {
                filtered.push_back(factor);
            } else {
                ++problem.upstream_pseudorange_residual_rejections;
            }
        }
        problem.pseudorange_factors.swap(filtered);
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
            factor.sigma_m = options.upstream_residual_snr
                                 ? upstream::snrPercentileSigma(
                                       current.signal,
                                       current.snr_dbhz,
                                       problem.upstream_snr_percentiles,
                                       'L')
                                 : options.tdcp_sigma_zenith_m /
                                       std::sqrt(sin_el);
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

long long unixMillis(const libgnss::GNSSTime& time) {
    // GPS time is UTC plus the fixed 18 leap seconds used by the GSDC 2023
    // release.  Keep this conversion local to the raw Android submission
    // bridge; the existing RINEX dogfood output remains unchanged.
    constexpr long long gps_epoch_unix_seconds = 315964800LL;
    constexpr long long seconds_per_week = 604800LL;
    constexpr long long leap_seconds = 18LL;
    const double unix_seconds =
        static_cast<double>(gps_epoch_unix_seconds) +
        static_cast<double>(time.week) * static_cast<double>(seconds_per_week) +
        time.tow - static_cast<double>(leap_seconds);
    return static_cast<long long>(std::llround(unix_seconds * 1000.0));
}

bool writeAtomicText(const std::string& path, const std::string& content) {
    const std::filesystem::path destination(path);
    std::error_code error;
    if (!destination.parent_path().empty()) {
        std::filesystem::create_directories(destination.parent_path(), error);
        if (error) return false;
    }
    const std::string temporary =
        path + ".tmp." + std::to_string(static_cast<long long>(::getpid()));
    {
        std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
        if (!output.is_open()) return false;
        output << content;
        output.flush();
        if (!output.good()) return false;
    }
    std::filesystem::rename(temporary, destination, error);
    if (error) {
        std::filesystem::remove(temporary);
        return false;
    }
    return true;
}

bool writeKeyedCsv(const std::string& path,
                   const std::string& trip_id,
                   const Problem& problem,
                   const SolveResult& result) {
    if (path.empty()) return true;
    if (problem.epochs.size() * kStateStride !=
        static_cast<std::size_t>(result.state.size())) {
        return false;
    }
    std::ostringstream output;
    output << "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n";
    output << std::fixed << std::setprecision(10);
    for (std::size_t i = 0; i < problem.epochs.size(); ++i) {
        libgnss::Vector3d position = problem.seed_positions[i];
        for (int component = 0; component < 3; ++component) {
            position(component) += result.state(positionColumn(i, component));
        }
        double latitude = 0.0;
        double longitude = 0.0;
        double height = 0.0;
        libgnss::ecef2geodetic(position, latitude, longitude, height);
        if (!position.allFinite() || !std::isfinite(latitude) ||
            !std::isfinite(longitude) || !std::isfinite(height) ||
            std::abs(latitude) > kPi / 2.0 || std::abs(longitude) > kPi) {
            return false;
        }
        output << trip_id << ',' << unixMillis(problem.epochs[i].time) << ','
               << latitude * kRadiansToDegrees << ','
               << longitude * kRadiansToDegrees << '\n';
    }
    return writeAtomicText(path, output.str());
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

void writeJsonDoubleOrNull(std::ostream& output, double value) {
    if (std::isfinite(value)) {
        output << value;
    } else {
        output << "null";
    }
}

bool forbiddenNativeInputPath(const std::string& path) {
    std::string lowered = path;
    std::transform(lowered.begin(), lowered.end(), lowered.begin(),
                   [](unsigned char character) {
                       return static_cast<char>(std::tolower(character));
                   });
    constexpr const char* markers[] = {
        "result_gnss", "ground_truth", "gt.mat", "sample_submission",
        "submission.csv", "submission_", "native-fgo-test-v5", "upstream-mat",
        "precomputed"};
    for (const char* marker : markers) {
        if (lowered.find(marker) != std::string::npos) return true;
    }
    return false;
}

bool forbiddenMatPath(const std::string& path) {
    std::string lowered = path;
    std::transform(lowered.begin(), lowered.end(), lowered.begin(),
                   [](unsigned char character) {
                       return static_cast<char>(std::tolower(character));
                   });
    return lowered.find(".mat") != std::string::npos;
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
           << "  \"backend\": \"eigen\",\n";
    if (problem.android_raw) {
        output << "  \"input_mode\": \"android_raw\",\n";
    }
    output
           << "  \"debug_problem_only\": " << jsonBool(options.debug_problem_only) << ",\n"
           << "  \"optimized_epochs\": " << epoch_count << ",\n"
           << "  \"valid_position_epochs\": " << epoch_count << ",\n"
           << "  \"valid_velocity_epochs\": " << epoch_count << ",\n"
           << "  \"seed_matched_epochs\": " << problem.seed_matched_epochs << ",\n"
           << "  \"seed_interpolated_epochs\": " << problem.seed_interpolated_epochs << ",\n";
    if (problem.android_raw) {
        output << "  \"native_spp_seed_epochs\": " << problem.native_spp_seed_epochs << ",\n"
               << "  \"native_spp_reinitializations\": "
               << problem.native_spp_reinitializations << ",\n"
               << "  \"native_spp_attempts\": " << problem.native_spp_attempts << ",\n"
               << "  \"native_spp_valid_solutions\": "
               << problem.native_spp_valid_solutions << ",\n"
               << "  \"native_spp_last_satellites\": "
               << problem.native_spp_last_satellites << ",\n"
               << "  \"native_spp_last_iterations\": "
               << problem.native_spp_last_iterations << ",\n"
               << "  \"native_spp_last_position_norm_m\": "
               << problem.native_spp_last_position_norm_m << ",\n"
               << "  \"native_spp_last_status\": "
               << problem.native_spp_last_status << ",\n"
               << "  \"native_spp_last_gdop\": " << problem.native_spp_last_gdop << ",\n"
               << "  \"native_spp_last_residual_rms_m\": "
               << problem.native_spp_last_residual_rms_m << ",\n"
               << "  \"native_spp_last_gdop_rejections\": "
               << problem.native_spp_last_gdop_rejections << ",\n"
               << "  \"native_spp_last_residual_rejections\": "
               << problem.native_spp_last_residual_rejections << ",\n"
               << "  \"native_spp_last_chi_rejections\": "
               << problem.native_spp_last_chi_rejections << ",\n"
               << "  \"native_spp_first_failure_status\": "
               << problem.native_spp_first_failure_status << ",\n"
               << "  \"native_spp_first_failure_satellites\": "
               << problem.native_spp_first_failure_satellites << ",\n"
               << "  \"native_spp_first_failure_position_norm_m\": "
               << problem.native_spp_first_failure_position_norm_m << ",\n"
               << "  \"native_spp_first_failure_gdop\": "
               << problem.native_spp_first_failure_gdop << ",\n"
               << "  \"native_spp_first_failure_residual_rms_m\": "
               << problem.native_spp_first_failure_residual_rms_m << ",\n"
               << "  \"native_spp_propagated_seed_epochs\": "
               << problem.native_spp_propagated_seed_epochs << ",\n"
               << "  \"native_spp_seed_failures\": " << problem.seed_failures << ",\n"
               << "  \"device_wls_seed_consumed\": false,\n";
    }
    output << "  \"nsat\": " << problem.nsat << ",\n"
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
           << "  \"graph_values\": " << 4U * epoch_count << ",\n";
    if (problem.android_raw) {
        output << "  \"include_galileo_e1\": " << jsonBool(options.include_galileo_e1) << ",\n"
               << "  \"include_l5\": " << jsonBool(options.include_l5) << ",\n";
    }
    output
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
           << "  \"residual_rms_mps\": " << result.residual_rms_mps;
    if (problem.android_raw) {
        const auto& counts = problem.android_raw_diagnostics;
        output << ",\n"
               << "  \"android_raw_diagnostics\": {\n"
               << "    \"input_rows\": " << counts.input_rows << ",\n"
               << "    \"raw_rows\": " << counts.raw_rows << ",\n"
               << "    \"selected_rows\": " << counts.selected_rows << ",\n"
               << "    \"selected_epochs\": " << counts.selected_epochs << ",\n"
               << "    \"skipped_invalid_timing_rows\": "
               << counts.skipped_invalid_timing_rows << ",\n"
               << "    \"skipped_invalid_quality_rows\": "
               << counts.skipped_invalid_quality_rows << ",\n"
               << "    \"masked_code_rows\": "
               << counts.masked_code_rows << ",\n"
               << "    \"masked_doppler_rows\": "
               << counts.masked_doppler_rows << ",\n"
               << "    \"masked_carrier_rows\": "
               << counts.masked_carrier_rows << ",\n"
               << "    \"skipped_unsupported_signal_rows\": "
               << counts.skipped_unsupported_signal_rows << ",\n"
               << "    \"deduplicated_rows\": " << counts.deduplicated_rows << ",\n"
               << "    \"gps_rows\": " << counts.gps_rows << ",\n"
               << "    \"gps_l1_rows\": " << counts.gps_l1_rows << ",\n"
               << "    \"gps_l5_rows\": " << counts.gps_l5_rows << ",\n"
               << "    \"glonass_l1_rows\": " << counts.glonass_l1_rows << ",\n"
               << "    \"galileo_e1_rows\": " << counts.galileo_e1_rows << ",\n"
               << "    \"galileo_e5_rows\": " << counts.galileo_e5_rows << ",\n"
               << "    \"beidou_l1_rows\": " << counts.beidou_l1_rows << ",\n"
               << "    \"beidou_l5_rows\": " << counts.beidou_l5_rows << ",\n"
               << "    \"carrier_rows\": " << counts.carrier_rows << ",\n"
               << "    \"doppler_rows\": " << counts.doppler_rows << ",\n"
               << "    \"carrier_loss_rows\": " << counts.carrier_loss_rows << ",\n"
               << "    \"clock_discontinuities\": " << counts.clock_discontinuities << ",\n"
               << "    \"timing_formula\": \"raw Android clock equation\",\n"
               << "    \"carrier_formula\": \"ADR metres divided by wavelength\",\n"
               << "    \"doppler_formula\": \"negative pseudorange rate divided by wavelength\"\n"
               << "  }";
        if (options.upstream_residual_snr) {
            output << ",\n"
                   << "  \"upstream_residual_snr\": {\n"
                   << "    \"enabled\": true,\n"
                   << "    \"source_rules\": \"taroz exobs_residuals.m + obserrmodel.m\",\n"
                   << "    \"snr_percentile\": 85,\n"
                   << "    \"snr_denominator_db\": 20,\n"
                   << "    \"snr_p85_l1_dbhz\": ";
            writeJsonDoubleOrNull(output,
                                  problem.upstream_snr_percentiles.l1_dbhz);
            output << ",\n"
                   << "    \"snr_p85_l5_dbhz\": ";
            writeJsonDoubleOrNull(output,
                                  problem.upstream_snr_percentiles.l5_dbhz);
            output << ",\n"
                   << "    \"minimum_snr_dbhz\": "
                   << effectiveMinimumSnr(options) << ",\n"
                   << "    \"minimum_elevation_deg\": "
                   << effectiveMinimumElevationDegrees(options) << ",\n"
                   << "    \"pd_pair_rejected_rows\": "
                   << problem.upstream_pd_pair_rejections << ",\n"
                   << "    \"ld_pair_rejected_rows\": "
                   << problem.upstream_ld_pair_rejections << ",\n"
                   << "    \"pseudorange_candidates\": "
                   << problem.upstream_pseudorange_candidates << ",\n"
                   << "    \"pseudorange_residual_rejected\": "
                   << problem.upstream_pseudorange_residual_rejections << ",\n"
                   << "    \"pseudorange_center_scope\": "
                   << "\"global_epoch_by_system_and_frequency\",\n"
                   << "    \"pseudorange_global_groups\": "
                   << problem.upstream_pseudorange_global_groups << ",\n"
                   << "    \"doppler_candidates\": "
                   << problem.upstream_doppler_candidates << ",\n"
                   << "    \"doppler_residual_rejected\": "
                   << problem.upstream_doppler_residual_rejections << ",\n"
                   << "    \"pseudorange_pair_thresholds_m\": {\"L1\": 40, \"L5\": 20},\n"
                   << "    \"doppler_residual_threshold_mps\": 3,\n"
                   << "    \"carrier_pair_threshold_m\": 1.5,\n"
                   << "    \"carrier_offset_m\": "
                   << (upstreamCarrierSignDevice(options.device_model) ? 1.117 : 0.0)
                   << ",\n"
                   << "    \"base_pseudorange_compensation\": \"not applied; unavailable under raw+nav-only contract\"\n"
                   << "  }";
        }
    }
    output << "\n"
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
        if (forbiddenMatPath(options.obs_path) ||
            forbiddenMatPath(options.android_raw_path) ||
            forbiddenMatPath(options.nav_path) ||
            forbiddenMatPath(options.seed_pos_path) ||
            forbiddenMatPath(options.out_csv_path) ||
            forbiddenMatPath(options.keyed_out_csv_path) ||
            forbiddenMatPath(options.factor_debug_csv_path) ||
            forbiddenMatPath(options.graph_csv_path) ||
            forbiddenMatPath(options.summary_json_path)) {
            std::cerr << "Error: native input/output contract rejects MATLAB .mat paths\n";
            return 2;
        }
        if (!options.android_raw_path.empty() &&
            (forbiddenNativeInputPath(options.android_raw_path) ||
             forbiddenNativeInputPath(options.nav_path))) {
            std::cerr << "Error: native Android input contract rejected a forbidden path\n";
            return 2;
        }
        if (!options.android_raw_path.empty()) {
            std::error_code error;
            if (!std::filesystem::is_regular_file(options.android_raw_path, error) || error ||
                !std::filesystem::is_regular_file(options.nav_path, error) || error) {
                std::cerr << "Error: native Android input contract requires regular raw/nav files\n";
                return 2;
            }
        }
        const Problem problem = buildProblem(options);
        const SolveResult result = solveProblem(problem, options);

        if (!writePerEpochCsv(options.out_csv_path, problem, result)) {
            std::cerr << "Error: failed to write velocity CSV: "
                      << options.out_csv_path << "\n";
            return 1;
        }
        if (!writeKeyedCsv(options.keyed_out_csv_path,
                           options.trip_id,
                           problem,
                           result)) {
            std::cerr << "Error: failed to atomically write keyed position CSV: "
                      << options.keyed_out_csv_path << "\n";
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
