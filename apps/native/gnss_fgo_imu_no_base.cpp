// Opt-in, no-base GNSS+IMU entry point for the smartphone research lane.
//
// This intentionally has a small surface: the existing FGO problem builder
// supplies no-base undifferenced pseudorange factors, while the GTSAM backend
// supplies the real Pose3/velocity/bias chain and CombinedImuFactor.  The
// production gnss_fgo defaults are not changed.  Android raw axes are never
// silently treated as body axes: the frozen taroz mounting rotation is
// applied explicitly after the raw adapter has converted only timestamps and
// stream alignment.

#include <libgnss++/algorithms/fgo.hpp>
#include <libgnss++/algorithms/carrier_code_leveling.hpp>
#include <libgnss++/algorithms/pdc_state_bridge.hpp>
#include <libgnss++/algorithms/upstream_position_offset.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/fusion/fusion_initialization.hpp>
#include <libgnss++/io/android_raw_gnss.hpp>
#include <libgnss++/io/imu.hpp>
#include <libgnss++/io/rinex.hpp>

#include <Eigen/Geometry>

#include <array>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <unistd.h>

namespace {

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kGpsEpochUnixSeconds = 315964800.0;
constexpr double kGpsUtcLeapSeconds = 18.0;
constexpr double kGravity = 9.80665;
constexpr int kDefaultEpochLimit = 30;
constexpr int kMinEpochLimit = 10;
constexpr int kMaxEpochLimit = 30;
constexpr std::size_t kStationarySamples = 250;
constexpr std::size_t kHeadingWindowEpochs = 25;
constexpr int kRequiredHeadingWindows = 3;
constexpr double kHeadingSpeedMinMps = 2.0;
constexpr double kHeadingSpeedMaxMps = 50.0;
constexpr double kHeadingVerticalSpeedMaxMps = 3.0;
constexpr double kHeadingConsistencyRad = 20.0 * kPi / 180.0;
constexpr double kGravityNormMin = 0.70 * kGravity;
constexpr double kGravityNormMax = 1.30 * kGravity;
constexpr double kGravityNormStdMax = 1.50;
constexpr double kUpstreamImuSyncCoefficient = 0.5;
constexpr double kNativeTdcpMaxGapS = 2.0;
constexpr double kNativeTdcpSigmaM = 0.03;
constexpr double kNativeTdcpCodePhaseJumpThresholdM = 10.0;

struct Options {
    std::string obs_path;
    std::string nav_path;
    std::string imu_path;
    std::string android_imu_path;
    std::string android_gnss_path;
    std::string out_path;
    std::string summary_path;
    std::string dataset_id = "native-fgo-v2-imu-no-base";
    int max_epochs = kDefaultEpochLimit;
    int skip_epochs = 0;
    bool all_epochs = false;
    bool android_raw_utc_key_contract = false;
    bool android_utc_wall_clock_fallback = false;
    bool fgo_imu_sparse_recovery = false;
    bool native_pdc_state_bridge = false;
    bool native_pdc_imu_tdcp = false;
    bool native_signal_bias_states = false;
    bool native_residual_ionosphere = false;
    bool native_upstream_quality = false;
    bool native_carrier_code_leveling = false;
    bool native_carrier_code_innovation_reset = false;
    bool native_carrier_code_primary_l1_e1 = false;
    bool native_carrier_code_gal_e1_e5a = false;
    bool native_upstream_stop_constraints = false;
    bool native_upstream_position_offset = false;
};

const char* carrierSignalName(libgnss::SignalType signal) {
    switch (signal) {
        case libgnss::SignalType::GPS_L1CA: return "GPS_L1CA";
        case libgnss::SignalType::GAL_E1: return "GAL_E1";
        case libgnss::SignalType::GAL_E5A: return "GAL_E5A";
        default: return "UNSUPPORTED";
    }
}

void usage(const char* program) {
    std::cout << "Usage: " << program
              << " (--obs <rover.obs> --imu <imu.csv>"
                 " | --android-gnss <device_gnss.csv> --android-imu <device_imu.csv>)"
                 " --nav <brdc.nav>"
                 " --out <submission.csv> --summary-json <summary.json>"
                 " [--dataset-id <id>] [--skip-epochs <n>]"
                 " [--max-epochs 10..30 | --all-epochs]"
                 " [--android-raw-utc-keys] [--fgo-imu-sparse-recovery]"
                 " [--android-utc-wall-clock-fallback]"
                 " [--native-pdc-state-bridge] [--native-pdc-imu-tdcp]"
                 " [--native-signal-bias-states] [--native-residual-ionosphere]"
                 " [--native-upstream-quality] [--native-carrier-code-leveling]"
                 " [--native-carrier-code-innovation-reset]"
                 " [--native-carrier-code-primary-l1-e1]"
                 " [--native-carrier-code-gal-e1-e5a]"
                 " [--native-upstream-stop-constraints]"
                 " [--native-upstream-position-offset]\n";
}

bool requireValue(int argc, char** argv, int& index, std::string& value) {
    if (index + 1 >= argc) {
        return false;
    }
    value = argv[++index];
    return !value.empty();
}

std::string phoneFromDatasetId(const std::string& dataset_id) {
    const std::size_t slash = dataset_id.find_last_of("/\\");
    return slash == std::string::npos ? dataset_id : dataset_id.substr(slash + 1U);
}

bool hasMatExtension(const std::string& path) {
    const std::size_t slash = path.find_last_of("/\\");
    const std::size_t dot = path.find_last_of('.');
    if (dot == std::string::npos || (slash != std::string::npos && dot < slash)) {
        return false;
    }
    std::string extension = path.substr(dot);
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return extension == ".mat";
}

bool parseArguments(int argc, char** argv, Options& options) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            usage(argv[0]);
            return false;
        }
        if (arg == "--obs") {
            if (!requireValue(argc, argv, i, options.obs_path)) return false;
        } else if (arg == "--nav") {
            if (!requireValue(argc, argv, i, options.nav_path)) return false;
        } else if (arg == "--imu") {
            if (!requireValue(argc, argv, i, options.imu_path)) return false;
        } else if (arg == "--android-imu") {
            if (!requireValue(argc, argv, i, options.android_imu_path)) return false;
        } else if (arg == "--android-gnss") {
            if (!requireValue(argc, argv, i, options.android_gnss_path)) return false;
        } else if (arg == "--out") {
            if (!requireValue(argc, argv, i, options.out_path)) return false;
        } else if (arg == "--summary-json") {
            if (!requireValue(argc, argv, i, options.summary_path)) return false;
        } else if (arg == "--dataset-id") {
            if (!requireValue(argc, argv, i, options.dataset_id)) return false;
        } else if (arg == "--max-epochs") {
            std::string value;
            if (!requireValue(argc, argv, i, value)) return false;
            try {
                std::size_t consumed = 0;
                options.max_epochs = std::stoi(value, &consumed);
                if (consumed != value.size()) return false;
            } catch (...) {
                return false;
            }
            if (options.max_epochs < kMinEpochLimit || options.max_epochs > kMaxEpochLimit) {
                std::cerr << "--max-epochs must be between " << kMinEpochLimit << " and "
                          << kMaxEpochLimit << " for the frozen short-window contract\n";
                return false;
            }
        } else if (arg == "--skip-epochs") {
            std::string value;
            if (!requireValue(argc, argv, i, value)) return false;
            try {
                std::size_t consumed = 0;
                options.skip_epochs = std::stoi(value, &consumed);
                if (consumed != value.size() || options.skip_epochs < 0) return false;
            } catch (...) {
                return false;
            }
        } else if (arg == "--all-epochs") {
            options.all_epochs = true;
        } else if (arg == "--android-raw-utc-keys") {
            options.android_raw_utc_key_contract = true;
        } else if (arg == "--android-utc-wall-clock-fallback") {
            options.android_utc_wall_clock_fallback = true;
        } else if (arg == "--fgo-imu-sparse-recovery") {
            options.fgo_imu_sparse_recovery = true;
        } else if (arg == "--native-pdc-state-bridge") {
            options.native_pdc_state_bridge = true;
        } else if (arg == "--native-pdc-imu-tdcp") {
            options.native_pdc_imu_tdcp = true;
        } else if (arg == "--native-signal-bias-states") {
            options.native_signal_bias_states = true;
        } else if (arg == "--native-residual-ionosphere") {
            options.native_residual_ionosphere = true;
        } else if (arg == "--native-upstream-quality") {
            options.native_upstream_quality = true;
        } else if (arg == "--native-carrier-code-leveling") {
            options.native_carrier_code_leveling = true;
        } else if (arg == "--native-carrier-code-innovation-reset") {
            options.native_carrier_code_innovation_reset = true;
        } else if (arg == "--native-carrier-code-primary-l1-e1") {
            options.native_carrier_code_primary_l1_e1 = true;
        } else if (arg == "--native-carrier-code-gal-e1-e5a") {
            options.native_carrier_code_gal_e1_e5a = true;
        } else if (arg == "--native-upstream-stop-constraints") {
            options.native_upstream_stop_constraints = true;
        } else if (arg == "--native-upstream-position-offset") {
            options.native_upstream_position_offset = true;
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            return false;
        }
    }
    const bool android_raw = !options.android_imu_path.empty();
    const bool has_observation_file = !options.obs_path.empty();
    if (options.nav_path.empty() ||
        (options.imu_path.empty() == options.android_imu_path.empty()) ||
        (options.android_imu_path.empty() != options.android_gnss_path.empty()) ||
        (android_raw == has_observation_file) ||
        options.out_path.empty() || options.summary_path.empty()) {
        usage(argv[0]);
        return false;
    }
    const std::array<const std::string*, 6> file_paths = {
        &options.obs_path, &options.nav_path, &options.imu_path,
        &options.android_imu_path, &options.android_gnss_path, &options.out_path};
    for (const std::string* path : file_paths) {
        if (path != nullptr && !path->empty() && hasMatExtension(*path)) {
            std::cerr << "MATLAB .mat paths are forbidden by the native/raw contract\n";
            return false;
        }
    }
    if (hasMatExtension(options.summary_path)) {
        std::cerr << "MATLAB .mat paths are forbidden by the native/raw contract\n";
        return false;
    }
    if (options.native_pdc_imu_tdcp) {
        // The TDCP candidate is deliberately a single raw-only recipe.  It
        // reuses the in-process PDC state bridge and therefore cannot be
        // accidentally invoked on a RINEX/precomputed lane.
        options.native_pdc_state_bridge = true;
    }
    if (options.android_raw_utc_key_contract && !android_raw) {
        std::cerr << "--android-raw-utc-keys requires the Android raw GNSS/IMU path\n";
        return false;
    }
    if (options.android_raw_utc_key_contract &&
        (!options.all_epochs || options.skip_epochs != 0)) {
        std::cerr << "--android-raw-utc-keys requires --all-epochs without --skip-epochs\n";
        return false;
    }
    if (options.android_utc_wall_clock_fallback &&
        (!android_raw || !options.android_raw_utc_key_contract ||
         !options.all_epochs || options.skip_epochs != 0)) {
        std::cerr << "--android-utc-wall-clock-fallback requires Android raw input, "
                     "--android-raw-utc-keys, --all-epochs, and no skipped epochs\n";
        return false;
    }
    if (options.fgo_imu_sparse_recovery &&
        (!android_raw || !options.android_raw_utc_key_contract)) {
        std::cerr << "--fgo-imu-sparse-recovery requires Android raw input and "
                     "--android-raw-utc-keys\n";
        return false;
    }
    if (options.native_pdc_state_bridge &&
        (!android_raw || !options.android_raw_utc_key_contract ||
         !options.all_epochs || options.skip_epochs != 0)) {
        std::cerr << "--native-pdc-state-bridge requires Android raw input, "
                     "--android-raw-utc-keys, --all-epochs, and no skipped epochs\n";
        return false;
    }
    if (options.native_pdc_imu_tdcp &&
        (!android_raw || !options.android_raw_utc_key_contract ||
         !options.all_epochs || options.skip_epochs != 0)) {
        std::cerr << "--native-pdc-imu-tdcp requires Android raw input, "
                     "--android-raw-utc-keys, --all-epochs, and no skipped epochs\n";
        return false;
    }
    if (options.native_signal_bias_states && !android_raw) {
        std::cerr << "--native-signal-bias-states requires Android raw GNSS/IMU input\n";
        return false;
    }
    if (options.native_residual_ionosphere &&
        (!android_raw || !options.android_raw_utc_key_contract ||
         !options.all_epochs || options.skip_epochs != 0)) {
        std::cerr << "--native-residual-ionosphere requires Android raw input, "
                     "--android-raw-utc-keys, --all-epochs, and no skipped epochs\n";
        return false;
    }
    if (options.native_residual_ionosphere &&
        (!options.native_signal_bias_states || !options.native_pdc_imu_tdcp)) {
        std::cerr << "--native-residual-ionosphere requires the frozen "
                     "--native-signal-bias-states and --native-pdc-imu-tdcp base recipe\n";
        return false;
    }
    if (options.native_upstream_quality &&
        (!android_raw || !options.android_raw_utc_key_contract ||
         !options.all_epochs || options.skip_epochs != 0 ||
         !options.native_residual_ionosphere ||
         !options.native_signal_bias_states || !options.native_pdc_imu_tdcp)) {
        std::cerr << "--native-upstream-quality requires the frozen Android raw "
                     "Phase12 base recipe and all epochs\n";
        return false;
    }
    if (options.native_carrier_code_leveling &&
        (!android_raw || !options.android_raw_utc_key_contract ||
         !options.all_epochs || options.skip_epochs != 0 ||
         !options.native_residual_ionosphere ||
         !options.native_signal_bias_states || !options.native_pdc_imu_tdcp ||
         options.native_upstream_quality)) {
        std::cerr << "--native-carrier-code-leveling requires the frozen Android "
                     "Phase12 base recipe, all epochs, and Phase13 off\n";
        return false;
    }
    if (options.native_carrier_code_innovation_reset &&
        !options.native_carrier_code_leveling) {
        std::cerr << "--native-carrier-code-innovation-reset requires "
                     "--native-carrier-code-leveling\n";
        return false;
    }
    if (options.native_carrier_code_primary_l1_e1 &&
        (!options.native_carrier_code_leveling ||
         !options.native_carrier_code_innovation_reset)) {
        std::cerr << "--native-carrier-code-primary-l1-e1 requires "
                     "--native-carrier-code-leveling and "
                     "--native-carrier-code-innovation-reset\n";
        return false;
    }
    if (options.native_carrier_code_gal_e1_e5a &&
        (!options.native_carrier_code_leveling ||
         !options.native_carrier_code_innovation_reset)) {
        std::cerr << "--native-carrier-code-gal-e1-e5a requires "
                     "--native-carrier-code-leveling and "
                     "--native-carrier-code-innovation-reset\n";
        return false;
    }
    if (options.native_upstream_stop_constraints &&
        (!android_raw || !options.android_raw_utc_key_contract ||
         !options.all_epochs || options.skip_epochs != 0)) {
        std::cerr << "--native-upstream-stop-constraints requires Android raw input, "
                     "--android-raw-utc-keys, --all-epochs, and no skipped epochs\n";
        return false;
    }
    if (options.native_upstream_position_offset &&
        (!android_raw || !options.android_raw_utc_key_contract ||
         !options.all_epochs || options.skip_epochs != 0)) {
        std::cerr << "--native-upstream-position-offset requires Android raw input, "
                     "--android-raw-utc-keys, --all-epochs, and no skipped epochs\n";
        return false;
    }
    if (options.native_carrier_code_primary_l1_e1 &&
        options.native_carrier_code_gal_e1_e5a) {
        std::cerr << "--native-carrier-code-primary-l1-e1 and "
                     "--native-carrier-code-gal-e1-e5a are mutually exclusive; "
                     "Phase 19 excludes GPS L1\n";
        return false;
    }
    return true;
}

bool atomicWrite(const std::string& path, const std::string& contents) {
    const std::filesystem::path destination(path);
    if (!destination.parent_path().empty()) {
        std::error_code error;
        std::filesystem::create_directories(destination.parent_path(), error);
        if (error) return false;
    }
    const std::string temporary = path + ".tmp." + std::to_string(static_cast<long long>(::getpid()));
    {
        std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
        if (!output.is_open()) return false;
        output << contents;
        output.flush();
        if (!output.good()) return false;
    }
    if (std::rename(temporary.c_str(), path.c_str()) != 0) {
        std::remove(temporary.c_str());
        return false;
    }
    return true;
}

double unixMillis(const libgnss::GNSSTime& time) {
    const double seconds = kGpsEpochUnixSeconds +
                           static_cast<double>(time.week) * 604800.0 + time.tow -
                           kGpsUtcLeapSeconds;
    return std::round(seconds * 1000.0);
}

Eigen::Matrix3d tarozMountingRotation() {
    // Exact frozen translation of gtsam.Rot3.RzRyRx(deg2rad([-85 178 -94]')):
    // Rz(-94 deg) * Ry(178 deg) * Rx(-85 deg).  Applying this matrix to the
    // Android sensor vector is equivalent to using the upstream body_P_sensor
    // rotation while keeping the native backend's body-FLU input contract.
    const double rx = -85.0 / kRadToDeg;
    const double ry = 178.0 / kRadToDeg;
    const double rz = -94.0 / kRadToDeg;
    return (Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

Eigen::Matrix3d rpyToBodyToNav(const Eigen::Vector3d& rpy_rad) {
    return (Eigen::AngleAxisd(rpy_rad.z(), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rpy_rad.y(), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rpy_rad.x(), Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

bool deriveGnssFirstVelocities(
    const libgnss::FGOProcessor::FGOProblem& problem,
    const libgnss::FGOProcessor::FGOResult& gnss_first_result,
    std::vector<libgnss::Vector3d>& velocities_enu,
    std::string& error) {
    const auto& solutions = gnss_first_result.solution.solutions;
    const auto& optimized_velocities = gnss_first_result.epoch_velocities_ecef_mps;
    if (problem.epochs.size() < 2 || solutions.size() != problem.epochs.size() ||
        optimized_velocities.size() != problem.epochs.size()) {
        error = "GNSS-first optimized velocity state count does not match observation epochs";
        return false;
    }
    const libgnss::Vector3d origin = solutions.front().position_ecef;
    if (!origin.allFinite() || origin.norm() < 1.0e6) {
        error = "GNSS-first solution has invalid ENU origin";
        return false;
    }
    double lat = 0.0;
    double lon = 0.0;
    double height = 0.0;
    libgnss::ecef2geodetic(origin, lat, lon, height);
    if (!std::isfinite(lat) || !std::isfinite(lon) || !std::isfinite(height)) {
        error = "GNSS-first solution has invalid geodetic origin";
        return false;
    }
    velocities_enu.resize(solutions.size());
    for (std::size_t i = 0; i < solutions.size(); ++i) {
        if (!solutions[i].position_ecef.allFinite()) {
            error = "GNSS-first solution contains non-finite position";
            return false;
        }
        // The upstream GNSS pass optimizes a velocity state from P+D.  Use
        // that state directly; a position-difference proxy would hide a
        // missing Doppler graph and is therefore not accepted by this mode.
        // Convert the ECEF velocity with the same linear ENU basis as the
        // position frame.
        const libgnss::Vector3d velocity_enu = libgnss::ecef2enu(
            optimized_velocities[i], lat, lon);
        if (!optimized_velocities[i].allFinite() || !velocity_enu.allFinite()) {
            error = "GNSS-first optimized velocity state is non-finite";
            return false;
        }
        velocities_enu[i] = velocity_enu;
    }
    return true;
}

struct ImuBuildReport {
    bool ok = false;
    bool heading_latched = false;
    std::string heading_initialization_mode;
    std::size_t velocity_heading_low_speed_count = 0;
    std::size_t velocity_heading_linear_fill_count = 0;
    std::size_t velocity_heading_nearest_fill_count = 0;
    bool gnss_first_attempted = false;
    bool gnss_first_converged = false;
    std::size_t gnss_first_epochs = 0;
    std::size_t gnss_first_doppler_factors = 0;
    std::size_t gnss_first_velocity_states = 0;
    int gnss_first_iterations = 0;
    double gnss_first_initial_cost = 0.0;
    double gnss_first_final_cost = 0.0;
    std::string gnss_first_failure;
    std::string failure;
    std::size_t loaded_samples = 0;
    std::size_t stationary_samples = 0;
    double gravity_mean_norm = 0.0;
    double gravity_norm_std = 0.0;
    int heading_windows = 0;
    bool android_raw = false;
    libgnss::io::AndroidRawGnssDiagnostics android_gnss_diagnostics;
    libgnss::AndroidImuCsvLoadResult android_load;
    libgnss::AndroidGnssTimeAnchorLoadResult android_gnss_anchor_load;
    libgnss::AndroidGnssUtcGpsMappingLoadResult android_gnss_utc_mapping_load;
};

struct RawUtcOutputReport {
    bool enabled = false;
    bool warmup_epoch_excluded = false;
    std::size_t raw_epoch_keys = 0;
    std::size_t target_epochs = 0;
    std::size_t exact_solution_epochs = 0;
    std::size_t interpolated_epochs = 0;
    std::size_t edge_hold_epochs = 0;
    std::size_t unresolved_epochs = 0;
    double solution_time_tolerance_ms = 0.0;
    double max_interpolation_gap_ms = 0.0;
    double max_edge_hold_gap_ms = 0.0;
};

struct NativePdcBridgeReport {
    bool enabled = false;
    bool ok = false;
    std::size_t epochs = 0;
    std::size_t pseudorange_rows = 0;
    std::size_t doppler_rows = 0;
    std::size_t motion_intervals = 0;
    std::size_t valid_position_states = 0;
    std::size_t rejected_position_states = 0;
    std::size_t valid_velocity_states = 0;
    std::size_t state_seeds = 0;
    int iterations = 0;
    double initial_cost = 0.0;
    double final_cost = 0.0;
    double max_velocity_norm_mps = 0.0;
    double max_clock_rate_abs_mps = 0.0;
    double max_normalized_rms = 0.0;
    std::size_t fgo_seed_displacement_count = 0;
    double fgo_seed_displacement_p50_m = 0.0;
    double fgo_seed_displacement_max_m = 0.0;
    std::string failure;
};

struct TdcpRuntimeReport {
    bool enabled = false;
    std::size_t factors_built = 0;
    std::size_t factors_inserted = 0;
    std::size_t candidate_pairs = 0;
    std::size_t rejected_gap = 0;
    std::size_t rejected_clock_discontinuity = 0;
    std::size_t rejected_missing_previous = 0;
    std::size_t rejected_loss_of_lock = 0;
    std::size_t rejected_invalid_measurement = 0;
    std::size_t rejected_code_phase_jump = 0;
    std::size_t finite_residuals = 0;
    std::size_t nonfinite_residuals = 0;
    std::size_t arc_count = 0;
    std::size_t min_arc_length_epochs = 0;
    std::size_t max_arc_length_epochs = 0;
    double median_arc_length_epochs = 0.0;
    double sigma_m = kNativeTdcpSigmaM;
    double max_gap_s = kNativeTdcpMaxGapS;
    double code_phase_jump_threshold_m = kNativeTdcpCodePhaseJumpThresholdM;
    double residual_rms_m = 0.0;
    double normalized_residual_rms = 0.0;
    double max_abs_residual_m = 0.0;
};

struct OutputPosition {
    std::int64_t utc_time_millis = 0;
    libgnss::Vector3d position_ecef = libgnss::Vector3d::Zero();
};

struct UpstreamPositionOffsetReport {
    bool enabled = false;
    bool applied = false;
    std::string phone;
    std::size_t corrected_epochs = 0U;
    double offset_rl_m = 0.0;
    double offset_ud_m = 0.0;
    double max_offset_enu_m = 0.0;
    std::string failure;
};

TdcpRuntimeReport evaluateTdcpRuntime(
    const libgnss::FGOProcessor::FGOProblem& problem,
    const libgnss::FGOProcessor::FGOResult& result,
    bool enabled) {
    TdcpRuntimeReport report;
    report.enabled = enabled;
    if (!enabled) return report;
    report.factors_built = problem.tdcp_factors.size();
    report.factors_inserted = result.diagnostics.tdcp_factors_inserted;
    report.candidate_pairs = problem.diagnostics.tdcp_candidate_pairs;
    report.rejected_gap = problem.diagnostics.tdcp_rejected_gap;
    report.rejected_clock_discontinuity =
        problem.diagnostics.tdcp_rejected_clock_discontinuity;
    report.rejected_missing_previous =
        problem.diagnostics.tdcp_rejected_missing_previous;
    report.rejected_loss_of_lock = problem.diagnostics.tdcp_rejected_loss_of_lock;
    report.rejected_invalid_measurement =
        problem.diagnostics.tdcp_rejected_invalid_measurement;
    report.rejected_code_phase_jump =
        problem.diagnostics.tdcp_rejected_code_phase_jump;

    using CarrierKey = std::pair<libgnss::SatelliteId, libgnss::SignalType>;
    struct ArcState {
        std::size_t last_current_epoch = 0;
        std::size_t length_epochs = 0;
    };
    std::map<CarrierKey, ArcState> active_arcs;
    std::vector<std::size_t> arc_lengths;
    arc_lengths.reserve(problem.tdcp_factors.size());
    double sum_squared = 0.0;
    double sum_normalized_squared = 0.0;
    for (const auto& factor : problem.tdcp_factors) {
        const CarrierKey key{factor.satellite, factor.signal};
        auto arc_it = active_arcs.find(key);
        if (arc_it == active_arcs.end() ||
            factor.previous_epoch_index != arc_it->second.last_current_epoch) {
            if (arc_it != active_arcs.end()) {
                arc_lengths.push_back(arc_it->second.length_epochs);
            }
            active_arcs[key] = {factor.current_epoch_index, 2U};
        } else {
            arc_it->second.last_current_epoch = factor.current_epoch_index;
            ++arc_it->second.length_epochs;
        }

        const bool indices_valid =
            factor.previous_epoch_index < result.solution.solutions.size() &&
            factor.current_epoch_index < result.solution.solutions.size();
        if (!indices_valid) {
            ++report.nonfinite_residuals;
            continue;
        }
        const auto& previous =
            result.solution.solutions[factor.previous_epoch_index];
        const auto& current = result.solution.solutions[factor.current_epoch_index];
        const double previous_range =
            (factor.previous_satellite_position_ecef - previous.position_ecef).norm();
        const double current_range =
            (factor.current_satellite_position_ecef - current.position_ecef).norm();
        const double residual =
            current_range + libgnss::constants::SPEED_OF_LIGHT * current.receiver_clock_bias -
            previous_range - libgnss::constants::SPEED_OF_LIGHT * previous.receiver_clock_bias -
            factor.delta_carrier_m;
        if (!std::isfinite(residual) || !(factor.sigma_m > 0.0) ||
            !std::isfinite(factor.sigma_m)) {
            ++report.nonfinite_residuals;
            continue;
        }
        ++report.finite_residuals;
        sum_squared += residual * residual;
        const double normalized = residual / factor.sigma_m;
        sum_normalized_squared += normalized * normalized;
        report.max_abs_residual_m =
            std::max(report.max_abs_residual_m, std::abs(residual));
        if (report.finite_residuals == 1U) report.sigma_m = factor.sigma_m;
    }
    for (const auto& [key, arc] : active_arcs) {
        (void)key;
        arc_lengths.push_back(arc.length_epochs);
    }
    if (!arc_lengths.empty()) {
        std::sort(arc_lengths.begin(), arc_lengths.end());
        report.arc_count = arc_lengths.size();
        report.min_arc_length_epochs = arc_lengths.front();
        report.max_arc_length_epochs = arc_lengths.back();
        const std::size_t middle = arc_lengths.size() / 2U;
        report.median_arc_length_epochs =
            arc_lengths.size() % 2U == 0U
                ? 0.5 * static_cast<double>(arc_lengths[middle - 1U] +
                                             arc_lengths[middle])
                : static_cast<double>(arc_lengths[middle]);
    }
    if (report.finite_residuals > 0U) {
        report.residual_rms_m =
            std::sqrt(sum_squared / static_cast<double>(report.finite_residuals));
        report.normalized_residual_rms = std::sqrt(
            sum_normalized_squared / static_cast<double>(report.finite_residuals));
    }
    return report;
}

bool populateNativePdcStateBridge(
    libgnss::FGOProcessor::FGOProblem& problem,
    NativePdcBridgeReport& report) {
    report.enabled = true;
    report.epochs = problem.epochs.size();
    report.pseudorange_rows = problem.pseudorange_factors.size();
    report.doppler_rows = problem.undifferenced_doppler_factors.size();
    if (problem.epochs.empty() || problem.pseudorange_factors.empty()) {
        report.failure = "native PDC bridge has no pseudorange rows";
        return false;
    }

    // buildPseudorangeProblem historically carries a fresh SPP clock in the
    // receiver_clock_bias_m field as seconds, while all FGO state equations
    // use metres.  The marker makes this candidate-only normalization
    // explicit and prevents magnitude-based unit guesses.
    for (auto& epoch : problem.epochs) {
        if (!epoch.receiver_clock_bias_is_meters) {
            if (!std::isfinite(epoch.receiver_clock_bias_m)) {
                report.failure = "non-finite SPP clock seed";
                return false;
            }
            epoch.receiver_clock_bias_m *= libgnss::constants::SPEED_OF_LIGHT;
            epoch.receiver_clock_bias_is_meters = true;
        }
    }

    std::vector<libgnss::pdc_state_bridge::EpochInput> epochs;
    epochs.reserve(problem.epochs.size());
    for (std::size_t epoch_index = 0; epoch_index < problem.epochs.size();
         ++epoch_index) {
        const auto& source = problem.epochs[epoch_index];
        if (!source.position_ecef.allFinite() ||
            !std::isfinite(source.receiver_clock_bias_m)) {
            report.failure = "non-finite PDC epoch seed";
            return false;
        }
        epochs.push_back({source.time,
                          source.position_ecef,
                          source.receiver_clock_bias_m,
                          epoch_index < problem.clock_jumps.size()
                              ? problem.clock_jumps[epoch_index]
                              : false});
    }

    std::vector<libgnss::pdc_state_bridge::PseudorangeRow> pseudorange_rows;
    pseudorange_rows.reserve(problem.pseudorange_factors.size());
    for (const auto& factor : problem.pseudorange_factors) {
        if (factor.epoch_index >= epochs.size()) {
            report.failure = "PDC row epoch index out of range";
            return false;
        }
        pseudorange_rows.push_back({factor.epoch_index,
                                    factor.satellite,
                                    factor.clock_group,
                                    factor.satellite_position_ecef,
                                    factor.corrected_pseudorange_m,
                                    factor.sigma_m});
    }
    std::vector<libgnss::pdc_state_bridge::DopplerRow> doppler_rows;
    doppler_rows.reserve(problem.undifferenced_doppler_factors.size());
    for (const auto& factor : problem.undifferenced_doppler_factors) {
        if (factor.epoch_index >= epochs.size()) {
            report.failure = "PDC Doppler row epoch index out of range";
            return false;
        }
        doppler_rows.push_back({factor.epoch_index,
                                factor.los,
                                factor.residual_mps,
                                factor.sigma_mps});
    }

    libgnss::pdc_state_bridge::Options bridge_options;
    // Match the dedicated native PDC recipe. These values are physical/config
    // defaults fixed before truth; this bridge does not learn from the route.
    bridge_options.max_iterations = 1000;
    bridge_options.min_pseudorange_rows = 4;
    bridge_options.min_doppler_rows = 4;
    bridge_options.pseudorange_huber_threshold_sigma = 1.234;
    bridge_options.doppler_huber_threshold_sigma = 1.234;
    bridge_options.position_prior_sigma_m = 1000.0;
    bridge_options.clock_prior_sigma_m = 1.0e6;
    bridge_options.velocity_prior_sigma_mps = 1000.0;
    bridge_options.clock_rate_prior_sigma_mps = 1000.0;
    bridge_options.motion_sigma_m = 0.1;
    bridge_options.clock_motion_sigma_m = 0.1;
    bridge_options.clock_jump_sigma_m = 1.0e6;
    bridge_options.inter_system_clock_motion_sigma_m = 1.0e-6;
    bridge_options.clock_rate_between_sigma_mps = 0.1;
    bridge_options.max_velocity_mps = 70.0;
    bridge_options.max_clock_rate_mps = 2000.0;
    bridge_options.max_normalized_rms = 25.0;
    bridge_options.max_position_norm_m = 1.0e7;

    const auto solve = libgnss::pdc_state_bridge::solve(
        epochs, pseudorange_rows, doppler_rows, bridge_options);
    report.motion_intervals = solve.motion_intervals;
    report.iterations = solve.iterations;
    report.initial_cost = solve.initial_cost;
    report.final_cost = solve.final_cost;
    report.max_velocity_norm_mps = solve.max_velocity_norm_mps;
    report.max_clock_rate_abs_mps = solve.max_clock_rate_abs_mps;
    for (const auto& estimate : solve.epochs) {
        if (estimate.valid) {
            ++report.valid_position_states;
            ++report.valid_velocity_states;
        } else {
            ++report.rejected_position_states;
        }
        if (std::isfinite(estimate.normalized_rms)) {
            report.max_normalized_rms =
                std::max(report.max_normalized_rms, estimate.normalized_rms);
        }
    }
    problem.native_pdc_state_seeds.clear();
    problem.native_pdc_state_seeds.reserve(solve.epochs.size());
    for (std::size_t epoch_index = 0; epoch_index < solve.epochs.size();
         ++epoch_index) {
        const auto& estimate = solve.epochs[epoch_index];
        if (!estimate.valid) continue;
        libgnss::FGOProcessor::NativePdcStateSeed seed;
        seed.epoch_index = epoch_index;
        seed.position_ecef = estimate.state.position_ecef;
        seed.velocity_ecef_mps = estimate.state.velocity_ecef_mps;
        seed.clock_bias_m = estimate.state.clock_bias_m;
        seed.clock_rate_mps = estimate.state.clock_rate_mps;
        seed.pseudorange_rows = estimate.pseudorange_rows;
        seed.doppler_rows = estimate.doppler_rows;
        seed.normalized_pseudorange_rms = estimate.normalized_rms;
        seed.has_position = seed.position_ecef.allFinite();
        seed.has_velocity = seed.velocity_ecef_mps.allFinite();
        seed.has_clock = std::isfinite(seed.clock_bias_m[0]);
        seed.has_clock_rate = std::isfinite(seed.clock_rate_mps);
        if (seed.has_position || seed.has_velocity) {
            problem.native_pdc_state_seeds.push_back(seed);
        }
    }
    report.state_seeds = problem.native_pdc_state_seeds.size();
    report.ok = solve.valid && report.state_seeds > 0;
    if (!report.ok) {
        std::ostringstream failure;
        failure << (solve.reason.empty() ? "native PDC state solve failed"
                                         : solve.reason)
                << "; converged=" << (solve.converged ? "true" : "false")
                << "; iterations=" << solve.iterations
                << "; valid_epochs=" << solve.valid_epochs
                << "; final_cost=" << solve.final_cost
                << "; max_velocity_mps=" << solve.max_velocity_norm_mps
                << "; max_clock_rate_mps=" << solve.max_clock_rate_abs_mps
                << "; max_normalized_rms=" << report.max_normalized_rms;
        if (!solve.epochs.empty()) {
            const auto& first = solve.epochs.front();
            double initial_sum_squared = 0.0;
            double initial_max_abs = 0.0;
            int initial_count = 0;
            for (const auto& row : pseudorange_rows) {
                if (row.epoch_index != 0) continue;
                const double residual =
                    (row.satellite_position_ecef - epochs.front().seed_position_ecef).norm() +
                    epochs.front().seed_clock_bias_m - row.corrected_pseudorange_m;
                if (std::isfinite(residual)) {
                    initial_sum_squared += residual * residual;
                    initial_max_abs = std::max(initial_max_abs, std::abs(residual));
                    ++initial_count;
                }
            }
            failure << "; first_reason=" << first.reason
                    << "; first_p_rows=" << first.pseudorange_rows
                    << "; first_d_rows=" << first.doppler_rows
                    << "; first_pos_norm=" << first.state.position_ecef.norm()
                    << "; first_pos_seed_norm="
                    << epochs.front().seed_position_ecef.norm()
                    << "; first_clock_m=" << first.state.clock_bias_m[0]
                    << "; first_vel_norm="
                    << first.state.velocity_ecef_mps.norm()
                    << "; seed_clock_m=" << epochs.front().seed_clock_bias_m
                    << "; first_initial_p_rms_m="
                    << (initial_count > 0
                            ? std::sqrt(initial_sum_squared /
                                       static_cast<double>(initial_count))
                            : std::numeric_limits<double>::quiet_NaN())
                    << "; first_initial_p_max_m=" << initial_max_abs;
        }
        report.failure = failure.str();
    }
    return report.ok;
}

bool buildImuInput(const std::string& path,
                   libgnss::FGOProcessor::FGOProblem& problem,
                   ImuBuildReport& report,
                   bool android_raw = false,
                   const std::vector<libgnss::AndroidGnssTimeAnchor>& gnss_time_anchors = {},
                   const std::vector<libgnss::Vector3d>* gnss_first_velocities_enu = nullptr,
                   const libgnss::AndroidGnssUtcGpsMapping* utc_gps_mapping = nullptr) {
    if (problem.epochs.size() < 2) {
        report.failure = "fewer than two GNSS epochs";
        return false;
    }

    libgnss::ImuSeries series;
    report.android_raw = android_raw;
    if (android_raw) {
        libgnss::AndroidImuCsvConfig android_config;
        android_config.require_gnss_elapsed_anchor = true;
        android_config.allow_utc_wall_clock_fallback = utc_gps_mapping != nullptr;
        android_config.imu_sync_coefficient = kUpstreamImuSyncCoefficient;
        report.android_load = libgnss::loadAndroidImuCsv(
            path, series, android_config, gnss_time_anchors, utc_gps_mapping);
        if (!report.android_load.ok || series.isEmpty()) {
            report.failure = report.android_load.error.empty()
                                 ? "empty Android IMU series"
                                 : report.android_load.error;
            return false;
        }
    } else {
        const libgnss::ImuCsvLoadResult processed_load =
            libgnss::loadImuCsv(path, series);
        if (!processed_load.ok || series.isEmpty()) {
            report.failure = processed_load.error.empty() ? "empty IMU series"
                                                           : processed_load.error;
            return false;
        }
    }
    series.sortByTime();
    const Eigen::Matrix3d mounting = tarozMountingRotation();
    std::vector<libgnss::ImuSample> samples;
    samples.reserve(series.samples.size());
    for (auto sample : series.samples) {
        const Eigen::Vector3d accel = mounting * sample.accel_raw;
        const Eigen::Vector3d gyro = mounting * sample.gyro_raw_radps;
        if (!accel.allFinite() || !gyro.allFinite()) {
            report.failure = "non-finite IMU sample";
            return false;
        }
        sample.accel_raw = accel;
        sample.gyro_raw_radps = gyro;
        samples.push_back(sample);
    }
    if (samples.size() < kStationarySamples) {
        report.failure = "IMU stream shorter than frozen leveling window";
        return false;
    }
    report.loaded_samples = samples.size();

    const std::size_t stationary_count = std::min(kStationarySamples, samples.size());
    std::vector<libgnss::ImuSample> stationary(samples.begin(),
                                                samples.begin() + stationary_count);
    Eigen::Vector3d accel_sum = Eigen::Vector3d::Zero();
    std::vector<double> norms;
    norms.reserve(stationary.size());
    for (const auto& sample : stationary) {
        accel_sum += sample.accel_raw;
        norms.push_back(sample.accel_raw.norm());
    }
    const double n = static_cast<double>(stationary.size());
    const Eigen::Vector3d accel_mean = accel_sum / n;
    const double mean_norm = accel_mean.norm();
    double variance = 0.0;
    for (double norm : norms) variance += (norm - mean_norm) * (norm - mean_norm);
    const double norm_std = std::sqrt(variance / n);
    report.stationary_samples = stationary.size();
    report.gravity_mean_norm = mean_norm;
    report.gravity_norm_std = norm_std;
    if (!std::isfinite(mean_norm) || !std::isfinite(norm_std) ||
        mean_norm < kGravityNormMin || mean_norm > kGravityNormMax ||
        norm_std > kGravityNormStdMax) {
        report.failure = "leveling window is not stationary/low-dynamics under frozen gravity gate";
        return false;
    }

    const libgnss::Vector3d origin_ecef = problem.epochs.front().position_ecef;
    double lat = 0.0, lon = 0.0, height = 0.0;
    libgnss::ecef2geodetic(origin_ecef, lat, lon, height);
    if (!std::isfinite(lat) || !std::isfinite(lon) || !origin_ecef.allFinite()) {
        report.failure = "invalid GNSS nav origin";
        return false;
    }

    const libgnss::NominalState aligned = libgnss::fusion_initialization::alignStatic(
        stationary, libgnss::Vector3d::Zero(), kGravity);
    libgnss::FusionState state;
    state.nominal = aligned;
    state.covariance.setIdentity();
    libgnss::Vector3d initial_velocity = libgnss::Vector3d::Zero();
    if (android_raw) {
        // The upstream Android pass obtains attitude from the GNSS-only
        // velocity sequence (`vel2rpy.m`) before constructing the IMU graph.
        // Do not reuse the legacy multi-window latch here: it can reject a
        // perfectly usable route before the joint optimizer gets a chance to
        // estimate yaw.  The GNSS-first sequence itself is truth-free and is
        // required by the raw contract, so a missing/invalid sequence fails
        // closed instead of silently falling back to a guessed heading.
        if (gnss_first_velocities_enu == nullptr ||
            gnss_first_velocities_enu->size() != problem.epochs.size()) {
            report.failure = "Android upstream mode requires GNSS-first velocity sequence";
            return false;
        }
        const auto velocity_heading = libgnss::fusion_initialization::velocityToRpy(
            *gnss_first_velocities_enu);
        if (!velocity_heading.ok || velocity_heading.rpy_rad.empty() ||
            velocity_heading.smoothed_velocity_enu.size() != problem.epochs.size()) {
            report.failure = velocity_heading.error.empty()
                                 ? "GNSS-first velocity heading initialization failed"
                                 : velocity_heading.error;
            return false;
        }
        report.heading_initialization_mode = "upstream-vel2rpy";
        report.velocity_heading_low_speed_count = velocity_heading.low_speed_count;
        report.velocity_heading_linear_fill_count = velocity_heading.linear_fill_count;
        report.velocity_heading_nearest_fill_count = velocity_heading.nearest_fill_count;
        state.nominal.attitude_body_to_enu = Eigen::Quaterniond(
            rpyToBodyToNav(velocity_heading.rpy_rad.front()));
        initial_velocity = velocity_heading.smoothed_velocity_enu.front();
    } else {
        libgnss::Vector3d previous_velocity = libgnss::Vector3d::Zero();
        libgnss::Vector3d velocity_sum = libgnss::Vector3d::Zero();
        int consistent = 0;
        int windows = 0;
        for (std::size_t i = 0; i + kHeadingWindowEpochs < problem.epochs.size(); ++i) {
            const std::size_t j = i + kHeadingWindowEpochs;
            const double dt = problem.epochs[j].time - problem.epochs[i].time;
            if (dt <= 1e-3) continue;
            const Eigen::Vector3d p0 = libgnss::ecef2enu(
                problem.epochs[i].position_ecef - origin_ecef, lat, lon);
            const Eigen::Vector3d p1 = libgnss::ecef2enu(
                problem.epochs[j].position_ecef - origin_ecef, lat, lon);
            const libgnss::Vector3d velocity = (p1 - p0) / dt;
            const double speed = std::hypot(velocity.x(), velocity.y());
            if (!std::isfinite(speed) || speed < kHeadingSpeedMinMps ||
                speed > kHeadingSpeedMaxMps || std::abs(velocity.z()) > kHeadingVerticalSpeedMaxMps) {
                consistent = 0;
                velocity_sum.setZero();
                continue;
            }
            ++windows;
            bool direction_consistent = true;
            if (consistent > 0) {
                const double previous_speed = std::hypot(previous_velocity.x(), previous_velocity.y());
                const double cosine = velocity.x() * previous_velocity.x() +
                                      velocity.y() * previous_velocity.y();
                direction_consistent = cosine /
                    std::max(1e-9, speed * previous_speed) >= std::cos(kHeadingConsistencyRad);
            }
            if (!direction_consistent) {
                consistent = 0;
                velocity_sum.setZero();
            }
            previous_velocity = velocity;
            velocity_sum += velocity;
            ++consistent;
            if (consistent >= kRequiredHeadingWindows) {
                const libgnss::Vector3d course = velocity_sum / static_cast<double>(consistent);
                if (libgnss::fusion_initialization::tryAlignHeading(state, course, 1.0, 5.0)) {
                    initial_velocity = course;
                    report.heading_latched = true;
                    break;
                }
            }
        }
        report.heading_windows = windows;
        report.heading_initialization_mode = "legacy-consistency-latch";
        if (!report.heading_latched) {
            report.failure = "GNSS course did not make heading observable under frozen gate";
            return false;
        }
    }

    auto& imu = problem.imu;
    imu.valid = true;
    imu.nav_origin_ecef = origin_ecef;
    imu.nav_origin_lat_rad = lat;
    imu.nav_origin_lon_rad = lon;
    imu.samples_body_flu = std::move(samples);
    imu.init_attitude_body_to_nav = state.nominal.attitude_body_to_enu.toRotationMatrix();
    imu.init_velocity_nav = initial_velocity;
    if (gnss_first_velocities_enu != nullptr &&
        gnss_first_velocities_enu->size() == problem.epochs.size()) {
        imu.stop_velocity_seeds_nav = *gnss_first_velocities_enu;
    }
    imu.init_accel_bias = aligned.accel_bias;
    imu.init_gyro_bias = aligned.gyro_bias;
    // The values are the public taroz pixel preset after the 0.5 synchronization
    // coefficient, with no truth-driven tuning.
    imu.noise.gravity_mps2 = kGravity;
    imu.noise.accel_noise_sigma = 0.025;
    imu.noise.gyro_noise_sigma = 0.0005;
    imu.noise.accel_bias_rw_sigma = 0.00025;
    imu.noise.gyro_bias_rw_sigma = 0.0000005;
    imu.noise.integration_sigma = 0.05;
    imu.init_attitude_sigma_roll_pitch_rad = 0.05;
    imu.init_attitude_sigma_yaw_rad = 5.0 / kRadToDeg;
    imu.init_velocity_sigma_mps = 0.5;
    imu.init_accel_bias_sigma = 0.1;
    imu.init_gyro_bias_sigma = 0.01;
    report.ok = true;
    return true;
}

void writeJsonString(std::ostringstream& out, const std::string& value) {
    out << '"';
    for (const char ch : value) {
        switch (ch) {
            case '\\': out << "\\\\"; break;
            case '"': out << "\\\""; break;
            case '\n': out << "\\n"; break;
            case '\r': out << "\\r"; break;
            case '\t': out << "\\t"; break;
            default: out << ch; break;
        }
    }
    out << '"';
}

std::string makeSummary(const Options& options,
                        const libgnss::FGOProcessor::FGOProblem& problem,
                        const libgnss::FGOProcessor::FGOResult& result,
                        const ImuBuildReport& imu_report,
                        bool fallback,
                        const NativePdcBridgeReport& pdc_bridge_report =
                            NativePdcBridgeReport{},
                        const RawUtcOutputReport& raw_utc_report = RawUtcOutputReport{},
                        const TdcpRuntimeReport& tdcp_report = TdcpRuntimeReport{},
                        const libgnss::carrier_code_leveling::Diagnostics&
                            carrier_code_leveling_report =
                                libgnss::carrier_code_leveling::Diagnostics{},
                        const UpstreamPositionOffsetReport& position_offset_report =
                            UpstreamPositionOffsetReport{}) {
    std::ostringstream out;
    out << std::setprecision(17);
    out << "{\n"
        << "  \"schema_version\": \"smartphone-r5-native-fgo-android-imu-no-base-run.v3\",\n"
        << "  \"dataset_id\": ";
    writeJsonString(out, options.dataset_id);
    out << ",\n  \"status\": "
        << (fallback ? "\"fallback-native-fgo-v1\"" : "\"imu-combined-factor\"") << ",\n"
        << "  \"truth_used\": false,\n"
        << "  \"base_factors\": false,\n"
        << "  \"no_base_contract\": true,\n"
        << "  \"production_default_changed\": false,\n"
        << "  \"fgo_imu_sparse_recovery\": "
        << (options.fgo_imu_sparse_recovery ? "true" : "false") << ",\n"
        << "  \"native_pdc_state_bridge\": "
        << (options.native_pdc_state_bridge ? "true" : "false") << ",\n"
        << "  \"native_pdc_imu_tdcp\": "
        << (options.native_pdc_imu_tdcp ? "true" : "false") << ",\n"
        << "  \"native_signal_bias_states\": "
        << (options.native_signal_bias_states ? "true" : "false") << ",\n"
        << "  \"native_residual_ionosphere\": "
        << (options.native_residual_ionosphere ? "true" : "false") << ",\n"
        << "  \"native_upstream_quality\": "
        << (options.native_upstream_quality ? "true" : "false") << ",\n"
        << "  \"native_carrier_code_leveling\": "
        << (options.native_carrier_code_leveling ? "true" : "false") << ",\n"
        << "  \"native_carrier_code_innovation_reset\": "
        << (options.native_carrier_code_innovation_reset ? "true" : "false")
        << ",\n"
        << "  \"native_upstream_stop_constraints\": "
        << (options.native_upstream_stop_constraints ? "true" : "false")
        << ",\n"
        << "  \"native_upstream_position_offset\": "
        << (options.native_upstream_position_offset ? "true" : "false")
        << ",\n"
        ;
    if (options.native_carrier_code_primary_l1_e1) {
        out << "  \"native_carrier_code_primary_l1_e1\": true,\n";
    }
    if (options.native_carrier_code_gal_e1_e5a) {
        out << "  \"native_carrier_code_gal_e1_e5a\": true,\n";
    }
    out << "  \"android_utc_wall_clock_fallback\": "
        << (options.android_utc_wall_clock_fallback ? "true" : "false") << ",\n"
        << "  \"android_utc_wall_clock_fallback_applied\": "
        << (imu_report.android_load.utc_wall_clock_fallback_applied
                ? "true"
                : "false") << ",\n"
        << "  \"native_pdc_state_bridge_report\": {\n"
        << "    \"enabled\": "
        << (pdc_bridge_report.enabled ? "true" : "false") << ",\n"
        << "    \"ok\": " << (pdc_bridge_report.ok ? "true" : "false") << ",\n"
        << "    \"epochs\": " << pdc_bridge_report.epochs << ",\n"
        << "    \"pseudorange_rows\": " << pdc_bridge_report.pseudorange_rows << ",\n"
        << "    \"doppler_rows\": " << pdc_bridge_report.doppler_rows << ",\n"
        << "    \"motion_intervals\": " << pdc_bridge_report.motion_intervals << ",\n"
        << "    \"valid_position_states\": "
        << pdc_bridge_report.valid_position_states << ",\n"
        << "    \"rejected_position_states\": "
        << pdc_bridge_report.rejected_position_states << ",\n"
        << "    \"valid_velocity_states\": "
        << pdc_bridge_report.valid_velocity_states << ",\n"
        << "    \"state_seeds\": " << pdc_bridge_report.state_seeds << ",\n"
        << "    \"iterations\": " << pdc_bridge_report.iterations << ",\n"
        << "    \"initial_cost\": " << pdc_bridge_report.initial_cost << ",\n"
        << "    \"final_cost\": " << pdc_bridge_report.final_cost << ",\n"
        << "    \"max_velocity_norm_mps\": "
        << pdc_bridge_report.max_velocity_norm_mps << ",\n"
        << "    \"max_clock_rate_abs_mps\": "
        << pdc_bridge_report.max_clock_rate_abs_mps << ",\n"
        << "    \"max_normalized_rms\": "
        << pdc_bridge_report.max_normalized_rms << ",\n"
        << "    \"fgo_seed_displacement_count\": "
        << pdc_bridge_report.fgo_seed_displacement_count << ",\n"
        << "    \"fgo_seed_displacement_p50_m\": "
        << pdc_bridge_report.fgo_seed_displacement_p50_m << ",\n"
        << "    \"fgo_seed_displacement_max_m\": "
        << pdc_bridge_report.fgo_seed_displacement_max_m << ",\n"
        << "    \"failure\": ";
    writeJsonString(out, pdc_bridge_report.failure);
    out << "\n  },\n"
        << "  \"skip_epochs\": " << options.skip_epochs << ",\n"
        << "  \"all_epochs\": " << (options.all_epochs ? "true" : "false") << ",\n"
        << "  \"inputs\": {\n"
        << "    \"observation\": ";
    if (options.obs_path.empty()) {
        out << "null";
    } else {
        writeJsonString(out, options.obs_path);
    }
    out << ",\n    \"navigation\": ";
    writeJsonString(out, options.nav_path);
    out << ",\n    \"imu\": ";
    writeJsonString(out, options.android_imu_path.empty()
                            ? options.imu_path
                            : options.android_imu_path);
    out << ",\n    \"android_gnss\": ";
    if (options.android_gnss_path.empty()) {
        out << "null";
    } else {
        writeJsonString(out, options.android_gnss_path);
    }
    out << "\n  },\n";
    if (imu_report.android_raw) {
        const auto& gnss = imu_report.android_gnss_diagnostics;
        out << "  \"android_gnss_diagnostics\": {\n"
            << "    \"input_rows\": " << gnss.input_rows << ",\n"
            << "    \"raw_rows\": " << gnss.raw_rows << ",\n"
            << "    \"selected_rows\": " << gnss.selected_rows << ",\n"
            << "    \"selected_epochs\": " << gnss.selected_epochs << ",\n"
            << "    \"carrier_rows\": " << gnss.carrier_rows << ",\n"
            << "    \"doppler_rows\": " << gnss.doppler_rows << ",\n"
            << "    \"clock_discontinuities\": " << gnss.clock_discontinuities << ",\n"
            << "    \"timing_formula\": ";
        writeJsonString(out, gnss.timing_formula);
        out << ",\n    \"no_device_wls_seed\": true\n"
            << "  },\n";
    }
    out
        << "  \"epochs\": {\n"
        << "    \"problem\": " << problem.epochs.size() << ",\n"
        << "    \"output\": " << result.solution.solutions.size() << ",\n"
        << "    \"sparse_epochs_retained\": "
        << result.diagnostics.sparse_epochs_retained << ",\n"
        << "    \"sparse_empty_epochs_retained\": "
        << result.diagnostics.sparse_empty_epochs_retained << ",\n"
        << "    \"pseudorange_factors\": " << problem.pseudorange_factors.size() << ",\n"
        << "    \"receiver_signal_bias_factors\": "
        << result.diagnostics.receiver_signal_bias_factors << ",\n"
        << "    \"receiver_signal_bias_states\": "
        << result.diagnostics.receiver_signal_bias_states << ",\n"
        << "    \"residual_ionosphere_factors\": "
        << result.diagnostics.residual_ionosphere_factors << ",\n"
        << "    \"residual_ionosphere_states\": "
        << result.diagnostics.residual_ionosphere_states << ",\n"
        << "    \"residual_ionosphere_resets\": "
        << result.diagnostics.residual_ionosphere_resets << ",\n"
        << "    \"tdcp_factors_built\": " << problem.tdcp_factors.size() << ",\n"
        << "    \"double_difference_pseudorange_factors\": "
        << problem.double_difference_pseudorange_factors.size() << ",\n"
        << "    \"double_difference_carrier_factors\": "
        << problem.double_difference_carrier_factors.size() << "\n"
        << "  },\n"
        << "  \"upstream_observable_quality\": {\n"
        << "    \"enabled\": "
        << (options.native_upstream_quality ? "true" : "false") << ",\n"
        << "    \"snr_percentile\": 85.0,\n"
        << "    \"snr_denominator_db\": 20.0,\n"
        << "    \"min_snr_dbhz\": 20.0,\n"
        << "    \"min_elevation_deg\": 5.0,\n"
        << "    \"max_adjacent_gap_s\": 1.5,\n"
        << "    \"tdcp_sigma_m_unchanged\": 0.03,\n"
        << "    \"snr_l1_dbhz\": ";
    if (options.native_upstream_quality &&
        std::isfinite(problem.diagnostics.upstream_snr_l1_dbhz)) {
        out << problem.diagnostics.upstream_snr_l1_dbhz;
    } else {
        out << "null";
    }
    out << ",\n    \"snr_l5_dbhz\": ";
    if (options.native_upstream_quality &&
        std::isfinite(problem.diagnostics.upstream_snr_l5_dbhz)) {
        out << problem.diagnostics.upstream_snr_l5_dbhz;
    } else {
        out << "null";
    }
    out << ",\n"
        << "    \"pseudorange_candidates\": "
        << problem.diagnostics.upstream_pseudorange_candidates << ",\n"
        << "    \"pseudorange_factors\": "
        << problem.diagnostics.upstream_pseudorange_factors << ",\n"
        << "    \"doppler_candidates\": "
        << problem.diagnostics.upstream_doppler_candidates << ",\n"
        << "    \"doppler_factors\": "
        << problem.diagnostics.upstream_doppler_factors << ",\n"
        << "    \"doppler_graph_factors\": "
        << result.diagnostics.undifferenced_doppler_factors_inserted << ",\n"
        << "    \"pd_pair_rejections\": "
        << problem.diagnostics.upstream_pd_pair_rejections << ",\n"
        << "    \"ld_pair_rejections\": "
        << problem.diagnostics.upstream_ld_pair_rejections << ",\n"
        << "    \"doppler_residual_rejections\": "
        << problem.diagnostics.upstream_doppler_residual_rejections << ",\n"
        << "    \"pseudorange_residual_rejections\": "
        << problem.diagnostics.upstream_pseudorange_residual_rejections << ",\n"
        << "    \"pseudorange_postfit_rms_m\": "
        << result.diagnostics.residual_rms_m << ",\n"
        << "    \"pseudorange_postfit_normalized_rms\": "
        << result.diagnostics.upstream_pseudorange_normalized_rms << ",\n"
        << "    \"doppler_postfit_rms_mps\": "
        << result.diagnostics.undifferenced_doppler_residual_rms_mps << ",\n"
        << "    \"doppler_postfit_normalized_rms\": "
        << result.diagnostics.upstream_doppler_normalized_rms << ",\n"
        << "    \"pseudorange_sigma_contract\": \"snr_scale*signal_type_factor\",\n"
        << "    \"doppler_sigma_contract\": \"snr_scale/12\",\n"
        << "    \"carrier_tdcp_contract\": \"Phase12 frozen sigma 0.03; upstream L weighting not enabled\"\n"
        << "  },\n"
        << "  \"receiver_signal_bias_estimates_m\": {";
    bool first_signal_bias = true;
    for (const auto& [key, value] : result.receiver_signal_bias_estimates_m) {
        if (!first_signal_bias) out << ",";
        first_signal_bias = false;
        out << "\"" << static_cast<int>(static_cast<unsigned char>(key.first))
            << ":" << static_cast<int>(static_cast<unsigned char>(key.second))
            << "\": " << value;
    }
    out << "},\n"
        << "  \"residual_ionosphere_contract\": {\n"
        << "    \"enabled\": "
        << (options.native_residual_ionosphere ? "true" : "false") << ",\n"
        << "    \"prior_sigma_m\": 10.0,\n"
        << "    \"random_walk_sigma_m_per_sqrt_s\": 1.0,\n"
        << "    \"max_abs_state_limit_m\": 30.0,\n"
        << "    \"max_gap_s\": 2.0,\n"
        << "    \"mapping\": \"thin-shell earth-radius 6371000 m, shell-height 350000 m\",\n"
        << "    \"state_units\": \"vertical L1 residual ionosphere metres\",\n"
        << "    \"sign\": \"positive coefficient*state increases predicted pseudorange\",\n"
        << "    \"factors\": " << result.diagnostics.residual_ionosphere_factors << ",\n"
        << "    \"states\": " << result.diagnostics.residual_ionosphere_states << ",\n"
        << "    \"resets\": " << result.diagnostics.residual_ionosphere_resets << ",\n"
        << "    \"invalid_coefficients\": "
        << result.diagnostics.residual_ionosphere_invalid_coefficients << ",\n"
        << "    \"max_abs_state_m\": "
        << result.diagnostics.residual_ionosphere_max_abs_m << ",\n"
        << "    \"rms_state_m\": "
        << result.diagnostics.residual_ionosphere_rms_m << ",\n"
        << "    \"min_coefficient\": "
        << result.diagnostics.residual_ionosphere_min_coefficient << ",\n"
        << "    \"max_coefficient\": "
        << result.diagnostics.residual_ionosphere_max_coefficient << "\n"
        << "  },\n"
        << "  \"tdcp_contract\": {\n"
        << "    \"enabled\": " << (tdcp_report.enabled ? "true" : "false") << ",\n"
        << "    \"factors_built\": " << tdcp_report.factors_built << ",\n"
        << "    \"factors_inserted\": " << tdcp_report.factors_inserted << ",\n"
        << "    \"candidate_pairs\": " << tdcp_report.candidate_pairs << ",\n"
        << "    \"rejected_gap\": " << tdcp_report.rejected_gap << ",\n"
        << "    \"rejected_clock_discontinuity\": "
        << tdcp_report.rejected_clock_discontinuity << ",\n"
        << "    \"rejected_missing_previous\": "
        << tdcp_report.rejected_missing_previous << ",\n"
        << "    \"rejected_loss_of_lock\": "
        << tdcp_report.rejected_loss_of_lock << ",\n"
        << "    \"rejected_invalid_measurement\": "
        << tdcp_report.rejected_invalid_measurement << ",\n"
        << "    \"rejected_code_phase_jump\": "
        << tdcp_report.rejected_code_phase_jump << ",\n"
        << "    \"finite_residuals\": " << tdcp_report.finite_residuals << ",\n"
        << "    \"nonfinite_residuals\": " << tdcp_report.nonfinite_residuals << ",\n"
        << "    \"arc_count\": " << tdcp_report.arc_count << ",\n"
        << "    \"min_arc_length_epochs\": "
        << tdcp_report.min_arc_length_epochs << ",\n"
        << "    \"median_arc_length_epochs\": "
        << tdcp_report.median_arc_length_epochs << ",\n"
        << "    \"max_arc_length_epochs\": "
        << tdcp_report.max_arc_length_epochs << ",\n"
        << "    \"sigma_m\": " << tdcp_report.sigma_m << ",\n"
        << "    \"max_gap_s\": " << tdcp_report.max_gap_s << ",\n"
        << "    \"code_phase_jump_threshold_m\": "
        << tdcp_report.code_phase_jump_threshold_m << ",\n"
        << "    \"residual_rms_m\": " << tdcp_report.residual_rms_m << ",\n"
        << "    \"normalized_residual_rms\": "
        << tdcp_report.normalized_residual_rms << ",\n"
        << "    \"max_abs_residual_m\": " << tdcp_report.max_abs_residual_m << ",\n"
        << "    \"pair_key\": \"(satellite,signal)\",\n"
        << "    \"adr_state_slip_fail_closed\": true,\n"
        << "    \"standalone_carrier_ambiguity_factors\": false,\n"
        << "    \"base_or_double_difference_factors\": false\n"
        << "  },\n"
        << "  \"upstream_stop_constraints\": {\n"
        << "    \"enabled\": "
        << (options.native_upstream_stop_constraints ? "true" : "false") << ",\n"
        << "    \"window_samples\": 500,\n"
        << "    \"acceleration_std_offset_mps2\": 0.08,\n"
        << "    \"gyro_std_offset_radps\": 0.005,\n"
        << "    \"gyro_norm_max_radps\": 0.05,\n"
        << "    \"velocity_threshold_mps\": 0.5,\n"
        << "    \"velocity_sigma_mps\": 0.01,\n"
        << "    \"velocity_huber_k_sigma\": 0.5,\n"
        << "    \"pose_rotation_sigma_rad\": 0.0017453292519943296,\n"
        << "    \"pose_translation_sigma_m\": 0.02,\n"
        << "    \"pose_huber_k_sigma\": 0.5,\n"
        << "    \"detection_source\": \"aligned raw IMU acceleration/gyro only\",\n"
        << "    \"epoch_mapping\": \"nearest with endpoint hold\",\n"
        << "    \"no_height_or_truth_factor\": true,\n"
        << "    \"detected_epochs\": "
        << result.diagnostics.upstream_stop_epochs << ",\n"
        << "    \"velocity_factors\": "
        << result.diagnostics.upstream_stop_velocity_factors << ",\n"
        << "    \"pose_factors\": "
        << result.diagnostics.upstream_stop_pose_factors << ",\n"
        << "    \"imu_samples\": "
        << result.diagnostics.upstream_stop_imu_samples << ",\n"
        << "    \"acceleration_std_threshold_mps2\": "
        << result.diagnostics.upstream_stop_acceleration_std_threshold_mps2 << ",\n"
        << "    \"gyro_std_threshold_radps\": "
        << result.diagnostics.upstream_stop_gyro_std_threshold_radps << "\n"
        << "  },\n"
        << "  \"upstream_position_offset\": {\n"
        << "    \"enabled\": "
        << (position_offset_report.enabled ? "true" : "false") << ",\n"
        << "    \"applied\": "
        << (position_offset_report.applied ? "true" : "false") << ",\n"
        << "    \"phone\": ";
    writeJsonString(out, position_offset_report.phone);
    out << ",\n"
        << "    \"corrected_epochs\": "
        << position_offset_report.corrected_epochs << ",\n"
        << "    \"offset_rl_m\": " << position_offset_report.offset_rl_m << ",\n"
        << "    \"offset_ud_m\": " << position_offset_report.offset_ud_m << ",\n"
        << "    \"max_offset_enu_m\": "
        << position_offset_report.max_offset_enu_m << ",\n"
        << "    \"rotation_contract\": \"Rx*Ry*Rz(rpy-[0,0,pi])\",\n"
        << "    \"source\": \"in-memory optimized GTSAM Rot3::rpy; raw-only\",\n"
        << "    \"failure\": ";
    writeJsonString(out, position_offset_report.failure);
    out << "\n  },\n"
        << "  \"graph\": {\n"
        << "    \"factors\": " << result.diagnostics.graph_factors << ",\n"
        << "    \"values\": " << result.diagnostics.graph_values << ",\n"
        << "    \"imu_intervals\": " << result.diagnostics.imu_intervals << ",\n"
        << "    \"upstream_stop_epochs\": "
        << result.diagnostics.upstream_stop_epochs << ",\n"
        << "    \"upstream_stop_velocity_factors\": "
        << result.diagnostics.upstream_stop_velocity_factors << ",\n"
        << "    \"upstream_stop_pose_factors\": "
        << result.diagnostics.upstream_stop_pose_factors << ",\n"
        << "    \"upstream_stop_imu_samples\": "
        << result.diagnostics.upstream_stop_imu_samples << ",\n"
        << "    \"upstream_stop_acceleration_std_threshold_mps2\": "
        << result.diagnostics.upstream_stop_acceleration_std_threshold_mps2 << ",\n"
        << "    \"upstream_stop_gyro_std_threshold_radps\": "
        << result.diagnostics.upstream_stop_gyro_std_threshold_radps << ",\n"
        << "    \"iterations\": " << result.diagnostics.iterations << ",\n"
        << "    \"converged\": " << (result.diagnostics.converged ? "true" : "false") << ",\n"
        << "    \"initial_cost\": " << result.diagnostics.initial_cost << ",\n"
        << "    \"final_cost\": " << result.diagnostics.final_cost << "\n"
        << "  },\n"
        << "  \"carrier_code_leveling\": {\n"
        << "    \"enabled\": "
        << (carrier_code_leveling_report.enabled ? "true" : "false") << ",\n"
        << "    \"signal\": ";
    writeJsonString(out, options.native_carrier_code_gal_e1_e5a
                             ? "GAL_E1+GAL_E5A"
                             : options.native_carrier_code_primary_l1_e1
                                   ? "GPS_L1CA+GAL_E1"
                                   : "Galileo E1");
    out << ",\n"
        << "    \"window_samples\": 30,\n"
        << "    \"max_gap_s\": 1.5,\n"
        << "    \"target_rows\": "
        << carrier_code_leveling_report.target_rows << ",\n"
        << "    \"eligible_rows\": "
        << carrier_code_leveling_report.eligible_rows << ",\n"
        << "    \"smoothed_rows\": "
        << carrier_code_leveling_report.smoothed_rows << ",\n"
        << "    \"arcs_started\": "
        << carrier_code_leveling_report.arcs_started << ",\n"
        << "    \"updates\": " << carrier_code_leveling_report.updates << ",\n"
        << "    \"reset_invalid_code\": "
        << carrier_code_leveling_report.reset_invalid_code << ",\n"
        << "    \"reset_invalid_adr\": "
        << carrier_code_leveling_report.reset_invalid_adr << ",\n"
        << "    \"reset_adr\": "
        << carrier_code_leveling_report.reset_adr << ",\n"
        << "    \"reset_cycle_slip\": "
        << carrier_code_leveling_report.reset_cycle_slip << ",\n"
        << "    \"reset_missing_or_nonfinite_adr\": "
        << carrier_code_leveling_report.reset_missing_or_nonfinite_adr << ",\n"
        << "    \"reset_gap\": "
        << carrier_code_leveling_report.reset_gap << ",\n"
        << "    \"reset_clock_discontinuity\": "
        << carrier_code_leveling_report.reset_clock_discontinuity << ",\n"
        << "    \"innovation_reset_enabled\": "
        << (options.native_carrier_code_innovation_reset ? "true" : "false")
        << ",\n"
        << "    \"innovation_reset_threshold_m\": "
        << carrier_code_leveling_report.innovation_reset_threshold_m << ",\n"
        << "    \"reset_innovation\": "
        << carrier_code_leveling_report.reset_innovation << ",\n"
        << "    \"max_abs_innovation_accepted_m\": "
        << carrier_code_leveling_report.max_abs_innovation_accepted_m << ",\n"
        << "    \"max_abs_innovation_rejected_m\": "
        << carrier_code_leveling_report.max_abs_innovation_rejected_m << ",\n"
        << "    \"max_abs_level_adjustment_m\": "
        << carrier_code_leveling_report.max_abs_level_adjustment_m << ",\n"
        << "    \"max_abs_phase_increment_m\": "
        << carrier_code_leveling_report.max_abs_phase_increment_m << ",\n"
        ;
    if (options.native_carrier_code_primary_l1_e1 ||
        options.native_carrier_code_gal_e1_e5a) {
        out << "    \"signals\": [";
        if (options.native_carrier_code_gal_e1_e5a) {
            out << "\"GAL_E1\", \"GAL_E5A\"";
        } else {
            out << "\"GPS_L1CA\", \"GAL_E1\"";
        }
        out << "],\n"
            << "    \"per_signal\": [";
        for (std::size_t index = 0U;
             index < carrier_code_leveling_report.per_signal.size(); ++index) {
            if (index != 0U) out << ",";
            const auto& signal_diagnostics =
                carrier_code_leveling_report.per_signal[index];
            out << "\n      {\n        \"signal\": ";
            writeJsonString(out, carrierSignalName(signal_diagnostics.signal));
            out << ",\n"
                << "        \"target_rows\": "
                << signal_diagnostics.target_rows << ",\n"
                << "        \"eligible_rows\": "
                << signal_diagnostics.eligible_rows << ",\n"
                << "        \"smoothed_rows\": "
                << signal_diagnostics.smoothed_rows << ",\n"
                << "        \"arcs_started\": "
                << signal_diagnostics.arcs_started << ",\n"
                << "        \"updates\": " << signal_diagnostics.updates << ",\n"
                << "        \"reset_invalid_code\": "
                << signal_diagnostics.reset_invalid_code << ",\n"
                << "        \"reset_invalid_adr\": "
                << signal_diagnostics.reset_invalid_adr << ",\n"
                << "        \"reset_adr\": "
                << signal_diagnostics.reset_adr << ",\n"
                << "        \"reset_cycle_slip\": "
                << signal_diagnostics.reset_cycle_slip << ",\n"
                << "        \"reset_missing_or_nonfinite_adr\": "
                << signal_diagnostics.reset_missing_or_nonfinite_adr << ",\n"
                << "        \"reset_gap\": " << signal_diagnostics.reset_gap << ",\n"
                << "        \"reset_clock_discontinuity\": "
                << signal_diagnostics.reset_clock_discontinuity << ",\n"
                << "        \"reset_innovation\": "
                << signal_diagnostics.reset_innovation << ",\n"
                << "        \"innovation_reset_threshold_m\": "
                << signal_diagnostics.innovation_reset_threshold_m << ",\n"
                << "        \"max_abs_innovation_accepted_m\": "
                << signal_diagnostics.max_abs_innovation_accepted_m << ",\n"
                << "        \"max_abs_innovation_rejected_m\": "
                << signal_diagnostics.max_abs_innovation_rejected_m << "\n"
                << "      }";
        }
        out << "\n    ],\n";
    }
    out << "    \"no_new_graph_state\": true,\n"
        << "    \"truth_free\": true\n"
        << "  },\n"
        << "  \"gnss_first\": {\n"
        << "    \"attempted\": " << (imu_report.gnss_first_attempted ? "true" : "false") << ",\n"
        << "    \"converged\": " << (imu_report.gnss_first_converged ? "true" : "false") << ",\n"
        << "    \"epochs\": " << imu_report.gnss_first_epochs << ",\n"
        << "    \"undifferenced_doppler_factors\": "
        << imu_report.gnss_first_doppler_factors << ",\n"
        << "    \"velocity_states_exported\": "
        << imu_report.gnss_first_velocity_states << ",\n"
        << "    \"iterations\": " << imu_report.gnss_first_iterations << ",\n"
        << "    \"initial_cost\": " << imu_report.gnss_first_initial_cost << ",\n"
        << "    \"final_cost\": " << imu_report.gnss_first_final_cost << ",\n"
        << "    \"failure\": ";
    writeJsonString(out, imu_report.gnss_first_failure);
    out << "\n  },\n"
        << "  \"imu_initialization\": {\n"
        << "    \"input_format\": ";
    writeJsonString(out, imu_report.android_raw ? "android-device_imu.csv"
                                                : "gpst-metric-imu.csv");
    out << ",\n"
        << "    \"loaded_samples\": " << imu_report.loaded_samples << ",\n"
        << "    \"stationary_samples\": " << imu_report.stationary_samples << ",\n"
        << "    \"gravity_mean_norm_mps2\": " << imu_report.gravity_mean_norm << ",\n"
        << "    \"gravity_norm_std_mps2\": " << imu_report.gravity_norm_std << ",\n"
        << "    \"heading_windows\": " << imu_report.heading_windows << ",\n"
        << "    \"heading_latched\": " << (imu_report.heading_latched ? "true" : "false") << ",\n"
        << "    \"heading_initialization_mode\": ";
    writeJsonString(out, imu_report.heading_initialization_mode);
    out << ",\n"
        << "    \"velocity_heading_low_speed_count\": "
        << imu_report.velocity_heading_low_speed_count << ",\n"
        << "    \"velocity_heading_linear_fill_count\": "
        << imu_report.velocity_heading_linear_fill_count << ",\n"
        << "    \"velocity_heading_nearest_fill_count\": "
        << imu_report.velocity_heading_nearest_fill_count << ",\n"
        << "    \"axis_contract\": \"identity raw-axis selection + explicit RzRyRx mounting\",\n"
        << "    \"mounting_rpy_deg_xyz\": [-85.0, 178.0, -94.0],\n"
        << "    \"timestamp_contract\": ";
    writeJsonString(out, imu_report.android_raw
                           ? (imu_report.android_load.utc_wall_clock_fallback_applied
                                  ? "Raw GNSS TimeNanos-FullBiasNanos-BiasNanos -> UTC milliseconds affine map; UTC wall-clock pairing; raw UTC -> GPST; elapsedRealtimeNanos absent and not fabricated"
                                  : "GNSS ChipsetElapsedRealtimeNanos -> UTC milliseconds by linear interpolation/extrapolation; gyro anchors; sync coefficient 0.5; UTC + 18 s -> GPST")
                           : "GPST week/TOW CSV; no runtime offset");
    out << ",\n"
        << "    \"android_alignment\": ";
    if (!imu_report.android_raw) {
        out << "null,\n";
    } else {
        const auto& alignment = imu_report.android_load;
        out << "{\n"
            << "      \"total_rows\": " << alignment.total_rows << ",\n"
            << "      \"accel_rows\": " << alignment.accel_rows << ",\n"
            << "      \"gyro_rows\": " << alignment.gyro_rows << ",\n"
            << "      \"unsupported_rows\": " << alignment.unsupported_rows << ",\n"
            << "      \"duplicate_accel_timestamps\": "
            << alignment.duplicate_accel_timestamps << ",\n"
            << "      \"duplicate_gyro_timestamps\": "
            << alignment.duplicate_gyro_timestamps << ",\n"
            << "      \"paired_rows\": " << alignment.paired_rows << ",\n"
            << "      \"exact_elapsed_matches\": "
            << alignment.exact_elapsed_matches << ",\n"
            << "      \"interpolated_rows\": " << alignment.interpolated_rows << ",\n"
            << "      \"endpoint_nearest_rows\": "
            << alignment.endpoint_nearest_rows << ",\n"
            << "      \"omitted_rows\": " << alignment.omitted_rows << ",\n"
            << "      \"median_abs_pair_offset_ms\": "
            << alignment.median_abs_pair_offset_ms << ",\n"
            << "      \"maximum_abs_pair_offset_ms\": "
            << alignment.maximum_abs_pair_offset_ms << ",\n"
            << "      \"gnss_anchor_points\": "
            << alignment.gnss_anchor_points << ",\n"
            << "      \"gnss_anchor_exact_rows\": "
            << alignment.gnss_anchor_exact_rows << ",\n"
            << "      \"gnss_anchor_interpolated_rows\": "
            << alignment.gnss_anchor_interpolated_rows << ",\n"
            << "      \"gnss_anchor_extrapolated_rows\": "
            << alignment.gnss_anchor_extrapolated_rows << ",\n"
            << "      \"first_mapped_utc_time_ms\": "
            << alignment.first_mapped_utc_time_ms << ",\n"
            << "      \"last_mapped_utc_time_ms\": "
            << alignment.last_mapped_utc_time_ms << ",\n"
            << "      \"imu_sync_coefficient\": "
            << alignment.imu_sync_coefficient << ",\n"
            << "      \"first_dt_s\": " << alignment.first_dt_s << ",\n"
            << "      \"last_dt_s\": " << alignment.last_dt_s << ",\n"
            << "      \"dt_tail_repeated\": "
            << (alignment.dt_tail_repeated ? "true" : "false") << ",\n"
            << "      \"anchor_input_rows\": "
            << imu_report.android_gnss_anchor_load.input_rows << ",\n"
            << "      \"anchor_raw_rows\": "
            << imu_report.android_gnss_anchor_load.raw_rows << ",\n"
            << "      \"anchor_duplicate_utc_timestamps\": "
            << imu_report.android_gnss_anchor_load.duplicate_utc_timestamps << ",\n"
            << "      \"utc_mapping_input_rows\": "
            << imu_report.android_gnss_utc_mapping_load.input_rows << ",\n"
            << "      \"utc_mapping_raw_rows\": "
            << imu_report.android_gnss_utc_mapping_load.raw_rows << ",\n"
            << "      \"utc_mapping_unsupported_rows\": "
            << imu_report.android_gnss_utc_mapping_load.unsupported_rows << ",\n"
            << "      \"utc_mapping_duplicate_utc_timestamps\": "
            << imu_report.android_gnss_utc_mapping_load.duplicate_utc_timestamps << ",\n"
            << "      \"utc_mapping_hardware_clock_count_field_present\": "
            << (imu_report.android_gnss_utc_mapping_load.mapping
                        .hardware_clock_count_field_present
                    ? "true"
                    : "false") << ",\n"
            << "      \"utc_mapping_hardware_clock_count_constant\": "
            << (imu_report.android_gnss_utc_mapping_load.mapping
                        .hardware_clock_count_constant
                    ? "true"
                    : "false") << ",\n"
            << "      \"first_gyro_elapsed_ns\": "
            << alignment.first_gyro_elapsed_ns << ",\n"
            << "      \"last_gyro_elapsed_ns\": "
            << alignment.last_gyro_elapsed_ns << ",\n"
            << "      \"elapsed_clock_preserved\": "
            << (alignment.elapsed_clock_preserved ? "true" : "false") << ",\n"
            << "      \"gnss_elapsed_anchor_applied\": "
            << (alignment.gnss_elapsed_anchor_applied ? "true" : "false") << ",\n"
            << "      \"utc_wall_clock_fallback_applied\": "
            << (alignment.utc_wall_clock_fallback_applied ? "true" : "false") << ",\n"
            << "      \"utc_mapping_anchors\": "
            << alignment.utc_mapping_anchors << ",\n"
            << "      \"utc_mapping_slope_ns_per_ms\": "
            << alignment.utc_mapping_slope_ns_per_ms << ",\n"
            << "      \"utc_mapping_drift_ppm\": "
            << alignment.utc_mapping_drift_ppm << ",\n"
            << "      \"utc_mapping_max_fit_residual_ms\": "
            << alignment.utc_mapping_max_fit_residual_ms << ",\n"
            << "      \"utc_mapping_max_anchor_gap_ms\": "
            << alignment.utc_mapping_max_anchor_gap_ms << "\n"
            << "    },\n";
    }
    out << "    \"failure\": ";
    writeJsonString(out, imu_report.failure);
    out << "\n  },\n";
    if (raw_utc_report.enabled) {
        out << "  \"raw_utc_key_contract\": {\n"
            << "    \"warmup_epoch_excluded\": true,\n"
            << "    \"raw_epoch_keys\": " << raw_utc_report.raw_epoch_keys << ",\n"
            << "    \"target_epochs\": " << raw_utc_report.target_epochs << ",\n"
            << "    \"exact_solution_epochs\": "
            << raw_utc_report.exact_solution_epochs << ",\n"
            << "    \"interpolated_epochs\": "
            << raw_utc_report.interpolated_epochs << ",\n"
            << "    \"edge_hold_epochs\": " << raw_utc_report.edge_hold_epochs << ",\n"
            << "    \"unresolved_epochs\": " << raw_utc_report.unresolved_epochs << ",\n"
            << "    \"solution_time_tolerance_ms\": "
            << raw_utc_report.solution_time_tolerance_ms << ",\n"
            << "    \"max_interpolation_gap_ms\": "
            << raw_utc_report.max_interpolation_gap_ms << ",\n"
            << "    \"max_edge_hold_gap_ms\": "
            << raw_utc_report.max_edge_hold_gap_ms << ",\n"
            << "    \"coordinate_interpolation\": \"ECEF linear then geodetic\",\n"
            << "    \"device_wls_coordinates_used\": false\n"
            << "  },\n";
    }
    out << "  \"output_contract\": {\n"
        << "    \"header\": \"phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\",\n"
        << "    \"finite_coordinates\": true,\n"
        << "    \"unix_time_leap_seconds\": 18,\n"
        << "    \"atomic_publish\": true\n"
        << "  }\n"
        << "}\n";
    return out.str();
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!parseArguments(argc, argv, options)) return 2;

    const bool android_raw = !options.android_imu_path.empty();
    libgnss::io::RINEXReader obs_reader;
    libgnss::io::RINEXReader::RINEXHeader obs_header;
    libgnss::io::AndroidRawGnssResult android_gnss;
    std::vector<libgnss::GNSSTime> android_raw_epoch_times;
    if (android_raw) {
        libgnss::io::AndroidRawGnssConfig android_gnss_config;
        std::string conversion_error;
        if (!libgnss::io::loadAndroidRawGnssCsv(
                options.android_gnss_path, android_gnss_config, android_gnss,
                conversion_error)) {
            std::cerr << "failed to convert raw Android GNSS: "
                      << conversion_error << "\n";
            return 1;
        }
        if (android_gnss.observations.isEmpty()) {
            std::cerr << "raw Android GNSS has no usable observation epochs\n";
            return 1;
        }
        android_raw_epoch_times.reserve(android_gnss.observations.epochs.size());
        for (const auto& raw_epoch : android_gnss.observations.epochs) {
            android_raw_epoch_times.push_back(raw_epoch.time);
        }
        // Raw mode consumes the Android observation series directly.  It does
        // not synthesize a RINEX intermediate and does not use the optional
        // device-provided WLS position retained by the adapter.
        obs_header.interval = 1.0;
    } else {
        if (!obs_reader.open(options.obs_path)) {
            std::cerr << "failed to open observation file\n";
            return 1;
        }
        if (!obs_reader.readHeader(obs_header)) {
            std::cerr << "failed to read observation header\n";
            return 1;
        }
    }
    libgnss::io::RINEXReader nav_reader;
    if (!nav_reader.open(options.nav_path)) {
        std::cerr << "failed to open navigation file\n";
        return 1;
    }
    libgnss::NavigationData nav;
    if (!nav_reader.readNavigationData(nav)) {
        std::cerr << "failed to read navigation file\n";
        return 1;
    }

    std::vector<libgnss::ObservationData> epochs;
    libgnss::ObservationData epoch;
    int observation_index = 0;
    if (android_raw) {
        for (auto& raw_epoch : android_gnss.observations.epochs) {
            if (observation_index++ < options.skip_epochs) continue;
            if (!options.all_epochs &&
                epochs.size() >= static_cast<std::size_t>(options.max_epochs)) {
                break;
            }
            // SPPProcessor uses the epoch position only as a numerical initial
            // frame when no receiver seed is supplied.  This fixed Earth
            // surface point is deliberately route/device independent; it is
            // not emitted and cannot carry a device-WLS/result coordinate.
            raw_epoch.receiver_position = libgnss::Vector3d(6378137.0, 0.0, 0.0);
            epochs.push_back(std::move(raw_epoch));
        }
    } else {
        while ((options.all_epochs ||
                epochs.size() < static_cast<std::size_t>(options.max_epochs)) &&
               obs_reader.readObservationEpoch(epoch)) {
            if (observation_index++ < options.skip_epochs) continue;
            if (obs_header.approximate_position.norm() > 1.0e6) {
                epoch.receiver_position = obs_header.approximate_position;
            }
            epochs.push_back(epoch);
        }
    }
    if (epochs.size() < 2) {
        std::cerr << "fewer than two observation epochs\n";
        return 1;
    }

    libgnss::carrier_code_leveling::Diagnostics carrier_code_leveling_report;
    if (options.native_carrier_code_leveling) {
        // Keep the leveling transform in the same process and before the FGO
        // problem builder.  Only raw P is changed for the explicitly selected
        // primary signals; carrier, Doppler, timestamps, nav, and every other
        // signal remain untouched.
        libgnss::ObservationSeries raw_series;
        raw_series.epochs = epochs;
        libgnss::carrier_code_leveling::Config leveling_config;
        leveling_config.enable_innovation_reset =
            options.native_carrier_code_innovation_reset;
        if (options.native_carrier_code_primary_l1_e1) {
            leveling_config.signals = {libgnss::SignalType::GPS_L1CA,
                                       libgnss::SignalType::GAL_E1};
        }
        leveling_config.include_gal_e5a = options.native_carrier_code_gal_e1_e5a;
        const auto leveling = libgnss::carrier_code_leveling::apply(
            raw_series, android_gnss.epoch_utc_time_millis,
            android_gnss.epoch_hardware_clock_discontinuity_count,
            leveling_config);
        if (!leveling.ok) {
            std::cerr << "carrier-code leveling failed closed: "
                      << leveling.error << "\n";
            return 1;
        }
        epochs = leveling.observations.epochs;
        carrier_code_leveling_report = leveling.diagnostics;
    }

    libgnss::FGOProcessor::FGOConfig config;
    config.backend = libgnss::FGOBackend::GTSAM;
    config.max_iterations = 12;
    config.use_pose3_state = true;
    config.use_imu = true;
    config.use_double_difference_factors = false;
    config.use_pseudorange_factors = true;
    config.use_carrier_phase_factors = false;
    // Ordinary TDCP remains disabled for the historical raw lane.  The
    // Phase10 candidate enables the already audited same-satellite/same-signal
    // ADR path only through its explicit opt-in, with no standalone ambiguity
    // or base/double-difference factors.
        config.use_tdcp_factors = options.native_pdc_imu_tdcp;
    config.use_single_difference_doppler_factors = false;
    config.use_single_difference_tdcp_factors = false;
    if (android_raw) {
        // Upstream's first GNSS pass is P+D with an explicit velocity state.
        // The corrected Android Doppler contract is already validated by the
        // raw adapter; keep it opt-in to this research entry point only.
        config.use_undifferenced_doppler_factors = true;
        config.use_corrected_undifferenced_doppler_factors = true;
    }
    if (options.fgo_imu_sparse_recovery) {
        // One fixed, raw-only recovery candidate: retain epochs below the
        // normal four-satellite floor for the IMU/temporal chain and admit
        // all supported secondary raw frequencies to the same P/D graph.
        // This is opt-in; the production/default graph remains unchanged.
        config.retain_sparse_epochs_for_imu = true;
        config.use_multi_frequency_double_difference = true;
    }
    if (options.native_pdc_state_bridge) {
        // One frozen bridge recipe: solve the full native P+D+temporal state
        // from the same in-memory raw rows, then use finite states only as
        // initial values for the CombinedImuFactor graph. No duplicate P/D
        // priors are added because the graph owns those measurements.
        // Secondary-frequency DD is intentionally not enabled here; this is a
        // PDC-state bridge, not the previously rejected sparse multi-frequency
        // coverage candidate.
        config.retain_sparse_epochs_for_imu = true;
        config.use_doppler_velocity_wls_initialization = true;
        // Match the already-used raw native SPP seed contract: a receiver-only
        // first pass avoids rejecting low-count mixed-system epochs before the
        // in-process P+D state solve sees their finite rows.
        config.spp_model_intersystem_bias = false;
        config.use_native_pdc_state_bridge = true;
    }
    if (options.native_pdc_imu_tdcp) {
        // Freeze the ordinary TDCP contract explicitly instead of inheriting
        // mutable library defaults: 30 ms ADR-difference noise, a 2 s
        // adjacent-epoch limit, and fail-closed loss-of-lock/code-phase gates.
        config.tdcp_sigma_m = kNativeTdcpSigmaM;
        config.max_tdcp_gap_s = kNativeTdcpMaxGapS;
        config.reject_tdcp_loss_of_lock = true;
        config.reject_tdcp_code_phase_jump = true;
        config.tdcp_code_phase_jump_threshold_m =
            kNativeTdcpCodePhaseJumpThresholdM;
    }
    if (options.native_signal_bias_states) {
        // Phase11 candidate: attach static meter-valued receiver secondary
        // signal-bias states to raw undifferenced code factors.  The prior is
        // frozen by the phase record and only regularizes the global gauge;
        // all Phase10 P/D/TDCP/IMU settings remain unchanged.  The common
        // FGO eligibility gate admits secondary raw observations only when
        // multi-frequency mode is enabled; enabling that gate here is what
        // makes the declared GPS L5/Galileo E5a bias states observable.  It
        // does not form a DD/IFLC combination or alter the primary factors.
        config.use_multi_frequency_double_difference = true;
        config.use_receiver_signal_bias_states = true;
        config.receiver_signal_bias_prior_sigma_m = 1000.0;
    }
    if (options.native_residual_ionosphere) {
        // Phase12 candidate: retain the Phase11 static receiver IFB states
        // and add one raw-geometry-mapped residual L1 ionosphere state per
        // epoch.  These are fixed physical defaults (thin-shell mapping in
        // the contract header, weak zero gauge, and random walk); no value is
        // selected from truth or a trajectory.
        config.use_residual_ionosphere_states = true;
        config.residual_ionosphere_prior_sigma_m = 10.0;
        config.residual_ionosphere_random_walk_sigma_m_per_sqrt_s = 1.0;
        config.residual_ionosphere_max_abs_m = 30.0;
        config.residual_ionosphere_max_gap_s = 2.0;
    }
    if (options.native_upstream_quality) {
        // Phase13 fixed raw-observable contract.  This is deliberately kept
        // separate from the frozen Phase12 TDCP sigma: only code and
        // receiver-only Doppler factors receive the upstream SNR model.
        config.use_upstream_observable_quality = true;
        config.upstream_snr_percentile = 85.0;
        config.upstream_min_snr_dbhz = 20.0;
        config.upstream_min_elevation_deg = 5.0;
        config.upstream_max_adjacent_gap_s = 1.5;
        // Pixel devices in the declared development recipe do not use the
        // legacy carrier sign-offset list.  An empty model is intentional:
        // it prevents device identity from selecting a quality mask.
        config.upstream_device_model.clear();
    }
    if (options.native_upstream_stop_constraints) {
        // These values are the pinned upstream parameters; they are not
        // selected from a truth file or from a route score.  Keep them in the
        // config object so the backend and its structural diagnostics expose
        // the complete candidate contract.
        config.use_upstream_stop_constraints = true;
        config.upstream_stop_window_samples = 500;
        config.upstream_stop_acceleration_std_offset_mps2 = 0.08;
        config.upstream_stop_gyro_std_offset_radps = 0.005;
        config.upstream_stop_gyro_norm_max_radps = 0.05;
        config.upstream_stop_velocity_threshold_mps = 0.5;
        config.upstream_stop_velocity_sigma_mps = 0.01;
        config.upstream_stop_velocity_huber_k_sigma = 0.5;
        config.upstream_stop_pose_rotation_sigma_rad =
            0.1 * kPi / 180.0;
        config.upstream_stop_pose_translation_sigma_m = 0.02;
        config.upstream_stop_pose_huber_k_sigma = 0.5;
    }
    config.pose3_lever_arm_body_m = libgnss::Vector3d::Zero();
    if (android_raw) {
        // The raw path starts SPP from the route-independent Earth-surface
        // initializer above.  Applying a nonzero elevation mask at that
        // deliberately distant point can discard the whole first epoch
        // before SPP has a chance to converge.  The native SPP seed still
        // applies its finite geometry/health checks; this only makes raw
        // startup independent of a device-provided coordinate.
        config.min_elevation_deg = 0.0;
        config.min_snr_dbhz = 0.0;
    }
    const libgnss::FGOProcessor processor(config);
    libgnss::FGOProcessor::FGOProblem problem;
    NativePdcBridgeReport pdc_bridge_report;
    bool pdc_bridge_prepopulated = false;
    if (options.native_upstream_quality) {
        // Keep the frozen Phase12 PDC initializer independent of the new
        // residual/SNR masks.  The initializer is not a second output lane:
        // its finite state is passed in-memory into the quality-enabled graph,
        // while every final P/D factor is built from the quality problem below.
        // This prevents the upstream SNR sigma (which is intentionally much
        // smaller than the PDC bridge's broad physical gate) from making the
        // unchanged initializer fail before the candidate graph is tested.
        libgnss::FGOProcessor::FGOConfig bridge_config = config;
        bridge_config.use_upstream_observable_quality = false;
        const libgnss::FGOProcessor bridge_processor(bridge_config);
        auto bridge_problem =
            bridge_processor.buildPseudorangeProblem(epochs, nav);
        if (bridge_problem.epochs.size() != epochs.size() ||
            !populateNativePdcStateBridge(bridge_problem, pdc_bridge_report)) {
            if (pdc_bridge_report.failure.empty()) {
                pdc_bridge_report.failure =
                    "unfiltered Phase12 PDC initializer epoch alignment failed";
            }
            std::cerr << "native PDC state bridge failed closed: "
                      << pdc_bridge_report.failure << "\n";
            return 1;
        }
        problem = processor.buildPseudorangeProblem(epochs, nav);
        if (problem.epochs.size() != bridge_problem.epochs.size()) {
            std::cerr << "quality candidate changed epoch indexing relative to the "
                         "frozen PDC initializer\n";
            return 1;
        }
        problem.native_pdc_state_seeds =
            std::move(bridge_problem.native_pdc_state_seeds);
        pdc_bridge_prepopulated = true;
    } else {
        problem = processor.buildPseudorangeProblem(epochs, nav);
    }
    if (problem.epochs.size() < 2 || problem.pseudorange_factors.empty()) {
        std::cerr << "no-base problem has insufficient seeded pseudorange factors\n";
        return 1;
    }

    if (options.native_pdc_state_bridge &&
        !pdc_bridge_prepopulated &&
        !populateNativePdcStateBridge(problem, pdc_bridge_report)) {
        std::cerr << "native PDC state bridge failed closed: "
                  << pdc_bridge_report.failure << "\n";
        return 1;
    }

    ImuBuildReport imu_report;
    if (android_raw) {
        imu_report.android_gnss_diagnostics = android_gnss.diagnostics;
    }
    std::vector<libgnss::Vector3d> gnss_first_velocities_enu;
    bool gnss_first_ok = !android_raw;
    if (android_raw) {
        // Upstream run_fgo.m performs a GNSS-only pass before the IMU pass.
        // Keep that handoff in memory: no device-WLS, result file, or truth
        // position can enter the Android initialization path.
        imu_report.gnss_first_attempted = true;
        libgnss::FGOProcessor::FGOConfig gnss_first_config = config;
        gnss_first_config.backend = libgnss::FGOBackend::GTSAM;
        gnss_first_config.use_imu = false;
        gnss_first_config.use_pose3_state = false;
        gnss_first_config.use_velocity_states = true;
        gnss_first_config.use_velocity_motion_factors = false;
        gnss_first_config.use_doppler_velocity_wls_initialization = false;
        // Upstream fgo_gnss.m permits up to 1000 LM iterations.  The raw
        // GNSS-first pass is a separate initializer, so use that published
        // bound without changing the frozen 12-iteration IMU pass.
        gnss_first_config.max_iterations = 1000;
        const libgnss::FGOProcessor gnss_first_processor(gnss_first_config);
        const libgnss::FGOProcessor::FGOResult gnss_first_result =
            gnss_first_processor.optimizeProblem(problem);
        imu_report.gnss_first_converged = gnss_first_result.diagnostics.converged;
        imu_report.gnss_first_epochs = gnss_first_result.solution.solutions.size();
        imu_report.gnss_first_doppler_factors =
            problem.undifferenced_doppler_factors.size();
        imu_report.gnss_first_velocity_states =
            gnss_first_result.epoch_velocities_ecef_mps.size();
        imu_report.gnss_first_iterations = gnss_first_result.diagnostics.iterations;
        imu_report.gnss_first_initial_cost = gnss_first_result.diagnostics.initial_cost;
        imu_report.gnss_first_final_cost = gnss_first_result.diagnostics.final_cost;
        std::string gnss_first_error;
        gnss_first_ok = !gnss_first_result.solution.isEmpty() &&
                        gnss_first_result.diagnostics.converged &&
                        deriveGnssFirstVelocities(problem, gnss_first_result,
                                                  gnss_first_velocities_enu,
                                                  gnss_first_error);
        if (!gnss_first_ok) {
            imu_report.gnss_first_failure = gnss_first_error.empty()
                                                 ? "GNSS-first optimizer did not converge"
                                                 : gnss_first_error;
            std::cerr << "GNSS-first initialization unavailable: "
                      << imu_report.gnss_first_failure << "\n";
        } else {
            // Seed the following in-memory problem with the GNSS-only result,
            // including its receiver clock in the internal metre convention.
            // This is the exact raw-observation handoff; the output is never
            // published as a separate lane.
            const auto& gnss_solutions = gnss_first_result.solution.solutions;
            for (std::size_t i = 0; i < gnss_solutions.size(); ++i) {
                problem.epochs[i].position_ecef = gnss_solutions[i].position_ecef;
                if (std::isfinite(gnss_solutions[i].receiver_clock_bias)) {
                    problem.epochs[i].receiver_clock_bias_m =
                        gnss_solutions[i].receiver_clock_bias *
                        libgnss::constants::SPEED_OF_LIGHT;
                    problem.epochs[i].receiver_clock_bias_is_meters = true;
                }
            }
        }
    }
    const std::string imu_path = android_raw ? options.android_imu_path : options.imu_path;
    std::vector<libgnss::AndroidGnssTimeAnchor> gnss_time_anchors;
    libgnss::AndroidGnssUtcGpsMapping android_utc_gps_mapping;
    if (android_raw) {
        if (options.android_utc_wall_clock_fallback) {
            // Prefer the authoritative monotonic Android anchor whenever it
            // is present.  The opt-in wall-clock mapping is a portability
            // fallback for the mi8-style all-blank elapsed column, never a
            // second competing clock selected for a route that already has
            // valid elapsed anchors.
            imu_report.android_gnss_anchor_load =
                libgnss::loadAndroidGnssTimeAnchors(options.android_gnss_path,
                                                    gnss_time_anchors);
            if (!imu_report.android_gnss_anchor_load.ok) {
                imu_report.android_gnss_utc_mapping_load =
                    libgnss::loadAndroidGnssUtcGpsMapping(
                        options.android_gnss_path, android_utc_gps_mapping);
                if (!imu_report.android_gnss_utc_mapping_load.ok) {
                    std::cerr << "failed to load raw GNSS UTC/GPS mapping: "
                              << imu_report.android_gnss_utc_mapping_load.error << "\n";
                    return 1;
                }
            }
        } else {
            imu_report.android_gnss_anchor_load =
                libgnss::loadAndroidGnssTimeAnchors(options.android_gnss_path,
                                                    gnss_time_anchors);
            if (!imu_report.android_gnss_anchor_load.ok) {
                std::cerr << "failed to load GNSS elapsed-time anchors: "
                          << imu_report.android_gnss_anchor_load.error << "\n";
                return 1;
            }
        }
    }
    bool use_imu = buildImuInput(imu_path, problem, imu_report, android_raw,
                                 gnss_time_anchors,
                                 android_raw && gnss_first_ok
                                     ? &gnss_first_velocities_enu
                                     : nullptr,
                                 options.android_utc_wall_clock_fallback
                                     ? &android_utc_gps_mapping
                                     : nullptr);
    bool fallback = false;
    libgnss::FGOProcessor::FGOResult result;
    if (use_imu) {
        result = processor.optimizeProblem(problem);
        if (result.solution.isEmpty() || !result.diagnostics.converged ||
            result.diagnostics.imu_intervals == 0) {
            if (options.native_upstream_stop_constraints) {
                std::cerr << "upstream stop-constraint candidate failed closed; "
                             "fallback is forbidden for candidate evaluation\n";
                return 1;
            }
            use_imu = false;
            fallback = true;
        }
    }
    if (!use_imu) {
        fallback = true;
        problem.imu.valid = false;
        libgnss::FGOProcessor::FGOConfig fallback_config = config;
        fallback_config.use_imu = false;
        fallback_config.use_pose3_state = false;
        const libgnss::FGOProcessor fallback_processor(fallback_config);
        result = fallback_processor.optimizeProblem(problem);
    }
    if (result.solution.isEmpty() || !result.diagnostics.converged) {
        std::cerr << "FGO produced no finite converged output\n";
        return 1;
    }
    if (options.native_residual_ionosphere) {
        const auto& residual_diagnostics = result.diagnostics;
        const bool finite_states =
            result.residual_ionosphere_estimates_m.size() == problem.epochs.size() &&
            std::all_of(result.residual_ionosphere_estimates_m.begin(),
                        result.residual_ionosphere_estimates_m.end(),
                        [](double value) { return std::isfinite(value); });
        const bool structural_ok =
            !fallback &&
            problem.diagnostics.residual_ionosphere_invalid_coefficients == 0U &&
            residual_diagnostics.residual_ionosphere_invalid_coefficients == 0U &&
            residual_diagnostics.residual_ionosphere_factors ==
                problem.pseudorange_factors.size() &&
            residual_diagnostics.residual_ionosphere_states == problem.epochs.size() &&
            finite_states &&
            residual_diagnostics.residual_ionosphere_max_abs_m <=
                config.residual_ionosphere_max_abs_m &&
            residual_diagnostics.residual_ionosphere_factors > 0U &&
            std::isfinite(residual_diagnostics.initial_cost) &&
            std::isfinite(residual_diagnostics.final_cost) &&
            residual_diagnostics.final_cost <= residual_diagnostics.initial_cost;
        if (!structural_ok) {
            std::cerr << "residual-ionosphere structural contract failed closed: "
                      << "fallback=" << (fallback ? "true" : "false")
                      << " factors=" << residual_diagnostics.residual_ionosphere_factors
                      << "/" << problem.pseudorange_factors.size()
                      << " states=" << residual_diagnostics.residual_ionosphere_states
                      << "/" << problem.epochs.size()
                      << " invalid="
                      << residual_diagnostics.residual_ionosphere_invalid_coefficients
                      << " max_abs_m="
                      << residual_diagnostics.residual_ionosphere_max_abs_m << "\n";
            return 1;
        }
    }
    if (!problem.double_difference_pseudorange_factors.empty() ||
        !problem.double_difference_carrier_factors.empty()) {
        std::cerr << "no-base contract violated by problem builder\n";
        return 1;
    }

    const TdcpRuntimeReport tdcp_report = evaluateTdcpRuntime(
        problem, result, options.native_pdc_imu_tdcp);
    if (options.native_pdc_imu_tdcp &&
        (tdcp_report.factors_built == 0U ||
         tdcp_report.factors_inserted != tdcp_report.factors_built ||
         tdcp_report.nonfinite_residuals != 0U)) {
        std::cerr << "native PDC+IMU TDCP contract failed: built="
                  << tdcp_report.factors_built
                  << " inserted=" << tdcp_report.factors_inserted
                  << " nonfinite_residuals=" << tdcp_report.nonfinite_residuals
                  << "\n";
        return 1;
    }

    // Keep the bridge's basin effect observable without reading truth: compare
    // each final FGO position with the corresponding finite PDC initializer.
    // This is a diagnostic only; it never changes a state or selects a lane.
    if (options.native_pdc_state_bridge &&
        !problem.native_pdc_state_seeds.empty()) {
        std::vector<double> displacements;
        displacements.reserve(problem.native_pdc_state_seeds.size());
        for (std::size_t epoch_index = 0; epoch_index < problem.epochs.size();
             ++epoch_index) {
            const auto seed_it = std::find_if(
                problem.native_pdc_state_seeds.begin(),
                problem.native_pdc_state_seeds.end(),
                [epoch_index](const auto& seed) {
                    return seed.epoch_index == epoch_index;
                });
            if (seed_it == problem.native_pdc_state_seeds.end() ||
                epoch_index >= result.solution.solutions.size()) {
                continue;
            }
            const auto& solution = result.solution.solutions[epoch_index];
            const double displacement =
                (solution.position_ecef - seed_it->position_ecef).norm();
            if (std::isfinite(displacement)) displacements.push_back(displacement);
        }
        if (!displacements.empty()) {
            std::sort(displacements.begin(), displacements.end());
            const std::size_t middle = displacements.size() / 2U;
            pdc_bridge_report.fgo_seed_displacement_count = displacements.size();
            pdc_bridge_report.fgo_seed_displacement_p50_m =
                (displacements.size() % 2U == 0U)
                    ? 0.5 * (displacements[middle - 1U] + displacements[middle])
                    : displacements[middle];
            pdc_bridge_report.fgo_seed_displacement_max_m = displacements.back();
        }
    }

    UpstreamPositionOffsetReport position_offset_report;
    position_offset_report.enabled = options.native_upstream_position_offset;
    if (options.native_upstream_position_offset) {
        position_offset_report.phone = phoneFromDatasetId(options.dataset_id);
        libgnss::upstream_position_offset::PhoneOffset phone_offset;
        if (!libgnss::upstream_position_offset::phoneOffset(
                position_offset_report.phone, phone_offset)) {
            position_offset_report.failure = "unknown phone family";
            std::cerr << "upstream position offset failed closed: "
                      << position_offset_report.failure << "\n";
            return 1;
        }
        position_offset_report.offset_rl_m = phone_offset.offset_rl_m;
        position_offset_report.offset_ud_m = phone_offset.offset_ud_m;
        if (fallback || !problem.imu.valid ||
            result.epoch_attitude_rpy_rad.size() !=
                result.solution.solutions.size()) {
            position_offset_report.failure =
                "candidate requires a finite native IMU Pose3 rpy for every output epoch";
            std::cerr << "upstream position offset failed closed: "
                      << position_offset_report.failure << "\n";
            return 1;
        }
        const double origin_lat = problem.imu.nav_origin_lat_rad;
        const double origin_lon = problem.imu.nav_origin_lon_rad;
        if (!std::isfinite(origin_lat) || !std::isfinite(origin_lon)) {
            position_offset_report.failure = "native ENU origin is non-finite";
            std::cerr << "upstream position offset failed closed: "
                      << position_offset_report.failure << "\n";
            return 1;
        }
        for (std::size_t index = 0; index < result.solution.solutions.size();
             ++index) {
            auto& solution = result.solution.solutions[index];
            const auto offset = libgnss::upstream_position_offset::offsetFromRpy(
                position_offset_report.phone,
                result.epoch_attitude_rpy_rad[index]);
            if (!offset.ok || !solution.position_ecef.allFinite()) {
                position_offset_report.failure =
                    "non-finite native attitude or position";
                std::cerr << "upstream position offset failed closed: "
                          << position_offset_report.failure << "\n";
                return 1;
            }
            const double offset_norm = offset.offset_enu_m.norm();
            const libgnss::Vector3d corrected =
                solution.position_ecef +
                libgnss::enu2ecef(offset.offset_enu_m, origin_lat, origin_lon);
            if (!corrected.allFinite() || corrected.norm() < 6.0e6 ||
                corrected.norm() > 7.0e6 || !std::isfinite(offset_norm)) {
                position_offset_report.failure =
                    "corrected position or offset is non-finite/out-of-Earth";
                std::cerr << "upstream position offset failed closed: "
                          << position_offset_report.failure << "\n";
                return 1;
            }
            solution.position_ecef = corrected;
            double lat = 0.0;
            double lon = 0.0;
            double height = 0.0;
            libgnss::ecef2geodetic(corrected, lat, lon, height);
            if (!std::isfinite(lat) || !std::isfinite(lon) ||
                !std::isfinite(height)) {
                position_offset_report.failure =
                    "corrected geodetic position is non-finite";
                std::cerr << "upstream position offset failed closed: "
                          << position_offset_report.failure << "\n";
                return 1;
            }
            solution.position_geodetic = libgnss::GeodeticCoord(lat, lon, height);
            position_offset_report.max_offset_enu_m =
                std::max(position_offset_report.max_offset_enu_m, offset_norm);
            ++position_offset_report.corrected_epochs;
        }
        position_offset_report.applied =
            position_offset_report.corrected_epochs ==
            result.solution.solutions.size();
        if (!position_offset_report.applied) {
            position_offset_report.failure = "not every native solution was corrected";
            std::cerr << "upstream position offset failed closed: "
                      << position_offset_report.failure << "\n";
            return 1;
        }
    }

    RawUtcOutputReport raw_utc_report;
    std::vector<OutputPosition> output_positions;
    if (options.android_raw_utc_key_contract) {
        constexpr double kSolutionTimeToleranceMs = 2.0;
        std::vector<libgnss::io::AndroidRawGnssSolutionPoint> solution_points;
        solution_points.reserve(result.solution.solutions.size());
        for (const auto& solution : result.solution.solutions) {
            solution_points.push_back({solution.time, solution.position_ecef});
        }
        libgnss::io::AndroidRawGnssEpochAlignment alignment;
        std::string alignment_error;
        if (!libgnss::io::alignAndroidRawGnssSolutionsToUtcKeys(
                android_raw_epoch_times, android_gnss.epoch_utc_time_millis,
                solution_points, kSolutionTimeToleranceMs, alignment,
                alignment_error)) {
            std::cerr << "failed to align output to raw UTC keys: "
                      << alignment_error << "\n";
            return 1;
        }
        raw_utc_report.enabled = true;
        raw_utc_report.warmup_epoch_excluded = true;
        raw_utc_report.raw_epoch_keys = android_gnss.epoch_utc_time_millis.size();
        raw_utc_report.target_epochs = alignment.target_epochs;
        raw_utc_report.exact_solution_epochs = alignment.exact_solution_epochs;
        raw_utc_report.interpolated_epochs = alignment.interpolated_epochs;
        raw_utc_report.edge_hold_epochs = alignment.edge_hold_epochs;
        raw_utc_report.unresolved_epochs = alignment.unresolved_epochs;
        raw_utc_report.solution_time_tolerance_ms = kSolutionTimeToleranceMs;
        raw_utc_report.max_interpolation_gap_ms = alignment.max_interpolation_gap_ms;
        raw_utc_report.max_edge_hold_gap_ms = alignment.max_edge_hold_gap_ms;
        output_positions.reserve(alignment.epochs.size());
        for (const auto& aligned : alignment.epochs) {
            output_positions.push_back({aligned.utc_time_millis,
                                        aligned.position_ecef});
        }
    } else {
        output_positions.reserve(result.solution.solutions.size());
        for (const auto& solution : result.solution.solutions) {
            const double timestamp = unixMillis(solution.time);
            if (!std::isfinite(timestamp)) {
                std::cerr << "non-finite output timestamp\n";
                return 1;
            }
            output_positions.push_back({static_cast<std::int64_t>(timestamp),
                                        solution.position_ecef});
        }
    }

    std::ostringstream csv;
    csv << "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n";
    std::size_t finite_rows = 0;
    for (const auto& output_position : output_positions) {
        if (!output_position.position_ecef.allFinite() ||
            output_position.position_ecef.norm() < 6.0e6 ||
            output_position.position_ecef.norm() > 7.0e6) {
            std::cerr << "non-finite or out-of-Earth output row\n";
            return 1;
        }
        double lat = 0.0;
        double lon = 0.0;
        double output_height = 0.0;
        libgnss::ecef2geodetic(output_position.position_ecef, lat, lon,
                               output_height);
        if (!std::isfinite(lat) || !std::isfinite(lon) ||
            std::abs(lat) > kPi / 2.0 || std::abs(lon) > kPi) {
            std::cerr << "non-finite or out-of-range output row\n";
            return 1;
        }
        csv << options.dataset_id << ',' << output_position.utc_time_millis << ','
            << std::fixed << std::setprecision(10) << lat * kRadToDeg << ','
            << lon * kRadToDeg << '\n';
        ++finite_rows;
    }
    if (finite_rows == 0 || !atomicWrite(options.out_path, csv.str())) {
        std::cerr << "failed to atomically publish output\n";
        return 1;
    }
    const std::string summary = makeSummary(options, problem, result, imu_report,
                                            fallback, pdc_bridge_report,
                                            raw_utc_report, tdcp_report,
                                            carrier_code_leveling_report,
                                            position_offset_report);
    if (!atomicWrite(options.summary_path, summary)) {
        std::cerr << "failed to atomically publish summary\n";
        return 1;
    }
    std::cout << "native no-base FGO: epochs=" << problem.epochs.size()
              << " output=" << finite_rows
              << " imu_intervals=" << result.diagnostics.imu_intervals
              << " graph_factors=" << result.diagnostics.graph_factors
              << " graph_values=" << result.diagnostics.graph_values
              << " fallback=" << (fallback ? "yes" : "no") << '\n';
    return 0;
}
