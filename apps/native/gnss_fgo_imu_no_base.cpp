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
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
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
};

void usage(const char* program) {
    std::cout << "Usage: " << program
              << " (--obs <rover.obs> --imu <imu.csv>"
                 " | --android-gnss <device_gnss.csv> --android-imu <device_imu.csv>)"
                 " --nav <brdc.nav>"
                 " --out <submission.csv> --summary-json <summary.json>"
                 " [--dataset-id <id>] [--skip-epochs <n>]"
                 " [--max-epochs 10..30 | --all-epochs]\n";
}

bool requireValue(int argc, char** argv, int& index, std::string& value) {
    if (index + 1 >= argc) {
        return false;
    }
    value = argv[++index];
    return !value.empty();
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
};

bool buildImuInput(const std::string& path,
                   libgnss::FGOProcessor::FGOProblem& problem,
                   ImuBuildReport& report,
                   bool android_raw = false,
                   const std::vector<libgnss::AndroidGnssTimeAnchor>& gnss_time_anchors = {},
                   const std::vector<libgnss::Vector3d>* gnss_first_velocities_enu = nullptr) {
    if (problem.epochs.size() < 2) {
        report.failure = "fewer than two GNSS epochs";
        return false;
    }

    libgnss::ImuSeries series;
    report.android_raw = android_raw;
    if (android_raw) {
        libgnss::AndroidImuCsvConfig android_config;
        android_config.require_gnss_elapsed_anchor = true;
        android_config.imu_sync_coefficient = kUpstreamImuSyncCoefficient;
        report.android_load = libgnss::loadAndroidImuCsv(
            path, series, android_config, gnss_time_anchors);
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
                        bool fallback) {
    std::ostringstream out;
    out << std::setprecision(17);
    out << "{\n"
        << "  \"schema_version\": \"smartphone-r5-native-fgo-android-imu-no-base-run.v2\",\n"
        << "  \"dataset_id\": ";
    writeJsonString(out, options.dataset_id);
    out << ",\n  \"status\": "
        << (fallback ? "\"fallback-native-fgo-v1\"" : "\"imu-combined-factor\"") << ",\n"
        << "  \"truth_used\": false,\n"
        << "  \"base_factors\": false,\n"
        << "  \"no_base_contract\": true,\n"
        << "  \"production_default_changed\": false,\n"
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
        << "    \"pseudorange_factors\": " << problem.pseudorange_factors.size() << ",\n"
        << "    \"tdcp_factors_built\": " << problem.tdcp_factors.size() << ",\n"
        << "    \"double_difference_pseudorange_factors\": "
        << problem.double_difference_pseudorange_factors.size() << ",\n"
        << "    \"double_difference_carrier_factors\": "
        << problem.double_difference_carrier_factors.size() << "\n"
        << "  },\n"
        << "  \"graph\": {\n"
        << "    \"factors\": " << result.diagnostics.graph_factors << ",\n"
        << "    \"values\": " << result.diagnostics.graph_values << ",\n"
        << "    \"imu_intervals\": " << result.diagnostics.imu_intervals << ",\n"
        << "    \"iterations\": " << result.diagnostics.iterations << ",\n"
        << "    \"converged\": " << (result.diagnostics.converged ? "true" : "false") << ",\n"
        << "    \"initial_cost\": " << result.diagnostics.initial_cost << ",\n"
        << "    \"final_cost\": " << result.diagnostics.final_cost << "\n"
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
                           ? "GNSS ChipsetElapsedRealtimeNanos -> UTC milliseconds by linear interpolation/extrapolation; gyro anchors; sync coefficient 0.5; UTC + 18 s -> GPST"
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
            << "      \"first_gyro_elapsed_ns\": "
            << alignment.first_gyro_elapsed_ns << ",\n"
            << "      \"last_gyro_elapsed_ns\": "
            << alignment.last_gyro_elapsed_ns << ",\n"
            << "      \"elapsed_clock_preserved\": "
            << (alignment.elapsed_clock_preserved ? "true" : "false") << ",\n"
            << "      \"gnss_elapsed_anchor_applied\": "
            << (alignment.gnss_elapsed_anchor_applied ? "true" : "false") << "\n"
            << "    },\n";
    }
    out << "    \"failure\": ";
    writeJsonString(out, imu_report.failure);
    out << "\n  },\n"
        << "  \"output_contract\": {\n"
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

    libgnss::FGOProcessor::FGOConfig config;
    config.backend = libgnss::FGOBackend::GTSAM;
    config.max_iterations = 12;
    config.use_pose3_state = true;
    config.use_imu = true;
    config.use_double_difference_factors = false;
    config.use_pseudorange_factors = true;
    config.use_carrier_phase_factors = false;
    config.use_tdcp_factors = false;
    config.use_single_difference_doppler_factors = false;
    config.use_single_difference_tdcp_factors = false;
    if (android_raw) {
        // Upstream's first GNSS pass is P+D with an explicit velocity state.
        // The corrected Android Doppler contract is already validated by the
        // raw adapter; keep it opt-in to this research entry point only.
        config.use_undifferenced_doppler_factors = true;
        config.use_corrected_undifferenced_doppler_factors = true;
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
    libgnss::FGOProcessor::FGOProblem problem =
        processor.buildPseudorangeProblem(epochs, nav);
    if (problem.epochs.size() < 2 || problem.pseudorange_factors.empty()) {
        std::cerr << "no-base problem has insufficient seeded pseudorange factors\n";
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
                }
            }
        }
    }
    const std::string imu_path = android_raw ? options.android_imu_path : options.imu_path;
    std::vector<libgnss::AndroidGnssTimeAnchor> gnss_time_anchors;
    if (android_raw) {
        imu_report.android_gnss_anchor_load =
            libgnss::loadAndroidGnssTimeAnchors(options.android_gnss_path,
                                                gnss_time_anchors);
        if (!imu_report.android_gnss_anchor_load.ok) {
            std::cerr << "failed to load GNSS elapsed-time anchors: "
                      << imu_report.android_gnss_anchor_load.error << "\n";
            return 1;
        }
    }
    bool use_imu = buildImuInput(imu_path, problem, imu_report, android_raw,
                                 gnss_time_anchors,
                                 android_raw && gnss_first_ok
                                     ? &gnss_first_velocities_enu
                                     : nullptr);
    bool fallback = false;
    libgnss::FGOProcessor::FGOResult result;
    if (use_imu) {
        result = processor.optimizeProblem(problem);
        if (result.solution.isEmpty() || !result.diagnostics.converged ||
            result.diagnostics.imu_intervals == 0) {
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
    if (!problem.double_difference_pseudorange_factors.empty() ||
        !problem.double_difference_carrier_factors.empty()) {
        std::cerr << "no-base contract violated by problem builder\n";
        return 1;
    }

    std::ostringstream csv;
    csv << "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n";
    std::size_t finite_rows = 0;
    for (const auto& solution : result.solution.solutions) {
        double lat = solution.position_geodetic.latitude;
        double lon = solution.position_geodetic.longitude;
        if (!std::isfinite(lat) || !std::isfinite(lon)) {
            double output_height = 0.0;
            libgnss::ecef2geodetic(solution.position_ecef, lat, lon, output_height);
        }
        const double timestamp = unixMillis(solution.time);
        if (!std::isfinite(timestamp) || !std::isfinite(lat) || !std::isfinite(lon) ||
            std::abs(lat) > kPi / 2.0 || std::abs(lon) > kPi) {
            std::cerr << "non-finite or out-of-range output row\n";
            return 1;
        }
        csv << options.dataset_id << ',' << static_cast<long long>(timestamp) << ','
            << std::fixed << std::setprecision(10) << lat * kRadToDeg << ','
            << lon * kRadToDeg << '\n';
        ++finite_rows;
    }
    if (finite_rows == 0 || !atomicWrite(options.out_path, csv.str())) {
        std::cerr << "failed to atomically publish output\n";
        return 1;
    }
    const std::string summary = makeSummary(options, problem, result, imu_report, fallback);
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
