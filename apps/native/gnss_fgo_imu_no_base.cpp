// Opt-in, no-base GNSS+IMU entry point for the smartphone research lane.
//
// This intentionally has a small surface: the existing FGO problem builder
// supplies no-base undifferenced pseudorange factors, while the GTSAM backend
// supplies the real Pose3/velocity/bias chain and CombinedImuFactor.  The
// production gnss_fgo defaults are not changed.  Android raw axes are never
// silently treated as body axes: the frozen taroz mounting rotation is
// applied explicitly to the already schema-converted IMU CSV.

#include <libgnss++/algorithms/fgo.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/fusion/fusion_initialization.hpp>
#include <libgnss++/io/imu.hpp>
#include <libgnss++/io/rinex.hpp>

#include <Eigen/Geometry>

#include <algorithm>
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

struct Options {
    std::string obs_path;
    std::string nav_path;
    std::string imu_path;
    std::string out_path;
    std::string summary_path;
    std::string dataset_id = "native-fgo-v2-imu-no-base";
    int max_epochs = kDefaultEpochLimit;
    int skip_epochs = 0;
};

void usage(const char* program) {
    std::cout << "Usage: " << program
              << " --obs <rover.obs> --nav <brdc.nav> --imu <imu.csv>"
                 " --out <submission.csv> --summary-json <summary.json>"
                 " [--dataset-id <id>] [--skip-epochs <n>] [--max-epochs 10..30]\n";
}

bool requireValue(int argc, char** argv, int& index, std::string& value) {
    if (index + 1 >= argc) {
        return false;
    }
    value = argv[++index];
    return !value.empty();
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
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            return false;
        }
    }
    if (options.obs_path.empty() || options.nav_path.empty() || options.imu_path.empty() ||
        options.out_path.empty() || options.summary_path.empty()) {
        usage(argv[0]);
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

struct ImuBuildReport {
    bool ok = false;
    bool heading_latched = false;
    std::string failure;
    std::size_t loaded_samples = 0;
    std::size_t stationary_samples = 0;
    double gravity_mean_norm = 0.0;
    double gravity_norm_std = 0.0;
    int heading_windows = 0;
};

bool buildImuInput(const std::string& path,
                   libgnss::FGOProcessor::FGOProblem& problem,
                   ImuBuildReport& report) {
    if (problem.epochs.size() < 2) {
        report.failure = "fewer than two GNSS epochs";
        return false;
    }

    libgnss::ImuSeries series;
    const auto load = libgnss::loadImuCsv(path, series);
    if (!load.ok || series.isEmpty()) {
        report.failure = load.error.empty() ? "empty IMU series" : load.error;
        return false;
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
    if (!report.heading_latched) {
        report.failure = "GNSS course did not make heading observable under frozen gate";
        return false;
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
        << "  \"schema_version\": \"smartphone-r5-native-fgo-v2-mat-no-base-run.v1\",\n"
        << "  \"dataset_id\": ";
    writeJsonString(out, options.dataset_id);
    out << ",\n  \"status\": "
        << (fallback ? "\"fallback-native-fgo-v1\"" : "\"imu-combined-factor\"") << ",\n"
        << "  \"truth_used\": false,\n"
        << "  \"base_factors\": false,\n"
        << "  \"no_base_contract\": true,\n"
        << "  \"production_default_changed\": false,\n"
        << "  \"skip_epochs\": " << options.skip_epochs << ",\n"
        << "  \"inputs\": {\n"
        << "    \"observation\": ";
    writeJsonString(out, options.obs_path);
    out << ",\n    \"navigation\": ";
    writeJsonString(out, options.nav_path);
    out << ",\n    \"imu\": ";
    writeJsonString(out, options.imu_path);
    out << "\n  },\n"
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
        << "  \"imu_initialization\": {\n"
        << "    \"loaded_samples\": " << imu_report.loaded_samples << ",\n"
        << "    \"stationary_samples\": " << imu_report.stationary_samples << ",\n"
        << "    \"gravity_mean_norm_mps2\": " << imu_report.gravity_mean_norm << ",\n"
        << "    \"gravity_norm_std_mps2\": " << imu_report.gravity_norm_std << ",\n"
        << "    \"heading_windows\": " << imu_report.heading_windows << ",\n"
        << "    \"heading_latched\": " << (imu_report.heading_latched ? "true" : "false") << ",\n"
        << "    \"axis_contract\": \"identity raw-axis selection + explicit RzRyRx mounting\",\n"
        << "    \"mounting_rpy_deg_xyz\": [-85.0, 178.0, -94.0],\n"
        << "    \"timestamp_contract\": \"GPST CSV; no runtime offset\",\n"
        << "    \"failure\": ";
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

    libgnss::io::RINEXReader obs_reader;
    if (!obs_reader.open(options.obs_path)) {
        std::cerr << "failed to open observation file\n";
        return 1;
    }
    libgnss::io::RINEXReader::RINEXHeader obs_header;
    if (!obs_reader.readHeader(obs_header)) {
        std::cerr << "failed to read observation header\n";
        return 1;
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
    while (epochs.size() < static_cast<std::size_t>(options.max_epochs) &&
           obs_reader.readObservationEpoch(epoch)) {
        if (observation_index++ < options.skip_epochs) continue;
        if (obs_header.approximate_position.norm() > 1.0e6) {
            epoch.receiver_position = obs_header.approximate_position;
        }
        epochs.push_back(epoch);
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
    config.pose3_lever_arm_body_m = libgnss::Vector3d::Zero();
    const libgnss::FGOProcessor processor(config);
    libgnss::FGOProcessor::FGOProblem problem =
        processor.buildPseudorangeProblem(epochs, nav);
    if (problem.epochs.size() < 2 || problem.pseudorange_factors.empty()) {
        std::cerr << "no-base problem has insufficient seeded pseudorange factors\n";
        return 1;
    }

    ImuBuildReport imu_report;
    bool use_imu = buildImuInput(options.imu_path, problem, imu_report);
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
