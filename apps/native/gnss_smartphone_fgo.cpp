// Opt-in native-only smartphone FGO entry point.
//
// This executable consumes raw Android device_gnss.csv plus broadcast RINEX
// navigation and calls the existing libgnss++ FGO processor.  It deliberately
// has no result-MAT, truth, sample-coordinate, or imported-v5 input path.  A
// separate native-only preflight command hashes the required device_imu.csv
// companion and is expected to be run before this executable. A finite,
// device-provided WLS ECEF field may be used only as an approximate SPP seed;
// result-MAT/submission coordinates are rejected by the path contract.

#include <libgnss++/algorithms/fgo.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/io/android_raw_gnss.hpp>
#include <libgnss++/io/rinex.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <unistd.h>

namespace {

constexpr double kGpsEpochUnixSeconds = 315964800.0;
constexpr double kGpsUtcLeapSeconds = 18.0;
constexpr double kSecondsPerWeek = 604800.0;
constexpr double kPi = 3.1415926535897932384626433832795;

struct Options {
    std::string device_gnss;
    std::string device_imu;
    std::string nav;
    std::string output;
    std::string summary;
    std::string trip_id = "native/phone";
    std::string device_model;
    int max_epochs = 0;
    bool include_galileo_e1 = true;
};

void usage(const char* program) {
    std::cout
        << "Usage: " << program
        << " --device-gnss device_gnss.csv --device-imu device_imu.csv"
           " --nav brdc.nav --out result.csv --summary-json summary.json\n"
        << "Options:\n"
        << "  --trip-id <route/phone>     Key prefix for native output\n"
        << "  --device-model <model>      Published ADR sign policy selector\n"
        << "  --max-epochs <n>             Limit native epochs (0 = all)\n"
        << "  --no-galileo-e1              Keep GPS L1 C/A only\n"
        << "  --help                       Show this help\n";
}

bool valueFor(int argc, char** argv, int& index, std::string& value) {
    if (index + 1 >= argc) return false;
    value = argv[++index];
    return !value.empty();
}

bool parseOptions(int argc, char** argv, Options& options) {
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--help" || argument == "-h") {
            usage(argv[0]);
            return false;
        }
        if (argument == "--device-gnss") {
            if (!valueFor(argc, argv, index, options.device_gnss)) return false;
        } else if (argument == "--device-imu") {
            if (!valueFor(argc, argv, index, options.device_imu)) return false;
        } else if (argument == "--nav") {
            if (!valueFor(argc, argv, index, options.nav)) return false;
        } else if (argument == "--out") {
            if (!valueFor(argc, argv, index, options.output)) return false;
        } else if (argument == "--summary-json") {
            if (!valueFor(argc, argv, index, options.summary)) return false;
        } else if (argument == "--trip-id") {
            if (!valueFor(argc, argv, index, options.trip_id)) return false;
        } else if (argument == "--device-model") {
            if (!valueFor(argc, argv, index, options.device_model)) return false;
        } else if (argument == "--max-epochs") {
            std::string value;
            if (!valueFor(argc, argv, index, value)) return false;
            try {
                std::size_t consumed = 0;
                options.max_epochs = std::stoi(value, &consumed);
                if (consumed != value.size() || options.max_epochs < 0) return false;
            } catch (...) {
                return false;
            }
        } else if (argument == "--no-galileo-e1") {
            options.include_galileo_e1 = false;
        } else {
            std::cerr << "unknown argument: " << argument << "\n";
            return false;
        }
    }
    if (options.device_gnss.empty() || options.device_imu.empty() ||
        options.nav.empty() || options.output.empty() || options.summary.empty()) {
        usage(argv[0]);
        return false;
    }
    return true;
}

bool forbiddenPath(const std::string& path) {
    std::string lowered = path;
    std::transform(lowered.begin(), lowered.end(), lowered.begin(),
                   [](unsigned char character) {
                       return static_cast<char>(std::tolower(character));
                   });
    constexpr const char* markers[] = {
        "result_gnss", "ground_truth", "gt.mat", "sample_submission",
        "submission.csv", "submission_", "native-fgo-test-v5", "upstream-mat",
        "precomputed"};
    for (const auto* marker : markers) {
        if (lowered.find(marker) != std::string::npos) return true;
    }
    return false;
}

bool regularFile(const std::string& path) {
    std::error_code error;
    return std::filesystem::is_regular_file(path, error) && !error;
}

bool atomicWrite(const std::string& path, const std::string& content) {
    const std::filesystem::path destination(path);
    std::error_code error;
    if (!destination.parent_path().empty()) {
        std::filesystem::create_directories(destination.parent_path(), error);
        if (error) return false;
    }
    const std::string temporary = path + ".tmp." + std::to_string(static_cast<long long>(::getpid()));
    {
        std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
        if (!output.is_open()) return false;
        output << content;
        output.flush();
        if (!output.good()) return false;
    }
    std::filesystem::rename(temporary, path, error);
    if (error) {
        std::filesystem::remove(temporary);
        return false;
    }
    return true;
}

long long unixMillis(const libgnss::GNSSTime& time) {
    const double unix_seconds = kGpsEpochUnixSeconds +
                                static_cast<double>(time.week) * kSecondsPerWeek +
                                time.tow - kGpsUtcLeapSeconds;
    return static_cast<long long>(std::llround(unix_seconds * 1000.0));
}

std::string jsonEscape(const std::string& value) {
    std::ostringstream output;
    for (const char character : value) {
        switch (character) {
            case '\\': output << "\\\\"; break;
            case '"': output << "\\\""; break;
            case '\n': output << "\\n"; break;
            case '\r': output << "\\r"; break;
            case '\t': output << "\\t"; break;
            default: output << character; break;
        }
    }
    return output.str();
}

std::string makeSummary(const Options& options,
                        const libgnss::io::AndroidRawGnssResult& converted,
                        const libgnss::FGOProcessor::FGOResult& result,
                        std::size_t source_epochs,
                        double wall_seconds) {
    const auto& counts = converted.diagnostics;
    std::ostringstream output;
    output << std::setprecision(std::numeric_limits<double>::max_digits10);
    output << "{\n"
           << "  \"schema_version\": \"smartphone-r5-native-only-fgo.v1\",\n"
           << "  \"native_inference_only\": true,\n"
           << "  \"truth_used\": false,\n"
           << "  \"result_mat_read\": false,\n"
           << "  \"sample_coordinates_read\": false,\n"
           << "  \"device_wls_seed_used\": "
           << (counts.receiver_position_rows > 0 ? "true" : "false") << ",\n"
           << "  \"device_gnss\": \"" << jsonEscape(options.device_gnss) << "\",\n"
           << "  \"device_imu_declared\": \"" << jsonEscape(options.device_imu) << "\",\n"
           << "  \"broadcast_nav\": \"" << jsonEscape(options.nav) << "\",\n"
           << "  \"trip_id\": \"" << jsonEscape(options.trip_id) << "\",\n"
           << "  \"device_model\": \"" << jsonEscape(options.device_model) << "\",\n"
           << "  \"source_epochs\": " << source_epochs << ",\n"
           << "  \"converted_epochs\": " << converted.observations.epochs.size() << ",\n"
           << "  \"output_epochs\": " << result.solution.solutions.size() << ",\n"
           << "  \"wall_seconds\": " << wall_seconds << ",\n"
           << "  \"conversion\": {\n"
           << "    \"input_rows\": " << counts.input_rows << ",\n"
           << "    \"raw_rows\": " << counts.raw_rows << ",\n"
           << "    \"selected_rows\": " << counts.selected_rows << ",\n"
           << "    \"selected_epochs\": " << counts.selected_epochs << ",\n"
           << "    \"skipped_invalid_timing_rows\": "
           << counts.skipped_invalid_timing_rows << ",\n"
           << "    \"device_wls_seed_rows\": " << counts.receiver_position_rows << ",\n"
           << "    \"gps_rows\": " << counts.gps_rows << ",\n"
           << "    \"galileo_e1_rows\": " << counts.galileo_e1_rows << ",\n"
           << "    \"carrier_rows\": " << counts.carrier_rows << ",\n"
           << "    \"doppler_rows\": " << counts.doppler_rows << ",\n"
           << "    \"carrier_loss_rows\": " << counts.carrier_loss_rows << ",\n"
           << "    \"clock_discontinuities\": " << counts.clock_discontinuities << ",\n"
           << "    \"timing_formula\": \"" << jsonEscape(counts.timing_formula) << "\",\n"
           << "    \"carrier_formula\": \"" << jsonEscape(counts.carrier_formula) << "\",\n"
           << "    \"doppler_formula\": \"" << jsonEscape(counts.doppler_formula) << "\"\n"
           << "  },\n"
           << "  \"fgo\": {\n"
           << "    \"backend\": \"eigen\",\n"
           << "    \"pseudorange_factors\": " << result.diagnostics.pseudorange_factors << ",\n"
           << "    \"tdcp_factors\": " << result.diagnostics.tdcp_factors << ",\n"
           << "    \"motion_factors\": " << result.diagnostics.motion_factors << ",\n"
           << "    \"iterations\": " << result.diagnostics.iterations << ",\n"
           << "    \"converged\": " << (result.diagnostics.converged ? "true" : "false") << ",\n"
           << "    \"residual_rms_m\": " << result.diagnostics.residual_rms_m << "\n"
           << "  },\n"
           << "  \"forbidden_input_policy\": {\n"
           << "    \"result_gnss_mat\": \"reject\",\n"
           << "    \"result_gnss_imu_mat\": \"reject\",\n"
           << "    \"ground_truth\": \"reject\",\n"
           << "    \"submission_coordinates\": \"reject\",\n"
           << "    \"v5_imported_output\": \"reject\",\n"
           << "    \"device_wls_seed_coordinates\": \"raw-device-approximate-seed-only\"\n"
           << "  }\n"
           << "}\n";
    return output.str();
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!parseOptions(argc, argv, options)) return 2;
    if (forbiddenPath(options.device_gnss) || forbiddenPath(options.device_imu) ||
        forbiddenPath(options.nav)) {
        std::cerr << "native-only input contract rejected a forbidden path\n";
        return 2;
    }
    if (!regularFile(options.device_gnss) || !regularFile(options.device_imu) ||
        !regularFile(options.nav)) {
        std::cerr << "native-only input contract requires regular GNSS/IMU/nav files\n";
        return 2;
    }

    libgnss::io::AndroidRawGnssConfig converter_config;
    converter_config.device_model = options.device_model;
    converter_config.include_galileo_e1 = options.include_galileo_e1;
    libgnss::io::AndroidRawGnssResult converted;
    std::string conversion_error;
    if (!libgnss::io::loadAndroidRawGnssCsv(
            options.device_gnss, converter_config, converted, conversion_error)) {
        std::cerr << "native Android GNSS conversion failed: " << conversion_error << "\n";
        return 1;
    }
    if (options.max_epochs > 0 &&
        converted.observations.epochs.size() > static_cast<std::size_t>(options.max_epochs)) {
        converted.observations.epochs.resize(static_cast<std::size_t>(options.max_epochs));
    }

    libgnss::io::RINEXReader nav_reader;
    if (!nav_reader.open(options.nav)) {
        std::cerr << "failed to open broadcast navigation\n";
        return 1;
    }
    libgnss::NavigationData navigation;
    if (!nav_reader.readNavigationData(navigation)) {
        std::cerr << "failed to decode broadcast navigation\n";
        return 1;
    }

    libgnss::FGOProcessor::FGOConfig config;
    config.backend = libgnss::FGOBackend::Eigen;
    config.max_iterations = 8;
    config.use_multi_constellation = options.include_galileo_e1;
    config.use_pseudorange_factors = true;
    config.use_tdcp_factors = true;
    config.use_motion_factors = true;
    config.use_position_motion_factors = true;
    config.use_clock_motion_factors = true;
    config.use_double_difference_factors = false;
    config.use_undifferenced_doppler_factors = false;
    config.use_single_difference_doppler_factors = false;
    config.use_single_difference_tdcp_factors = false;
    config.use_carrier_phase_factors = false;
    const libgnss::FGOProcessor processor(config);
    const auto start = std::chrono::steady_clock::now();
    const auto result = processor.optimize(converted.observations.epochs, navigation);
    const auto finish = std::chrono::steady_clock::now();
    const double wall_seconds =
        std::chrono::duration<double>(finish - start).count();
    if (result.solution.isEmpty()) {
        std::cerr << "native FGO produced no output epochs\n";
        return 1;
    }

    std::ostringstream csv;
    csv << "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n";
    for (const auto& solution : result.solution.solutions) {
        double latitude = solution.position_geodetic.latitude;
        double longitude = solution.position_geodetic.longitude;
        double height = solution.position_geodetic.height;
        if (!std::isfinite(latitude) || !std::isfinite(longitude) ||
            !std::isfinite(height)) {
            libgnss::ecef2geodetic(solution.position_ecef, latitude, longitude, height);
        }
        if (!std::isfinite(latitude) || !std::isfinite(longitude) ||
            std::abs(latitude) > kPi / 2.0 || std::abs(longitude) > kPi) {
            std::cerr << "native FGO produced a non-finite/out-of-range coordinate\n";
            return 1;
        }
        csv << options.trip_id << ',' << unixMillis(solution.time) << ','
            << std::fixed << std::setprecision(10)
            << latitude * 180.0 / kPi << ',' << longitude * 180.0 / kPi << '\n';
    }
    if (!atomicWrite(options.output, csv.str())) {
        std::cerr << "failed to atomically publish native FGO output\n";
        return 1;
    }
    const std::string summary = makeSummary(
        options, converted, result, converted.observations.epochs.size(), wall_seconds);
    if (!atomicWrite(options.summary, summary)) {
        std::cerr << "failed to atomically publish native FGO summary\n";
        return 1;
    }
    std::cout << "native smartphone FGO: epochs=" << converted.observations.epochs.size()
              << " output=" << result.solution.solutions.size()
              << " pr_factors=" << result.diagnostics.pseudorange_factors
              << " tdcp_factors=" << result.diagnostics.tdcp_factors
              << " converged=" << (result.diagnostics.converged ? "yes" : "no")
              << " wall_s=" << wall_seconds << '\n';
    return 0;
}
