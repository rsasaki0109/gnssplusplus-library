// Truth-free raw Android Doppler measurement-contract audit.
//
// This executable is deliberately diagnostic-only.  It loads raw Android
// GNSS + broadcast navigation, constructs the same raw SPP-seeded FGO
// problem used by the Phase40 candidate, and writes the first epoch with at
// least four corrected undifferenced Doppler rows.  The row report keeps the
// source-domain Android rate, converted Hz observable, wavelength, broadcast
// satellite state, Earth-rotation state, LOS, known satellite range-rate,
// receiver-only residual, and the corresponding FGO factor fields side by
// side.  No truth, validation, holdout, MATLAB, result, submission, device
// WLS, or precomputed-coordinate input is accepted or consulted.

#include <libgnss++/algorithms/doppler_contract.hpp>
#include <libgnss++/algorithms/doppler_velocity_wls.hpp>
#include <libgnss++/algorithms/fgo.hpp>
#include <libgnss++/algorithms/spp_velocity.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/io/android_raw_gnss.hpp>
#include <libgnss++/io/rinex.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace {

using libgnss::FGOProcessor;
using libgnss::GNSSTime;
using libgnss::GNSSSystem;
using libgnss::Observation;
using libgnss::ObservationData;
using libgnss::SatelliteId;
using libgnss::SignalType;
using libgnss::Vector3d;

constexpr double kInitialRawSeedX = 6378137.0;
constexpr double kInitialRawSeedY = 0.0;
constexpr double kInitialRawSeedZ = 0.0;

struct Options {
    std::string android_gnss;
    std::string nav;
    std::string rows_csv;
    std::string summary_json;
    std::string dataset_id = "2021-08-24-20-32-us-ca-mtv-h/pixel5";
};

struct PreparedRow {
    bool state_valid = false;
    bool spp_candidate = false;
    Vector3d satellite_position = Vector3d::Zero();
    Vector3d satellite_velocity = Vector3d::Zero();
    Vector3d corrected_satellite_position = Vector3d::Zero();
    Vector3d corrected_satellite_velocity = Vector3d::Zero();
    Vector3d los_unrotated_receiver_to_satellite = Vector3d::Zero();
    Vector3d los_receiver_to_satellite = Vector3d::Zero();
    double satellite_clock_bias_s = std::numeric_limits<double>::quiet_NaN();
    double satellite_clock_drift_sps = std::numeric_limits<double>::quiet_NaN();
    double wavelength_m = std::numeric_limits<double>::quiet_NaN();
    double frequency_hz = std::numeric_limits<double>::quiet_NaN();
    double earth_rotation_angle_rad = std::numeric_limits<double>::quiet_NaN();
    double unrotated_satellite_range_rate_mps =
        std::numeric_limits<double>::quiet_NaN();
    double earth_rotation_sagnac_range_rate_mps =
        std::numeric_limits<double>::quiet_NaN();
    double earth_rotation_range_rate_delta_mps =
        std::numeric_limits<double>::quiet_NaN();
    double satellite_range_rate_mps = std::numeric_limits<double>::quiet_NaN();
    double measured_range_rate_mps = std::numeric_limits<double>::quiet_NaN();
    double expected_receiver_residual_mps =
        std::numeric_limits<double>::quiet_NaN();
};

struct RowKey {
    SatelliteId satellite;
    SignalType signal = SignalType::GPS_L1CA;

    bool operator<(const RowKey& other) const {
        if (satellite != other.satellite) return satellite < other.satellite;
        return static_cast<unsigned int>(signal) <
               static_cast<unsigned int>(other.signal);
    }
};

void usage(const char* program) {
    std::cout << "Usage: " << program
              << " --android-gnss device_gnss.csv --nav brdc.nav"
                 " --rows-csv audit_rows.csv --summary-json audit.json"
                 " [--dataset-id route/phone]\n";
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
        if (argument == "--android-gnss") {
            if (!valueFor(argc, argv, index, options.android_gnss)) return false;
        } else if (argument == "--nav") {
            if (!valueFor(argc, argv, index, options.nav)) return false;
        } else if (argument == "--rows-csv") {
            if (!valueFor(argc, argv, index, options.rows_csv)) return false;
        } else if (argument == "--summary-json") {
            if (!valueFor(argc, argv, index, options.summary_json)) return false;
        } else if (argument == "--dataset-id") {
            if (!valueFor(argc, argv, index, options.dataset_id)) return false;
        } else {
            std::cerr << "unknown argument: " << argument << "\n";
            return false;
        }
    }
    if (options.android_gnss.empty() || options.nav.empty() ||
        options.rows_csv.empty() || options.summary_json.empty()) {
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
        ".mat", "ground_truth", "validation", "holdout", "kaggle",
        "result_gnss", "sample_submission", "submission.csv", "precomputed",
        "device_wls"};
    for (const auto* marker : markers) {
        if (lowered.find(marker) != std::string::npos) return true;
    }
    return false;
}

bool regularFile(const std::string& path) {
    std::error_code error;
    return std::filesystem::is_regular_file(path, error) && !error;
}

bool atomicWrite(const std::string& path, const std::string& contents) {
    const std::filesystem::path destination(path);
    std::error_code error;
    if (!destination.parent_path().empty()) {
        std::filesystem::create_directories(destination.parent_path(), error);
        if (error) return false;
    }
    const std::filesystem::path temporary = destination.string() + ".tmp";
    {
        std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
        if (!output.is_open()) return false;
        output << contents;
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

const char* systemName(GNSSSystem system) {
    switch (system) {
        case GNSSSystem::GPS: return "GPS";
        case GNSSSystem::GLONASS: return "GLONASS";
        case GNSSSystem::Galileo: return "Galileo";
        case GNSSSystem::BeiDou: return "BeiDou";
        case GNSSSystem::QZSS: return "QZSS";
        case GNSSSystem::SBAS: return "SBAS";
        case GNSSSystem::NavIC: return "NavIC";
        default: return "UNKNOWN";
    }
}

const char* signalName(SignalType signal) {
    switch (signal) {
        case SignalType::GPS_L1CA: return "GPS_L1CA";
        case SignalType::GPS_L1P: return "GPS_L1P";
        case SignalType::GPS_L2P: return "GPS_L2P";
        case SignalType::GPS_L2C: return "GPS_L2C";
        case SignalType::GPS_L5: return "GPS_L5";
        case SignalType::GLO_L1CA: return "GLO_L1CA";
        case SignalType::GLO_L1P: return "GLO_L1P";
        case SignalType::GLO_L2CA: return "GLO_L2CA";
        case SignalType::GLO_L2P: return "GLO_L2P";
        case SignalType::GAL_E1: return "GAL_E1";
        case SignalType::GAL_E5A: return "GAL_E5A";
        case SignalType::GAL_E5B: return "GAL_E5B";
        case SignalType::GAL_E6: return "GAL_E6";
        case SignalType::BDS_B1I: return "BDS_B1I";
        case SignalType::BDS_B2I: return "BDS_B2I";
        case SignalType::BDS_B3I: return "BDS_B3I";
        case SignalType::BDS_B1C: return "BDS_B1C";
        case SignalType::BDS_B2A: return "BDS_B2A";
        case SignalType::QZS_L1CA: return "QZS_L1CA";
        case SignalType::QZS_L2C: return "QZS_L2C";
        case SignalType::QZS_L5: return "QZS_L5";
        default: return "UNKNOWN";
    }
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

long long unixMillis(const GNSSTime& time) {
    constexpr double kGpsEpochUnixSeconds = 315964800.0;
    constexpr double kGpsUtcLeapSeconds = 18.0;
    return static_cast<long long>(std::llround(
        (kGpsEpochUnixSeconds + static_cast<double>(time.week) * 604800.0 +
         time.tow - kGpsUtcLeapSeconds) * 1000.0));
}

bool finiteVector(const Vector3d& value) {
    return value.allFinite();
}

bool sameEpoch(const GNSSTime& lhs, const GNSSTime& rhs) {
    return lhs.week == rhs.week && std::abs(lhs.tow - rhs.tow) < 1e-6;
}

PreparedRow prepareRow(const ObservationData& epoch,
                       const Observation& observation,
                       const libgnss::NavigationData& nav,
                       const Vector3d& receiver_position) {
    PreparedRow row;
    if (!observation.valid || !observation.has_doppler ||
        observation.doppler == 0.0) {
        return row;
    }
    row.spp_candidate = true;
    double travel_time_s = 0.075;
    if (observation.has_pseudorange && observation.pseudorange > 0.0) {
        travel_time_s = observation.pseudorange /
                        libgnss::constants::SPEED_OF_LIGHT;
    }
    GNSSTime transmit_time = epoch.time - travel_time_s;
    double satellite_clock_bias = 0.0;
    double satellite_clock_drift = 0.0;
    if (!nav.calculateSatelliteState(
            observation.satellite, transmit_time, row.satellite_position,
            row.satellite_velocity, satellite_clock_bias,
            satellite_clock_drift)) {
        row.spp_candidate = false;
        return row;
    }
    transmit_time = transmit_time - satellite_clock_bias;
    if (!nav.calculateSatelliteState(
            observation.satellite, transmit_time, row.satellite_position,
            row.satellite_velocity, satellite_clock_bias,
            satellite_clock_drift)) {
        row.spp_candidate = false;
        return row;
    }
    row.satellite_clock_bias_s = satellite_clock_bias;
    row.satellite_clock_drift_sps = satellite_clock_drift;
    const auto* eph = nav.getEphemeris(observation.satellite, transmit_time);
    row.frequency_hz = libgnss::signalFrequencyHz(observation.signal, eph);
    if (!(row.frequency_hz > 0.0) || !std::isfinite(row.frequency_hz)) {
        row.spp_candidate = false;
        return row;
    }
    row.wavelength_m = libgnss::signalWavelengthMeters(observation);
    if (!(row.wavelength_m > 0.0) || !std::isfinite(row.wavelength_m)) {
        row.wavelength_m = libgnss::signalWavelengthMeters(
            observation.signal, eph);
    }
    if (!(row.wavelength_m > 0.0) || !std::isfinite(row.wavelength_m)) {
        row.spp_candidate = false;
        return row;
    }
    const double signal_travel_s =
        (row.satellite_position - receiver_position).norm() /
        libgnss::constants::SPEED_OF_LIGHT;
    row.earth_rotation_angle_rad =
        libgnss::constants::OMEGA_E * signal_travel_s;
    if (!libgnss::doppler_contract::earthRotationCorrectedSatelliteState(
            row.satellite_position, row.satellite_velocity, receiver_position,
            row.corrected_satellite_position,
            row.corrected_satellite_velocity)) {
        row.spp_candidate = false;
        return row;
    }
    if (!libgnss::doppler_contract::receiverToSatelliteGeometry(
            row.satellite_position, receiver_position,
            row.los_unrotated_receiver_to_satellite)) {
        row.spp_candidate = false;
        return row;
    }
    if (!libgnss::doppler_contract::receiverToSatelliteGeometry(
            row.corrected_satellite_position, receiver_position,
            row.los_receiver_to_satellite)) {
        row.spp_candidate = false;
        return row;
    }
    row.measured_range_rate_mps =
        libgnss::doppler_contract::rinexDopplerToRangeRate(
            observation.doppler, row.frequency_hz);
    row.satellite_range_rate_mps =
        row.corrected_satellite_velocity.dot(row.los_receiver_to_satellite);
    row.unrotated_satellite_range_rate_mps =
        row.satellite_velocity.dot(row.los_unrotated_receiver_to_satellite);
    row.earth_rotation_sagnac_range_rate_mps =
        libgnss::constants::OMEGA_E / libgnss::constants::SPEED_OF_LIGHT *
        (row.satellite_velocity(1) * receiver_position(0) -
         row.satellite_velocity(0) * receiver_position(1));
    row.earth_rotation_range_rate_delta_mps =
        row.satellite_range_rate_mps - row.unrotated_satellite_range_rate_mps;
    row.expected_receiver_residual_mps =
        libgnss::doppler_contract::receiverOnlyResidual(
            row.measured_range_rate_mps, row.satellite_range_rate_mps,
            row.satellite_clock_drift_sps);
    row.state_valid =
        finiteVector(row.corrected_satellite_position) &&
        finiteVector(row.corrected_satellite_velocity) &&
        finiteVector(row.los_unrotated_receiver_to_satellite) &&
        finiteVector(row.los_receiver_to_satellite) &&
        std::isfinite(row.measured_range_rate_mps) &&
        std::isfinite(row.unrotated_satellite_range_rate_mps) &&
        std::isfinite(row.earth_rotation_sagnac_range_rate_mps) &&
        std::isfinite(row.earth_rotation_range_rate_delta_mps) &&
        std::isfinite(row.satellite_range_rate_mps) &&
        std::isfinite(row.expected_receiver_residual_mps);
    return row;
}

std::string csvNumber(double value) {
    if (!std::isfinite(value)) return "";
    std::ostringstream output;
    output << std::setprecision(std::numeric_limits<double>::max_digits10)
           << value;
    return output.str();
}

struct Metrics {
    std::size_t rows_reported = 0;
    std::size_t rows_with_raw_rate = 0;
    std::size_t spp_candidates = 0;
    std::size_t fgo_rows = 0;
    std::size_t selection_differences = 0;
    double max_raw_adapter_identity_abs_mps = 0.0;
    double max_adapter_measured_identity_abs_mps = 0.0;
    double max_fgo_measured_identity_abs_mps = 0.0;
    double max_fgo_modeled_identity_abs_mps = 0.0;
    double max_fgo_residual_identity_abs_mps = 0.0;
    double max_fgo_los_identity = 0.0;
    double max_earth_rotation_sagnac_mps = 0.0;
    double max_earth_rotation_range_rate_delta_mps = 0.0;
    double max_satellite_position_identity_m = 0.0;
    double max_satellite_velocity_identity_mps = 0.0;
    double max_fgo_wavelength_identity_m = 0.0;
    double max_satellite_range_rate_identity_mps = 0.0;
    double max_satellite_clock_drift_identity_mps = 0.0;
};

void updateMax(double& target, double value) {
    if (std::isfinite(value)) target = std::max(target, std::abs(value));
}

std::string makeSummary(
    const Options& options,
    const libgnss::io::AndroidRawGnssResult& converted,
    const FGOProcessor::FGOProblem& problem,
    std::size_t first_epoch_index,
    std::size_t first_raw_epoch_index,
    std::size_t raw_rows,
    std::size_t fgo_rows,
    const Metrics& metrics,
    const libgnss::spp_velocity::DopplerVelocityResult& spp_result,
    const libgnss::doppler_velocity_wls::Estimate& fgo_estimate,
    const std::string& spp_state_json,
    const std::string& fgo_state_json) {
    std::ostringstream output;
    output << std::setprecision(std::numeric_limits<double>::max_digits10);
    const auto& diagnostics = converted.diagnostics;
    const auto& epoch = problem.epochs[first_epoch_index];
    output << "{\n"
           << "  \"schema_version\": \"smartphone-r5-phase41-doppler-contract-audit.v1\",\n"
           << "  \"phase\": 41,\n"
           << "  \"status\": \"diagnostic\",\n"
           << "  \"dataset_id\": \"" << jsonEscape(options.dataset_id)
           << "\",\n"
           << "  \"truth_used\": false,\n"
           << "  \"precomputed_coordinates_used\": false,\n"
           << "  \"device_wls_coordinates_used\": false,\n"
           << "  \"raw_input\": {\n"
           << "    \"selected_epochs\": " << converted.observations.epochs.size()
           << ",\n"
           << "    \"selected_rows\": " << diagnostics.selected_rows << ",\n"
           << "    \"doppler_rows\": " << diagnostics.doppler_rows << ",\n"
           << "    \"raw_rate_field\": \"Observation::pseudorange_rate_mps\",\n"
           << "    \"raw_rate_unit\": \"m/s\",\n"
           << "    \"adapter_field\": \"Observation::doppler\",\n"
           << "    \"adapter_unit\": \"Hz\",\n"
           << "    \"adapter_formula\": \"D=-PseudorangeRateMetersPerSecond/wavelength_m\"\n"
           << "  },\n"
           << "  \"problem\": {\n"
           << "    \"epochs\": " << problem.epochs.size() << ",\n"
           << "    \"undifferenced_doppler_factors\": "
           << problem.undifferenced_doppler_factors.size() << ",\n"
           << "    \"first_solvable_epoch_index\": " << first_epoch_index << ",\n"
           << "    \"first_solvable_raw_epoch_index\": " << first_raw_epoch_index << ",\n"
           << "    \"first_solvable_gps_week\": " << epoch.time.week << ",\n"
           << "    \"first_solvable_gps_tow\": " << epoch.time.tow << ",\n"
           << "    \"first_solvable_utc_time_millis\": "
           << unixMillis(converted.observations.epochs[first_raw_epoch_index].time)
           << ",\n"
           << "    \"raw_rows_at_first_solvable\": " << raw_rows << ",\n"
           << "    \"fgo_rows_at_first_solvable\": " << fgo_rows << "\n"
           << "  },\n"
           << "  \"row_selection\": {\n"
           << "    \"spp_candidate_rows\": " << metrics.spp_candidates << ",\n"
           << "    \"fgo_factor_rows\": " << metrics.fgo_rows << ",\n"
           << "    \"selection_difference_rows\": "
           << metrics.selection_differences << ",\n"
           << "    \"selection_difference_is_contract_bug\": false\n"
           << "  },\n"
           << "  \"identity_differences\": {\n"
           << "    \"max_abs_raw_rate_minus_neg_doppler_lambda_mps\": "
           << metrics.max_raw_adapter_identity_abs_mps << ",\n"
           << "    \"max_abs_adapter_measured_range_rate_mps\": "
           << metrics.max_adapter_measured_identity_abs_mps << ",\n"
           << "    \"max_abs_fgo_measured_minus_neg_doppler_lambda_mps\": "
           << metrics.max_fgo_measured_identity_abs_mps << ",\n"
           << "    \"max_abs_fgo_modeled_satellite_range_rate_mps\": "
           << metrics.max_fgo_modeled_identity_abs_mps << ",\n"
           << "    \"max_abs_fgo_residual_minus_independent_receiver_residual_mps\": "
           << metrics.max_fgo_residual_identity_abs_mps << ",\n"
           << "    \"max_fgo_los_minus_negative_receiver_to_satellite_los\": "
           << metrics.max_fgo_los_identity << ",\n"
           << "    \"max_abs_earth_rotation_sagnac_range_rate_mps\": "
           << metrics.max_earth_rotation_sagnac_mps << ",\n"
           << "    \"max_abs_corrected_minus_unrotated_satellite_range_rate_mps\": "
           << metrics.max_earth_rotation_range_rate_delta_mps << ",\n"
           << "    \"max_satellite_position_difference_m\": "
           << metrics.max_satellite_position_identity_m << ",\n"
           << "    \"max_satellite_velocity_difference_mps\": "
           << metrics.max_satellite_velocity_identity_mps << ",\n"
           << "    \"max_fgo_wavelength_difference_m\": "
           << metrics.max_fgo_wavelength_identity_m << ",\n"
           << "    \"max_satellite_range_rate_difference_mps\": "
           << metrics.max_satellite_range_rate_identity_mps << ",\n"
           << "    \"max_satellite_clock_drift_difference_mps\": "
           << metrics.max_satellite_clock_drift_identity_mps << "\n"
           << "  },\n"
           << "  \"solver_comparison\": {\n"
           << "    \"spp_solver\": \"libgnss::spp_velocity::solveVelocityFromObservations\",\n"
           << "    \"fgo_solver\": \"FGOProblem::doppler_velocity_wls_estimates\",\n"
           << "    \"same_epoch\": true,\n"
           << "    \"same_raw_seed_position\": true,\n"
           << "    \"spp_ok\": " << (spp_result.ok ? "true" : "false") << ",\n"
           << "    \"fgo_valid\": " << (fgo_estimate.valid ? "true" : "false") << ",\n"
           << "    \"spp_state\": " << spp_state_json << ",\n"
           << "    \"fgo_state\": " << fgo_state_json << ",\n"
           << "    \"comparison_is_diagnostic_only\": true\n"
           << "  },\n"
           << "  \"contract_verdict\": {\n"
           << "    \"concrete_bug_proven\": false,\n"
           << "    \"correction_applied\": false,\n"
           << "    \"classification\": \"diagnosis-no-go-until-numerical-or-code-mismatch\"\n"
           << "  },\n"
           << "  \"forbidden_input_accounting\": {\n"
           << "    \"truth_open_count\": 0,\n"
           << "    \"validation_open_count\": 0,\n"
           << "    \"holdout_open_count\": 0,\n"
           << "    \"mat_read_or_generated\": false,\n"
           << "    \"kaggle_or_token_access\": false\n"
           << "  }\n"
           << "}\n";
    return output.str();
}

std::string stateJson(const Vector3d& velocity,
                      double clock_rate_mps,
                      bool ok,
                      double residual_rms_mps,
                      int rows) {
    std::ostringstream output;
    output << std::setprecision(std::numeric_limits<double>::max_digits10)
           << "{\"ok\":" << (ok ? "true" : "false")
           << ",\"rows\":" << rows;
    if (velocity.allFinite()) {
        output << ",\"velocity_ecef_mps\":[" << velocity.x() << ','
               << velocity.y() << ',' << velocity.z() << ']'
               << ",\"velocity_norm_mps\":" << velocity.norm();
    } else {
        output << ",\"velocity_ecef_mps\":null,\"velocity_norm_mps\":null";
    }
    if (std::isfinite(clock_rate_mps)) {
        output << ",\"clock_rate_mps\":" << clock_rate_mps;
    } else {
        output << ",\"clock_rate_mps\":null";
    }
    if (std::isfinite(residual_rms_mps)) {
        output << ",\"residual_rms_mps\":" << residual_rms_mps;
    } else {
        output << ",\"residual_rms_mps\":null";
    }
    output << '}';
    return output.str();
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!parseOptions(argc, argv, options)) return 2;
    if (forbiddenPath(options.android_gnss) || forbiddenPath(options.nav) ||
        forbiddenPath(options.rows_csv) || forbiddenPath(options.summary_json)) {
        std::cerr << "Phase41 raw-only path contract rejected an input/output path\n";
        return 2;
    }
    if (!regularFile(options.android_gnss) || !regularFile(options.nav)) {
        std::cerr << "Phase41 audit requires regular raw GNSS and broadcast nav files\n";
        return 2;
    }

    libgnss::io::AndroidRawGnssConfig android_config;
    // Ignore the optional enriched pseudorange column.  The loader still
    // validates raw clock fields; no device WLS coordinate is used below.
    android_config.verify_enriched_pseudorange = false;
    libgnss::io::AndroidRawGnssResult converted;
    std::string conversion_error;
    if (!libgnss::io::loadAndroidRawGnssCsv(
            options.android_gnss, android_config, converted, conversion_error)) {
        std::cerr << "raw Android GNSS conversion failed: " << conversion_error
                  << '\n';
        return 1;
    }
    if (converted.observations.epochs.empty()) {
        std::cerr << "raw Android GNSS has no selected epochs\n";
        return 1;
    }

    libgnss::io::RINEXReader nav_reader;
    if (!nav_reader.open(options.nav)) {
        std::cerr << "failed to open broadcast navigation\n";
        return 1;
    }
    libgnss::NavigationData nav;
    if (!nav_reader.readNavigationData(nav)) {
        std::cerr << "failed to decode broadcast navigation\n";
        return 1;
    }

    const Vector3d raw_seed(kInitialRawSeedX, kInitialRawSeedY,
                            kInitialRawSeedZ);
    std::vector<ObservationData> epochs = converted.observations.epochs;
    for (auto& epoch : epochs) {
        // The Android device WLS field is explicitly ignored.  This fixed
        // Earth-surface point is only SPP's numerical startup seed; each
        // problem epoch then owns the raw SPP seed produced by the builder.
        epoch.receiver_position = raw_seed;
    }

    FGOProcessor::FGOConfig config;
    config.backend = libgnss::FGOBackend::Eigen;
    config.max_iterations = 12;
    config.use_pose3_state = true;
    config.use_imu = false;
    config.use_double_difference_factors = false;
    config.use_pseudorange_factors = true;
    config.use_carrier_phase_factors = false;
    config.use_tdcp_factors = false;
    config.use_single_difference_doppler_factors = false;
    config.use_single_difference_tdcp_factors = false;
    config.use_undifferenced_doppler_factors = true;
    config.use_corrected_undifferenced_doppler_factors = true;
    config.use_doppler_velocity_wls_initialization = true;
    config.doppler_velocity_wls_edge_hold_max_s = 1.0;
    config.use_quality_anchor_initialization = true;
    config.min_elevation_deg = 0.0;
    config.min_snr_dbhz = 0.0;
    const FGOProcessor processor(config);
    const FGOProcessor::FGOProblem problem =
        processor.buildPseudorangeProblem(epochs, nav);
    if (problem.epochs.empty() ||
        problem.doppler_velocity_wls_estimates.size() != problem.epochs.size()) {
        std::cerr << "FGO problem did not preserve epoch/WLS alignment\n";
        return 1;
    }

    std::vector<std::vector<const FGOProcessor::UndifferencedDopplerFactor*>>
        fgo_by_epoch(problem.epochs.size());
    for (const auto& factor : problem.undifferenced_doppler_factors) {
        if (factor.epoch_index < fgo_by_epoch.size()) {
            fgo_by_epoch[factor.epoch_index].push_back(&factor);
        }
    }
    std::size_t first_epoch_index = problem.epochs.size();
    for (std::size_t index = 0; index < fgo_by_epoch.size(); ++index) {
        if (fgo_by_epoch[index].size() >= 4U) {
            first_epoch_index = index;
            break;
        }
    }
    if (first_epoch_index == problem.epochs.size()) {
        std::cerr << "no first solvable epoch with four FGO Doppler rows\n";
        return 1;
    }
    const auto& first_epoch = problem.epochs[first_epoch_index];
    std::size_t first_raw_epoch_index = epochs.size();
    for (std::size_t index = 0; index < epochs.size(); ++index) {
        if (sameEpoch(epochs[index].time, first_epoch.time)) {
            first_raw_epoch_index = index;
            break;
        }
    }
    if (first_raw_epoch_index == epochs.size()) {
        std::cerr << "cannot map first FGO epoch to raw epoch\n";
        return 1;
    }

    std::map<RowKey, const FGOProcessor::UndifferencedDopplerFactor*>
        factors_by_key;
    for (const auto* factor : fgo_by_epoch[first_epoch_index]) {
        factors_by_key[{factor->satellite, factor->signal}] = factor;
    }

    Metrics metrics;
    std::vector<const Observation*> raw_doppler_rows;
    for (const auto& observation : epochs[first_raw_epoch_index].observations) {
        if (observation.valid && observation.has_doppler &&
            observation.doppler != 0.0) {
            raw_doppler_rows.push_back(&observation);
        }
    }
    if (raw_doppler_rows.empty()) {
        std::cerr << "first solvable epoch has no raw Doppler observations\n";
        return 1;
    }
    const auto& seed = first_epoch.position_ecef;
    const auto spp_result = libgnss::spp_velocity::solveVelocityFromObservations(
        epochs[first_raw_epoch_index], nav, seed, 0.5, 4);
    const auto& fgo_estimate =
        problem.doppler_velocity_wls_estimates[first_epoch_index];

    std::ostringstream rows;
    rows << std::setprecision(std::numeric_limits<double>::max_digits10);
    rows << "dataset_id,epoch_index,raw_epoch_index,utc_time_millis,gps_week,gps_tow,"
            "satellite,system,prn,signal,raw_rate_mps,has_raw_rate,adapter_doppler_hz,"
            "frequency_hz,wavelength_m,measured_minus_neg_doppler_lambda_mps,"
            "sat_pos_x_m,sat_pos_y_m,sat_pos_z_m,sat_vel_x_mps,sat_vel_y_mps,sat_vel_z_mps,"
            "sat_clock_drift_sps,sat_clock_drift_mps,earth_rotation_angle_rad,"
            "los_unrotated_receiver_to_sat_x,los_unrotated_receiver_to_sat_y,"
            "los_unrotated_receiver_to_sat_z,unrotated_satellite_range_rate_mps,"
            "earth_rotation_sagnac_range_rate_mps,earth_rotation_range_rate_delta_mps,"
            "corrected_sat_pos_x_m,corrected_sat_pos_y_m,corrected_sat_pos_z_m,"
            "corrected_sat_vel_x_mps,corrected_sat_vel_y_mps,corrected_sat_vel_z_mps,"
            "los_receiver_to_sat_x,los_receiver_to_sat_y,los_receiver_to_sat_z,"
            "known_satellite_range_rate_mps,independent_receiver_residual_mps,"
            "spp_candidate,fgo_factor_present,fgo_los_x,fgo_los_y,fgo_los_z,"
            "fgo_sat_pos_x_m,fgo_sat_pos_y_m,fgo_sat_pos_z_m,"
            "fgo_sat_vel_x_mps,fgo_sat_vel_y_mps,fgo_sat_vel_z_mps,fgo_wavelength_m,"
            "fgo_residual_mps,fgo_measured_range_rate_mps,fgo_satellite_range_rate_mps,"
            "fgo_satellite_clock_drift_mps,fgo_uses_rotated_state,"
            "raw_adapter_identity_abs_mps,fgo_measured_identity_abs_mps,"
            "fgo_residual_identity_abs_mps,fgo_los_identity\n";

    for (const Observation* observation : raw_doppler_rows) {
        ++metrics.rows_reported;
        const PreparedRow prepared =
            prepareRow(epochs[first_raw_epoch_index], *observation, nav, seed);
        const bool has_raw_rate = observation->has_pseudorange_rate_mps &&
                                  std::isfinite(observation->pseudorange_rate_mps);
        if (has_raw_rate) ++metrics.rows_with_raw_rate;
        if (prepared.spp_candidate) ++metrics.spp_candidates;
        updateMax(metrics.max_earth_rotation_sagnac_mps,
                  prepared.earth_rotation_sagnac_range_rate_mps);
        updateMax(metrics.max_earth_rotation_range_rate_delta_mps,
                  prepared.earth_rotation_range_rate_delta_mps);
        const auto factor_it = factors_by_key.find(
            {observation->satellite, observation->signal});
        const auto* factor = factor_it == factors_by_key.end()
                                 ? nullptr
                                 : factor_it->second;
        if (factor != nullptr) {
            ++metrics.fgo_rows;
        }
        if (prepared.spp_candidate != (factor != nullptr)) {
            ++metrics.selection_differences;
        }
        double adapter_identity = std::numeric_limits<double>::quiet_NaN();
        if (has_raw_rate && std::isfinite(prepared.wavelength_m)) {
            adapter_identity = observation->pseudorange_rate_mps -
                               (-observation->doppler * prepared.wavelength_m);
            updateMax(metrics.max_raw_adapter_identity_abs_mps, adapter_identity);
        }
        double adapter_measured_identity = std::numeric_limits<double>::quiet_NaN();
        if (std::isfinite(prepared.measured_range_rate_mps) &&
            std::isfinite(prepared.wavelength_m)) {
            adapter_measured_identity = prepared.measured_range_rate_mps -
                                        (-observation->doppler * prepared.wavelength_m);
            updateMax(metrics.max_adapter_measured_identity_abs_mps,
                      adapter_measured_identity);
        }
        double fgo_measured_identity = std::numeric_limits<double>::quiet_NaN();
        double fgo_residual_identity = std::numeric_limits<double>::quiet_NaN();
        double fgo_los_identity = std::numeric_limits<double>::quiet_NaN();
        double fgo_satellite_position_identity =
            std::numeric_limits<double>::quiet_NaN();
        double fgo_satellite_velocity_identity =
            std::numeric_limits<double>::quiet_NaN();
        double fgo_wavelength_identity = std::numeric_limits<double>::quiet_NaN();
        if (factor != nullptr) {
            fgo_measured_identity = factor->measured_range_rate_mps -
                                    (-observation->doppler * prepared.wavelength_m);
            fgo_residual_identity = factor->residual_mps -
                                    prepared.expected_receiver_residual_mps;
            const Vector3d expected_factor_los =
                -prepared.los_receiver_to_satellite;
            fgo_los_identity = (factor->los - expected_factor_los).norm();
            fgo_satellite_position_identity =
                (factor->satellite_position_ecef -
                 prepared.corrected_satellite_position)
                    .norm();
            fgo_satellite_velocity_identity =
                (factor->satellite_velocity_ecef -
                 prepared.corrected_satellite_velocity)
                    .norm();
            fgo_wavelength_identity = factor->wavelength_m - prepared.wavelength_m;
            updateMax(metrics.max_fgo_measured_identity_abs_mps,
                      fgo_measured_identity);
            updateMax(metrics.max_fgo_residual_identity_abs_mps,
                      fgo_residual_identity);
            updateMax(metrics.max_fgo_los_identity, fgo_los_identity);
            updateMax(metrics.max_satellite_position_identity_m,
                      fgo_satellite_position_identity);
            updateMax(metrics.max_satellite_velocity_identity_mps,
                      fgo_satellite_velocity_identity);
            updateMax(metrics.max_fgo_wavelength_identity_m,
                      fgo_wavelength_identity);
            updateMax(metrics.max_fgo_modeled_identity_abs_mps,
                      factor->satellite_range_rate_mps -
                          prepared.satellite_range_rate_mps);
            updateMax(metrics.max_satellite_range_rate_identity_mps,
                      factor->satellite_range_rate_mps -
                          prepared.satellite_range_rate_mps);
            updateMax(metrics.max_satellite_clock_drift_identity_mps,
                      factor->satellite_clock_drift_mps -
                          prepared.satellite_clock_drift_sps *
                              libgnss::constants::SPEED_OF_LIGHT);
        }
        rows << options.dataset_id << ',' << first_epoch_index << ','
             << first_raw_epoch_index << ','
             << unixMillis(epochs[first_raw_epoch_index].time)
             << ',' << first_epoch.time.week << ',' << first_epoch.time.tow << ','
             << observation->satellite.toString() << ','
             << systemName(observation->satellite.system) << ','
             << static_cast<int>(observation->satellite.prn) << ','
             << signalName(observation->signal) << ','
             << csvNumber(observation->pseudorange_rate_mps) << ','
             << (has_raw_rate ? 1 : 0) << ',' << csvNumber(observation->doppler)
             << ',' << csvNumber(prepared.frequency_hz) << ','
             << csvNumber(prepared.wavelength_m) << ','
             << csvNumber(adapter_identity) << ','
             << csvNumber(prepared.satellite_position.x()) << ','
             << csvNumber(prepared.satellite_position.y()) << ','
             << csvNumber(prepared.satellite_position.z()) << ','
             << csvNumber(prepared.satellite_velocity.x()) << ','
             << csvNumber(prepared.satellite_velocity.y()) << ','
             << csvNumber(prepared.satellite_velocity.z()) << ','
             << csvNumber(prepared.satellite_clock_drift_sps) << ','
             << csvNumber(prepared.satellite_clock_drift_sps *
                          libgnss::constants::SPEED_OF_LIGHT) << ','
             << csvNumber(prepared.earth_rotation_angle_rad) << ','
             << csvNumber(prepared.los_unrotated_receiver_to_satellite.x()) << ','
             << csvNumber(prepared.los_unrotated_receiver_to_satellite.y()) << ','
             << csvNumber(prepared.los_unrotated_receiver_to_satellite.z()) << ','
             << csvNumber(prepared.unrotated_satellite_range_rate_mps) << ','
             << csvNumber(prepared.earth_rotation_sagnac_range_rate_mps) << ','
             << csvNumber(prepared.earth_rotation_range_rate_delta_mps) << ','
             << csvNumber(prepared.corrected_satellite_position.x()) << ','
             << csvNumber(prepared.corrected_satellite_position.y()) << ','
             << csvNumber(prepared.corrected_satellite_position.z()) << ','
             << csvNumber(prepared.corrected_satellite_velocity.x()) << ','
             << csvNumber(prepared.corrected_satellite_velocity.y()) << ','
             << csvNumber(prepared.corrected_satellite_velocity.z()) << ','
             << csvNumber(prepared.los_receiver_to_satellite.x()) << ','
             << csvNumber(prepared.los_receiver_to_satellite.y()) << ','
             << csvNumber(prepared.los_receiver_to_satellite.z()) << ','
             << csvNumber(prepared.satellite_range_rate_mps) << ','
             << csvNumber(prepared.expected_receiver_residual_mps) << ','
             << (prepared.spp_candidate ? 1 : 0) << ','
             << (factor != nullptr ? 1 : 0) << ','
             << csvNumber(factor == nullptr
                               ? std::numeric_limits<double>::quiet_NaN()
                               : factor->los.x())
             << ','
             << csvNumber(factor == nullptr
                               ? std::numeric_limits<double>::quiet_NaN()
                               : factor->los.y())
             << ','
             << csvNumber(factor == nullptr
                               ? std::numeric_limits<double>::quiet_NaN()
                               : factor->los.z())
             << ','
             << csvNumber(factor == nullptr
                               ? std::numeric_limits<double>::quiet_NaN()
                               : factor->satellite_position_ecef.x())
             << ','
             << csvNumber(factor == nullptr
                               ? std::numeric_limits<double>::quiet_NaN()
                               : factor->satellite_position_ecef.y())
             << ','
             << csvNumber(factor == nullptr
                               ? std::numeric_limits<double>::quiet_NaN()
                               : factor->satellite_position_ecef.z())
             << ','
             << csvNumber(factor == nullptr
                               ? std::numeric_limits<double>::quiet_NaN()
                               : factor->satellite_velocity_ecef.x())
             << ','
             << csvNumber(factor == nullptr
                               ? std::numeric_limits<double>::quiet_NaN()
                               : factor->satellite_velocity_ecef.y())
             << ','
             << csvNumber(factor == nullptr
                               ? std::numeric_limits<double>::quiet_NaN()
                               : factor->satellite_velocity_ecef.z())
             << ','
             << csvNumber(factor == nullptr
                               ? std::numeric_limits<double>::quiet_NaN()
                               : factor->wavelength_m)
             << ','
             << csvNumber(factor == nullptr
                               ? std::numeric_limits<double>::quiet_NaN()
                               : factor->residual_mps)
             << ','
             << csvNumber(factor == nullptr
                               ? std::numeric_limits<double>::quiet_NaN()
                               : factor->measured_range_rate_mps)
             << ','
             << csvNumber(factor == nullptr
                               ? std::numeric_limits<double>::quiet_NaN()
                               : factor->satellite_range_rate_mps)
             << ','
             << csvNumber(factor == nullptr
                               ? std::numeric_limits<double>::quiet_NaN()
                               : factor->satellite_clock_drift_mps)
             << ',' << (factor != nullptr && factor->uses_rotated_satellite_state ? 1 : 0)
             << ',' << csvNumber(adapter_identity) << ','
             << csvNumber(fgo_measured_identity) << ','
             << csvNumber(fgo_residual_identity) << ','
             << csvNumber(fgo_los_identity) << '\n';
    }
    if (!atomicWrite(options.rows_csv, rows.str())) {
        std::cerr << "failed to publish audit row CSV\n";
        return 1;
    }

    const std::string spp_json = stateJson(
        spp_result.velocity_ecef, spp_result.receiver_clock_drift *
                                      libgnss::constants::SPEED_OF_LIGHT,
        spp_result.ok, spp_result.residual_rms_mps,
        spp_result.num_satellites_used);
    const std::string fgo_json = stateJson(
        fgo_estimate.velocity_ecef_mps, fgo_estimate.clock_rate_mps,
        fgo_estimate.valid, fgo_estimate.normalized_rms,
        fgo_estimate.rows);
    const std::string summary = makeSummary(
        options, converted, problem, first_epoch_index, first_raw_epoch_index,
        metrics.rows_reported, metrics.fgo_rows, metrics, spp_result,
        fgo_estimate, spp_json, fgo_json);
    if (!atomicWrite(options.summary_json, summary)) {
        std::cerr << "failed to publish audit summary\n";
        return 1;
    }
    std::cout << "Phase41 Doppler contract audit: dataset=" << options.dataset_id
              << " first_epoch=" << first_epoch_index
              << " raw_rows=" << metrics.rows_reported
              << " fgo_rows=" << metrics.fgo_rows
              << " spp_candidates=" << metrics.spp_candidates
              << " selection_differences=" << metrics.selection_differences
              << " max_adapter_identity_mps="
              << metrics.max_raw_adapter_identity_abs_mps
              << " max_fgo_residual_identity_mps="
              << metrics.max_fgo_residual_identity_abs_mps << '\n';
    return 0;
}
