// Truth-free raw Android Doppler row-attrition audit.
//
// This executable replays the unchanged Phase41 FGO problem builder and
// classifies every parsed Android Raw row by the first exclusive stage at
// which it cannot reach a corrected undifferenced Doppler factor.  It keeps
// loader-rejected rows (unsupported signals, invalid raw timing/quality, and
// invalid raw pseudorange) in the row report, while selected Observation rows
// are checked against the exact FGO state/frequency/geometry/dt contract.
// No truth, IMU, result, submission, device-WLS coordinate, or precomputed
// coordinate input is accepted or consulted.

#include <libgnss++/algorithms/doppler_contract.hpp>
#include <libgnss++/algorithms/fgo.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/io/android_raw_gnss.hpp>
#include <libgnss++/io/rinex.hpp>

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
using RawDiagnostic = libgnss::io::AndroidRawGnssRowDiagnostic;

constexpr std::size_t kNoIndex = std::numeric_limits<std::size_t>::max();
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

struct RowKey {
    SatelliteId satellite;
    SignalType signal = SignalType::GPS_L1CA;

    bool operator<(const RowKey& other) const {
        if (satellite != other.satellite) return satellite < other.satellite;
        return static_cast<unsigned int>(signal) <
               static_cast<unsigned int>(other.signal);
    }
};

struct RowAudit {
    const RawDiagnostic* raw = nullptr;
    const Observation* observation = nullptr;
    std::size_t raw_epoch_index = kNoIndex;
    std::size_t problem_epoch_index = kNoIndex;
    std::string satellite;
    std::string signal;
    std::string first_reason;
    bool valid = false;
    bool doppler = false;
    bool raw_snr_masked = false;
    bool raw_multipath_masked = false;
    bool supported_signal = false;
    bool frequency_valid = false;
    bool pseudorange_valid = false;
    bool transmit_time_valid = false;
    bool nav_state_call1 = false;
    bool nav_state_call2 = false;
    bool ephemeris_present = false;
    bool ephemeris_healthy = false;
    bool geometry_valid = false;
    bool snr_mask_pass = false;
    bool elevation_mask_pass = false;
    bool clock_jump = false;
    double dt_s = std::numeric_limits<double>::quiet_NaN();
    bool dt_valid = false;
    bool includes_receiver_clock_drift = false;
    bool fgo_factor_present = false;
};

struct Counts {
    std::size_t raw_rows = 0;
    std::size_t selected_rows = 0;
    std::size_t invalid_rows = 0;
    std::size_t doppler_rows = 0;
    std::size_t raw_snr_masked_rows = 0;
    std::size_t raw_multipath_masked_rows = 0;
    std::size_t valid_rows = 0;
    std::size_t supported_signal_rows = 0;
    std::size_t frequency_rows = 0;
    std::size_t raw_pseudorange_rows = 0;
    std::size_t transmit_time_rows = 0;
    std::size_t nav_state_call1_rows = 0;
    std::size_t nav_state_call2_rows = 0;
    std::size_t ephemeris_present_rows = 0;
    std::size_t ephemeris_healthy_rows = 0;
    std::size_t geometry_rows = 0;
    std::size_t snr_mask_pass_rows = 0;
    std::size_t elevation_mask_pass_rows = 0;
    std::size_t problem_epoch_rows = 0;
    std::size_t clock_jump_rows = 0;
    std::size_t dt_valid_rows = 0;
    std::size_t receiver_clock_drift_rows = 0;
    std::size_t fgo_factor_rows = 0;
    std::map<std::string, std::size_t> first_reason;
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

std::string csvNumber(double value) {
    if (!std::isfinite(value)) return "";
    std::ostringstream output;
    output << std::setprecision(std::numeric_limits<double>::max_digits10)
           << value;
    return output.str();
}

long long unixMillis(const GNSSTime& time) {
    constexpr double kGpsEpochUnixSeconds = 315964800.0;
    constexpr double kGpsUtcLeapSeconds = 18.0;
    return static_cast<long long>(std::llround(
        (kGpsEpochUnixSeconds + static_cast<double>(time.week) * 604800.0 +
         time.tow - kGpsUtcLeapSeconds) * 1000.0));
}

bool sameEpoch(const GNSSTime& lhs, const GNSSTime& rhs) {
    return lhs.week == rhs.week && std::abs(lhs.tow - rhs.tow) < 1e-6;
}

bool finiteTime(const GNSSTime& time) {
    return time.week >= 0 && std::isfinite(time.tow) && time.tow >= 0.0 &&
           time.tow < 604800.0;
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

bool supportedFgoSignal(const SatelliteId& satellite, SignalType signal) {
    switch (signal) {
        case SignalType::GPS_L1CA:
            return satellite.system == GNSSSystem::GPS;
        case SignalType::GLO_L1CA:
            return satellite.system == GNSSSystem::GLONASS;
        case SignalType::GAL_E1:
            return satellite.system == GNSSSystem::Galileo;
        case SignalType::BDS_B1I:
        case SignalType::BDS_B1C:
            return satellite.system == GNSSSystem::BeiDou;
        case SignalType::QZS_L1CA:
            return satellite.system == GNSSSystem::QZSS;
        default:
            return false;
    }
}

bool healthyForPositioning(const Observation& observation,
                           const libgnss::Ephemeris& eph) {
    int health = static_cast<int>(eph.health);
    if (observation.satellite.system == GNSSSystem::QZSS) health &= 0xFE;
    return health == 0;
}

std::string rawSatelliteName(const RawDiagnostic& raw) {
    char prefix = '?';
    switch (raw.constellation) {
        case 1: prefix = 'G'; break;
        case 3: prefix = 'R'; break;
        case 4: prefix = 'J'; break;
        case 5: prefix = 'C'; break;
        case 6: prefix = 'E'; break;
        default: break;
    }
    return std::string(1, prefix) + std::to_string(raw.svid);
}

void setReason(RowAudit& row, const char* reason) {
    if (row.first_reason.empty()) row.first_reason = reason;
}

void addCounts(Counts& counts, const RowAudit& row) {
    ++counts.raw_rows;
    if (row.raw != nullptr && row.raw->selected) ++counts.selected_rows;
    if (row.raw != nullptr && row.raw->snr_masked) ++counts.raw_snr_masked_rows;
    if (row.raw != nullptr && row.raw->multipath_masked) {
        ++counts.raw_multipath_masked_rows;
    }
    if (row.valid) ++counts.valid_rows;
    if (row.doppler) ++counts.doppler_rows;
    if (row.supported_signal) ++counts.supported_signal_rows;
    if (row.frequency_valid) ++counts.frequency_rows;
    if (row.pseudorange_valid) ++counts.raw_pseudorange_rows;
    if (row.transmit_time_valid) ++counts.transmit_time_rows;
    if (row.nav_state_call1) ++counts.nav_state_call1_rows;
    if (row.nav_state_call2) ++counts.nav_state_call2_rows;
    if (row.ephemeris_present) ++counts.ephemeris_present_rows;
    if (row.ephemeris_healthy) ++counts.ephemeris_healthy_rows;
    if (row.geometry_valid) ++counts.geometry_rows;
    if (row.snr_mask_pass) ++counts.snr_mask_pass_rows;
    if (row.elevation_mask_pass) ++counts.elevation_mask_pass_rows;
    if (row.problem_epoch_index != kNoIndex) ++counts.problem_epoch_rows;
    if (row.clock_jump) ++counts.clock_jump_rows;
    if (row.dt_valid) ++counts.dt_valid_rows;
    if (row.includes_receiver_clock_drift) ++counts.receiver_clock_drift_rows;
    if (row.fgo_factor_present) ++counts.fgo_factor_rows;
    const std::string reason = row.first_reason.empty()
                                   ? "unclassified"
                                   : row.first_reason;
    ++counts.first_reason[reason];
}

void emitJsonCounts(std::ostringstream& output, const Counts& counts,
                    const std::string& indent) {
    output << indent << "\"raw_rows\": " << counts.raw_rows << ",\n"
           << indent << "\"selected_rows\": " << counts.selected_rows << ",\n"
           << indent << "\"valid_rows\": " << counts.valid_rows << ",\n"
           << indent << "\"doppler_rows\": " << counts.doppler_rows << ",\n"
           << indent << "\"raw_snr_masked_rows\": "
           << counts.raw_snr_masked_rows << ",\n"
           << indent << "\"raw_multipath_masked_rows\": "
           << counts.raw_multipath_masked_rows << ",\n"
           << indent << "\"supported_signal_rows\": "
           << counts.supported_signal_rows << ",\n"
           << indent << "\"frequency_rows\": " << counts.frequency_rows << ",\n"
           << indent << "\"raw_pseudorange_rows\": "
           << counts.raw_pseudorange_rows << ",\n"
           << indent << "\"transmit_time_rows\": "
           << counts.transmit_time_rows << ",\n"
           << indent << "\"nav_state_call1_rows\": "
           << counts.nav_state_call1_rows << ",\n"
           << indent << "\"nav_state_call2_rows\": "
           << counts.nav_state_call2_rows << ",\n"
           << indent << "\"ephemeris_present_rows\": "
           << counts.ephemeris_present_rows << ",\n"
           << indent << "\"ephemeris_healthy_rows\": "
           << counts.ephemeris_healthy_rows << ",\n"
           << indent << "\"geometry_rows\": " << counts.geometry_rows << ",\n"
           << indent << "\"snr_mask_pass_rows\": "
           << counts.snr_mask_pass_rows << ",\n"
           << indent << "\"elevation_mask_pass_rows\": "
           << counts.elevation_mask_pass_rows << ",\n"
           << indent << "\"problem_epoch_rows\": "
           << counts.problem_epoch_rows << ",\n"
           << indent << "\"clock_jump_rows\": "
           << counts.clock_jump_rows << ",\n"
           << indent << "\"dt_valid_rows\": " << counts.dt_valid_rows << ",\n"
           << indent << "\"receiver_clock_drift_rows\": "
           << counts.receiver_clock_drift_rows << ",\n"
           << indent << "\"fgo_factor_rows\": " << counts.fgo_factor_rows
           << ",\n"
           << indent << "\"first_reason_counts\": {\n";
    std::size_t emitted = 0;
    for (const auto& [reason, count] : counts.first_reason) {
        output << indent << "  \"" << jsonEscape(reason) << "\": " << count;
        if (++emitted != counts.first_reason.size()) output << ',';
        output << '\n';
    }
    output << indent << "}\n";
}

std::string makeSummary(
    const Options& options,
    const libgnss::io::AndroidRawGnssResult& converted,
    const FGOProcessor::FGOProblem& problem,
    std::size_t first_problem_epoch,
    std::size_t first_raw_epoch,
    const Counts& aggregate,
    const Counts& first_epoch,
    const GNSSTime& first_time,
    std::int64_t first_utc_millis,
    std::size_t first_selected_rows,
    std::size_t first_factor_rows,
    std::size_t clock_jump_epochs,
    std::size_t mapping_mismatches) {
    std::ostringstream output;
    output << std::setprecision(std::numeric_limits<double>::max_digits10);
    output << "{\n"
           << "  \"schema_version\": \"smartphone-r5-phase42-doppler-row-attrition-audit.v1\",\n"
           << "  \"phase\": 42,\n"
           << "  \"status\": \"diagnostic\",\n"
           << "  \"dataset_id\": \"" << jsonEscape(options.dataset_id)
           << "\",\n"
           << "  \"truth_used\": false,\n"
           << "  \"precomputed_coordinates_used\": false,\n"
           << "  \"device_wls_coordinates_used\": false,\n"
           << "  \"raw_input\": {\n"
           << "    \"selected_epochs\": " << converted.observations.epochs.size()
           << ",\n"
           << "    \"parsed_raw_rows\": " << converted.raw_row_diagnostics.size()
           << ",\n"
           << "    \"loader_raw_rows\": " << converted.diagnostics.raw_rows << ",\n"
           << "    \"loader_selected_rows\": " << converted.diagnostics.selected_rows
           << ",\n"
           << "    \"loader_doppler_rows\": " << converted.diagnostics.doppler_rows
           << ",\n"
           << "    \"loader_skipped_unsupported_signal_rows\": "
           << converted.diagnostics.skipped_unsupported_signal_rows << ",\n"
           << "    \"loader_skipped_invalid_timing_rows\": "
           << converted.diagnostics.skipped_invalid_timing_rows << ",\n"
           << "    \"loader_skipped_invalid_quality_rows\": "
           << converted.diagnostics.skipped_invalid_quality_rows << ",\n"
           << "    \"loader_deduplicated_rows\": "
           << converted.diagnostics.deduplicated_rows << ",\n"
           << "    \"raw_field\": \"Observation::pseudorange_rate_mps [m/s]\",\n"
           << "    \"adapter_field\": \"Observation::doppler [Hz]\",\n"
           << "    \"raw_loader_open_count\": 1\n"
           << "  },\n"
           << "  \"navigation\": {\n"
           << "    \"broadcast_nav_open_count\": 1,\n"
           << "    \"state_calls_per_selected_row\": \"call1 then clock-corrected call2\",\n"
           << "    \"health_rule\": \"Ephemeris health == 0 (QZSS bit 0 ignored)\"\n"
           << "  },\n"
           << "  \"first_solvable_epoch\": {\n"
           << "    \"problem_epoch_index\": " << first_problem_epoch << ",\n"
           << "    \"raw_epoch_index\": " << first_raw_epoch << ",\n"
           << "    \"utc_time_millis\": " << first_utc_millis << ",\n"
           << "    \"gps_week\": " << first_time.week << ",\n"
           << "    \"gps_tow\": " << first_time.tow << ",\n"
           << "    \"utc_gps_mapping_mismatch_rows\": " << mapping_mismatches << ",\n"
           << "    \"raw_rows\": " << first_epoch.raw_rows << ",\n"
           << "    \"selected_rows\": " << first_selected_rows << ",\n"
           << "    \"selected_to_factor_difference_rows\": "
           << (first_selected_rows >= first_factor_rows
                   ? first_selected_rows - first_factor_rows
                   : 0)
           << ",\n"
           << "    \"raw_to_factor_difference_rows\": "
           << (first_epoch.raw_rows >= first_factor_rows
                   ? first_epoch.raw_rows - first_factor_rows
                   : 0)
           << ",\n"
           << "    \"factor_rows\": " << first_factor_rows << ",\n"
           << "    \"clock_jump_epochs_in_problem\": " << clock_jump_epochs << ",\n"
           << "    \"stage_counts\": {\n";
    emitJsonCounts(output, first_epoch, "      ");
    output << "    }\n"
           << "  },\n"
           << "  \"route_aggregate\": {\n"
           << "    \"problem_epochs\": " << problem.epochs.size() << ",\n"
           << "    \"final_corrected_undifferenced_factor_rows\": "
           << problem.undifferenced_doppler_factors.size() << ",\n"
           << "    \"stage_counts\": {\n";
    emitJsonCounts(output, aggregate, "      ");
    output << "    }\n"
           << "  },\n"
           << "  \"pipeline\": {\n"
           << "    \"first_reason_is_exclusive\": true,\n"
           << "    \"stage_order\": [\n"
           << "      \"raw_row\",\n"
           << "      \"valid_and_doppler_mask\",\n"
           << "      \"raw_snr_and_multipath_mask\",\n"
           << "      \"supported_constellation_signal_frequency\",\n"
           << "      \"raw_pseudorange_validity\",\n"
           << "      \"transmit_time\",\n"
           << "      \"nav_state_call1\",\n"
           << "      \"nav_state_call2\",\n"
           << "      \"ephemeris_present_health\",\n"
           << "      \"snr_elevation_masks\",\n"
           << "      \"clock_jump_dt\",\n"
           << "      \"includes_receiver_clock_drift\",\n"
           << "      \"final_factor\"\n"
           << "    ],\n"
           << "    \"factor_definition\": \"Unchanged FGOProblem corrected undifferenced factor with includes_receiver_clock_drift=true\",\n"
           << "    \"raw_key\": \"dataset_id/raw_row_index/utc_time_millis/raw_epoch_index/satellite/signal\"\n"
           << "  },\n"
           << "  \"decision\": {\n"
           << "    \"concrete_generalizable_bug_proven\": false,\n"
           << "    \"correction_applied\": false,\n"
           << "    \"threshold_or_sign_change\": false,\n"
           << "    \"classification\": \"diagnosis-only; attrition evidence\",\n"
           << "    \"reason\": \"This executable measures row selection on frozen raw/nav inputs; implausible velocity, threshold relaxation, and coordinate substitution are not bug evidence.\"\n"
           << "  },\n"
           << "  \"forbidden_input_accounting\": {\n"
           << "    \"truth_open_count\": 0,\n"
           << "    \"validation_open_count\": 0,\n"
           << "    \"holdout_open_count\": 0,\n"
           << "    \"imu_open_count\": 0,\n"
           << "    \"mat_read_or_generated\": false,\n"
           << "    \"precomputed_coordinate_read_count\": 0,\n"
           << "    \"device_wls_coordinate_read_count\": 0,\n"
           << "    \"kaggle_or_token_access\": false\n"
           << "  }\n"
           << "}\n";
    return output.str();
}

FGOProcessor::FGOProblem buildPhase41Problem(
    const libgnss::io::AndroidRawGnssResult& converted,
    const libgnss::NavigationData& nav) {
    const Vector3d raw_seed(kInitialRawSeedX, kInitialRawSeedY,
                            kInitialRawSeedZ);
    std::vector<ObservationData> epochs = converted.observations.epochs;
    for (auto& epoch : epochs) epoch.receiver_position = raw_seed;

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
    return processor.buildPseudorangeProblem(epochs, nav);
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!parseOptions(argc, argv, options)) return 2;
    if (forbiddenPath(options.android_gnss) || forbiddenPath(options.nav) ||
        forbiddenPath(options.rows_csv) || forbiddenPath(options.summary_json)) {
        std::cerr << "Phase42 raw-only path contract rejected an input/output path\n";
        return 2;
    }
    if (!regularFile(options.android_gnss) || !regularFile(options.nav)) {
        std::cerr << "Phase42 audit requires regular raw GNSS and broadcast nav files\n";
        return 2;
    }

    libgnss::io::AndroidRawGnssConfig android_config;
    android_config.verify_enriched_pseudorange = false;
    libgnss::io::AndroidRawGnssResult converted;
    std::string conversion_error;
    if (!libgnss::io::loadAndroidRawGnssCsv(
            options.android_gnss, android_config, converted, conversion_error)) {
        std::cerr << "raw Android GNSS conversion failed: " << conversion_error
                  << '\n';
        return 1;
    }
    if (converted.observations.epochs.empty() ||
        converted.raw_row_diagnostics.empty()) {
        std::cerr << "raw Android GNSS has no selected epochs/raw rows\n";
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

    // Keep this exact Phase41 problem build in one process.  It intentionally
    // does not pass the raw device WLS columns to the graph.
    const FGOProcessor::FGOProblem problem =
        buildPhase41Problem(converted, nav);
    if (problem.epochs.empty() ||
        problem.doppler_velocity_wls_estimates.size() != problem.epochs.size()) {
        std::cerr << "FGO problem did not preserve epoch/WLS alignment\n";
        return 1;
    }

    std::vector<std::size_t> problem_for_raw(converted.observations.epochs.size(),
                                             kNoIndex);
    for (std::size_t p = 0; p < problem.epochs.size(); ++p) {
        for (std::size_t r = 0; r < converted.observations.epochs.size(); ++r) {
            if (sameEpoch(problem.epochs[p].time,
                          converted.observations.epochs[r].time)) {
                problem_for_raw[r] = p;
                break;
            }
        }
    }

    std::vector<std::vector<const FGOProcessor::UndifferencedDopplerFactor*>>
        factors_by_epoch(problem.epochs.size());
    for (const auto& factor : problem.undifferenced_doppler_factors) {
        if (factor.epoch_index < factors_by_epoch.size()) {
            factors_by_epoch[factor.epoch_index].push_back(&factor);
        }
    }
    std::size_t first_problem_epoch = problem.epochs.size();
    for (std::size_t p = 0; p < factors_by_epoch.size(); ++p) {
        if (factors_by_epoch[p].size() >= 4U) {
            first_problem_epoch = p;
            break;
        }
    }
    if (first_problem_epoch == problem.epochs.size()) {
        std::cerr << "no first solvable epoch with four FGO Doppler rows\n";
        return 1;
    }
    std::size_t first_raw_epoch = converted.observations.epochs.size();
    for (std::size_t r = 0; r < converted.observations.epochs.size(); ++r) {
        if (sameEpoch(problem.epochs[first_problem_epoch].time,
                      converted.observations.epochs[r].time)) {
            first_raw_epoch = r;
            break;
        }
    }
    if (first_raw_epoch == converted.observations.epochs.size()) {
        std::cerr << "cannot map first FGO epoch to raw epoch\n";
        return 1;
    }

    std::vector<std::map<RowKey, const FGOProcessor::UndifferencedDopplerFactor*>>
        factors_by_key(problem.epochs.size());
    for (const auto& factor : problem.undifferenced_doppler_factors) {
        if (factor.epoch_index < factors_by_key.size()) {
            factors_by_key[factor.epoch_index][{factor.satellite, factor.signal}] =
                &factor;
        }
    }

    std::map<std::int64_t, std::size_t> raw_epoch_by_utc;
    for (std::size_t r = 0; r < converted.epoch_utc_time_millis.size(); ++r) {
        raw_epoch_by_utc.emplace(converted.epoch_utc_time_millis[r], r);
    }

    std::vector<RowAudit> rows;
    rows.reserve(converted.raw_row_diagnostics.size());
    for (const auto& raw : converted.raw_row_diagnostics) {
        RowAudit row;
        row.raw = &raw;
        row.satellite = raw.has_parsed_signal
                            ? SatelliteId(raw.system,
                                          static_cast<std::uint8_t>(raw.svid))
                                  .toString()
                            : rawSatelliteName(raw);
        row.signal = raw.has_parsed_signal ? signalName(raw.signal)
                                           : raw.signal_token;
        if (raw.selected) {
            row.raw_epoch_index = raw.selected_epoch_index;
        } else {
            const auto raw_epoch_it = raw_epoch_by_utc.find(raw.utc_time_millis);
            if (raw_epoch_it != raw_epoch_by_utc.end()) {
                row.raw_epoch_index = raw_epoch_it->second;
            }
        }
        if (row.raw_epoch_index < problem_for_raw.size()) {
            row.problem_epoch_index = problem_for_raw[row.raw_epoch_index];
        }

        if (!raw.selected) {
            row.first_reason = raw.loader_reason.empty()
                                   ? "loader_rejected_raw_row"
                                   : raw.loader_reason;
            rows.push_back(row);
            continue;
        }
        if (row.raw_epoch_index >= converted.observations.epochs.size()) {
            setReason(row, "raw_epoch_mapping_missing");
            rows.push_back(row);
            continue;
        }
        for (const auto& observation :
             converted.observations.epochs[row.raw_epoch_index].observations) {
            if (observation.raw_row_index == raw.raw_row_index) {
                row.observation = &observation;
                break;
            }
        }
        if (row.observation == nullptr) {
            setReason(row, "selected_observation_mapping_missing");
            rows.push_back(row);
            continue;
        }
        const Observation& observation = *row.observation;
        row.valid = observation.valid;
        row.doppler = observation.has_doppler &&
                      std::isfinite(observation.doppler) &&
                      observation.doppler != 0.0;
        row.raw_snr_masked = raw.snr_masked;
        row.raw_multipath_masked = raw.multipath_masked;
        if (!row.valid) {
            setReason(row, "invalid_observation");
        } else if (raw.snr_masked) {
            setReason(row, "raw_snr_mask");
        } else if (raw.multipath_masked) {
            setReason(row, "raw_multipath_mask");
        } else if (!row.doppler) {
            setReason(row, "doppler_mask_or_zero");
        }
        const SatelliteId satellite = observation.satellite;
        row.supported_signal = raw.supported_signal &&
                               supportedFgoSignal(satellite, observation.signal);
        row.frequency_valid = row.supported_signal &&
                              std::isfinite(libgnss::signalFrequencyHz(observation)) &&
                              libgnss::signalFrequencyHz(observation) > 0.0;
        if (row.first_reason.empty() && !row.supported_signal) {
            setReason(row, "unsupported_constellation_signal_frequency");
        } else if (row.first_reason.empty() && !row.frequency_valid) {
            setReason(row, "unsupported_constellation_signal_frequency");
        }
        row.pseudorange_valid = observation.has_pseudorange &&
                                std::isfinite(observation.pseudorange) &&
                                observation.pseudorange > 0.0;
        if (row.first_reason.empty() && !row.pseudorange_valid) {
            setReason(row, "raw_pseudorange_invalid");
        }
        if (row.first_reason.empty() && row.problem_epoch_index == kNoIndex) {
            setReason(row, "problem_epoch_unavailable");
        }
        if (row.first_reason.empty()) {
            const auto& epoch = converted.observations.epochs[row.raw_epoch_index];
            const double travel_time_s =
                observation.pseudorange / libgnss::constants::SPEED_OF_LIGHT;
            const GNSSTime transmit_time = epoch.time - travel_time_s;
            row.transmit_time_valid = std::isfinite(travel_time_s) &&
                                      travel_time_s > 0.0 &&
                                      finiteTime(transmit_time);
            if (!row.transmit_time_valid) {
                setReason(row, "transmit_time_invalid");
            } else {
                Vector3d satellite_position = Vector3d::Zero();
                Vector3d satellite_velocity = Vector3d::Zero();
                double satellite_clock_bias = 0.0;
                double satellite_clock_drift = 0.0;
                row.nav_state_call1 = nav.calculateSatelliteState(
                    satellite, transmit_time, satellite_position,
                    satellite_velocity, satellite_clock_bias,
                    satellite_clock_drift);
                if (!row.nav_state_call1) {
                    setReason(row, "nav_state_call1_failed");
                } else {
                    const GNSSTime corrected_transmit_time =
                        transmit_time - satellite_clock_bias;
                    row.nav_state_call2 = nav.calculateSatelliteState(
                        satellite, corrected_transmit_time, satellite_position,
                        satellite_velocity, satellite_clock_bias,
                        satellite_clock_drift);
                    if (!row.nav_state_call2) {
                        setReason(row, "nav_state_call2_failed");
                    } else {
                        const libgnss::Ephemeris* eph =
                            nav.getEphemeris(satellite, corrected_transmit_time);
                        row.ephemeris_present = eph != nullptr;
                        row.ephemeris_healthy = eph != nullptr &&
                                                healthyForPositioning(observation, *eph);
                        if (!row.ephemeris_present) {
                            setReason(row, "ephemeris_missing");
                        } else if (!row.ephemeris_healthy) {
                            setReason(row, "ephemeris_unhealthy");
                        } else {
                            const std::size_t p = row.problem_epoch_index;
                            const Vector3d seed = problem.epochs[p].position_ecef;
                            Vector3d corrected_position = Vector3d::Zero();
                            Vector3d corrected_velocity = Vector3d::Zero();
                            row.geometry_valid =
                                libgnss::doppler_contract::earthRotationCorrectedSatelliteState(
                                    satellite_position, satellite_velocity, seed,
                                    corrected_position, corrected_velocity);
                            if (!row.geometry_valid) {
                                setReason(row, "geometry_invalid");
                            } else {
                                const auto geometry = nav.calculateGeometry(
                                    seed, corrected_position);
                                row.snr_mask_pass = std::isfinite(observation.snr) &&
                                                     observation.snr >= 0.0;
                                row.elevation_mask_pass =
                                    std::isfinite(geometry.elevation) &&
                                    geometry.elevation >= 0.0;
                                if (!row.snr_mask_pass) {
                                    setReason(row, "snr_mask");
                                } else if (!row.elevation_mask_pass) {
                                    setReason(row, "elevation_mask");
                                } else {
                                    Vector3d doppler_los = Vector3d::Zero();
                                    double modeled_range_rate = 0.0;
                                    const bool doppler_geometry_valid =
                                        libgnss::doppler_contract::knownSatelliteRangeRate(
                                            satellite_position, satellite_velocity,
                                            seed, true, doppler_los,
                                            modeled_range_rate);
                                    const double wavelength =
                                        libgnss::signalWavelengthMeters(observation);
                                    const double measured_range_rate =
                                        libgnss::doppler_contract::rinexDopplerToRangeRate(
                                            observation.doppler,
                                            libgnss::constants::SPEED_OF_LIGHT /
                                                wavelength);
                                    const double residual =
                                        libgnss::doppler_contract::receiverOnlyResidual(
                                            measured_range_rate, modeled_range_rate,
                                            satellite_clock_drift);
                                    if (!doppler_geometry_valid ||
                                        !std::isfinite(wavelength) || wavelength <= 0.0 ||
                                        !std::isfinite(residual)) {
                                        setReason(row, "doppler_geometry_or_residual_invalid");
                                    } else {
                                        row.clock_jump = p < problem.clock_jumps.size()
                                                              ? problem.clock_jumps[p]
                                                              : false;
                                        if (p == 0U) {
                                        // The corrected Phase41 FGO branch
                                        // intentionally cannot identify receiver
                                        // clock drift from one graph epoch.
                                            setReason(row, "clock_drift_unavailable_first_epoch");
                                        } else {
                                        row.dt_s = problem.epochs[p].time -
                                                   problem.epochs[p - 1U].time;
                                        row.dt_valid = std::isfinite(row.dt_s) &&
                                                        row.dt_s > 0.0 &&
                                                        row.dt_s <= 1.0;
                                        if (!row.dt_valid) {
                                            setReason(row, "clock_dt_invalid_or_gap");
                                        } else {
                                            const auto factor_it = factors_by_key[p].find(
                                                {satellite, observation.signal});
                                            if (factor_it == factors_by_key[p].end()) {
                                                setReason(row, "final_factor_missing");
                                            } else {
                                                row.fgo_factor_present = true;
                                                row.includes_receiver_clock_drift =
                                                    factor_it->second->includes_receiver_clock_drift;
                                                if (!row.includes_receiver_clock_drift) {
                                                    setReason(row, "receiver_clock_drift_not_included");
                                                } else {
                                                    setReason(row, "accepted_final_factor");
                                                }
                                            }
                                        }
                                    }
                                        }
                                }
                            }
                        }
                    }
                }
            }
        }
        rows.push_back(row);
    }

    Counts aggregate;
    Counts first_epoch;
    std::size_t first_selected_rows = 0;
    std::size_t first_factor_rows = 0;
    std::size_t mapping_mismatches = 0;
    for (const auto& row : rows) {
        addCounts(aggregate, row);
        if (row.raw_epoch_index == first_raw_epoch) {
            addCounts(first_epoch, row);
            if (row.raw != nullptr && row.raw->selected) ++first_selected_rows;
            if (row.fgo_factor_present) ++first_factor_rows;
            if (row.raw != nullptr && row.raw->selected &&
                row.raw_epoch_index < converted.epoch_utc_time_millis.size() &&
                converted.epoch_utc_time_millis[row.raw_epoch_index] !=
                    row.raw->utc_time_millis) {
                ++mapping_mismatches;
            }
        }
    }
    std::size_t clock_jump_epochs = 0;
    for (std::size_t p = 0; p < problem.clock_jumps.size(); ++p) {
        if (problem.clock_jumps[p]) ++clock_jump_epochs;
    }
    if (first_factor_rows != factors_by_epoch[first_problem_epoch].size()) {
        std::cerr << "first epoch row/factor identity count mismatch\n";
        return 1;
    }
    if (aggregate.fgo_factor_rows != problem.undifferenced_doppler_factors.size()) {
        std::cerr << "route factor count does not match unchanged FGO problem\n";
        return 1;
    }
    if (aggregate.raw_rows != converted.diagnostics.raw_rows) {
        std::cerr << "parsed raw-row provenance count does not match loader raw count\n";
        return 1;
    }

    std::ostringstream csv;
    csv << std::setprecision(std::numeric_limits<double>::max_digits10);
    csv << "dataset_id,raw_row_index,input_row_index,raw_epoch_index,problem_epoch_index,"
           "utc_time_millis,gps_week,gps_tow,satellite,system,prn,signal,signal_token,"
           "carrier_frequency_hz,raw_rate_mps,cn0_dbhz,snr_masked,multipath_masked,"
           "code_masked,doppler_masked,selected,valid,doppler,raw_pseudorange_valid,"
           "supported_signal,frequency_valid,transmit_time_valid,nav_state_call1,"
           "nav_state_call2,ephemeris_present,ephemeris_healthy,geometry_valid,"
           "snr_mask_pass,elevation_mask_pass,clock_jump,dt_s,dt_valid,"
           "includes_receiver_clock_drift,fgo_factor_present,first_reason\n";
    for (const auto& row : rows) {
        const RawDiagnostic& raw = *row.raw;
        int week = 0;
        double tow = std::numeric_limits<double>::quiet_NaN();
        if (row.raw_epoch_index < converted.observations.epochs.size()) {
            week = converted.observations.epochs[row.raw_epoch_index].time.week;
            tow = converted.observations.epochs[row.raw_epoch_index].time.tow;
        }
        const std::string system = raw.has_parsed_signal
                                        ? systemName(raw.system)
                                        : (raw.constellation == 4 ? "QZSS" : "UNKNOWN");
        csv << options.dataset_id << ',' << raw.raw_row_index << ','
            << raw.input_row_index << ','
            << (row.raw_epoch_index == kNoIndex ? "" : std::to_string(row.raw_epoch_index))
            << ','
            << (row.problem_epoch_index == kNoIndex
                    ? ""
                    : std::to_string(row.problem_epoch_index))
            << ',' << raw.utc_time_millis << ',' << week << ',' << csvNumber(tow)
            << ',' << row.satellite << ',' << system << ',' << raw.svid << ','
            << row.signal << ',' << raw.signal_token << ','
            << csvNumber(raw.carrier_frequency_hz) << ','
            << csvNumber(raw.pseudorange_rate_mps) << ',' << csvNumber(raw.cn0_dbhz)
            << ',' << (raw.snr_masked ? 1 : 0) << ','
            << (raw.multipath_masked ? 1 : 0) << ','
            << (raw.code_masked ? 1 : 0) << ','
            << (raw.doppler_masked ? 1 : 0) << ',' << (raw.selected ? 1 : 0)
            << ',' << (row.valid ? 1 : 0) << ',' << (row.doppler ? 1 : 0) << ','
            << (row.pseudorange_valid ? 1 : 0) << ','
            << (row.supported_signal ? 1 : 0) << ','
            << (row.frequency_valid ? 1 : 0) << ','
            << (row.transmit_time_valid ? 1 : 0) << ','
            << (row.nav_state_call1 ? 1 : 0) << ','
            << (row.nav_state_call2 ? 1 : 0) << ','
            << (row.ephemeris_present ? 1 : 0) << ','
            << (row.ephemeris_healthy ? 1 : 0) << ','
            << (row.geometry_valid ? 1 : 0) << ','
            << (row.snr_mask_pass ? 1 : 0) << ','
            << (row.elevation_mask_pass ? 1 : 0) << ','
            << (row.clock_jump ? 1 : 0) << ',' << csvNumber(row.dt_s) << ','
            << (row.dt_valid ? 1 : 0) << ','
            << (row.includes_receiver_clock_drift ? 1 : 0) << ','
            << (row.fgo_factor_present ? 1 : 0) << ',' << row.first_reason << '\n';
    }
    if (!atomicWrite(options.rows_csv, csv.str())) {
        std::cerr << "failed to publish row attrition CSV\n";
        return 1;
    }
    const std::string summary = makeSummary(
        options, converted, problem, first_problem_epoch, first_raw_epoch,
        aggregate, first_epoch, problem.epochs[first_problem_epoch].time,
        converted.epoch_utc_time_millis[first_raw_epoch], first_selected_rows,
        first_factor_rows, clock_jump_epochs, mapping_mismatches);
    if (!atomicWrite(options.summary_json, summary)) {
        std::cerr << "failed to publish row attrition summary\n";
        return 1;
    }
    std::cout << "Phase42 Doppler row attrition audit: dataset="
              << options.dataset_id << " first_problem_epoch="
              << first_problem_epoch << " first_raw_epoch=" << first_raw_epoch
              << " raw_rows=" << first_epoch.raw_rows
              << " selected_rows=" << first_selected_rows
              << " fgo_rows=" << first_factor_rows
              << " aggregate_raw_rows=" << aggregate.raw_rows
              << " aggregate_fgo_rows=" << aggregate.fgo_factor_rows << '\n';
    return 0;
}
