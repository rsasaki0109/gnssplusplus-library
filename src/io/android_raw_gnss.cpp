#include <libgnss++/io/android_raw_gnss.hpp>

#include <libgnss++/core/constants.hpp>

#include <algorithm>
#include <charconv>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <climits>
#include <initializer_list>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string_view>
#include <utility>
#include <vector>

namespace libgnss::io {
namespace {

constexpr long double kNanosecondsPerSecond = 1.0e9L;
constexpr long double kSecondsPerWeek = 604800.0L;
constexpr long double kHalfWeek = 302400.0L;
constexpr std::int64_t kTimeJumpNanoseconds = 1'000'000'000LL;
constexpr std::int64_t kMaxReasonableTimeNanoseconds = 4'000'000'000'000'000'000LL;
constexpr double kGpsL1FrequencyHz = constants::GPS_L1_FREQ;
constexpr double kGpsL5FrequencyHz = constants::GPS_L5_FREQ;
constexpr double kGlonassL1BaseFrequencyHz = constants::GLO_L1_BASE_FREQ;
constexpr double kGlonassL1StepFrequencyHz = constants::GLO_L1_STEP_FREQ;
constexpr double kGalileoE1FrequencyHz = constants::GAL_E1_FREQ;
constexpr double kGalileoE5aFrequencyHz = constants::GAL_E5A_FREQ;
constexpr double kBeiDouB1iFrequencyHz = constants::BDS_B1I_FREQ;
constexpr double kBeiDouB1cFrequencyHz = constants::BDS_B1C_FREQ;
constexpr double kBeiDouB2aFrequencyHz = constants::BDS_B2A_FREQ;

struct ColumnMap {
    std::map<std::string, std::size_t> index;

    std::string value(const std::vector<std::string>& row,
                      std::string_view name) const {
        const auto it = index.find(std::string(name));
        if (it == index.end() || it->second >= row.size()) return {};
        return row[it->second];
    }

    bool has(std::string_view name) const {
        return index.find(std::string(name)) != index.end();
    }
};

std::string trim(std::string value) {
    const auto not_space = [](unsigned char character) {
        return character != ' ' && character != '\t' && character != '\r' &&
               character != '\n';
    };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
    value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
    return value;
}

std::string normaliseHeader(std::string value) {
    value = trim(std::move(value));
    std::string normalised;
    normalised.reserve(value.size());
    for (const unsigned char character : value) {
        if (std::isalnum(character)) {
            normalised.push_back(static_cast<char>(std::tolower(character)));
        }
    }
    return normalised;
}

bool parseCsvLine(const std::string& line,
                 std::vector<std::string>& fields,
                 std::string& error) {
    fields.clear();
    std::string field;
    bool quoted = false;
    bool quote_closed = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        const char character = line[i];
        if (quoted) {
            if (character == '"') {
                if (i + 1 < line.size() && line[i + 1] == '"') {
                    field.push_back('"');
                    ++i;
                } else {
                    quoted = false;
                    quote_closed = true;
                }
            } else {
                field.push_back(character);
            }
            continue;
        }
        if (character == '"') {
            if (!field.empty() || quote_closed) {
                error = "quote appeared in the middle of a CSV field";
                return false;
            }
            quoted = true;
            continue;
        }
        if (character == ',') {
            fields.push_back(trim(std::move(field)));
            field.clear();
            quote_closed = false;
            continue;
        }
        if (quote_closed && character != ' ' && character != '\t') {
            error = "characters followed a closing CSV quote";
            return false;
        }
        field.push_back(character);
    }
    if (quoted) {
        error = "unterminated CSV quote";
        return false;
    }
    fields.push_back(trim(std::move(field)));
    return true;
}

bool parseFiniteDouble(const std::string& token,
                       double& value,
                       bool allow_blank = false) {
    const std::string text = trim(token);
    if (text.empty()) {
        if (allow_blank) {
            value = std::numeric_limits<double>::quiet_NaN();
            return true;
        }
        return false;
    }
    char* end = nullptr;
    const double parsed = std::strtod(text.c_str(), &end);
    if (end == text.c_str() || *end != '\0' || !std::isfinite(parsed)) {
        return false;
    }
    value = parsed;
    return true;
}

bool parseInt64(const std::string& token, std::int64_t& value,
                bool allow_blank = false) {
    const std::string text = trim(token);
    if (text.empty()) {
        if (allow_blank) {
            value = 0;
            return true;
        }
        return false;
    }
    const char* first = text.data();
    const char* last = first + text.size();
    const auto parsed = std::from_chars(first, last, value);
    return parsed.ec == std::errc() && parsed.ptr == last;
}

bool parseInt(const std::string& token, int& value, bool allow_blank = false) {
    std::int64_t parsed = 0;
    if (!parseInt64(token, parsed, allow_blank) ||
        parsed < std::numeric_limits<int>::min() ||
        parsed > std::numeric_limits<int>::max()) {
        return false;
    }
    value = static_cast<int>(parsed);
    return true;
}

bool closeEnough(double lhs, double rhs, double tolerance) {
    return std::isfinite(lhs) && std::isfinite(rhs) &&
           std::abs(lhs - rhs) <= tolerance;
}

bool supportedDeviceAdrSign(const std::string& model) {
    return model == "sm-a205u" || model == "sm-a217m" ||
           model == "sm-a505g" || model == "sm-a505u" ||
           model == "sm-a600t";
}

bool publishedGlonassCarrierExcluded(const std::string& model) {
    return model == "sm-a205u" || model == "sm-a217m" ||
           model == "samsungs22ultra" || model == "sm-s908b" ||
           model == "sm-a505g" || model == "sm-a600t" ||
           model == "sm-a505u";
}

bool signalTokenIs(const std::string& token,
                   std::initializer_list<const char*> candidates) {
    for (const char* candidate : candidates) {
        if (token == candidate) return true;
    }
    return false;
}

bool parseSignal(const std::string& signal_token,
                 int constellation,
                 double frequency_hz,
                 bool include_galileo,
                 bool include_l5,
                 SignalType& signal,
                 GNSSSystem& system,
                 int& glonass_frequency_channel) {
    std::string signal_name = trim(signal_token);
    std::transform(signal_name.begin(), signal_name.end(), signal_name.begin(),
                   [](unsigned char character) {
                       return static_cast<char>(std::toupper(character));
                   });
    glonass_frequency_channel = 0;
    const double frequency_tolerance = 1000.0;
    if (constellation == 1 &&
        (signalTokenIs(signal_name, {"GPS_L1_CA", "GPS_L1C", "L1"}) ||
         (signal_name.empty() &&
          closeEnough(frequency_hz, kGpsL1FrequencyHz, frequency_tolerance)))) {
        if (!closeEnough(frequency_hz, kGpsL1FrequencyHz, frequency_tolerance)) {
            return false;
        }
        signal = SignalType::GPS_L1CA;
        system = GNSSSystem::GPS;
        return true;
    }
    if (constellation == 1 && include_l5 &&
        (signalTokenIs(signal_name, {"GPS_L5_Q", "GPS_L5", "L5"}) ||
         (signal_name.empty() &&
          closeEnough(frequency_hz, kGpsL5FrequencyHz, frequency_tolerance)))) {
        if (!closeEnough(frequency_hz, kGpsL5FrequencyHz, frequency_tolerance)) {
            return false;
        }
        signal = SignalType::GPS_L5;
        system = GNSSSystem::GPS;
        return true;
    }
    if (constellation == 3 &&
        (signalTokenIs(signal_name, {"GLO_G1_CA", "GLO_G1C", "GLO_L1", "L1"}) ||
         signal_name.empty())) {
        const double channel_value =
            (frequency_hz - kGlonassL1BaseFrequencyHz) /
            kGlonassL1StepFrequencyHz;
        const int channel = static_cast<int>(std::llround(channel_value));
        if (channel < -7 || channel > 6 ||
            !closeEnough(frequency_hz,
                         kGlonassL1BaseFrequencyHz +
                             static_cast<double>(channel) * kGlonassL1StepFrequencyHz,
                         frequency_tolerance)) {
            return false;
        }
        signal = SignalType::GLO_L1CA;
        system = GNSSSystem::GLONASS;
        glonass_frequency_channel = channel;
        return true;
    }
    if (constellation == 6 &&
        (signalTokenIs(signal_name, {"GAL_E1_C_P", "GAL_E1_C", "GAL_E1", "L1"}) ||
         (signal_name.empty() &&
          closeEnough(frequency_hz, kGalileoE1FrequencyHz, frequency_tolerance)))) {
        if (!closeEnough(frequency_hz, kGalileoE1FrequencyHz, frequency_tolerance)) {
            return false;
        }
        if (!include_galileo) return false;
        signal = SignalType::GAL_E1;
        system = GNSSSystem::Galileo;
        return true;
    }
    if (constellation == 6 && include_galileo && include_l5 &&
        (signalTokenIs(signal_name, {"GAL_E5A_Q", "GAL_E5A", "GAL_E5", "L5"}) ||
         (signal_name.empty() &&
          closeEnough(frequency_hz, kGalileoE5aFrequencyHz, frequency_tolerance)))) {
        if (!closeEnough(frequency_hz, kGalileoE5aFrequencyHz, frequency_tolerance)) {
            return false;
        }
        signal = SignalType::GAL_E5A;
        system = GNSSSystem::Galileo;
        return true;
    }
    if (constellation == 5 &&
        (signalTokenIs(signal_name, {"BDS_B1I", "BDS_B1_I", "BDS_B1", "B1I"}) ||
         (signal_name.empty() &&
          closeEnough(frequency_hz, kBeiDouB1iFrequencyHz, frequency_tolerance)))) {
        if (!closeEnough(frequency_hz, kBeiDouB1iFrequencyHz, frequency_tolerance)) {
            return false;
        }
        signal = SignalType::BDS_B1I;
        system = GNSSSystem::BeiDou;
        return true;
    }
    if (constellation == 5 &&
        (signalTokenIs(signal_name, {"BDS_B1C", "BDS_B1_C"}) ||
         (signal_name.empty() &&
          closeEnough(frequency_hz, kBeiDouB1cFrequencyHz, frequency_tolerance)))) {
        if (!closeEnough(frequency_hz, kBeiDouB1cFrequencyHz, frequency_tolerance)) {
            return false;
        }
        signal = SignalType::BDS_B1C;
        system = GNSSSystem::BeiDou;
        return true;
    }
    if (constellation == 5 && include_l5 &&
        (signalTokenIs(signal_name, {"BDS_B2A", "BDS_B2A_Q", "BDS_B2A_I", "L5"}) ||
         (signal_name.empty() &&
          closeEnough(frequency_hz, kBeiDouB2aFrequencyHz, frequency_tolerance)))) {
        if (!closeEnough(frequency_hz, kBeiDouB2aFrequencyHz, frequency_tolerance)) {
            return false;
        }
        signal = SignalType::BDS_B2A;
        system = GNSSSystem::BeiDou;
        return true;
    }
    return false;
}

struct RawRow {
    std::int64_t utc_millis = 0;
    std::int64_t time_nanos = 0;
    std::int64_t full_bias_nanos = 0;
    double bias_nanos = 0.0;
    double drift_nanos_per_second = std::numeric_limits<double>::quiet_NaN();
    bool has_drift_nanos_per_second = false;
    double bias_uncertainty_nanos = std::numeric_limits<double>::quiet_NaN();
    std::int64_t received_sv_time_nanos = 0;
    double time_offset_nanos = 0.0;
    int hardware_clock_discontinuity_count = 0;
    int svid = 0;
    int constellation = 0;
    int adr_state = 0;
    int state = 0;
    bool has_state = false;
    int multipath_indicator = 0;
    bool has_multipath_indicator = false;
    double pseudorange_rate_mps = 0.0;
    double adr_m = std::numeric_limits<double>::quiet_NaN();
    double cn0_dbhz = 0.0;
    bool has_cn0_dbhz = false;
    double carrier_frequency_hz = 0.0;
    double raw_pseudorange_m = std::numeric_limits<double>::quiet_NaN();
    double receiver_x = std::numeric_limits<double>::quiet_NaN();
    double receiver_y = std::numeric_limits<double>::quiet_NaN();
    double receiver_z = std::numeric_limits<double>::quiet_NaN();
    std::string signal;
};

bool parseRawRow(const std::vector<std::string>& row,
                 const ColumnMap& columns,
                 bool parse_enriched_pseudorange,
                 RawRow& output,
                 std::string& error) {
    const auto requiredInt = [&](std::string_view name, std::int64_t& target) {
        if (!parseInt64(columns.value(row, name), target)) {
            error = "invalid or missing " + std::string(name);
            return false;
        }
        return true;
    };
    if (!requiredInt("utctimemillis", output.utc_millis) ||
        !requiredInt("timenanos", output.time_nanos) ||
        !requiredInt("fullbiasnanos", output.full_bias_nanos) ||
        !requiredInt("receivedsvtimenanos", output.received_sv_time_nanos)) {
        return false;
    }
    if (output.time_nanos < 0 || output.time_nanos > kMaxReasonableTimeNanoseconds ||
        output.received_sv_time_nanos < 0) {
        error = "raw Android timing is outside the finite range";
        return false;
    }
    if (!parseFiniteDouble(columns.value(row, "biasnanos"), output.bias_nanos, true) ||
        !std::isfinite(output.bias_nanos) ||
        !parseInt(columns.value(row, "svid"), output.svid) ||
        !parseInt(columns.value(row, "constellationtype"), output.constellation) ||
        !parseInt(columns.value(row, "accumulateddeltarangestate"), output.adr_state) ||
        !parseFiniteDouble(columns.value(row, "pseudorangeratemeterspersecond"),
                           output.pseudorange_rate_mps) ||
        !parseFiniteDouble(columns.value(row, "carrierfrequencyhz"),
                           output.carrier_frequency_hz)) {
        error = "invalid or missing raw Android measurement field";
        return false;
    }
    if (columns.has("biasuncertaintynanos") &&
        !parseFiniteDouble(columns.value(row, "biasuncertaintynanos"),
                           output.bias_uncertainty_nanos, true)) {
        error = "invalid BiasUncertaintyNanos";
        return false;
    }
    if (columns.has("driftnanospersecond")) {
        if (!parseFiniteDouble(columns.value(row, "driftnanospersecond"),
                               output.drift_nanos_per_second, true)) {
            error = "invalid DriftNanosPerSecond";
            return false;
        }
        output.has_drift_nanos_per_second =
            std::isfinite(output.drift_nanos_per_second);
    }
    if (columns.has("timeoffsetnanos") &&
        !parseFiniteDouble(columns.value(row, "timeoffsetnanos"),
                           output.time_offset_nanos, true)) {
        error = "invalid TimeOffsetNanos";
        return false;
    }
    if (columns.has("hardwareclockdiscontinuitycount") &&
        !parseInt(columns.value(row, "hardwareclockdiscontinuitycount"),
                  output.hardware_clock_discontinuity_count, true)) {
        error = "invalid HardwareClockDiscontinuityCount";
        return false;
    }
    if (columns.has("state")) {
        if (!parseInt(columns.value(row, "state"), output.state, true)) {
            error = "invalid State";
            return false;
        }
        output.has_state = true;
    }
    if (columns.has("multipathindicator")) {
        if (!parseInt(columns.value(row, "multipathindicator"),
                      output.multipath_indicator, true)) {
            error = "invalid MultipathIndicator";
            return false;
        }
        output.has_multipath_indicator = true;
    }
    if (columns.has("accumulateddeltarangemeters") &&
        !parseFiniteDouble(columns.value(row, "accumulateddeltarangemeters"),
                           output.adr_m, true)) {
        error = "invalid AccumulatedDeltaRangeMeters";
        return false;
    }
    if (columns.has("cn0dbhz") &&
        !parseFiniteDouble(columns.value(row, "cn0dbhz"), output.cn0_dbhz, true)) {
        error = "invalid Cn0DbHz";
        return false;
    }
    output.has_cn0_dbhz = columns.has("cn0dbhz");
    if (parse_enriched_pseudorange && columns.has("rawpseudorangemeters") &&
        !parseFiniteDouble(columns.value(row, "rawpseudorangemeters"),
                           output.raw_pseudorange_m, true)) {
        error = "invalid RawPseudorangeMeters";
        return false;
    }
    if (columns.has("signaltype")) {
        output.signal = columns.value(row, "signaltype");
    }
    const auto parseOptionalPosition = [&](std::string_view name, double& target) {
        return !columns.has(name) ||
               parseFiniteDouble(columns.value(row, name), target, true);
    };
    if (!parseOptionalPosition("wlspositionxecefmeters", output.receiver_x) ||
        !parseOptionalPosition("wlspositionyecefmeters", output.receiver_y) ||
        !parseOptionalPosition("wlspositionzecefmeters", output.receiver_z)) {
        error = "invalid raw device WLS seed position";
        return false;
    }
    return true;
}

bool rawClockToGpsTime(const RawRow& row,
                       std::int64_t base_full_bias_nanos,
                       GNSSTime& time,
                       double& clock_bias_seconds) {
    const long double clock_ns =
        static_cast<long double>(row.time_nanos) -
        static_cast<long double>(base_full_bias_nanos);
    const long double gps_seconds = clock_ns / kNanosecondsPerSecond;
    if (!std::isfinite(static_cast<double>(gps_seconds)) || gps_seconds < 0.0L) {
        return false;
    }
    const long double week_value = std::floor(gps_seconds / kSecondsPerWeek);
    if (week_value < 0.0L || week_value > static_cast<long double>(INT_MAX)) {
        return false;
    }
    const long double tow =
        gps_seconds - week_value * kSecondsPerWeek -
        static_cast<long double>(row.bias_nanos) / kNanosecondsPerSecond;
    if (!std::isfinite(static_cast<double>(tow)) || tow < -1e-6L ||
        tow >= kSecondsPerWeek + 1e-6L) {
        return false;
    }
    time = GNSSTime(static_cast<int>(week_value), static_cast<double>(tow));
    clock_bias_seconds =
        static_cast<double>((static_cast<long double>(row.full_bias_nanos) -
                             static_cast<long double>(base_full_bias_nanos)) /
                            kNanosecondsPerSecond);
    return std::isfinite(clock_bias_seconds);
}

double glonassTowToGps(double glonass_tow, long double gps_tow_reference) {
    // Port of gnsslog2obs.m: GLONASS transmit time is a time-of-day value,
    // while the receiver epoch is GPST.  The day wrap is resolved against the
    // receiver GPST reference before the common nearest-week unwrap.
    constexpr long double seconds_per_day = 86400.0L;
    const long double day_of_week =
        std::floor(gps_tow_reference / seconds_per_day);
    long double gpst = static_cast<long double>(glonass_tow) +
                       day_of_week * seconds_per_day - 3.0L * 3600.0L + 18.0L;
    const long double day_offset =
        day_of_week - std::floor(gpst / seconds_per_day);
    gpst += day_offset * seconds_per_day;
    return static_cast<double>(gpst);
}

bool rawPseudorange(const RawRow& row,
                    std::int64_t base_full_bias_nanos,
                    GNSSSystem system,
                    double& pseudorange_m) {
    const long double clock_ns =
        static_cast<long double>(row.time_nanos) -
        static_cast<long double>(base_full_bias_nanos);
    const long double gps_seconds = clock_ns / kNanosecondsPerSecond;
    const long double week_value = std::floor(gps_seconds / kSecondsPerWeek);
    long double tow_rx = gps_seconds - week_value * kSecondsPerWeek -
                         static_cast<long double>(row.bias_nanos) /
                             kNanosecondsPerSecond;
    tow_rx -= static_cast<long double>(row.time_offset_nanos) /
              kNanosecondsPerSecond;
    long double tow_tx = static_cast<long double>(row.received_sv_time_nanos) /
                         kNanosecondsPerSecond;
    if (system == GNSSSystem::GLONASS) {
        tow_tx = static_cast<long double>(
            glonassTowToGps(static_cast<double>(tow_tx), tow_rx));
    } else if (system == GNSSSystem::BeiDou) {
        // BeiDou time is GPST+14 seconds in the upstream converter.
        tow_tx += 14.0L;
    }
    long double delta = tow_rx - tow_tx;
    while (delta > kHalfWeek) {
        delta -= kSecondsPerWeek;
    }
    while (delta < -kHalfWeek) {
        delta += kSecondsPerWeek;
    }
    pseudorange_m = static_cast<double>(delta * constants::SPEED_OF_LIGHT);
    return std::isfinite(pseudorange_m) && pseudorange_m > 0.0;
}

bool validReceiverPosition(const RawRow& row, Vector3d& position) {
    if (!std::isfinite(row.receiver_x) || !std::isfinite(row.receiver_y) ||
        !std::isfinite(row.receiver_z)) {
        return false;
    }
    position = Vector3d(row.receiver_x, row.receiver_y, row.receiver_z);
    const double norm = position.norm();
    return position.allFinite() && norm >= 6.0e6 && norm <= 7.0e6;
}

struct EpochAccumulator {
    GNSSTime time;
    double clock_bias_seconds = 0.0;
    double receiver_clock_drift_mps =
        std::numeric_limits<double>::quiet_NaN();
    int hardware_clock_discontinuity_count = 0;
    Vector3d receiver_position = Vector3d::Zero();
    bool have_receiver_position = false;
    std::map<std::pair<SatelliteId, SignalType>, Observation> observations;
    std::int64_t utc_millis = 0;
};

bool appendEpoch(EpochAccumulator& accumulator,
                 AndroidRawGnssResult& result,
                 std::string& error) {
    if (accumulator.observations.empty()) return true;
    ObservationData epoch(accumulator.time);
    epoch.receiver_clock_bias = accumulator.clock_bias_seconds;
    epoch.receiver_clock_drift_mps = accumulator.receiver_clock_drift_mps;
    if (accumulator.have_receiver_position) {
        epoch.receiver_position = accumulator.receiver_position;
        ++result.diagnostics.receiver_position_rows;
    }
    for (const auto& entry : accumulator.observations) {
        epoch.addObservation(entry.second);
    }
    if (epoch.observations.empty()) {
        error = "selected epoch has no observations";
        return false;
    }
    result.observations.addEpoch(epoch);
    result.epoch_utc_time_millis.push_back(accumulator.utc_millis);
    result.epoch_hardware_clock_discontinuity_count.push_back(
        accumulator.hardware_clock_discontinuity_count);
    ++result.diagnostics.selected_epochs;
    result.diagnostics.last_gps_tow = accumulator.time.tow;
    return true;
}

}  // namespace

bool loadAndroidRawGnssCsv(const std::string& path,
                           const AndroidRawGnssConfig& config,
                           AndroidRawGnssResult& result,
                           std::string& error) {
    result = AndroidRawGnssResult{};
    std::string lowered_path = path;
    std::transform(lowered_path.begin(), lowered_path.end(), lowered_path.begin(),
                   [](unsigned char character) {
                       return static_cast<char>(std::tolower(character));
                   });
    if (lowered_path.find(".mat") != std::string::npos) {
        error = "MATLAB .mat inputs are disabled by the native-only contract";
        return false;
    }
    result.diagnostics.timing_formula =
        "segment_base=first FullBiasNanos; reset when successive |TimeNanos "
        "delta|>1e9 ns; week=floor((TimeNanos-segment_base)/1e9/604800); "
        "epoch_tow=(TimeNanos-segment_base-week*604800e9)/1e9-BiasNanos/1e9; "
        "per-signal tow_rx=epoch_tow-TimeOffsetNanos/1e9 (long double)";
    result.diagnostics.carrier_formula =
        "L=AccumulatedDeltaRangeMeters/(c/CarrierFrequencyHz); "
        "published five-device ADR sign correction is applied";
    result.diagnostics.doppler_formula =
        "D=-PseudorangeRateMetersPerSecond/(c/CarrierFrequencyHz)";
    result.diagnostics.adr_sign_policy =
        "negate ADR for sm-a205u, sm-a217m, sm-a505g, sm-a505u, sm-a600t; "
        "otherwise preserve sign";

    std::ifstream input(path);
    if (!input.is_open()) {
        error = "failed to open raw Android GNSS CSV: " + path;
        return false;
    }
    std::string line;
    if (!std::getline(input, line)) {
        error = "raw Android GNSS CSV has no header: " + path;
        return false;
    }
    std::vector<std::string> header;
    if (!parseCsvLine(line, header, error)) {
        error = "invalid raw Android GNSS header: " + error;
        return false;
    }
    ColumnMap columns;
    for (std::size_t i = 0; i < header.size(); ++i) {
        const std::string key = normaliseHeader(header[i]);
        if (key.empty() || columns.index.count(key) != 0U) {
            error = "raw Android GNSS header has an empty or duplicate field";
            return false;
        }
        columns.index.emplace(key, i);
    }
    result.diagnostics.enriched_pseudorange_input_ignored =
        columns.has("rawpseudorangemeters") && !config.verify_enriched_pseudorange;
    const std::vector<std::string> required = {
        "messagetype", "utctimemillis", "timenanos", "fullbiasnanos",
        "receivedsvtimenanos", "svid", "constellationtype",
        "pseudorangeratemeterspersecond", "accumulateddeltarangestate",
        "accumulateddeltarangemeters", "carrierfrequencyhz"};
    for (const auto& field : required) {
        if (!columns.has(field)) {
            error = "raw Android GNSS header lacks required field: " + field;
            return false;
        }
    }
    if (config.require_raw_android_clock &&
        (!columns.has("timenanos") || !columns.has("fullbiasnanos") ||
         !columns.has("receivedsvtimenanos"))) {
        error = "raw Android clock columns are required by the native-only contract";
        return false;
    }

    bool have_epoch = false;
    EpochAccumulator accumulator;
    std::int64_t base_full_bias_nanos = 0;
    std::int64_t previous_time_nanos = 0;
    std::int64_t previous_utc_millis = std::numeric_limits<std::int64_t>::min();
    int previous_clock_discontinuity = 0;
    bool have_previous = false;
    std::vector<std::string> row;
    while (std::getline(input, line)) {
        // MATLAB readtable ignores a trailing blank record.  Keep that
        // ingestion compatibility explicit without accepting malformed
        // non-empty CSV rows.
        if (trim(line).empty()) continue;
        ++result.diagnostics.input_rows;
        if (!parseCsvLine(line, row, error) || row.size() != header.size()) {
            error = "invalid raw Android GNSS row " +
                    std::to_string(result.diagnostics.input_rows + 1U) +
                    (error.empty() ? std::string{} : ": " + error);
            return false;
        }
        if (trim(columns.value(row, "messagetype")) != "Raw") {
            ++result.diagnostics.skipped_non_raw_rows;
            continue;
        }
        ++result.diagnostics.raw_rows;
        RawRow raw;
        if (!parseRawRow(row, columns, config.verify_enriched_pseudorange, raw,
                         error)) {
            error = "raw Android GNSS row " +
                    std::to_string(result.diagnostics.input_rows + 1U) +
                    ": " + error;
            return false;
        }
        if (!config.verify_enriched_pseudorange &&
            columns.has("rawpseudorangemeters")) {
            ++result.diagnostics.enriched_pseudorange_ignored_rows;
        }
        // gnsslog2obs.m removes zero receiver times and transmit times below
        // 1e10 ns before constructing an epoch (its lines 19-22 and 83-89).
        // Treat these as invalid raw measurements, not as solver failures;
        // retaining them would create a false week-boundary pseudorange.
        if (raw.time_nanos == 0 || raw.received_sv_time_nanos < 10'000'000'000LL) {
            ++result.diagnostics.skipped_invalid_timing_rows;
            continue;
        }
        // This is the same quality gate as gnsslog2obs.m: rows with an
        // unusable receiver-clock uncertainty are removed before frequency
        // grouping and epoch construction.  Older synthetic fixtures may not
        // carry the optional column, in which case no extra gate is applied.
        if (std::isfinite(raw.bias_uncertainty_nanos) &&
            raw.bias_uncertainty_nanos > 1.0e4) {
            ++result.diagnostics.skipped_invalid_quality_rows;
            continue;
        }
        if (raw.utc_millis < 0 ||
            (have_previous && raw.utc_millis < previous_utc_millis)) {
            error = "raw Android utcTimeMillis is not monotonic";
            return false;
        }
        if (!have_previous) {
            base_full_bias_nanos = raw.full_bias_nanos;
            previous_time_nanos = raw.time_nanos;
            previous_clock_discontinuity = raw.hardware_clock_discontinuity_count;
            have_previous = true;
        } else if (std::abs(static_cast<long double>(raw.time_nanos) -
                            static_cast<long double>(previous_time_nanos)) >
                   static_cast<long double>(kTimeJumpNanoseconds)) {
            base_full_bias_nanos = raw.full_bias_nanos;
            ++result.diagnostics.clock_discontinuities;
        } else if (raw.hardware_clock_discontinuity_count !=
                   previous_clock_discontinuity) {
            ++result.diagnostics.clock_discontinuities;
        }

        if (!have_epoch || raw.utc_millis != accumulator.utc_millis) {
            if (have_epoch && !appendEpoch(accumulator, result, error)) return false;
            GNSSTime epoch_time;
            double clock_bias_seconds = 0.0;
            if (!rawClockToGpsTime(raw, base_full_bias_nanos, epoch_time,
                                   clock_bias_seconds)) {
                error = "raw Android clock cannot be represented as GPST";
                return false;
            }
            accumulator = EpochAccumulator{};
            accumulator.utc_millis = raw.utc_millis;
            accumulator.time = epoch_time;
            accumulator.clock_bias_seconds = clock_bias_seconds;
            accumulator.receiver_clock_drift_mps =
                raw.has_drift_nanos_per_second
                    ? raw.drift_nanos_per_second *
                          constants::SPEED_OF_LIGHT / 1.0e9
                    : std::numeric_limits<double>::quiet_NaN();
            accumulator.hardware_clock_discontinuity_count =
                raw.hardware_clock_discontinuity_count;
            have_epoch = true;
            if (result.diagnostics.selected_epochs == 0U) {
                result.diagnostics.first_gps_week = epoch_time.week;
                result.diagnostics.first_gps_tow = epoch_time.tow;
            }
        }
        previous_utc_millis = raw.utc_millis;
        previous_time_nanos = raw.time_nanos;
        previous_clock_discontinuity = raw.hardware_clock_discontinuity_count;

        SignalType signal;
        GNSSSystem system;
        int glonass_frequency_channel = 0;
        if (!parseSignal(raw.signal, raw.constellation, raw.carrier_frequency_hz,
                         config.include_galileo_e1, config.include_l5, signal,
                         system, glonass_frequency_channel)) {
            ++result.diagnostics.skipped_unsupported_signal_rows;
            continue;
        }
        // gnsslog2obs.m drops QZSS/SBAS/IRNSS and unknown GLONASS SVIDs.
        // parseSignal already rejects the former; retain the GLONASS bound
        // here instead of accidentally accepting a synthetic PRN.
        const int prn_max = system == GNSSSystem::GPS
                                ? 32
                                : (system == GNSSSystem::GLONASS ? 24
                                                                 : (system == GNSSSystem::BeiDou ? 63 : 36));
        if (raw.svid < 1 || raw.svid > prn_max) {
            if (system == GNSSSystem::GLONASS) {
                ++result.diagnostics.skipped_unsupported_signal_rows;
                continue;
            }
            error = "supported raw Android row has an invalid SVID";
            return false;
        }
        double pseudorange_m = 0.0;
        if (!rawPseudorange(raw, base_full_bias_nanos, system, pseudorange_m)) {
            // taroz/gsdc2023 keeps these rows through gnsslog2obs.m and
            // removes them in exobs.m's P<1e7 mask.  A non-positive raw-clock
            // range is the same invalid code observation at this boundary;
            // drop only this row so a bad satellite cannot discard an entire
            // route.  No carrier/coordinate is synthesized for it.
            ++result.diagnostics.skipped_invalid_quality_rows;
            continue;
        }
        if (std::isfinite(raw.raw_pseudorange_m)) {
            ++result.diagnostics.enriched_pseudorange_checks;
            if (!closeEnough(pseudorange_m, raw.raw_pseudorange_m,
                             config.enriched_pseudorange_tolerance_m)) {
                ++result.diagnostics.enriched_pseudorange_mismatches;
                if (config.verify_enriched_pseudorange) {
                    error = "raw-clock pseudorange disagrees with enriched RawPseudorangeMeters";
                    return false;
                }
            }
        }

        // Port the raw-status part of taroz's exobs.m before constructing the
        // native observation.  The MATLAB constants are:
        //   PSTATE_CODE_LOCK=2^0|2^10,
        //   PSTATE_TOD_OK=2^7|2^15, PSTATE_TOW_OK=2^3|2^14,
        //   LSTATE_SLIP=2^1|2^2, LSTATE_VALID=2^0, and MultipathIndicator=1.
        // A missing optional status column is kept compatible with older
        // Android exports; a present column is never silently ignored.
        const bool multipath = raw.has_multipath_indicator &&
                               raw.multipath_indicator == 1;
        const bool low_snr = raw.has_cn0_dbhz && raw.cn0_dbhz < 20.0;
        const int code_lock_mask = (1 << 0) | (1 << 10);
        const int transmit_time_mask =
            system == GNSSSystem::GLONASS ? ((1 << 7) | (1 << 15))
                                          : ((1 << 3) | (1 << 14));
        const bool status_code_invalid =
            raw.has_state && ((raw.state & code_lock_mask) == 0 ||
                              (raw.state & transmit_time_mask) == 0);
        const bool code_masked = low_snr || multipath ||
                                 pseudorange_m < 1.0e7 ||
                                 pseudorange_m > 4.0e7 || status_code_invalid;
        const bool doppler_masked = low_snr || multipath;
        const bool carrier_masked =
            low_snr || multipath || (raw.adr_state & ((1 << 1) | (1 << 2))) != 0 ||
            (raw.adr_state & (1 << 0)) == 0 ||
            (system == GNSSSystem::GLONASS &&
             publishedGlonassCarrierExcluded(config.device_model));
        if (code_masked) ++result.diagnostics.masked_code_rows;
        if (doppler_masked) ++result.diagnostics.masked_doppler_rows;
        if (carrier_masked) ++result.diagnostics.masked_carrier_rows;
        const double wavelength =
            constants::SPEED_OF_LIGHT / raw.carrier_frequency_hz;
        Observation observation(SatelliteId(system, static_cast<uint8_t>(raw.svid)),
                                 signal);
        observation.pseudorange = pseudorange_m;
        observation.has_pseudorange = true;
        switch (signal) {
            case SignalType::GPS_L5:
                observation.pseudorange_observation_type = "C5I";
                break;
            case SignalType::GAL_E5A:
                observation.pseudorange_observation_type = "C5I";
                break;
            case SignalType::BDS_B1I:
                observation.pseudorange_observation_type = "C2I";
                break;
            case SignalType::BDS_B2A:
                observation.pseudorange_observation_type = "C5P";
                break;
            default:
                observation.pseudorange_observation_type = "C1C";
                break;
        }
        observation.snr = std::isfinite(raw.cn0_dbhz) ? raw.cn0_dbhz : 0.0;
        observation.valid = true;
        if (system == GNSSSystem::GLONASS) {
            observation.has_glonass_frequency_channel = true;
            observation.glonass_frequency_channel = glonass_frequency_channel;
        }
        observation.signal_strength =
            static_cast<int>(std::clamp(std::round(observation.snr / 6.0), 0.0, 9.0));
        observation.doppler = -raw.pseudorange_rate_mps / wavelength;
        observation.has_doppler = !doppler_masked &&
                                  std::isfinite(observation.doppler);
        if (observation.has_doppler) ++result.diagnostics.doppler_rows;
        observation.has_pseudorange = !code_masked;
        if (std::isfinite(raw.adr_m) && raw.adr_m != 0.0 &&
            std::abs(raw.adr_m) < 1.0e9 && !carrier_masked) {
            double carrier_m = raw.adr_m;
            if (supportedDeviceAdrSign(config.device_model)) carrier_m = -carrier_m;
            observation.carrier_phase = carrier_m / wavelength;
            observation.has_carrier_phase = std::isfinite(observation.carrier_phase);
            observation.carrier_phase_observation_type =
                signal == SignalType::GPS_L5 || signal == SignalType::GAL_E5A ||
                        signal == SignalType::BDS_B2A
                    ? "L5I"
                    : "L1C";
            if (observation.has_carrier_phase) ++result.diagnostics.carrier_rows;
        }
        if ((raw.adr_state & (0x02 | 0x04)) != 0) {
            observation.lli = 1;
            observation.loss_of_lock = true;
            ++result.diagnostics.carrier_loss_rows;
        }
        const SatelliteId satellite(system, static_cast<uint8_t>(raw.svid));
        const auto key = std::make_pair(satellite, signal);
        if (accumulator.observations.count(key) != 0U) {
            if ((config.device_model == "sm-a205u" ||
                 config.device_model == "sm-a600t")) {
                // The published converter keeps one half of these phones'
                // repeated blocks.  At the per-epoch boundary an identical
                // key is equivalent; drop the duplicate deterministically.
                ++result.diagnostics.deduplicated_rows;
                continue;
            }
            error = "duplicate supported satellite/signal row in one raw epoch";
            return false;
        }
        accumulator.observations.emplace(key, observation);
        Vector3d receiver_position;
        if (validReceiverPosition(raw, receiver_position)) {
            if (accumulator.have_receiver_position &&
                (accumulator.receiver_position - receiver_position).norm() > 1e-3) {
                error = "raw epoch has inconsistent device WLS seed positions";
                return false;
            }
            accumulator.receiver_position = receiver_position;
            accumulator.have_receiver_position = true;
        }
        ++result.diagnostics.selected_rows;
        switch (signal) {
            case SignalType::GPS_L1CA:
                ++result.diagnostics.gps_rows;
                ++result.diagnostics.gps_l1_rows;
                break;
            case SignalType::GPS_L5:
                ++result.diagnostics.gps_rows;
                ++result.diagnostics.gps_l5_rows;
                break;
            case SignalType::GLO_L1CA:
                ++result.diagnostics.glonass_l1_rows;
                break;
            case SignalType::GAL_E1:
                ++result.diagnostics.galileo_e1_rows;
                break;
            case SignalType::GAL_E5A:
                ++result.diagnostics.galileo_e5_rows;
                break;
            case SignalType::BDS_B1I:
            case SignalType::BDS_B1C:
                ++result.diagnostics.beidou_l1_rows;
                break;
            case SignalType::BDS_B2A:
                ++result.diagnostics.beidou_l5_rows;
                break;
            default:
                break;
        }
    }
    if (have_epoch && !appendEpoch(accumulator, result, error)) return false;
    if (result.observations.epochs.empty()) {
        error = "raw Android GNSS CSV has no supported finite observations";
        return false;
    }
    if (result.epoch_utc_time_millis.size() !=
        result.observations.epochs.size()) {
        error = "raw Android UTC epoch-key alignment invariant failed";
        return false;
    }
    if (result.epoch_hardware_clock_discontinuity_count.size() !=
        result.observations.epochs.size()) {
        error = "raw Android hardware-clock epoch alignment invariant failed";
        return false;
    }
    return true;
}

bool alignAndroidRawGnssSolutionsToUtcKeys(
    const std::vector<GNSSTime>& raw_epoch_times,
    const std::vector<std::int64_t>& epoch_utc_time_millis,
    const std::vector<AndroidRawGnssSolutionPoint>& solutions,
    double solution_time_tolerance_ms,
    AndroidRawGnssEpochAlignment& alignment,
    std::string& error) {
    alignment = AndroidRawGnssEpochAlignment{};
    if (raw_epoch_times.size() != epoch_utc_time_millis.size()) {
        error = "raw observation epochs and integer UTC keys are not one-to-one";
        return false;
    }
    if (raw_epoch_times.size() < 2U) {
        error = "at least two raw epochs are required for the warm-up contract";
        return false;
    }
    if (solutions.empty() || !std::isfinite(solution_time_tolerance_ms) ||
        solution_time_tolerance_ms < 0.0) {
        error = "solution alignment requires finite solutions and tolerance";
        return false;
    }
    for (std::size_t i = 1U; i < epoch_utc_time_millis.size(); ++i) {
        if (epoch_utc_time_millis[i] <= epoch_utc_time_millis[i - 1U]) {
            error = "raw integer UTC epoch keys are not strictly increasing";
            return false;
        }
    }

    const std::size_t raw_epoch_count = raw_epoch_times.size();
    std::vector<int> solution_for_raw(raw_epoch_count, -1);
    std::size_t raw_cursor = 0U;
    for (std::size_t solution_index = 0U;
         solution_index < solutions.size(); ++solution_index) {
        const auto& solution = solutions[solution_index];
        if (!solution.position_ecef.allFinite() ||
            solution.position_ecef.norm() < 6.0e6 ||
            solution.position_ecef.norm() > 7.0e6) {
            error = "native solution has non-finite or out-of-Earth ECEF position";
            return false;
        }
        if (solution_index > 0U &&
            solution.time < solutions[solution_index - 1U].time) {
            error = "native solution times are not monotonic";
            return false;
        }
        while (raw_cursor + 1U < raw_epoch_count) {
            const double current_delta = std::abs(
                raw_epoch_times[raw_cursor] - solution.time);
            const double next_delta = std::abs(
                raw_epoch_times[raw_cursor + 1U] - solution.time);
            if (next_delta < current_delta) {
                ++raw_cursor;
                continue;
            }
            break;
        }
        const double delta_ms = 1000.0 * std::abs(
            raw_epoch_times[raw_cursor] - solution.time);
        if (!std::isfinite(delta_ms) || delta_ms > solution_time_tolerance_ms) {
            error = "native solution time is outside raw UTC epoch tolerance";
            return false;
        }
        if (solution_for_raw[raw_cursor] >= 0) {
            error = "multiple native solutions matched one raw UTC epoch";
            return false;
        }
        solution_for_raw[raw_cursor] = static_cast<int>(solution_index);
    }

    alignment.target_epochs = raw_epoch_count - 1U;
    alignment.epochs.reserve(alignment.target_epochs);
    for (std::size_t raw_index = 1U; raw_index < raw_epoch_count; ++raw_index) {
        AndroidRawGnssAlignedEpoch aligned;
        aligned.utc_time_millis = epoch_utc_time_millis[raw_index];
        const int exact_solution_index = solution_for_raw[raw_index];
        if (exact_solution_index >= 0) {
            aligned.position_ecef =
                solutions[static_cast<std::size_t>(exact_solution_index)].position_ecef;
            aligned.source = AndroidRawGnssEpochSource::ExactSolution;
            ++alignment.exact_solution_epochs;
            alignment.epochs.push_back(aligned);
            continue;
        }

        std::size_t left = raw_index;
        while (left > 0U && solution_for_raw[left] < 0) --left;
        std::size_t right = raw_index;
        while (right + 1U < raw_epoch_count && solution_for_raw[right] < 0) {
            ++right;
        }
        const bool have_left = solution_for_raw[left] >= 0 && left < raw_index;
        const bool have_right = solution_for_raw[right] >= 0 && right > raw_index;
        if (have_left && have_right) {
            const double span_ms = static_cast<double>(
                epoch_utc_time_millis[right] - epoch_utc_time_millis[left]);
            const double offset_ms = static_cast<double>(
                epoch_utc_time_millis[raw_index] - epoch_utc_time_millis[left]);
            if (!(span_ms > 0.0) || offset_ms < 0.0 || offset_ms > span_ms) {
                error = "invalid same-trip ECEF interpolation bracket";
                return false;
            }
            const double fraction = offset_ms / span_ms;
            const Vector3d& left_position =
                solutions[static_cast<std::size_t>(solution_for_raw[left])].position_ecef;
            const Vector3d& right_position =
                solutions[static_cast<std::size_t>(solution_for_raw[right])].position_ecef;
            aligned.position_ecef = left_position + fraction * (right_position - left_position);
            aligned.source = AndroidRawGnssEpochSource::EcefLinearInterpolation;
            alignment.max_interpolation_gap_ms = std::max(
                alignment.max_interpolation_gap_ms, span_ms);
            ++alignment.interpolated_epochs;
        } else {
            const std::size_t edge = have_left ? left : (have_right ? right : raw_epoch_count);
            if (edge == raw_epoch_count) {
                ++alignment.unresolved_epochs;
                continue;
            }
            aligned.position_ecef =
                solutions[static_cast<std::size_t>(solution_for_raw[edge])].position_ecef;
            aligned.source = AndroidRawGnssEpochSource::NearestEdgeHold;
            alignment.max_edge_hold_gap_ms = std::max(
                alignment.max_edge_hold_gap_ms,
                std::abs(static_cast<double>(epoch_utc_time_millis[raw_index] -
                                             epoch_utc_time_millis[edge])));
            ++alignment.edge_hold_epochs;
        }
        if (!aligned.position_ecef.allFinite() ||
            aligned.position_ecef.norm() < 6.0e6 ||
            aligned.position_ecef.norm() > 7.0e6) {
            error = "aligned ECEF position is non-finite or out of Earth range";
            return false;
        }
        alignment.epochs.push_back(aligned);
    }
    if (alignment.unresolved_epochs != 0U ||
        alignment.epochs.size() != alignment.target_epochs) {
        error = "one or more target raw UTC epochs could not be resolved";
        return false;
    }
    return true;
}

}  // namespace libgnss::io
