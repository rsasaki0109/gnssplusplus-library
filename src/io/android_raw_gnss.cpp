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
constexpr double kGalileoE1FrequencyHz = constants::GAL_E1_FREQ;

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

bool parseSignal(const std::string& signal_token,
                 int constellation,
                 double frequency_hz,
                 bool include_galileo,
                 SignalType& signal,
                 GNSSSystem& system) {
    const std::string signal_name = trim(signal_token);
    if (signal_name == "GPS_L1_CA" ||
        (signal_name.empty() && constellation == 1 &&
         closeEnough(frequency_hz, kGpsL1FrequencyHz, 1000.0))) {
        signal = SignalType::GPS_L1CA;
        system = GNSSSystem::GPS;
        return closeEnough(frequency_hz, kGpsL1FrequencyHz, 1000.0);
    }
    if (signal_name == "GAL_E1_C_P" ||
        (signal_name.empty() && constellation == 6 &&
         closeEnough(frequency_hz, kGalileoE1FrequencyHz, 1000.0))) {
        if (!include_galileo) return false;
        signal = SignalType::GAL_E1;
        system = GNSSSystem::Galileo;
        return closeEnough(frequency_hz, kGalileoE1FrequencyHz, 1000.0);
    }
    return false;
}

struct RawRow {
    std::int64_t utc_millis = 0;
    std::int64_t time_nanos = 0;
    std::int64_t full_bias_nanos = 0;
    double bias_nanos = 0.0;
    std::int64_t received_sv_time_nanos = 0;
    int hardware_clock_discontinuity_count = 0;
    int svid = 0;
    int constellation = 0;
    int adr_state = 0;
    double pseudorange_rate_mps = 0.0;
    double adr_m = std::numeric_limits<double>::quiet_NaN();
    double cn0_dbhz = 0.0;
    double carrier_frequency_hz = 0.0;
    double raw_pseudorange_m = std::numeric_limits<double>::quiet_NaN();
    double receiver_x = std::numeric_limits<double>::quiet_NaN();
    double receiver_y = std::numeric_limits<double>::quiet_NaN();
    double receiver_z = std::numeric_limits<double>::quiet_NaN();
    std::string signal;
};

bool parseRawRow(const std::vector<std::string>& row,
                 const ColumnMap& columns,
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
    if (columns.has("hardwareclockdiscontinuitycount") &&
        !parseInt(columns.value(row, "hardwareclockdiscontinuitycount"),
                  output.hardware_clock_discontinuity_count, true)) {
        error = "invalid HardwareClockDiscontinuityCount";
        return false;
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
    if (columns.has("rawpseudorangemeters") &&
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

bool rawPseudorange(const RawRow& row,
                    std::int64_t base_full_bias_nanos,
                    double& pseudorange_m) {
    const long double clock_ns =
        static_cast<long double>(row.time_nanos) -
        static_cast<long double>(base_full_bias_nanos);
    const long double gps_seconds = clock_ns / kNanosecondsPerSecond;
    const long double week_value = std::floor(gps_seconds / kSecondsPerWeek);
    long double tow_rx = gps_seconds - week_value * kSecondsPerWeek -
                         static_cast<long double>(row.bias_nanos) /
                             kNanosecondsPerSecond;
    long double tow_tx = static_cast<long double>(row.received_sv_time_nanos) /
                         kNanosecondsPerSecond;
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
    result.diagnostics.timing_formula =
        "week=floor((TimeNanos-baseFullBiasNanos)/1e9/604800); "
        "tow_rx=(TimeNanos-baseFullBiasNanos-week*604800e9)/1e9-BiasNanos/1e9";
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
        if (!parseRawRow(row, columns, raw, error)) {
            error = "raw Android GNSS row " +
                    std::to_string(result.diagnostics.input_rows + 1U) +
                    ": " + error;
            return false;
        }
        // gnsslog2obs.m removes zero receiver times and transmit times below
        // 1e10 ns before constructing an epoch (its lines 19-22 and 83-89).
        // Treat these as invalid raw measurements, not as solver failures;
        // retaining them would create a false week-boundary pseudorange.
        if (raw.time_nanos == 0 || raw.received_sv_time_nanos < 10'000'000'000LL) {
            ++result.diagnostics.skipped_invalid_timing_rows;
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
        if (!parseSignal(raw.signal, raw.constellation, raw.carrier_frequency_hz,
                         config.include_galileo_e1, signal, system)) {
            ++result.diagnostics.skipped_unsupported_signal_rows;
            continue;
        }
        const int prn_max = system == GNSSSystem::GPS ? 32 : 36;
        if (raw.svid < 1 || raw.svid > prn_max) {
            error = "supported raw Android row has an invalid SVID";
            return false;
        }
        double pseudorange_m = 0.0;
        if (!rawPseudorange(raw, base_full_bias_nanos, pseudorange_m)) {
            error = "raw Android row produced a non-positive pseudorange";
            return false;
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
        const double wavelength =
            constants::SPEED_OF_LIGHT / raw.carrier_frequency_hz;
        Observation observation(SatelliteId(system, static_cast<uint8_t>(raw.svid)),
                                 signal);
        observation.pseudorange = pseudorange_m;
        observation.has_pseudorange = true;
        observation.pseudorange_observation_type = "C1C";
        observation.snr = std::isfinite(raw.cn0_dbhz) ? raw.cn0_dbhz : 0.0;
        observation.valid = true;
        observation.signal_strength =
            static_cast<int>(std::clamp(std::round(observation.snr / 6.0), 0.0, 9.0));
        observation.doppler = -raw.pseudorange_rate_mps / wavelength;
        observation.has_doppler = std::isfinite(observation.doppler);
        if (observation.has_doppler) ++result.diagnostics.doppler_rows;
        if (std::isfinite(raw.adr_m) && std::abs(raw.adr_m) < 1.0e9 &&
            (raw.adr_state & 0x01) != 0) {
            double carrier_m = raw.adr_m;
            if (supportedDeviceAdrSign(config.device_model)) carrier_m = -carrier_m;
            observation.carrier_phase = carrier_m / wavelength;
            observation.has_carrier_phase = std::isfinite(observation.carrier_phase);
            observation.carrier_phase_observation_type = "L1C";
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
        if (system == GNSSSystem::GPS) {
            ++result.diagnostics.gps_rows;
        } else {
            ++result.diagnostics.galileo_e1_rows;
        }
    }
    if (have_epoch && !appendEpoch(accumulator, result, error)) return false;
    if (result.observations.epochs.empty()) {
        error = "raw Android GNSS CSV has no supported finite observations";
        return false;
    }
    return true;
}

}  // namespace libgnss::io
