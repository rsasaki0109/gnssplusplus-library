#include <libgnss++/io/imu.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <unordered_map>

namespace libgnss {

namespace {

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kStandardGravityMps2 = 9.80665;
constexpr double kUnixSecondsAtGpsEpoch = 315964800.0;
constexpr double kSecondsPerGpsWeek = 604800.0;

// Direct port of analyze_ppc_imu_coverage.py's normalize_header(): strip to
// lowercase alphanumeric characters only, so "GPS TOW (s)", " Ang Rate X (deg/s)"
// (irregular double spaces included) and "gpstows" all normalize identically.
std::string normalizeHeader(const std::string& name) {
    std::string trimmed;
    // Strip leading/trailing whitespace first (matches Python's str.strip()).
    size_t begin = 0;
    size_t end = name.size();
    while (begin < end && std::isspace(static_cast<unsigned char>(name[begin]))) ++begin;
    while (end > begin && std::isspace(static_cast<unsigned char>(name[end - 1]))) --end;
    trimmed = name.substr(begin, end - begin);

    std::string normalized;
    normalized.reserve(trimmed.size());
    for (char ch : trimmed) {
        const unsigned char uch = static_cast<unsigned char>(ch);
        if (std::isalnum(uch)) {
            normalized.push_back(static_cast<char>(std::tolower(uch)));
        }
    }
    return normalized;
}

std::vector<std::string> splitCsvLine(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {
        fields.push_back(field);
    }
    // std::getline drops a trailing empty field after the last comma; CSV
    // rows here never end with a trailing comma, so no fix-up needed.
    return fields;
}

using HeaderLookup = std::unordered_map<std::string, size_t>;

// find_column(): return the resolved raw header text for the first matching
// candidate, or empty string if none of the candidates were found.
std::string findColumn(const HeaderLookup& lookup,
                       const std::vector<std::string>& raw_headers,
                       const std::vector<std::string>& candidates,
                       size_t& out_index) {
    for (const auto& candidate : candidates) {
        auto it = lookup.find(candidate);
        if (it != lookup.end()) {
            out_index = it->second;
            return raw_headers[it->second];
        }
    }
    return {};
}

double parseDouble(const std::string& text, bool& ok) {
    try {
        size_t consumed = 0;
        double value = std::stod(text, &consumed);
        ok = true;
        return value;
    } catch (...) {
        ok = false;
        return 0.0;
    }
}

std::string trimWhitespace(const std::string& text) {
    size_t begin = 0;
    size_t end = text.size();
    while (begin < end && std::isspace(static_cast<unsigned char>(text[begin]))) ++begin;
    while (end > begin && std::isspace(static_cast<unsigned char>(text[end - 1]))) --end;
    return text.substr(begin, end - begin);
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

bool parseInt64(const std::string& text, std::int64_t& value) {
    const std::string trimmed = trimWhitespace(text);
    if (trimmed.empty()) return false;
    try {
        size_t consumed = 0;
        const long long parsed = std::stoll(trimmed, &consumed);
        if (consumed != trimmed.size()) return false;
        value = static_cast<std::int64_t>(parsed);
        return true;
    } catch (...) {
        return false;
    }
}

struct AndroidRawImuSample {
    std::int64_t utc_time_ms = 0;
    // The pairing clock is either the original monotonic Android clock or,
    // only for the explicit raw UTC fallback, UTC milliseconds converted to
    // integer nanoseconds.  Never fabricate elapsedRealtimeNanos: the
    // public elapsed field remains -1 in the latter mode.
    std::int64_t elapsed_time_ns = -1;
    std::int64_t pairing_time_ns = -1;
    Eigen::Vector3d measurement = Eigen::Vector3d::Zero();
    Eigen::Vector3d bias = Eigen::Vector3d::Zero();
};

GNSSTime androidUtcToGpsTime(double utc_time_ms, double leap_seconds) {
    const double gps_seconds = utc_time_ms / 1000.0 -
                               kUnixSecondsAtGpsEpoch + leap_seconds;
    const int week = static_cast<int>(std::floor(gps_seconds / kSecondsPerGpsWeek));
    return GNSSTime(week, gps_seconds - static_cast<double>(week) * kSecondsPerGpsWeek);
}

GNSSTime gpsNanosToGpsTime(long double gps_time_nanos) {
    if (!std::isfinite(static_cast<double>(gps_time_nanos))) {
        return GNSSTime();
    }
    const long double gps_seconds = gps_time_nanos / 1.0e9L;
    const long double week_value = std::floor(gps_seconds / kSecondsPerGpsWeek);
    if (!std::isfinite(static_cast<double>(week_value)) ||
        week_value < static_cast<long double>(std::numeric_limits<int>::min()) ||
        week_value > static_cast<long double>(std::numeric_limits<int>::max())) {
        return GNSSTime();
    }
    const int week = static_cast<int>(week_value);
    return GNSSTime(week, static_cast<double>(
        gps_seconds - static_cast<long double>(week) * kSecondsPerGpsWeek));
}

bool validateAndroidGnssTimeAnchors(
    const std::vector<AndroidGnssTimeAnchor>& anchors,
    std::string& error) {
    if (anchors.size() < 2U) {
        error = "GNSS elapsed-time anchor requires at least two unique raw epochs";
        return false;
    }
    for (std::size_t i = 0; i < anchors.size(); ++i) {
        if (anchors[i].utc_time_ms < 0 || anchors[i].elapsed_realtime_nanos < 0) {
            error = "GNSS elapsed-time anchor contains a negative timestamp";
            return false;
        }
        if (i > 0U &&
            (anchors[i - 1U].utc_time_ms >= anchors[i].utc_time_ms ||
             anchors[i - 1U].elapsed_realtime_nanos >= anchors[i].elapsed_realtime_nanos)) {
            error = "GNSS elapsed-time anchors must be strictly increasing in UTC and elapsed time";
            return false;
        }
    }
    return true;
}

double interpolateAndroidGnssUtc(
    const std::vector<AndroidGnssTimeAnchor>& anchors,
    std::int64_t elapsed_realtime_nanos,
    bool& exact,
    bool& extrapolated) {
    const auto upper = std::lower_bound(
        anchors.begin(), anchors.end(), elapsed_realtime_nanos,
        [](const AndroidGnssTimeAnchor& anchor, std::int64_t elapsed) {
            return anchor.elapsed_realtime_nanos < elapsed;
        });
    exact = upper != anchors.end() &&
            upper->elapsed_realtime_nanos == elapsed_realtime_nanos;
    extrapolated = upper == anchors.begin() || upper == anchors.end();
    std::size_t lower_index = 0U;
    std::size_t upper_index = 1U;
    if (exact) {
        return static_cast<double>(upper->utc_time_ms);
    }
    if (upper == anchors.begin()) {
        lower_index = 0U;
        upper_index = 1U;
    } else if (upper == anchors.end()) {
        lower_index = anchors.size() - 2U;
        upper_index = anchors.size() - 1U;
    } else {
        lower_index = static_cast<std::size_t>(upper - anchors.begin()) - 1U;
        upper_index = lower_index + 1U;
    }
    const long double x0 = static_cast<long double>(anchors[lower_index].elapsed_realtime_nanos);
    const long double x1 = static_cast<long double>(anchors[upper_index].elapsed_realtime_nanos);
    const long double y0 = static_cast<long double>(anchors[lower_index].utc_time_ms);
    const long double y1 = static_cast<long double>(anchors[upper_index].utc_time_ms);
    const long double query = static_cast<long double>(elapsed_realtime_nanos);
    const long double mapped = y0 + (query - x0) * (y1 - y0) / (x1 - x0);
    return static_cast<double>(mapped);
}

}  // namespace

std::vector<ImuSample> ImuSeries::getSamples(const GNSSTime& start, const GNSSTime& end) const {
    std::vector<ImuSample> result;
    for (const auto& sample : samples) {
        if (sample.time >= start && sample.time <= end) {
            result.push_back(sample);
        }
    }
    return result;
}

void ImuSeries::sortByTime() {
    std::stable_sort(samples.begin(), samples.end(),
                     [](const ImuSample& a, const ImuSample& b) { return a.time < b.time; });
}

void ImuSeries::shiftTime(double offset_s) {
    if (offset_s == 0.0) {
        return;
    }
    for (auto& sample : samples) {
        // GNSSTime::operator+ only normalizes tow overflow and operator-
        // only underflow, so route by sign.
        if (offset_s > 0.0) {
            sample.time = sample.time + offset_s;
        } else {
            sample.time = sample.time - (-offset_s);
        }
    }
}

ImuCsvLoadResult loadImuCsv(const std::string& path, ImuSeries& out) {
    ImuCsvLoadResult result;
    out.samples.clear();

    if (hasMatExtension(path)) {
        result.error = "MATLAB .mat inputs are forbidden by the raw/native IMU contract";
        return result;
    }
    std::ifstream file(path);
    if (!file.is_open()) {
        result.error = "could not open IMU CSV file: " + path;
        return result;
    }

    std::string header_line;
    if (!std::getline(file, header_line)) {
        result.error = "IMU CSV file is empty: " + path;
        return result;
    }
    // Strip a possible trailing '\r' (CRLF line endings).
    if (!header_line.empty() && header_line.back() == '\r') {
        header_line.pop_back();
    }

    const std::vector<std::string> raw_headers = splitCsvLine(header_line);
    HeaderLookup lookup;
    for (size_t i = 0; i < raw_headers.size(); ++i) {
        lookup.emplace(normalizeHeader(raw_headers[i]), i);
    }

    static const std::vector<std::string> kTimeCandidates = {
        "gpstows", "gpstow", "gpssecondsofweek", "gpssecondsweek",
        "tow", "tows", "time", "times", "timestamp"};
    static const std::vector<std::string> kWeekCandidates = {"gpsweek", "week"};
    static const std::array<std::vector<std::string>, 3> kAccelCandidates = {
        std::vector<std::string>{"accxms2", "accx", "accelx", "accelerationx", "ax"},
        std::vector<std::string>{"accyms2", "accy", "accely", "accelerationy", "ay"},
        std::vector<std::string>{"acczms2", "accz", "accelz", "accelerationz", "az"},
    };
    static const std::array<std::vector<std::string>, 3> kGyroCandidates = {
        std::vector<std::string>{"angratexdegs", "angratex", "angularratexdegs",
                                 "angularratex", "gyroxdegs", "gyrox", "gyrx", "wx"},
        std::vector<std::string>{"angrateydegs", "angratey", "angularrateydegs",
                                 "angularratey", "gyroydegs", "gyroy", "gyry", "wy"},
        std::vector<std::string>{"angratezdegs", "angratez", "angularratezdegs",
                                 "angularratez", "gyrozdegs", "gyroz", "gyrz", "wz"},
    };

    size_t time_index = 0, week_index = 0;
    std::array<size_t, 3> accel_index{};
    std::array<size_t, 3> gyro_index{};

    result.time_column = findColumn(lookup, raw_headers, kTimeCandidates, time_index);
    if (result.time_column.empty()) {
        result.error = "could not find time column (tried: GPS TOW (s), tow, time, ...)";
        return result;
    }
    result.week_column = findColumn(lookup, raw_headers, kWeekCandidates, week_index);
    if (result.week_column.empty()) {
        result.error = "could not find GPS week column (tried: GPS Week, week)";
        return result;
    }
    static const char* kAxisNames[3] = {"X", "Y", "Z"};
    for (int axis = 0; axis < 3; ++axis) {
        result.accel_columns[axis] =
            findColumn(lookup, raw_headers, kAccelCandidates[axis], accel_index[axis]);
        if (result.accel_columns[axis].empty()) {
            result.error = std::string("could not find accel ") + kAxisNames[axis] +
                            " column (tried: Acc " + kAxisNames[axis] + " (m/s^2), ...)";
            return result;
        }
        result.gyro_columns[axis] =
            findColumn(lookup, raw_headers, kGyroCandidates[axis], gyro_index[axis]);
        if (result.gyro_columns[axis].empty()) {
            result.error = std::string("could not find gyro ") + kAxisNames[axis] +
                            " column (tried: Ang Rate " + kAxisNames[axis] + " (deg/s), ...)";
            return result;
        }
    }

    const size_t required_columns = raw_headers.size();
    std::string line;
    int row_number = 1;  // header was row 0
    while (std::getline(file, line)) {
        ++row_number;
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (line.empty()) {
            continue;
        }
        const std::vector<std::string> fields = splitCsvLine(line);
        if (fields.size() < required_columns) {
            result.error = "row " + std::to_string(row_number) +
                            ": expected " + std::to_string(required_columns) +
                            " columns, got " + std::to_string(fields.size());
            return result;
        }

        bool ok = true;
        ImuSample sample;
        double tow = parseDouble(fields[time_index], ok);
        if (!ok) {
            result.error = "row " + std::to_string(row_number) + ": invalid time value";
            return result;
        }
        double week_value = parseDouble(fields[week_index], ok);
        if (!ok) {
            result.error = "row " + std::to_string(row_number) + ": invalid GPS week value";
            return result;
        }
        sample.time = GNSSTime(static_cast<int>(week_value), tow);

        for (int axis = 0; axis < 3; ++axis) {
            double accel_value = parseDouble(fields[accel_index[axis]], ok);
            if (!ok) {
                result.error = "row " + std::to_string(row_number) + ": invalid accel value";
                return result;
            }
            sample.accel_raw(axis) = accel_value;

            double gyro_value_degps = parseDouble(fields[gyro_index[axis]], ok);
            if (!ok) {
                result.error = "row " + std::to_string(row_number) + ": invalid gyro value";
                return result;
            }
            sample.gyro_raw_radps(axis) = gyro_value_degps * kDegToRad;
        }

        out.samples.push_back(sample);
    }

    result.row_count = static_cast<int>(out.samples.size());
    result.ok = true;
    return result;
}

AndroidGnssTimeAnchorLoadResult loadAndroidGnssTimeAnchors(
    const std::string& path,
    std::vector<AndroidGnssTimeAnchor>& anchors) {
    AndroidGnssTimeAnchorLoadResult result;
    anchors.clear();
    if (hasMatExtension(path)) {
        result.error = "MATLAB .mat inputs are forbidden by the raw/native IMU contract";
        return result;
    }

    std::ifstream file(path);
    if (!file.is_open()) {
        result.error = "could not open raw Android GNSS CSV for time anchors: " + path;
        return result;
    }
    std::string header_line;
    if (!std::getline(file, header_line)) {
        result.error = "raw Android GNSS CSV is empty: " + path;
        return result;
    }
    if (!header_line.empty() && header_line.back() == '\r') header_line.pop_back();
    const std::vector<std::string> raw_headers = splitCsvLine(header_line);
    HeaderLookup lookup;
    for (std::size_t i = 0; i < raw_headers.size(); ++i) {
        const std::string normalized = normalizeHeader(raw_headers[i]);
        if (normalized.empty() || !lookup.emplace(normalized, i).second) {
            result.error = "raw Android GNSS anchor header has duplicate or empty fields";
            return result;
        }
    }
    const auto find_required = [&](const char* name, std::size_t& index) {
        const auto it = lookup.find(name);
        if (it == lookup.end()) return false;
        index = it->second;
        return true;
    };
    std::size_t message_index = 0U;
    std::size_t utc_index = 0U;
    std::size_t elapsed_index = 0U;
    if (!find_required("messagetype", message_index) ||
        !find_required("utctimemillis", utc_index) ||
        !find_required("chipsetelapsedrealtimenanos", elapsed_index)) {
        result.error = "raw Android GNSS anchor CSV requires MessageType, utcTimeMillis, "
                       "and ChipsetElapsedRealtimeNanos";
        return result;
    }

    std::map<std::int64_t, AndroidGnssTimeAnchor> by_utc;
    std::string line;
    std::size_t row_number = 1U;
    while (std::getline(file, line)) {
        ++row_number;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (trimWhitespace(line).empty()) continue;
        ++result.input_rows;
        const std::vector<std::string> fields = splitCsvLine(line);
        if (fields.size() != raw_headers.size()) {
            result.error = "raw Android GNSS anchor row " + std::to_string(row_number) +
                           ": expected " + std::to_string(raw_headers.size()) +
                           " columns, got " + std::to_string(fields.size());
            anchors.clear();
            return result;
        }
        if (trimWhitespace(fields[message_index]) != "Raw") {
            ++result.unsupported_rows;
            continue;
        }
        ++result.raw_rows;
        AndroidGnssTimeAnchor anchor;
        if (!parseInt64(fields[utc_index], anchor.utc_time_ms) ||
            !parseInt64(fields[elapsed_index], anchor.elapsed_realtime_nanos) ||
            anchor.utc_time_ms < 0 || anchor.elapsed_realtime_nanos < 0) {
            result.error = "raw Android GNSS anchor row " + std::to_string(row_number) +
                           ": timestamps must be non-negative integers";
            anchors.clear();
            return result;
        }
        const auto [unused, inserted] = by_utc.emplace(anchor.utc_time_ms, anchor);
        if (!inserted) ++result.duplicate_utc_timestamps;
        (void)unused;
    }

    anchors.reserve(by_utc.size());
    for (const auto& [unused, anchor] : by_utc) {
        anchors.push_back(anchor);
        (void)unused;
    }
    result.unique_anchors = anchors.size();
    std::string validation_error;
    if (!validateAndroidGnssTimeAnchors(anchors, validation_error)) {
        result.error = validation_error;
        anchors.clear();
        return result;
    }
    result.first_utc_time_ms = anchors.front().utc_time_ms;
    result.last_utc_time_ms = anchors.back().utc_time_ms;
    result.first_elapsed_realtime_nanos = anchors.front().elapsed_realtime_nanos;
    result.last_elapsed_realtime_nanos = anchors.back().elapsed_realtime_nanos;
    result.ok = true;
    return result;
}

AndroidGnssUtcGpsMappingLoadResult loadAndroidGnssUtcGpsMapping(
    const std::string& path,
    AndroidGnssUtcGpsMapping& mapping) {
    AndroidGnssUtcGpsMappingLoadResult result;
    mapping = AndroidGnssUtcGpsMapping{};
    if (hasMatExtension(path)) {
        result.error = "MATLAB .mat inputs are forbidden by the raw/native IMU contract";
        return result;
    }

    std::ifstream file(path);
    if (!file.is_open()) {
        result.error = "could not open raw Android GNSS CSV for UTC/GPS mapping: " + path;
        return result;
    }
    std::string header_line;
    if (!std::getline(file, header_line)) {
        result.error = "raw Android GNSS CSV is empty: " + path;
        return result;
    }
    if (!header_line.empty() && header_line.back() == '\r') header_line.pop_back();
    const std::vector<std::string> raw_headers = splitCsvLine(header_line);
    HeaderLookup lookup;
    for (std::size_t i = 0; i < raw_headers.size(); ++i) {
        const std::string normalized = normalizeHeader(raw_headers[i]);
        if (normalized.empty() || !lookup.emplace(normalized, i).second) {
            result.error = "raw Android GNSS UTC/GPS mapping header has duplicate or empty fields";
            return result;
        }
    }
    const auto find_required = [&](const char* name, std::size_t& index) {
        const auto it = lookup.find(name);
        if (it == lookup.end()) return false;
        index = it->second;
        return true;
    };
    std::size_t message_index = 0U;
    std::size_t utc_index = 0U;
    std::size_t time_index = 0U;
    std::size_t full_bias_index = 0U;
    std::size_t bias_index = 0U;
    if (!find_required("messagetype", message_index) ||
        !find_required("utctimemillis", utc_index) ||
        !find_required("timenanos", time_index) ||
        !find_required("fullbiasnanos", full_bias_index) ||
        !find_required("biasnanos", bias_index)) {
        result.error = "raw Android GNSS UTC/GPS mapping requires MessageType, utcTimeMillis, "
                       "TimeNanos, FullBiasNanos, and BiasNanos";
        return result;
    }
    std::size_t discontinuity_index = 0U;
    const bool has_discontinuity_count =
        find_required("hardwareclockdiscontinuitycount", discontinuity_index);

    struct RawAnchor {
        std::int64_t utc_time_ms = 0;
        long double gps_time_nanos = 0.0L;
        int discontinuity_count = 0;
    };
    std::map<std::int64_t, RawAnchor> by_utc;
    std::string line;
    std::size_t row_number = 1U;
    while (std::getline(file, line)) {
        ++row_number;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (trimWhitespace(line).empty()) continue;
        ++result.input_rows;
        const std::vector<std::string> fields = splitCsvLine(line);
        if (fields.size() != raw_headers.size()) {
            result.error = "raw Android GNSS UTC/GPS mapping row " +
                           std::to_string(row_number) + ": expected " +
                           std::to_string(raw_headers.size()) + " columns, got " +
                           std::to_string(fields.size());
            return result;
        }
        if (trimWhitespace(fields[message_index]) != "Raw") {
            ++result.unsupported_rows;
            continue;
        }
        ++result.raw_rows;
        RawAnchor anchor;
        std::int64_t time_nanos = 0;
        std::int64_t full_bias_nanos = 0;
        if (!parseInt64(fields[utc_index], anchor.utc_time_ms) ||
            !parseInt64(fields[time_index], time_nanos) ||
            !parseInt64(fields[full_bias_index], full_bias_nanos) ||
            anchor.utc_time_ms < 0) {
            result.error = "raw Android GNSS UTC/GPS mapping row " +
                           std::to_string(row_number) +
                           ": UTC, TimeNanos, and FullBiasNanos must be valid integers";
            return result;
        }
        bool bias_ok = false;
        const double bias_nanos = parseDouble(fields[bias_index], bias_ok);
        if (!bias_ok || !std::isfinite(bias_nanos)) {
            result.error = "raw Android GNSS UTC/GPS mapping row " +
                           std::to_string(row_number) + ": BiasNanos must be finite";
            return result;
        }
        if (has_discontinuity_count) {
            std::int64_t count = 0;
            if (!parseInt64(fields[discontinuity_index], count) ||
                count < 0 || count > std::numeric_limits<int>::max()) {
                result.error = "raw Android GNSS UTC/GPS mapping row " +
                               std::to_string(row_number) +
                               ": HardwareClockDiscontinuityCount must be a non-negative integer";
                return result;
            }
            anchor.discontinuity_count = static_cast<int>(count);
        }
        anchor.gps_time_nanos = static_cast<long double>(time_nanos) -
                                static_cast<long double>(full_bias_nanos) -
                                static_cast<long double>(bias_nanos);
        if (!std::isfinite(static_cast<double>(anchor.gps_time_nanos))) {
            result.error = "raw Android GNSS UTC/GPS mapping contains a non-finite GPS time";
            return result;
        }
        const auto [unused, inserted] = by_utc.emplace(anchor.utc_time_ms, anchor);
        if (!inserted) ++result.duplicate_utc_timestamps;
        (void)unused;
    }

    if (by_utc.size() < 3U) {
        result.error = "raw Android UTC/GPS mapping requires at least three unique raw epochs";
        return result;
    }
    std::vector<RawAnchor> anchors;
    anchors.reserve(by_utc.size());
    for (const auto& [unused, anchor] : by_utc) {
        anchors.push_back(anchor);
        (void)unused;
    }
    if (has_discontinuity_count) {
        const int expected_count = anchors.front().discontinuity_count;
        for (const RawAnchor& anchor : anchors) {
            if (anchor.discontinuity_count != expected_count) {
                result.error = "raw Android UTC/GPS mapping rejects a hardware-clock "
                               "discontinuity within the anchor span";
                return result;
            }
        }
        mapping.hardware_clock_count_field_present = true;
        mapping.hardware_clock_count_constant = true;
        mapping.hardware_clock_discontinuity_count = expected_count;
    }
    long double max_gap_ms = 0.0L;
    for (std::size_t i = 1U; i < anchors.size(); ++i) {
        if (anchors[i - 1U].utc_time_ms >= anchors[i].utc_time_ms ||
            anchors[i - 1U].gps_time_nanos >= anchors[i].gps_time_nanos) {
            result.error = "raw Android UTC/GPS mapping anchors must be strictly increasing";
            return result;
        }
        max_gap_ms = std::max(max_gap_ms, static_cast<long double>(
            anchors[i].utc_time_ms - anchors[i - 1U].utc_time_ms));
    }
    constexpr long double kMaximumAnchorGapMs = 5000.0L;
    if (max_gap_ms > kMaximumAnchorGapMs) {
        result.error = "raw Android UTC/GPS mapping anchor gap exceeds 5000 ms";
        return result;
    }

    const long double utc0 = static_cast<long double>(anchors.front().utc_time_ms);
    const long double gps0 = anchors.front().gps_time_nanos;
    long double sum_xx = 0.0L;
    long double sum_xy = 0.0L;
    for (const RawAnchor& anchor : anchors) {
        const long double x = static_cast<long double>(anchor.utc_time_ms) - utc0;
        const long double y = anchor.gps_time_nanos - gps0;
        sum_xx += x * x;
        sum_xy += x * y;
    }
    if (!(sum_xx > 0.0L) || !std::isfinite(static_cast<double>(sum_xx)) ||
        !std::isfinite(static_cast<double>(sum_xy))) {
        result.error = "raw Android UTC/GPS mapping has no finite time span";
        return result;
    }
    const long double slope = sum_xy / sum_xx;
    long double sum_residual = 0.0L;
    for (const RawAnchor& anchor : anchors) {
        const long double x = static_cast<long double>(anchor.utc_time_ms) - utc0;
        const long double y = anchor.gps_time_nanos - gps0;
        sum_residual += y - slope * x;
    }
    const long double intercept = sum_residual / static_cast<long double>(anchors.size());
    std::vector<double> residuals_ms;
    residuals_ms.reserve(anchors.size());
    long double max_residual_ms = 0.0L;
    for (const RawAnchor& anchor : anchors) {
        const long double x = static_cast<long double>(anchor.utc_time_ms) - utc0;
        const long double y = anchor.gps_time_nanos - gps0;
        const long double residual_ms = (intercept + slope * x - y) / 1.0e6L;
        if (!std::isfinite(static_cast<double>(residual_ms))) {
            result.error = "raw Android UTC/GPS mapping fit is non-finite";
            return result;
        }
        const double abs_residual_ms = static_cast<double>(std::abs(residual_ms));
        residuals_ms.push_back(abs_residual_ms);
        max_residual_ms = std::max(max_residual_ms, static_cast<long double>(abs_residual_ms));
    }
    std::sort(residuals_ms.begin(), residuals_ms.end());
    const double p95_index = (static_cast<double>(residuals_ms.size()) - 1.0) * 0.95;
    const std::size_t p95_lower = static_cast<std::size_t>(std::floor(p95_index));
    const std::size_t p95_upper = std::min(p95_lower + 1U, residuals_ms.size() - 1U);
    const double p95_alpha = p95_index - static_cast<double>(p95_lower);
    const double p95_residual_ms = residuals_ms[p95_lower] +
                                   p95_alpha * (residuals_ms[p95_upper] -
                                                residuals_ms[p95_lower]);
    const long double drift_ppm = (slope / 1.0e6L - 1.0L) * 1.0e6L;
    constexpr long double kMaximumDriftPpm = 1000.0L;
    constexpr long double kMaximumFitResidualMs = 2.0L;
    if (!std::isfinite(static_cast<double>(slope)) ||
        std::abs(drift_ppm) > kMaximumDriftPpm ||
        max_residual_ms > kMaximumFitResidualMs) {
        result.error = "raw Android UTC/GPS mapping violates fixed drift/residual bounds";
        return result;
    }
    const long double ref_gps_rounded = std::round(gps0);
    if (ref_gps_rounded < static_cast<long double>(std::numeric_limits<std::int64_t>::min()) ||
        ref_gps_rounded > static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
        result.error = "raw Android UTC/GPS mapping reference GPS time is out of range";
        return result;
    }
    mapping.valid = true;
    mapping.reference_utc_time_ms = anchors.front().utc_time_ms;
    mapping.reference_gps_time_nanos = static_cast<std::int64_t>(ref_gps_rounded);
    mapping.intercept_offset_nanos = static_cast<double>(
        (gps0 - ref_gps_rounded) + intercept);
    mapping.slope_nanos_per_ms = static_cast<double>(slope);
    mapping.drift_ppm = static_cast<double>(drift_ppm);
    mapping.maximum_fit_residual_ms = static_cast<double>(max_residual_ms);
    mapping.p95_fit_residual_ms = p95_residual_ms;
    mapping.maximum_anchor_gap_ms = static_cast<double>(max_gap_ms);
    mapping.unique_anchors = anchors.size();
    result.mapping = mapping;
    result.ok = true;
    return result;
}

AndroidImuCsvLoadResult loadAndroidImuCsv(
    const std::string& path,
    ImuSeries& out,
    const AndroidImuCsvConfig& config,
    const std::vector<AndroidGnssTimeAnchor>& gnss_time_anchors,
    const AndroidGnssUtcGpsMapping* utc_gps_mapping) {
    AndroidImuCsvLoadResult result;
    out.samples.clear();

    if (hasMatExtension(path)) {
        result.error = "MATLAB .mat inputs are forbidden by the raw/native IMU contract";
        return result;
    }
    if (!std::isfinite(config.gps_utc_leap_seconds) ||
        config.gps_utc_leap_seconds < 0.0 || config.gps_utc_leap_seconds > 60.0) {
        result.error = "invalid GPS-UTC leap-second configuration";
        return result;
    }
    if (!std::isfinite(config.maximum_accel_pair_offset_s) ||
        config.maximum_accel_pair_offset_s <= 0.0) {
        result.error = "invalid Android IMU pairing bound";
        return result;
    }
    if (!std::isfinite(config.imu_sync_coefficient) ||
        config.imu_sync_coefficient <= 0.0 || config.imu_sync_coefficient > 1.0) {
        result.error = "invalid Android IMU synchronization coefficient";
        return result;
    }
    if (config.require_gnss_elapsed_anchor &&
        config.imu_sync_coefficient != 0.5) {
        result.error = "raw Android IMU anchor contract fixes sync coefficient at 0.5";
        return result;
    }
    const bool use_gnss_anchor = !gnss_time_anchors.empty();
    const bool use_utc_wall_clock_fallback =
        !use_gnss_anchor && config.allow_utc_wall_clock_fallback &&
        utc_gps_mapping != nullptr;
    if (use_gnss_anchor) {
        std::string validation_error;
        if (!validateAndroidGnssTimeAnchors(gnss_time_anchors, validation_error)) {
            result.error = validation_error;
            return result;
        }
    } else if (use_utc_wall_clock_fallback) {
        if (!utc_gps_mapping->valid || utc_gps_mapping->unique_anchors < 3U ||
            !std::isfinite(utc_gps_mapping->slope_nanos_per_ms) ||
            !std::isfinite(utc_gps_mapping->drift_ppm) ||
            !std::isfinite(utc_gps_mapping->maximum_fit_residual_ms) ||
            utc_gps_mapping->maximum_fit_residual_ms > 2.0 ||
            std::abs(utc_gps_mapping->drift_ppm) > 1000.0) {
            result.error = "raw Android UTC wall-clock fallback requires a validated GNSS mapping";
            return result;
        }
    } else if (config.require_gnss_elapsed_anchor) {
        result.error = "raw Android IMU inference requires GNSS elapsed-time anchors";
        return result;
    }
    result.gnss_anchor_points = gnss_time_anchors.size();
    result.imu_sync_coefficient = config.imu_sync_coefficient;
    if (use_utc_wall_clock_fallback) {
        result.utc_wall_clock_fallback_applied = true;
        result.utc_mapping_anchors = utc_gps_mapping->unique_anchors;
        result.utc_mapping_slope_ns_per_ms = utc_gps_mapping->slope_nanos_per_ms;
        result.utc_mapping_drift_ppm = utc_gps_mapping->drift_ppm;
        result.utc_mapping_max_fit_residual_ms =
            utc_gps_mapping->maximum_fit_residual_ms;
        result.utc_mapping_max_anchor_gap_ms =
            utc_gps_mapping->maximum_anchor_gap_ms;
    }

    std::ifstream file(path);
    if (!file.is_open()) {
        result.error = "could not open Android IMU CSV file: " + path;
        return result;
    }

    std::string header_line;
    if (!std::getline(file, header_line)) {
        result.error = "Android IMU CSV file is empty: " + path;
        return result;
    }
    if (!header_line.empty() && header_line.back() == '\r') header_line.pop_back();
    const std::vector<std::string> raw_headers = splitCsvLine(header_line);
    HeaderLookup lookup;
    for (size_t i = 0; i < raw_headers.size(); ++i) {
        const std::string normalized = normalizeHeader(raw_headers[i]);
        if (normalized.empty() || !lookup.emplace(normalized, i).second) {
            result.error = "Android IMU CSV has duplicate or empty header fields";
            return result;
        }
    }

    const auto requiredColumn = [&](const char* name, size_t& index) -> bool {
        const auto it = lookup.find(name);
        if (it == lookup.end()) return false;
        index = it->second;
        return true;
    };
    size_t message_index = 0;
    size_t utc_index = 0;
    size_t elapsed_index = 0;
    std::array<size_t, 3> measurement_index{};
    std::array<size_t, 3> bias_index{};
    if (!requiredColumn("messagetype", message_index) ||
        !requiredColumn("utctimemillis", utc_index) ||
        !requiredColumn("elapsedrealtimenanos", elapsed_index)) {
        result.error = "Android IMU CSV requires MessageType, utcTimeMillis, and "
                       "elapsedRealtimeNanos";
        return result;
    }
    const char* const measurement_names[3] = {
        "measurementx", "measurementy", "measurementz"};
    const char* const bias_names[3] = {"biasx", "biasy", "biasz"};
    for (int axis = 0; axis < 3; ++axis) {
        if (!requiredColumn(measurement_names[axis], measurement_index[axis]) ||
            !requiredColumn(bias_names[axis], bias_index[axis])) {
            result.error = "Android IMU CSV requires MeasurementX/Y/Z and BiasX/Y/Z";
            return result;
        }
    }

    std::map<std::int64_t, AndroidRawImuSample> accel_by_utc;
    std::map<std::int64_t, AndroidRawImuSample> gyro_by_utc;
    // -1 = not seen, 0 = monotonic elapsedRealtimeNanos, 1 = explicit UTC
    // wall-clock fallback.  Mixing clock domains would make pairing and
    // synchronization physically meaningless, so reject it.
    int timestamp_mode = -1;
    std::string line;
    std::size_t row_number = 1;
    while (std::getline(file, line)) {
        ++row_number;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (trimWhitespace(line).empty()) continue;
        ++result.total_rows;
        const std::vector<std::string> fields = splitCsvLine(line);
        if (fields.size() != raw_headers.size()) {
            result.error = "Android IMU row " + std::to_string(row_number) +
                           ": expected " + std::to_string(raw_headers.size()) +
                           " columns, got " + std::to_string(fields.size());
            out.samples.clear();
            return result;
        }
        const std::string message_type = trimWhitespace(fields[message_index]);
        const bool is_accel = message_type == "UncalAccel";
        const bool is_gyro = message_type == "UncalGyro";
        if (!is_accel && !is_gyro) {
            ++result.unsupported_rows;
            continue;
        }

        AndroidRawImuSample sample;
        if (!parseInt64(fields[utc_index], sample.utc_time_ms) ||
            sample.utc_time_ms < 0) {
            result.error = "Android IMU row " + std::to_string(row_number) +
                           ": invalid integer timestamp";
            out.samples.clear();
            return result;
        }
        const std::string elapsed_text = trimWhitespace(fields[elapsed_index]);
        if (elapsed_text.empty()) {
            if (!use_utc_wall_clock_fallback) {
                result.error = "Android IMU row " + std::to_string(row_number) +
                               ": elapsedRealtimeNanos is empty and UTC wall-clock "
                               "fallback is not explicitly enabled";
                out.samples.clear();
                return result;
            }
            constexpr std::int64_t kMillisToNanos = 1000000;
            if (sample.utc_time_ms > std::numeric_limits<std::int64_t>::max() /
                                        kMillisToNanos) {
                result.error = "Android IMU row " + std::to_string(row_number) +
                               ": UTC timestamp overflows pairing clock";
                out.samples.clear();
                return result;
            }
            sample.elapsed_time_ns = -1;
            sample.pairing_time_ns = sample.utc_time_ms * kMillisToNanos;
            if (timestamp_mode == 0) {
                result.error = "Android IMU mixes elapsedRealtimeNanos and UTC wall-clock rows";
                out.samples.clear();
                return result;
            }
            timestamp_mode = 1;
        } else {
            if (!parseInt64(elapsed_text, sample.elapsed_time_ns) ||
                sample.elapsed_time_ns < 0) {
                result.error = "Android IMU row " + std::to_string(row_number) +
                               ": invalid integer timestamp";
                out.samples.clear();
                return result;
            }
            sample.pairing_time_ns = sample.elapsed_time_ns;
            if (timestamp_mode == 1) {
                result.error = "Android IMU mixes UTC wall-clock and elapsedRealtimeNanos rows";
                out.samples.clear();
                return result;
            }
            timestamp_mode = 0;
        }
        bool ok = true;
        for (int axis = 0; axis < 3; ++axis) {
            sample.measurement(axis) = parseDouble(fields[measurement_index[axis]], ok);
            if (!ok || !std::isfinite(sample.measurement(axis))) {
                result.error = "Android IMU row " + std::to_string(row_number) +
                               ": non-finite measurement";
                out.samples.clear();
                return result;
            }
            sample.bias(axis) = parseDouble(fields[bias_index[axis]], ok);
            if (!ok || !std::isfinite(sample.bias(axis))) {
                result.error = "Android IMU row " + std::to_string(row_number) +
                               ": non-finite bias";
                out.samples.clear();
                return result;
            }
        }

        auto& stream = is_accel ? accel_by_utc : gyro_by_utc;
        auto [it, inserted] = stream.emplace(sample.utc_time_ms, sample);
        if (!inserted) {
            if (is_accel) {
                ++result.duplicate_accel_timestamps;
            } else {
                ++result.duplicate_gyro_timestamps;
            }
            // Match deviceimu2imu.m's unique(utcTimeMillis) contract: retain
            // the first source row, then sort by the unique timestamp.
            (void)it;
        }
    }

    result.accel_rows = accel_by_utc.size();
    result.gyro_rows = gyro_by_utc.size();
    if (accel_by_utc.empty() || gyro_by_utc.empty()) {
        result.error = "Android IMU CSV must contain finite UncalAccel and UncalGyro rows";
        return result;
    }

    std::vector<AndroidRawImuSample> accel;
    std::vector<AndroidRawImuSample> gyro;
    accel.reserve(accel_by_utc.size());
    gyro.reserve(gyro_by_utc.size());
    for (const auto& [unused, sample] : accel_by_utc) {
        (void)unused;
        accel.push_back(sample);
    }
    for (const auto& [unused, sample] : gyro_by_utc) {
        (void)unused;
        gyro.push_back(sample);
    }
    const auto elapsed_order = [](const AndroidRawImuSample& lhs,
                                  const AndroidRawImuSample& rhs) {
        if (lhs.pairing_time_ns != rhs.pairing_time_ns) {
            return lhs.pairing_time_ns < rhs.pairing_time_ns;
        }
        return lhs.utc_time_ms < rhs.utc_time_ms;
    };
    std::sort(accel.begin(), accel.end(), elapsed_order);
    std::sort(gyro.begin(), gyro.end(), elapsed_order);
    for (size_t i = 1; i < accel.size(); ++i) {
        if (accel[i - 1].pairing_time_ns >= accel[i].pairing_time_ns) {
            result.error = "Android accelerometer pairing timestamps are not strictly increasing";
            return result;
        }
    }
    for (size_t i = 1; i < gyro.size(); ++i) {
        if (gyro[i - 1].pairing_time_ns >= gyro[i].pairing_time_ns) {
            result.error = "Android gyroscope pairing timestamps are not strictly increasing";
            return result;
        }
    }

    std::vector<double> pair_offsets_ms;
    pair_offsets_ms.reserve(gyro.size());
    const std::int64_t maximum_offset_ns = static_cast<std::int64_t>(std::llround(
        config.maximum_accel_pair_offset_s * 1.0e9));
    const auto accel_by_elapsed = [](const AndroidRawImuSample& sample) {
        return sample.pairing_time_ns;
    };
    for (const auto& gyro_sample : gyro) {
        const auto upper = std::lower_bound(
            accel.begin(), accel.end(), gyro_sample.pairing_time_ns,
            [&](const AndroidRawImuSample& sample, std::int64_t elapsed) {
                return accel_by_elapsed(sample) < elapsed;
            });
        const AndroidRawImuSample* nearest = nullptr;
        if (upper != accel.end()) nearest = &*upper;
        if (upper != accel.begin()) {
            const auto previous = upper - 1;
            if (nearest == nullptr ||
                std::llabs(previous->pairing_time_ns - gyro_sample.pairing_time_ns) <=
                    std::llabs(nearest->pairing_time_ns - gyro_sample.pairing_time_ns)) {
                nearest = &*previous;
            }
        }
        if (nearest == nullptr ||
            std::llabs(nearest->pairing_time_ns - gyro_sample.pairing_time_ns) >
                maximum_offset_ns) {
            ++result.omitted_rows;
            continue;
        }

        Eigen::Vector3d acceleration;
        if (upper != accel.end() &&
            upper->pairing_time_ns == gyro_sample.pairing_time_ns) {
            acceleration = upper->measurement;
            ++result.exact_elapsed_matches;
        } else if (upper != accel.end() && upper != accel.begin() &&
                   (upper - 1)->pairing_time_ns < gyro_sample.pairing_time_ns &&
                   upper->pairing_time_ns > gyro_sample.pairing_time_ns) {
            const auto& lower_sample = *(upper - 1);
            const double denominator = static_cast<double>(
                upper->pairing_time_ns - lower_sample.pairing_time_ns);
            const double alpha = static_cast<double>(
                gyro_sample.pairing_time_ns - lower_sample.pairing_time_ns) / denominator;
            acceleration = (1.0 - alpha) * lower_sample.measurement +
                           alpha * upper->measurement;
            ++result.interpolated_rows;
        } else {
            if (!config.allow_endpoint_nearest) {
                ++result.omitted_rows;
                continue;
            }
            acceleration = nearest->measurement;
            ++result.endpoint_nearest_rows;
        }
        if (!acceleration.allFinite()) {
            result.error = "Android IMU acceleration alignment produced a non-finite sample";
            out.samples.clear();
            return result;
        }

        ImuSample output_sample;
        bool anchor_exact = false;
        bool anchor_extrapolated = false;
        double synchronized_utc_ms = static_cast<double>(gyro_sample.utc_time_ms);
        GNSSTime synchronized_gps_time;
        if (use_gnss_anchor) {
            synchronized_utc_ms = interpolateAndroidGnssUtc(
                gnss_time_anchors, gyro_sample.pairing_time_ns,
                anchor_exact, anchor_extrapolated);
            if (!std::isfinite(synchronized_utc_ms)) {
                result.error = "Android IMU GNSS-anchor interpolation produced a non-finite UTC time";
                out.samples.clear();
                return result;
            }
            if (anchor_exact) {
                ++result.gnss_anchor_exact_rows;
            } else if (anchor_extrapolated) {
                ++result.gnss_anchor_extrapolated_rows;
            } else {
                ++result.gnss_anchor_interpolated_rows;
            }
            synchronized_gps_time = androidUtcToGpsTime(
                synchronized_utc_ms, config.gps_utc_leap_seconds);
        } else if (use_utc_wall_clock_fallback) {
            const long double mapped_gps_nanos =
                utc_gps_mapping->gpsNanosAtUtc(gyro_sample.utc_time_ms);
            synchronized_gps_time = gpsNanosToGpsTime(mapped_gps_nanos);
        } else {
            synchronized_gps_time = androidUtcToGpsTime(
                synchronized_utc_ms, config.gps_utc_leap_seconds);
        }
        output_sample.time = synchronized_gps_time;
        output_sample.elapsed_realtime_nanos = gyro_sample.elapsed_time_ns;
        output_sample.accel_raw = acceleration;
        // Android UncalGyro is already rad/s.  Bias is intentionally not
        // folded into the measurement: the upstream deviceimu2imu.m path
        // exports xyz and leaves bias for the estimator state/contract.
        output_sample.gyro_raw_radps = gyro_sample.measurement;
        if (!std::isfinite(output_sample.time.tow) ||
            !output_sample.accel_raw.allFinite() ||
            !output_sample.gyro_raw_radps.allFinite()) {
            result.error = "Android IMU conversion produced a non-finite GPST sample";
            out.samples.clear();
            return result;
        }
        out.samples.push_back(output_sample);
        ++result.paired_rows;
        pair_offsets_ms.push_back(
            static_cast<double>(std::llabs(nearest->pairing_time_ns -
                                           gyro_sample.pairing_time_ns)) /
            1.0e6);
    }

    if (out.samples.empty()) {
        result.error = "Android IMU streams have no pair within the fixed elapsed-time bound";
        return result;
    }
    std::sort(out.samples.begin(), out.samples.end(),
              [](const ImuSample& lhs, const ImuSample& rhs) {
                  return lhs.time < rhs.time;
              });
    for (size_t i = 1; i < out.samples.size(); ++i) {
        if (out.samples[i - 1].time >= out.samples[i].time) {
            result.error = "Android gyro UTC timestamps are not strictly increasing";
            out.samples.clear();
            return result;
        }
    }

    if (use_gnss_anchor && out.samples.size() < 2U) {
        result.error = "GNSS-anchor synchronized IMU stream needs at least two gyro samples";
        out.samples.clear();
        return result;
    }
    if (out.samples.size() >= 2U) {
        result.first_dt_s = out.samples[1].time - out.samples[0].time;
        result.last_dt_s = out.samples.back().time - out.samples[out.samples.size() - 2U].time;
        result.dt_tail_repeated = std::isfinite(result.first_dt_s) &&
                                   std::isfinite(result.last_dt_s) &&
                                   result.last_dt_s > 0.0;
    }
    if (!out.samples.empty()) {
        const auto to_utc_ms = [&](const GNSSTime& sample_time) {
            return (kUnixSecondsAtGpsEpoch +
                    static_cast<double>(sample_time.week) * kSecondsPerGpsWeek +
                    sample_time.tow - config.gps_utc_leap_seconds) * 1000.0;
        };
        result.first_mapped_utc_time_ms = to_utc_ms(out.samples.front().time);
        result.last_mapped_utc_time_ms = to_utc_ms(out.samples.back().time);
    }
    result.gnss_elapsed_anchor_applied = use_gnss_anchor;

    result.first_gyro_utc_ms = gyro.front().utc_time_ms;
    result.last_gyro_utc_ms = gyro.back().utc_time_ms;
    result.first_gyro_elapsed_ns = gyro.front().elapsed_time_ns;
    result.last_gyro_elapsed_ns = gyro.back().elapsed_time_ns;
    result.elapsed_clock_preserved = !use_utc_wall_clock_fallback;
    if (!pair_offsets_ms.empty()) {
        std::sort(pair_offsets_ms.begin(), pair_offsets_ms.end());
        const size_t middle = pair_offsets_ms.size() / 2U;
        result.median_abs_pair_offset_ms =
            pair_offsets_ms.size() % 2U == 0U
                ? 0.5 * (pair_offsets_ms[middle - 1U] + pair_offsets_ms[middle])
                : pair_offsets_ms[middle];
        result.maximum_abs_pair_offset_ms = pair_offsets_ms.back();
    }
    result.ok = true;
    return result;
}

ImuCsvLoadResult loadRtklibExplorerImuCsv(const std::string& path, ImuSeries& out) {
    ImuCsvLoadResult result;
    out.samples.clear();

    if (hasMatExtension(path)) {
        result.error = "MATLAB .mat inputs are forbidden by the raw/native IMU contract";
        return result;
    }
    std::ifstream file(path);
    if (!file.is_open()) {
        result.error = "could not open IMU CSV file: " + path;
        return result;
    }

    std::string header_line;
    if (!std::getline(file, header_line)) {
        result.error = "IMU CSV file is empty: " + path;
        return result;
    }
    if (!header_line.empty() && header_line.back() == '\r') {
        header_line.pop_back();
    }

    const std::vector<std::string> raw_headers = splitCsvLine(header_line);
    HeaderLookup lookup;
    for (size_t i = 0; i < raw_headers.size(); ++i) {
        lookup.emplace(normalizeHeader(raw_headers[i]), i);
    }

    size_t time_index = 0;
    std::array<size_t, 3> accel_index{};
    std::array<size_t, 3> gyro_index{};
    result.time_column =
        findColumn(lookup, raw_headers, {"unixtimes", "unixtime", "timestamp"},
                   time_index);
    if (result.time_column.empty()) {
        result.error = "could not find GPST-referenced UNIX time column";
        return result;
    }

    static const std::array<std::vector<std::string>, 3> kAccelCandidates = {
        std::vector<std::string>{"accxg", "accx"},
        std::vector<std::string>{"accyg", "accy"},
        std::vector<std::string>{"acczg", "accz"},
    };
    static const std::array<std::vector<std::string>, 3> kGyroCandidates = {
        std::vector<std::string>{"gyroxrs", "gyrox"},
        std::vector<std::string>{"gyroyrs", "gyroy"},
        std::vector<std::string>{"gyrozrs", "gyroz"},
    };
    static const char* kAxisNames[3] = {"X", "Y", "Z"};
    for (int axis = 0; axis < 3; ++axis) {
        result.accel_columns[axis] =
            findColumn(lookup, raw_headers, kAccelCandidates[axis], accel_index[axis]);
        result.gyro_columns[axis] =
            findColumn(lookup, raw_headers, kGyroCandidates[axis], gyro_index[axis]);
        if (result.accel_columns[axis].empty() ||
            result.gyro_columns[axis].empty()) {
            result.error = std::string("could not find rtklibexplorer accel/gyro ") +
                           kAxisNames[axis] + " columns";
            return result;
        }
    }

    const size_t required_columns = raw_headers.size();
    std::string line;
    int row_number = 1;
    while (std::getline(file, line)) {
        ++row_number;
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (line.empty()) {
            continue;
        }
        const std::vector<std::string> fields = splitCsvLine(line);
        if (fields.size() < required_columns) {
            result.error = "row " + std::to_string(row_number) +
                           ": expected " + std::to_string(required_columns) +
                           " columns, got " + std::to_string(fields.size());
            return result;
        }

        bool ok = true;
        const double unix_gpst = parseDouble(fields[time_index], ok);
        if (!ok || !std::isfinite(unix_gpst) ||
            unix_gpst < kUnixSecondsAtGpsEpoch) {
            result.error = "row " + std::to_string(row_number) +
                           ": invalid GPST-referenced UNIX time value";
            return result;
        }
        const double gps_seconds = unix_gpst - kUnixSecondsAtGpsEpoch;
        const int gps_week =
            static_cast<int>(std::floor(gps_seconds / kSecondsPerGpsWeek));
        ImuSample sample;
        sample.time =
            GNSSTime(gps_week, gps_seconds - gps_week * kSecondsPerGpsWeek);

        for (int axis = 0; axis < 3; ++axis) {
            const double accel_g = parseDouble(fields[accel_index[axis]], ok);
            if (!ok) {
                result.error = "row " + std::to_string(row_number) +
                               ": invalid accel value";
                return result;
            }
            const double gyro_radps = parseDouble(fields[gyro_index[axis]], ok);
            if (!ok) {
                result.error = "row " + std::to_string(row_number) +
                               ": invalid gyro value";
                return result;
            }
            sample.accel_raw(axis) = accel_g * kStandardGravityMps2;
            sample.gyro_raw_radps(axis) = gyro_radps;
        }
        out.samples.push_back(sample);
    }

    result.row_count = static_cast<int>(out.samples.size());
    result.ok = true;
    return result;
}

}  // namespace libgnss
