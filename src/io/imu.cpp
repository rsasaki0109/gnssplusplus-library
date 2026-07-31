#include <libgnss++/io/imu.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
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

ImuCsvLoadResult loadRtklibExplorerImuCsv(const std::string& path, ImuSeries& out) {
    ImuCsvLoadResult result;
    out.samples.clear();

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
