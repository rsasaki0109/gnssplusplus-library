#pragma once

#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/types.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace libgnss_apps {

struct SeedPosition {
    libgnss::GNSSTime time;
    libgnss::Vector3d position_ecef = libgnss::Vector3d::Zero();
};

namespace seed_positions_detail {

inline constexpr double kPi = 3.141592653589793238462643383279502884;
inline constexpr double kDegreesToRadians = kPi / 180.0;

inline bool parseSeparatedIntegers(std::string value,
                                   char separator,
                                   int& first,
                                   int& second,
                                   int& third) {
    std::replace(value.begin(), value.end(), separator, ' ');
    std::istringstream input(value);
    return static_cast<bool>(input >> first >> second >> third);
}

inline bool parseRtklibTime(std::string value,
                            int& hour,
                            int& minute,
                            double& second) {
    std::replace(value.begin(), value.end(), ':', ' ');
    std::istringstream input(value);
    return static_cast<bool>(input >> hour >> minute >> second);
}

inline bool isLeapYear(int year) {
    return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

inline libgnss::GNSSTime gpstCalendarToTime(int year,
                                            int month,
                                            int day,
                                            int hour,
                                            int minute,
                                            double second) {
    int days_since_gps_epoch = 0;
    for (int current_year = 1980; current_year < year; ++current_year) {
        days_since_gps_epoch += isLeapYear(current_year) ? 366 : 365;
    }

    int days_in_month[] = {
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (isLeapYear(year)) {
        days_in_month[1] = 29;
    }
    for (int current_month = 1; current_month < month; ++current_month) {
        days_since_gps_epoch += days_in_month[current_month - 1];
    }
    days_since_gps_epoch += day;
    days_since_gps_epoch -= 6;

    const int gps_week = days_since_gps_epoch / 7;
    const int day_of_week = days_since_gps_epoch % 7;
    const double tow = static_cast<double>(day_of_week) * 86400.0 +
                       static_cast<double>(hour) * 3600.0 +
                       static_cast<double>(minute) * 60.0 + second;
    return libgnss::GNSSTime(gps_week, tow);
}

inline bool parseRtklibSeedPositionLine(const std::string& line,
                                        SeedPosition& seed) {
    std::istringstream input(line);
    std::string date_token;
    std::string time_token;
    if (!(input >> date_token >> time_token)) {
        return false;
    }

    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    double second = 0.0;
    if (!parseSeparatedIntegers(date_token, '/', year, month, day) ||
        !parseRtklibTime(time_token, hour, minute, second)) {
        return false;
    }

    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double height_m = 0.0;
    if (!(input >> latitude_deg >> longitude_deg >> height_m)) {
        return false;
    }
    if (!std::isfinite(latitude_deg) || !std::isfinite(longitude_deg) ||
        !std::isfinite(height_m)) {
        return false;
    }

    seed.time = gpstCalendarToTime(year, month, day, hour, minute, second);
    seed.position_ecef = libgnss::geodetic2ecef(latitude_deg * kDegreesToRadians,
                                                longitude_deg * kDegreesToRadians,
                                                height_m);
    return seed.position_ecef.norm() > 1e6;
}

inline bool parseSeedPositionLine(const std::string& line, SeedPosition& seed) {
    std::istringstream input(line);
    std::string first_token;
    if (!(input >> first_token)) {
        return false;
    }
    if (first_token[0] == '%' || first_token[0] == '#') {
        return false;
    }
    if (first_token.find('/') != std::string::npos) {
        return parseRtklibSeedPositionLine(line, seed);
    }

    int week = 0;
    try {
        std::size_t consumed = 0;
        week = std::stoi(first_token, &consumed);
        if (consumed != first_token.size()) {
            return false;
        }
    } catch (const std::exception&) {
        return false;
    }

    double tow = 0.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    if (!(input >> tow >> x >> y >> z)) {
        return false;
    }
    if (!std::isfinite(tow) || !std::isfinite(x) ||
        !std::isfinite(y) || !std::isfinite(z)) {
        return false;
    }

    seed.time = libgnss::GNSSTime(week, tow);
    seed.position_ecef = libgnss::Vector3d(x, y, z);
    return seed.position_ecef.norm() > 1e6;
}

}  // namespace seed_positions_detail

inline std::vector<SeedPosition> readSeedPositions(const std::string& path) {
    std::ifstream input(path);
    if (!input.is_open()) {
        throw std::runtime_error("failed to open seed POS file: " + path);
    }

    std::vector<SeedPosition> seeds;
    std::string line;
    while (std::getline(input, line)) {
        SeedPosition seed;
        if (seed_positions_detail::parseSeedPositionLine(line, seed)) {
            seeds.push_back(seed);
        }
    }
    if (seeds.empty()) {
        throw std::runtime_error(
            "seed POS file has no readable LibGNSS++ or RTKLIB POS rows: " + path);
    }

    std::sort(seeds.begin(),
              seeds.end(),
              [](const SeedPosition& lhs, const SeedPosition& rhs) {
                  return lhs.time < rhs.time;
              });
    return seeds;
}

inline bool findSeedPosition(const std::vector<SeedPosition>& seeds,
                             const libgnss::GNSSTime& time,
                             double tolerance_s,
                             double interpolation_max_gap_s,
                             std::size_t& cursor,
                             libgnss::Vector3d& position_ecef,
                             bool& interpolated) {
    interpolated = false;
    while (cursor < seeds.size() && seeds[cursor].time - time < -tolerance_s) {
        ++cursor;
    }

    std::size_t best_index = seeds.size();
    double best_abs_dt = tolerance_s;
    if (cursor < seeds.size()) {
        const double abs_dt = std::abs(seeds[cursor].time - time);
        if (abs_dt <= best_abs_dt) {
            best_index = cursor;
            best_abs_dt = abs_dt;
        }
    }
    if (cursor > 0) {
        const std::size_t previous_index = cursor - 1;
        const double abs_dt = std::abs(seeds[previous_index].time - time);
        if (abs_dt <= best_abs_dt) {
            best_index = previous_index;
        }
    }
    if (best_index == seeds.size()) {
        if (interpolation_max_gap_s <= 0.0 || cursor == 0 ||
            cursor >= seeds.size()) {
            return false;
        }
        const SeedPosition& lower = seeds[cursor - 1];
        const SeedPosition& upper = seeds[cursor];
        const double gap_s = upper.time - lower.time;
        const double lower_dt_s = time - lower.time;
        if (gap_s <= 0.0 || gap_s > interpolation_max_gap_s ||
            lower_dt_s < 0.0 || lower_dt_s > gap_s) {
            return false;
        }
        const double alpha = lower_dt_s / gap_s;
        position_ecef =
            lower.position_ecef +
            alpha * (upper.position_ecef - lower.position_ecef);
        interpolated = true;
        return position_ecef.norm() > 1e6;
    }

    position_ecef = seeds[best_index].position_ecef;
    return true;
}

}  // namespace libgnss_apps
