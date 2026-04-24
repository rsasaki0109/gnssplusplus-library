#include <libgnss++/algorithms/ppp.hpp>
#include <libgnss++/algorithms/ppp_ar.hpp>
#include <libgnss++/algorithms/ppp_atmosphere.hpp>
#include <libgnss++/algorithms/ppp_clas.hpp>
#include <libgnss++/algorithms/ppp_osr.hpp>
#include <libgnss++/algorithms/ppp_utils.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signal_policy.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/io/qzss_l6.hpp>
#include <libgnss++/io/rtcm.hpp>
#include <libgnss++/models/troposphere.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <fstream>
#include <cmath>
#include <ctime>
#include <cctype>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <set>
#include <string>
#include <vector>

namespace libgnss {

namespace {

constexpr double kDegreesToRadians = M_PI / 180.0;
constexpr double kDefaultZenithDelayMeters = 2.3;
constexpr double kRtkLibMinElevationRadians = 5.0 * kDegreesToRadians;
constexpr double kRtkLibGlonassErrorFactor = 1.5;
constexpr double kRtkLibSbasErrorFactor = 3.0;
constexpr double kRtkLibGpsL5ErrorFactor = 10.0;
constexpr std::array<double, 11> kOceanLoadingPeriodsSeconds = {
    12.4206012 * 3600.0,   // M2
    12.0 * 3600.0,         // S2
    12.65834751 * 3600.0,  // N2
    11.967234 * 3600.0,    // K2
    23.93447213 * 3600.0,  // K1
    25.81933871 * 3600.0,  // O1
    24.065889 * 3600.0,    // P1
    26.8683567 * 3600.0,   // Q1
    327.8599387 * 3600.0,  // Mf
    661.3111655 * 3600.0,  // Mm
    4382.905 * 3600.0      // Ssa
};

std::string trimCopy(const std::string& text) {
    const size_t first = text.find_first_not_of(' ');
    if (first == std::string::npos) {
        return "";
    }
    const size_t last = text.find_last_not_of(' ');
    return text.substr(first, last - first + 1);
}

std::string normalizeAntennaType(const std::string& antenna_type) {
    std::string normalized = trimCopy(antenna_type);
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });
    return normalized;
}

std::string normalizeStationName(const std::string& station_name) {
    return normalizeAntennaType(station_name);
}

SignalType antexSignalType(const std::string& code) {
    const std::string trimmed = trimCopy(code);
    if (trimmed == "G01") return SignalType::GPS_L1CA;
    if (trimmed == "G02") return SignalType::GPS_L2C;
    if (trimmed == "G05") return SignalType::GPS_L5;
    if (trimmed == "R01") return SignalType::GLO_L1CA;
    if (trimmed == "R02") return SignalType::GLO_L2CA;
    if (trimmed == "E01") return SignalType::GAL_E1;
    if (trimmed == "E05") return SignalType::GAL_E5A;
    if (trimmed == "E07") return SignalType::GAL_E5B;
    if (trimmed == "E06") return SignalType::GAL_E6;
    if (trimmed == "C02") return SignalType::BDS_B1I;
    if (trimmed == "C07") return SignalType::BDS_B2I;
    if (trimmed == "C06") return SignalType::BDS_B3I;
    if (trimmed == "C01") return SignalType::BDS_B1C;
    if (trimmed == "C05") return SignalType::BDS_B2A;
    if (trimmed == "J01") return SignalType::QZS_L1CA;
    if (trimmed == "J02") return SignalType::QZS_L2C;
    if (trimmed == "J05") return SignalType::QZS_L5;
    return SignalType::SIGNAL_TYPE_COUNT;
}

std::string extractAntexFrequencyCode(const std::string& line) {
    for (size_t index = 0; index + 2 < std::min<size_t>(line.size(), 20U); ++index) {
        if (std::isalpha(static_cast<unsigned char>(line[index])) &&
            std::isdigit(static_cast<unsigned char>(line[index + 1])) &&
            std::isdigit(static_cast<unsigned char>(line[index + 2]))) {
            return line.substr(index, 3);
        }
    }
    return "";
}

bool parseAntexNoaziValues(const std::string& line,
                           std::array<double, 19>& pcv_m) {
    pcv_m.fill(0.0);
    std::istringstream iss(line);
    std::string token;
    if (!(iss >> token) || token != "NOAZI") {
        return false;
    }
    double value_mm = 0.0;
    int count = 0;
    while (count < static_cast<int>(pcv_m.size()) && iss >> value_mm) {
        pcv_m[static_cast<size_t>(count)] = value_mm * 1e-3;
        ++count;
    }
    if (count == 0) {
        return false;
    }
    for (; count < static_cast<int>(pcv_m.size()); ++count) {
        pcv_m[static_cast<size_t>(count)] = pcv_m[static_cast<size_t>(count - 1)];
    }
    return true;
}

double interpolateNoaziPcv(const std::array<double, 19>& pcv_m,
                           double zenith_angle_deg) {
    const double a = zenith_angle_deg / 5.0;
    const int index = static_cast<int>(a);
    if (index < 0) {
        return pcv_m.front();
    }
    if (index >= 18) {
        return pcv_m.back();
    }
    const double fraction = a - static_cast<double>(index);
    return pcv_m[static_cast<size_t>(index)] * (1.0 - fraction) +
           pcv_m[static_cast<size_t>(index + 1)] * fraction;
}

bool loadReceiverAntexOffsets(
    const std::string& filename,
    std::map<std::string, ReceiverAntexEntry>& receiver_offsets) {
    receiver_offsets.clear();
    std::ifstream input(filename);
    if (!input.is_open()) {
        return false;
    }

    bool in_antenna = false;
    bool receiver_entry = false;
    SignalType current_signal = SignalType::SIGNAL_TYPE_COUNT;
    std::string current_type;
    ReceiverAntexEntry current_entry;
    std::string line;
    while (std::getline(input, line)) {
        const std::string label = line.size() >= 60 ? trimCopy(line.substr(60)) : "";
        if (label == "START OF ANTENNA") {
            in_antenna = true;
            receiver_entry = false;
            current_signal = SignalType::SIGNAL_TYPE_COUNT;
            current_type.clear();
            current_entry = ReceiverAntexEntry{};
            continue;
        }
        if (!in_antenna) {
            continue;
        }
        if (label == "END OF ANTENNA") {
            if (receiver_entry && !current_type.empty() &&
                (!current_entry.offsets_enu_m.empty() || !current_entry.noazi_pcv_m.empty())) {
                receiver_offsets[current_type] = current_entry;
            }
            in_antenna = false;
            continue;
        }
        if (label == "TYPE / SERIAL NO") {
            current_type = normalizeAntennaType(line.substr(0, 20));
            const std::string serial = trimCopy(line.substr(20, 20));
            receiver_entry = !(serial.size() == 3 &&
                               std::isalpha(static_cast<unsigned char>(serial[0])) &&
                               std::isdigit(static_cast<unsigned char>(serial[1])) &&
                               std::isdigit(static_cast<unsigned char>(serial[2])));
            continue;
        }
        if (!receiver_entry) {
            continue;
        }
        if (label == "START OF FREQUENCY") {
            current_signal = antexSignalType(extractAntexFrequencyCode(line));
            continue;
        }
        if (label == "END OF FREQUENCY") {
            current_signal = SignalType::SIGNAL_TYPE_COUNT;
            continue;
        }
        if (label == "NORTH / EAST / UP" &&
            current_signal != SignalType::SIGNAL_TYPE_COUNT) {
            std::istringstream iss(line.substr(0, std::min<size_t>(line.size(), 30U)));
            double north_mm = 0.0;
            double east_mm = 0.0;
            double up_mm = 0.0;
            if (iss >> north_mm >> east_mm >> up_mm) {
                current_entry.offsets_enu_m[current_signal] =
                    Vector3d(east_mm * 1e-3, north_mm * 1e-3, up_mm * 1e-3);
            }
            continue;
        }
        if (line.find("NOAZI") != std::string::npos &&
            current_signal != SignalType::SIGNAL_TYPE_COUNT) {
            std::array<double, 19> pcv_m{};
            if (parseAntexNoaziValues(line, pcv_m)) {
                current_entry.noazi_pcv_m[current_signal] = pcv_m;
            }
        }
    }

    return !receiver_offsets.empty();
}

bool satelliteIdFromAntexSerial(const std::string& serial_field, SatelliteId& satellite) {
    const std::string serial = trimCopy(serial_field);
    if (serial.size() < 2) {
        return false;
    }
    GNSSSystem system = GNSSSystem::UNKNOWN;
    switch (serial[0]) {
        case 'G': system = GNSSSystem::GPS; break;
        case 'R': system = GNSSSystem::GLONASS; break;
        case 'E': system = GNSSSystem::Galileo; break;
        case 'C': system = GNSSSystem::BeiDou; break;
        case 'J': system = GNSSSystem::QZSS; break;
        default: return false;
    }
    try {
        const int prn = std::stoi(serial.substr(1));
        if (prn <= 0 || prn > 255) {
            return false;
        }
        satellite = SatelliteId(system, static_cast<uint8_t>(prn));
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool parseAntexEpoch(const std::string& line, GNSSTime& time) {
    std::istringstream iss(line.substr(0, std::min<size_t>(line.size(), 50U)));
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    double second = 0.0;
    if (!(iss >> year >> month >> day >> hour >> minute >> second)) {
        return false;
    }
    std::tm epoch_tm{};
    epoch_tm.tm_year = year - 1900;
    epoch_tm.tm_mon = month - 1;
    epoch_tm.tm_mday = day;
    epoch_tm.tm_hour = hour;
    epoch_tm.tm_min = minute;
    epoch_tm.tm_sec = static_cast<int>(std::floor(second));
    const time_t seconds_since_unix = timegm(&epoch_tm);
    if (seconds_since_unix == static_cast<time_t>(-1)) {
        return false;
    }
    const auto tp = std::chrono::system_clock::from_time_t(seconds_since_unix) +
        std::chrono::microseconds(
            static_cast<long long>(std::llround((second - std::floor(second)) * 1e6)));
    time = GNSSTime::fromSystemTime(tp);
    return true;
}

bool loadSatelliteAntexOffsets(
    const std::string& filename,
    std::map<SatelliteId, std::vector<SatelliteAntexEntry>>& satellite_offsets) {
    satellite_offsets.clear();
    std::ifstream input(filename);
    if (!input.is_open()) {
        return false;
    }

    bool in_antenna = false;
    bool satellite_entry = false;
    SignalType current_signal = SignalType::SIGNAL_TYPE_COUNT;
    SatelliteId current_satellite;
    SatelliteAntexEntry current_entry;
    std::string line;
    while (std::getline(input, line)) {
        const std::string label = line.size() >= 60 ? trimCopy(line.substr(60)) : "";
        if (label == "START OF ANTENNA") {
            in_antenna = true;
            satellite_entry = false;
            current_signal = SignalType::SIGNAL_TYPE_COUNT;
            current_satellite = SatelliteId();
            current_entry = SatelliteAntexEntry{};
            continue;
        }
        if (!in_antenna) {
            continue;
        }
        if (label == "END OF ANTENNA") {
            if (satellite_entry && !current_entry.offsets_neu_m.empty()) {
                satellite_offsets[current_satellite].push_back(current_entry);
            }
            in_antenna = false;
            continue;
        }
        if (label == "TYPE / SERIAL NO") {
            satellite_entry = satelliteIdFromAntexSerial(line.substr(20, 20), current_satellite);
            continue;
        }
        if (!satellite_entry) {
            continue;
        }
        if (label == "VALID FROM") {
            current_entry.has_valid_from = parseAntexEpoch(line, current_entry.valid_from);
            continue;
        }
        if (label == "VALID UNTIL") {
            current_entry.has_valid_until = parseAntexEpoch(line, current_entry.valid_until);
            continue;
        }
        if (label == "START OF FREQUENCY") {
            current_signal = antexSignalType(extractAntexFrequencyCode(line));
            continue;
        }
        if (label == "END OF FREQUENCY") {
            current_signal = SignalType::SIGNAL_TYPE_COUNT;
            continue;
        }
        if (label == "NORTH / EAST / UP" &&
            current_signal != SignalType::SIGNAL_TYPE_COUNT) {
            std::istringstream iss(line.substr(0, std::min<size_t>(line.size(), 30U)));
            double north_mm = 0.0;
            double east_mm = 0.0;
            double up_mm = 0.0;
            if (iss >> north_mm >> east_mm >> up_mm) {
                current_entry.offsets_neu_m[current_signal] =
                    Vector3d(north_mm * 1e-3, east_mm * 1e-3, up_mm * 1e-3);
            }
        }
    }

    for (auto& [satellite, entries] : satellite_offsets) {
        std::sort(entries.begin(), entries.end(), [](const SatelliteAntexEntry& lhs,
                                                      const SatelliteAntexEntry& rhs) {
            if (lhs.has_valid_from != rhs.has_valid_from) {
                return lhs.has_valid_from;
            }
            if (!lhs.has_valid_from) {
                return false;
            }
            return lhs.valid_from < rhs.valid_from;
        });
    }
    return !satellite_offsets.empty();
}

bool parseBlqValues(const std::string& line, std::array<double, 11>& values) {
    std::istringstream iss(line);
    for (double& value : values) {
        if (!(iss >> value)) {
            return false;
        }
    }
    return true;
}

bool loadOceanLoadingCoefficients(
    const std::string& filename,
    const std::string& station_name,
    std::array<double, 11>& up_amplitudes_m,
    std::array<double, 11>& west_amplitudes_m,
    std::array<double, 11>& south_amplitudes_m,
    std::array<double, 11>& up_phases_deg,
    std::array<double, 11>& west_phases_deg,
    std::array<double, 11>& south_phases_deg) {
    const std::string normalized_station = normalizeStationName(station_name);
    if (normalized_station.empty()) {
        return false;
    }

    std::ifstream input(filename);
    if (!input.is_open()) {
        return false;
    }

    bool collecting = false;
    int collected_rows = 0;
    std::string line;
    while (std::getline(input, line)) {
        const std::string trimmed = trimCopy(line);
        if (trimmed.empty() || trimmed.rfind("$$", 0) == 0) {
            continue;
        }

        if (!collecting) {
            std::istringstream header_stream(trimmed);
            std::string first_token;
            header_stream >> first_token;
            if (first_token.empty()) {
                continue;
            }
            const bool numeric_header =
                std::isdigit(static_cast<unsigned char>(first_token.front())) ||
                first_token.front() == '-' || first_token.front() == '+';
            if (numeric_header) {
                continue;
            }
            collecting = normalizeStationName(first_token) == normalized_station;
            collected_rows = 0;
            continue;
        }

        std::array<double, 11> values{};
        if (!parseBlqValues(trimmed, values)) {
            collecting = false;
            collected_rows = 0;
            continue;
        }

        switch (collected_rows) {
            case 0: up_amplitudes_m = values; break;
            case 1: west_amplitudes_m = values; break;
            case 2: south_amplitudes_m = values; break;
            case 3: up_phases_deg = values; break;
            case 4: west_phases_deg = values; break;
            case 5: south_phases_deg = values; break;
            default: break;
        }
        ++collected_rows;
        if (collected_rows == 6) {
            return true;
        }
    }

    return false;
}

bool pppDebugEnabled() {
    return ppp_shared::pppDebugEnabled();
}

int dayOfYearFromTime(const GNSSTime& time) {
    const auto system_time = time.toSystemTime();
    const std::time_t utc_seconds = std::chrono::system_clock::to_time_t(system_time);
    std::tm utc_tm{};
    gmtime_r(&utc_seconds, &utc_tm);
    return utc_tm.tm_yday + 1;
}

double julianDateFromTime(const GNSSTime& time) {
    // GPS epoch (1980-01-06 00:00:00) is JD 2444244.5.
    return 2444244.5 + static_cast<double>(time.week) * 7.0 + time.tow / 86400.0;
}

double wrapRadians(double angle_rad) {
    const double wrapped = std::fmod(angle_rad, 2.0 * M_PI);
    return wrapped < 0.0 ? wrapped + 2.0 * M_PI : wrapped;
}

double greenwichMeanSiderealTime(const GNSSTime& time) {
    const double jd = julianDateFromTime(time);
    const double t = (jd - 2451545.0) / 36525.0;
    const double gmst_deg =
        280.46061837 + 360.98564736629 * (jd - 2451545.0) +
        0.000387933 * t * t - (t * t * t) / 38710000.0;
    return wrapRadians(gmst_deg * kDegreesToRadians);
}

Vector3d rotateEciToEcef(const Vector3d& eci, double gmst_rad) {
    const double cos_gmst = std::cos(gmst_rad);
    const double sin_gmst = std::sin(gmst_rad);
    return Vector3d(
        cos_gmst * eci.x() + sin_gmst * eci.y(),
        -sin_gmst * eci.x() + cos_gmst * eci.y(),
        eci.z());
}

Vector3d approximateSunPositionEcef(const GNSSTime& time) {
    constexpr double kAstronomicalUnitMeters = 149597870700.0;
    const double days_since_j2000 = julianDateFromTime(time) - 2451545.0;
    const double mean_longitude =
        wrapRadians((280.460 + 0.9856474 * days_since_j2000) * kDegreesToRadians);
    const double mean_anomaly =
        wrapRadians((357.528 + 0.9856003 * days_since_j2000) * kDegreesToRadians);
    const double ecliptic_longitude =
        wrapRadians(mean_longitude +
                    (1.915 * std::sin(mean_anomaly) +
                     0.020 * std::sin(2.0 * mean_anomaly)) * kDegreesToRadians);
    const double obliquity =
        (23.439 - 0.0000004 * days_since_j2000) * kDegreesToRadians;
    const double radius_au =
        1.00014 - 0.01671 * std::cos(mean_anomaly) -
        0.00014 * std::cos(2.0 * mean_anomaly);
    const double radius_m = radius_au * kAstronomicalUnitMeters;
    const Vector3d eci(
        radius_m * std::cos(ecliptic_longitude),
        radius_m * std::cos(obliquity) * std::sin(ecliptic_longitude),
        radius_m * std::sin(obliquity) * std::sin(ecliptic_longitude));
    return rotateEciToEcef(eci, greenwichMeanSiderealTime(time));
}

bool normalizeVector(const Vector3d& input, Vector3d& output) {
    const double norm = input.norm();
    if (!std::isfinite(norm) || norm <= 0.0) {
        return false;
    }
    output = input / norm;
    return true;
}

double nominalYawAngle(double beta_rad, double orbit_angle_rad) {
    if (std::abs(beta_rad) < 1e-12 && std::abs(orbit_angle_rad) < 1e-12) {
        return M_PI;
    }
    return std::atan2(-std::tan(beta_rad), std::sin(orbit_angle_rad)) + M_PI;
}

double calculatePhaseWindupCycles(const GNSSTime& time,
                                  const Vector3d& receiver_position_ecef,
                                  const Vector3d& satellite_position_ecef,
                                  const Vector3d& satellite_velocity_ecef,
                                  double previous_windup_cycles) {
    const Vector3d sun_position_ecef = approximateSunPositionEcef(time);

    Vector3d satellite_unit;
    Vector3d sun_unit;
    Vector3d orbit_normal_unit;
    Vector3d sun_orbit_cross_unit;
    Vector3d adjusted_velocity = satellite_velocity_ecef;
    adjusted_velocity.x() -= constants::OMEGA_E * satellite_position_ecef.y();
    adjusted_velocity.y() += constants::OMEGA_E * satellite_position_ecef.x();
    const Vector3d orbit_normal = satellite_position_ecef.cross(adjusted_velocity);
    const Vector3d sun_orbit_cross = sun_position_ecef.cross(orbit_normal);
    if (!normalizeVector(satellite_position_ecef, satellite_unit) ||
        !normalizeVector(sun_position_ecef, sun_unit) ||
        !normalizeVector(orbit_normal, orbit_normal_unit) ||
        !normalizeVector(sun_orbit_cross, sun_orbit_cross_unit)) {
        return previous_windup_cycles;
    }

    const double beta = M_PI_2 - std::acos(std::clamp(sun_unit.dot(orbit_normal_unit), -1.0, 1.0));
    const double orbit_angle = std::acos(std::clamp(satellite_unit.dot(sun_orbit_cross_unit), -1.0, 1.0));
    double mu = M_PI_2 + (satellite_unit.dot(sun_unit) <= 0.0 ? -orbit_angle : orbit_angle);
    if (mu < -M_PI_2) {
        mu += 2.0 * M_PI;
    } else if (mu >= M_PI_2) {
        mu -= 2.0 * M_PI;
    }

    const double yaw = nominalYawAngle(beta, mu);
    Vector3d satellite_x_nominal = orbit_normal_unit.cross(satellite_unit);
    if (!normalizeVector(satellite_x_nominal, satellite_x_nominal)) {
        return previous_windup_cycles;
    }
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const Vector3d satellite_x = -sin_yaw * orbit_normal_unit + cos_yaw * satellite_x_nominal;
    const Vector3d satellite_y = -cos_yaw * orbit_normal_unit - sin_yaw * satellite_x_nominal;

    Vector3d line_of_sight;
    if (!normalizeVector(receiver_position_ecef - satellite_position_ecef, line_of_sight)) {
        return previous_windup_cycles;
    }
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;
    ecef2geodetic(receiver_position_ecef, latitude_rad, longitude_rad, height_m);
    const Vector3d receiver_x = enu2ecef(Vector3d(0.0, 1.0, 0.0), latitude_rad, longitude_rad);
    const Vector3d receiver_y = -enu2ecef(Vector3d(1.0, 0.0, 0.0), latitude_rad, longitude_rad);

    const Vector3d satellite_dipole =
        satellite_x - line_of_sight * line_of_sight.dot(satellite_x) - line_of_sight.cross(satellite_y);
    const Vector3d receiver_dipole =
        receiver_x - line_of_sight * line_of_sight.dot(receiver_x) + line_of_sight.cross(receiver_y);
    const double norm_product = satellite_dipole.norm() * receiver_dipole.norm();
    if (!std::isfinite(norm_product) || norm_product <= 0.0) {
        return previous_windup_cycles;
    }
    const double cosine = std::clamp(satellite_dipole.dot(receiver_dipole) / norm_product, -1.0, 1.0);
    double phase = std::acos(cosine) / (2.0 * M_PI);
    if (line_of_sight.dot(satellite_dipole.cross(receiver_dipole)) < 0.0) {
        phase = -phase;
    }
    return phase + std::floor(previous_windup_cycles - phase + 0.5);
}

Vector3d approximateMoonPositionEcef(const GNSSTime& time) {
    constexpr double kKilometersToMeters = 1000.0;
    const double days_since_j2000 = julianDateFromTime(time) - 2451545.0;
    const double mean_longitude =
        wrapRadians((218.316 + 13.176396 * days_since_j2000) * kDegreesToRadians);
    const double mean_anomaly_moon =
        wrapRadians((134.963 + 13.064993 * days_since_j2000) * kDegreesToRadians);
    const double mean_anomaly_sun =
        wrapRadians((357.529 + 0.98560028 * days_since_j2000) * kDegreesToRadians);
    const double elongation =
        wrapRadians((297.850 + 12.190749 * days_since_j2000) * kDegreesToRadians);
    const double argument_of_latitude =
        wrapRadians((93.272 + 13.229350 * days_since_j2000) * kDegreesToRadians);

    const double ecliptic_longitude =
        wrapRadians(mean_longitude +
                    (6.289 * std::sin(mean_anomaly_moon) +
                     1.274 * std::sin(2.0 * elongation - mean_anomaly_moon) +
                     0.658 * std::sin(2.0 * elongation) +
                     0.214 * std::sin(2.0 * mean_anomaly_moon) -
                     0.186 * std::sin(mean_anomaly_sun)) *
                        kDegreesToRadians);
    const double ecliptic_latitude =
        (5.128 * std::sin(argument_of_latitude) +
         0.280 * std::sin(mean_anomaly_moon + argument_of_latitude) +
         0.277 * std::sin(mean_anomaly_moon - argument_of_latitude) +
         0.173 * std::sin(2.0 * elongation - argument_of_latitude)) *
        kDegreesToRadians;
    const double radius_m =
        (385001.0 - 20905.0 * std::cos(mean_anomaly_moon) -
         3699.0 * std::cos(2.0 * elongation - mean_anomaly_moon) -
         2956.0 * std::cos(2.0 * elongation) -
         570.0 * std::cos(2.0 * mean_anomaly_moon)) *
        kKilometersToMeters;
    const double obliquity =
        (23.439 - 0.0000004 * days_since_j2000) * kDegreesToRadians;
    const double cos_lat = std::cos(ecliptic_latitude);
    const Vector3d eci(
        radius_m * cos_lat * std::cos(ecliptic_longitude),
        radius_m *
            (std::cos(obliquity) * cos_lat * std::sin(ecliptic_longitude) -
             std::sin(obliquity) * std::sin(ecliptic_latitude)),
        radius_m *
            (std::sin(obliquity) * cos_lat * std::sin(ecliptic_longitude) +
             std::cos(obliquity) * std::sin(ecliptic_latitude)));
    return rotateEciToEcef(eci, greenwichMeanSiderealTime(time));
}

Vector3d surfaceUpVector(const Vector3d& receiver_position) {
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;
    ecef2geodetic(receiver_position, latitude_rad, longitude_rad, height_m);
    return Vector3d(
               std::cos(latitude_rad) * std::cos(longitude_rad),
               std::cos(latitude_rad) * std::sin(longitude_rad),
               std::sin(latitude_rad))
        .normalized();
}

Vector3d bodyTideDisplacement(const Vector3d& receiver_position,
                              const Vector3d& body_position_ecef,
                              double body_gm_m3_s2) {
    constexpr double kEarthGM = 3.986004418e14;
    constexpr double kLoveH2 = 0.6078;
    constexpr double kLoveL2 = 0.0847;

    const double body_distance_m = body_position_ecef.norm();
    if (!std::isfinite(body_distance_m) || body_distance_m <= 1.0) {
        return Vector3d::Zero();
    }

    const Vector3d up = surfaceUpVector(receiver_position);
    const Vector3d body_dir = body_position_ecef / body_distance_m;
    const double projection = std::clamp(up.dot(body_dir), -1.0, 1.0);
    const Vector3d tangential = body_dir - projection * up;
    const double scale =
        body_gm_m3_s2 / kEarthGM *
        std::pow(constants::WGS84_A / body_distance_m, 3.0) *
        constants::WGS84_A;
    return scale * (kLoveH2 * (1.5 * projection * projection - 0.5) * up +
                    3.0 * kLoveL2 * projection * tangential);
}

double modeledZenithTroposphereDelayMetersImpl(const Vector3d& receiver_position,
                                               const GNSSTime& time) {
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;
    ecef2geodetic(receiver_position, latitude_rad, longitude_rad, height_m);
    const auto delay =
        models::estimateZenithTroposphereClimatology(latitude_rad, height_m, dayOfYearFromTime(time));
    const double total_delay_m = delay.totalDelayMeters();
    return std::isfinite(total_delay_m) && total_delay_m > 0.0 ?
        total_delay_m :
        kDefaultZenithDelayMeters;
}

double modeledTroposphereDelayMeters(const Vector3d& receiver_position,
                                     double elevation,
                                     const GNSSTime& time) {
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;
    ecef2geodetic(receiver_position, latitude_rad, longitude_rad, height_m);
    const double modeled_delay = models::modeledTroposphereDelayClimatology(
        latitude_rad,
        height_m,
        elevation,
        dayOfYearFromTime(time));
    return std::isfinite(modeled_delay) && modeled_delay > 0.0 ?
        modeled_delay :
        kDefaultZenithDelayMeters;
}

double wetTroposphereMapping(const Vector3d& receiver_position,
                             double elevation,
                             const GNSSTime& time) {
    (void)time;
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;
    ecef2geodetic(receiver_position, latitude_rad, longitude_rad, height_m);
    return models::niellWetMapping(latitude_rad, elevation);
}

double estimatedTroposphereDelayFromState(double modeled_slant_delay_m,
                                          double wet_mapping,
                                          double modeled_zenith_delay_m,
                                          double zenith_delay_m) {
    if (std::isfinite(modeled_slant_delay_m) && modeled_slant_delay_m > 0.0 &&
        std::isfinite(wet_mapping) && wet_mapping > 0.0 &&
        std::isfinite(modeled_zenith_delay_m) && modeled_zenith_delay_m > 0.0) {
        return modeled_slant_delay_m + wet_mapping * (zenith_delay_m - modeled_zenith_delay_m);
    }
    return wet_mapping * zenith_delay_m;
}

double estimatedTroposphereDelayMeters(const Vector3d& receiver_position,
                                       double elevation,
                                       const GNSSTime& time,
                                       double zenith_delay_m) {
    return estimatedTroposphereDelayFromState(
        modeledTroposphereDelayMeters(receiver_position, elevation, time),
        wetTroposphereMapping(receiver_position, elevation, time),
        modeledZenithTroposphereDelayMetersImpl(receiver_position, time),
        zenith_delay_m);
}

int ionosphereConstraintSystemIndex(GNSSSystem system) {
    switch (system) {
        case GNSSSystem::GPS: return 0;
        case GNSSSystem::GLONASS: return 1;
        case GNSSSystem::Galileo: return 2;
        case GNSSSystem::QZSS: return 3;
        default: return -1;
    }
}

bool stecConstraintSigmaMeters(const std::map<std::string, std::string>& atmos_tokens,
                                const SatelliteId& satellite,
                                double& sigma_m) {
    sigma_m = 0.5;
    const std::string stec_std_key = "atmos_stec_std_l1_m:" + satellite.toString();
    double broadcast_sigma_m = 0.0;
    if (ppp_atmosphere::parseAtmosTokenDouble(
            atmos_tokens, stec_std_key, broadcast_sigma_m) &&
        std::isfinite(broadcast_sigma_m) && broadcast_sigma_m > 0.0) {
        if (broadcast_sigma_m > 1.0) {
            return false;
        }
        sigma_m = std::max(1e-3, broadcast_sigma_m);
    }
    return true;
}

bool usablePseudorangeObservation(const Observation& observation) {
    return observation.valid &&
           observation.has_pseudorange &&
           observation.pseudorange > 0.0 &&
           std::isfinite(observation.pseudorange);
}

double observationCodeBiasMeters(GNSSSystem system,
                                 SignalType primary_signal,
                                 SignalType secondary_signal,
                                 bool use_ionosphere_free,
                                 const std::map<uint8_t, double>& code_bias_m,
                                 double coeff_primary,
                                 double coeff_secondary) {
    const uint8_t primary_id = rtcmSsrSignalId(system, primary_signal);
    if (primary_id == 0U) {
        return 0.0;
    }
    const auto primary_it = code_bias_m.find(primary_id);
    const double primary_bias =
        primary_it != code_bias_m.end() ? primary_it->second : 0.0;
    if (!use_ionosphere_free || secondary_signal == SignalType::SIGNAL_TYPE_COUNT) {
        return primary_bias;
    }

    const uint8_t secondary_id = rtcmSsrSignalId(system, secondary_signal);
    if (secondary_id == 0U) {
        return coeff_primary * primary_bias;
    }
    const auto secondary_it = code_bias_m.find(secondary_id);
    const double secondary_bias =
        secondary_it != code_bias_m.end() ? secondary_it->second : 0.0;
    return coeff_primary * primary_bias + coeff_secondary * secondary_bias;
}

double observationPhaseBiasMeters(GNSSSystem system,
                                  SignalType primary_signal,
                                  SignalType secondary_signal,
                                  bool use_ionosphere_free,
                                  const std::map<uint8_t, double>& phase_bias_m,
                                  double coeff_primary,
                                  double coeff_secondary) {
    const uint8_t primary_id = rtcmSsrSignalId(system, primary_signal);
    if (primary_id == 0U) {
        return 0.0;
    }
    const auto primary_it = phase_bias_m.find(primary_id);
    const double primary_bias =
        primary_it != phase_bias_m.end() ? primary_it->second : 0.0;
    if (!use_ionosphere_free || secondary_signal == SignalType::SIGNAL_TYPE_COUNT) {
        return primary_bias;
    }

    const uint8_t secondary_id = rtcmSsrSignalId(system, secondary_signal);
    if (secondary_id == 0U) {
        return coeff_primary * primary_bias;
    }
    const auto secondary_it = phase_bias_m.find(secondary_id);
    const double secondary_bias =
        secondary_it != phase_bias_m.end() ? secondary_it->second : 0.0;
    return coeff_primary * primary_bias + coeff_secondary * secondary_bias;
}

bool hasSsrBiasForSignal(GNSSSystem system,
                         SignalType signal,
                         const std::map<uint8_t, double>& bias_m) {
    const uint8_t signal_id = rtcmSsrSignalId(system, signal);
    return signal_id != 0U && bias_m.find(signal_id) != bias_m.end();
}

bool observationHasRequiredSsrBiases(GNSSSystem system,
                                     SignalType primary_signal,
                                     SignalType secondary_signal,
                                     bool use_ionosphere_free,
                                     const std::map<uint8_t, double>& bias_m) {
    if (!hasSsrBiasForSignal(system, primary_signal, bias_m)) {
        return false;
    }
    if (!use_ionosphere_free || secondary_signal == SignalType::SIGNAL_TYPE_COUNT) {
        return true;
    }
    return hasSsrBiasForSignal(system, secondary_signal, bias_m);
}

bool observationHasRequiredSsrPhaseBiases(GNSSSystem system,
                                          SignalType primary_signal,
                                          SignalType secondary_signal,
                                          bool use_ionosphere_free,
                                          const std::map<uint8_t, double>& phase_bias_m) {
    if (system == GNSSSystem::GLONASS) {
        return true;
    }
    return observationHasRequiredSsrBiases(
        system, primary_signal, secondary_signal, use_ionosphere_free, phase_bias_m);
}

std::string biasObservationCode(SignalType signal) {
    switch (signal) {
        case SignalType::GPS_L1CA: return "C1C";
        case SignalType::GPS_L1P: return "C1P";
        case SignalType::GPS_L2P: return "C2P";
        case SignalType::GPS_L2C: return "C2W";
        case SignalType::GPS_L5: return "C5Q";
        case SignalType::GLO_L1CA: return "C1C";
        case SignalType::GLO_L1P: return "C1P";
        case SignalType::GLO_L2CA: return "C2C";
        case SignalType::GLO_L2P: return "C2P";
        case SignalType::GAL_E1: return "C1C";
        case SignalType::GAL_E5A: return "C5Q";
        case SignalType::GAL_E5B: return "C7Q";
        case SignalType::GAL_E6: return "C6C";
        case SignalType::BDS_B1I: return "C2I";
        case SignalType::BDS_B2I: return "C7I";
        case SignalType::BDS_B3I: return "C6I";
        case SignalType::BDS_B1C: return "C1C";
        case SignalType::BDS_B2A: return "C5Q";
        case SignalType::QZS_L1CA: return "C1C";
        case SignalType::QZS_L2C: return "C2L";
        case SignalType::QZS_L5: return "C5Q";
        default: return "";
    }
}

double dcbEntryBiasMeters(const DCBEntry& entry) {
    std::string unit = trimCopy(entry.unit);
    std::transform(unit.begin(), unit.end(), unit.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });
    if (unit == "NS") {
        return entry.bias * constants::SPEED_OF_LIGHT * 1e-9;
    }
    if (unit == "M" || unit == "METER" || unit == "METERS") {
        return entry.bias;
    }
    return std::numeric_limits<double>::quiet_NaN();
}

std::string canonicalBiasObservationCode(SignalType fallback_signal,
                                           const std::string& observation_code) {
    if (observation_code.size() >= 3 && observation_code[0] == 'C') {
        return observation_code.substr(0, 3);
    }
    if (observation_code.size() >= 2 && observation_code[0] == 'P' &&
        observation_code[1] >= '0' && observation_code[1] <= '9') {
        return std::string{"C"} + observation_code[1] + "P";
    }
    return biasObservationCode(fallback_signal);
}

bool findOsbBiasMeters(const DCBProducts& dcb_products,
                       const SatelliteId& satellite,
                       const std::string& observation_code,
                       double& bias_m) {
    for (const auto& entry : dcb_products.entries) {
        if (!entry.valid || !(entry.satellite == satellite) || entry.bias_type != "OSB") {
            continue;
        }
        if (entry.observation_1 != observation_code && entry.observation_2 != observation_code) {
            continue;
        }
        const double converted = dcbEntryBiasMeters(entry);
        if (!std::isfinite(converted)) {
            continue;
        }
        bias_m = converted;
        return true;
    }
    return false;
}

double observationDcbBiasMeters(const DCBProducts& dcb_products,
                                const SatelliteId& satellite,
                                SignalType primary_signal,
                                SignalType secondary_signal,
                                const std::string& primary_observation_code,
                                const std::string& secondary_observation_code,
                                bool use_ionosphere_free,
                                double coeff_primary,
                                double coeff_secondary) {
    const std::string primary_code =
        canonicalBiasObservationCode(primary_signal, primary_observation_code);
    if (primary_code.empty()) {
        return 0.0;
    }

    double primary_bias_m = 0.0;
    const bool have_primary = findOsbBiasMeters(dcb_products, satellite, primary_code, primary_bias_m);
    if (!use_ionosphere_free || secondary_signal == SignalType::SIGNAL_TYPE_COUNT) {
        return have_primary ? primary_bias_m : 0.0;
    }

    const std::string secondary_code =
        canonicalBiasObservationCode(secondary_signal, secondary_observation_code);
    if (secondary_code.empty()) {
        return have_primary ? coeff_primary * primary_bias_m : 0.0;
    }

    double secondary_bias_m = 0.0;
    const bool have_secondary =
        findOsbBiasMeters(dcb_products, satellite, secondary_code, secondary_bias_m);
    if (!have_primary && !have_secondary) {
        return 0.0;
    }
    return coeff_primary * (have_primary ? primary_bias_m : 0.0) +
           coeff_secondary * (have_secondary ? secondary_bias_m : 0.0);
}

bool ionexPiercePointAndMapping(const IONEXProducts& ionex_products,
                                const Vector3d& receiver_position,
                                double azimuth_rad,
                                double elevation_rad,
                                double& ipp_lat_deg,
                                double& ipp_lon_deg,
                                double& mapping_factor) {
    if (elevation_rad <= 0.0) {
        return false;
    }
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;
    ecef2geodetic(receiver_position, latitude_rad, longitude_rad, height_m);

    const double earth_radius_m =
        ionex_products.base_radius_km > 0.0 ?
            ionex_products.base_radius_km * 1000.0 :
            constants::WGS84_A;
    const double shell_height_m =
        !ionex_products.height_grid.empty() ?
            ionex_products.height_grid.front() * 1000.0 :
            450000.0;
    if (earth_radius_m <= 0.0 || shell_height_m < 0.0) {
        return false;
    }

    const double ratio = earth_radius_m / (earth_radius_m + shell_height_m);
    const double cos_elevation = std::cos(elevation_rad);
    const double argument = std::clamp(ratio * cos_elevation, -1.0, 1.0);
    const double psi = M_PI_2 - elevation_rad - std::asin(argument);
    const double sin_lat = std::sin(latitude_rad);
    const double cos_lat = std::cos(latitude_rad);
    const double sin_psi = std::sin(psi);
    const double cos_psi = std::cos(psi);

    const double ipp_lat_rad = std::asin(
        std::clamp(sin_lat * cos_psi + cos_lat * sin_psi * std::cos(azimuth_rad), -1.0, 1.0));
    const double ipp_lon_rad = longitude_rad + std::atan2(
        sin_psi * std::sin(azimuth_rad),
        cos_lat * cos_psi - sin_lat * sin_psi * std::cos(azimuth_rad));

    const double mapping_argument = ratio * cos_elevation;
    mapping_factor = 1.0 / std::sqrt(std::max(1e-12, 1.0 - mapping_argument * mapping_argument));
    ipp_lat_deg = ipp_lat_rad / kDegreesToRadians;
    ipp_lon_deg = ipp_lon_rad / kDegreesToRadians;
    while (ipp_lon_deg > 180.0) {
        ipp_lon_deg -= 360.0;
    }
    while (ipp_lon_deg < -180.0) {
        ipp_lon_deg += 360.0;
    }
    return std::isfinite(mapping_factor);
}

std::vector<SignalType> primarySignals(GNSSSystem system) {
    switch (system) {
        case GNSSSystem::GPS: return {SignalType::GPS_L1CA, SignalType::GPS_L1P};
        case GNSSSystem::GLONASS: return {SignalType::GLO_L1CA, SignalType::GLO_L1P};
        case GNSSSystem::Galileo: return {SignalType::GAL_E1};
        case GNSSSystem::BeiDou: return {SignalType::BDS_B1I, SignalType::BDS_B1C};
        case GNSSSystem::QZSS: return {SignalType::QZS_L1CA};
        default: return {};
    }
}

std::vector<SignalType> secondarySignals(GNSSSystem system) {
    switch (system) {
        case GNSSSystem::GPS: return {SignalType::GPS_L2P, SignalType::GPS_L2C, SignalType::GPS_L5};
        case GNSSSystem::GLONASS: return {SignalType::GLO_L2CA, SignalType::GLO_L2P};
        case GNSSSystem::Galileo: return {SignalType::GAL_E5A, SignalType::GAL_E5B, SignalType::GAL_E6};
        case GNSSSystem::BeiDou: return {SignalType::BDS_B2I, SignalType::BDS_B2A, SignalType::BDS_B3I};
        case GNSSSystem::QZSS: return {SignalType::QZS_L2C, SignalType::QZS_L5};
        default: return {};
    }
}

bool estimatePrimaryIonosphereDelaySeedMeters(
    const Observation& primary,
    const Observation& secondary,
    const Ephemeris* eph,
    double& ionosphere_delay_m);

const Observation* findObservationForSignals(const ObservationData& obs,
                                             const SatelliteId& sat,
                                             const std::vector<SignalType>& candidates) {
    for (const auto signal : candidates) {
        const Observation* candidate = obs.getObservation(sat, signal);
        if (candidate == nullptr || !usablePseudorangeObservation(*candidate)) {
            continue;
        }
        return candidate;
    }
    return nullptr;
}

bool estimatePrimaryIonosphereDelaySeedMeters(
    const ObservationData& obs,
    const NavigationData& nav,
    const SatelliteId& sat,
    double& ionosphere_delay_m) {
    const Observation* primary = findObservationForSignals(obs, sat, primarySignals(sat.system));
    const Observation* secondary = findObservationForSignals(obs, sat, secondarySignals(sat.system));
    if (primary == nullptr || secondary == nullptr) {
        return false;
    }
    const Ephemeris* eph = nav.getEphemeris(sat, obs.time);
    return estimatePrimaryIonosphereDelaySeedMeters(
        *primary, *secondary, eph, ionosphere_delay_m);
}

bool estimatePrimaryIonosphereDelaySeedMeters(
    const Observation& primary,
    const Observation& secondary,
    const Ephemeris* eph,
    double& ionosphere_delay_m) {
    const double f1 = signalFrequencyHz(primary.signal, eph);
    const double f2 = signalFrequencyHz(secondary.signal, eph);
    if (f1 <= 0.0 || f2 <= 0.0 || std::abs(f1 - f2) < 1.0) {
        return false;
    }
    const double denominator = 1.0 - (f1 * f1) / (f2 * f2);
    if (std::abs(denominator) < 1e-12) {
        return false;
    }
    ionosphere_delay_m = (primary.pseudorange - secondary.pseudorange) / denominator;
    return std::isfinite(ionosphere_delay_m);
}

const Observation* findCarrierObservationForSignals(const ObservationData& obs,
                                                    const SatelliteId& sat,
                                                    const std::vector<SignalType>& candidates) {
    return ppp_utils::findCarrierObservation(obs, sat, candidates);
}

double clampDt(double dt) {
    if (!std::isfinite(dt) || dt <= 0.0) {
        return 1.0;
    }
    return std::min(dt, 300.0);
}

bool validReceiverSeed(const Vector3d& receiver_position) {
    return std::isfinite(receiver_position.x()) &&
           std::isfinite(receiver_position.y()) &&
           std::isfinite(receiver_position.z()) &&
           receiver_position.norm() > 1000.0;
}

bool isUsablePppSatellite(const SatelliteId& satellite) {
    return !signal_policy::isBeiDouGeoSatellite(satellite);
}

ReceiverClockBiasGroup receiverClockBiasGroupForSatellite(const SatelliteId& satellite) {
    switch (satellite.system) {
        case GNSSSystem::GPS:
        case GNSSSystem::QZSS:
            return ReceiverClockBiasGroup::GPS;
        case GNSSSystem::GLONASS:
            return ReceiverClockBiasGroup::GLONASS;
        case GNSSSystem::Galileo:
            return ReceiverClockBiasGroup::Galileo;
        case GNSSSystem::BeiDou:
            if (signal_policy::isBeiDou2Satellite(satellite)) {
                return ReceiverClockBiasGroup::BeiDou2;
            }
            if (signal_policy::isBeiDou3Satellite(satellite)) {
                return ReceiverClockBiasGroup::BeiDou3;
            }
            return ReceiverClockBiasGroup::BeiDou;
        case GNSSSystem::NavIC:
            return ReceiverClockBiasGroup::NavIC;
        default:
            return ReceiverClockBiasGroup::UNKNOWN;
    }
}

double seededReceiverClockBiasMeters(const PositionSolution* seed_solution,
                                     ReceiverClockBiasGroup group,
                                     double fallback_bias_m) {
    if (seed_solution == nullptr || !seed_solution->isValid()) {
        return fallback_bias_m;
    }
    const auto exact_it = seed_solution->receiver_clock_biases_m.find(group);
    if (exact_it != seed_solution->receiver_clock_biases_m.end() &&
        std::isfinite(exact_it->second)) {
        return exact_it->second;
    }
    if (group == ReceiverClockBiasGroup::BeiDou2 || group == ReceiverClockBiasGroup::BeiDou3) {
        const auto bds_it = seed_solution->receiver_clock_biases_m.find(ReceiverClockBiasGroup::BeiDou);
        if (bds_it != seed_solution->receiver_clock_biases_m.end() &&
            std::isfinite(bds_it->second)) {
            return bds_it->second;
        }
    }
    if (group == ReceiverClockBiasGroup::GPS && std::isfinite(seed_solution->receiver_clock_bias)) {
        return seed_solution->receiver_clock_bias;
    }
    return fallback_bias_m;
}

double safeVariance(double variance, double floor_value) {
    if (!std::isfinite(variance) || variance <= 0.0) {
        return floor_value;
    }
    return std::max(variance, floor_value);
}

double rtklibSystemErrorFactor(GNSSSystem system) {
    switch (system) {
        case GNSSSystem::GLONASS:
            return kRtkLibGlonassErrorFactor;
        case GNSSSystem::SBAS:
            return kRtkLibSbasErrorFactor;
        default:
            return 1.0;
    }
}

bool usesGpsL5ErrorFactor(SignalType signal) {
    return signal == SignalType::GPS_L5 || signal == SignalType::QZS_L5;
}

double wrapFractionalCycle(double value) {
    if (!std::isfinite(value)) {
        return 0.0;
    }
    double wrapped = std::fmod(value + 0.5, 1.0);
    if (wrapped < 0.0) {
        wrapped += 1.0;
    }
    return wrapped - 0.5;
}

GNSSTime normalizeGpsWeekTow(int week, double tow) {
    while (tow < 0.0) {
        tow += constants::SECONDS_PER_WEEK;
        --week;
    }
    while (tow >= constants::SECONDS_PER_WEEK) {
        tow -= constants::SECONDS_PER_WEEK;
        ++week;
    }
    return GNSSTime(week, tow);
}

GNSSTime alignSsrTimeToNavigationWeek(const NavigationData& nav,
                                      const SatelliteId& satellite,
                                      const GNSSTime& ssr_time) {
    const auto eph_it = nav.ephemeris_data.find(satellite);
    if (eph_it == nav.ephemeris_data.end() || eph_it->second.empty()) {
        return ssr_time;
    }
    const auto& ephemerides = eph_it->second;

    GNSSTime best_time = ssr_time;
    double best_age = std::numeric_limits<double>::infinity();
    for (const auto& eph : ephemerides) {
        int candidate_week = eph.toe.week != 0 ? eph.toe.week : static_cast<int>(eph.week);
        if (candidate_week == 0) {
            candidate_week = ssr_time.week;
        }
        GNSSTime candidate = normalizeGpsWeekTow(candidate_week, ssr_time.tow);
        const double dt = candidate - eph.toe;
        if (dt < -constants::SECONDS_PER_WEEK / 2.0) {
            candidate = normalizeGpsWeekTow(candidate.week + 1, candidate.tow);
        } else if (dt > constants::SECONDS_PER_WEEK / 2.0) {
            candidate = normalizeGpsWeekTow(candidate.week - 1, candidate.tow);
        }

        const double age = std::abs(candidate - eph.toe);
        if (age < best_age) {
            best_age = age;
            best_time = candidate;
        }
    }
    return best_time;
}

Vector3d ssrRacToEcef(const Vector3d& position_ecef,
                      const Vector3d& velocity_ecef,
                      const Vector3d& rac_correction) {
    return ppp_utils::ssrRacToEcef(position_ecef, velocity_ecef, rac_correction);
}

void mergeRtcmSsrCorrection(const io::RTCMSSRCorrection& input,
                            io::RTCMSSRCorrection& merged) {
    merged.satellite = input.satellite;
    merged.time = input.time;
    merged.update_interval_seconds = input.update_interval_seconds;
    merged.issue_of_data = input.issue_of_data;
    merged.provider_id = input.provider_id;
    merged.solution_id = input.solution_id;
    merged.reference_datum = input.reference_datum;
    if (input.iode >= 0) {
        merged.iode = input.iode;
    }
    if (input.iodcrc >= 0) {
        merged.iodcrc = input.iodcrc;
    }
    if (input.has_orbit) {
        merged.orbit_delta_rac_m = input.orbit_delta_rac_m;
        merged.orbit_rate_rac_mps = input.orbit_rate_rac_mps;
        merged.has_orbit = true;
    }
    if (input.has_clock) {
        merged.clock_delta_poly = input.clock_delta_poly;
        merged.has_clock = true;
    }
    if (input.has_code_bias) {
        for (const auto& [signal_id, bias_m] : input.code_bias_m) {
            merged.code_bias_m[signal_id] = bias_m;
        }
        merged.has_code_bias = !merged.code_bias_m.empty();
    }
    if (input.has_ura) {
        merged.ura_index = input.ura_index;
        merged.ura_sigma_m = input.ura_sigma_m;
        merged.has_ura = true;
    }
    if (input.has_high_rate_clock) {
        merged.high_rate_clock_m = input.high_rate_clock_m;
        merged.has_high_rate_clock = true;
    }
}

}  // namespace

double PPPProcessor::modeledZenithTroposphereDelayMeters(
    const Vector3d& receiver_position,
    const GNSSTime& time) {
    return modeledZenithTroposphereDelayMetersImpl(receiver_position, time);
}

PPPProcessor::PPPProcessor() : spp_processor_() {
    reset();
}

PPPProcessor::PPPProcessor(const PPPConfig& ppp_config)
    : ppp_config_(ppp_config), spp_processor_() {
    reset();
}

bool PPPProcessor::initialize(const ProcessorConfig& config) {
    config_ = config;
    ppp_config_.use_precise_orbits = config.use_precise_orbits;
    ppp_config_.use_precise_clocks = config.use_precise_clocks;
    if (ppp_config_.orbit_file_path.empty()) {
        ppp_config_.orbit_file_path = config.orbit_file_path;
    }
    if (ppp_config_.clock_file_path.empty()) {
        ppp_config_.clock_file_path = config.clock_file_path;
    }

    spp_processor_.initialize(config);
    reset();

    receiver_antex_offsets_.clear();
    satellite_antex_offsets_.clear();
    receiver_antex_loaded_ = false;
    if (!ppp_config_.antex_file_path.empty()) {
        receiver_antex_loaded_ =
            loadReceiverAntexOffsets(ppp_config_.antex_file_path, receiver_antex_offsets_);
        loadSatelliteAntexOffsets(ppp_config_.antex_file_path, satellite_antex_offsets_);
        if (!receiver_antex_loaded_ && satellite_antex_offsets_.empty()) {
            return false;
        }
    }

    ocean_loading_loaded_ = false;
    ocean_loading_coefficients_ = OceanLoadingCoefficients{};
    if (ppp_config_.apply_ocean_loading && !ppp_config_.ocean_loading_file_path.empty()) {
        ocean_loading_loaded_ = loadOceanLoadingCoefficients(
            ppp_config_.ocean_loading_file_path,
            ppp_config_.ocean_loading_station_name,
            ocean_loading_coefficients_.up_amplitudes_m,
            ocean_loading_coefficients_.west_amplitudes_m,
            ocean_loading_coefficients_.south_amplitudes_m,
            ocean_loading_coefficients_.up_phases_deg,
            ocean_loading_coefficients_.west_phases_deg,
            ocean_loading_coefficients_.south_phases_deg);
        if (!ocean_loading_loaded_) {
            return false;
        }
    }

    if ((ppp_config_.use_precise_orbits && !ppp_config_.orbit_file_path.empty()) ||
        (ppp_config_.use_precise_clocks && !ppp_config_.clock_file_path.empty())) {
        loadPreciseProducts(ppp_config_.orbit_file_path, ppp_config_.clock_file_path);
    }
    if (ppp_config_.use_ssr_corrections && !ppp_config_.ssr_file_path.empty()) {
        loadSSRProducts(ppp_config_.ssr_file_path);
    }
    if (!ppp_config_.ionex_file_path.empty() &&
        !loadIONEXProducts(ppp_config_.ionex_file_path)) {
        return false;
    }
    if (!ppp_config_.dcb_file_path.empty() &&
        !loadDCBProducts(ppp_config_.dcb_file_path)) {
        return false;
    }
    return true;
}

PositionSolution PPPProcessor::processEpochStandard(
    const ObservationData& obs,
    const NavigationData& nav,
    const char* clas_hybrid_fallback_reason) {
    const auto processing_start = std::chrono::steady_clock::now();

    PositionSolution solution;
    solution.time = obs.time;
    solution.status = SolutionStatus::NONE;
    last_clas_hybrid_fallback_used_ = clas_hybrid_fallback_reason != nullptr;
    last_clas_hybrid_fallback_reason_ =
        clas_hybrid_fallback_reason != nullptr ? clas_hybrid_fallback_reason : "";
    last_ar_ratio_ = 0.0;
    last_fixed_ambiguities_ = 0;
    last_applied_atmos_trop_corrections_ = 0;
    last_applied_atmos_iono_corrections_ = 0;
    last_applied_atmos_trop_m_ = 0.0;
    last_applied_atmos_iono_m_ = 0.0;
    last_applied_ionex_corrections_ = 0;
    last_applied_dcb_corrections_ = 0;
    last_applied_ionex_m_ = 0.0;
    last_applied_dcb_m_ = 0.0;
    last_ssr_application_diagnostics_.clear();
    last_filter_iteration_diagnostics_.clear();
    last_residual_diagnostics_.clear();
    PositionSolution seed_solution;
    const bool use_seed_assist = useLowDynamicsBroadcastSeedAssist();
    const bool need_seed_solution =
        ppp_config_.reset_clock_to_spp_each_epoch ||
        (ppp_config_.kinematic_mode && ppp_config_.reset_kinematic_position_to_spp_each_epoch) ||
        use_seed_assist ||
        !filter_initialized_;
    const PositionSolution* seed_ptr = nullptr;

    try {
        if (need_seed_solution) {
            seed_solution = spp_processor_.processEpoch(obs, nav);
            if (seed_solution.isValid()) {
                seed_ptr = &seed_solution;
            }
        }
        if (!filter_initialized_) {
            if (!initializeFilter(obs, nav, seed_ptr)) {
                solution = seed_ptr != nullptr ? seed_solution : spp_processor_.processEpoch(obs, nav);
            } else {
                filter_initialized_ = true;
                convergence_start_time_ = obs.time;
            }
        }

        if (filter_initialized_) {
            const double dt = has_last_processed_time_ ? clampDt(obs.time - last_processed_time_) : 1.0;
            detectCycleSlips(obs);
            predictState(dt, seed_ptr);

            bool updated = updateFilter(obs, nav);
            if (!updated && ppp_config_.enable_ambiguity_resolution) {
                bool had_fixed_ambiguities = false;
                for (auto& [satellite, ambiguity] : ambiguity_states_) {
                    if (!ambiguity.is_fixed) {
                        continue;
                    }
                    had_fixed_ambiguities = true;
                    resetAmbiguity(satellite, SignalType::GPS_L1CA);
                }
                if (had_fixed_ambiguities) {
                    if (pppDebugEnabled()) {
                        std::cerr << "[PPP-AR] retry update after resetting fixed ambiguities\n";
                    }
                    updated = updateFilter(obs, nav);
                }
            }

            if (updated) {
                const auto if_obs = formIonosphereFree(obs, nav);
                auto corrected_if_obs = if_obs;
                applyPreciseCorrections(corrected_if_obs, nav, obs.time);
                checkConvergence(obs.time);
                PositionSolution float_solution = generateSolution(obs.time, corrected_if_obs);
                float_solution.status = SolutionStatus::PPP_FLOAT;
                float_solution.ratio = 0.0;
                float_solution.num_fixed_ambiguities = 0;
                const PPPState float_filter_state = filter_state_;
                const auto float_ambiguity_states = ambiguity_states_;
                const auto float_phase_ambiguity_admission_offsets =
                    phase_ambiguity_admission_offsets_m_;
                const bool fixed =
                    ppp_config_.enable_ambiguity_resolution && resolveAmbiguities(obs, nav);
                solution = fixed ? generateSolution(obs.time, corrected_if_obs) : float_solution;
                solution.status = fixed ? SolutionStatus::PPP_FIXED : SolutionStatus::PPP_FLOAT;
                solution.ratio = fixed ? last_ar_ratio_ : 0.0;
                solution.num_fixed_ambiguities = fixed ? last_fixed_ambiguities_ : 0;
                // For WLNL fix: override position with WLS using fixed NL ambiguities
                const double max_fixed_position_jump_m =
                    ppp_config_.kinematic_mode ? 25.0 : 10.0;
                bool accepted_fixed_solution = fixed;
                if (fixed &&
                    (solution.position_ecef - float_solution.position_ecef).norm() >
                        max_fixed_position_jump_m) {
                    accepted_fixed_solution = false;
                    if (pppDebugEnabled()) {
                        std::cerr << "[PPP-AR] reject fixed filter-state jump="
                                  << (solution.position_ecef - float_solution.position_ecef).norm()
                                  << "\n";
                    }
                }
                if (fixed && ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL) {
                    Vector3d fixed_position;
                    if (solveFixedPosition(obs, nav, fixed_position)) {
                        if ((fixed_position - float_solution.position_ecef).norm() <= max_fixed_position_jump_m) {
                            solution.position_ecef = fixed_position;
                        } else if (pppDebugEnabled()) {
                            accepted_fixed_solution = false;
                            std::cerr << "[PPP-AR] reject WLNL fixed position jump="
                                      << (fixed_position - float_solution.position_ecef).norm()
                                      << "\n";
                        } else {
                            accepted_fixed_solution = false;
                        }
                    }
                }
                if (!accepted_fixed_solution && fixed) {
                    solution = float_solution;
                    solution.status = SolutionStatus::PPP_FLOAT;
                    solution.ratio = 0.0;
                    solution.num_fixed_ambiguities = 0;
                }
                if (accepted_fixed_solution) {
                    double latitude = 0.0;
                    double longitude = 0.0;
                    double height = 0.0;
                    ecef2geodetic(solution.position_ecef, latitude, longitude, height);
                    solution.position_geodetic = GeodeticCoord(latitude, longitude, height);
                }
                had_fixed_last_epoch_ = accepted_fixed_solution;
                if (fixed && !accepted_fixed_solution) {
                    filter_state_ = float_filter_state;
                    ambiguity_states_ = float_ambiguity_states;
                    phase_ambiguity_admission_offsets_m_ =
                        float_phase_ambiguity_admission_offsets;
                } else if (accepted_fixed_solution && ppp_config_.ar_method != PPPConfig::ARMethod::DD_WLNL) {
                    // For DD_IFLC/DD_PER_FREQ: revert to float state to avoid
                    // poisoning later epochs with a bad fix.
                    filter_state_ = float_filter_state;
                    ambiguity_states_ = float_ambiguity_states;
                    phase_ambiguity_admission_offsets_m_ =
                        float_phase_ambiguity_admission_offsets;
                }
                // For DD_WLNL: keep holdamb-updated state — the constraint
                // is applied via Kalman update and propagates naturally.
            } else if (!solution.isValid()) {
                if (pppDebugEnabled()) {
                    std::cerr << "[PPP] updateFilter returned false at week=" << obs.time.week
                              << " tow=" << obs.time.tow << "\n";
                }
                if (use_seed_assist) {
                    recoverLowDynamicsBroadcastState(obs, seed_ptr);
                }
                solution = seed_ptr != nullptr ? seed_solution : spp_processor_.processEpoch(obs, nav);
            }
        }
    } catch (const std::exception& e) {
        if (pppDebugEnabled()) {
            std::cerr << "[PPP] exception at week=" << obs.time.week
                      << " tow=" << obs.time.tow << ": " << e.what() << "\n";
        }
        solution = seed_ptr != nullptr ? seed_solution : spp_processor_.processEpoch(obs, nav);
    }

    has_last_processed_time_ = true;
    last_processed_time_ = obs.time;

    const auto processing_end = std::chrono::steady_clock::now();
    solution.processing_time_ms =
        std::chrono::duration<double, std::milli>(processing_end - processing_start).count();
    updateStatistics(solution.isValid());
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        total_convergence_time_ += solution.processing_time_ms;
    }
    return solution;
}

PositionSolution PPPProcessor::processEpoch(const ObservationData& obs, const NavigationData& nav) {
    if (ppp_config_.use_clas_osr_filter) {
        return processEpochCLAS(obs, nav);
    }

    // Standard PPP path: precise/broadcast navigation with optional inline
    // OSR corrections. Experiment arms use this as the stable control and as
    // the hybrid fallback target for CLAS epoch-policy trials.
    return processEpochStandard(obs, nav);
}

ProcessorStats PPPProcessor::getStats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ProcessorStats stats;
    stats.total_epochs = total_epochs_processed_;
    stats.valid_solutions = converged_solutions_;
    if (total_epochs_processed_ > 0) {
        stats.average_processing_time_ms = total_convergence_time_ / total_epochs_processed_;
    }
    return stats;
}

void PPPProcessor::reset() {
    filter_initialized_ = false;
    converged_ = false;
    convergence_time_ = 0.0;
    filter_state_ = PPPState{};
    ambiguity_states_.clear();
    phase_ambiguity_admission_offsets_m_.clear();
    clas_dispersion_compensation_.clear();
    clas_sis_continuity_.clear();
    clas_phase_bias_repair_.clear();
    windup_cache_.clear();
    recent_positions_.clear();
    has_last_processed_time_ = false;
    last_processed_time_ = GNSSTime();
    precise_products_loaded_ = !precise_products_.orbit_clock_data.empty();
    ssr_products_loaded_ = !ssr_products_.orbit_clock_corrections.empty();
    ionex_products_loaded_ = !ionex_products_.tec_maps.empty();
    dcb_products_loaded_ = !dcb_products_.entries.empty();
    ocean_loading_loaded_ = false;
    ocean_loading_coefficients_ = OceanLoadingCoefficients{};
    static_anchor_position_.setZero();
    has_static_anchor_position_ = false;
    last_ar_ratio_ = 0.0;
    last_fixed_ambiguities_ = 0;
    last_applied_atmos_trop_corrections_ = 0;
    last_applied_atmos_iono_corrections_ = 0;
    last_applied_atmos_trop_m_ = 0.0;
    last_applied_atmos_iono_m_ = 0.0;
    last_applied_ionex_corrections_ = 0;
    last_applied_dcb_corrections_ = 0;
    last_applied_ionex_m_ = 0.0;
    last_applied_dcb_m_ = 0.0;
    last_ssr_application_diagnostics_.clear();
    last_filter_iteration_diagnostics_.clear();
    last_residual_diagnostics_.clear();

    std::lock_guard<std::mutex> lock(stats_mutex_);
    total_epochs_processed_ = 0;
    converged_solutions_ = 0;
    total_convergence_time_ = 0.0;
}

bool PPPProcessor::loadPreciseProducts(const std::string& orbit_file, const std::string& clock_file) {
    precise_products_.clear();

    bool loaded_any = false;
    if (!orbit_file.empty()) {
        loaded_any = precise_products_.loadSP3File(orbit_file) || loaded_any;
    }
    if (!clock_file.empty()) {
        loaded_any = precise_products_.loadClockFile(clock_file) || loaded_any;
    }
    precise_products_loaded_ = loaded_any;
    return precise_products_loaded_;
}

bool PPPProcessor::loadSSRProducts(const std::string& ssr_file) {
    ssr_products_.clear();
    if (ssr_file.empty()) {
        ssr_products_loaded_ = false;
        return false;
    }
    // Auto-detect L6 binary vs CSV based on file content
    if (ssr_file.size() >= 4) {
        std::ifstream probe(ssr_file, std::ios::binary);
        uint32_t magic = 0;
        if (probe.read(reinterpret_cast<char*>(&magic), 4)) {
            // L6 preamble in big-endian: 0x1ACFFC1D
            const uint32_t preamble =
                ((magic & 0xFF) << 24) | (((magic >> 8) & 0xFF) << 16) |
                (((magic >> 16) & 0xFF) << 8) | ((magic >> 24) & 0xFF);
            if (preamble == 0x1ACFFC1D) {
                return loadL6Products(ssr_file);
            }
        }
    }
    ssr_products_loaded_ = ssr_products_.loadCSVFile(ssr_file);
    return ssr_products_loaded_;
}

bool PPPProcessor::loadSSRProducts(const SSRProducts& products) {
    ssr_products_ = products;
    ssr_products_loaded_ = ssr_products_.orbit_clock_corrections.size() > 0U;
    return ssr_products_loaded_;
}

bool PPPProcessor::loadL6Products(const std::string& l6_file) {
    // Strategy: use Python expander to convert L6 → expanded CSV,
    // then load via the battle-tested CSV loader (0.14m accuracy).
    // Falls back to C++ native L6 decoder if Python is unavailable.
    int gps_week = ppp_config_.l6_gps_week > 0 ? ppp_config_.l6_gps_week : last_obs_gps_week_;
    if (gps_week <= 0) gps_week = 2068;

    // Try Python expander first
    const std::string tmp_csv = "/tmp/gnsspp_l6_expanded_" +
        std::to_string(std::hash<std::string>{}(l6_file)) + ".csv";
    const std::string cmd =
        "python3 -c \""
        "import sys; sys.path.insert(0, 'apps'); "
        "from pathlib import Path; "
        "from gnss_clas_ppp import expand_qzss_l6_source; "
        "expand_qzss_l6_source('" + l6_file + "', " +
        std::to_string(gps_week) + ", Path('" + tmp_csv + "'))\" 2>/dev/null";

    const int ret = std::system(cmd.c_str());
    if (ret == 0) {
        ssr_products_loaded_ = ssr_products_.loadCSVFile(tmp_csv);
        if (ssr_products_loaded_) {
            return true;
        }
    }

    // Fallback: C++ native L6 decoder
    qzss_l6::L6Decoder decoder;
    auto epochs = decoder.decodeFile(l6_file, gps_week);
    if (epochs.empty()) {
        ssr_products_loaded_ = false;
        return false;
    }
    int best_network = 0;
    if (ppp_config_.approximate_position.norm() > 1e3) {
        double lat = 0, lon = 0, h = 0;
        ecef2geodetic(ppp_config_.approximate_position, lat, lon, h);
        lat *= 180.0 / M_PI;
        lon *= 180.0 / M_PI;
        if (lat > 34 && lat < 38 && lon > 138 && lon < 142)
            best_network = 7;
    }
    qzss_l6::populateSSRProducts(epochs, ssr_products_, best_network);
    ssr_products_loaded_ = true;
    return true;
}

bool PPPProcessor::loadIONEXProducts(const std::string& ionex_file) {
    ionex_products_.clear();
    if (ionex_file.empty()) {
        ionex_products_loaded_ = false;
        return false;
    }
    ionex_products_loaded_ = ionex_products_.loadIONEXFile(ionex_file);
    return ionex_products_loaded_;
}

bool PPPProcessor::loadDCBProducts(const std::string& dcb_file) {
    dcb_products_.clear();
    if (dcb_file.empty()) {
        dcb_products_loaded_ = false;
        return false;
    }
    dcb_products_loaded_ = dcb_products_.loadFile(dcb_file);
    return dcb_products_loaded_;
}

bool PPPProcessor::loadRTCMSSRProducts(const std::string& rtcm_file,
                                       const NavigationData& nav,
                                       double sample_step_seconds) {
    ssr_products_.clear();
    if (rtcm_file.empty() || sample_step_seconds <= 0.0) {
        ssr_products_loaded_ = false;
        return false;
    }

    io::RTCMReader reader;
    if (!reader.open(rtcm_file)) {
        ssr_products_loaded_ = false;
        return false;
    }

    io::RTCMProcessor rtcm_processor;
    std::map<SatelliteId, io::RTCMSSRCorrection> pending_corrections;
    io::RTCMMessage message;
    size_t sampled_corrections = 0;

    while (reader.readMessage(message)) {
        std::vector<io::RTCMSSRCorrection> decoded_corrections;
        if (!rtcm_processor.decodeSSRCorrections(message, decoded_corrections)) {
            continue;
        }

        for (const auto& correction : decoded_corrections) {
            io::RTCMSSRCorrection aligned_correction = correction;
            aligned_correction.time =
                alignSsrTimeToNavigationWeek(nav, aligned_correction.satellite, aligned_correction.time);

            auto& merged = pending_corrections[aligned_correction.satellite];
            const bool same_group =
                merged.satellite == aligned_correction.satellite &&
                std::abs(merged.time - aligned_correction.time) < 1e-6 &&
                merged.issue_of_data == aligned_correction.issue_of_data &&
                merged.provider_id == aligned_correction.provider_id &&
                merged.solution_id == aligned_correction.solution_id;
            if (!same_group) {
                merged = io::RTCMSSRCorrection{};
            }
            mergeRtcmSsrCorrection(aligned_correction, merged);
            if (!merged.has_orbit || !merged.has_clock) {
                continue;
            }

            Vector3d base_position = Vector3d::Zero();
            Vector3d base_velocity = Vector3d::Zero();
            double base_clock_bias = 0.0;
            double base_clock_drift = 0.0;
            if (!nav.calculateSatelliteState(
                    merged.satellite,
                    merged.time,
                    base_position,
                    base_velocity,
                    base_clock_bias,
                    base_clock_drift)) {
                continue;
            }

            const double update_interval =
                merged.update_interval_seconds > 0.0 ?
                    merged.update_interval_seconds :
                    sample_step_seconds;
            const int sample_count = std::max(
                1,
                static_cast<int>(std::floor(update_interval / sample_step_seconds + 1e-9)));
            for (int sample_index = 0; sample_index < sample_count; ++sample_index) {
                const double dt = sample_step_seconds * static_cast<double>(sample_index);
                const GNSSTime sample_time = merged.time + dt;
                Vector3d position = base_position;
                Vector3d velocity = base_velocity;
                double clock_bias = 0.0;
                double clock_drift = 0.0;
                if (!nav.calculateSatelliteState(
                        merged.satellite,
                        sample_time,
                        position,
                        velocity,
                        clock_bias,
                        clock_drift)) {
                    continue;
                }

                SSROrbitClockCorrection sampled;
                sampled.satellite = merged.satellite;
                sampled.time = sample_time;
                if (merged.iode >= 0) {
                    sampled.orbit_iode = merged.iode;
                }
                sampled.orbit_correction_ecef = ssrRacToEcef(
                    position,
                    velocity,
                    merged.orbit_delta_rac_m + merged.orbit_rate_rac_mps * dt);
                sampled.clock_correction_m =
                    merged.clock_delta_poly.x() +
                    merged.clock_delta_poly.y() * dt +
                    merged.clock_delta_poly.z() * dt * dt;
                if (merged.has_high_rate_clock) {
                    sampled.clock_correction_m += merged.high_rate_clock_m;
                }
                if (merged.has_ura) {
                    sampled.ura_sigma_m = merged.ura_sigma_m;
                    sampled.ura_valid = true;
                }
                if (merged.has_code_bias) {
                    sampled.code_bias_m = merged.code_bias_m;
                    sampled.code_bias_valid = !sampled.code_bias_m.empty();
                }
                sampled.orbit_valid = true;
                sampled.clock_valid = true;
                ssr_products_.addCorrection(sampled);
                ++sampled_corrections;
            }
        }
    }

    ssr_products_loaded_ = sampled_corrections > 0U;
    return ssr_products_loaded_;
}

bool PPPProcessor::interpolateLoadedSSRCorrection(const SatelliteId& sat,
                                                  const GNSSTime& time,
                                                  Vector3d& orbit_correction_ecef,
                                                  double& clock_correction_m,
                                                  double* ura_sigma_m,
                                                  std::map<uint8_t, double>* code_bias_m,
                                                  std::map<uint8_t, double>* phase_bias_m,
                                                  std::map<std::string, std::string>* atmos_tokens) const {
    if (!ssr_products_loaded_) {
        orbit_correction_ecef.setZero();
        clock_correction_m = 0.0;
        if (ura_sigma_m != nullptr) {
            *ura_sigma_m = 0.0;
        }
        if (code_bias_m != nullptr) {
            code_bias_m->clear();
        }
        if (phase_bias_m != nullptr) {
            phase_bias_m->clear();
        }
        if (atmos_tokens != nullptr) {
            atmos_tokens->clear();
        }
        return false;
    }
    return ssr_products_.interpolateCorrection(
        sat,
        time,
        orbit_correction_ecef,
        clock_correction_m,
        ura_sigma_m,
        code_bias_m,
        phase_bias_m,
        atmos_tokens,
        nullptr,
        nullptr,
        nullptr,
        0,
        ppp_config_.allow_future_ssr_corrections,
        ppp_config_.require_ssr_orbit_clock);
}

bool PPPProcessor::initializeFilter(const ObservationData& obs,
                                    const NavigationData& nav,
                                    const PositionSolution* seed_solution) {
    Vector3d initial_position = Vector3d::Zero();
    double initial_clock_bias_m = 0.0;
    PositionSolution generated_spp_solution;
    const PositionSolution* clock_seed_solution =
        seed_solution != nullptr && seed_solution->isValid() ? seed_solution : nullptr;
    if (ppp_config_.prefer_receiver_position_seed &&
        validReceiverSeed(obs.receiver_position)) {
        initial_position = obs.receiver_position;
        initial_clock_bias_m =
            clock_seed_solution != nullptr ?
                clock_seed_solution->receiver_clock_bias :
                obs.receiver_clock_bias * constants::SPEED_OF_LIGHT;
    } else if (clock_seed_solution != nullptr) {
        initial_position = clock_seed_solution->position_ecef;
        initial_clock_bias_m = clock_seed_solution->receiver_clock_bias;
    } else {
        generated_spp_solution = spp_processor_.processEpoch(obs, nav);
        if (generated_spp_solution.isValid()) {
            clock_seed_solution = &generated_spp_solution;
            initial_position = generated_spp_solution.position_ecef;
            initial_clock_bias_m = generated_spp_solution.receiver_clock_bias;
        } else if (validReceiverSeed(obs.receiver_position)) {
            initial_position = obs.receiver_position;
            initial_clock_bias_m = obs.receiver_clock_bias * constants::SPEED_OF_LIGHT;
        } else {
            return false;
        }
    }

    // Allocate ISB states for non-GPS systems when estimate_ionosphere is on
    int base_states = 9;  // pos(3) + vel(3) + gps_clk(1) + glo_clk(1) + trop(1)
    filter_state_.gal_clock_index = -1;
    filter_state_.qzs_clock_index = -1;
    filter_state_.bds_clock_index = -1;
    filter_state_.bds2_clock_index = -1;
    filter_state_.bds3_clock_index = -1;
    if (ppp_config_.estimate_ionosphere) {
        std::set<ReceiverClockBiasGroup> visible_clock_groups;
        for (const auto& sat : obs.getSatellites()) {
            if (isUsablePppSatellite(sat)) {
                visible_clock_groups.insert(receiverClockBiasGroupForSatellite(sat));
            }
        }
        if (false && visible_clock_groups.count(ReceiverClockBiasGroup::Galileo)) {
            filter_state_.gal_clock_index = base_states++;
        }
        // QZSS stays tied to the GPS receiver clock in the current model.
        if (false && visible_clock_groups.count(ReceiverClockBiasGroup::QZSS)) {
            filter_state_.qzs_clock_index = base_states++;
        }
        if (visible_clock_groups.count(ReceiverClockBiasGroup::BeiDou2)) {
            filter_state_.bds2_clock_index = base_states++;
        }
        if (visible_clock_groups.count(ReceiverClockBiasGroup::BeiDou3)) {
            filter_state_.bds3_clock_index = base_states++;
        }
    }
    filter_state_.trop_index = base_states > 9 ? base_states - 1 : 8;
    // Actually keep trop at its original position and shift ISB before it
    // Simpler: put ISB after trop
    filter_state_.trop_index = 8;  // keep original
    int isb_start = 9;
    if (ppp_config_.estimate_ionosphere) {
        if (filter_state_.gal_clock_index >= 0) filter_state_.gal_clock_index = isb_start++;
        if (filter_state_.qzs_clock_index >= 0) filter_state_.qzs_clock_index = isb_start++;
        if (filter_state_.bds_clock_index >= 0) filter_state_.bds_clock_index = isb_start++;
        if (filter_state_.bds2_clock_index >= 0) filter_state_.bds2_clock_index = isb_start++;
        if (filter_state_.bds3_clock_index >= 0) filter_state_.bds3_clock_index = isb_start++;
    }

    std::map<SatelliteId, double> initial_ionosphere_delay_seeds_m;
    if (ppp_config_.estimate_ionosphere && !ppp_config_.use_ionosphere_free) {
        for (const auto& sat : obs.getSatellites()) {
            if (!isUsablePppSatellite(sat)) {
                continue;
            }
            double ionosphere_delay_m = 0.0;
            if (estimatePrimaryIonosphereDelaySeedMeters(
                    obs, nav, sat, ionosphere_delay_m)) {
                initial_ionosphere_delay_seeds_m[sat] = ionosphere_delay_m;
            }
        }
    }

    // Reserve ionosphere states for visible satellites
    int n_iono_states = 0;
    filter_state_.ionosphere_indices.clear();
    if (ppp_config_.estimate_ionosphere) {
        filter_state_.iono_index = isb_start;
        for (const auto& sat : obs.getSatellites()) {
            if (!isUsablePppSatellite(sat)) {
                continue;
            }
            filter_state_.ionosphere_indices[sat] = filter_state_.iono_index + n_iono_states;
            ++n_iono_states;
        }
    }
    const int n_isb = isb_start - 9;
    const int n_states = 9 + n_isb + n_iono_states;
    filter_state_.amb_index = 9 + n_isb + n_iono_states;
    filter_state_.state = VectorXd::Zero(n_states);
    filter_state_.covariance = MatrixXd::Identity(n_states, n_states);
    filter_state_.state.segment(filter_state_.pos_index, 3) = initial_position;
    filter_state_.state.segment(filter_state_.vel_index, 3).setZero();
    filter_state_.state(filter_state_.clock_index) = seededReceiverClockBiasMeters(
        clock_seed_solution, ReceiverClockBiasGroup::GPS, initial_clock_bias_m);
    filter_state_.state(filter_state_.glo_clock_index) = seededReceiverClockBiasMeters(
        clock_seed_solution, ReceiverClockBiasGroup::GLONASS, initial_clock_bias_m);
    // Initialize ISB states
    if (filter_state_.gal_clock_index >= 0)
        filter_state_.state(filter_state_.gal_clock_index) = seededReceiverClockBiasMeters(
            clock_seed_solution, ReceiverClockBiasGroup::Galileo, initial_clock_bias_m);
    if (filter_state_.qzs_clock_index >= 0)
        filter_state_.state(filter_state_.qzs_clock_index) = seededReceiverClockBiasMeters(
            clock_seed_solution, ReceiverClockBiasGroup::GPS, initial_clock_bias_m);
    if (filter_state_.bds_clock_index >= 0)
        filter_state_.state(filter_state_.bds_clock_index) = seededReceiverClockBiasMeters(
            clock_seed_solution, ReceiverClockBiasGroup::BeiDou, initial_clock_bias_m);
    if (filter_state_.bds2_clock_index >= 0)
        filter_state_.state(filter_state_.bds2_clock_index) = seededReceiverClockBiasMeters(
            clock_seed_solution, ReceiverClockBiasGroup::BeiDou2, initial_clock_bias_m);
    if (filter_state_.bds3_clock_index >= 0)
        filter_state_.state(filter_state_.bds3_clock_index) = seededReceiverClockBiasMeters(
            clock_seed_solution, ReceiverClockBiasGroup::BeiDou3, initial_clock_bias_m);
    filter_state_.state(filter_state_.trop_index) =
        ppp_config_.estimate_troposphere ?
            modeledZenithTroposphereDelayMeters(initial_position, obs.time) :
            kDefaultZenithDelayMeters;
    const bool use_broadcast_rtklib_model = !precise_products_loaded_;
    filter_state_.covariance.block(filter_state_.pos_index, filter_state_.pos_index, 3, 3) *=
        use_broadcast_rtklib_model ? ppp_config_.initial_position_variance : 100.0;
    filter_state_.covariance.block(filter_state_.vel_index, filter_state_.vel_index, 3, 3) *=
        use_broadcast_rtklib_model ? ppp_config_.initial_velocity_variance : 25.0;
    filter_state_.covariance(filter_state_.clock_index, filter_state_.clock_index) =
        use_broadcast_rtklib_model ? ppp_config_.initial_clock_variance : 1e8;
    filter_state_.covariance(filter_state_.glo_clock_index, filter_state_.glo_clock_index) =
        use_broadcast_rtklib_model ? ppp_config_.initial_clock_variance : 1e8;
    filter_state_.covariance(filter_state_.trop_index, filter_state_.trop_index) =
        use_broadcast_rtklib_model ? ppp_config_.initial_troposphere_variance : 25.0;
    // ISB covariance
    if (filter_state_.gal_clock_index >= 0)
        filter_state_.covariance(filter_state_.gal_clock_index, filter_state_.gal_clock_index) = 1e8;
    if (filter_state_.qzs_clock_index >= 0)
        filter_state_.covariance(filter_state_.qzs_clock_index, filter_state_.qzs_clock_index) = 1e8;
    if (filter_state_.bds_clock_index >= 0)
        filter_state_.covariance(filter_state_.bds_clock_index, filter_state_.bds_clock_index) = 1e8;
    if (filter_state_.bds2_clock_index >= 0)
        filter_state_.covariance(filter_state_.bds2_clock_index, filter_state_.bds2_clock_index) = 1e8;
    if (filter_state_.bds3_clock_index >= 0)
        filter_state_.covariance(filter_state_.bds3_clock_index, filter_state_.bds3_clock_index) = 1e8;
    // Initialize per-satellite ionosphere states
    if (ppp_config_.estimate_ionosphere) {
        for (const auto& [sat, idx] : filter_state_.ionosphere_indices) {
            const auto seed_it = initial_ionosphere_delay_seeds_m.find(sat);
            filter_state_.state(idx) =
                seed_it != initial_ionosphere_delay_seeds_m.end() ? seed_it->second : 0.0;
            filter_state_.covariance(idx, idx) = ppp_config_.initial_ionosphere_variance;
        }
    }
    filter_state_.total_states = n_states;
    if (!ppp_config_.kinematic_mode || ppp_config_.low_dynamics_mode) {
        static_anchor_position_ = initial_position;
        has_static_anchor_position_ = true;
    }
    constrainStaticVelocityStates();
    constrainStaticAnchorPosition();
    return true;
}

void PPPProcessor::predictState(double dt, const PositionSolution* seed_solution) {
    if (!filter_initialized_) {
        return;
    }

    const bool use_broadcast_rtklib_model = !precise_products_loaded_;
    const bool use_dynamic_prediction =
        ppp_config_.kinematic_mode && (!use_broadcast_rtklib_model || ppp_config_.use_dynamics_model);
    MatrixXd F = MatrixXd::Identity(filter_state_.total_states, filter_state_.total_states);
    if (use_dynamic_prediction) {
        F.block(filter_state_.pos_index, filter_state_.vel_index, 3, 3) =
            MatrixXd::Identity(3, 3) * dt;
    }

    filter_state_.state = F * filter_state_.state;
    if (!ppp_config_.kinematic_mode || !use_dynamic_prediction) {
        filter_state_.state.segment(filter_state_.vel_index, 3).setZero();
    }

    MatrixXd Q = MatrixXd::Zero(filter_state_.total_states, filter_state_.total_states);
    if (!ppp_config_.kinematic_mode) {
        Q.block(filter_state_.pos_index, filter_state_.pos_index, 3, 3) =
            MatrixXd::Identity(3, 3) * ppp_config_.process_noise_position * dt;
    } else if (use_dynamic_prediction) {
        Q.block(filter_state_.pos_index, filter_state_.pos_index, 3, 3) =
            MatrixXd::Identity(3, 3) * ppp_config_.process_noise_position * dt;
    }
    if (use_dynamic_prediction) {
        Q.block(filter_state_.vel_index, filter_state_.vel_index, 3, 3) =
            MatrixXd::Identity(3, 3) * ppp_config_.process_noise_velocity * dt;
    } else {
        Q.block(filter_state_.vel_index, filter_state_.vel_index, 3, 3) =
            MatrixXd::Identity(3, 3) * 1e-12;
    }
    const double clock_process_noise =
        use_broadcast_rtklib_model ? ppp_config_.process_noise_clock : 100.0;
    Q(filter_state_.clock_index, filter_state_.clock_index) = clock_process_noise * dt;
    Q(filter_state_.glo_clock_index, filter_state_.glo_clock_index) = clock_process_noise * dt;
    // ISB process noise (white noise, re-estimated each epoch)
    if (filter_state_.gal_clock_index >= 0)
        Q(filter_state_.gal_clock_index, filter_state_.gal_clock_index) = clock_process_noise * dt;
    if (filter_state_.qzs_clock_index >= 0)
        Q(filter_state_.qzs_clock_index, filter_state_.qzs_clock_index) = clock_process_noise * dt;
    if (filter_state_.bds_clock_index >= 0)
        Q(filter_state_.bds_clock_index, filter_state_.bds_clock_index) = clock_process_noise * dt;
    if (filter_state_.bds2_clock_index >= 0)
        Q(filter_state_.bds2_clock_index, filter_state_.bds2_clock_index) = clock_process_noise * dt;
    if (filter_state_.bds3_clock_index >= 0)
        Q(filter_state_.bds3_clock_index, filter_state_.bds3_clock_index) = clock_process_noise * dt;
    Q(filter_state_.trop_index, filter_state_.trop_index) =
        ppp_config_.process_noise_troposphere * dt;
    // Ionosphere process noise
    if (ppp_config_.estimate_ionosphere) {
        for (const auto& [sat, idx] : filter_state_.ionosphere_indices) {
            (void)sat;
            if (idx >= 0 && idx < filter_state_.total_states) {
                Q(idx, idx) = ppp_config_.process_noise_ionosphere * dt;
            }
        }
    }
    for (const auto& [sat, idx] : filter_state_.ambiguity_indices) {
        (void)sat;
        if (idx >= 0 && idx < filter_state_.total_states) {
            Q(idx, idx) = ppp_config_.process_noise_ambiguity * dt;
        }
    }
    for (const auto& [key, idx] : filter_state_.frequency_ambiguity_indices) {
        (void)key;
        if (idx >= 0 && idx < filter_state_.total_states) {
            Q(idx, idx) = ppp_config_.process_noise_ambiguity * dt;
        }
    }

    filter_state_.covariance = F * filter_state_.covariance * F.transpose() + Q;
    if (use_broadcast_rtklib_model && seed_solution != nullptr && seed_solution->isValid()) {
        if (ppp_config_.reset_clock_to_spp_each_epoch || useLowDynamicsBroadcastSeedAssist()) {
            const double fallback_clock_bias_m = seed_solution->receiver_clock_bias;
            reinitializeScalarState(
                filter_state_.clock_index,
                seededReceiverClockBiasMeters(
                    seed_solution, ReceiverClockBiasGroup::GPS, fallback_clock_bias_m),
                ppp_config_.initial_clock_variance);
            reinitializeScalarState(
                filter_state_.glo_clock_index,
                seededReceiverClockBiasMeters(
                    seed_solution, ReceiverClockBiasGroup::GLONASS, fallback_clock_bias_m),
                ppp_config_.initial_clock_variance);
            if (filter_state_.gal_clock_index >= 0) {
                reinitializeScalarState(
                    filter_state_.gal_clock_index,
                    seededReceiverClockBiasMeters(
                        seed_solution, ReceiverClockBiasGroup::Galileo, fallback_clock_bias_m),
                    ppp_config_.initial_clock_variance);
            }
            if (filter_state_.qzs_clock_index >= 0) {
                reinitializeScalarState(
                    filter_state_.qzs_clock_index,
                    seededReceiverClockBiasMeters(
                        seed_solution, ReceiverClockBiasGroup::GPS, fallback_clock_bias_m),
                    ppp_config_.initial_clock_variance);
            }
            if (filter_state_.bds_clock_index >= 0) {
                reinitializeScalarState(
                    filter_state_.bds_clock_index,
                    seededReceiverClockBiasMeters(
                        seed_solution, ReceiverClockBiasGroup::BeiDou, fallback_clock_bias_m),
                    ppp_config_.initial_clock_variance);
            }
            if (filter_state_.bds2_clock_index >= 0) {
                reinitializeScalarState(
                    filter_state_.bds2_clock_index,
                    seededReceiverClockBiasMeters(
                        seed_solution, ReceiverClockBiasGroup::BeiDou2, fallback_clock_bias_m),
                    ppp_config_.initial_clock_variance);
            }
            if (filter_state_.bds3_clock_index >= 0) {
                reinitializeScalarState(
                    filter_state_.bds3_clock_index,
                    seededReceiverClockBiasMeters(
                        seed_solution, ReceiverClockBiasGroup::BeiDou3, fallback_clock_bias_m),
                    ppp_config_.initial_clock_variance);
            }
        }
        if (ppp_config_.kinematic_mode &&
            !ppp_config_.low_dynamics_mode &&
            !use_dynamic_prediction &&
            ppp_config_.reset_kinematic_position_to_spp_each_epoch) {
            reinitializeVectorState(
                filter_state_.pos_index,
                seed_solution->position_ecef,
                ppp_config_.initial_position_variance);
        }
    }
    constrainStaticVelocityStates();
    constrainStaticAnchorPosition();
}

bool PPPProcessor::updateFilter(const ObservationData& obs, const NavigationData& nav) {
    last_filter_iteration_diagnostics_.clear();
    last_residual_diagnostics_.clear();

    auto if_obs = formIonosphereFree(obs, nav);
    if (if_obs.size() < static_cast<size_t>(config_.min_satellites)) {
        if (pppDebugEnabled()) {
            std::cerr << "[PPP] insufficient IF observations: " << if_obs.size()
                      << " < " << config_.min_satellites << "\n";
        }
        return false;
    }

    ensureIonosphereStates(if_obs);
    applyPreciseCorrections(if_obs, nav, obs.time);
    ensureAmbiguityStates(if_obs);
    size_t valid_count = 0;
    for (const auto& entry : if_obs) {
        valid_count += entry.valid ? 1U : 0U;
    }
    if (valid_count < static_cast<size_t>(config_.min_satellites)) {
        if (pppDebugEnabled()) {
            std::cerr << "[PPP] insufficient corrected observations: " << valid_count
                      << " < " << config_.min_satellites << "\n";
        }
        return false;
    }

    const int filter_iterations = precise_products_loaded_ ? 3 : ppp_config_.filter_iterations;
    for (int iteration = 0; iteration < filter_iterations; ++iteration) {
        MeasurementEquation meas_eq = formMeasurementEquations(if_obs, nav, obs.time);

        const int measurement_rows = static_cast<int>(meas_eq.observations.size());
        if (measurement_rows < config_.min_satellites) {
            if (pppDebugEnabled()) {
                std::cerr << "[PPP] insufficient measurement rows: " << measurement_rows
                          << " < " << config_.min_satellites << "\n";
            }
            return false;
        }

        PPPFilterIterationDiagnostic iteration_diagnostic;
        iteration_diagnostic.iteration = iteration;
        iteration_diagnostic.rows = measurement_rows;
        double code_residual_sum_sq = 0.0;
        double phase_residual_sum_sq = 0.0;
        double code_residual_max_abs = -1.0;
        double phase_residual_max_abs = -1.0;
        std::vector<PPPResidualDiagnostic> iteration_residual_diagnostics;
        iteration_residual_diagnostics.reserve(
            static_cast<size_t>(measurement_rows) +
            meas_eq.phase_candidate_diagnostics.size());
        for (int row_index = 0; row_index < measurement_rows; ++row_index) {
            const auto row = static_cast<size_t>(row_index);
            const SatelliteId satellite =
                row < meas_eq.row_satellites.size() ? meas_eq.row_satellites[row] : SatelliteId{};
            const bool is_phase = row < meas_eq.row_is_phase.size() && meas_eq.row_is_phase[row];
            const bool is_ionosphere_constraint =
                row < meas_eq.row_is_ionosphere_constraint.size() &&
                meas_eq.row_is_ionosphere_constraint[row];
            const double residual = meas_eq.residuals(row_index);
            const double abs_residual = std::abs(residual);
            if (is_ionosphere_constraint) {
                ++iteration_diagnostic.ionosphere_constraint_rows;
            } else if (is_phase) {
                ++iteration_diagnostic.phase_rows;
                phase_residual_sum_sq += residual * residual;
                if (abs_residual > phase_residual_max_abs) {
                    phase_residual_max_abs = abs_residual;
                    iteration_diagnostic.phase_residual_max_abs_m = abs_residual;
                    iteration_diagnostic.phase_residual_max_sat = satellite;
                }
            } else {
                ++iteration_diagnostic.code_rows;
                code_residual_sum_sq += residual * residual;
                if (abs_residual > code_residual_max_abs) {
                    code_residual_max_abs = abs_residual;
                    iteration_diagnostic.code_residual_max_abs_m = abs_residual;
                    iteration_diagnostic.code_residual_max_sat = satellite;
                }
            }

            PPPResidualDiagnostic residual_diagnostic;
            residual_diagnostic.iteration = iteration;
            residual_diagnostic.row_index = row_index;
            residual_diagnostic.satellite = satellite;
            residual_diagnostic.primary_signal =
                row < meas_eq.row_primary_signals.size() ?
                    meas_eq.row_primary_signals[row] : SignalType::SIGNAL_TYPE_COUNT;
            residual_diagnostic.secondary_signal =
                row < meas_eq.row_secondary_signals.size() ?
                    meas_eq.row_secondary_signals[row] : SignalType::SIGNAL_TYPE_COUNT;
            residual_diagnostic.primary_observation_code =
                row < meas_eq.row_primary_observation_codes.size() ?
                    meas_eq.row_primary_observation_codes[row] : std::string{};
            residual_diagnostic.secondary_observation_code =
                row < meas_eq.row_secondary_observation_codes.size() ?
                    meas_eq.row_secondary_observation_codes[row] : std::string{};
            residual_diagnostic.frequency_index =
                row < meas_eq.row_frequency_indices.size() ? meas_eq.row_frequency_indices[row] : 0;
            residual_diagnostic.ionosphere_coefficient =
                row < meas_eq.row_ionosphere_coefficients.size() ?
                    meas_eq.row_ionosphere_coefficients[row] : 1.0;
            residual_diagnostic.receiver_clock_state_index =
                row < meas_eq.row_receiver_clock_state_indices.size() ?
                    meas_eq.row_receiver_clock_state_indices[row] : -1;
            residual_diagnostic.receiver_clock_design_coeff =
                row < meas_eq.row_receiver_clock_design_coefficients.size() ?
                    meas_eq.row_receiver_clock_design_coefficients[row] : 0.0;
            residual_diagnostic.ionosphere_state_index =
                row < meas_eq.row_ionosphere_state_indices.size() ?
                    meas_eq.row_ionosphere_state_indices[row] : -1;
            residual_diagnostic.ionosphere_design_coeff =
                row < meas_eq.row_ionosphere_design_coefficients.size() ?
                    meas_eq.row_ionosphere_design_coefficients[row] : 0.0;
            residual_diagnostic.ambiguity_state_index =
                row < meas_eq.row_ambiguity_state_indices.size() ?
                    meas_eq.row_ambiguity_state_indices[row] : -1;
            residual_diagnostic.ambiguity_design_coeff =
                row < meas_eq.row_ambiguity_design_coefficients.size() ?
                    meas_eq.row_ambiguity_design_coefficients[row] : 0.0;
            residual_diagnostic.ambiguity_lock_count =
                row < meas_eq.row_ambiguity_lock_counts.size() ?
                    meas_eq.row_ambiguity_lock_counts[row] : -1;
            residual_diagnostic.required_lock_count =
                row < meas_eq.row_required_lock_counts.size() ?
                    meas_eq.row_required_lock_counts[row] : 0;
            residual_diagnostic.phase_limit_m =
                row < meas_eq.row_phase_limits_m.size() ? meas_eq.row_phase_limits_m[row] : 0.0;
            residual_diagnostic.phase_skip_reason =
                row < meas_eq.row_phase_skip_reasons.size() ?
                    meas_eq.row_phase_skip_reasons[row] : std::string{};
            residual_diagnostic.carrier_phase = is_phase;
            residual_diagnostic.ionosphere_constraint = is_ionosphere_constraint;
            residual_diagnostic.phase_accepted = is_phase;
            residual_diagnostic.phase_ready = is_phase;
            residual_diagnostic.observation_m = meas_eq.observations(row_index);
            residual_diagnostic.predicted_m = meas_eq.predicted(row_index);
            residual_diagnostic.residual_m = residual;
            residual_diagnostic.variance_m2 = meas_eq.weight_matrix(row_index, row_index);
            residual_diagnostic.elevation_deg =
                row < meas_eq.row_elevation_deg.size() ? meas_eq.row_elevation_deg[row] : 0.0;
            residual_diagnostic.iono_state_m =
                row < meas_eq.row_iono_state_m.size() ? meas_eq.row_iono_state_m[row] : 0.0;
            iteration_residual_diagnostics.push_back(residual_diagnostic);
        }
        int phase_candidate_index = 0;
        for (auto candidate : meas_eq.phase_candidate_diagnostics) {
            candidate.iteration = iteration;
            candidate.row_index = measurement_rows + phase_candidate_index;
            iteration_residual_diagnostics.push_back(candidate);
            ++phase_candidate_index;
        }
        if (iteration_diagnostic.code_rows > 0) {
            iteration_diagnostic.code_residual_rms_m = std::sqrt(
                code_residual_sum_sq / static_cast<double>(iteration_diagnostic.code_rows));
        }
        if (iteration_diagnostic.phase_rows > 0) {
            iteration_diagnostic.phase_residual_rms_m = std::sqrt(
                phase_residual_sum_sq / static_cast<double>(iteration_diagnostic.phase_rows));
        }

        const MatrixXd innovation_covariance =
            meas_eq.design_matrix * filter_state_.covariance * meas_eq.design_matrix.transpose() +
            meas_eq.weight_matrix;
        const MatrixXd innovation_inverse =
            innovation_covariance.ldlt().solve(MatrixXd::Identity(
                innovation_covariance.rows(), innovation_covariance.cols()));
        const MatrixXd gain =
            filter_state_.covariance * meas_eq.design_matrix.transpose() * innovation_inverse;
        const VectorXd delta_state = gain * meas_eq.residuals;
        const auto deltaAt = [&](int index) {
            return index >= 0 && index < delta_state.size() ? delta_state(index) : 0.0;
        };
        const auto stateAt = [&](int index) {
            return index >= 0 && index < filter_state_.state.size()
                ? filter_state_.state(index) : 0.0;
        };

        const Vector3d position_delta = delta_state.segment(filter_state_.pos_index, 3);
        iteration_diagnostic.pos_delta_m = position_delta.norm();
        iteration_diagnostic.pos_delta_x_m = position_delta(0);
        iteration_diagnostic.pos_delta_y_m = position_delta(1);
        iteration_diagnostic.pos_delta_z_m = position_delta(2);
        iteration_diagnostic.clock_delta_m = deltaAt(filter_state_.clock_index);
        iteration_diagnostic.glo_clock_delta_m = deltaAt(filter_state_.glo_clock_index);
        iteration_diagnostic.gal_clock_delta_m = deltaAt(filter_state_.gal_clock_index);
        iteration_diagnostic.qzs_clock_delta_m = deltaAt(filter_state_.qzs_clock_index);
        iteration_diagnostic.bds_clock_delta_m = deltaAt(filter_state_.bds_clock_index);
        iteration_diagnostic.bds2_clock_delta_m = deltaAt(filter_state_.bds2_clock_index);
        iteration_diagnostic.bds3_clock_delta_m = deltaAt(filter_state_.bds3_clock_index);
        iteration_diagnostic.trop_delta_m = deltaAt(filter_state_.trop_index);
        double iono_delta_sum_sq = 0.0;
        double iono_delta_max_abs = 0.0;
        int iono_delta_count = 0;
        for (const auto& [satellite, index] : filter_state_.ionosphere_indices) {
            (void)satellite;
            if (index < 0 || index >= delta_state.size()) {
                continue;
            }
            const double delta = delta_state(index);
            iono_delta_sum_sq += delta * delta;
            iono_delta_max_abs = std::max(iono_delta_max_abs, std::abs(delta));
            ++iono_delta_count;
        }
        if (iono_delta_count > 0) {
            iteration_diagnostic.iono_delta_rms_m =
                std::sqrt(iono_delta_sum_sq / static_cast<double>(iono_delta_count));
            iteration_diagnostic.iono_delta_max_abs_m = iono_delta_max_abs;
        }

        if (pppDebugEnabled()) {
            std::cerr << "[PPP] iter=" << iteration
                      << " rows=" << measurement_rows
                      << " pos_delta=" << iteration_diagnostic.pos_delta_m
                      << " clock_delta=" << iteration_diagnostic.clock_delta_m
                      << " trop_delta=" << iteration_diagnostic.trop_delta_m
                      << "\n";
        }

        filter_state_.state += delta_state;
        constrainStaticAnchorPosition();

        const Vector3d position_state = filter_state_.state.segment(filter_state_.pos_index, 3);
        iteration_diagnostic.position_x_m = position_state(0);
        iteration_diagnostic.position_y_m = position_state(1);
        iteration_diagnostic.position_z_m = position_state(2);
        iteration_diagnostic.clock_state_m = stateAt(filter_state_.clock_index);
        iteration_diagnostic.glo_clock_state_m = stateAt(filter_state_.glo_clock_index);
        iteration_diagnostic.gal_clock_state_m = stateAt(filter_state_.gal_clock_index);
        iteration_diagnostic.qzs_clock_state_m = stateAt(filter_state_.qzs_clock_index);
        iteration_diagnostic.bds_clock_state_m = stateAt(filter_state_.bds_clock_index);
        iteration_diagnostic.bds2_clock_state_m = stateAt(filter_state_.bds2_clock_index);
        iteration_diagnostic.bds3_clock_state_m = stateAt(filter_state_.bds3_clock_index);
        iteration_diagnostic.trop_state_m = stateAt(filter_state_.trop_index);
        double iono_state_sum_sq = 0.0;
        double iono_state_max_abs = 0.0;
        int iono_state_count = 0;
        for (const auto& [satellite, index] : filter_state_.ionosphere_indices) {
            (void)satellite;
            if (index < 0 || index >= filter_state_.state.size()) {
                continue;
            }
            const double state = filter_state_.state(index);
            iono_state_sum_sq += state * state;
            iono_state_max_abs = std::max(iono_state_max_abs, std::abs(state));
            ++iono_state_count;
        }
        if (iono_state_count > 0) {
            iteration_diagnostic.iono_state_rms_m =
                std::sqrt(iono_state_sum_sq / static_cast<double>(iono_state_count));
            iteration_diagnostic.iono_state_max_abs_m = iono_state_max_abs;
        }

        const auto kalmanGainAt = [&](int state_index, int row_index) {
            return state_index >= 0 && state_index < gain.rows() &&
                   row_index >= 0 && row_index < gain.cols()
                ? gain(state_index, row_index)
                : 0.0;
        };
        const auto innovationVarianceAt = [&](int row_index) {
            return row_index >= 0 && row_index < innovation_covariance.rows() &&
                   row_index < innovation_covariance.cols()
                ? innovation_covariance(row_index, row_index)
                : 0.0;
        };
        const auto innovationInverseDiagonalAt = [&](int row_index) {
            return row_index >= 0 && row_index < innovation_inverse.rows() &&
                   row_index < innovation_inverse.cols()
                ? innovation_inverse(row_index, row_index)
                : 0.0;
        };
        const auto measurementRowKind = [&](int row_index) {
            const auto row = static_cast<size_t>(row_index);
            if (row_index >= 0 && row < meas_eq.row_is_ionosphere_constraint.size() &&
                meas_eq.row_is_ionosphere_constraint[row]) {
                return 2;
            }
            if (row_index >= 0 && row < meas_eq.row_is_phase.size() &&
                meas_eq.row_is_phase[row]) {
                return 1;
            }
            return 0;
        };
        const auto accumulateRowCouplings = [&, measurementRowKind](
            const MatrixXd& matrix,
            int row_index,
            double& code_abs_sum,
            double& phase_abs_sum,
            double& ionosphere_constraint_abs_sum) {
            code_abs_sum = 0.0;
            phase_abs_sum = 0.0;
            ionosphere_constraint_abs_sum = 0.0;
            if (row_index < 0 || row_index >= matrix.rows()) {
                return;
            }
            for (int column = 0; column < matrix.cols(); ++column) {
                if (column == row_index) {
                    continue;
                }
                const double value = std::abs(matrix(row_index, column));
                switch (measurementRowKind(column)) {
                    case 2:
                        ionosphere_constraint_abs_sum += value;
                        break;
                    case 1:
                        phase_abs_sum += value;
                        break;
                    default:
                        code_abs_sum += value;
                        break;
                }
            }
        };
        const auto updateContributionAt = [&](int state_index, int row_index) {
            return kalmanGainAt(state_index, row_index) *
                   (row_index >= 0 && row_index < meas_eq.residuals.size() ?
                        meas_eq.residuals(row_index) : 0.0);
        };
        const int accepted_diagnostic_rows = std::min(
            measurement_rows,
            static_cast<int>(iteration_residual_diagnostics.size()));
        for (int row_index = 0; row_index < accepted_diagnostic_rows; ++row_index) {
            auto& diagnostic = iteration_residual_diagnostics[static_cast<size_t>(row_index)];
            diagnostic.innovation_variance_m2 = innovationVarianceAt(row_index);
            diagnostic.innovation_inverse_diagonal_1_per_m2 =
                innovationInverseDiagonalAt(row_index);
            accumulateRowCouplings(
                innovation_covariance,
                row_index,
                diagnostic.innovation_covariance_code_coupling_abs_m2,
                diagnostic.innovation_covariance_phase_coupling_abs_m2,
                diagnostic.innovation_covariance_ionosphere_constraint_coupling_abs_m2);
            accumulateRowCouplings(
                innovation_inverse,
                row_index,
                diagnostic.innovation_inverse_code_coupling_abs_1_per_m2,
                diagnostic.innovation_inverse_phase_coupling_abs_1_per_m2,
                diagnostic.innovation_inverse_ionosphere_constraint_coupling_abs_1_per_m2);
            diagnostic.position_x_kalman_gain = kalmanGainAt(filter_state_.pos_index, row_index);
            diagnostic.position_y_kalman_gain = kalmanGainAt(filter_state_.pos_index + 1, row_index);
            diagnostic.position_z_kalman_gain = kalmanGainAt(filter_state_.pos_index + 2, row_index);
            diagnostic.position_update_contribution_x_m = updateContributionAt(
                filter_state_.pos_index, row_index);
            diagnostic.position_update_contribution_y_m = updateContributionAt(
                filter_state_.pos_index + 1, row_index);
            diagnostic.position_update_contribution_z_m = updateContributionAt(
                filter_state_.pos_index + 2, row_index);
            diagnostic.position_update_contribution_3d_m = std::sqrt(
                diagnostic.position_update_contribution_x_m *
                    diagnostic.position_update_contribution_x_m +
                diagnostic.position_update_contribution_y_m *
                    diagnostic.position_update_contribution_y_m +
                diagnostic.position_update_contribution_z_m *
                    diagnostic.position_update_contribution_z_m);
            diagnostic.receiver_clock_kalman_gain = kalmanGainAt(
                diagnostic.receiver_clock_state_index, row_index);
            diagnostic.ionosphere_kalman_gain = kalmanGainAt(
                diagnostic.ionosphere_state_index, row_index);
            diagnostic.ambiguity_kalman_gain = kalmanGainAt(
                diagnostic.ambiguity_state_index, row_index);
            diagnostic.receiver_clock_update_contribution_m = updateContributionAt(
                diagnostic.receiver_clock_state_index, row_index);
            diagnostic.ionosphere_update_contribution_m = updateContributionAt(
                diagnostic.ionosphere_state_index, row_index);
            diagnostic.ambiguity_update_contribution_m = updateContributionAt(
                diagnostic.ambiguity_state_index, row_index);
        }
        last_residual_diagnostics_.insert(
            last_residual_diagnostics_.end(),
            iteration_residual_diagnostics.begin(),
            iteration_residual_diagnostics.end());

        last_filter_iteration_diagnostics_.push_back(iteration_diagnostic);

        const MatrixXd identity =
            MatrixXd::Identity(filter_state_.total_states, filter_state_.total_states);
        const MatrixXd kh = gain * meas_eq.design_matrix;
        filter_state_.covariance =
            (identity - kh) * filter_state_.covariance * (identity - kh).transpose() +
            gain * meas_eq.weight_matrix * gain.transpose();

        if (iteration_diagnostic.pos_delta_m < 1e-4 &&
            std::abs(iteration_diagnostic.clock_delta_m) < 1e-3 &&
            std::abs(iteration_diagnostic.trop_delta_m) < 1e-3) {
            break;
        }
    }

    if (pppDebugEnabled()) {
        std::cerr << "[PPP] state clock=" << filter_state_.state(filter_state_.clock_index)
                  << " trop=" << filter_state_.state(filter_state_.trop_index)
                  << " pos_norm=" << filter_state_.state.segment(filter_state_.pos_index, 3).norm()
                  << "\n";
    }

    // Save covariance before anchor/velocity constraints destroy cross-terms.
    pre_anchor_covariance_ = filter_state_.covariance;

    updateAmbiguityStates(obs);
    constrainStaticVelocityStates();
    constrainStaticAnchorPosition();
    return true;
}

std::vector<PPPProcessor::IonosphereFreeObs> PPPProcessor::formIonosphereFree(
    const ObservationData& obs,
    const NavigationData& nav) {
    std::vector<IonosphereFreeObs> combined;
    combined.reserve(ppp_config_.use_ionosphere_free ? obs.getSatellites().size()
                                                     : obs.observations.size());

    for (const auto& sat : obs.getSatellites()) {
        if (!ppp_config_.allowed_systems.empty() &&
            ppp_config_.allowed_systems.count(sat.system) == 0U) {
            continue;
        }
        // Match SPP/RTK until native BDS2 GEO clock/ionosphere handling is complete.
        if (!isUsablePppSatellite(sat)) {
            continue;
        }
        const Observation* primary = findObservationForSignals(obs, sat, primarySignals(sat.system));
        if (primary == nullptr) {
            continue;
        }

        const Ephemeris* eph = nav.getEphemeris(sat, obs.time);
        IonosphereFreeObs entry;
        entry.satellite = sat;
        entry.frequency_index = 0;

        if (!ppp_config_.use_ionosphere_free) {
            entry.pseudorange_if = primary->pseudorange;
            entry.primary_signal = primary->signal;
            entry.primary_observation_code = primary->observation_code;
            entry.primary_code_bias_coeff = 1.0;
            entry.secondary_code_bias_coeff = 0.0;
            if (primary->has_carrier_phase) {
                const double wavelength = signalWavelengthMeters(primary->signal, eph);
                if (wavelength > 0.0) {
                    entry.carrier_phase_if = primary->carrier_phase * wavelength;
                    entry.ambiguity_scale_m = wavelength;
                    entry.has_carrier_phase = true;
                }
            }
            entry.variance_pr = safeVariance(
                ppp_config_.pseudorange_sigma * ppp_config_.pseudorange_sigma, 1e-6);
            entry.variance_cp = safeVariance(
                ppp_config_.carrier_phase_sigma * ppp_config_.carrier_phase_sigma, 1e-8);
            entry.valid = true;
            // Fall through to SSR correction application below.
            // But skip IFLC formation — keep L1-only entry.
        }

        const Observation* secondary =
            findObservationForSignals(obs, sat, secondarySignals(sat.system));

        // Per-frequency mode: skip IFLC formation, keep L1-only entry
        // SSR corrections (orbit/clock/bias/iono) still applied below
        if (!ppp_config_.use_ionosphere_free) {
            if (secondary != nullptr) {
                entry.secondary_signal = secondary->signal;
                entry.secondary_observation_code = secondary->observation_code;
                // Store secondary observation info for MW combination in AR
                entry.primary_code_bias_coeff = 1.0;
                entry.secondary_code_bias_coeff = 0.0;
                double ionosphere_delay_m = 0.0;
                if (estimatePrimaryIonosphereDelaySeedMeters(
                        *primary, *secondary, eph, ionosphere_delay_m)) {
                    entry.ionosphere_delay_seed_m = ionosphere_delay_m;
                    entry.has_ionosphere_delay_seed = true;
                }
            }
            entry.valid = true;
            combined.push_back(entry);
            if (ppp_config_.estimate_ionosphere) {
                const double primary_frequency_hz = signalFrequencyHz(primary->signal, eph);
                if (primary_frequency_hz > 0.0) {
                    std::set<long long> emitted_frequencies_hz;
                    emitted_frequencies_hz.insert(
                        static_cast<long long>(std::llround(primary_frequency_hz)));
                    int next_frequency_index = 1;
                    for (const auto signal : secondarySignals(sat.system)) {
                        const Observation* extra = obs.getObservation(sat, signal);
                        if (extra == nullptr || !usablePseudorangeObservation(*extra)) {
                            continue;
                        }
                        const double frequency_hz = signalFrequencyHz(extra->signal, eph);
                        if (frequency_hz <= 0.0 || !std::isfinite(frequency_hz) ||
                            std::abs(frequency_hz - primary_frequency_hz) < 1.0) {
                            continue;
                        }
                        const long long rounded_frequency_hz =
                            static_cast<long long>(std::llround(frequency_hz));
                        if (!emitted_frequencies_hz.insert(rounded_frequency_hz).second) {
                            continue;
                        }

                        IonosphereFreeObs code_entry = entry;
                        code_entry.primary_signal = extra->signal;
                        code_entry.secondary_signal = SignalType::SIGNAL_TYPE_COUNT;
                        code_entry.primary_observation_code = extra->observation_code;
                        code_entry.secondary_observation_code.clear();
                        code_entry.frequency_index = next_frequency_index++;
                        code_entry.primary_code_bias_coeff = 1.0;
                        code_entry.secondary_code_bias_coeff = 0.0;
                        code_entry.ionosphere_coefficient =
                            (primary_frequency_hz * primary_frequency_hz) /
                            (frequency_hz * frequency_hz);
                        code_entry.pseudorange_if = extra->pseudorange;
                        code_entry.carrier_phase_if = 0.0;
                        code_entry.ambiguity_scale_m = 0.0;
                        code_entry.has_carrier_phase = false;
                        if (ppp_config_.enable_per_frequency_phase_bias_states &&
                            extra->has_carrier_phase) {
                            const double wavelength = signalWavelengthMeters(extra->signal, eph);
                            if (wavelength > 0.0) {
                                code_entry.carrier_phase_if = extra->carrier_phase * wavelength;
                                code_entry.ambiguity_scale_m = wavelength;
                                code_entry.variance_cp = safeVariance(
                                    ppp_config_.carrier_phase_sigma * ppp_config_.carrier_phase_sigma,
                                    1e-8);
                                code_entry.has_carrier_phase = true;
                            }
                        }
                        code_entry.pseudorange_code_bias_m = 0.0;
                        code_entry.carrier_phase_bias_m = 0.0;
                        code_entry.atmospheric_trop_correction_m = 0.0;
                        code_entry.atmospheric_iono_correction_m = 0.0;
                        code_entry.ionosphere_constraint_m = 0.0;
                        code_entry.ionosphere_constraint_sigma_m = 0.0;
                        code_entry.has_ionosphere_constraint = false;
                        code_entry.allow_ionosphere_constraint = false;
                        code_entry.ssr_atmos_tokens.clear();
                        combined.push_back(code_entry);
                    }
                }
            }
            continue;
        }

        if (secondary == nullptr) {
            entry.pseudorange_if = primary->pseudorange;
            entry.primary_signal = primary->signal;
            entry.primary_observation_code = primary->observation_code;
            entry.primary_code_bias_coeff = 1.0;
            entry.secondary_code_bias_coeff = 0.0;
            if (primary->has_carrier_phase) {
                const double wavelength = signalWavelengthMeters(primary->signal, eph);
                if (wavelength > 0.0) {
                    entry.carrier_phase_if = primary->carrier_phase * wavelength;
                    entry.ambiguity_scale_m = wavelength;
                    entry.has_carrier_phase = true;
                }
            }
            entry.variance_pr = safeVariance(
                ppp_config_.pseudorange_sigma * ppp_config_.pseudorange_sigma, 1e-6);
            entry.variance_cp = safeVariance(
                ppp_config_.carrier_phase_sigma * ppp_config_.carrier_phase_sigma, 1e-8);
            entry.valid = true;
            combined.push_back(entry);
            continue;
        }

        const double f1 = signalFrequencyHz(primary->signal, eph);
        const double f2 = signalFrequencyHz(secondary->signal, eph);
        if (f1 <= 0.0 || f2 <= 0.0 || std::abs(f1 - f2) < 1.0) {
            continue;
        }

        const auto coefficients = ppp_utils::getIonosphereFreeCoefficients(f1, f2);
        entry.pseudorange_if =
            coefficients.first * primary->pseudorange + coefficients.second * secondary->pseudorange;
        entry.primary_signal = primary->signal;
        entry.secondary_signal = secondary->signal;
        entry.primary_observation_code = primary->observation_code;
        entry.secondary_observation_code = secondary->observation_code;
        entry.primary_code_bias_coeff = coefficients.first;
        entry.secondary_code_bias_coeff = coefficients.second;
        double ionosphere_delay_m = 0.0;
        if (estimatePrimaryIonosphereDelaySeedMeters(
                *primary, *secondary, eph, ionosphere_delay_m)) {
            entry.ionosphere_delay_seed_m = ionosphere_delay_m;
            entry.has_ionosphere_delay_seed = true;
        }
        entry.pseudorange_code_bias_m = 0.0;
        entry.variance_pr = safeVariance(
            (coefficients.first * coefficients.first +
             coefficients.second * coefficients.second) *
                ppp_config_.pseudorange_sigma * ppp_config_.pseudorange_sigma,
            1e-6);

        if (primary->has_carrier_phase && secondary->has_carrier_phase) {
            const double lambda1 = signalWavelengthMeters(primary->signal, eph);
            const double lambda2 = signalWavelengthMeters(secondary->signal, eph);
            if (lambda1 > 0.0 && lambda2 > 0.0) {
                const double l1_m = primary->carrier_phase * lambda1;
                const double l2_m = secondary->carrier_phase * lambda2;
                entry.carrier_phase_if = coefficients.first * l1_m + coefficients.second * l2_m;
                entry.ambiguity_scale_m = lambda1;
                entry.variance_cp = safeVariance(
                    (coefficients.first * coefficients.first +
                     coefficients.second * coefficients.second) *
                        ppp_config_.carrier_phase_sigma * ppp_config_.carrier_phase_sigma,
                    1e-8);
                entry.has_carrier_phase = true;
            }
        }

        entry.valid = std::isfinite(entry.pseudorange_if);
        combined.push_back(entry);
    }

    return combined;
}

Vector3d PPPProcessor::calculateReceiverAntennaOffsetEcef(
    const Vector3d& receiver_marker_position,
    const IonosphereFreeObs& observation) const {
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;
    ecef2geodetic(receiver_marker_position, latitude_rad, longitude_rad, height_m);

    Vector3d offset_enu = ppp_config_.receiver_antenna_delta_enu;
    if (receiver_antex_loaded_ && !ppp_config_.receiver_antenna_type.empty()) {
        const auto antenna_it =
            receiver_antex_offsets_.find(normalizeAntennaType(ppp_config_.receiver_antenna_type));
        if (antenna_it != receiver_antex_offsets_.end()) {
            auto signal_offset = [&](SignalType signal) -> Vector3d {
                const auto it = antenna_it->second.offsets_enu_m.find(signal);
                return it != antenna_it->second.offsets_enu_m.end() ? it->second : Vector3d::Zero();
            };
            if (!ppp_config_.use_ionosphere_free ||
                observation.secondary_signal == SignalType::SIGNAL_TYPE_COUNT) {
                offset_enu += signal_offset(observation.primary_signal);
            } else {
                offset_enu +=
                    observation.primary_code_bias_coeff * signal_offset(observation.primary_signal) +
                    observation.secondary_code_bias_coeff *
                        signal_offset(observation.secondary_signal);
            }
        }
    }

    return enu2ecef(offset_enu, latitude_rad, longitude_rad);
}

double PPPProcessor::calculateReceiverAntennaPcvMeters(
    const IonosphereFreeObs& observation,
    double elevation_rad) const {
    if (!receiver_antex_loaded_ || ppp_config_.receiver_antenna_type.empty() ||
        !std::isfinite(elevation_rad)) {
        return 0.0;
    }
    const auto antenna_it =
        receiver_antex_offsets_.find(normalizeAntennaType(ppp_config_.receiver_antenna_type));
    if (antenna_it == receiver_antex_offsets_.end()) {
        return 0.0;
    }

    const double zenith_angle_deg = 90.0 - elevation_rad / kDegreesToRadians;
    auto signal_pcv = [&](SignalType signal) -> double {
        const auto it = antenna_it->second.noazi_pcv_m.find(signal);
        return it != antenna_it->second.noazi_pcv_m.end() ?
            interpolateNoaziPcv(it->second, zenith_angle_deg) : 0.0;
    };

    if (!ppp_config_.use_ionosphere_free ||
        observation.secondary_signal == SignalType::SIGNAL_TYPE_COUNT) {
        return signal_pcv(observation.primary_signal);
    }
    return observation.primary_code_bias_coeff * signal_pcv(observation.primary_signal) +
           observation.secondary_code_bias_coeff * signal_pcv(observation.secondary_signal);
}

Vector3d PPPProcessor::calculateSatelliteAntennaOffsetEcef(
    const IonosphereFreeObs& observation,
    const Vector3d& satellite_position_ecef,
    const GNSSTime& time) const {
    const auto satellite_it = satellite_antex_offsets_.find(observation.satellite);
    if (satellite_it == satellite_antex_offsets_.end() ||
        satellite_position_ecef.squaredNorm() <= 0.0) {
        return Vector3d::Zero();
    }

    const SatelliteAntexEntry* selected_entry = nullptr;
    for (const auto& entry : satellite_it->second) {
        if (entry.has_valid_from && time < entry.valid_from) {
            continue;
        }
        if (entry.has_valid_until && time > entry.valid_until) {
            continue;
        }
        selected_entry = &entry;
    }
    if (selected_entry == nullptr) {
        return Vector3d::Zero();
    }

    auto signal_offset = [&](SignalType signal) -> Vector3d {
        const auto it = selected_entry->offsets_neu_m.find(signal);
        return it != selected_entry->offsets_neu_m.end() ? it->second : Vector3d::Zero();
    };

    // MADOCALIB/RTKLIB satantoff() always combines the satellite PCO as the
    // iono-free LC of the two primary frequencies (preceph.c:617-621).  The
    // SSR orbit corrections are defined relative to that IFLC phase center,
    // so the same combination must be applied even when the filter itself
    // runs in estimated-ionosphere mode; otherwise a large per-satellite
    // offset (tens of cm) is introduced into the range.
    Vector3d offset_neu = Vector3d::Zero();
    if (observation.secondary_signal == SignalType::SIGNAL_TYPE_COUNT) {
        offset_neu = signal_offset(observation.primary_signal);
    } else {
        const double f1 = signalFrequencyHz(observation.primary_signal);
        const double f2 = signalFrequencyHz(observation.secondary_signal);
        const auto iflc = ppp_utils::getIonosphereFreeCoefficients(f1, f2);
        offset_neu =
            iflc.first * signal_offset(observation.primary_signal) +
            iflc.second * signal_offset(observation.secondary_signal);
    }
    if (offset_neu.squaredNorm() <= 0.0) {
        return Vector3d::Zero();
    }

    const Vector3d ez = -satellite_position_ecef.normalized();
    const Vector3d sun_position_ecef = approximateSunPositionEcef(time);
    Vector3d sun_direction = sun_position_ecef - satellite_position_ecef;
    if (sun_direction.squaredNorm() <= 0.0) {
        return Vector3d::Zero();
    }
    sun_direction.normalize();
    Vector3d ey = ez.cross(sun_direction);
    if (ey.squaredNorm() <= 0.0) {
        return Vector3d::Zero();
    }
    ey.normalize();
    const Vector3d ex = ey.cross(ez).normalized();

    // MADOCALIB/RTKLIB convention (preceph.c satantoff):
    //   dant = pcv->off[a][0]*ex + pcv->off[a][1]*ey + pcv->off[a][2]*ez
    //   rs   = rss + dant   (satellite center-of-mass -> phase center)
    // ANTEX satellite PCO components are stored as (X, Y, Z) in the
    // satellite body-fixed frame despite the "NORTH / EAST / UP" header
    // label.  Native's offset_neu storage preserves the file order
    // (offset.x = first field = body X, etc.).
    return offset_neu.x() * ex + offset_neu.y() * ey + offset_neu.z() * ez;
}

void PPPProcessor::applyPreciseCorrections(std::vector<IonosphereFreeObs>& observations,
                                           const NavigationData& nav,
                                           const GNSSTime& time) {
    const Vector3d receiver_marker_position = applyGeophysicalCorrections(
        filter_state_.state.segment(filter_state_.pos_index, 3), time);
    const double elevation_mask = config_.elevation_mask * kDegreesToRadians;
    last_ssr_application_diagnostics_.clear();

    // Pre-fetch epoch-wide atmosphere tokens from any satellite that has them.
    // CLAS atmosphere corrections are network-wide, not per-satellite, so we
    // retrieve them once and share across all observations in this epoch.
    std::map<std::string, std::string> epoch_atmos_tokens;
    if (ssr_products_loaded_) {
        std::vector<SatelliteId> epoch_satellites;
        epoch_satellites.reserve(observations.size());
        for (const auto& obs_scan : observations) {
            if (obs_scan.valid) {
                epoch_satellites.push_back(obs_scan.satellite);
            }
        }
        epoch_atmos_tokens =
            selectClasEpochAtmosTokens(
                ssr_products_, epoch_satellites, time, receiver_marker_position, ppp_config_);
    }
    const int preferred_network_id = preferredClasNetworkId(epoch_atmos_tokens);

    for (auto& observation : observations) {
        const bool collect_diagnostics =
            ssr_products_loaded_ || ionex_products_loaded_ || dcb_products_loaded_;
        SSRApplicationDiagnostic diagnostic;
        diagnostic.satellite = observation.satellite;
        diagnostic.primary_signal = observation.primary_signal;
        diagnostic.secondary_signal = observation.secondary_signal;
        diagnostic.primary_observation_code = observation.primary_observation_code;
        diagnostic.secondary_observation_code = observation.secondary_observation_code;
        diagnostic.frequency_index = observation.frequency_index;
        diagnostic.primary_code_bias_coeff = observation.primary_code_bias_coeff;
        diagnostic.secondary_code_bias_coeff = observation.secondary_code_bias_coeff;
        diagnostic.ionosphere_coefficient = observation.ionosphere_coefficient;
        diagnostic.has_carrier_phase = observation.has_carrier_phase;
        diagnostic.preferred_network_id = preferred_network_id;
        bool diagnostic_pushed = false;
        const auto pushDiagnostic = [&]() {
            if (collect_diagnostics && !diagnostic_pushed) {
                diagnostic.has_carrier_phase = observation.has_carrier_phase;
                last_ssr_application_diagnostics_.push_back(diagnostic);
                diagnostic_pushed = true;
            }
        };

        double deferred_variance_pr = 0.0;
        double deferred_variance_cp = 0.0;
        bool applied_ssr_code_bias = false;
        bool applied_ssr_iono = false;
        bool applied_satellite_antenna_offset = false;
        const Vector3d receiver_antenna_offset_ecef =
            calculateReceiverAntennaOffsetEcef(receiver_marker_position, observation);
        const Vector3d receiver_position = receiver_marker_position + receiver_antenna_offset_ecef;
        const Ephemeris* eph = ppp_config_.use_rtklib_broadcast_selection
            ? nav.getRtklibEphemeris(observation.satellite, time)
            : nav.getEphemeris(observation.satellite, time);
        if (eph != nullptr) {
            diagnostic.broadcast_iode = static_cast<int>(eph->iode);
        }
        Vector3d sat_position = Vector3d::Zero();
        Vector3d sat_velocity = Vector3d::Zero();
        double sat_clock_bias = 0.0;
        double sat_clock_drift = 0.0;

        auto applySatelliteAntennaOffset = [&]() {
            if (!applied_satellite_antenna_offset) {
                // Skip when SSR orbits are delivered at the antenna phase
                // center (MADOCALIB EPHOPT_SSRAPC); re-applying the PCO
                // here would double-correct the satellite position.
                if (!ppp_config_.ssr_orbit_reference_is_apc) {
                    sat_position +=
                        calculateSatelliteAntennaOffsetEcef(observation, sat_position, time);
                }
                applied_satellite_antenna_offset = true;
            }
        };
        auto calculateBroadcastState = [&](const Ephemeris& broadcast_eph,
                                           const GNSSTime& state_time) {
            return broadcast_eph.calculateSatelliteState(
                state_time, sat_position, sat_velocity, sat_clock_bias, sat_clock_drift);
        };

        bool have_precise = false;
        if (precise_products_loaded_) {
            have_precise = precise_products_.interpolateOrbitClock(
                observation.satellite,
                time,
                sat_position,
                sat_velocity,
                sat_clock_bias,
                sat_clock_drift);
        }

        if (!have_precise) {
            // First pass: compute satellite state at reception time
            if (eph == nullptr || !calculateBroadcastState(*eph, time)) {
                diagnostic.orbit_clock_skip_reason =
                    eph == nullptr ? "broadcast_ephemeris_unavailable" : "broadcast_state_failed";
                observation.valid = false;
                pushDiagnostic();
                continue;
            }
            // Light travel time correction: re-evaluate at emission time
            if (observation.pseudorange_if > 0.0) {
                const double travel_time = observation.pseudorange_if / constants::SPEED_OF_LIGHT;
                const GNSSTime emission_time = time - travel_time + sat_clock_bias;
                if (!calculateBroadcastState(*eph, emission_time)) {
                    diagnostic.orbit_clock_skip_reason = "broadcast_state_failed";
                    observation.valid = false;
                    pushDiagnostic();
                    continue;
                }
            }
        }

        if (ssr_products_loaded_) {
            Vector3d orbit_correction_ecef = Vector3d::Zero();
            double clock_correction_m = 0.0;
            double ura_sigma_m = 0.0;
            std::map<uint8_t, double> code_bias_m;
            std::map<uint8_t, double> phase_bias_m;
            std::map<std::string, std::string> atmos_tokens;
            int orbit_iode = -1;
            const bool ssr_ok = ssr_products_.interpolateCorrection(
                    observation.satellite,
                    time,
                    orbit_correction_ecef,
                    clock_correction_m,
                    &ura_sigma_m,
                    &code_bias_m,
                    &phase_bias_m,
                    &atmos_tokens,
                    nullptr,
                    nullptr,
                    nullptr,
                    preferred_network_id,
                    ppp_config_.allow_future_ssr_corrections,
                    ppp_config_.require_ssr_orbit_clock,
                    &orbit_iode);
            // Fall back to epoch-wide atmosphere tokens when per-satellite
            // atmos are empty (CLAS broadcasts network-wide corrections).
            if (atmos_tokens.empty() && !epoch_atmos_tokens.empty()) {
                atmos_tokens = epoch_atmos_tokens;
            }
            diagnostic.ssr_available = ssr_ok;
            diagnostic.ssr_orbit_iode = orbit_iode;
            diagnostic.ura_sigma_m = ura_sigma_m;
            diagnostic.atmos_token_count = static_cast<int>(atmos_tokens.size());
            if (!ssr_ok && !have_precise) {
                diagnostic.orbit_clock_skip_reason = "ssr_unavailable";
            }
            if (ssr_ok) {
                if (!have_precise) {
                    const bool iode_gate_after_warmup =
                        ppp_config_.ssr_orbit_iode_admission_gate_warmup_epochs <= 0 ||
                        total_epochs_processed_ >=
                            static_cast<size_t>(
                                ppp_config_.ssr_orbit_iode_admission_gate_warmup_epochs);
                    if ((ppp_config_.enforce_ssr_orbit_iode ||
                         (ppp_config_.enforce_ssr_orbit_iode_admission_only &&
                          iode_gate_after_warmup)) &&
                        orbit_iode >= 0) {
                        const Ephemeris* matched_eph = nav.getEphemerisByIode(
                            observation.satellite, static_cast<uint16_t>(orbit_iode), time);
                        if (matched_eph == nullptr) {
                            diagnostic.orbit_clock_skip_reason =
                                "ssr_orbit_iode_no_matching_ephemeris";
                            observation.valid = false;
                            pushDiagnostic();
                            continue;
                        }
                        if (ppp_config_.enforce_ssr_orbit_iode) {
                            eph = matched_eph;
                            diagnostic.broadcast_iode = static_cast<int>(eph->iode);
                            if (!calculateBroadcastState(*eph, time)) {
                                diagnostic.orbit_clock_skip_reason = "ssr_orbit_iode_state_failed";
                                observation.valid = false;
                                pushDiagnostic();
                                continue;
                            }
                            if (observation.pseudorange_if > 0.0) {
                                const double travel_time =
                                    observation.pseudorange_if / constants::SPEED_OF_LIGHT;
                                const GNSSTime emission_time = time - travel_time + sat_clock_bias;
                                if (!calculateBroadcastState(*eph, emission_time)) {
                                    diagnostic.orbit_clock_skip_reason =
                                        "ssr_orbit_iode_state_failed";
                                    observation.valid = false;
                                    pushDiagnostic();
                                    continue;
                                }
                            }
                        }
                    }
                    if (ssr_products_.orbitCorrectionsAreRac()) {
                        orbit_correction_ecef =
                            ssrRacToEcef(sat_position, sat_velocity, orbit_correction_ecef);
                    }
                    diagnostic.orbit_clock_applied = true;
                    diagnostic.orbit_correction_x_m = orbit_correction_ecef.x();
                    diagnostic.orbit_correction_y_m = orbit_correction_ecef.y();
                    diagnostic.orbit_correction_z_m = orbit_correction_ecef.z();
                    diagnostic.clock_correction_m = clock_correction_m;
                    sat_position += orbit_correction_ecef;
                    sat_clock_bias += clock_correction_m / constants::SPEED_OF_LIGHT;
                }
                applySatelliteAntennaOffset();
                if (ppp_config_.require_ssr_observation_biases &&
                    !observationHasRequiredSsrBiases(
                        observation.satellite.system,
                        observation.primary_signal,
                        observation.secondary_signal,
                        ppp_config_.use_ionosphere_free,
                        code_bias_m)) {
                    diagnostic.orbit_clock_skip_reason = "ssr_code_bias_unavailable";
                    observation.valid = false;
                    pushDiagnostic();
                    continue;
                }
                const double code_bias = observationCodeBiasMeters(
                    observation.satellite.system,
                    observation.primary_signal,
                    observation.secondary_signal,
                    ppp_config_.use_ionosphere_free,
                    code_bias_m,
                    observation.primary_code_bias_coeff,
                    observation.secondary_code_bias_coeff);
                observation.pseudorange_if -= code_bias;
                observation.pseudorange_code_bias_m = code_bias;
                diagnostic.code_bias_m = code_bias;
                applied_ssr_code_bias =
                    ppp_config_.require_ssr_observation_biases || std::abs(code_bias) > 0.0;
                if (observation.has_carrier_phase &&
                    ppp_config_.require_ssr_observation_biases &&
                    !observationHasRequiredSsrPhaseBiases(
                        observation.satellite.system,
                        observation.primary_signal,
                        observation.secondary_signal,
                        ppp_config_.use_ionosphere_free,
                        phase_bias_m)) {
                    observation.has_carrier_phase = false;
                    observation.carrier_phase_if = 0.0;
                    observation.carrier_phase_bias_m = 0.0;
                    observation.ambiguity_scale_m = 0.0;
                }
                if (observation.has_carrier_phase && !phase_bias_m.empty()) {
                    const double phase_bias = observationPhaseBiasMeters(
                        observation.satellite.system,
                        observation.primary_signal,
                        observation.secondary_signal,
                        ppp_config_.use_ionosphere_free,
                        phase_bias_m,
                        observation.primary_code_bias_coeff,
                        observation.secondary_code_bias_coeff);
                    observation.carrier_phase_if -= phase_bias;
                    observation.carrier_phase_bias_m = phase_bias;
                    diagnostic.phase_bias_m = phase_bias;
                }
                if (ura_sigma_m > 0.0 && std::isfinite(ura_sigma_m)) {
                    const double ura_variance = ura_sigma_m * ura_sigma_m;
                    deferred_variance_pr += ura_variance;
                    deferred_variance_cp += ura_variance;
                }
                const auto ssr_geometry = nav.calculateGeometry(receiver_position, sat_position);
                const double trop_correction_m =
                    ppp_atmosphere::atmosphericTroposphereCorrectionMeters(
                        atmos_tokens,
                        receiver_position,
                        time,
                        ssr_geometry.elevation,
                        ppp_config_.clas_expanded_value_construction_policy,
                        ppp_config_.clas_subtype12_value_construction_policy,
                        ppp_config_.clas_expanded_residual_sampling_policy);
                if (std::isfinite(trop_correction_m)) {
                    diagnostic.trop_correction_m = trop_correction_m;
                }
                if (std::isfinite(trop_correction_m) && std::abs(trop_correction_m) > 0.0) {
                    observation.pseudorange_if -= trop_correction_m;
                    if (observation.has_carrier_phase) {
                        observation.carrier_phase_if -= trop_correction_m;
                    }
                    observation.atmospheric_trop_correction_m = trop_correction_m;
                    ++last_applied_atmos_trop_corrections_;
                    last_applied_atmos_trop_m_ += std::abs(trop_correction_m);
                }
                const double stec_tecu =
                    ppp_atmosphere::atmosphericStecTecu(
                        atmos_tokens,
                        observation.satellite,
                        receiver_position,
                        ppp_config_.clas_expanded_value_construction_policy,
                        ppp_config_.clas_subtype12_value_construction_policy,
                        ppp_config_.clas_expanded_residual_sampling_policy);
                if (std::isfinite(stec_tecu)) {
                    diagnostic.stec_tecu = stec_tecu;
                }
                if (pppDebugEnabled()) {
                    std::cerr << "[PPP-STEC] " << observation.satellite.toString()
                              << " stec_tecu=" << stec_tecu
                              << " atmos_tokens_size=" << atmos_tokens.size()
                              << " use_iflc=" << ppp_config_.use_ionosphere_free
                              << "\n";
                }
                const double ionosphere_correction_m = ppp_atmosphere::observationIonosphereDelayMeters(
                    eph,
                    observation.primary_signal,
                    observation.secondary_signal,
                    ppp_config_.use_ionosphere_free,
                    stec_tecu,
                    observation.primary_code_bias_coeff,
                    observation.secondary_code_bias_coeff);
                if (std::isfinite(ionosphere_correction_m)) {
                    diagnostic.iono_correction_m = ionosphere_correction_m;
                }
                double stec_sigma_m = 0.5;
                const bool use_stec_constraint =
                    stecConstraintSigmaMeters(atmos_tokens, observation.satellite, stec_sigma_m);
                if (ppp_config_.estimate_ionosphere &&
                    observation.allow_ionosphere_constraint &&
                    use_stec_constraint &&
                    std::isfinite(stec_tecu) && std::abs(stec_tecu) > 0.0) {
                    const auto iono_it = filter_state_.ionosphere_indices.find(observation.satellite);
                    const int system_index =
                        ionosphereConstraintSystemIndex(observation.satellite.system);
                    if (iono_it != filter_state_.ionosphere_indices.end() &&
                        system_index >= 0) {
                        const double iono_delay_m = ppp_atmosphere::ionosphereDelayMetersFromTecu(
                            observation.primary_signal, eph, stec_tecu);
                        if (std::isfinite(iono_delay_m)) {
                            // MADOCALIB injects MIONO as a PPP pseudo-observation
                            // after removing a per-system common bias. Store the
                            // candidate here; formMeasurementEquations adds the
                            // biased constraint rows using the current iteration state.
                            observation.has_ionosphere_constraint = true;
                            observation.ionosphere_constraint_m = iono_delay_m;
                            observation.ionosphere_constraint_sigma_m = stec_sigma_m;
                            observation.atmospheric_iono_correction_m = ionosphere_correction_m;
                            applied_ssr_iono = true;
                        }
                    }
                } else if (!ppp_config_.estimate_ionosphere && use_stec_constraint &&
                    std::isfinite(ionosphere_correction_m) &&
                    std::abs(ionosphere_correction_m) > 0.0) {
                    // Direct observation correction (non-estimation mode)
                    observation.pseudorange_if -= ionosphere_correction_m;
                    if (observation.has_carrier_phase) {
                        observation.carrier_phase_if += ionosphere_correction_m;
                    }
                    observation.atmospheric_iono_correction_m = ionosphere_correction_m;
                    ++last_applied_atmos_iono_corrections_;
                    last_applied_atmos_iono_m_ += std::abs(ionosphere_correction_m);
                    applied_ssr_iono = true;
                }
            }
        }

        applySatelliteAntennaOffset();
        const auto geometry = nav.calculateGeometry(receiver_position, sat_position);
        diagnostic.receiver_position_x_m = receiver_position.x();
        diagnostic.receiver_position_y_m = receiver_position.y();
        diagnostic.receiver_position_z_m = receiver_position.z();
        diagnostic.satellite_position_x_m = sat_position.x();
        diagnostic.satellite_position_y_m = sat_position.y();
        diagnostic.satellite_position_z_m = sat_position.z();
        diagnostic.satellite_clock_bias_m = sat_clock_bias * constants::SPEED_OF_LIGHT;
        diagnostic.geometric_range_m = geometry.distance;
        if (std::isfinite(geometry.distance) && geometry.distance > 0.0) {
            const Vector3d line_of_sight = (sat_position - receiver_position) / geometry.distance;
            diagnostic.line_of_sight_x = line_of_sight.x();
            diagnostic.line_of_sight_y = line_of_sight.y();
            diagnostic.line_of_sight_z = line_of_sight.z();
        }
        diagnostic.elevation_deg = geometry.elevation / kDegreesToRadians;
        if (!std::isfinite(geometry.distance) || geometry.elevation < elevation_mask) {
            observation.valid = false;
            diagnostic.variance_pr = safeVariance(observation.variance_pr + deferred_variance_pr, 1e-6);
            diagnostic.variance_cp = safeVariance(observation.variance_cp + deferred_variance_cp, 1e-8);
            pushDiagnostic();
            continue;
        }
        if (ppp_config_.require_ssr_orbit_clock && ssr_products_loaded_ &&
            !have_precise && !diagnostic.orbit_clock_applied) {
            if (diagnostic.orbit_clock_skip_reason.empty()) {
                diagnostic.orbit_clock_skip_reason = "ssr_orbit_clock_required";
            }
            observation.valid = false;
            diagnostic.variance_pr = safeVariance(observation.variance_pr + deferred_variance_pr, 1e-6);
            diagnostic.variance_cp = safeVariance(observation.variance_cp + deferred_variance_cp, 1e-8);
            pushDiagnostic();
            continue;
        }

        if (observation.has_ionosphere_constraint) {
            ++last_applied_atmos_iono_corrections_;
            last_applied_atmos_iono_m_ += std::abs(observation.atmospheric_iono_correction_m);
            diagnostic.ionosphere_estimation_constraint = true;
        }

        if (observation.has_carrier_phase) {
            double& previous_windup_cycles = windup_cache_[observation.satellite];
            const double phase_windup_cycles = calculatePhaseWindupCycles(
                time,
                receiver_position,
                sat_position,
                sat_velocity,
                previous_windup_cycles);
            previous_windup_cycles = phase_windup_cycles;
            auto signal_windup_m = [&](SignalType signal) -> double {
                const double wavelength = signalWavelengthMeters(signal, eph);
                return wavelength > 0.0 ? phase_windup_cycles * wavelength : 0.0;
            };
            double phase_windup_m = 0.0;
            if (!ppp_config_.use_ionosphere_free ||
                observation.secondary_signal == SignalType::SIGNAL_TYPE_COUNT) {
                phase_windup_m = signal_windup_m(observation.primary_signal);
            } else {
                phase_windup_m =
                    observation.primary_code_bias_coeff * signal_windup_m(observation.primary_signal) +
                    observation.secondary_code_bias_coeff * signal_windup_m(observation.secondary_signal);
            }
            if (std::isfinite(phase_windup_m) && std::abs(phase_windup_m) > 0.0) {
                observation.carrier_phase_if -= phase_windup_m;
            }
        }

        const double receiver_antenna_pcv_m =
            calculateReceiverAntennaPcvMeters(observation, geometry.elevation);
        if (std::isfinite(receiver_antenna_pcv_m) &&
            std::abs(receiver_antenna_pcv_m) > 0.0) {
            observation.pseudorange_if -= receiver_antenna_pcv_m;
            if (observation.has_carrier_phase) {
                observation.carrier_phase_if -= receiver_antenna_pcv_m;
            }
        }

        if (!applied_ssr_code_bias && dcb_products_loaded_) {
            const double dcb_bias_m = observationDcbBiasMeters(
                dcb_products_,
                observation.satellite,
                observation.primary_signal,
                observation.secondary_signal,
                observation.primary_observation_code,
                observation.secondary_observation_code,
                ppp_config_.use_ionosphere_free,
                observation.primary_code_bias_coeff,
                observation.secondary_code_bias_coeff);
            if (std::isfinite(dcb_bias_m) && std::abs(dcb_bias_m) > 0.0) {
                observation.pseudorange_if -= dcb_bias_m;
                observation.pseudorange_code_bias_m += dcb_bias_m;
                ++last_applied_dcb_corrections_;
                last_applied_dcb_m_ += std::abs(dcb_bias_m);
                diagnostic.dcb_applied = true;
                diagnostic.dcb_bias_m = dcb_bias_m;
            }
        }

        if (!applied_ssr_iono && ionex_products_loaded_) {
            double ipp_lat_deg = 0.0;
            double ipp_lon_deg = 0.0;
            double mapping_factor = 0.0;
            double vertical_tecu = 0.0;
            if (ionexPiercePointAndMapping(
                    ionex_products_,
                    receiver_position,
                    geometry.azimuth,
                    geometry.elevation,
                    ipp_lat_deg,
                    ipp_lon_deg,
                    mapping_factor) &&
                ionex_products_.interpolateTecu(time, ipp_lat_deg, ipp_lon_deg, vertical_tecu, nullptr)) {
                const double stec_tecu = mapping_factor * vertical_tecu;
                const double ionosphere_correction_m = ppp_atmosphere::observationIonosphereDelayMeters(
                    eph,
                    observation.primary_signal,
                    observation.secondary_signal,
                    ppp_config_.use_ionosphere_free,
                    stec_tecu,
                    observation.primary_code_bias_coeff,
                    observation.secondary_code_bias_coeff);
                if (std::isfinite(stec_tecu)) {
                    diagnostic.stec_tecu = stec_tecu;
                }
                if (std::isfinite(ionosphere_correction_m)) {
                    diagnostic.iono_correction_m += ionosphere_correction_m;
                }
                if (std::isfinite(ionosphere_correction_m) &&
                    std::abs(ionosphere_correction_m) > 0.0) {
                    observation.pseudorange_if -= ionosphere_correction_m;
                    if (observation.has_carrier_phase) {
                        observation.carrier_phase_if += ionosphere_correction_m;
                    }
                    observation.atmospheric_iono_correction_m += ionosphere_correction_m;
                    ++last_applied_atmos_iono_corrections_;
                    last_applied_atmos_iono_m_ += std::abs(ionosphere_correction_m);
                    ++last_applied_ionex_corrections_;
                    last_applied_ionex_m_ += std::abs(ionosphere_correction_m);
                    diagnostic.ionex_applied = true;
                    diagnostic.ionex_iono_m += ionosphere_correction_m;
                }
            }
        }

        observation.satellite_position = sat_position;
        observation.satellite_velocity = sat_velocity;
        observation.satellite_clock_bias = sat_clock_bias;
        observation.satellite_clock_drift = sat_clock_drift;
        observation.elevation = geometry.elevation;
        observation.azimuth = geometry.azimuth;
        observation.trop_mapping =
            calculateMappingFunction(receiver_position, geometry.elevation, time);
        observation.modeled_trop_delay_m =
            modeledTroposphereDelayMeters(receiver_position, geometry.elevation, time);
        observation.modeled_zenith_trop_delay_m =
            modeledZenithTroposphereDelayMeters(receiver_position, time);
        if (ppp_config_.use_rtklib_measurement_variance && !precise_products_loaded_) {
            observation.variance_pr = measurementVariance(observation, false);
            observation.variance_cp = measurementVariance(observation, true);
        }
        observation.variance_pr = safeVariance(observation.variance_pr + deferred_variance_pr, 1e-6);
        observation.variance_cp = safeVariance(observation.variance_cp + deferred_variance_cp, 1e-8);
        observation.receiver_position = receiver_position;
        if (geometry.distance > 0.0) {
            const Vector3d los_unit = (sat_position - receiver_position) / geometry.distance;
            observation.antenna_pco_m = los_unit.dot(receiver_antenna_offset_ecef);
        } else {
            observation.antenna_pco_m = 0.0;
        }
        // In CLAS-PPP mode (estimate_ionosphere), reject satellites without
        // both SSR orbit/clock and ionosphere corrections — matching CLASLIB's
        // corrmeas() which returns 0 when STEC correction fails.
        observation.valid = true;
        diagnostic.variance_pr = observation.variance_pr;
        diagnostic.variance_cp = observation.variance_cp;
        diagnostic.valid_after_corrections = true;
        pushDiagnostic();
    }
}

double PPPProcessor::measurementVariance(const IonosphereFreeObs& observation,
                                         bool carrier_phase) const {
    const double sin_elevation = std::sin(std::max(observation.elevation, kRtkLibMinElevationRadians));
    double factor = rtklibSystemErrorFactor(observation.satellite.system);
    if (usesGpsL5ErrorFactor(observation.primary_signal) ||
        usesGpsL5ErrorFactor(observation.secondary_signal)) {
        factor *= kRtkLibGpsL5ErrorFactor;
    }
    if (ppp_config_.use_ionosphere_free) {
        factor *= 3.0;
    }
    if (!carrier_phase) {
        const double ratio =
            observation.secondary_signal == SignalType::SIGNAL_TYPE_COUNT ?
                ppp_config_.code_phase_error_ratio_l1 :
                ppp_config_.code_phase_error_ratio_l2;
        factor *= ratio > 0.0 ? ratio : ppp_config_.code_phase_error_ratio_l1;
    }
    return safeVariance(
        std::pow(factor * ppp_config_.rtklib_phase_error_m, 2.0) +
            std::pow(factor * ppp_config_.rtklib_phase_error_elevation_m / sin_elevation, 2.0),
        carrier_phase ? 1e-8 : 1e-6);
}

void PPPProcessor::detectCycleSlips(const ObservationData& obs) {
    if (!ppp_config_.enable_cycle_slip_detection) {
        return;
    }

    constexpr double kMinimumGeometryFreeSlipThresholdMeters = 0.5;
    constexpr double kMinimumMwSlipThresholdMeters = 10.0;
    // Enable combination (GF/MW) slip detection in SSR mode even for static,
    // because MW averaging is needed for Wide-Lane AR.
    const bool use_combination_slip_detection =
        ppp_config_.kinematic_mode || (ssr_products_loaded_ && ppp_config_.use_ionosphere_free);

    for (const auto& satellite : obs.getSatellites()) {
        const std::vector<SignalType> primary_candidates =
            satellite.system == GNSSSystem::GPS ?
                std::vector<SignalType>{SignalType::GPS_L1CA, SignalType::GPS_L1P} :
                primarySignals(satellite.system);
        const std::vector<SignalType> secondary_candidates =
            satellite.system == GNSSSystem::GPS ?
                std::vector<SignalType>{SignalType::GPS_L2P, SignalType::GPS_L2C, SignalType::GPS_L5} :
                secondarySignals(satellite.system);
        const Observation* primary =
            findCarrierObservationForSignals(obs, satellite, primary_candidates);
        if (primary == nullptr) {
            if (pppDebugEnabled() && satellite.system == GNSSSystem::GPS) {
                // Debug: list all observations for this satellite
                std::cerr << "[PPP-SLIP] " << satellite.toString() << " primary=null, obs: ";
                for (const auto& m : obs.observations) {
                    if (m.satellite == satellite) {
                        std::cerr << static_cast<int>(m.signal) << "(cp=" << m.has_carrier_phase
                                  << ",v=" << m.valid << ",L=" << m.carrier_phase << ") ";
                    }
                }
                std::cerr << "\n";
            }
            continue;
        }

        auto& ambiguity = ambiguity_states_[satellite];
        bool lli_slip = false;
        for (const auto& measurement : obs.observations) {
            if (!(measurement.satellite == satellite) ||
                !measurement.valid ||
                !measurement.has_carrier_phase) {
                continue;
            }
            if (measurement.loss_of_lock) {
                lli_slip = true;
                break;
            }
        }

        bool gf_slip = false;
        bool mw_slip = false;
        bool have_gf = false;
        bool have_mw = false;
        double gf_m = 0.0;
        double mw_m = 0.0;

        const Observation* secondary =
            use_combination_slip_detection ?
                findCarrierObservationForSignals(obs, satellite, secondary_candidates) :
                nullptr;
        if (secondary != nullptr) {
            const double lambda1 = signalWavelengthMeters(*primary);
            const double lambda2 = signalWavelengthMeters(*secondary);
            if (lambda1 > 0.0 && lambda2 > 0.0) {
                gf_m = primary->carrier_phase * lambda1 - secondary->carrier_phase * lambda2;
                have_gf = std::isfinite(gf_m);
                if (have_gf &&
                    ambiguity.has_last_geometry_free &&
                    std::abs(gf_m - ambiguity.last_geometry_free_m) >
                        std::max(ppp_config_.cycle_slip_threshold,
                                 kMinimumGeometryFreeSlipThresholdMeters)) {
                    gf_slip = true;
                }
            }

            if (satellite.system != GNSSSystem::GLONASS &&
                primary->has_pseudorange && secondary->has_pseudorange &&
                std::isfinite(primary->pseudorange) && std::isfinite(secondary->pseudorange)) {
                const double f1 = signalFrequencyHz(*primary);
                const double f2 = signalFrequencyHz(*secondary);
                if (f1 > 0.0 && f2 > 0.0 && std::abs(f1 - f2) >= 1.0) {
                    mw_m = ppp_utils::calculateMelbourneWubbena(
                        primary->carrier_phase,
                        secondary->carrier_phase,
                        primary->pseudorange,
                        secondary->pseudorange,
                        f1,
                        f2);
                    have_mw = std::isfinite(mw_m);
                    if (have_mw &&
                        ambiguity.has_last_melbourne_wubbena &&
                        std::abs(mw_m - ambiguity.last_melbourne_wubbena_m) >
                            std::max(kMinimumMwSlipThresholdMeters,
                                     ppp_config_.cycle_slip_threshold * 100.0)) {
                        mw_slip = true;
                    }
                }
            }

        }

        if (lli_slip || gf_slip || mw_slip) {
            if (pppDebugEnabled()) {
                std::string reason;
                if (lli_slip) {
                    reason += "lli";
                }
                if (gf_slip) {
                    if (!reason.empty()) {
                        reason += "+";
                    }
                    reason += "geometry-free";
                }
                if (mw_slip) {
                    if (!reason.empty()) {
                        reason += "+";
                    }
                    reason += "melbourne-wubbena";
                }
                std::cerr << "[PPP] cycle slip reset " << satellite.toString()
                          << " reason=" << reason;
                if (have_gf) {
                    std::cerr << " gf_m=" << gf_m;
                }
                if (have_mw) {
                    std::cerr << " mw_m=" << mw_m;
                }
                std::cerr << "\n";
            }
            resetAmbiguity(satellite, primary->signal);
        }

        auto& updated_ambiguity = ambiguity_states_[satellite];
        if (have_gf) {
            updated_ambiguity.last_geometry_free_m = gf_m;
            updated_ambiguity.has_last_geometry_free = true;
        }
        if (have_mw) {
            updated_ambiguity.last_melbourne_wubbena_m = mw_m;
            updated_ambiguity.has_last_melbourne_wubbena = true;
            // Accumulate MW average for Wide-Lane AR
            constexpr double lambda_wl_gps = constants::SPEED_OF_LIGHT / (1575.42e6 - 1227.60e6);
            const double mw_cycles = mw_m / lambda_wl_gps;
            if (!mw_slip) {
                updated_ambiguity.mw_sum_cycles += mw_cycles;
                updated_ambiguity.mw_count += 1;
                updated_ambiguity.mw_mean_cycles =
                    updated_ambiguity.mw_sum_cycles / updated_ambiguity.mw_count;
            } else {
                // Reset MW averaging on cycle slip
                updated_ambiguity.mw_sum_cycles = mw_cycles;
                updated_ambiguity.mw_count = 1;
                updated_ambiguity.mw_mean_cycles = mw_cycles;
                updated_ambiguity.wl_is_fixed = false;
            }
        }
    }
}

void PPPProcessor::ensureIonosphereStates(const std::vector<IonosphereFreeObs>& observations) {
    if (!ppp_config_.estimate_ionosphere) {
        return;
    }
    for (const auto& observation : observations) {
        if (!observation.valid || !isUsablePppSatellite(observation.satellite)) {
            continue;
        }
        getOrCreateIonosphereState(observation);
    }
}

int PPPProcessor::getOrCreateIonosphereState(const IonosphereFreeObs& observation) {
    const auto existing = filter_state_.ionosphere_indices.find(observation.satellite);
    if (existing != filter_state_.ionosphere_indices.end()) {
        return existing->second;
    }

    const int new_index = filter_state_.total_states;
    filter_state_.total_states += 1;
    filter_state_.state.conservativeResize(filter_state_.total_states);
    filter_state_.state(new_index) =
        observation.has_ionosphere_delay_seed ? observation.ionosphere_delay_seed_m : 0.0;
    filter_state_.covariance.conservativeResize(filter_state_.total_states, filter_state_.total_states);
    filter_state_.covariance.row(new_index).setZero();
    filter_state_.covariance.col(new_index).setZero();
    filter_state_.covariance(new_index, new_index) = ppp_config_.initial_ionosphere_variance;
    filter_state_.ionosphere_indices[observation.satellite] = new_index;
    return new_index;
}

void PPPProcessor::ensureAmbiguityStates(const std::vector<IonosphereFreeObs>& observations) {
    for (const auto& observation : observations) {
        if (!observation.valid || !observation.has_carrier_phase) {
            continue;
        }
        getOrCreateAmbiguityState(observation);
    }
}

void PPPProcessor::reinitializeVectorState(int start_index,
                                           const Vector3d& value,
                                           double variance) {
    for (int offset = 0; offset < 3; ++offset) {
        const int index = start_index + offset;
        filter_state_.state(index) = value(offset);
        filter_state_.covariance.row(index).setZero();
        filter_state_.covariance.col(index).setZero();
        filter_state_.covariance(index, index) = variance;
    }
}

void PPPProcessor::reinitializeScalarState(int index, double value, double variance) {
    filter_state_.state(index) = value;
    filter_state_.covariance.row(index).setZero();
    filter_state_.covariance.col(index).setZero();
    filter_state_.covariance(index, index) = variance;
}

bool PPPProcessor::useLowDynamicsBroadcastSeedAssist() const {
    return ppp_config_.kinematic_mode &&
           ppp_config_.low_dynamics_mode &&
           !precise_products_loaded_;
}

void PPPProcessor::recoverLowDynamicsBroadcastState(const ObservationData& obs,
                                                    const PositionSolution* seed_solution) {
    if (!useLowDynamicsBroadcastSeedAssist()) {
        return;
    }

    Vector3d recovered_position = static_anchor_position_;
    double recovered_clock_bias_m = 0.0;
    bool have_seed = false;
    if (seed_solution != nullptr && seed_solution->isValid()) {
        recovered_position = seed_solution->position_ecef;
        recovered_clock_bias_m = seed_solution->receiver_clock_bias;
        have_seed = true;
    } else if (validReceiverSeed(obs.receiver_position)) {
        recovered_position = obs.receiver_position;
        recovered_clock_bias_m = obs.receiver_clock_bias * constants::SPEED_OF_LIGHT;
        have_seed = true;
    }
    if (!have_seed) {
        return;
    }

    if (!has_static_anchor_position_) {
        static_anchor_position_ = recovered_position;
        has_static_anchor_position_ = true;
    }

    const Vector3d anchored_position =
        has_static_anchor_position_ ?
            0.85 * static_anchor_position_ + 0.15 * recovered_position :
            recovered_position;
    reinitializeVectorState(
        filter_state_.pos_index,
        anchored_position,
        std::min(ppp_config_.initial_position_variance, 36.0));
    const double recovered_gps_clock_bias_m = seededReceiverClockBiasMeters(
        seed_solution, ReceiverClockBiasGroup::GPS, recovered_clock_bias_m);
    reinitializeScalarState(
        filter_state_.clock_index,
        recovered_gps_clock_bias_m,
        ppp_config_.initial_clock_variance);
    reinitializeScalarState(
        filter_state_.glo_clock_index,
        seededReceiverClockBiasMeters(
            seed_solution, ReceiverClockBiasGroup::GLONASS, recovered_gps_clock_bias_m),
        ppp_config_.initial_clock_variance);
    if (filter_state_.bds2_clock_index >= 0) {
        reinitializeScalarState(
            filter_state_.bds2_clock_index,
            seededReceiverClockBiasMeters(
                seed_solution, ReceiverClockBiasGroup::BeiDou2, recovered_gps_clock_bias_m),
            ppp_config_.initial_clock_variance);
    }
    if (filter_state_.bds3_clock_index >= 0) {
        reinitializeScalarState(
            filter_state_.bds3_clock_index,
            seededReceiverClockBiasMeters(
                seed_solution, ReceiverClockBiasGroup::BeiDou3, recovered_gps_clock_bias_m),
            ppp_config_.initial_clock_variance);
    }
    reinitializeScalarState(
        filter_state_.trop_index,
        modeledZenithTroposphereDelayMeters(anchored_position, obs.time),
        ppp_config_.initial_troposphere_variance);

    std::vector<SatelliteId> satellites_to_reset;
    satellites_to_reset.reserve(ambiguity_states_.size());
    for (const auto& [satellite, ambiguity] : ambiguity_states_) {
        (void)ambiguity;
        satellites_to_reset.push_back(satellite);
    }
    for (const auto& satellite : satellites_to_reset) {
        resetAmbiguity(satellite, SignalType::GPS_L1CA);
    }

    recent_positions_.clear();
    converged_ = false;
    convergence_start_time_ = obs.time;
    constrainStaticVelocityStates();
    constrainStaticAnchorPosition();
}

int PPPProcessor::receiverClockStateIndex(const SatelliteId& satellite) const {
    switch (satellite.system) {
        case GNSSSystem::GLONASS:
            return filter_state_.glo_clock_index;
        case GNSSSystem::Galileo:
            return filter_state_.gal_clock_index >= 0
                ? filter_state_.gal_clock_index : filter_state_.clock_index;
        case GNSSSystem::QZSS:
            return filter_state_.qzs_clock_index >= 0
                ? filter_state_.qzs_clock_index : filter_state_.clock_index;
        case GNSSSystem::BeiDou:
            if (signal_policy::isBeiDou2Satellite(satellite) &&
                filter_state_.bds2_clock_index >= 0) {
                return filter_state_.bds2_clock_index;
            }
            if (signal_policy::isBeiDou3Satellite(satellite) &&
                filter_state_.bds3_clock_index >= 0) {
                return filter_state_.bds3_clock_index;
            }
            return filter_state_.bds_clock_index >= 0
                ? filter_state_.bds_clock_index : filter_state_.clock_index;
        default:
            return filter_state_.clock_index;
    }
}

double PPPProcessor::receiverClockBiasMeters(const SatelliteId& satellite) const {
    return filter_state_.state(receiverClockStateIndex(satellite));
}

int PPPProcessor::ambiguityStateIndex(const SatelliteId& satellite) const {
    const auto it = filter_state_.ambiguity_indices.find(satellite);
    return it == filter_state_.ambiguity_indices.end() ? -1 : it->second;
}

int PPPProcessor::getOrCreateAmbiguityState(const IonosphereFreeObs& observation) {
    const bool use_frequency_state =
        ppp_config_.enable_per_frequency_phase_bias_states &&
        !ppp_config_.use_ionosphere_free &&
        observation.frequency_index > 0;
    const auto register_primary_frequency_ambiguity = [&](int state_index) {
        if (!ppp_config_.use_ionosphere_free) {
            filter_state_.frequency_ambiguity_indices[
                ppp_shared::frequencyAmbiguityKey(observation.satellite, 0)] = state_index;
        }
    };

    if (use_frequency_state) {
        const auto key = ppp_shared::frequencyAmbiguityKey(
            observation.satellite, observation.frequency_index);
        const auto existing_frequency = filter_state_.frequency_ambiguity_indices.find(key);
        if (existing_frequency != filter_state_.frequency_ambiguity_indices.end()) {
            const auto ambiguity_it = ambiguity_states_.find(observation.satellite);
            if (ambiguity_it != ambiguity_states_.end() &&
                ambiguity_it->second.needs_reinitialization) {
                initializeFrequencyAmbiguityState(observation, existing_frequency->second);
            }
            return existing_frequency->second;
        }

        const int new_index = filter_state_.total_states;
        filter_state_.total_states += 1;
        filter_state_.state.conservativeResize(filter_state_.total_states);
        filter_state_.state(new_index) = 0.0;
        filter_state_.covariance.conservativeResize(filter_state_.total_states, filter_state_.total_states);
        filter_state_.covariance.row(new_index).setZero();
        filter_state_.covariance.col(new_index).setZero();
        filter_state_.covariance(new_index, new_index) =
            precise_products_loaded_ ? 100.0 : ppp_config_.initial_ambiguity_variance;
        filter_state_.frequency_ambiguity_indices[key] = new_index;
        initializeFrequencyAmbiguityState(observation, new_index);
        return new_index;
    }

    const auto existing = filter_state_.ambiguity_indices.find(observation.satellite);
    if (existing != filter_state_.ambiguity_indices.end()) {
        auto& ambiguity = ambiguity_states_[observation.satellite];
        if (ambiguity.needs_reinitialization) {
            initializeAmbiguityState(observation, existing->second);
        }
        register_primary_frequency_ambiguity(existing->second);
        return existing->second;
    }

    const int new_index = filter_state_.total_states;
    filter_state_.total_states += 1;
    filter_state_.state.conservativeResize(filter_state_.total_states);
    filter_state_.state(new_index) = 0.0;
    filter_state_.covariance.conservativeResize(filter_state_.total_states, filter_state_.total_states);
    filter_state_.covariance.row(new_index).setZero();
    filter_state_.covariance.col(new_index).setZero();
    filter_state_.covariance(new_index, new_index) =
        precise_products_loaded_ ? 100.0 : ppp_config_.initial_ambiguity_variance;
    filter_state_.ambiguity_indices[observation.satellite] = new_index;
    register_primary_frequency_ambiguity(new_index);
    initializeAmbiguityState(observation, new_index);
    return new_index;
}

double PPPProcessor::phaseAmbiguityInitializerPrediction(
    const IonosphereFreeObs& observation,
    bool include_ionosphere_state) const {
    const Vector3d receiver_position =
        observation.receiver_position.norm() > 1000.0 ?
            observation.receiver_position :
            filter_state_.state.segment(filter_state_.pos_index, 3);
    const double clock_bias_m = receiverClockBiasMeters(observation.satellite);
    const double zenith_delay = filter_state_.state(filter_state_.trop_index);
    double predicted =
        geodist(observation.satellite_position, receiver_position) +
        clock_bias_m -
        constants::SPEED_OF_LIGHT * observation.satellite_clock_bias +
        (ppp_config_.estimate_troposphere ?
            estimatedTroposphereDelayFromState(
                observation.modeled_trop_delay_m,
                observation.trop_mapping,
                observation.modeled_zenith_trop_delay_m,
                zenith_delay) :
            observation.modeled_trop_delay_m);
    if (include_ionosphere_state && ppp_config_.estimate_ionosphere) {
        const auto iono_it = filter_state_.ionosphere_indices.find(observation.satellite);
        if (iono_it != filter_state_.ionosphere_indices.end()) {
            const int index = iono_it->second;
            if (index >= 0 && index < filter_state_.state.size()) {
                predicted -= observation.ionosphere_coefficient * filter_state_.state(index);
            }
        }
    }
    return predicted;
}

void PPPProcessor::initializeFrequencyAmbiguityState(
    const IonosphereFreeObs& observation,
    int state_index) {
    const bool include_ionosphere_state =
        ppp_config_.initialize_phase_ambiguity_with_ionosphere_state &&
        ppp_config_.estimate_ionosphere;
    const double predicted =
        phaseAmbiguityInitializerPrediction(observation, include_ionosphere_state);
    const double legacy_predicted = phaseAmbiguityInitializerPrediction(observation, false);
    filter_state_.state(state_index) = observation.carrier_phase_if - predicted;
    phase_ambiguity_admission_offsets_m_[state_index] =
        include_ionosphere_state ? (predicted - legacy_predicted) : 0.0;
    filter_state_.covariance(state_index, state_index) =
        precise_products_loaded_ ? 25.0 : ppp_config_.initial_ambiguity_variance;
}

void PPPProcessor::initializeAmbiguityState(const IonosphereFreeObs& observation, int state_index) {
    const bool include_ionosphere_state =
        ppp_config_.initialize_phase_ambiguity_with_ionosphere_state &&
        ppp_config_.estimate_ionosphere;
    const double predicted =
        phaseAmbiguityInitializerPrediction(observation, include_ionosphere_state);
    const double legacy_predicted = phaseAmbiguityInitializerPrediction(observation, false);
    filter_state_.state(state_index) = observation.carrier_phase_if - predicted;
    phase_ambiguity_admission_offsets_m_[state_index] =
        include_ionosphere_state ? (predicted - legacy_predicted) : 0.0;
    filter_state_.covariance(state_index, state_index) =
        precise_products_loaded_ ? 25.0 : ppp_config_.initial_ambiguity_variance;

    auto& ambiguity = ambiguity_states_[observation.satellite];
    ambiguity.float_value = filter_state_.state(state_index);
    ambiguity.is_fixed = false;
    ambiguity.lock_count = 0;
    ambiguity.quality_indicator = 0.0;
    ambiguity.ambiguity_scale_m = observation.ambiguity_scale_m;
    ambiguity.needs_reinitialization = false;
}

bool PPPProcessor::resolveAmbiguities(const ObservationData& obs, const NavigationData& nav) {
    last_ar_ratio_ = 0.0;
    last_fixed_ambiguities_ = 0;

    if (!ppp_config_.enable_ambiguity_resolution || (!precise_products_loaded_ && !ssr_products_loaded_)) {
        if (pppDebugEnabled()) {
            std::cerr << "[PPP-AR] skipped: enabled=" << ppp_config_.enable_ambiguity_resolution
                      << " precise=" << precise_products_loaded_ << "\n";
        }
        return false;
    }

    if (pppDebugEnabled()) {
        int total_amb = 0, ready_amb = 0;
        for (const auto& [satellite, state_index] : filter_state_.ambiguity_indices) {
            ++total_amb;
            const auto amb_it = ambiguity_states_.find(satellite);
            if (amb_it != ambiguity_states_.end()) {
                const auto& a = amb_it->second;
                if (!a.needs_reinitialization && a.lock_count >= ppp_config_.convergence_min_epochs &&
                    std::isfinite(a.ambiguity_scale_m) && a.ambiguity_scale_m > 0.0) {
                    ++ready_amb;
                }
            }
        }
        std::cerr << "[PPP-AR-DBG] total_amb=" << total_amb
                  << " ready=" << ready_amb
                  << " min_epochs=" << ppp_config_.convergence_min_epochs << "\n";
    }

    if (ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL) {
        // WLNL works with IFLC ambiguities (standard PPP) and also with
        // per-frequency ambiguities when CLAS OSR corrections are loaded
        // (NL float values are computed from corrected dual-freq observations).
        return resolveAmbiguitiesWLNL(obs, nav);
    }
    // DD_IFLC and DD_PER_FREQ fall through to existing DD-AR code below

    const int ar_min_lock = ssr_products_loaded_ ?
        std::min(ppp_config_.convergence_min_epochs, 10) :
        ppp_config_.convergence_min_epochs;
    const auto eligible_ambiguities = ppp_ar::collectEligibleAmbiguities(
        filter_state_, ambiguity_states_, ar_min_lock);

    if (static_cast<int>(eligible_ambiguities.satellites.size()) <
        ppp_config_.min_satellites_for_ar) {
        if (pppDebugEnabled()) {
            std::cerr << "[PPP-AR] skipped: candidates="
                      << eligible_ambiguities.satellites.size()
                      << " min=" << ppp_config_.min_satellites_for_ar << "\n";
        }
        return false;
    }

    std::map<SatelliteId, double> real_satellite_elevations;
    if (ppp_config_.use_clas_osr_filter) {
        for (const auto& satellite : eligible_ambiguities.satellites) {
            const SatelliteId real_satellite = ppp_ar::clasRealSatellite(satellite);
            if (real_satellite_elevations.find(real_satellite) != real_satellite_elevations.end()) {
                continue;
            }

            Vector3d sat_pos;
            Vector3d sat_vel;
            double sat_clk = 0.0;
            double sat_drift = 0.0;
            if (!nav.calculateSatelliteState(real_satellite, obs.time, sat_pos, sat_vel, sat_clk, sat_drift)) {
                continue;
            }
            if (ssr_products_loaded_) {
                Vector3d orbit_corr;
                double clock_corr = 0.0;
                if (ssr_products_.interpolateCorrection(real_satellite,
                                                        obs.time,
                                                        orbit_corr,
                                                        clock_corr,
                                                        nullptr,
                                                        nullptr,
                                                        nullptr,
                                                        nullptr,
                                                        nullptr,
                                                        nullptr,
                                                        nullptr,
                                                        0,
                                                        ppp_config_.allow_future_ssr_corrections,
                                                        ppp_config_.require_ssr_orbit_clock)) {
                    if (ssr_products_.orbitCorrectionsAreRac()) {
                        orbit_corr = ssrRacToEcef(sat_pos, sat_vel, orbit_corr);
                    }
                    sat_pos += orbit_corr;
                }
            }
            const Vector3d receiver_position =
                filter_state_.state.segment(filter_state_.pos_index, 3);
            const Vector3d line_of_sight = sat_pos - receiver_position;
            const double line_of_sight_norm = line_of_sight.norm();
            if (line_of_sight_norm <= 0.0 || receiver_position.norm() <= 0.0) {
                continue;
            }
            const double elevation = std::asin(
                line_of_sight.normalized().dot(receiver_position.normalized()));
            real_satellite_elevations[real_satellite] = elevation;
        }
    }

    ppp_ar::DdFixAttempt best_attempt = ppp_ar::tryDirectDdFixWithPar(
        ppp_config_,
        filter_state_,
        pre_anchor_covariance_,
        ambiguity_states_,
        eligible_ambiguities,
        real_satellite_elevations,
        pppDebugEnabled());

    if (!best_attempt.fixed) {
        if (pppDebugEnabled()) {
            std::cerr << "[PPP-AR] DD ratio reject: ratio=" << best_attempt.ratio
                      << " threshold=" << best_attempt.required_ratio << "\n";
        }
        return false;
    }

    last_ar_ratio_ = best_attempt.ratio;
    last_fixed_ambiguities_ = best_attempt.nb;
    filter_state_ = std::move(best_attempt.state);
    ambiguity_states_ = std::move(best_attempt.ambiguities);

    if (pppDebugEnabled()) {
        std::cerr << "[PPP-AR] DD fixed: nb=" << best_attempt.nb
                  << " ratio=" << best_attempt.ratio
                  << " threshold=" << best_attempt.required_ratio << "\n";
    }
    return true;
}

double PPPProcessor::calculateTroposphericDelay(const Vector3d& receiver_pos,
                                                const Vector3d& satellite_pos,
                                                const GNSSTime& time,
                                                double zenith_delay) const {
    double lat = 0.0;
    double lon = 0.0;
    double h = 0.0;
    ecef2geodetic(receiver_pos, lat, lon, h);
    const Vector3d los_enu = ecef2enu(satellite_pos - receiver_pos, lat, lon);
    const double horizontal = std::hypot(los_enu.x(), los_enu.y());
    const double elevation = std::atan2(los_enu.z(), horizontal);
    if (!ppp_config_.estimate_troposphere) {
        return modeledTroposphereDelayMeters(receiver_pos, elevation, time);
    }
    return estimatedTroposphereDelayMeters(receiver_pos, elevation, time, zenith_delay);
}

double PPPProcessor::calculateMappingFunction(const Vector3d& receiver_pos,
                                              double elevation,
                                              const GNSSTime& time) const {
    return wetTroposphereMapping(receiver_pos, elevation, time);
}

Vector3d PPPProcessor::applyGeophysicalCorrections(const Vector3d& position,
                                                   const GNSSTime& time) const {
    Vector3d corrected = position;
    if (ppp_config_.apply_solid_earth_tides) {
        corrected += calculateSolidEarthTides(position, time);
    }
    if (ppp_config_.apply_ocean_loading) {
        corrected += calculateOceanLoading(position, time);
    }
    return corrected;
}

Vector3d PPPProcessor::calculateSolidEarthTides(const Vector3d& position,
                                                const GNSSTime& time) const {
    constexpr double kSunGM = 1.32712440018e20;
    constexpr double kMoonGM = 4.902801e12;
    if (!position.allFinite() || position.norm() < constants::WGS84_A * 0.5) {
        return Vector3d::Zero();
    }
    const Vector3d sun_position = approximateSunPositionEcef(time);
    const Vector3d moon_position = approximateMoonPositionEcef(time);
    return bodyTideDisplacement(position, sun_position, kSunGM) +
           bodyTideDisplacement(position, moon_position, kMoonGM);
}

Vector3d PPPProcessor::calculateOceanLoading(const Vector3d& position,
                                             const GNSSTime& time) const {
    if (!ocean_loading_loaded_ || !position.allFinite() ||
        position.norm() < constants::WGS84_A * 0.5) {
        return Vector3d::Zero();
    }

    const auto system_time = time.toSystemTime();
    const double seconds_since_unix =
        std::chrono::duration<double>(system_time.time_since_epoch()).count();

    double up_m = 0.0;
    double west_m = 0.0;
    double south_m = 0.0;
    for (size_t i = 0; i < kOceanLoadingPeriodsSeconds.size(); ++i) {
        const double period_seconds = kOceanLoadingPeriodsSeconds[i];
        if (period_seconds <= 0.0) {
            continue;
        }
        const double angle_rad = 2.0 * M_PI * seconds_since_unix / period_seconds;
        up_m += ocean_loading_coefficients_.up_amplitudes_m[i] *
            std::cos(angle_rad - ocean_loading_coefficients_.up_phases_deg[i] * kDegreesToRadians);
        west_m += ocean_loading_coefficients_.west_amplitudes_m[i] *
            std::cos(angle_rad - ocean_loading_coefficients_.west_phases_deg[i] * kDegreesToRadians);
        south_m += ocean_loading_coefficients_.south_amplitudes_m[i] *
            std::cos(angle_rad - ocean_loading_coefficients_.south_phases_deg[i] * kDegreesToRadians);
    }

    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;
    ecef2geodetic(position, latitude_rad, longitude_rad, height_m);
    const Vector3d enu_offset(-west_m, -south_m, up_m);
    return enu2ecef(enu_offset, latitude_rad, longitude_rad);
}

PPPProcessor::MeasurementEquation PPPProcessor::formMeasurementEquations(
    const std::vector<IonosphereFreeObs>& observations,
    const NavigationData& nav,
    const GNSSTime& time) {
    (void)nav;

    std::vector<Eigen::RowVectorXd> rows;
    std::vector<double> measured_values;
    std::vector<double> predicted_values;
    std::vector<double> variances;
    std::vector<SatelliteId> row_satellites;
    std::vector<SignalType> row_primary_signals;
    std::vector<SignalType> row_secondary_signals;
    std::vector<std::string> row_primary_observation_codes;
    std::vector<std::string> row_secondary_observation_codes;
    std::vector<int> row_frequency_indices;
    std::vector<double> row_ionosphere_coefficients;
    std::vector<int> row_receiver_clock_state_indices;
    std::vector<double> row_receiver_clock_design_coefficients;
    std::vector<int> row_ionosphere_state_indices;
    std::vector<double> row_ionosphere_design_coefficients;
    std::vector<int> row_ambiguity_state_indices;
    std::vector<double> row_ambiguity_design_coefficients;
    std::vector<int> row_ambiguity_lock_counts;
    std::vector<int> row_required_lock_counts;
    std::vector<double> row_phase_limits_m;
    std::vector<std::string> row_phase_skip_reasons;
    std::vector<bool> row_is_phase;
    std::vector<bool> row_is_ionosphere_constraint;
    std::vector<double> row_elevation_deg;
    std::vector<double> row_iono_state_m;
    std::vector<PPPResidualDiagnostic> phase_candidate_diagnostics;
    std::set<std::pair<SatelliteId, int>> before_excluded_phase_pairs_to_reset;
    std::set<SatelliteId> satellites_with_phase_rows;

    const double zenith_delay = filter_state_.state(filter_state_.trop_index);
    const bool use_phase_rows =
        ppp_config_.use_carrier_phase_without_precise_products || precise_products_loaded_ || ssr_products_loaded_;

    for (const auto& observation : observations) {
        if (!observation.valid) {
            continue;
        }

        const Vector3d receiver_position =
            observation.receiver_position.norm() > 1000.0 ?
                observation.receiver_position :
                filter_state_.state.segment(filter_state_.pos_index, 3);

        const Vector3d range_vector = observation.satellite_position - receiver_position;
        const double euclidean_range = range_vector.norm();
        if (!std::isfinite(euclidean_range) || euclidean_range <= 1.0) {
            continue;
        }

        const double geometric_range = geodist(observation.satellite_position, receiver_position);
        const Vector3d line_of_sight = range_vector / euclidean_range;
        const double troposphere_delay =
            ppp_config_.estimate_troposphere ?
                estimatedTroposphereDelayFromState(
                    observation.modeled_trop_delay_m,
                    observation.trop_mapping,
                    observation.modeled_zenith_trop_delay_m,
                    zenith_delay) :
                observation.modeled_trop_delay_m;
        const int clock_state_index = receiverClockStateIndex(observation.satellite);
        const double clock_bias_m = receiverClockBiasMeters(observation.satellite);

        Eigen::RowVectorXd row = Eigen::RowVectorXd::Zero(filter_state_.total_states);
        row.segment(filter_state_.pos_index, 3) = -line_of_sight;
        row(clock_state_index) = 1.0;
        row(filter_state_.trop_index) =
            ppp_config_.estimate_troposphere ? observation.trop_mapping : 0.0;
        // Per-satellite ionosphere state: pseudorange has +iono contribution
        int ionosphere_state_index = -1;
        double iono_state_m = 0.0;
        double ionosphere_contribution_m = 0.0;
        if (ppp_config_.estimate_ionosphere) {
            const auto iono_it = filter_state_.ionosphere_indices.find(observation.satellite);
            if (iono_it != filter_state_.ionosphere_indices.end()) {
                ionosphere_state_index = iono_it->second;
                row(ionosphere_state_index) = observation.ionosphere_coefficient;
                iono_state_m = filter_state_.state(ionosphere_state_index);
                ionosphere_contribution_m =
                    observation.ionosphere_coefficient * iono_state_m;
            }
        }

        const double predicted =
            geometric_range + clock_bias_m -
            constants::SPEED_OF_LIGHT * observation.satellite_clock_bias + troposphere_delay
            + ionosphere_contribution_m;
        const double residual = observation.pseudorange_if - predicted;

        if (pppDebugEnabled()) {
            std::cerr << "[PPP-OBS] " << observation.satellite.toString()
                      << " pr=" << observation.pseudorange_if
                      << " geo=" << geometric_range
                      << " clk=" << clock_bias_m
                      << " satclk=" << (constants::SPEED_OF_LIGHT * observation.satellite_clock_bias)
                      << " trop=" << troposphere_delay
                      << " pred=" << predicted
                      << " resid=" << residual
                      << "\n";
        }

        if (ppp_config_.enable_outlier_detection) {
            const double sigma = std::sqrt(std::max(
                safeVariance(observation.variance_pr, 1e-6),
                (row * filter_state_.covariance * row.transpose())(0, 0)));
            const double code_residual_floor =
                precise_products_loaded_
                    ? (converged_ ? 500.0 : 50000.0)
                    : 20000.0;
            const double residual_limit =
                std::max(
                    ppp_config_.outlier_threshold * sigma * 10.0,
                    code_residual_floor);
            if (std::abs(residual) > residual_limit) {
                if (pppDebugEnabled()) {
                    std::cerr << "[PPP] reject code " << observation.satellite.toString()
                              << " pr_if=" << observation.pseudorange_if
                              << " pred=" << predicted
                              << " residual=" << residual
                              << " limit=" << residual_limit << "\n";
                }
                continue;
            }
        }

        rows.push_back(row);
        measured_values.push_back(observation.pseudorange_if);
        predicted_values.push_back(predicted);
        variances.push_back(safeVariance(observation.variance_pr, 1e-6));
        row_satellites.push_back(observation.satellite);
        row_primary_signals.push_back(observation.primary_signal);
        row_secondary_signals.push_back(observation.secondary_signal);
        row_primary_observation_codes.push_back(observation.primary_observation_code);
        row_secondary_observation_codes.push_back(observation.secondary_observation_code);
        row_frequency_indices.push_back(observation.frequency_index);
        row_ionosphere_coefficients.push_back(observation.ionosphere_coefficient);
        row_receiver_clock_state_indices.push_back(clock_state_index);
        row_receiver_clock_design_coefficients.push_back(1.0);
        row_ionosphere_state_indices.push_back(ionosphere_state_index);
        row_ionosphere_design_coefficients.push_back(
            ionosphere_state_index >= 0 ? observation.ionosphere_coefficient : 0.0);
        row_ambiguity_state_indices.push_back(-1);
        row_ambiguity_design_coefficients.push_back(0.0);
        row_ambiguity_lock_counts.push_back(-1);
        row_required_lock_counts.push_back(0);
        row_phase_limits_m.push_back(0.0);
        row_phase_skip_reasons.emplace_back();
        row_is_phase.push_back(false);
        row_is_ionosphere_constraint.push_back(false);
        row_elevation_deg.push_back(observation.elevation / kDegreesToRadians);
        row_iono_state_m.push_back(iono_state_m);

        if (use_phase_rows && observation.has_carrier_phase) {
            int ambiguity_index = ambiguityStateIndex(observation.satellite);
            if (ppp_config_.enable_per_frequency_phase_bias_states &&
                !ppp_config_.use_ionosphere_free &&
                observation.frequency_index > 0) {
                const auto frequency_ambiguity_it =
                    filter_state_.frequency_ambiguity_indices.find(
                        ppp_shared::frequencyAmbiguityKey(
                            observation.satellite, observation.frequency_index));
                ambiguity_index =
                    frequency_ambiguity_it == filter_state_.frequency_ambiguity_indices.end() ?
                        -1 : frequency_ambiguity_it->second;
            }

            const int required_lock_count =
                precise_products_loaded_ ?
                    ppp_config_.convergence_min_epochs :
                    ppp_config_.phase_measurement_min_lock_count;
            const auto ambiguity_it = ambiguity_states_.find(observation.satellite);
            const int ambiguity_lock_count =
                ambiguity_it == ambiguity_states_.end() ? -1 : ambiguity_it->second.lock_count;
            const bool ambiguity_needs_reinitialization =
                ambiguity_it == ambiguity_states_.end() || ambiguity_it->second.needs_reinitialization;
            const bool lock_count_ready =
                ambiguity_it != ambiguity_states_.end() &&
                !ambiguity_it->second.needs_reinitialization &&
                ambiguity_it->second.lock_count >= required_lock_count;
            bool phase_ready = lock_count_ready;
            int effective_required_lock_count = required_lock_count;

            PPPResidualDiagnostic phase_candidate;
            phase_candidate.satellite = observation.satellite;
            phase_candidate.primary_signal = observation.primary_signal;
            phase_candidate.secondary_signal = observation.secondary_signal;
            phase_candidate.primary_observation_code = observation.primary_observation_code;
            phase_candidate.secondary_observation_code = observation.secondary_observation_code;
            phase_candidate.frequency_index = observation.frequency_index;
            phase_candidate.ionosphere_coefficient = observation.ionosphere_coefficient;
            phase_candidate.carrier_phase = true;
            phase_candidate.phase_candidate = true;
            phase_candidate.phase_ready = phase_ready;
            phase_candidate.phase_accepted = false;
            phase_candidate.receiver_clock_state_index = clock_state_index;
            phase_candidate.receiver_clock_design_coeff = 1.0;
            phase_candidate.ionosphere_state_index = ionosphere_state_index;
            phase_candidate.ionosphere_design_coeff =
                ionosphere_state_index >= 0 ? -observation.ionosphere_coefficient : 0.0;
            phase_candidate.ambiguity_state_index = ambiguity_index;
            phase_candidate.ambiguity_design_coeff =
                ambiguity_index >= 0 ? 1.0 : 0.0;
            phase_candidate.ambiguity_lock_count = ambiguity_lock_count;
            phase_candidate.required_lock_count = required_lock_count;
            phase_candidate.observation_m = observation.carrier_phase_if;
            phase_candidate.variance_m2 = safeVariance(observation.variance_cp, 1e-8);
            phase_candidate.elevation_deg = observation.elevation / kDegreesToRadians;
            phase_candidate.iono_state_m = iono_state_m;

            if (ambiguity_index < 0 || ambiguity_index >= filter_state_.total_states) {
                phase_candidate.phase_skip_reason = "no_ambiguity_state";
                phase_candidate_diagnostics.push_back(phase_candidate);
                continue;
            }

            // predicted for phase: geo + clk - satclk + trop - iono + amb
            // (predicted already includes +iono, so subtract 2*iono for phase)
            double iono_phase_correction = 0.0;
            if (ppp_config_.estimate_ionosphere) {
                const auto iono_it = filter_state_.ionosphere_indices.find(observation.satellite);
                if (iono_it != filter_state_.ionosphere_indices.end()) {
                    iono_phase_correction =
                        -2.0 * observation.ionosphere_coefficient *
                        filter_state_.state(iono_it->second);
                }
            }
            const double ambiguity_state = filter_state_.state(ambiguity_index);
            const double predicted_phase = predicted + iono_phase_correction + ambiguity_state;
            const double phase_residual = observation.carrier_phase_if - predicted_phase;
            double predicted_phase_for_gate = predicted_phase;
            double phase_gate_residual = phase_residual;
            const auto admission_offset_it =
                phase_ambiguity_admission_offsets_m_.find(ambiguity_index);
            if (admission_offset_it != phase_ambiguity_admission_offsets_m_.end() &&
                admission_offset_it->second != 0.0) {
                predicted_phase_for_gate =
                    predicted + iono_phase_correction + ambiguity_state +
                    admission_offset_it->second;
                phase_gate_residual = observation.carrier_phase_if - predicted_phase_for_gate;
            }
            double phase_residual_floor =
                ppp_config_.kinematic_mode
                    ? (converged_ ? 20.0 :
                                      ppp_config_.kinematic_preconvergence_phase_residual_floor_m)
                    : (converged_ ? 10.0 : 50.0);
            const auto phase_admission_pair =
                std::make_pair(observation.satellite, observation.frequency_index);
            const auto residual_floor_it =
                ppp_config_.phase_admission_residual_floor_by_satellite_frequency_pair.find(
                    phase_admission_pair);
            if (residual_floor_it !=
                ppp_config_.phase_admission_residual_floor_by_satellite_frequency_pair.end()) {
                phase_residual_floor = residual_floor_it->second;
            }
            const double phase_limit =
                std::max(
                    ppp_config_.outlier_threshold *
                        std::sqrt(safeVariance(observation.variance_cp, 1e-8)) * 10.0,
                    phase_residual_floor);
            phase_candidate.predicted_m = predicted_phase_for_gate;
            phase_candidate.residual_m = phase_gate_residual;
            phase_candidate.phase_limit_m = phase_limit;

            if (ppp_config_.phase_admission_excluded_satellite_frequency_pairs.count(
                    phase_admission_pair) > 0) {
                phase_candidate.phase_skip_reason = "excluded_sat_frequency_pair";
                phase_candidate_diagnostics.push_back(phase_candidate);
                continue;
            }
            const auto before_exclusion_it =
                ppp_config_.phase_admission_excluded_before_by_satellite_frequency_pair.find(
                    phase_admission_pair);
            if (before_exclusion_it !=
                    ppp_config_.phase_admission_excluded_before_by_satellite_frequency_pair.end() &&
                time < before_exclusion_it->second) {
                phase_candidate.phase_skip_reason = "excluded_sat_frequency_pair_before_time";
                phase_candidate_diagnostics.push_back(phase_candidate);
                if (ppp_config_.reset_phase_ambiguity_on_before_exclusion) {
                    before_excluded_phase_pairs_to_reset.insert(phase_admission_pair);
                }
                continue;
            }

            const bool warm_start_system_allowed =
                ppp_config_.initial_phase_admission_warm_start_systems.empty() ||
                ppp_config_.initial_phase_admission_warm_start_systems.count(
                    observation.satellite.system) > 0;
            const bool warm_start_satellite_allowed =
                ppp_config_.initial_phase_admission_warm_start_satellites.empty() ||
                ppp_config_.initial_phase_admission_warm_start_satellites.count(
                    observation.satellite) > 0;
            const bool warm_start_frequency_allowed =
                ppp_config_.initial_phase_admission_warm_start_frequency_indexes.empty() ||
                ppp_config_.initial_phase_admission_warm_start_frequency_indexes.count(
                    observation.frequency_index) > 0;
            const bool warm_start_satellite_frequency_allowed =
                ppp_config_.initial_phase_admission_warm_start_satellite_frequency_pairs.empty() ||
                ppp_config_.initial_phase_admission_warm_start_satellite_frequency_pairs.count(
                    {observation.satellite, observation.frequency_index}) > 0;
            const bool warm_start_common =
                ssr_products_loaded_ && !precise_products_loaded_ &&
                !ppp_config_.use_ionosphere_free &&
                warm_start_system_allowed &&
                warm_start_satellite_allowed &&
                warm_start_frequency_allowed &&
                warm_start_satellite_frequency_allowed &&
                ambiguity_it != ambiguity_states_.end() &&
                !ambiguity_needs_reinitialization &&
                ambiguity_lock_count == 0 &&
                required_lock_count > 0 &&
                std::abs(phase_gate_residual) <= phase_limit;
            const bool initial_phase_warm_start =
                warm_start_common &&
                ((ppp_config_.enable_initial_phase_admission_warm_start &&
                  observation.frequency_index == 0 &&
                  observation.satellite.system != GNSSSystem::Galileo) ||
                 ppp_config_.enable_all_frequency_initial_phase_admission_warm_start);
            if (initial_phase_warm_start) {
                phase_ready = true;
                effective_required_lock_count = 0;
                phase_candidate.phase_ready = true;
                phase_candidate.required_lock_count = effective_required_lock_count;
            }

            if (!phase_ready) {
                if (ambiguity_it == ambiguity_states_.end()) {
                    phase_candidate.phase_skip_reason = "missing_ambiguity";
                } else if (ambiguity_needs_reinitialization) {
                    phase_candidate.phase_skip_reason = "ambiguity_reinitializing";
                } else {
                    phase_candidate.phase_skip_reason = "lock_count";
                }
                phase_candidate_diagnostics.push_back(phase_candidate);
                continue;
            }
            if (ppp_config_.enable_outlier_detection &&
                std::abs(phase_gate_residual) > phase_limit) {
                phase_candidate.phase_skip_reason = "outlier";
                phase_candidate_diagnostics.push_back(phase_candidate);
                continue;
            }

            phase_ambiguity_admission_offsets_m_.erase(ambiguity_index);

            Eigen::RowVectorXd phase_row = Eigen::RowVectorXd::Zero(filter_state_.total_states);
            phase_row.segment(filter_state_.pos_index, 3) = -line_of_sight;
            phase_row(clock_state_index) = 1.0;
            phase_row(filter_state_.trop_index) =
                ppp_config_.estimate_troposphere ? observation.trop_mapping : 0.0;
            // Per-satellite ionosphere: carrier phase has -iono contribution
            if (ppp_config_.estimate_ionosphere && ionosphere_state_index >= 0) {
                phase_row(ionosphere_state_index) = -observation.ionosphere_coefficient;
            }
            phase_row(ambiguity_index) = 1.0;

            rows.push_back(phase_row);
            measured_values.push_back(observation.carrier_phase_if);
            predicted_values.push_back(predicted_phase);
            variances.push_back(safeVariance(observation.variance_cp, 1e-8));
            row_satellites.push_back(observation.satellite);
            row_primary_signals.push_back(observation.primary_signal);
            row_secondary_signals.push_back(observation.secondary_signal);
            row_primary_observation_codes.push_back(observation.primary_observation_code);
            row_secondary_observation_codes.push_back(observation.secondary_observation_code);
            row_frequency_indices.push_back(observation.frequency_index);
            row_ionosphere_coefficients.push_back(observation.ionosphere_coefficient);
            row_receiver_clock_state_indices.push_back(clock_state_index);
            row_receiver_clock_design_coefficients.push_back(1.0);
            row_ionosphere_state_indices.push_back(ionosphere_state_index);
            row_ionosphere_design_coefficients.push_back(
                ionosphere_state_index >= 0 ? -observation.ionosphere_coefficient : 0.0);
            row_ambiguity_state_indices.push_back(ambiguity_index);
            row_ambiguity_design_coefficients.push_back(1.0);
            row_ambiguity_lock_counts.push_back(ambiguity_lock_count);
            row_required_lock_counts.push_back(effective_required_lock_count);
            row_phase_limits_m.push_back(phase_limit);
            row_phase_skip_reasons.emplace_back();
            row_is_phase.push_back(true);
            row_is_ionosphere_constraint.push_back(false);
            satellites_with_phase_rows.insert(observation.satellite);
            row_elevation_deg.push_back(observation.elevation / kDegreesToRadians);
            row_iono_state_m.push_back(iono_state_m);
        }
    }

    if (ppp_config_.estimate_ionosphere) {
        constexpr int kConstraintSystemCount = 4;
        std::array<double, kConstraintSystemCount> system_bias_sum{};
        std::array<int, kConstraintSystemCount> system_bias_count{};
        for (const auto& observation : observations) {
            if (!observation.valid || !observation.has_ionosphere_constraint) {
                continue;
            }
            const int system_index =
                ionosphereConstraintSystemIndex(observation.satellite.system);
            const auto iono_it = filter_state_.ionosphere_indices.find(observation.satellite);
            if (system_index < 0 || iono_it == filter_state_.ionosphere_indices.end()) {
                continue;
            }
            const int state_index = iono_it->second;
            if (state_index < 0 || state_index >= filter_state_.state.size()) {
                continue;
            }
            system_bias_sum[static_cast<size_t>(system_index)] +=
                observation.ionosphere_constraint_m - filter_state_.state(state_index);
            ++system_bias_count[static_cast<size_t>(system_index)];
        }

        for (const auto& observation : observations) {
            if (!observation.valid || !observation.has_ionosphere_constraint) {
                continue;
            }
            const int system_index =
                ionosphereConstraintSystemIndex(observation.satellite.system);
            const auto iono_it = filter_state_.ionosphere_indices.find(observation.satellite);
            if (system_index < 0 || iono_it == filter_state_.ionosphere_indices.end()) {
                continue;
            }
            const int state_index = iono_it->second;
            if (state_index < 0 || state_index >= filter_state_.total_states) {
                continue;
            }
            const size_t bias_index = static_cast<size_t>(system_index);
            if (system_bias_count[bias_index] <= 0) {
                continue;
            }
            const double system_bias_m =
                system_bias_sum[bias_index] / static_cast<double>(system_bias_count[bias_index]);
            const double constrained_delay_m =
                observation.ionosphere_constraint_m - system_bias_m;
            if (!std::isfinite(constrained_delay_m) ||
                !std::isfinite(observation.ionosphere_constraint_sigma_m) ||
                observation.ionosphere_constraint_sigma_m <= 0.0) {
                continue;
            }

            Eigen::RowVectorXd iono_row = Eigen::RowVectorXd::Zero(filter_state_.total_states);
            iono_row(state_index) = 1.0;
            const double iono_state_m = filter_state_.state(state_index);
            rows.push_back(iono_row);
            measured_values.push_back(constrained_delay_m);
            predicted_values.push_back(iono_state_m);
            variances.push_back(safeVariance(
                observation.ionosphere_constraint_sigma_m *
                    observation.ionosphere_constraint_sigma_m,
                1e-6));
            row_satellites.push_back(observation.satellite);
            row_primary_signals.push_back(observation.primary_signal);
            row_secondary_signals.push_back(observation.secondary_signal);
            row_primary_observation_codes.push_back(observation.primary_observation_code);
            row_secondary_observation_codes.push_back(observation.secondary_observation_code);
            row_frequency_indices.push_back(observation.frequency_index);
            row_ionosphere_coefficients.push_back(observation.ionosphere_coefficient);
            row_receiver_clock_state_indices.push_back(-1);
            row_receiver_clock_design_coefficients.push_back(0.0);
            row_ionosphere_state_indices.push_back(state_index);
            row_ionosphere_design_coefficients.push_back(1.0);
            row_ambiguity_state_indices.push_back(-1);
            row_ambiguity_design_coefficients.push_back(0.0);
            row_ambiguity_lock_counts.push_back(-1);
            row_required_lock_counts.push_back(0);
            row_phase_limits_m.push_back(0.0);
            row_phase_skip_reasons.emplace_back();
            row_is_phase.push_back(false);
            row_is_ionosphere_constraint.push_back(true);
            row_elevation_deg.push_back(observation.elevation / kDegreesToRadians);
            row_iono_state_m.push_back(iono_state_m);
        }
    }

    for (const auto& [satellite, frequency_index] : before_excluded_phase_pairs_to_reset) {
        (void)frequency_index;
        if (satellites_with_phase_rows.count(satellite) == 0) {
            resetAmbiguity(satellite, SignalType::SIGNAL_TYPE_COUNT);
        }
    }

    MeasurementEquation equation;
    const int n_obs = static_cast<int>(rows.size());
    equation.design_matrix = MatrixXd::Zero(n_obs, filter_state_.total_states);
    equation.observations = VectorXd::Zero(n_obs);
    equation.predicted = VectorXd::Zero(n_obs);
    equation.weight_matrix = MatrixXd::Zero(n_obs, n_obs);
    equation.residuals = VectorXd::Zero(n_obs);

    for (int i = 0; i < n_obs; ++i) {
        equation.design_matrix.row(i) = rows[static_cast<size_t>(i)];
        equation.observations(i) = measured_values[static_cast<size_t>(i)];
        equation.predicted(i) = predicted_values[static_cast<size_t>(i)];
        equation.weight_matrix(i, i) = variances[static_cast<size_t>(i)];
        equation.residuals(i) = measured_values[static_cast<size_t>(i)] -
                                predicted_values[static_cast<size_t>(i)];
    }
    equation.row_satellites = row_satellites;
    equation.row_primary_signals = row_primary_signals;
    equation.row_secondary_signals = row_secondary_signals;
    equation.row_primary_observation_codes = row_primary_observation_codes;
    equation.row_secondary_observation_codes = row_secondary_observation_codes;
    equation.row_frequency_indices = row_frequency_indices;
    equation.row_ionosphere_coefficients = row_ionosphere_coefficients;
    equation.row_receiver_clock_state_indices = row_receiver_clock_state_indices;
    equation.row_receiver_clock_design_coefficients = row_receiver_clock_design_coefficients;
    equation.row_ionosphere_state_indices = row_ionosphere_state_indices;
    equation.row_ionosphere_design_coefficients = row_ionosphere_design_coefficients;
    equation.row_ambiguity_state_indices = row_ambiguity_state_indices;
    equation.row_ambiguity_design_coefficients = row_ambiguity_design_coefficients;
    equation.row_ambiguity_lock_counts = row_ambiguity_lock_counts;
    equation.row_required_lock_counts = row_required_lock_counts;
    equation.row_phase_limits_m = row_phase_limits_m;
    equation.row_phase_skip_reasons = row_phase_skip_reasons;
    equation.row_is_phase = row_is_phase;
    equation.row_is_ionosphere_constraint = row_is_ionosphere_constraint;
    equation.row_elevation_deg = row_elevation_deg;
    equation.row_iono_state_m = row_iono_state_m;
    equation.phase_candidate_diagnostics = phase_candidate_diagnostics;
    return equation;
}

void PPPProcessor::checkConvergence(const GNSSTime& current_time) {
    if (converged_) {
        return;
    }

    recent_positions_.push_back(filter_state_.state.segment(filter_state_.pos_index, 3));
    if (recent_positions_.size() > static_cast<size_t>(ppp_config_.convergence_min_epochs)) {
        recent_positions_.erase(recent_positions_.begin());
    }

    if (recent_positions_.size() < static_cast<size_t>(ppp_config_.convergence_min_epochs)) {
        return;
    }

    Vector3d mean = Vector3d::Zero();
    for (const auto& position : recent_positions_) {
        mean += position;
    }
    mean /= static_cast<double>(recent_positions_.size());

    double max_deviation = 0.0;
    for (const auto& position : recent_positions_) {
        max_deviation = std::max(max_deviation, (position - mean).norm());
    }

    if (max_deviation < ppp_config_.convergence_threshold_horizontal) {
        converged_ = true;
        convergence_time_ = current_time - convergence_start_time_;
    }
}

PositionSolution PPPProcessor::generateSolution(const GNSSTime& time,
                                                const std::vector<IonosphereFreeObs>& observations) {
    PositionSolution solution;
    solution.time = time;
    solution.status = SolutionStatus::PPP_FLOAT;
    solution.num_satellites = 0;
    solution.num_frequencies = ppp_config_.use_ionosphere_free ? 2 : 1;

    if (!filter_initialized_) {
        return solution;
    }

    solution.position_ecef = filter_state_.state.segment(filter_state_.pos_index, 3);
    solution.receiver_clock_bias =
        filter_state_.state(filter_state_.clock_index) / constants::SPEED_OF_LIGHT;
    solution.receiver_clock_biases_m[ReceiverClockBiasGroup::GPS] =
        filter_state_.state(filter_state_.clock_index);
    solution.receiver_clock_biases_m[ReceiverClockBiasGroup::GLONASS] =
        filter_state_.state(filter_state_.glo_clock_index);
    if (filter_state_.gal_clock_index >= 0) {
        solution.receiver_clock_biases_m[ReceiverClockBiasGroup::Galileo] =
            filter_state_.state(filter_state_.gal_clock_index);
    }
    if (filter_state_.qzs_clock_index >= 0) {
        solution.receiver_clock_biases_m[ReceiverClockBiasGroup::QZSS] =
            filter_state_.state(filter_state_.qzs_clock_index);
    }
    if (filter_state_.bds_clock_index >= 0) {
        solution.receiver_clock_biases_m[ReceiverClockBiasGroup::BeiDou] =
            filter_state_.state(filter_state_.bds_clock_index);
    }
    if (filter_state_.bds2_clock_index >= 0) {
        solution.receiver_clock_biases_m[ReceiverClockBiasGroup::BeiDou2] =
            filter_state_.state(filter_state_.bds2_clock_index);
    }
    if (filter_state_.bds3_clock_index >= 0) {
        solution.receiver_clock_biases_m[ReceiverClockBiasGroup::BeiDou3] =
            filter_state_.state(filter_state_.bds3_clock_index);
    }
    solution.position_covariance =
        filter_state_.covariance.block(filter_state_.pos_index, filter_state_.pos_index, 3, 3);

    double latitude = 0.0;
    double longitude = 0.0;
    double height = 0.0;
    ecef2geodetic(solution.position_ecef, latitude, longitude, height);
    solution.position_geodetic = GeodeticCoord(latitude, longitude, height);

    std::set<SatelliteId> counted_satellites;
    bool saw_secondary_frequency = false;
    for (const auto& observation : observations) {
        if (!observation.valid) {
            continue;
        }
        if (counted_satellites.insert(observation.satellite).second) {
            solution.satellites_used.push_back(observation.satellite);
            solution.satellite_elevations.push_back(observation.elevation);
        }
        if (observation.secondary_signal != SignalType::SIGNAL_TYPE_COUNT) {
            saw_secondary_frequency = true;
        } else {
            const auto primary_candidates = primarySignals(observation.satellite.system);
            saw_secondary_frequency = saw_secondary_frequency ||
                (std::find(primary_candidates.begin(), primary_candidates.end(),
                           observation.primary_signal) == primary_candidates.end());
        }
        const double geometric_range = geodist(observation.satellite_position, solution.position_ecef);
        const double clock_bias_m = receiverClockBiasMeters(observation.satellite);
        const double predicted = geometric_range +
            clock_bias_m -
            constants::SPEED_OF_LIGHT * observation.satellite_clock_bias +
            (ppp_config_.estimate_troposphere
                ? estimatedTroposphereDelayFromState(
                    observation.modeled_trop_delay_m,
                    observation.trop_mapping,
                    observation.modeled_zenith_trop_delay_m,
                    filter_state_.state(filter_state_.trop_index))
                : observation.modeled_trop_delay_m);
        solution.satellite_residuals.push_back(observation.pseudorange_if - predicted);
    }
    solution.num_satellites = static_cast<int>(counted_satellites.size());
    if (!ppp_config_.use_ionosphere_free) {
        solution.num_frequencies = saw_secondary_frequency ? 2 : 1;
    }

    if (!solution.satellite_residuals.empty()) {
        double sum_sq = 0.0;
        for (const double residual : solution.satellite_residuals) {
            sum_sq += residual * residual;
        }
        solution.residual_rms =
            std::sqrt(sum_sq / static_cast<double>(solution.satellite_residuals.size()));
    }

    return solution;
}

Vector3d PPPProcessor::calculatePositionAccuracy() const {
    if (!filter_initialized_) {
        return Vector3d::Constant(std::numeric_limits<double>::infinity());
    }
    return Vector3d(
        std::sqrt(std::max(0.0, filter_state_.covariance(filter_state_.pos_index + 0, filter_state_.pos_index + 0))),
        std::sqrt(std::max(0.0, filter_state_.covariance(filter_state_.pos_index + 1, filter_state_.pos_index + 1))),
        std::sqrt(std::max(0.0, filter_state_.covariance(filter_state_.pos_index + 2, filter_state_.pos_index + 2))));
}

void PPPProcessor::updateAmbiguityStates(const ObservationData& obs) {
    for (const auto& measurement : obs.observations) {
        if (!measurement.valid || !measurement.has_carrier_phase) {
            continue;
        }
        auto& ambiguity = ambiguity_states_[measurement.satellite];
        ambiguity.last_phase = measurement.carrier_phase;
        ambiguity.last_time = obs.time;
        ambiguity.lock_count++;
        ambiguity.quality_indicator = measurement.snr;
        const int ambiguity_index = ambiguityStateIndex(measurement.satellite);
        if (ambiguity_index >= 0 && ambiguity_index < filter_state_.total_states) {
            ambiguity.float_value = filter_state_.state(ambiguity_index);
            if (!ppp_config_.kinematic_mode &&
                precise_products_loaded_ &&
                ambiguity.ambiguity_scale_m > 0.0) {
                const double cycles = ambiguity.float_value / ambiguity.ambiguity_scale_m;
                const double fractional_cycles = wrapFractionalCycle(cycles);
                if (ambiguity.fractional_bias_samples == 0) {
                    ambiguity.fractional_bias_cycles = fractional_cycles;
                    ambiguity.fractional_bias_samples = 1;
                } else {
                    double aligned_fraction = fractional_cycles;
                    while (aligned_fraction - ambiguity.fractional_bias_cycles > 0.5) {
                        aligned_fraction -= 1.0;
                    }
                    while (aligned_fraction - ambiguity.fractional_bias_cycles < -0.5) {
                        aligned_fraction += 1.0;
                    }
                    ambiguity.fractional_bias_cycles =
                        wrapFractionalCycle(
                            (ambiguity.fractional_bias_cycles *
                                 static_cast<double>(ambiguity.fractional_bias_samples) +
                             aligned_fraction) /
                            static_cast<double>(ambiguity.fractional_bias_samples + 1));
                    ambiguity.fractional_bias_samples++;
                }
            }
        }
    }
}

void PPPProcessor::resetAmbiguity(const SatelliteId& satellite, SignalType signal) {
    (void)signal;
    auto& ambiguity = ambiguity_states_[satellite];
    ambiguity = PPPAmbiguityInfo{};
    ambiguity.needs_reinitialization = true;

    const auto reset_state_index = [&](int ambiguity_index) {
        phase_ambiguity_admission_offsets_m_.erase(ambiguity_index);
        if (ambiguity_index >= 0 && ambiguity_index < filter_state_.total_states) {
            filter_state_.state(ambiguity_index) = 0.0;
            filter_state_.covariance.row(ambiguity_index).setZero();
            filter_state_.covariance.col(ambiguity_index).setZero();
            filter_state_.covariance(ambiguity_index, ambiguity_index) =
                precise_products_loaded_ ? 1e6 : ppp_config_.initial_ambiguity_variance;
        }
    };

    reset_state_index(ambiguityStateIndex(satellite));
    for (const auto& [key, state_index] : filter_state_.frequency_ambiguity_indices) {
        if (key.first == satellite) {
            reset_state_index(state_index);
        }
    }

    const auto sat_it = ambiguity_states_.find(satellite);
    if (sat_it == ambiguity_states_.end()) {
        return;
    }
}

void PPPProcessor::constrainStaticVelocityStates() {
    if ((ppp_config_.kinematic_mode && !ppp_config_.low_dynamics_mode) ||
        filter_state_.total_states < filter_state_.vel_index + 3) {
        return;
    }

    filter_state_.state.segment(filter_state_.vel_index, 3).setZero();
    for (int axis = 0; axis < 3; ++axis) {
        const int idx = filter_state_.vel_index + axis;
        filter_state_.covariance.row(idx).setZero();
        filter_state_.covariance.col(idx).setZero();
        filter_state_.covariance(idx, idx) = 1e-12;
    }
}

void PPPProcessor::constrainStaticAnchorPosition() {
    if ((ppp_config_.kinematic_mode && !ppp_config_.low_dynamics_mode) ||
        !has_static_anchor_position_) {
        return;
    }
    if (precise_products_loaded_ && !ppp_config_.estimate_troposphere) {
        return;
    }
    // SSR mode note: hard blend (70% SPP) caps accuracy at ~4m but prevents
    // divergence. Soft anchor (pseudo-observation) tested with sigma=3-20m:
    //   3m: 4.3m (worse than hard), 5m: 4.8m, 10m: 6.2m, inf: 10.6m (diverges)
    // Root cause of 4m floor is 20-40m SSR residual spread (light travel time).
    // Fix the residuals first, then loosen the anchor.

    const double anchor_blend =
        ppp_config_.low_dynamics_mode
            ? (precise_products_loaded_ ? (converged_ ? 0.65 : 0.9) : 0.95)
            : (precise_products_loaded_ ? (converged_ ? 0.5 : 0.85)
               : (ssr_products_loaded_
                    ? (converged_ ? 0.5 : 0.7)
                    : 1.0));
    filter_state_.state.segment(filter_state_.pos_index, 3) =
        anchor_blend * static_anchor_position_ +
        (1.0 - anchor_blend) * filter_state_.state.segment(filter_state_.pos_index, 3);
    if (precise_products_loaded_) {
        Vector3d deviation =
            filter_state_.state.segment(filter_state_.pos_index, 3) - static_anchor_position_;
        const double deviation_norm = deviation.norm();
        const double max_static_deviation_m =
            ppp_config_.low_dynamics_mode
                ? (converged_ ? 15.0 : 8.0)
                : (converged_ ? 100.0 : 25.0);
        if (std::isfinite(deviation_norm) && deviation_norm > max_static_deviation_m) {
            deviation *= max_static_deviation_m / deviation_norm;
            filter_state_.state.segment(filter_state_.pos_index, 3) =
                static_anchor_position_ + deviation;
        }
    } else if (ppp_config_.low_dynamics_mode) {
        Vector3d deviation =
            filter_state_.state.segment(filter_state_.pos_index, 3) - static_anchor_position_;
        const double deviation_norm = deviation.norm();
        const double max_static_deviation_m = converged_ ? 10.0 : 6.0;
        if (std::isfinite(deviation_norm) && deviation_norm > max_static_deviation_m) {
            deviation *= max_static_deviation_m / deviation_norm;
            filter_state_.state.segment(filter_state_.pos_index, 3) =
                static_anchor_position_ + deviation;
        }
    }
    filter_state_.covariance.block(filter_state_.pos_index, filter_state_.pos_index, 3, 3) =
        MatrixXd::Identity(3, 3) *
        (ppp_config_.low_dynamics_mode
             ? (precise_products_loaded_ ? 4.0 : 9.0)
             : (precise_products_loaded_ ? 9.0 : 4.0));
    for (int axis = 0; axis < 3; ++axis) {
        const int idx = filter_state_.pos_index + axis;
        for (int col = 0; col < filter_state_.total_states; ++col) {
            if (col >= filter_state_.pos_index && col < filter_state_.pos_index + 3) {
                continue;
            }
            filter_state_.covariance(idx, col) = 0.0;
            filter_state_.covariance(col, idx) = 0.0;
        }
    }

    const double min_trop = 0.5;
    const double max_trop = precise_products_loaded_ ? 30.0 : 100.0;
    filter_state_.state(filter_state_.trop_index) =
        std::clamp(filter_state_.state(filter_state_.trop_index), min_trop, max_trop);
    filter_state_.covariance(filter_state_.trop_index, filter_state_.trop_index) =
        std::min(filter_state_.covariance(filter_state_.trop_index, filter_state_.trop_index), 25.0);
}


VectorXd PPPProcessor::calculateResiduals(const std::vector<IonosphereFreeObs>& observations,
                                          const NavigationData& nav,
                                          const GNSSTime& time) const {
    (void)nav;
    (void)time;
    std::vector<double> residuals;
    residuals.reserve(observations.size());

    const Vector3d receiver_position =
        filter_state_.state.segment(filter_state_.pos_index, 3);
    const double zenith_delay = filter_state_.state(filter_state_.trop_index);

    for (const auto& observation : observations) {
        if (!observation.valid) {
            continue;
        }
        const double clock_bias_m = receiverClockBiasMeters(observation.satellite);
        const double predicted = geodist(observation.satellite_position, receiver_position) +
            clock_bias_m -
            constants::SPEED_OF_LIGHT * observation.satellite_clock_bias +
            (ppp_config_.estimate_troposphere ?
                estimatedTroposphereDelayFromState(
                    observation.modeled_trop_delay_m,
                    observation.trop_mapping,
                    observation.modeled_zenith_trop_delay_m,
                    zenith_delay) :
                observation.modeled_trop_delay_m);
        residuals.push_back(observation.pseudorange_if - predicted);
    }

    VectorXd residual_vector = VectorXd::Zero(static_cast<int>(residuals.size()));
    for (size_t i = 0; i < residuals.size(); ++i) {
        residual_vector(static_cast<int>(i)) = residuals[i];
    }
    return residual_vector;
}

void PPPProcessor::updateStatistics(bool converged) const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    const_cast<size_t&>(total_epochs_processed_)++;
    if (converged) {
        const_cast<size_t&>(converged_solutions_)++;
    }
}

namespace ppp_utils {

std::pair<double, double> getIonosphereFreeCoefficients(double f1, double f2) {
    const double denom = f1 * f1 - f2 * f2;
    if (!std::isfinite(denom) || std::abs(denom) < 1.0) {
        return {1.0, 0.0};
    }
    return {
        (f1 * f1) / denom,
        -(f2 * f2) / denom,
    };
}

double calculateMelbourneWubbena(double l1_phase,
                                 double l2_phase,
                                 double p1_range,
                                 double p2_range,
                                 double f1,
                                 double f2) {
    if (f1 <= 0.0 || f2 <= 0.0 || std::abs(f1 - f2) < 1.0) {
        return 0.0;
    }
    return (l1_phase - l2_phase) * constants::SPEED_OF_LIGHT / (f1 - f2) -
           (f1 * p1_range + f2 * p2_range) / (f1 + f2);
}

double calculateGeometryFree(double l1_phase, double l2_phase) {
    return l1_phase - l2_phase;
}

bool interpolatePreciseOrbit(const std::vector<PreciseOrbitClock>& orbit_data,
                             const GNSSTime& time,
                             Vector3d& position,
                             Vector3d& velocity) {
    if (orbit_data.empty()) {
        return false;
    }
    auto upper = std::lower_bound(
        orbit_data.begin(), orbit_data.end(), time,
        [](const PreciseOrbitClock& lhs, const GNSSTime& rhs) {
            return lhs.time < rhs;
        });
    if (upper == orbit_data.begin()) {
        if (!upper->position_valid) {
            return false;
        }
        position = upper->position;
        velocity = upper->velocity;
        return true;
    }
    if (upper == orbit_data.end()) {
        const auto& last = orbit_data.back();
        if (!last.position_valid) {
            return false;
        }
        position = last.position;
        velocity = last.velocity;
        return true;
    }

    const auto& before = *(upper - 1);
    const auto& after = *upper;
    if (!before.position_valid || !after.position_valid) {
        return false;
    }
    const double dt = after.time - before.time;
    if (std::abs(dt) < 1e-9) {
        position = before.position;
        velocity = before.velocity;
        return true;
    }
    const double alpha = (time - before.time) / dt;
    position = before.position + alpha * (after.position - before.position);
    velocity = (after.position - before.position) / dt;
    return true;
}

bool interpolatePreciseClock(const std::vector<PreciseOrbitClock>& clock_data,
                             const GNSSTime& time,
                             double& clock_bias,
                             double& clock_drift) {
    if (clock_data.empty()) {
        return false;
    }
    auto upper = std::lower_bound(
        clock_data.begin(), clock_data.end(), time,
        [](const PreciseOrbitClock& lhs, const GNSSTime& rhs) {
            return lhs.time < rhs;
        });
    if (upper == clock_data.begin()) {
        if (!upper->clock_valid) {
            return false;
        }
        clock_bias = upper->clock_bias;
        clock_drift = upper->clock_drift;
        return true;
    }
    if (upper == clock_data.end()) {
        const auto& last = clock_data.back();
        if (!last.clock_valid) {
            return false;
        }
        clock_bias = last.clock_bias;
        clock_drift = last.clock_drift;
        return true;
    }

    const auto& before = *(upper - 1);
    const auto& after = *upper;
    if (!before.clock_valid || !after.clock_valid) {
        return false;
    }
    const double dt = after.time - before.time;
    if (std::abs(dt) < 1e-9) {
        clock_bias = before.clock_bias;
        clock_drift = before.clock_drift;
        return true;
    }
    const double alpha = (time - before.time) / dt;
    clock_bias = before.clock_bias + alpha * (after.clock_bias - before.clock_bias);
    clock_drift = (after.clock_bias - before.clock_bias) / dt;
    return true;
}

Vector3d calculateSatelliteAntennaPCO(const SatelliteId& satellite,
                                      const Vector3d& satellite_pos,
                                      const Vector3d& sun_pos) {
    (void)satellite;
    (void)satellite_pos;
    (void)sun_pos;
    return Vector3d::Zero();
}

Vector3d calculateReceiverAntennaPCO(const Vector3d& receiver_pos,
                                     const Vector3d& satellite_pos,
                                     const std::string& antenna_type) {
    (void)receiver_pos;
    (void)satellite_pos;
    (void)antenna_type;
    return Vector3d::Zero();
}

double calculatePhaseWindup(const Vector3d& receiver_pos,
                            const Vector3d& satellite_pos,
                            double previous_windup) {
    // Phase wind-up correction based on Wu et al. (1993)
    // Returns accumulated phase wind-up in cycles.
    const Vector3d e_sr = (receiver_pos - satellite_pos).normalized();

    // Satellite frame: use sun direction for yaw steering
    // Simplified: assume satellite Z points to Earth center, X in sun direction
    const Vector3d e_z_sat = -satellite_pos.normalized();
    // Sun direction (simplified: assume along +X in ECEF for now)
    // A proper implementation would use solar ephemeris.
    // Using cross-track direction as approximation:
    Vector3d e_x_sat = e_sr.cross(e_z_sat);
    if (e_x_sat.norm() < 1e-10) return previous_windup;
    e_x_sat.normalize();
    const Vector3d e_y_sat = e_z_sat.cross(e_x_sat);

    // Receiver frame: Z = up, X = east, Y = north (approximate)
    const Vector3d e_z_rcv = receiver_pos.normalized();
    Vector3d e_x_rcv = Vector3d(-receiver_pos.y(), receiver_pos.x(), 0.0);
    if (e_x_rcv.norm() < 1e-10) return previous_windup;
    e_x_rcv.normalize();
    const Vector3d e_y_rcv = e_z_rcv.cross(e_x_rcv);

    // Effective dipole vectors
    const Vector3d d_sat = e_x_sat - e_sr * (e_sr.dot(e_x_sat))
                         + e_sr.cross(e_y_sat);
    const Vector3d d_rcv = e_x_rcv - e_sr * (e_sr.dot(e_x_rcv))
                         + e_sr.cross(e_y_rcv);

    const double cos_phi = d_sat.dot(d_rcv) / (d_sat.norm() * d_rcv.norm() + 1e-20);
    const double cross_sign = e_sr.dot(d_sat.cross(d_rcv));
    double dphi = std::acos(std::clamp(cos_phi, -1.0, 1.0)) / (2.0 * M_PI);
    if (cross_sign < 0.0) dphi = -dphi;

    // Accumulate full cycles
    double windup = previous_windup + dphi;
    // Remove large jumps (> 0.5 cycle)
    while (windup - previous_windup > 0.5) windup -= 1.0;
    while (windup - previous_windup < -0.5) windup += 1.0;

    return windup;
}

}  // namespace ppp_utils

}  // namespace libgnss
