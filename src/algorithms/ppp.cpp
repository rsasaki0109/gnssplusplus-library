#include <libgnss++/algorithms/ppp.hpp>
#include <libgnss++/algorithms/ppp_ar.hpp>
#include <libgnss++/algorithms/ppp_atmosphere.hpp>
#include <libgnss++/algorithms/ppp_clas.hpp>
#include <libgnss++/algorithms/ppp_osr.hpp>
#include <libgnss++/algorithms/ppp_utils.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/iers/earth_rotation.hpp>
#include <libgnss++/iers/ephemeris.hpp>
#include <libgnss++/iers/sub_daily_eop.hpp>
#include <libgnss++/iers/tides.hpp>
#include <libgnss++/io/qzss_l6.hpp>
#include <libgnss++/io/rtcm.hpp>
#include <libgnss++/models/troposphere.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <fstream>
#include <memory>
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

GNSSTime parseAntexEpochLine(const std::string& line) {
    int year = 0, month = 0, day = 0, hour = 0, minute = 0;
    double second = 0.0;
    std::istringstream iss(line.substr(0, std::min<size_t>(line.size(), 60U)));
    if (!(iss >> year >> month >> day >> hour >> minute >> second)) {
        return GNSSTime{};
    }
    std::tm tm{};
    tm.tm_year = year - 1900;
    tm.tm_mon = month - 1;
    tm.tm_mday = day;
    tm.tm_hour = hour;
    tm.tm_min = minute;
    tm.tm_sec = static_cast<int>(second);
    const std::time_t calendar = timegm(&tm);
    if (calendar == static_cast<std::time_t>(-1)) {
        return GNSSTime{};
    }
    return GNSSTime::fromSystemTime(
        std::chrono::system_clock::from_time_t(calendar) +
        std::chrono::microseconds(static_cast<long>((second - tm.tm_sec) * 1e6)));
}

bool loadSatelliteAntexOffsets(
    const std::string& filename,
    std::vector<SatelliteAntexEntry>& satellite_entries) {
    satellite_entries.clear();
    std::ifstream input(filename);
    if (!input.is_open()) {
        return false;
    }
    SatelliteAntexEntry current;
    bool in_antenna = false;
    bool satellite_entry = false;
    SignalType current_signal = SignalType::SIGNAL_TYPE_COUNT;
    std::string line;
    while (std::getline(input, line)) {
        const std::string label = line.size() >= 60 ? trimCopy(line.substr(60)) : "";
        if (label == "START OF ANTENNA") {
            current = SatelliteAntexEntry{};
            satellite_entry = false;
            current_signal = SignalType::SIGNAL_TYPE_COUNT;
            in_antenna = true;
            continue;
        }
        if (!in_antenna) continue;
        if (label == "END OF ANTENNA") {
            if (satellite_entry && !current.body_offsets_m.empty()) {
                satellite_entries.push_back(current);
            }
            in_antenna = false;
            continue;
        }
        if (label == "TYPE / SERIAL NO") {
            // Satellite entries place the PRN at columns 20-22 (3-char
            // SVS code: G01, R26, E07, ...). Receiver entries put a
            // free-form serial there.
            const std::string serial = trimCopy(line.substr(20, 20));
            satellite_entry =
                serial.size() == 3 &&
                std::isalpha(static_cast<unsigned char>(serial[0])) &&
                std::isdigit(static_cast<unsigned char>(serial[1])) &&
                std::isdigit(static_cast<unsigned char>(serial[2]));
            if (satellite_entry) {
                GNSSSystem system = GNSSSystem::UNKNOWN;
                switch (serial[0]) {
                    case 'G': system = GNSSSystem::GPS; break;
                    case 'R': system = GNSSSystem::GLONASS; break;
                    case 'E': system = GNSSSystem::Galileo; break;
                    case 'C': system = GNSSSystem::BeiDou; break;
                    case 'J': system = GNSSSystem::QZSS; break;
                    case 'S': system = GNSSSystem::SBAS; break;
                    case 'I': system = GNSSSystem::NavIC; break;
                    default: break;
                }
                if (system == GNSSSystem::UNKNOWN) {
                    satellite_entry = false;
                } else {
                    try {
                        current.satellite = SatelliteId(
                            system,
                            static_cast<uint8_t>(std::stoi(serial.substr(1))));
                    } catch (const std::exception&) {
                        satellite_entry = false;
                    }
                }
            }
            continue;
        }
        if (!satellite_entry) continue;
        if (label == "VALID FROM") {
            current.valid_from = parseAntexEpochLine(line);
            continue;
        }
        if (label == "VALID UNTIL") {
            current.valid_until = parseAntexEpochLine(line);
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
            double north_mm = 0.0, east_mm = 0.0, up_mm = 0.0;
            if (iss >> north_mm >> east_mm >> up_mm) {
                current.body_offsets_m[current_signal] =
                    Vector3d(north_mm * 1e-3, east_mm * 1e-3, up_mm * 1e-3);
            }
        }
    }
    return !satellite_entries.empty();
}

bool loadReceiverAntexOffsets(
    const std::string& filename,
    std::map<std::string, std::map<SignalType, Vector3d>>& receiver_offsets) {
    receiver_offsets.clear();
    std::ifstream input(filename);
    if (!input.is_open()) {
        return false;
    }

    bool in_antenna = false;
    bool receiver_entry = false;
    SignalType current_signal = SignalType::SIGNAL_TYPE_COUNT;
    std::string current_type;
    std::map<SignalType, Vector3d> current_offsets;
    std::string line;
    while (std::getline(input, line)) {
        const std::string label = line.size() >= 60 ? trimCopy(line.substr(60)) : "";
        if (label == "START OF ANTENNA") {
            in_antenna = true;
            receiver_entry = false;
            current_signal = SignalType::SIGNAL_TYPE_COUNT;
            current_type.clear();
            current_offsets.clear();
            continue;
        }
        if (!in_antenna) {
            continue;
        }
        if (label == "END OF ANTENNA") {
            if (receiver_entry && !current_type.empty() && !current_offsets.empty()) {
                receiver_offsets[current_type] = current_offsets;
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
                current_offsets[current_signal] =
                    Vector3d(east_mm * 1e-3, north_mm * 1e-3, up_mm * 1e-3);
            }
        }
    }

    return !receiver_offsets.empty();
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
                                bool use_ionosphere_free,
                                double coeff_primary,
                                double coeff_secondary) {
    const std::string primary_code = biasObservationCode(primary_signal);
    if (primary_code.empty()) {
        return 0.0;
    }

    double primary_bias_m = 0.0;
    const bool have_primary = findOsbBiasMeters(dcb_products, satellite, primary_code, primary_bias_m);
    if (!use_ionosphere_free || secondary_signal == SignalType::SIGNAL_TYPE_COUNT) {
        return have_primary ? primary_bias_m : 0.0;
    }

    const std::string secondary_code = biasObservationCode(secondary_signal);
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
        case GNSSSystem::GPS: return {SignalType::GPS_L1CA};
        case GNSSSystem::GLONASS: return {SignalType::GLO_L1CA, SignalType::GLO_L1P};
        case GNSSSystem::Galileo: return {SignalType::GAL_E1};
        case GNSSSystem::BeiDou: return {SignalType::BDS_B1I, SignalType::BDS_B1C};
        case GNSSSystem::QZSS: return {SignalType::QZS_L1CA};
        default: return {};
    }
}

std::vector<SignalType> secondarySignals(GNSSSystem system) {
    switch (system) {
        case GNSSSystem::GPS: return {SignalType::GPS_L2C, SignalType::GPS_L5};
        case GNSSSystem::GLONASS: return {SignalType::GLO_L2CA, SignalType::GLO_L2P};
        case GNSSSystem::Galileo: return {SignalType::GAL_E5A, SignalType::GAL_E5B, SignalType::GAL_E6};
        case GNSSSystem::BeiDou: return {SignalType::BDS_B2I, SignalType::BDS_B2A, SignalType::BDS_B3I};
        case GNSSSystem::QZSS: return {SignalType::QZS_L2C, SignalType::QZS_L5};
        default: return {};
    }
}

const Observation* findObservationForSignals(const ObservationData& obs,
                                             const SatelliteId& sat,
                                             const std::vector<SignalType>& candidates) {
    for (const auto signal : candidates) {
        const Observation* candidate = obs.getObservation(sat, signal);
        if (candidate == nullptr || !candidate->valid || !candidate->has_pseudorange) {
            continue;
        }
        if (candidate->pseudorange <= 0.0 || !std::isfinite(candidate->pseudorange)) {
            continue;
        }
        return candidate;
    }
    return nullptr;
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
    receiver_antex_loaded_ = false;
    satellite_antex_offsets_.clear();
    satellite_antex_loaded_ = false;
    if (!ppp_config_.antex_file_path.empty()) {
        receiver_antex_loaded_ =
            loadReceiverAntexOffsets(ppp_config_.antex_file_path, receiver_antex_offsets_);
        // Receiver section may be empty for sat-only ANTEX files; the load
        // is fatal only if no satellite block also ends up populated.
        satellite_antex_loaded_ =
            loadSatelliteAntexOffsets(ppp_config_.antex_file_path, satellite_antex_offsets_);
        if (!receiver_antex_loaded_ && !satellite_antex_loaded_) {
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
    if (!ppp_config_.eop_path.empty() && !loadEopC04(ppp_config_.eop_path)) {
        return false;
    }
    atm_tidal_loading_loaded_ = false;
    atm_tidal_loading_coefficients_ = libgnss::iers::AtmosphericTidalLoadingCoefficients{};
    if (!ppp_config_.atm_tidal_loading_path.empty() &&
        !loadAtmosphericTidalLoading(ppp_config_.atm_tidal_loading_path)) {
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
                } else if (accepted_fixed_solution && ppp_config_.ar_method != PPPConfig::ARMethod::DD_WLNL) {
                    // For DD_IFLC/DD_PER_FREQ: revert to float state to avoid
                    // poisoning later epochs with a bad fix.
                    filter_state_ = float_filter_state;
                    ambiguity_states_ = float_ambiguity_states;
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
    clas_dispersion_compensation_.clear();
    clas_sis_continuity_.clear();
    clas_phase_bias_repair_.clear();
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

bool PPPProcessor::loadEopC04(const std::string& path) {
    eop_table_.reset();
    if (path.empty()) {
        return false;
    }
    try {
        // Auto-detect IERS C04 vs Bulletin A. Both formats are
        // accepted on the same `--eop-c04` CLI knob; the discriminator
        // is the `I`/`P` flag character at fixed columns in Bulletin
        // A (C04 has numeric data in those columns).
        eop_table_ = std::make_unique<libgnss::iers::EopTable>(
            libgnss::iers::EopTable::fromFile(path));
    } catch (const std::exception&) {
        eop_table_.reset();
        return false;
    }
    return true;
}

bool PPPProcessor::loadAtmosphericTidalLoading(const std::string& path) {
    atm_tidal_loading_coefficients_ =
        libgnss::iers::AtmosphericTidalLoadingCoefficients{};
    atm_tidal_loading_loaded_ = false;
    if (path.empty()) {
        return false;
    }
    std::ifstream in(path);
    if (!in) {
        return false;
    }
    bool have_s1 = false;
    bool have_s2 = false;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line.front() == '$' || line.front() == '#') {
            continue;
        }
        std::istringstream iss(line);
        std::string tag;
        iss >> tag;
        std::size_t idx = 0;
        if (tag == "S1") {
            idx = 0;
        } else if (tag == "S2") {
            idx = 1;
        } else {
            // Anything that is not a comment and not an S1/S2 tag is
            // assumed to be the station-name line; ignore.
            continue;
        }
        double r_amp = 0.0, w_amp = 0.0, s_amp = 0.0;
        double r_pha = 0.0, w_pha = 0.0, s_pha = 0.0;
        if (!(iss >> r_amp >> w_amp >> s_amp >> r_pha >> w_pha >> s_pha)) {
            return false;
        }
        atm_tidal_loading_coefficients_.radial_amplitudes_m[idx] = r_amp;
        atm_tidal_loading_coefficients_.west_amplitudes_m[idx]   = w_amp;
        atm_tidal_loading_coefficients_.south_amplitudes_m[idx]  = s_amp;
        atm_tidal_loading_coefficients_.radial_phases_deg[idx]   = r_pha;
        atm_tidal_loading_coefficients_.west_phases_deg[idx]     = w_pha;
        atm_tidal_loading_coefficients_.south_phases_deg[idx]    = s_pha;
        if (idx == 0) have_s1 = true; else have_s2 = true;
    }
    atm_tidal_loading_loaded_ = have_s1 && have_s2;
    return atm_tidal_loading_loaded_;
}

libgnss::iers::EarthOrientationParams
PPPProcessor::getEarthOrientationParams(const GNSSTime& time) const {
    if (!eop_table_) {
        return libgnss::iers::EarthOrientationParams{};
    }
    const double mjd_utc = libgnss::iers::gnssTimeToMjdUtc(time);
    auto eop = eop_table_->interpolateAt(mjd_utc);
    if (ppp_config_.use_iers_sub_daily_eop) {
        const auto delta = libgnss::iers::subDailyEopCorrection(
            mjd_utc, eop.ut1_minus_utc_seconds);
        eop.xp_arcsec               += delta.dxp_arcsec;
        eop.yp_arcsec               += delta.dyp_arcsec;
        eop.ut1_minus_utc_seconds   += delta.dut1_seconds;
    }
    return eop;
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
        atmos_tokens);
}

bool PPPProcessor::initializeFilter(const ObservationData& obs,
                                    const NavigationData& nav,
                                    const PositionSolution* seed_solution) {
    Vector3d initial_position = Vector3d::Zero();
    double initial_clock_bias_m = 0.0;
    if (seed_solution != nullptr && seed_solution->isValid()) {
        initial_position = seed_solution->position_ecef;
        initial_clock_bias_m = seed_solution->receiver_clock_bias;
    } else {
        auto spp_solution = spp_processor_.processEpoch(obs, nav);
        if (spp_solution.isValid()) {
            initial_position = spp_solution.position_ecef;
            initial_clock_bias_m = spp_solution.receiver_clock_bias;
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
    if (ppp_config_.estimate_ionosphere) {
        std::set<GNSSSystem> visible_systems;
        for (const auto& sat : obs.getSatellites()) visible_systems.insert(sat.system);
        if (false && visible_systems.count(GNSSSystem::Galileo)) {
            filter_state_.gal_clock_index = base_states++;
        }
        if (false && visible_systems.count(GNSSSystem::QZSS)) {
            filter_state_.qzs_clock_index = base_states++;
        }
        if (false && visible_systems.count(GNSSSystem::BeiDou)) {
            filter_state_.bds_clock_index = base_states++;
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
    }

    // Reserve ionosphere states for visible satellites
    int n_iono_states = 0;
    filter_state_.ionosphere_indices.clear();
    if (ppp_config_.estimate_ionosphere) {
        filter_state_.iono_index = isb_start;
        for (const auto& sat : obs.getSatellites()) {
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
    filter_state_.state(filter_state_.clock_index) = initial_clock_bias_m;
    filter_state_.state(filter_state_.glo_clock_index) = initial_clock_bias_m;
    // Initialize ISB states
    if (filter_state_.gal_clock_index >= 0)
        filter_state_.state(filter_state_.gal_clock_index) = initial_clock_bias_m;
    if (filter_state_.qzs_clock_index >= 0)
        filter_state_.state(filter_state_.qzs_clock_index) = initial_clock_bias_m;
    if (filter_state_.bds_clock_index >= 0)
        filter_state_.state(filter_state_.bds_clock_index) = initial_clock_bias_m;
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
    // Initialize per-satellite ionosphere states
    if (ppp_config_.estimate_ionosphere) {
        for (const auto& [sat, idx] : filter_state_.ionosphere_indices) {
            filter_state_.state(idx) = 0.0;  // Initial iono delay = 0 (will be constrained by STEC)
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
    Q(filter_state_.trop_index, filter_state_.trop_index) =
        ppp_config_.process_noise_troposphere * dt;
    // Ionosphere process noise
    if (ppp_config_.estimate_ionosphere) {
        for (const auto& [sat, idx] : filter_state_.ionosphere_indices) {
            Q(idx, idx) = ppp_config_.process_noise_ionosphere * dt;
        }
    }
    for (int idx = filter_state_.amb_index; idx < filter_state_.total_states; ++idx) {
        Q(idx, idx) = ppp_config_.process_noise_ambiguity * dt;
    }

    filter_state_.covariance = F * filter_state_.covariance * F.transpose() + Q;
    if (use_broadcast_rtklib_model && seed_solution != nullptr && seed_solution->isValid()) {
        if (ppp_config_.reset_clock_to_spp_each_epoch || useLowDynamicsBroadcastSeedAssist()) {
            reinitializeScalarState(
                filter_state_.clock_index,
                seed_solution->receiver_clock_bias,
                ppp_config_.initial_clock_variance);
            reinitializeScalarState(
                filter_state_.glo_clock_index,
                seed_solution->receiver_clock_bias,
                ppp_config_.initial_clock_variance);
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
    auto if_obs = formIonosphereFree(obs, nav);
    if (if_obs.size() < static_cast<size_t>(config_.min_satellites)) {
        if (pppDebugEnabled()) {
            std::cerr << "[PPP] insufficient IF observations: " << if_obs.size()
                      << " < " << config_.min_satellites << "\n";
        }
        return false;
    }

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

        if (meas_eq.observations.size() < config_.min_satellites) {
            if (pppDebugEnabled()) {
                std::cerr << "[PPP] insufficient measurement rows: " << meas_eq.observations.size()
                          << " < " << config_.min_satellites << "\n";
            }
            return false;
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

        if (pppDebugEnabled()) {
            std::cerr << "[PPP] iter=" << iteration
                      << " rows=" << meas_eq.observations.size()
                      << " pos_delta=" << delta_state.segment(filter_state_.pos_index, 3).norm()
                      << " clock_delta=" << delta_state(filter_state_.clock_index)
                      << " trop_delta=" << delta_state(filter_state_.trop_index)
                      << "\n";
        }

        filter_state_.state += delta_state;
        constrainStaticAnchorPosition();

        const MatrixXd identity =
            MatrixXd::Identity(filter_state_.total_states, filter_state_.total_states);
        const MatrixXd kh = gain * meas_eq.design_matrix;
        filter_state_.covariance =
            (identity - kh) * filter_state_.covariance * (identity - kh).transpose() +
            gain * meas_eq.weight_matrix * gain.transpose();

        if (delta_state.segment(filter_state_.pos_index, 3).norm() < 1e-4 &&
            std::abs(delta_state(filter_state_.clock_index)) < 1e-3 &&
            std::abs(delta_state(filter_state_.trop_index)) < 1e-3) {
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
    combined.reserve(obs.getSatellites().size());

    for (const auto& sat : obs.getSatellites()) {
        const Observation* primary = findObservationForSignals(obs, sat, primarySignals(sat.system));
        if (primary == nullptr) {
            continue;
        }

        const Ephemeris* eph = nav.getEphemeris(sat, obs.time);
        IonosphereFreeObs entry;
        entry.satellite = sat;

        if (!ppp_config_.use_ionosphere_free) {
            entry.pseudorange_if = primary->pseudorange;
            entry.primary_signal = primary->signal;
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
                // Store secondary observation info for MW combination in AR
                const double f1 = signalFrequencyHz(primary->signal, eph);
                const double f2 = signalFrequencyHz(secondary->signal, eph);
                if (f1 > 0.0 && f2 > 0.0) {
                    entry.primary_code_bias_coeff = 1.0;
                    entry.secondary_code_bias_coeff = 0.0;
                }
            }
            entry.valid = true;
            combined.push_back(entry);
            continue;
        }

        if (secondary == nullptr) {
            entry.pseudorange_if = primary->pseudorange;
            entry.primary_signal = primary->signal;
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
        entry.primary_code_bias_coeff = coefficients.first;
        entry.secondary_code_bias_coeff = coefficients.second;
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
                const auto it = antenna_it->second.find(signal);
                return it != antenna_it->second.end() ? it->second : Vector3d::Zero();
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

void PPPProcessor::applyPreciseCorrections(std::vector<IonosphereFreeObs>& observations,
                                           const NavigationData& nav,
                                           const GNSSTime& time) {
    const Vector3d receiver_marker_position = applyGeophysicalCorrections(
        filter_state_.state.segment(filter_state_.pos_index, 3), time);
    const double elevation_mask = config_.elevation_mask * kDegreesToRadians;

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

    // Sun position is needed for the satellite body frame in phase wind-up.
    // We compute it once per epoch and reuse for every satellite.
    const Vector3d sun_position_ecef = approximateSunPositionEcef(time);

    for (auto& observation : observations) {
        double deferred_variance_pr = 0.0;
        double deferred_variance_cp = 0.0;
        bool applied_ssr_code_bias = false;
        bool applied_ssr_iono = false;
        const Vector3d receiver_antenna_offset_ecef =
            calculateReceiverAntennaOffsetEcef(receiver_marker_position, observation);
        const Vector3d receiver_position = receiver_marker_position + receiver_antenna_offset_ecef;
        const Ephemeris* eph = nav.getEphemeris(observation.satellite, time);
        Vector3d sat_position = Vector3d::Zero();
        Vector3d sat_velocity = Vector3d::Zero();
        double sat_clock_bias = 0.0;
        double sat_clock_drift = 0.0;

        bool have_precise = false;
        if (precise_products_loaded_) {
            have_precise = precise_products_.interpolateOrbitClock(
                observation.satellite,
                time,
                sat_position,
                sat_velocity,
                sat_clock_bias,
                sat_clock_drift);
            if (have_precise) {
                // Light-travel-time iteration: the first SP3 sample is taken
                // at reception time, so re-sample at the implied emission
                // epoch (rcv − τ) to place sat_position on the orbit when the
                // photon left the satellite. The downstream geodist() handles
                // the Sagnac (frame-rotation) term.
                const double initial_distance =
                    (sat_position - receiver_position).norm();
                if (std::isfinite(initial_distance) && initial_distance > 0.0) {
                    const double travel_time =
                        initial_distance / constants::SPEED_OF_LIGHT;
                    const GNSSTime emission_time = time - travel_time;
                    Vector3d em_position = sat_position;
                    Vector3d em_velocity = sat_velocity;
                    double em_clock_bias = sat_clock_bias;
                    double em_clock_drift = sat_clock_drift;
                    if (precise_products_.interpolateOrbitClock(
                            observation.satellite,
                            emission_time,
                            em_position,
                            em_velocity,
                            em_clock_bias,
                            em_clock_drift)) {
                        sat_position = em_position;
                        sat_velocity = em_velocity;
                        sat_clock_bias = em_clock_bias;
                        sat_clock_drift = em_clock_drift;
                    }
                }
            }
        }

        if (!have_precise) {
            // First pass: compute satellite state at reception time
            if (!nav.calculateSatelliteState(
                    observation.satellite,
                    time,
                    sat_position,
                    sat_velocity,
                    sat_clock_bias,
                    sat_clock_drift)) {
                observation.valid = false;
                continue;
            }
            // Light travel time correction: re-evaluate at emission time
            if (observation.pseudorange_if > 0.0) {
                const double travel_time = observation.pseudorange_if / constants::SPEED_OF_LIGHT;
                const GNSSTime emission_time = time - travel_time + sat_clock_bias;
                if (!nav.calculateSatelliteState(
                        observation.satellite,
                        emission_time,
                        sat_position,
                        sat_velocity,
                        sat_clock_bias,
                        sat_clock_drift)) {
                    observation.valid = false;
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
                    preferred_network_id);
            // Fall back to epoch-wide atmosphere tokens when per-satellite
            // atmos are empty (CLAS broadcasts network-wide corrections).
            if (atmos_tokens.empty() && !epoch_atmos_tokens.empty()) {
                atmos_tokens = epoch_atmos_tokens;
            }
            if (ssr_ok) {
                if (!have_precise) {
                    if (ssr_products_.orbitCorrectionsAreRac()) {
                        orbit_correction_ecef =
                            ssrRacToEcef(sat_position, sat_velocity, orbit_correction_ecef);
                    }
                    sat_position += orbit_correction_ecef;
                    sat_clock_bias += clock_correction_m / constants::SPEED_OF_LIGHT;
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
                applied_ssr_code_bias = std::abs(code_bias) > 0.0;
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
                if (ppp_config_.estimate_ionosphere && std::isfinite(stec_tecu) &&
                    std::abs(stec_tecu) > 0.0) {
                    // In ionosphere estimation mode, inject STEC as a tight constraint
                    // on the ionosphere state instead of correcting observations.
                    const auto iono_it = filter_state_.ionosphere_indices.find(observation.satellite);
                    if (iono_it != filter_state_.ionosphere_indices.end()) {
                        const double iono_delay_m = ppp_atmosphere::ionosphereDelayMetersFromTecu(
                            observation.primary_signal, eph, stec_tecu);
                        if (std::isfinite(iono_delay_m)) {
                            const int idx = iono_it->second;
                            const double innovation = iono_delay_m - filter_state_.state(idx);
                            const double stec_sigma = 0.5;  // 50cm STEC constraint
                            const double S = filter_state_.covariance(idx, idx) + stec_sigma * stec_sigma;
                            if (S > 0.0) {
                                VectorXd K = filter_state_.covariance.col(idx) / S;
                                filter_state_.state += K * innovation;
                                filter_state_.covariance -= K * filter_state_.covariance.row(idx);
                            }
                        }
                    }
                    ++last_applied_atmos_iono_corrections_;
                    last_applied_atmos_iono_m_ += std::abs(ionosphere_correction_m);
                    applied_ssr_iono = true;
                } else if (std::isfinite(ionosphere_correction_m) &&
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

        // Satellite antenna PCO from ANTEX. The IGS final products since
        // the 2017 convention switch publish SP3 / CLK at the iono-free
        // combination antenna phase centre (so no shift is needed by
        // default). For SP3 sources known to report centre of mass,
        // setting apply_satellite_antenna_pco rotates the body-frame PCO
        // (constructed via the yaw-steering attitude) into ECEF and
        // shifts sat_position so geodist() returns the antenna range.
        if (satellite_antex_loaded_ && ppp_config_.apply_satellite_antenna_pco) {
            Vector3d pco_body_if = Vector3d::Zero();
            bool have_pco = false;
            for (const auto& entry : satellite_antex_offsets_) {
                if (!(entry.satellite == observation.satellite)) continue;
                if (entry.valid_from.week != 0 && time < entry.valid_from) continue;
                if (entry.valid_until.week != 0 && entry.valid_until < time) continue;
                const auto primary_it = entry.body_offsets_m.find(observation.primary_signal);
                if (primary_it == entry.body_offsets_m.end()) break;
                if (ppp_config_.use_ionosphere_free &&
                    observation.secondary_signal != SignalType::SIGNAL_TYPE_COUNT) {
                    const auto secondary_it =
                        entry.body_offsets_m.find(observation.secondary_signal);
                    if (secondary_it == entry.body_offsets_m.end()) break;
                    pco_body_if =
                        observation.primary_code_bias_coeff * primary_it->second +
                        observation.secondary_code_bias_coeff * secondary_it->second;
                } else {
                    pco_body_if = primary_it->second;
                }
                have_pco = true;
                break;
            }
            if (have_pco && sat_position.norm() > 1.0) {
                // Yaw-steering body frame: z toward Earth, y orthogonal to
                // the spacecraft–Sun plane, x toward the Sun side.
                const Vector3d e_z_sat = -sat_position.normalized();
                const Vector3d sun_los = sun_position_ecef - sat_position;
                if (sun_los.norm() > 1.0) {
                    Vector3d e_y_sat = e_z_sat.cross(sun_los).normalized();
                    if (e_y_sat.allFinite() && e_y_sat.norm() > 0.5) {
                        const Vector3d e_x_sat = e_y_sat.cross(e_z_sat);
                        const Vector3d pco_ecef =
                            pco_body_if.x() * e_x_sat +
                            pco_body_if.y() * e_y_sat +
                            pco_body_if.z() * e_z_sat;
                        sat_position += pco_ecef;
                    }
                }
            }
        }

        const auto geometry = nav.calculateGeometry(receiver_position, sat_position);
        if (!std::isfinite(geometry.distance) || geometry.elevation < elevation_mask) {
            observation.valid = false;
            continue;
        }

        if (!applied_ssr_code_bias && dcb_products_loaded_) {
            const double dcb_bias_m = observationDcbBiasMeters(
                dcb_products_,
                observation.satellite,
                observation.primary_signal,
                observation.secondary_signal,
                ppp_config_.use_ionosphere_free,
                observation.primary_code_bias_coeff,
                observation.secondary_code_bias_coeff);
            if (std::isfinite(dcb_bias_m) && std::abs(dcb_bias_m) > 0.0) {
                observation.pseudorange_if -= dcb_bias_m;
                observation.pseudorange_code_bias_m += dcb_bias_m;
                ++last_applied_dcb_corrections_;
                last_applied_dcb_m_ += std::abs(dcb_bias_m);
            }
        }

        // The dual-frequency ionosphere-free combination already cancels the
        // first-order ionospheric delay analytically, so subtracting an
        // IONEX-derived correction on top of an IF observable can only
        // introduce numerical-precision and TEC-mapping noise (a few cm
        // routinely observed at TSKB/GRAZ). Skip the subtraction in IF mode
        // unless the filter is estimating a per-satellite STEC state, in
        // which case the IONEX value is injected as a tight constraint
        // earlier in this loop rather than subtracted from observations.
        const bool ionex_applies_to_observation =
            !ppp_config_.use_ionosphere_free ||
            observation.secondary_signal == SignalType::SIGNAL_TYPE_COUNT ||
            ppp_config_.estimate_ionosphere;
        if (!applied_ssr_iono && ionex_products_loaded_ && ionex_applies_to_observation) {
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

        // Phase wind-up correction (Wu et al. 1993). Same cycle count for L1
        // and L2 — convert via the IF combination wavelength c/(f1+f2),
        // which is c1*λ1 + c2*λ2 with the standard IFLC coefficients.
        if (observation.has_carrier_phase) {
            const double previous_windup =
                windup_cache_.count(observation.satellite) ?
                    windup_cache_[observation.satellite] : 0.0;
            const double new_windup = ppp_utils::calculatePhaseWindup(
                receiver_position, sat_position, sun_position_ecef, previous_windup);
            windup_cache_[observation.satellite] = new_windup;

            double windup_wavelength_m = 0.0;
            if (ppp_config_.use_ionosphere_free &&
                observation.secondary_signal != SignalType::SIGNAL_TYPE_COUNT) {
                const double f1 = signalFrequencyHz(observation.primary_signal, eph);
                const double f2 = signalFrequencyHz(observation.secondary_signal, eph);
                if (f1 > 0.0 && f2 > 0.0 && f1 + f2 > 0.0) {
                    windup_wavelength_m = constants::SPEED_OF_LIGHT / (f1 + f2);
                }
            } else {
                windup_wavelength_m = signalWavelengthMeters(observation.primary_signal, eph);
            }
            if (windup_wavelength_m > 0.0 && std::isfinite(new_windup)) {
                const double windup_correction_m = new_windup * windup_wavelength_m;
                observation.carrier_phase_if -= windup_correction_m;
            }
        }

        // In CLAS-PPP mode (estimate_ionosphere), reject satellites without
        // both SSR orbit/clock and ionosphere corrections — matching CLASLIB's
        // corrmeas() which returns 0 when STEC correction fails.
        observation.valid = true;
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
                std::vector<SignalType>{SignalType::GPS_L2C, SignalType::GPS_L2P, SignalType::GPS_L5} :
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
    reinitializeScalarState(
        filter_state_.clock_index,
        recovered_clock_bias_m,
        ppp_config_.initial_clock_variance);
    reinitializeScalarState(
        filter_state_.glo_clock_index,
        recovered_clock_bias_m,
        ppp_config_.initial_clock_variance);
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
    const auto existing = filter_state_.ambiguity_indices.find(observation.satellite);
    if (existing != filter_state_.ambiguity_indices.end()) {
        auto& ambiguity = ambiguity_states_[observation.satellite];
        if (ambiguity.needs_reinitialization) {
            initializeAmbiguityState(observation, existing->second);
        }
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
    initializeAmbiguityState(observation, new_index);
    return new_index;
}

void PPPProcessor::initializeAmbiguityState(const IonosphereFreeObs& observation, int state_index) {
    const Vector3d receiver_position =
        observation.receiver_position.norm() > 1000.0 ?
            observation.receiver_position :
            filter_state_.state.segment(filter_state_.pos_index, 3);
    const double clock_bias_m = receiverClockBiasMeters(observation.satellite);
    const double zenith_delay = filter_state_.state(filter_state_.trop_index);
    const double predicted =
        geodist(observation.satellite_position, receiver_position) +
        clock_bias_m -
        constants::SPEED_OF_LIGHT * observation.satellite_clock_bias +
        (ppp_config_.estimate_troposphere ?
            observation.trop_mapping * zenith_delay :
            observation.modeled_trop_delay_m);
    filter_state_.state(state_index) = observation.carrier_phase_if - predicted;
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
                if (ssr_products_.interpolateCorrection(real_satellite, obs.time, orbit_corr, clock_corr,
                                                        nullptr, nullptr, nullptr, nullptr)) {
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
    const double mapping = calculateMappingFunction(receiver_pos, elevation, time);
    return mapping * zenith_delay;
}

double PPPProcessor::calculateMappingFunction(const Vector3d& receiver_pos,
                                              double elevation,
                                              const GNSSTime& time) const {
    double lat = 0.0;
    double lon = 0.0;
    double h = 0.0;
    ecef2geodetic(receiver_pos, lat, lon, h);
    const double hydrostatic =
        models::niellHydrostaticMapping(lat, h, elevation, dayOfYearFromTime(time));
    // Hydrostatic (dry) mapping only — the KF tracks a single zenith delay
    // and the hydrostatic component dominates (2.3m vs 0.25m wet).
    return hydrostatic;
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
    if (ppp_config_.use_iers_pole_tide) {
        corrected += calculatePoleTide(position, time);
    }
    if (ppp_config_.use_iers_atm_tidal_loading) {
        corrected += calculateAtmosphericTidalLoading(position, time);
    }
    return corrected;
}

Vector3d PPPProcessor::calculateSolidEarthTides(const Vector3d& position,
                                                const GNSSTime& time) const {
    if (!position.allFinite() || position.norm() < constants::WGS84_A * 0.5) {
        return Vector3d::Zero();
    }

    if (ppp_config_.use_iers_solid_tide) {
        // IERS Conventions 2010 §7.1.1 (Dehant) Step-1 + Step-2 model
        // via the libgnss::iers wrapper. Sun and Moon are supplied in
        // ICRS — see the FRAME NOTE in libgnss++/iers/tides.hpp for
        // why this is the correct frame for the IERS routine despite
        // its public documentation calling it "ECEF".
        const double mjd_utc = libgnss::iers::gnssTimeToMjdUtc(time);
        const Vector3d sun_icrs  = libgnss::iers::sunPositionIcrs(mjd_utc);
        const Vector3d moon_icrs = libgnss::iers::moonPositionIcrs(mjd_utc);
        return libgnss::iers::solidEarthTideDisplacement(
            mjd_utc, position, sun_icrs, moon_icrs);
    }

    // Default path: simplified Step-1-only Love-number body-tide
    // approximation kept for behavioral compatibility while the
    // IERS path stays opt-in pending truth-bench validation. See
    // docs/iers-integration-plan.md.
    constexpr double kSunGM  = 1.32712440018e20;
    constexpr double kMoonGM = 4.902801e12;
    const Vector3d sun_position  = approximateSunPositionEcef(time);
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

    double up_m = 0.0;
    double west_m = 0.0;
    double south_m = 0.0;

    if (ppp_config_.use_iers_ocean_loading) {
        // IERS Conventions 2010 §7.1.2 HARDISP path: spline-interpolated
        // admittance over 342 reference harmonics, with proper
        // astronomical (Doodson) phase reference. The wrapper takes a
        // BLQ in the libgnss::iers struct shape; we copy the existing
        // PPPProcessor cache (same 11-constituent BLQ block) into it.
        libgnss::iers::OceanLoadingBlq blq;
        for (std::size_t i = 0; i < blq.radial_amplitudes_m.size(); ++i) {
            blq.radial_amplitudes_m[i] =
                ocean_loading_coefficients_.up_amplitudes_m[i];
            blq.west_amplitudes_m[i] =
                ocean_loading_coefficients_.west_amplitudes_m[i];
            blq.south_amplitudes_m[i] =
                ocean_loading_coefficients_.south_amplitudes_m[i];
            blq.radial_phases_deg[i] =
                ocean_loading_coefficients_.up_phases_deg[i];
            blq.west_phases_deg[i] =
                ocean_loading_coefficients_.west_phases_deg[i];
            blq.south_phases_deg[i] =
                ocean_loading_coefficients_.south_phases_deg[i];
        }
        const double mjd_utc = libgnss::iers::gnssTimeToMjdUtc(time);
        const Vector3d disp_local =
            libgnss::iers::oceanLoadingDisplacement(mjd_utc, blq);
        up_m    = disp_local.x();
        west_m  = disp_local.y();
        south_m = disp_local.z();
    } else {
        // Legacy direct-sum path (kept as the default for now). Note
        // that this uses Unix epoch as the phase reference rather than
        // the proper IERS Doodson astronomical argument, so it is best
        // viewed as a placeholder; the IERS HARDISP path above is the
        // physically correct routine.
        const auto system_time = time.toSystemTime();
        const double seconds_since_unix =
            std::chrono::duration<double>(system_time.time_since_epoch()).count();
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
    }

    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;
    ecef2geodetic(position, latitude_rad, longitude_rad, height_m);
    const Vector3d enu_offset(-west_m, -south_m, up_m);
    return enu2ecef(enu_offset, latitude_rad, longitude_rad);
}

Vector3d PPPProcessor::calculatePoleTide(const Vector3d& position,
                                         const GNSSTime& time) const {
    if (!eop_table_ || !position.allFinite() ||
        position.norm() < constants::WGS84_A * 0.5) {
        return Vector3d::Zero();
    }
    const double mjd_utc = libgnss::iers::gnssTimeToMjdUtc(time);
    libgnss::iers::EarthOrientationParams eop;
    try {
        // getEarthOrientationParams applies the sub-daily correction
        // when ppp_config_.use_iers_sub_daily_eop is enabled — pole
        // tide automatically picks up the higher-frequency CIP wobble
        // when both flags are on.
        eop = getEarthOrientationParams(time);
    } catch (const std::out_of_range&) {
        return Vector3d::Zero();
    }
    return libgnss::iers::poleTideDisplacement(mjd_utc, position, eop);
}

Vector3d PPPProcessor::calculateAtmosphericTidalLoading(
    const Vector3d& position, const GNSSTime& time) const {
    if (!atm_tidal_loading_loaded_ || !position.allFinite() ||
        position.norm() < constants::WGS84_A * 0.5) {
        return Vector3d::Zero();
    }
    const double mjd_utc = libgnss::iers::gnssTimeToMjdUtc(time);
    return libgnss::iers::atmosphericTidalLoadingDisplacement(
        mjd_utc, position, atm_tidal_loading_coefficients_);
}

PPPProcessor::MeasurementEquation PPPProcessor::formMeasurementEquations(
    const std::vector<IonosphereFreeObs>& observations,
    const NavigationData& nav,
    const GNSSTime& time) {
    (void)nav;
    (void)time;

    std::vector<Eigen::RowVectorXd> rows;
    std::vector<double> measured_values;
    std::vector<double> predicted_values;
    std::vector<double> variances;
    std::vector<SatelliteId> row_satellites;
    std::vector<bool> row_is_phase;

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
                observation.trop_mapping * zenith_delay :
                observation.modeled_trop_delay_m;
        const int clock_state_index = receiverClockStateIndex(observation.satellite);
        const double clock_bias_m = receiverClockBiasMeters(observation.satellite);

        Eigen::RowVectorXd row = Eigen::RowVectorXd::Zero(filter_state_.total_states);
        row.segment(filter_state_.pos_index, 3) = -line_of_sight;
        row(clock_state_index) = 1.0;
        row(filter_state_.trop_index) =
            ppp_config_.estimate_troposphere ? observation.trop_mapping : 0.0;
        // Per-satellite ionosphere state: pseudorange has +iono contribution
        double iono_state_m = 0.0;
        if (ppp_config_.estimate_ionosphere) {
            const auto iono_it = filter_state_.ionosphere_indices.find(observation.satellite);
            if (iono_it != filter_state_.ionosphere_indices.end()) {
                row(iono_it->second) = 1.0;  // +iono for pseudorange
                iono_state_m = filter_state_.state(iono_it->second);
            }
        }

        const double predicted =
            geometric_range + clock_bias_m -
            constants::SPEED_OF_LIGHT * observation.satellite_clock_bias + troposphere_delay
            + iono_state_m;
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
        row_is_phase.push_back(false);

        if (use_phase_rows && observation.has_carrier_phase) {
            const int ambiguity_index = ambiguityStateIndex(observation.satellite);
            if (ambiguity_index >= 0 && ambiguity_index < filter_state_.total_states) {
                const auto ambiguity_it = ambiguity_states_.find(observation.satellite);
                const int required_lock_count =
                    precise_products_loaded_ ?
                        ppp_config_.convergence_min_epochs :
                        ppp_config_.phase_measurement_min_lock_count;
                const bool phase_ready =
                    ambiguity_it != ambiguity_states_.end() &&
                    !ambiguity_it->second.needs_reinitialization &&
                    ambiguity_it->second.lock_count >= required_lock_count;
                if (!phase_ready) {
                    continue;
                }
                // predicted for phase: geo + clk - satclk + trop - iono + amb
                // (predicted already includes +iono, so subtract 2*iono for phase)
                double iono_phase_correction = 0.0;
                if (ppp_config_.estimate_ionosphere) {
                    const auto iono_it = filter_state_.ionosphere_indices.find(observation.satellite);
                    if (iono_it != filter_state_.ionosphere_indices.end()) {
                        iono_phase_correction = -2.0 * filter_state_.state(iono_it->second);
                    }
                }
                const double predicted_phase = predicted + iono_phase_correction
                                               + filter_state_.state(ambiguity_index);
                const double phase_residual = observation.carrier_phase_if - predicted_phase;
                const double phase_residual_floor =
                    ppp_config_.kinematic_mode
                        ? (converged_ ? 20.0 : 200.0)
                        : (converged_ ? 10.0 : 50.0);
                const double phase_limit =
                    std::max(
                        ppp_config_.outlier_threshold *
                            std::sqrt(safeVariance(observation.variance_cp, 1e-8)) * 10.0,
                        phase_residual_floor);
                if (!ppp_config_.enable_outlier_detection ||
                    std::abs(phase_residual) <= phase_limit) {
                    Eigen::RowVectorXd phase_row = Eigen::RowVectorXd::Zero(filter_state_.total_states);
                    phase_row.segment(filter_state_.pos_index, 3) = -line_of_sight;
                    phase_row(clock_state_index) = 1.0;
                    phase_row(filter_state_.trop_index) =
                        ppp_config_.estimate_troposphere ? observation.trop_mapping : 0.0;
                    // Per-satellite ionosphere: carrier phase has -iono contribution
                    if (ppp_config_.estimate_ionosphere) {
                        const auto iono_it = filter_state_.ionosphere_indices.find(observation.satellite);
                        if (iono_it != filter_state_.ionosphere_indices.end()) {
                            phase_row(iono_it->second) = -1.0;
                        }
                    }
                    phase_row(ambiguity_index) = 1.0;

                    rows.push_back(phase_row);
                    measured_values.push_back(observation.carrier_phase_if);
                    predicted_values.push_back(predicted_phase);
                    variances.push_back(safeVariance(observation.variance_cp, 1e-8));
                    row_satellites.push_back(observation.satellite);
                    row_is_phase.push_back(true);
                }
            }
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
    equation.row_is_phase = row_is_phase;
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
    solution.position_covariance =
        filter_state_.covariance.block(filter_state_.pos_index, filter_state_.pos_index, 3, 3);

    double latitude = 0.0;
    double longitude = 0.0;
    double height = 0.0;
    ecef2geodetic(solution.position_ecef, latitude, longitude, height);
    solution.position_geodetic = GeodeticCoord(latitude, longitude, height);

    for (const auto& observation : observations) {
        if (!observation.valid) {
            continue;
        }
        solution.num_satellites++;
        solution.satellites_used.push_back(observation.satellite);
        solution.satellite_elevations.push_back(observation.elevation);
        const double geometric_range = geodist(observation.satellite_position, solution.position_ecef);
        const double clock_bias_m = receiverClockBiasMeters(observation.satellite);
        const double predicted = geometric_range +
            clock_bias_m -
            constants::SPEED_OF_LIGHT * observation.satellite_clock_bias +
            (ppp_config_.estimate_troposphere
                ? observation.trop_mapping * filter_state_.state(filter_state_.trop_index)
                : observation.modeled_trop_delay_m);
        solution.satellite_residuals.push_back(observation.pseudorange_if - predicted);
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

    const int ambiguity_index = ambiguityStateIndex(satellite);
    if (ambiguity_index >= 0 && ambiguity_index < filter_state_.total_states) {
        filter_state_.state(ambiguity_index) = 0.0;
        filter_state_.covariance.row(ambiguity_index).setZero();
        filter_state_.covariance.col(ambiguity_index).setZero();
        filter_state_.covariance(ambiguity_index, ambiguity_index) =
            precise_products_loaded_ ? 1e6 : ppp_config_.initial_ambiguity_variance;
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
                observation.trop_mapping * zenith_delay :
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
                            const Vector3d& sun_position_ecef,
                            double previous_windup) {
    // Phase wind-up (Wu et al. 1993) — accumulated cycles between the
    // satellite and receiver right-handed dipole frames.
    const Vector3d los_sr = receiver_pos - satellite_pos;
    const double los_norm = los_sr.norm();
    if (los_norm < 1.0) return previous_windup;
    const Vector3d e_sr = los_sr / los_norm;

    // Satellite body frame using a nominal yaw-steering attitude:
    //   z = -rs / |rs|              (toward Earth center)
    //   y = z × (sun - rs) / |...|  (perpendicular to spacecraft–Sun plane)
    //   x = y × z                   (toward Sun side)
    const Vector3d e_z_sat = -satellite_pos.normalized();
    Vector3d sun_dir;
    if (sun_position_ecef.norm() > 1.0) {
        sun_dir = sun_position_ecef - satellite_pos;
    } else {
        // Fallback: cross-track approximation (gives wrong magnitude but
        // keeps the correction bounded when no solar ephemeris is supplied).
        sun_dir = e_sr.cross(e_z_sat);
    }
    const double sun_norm = sun_dir.norm();
    if (sun_norm < 1e-3) return previous_windup;
    sun_dir /= sun_norm;
    Vector3d e_y_sat = e_z_sat.cross(sun_dir);
    const double y_norm = e_y_sat.norm();
    if (y_norm < 1e-10) return previous_windup;
    e_y_sat /= y_norm;
    const Vector3d e_x_sat = e_y_sat.cross(e_z_sat);

    // Receiver local north-east-up (north, east, up are standard PPP/RTKLIB):
    //   x_rcv → North, y_rcv → East, z_rcv → Up.
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;
    ecef2geodetic(receiver_pos, latitude_rad, longitude_rad, height_m);
    const double sin_lat = std::sin(latitude_rad);
    const double cos_lat = std::cos(latitude_rad);
    const double sin_lon = std::sin(longitude_rad);
    const double cos_lon = std::cos(longitude_rad);
    // North (X): -sin(lat) cos(lon), -sin(lat) sin(lon), cos(lat)
    const Vector3d e_x_rcv(-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat);
    // East  (Y): -sin(lon),                cos(lon),           0
    const Vector3d e_y_rcv(-sin_lon, cos_lon, 0.0);

    // Effective dipole vectors (Wu et al. 1993, eq. 7)
    const Vector3d d_sat = e_x_sat - e_sr * (e_sr.dot(e_x_sat))
                         - e_sr.cross(e_y_sat);
    const Vector3d d_rcv = e_x_rcv - e_sr * (e_sr.dot(e_x_rcv))
                         + e_sr.cross(e_y_rcv);

    const double d_sat_norm = d_sat.norm();
    const double d_rcv_norm = d_rcv.norm();
    if (d_sat_norm < 1e-10 || d_rcv_norm < 1e-10) return previous_windup;
    const double cos_phi = std::clamp(d_sat.dot(d_rcv) / (d_sat_norm * d_rcv_norm), -1.0, 1.0);
    double dphi = std::acos(cos_phi) / (2.0 * M_PI);
    if (e_sr.dot(d_sat.cross(d_rcv)) < 0.0) dphi = -dphi;

    // Resolve the 2π ambiguity against the prior cycle count.
    const double n = std::round(previous_windup - dphi);
    return dphi + n;
}

}  // namespace ppp_utils

}  // namespace libgnss
