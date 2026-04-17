#include <libgnss++/algorithms/tidal.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>

#include <algorithm>
#include <cmath>

namespace libgnss {
namespace tidal {

namespace {

constexpr double kDegreesToRadians = M_PI / 180.0;

double wrapRadians(double angle_rad) {
    const double wrapped = std::fmod(angle_rad, 2.0 * M_PI);
    return wrapped < 0.0 ? wrapped + 2.0 * M_PI : wrapped;
}

}  // namespace

double julianDateFromTime(const GNSSTime& time) {
    // GPS epoch (1980-01-06 00:00:00) is JD 2444244.5.
    return 2444244.5 + static_cast<double>(time.week) * 7.0 + time.tow / 86400.0;
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

Vector3d calculateSolidEarthTides(const Vector3d& receiver_position,
                                  const GNSSTime& time) {
    constexpr double kSunGM = 1.32712440018e20;
    constexpr double kMoonGM = 4.902801e12;
    if (!receiver_position.allFinite() ||
        receiver_position.norm() < constants::WGS84_A * 0.5) {
        return Vector3d::Zero();
    }
    const Vector3d sun_position = approximateSunPositionEcef(time);
    const Vector3d moon_position = approximateMoonPositionEcef(time);
    return bodyTideDisplacement(receiver_position, sun_position, kSunGM) +
           bodyTideDisplacement(receiver_position, moon_position, kMoonGM);
}

}  // namespace tidal
}  // namespace libgnss
