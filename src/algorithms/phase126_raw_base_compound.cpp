#include <libgnss++/algorithms/phase126_raw_base_compound.hpp>

#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>

#include <cmath>
#include <limits>
#include <numbers>

namespace libgnss::phase126_raw_base {
namespace {

bool earthValid(const Vector3d& position) {
    const double norm = position.norm();
    return position.allFinite() && std::isfinite(norm) && norm >= 6.0e6 &&
           norm <= 7.0e6;
}

}  // namespace

bool validateStationReference(const StationReference& reference,
                              std::string& failure) {
    failure.clear();
    if (!reference.has_approximate_position) {
        failure = "RINEX header lacks APPROX POSITION XYZ";
        return false;
    }
    if (!earthValid(reference.approximate_position_ecef)) {
        failure = "RINEX APPROX POSITION XYZ is not finite/Earth-valid";
        return false;
    }
    if (reference.has_antenna_delta && !reference.antenna_delta_enu.allFinite()) {
        failure = "RINEX antenna delta is non-finite";
        return false;
    }
    if (!reference.antenna_reference_convention_proven) {
        failure = "RINEX antenna reference convention is unproven";
        return false;
    }
    return true;
}

Vector3d enuDeltaToEcef(const Vector3d& reference_ecef,
                        const Vector3d& delta_enu) {
    if (!earthValid(reference_ecef) || !delta_enu.allFinite()) {
        return Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    }
    double latitude = 0.0;
    double longitude = 0.0;
    double height = 0.0;
    ecef2geodetic(reference_ecef, latitude, longitude, height);
    if (!std::isfinite(latitude) || !std::isfinite(longitude) ||
        !std::isfinite(height)) {
        return Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    }
    const double sin_lat = std::sin(latitude);
    const double cos_lat = std::cos(latitude);
    const double sin_lon = std::sin(longitude);
    const double cos_lon = std::cos(longitude);
    const Vector3d east(-sin_lon, cos_lon, 0.0);
    const Vector3d north(-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat);
    const Vector3d up(cos_lat * cos_lon, cos_lat * sin_lon, sin_lat);
    return east * delta_enu.x() + north * delta_enu.y() + up * delta_enu.z();
}

Vector3d antennaReferenceEcef(const StationReference& reference,
                             bool apply_delta) {
    std::string failure;
    if (!validateStationReference(reference, failure)) {
        return Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    }
    if (!apply_delta || !reference.has_antenna_delta) {
        return reference.approximate_position_ecef;
    }
    const Vector3d delta = enuDeltaToEcef(reference.approximate_position_ecef,
                                           reference.antenna_delta_enu);
    if (!delta.allFinite()) {
        return Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    }
    return reference.approximate_position_ecef + delta;
}

Geometry geodistWithSagnac(const Vector3d& receiver_ecef,
                            const Vector3d& satellite_ecef) {
    Geometry geometry;
    const Vector3d delta = satellite_ecef - receiver_ecef;
    const double raw_range = delta.norm();
    if (!receiver_ecef.allFinite() || !satellite_ecef.allFinite() ||
        !(raw_range > 0.0) || !std::isfinite(raw_range)) {
        geometry.range_m = std::numeric_limits<double>::quiet_NaN();
        geometry.elevation_rad = std::numeric_limits<double>::quiet_NaN();
        geometry.azimuth_rad = std::numeric_limits<double>::quiet_NaN();
        geometry.line_of_sight = Vector3d::Constant(
            std::numeric_limits<double>::quiet_NaN());
        return geometry;
    }
    geometry.line_of_sight = delta / raw_range;
    geometry.range_m = raw_range +
                       constants::OMEGA_E / constants::SPEED_OF_LIGHT *
                           (satellite_ecef.x() * receiver_ecef.y() -
                            satellite_ecef.y() * receiver_ecef.x());

    double latitude = 0.0;
    double longitude = 0.0;
    double height = 0.0;
    ecef2geodetic(receiver_ecef, latitude, longitude, height);
    if (!std::isfinite(latitude) || !std::isfinite(longitude) ||
        !std::isfinite(height)) {
        geometry.elevation_rad = std::numeric_limits<double>::quiet_NaN();
        geometry.azimuth_rad = std::numeric_limits<double>::quiet_NaN();
        return geometry;
    }
    const double sin_lat = std::sin(latitude);
    const double cos_lat = std::cos(latitude);
    const double sin_lon = std::sin(longitude);
    const double cos_lon = std::cos(longitude);
    const Vector3d east(-sin_lon, cos_lon, 0.0);
    const Vector3d north(-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat);
    const Vector3d up(cos_lat * cos_lon, cos_lat * sin_lon, sin_lat);
    const Vector3d local(geometry.line_of_sight.dot(east),
                         geometry.line_of_sight.dot(north),
                         geometry.line_of_sight.dot(up));
    geometry.elevation_rad = std::atan2(
        local.z(), std::hypot(local.x(), local.y()));
    geometry.azimuth_rad = std::atan2(local.x(), local.y());
    if (geometry.azimuth_rad < 0.0) {
        geometry.azimuth_rad += 2.0 * std::numbers::pi;
    }
    return geometry;
}

double officialBaseCodeResidual(double pseudorange_m,
                                double satellite_clock_m,
                                double geometric_range_m,
                                double ionosphere_m,
                                double troposphere_m) {
    return pseudorange_m + satellite_clock_m - geometric_range_m -
           ionosphere_m - troposphere_m;
}

}  // namespace libgnss::phase126_raw_base
