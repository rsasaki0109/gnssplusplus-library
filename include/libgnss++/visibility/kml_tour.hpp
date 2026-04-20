#pragma once

#include <string>
#include <vector>
#include <cmath>

namespace libgnss {
namespace visibility {

/**
 * @brief Waypoint for a KML tour (one camera position).
 */
struct KmlWaypoint {
    double lat;      // Latitude (degrees)
    double lon;      // Longitude (degrees)
    double alt;      // Orthometric height (m), NOT ellipsoidal
    double heading;  // Camera heading (degrees), North = 0
};

/**
 * @brief Write a KML tour file for Google Earth virtual fisheye image capture.
 *
 * Ported from write_kml_tour.m in taroz/ge-gnss-visibility.
 * The camera faces straight up (tilt = 180) with the specified FOV.
 *
 * @param filepath   Output KML file path
 * @param waypoints  Camera positions along the trajectory
 * @param fov        Horizontal field of view in degrees (default 160)
 * @param dt         Time interval between waypoints in seconds (default 1.0)
 * @return true on success
 */
bool writeKmlTour(const std::string& filepath,
                  const std::vector<KmlWaypoint>& waypoints,
                  double fov = 160.0,
                  double dt = 1.0);

/**
 * @brief Wrap angle to [0, 360) range.
 */
inline double wrapTo360(double angle) {
    double r = std::fmod(angle, 360.0);
    return r < 0.0 ? r + 360.0 : r;
}

}  // namespace visibility
}  // namespace libgnss
