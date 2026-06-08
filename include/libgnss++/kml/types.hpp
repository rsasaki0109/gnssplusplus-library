#pragma once

#include <string>
#include <vector>
#include <cmath>
#include <cstdint>

namespace libgnss {
namespace kml {

/**
 * @brief Geographic location (WGS84).
 */
struct Location {
    double lat;  ///< Latitude (degrees)
    double lon;  ///< Longitude (degrees)
    double alt;  ///< Altitude (meters, orthometric)

    Location() : lat(0), lon(0), alt(0) {}
    Location(double lat, double lon, double alt) : lat(lat), lon(lon), alt(alt) {}

    bool isNaN() const { return std::isnan(lat) || std::isnan(lon) || std::isnan(alt); }
};

/**
 * @brief 3D orientation angles.
 */
struct Orientation {
    double heading;  ///< Heading (degrees)
    double tilt;     ///< Tilt (degrees)
    double roll;     ///< Roll (degrees)

    Orientation() : heading(0), tilt(0), roll(0) {}
    Orientation(double h, double t, double r) : heading(h), tilt(t), roll(r) {}
};

/**
 * @brief RGB color (0.0 to 1.0 per channel).
 */
struct Color {
    double r, g, b;

    Color() : r(0), g(0), b(0) {}
    Color(double r, double g, double b) : r(r), g(g), b(b) {}
};

/**
 * @brief KML altitude mode.
 */
enum class AltitudeMode : uint8_t {
    ClampToGround,
    RelativeToGround,
    Absolute
};

/**
 * @brief Parameters for LookAt camera view.
 */
struct LookAtParams {
    double heading;  ///< Heading (degrees)
    double tilt;     ///< Tilt (degrees)
    double range;    ///< Range from target (meters)

    LookAtParams() : heading(0), tilt(50), range(90) {}
    LookAtParams(double h, double t, double r) : heading(h), tilt(t), range(r) {}
};

/**
 * @brief Parameters for free Camera view.
 */
struct CameraParams {
    double heading;
    double tilt;
    double roll;

    CameraParams() : heading(0), tilt(0), roll(0) {}
    CameraParams(double h, double t, double r) : heading(h), tilt(t), roll(r) {}
};

/**
 * @brief KML fly-to mode.
 */
enum class FlyToMode : uint8_t {
    Smooth,
    Bounce
};

/// Convenience alias for KML string fragments
using KmlString = std::string;

/// Convert AltitudeMode enum to KML string
std::string altitudeModeStr(AltitudeMode mode);

/// Convert FlyToMode enum to KML string
std::string flyToModeStr(FlyToMode mode);

}  // namespace kml
}  // namespace libgnss
