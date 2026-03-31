#pragma once

#include "types.hpp"
#include <vector>
#include <string>
#include <array>

namespace libgnss {
namespace kml {

/**
 * @brief Convert RGB color + alpha to KML hex color string (ABGR format).
 * @param col RGB color (0.0 to 1.0)
 * @param alpha Transparency (0.0 to 1.0)
 * @return 8-character hex string in AABBGGRR format
 */
std::string col2hex(const Color& col, double alpha = 1.0);

/**
 * @brief Wrap angle to [-360, +360] range for KML heading.
 */
double wrapTo720(double angle);

/**
 * @brief Unwrap angle array to remove 360-degree discontinuities.
 * @param angles Input angles in degrees
 * @return Unwrapped angles
 */
std::vector<double> unwrap360(const std::vector<double>& angles);

/**
 * @brief Cube vertex and face index result.
 */
struct CubeVertices {
    std::array<Location, 8> vertices;
    /// Face indices: 5 faces (top, front, right, back, left), 4 vertices each
    static constexpr int NUM_FACES = 5;
    static constexpr int VERTS_PER_FACE = 4;
    std::array<std::array<int, 4>, 5> faces;
};

/**
 * @brief Calculate cube vertices in geodetic coordinates.
 * @param center Center location (lat/lon/alt)
 * @param side_length Cube side length in meters
 * @return Vertices and face indices
 */
CubeVertices calcCubeVertices(const Location& center, double side_length);

/**
 * @brief Convert ENU offset to geodetic (LLH) coordinates.
 * @param east East offset (meters)
 * @param north North offset (meters)
 * @param up Up offset (meters)
 * @param ref_lat Reference latitude (degrees)
 * @param ref_lon Reference longitude (degrees)
 * @param ref_alt Reference altitude (meters)
 * @return Location in geodetic coordinates
 */
Location enu2llh(double east, double north, double up,
                 double ref_lat, double ref_lon, double ref_alt);

}  // namespace kml
}  // namespace libgnss
