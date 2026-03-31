#pragma once

#include "types.hpp"
#include <vector>
#include <string>

namespace libgnss {
namespace kml {

/**
 * @brief Create KML Point placemarks.
 * @param point_ids Point identifiers
 * @param locations Locations (lat/lon/alt)
 * @param scale Icon scale
 * @param colors Colors per point (or single color for all)
 * @param alpha Transparency (0.0 to 1.0)
 * @param mode Altitude mode
 */
KmlString createPoints(const std::vector<std::string>& point_ids,
                       const std::vector<Location>& locations,
                       double scale,
                       const std::vector<Color>& colors,
                       double alpha = 1.0,
                       AltitudeMode mode = AltitudeMode::ClampToGround);

/**
 * @brief Create KML LineString placemark.
 */
KmlString createLine(const std::string& name,
                     const std::vector<Location>& locations,
                     double line_width,
                     const Color& color,
                     double alpha = 1.0,
                     int draw_order = 0,
                     AltitudeMode mode = AltitudeMode::ClampToGround);

/**
 * @brief Create KML Polygon cube placemarks.
 */
KmlString createCubes(const std::vector<std::string>& cube_ids,
                      const std::vector<Location>& locations,
                      double side_length,
                      const std::vector<Color>& colors,
                      double alpha = 1.0,
                      AltitudeMode mode = AltitudeMode::RelativeToGround);

/**
 * @brief Create KML 3D Model placemarks.
 * @param model_ids Model identifiers
 * @param files DAE model file paths
 * @param locations Model locations
 * @param orientations Model orientations (heading/tilt/roll)
 * @param scale Model scale
 * @param mode Altitude mode
 */
KmlString createModels(const std::vector<std::string>& model_ids,
                       const std::vector<std::string>& files,
                       const std::vector<Location>& locations,
                       const std::vector<Orientation>& orientations,
                       double scale = 1.0,
                       AltitudeMode mode = AltitudeMode::ClampToGround);

/// Single model convenience overload
KmlString createModel(const std::string& model_id,
                      const std::string& file,
                      const Location& location,
                      const Orientation& orientation,
                      double scale = 1.0,
                      AltitudeMode mode = AltitudeMode::ClampToGround);

/**
 * @brief Create KML ScreenOverlay element.
 */
KmlString createScreenOverlay(const std::string& screen_id,
                              const std::string& file);

}  // namespace kml
}  // namespace libgnss
