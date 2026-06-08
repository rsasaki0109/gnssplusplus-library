#pragma once

#include "types.hpp"
#include <vector>
#include <string>
#include <optional>

namespace libgnss {
namespace kml {

/**
 * @brief Create AnimatedUpdate for Point elements.
 * @param point_ids Point identifiers to update
 * @param duration Animation duration (seconds)
 * @param locations New locations
 * @param delay Delayed start (seconds)
 * @param colors Optional new colors
 * @param alpha Optional new transparency
 */
KmlString animatedUpdatePoints(const std::vector<std::string>& point_ids,
                               double duration,
                               const std::vector<Location>& locations,
                               double delay = 0.0,
                               const std::vector<Color>& colors = {},
                               double alpha = 1.0);

/**
 * @brief Create AnimatedUpdate for Model elements.
 * @param model_ids Model identifiers to update
 * @param duration Animation duration (seconds)
 * @param locations New locations
 * @param orientations Optional new orientations (empty = no change)
 * @param files Optional new model files (empty = no change)
 * @param delay Delayed start (seconds)
 */
KmlString animatedUpdateModels(const std::vector<std::string>& model_ids,
                               double duration,
                               const std::vector<Location>& locations,
                               const std::vector<Orientation>& orientations = {},
                               const std::vector<std::string>& files = {},
                               double delay = 0.0);

/// Single model convenience overload
KmlString animatedUpdateModel(const std::string& model_id,
                              double duration,
                              const Location& location,
                              const std::optional<Orientation>& orientation = std::nullopt,
                              const std::optional<std::string>& file = std::nullopt,
                              double delay = 0.0);

/**
 * @brief Create AnimatedUpdate for Cube elements.
 */
KmlString animatedUpdateCubes(const std::vector<std::string>& cube_ids,
                              double duration,
                              const std::vector<Location>& locations,
                              double side_length,
                              const std::vector<Color>& colors = {},
                              double alpha = 1.0,
                              double delay = 0.0);

}  // namespace kml
}  // namespace libgnss
