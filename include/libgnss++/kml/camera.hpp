#pragma once

#include "types.hpp"

namespace libgnss {
namespace kml {

/**
 * @brief Create KML LookAt element.
 */
KmlString createLookAt(const Location& location,
                       const LookAtParams& cam,
                       AltitudeMode mode = AltitudeMode::ClampToGround);

/**
 * @brief Create KML Camera element.
 */
KmlString createCamera(const Location& location,
                       const CameraParams& cam,
                       AltitudeMode mode = AltitudeMode::ClampToGround);

}  // namespace kml
}  // namespace libgnss
