#pragma once

#include <vector>
#include "libgnss++/visibility/fisheye_camera.hpp"

namespace libgnss {
namespace visibility {

/**
 * @brief Result of projecting (az, el) points onto a fisheye image.
 */
struct FisheyeProjection {
    std::vector<double> px;       // X pixel coordinates (0 = left)
    std::vector<double> py;       // Y pixel coordinates (0 = top)
    std::vector<int> valid_idx;   // Indices of input points that fall inside the image
};

/**
 * @brief Project azimuth/elevation angles to fisheye image pixel coordinates.
 *
 * Ported from proj_fisheye.m in taroz/ge-gnss-visibility.
 *
 * @param az      Azimuth angles in degrees (North = 0, clockwise)
 * @param el      Elevation angles in degrees (0 = horizon, 90 = zenith)
 * @param camera  Fisheye camera model
 * @param scale   Output image scale factor (default 1.0)
 * @return Projected pixel coordinates and valid indices
 */
FisheyeProjection projFisheye(const std::vector<double>& az,
                              const std::vector<double>& el,
                              const FisheyeCamera& camera,
                              double scale = 1.0);

}  // namespace visibility
}  // namespace libgnss
