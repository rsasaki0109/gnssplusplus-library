#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include "libgnss++/visibility/fisheye_camera.hpp"
#include "libgnss++/visibility/proj_fisheye.hpp"

namespace libgnss {
namespace visibility {

/**
 * @brief LOS/NLOS determination result for a set of satellites.
 */
struct VisibilityResult {
    std::vector<double> px;       // Projected X pixel coordinates
    std::vector<double> py;       // Projected Y pixel coordinates
    std::vector<int> valid_idx;   // Indices of satellites inside the image
    std::vector<bool> is_nlos;    // true if NLOS (obstacle blocks LOS)
    int num_los = 0;
    int num_nlos = 0;
};

/**
 * @brief Determine LOS/NLOS visibility of satellites on a fisheye image.
 *
 * Projects satellite (az, el) onto the fisheye image and checks whether
 * each satellite's pixel falls on the obstacle mask.
 *
 * @param az              Azimuth angles in degrees (per satellite)
 * @param el              Elevation angles in degrees (per satellite)
 * @param obstacle_mask   Binary mask (CV_8UC1): 255 = obstacle
 * @param camera          Fisheye camera model
 * @return VisibilityResult with per-satellite LOS/NLOS status
 */
VisibilityResult determineSatelliteVisibility(
    const std::vector<double>& az,
    const std::vector<double>& el,
    const cv::Mat& obstacle_mask,
    const FisheyeCamera& camera);

}  // namespace visibility
}  // namespace libgnss
