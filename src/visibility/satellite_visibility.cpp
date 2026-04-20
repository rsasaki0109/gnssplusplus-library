#include "libgnss++/visibility/satellite_visibility.hpp"

#include <cmath>

namespace libgnss {
namespace visibility {

VisibilityResult determineSatelliteVisibility(
    const std::vector<double>& az,
    const std::vector<double>& el,
    const cv::Mat& obstacle_mask,
    const FisheyeCamera& camera) {
    VisibilityResult result;

    if (obstacle_mask.empty()) return result;

    // Project satellite positions to fisheye image
    auto proj = projFisheye(az, el, camera);

    result.px = proj.px;
    result.py = proj.py;
    result.valid_idx = proj.valid_idx;

    int n = static_cast<int>(proj.px.size());
    result.is_nlos.resize(n, false);
    result.num_los = 0;
    result.num_nlos = 0;

    for (int i = 0; i < n; ++i) {
        int px = std::min(std::max(static_cast<int>(std::round(proj.px[i])), 0),
                          obstacle_mask.cols - 1);
        int py = std::min(std::max(static_cast<int>(std::round(proj.py[i])), 0),
                          obstacle_mask.rows - 1);

        bool is_blocked = obstacle_mask.at<uint8_t>(py, px) > 0;
        result.is_nlos[i] = is_blocked;

        if (is_blocked) {
            ++result.num_nlos;
        } else {
            ++result.num_los;
        }
    }

    return result;
}

}  // namespace visibility
}  // namespace libgnss
