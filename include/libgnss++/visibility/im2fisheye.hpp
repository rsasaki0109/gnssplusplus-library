#pragma once

#include <opencv2/core.hpp>
#include "libgnss++/visibility/fisheye_camera.hpp"

namespace libgnss {
namespace visibility {

/**
 * @brief Convert a pinhole camera image to a fisheye camera image.
 *
 * Ported from im2fisheye.m in taroz/ge-gnss-visibility.
 * The input image must be square (MxM). It is upscaled 2x with nearest
 * neighbour interpolation, then each pixel is reprojected through the
 * fisheye camera model.
 *
 * @param pinhole_image  Square input image (e.g. 1200x1200 from Google Earth)
 * @param fov            Field of view of the pinhole image in degrees
 * @param camera         Fisheye camera model
 * @return Fisheye image of size (camera.height x camera.width)
 */
cv::Mat im2fisheye(const cv::Mat& pinhole_image,
                   double fov,
                   const FisheyeCamera& camera);

}  // namespace visibility
}  // namespace libgnss
