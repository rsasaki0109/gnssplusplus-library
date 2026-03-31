#pragma once

#include <opencv2/core.hpp>

namespace libgnss {
namespace visibility {

/**
 * @brief Extract obstacle mask by comparing a fisheye image against an open-sky reference.
 *
 * Ported from mask_fisheye.m in taroz/ge-gnss-visibility.
 * Computes per-pixel RGB difference, thresholds to binary mask,
 * then applies morphological opening to remove noise.
 *
 * @param fisheye_image   Current fisheye image (BGR, 8-bit)
 * @param opensky_image   Open-sky reference fisheye image (BGR, 8-bit)
 * @param diff_threshold  Threshold for sum-of-absolute-differences across channels (default 25)
 * @param morph_radius    Disk radius for morphological noise removal (default 3)
 * @return Binary mask (CV_8UC1): 255 = obstacle, 0 = open sky
 */
cv::Mat maskFisheye(const cv::Mat& fisheye_image,
                    const cv::Mat& opensky_image,
                    int diff_threshold = 25,
                    int morph_radius = 3);

}  // namespace visibility
}  // namespace libgnss
