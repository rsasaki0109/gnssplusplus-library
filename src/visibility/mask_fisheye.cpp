#include "libgnss++/visibility/mask_fisheye.hpp"

#include <opencv2/imgproc.hpp>

namespace libgnss {
namespace visibility {

cv::Mat maskFisheye(const cv::Mat& fisheye_image,
                    const cv::Mat& opensky_image,
                    int diff_threshold,
                    int morph_radius) {
    if (fisheye_image.empty() || opensky_image.empty()) return cv::Mat();
    if (fisheye_image.size() != opensky_image.size() ||
        fisheye_image.type() != opensky_image.type()) return cv::Mat();

    // Compute per-pixel sum of absolute differences across channels
    // MATLAB: imdiff = sum(abs(double(imfish)-double(imopensky)),3)
    cv::Mat fish_f, open_f, diff_f;
    fisheye_image.convertTo(fish_f, CV_32FC3);
    opensky_image.convertTo(open_f, CV_32FC3);

    cv::absdiff(fish_f, open_f, diff_f);

    // Sum across channels
    std::vector<cv::Mat> channels(3);
    cv::split(diff_f, channels);
    cv::Mat channel_sum = channels[0] + channels[1] + channels[2];

    // Threshold to binary mask
    cv::Mat mask;
    cv::threshold(channel_sum, mask, static_cast<double>(diff_threshold), 255.0, cv::THRESH_BINARY);
    mask.convertTo(mask, CV_8UC1);

    // Morphological opening to remove noise (matching MATLAB strel("disk",r))
    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(2 * morph_radius + 1, 2 * morph_radius + 1));

    // MATLAB: mask = ~imopen(~mask, se); mask = imopen(mask, se);
    cv::Mat inv_mask;
    cv::bitwise_not(mask, inv_mask);
    cv::morphologyEx(inv_mask, inv_mask, cv::MORPH_OPEN, kernel);
    cv::bitwise_not(inv_mask, mask);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

    return mask;
}

}  // namespace visibility
}  // namespace libgnss
