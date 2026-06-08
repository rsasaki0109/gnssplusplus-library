#include "libgnss++/visibility/im2fisheye.hpp"
#include "libgnss++/visibility/proj_fisheye.hpp"

#include <opencv2/imgproc.hpp>
#include <cmath>
#include <stdexcept>

namespace libgnss {
namespace visibility {

cv::Mat im2fisheye(const cv::Mat& pinhole_image,
                   double fov,
                   const FisheyeCamera& camera) {
    if (pinhole_image.rows != pinhole_image.cols) {
        throw std::invalid_argument("Input image must be square");
    }
    if (pinhole_image.channels() != 3) {
        throw std::invalid_argument("Input image must have 3 channels (BGR)");
    }

    // Upscale 2x with nearest neighbor (matching MATLAB im2fisheye)
    cv::Mat upscaled;
    cv::resize(pinhole_image, upscaled, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST);

    int n = upscaled.cols;  // == upscaled.rows (square)

    // Compute polynomial for projection
    auto poly = camera.fitPolynomial();
    if (poly.empty()) {
        throw std::runtime_error("Failed to fit fisheye camera polynomial");
    }

    // Focal length in pixels
    double half_fov_rad = (fov / 2.0) * M_PI / 180.0;
    double f = (n / 2.0) / std::tan(half_fov_rad);
    double half_n = n / 2.0;
    double sh = static_cast<double>(camera.height);

    // Inverse mapping: for each destination (fisheye) pixel, find the source
    // (pinhole) pixel. This eliminates holes in the output.
    //
    // Fisheye pixel (fx, fy) → (az, el) → pinhole pixel (sx, sy)
    //
    // Fisheye model: mx = r*cos(-az+90) + xc, my = h - (r*sin(-az+90) + yc)
    //   → r, az from (fx, fy):
    //     dx = fx - xc, dy = -(fy - (h - yc)) = (h - yc) - fy
    //     r = sqrt(dx^2 + dy^2), az = 90 - atan2(dy, dx) in deg
    //   → el from r via inverse poly:  el = inverse_polyval(r)
    //   For linear poly [a, b]: r = a*el + b → el = (r - b) / a
    //
    // Pinhole model: el = 90 - atan(r_pin/f), az = atan2(y,x) + 90
    //   → r_pin = f * tan((90 - el) * pi/180)
    //   → x = r_pin * cos(az - 90), y = r_pin * sin(az - 90)
    //   → sx = x + half_n, sy = y + half_n

    // Pre-compute inverse poly for linear case
    // poly = [a, b], r = a*el + b → el = (r - b) / a
    bool is_linear = (poly.size() == 2);
    double poly_a = is_linear ? poly[0] : 0.0;
    double poly_b = is_linear ? poly[1] : 0.0;

    // For non-linear: build a lookup table (el as function of r)
    // Not needed for standard fishcam.xml which is always linear

    cv::Mat fisheye = cv::Mat::zeros(camera.height, camera.width, CV_8UC3);

    double xc = camera.xc;
    double yc_offset = sh - camera.yc;  // h - yc

    for (int fy = 0; fy < camera.height; ++fy) {
        cv::Vec3b* dst_row = fisheye.ptr<cv::Vec3b>(fy);
        double dy = yc_offset - fy;

        for (int fx = 0; fx < camera.width; ++fx) {
            double dx = fx - xc;
            double r_fish = std::sqrt(dx * dx + dy * dy);

            if (r_fish < 0.5) {
                // At center: elevation = 90 (zenith), map to pinhole center
                int sx = static_cast<int>(half_n);
                int sy = static_cast<int>(half_n);
                if (sx >= 0 && sx < n && sy >= 0 && sy < n) {
                    dst_row[fx] = upscaled.at<cv::Vec3b>(sy, sx);
                }
                continue;
            }

            // Inverse fisheye: r → el
            double el;
            if (is_linear) {
                if (std::abs(poly_a) < 1e-15) continue;
                el = (r_fish - poly_b) / poly_a;
            } else {
                // Fallback: numerical inversion (Newton's method, 5 iterations)
                el = 45.0;  // initial guess
                for (int iter = 0; iter < 5; ++iter) {
                    double r_est = polyval(poly, el);
                    double dr = r_est - r_fish;
                    // Derivative: dr/del ≈ poly[0] for linear, approximate for higher
                    double deriv = poly[0];
                    for (size_t k = 1; k + 1 < poly.size(); ++k) {
                        deriv = deriv * el + poly[k];
                    }
                    if (std::abs(deriv) < 1e-15) break;
                    el -= dr / deriv;
                }
            }

            if (el < -10.0 || el > 90.0) continue;  // out of valid range

            // az from fisheye pixel
            double az = 90.0 - std::atan2(dy, dx) * 180.0 / M_PI;

            // Pinhole: (az, el) → pixel
            double r_pin = f * std::tan((90.0 - el) * M_PI / 180.0);
            double az_rad = (az - 90.0) * M_PI / 180.0;
            double px = r_pin * std::cos(az_rad) + half_n;
            double py = r_pin * std::sin(az_rad) + half_n;

            int sx = static_cast<int>(std::round(px));
            int sy = static_cast<int>(std::round(py));

            if (sx >= 0 && sx < n && sy >= 0 && sy < n) {
                dst_row[fx] = upscaled.at<cv::Vec3b>(sy, sx);
            }
        }
    }

    // Flip horizontally: east to right, west to left
    cv::Mat flipped;
    cv::flip(fisheye, flipped, 1);

    return flipped;
}

}  // namespace visibility
}  // namespace libgnss
