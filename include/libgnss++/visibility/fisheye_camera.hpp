#pragma once

#include <string>
#include <vector>
#include <cmath>

namespace libgnss {
namespace visibility {

/**
 * @brief Fisheye camera model loaded from XML calibration file.
 *
 * Ported from fishcam.xml / proj_fisheye.m in taroz/ge-gnss-visibility.
 * Maps (azimuth, elevation) angles to fisheye image pixel coordinates
 * using a polynomial fit of elevation-to-radial-distance.
 */
struct FisheyeCamera {
    int width = 600;
    int height = 600;
    double xc = 300.0;
    double yc = 300.0;
    std::vector<double> r_calib;   // radial distance samples (pixels)
    std::vector<double> el_calib;  // elevation angle samples (degrees)
    int poly_degree = 1;

    /**
     * @brief Load camera model from XML file (fishcam.xml format).
     * @return true on success
     */
    bool loadFromXml(const std::string& path);

    /**
     * @brief Compute polynomial coefficients for el -> r mapping.
     *
     * Fits polynomial of degree poly_degree from (el_calib, r_calib).
     * Returns coefficients [c_n, c_{n-1}, ..., c_0] (highest degree first).
     * Results are cached after the first call.
     */
    std::vector<double> fitPolynomial() const;

private:
    mutable std::vector<double> poly_cache_;
    mutable bool poly_dirty_ = true;
};

/**
 * @brief Evaluate polynomial p at value x.
 * Coefficients are [c_n, c_{n-1}, ..., c_0] (highest degree first).
 */
inline double polyval(const std::vector<double>& p, double x) {
    double result = 0.0;
    for (const auto& c : p) {
        result = result * x + c;
    }
    return result;
}

}  // namespace visibility
}  // namespace libgnss
