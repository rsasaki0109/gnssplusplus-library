#include "libgnss++/visibility/proj_fisheye.hpp"

#include <cmath>

namespace libgnss {
namespace visibility {

namespace {
constexpr double kDeg2Rad = M_PI / 180.0;
}

FisheyeProjection projFisheye(const std::vector<double>& az,
                              const std::vector<double>& el,
                              const FisheyeCamera& camera,
                              double scale) {
    FisheyeProjection result;

    if (az.size() != el.size() || az.empty()) {
        return result;
    }

    // Compute polynomial coefficients: elevation -> radial distance
    auto poly = camera.fitPolynomial();
    if (poly.empty()) return result;

    int n = static_cast<int>(az.size());
    double sw = scale * camera.width;
    double sh = scale * camera.height;
    double sxc = scale * camera.xc;
    double syc = scale * camera.yc;

    result.px.reserve(n);
    result.py.reserve(n);
    result.valid_idx.reserve(n);

    for (int i = 0; i < n; ++i) {
        double r = scale * polyval(poly, el[i]);

        // Convert polar to Cartesian image coordinates
        double angle_rad = (-az[i] + 90.0) * kDeg2Rad;
        double mx = r * std::cos(angle_rad) + sxc;
        double my = sh - (r * std::sin(angle_rad) + syc);

        // Check bounds
        if (mx >= 0.0 && mx < sw && my >= 0.0 && my < sh) {
            result.px.push_back(mx);
            result.py.push_back(my);
            result.valid_idx.push_back(i);
        }
    }

    return result;
}

}  // namespace visibility
}  // namespace libgnss
