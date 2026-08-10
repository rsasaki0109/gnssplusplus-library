#pragma once

#include <cmath>

namespace libgnss_apps {

inline double robustHuberLoss(double whitened_error, double threshold) {
    const double abs_error = std::abs(whitened_error);
    if (abs_error <= threshold) {
        return 0.5 * whitened_error * whitened_error;
    }
    return threshold * (abs_error - 0.5 * threshold);
}

inline double robustHuberWeight(double whitened_error, double threshold) {
    const double abs_error = std::abs(whitened_error);
    if (abs_error <= threshold || abs_error <= 0.0) {
        return 1.0;
    }
    return threshold / abs_error;
}

}  // namespace libgnss_apps
