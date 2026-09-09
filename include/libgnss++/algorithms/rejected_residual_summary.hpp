#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>

namespace libgnss {
// Aggregate only; deliberately contains no observation or position series.
struct RejectedResidualSummary {
    std::size_t positive = 0, negative = 0, zero = 0, nonfinite = 0;
    double max_abs_m = 0;
    void observe(double centered_residual_m) {
        if (!std::isfinite(centered_residual_m)) { ++nonfinite; return; }
        if (centered_residual_m > 0) ++positive;
        else if (centered_residual_m < 0) ++negative;
        else ++zero;
        max_abs_m = std::max(max_abs_m, std::abs(centered_residual_m));
    }
};
}
