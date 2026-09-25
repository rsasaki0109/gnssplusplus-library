#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace libgnss::smartphone_temporal {
struct Mi8ClockDrift {
    bool ok = false;
    std::vector<double> values;
    std::size_t magnitude_masks = 0;
    std::size_t jump_masks = 0;
    std::size_t filled_values = 0;
};

// taroz/gsdc2023 preprocessing.m: gradient(obs.clk), |D|>1000 mask,
// linear fill, both sides of |diff(D)|>50 mask, linear then nearest fill.
// Like MATLAB gradient(A), spacing is one sample, not elapsed GPS time.
// Values are initialization metadata, not new Doppler observations.
inline Mi8ClockDrift mi8ClockDrift(const std::vector<double>& clock_m) {
    Mi8ClockDrift result;
    const auto n = clock_m.size();
    if (n < 2 || std::any_of(clock_m.begin(), clock_m.end(),
                           [](double v) { return !std::isfinite(v); })) return result;
    const double missing = std::numeric_limits<double>::quiet_NaN();
    auto& d = result.values;
    d.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        const auto l = i ? i - 1 : i;
        const auto r = i + 1 < n ? i + 1 : i;
        d[i] = (clock_m[r] - clock_m[l]) / static_cast<double>(r - l);
        if (!std::isfinite(d[i]) || std::abs(d[i]) > 1000) {
            d[i] = missing;
            ++result.magnitude_masks;
        }
    }
    const auto fill = [&]() {
        std::vector<std::size_t> known;
        for (std::size_t i = 0; i < n; ++i) if (std::isfinite(d[i])) known.push_back(i);
        if (known.empty()) return false;
        for (std::size_t i = 0; i < n; ++i) {
            if (std::isfinite(d[i])) continue;
            if (known.size() == 1) {
                // The final nearest fill is defined even with one anchor.
                d[i] = d[known.front()];
            } else {
                auto upper = std::upper_bound(known.begin(), known.end(), i);
                const auto r = upper == known.begin() ? known[1]
                             : upper == known.end() ? known.back() : *upper;
                const auto l = upper == known.begin() ? known.front()
                             : upper == known.end() ? known[known.size() - 2] : *(upper - 1);
                const double a = (static_cast<double>(i) - l) / (r - l);
                d[i] = d[l] + a * (d[r] - d[l]);
            }
            ++result.filled_values;
        }
        return true;
    };
    if (!fill()) return result;
    std::vector<bool> jump(n, false);
    for (std::size_t i = 1; i < n; ++i)
        if (std::abs(d[i] - d[i - 1]) > 50) jump[i - 1] = jump[i] = true;
    for (std::size_t i = 0; i < n; ++i) if (jump[i]) {
        d[i] = missing;
        ++result.jump_masks;
    }
    if (!fill()) return result;
    result.ok = std::all_of(d.begin(), d.end(), [](double v) { return std::isfinite(v); });
    return result;
}
}  // namespace libgnss::smartphone_temporal
