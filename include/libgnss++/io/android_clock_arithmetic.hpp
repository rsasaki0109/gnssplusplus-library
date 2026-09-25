#pragma once
#include <cstdint>
#include <limits>

namespace libgnss::io::android_clock {
constexpr std::int64_t nanos_per_second = 1'000'000'000LL;
constexpr std::int64_t nanos_per_day = 86'400LL * nanos_per_second;
constexpr std::int64_t nanos_per_week = 604'800LL * nanos_per_second;

// Subtract before converting to floating point. MSVC long double has only
// double precision, which cannot preserve ns at Android FullBias magnitudes.
inline bool difference(std::int64_t a, std::int64_t b, std::int64_t& value) {
    if ((b > 0 && a < std::numeric_limits<std::int64_t>::min() + b) ||
        (b < 0 && a > std::numeric_limits<std::int64_t>::max() + b))
        return false;
    value = a - b;
    return true;
}
inline std::int64_t positiveModulo(std::int64_t value, std::int64_t period) {
    const auto remainder = value % period;
    return remainder < 0 ? remainder + period : remainder;
}
}  // namespace libgnss::io::android_clock
