#pragma once

// Strict numbers for command-line options: the whole value must be a number
// ("1.5x", "abc", "nan" are refused), and the error names the option, e.g.
// "invalid number for --ratio: abc" (std::stod alone accepted a numeric
// prefix and failed with just "stod").

#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>

namespace libgnss_apps {

inline double optionDouble(const std::string& option, const std::string& value) {
    try {
        std::size_t consumed = 0;
        const double parsed = std::stod(value, &consumed);
        if (consumed == value.size() && std::isfinite(parsed)) {
            return parsed;
        }
    } catch (const std::exception&) {
    }
    throw std::invalid_argument("invalid number for " + option + ": " + value);
}

inline int optionInt(const std::string& option, const std::string& value) {
    try {
        std::size_t consumed = 0;
        const int parsed = std::stoi(value, &consumed);
        if (consumed == value.size()) {
            return parsed;
        }
    } catch (const std::exception&) {
    }
    throw std::invalid_argument("invalid integer for " + option + ": " + value);
}

/// A count: no sign, so "-5" is refused instead of wrapping around.
inline unsigned long long optionUnsigned(const std::string& option, const std::string& value) {
    if (!value.empty() && value.front() >= '0' && value.front() <= '9') {
        try {
            std::size_t consumed = 0;
            const unsigned long long parsed = std::stoull(value, &consumed);
            if (consumed == value.size()) {
                return parsed;
            }
        } catch (const std::exception&) {
        }
    }
    throw std::invalid_argument("invalid count for " + option + ": " + value);
}

}  // namespace libgnss_apps
