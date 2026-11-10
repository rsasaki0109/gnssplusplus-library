#pragma once

// Strict numbers for command-line options: the whole value must be a number
// ("1.5x", "abc", "nan" are refused), and the error names the option, e.g.
// "invalid number for --ratio: abc" (std::stod alone accepted a numeric
// prefix and failed with just "stod").

#include <cctype>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>

namespace libgnss_apps {

/// The value without surrounding blanks ("0.1 " from a quoted list).
inline std::string trimmedOptionValue(const std::string& value) {
    std::size_t first = 0;
    std::size_t last = value.size();
    while (first < last && std::isspace(static_cast<unsigned char>(value[first]))) {
        ++first;
    }
    while (last > first && std::isspace(static_cast<unsigned char>(value[last - 1]))) {
        --last;
    }
    return value.substr(first, last - first);
}

inline double optionDouble(const std::string& option, const std::string& value) {
    const std::string text = trimmedOptionValue(value);
    try {
        std::size_t consumed = 0;
        const double parsed = std::stod(text, &consumed);
        if (!text.empty() && consumed == text.size() && std::isfinite(parsed)) {
            return parsed;
        }
    } catch (const std::exception&) {
    }
    throw std::invalid_argument("invalid number for " + option + ": " + value);
}

inline int optionInt(const std::string& option, const std::string& value) {
    const std::string text = trimmedOptionValue(value);
    try {
        std::size_t consumed = 0;
        const int parsed = std::stoi(text, &consumed);
        if (!text.empty() && consumed == text.size()) {
            return parsed;
        }
    } catch (const std::exception&) {
    }
    throw std::invalid_argument("invalid integer for " + option + ": " + value);
}

/// A count: no sign, so "-5" is refused instead of wrapping around.
inline unsigned long long optionUnsigned(const std::string& option, const std::string& value) {
    std::string text = trimmedOptionValue(value);
    if (!text.empty() && text.front() == '+') {
        text.erase(0, 1);
    }
    if (!text.empty() && text.front() >= '0' && text.front() <= '9') {
        try {
            std::size_t consumed = 0;
            const unsigned long long parsed = std::stoull(text, &consumed);
            if (consumed == text.size()) {
                return parsed;
            }
        } catch (const std::exception&) {
        }
    }
    throw std::invalid_argument("invalid count for " + option + ": " + value);
}

}  // namespace libgnss_apps
