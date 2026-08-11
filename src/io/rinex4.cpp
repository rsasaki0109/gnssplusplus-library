#include <libgnss++/io/rinex4.hpp>

#include <cctype>
#include <algorithm>
#include <cmath>
#include <sstream>

namespace libgnss::io::rinex4 {

namespace {

bool isKnownConstellation(char system) {
    return system == 'G' || system == 'R' || system == 'E' ||
           system == 'C' || system == 'J' || system == 'S' ||
           system == 'I';
}

}  // namespace

bool parseNavigationRecordHeader(const std::string& line,
                                 NavigationRecordHeader& record) {
    std::istringstream fields(line);
    std::string marker;
    if (!(fields >> marker >> record.record_type >> record.source >> record.message_type) ||
        marker != ">") {
        return false;
    }
    record.subtype.clear();
    std::string trailing_token;
    if (fields >> record.subtype) {
        // RINEX 4.02 Table 22 does not define EPH subtypes yet. A nonblank
        // token is therefore not valid EPH metadata, and any further token
        // is malformed for every record type.
        if (record.record_type == "EPH" || (fields >> trailing_token)) {
            return false;
        }
    }

    if (record.record_type.size() != 3 || record.source.empty() ||
        record.message_type.empty()) {
        return false;
    }
    if (std::any_of(record.message_type.begin(), record.message_type.end(),
                    [](unsigned char c) {
                        return std::isalpha(c) && !std::isupper(c);
                    })) {
        return false;
    }

    // EPH records identify a transmitting satellite (for example G01),
    // whereas STO/ION/EOP records may identify only a constellation (for
    // example R).  Validate the satellite form only when it is required.
    if (record.record_type == "EPH") {
        if (record.source.size() != 3 ||
            !isKnownConstellation(record.source[0]) ||
            !std::isdigit(static_cast<unsigned char>(record.source[1])) ||
            !std::isdigit(static_cast<unsigned char>(record.source[2]))) {
            return false;
        }
        record.system = record.source[0];
        record.prn = (record.source[1] - '0') * 10 + (record.source[2] - '0');
        if (record.prn <= 0) {
            return false;
        }
    }
    record.navigation_message_type =
        navigationMessageTypeFromRinexToken(record.message_type);
    return true;
}

bool supportsEphemerisMessage(char system, const std::string& message_type) {
    return supportsEphemerisMessage(
        system, navigationMessageTypeFromRinexToken(message_type));
}

NavigationMessageType navigationMessageTypeFromRinexToken(const std::string& token) {
    if (token == "LNAV") return NavigationMessageType::LNAV;
    if (token == "FDMA") return NavigationMessageType::FDMA;
    if (token == "FNAV") return NavigationMessageType::FNAV;
    if (token == "INAV") return NavigationMessageType::INAV;
    if (token == "D1") return NavigationMessageType::D1;
    if (token == "D2") return NavigationMessageType::D2;
    if (token == "SBAS") return NavigationMessageType::SBAS;
    if (token == "CNAV") return NavigationMessageType::CNAV;
    if (token == "CNV1") return NavigationMessageType::CNV1;
    if (token == "CNV2") return NavigationMessageType::CNV2;
    if (token == "CNV3") return NavigationMessageType::CNV3;
    if (token == "L1NV") return NavigationMessageType::L1NV;
    if (token == "L1OC") return NavigationMessageType::L1OC;
    if (token == "L3OC") return NavigationMessageType::L3OC;
    return NavigationMessageType::Unknown;
}

const char* navigationMessageTypeName(NavigationMessageType type) {
    switch (type) {
        case NavigationMessageType::LNAV: return "LNAV";
        case NavigationMessageType::FDMA: return "FDMA";
        case NavigationMessageType::FNAV: return "FNAV";
        case NavigationMessageType::INAV: return "INAV";
        case NavigationMessageType::D1: return "D1";
        case NavigationMessageType::D2: return "D2";
        case NavigationMessageType::SBAS: return "SBAS";
        case NavigationMessageType::CNAV: return "CNAV";
        case NavigationMessageType::CNV1: return "CNV1";
        case NavigationMessageType::CNV2: return "CNV2";
        case NavigationMessageType::CNV3: return "CNV3";
        case NavigationMessageType::L1NV: return "L1NV";
        case NavigationMessageType::L1OC: return "L1OC";
        case NavigationMessageType::L3OC: return "L3OC";
        case NavigationMessageType::Unknown: return "UNKNOWN";
    }
    return "UNKNOWN";
}

bool supportsEphemerisMessage(char system, NavigationMessageType type) {
    // These are the RINEX 4 records whose body layout is compatible with the
    // existing RINEX 2/3 parser.  New signal-specific records (CNAV/CNVx,
    // NavIC L1NV, and GLONASS CDMA L1OC/L3OC) must not be passed to it: doing
    // so would turn their fields into a plausible but incorrect ephemeris.
    switch (system) {
        case 'G':
        case 'J':
            return type == NavigationMessageType::LNAV;
        case 'R':
            return type == NavigationMessageType::FDMA;
        case 'E':
            return type == NavigationMessageType::FNAV ||
                   type == NavigationMessageType::INAV;
        case 'C':
            return type == NavigationMessageType::D1 ||
                   type == NavigationMessageType::D2;
        default:
            return false;
    }
}

namespace {

std::string trim(const std::string& value) {
    const size_t first = value.find_first_not_of(' ');
    if (first == std::string::npos) {
        return {};
    }
    const size_t last = value.find_last_not_of(' ');
    return value.substr(first, last - first + 1);
}

bool isUnsignedDigits(const std::string& value) {
    return !value.empty() &&
           std::all_of(value.begin(), value.end(), [](unsigned char c) {
               return std::isdigit(c) != 0;
           });
}

bool isUnsignedDigitsOfWidth(const std::string& value, size_t width) {
    return value.size() == width && isUnsignedDigits(value);
}

bool isCountToken(const std::string& value) {
    return value.size() >= 1 && value.size() <= 3 && isUnsignedDigits(value);
}

bool parseFixedDecimal(const std::string& text, int fractional_digits, double& value) {
    const std::string token = trim(text);
    if (token.empty()) {
        return false;
    }
    const size_t dot = token.find('.');
    if (dot == std::string::npos || token.find('.', dot + 1) != std::string::npos) {
        return false;
    }
    const size_t integer_begin = (token[0] == '+' || token[0] == '-') ? 1 : 0;
    if (integer_begin == token.size() || dot <= integer_begin ||
        dot - integer_begin == 0 || token.size() - dot - 1 !=
            static_cast<size_t>(fractional_digits)) {
        return false;
    }
    if (!isUnsignedDigits(token.substr(integer_begin, dot - integer_begin)) ||
        !isUnsignedDigits(token.substr(dot + 1))) {
        return false;
    }
    try {
        value = std::stod(token);
    } catch (...) {
        return false;
    }
    return std::isfinite(value);
}

bool isLeapYear(int year) {
    return (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
}

bool validCalendar(const ObservationEpochHeader& epoch) {
    if (epoch.year < 1980 || epoch.year > 9999 || epoch.month < 1 || epoch.month > 12 ||
        epoch.day < 1 || epoch.hour < 0 || epoch.hour > 23 || epoch.minute < 0 ||
        epoch.minute > 59 || epoch.second < 0.0 || epoch.second > 60.0) {
        return false;
    }
    static constexpr int kDaysInMonth[] = {
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    int days = kDaysInMonth[epoch.month - 1];
    if (epoch.month == 2 && isLeapYear(epoch.year)) {
        days = 29;
    }
    return epoch.day <= days;
}

bool parseSecondsToken(const std::string& token, double& seconds) {
    const size_t dot = token.find('.');
    if (dot == std::string::npos || token.find_first_of("eEdD") != std::string::npos) {
        return false;
    }
    const size_t fractional_digits = token.size() - dot - 1;
    if (fractional_digits != 7) {
        return false;
    }
    if (!parseFixedDecimal(token, static_cast<int>(fractional_digits), seconds)) {
        return false;
    }
    return seconds >= 0.0 && seconds <= 60.0;
}

}  // namespace

bool parseObservationEpochHeader(const std::string& line,
                                 ObservationEpochHeader& epoch) {
    epoch = ObservationEpochHeader();
    if (line.empty() || line[0] != '>') {
        return false;
    }

    std::istringstream tokens(line.substr(1));
    std::string year_token;
    if (!(tokens >> year_token)) {
        return false;
    }

    // Event records may omit their date/time fields.  Their flag and count
    // remain in the normal fixed columns, and the data records are consumed
    // by RINEXReader before it searches for the next ordinary epoch.
    if (year_token.size() != 4 || !isUnsignedDigits(year_token)) {
        int flag = 0;
        int count = 0;
        std::string count_token;
        if (!isUnsignedDigitsOfWidth(year_token, 1) ||
            !(tokens >> count_token) || !isCountToken(count_token)) {
            return false;
        }
        try {
            flag = std::stoi(year_token);
            count = std::stoi(count_token);
        } catch (...) {
            return false;
        }
        if (flag < 2 || flag > 6 || count < 0 || count > 999) {
            return false;
        }
        epoch.flag = flag;
        epoch.record_count = count;
        return true;
    }

    std::string month_token;
    std::string day_token;
    std::string hour_token;
    std::string minute_token;
    std::string second_token;
    std::string flag_token;
    std::string count_token;
    if (!(tokens >> month_token >> day_token >> hour_token >> minute_token >>
          second_token >> flag_token >> count_token)) {
        return false;
    }
    if (!isUnsignedDigitsOfWidth(month_token, 2) ||
        !isUnsignedDigitsOfWidth(day_token, 2) ||
        !isUnsignedDigitsOfWidth(hour_token, 2) ||
        !isUnsignedDigitsOfWidth(minute_token, 2) ||
        !isUnsignedDigitsOfWidth(flag_token, 1) || !isCountToken(count_token)) {
        return false;
    }
    try {
        epoch.year = std::stoi(year_token);
        epoch.month = std::stoi(month_token);
        epoch.day = std::stoi(day_token);
        epoch.hour = std::stoi(hour_token);
        epoch.minute = std::stoi(minute_token);
        epoch.flag = std::stoi(flag_token);
        epoch.record_count = std::stoi(count_token);
    } catch (...) {
        return false;
    }
    if (!parseSecondsToken(second_token, epoch.second) || epoch.flag < 0 || epoch.flag > 6 ||
        epoch.record_count < 0 || epoch.record_count > 999) {
        return false;
    }
    epoch.has_date = true;

    // Table A3: six reserved columns follow the count, then F15.12, one
    // reserved column, and an optional I5 extension.  A short line simply
    // omits optional fields; a present but malformed field is rejected.
    if (line.size() > 35) {
        const size_t reserved_length = std::min<size_t>(6, line.size() - 35);
        if (line.compare(35, reserved_length,
                         std::string(reserved_length, ' ')) != 0) {
            return false;
        }
    }
    if (line.size() > 41) {
        const std::string clock_field = line.substr(41, 15);
        if (!trim(clock_field).empty()) {
            if (!parseFixedDecimal(clock_field, 12, epoch.receiver_clock_offset)) {
                return false;
            }
            epoch.has_receiver_clock_offset = true;
        }
    }
    if (line.size() > 57) {
        const std::string extra_field = line.substr(57, 5);
        const std::string extra = trim(extra_field);
        if (!extra.empty()) {
            if (extra.size() != 5 || !isUnsignedDigits(extra)) {
                return false;
            }
            try {
                epoch.extra_second_digits = std::stoi(extra);
            } catch (...) {
                return false;
            }
            double combined_second = 0.0;
            if (!parseFixedDecimal(second_token + extra, 12, combined_second)) {
                return false;
            }
            epoch.second = combined_second;
        }
    }
    if (line.size() > 62 && !trim(line.substr(62)).empty()) {
        return false;
    }
    if (epoch.second > 60.0) {
        return false;
    }
    return validCalendar(epoch);
}

bool isCompactRinexPath(const std::string& filename) {
    std::string lower = filename;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    const auto ends_with = [&lower](const std::string& suffix) {
        return lower.size() >= suffix.size() &&
               lower.compare(lower.size() - suffix.size(), suffix.size(), suffix) == 0;
    };
    return ends_with(".crx") || ends_with(".crx.gz");
}

}  // namespace libgnss::io::rinex4
