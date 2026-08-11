#include <libgnss++/io/rinex4.hpp>

#include <cctype>
#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <sstream>
#include <utility>

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
    record = NavigationRecordHeader();
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
    // example R).  Keep the distinction in the parsed header so system-data
    // validators can apply their record-specific source rules.
    if (record.source.size() != 1 && record.source.size() != 3) {
        return false;
    }
    if (!isKnownConstellation(record.source[0])) {
        return false;
    }
    record.system = record.source[0];
    if (record.source.size() == 3) {
        if (!std::isdigit(static_cast<unsigned char>(record.source[1])) ||
            !std::isdigit(static_cast<unsigned char>(record.source[2]))) {
            return false;
        }
        record.prn = (record.source[1] - '0') * 10 + (record.source[2] - '0');
        if (record.prn <= 0) {
            return false;
        }
    } else if (record.record_type == "EPH") {
        return false;
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

bool parseDateField(const std::string& field, size_t width, int& value) {
    if (field.size() != width || !isUnsignedDigits(field)) {
        return false;
    }
    try {
        size_t consumed = 0;
        value = std::stoi(field, &consumed);
        return consumed == field.size();
    } catch (...) {
        return false;
    }
}

bool validCalendar(const CalendarTime& time) {
    if (time.year < 1 || time.year > 9999 || time.month < 1 || time.month > 12 ||
        time.hour < 0 || time.hour > 23 || time.minute < 0 || time.minute > 59 ||
        time.second < 0 || time.second > 60) {
        return false;
    }
    static constexpr int kDaysInMonth[] = {
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    int days = kDaysInMonth[time.month - 1];
    if (time.month == 2 && isLeapYear(time.year)) {
        days = 29;
    }
    return time.day >= 1 && time.day <= days;
}

bool parseCalendarPrefix(const std::string& line,
                         CalendarTime& time,
                         size_t& field_start) {
    if (line.size() < 24 ||
        line[0] != ' ' || line[1] != ' ' || line[2] != ' ' || line[3] != ' ' ||
        line[8] != ' ' || line[11] != ' ' || line[14] != ' ' ||
        line[17] != ' ' || line[20] != ' ' || line[23] != ' ') {
        return false;
    }
    CalendarTime parsed;
    if (!parseDateField(line.substr(4, 4), 4, parsed.year) ||
        !parseDateField(line.substr(9, 2), 2, parsed.month) ||
        !parseDateField(line.substr(12, 2), 2, parsed.day) ||
        !parseDateField(line.substr(15, 2), 2, parsed.hour) ||
        !parseDateField(line.substr(18, 2), 2, parsed.minute) ||
        !parseDateField(line.substr(21, 2), 2, parsed.second) ||
        !validCalendar(parsed)) {
        return false;
    }
    time = parsed;
    field_start = 24;
    return true;
}

bool parseFloatingField(const std::string& field, double& value) {
    std::string token = trim(field);
    if (token.empty()) {
        return false;
    }
    for (char& c : token) {
        if (c == 'D' || c == 'd') {
            c = 'E';
        }
    }
    try {
        size_t consumed = 0;
        value = std::stod(token, &consumed);
        return consumed == token.size() && std::isfinite(value);
    } catch (...) {
        return false;
    }
}

bool parseFixedNumbers(const std::string& line,
                       size_t field_start,
                       size_t count,
                       std::vector<double>& values) {
    const size_t end = field_start + count * 19;
    if (line.size() < end) {
        return false;
    }
    std::vector<double> parsed;
    parsed.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        double value = 0.0;
        if (!parseFloatingField(line.substr(field_start + i * 19, 19), value)) {
            return false;
        }
        parsed.push_back(value);
    }
    if (!trim(line.substr(end)).empty()) {
        return false;
    }
    values = std::move(parsed);
    return true;
}

bool parseContinuationNumbers(const std::string& line,
                              size_t field_start,
                              size_t count,
                              std::vector<double>& values) {
    if (line.size() < 4 || line.compare(0, 4, "    ") != 0) {
        return false;
    }
    return parseFixedNumbers(line, field_start, count, values);
}

bool parseA18Fields(const std::string& line,
                    size_t field_start,
                    size_t count,
                    std::vector<std::string>& values) {
    const size_t expected_end = field_start + count * 19 - 1;
    if (line.size() > expected_end && !trim(line.substr(expected_end)).empty()) {
        return false;
    }
    std::string padded = line;
    padded.resize(expected_end, ' ');
    std::vector<std::string> parsed;
    parsed.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        const size_t start = field_start + i * 19;
        if (start == 0 || padded[start - 1] != ' ') {
            return false;
        }
        parsed.push_back(trim(padded.substr(start, 18)));
    }
    values = std::move(parsed);
    return true;
}

bool parseDateAndNumbers(const std::string& line,
                         CalendarTime& time,
                         size_t count,
                         std::vector<double>& values) {
    size_t field_start = 0;
    return parseCalendarPrefix(line, time, field_start) &&
           parseFixedNumbers(line, field_start, count, values);
}

bool isSystemTimeMessage(char system, const std::string& message_type) {
    if (message_type == "LNAV") {
        return system == 'G' || system == 'J' || system == 'I';
    }
    if (message_type == "FDMA" || message_type == "LXOC") {
        return system == 'R';
    }
    if (message_type == "IFNV" || message_type == "LEG") {
        return system == 'E';
    }
    if (message_type == "D1D2") {
        return system == 'C';
    }
    if (message_type == "SBAS") {
        return system == 'S';
    }
    if (message_type == "CNVX") {
        return system == 'G' || system == 'J' || system == 'C';
    }
    if (message_type == "L1NV") {
        return system == 'I';
    }
    return false;
}

bool isEarthOrientationMessage(char system, const std::string& message_type) {
    if (message_type == "CNVX") {
        return system == 'G' || system == 'C' || system == 'J';
    }
    if (message_type == "LNAV" || message_type == "L1NV") {
        return system == 'I';
    }
    if (message_type == "LXOC") {
        return system == 'R';
    }
    return false;
}

bool isTimeOffsetCode(const std::string& code) {
    static constexpr const char* kCodes[] = {
        "GPUT", "GLUT", "GLGP", "GAUT", "GAGP", "GAGL",
        "BDUT", "BDGP", "BDGL", "BDGA", "QZUT", "QZGP",
        "QZGL", "QZGA", "QZBD", "IRUT", "IRGP", "IRGL",
        "IRGA", "IRBD", "IRQZ", "SBUT", "SBGP", "SBGL",
        "SBGA", "SBBD", "SBQZ", "SBIR"};
    return std::any_of(std::begin(kCodes), std::end(kCodes),
                       [&](const char* value) { return code == value; });
}

const char* timeSystemPrefix(char system) {
    switch (system) {
        case 'G': return "GP";
        case 'R': return "GL";
        case 'E': return "GA";
        case 'C': return "BD";
        case 'J': return "QZ";
        case 'I': return "IR";
        case 'S': return "SB";
        default: return "";
    }
}

bool isUtcId(const std::string& value) {
    static constexpr const char* kIds[] = {
        "UTC(USNO)", "UTC(SU)", "UTCGAL", "UTC(NTSC)", "UTC(NICT)",
        "UTC(NPLI)", "UTCIRN", "UTC(OP)", "UTC(NIST)"};
    return std::any_of(std::begin(kIds), std::end(kIds),
                       [&](const char* id) { return value == id; });
}

bool isSbasId(const std::string& value) {
    static constexpr const char* kIds[] = {
        "WAAS", "EGNOS", "MSAS", "GAGAN", "SDCM", "BDSBAS",
        "KASS", "A-SBAS", "SPAN"};
    return std::any_of(std::begin(kIds), std::end(kIds),
                       [&](const char* id) { return value == id; });
}

bool isUtcIdForSystem(char system, const std::string& value) {
    if (!isUtcId(value)) {
        return false;
    }
    switch (system) {
        case 'G': return value == "UTC(USNO)";
        case 'R': return value == "UTC(SU)";
        case 'E': return value == "UTCGAL";
        case 'C': return value == "UTC(NTSC)";
        case 'J': return value == "UTC(NICT)";
        case 'I': return value == "UTCIRN" || value == "UTC(NPLI)";
        // Table 27 defines the SBAS UTC realization identifiers explicitly;
        // the identifiers used by the other constellations are not valid for
        // an SBUT record.
        case 'S': return value == "UTC(USNO)" || value == "UTC(NICT)" ||
                         value == "UTC(NIST)" || value == "UTC(OP)" ||
                         value == "UTC(NTSC)";
        default: return false;
    }
}

bool validStoContext(const NavigationRecordHeader& header,
                     const std::string& correction_type,
                     const std::string& sbas_id,
                     const std::string& utc_id) {
    if (header.record_type != "STO" || !header.subtype.empty() ||
        !isSystemTimeMessage(header.system, header.message_type) ||
        !isTimeOffsetCode(correction_type) ||
        correction_type.substr(0, 2) != timeSystemPrefix(header.system)) {
        return false;
    }
    const bool is_sbas_utc = correction_type == "SBUT";
    const bool is_utc = correction_type.size() == 4 &&
                        correction_type.substr(2) == "UT";
    if (is_sbas_utc) {
        return header.system == 'S' && isSbasId(sbas_id) &&
               isUtcIdForSystem(header.system, utc_id);
    }
    return sbas_id.empty() &&
           (is_utc ? isUtcIdForSystem(header.system, utc_id) : utc_id.empty());
}

bool validIonosphereHeader(const NavigationRecordHeader& header) {
    if (header.record_type != "ION") {
        return false;
    }
    if (header.system == 'E') {
        return header.message_type == "IFNV" && header.subtype.empty();
    }
    if (header.system == 'R') {
        return header.message_type == "LXOC" && header.subtype.empty();
    }
    if (header.system == 'I') {
        if (header.message_type == "L1NV") {
            return header.subtype == "KLOB" || header.subtype == "NEQN";
        }
        return header.message_type == "LNAV" && header.subtype.empty();
    }
    if (header.system == 'C' &&
        (header.message_type == "D1D2" || header.message_type == "CNVX")) {
        return header.subtype.empty();
    }
    if (header.system == 'G' || header.system == 'J') {
        if (header.message_type == "LNAV") {
            return header.subtype.empty();
        }
        if (header.message_type == "CNVX") {
            return header.system == 'J'
                ? (header.subtype == "WIDE" || header.subtype == "JAPN")
                : header.subtype.empty();
        }
    }
    return false;
}

}  // namespace

bool parseSystemTimeOffsetRecord(const NavigationRecordHeader& header,
                                 const std::vector<std::string>& body,
                                 SystemTimeOffsetRecord& record) {
    if (body.size() != 2 || !isSystemTimeMessage(header.system, header.message_type) ||
        header.record_type != "STO" || !header.subtype.empty()) {
        return false;
    }

    SystemTimeOffsetRecord parsed;
    parsed.header = header;
    size_t field_start = 0;
    std::vector<std::string> text_fields;
    if (!parseCalendarPrefix(body[0], parsed.reference_epoch, field_start) ||
        !parseA18Fields(body[0], field_start, 3, text_fields) ||
        text_fields.size() != 3 ||
        !validStoContext(header, text_fields[0], text_fields[1], text_fields[2])) {
        return false;
    }
    std::vector<double> numbers;
    if (!parseContinuationNumbers(body[1], 4, 4, numbers)) {
        return false;
    }
    parsed.correction_type = text_fields[0];
    parsed.sbas_id = text_fields[1];
    parsed.utc_id = text_fields[2];
    parsed.transmission_time = numbers[0];
    parsed.a0 = numbers[1];
    parsed.a1 = numbers[2];
    parsed.a2 = numbers[3];
    record = std::move(parsed);
    return true;
}

bool parseEarthOrientationRecord(const NavigationRecordHeader& header,
                                 const std::vector<std::string>& body,
                                 EarthOrientationRecord& record) {
    if (body.size() != 3 || header.record_type != "EOP" ||
        !header.subtype.empty() || !isEarthOrientationMessage(
            header.system, header.message_type)) {
        return false;
    }

    EarthOrientationRecord parsed;
    parsed.header = header;
    std::vector<double> numbers;
    if (!parseDateAndNumbers(body[0], parsed.reference_epoch, 3, numbers)) {
        return false;
    }
    parsed.x_p = numbers[0];
    parsed.x_p_rate = numbers[1];
    parsed.x_p_acceleration = numbers[2];

    if (body[1].size() < 23 || body[1][0] != ' ' || body[1][1] != ' ' ||
        body[1][2] != ' ' || body[1][3] != ' ') {
        return false;
    }
    if (!trim(body[1].substr(4, 19)).empty()) {
        return false;
    }
    std::vector<double> y_values;
    std::vector<double> ut1_values;
    if (!parseContinuationNumbers(body[1], 23, 3, y_values) ||
        !parseContinuationNumbers(body[2], 4, 4, ut1_values)) {
        return false;
    }
    parsed.y_p = y_values[0];
    parsed.y_p_rate = y_values[1];
    parsed.y_p_acceleration = y_values[2];
    parsed.transmission_time = ut1_values[0];
    parsed.delta_ut1 = ut1_values[1];
    parsed.delta_ut1_rate = ut1_values[2];
    parsed.delta_ut1_acceleration = ut1_values[3];
    record = std::move(parsed);
    return true;
}

bool parseIonosphereRecord(const NavigationRecordHeader& header,
                           const std::vector<std::string>& body,
                           IonosphereRecord& record) {
    if (!validIonosphereHeader(header)) {
        return false;
    }

    IonosphereRecord parsed;
    parsed.header = header;
    std::vector<double> numbers;

    if ((header.system == 'G' || header.system == 'J' ||
         header.system == 'I') &&
        (header.message_type == "LNAV" ||
         (header.message_type == "CNVX" &&
          (header.system == 'G' || header.system == 'J')))) {
        KlobucharIonosphere model;
        if (body.size() != 3 ||
            !parseDateAndNumbers(body[0], model.transmit_time, 3, numbers)) {
            return false;
        }
        model.alpha[0] = numbers[0];
        model.alpha[1] = numbers[1];
        model.alpha[2] = numbers[2];
        std::vector<double> beta;
        if (!parseContinuationNumbers(body[1], 4, 4, beta) ||
            !parseContinuationNumbers(body[2], 4, 1, numbers)) {
            return false;
        }
        model.alpha[3] = beta[0];
        model.beta[0] = beta[1];
        model.beta[1] = beta[2];
        model.beta[2] = beta[3];
        model.beta[3] = numbers[0];
        parsed.payload = model;
    } else if (header.system == 'C' && header.message_type == "D1D2") {
        KlobucharIonosphere model;
        if (body.size() != 3 ||
            !parseDateAndNumbers(body[0], model.transmit_time, 3, numbers)) {
            return false;
        }
        model.alpha[0] = numbers[0];
        model.alpha[1] = numbers[1];
        model.alpha[2] = numbers[2];
        std::vector<double> beta;
        if (!parseContinuationNumbers(body[1], 4, 4, beta) ||
            !parseContinuationNumbers(body[2], 4, 1, numbers)) {
            return false;
        }
        model.alpha[3] = beta[0];
        model.beta[0] = beta[1];
        model.beta[1] = beta[2];
        model.beta[2] = beta[3];
        model.beta[3] = numbers[0];
        parsed.payload = model;
    } else if (header.system == 'E') {
        NequickGIonosphere model;
        if (body.size() != 2 ||
            !parseDateAndNumbers(body[0], model.transmit_time, 3, numbers)) {
            return false;
        }
        model.ai[0] = numbers[0];
        model.ai[1] = numbers[1];
        model.ai[2] = numbers[2];
        std::vector<double> flags;
        if (!parseContinuationNumbers(body[1], 4, 1, flags)) {
            return false;
        }
        if (!std::isfinite(flags[0]) || flags[0] < 0.0 || flags[0] > 31.0 ||
            std::floor(flags[0]) != flags[0]) {
            return false;
        }
        model.disturbance_flags = static_cast<int>(flags[0]);
        parsed.payload = model;
    } else if (header.system == 'C' && header.message_type == "CNVX") {
        BdgimIonosphere model;
        if (body.size() != 3 ||
            !parseDateAndNumbers(body[0], model.transmit_time, 3, numbers)) {
            return false;
        }
        model.alpha[0] = numbers[0];
        model.alpha[1] = numbers[1];
        model.alpha[2] = numbers[2];
        std::vector<double> line1;
        std::vector<double> line2;
        if (!parseContinuationNumbers(body[1], 4, 4, line1) ||
            !parseContinuationNumbers(body[2], 4, 2, line2)) {
            return false;
        }
        for (size_t i = 0; i < line1.size(); ++i) {
            model.alpha[3 + i] = line1[i];
        }
        model.alpha[7] = line2[0];
        model.alpha[8] = line2[1];
        parsed.payload = model;
    } else if (header.system == 'I' && header.message_type == "L1NV" &&
               header.subtype == "KLOB") {
        NavicKlobucharIonosphere model;
        if (body.size() != 4 ||
            !parseDateAndNumbers(body[0], model.transmit_time, 1, numbers)) {
            return false;
        }
        model.issue_of_data = numbers[0];
        std::vector<double> alpha;
        std::vector<double> beta;
        std::vector<double> region_bounds;
        if (!parseContinuationNumbers(body[1], 4, 4, alpha) ||
            !parseContinuationNumbers(body[2], 4, 4, beta) ||
            !parseContinuationNumbers(body[3], 4, 4, region_bounds)) {
            return false;
        }
        std::copy(alpha.begin(), alpha.end(), model.alpha.begin());
        std::copy(beta.begin(), beta.end(), model.beta.begin());
        std::copy(region_bounds.begin(), region_bounds.end(), model.region_bounds.begin());
        parsed.payload = model;
    } else if (header.system == 'I' && header.message_type == "L1NV" &&
               header.subtype == "NEQN") {
        NavicNequickNIonosphere model;
        if (body.size() != 7 ||
            !parseDateAndNumbers(body[0], model.transmit_time, 1, numbers)) {
            return false;
        }
        model.issue_of_data = numbers[0];
        for (size_t region = 0; region < 3; ++region) {
            std::vector<double> coefficients;
            std::vector<double> bounds;
            if (!parseContinuationNumbers(body[1 + region * 2], 4, 4, coefficients) ||
                !parseContinuationNumbers(body[2 + region * 2], 4, 4, bounds)) {
                return false;
            }
            model.coefficients[region][0] = coefficients[0];
            model.coefficients[region][1] = coefficients[1];
            model.coefficients[region][2] = coefficients[2];
            if (!std::isfinite(coefficients[3]) || coefficients[3] < 0.0 ||
                coefficients[3] > static_cast<double>(std::numeric_limits<int>::max()) ||
                std::floor(coefficients[3]) != coefficients[3]) {
                return false;
            }
            model.disturbance_flags[region] = static_cast<int>(coefficients[3]);
            std::copy(bounds.begin(), bounds.end(), model.region_bounds[region].begin());
        }
        parsed.payload = model;
    } else if (header.system == 'R' && header.message_type == "LXOC") {
        GlonassCdmaIonosphere model;
        if (body.size() != 1 ||
            !parseDateAndNumbers(body[0], model.transmit_time, 3, numbers)) {
            return false;
        }
        model.c_a = numbers[0];
        model.c_f10_7 = numbers[1];
        model.c_ap = numbers[2];
        parsed.payload = model;
    } else {
        return false;
    }

    record = std::move(parsed);
    return true;
}

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
