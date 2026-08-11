#include <libgnss++/io/rinex4.hpp>

#include <cctype>
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
    fields >> record.subtype;

    if (record.record_type.size() != 3 || record.source.empty() ||
        record.message_type.empty()) {
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
    return true;
}

bool supportsEphemerisMessage(char system, const std::string& message_type) {
    // These are the RINEX 4 records whose body layout is compatible with the
    // existing RINEX 2/3 parser.  New signal-specific records (CNAV/CNVx,
    // NavIC L1NV, and GLONASS CDMA L1OC/L3OC) must not be passed to it: doing
    // so would turn their fields into a plausible but incorrect ephemeris.
    switch (system) {
        case 'G':
        case 'J':
            return message_type == "LNAV";
        case 'R':
            return message_type == "FDMA";
        case 'E':
            return message_type == "FNAV" || message_type == "INAV";
        case 'C':
            return message_type == "D1" || message_type == "D2";
        default:
            return false;
    }
}

}  // namespace libgnss::io::rinex4
