#pragma once

#include <string>

namespace libgnss::io::rinex4 {

inline constexpr double kVersion = 4.02;

/**
 * @brief Metadata carried by a RINEX 4 navigation data-record header.
 *
 * The source field is a satellite identifier for EPH records (for example
 * G01) and may be constellation-only for STO/EOP/ION records (for example
 * R).  The body parser is intentionally kept in RINEXReader; this type only
 * describes the format boundary.
 */
struct NavigationRecordHeader {
    std::string record_type;
    std::string source;
    std::string message_type;
    std::string subtype;
    char system = '\0';
    int prn = 0;
};

/**
 * @brief Parse a RINEX 4 navigation data-record header line.
 *
 * Returns false for malformed syntax.  Known-but-not-yet-supported GNSS
 * constellations remain syntactically valid so callers can report them as an
 * unsupported EPH message rather than treating them as malformed input.
 */
bool parseNavigationRecordHeader(const std::string& line,
                                 NavigationRecordHeader& record);

/**
 * @brief Check whether an EPH message can reuse the existing body parser.
 */
bool supportsEphemerisMessage(char system, const std::string& message_type);

}  // namespace libgnss::io::rinex4
