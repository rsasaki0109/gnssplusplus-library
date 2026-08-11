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

/**
 * @brief Parsed metadata from a RINEX 4 observation epoch record.
 */
struct ObservationEpochHeader {
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    double second = 0.0;
    int flag = 0;
    int record_count = 0;
    double receiver_clock_offset = 0.0;
    bool has_receiver_clock_offset = false;
    int extra_second_digits = 0;
    bool has_date = false;
};

/**
 * @brief Parse a RINEX 4 observation epoch/event record.
 *
 * The required calendar, seconds, flag, and count fields are read as tokens
 * so the optional picosecond extension cannot shift the flag or count.  The
 * optional clock estimate and extension are then checked at their fixed
 * Table A3 locations.
 */
bool parseObservationEpochHeader(const std::string& line,
                                 ObservationEpochHeader& epoch);

/**
 * @brief Return true only for CompactRINEX filename suffixes.
 */
bool isCompactRinexPath(const std::string& filename);

}  // namespace libgnss::io::rinex4
