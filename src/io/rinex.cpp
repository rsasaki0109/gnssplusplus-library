#include <libgnss++/io/rinex.hpp>
#include <libgnss++/core/signal_policy.hpp>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <cmath>

namespace libgnss {
namespace io {

namespace {

GNSSSystem systemFromRinexChar(char sys_char) {
    switch (sys_char) {
        case 'G': return GNSSSystem::GPS;
        case 'R': return GNSSSystem::GLONASS;
        case 'E': return GNSSSystem::Galileo;
        case 'C': return GNSSSystem::BeiDou;
        case 'J': return GNSSSystem::QZSS;
        case 'S': return GNSSSystem::SBAS;
        case 'I': return GNSSSystem::NavIC;
        default: return GNSSSystem::UNKNOWN;
    }
}

bool supportsBroadcastNavigationSystem(char sys_char) {
    return sys_char == 'G' || sys_char == 'R' || sys_char == 'E' || sys_char == 'C' || sys_char == 'J';
}

int leapSecondsForDate(int year, int month, int day) {
    struct LeapEntry { int year; int month; int day; int leap_seconds; };
    static constexpr LeapEntry kLeapTable[] = {
        {1981, 7, 1, 1},  {1982, 7, 1, 2},  {1983, 7, 1, 3},  {1985, 7, 1, 4},
        {1988, 1, 1, 5},  {1990, 1, 1, 6},  {1991, 1, 1, 7},  {1992, 7, 1, 8},
        {1993, 7, 1, 9},  {1994, 7, 1, 10}, {1996, 1, 1, 11}, {1997, 7, 1, 12},
        {1999, 1, 1, 13}, {2006, 1, 1, 14}, {2009, 1, 1, 15}, {2012, 7, 1, 16},
        {2015, 7, 1, 17}, {2017, 1, 1, 18},
    };

    int leap_seconds = 0;
    for (const auto& entry : kLeapTable) {
        if (year > entry.year ||
            (year == entry.year && (month > entry.month ||
             (month == entry.month && day >= entry.day)))) {
            leap_seconds = entry.leap_seconds;
        }
    }
    return leap_seconds;
}

void parseCalendarFields(const std::string& time_field,
                         int& year,
                         int& month,
                         int& day,
                         int& hour,
                         int& minute,
                         double& second) {
    std::istringstream iss(time_field);
    iss >> year >> month >> day >> hour >> minute >> second;
    if (year < 80) {
        year += 2000;
    } else if (year < 100) {
        year += 1900;
    }
}

GNSSTime normalizeWeekTow(int week, double tow) {
    while (tow < 0.0) {
        tow += constants::SECONDS_PER_WEEK;
        week--;
    }
    while (tow >= constants::SECONDS_PER_WEEK) {
        tow -= constants::SECONDS_PER_WEEK;
        week++;
    }
    return GNSSTime(week, tow);
}

GNSSTime utcToGpst(const GNSSTime& utc_time, int year, int month, int day) {
    return utc_time + static_cast<double>(leapSecondsForDate(year, month, day));
}

GNSSTime adjustDay(const GNSSTime& time, const GNSSTime& reference) {
    const double diff = time - reference;
    if (diff < -43200.0) return time + 86400.0;
    if (diff > 43200.0) return time - 86400.0;
    return time;
}

SignalType primarySignalForSystem(GNSSSystem system) {
    return signal_policy::primarySignalForSystem(system);
}

SignalType secondarySignalForSystem(GNSSSystem system) {
    return signal_policy::secondarySignalForSystem(system);
}

SignalType signalForObservationType(GNSSSystem system, const std::string& obs_type, bool primary) {
    return signal_policy::signalForObservationType(system, obs_type, primary);
}

GNSSTime bdtToGpst(const GNSSTime& time) {
    return time + 14.0;
}

GNSSTime gpstToBdt(const GNSSTime& time) {
    return time - 14.0;
}

GNSSTime bdtWeekTowToGpst(int week, double tow) {
    static constexpr int kBdtWeekOffset = 1356;
    return GNSSTime(week + kBdtWeekOffset, tow) + 14.0;
}

std::string trimCopy(const std::string& text) {
    const size_t first = text.find_first_not_of(' ');
    if (first == std::string::npos) {
        return "";
    }
    const size_t last = text.find_last_not_of(' ');
    return text.substr(first, last - first + 1);
}

int rinexBand(const std::string& obs_type) {
    return signal_policy::rinexBand(obs_type);
}

bool isPrimaryBand(GNSSSystem system, int band) {
    return signal_policy::observationPriority(system, "C" + std::to_string(band), true) < 100;
}

bool isSecondaryBand(GNSSSystem system, int band) {
    return signal_policy::observationPriority(system, "C" + std::to_string(band), false) < 100;
}

int bandPriority(GNSSSystem system, int band, bool primary) {
    return signal_policy::observationPriority(system, "C" + std::to_string(band), primary);
}

constexpr int kInvalidObservationPriority = 1000;

std::string trackingPriorityChars(GNSSSystem system, int band) {
    switch (system) {
        case GNSSSystem::GPS:
            if (band == 1) return "CPYWMNSL";
            if (band == 2) return "PYWCMNDLSX";
            if (band == 5) return "QXI";
            break;
        case GNSSSystem::GLONASS:
            if (band == 1 || band == 2) return "CP";
            break;
        case GNSSSystem::Galileo:
            if (band == 1) return "CABXZ";
            if (band == 5 || band == 7 || band == 8) return "QXI";
            if (band == 6) return "CXE";
            break;
        case GNSSSystem::BeiDou:
            if (band == 2 || band == 6) return "IQX";
            if (band == 7) return "DIQX";
            if (band == 1 || band == 5 || band == 8) return "PXD";
            break;
        case GNSSSystem::QZSS:
            if (band == 1) return "LXSCE";
            if (band == 2) return "LXS";
            if (band == 5) return "QXI";
            if (band == 6) return "SEZ";
            break;
        case GNSSSystem::NavIC:
            if (band == 5 || band == 9) return "ABCX";
            break;
        default:
            break;
    }
    return "";
}

int trackingCodePriority(GNSSSystem system, const std::string& obs_type) {
    if (obs_type.size() < 3) {
        return 0;
    }
    const std::string priorities = trackingPriorityChars(system, rinexBand(obs_type));
    if (priorities.empty()) {
        return 0;
    }
    const size_t pos = priorities.find(obs_type[2]);
    return pos == std::string::npos ? static_cast<int>(priorities.size()) : static_cast<int>(pos);
}

int observationSelectionPriority(GNSSSystem system, const std::string& obs_type, bool primary) {
    const int signal_priority = signal_policy::observationPriority(system, obs_type, primary);
    if (signal_priority >= 100) {
        return kInvalidObservationPriority;
    }
    return signal_priority * 16 + trackingCodePriority(system, obs_type);
}

std::string observationSelectionKey(const std::string& obs_type) {
    return obs_type.size() >= 3 ? obs_type.substr(1, 2) : std::string{};
}

bool matchesObservationSelection(const std::string& obs_type,
                                 int selected_band,
                                 const std::string& selected_key) {
    if (selected_band < 0 || rinexBand(obs_type) != selected_band) {
        return false;
    }
    return selected_key.empty() || observationSelectionKey(obs_type) == selected_key;
}

bool isCodeObservationType(const std::string& obs_type) {
    return !obs_type.empty() && (obs_type[0] == 'C' || obs_type[0] == 'P');
}

bool isCarrierObservationType(const std::string& obs_type) {
    return !obs_type.empty() && obs_type[0] == 'L';
}

bool isDopplerObservationType(const std::string& obs_type) {
    return !obs_type.empty() && obs_type[0] == 'D';
}

bool isSnrObservationType(const std::string& obs_type) {
    return !obs_type.empty() && obs_type[0] == 'S';
}

char rinexCharForSystem(GNSSSystem system) {
    switch (system) {
        case GNSSSystem::GPS: return 'G';
        case GNSSSystem::GLONASS: return 'R';
        case GNSSSystem::Galileo: return 'E';
        case GNSSSystem::BeiDou: return 'C';
        case GNSSSystem::QZSS: return 'J';
        case GNSSSystem::SBAS: return 'S';
        case GNSSSystem::NavIC: return 'I';
        default: return 'G';
    }
}

void gpstToCalendar(const GNSSTime& time,
                    int& year,
                    int& month,
                    int& day,
                    int& hour,
                    int& minute,
                    double& second) {
    const int total_days = time.week * 7 + static_cast<int>(std::floor(time.tow / 86400.0));
    const double seconds_of_day = time.tow - std::floor(time.tow / 86400.0) * 86400.0;

    // GPS epoch 1980-01-06 maps to civil day index 3657 relative to 1970-01-01.
    int z = total_days + 3657;
    z += 719468;
    const int era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned doe = static_cast<unsigned>(z - era * 146097);
    const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    int y = static_cast<int>(yoe) + era * 400;
    const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const unsigned mp = (5 * doy + 2) / 153;
    const unsigned d = doy - (153 * mp + 2) / 5 + 1;
    const unsigned m = mp + (mp < 10 ? 3 : -9);
    y += (m <= 2);

    year = y;
    month = static_cast<int>(m);
    day = static_cast<int>(d);
    hour = static_cast<int>(seconds_of_day / 3600.0);
    minute = static_cast<int>((seconds_of_day - hour * 3600.0) / 60.0);
    second = seconds_of_day - hour * 3600.0 - minute * 60.0;
}

void gpstToUtcCalendar(const GNSSTime& time,
                       int& year,
                       int& month,
                       int& day,
                       int& hour,
                       int& minute,
                       double& second) {
    int gps_year = 0;
    int gps_month = 0;
    int gps_day = 0;
    int gps_hour = 0;
    int gps_minute = 0;
    double gps_second = 0.0;
    gpstToCalendar(time, gps_year, gps_month, gps_day, gps_hour, gps_minute, gps_second);
    const int leap_seconds = leapSecondsForDate(gps_year, gps_month, gps_day);
    gpstToCalendar(time - static_cast<double>(leap_seconds), year, month, day, hour, minute, second);
}

std::string formatRinexFloat(double value) {
    std::ostringstream oss;
    oss << std::uppercase << std::scientific << std::setw(19) << std::setprecision(12) << value;
    std::string formatted = oss.str();
    std::replace(formatted.begin(), formatted.end(), 'E', 'D');
    return formatted;
}

void assignObservationField(Observation& obs,
                            const std::string& obs_type,
                            double value,
                            int lli,
                            int signal_strength) {
    if (isCodeObservationType(obs_type)) {
        obs.observation_code = obs_type;
        obs.pseudorange = value;
        obs.has_pseudorange = true;
    } else if (isCarrierObservationType(obs_type)) {
        obs.carrier_phase = value;
        obs.has_carrier_phase = true;
        obs.lli = static_cast<uint8_t>(lli);
        obs.loss_of_lock = (lli & 0x01) != 0;
    } else if (isDopplerObservationType(obs_type)) {
        obs.doppler = value;
        obs.has_doppler = true;
    } else if (isSnrObservationType(obs_type)) {
        obs.snr = value;
    }

    if (signal_strength > 0) {
        obs.signal_strength = signal_strength;
        if (obs.snr == 0.0) {
            obs.snr = static_cast<double>(signal_strength);
        }
    }
}

// For a single satellite epoch, emit one additional Observation per band
// that is neither the primary-selected nor secondary-selected band but
// still maps to a valid secondary-priority tracking code.  This enables
// 3rd-frequency ambiguity states (EWL cascade, QZSS/GPS L5, Galileo E5a,
// BDS B2a) to be populated even though the primary/secondary picker only
// keeps two bands.  The emitted Observation carries the code, phase,
// Doppler, and SNR fields for the best tracking code within the band.
void emitExtraBandObservations(
    GNSSSystem system,
    const SatelliteId& sat,
    const std::vector<std::string>& obs_types,
    const std::vector<double>& obs_values,
    const std::vector<int>& lli_flags,
    const std::vector<int>& signal_strength,
    int primary_band,
    int secondary_band,
    ObservationData& obs_data) {
    struct BandChoice {
        int priority = kInvalidObservationPriority;
        std::string key;
    };
    std::map<int, BandChoice> band_choices;

    const size_t obs_count = std::min(obs_types.size(), obs_values.size());
    for (size_t i = 0; i < obs_count; ++i) {
        if (obs_values[i] == 0.0) continue;
        const std::string& obs_type = obs_types[i];
        if (!isCodeObservationType(obs_type)) continue;
        const int band = rinexBand(obs_type);
        if (band < 0 || band == primary_band || band == secondary_band) continue;
        const int priority = observationSelectionPriority(system, obs_type, false);
        if (priority >= kInvalidObservationPriority) continue;
        auto it = band_choices.find(band);
        if (it == band_choices.end() || priority < it->second.priority) {
            band_choices[band] = {priority, observationSelectionKey(obs_type)};
        }
    }

    for (const auto& [band, choice] : band_choices) {
        Observation obs_extra;
        obs_extra.satellite = sat;
        obs_extra.signal = secondarySignalForSystem(system);
        obs_extra.valid = true;
        bool has_data = false;

        for (size_t i = 0; i < obs_count; ++i) {
            if (obs_values[i] == 0.0) continue;
            const std::string& obs_type = obs_types[i];
            if (!matchesObservationSelection(obs_type, band, choice.key)) continue;
            if (!has_data) {
                obs_extra.signal = signalForObservationType(system, obs_type, false);
            }
            assignObservationField(
                obs_extra, obs_type, obs_values[i],
                lli_flags[i], signal_strength[i]);
            has_data = true;
        }

        if (has_data) {
            obs_data.addObservation(obs_extra);
        }
    }
}

}  // namespace

bool RINEXReader::open(const std::string& filename) {
    file_.open(filename);
    current_line_ = 0;
    rinex3_obs_types_system_ = '\0';
    rinex3_obs_types_expected_ = 0;
    return file_.is_open();
}

void RINEXReader::close() {
    if (file_.is_open()) {
        file_.close();
    }
}

bool RINEXReader::readHeader(RINEXHeader& header) {
    if (!file_.is_open()) {
        return false;
    }

    rinex3_obs_types_system_ = '\0';
    rinex3_obs_types_expected_ = 0;

    std::string line;
    while (readLine(line)) {
        if (line.find("END OF HEADER") != std::string::npos) {
            break;
        }
        
        if (!parseHeaderLine(line, header)) {
            continue; // Skip invalid lines
        }
    }
    
    header_ = header;
    return true;
}

bool RINEXReader::readObservationEpoch(ObservationData& obs_data) {
    if (!file_.is_open()) {
        return false;
    }

    obs_data.clear();

    std::string line;

    // Skip lines until we find a valid epoch line
    while (readLine(line)) {
        // Skip comment lines and other header-like lines
        if (line.length() >= 60) {
            std::string label = line.substr(60);
            if (label.find("COMMENT") != std::string::npos) {
                continue;  // Skip comment lines
            }
        }

        // Check if this looks like an epoch line (RINEX 2.x format)
        // Format: " YY MM DD HH MM SS.SSSSSSS  0  N..."
        if (line.length() >= 32 && header_.version < 3.0) {
            // Epoch lines have year/month/day at specific positions
            // Position 1-2: year, position 4-5: month, position 7-8: day
            // Check if these positions contain digits or spaces (for alignment)
            bool looks_like_epoch = true;

            // Check year position (chars 1-2 should be digits)
            if (line.length() > 2) {
                char c1 = line[1];
                char c2 = line[2];
                if (!((c1 == ' ' || std::isdigit(c1)) && (c2 == ' ' || std::isdigit(c2)))) {
                    looks_like_epoch = false;
                }
            }

            // Additional check: observation data lines start with large numbers (negative pseudoranges)
            // They typically start with " -" for negative values
            if (line.length() > 10 && line[1] == '-') {
                looks_like_epoch = false;  // This is observation data, not an epoch line
            }

            if (looks_like_epoch) {
                // Try to parse it
                try {
                    return parseObservationEpochV2(line, obs_data);
                } catch (const std::exception&) {
                    // If parsing fails, continue looking for next epoch
                    continue;
                }
            }
        } else if (header_.version >= 3.0) {
            // RINEX 3.x format starts with '>'
            if (line[0] == '>') {
                return parseObservationEpochV3(line, obs_data);
            }
        }
    }

    return false;  // No more epochs found
}

bool RINEXReader::readAllObservations(ObservationSeries& obs_series) {
    obs_series.clear();
    
    ObservationData obs_data;
    while (readObservationEpoch(obs_data)) {
        obs_series.addEpoch(obs_data);
    }
    
    return !obs_series.isEmpty();
}

bool RINEXReader::readNavigationData(NavigationData& nav_data) {
    if (!file_.is_open()) {
        return false;
    }

    nav_data.clear();

    // Skip header if not already done, but parse version info and iono params
    std::string line;
    bool header_found = false;
    while (readLine(line)) {
        if (line.find("END OF HEADER") != std::string::npos) {
            header_found = true;
            break;
        }
        // Parse header lines to get version info
        if (line.length() >= 60) {
            std::string label = line.substr(60);
            if (label.find("RINEX VERSION") != std::string::npos) {
                header_.version = std::stod(line.substr(0, 9));
            }
            // Parse Klobuchar ionosphere parameters (RINEX 2: ION ALPHA / ION BETA)
            else if (label.find("ION ALPHA") != std::string::npos) {
                // Format: 4 values in D-format, 12 chars each starting at col 2
                auto parseDval = [](const std::string& s) -> double {
                    std::string v = s;
                    v.erase(0, v.find_first_not_of(' '));
                    v.erase(v.find_last_not_of(' ') + 1);
                    if (v.empty()) return 0.0;
                    auto dp = v.find('D'); if (dp != std::string::npos) v[dp] = 'E';
                    dp = v.find('d'); if (dp != std::string::npos) v[dp] = 'E';
                    try { return std::stod(v); } catch (...) { return 0.0; }
                };
                nav_data.ionosphere_model.alpha[0] = parseDval(line.substr(2, 12));
                nav_data.ionosphere_model.alpha[1] = parseDval(line.substr(14, 12));
                nav_data.ionosphere_model.alpha[2] = parseDval(line.substr(26, 12));
                nav_data.ionosphere_model.alpha[3] = parseDval(line.substr(38, 12));
                nav_data.ionosphere_model.valid = true;
            }
            else if (label.find("ION BETA") != std::string::npos) {
                auto parseDval = [](const std::string& s) -> double {
                    std::string v = s;
                    v.erase(0, v.find_first_not_of(' '));
                    v.erase(v.find_last_not_of(' ') + 1);
                    if (v.empty()) return 0.0;
                    auto dp = v.find('D'); if (dp != std::string::npos) v[dp] = 'E';
                    dp = v.find('d'); if (dp != std::string::npos) v[dp] = 'E';
                    try { return std::stod(v); } catch (...) { return 0.0; }
                };
                nav_data.ionosphere_model.beta[0] = parseDval(line.substr(2, 12));
                nav_data.ionosphere_model.beta[1] = parseDval(line.substr(14, 12));
                nav_data.ionosphere_model.beta[2] = parseDval(line.substr(26, 12));
                nav_data.ionosphere_model.beta[3] = parseDval(line.substr(38, 12));
            }
            // Parse IONOSPHERIC CORR (RINEX 3: GPSA / GPSB)
            else if (label.find("IONOSPHERIC CORR") != std::string::npos) {
                auto parseDval = [](const std::string& s) -> double {
                    std::string v = s;
                    v.erase(0, v.find_first_not_of(' '));
                    v.erase(v.find_last_not_of(' ') + 1);
                    if (v.empty()) return 0.0;
                    auto dp = v.find('D'); if (dp != std::string::npos) v[dp] = 'E';
                    dp = v.find('d'); if (dp != std::string::npos) v[dp] = 'E';
                    try { return std::stod(v); } catch (...) { return 0.0; }
                };
                std::string corr_type = line.substr(0, 4);
                corr_type.erase(corr_type.find_last_not_of(' ') + 1);
                if (corr_type == "GPSA") {
                    nav_data.ionosphere_model.alpha[0] = parseDval(line.substr(5, 12));
                    nav_data.ionosphere_model.alpha[1] = parseDval(line.substr(17, 12));
                    nav_data.ionosphere_model.alpha[2] = parseDval(line.substr(29, 12));
                    nav_data.ionosphere_model.alpha[3] = parseDval(line.substr(41, 12));
                    nav_data.ionosphere_model.valid = true;
                } else if (corr_type == "GPSB") {
                    nav_data.ionosphere_model.beta[0] = parseDval(line.substr(5, 12));
                    nav_data.ionosphere_model.beta[1] = parseDval(line.substr(17, 12));
                    nav_data.ionosphere_model.beta[2] = parseDval(line.substr(29, 12));
                    nav_data.ionosphere_model.beta[3] = parseDval(line.substr(41, 12));
                }
            }
        }
    }

    if (!header_found) {
        // Header already processed, rewind might not work, so just continue
    }

    std::vector<std::string> eph_lines;
    int eph_detected = 0;
    int eph_parsed = 0;
    int eph_skipped_non_gps = 0;
    bool skipping_non_gps = false;
    int skip_lines_remaining = 0;

    // Detect RINEX version for navigation file format differences
    bool is_rinex3 = (header_.version >= 3.0);

    try {
        while (readLine(line)) {
            // Skip very short lines but allow shorter navigation data lines
            if (line.length() < 19) continue;

            // If we're skipping a non-GPS satellite's record, count down lines
            if (skipping_non_gps) {
                skip_lines_remaining--;
                if (skip_lines_remaining <= 0) {
                    skipping_non_gps = false;
                }
                continue;
            }

            // Check if this is start of new ephemeris
            bool is_new_ephemeris = false;
            bool is_gps = true;

            if (is_rinex3 && line.length() >= 5) {
                // RINEX 3: first char is system identifier (G, R, E, C, J, S)
                // followed by 2-digit PRN, then 4-digit year
                char sys_char = line[0];
                bool is_sys_char = (sys_char == 'G' || sys_char == 'R' || sys_char == 'E' ||
                                    sys_char == 'C' || sys_char == 'J' || sys_char == 'S');
                if (is_sys_char) {
                    // Check that chars 1-2 are PRN digits
                    bool has_prn = (std::isdigit(line[1]) || line[1] == ' ') &&
                                   std::isdigit(line[2]);
                    if (has_prn) {
                        is_new_ephemeris = true;
                        is_gps = (sys_char == 'G');
                    }
                }
            } else if (!is_rinex3 && line.length() >= 5) {
                // RINEX 2: PRN at pos 0-1, year at pos 3-4
                bool has_year = std::isdigit(line[3]) && std::isdigit(line[4]);
                bool has_prn = (line[0] == ' ' && std::isdigit(line[1])) ||
                               (std::isdigit(line[0]) && std::isdigit(line[1]));
                is_new_ephemeris = has_prn && has_year;
            }

            if (is_new_ephemeris) {
                eph_detected++;

                if (is_rinex3 && !supportsBroadcastNavigationSystem(line[0])) {
                    // Skip non-GPS: process any pending GPS ephemeris first
                    if (!eph_lines.empty()) {
                        Ephemeris eph;
                        if (parseNavigationMessage(eph_lines, eph)) {
                            nav_data.addEphemeris(eph);
                            eph_parsed++;
                        }
                        eph_lines.clear();
                    }
                    // Skip the continuation lines of this non-GPS record
                    // GLONASS (R) and SBAS (S): 3 continuation lines (4 total)
                    // Galileo (E), BeiDou (C), QZSS (J): 7 continuation lines (8 total)
                    skipping_non_gps = true;
                    char skip_sys = line[0];
                    if (skip_sys == 'R' || skip_sys == 'S') {
                        skip_lines_remaining = 3;
                    } else {
                        skip_lines_remaining = 7;
                    }
                    eph_skipped_non_gps++;
                    continue;
                }

                // Process previous GPS ephemeris if exists
                if (!eph_lines.empty()) {
                    Ephemeris eph;
                    if (parseNavigationMessage(eph_lines, eph)) {
                        nav_data.addEphemeris(eph);
                        eph_parsed++;
                    }
                    eph_lines.clear();
                }
            }

            if (!skipping_non_gps) {
                eph_lines.push_back(line);
            }
        }

        // Process last ephemeris
        if (!eph_lines.empty()) {
            Ephemeris eph;
            if (parseNavigationMessage(eph_lines, eph)) {
                nav_data.addEphemeris(eph);
                eph_parsed++;
            }
        }

    } catch (const std::invalid_argument& e) {
        std::cerr << "Navigation data parsing error (invalid_argument): " << e.what() << std::endl;
        std::cerr << "Line number: " << current_line_ << std::endl;
        if (!eph_lines.empty()) {
            std::cerr << "Problem line: " << eph_lines.back() << std::endl;
        }
        throw;
    } catch (const std::out_of_range& e) {
        std::cerr << "Navigation data parsing error (out_of_range): " << e.what() << std::endl;
        std::cerr << "Line number: " << current_line_ << std::endl;
        throw;
    }

    return !nav_data.isEmpty();
}

bool RINEXReader::parseHeaderLine(const std::string& line, RINEXHeader& header) {
    if (line.length() < 60) return false;
    
    std::string label = line.substr(60);
    
    if (label.find("RINEX VERSION") != std::string::npos) {
        header.version = std::stod(line.substr(0, 9));
        char file_type = line[20];
        switch (file_type) {
            case 'O': header.file_type = FileType::OBSERVATION; break;
            case 'N': header.file_type = FileType::NAVIGATION; break;
            case 'M': header.file_type = FileType::METEOROLOGICAL; break;
            case 'C': header.file_type = FileType::CLOCK; break;
            default: header.file_type = FileType::UNKNOWN; break;
        }
        header.satellite_system = line.substr(40, 1);
    }
    else if (label.find("PGM / RUN BY / DATE") != std::string::npos) {
        header.program = line.substr(0, 20);
        header.run_by = line.substr(20, 20);
        header.date = line.substr(40, 20);
    }
    else if (label.find("MARKER NAME") != std::string::npos) {
        header.marker_name = line.substr(0, 60);
    }
    else if (label.find("ANT # / TYPE") != std::string::npos) {
        header.antenna_number = trimCopy(line.substr(0, 20));
        header.antenna_type = trimCopy(line.substr(20, 20));
    }
    else if (label.find("APPROX POSITION XYZ") != std::string::npos) {
        header.approximate_position(0) = std::stod(line.substr(0, 14));
        header.approximate_position(1) = std::stod(line.substr(14, 14));
        header.approximate_position(2) = std::stod(line.substr(28, 14));
    }
    else if (label.find("ANTENNA: DELTA H/E/N") != std::string::npos) {
        const double height = std::stod(line.substr(0, 14));
        const double east = std::stod(line.substr(14, 14));
        const double north = std::stod(line.substr(28, 14));
        header.antenna_delta = Vector3d(east, north, height);
    }
    else if (label.find("# / TYPES OF OBSERV") != std::string::npos) {
        // RINEX 2: Parse number of observation types
        int num_types = std::stoi(line.substr(0, 6));

        // Parse observation types (starting at position 10, 6 characters each)
        for (int i = 0; i < num_types && i < 9; ++i) {  // Max 9 types per line
            size_t pos = 10 + i * 6;
            if (pos + 2 <= line.length()) {
                std::string obs_type = line.substr(pos, 2);
                obs_type.erase(0, obs_type.find_first_not_of(' '));
                obs_type.erase(obs_type.find_last_not_of(' ') + 1);
                if (!obs_type.empty()) {
                    header.observation_types.push_back(obs_type);
                }
            }
        }
    }
    else if (label.find("SYS / # / OBS TYPES") != std::string::npos) {
        // RINEX 3: Per-system observation types. The list may continue on
        // following header lines whose system/count fields are blank.
        const char sys_char = line.empty() ? ' ' : line[0];
        if (sys_char != ' ') {
            rinex3_obs_types_system_ = sys_char;
            rinex3_obs_types_expected_ = std::stoi(line.substr(3, 3));
            header.system_obs_types[sys_char].clear();
        } else if (rinex3_obs_types_system_ == '\0') {
            return true;
        }

        auto& types = header.system_obs_types[rinex3_obs_types_system_];
        const size_t field_length = std::min<size_t>(line.length(), 60);
        for (int i = 0; i < 13 && static_cast<int>(types.size()) < rinex3_obs_types_expected_; ++i) {
            const size_t pos = 7 + static_cast<size_t>(i) * 4;
            if (pos + 3 > field_length) {
                break;
            }
            std::string obs_type = line.substr(pos, 3);
            obs_type.erase(0, obs_type.find_first_not_of(' '));
            obs_type.erase(obs_type.find_last_not_of(' ') + 1);
            if (!obs_type.empty()) {
                types.push_back(obs_type);
            }
        }

        // Also populate the generic observation_types with GPS types for backward compatibility.
        if (rinex3_obs_types_system_ == 'G') {
            header.observation_types = types;
        }

        if (rinex3_obs_types_expected_ <= 0 ||
            static_cast<int>(types.size()) >= rinex3_obs_types_expected_) {
            rinex3_obs_types_system_ = '\0';
            rinex3_obs_types_expected_ = 0;
        }
    }

    return true;
}

bool RINEXReader::parseObservationEpochV2(const std::string& line, ObservationData& obs_data) {
    // Simplified RINEX 2.x parsing
    if (line.length() < 32) return false;

    int year = 0, month = 0, day = 0, hour = 0, minute = 0;
    double second = 0.0;

    try {
        // Parse time - RINEX 2.x format: " YY MM DD HH MM SS.SSSSSSS"
        // Positions are 1-indexed in RINEX spec, but 0-indexed in substr
        std::string year_str = line.substr(1, 2);    // pos 2-3
        std::string month_str = line.substr(4, 2);   // pos 5-6
        std::string day_str = line.substr(7, 2);     // pos 8-9

        // Trim whitespace before parsing
        std::string hour_str = line.substr(10, 2);    // pos 11-12
        std::string min_str = line.substr(13, 2);     // pos 14-15
        std::string sec_str = line.substr(15, 11);    // pos 16-26

        // Remove leading and trailing spaces
        year_str.erase(0, year_str.find_first_not_of(' '));
        month_str.erase(0, month_str.find_first_not_of(' '));
        day_str.erase(0, day_str.find_first_not_of(' '));
        hour_str.erase(0, hour_str.find_first_not_of(' '));
        min_str.erase(0, min_str.find_first_not_of(' '));
        sec_str.erase(0, sec_str.find_first_not_of(' '));

        // Remove trailing spaces
        if (!sec_str.empty()) {
            size_t end = sec_str.find_last_not_of(' ');
            if (end != std::string::npos) {
                sec_str = sec_str.substr(0, end + 1);
            }
        }

        year = year_str.empty() ? 0 : std::stoi(year_str);
        month = month_str.empty() ? 0 : std::stoi(month_str);
        day = day_str.empty() ? 0 : std::stoi(day_str);
        hour = hour_str.empty() ? 0 : std::stoi(hour_str);
        minute = min_str.empty() ? 0 : std::stoi(min_str);
        second = sec_str.empty() ? 0.0 : std::stod(sec_str);

        // Convert 2-digit year to 4-digit (80-99 = 1980-1999, 00-79 = 2000-2079)
        if (year < 80) {
            year += 2000;
        } else {
            year += 1900;
        }

    } catch (const std::exception& e) {
        std::cerr << "Observation epoch parsing error: " << e.what() << std::endl;
        std::cerr << "Line: " << line << std::endl;
        std::cerr << "Line length: " << line.length() << std::endl;
        throw;
    }

    // Convert calendar date to GPS week and TOW
    // GPS time started on January 6, 1980 (GPS week 0, day 0)
    // Calculate days since GPS epoch
    int days_since_gps_epoch = 0;

    // Count days from 1980 to the given year
    for (int y = 1980; y < year; y++) {
        bool is_leap = (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
        days_since_gps_epoch += is_leap ? 366 : 365;
    }

    // Count days in current year up to current month
    int days_in_month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    bool is_leap = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
    if (is_leap) days_in_month[1] = 29;

    for (int m = 1; m < month; m++) {
        days_since_gps_epoch += days_in_month[m - 1];
    }

    days_since_gps_epoch += day;

    // Subtract the 6 days from Jan 1 to Jan 6, 1980
    days_since_gps_epoch -= 6;

    // Calculate GPS week and day of week
    int gps_week = days_since_gps_epoch / 7;
    int day_of_week = days_since_gps_epoch % 7;

    // Calculate time of week in seconds
    double tow = day_of_week * 86400.0 + hour * 3600.0 + minute * 60.0 + second;

    obs_data.time = GNSSTime(gps_week, tow);

    // Parse number of satellites (pos 30-32, 0-indexed: 29-31)
    std::string num_sats_str = line.substr(29, 3);
    num_sats_str.erase(0, num_sats_str.find_first_not_of(' '));
    int num_sats = num_sats_str.empty() ? 0 : std::stoi(num_sats_str);

    // Parse satellite PRNs from epoch line (positions 33+)
    std::vector<SatelliteId> satellites;
    size_t prn_pos = 32;  // Start after satellite count
    std::string current_line = line;  // Local copy for continued parsing

    for (int i = 0; i < num_sats; ++i) {
        // Each PRN takes 3 characters: system code (1 char) + PRN (2 chars)
        if (prn_pos + 2 < current_line.length()) {
            char sys_char = current_line[prn_pos];
            std::string prn_str = current_line.substr(prn_pos + 1, 2);
            prn_str.erase(0, prn_str.find_first_not_of(' '));

            if (!prn_str.empty()) {
                int prn = std::stoi(prn_str);
                GNSSSystem system = GNSSSystem::GPS;  // Default to GPS
                if (sys_char == 'G') system = GNSSSystem::GPS;
                else if (sys_char == 'R') system = GNSSSystem::GLONASS;
                else if (sys_char == 'E') system = GNSSSystem::Galileo;
                else if (sys_char == 'C') system = GNSSSystem::BeiDou;

                satellites.push_back(SatelliteId(system, prn));
            }
        }
        prn_pos += 3;

        // If we exceed current line, satellites continue on next line
        if (prn_pos >= current_line.length() && (i + 1) % 12 == 0 && (i + 1) < num_sats) {
            if (!readLine(current_line)) break;
            prn_pos = 32;
        }
    }

    // Read observation data for each satellite
    int num_obs_types = header_.observation_types.size();
    if (num_obs_types == 0) num_obs_types = 4;  // Default: L1, C1, L2, P2

    for (size_t sat_idx = 0; sat_idx < satellites.size(); ++sat_idx) {
        SatelliteId sat = satellites[sat_idx];

        // Calculate number of lines needed for this satellite (5 obs per line)
        int lines_per_sat = (num_obs_types + 4) / 5;

        std::vector<double> obs_values(num_obs_types, 0.0);
        std::vector<int> lli_flags(num_obs_types, 0);
        std::vector<int> signal_strength(num_obs_types, 0);

        // Read observation lines for this satellite
        for (int line_idx = 0; line_idx < lines_per_sat; ++line_idx) {
            std::string obs_line;
            if (!readLine(obs_line)) break;

            // Each observation occupies 16 characters
            for (int obs_in_line = 0; obs_in_line < 5; ++obs_in_line) {
                int obs_idx = line_idx * 5 + obs_in_line;
                if (obs_idx >= num_obs_types) break;

                size_t col_start = obs_in_line * 16;
                if (col_start + 14 > obs_line.length()) continue;

                // Parse observation value (14 characters, right-justified)
                std::string obs_str = obs_line.substr(col_start, 14);
                obs_str.erase(0, obs_str.find_first_not_of(' '));
                obs_str.erase(obs_str.find_last_not_of(' ') + 1);

                if (!obs_str.empty() && obs_str != "0.000" && obs_str != "0.0") {
                    try {
                        obs_values[obs_idx] = std::stod(obs_str);
                    } catch (...) {
                        obs_values[obs_idx] = 0.0;
                    }
                }

                // Parse LLI flag (position 14-15)
                if (col_start + 14 < obs_line.length() && obs_line[col_start + 14] != ' ') {
                    lli_flags[obs_idx] = obs_line[col_start + 14] - '0';
                }

                // Parse signal strength (position 15-16)
                if (col_start + 15 < obs_line.length() && obs_line[col_start + 15] != ' ') {
                    signal_strength[obs_idx] = obs_line[col_start + 15] - '0';
                }
            }
        }

        Observation obs_primary;
        obs_primary.satellite = sat;
        obs_primary.signal = primarySignalForSystem(sat.system);
        obs_primary.valid = true;
        bool has_primary_data = false;
        int primary_priority = kInvalidObservationPriority;
        int primary_band = -1;
        std::string primary_key;

        Observation obs_secondary;
        obs_secondary.satellite = sat;
        obs_secondary.signal = secondarySignalForSystem(sat.system);
        obs_secondary.valid = true;
        bool has_secondary_data = false;
        int secondary_priority = kInvalidObservationPriority;
        int secondary_band = -1;
        std::string secondary_key;

        for (size_t i = 0; i < header_.observation_types.size() && i < obs_values.size(); ++i) {
            if (obs_values[i] == 0.0) continue;

            const std::string& obs_type = header_.observation_types[i];
            if (!isCodeObservationType(obs_type)) {
                continue;
            }

            const int band = rinexBand(obs_type);
            const int primary_candidate = observationSelectionPriority(sat.system, obs_type, true);
            const int secondary_candidate = observationSelectionPriority(sat.system, obs_type, false);
            if (primary_candidate < primary_priority) {
                obs_primary.signal = signalForObservationType(sat.system, obs_type, true);
                primary_priority = primary_candidate;
                primary_band = band;
                primary_key = observationSelectionKey(obs_type);
            }
            if (secondary_candidate < secondary_priority) {
                obs_secondary.signal = signalForObservationType(sat.system, obs_type, false);
                secondary_priority = secondary_candidate;
                secondary_band = band;
                secondary_key = observationSelectionKey(obs_type);
            }
        }

        for (size_t i = 0; i < header_.observation_types.size() && i < obs_values.size(); ++i) {
            if (obs_values[i] == 0.0) continue;

            const std::string& obs_type = header_.observation_types[i];
            if (primary_priority < kInvalidObservationPriority &&
                matchesObservationSelection(obs_type, primary_band, primary_key)) {
                assignObservationField(obs_primary, obs_type, obs_values[i], lli_flags[i], signal_strength[i]);
                has_primary_data = true;
            } else if (secondary_priority < kInvalidObservationPriority &&
                       matchesObservationSelection(obs_type, secondary_band, secondary_key)) {
                assignObservationField(obs_secondary, obs_type, obs_values[i], lli_flags[i], signal_strength[i]);
                has_secondary_data = true;
            }
        }

        // Add observations if they have data
        if (has_primary_data) {
            obs_data.addObservation(obs_primary);
        }
        if (has_secondary_data) {
            obs_data.addObservation(obs_secondary);
        }

        if (emit_extra_band_observations_) {
            emitExtraBandObservations(
                sat.system, sat, header_.observation_types,
                obs_values, lli_flags, signal_strength,
                primary_band, secondary_band, obs_data);
        }
    }

    return true;
}

bool RINEXReader::parseObservationEpochV3(const std::string& epoch_line, ObservationData& obs_data) {
    // RINEX 3.x epoch line format:
    // "> YYYY MM DD HH MM SS.SSSSSSS  flag  num_sats"
    // Pos: 0=>, 2-5=year, 6-9=month, etc.
    if (epoch_line.length() < 35 || epoch_line[0] != '>') return false;

    try {
        int year = std::stoi(epoch_line.substr(2, 4));
        int month = std::stoi(epoch_line.substr(7, 2));
        int day = std::stoi(epoch_line.substr(10, 2));
        int hour = std::stoi(epoch_line.substr(13, 2));
        int minute = std::stoi(epoch_line.substr(16, 2));
        double second = std::stod(epoch_line.substr(18, 13));

        // Convert to GPS time
        auto days_from_civil = [](int y, unsigned m, unsigned d) -> int {
            y -= m <= 2;
            const int era = (y >= 0 ? y : y - 399) / 400;
            const unsigned yoe = static_cast<unsigned>(y - era * 400);
            const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
            const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
            return era * 146097 + static_cast<int>(doe) - 719468;
        };

        const int gps_epoch_days = days_from_civil(1980, 1, 6);
        const int current_days = days_from_civil(year, static_cast<unsigned>(month), static_cast<unsigned>(day));
        const int days_since_gps = current_days - gps_epoch_days;
        const int gps_week = days_since_gps / 7;
        const int day_of_week = days_since_gps % 7;
        const double tow = day_of_week * 86400.0 + hour * 3600.0 + minute * 60.0 + second;

        obs_data.time = GNSSTime(gps_week, tow);

        // Parse epoch flag and number of satellites
        std::string flag_str = epoch_line.substr(31, 2);
        flag_str.erase(0, flag_str.find_first_not_of(' '));
        // int epoch_flag = flag_str.empty() ? 0 : std::stoi(flag_str);

        std::string num_sats_str = epoch_line.substr(33, 3);
        num_sats_str.erase(0, num_sats_str.find_first_not_of(' '));
        int num_sats = num_sats_str.empty() ? 0 : std::stoi(num_sats_str);

        // Read each satellite line
        for (int s = 0; s < num_sats; ++s) {
            std::string sat_line;
            if (!readLine(sat_line)) break;
            if (sat_line.length() < 3) continue;

            // Parse system char and PRN
            char sys_char = sat_line[0];
            std::string prn_str = sat_line.substr(1, 2);
            prn_str.erase(0, prn_str.find_first_not_of(' '));
            if (prn_str.empty()) continue;
            int prn = std::stoi(prn_str);

            const GNSSSystem system = systemFromRinexChar(sys_char);
            if (system == GNSSSystem::UNKNOWN) continue;

            auto sys_it = header_.system_obs_types.find(sys_char);
            const std::vector<std::string>& obs_types =
                (sys_it != header_.system_obs_types.end()) ? sys_it->second : header_.observation_types;
            int num_obs_types = static_cast<int>(obs_types.size());
            if (num_obs_types == 0) num_obs_types = 4;

            SatelliteId sat(system, prn);

            // Parse observation values
            // Each observation occupies 16 characters starting at position 3
            // Format: 14.3f + LLI(1) + SS(1) = 16 chars per observation
            std::vector<double> obs_values(num_obs_types, 0.0);
            std::vector<int> lli_flags(num_obs_types, 0);
            std::vector<int> signal_strength(num_obs_types, 0);

            for (int i = 0; i < num_obs_types; ++i) {
                size_t col_start = 3 + i * 16;
                if (col_start + 14 > sat_line.length()) continue;

                std::string obs_str = sat_line.substr(col_start, 14);
                obs_str.erase(0, obs_str.find_first_not_of(' '));
                obs_str.erase(obs_str.find_last_not_of(' ') + 1);

                if (!obs_str.empty()) {
                    try {
                        obs_values[i] = std::stod(obs_str);
                    } catch (...) {
                        obs_values[i] = 0.0;
                    }
                }

                // Parse LLI flag (position col_start + 14)
                if (col_start + 14 < sat_line.length() && sat_line[col_start + 14] != ' ') {
                    lli_flags[i] = sat_line[col_start + 14] - '0';
                }

                // Parse signal strength (position col_start + 15)
                if (col_start + 15 < sat_line.length() && sat_line[col_start + 15] != ' ') {
                    signal_strength[i] = sat_line[col_start + 15] - '0';
                }
            }

            Observation obs_primary;
            obs_primary.satellite = sat;
            obs_primary.signal = primarySignalForSystem(system);
            obs_primary.valid = true;
            bool has_primary_data = false;
            int primary_priority = kInvalidObservationPriority;
            int primary_band = -1;
            std::string primary_key;

            Observation obs_secondary;
            obs_secondary.satellite = sat;
            obs_secondary.signal = secondarySignalForSystem(system);
            obs_secondary.valid = true;
            bool has_secondary_data = false;
            int secondary_priority = kInvalidObservationPriority;
            int secondary_band = -1;
            std::string secondary_key;

            for (size_t i = 0; i < obs_types.size() && i < obs_values.size(); ++i) {
                if (obs_values[i] == 0.0) continue;

                const std::string& obs_type = obs_types[i];
                if (!isCodeObservationType(obs_type)) {
                    continue;
                }

                const int band = rinexBand(obs_type);
                const int primary_candidate = observationSelectionPriority(system, obs_type, true);
                const int secondary_candidate = observationSelectionPriority(system, obs_type, false);
                if (primary_candidate < primary_priority) {
                    obs_primary.signal = signalForObservationType(system, obs_type, true);
                    primary_priority = primary_candidate;
                    primary_band = band;
                    primary_key = observationSelectionKey(obs_type);
                }
                if (secondary_candidate < secondary_priority) {
                    obs_secondary.signal = signalForObservationType(system, obs_type, false);
                    secondary_priority = secondary_candidate;
                    secondary_band = band;
                    secondary_key = observationSelectionKey(obs_type);
                }
            }

            for (size_t i = 0; i < obs_types.size() && i < obs_values.size(); ++i) {
                if (obs_values[i] == 0.0) continue;

                const std::string& obs_type = obs_types[i];
                if (primary_priority < kInvalidObservationPriority &&
                    matchesObservationSelection(obs_type, primary_band, primary_key)) {
                    assignObservationField(obs_primary, obs_type, obs_values[i], lli_flags[i], signal_strength[i]);
                    has_primary_data = true;
                } else if (secondary_priority < kInvalidObservationPriority &&
                           matchesObservationSelection(obs_type, secondary_band, secondary_key)) {
                    assignObservationField(obs_secondary, obs_type, obs_values[i], lli_flags[i], signal_strength[i]);
                    has_secondary_data = true;
                }
            }

            if (has_primary_data) {
                obs_data.addObservation(obs_primary);
            }
            if (has_secondary_data) {
                obs_data.addObservation(obs_secondary);
            }

            if (emit_extra_band_observations_) {
                emitExtraBandObservations(
                    system, sat, obs_types,
                    obs_values, lli_flags, signal_strength,
                    primary_band, secondary_band, obs_data);
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "RINEX 3 observation epoch parsing error: " << e.what() << std::endl;
        return false;
    }

    return true;
}

bool RINEXReader::parseNavigationMessage(const std::vector<std::string>& lines, Ephemeris& eph) {
    if (lines.empty()) {
        return false;
    }
    const std::string& first_line = lines[0];
    if (first_line.length() < 3) return false;

    // Determine if this is RINEX 3 format (first char is a letter)
    bool is_v3 = std::isalpha(first_line[0]);

    // Column offsets differ between RINEX 2 and 3
    // RINEX 2: PRN(2) + time(19) starting at col 3, data at cols 3,22,41,60
    // RINEX 3: PRN(3) + time(20) starting at col 3, data at cols 4,23,42,61 for continuation
    //          First line: af0 at 23, af1 at 42, af2 at 61
    int c0, c1, c2, c3;  // Column starts for 4 values per continuation line
    int af0_col, af1_col, af2_col;  // Column starts for first line clock params
    if (is_v3) {
        c0 = 4; c1 = 23; c2 = 42; c3 = 61;
        af0_col = 23; af1_col = 42; af2_col = 61;
    } else {
        c0 = 3; c1 = 22; c2 = 41; c3 = 60;
        af0_col = 22; af1_col = 41; af2_col = 60;
    }

    try {
        // Parse satellite ID
        if (is_v3) {
            // RINEX 3: "G27", "E04", etc.
            char sys_char = first_line[0];
            if (!supportsBroadcastNavigationSystem(sys_char)) return false;
            std::string prn_str = first_line.substr(1, 2);
            prn_str.erase(0, prn_str.find_first_not_of(' '));
            if (prn_str.empty()) return false;
            eph.satellite = SatelliteId(systemFromRinexChar(sys_char), std::stoi(prn_str));
        } else {
            // RINEX 2: " 27" or "27"
            std::string sat_id_str = first_line.substr(0, 2);
            sat_id_str.erase(0, sat_id_str.find_first_not_of(' '));
            if (sat_id_str.empty()) return false;
            eph.satellite = SatelliteId(GNSSSystem::GPS, std::stoi(sat_id_str));
        }

        // Helper lambda to parse D-formatted scientific notation
        auto parseD = [](const std::string& line, size_t start, size_t len) -> double {
            if (start + len > line.length()) return 0.0;
            std::string val = line.substr(start, len);
            val.erase(0, val.find_first_not_of(' '));
            if (val.empty()) return 0.0;
            // Replace D with E for C++ parsing
            size_t d_pos = val.find('D');
            if (d_pos != std::string::npos) val[d_pos] = 'E';
            d_pos = val.find('d');
            if (d_pos != std::string::npos) val[d_pos] = 'E';
            try {
                return std::stod(val);
            } catch (...) {
                return 0.0;
            }
        };

        if (is_v3 && first_line[0] == 'R') {
            if (lines.size() < 4) {
                return false;
            }

            const std::string time_field = first_line.substr(3, 20);
            int year = 0;
            int month = 0;
            int day = 0;
            int hour = 0;
            int minute = 0;
            double second = 0.0;
            parseCalendarFields(time_field, year, month, day, hour, minute, second);

            const GNSSTime toc_utc_raw = parseTime(time_field, header_.version);
            const double rounded_tow = std::floor((toc_utc_raw.tow + 450.0) / 900.0) * 900.0;
            const GNSSTime toc_utc = normalizeWeekTow(toc_utc_raw.week, rounded_tow);
            const int day_of_week = static_cast<int>(std::floor(toc_utc_raw.tow / 86400.0));
            const double tod_utc = std::fmod(parseD(first_line, af2_col, 19), 86400.0);
            const GNSSTime tof_utc = adjustDay(
                normalizeWeekTow(toc_utc.week, tod_utc + day_of_week * 86400.0),
                toc_utc);

            eph.toe = utcToGpst(toc_utc, year, month, day);
            eph.toc = eph.toe;
            eph.tof = utcToGpst(tof_utc, year, month, day);
            eph.week = static_cast<uint16_t>(eph.toe.week);
            eph.iode = static_cast<uint16_t>(std::fmod(toc_utc_raw.tow + 10800.0, 86400.0) / 900.0 + 0.5);
            eph.glonass_taun = -parseD(first_line, af0_col, 19);
            eph.glonass_gamn = parseD(first_line, af1_col, 19);

            eph.glonass_position = Vector3d(
                parseD(lines[1], c0, 19) * 1e3,
                parseD(lines[2], c0, 19) * 1e3,
                parseD(lines[3], c0, 19) * 1e3);
            eph.glonass_velocity = Vector3d(
                parseD(lines[1], c1, 19) * 1e3,
                parseD(lines[2], c1, 19) * 1e3,
                parseD(lines[3], c1, 19) * 1e3);
            eph.glonass_acceleration = Vector3d(
                parseD(lines[1], c2, 19) * 1e3,
                parseD(lines[2], c2, 19) * 1e3,
                parseD(lines[3], c2, 19) * 1e3);
            eph.health = static_cast<uint8_t>(std::max(0.0, parseD(lines[1], c3, 19)));
            eph.glonass_frequency_channel = static_cast<int>(parseD(lines[2], c3, 19));
            if (eph.glonass_frequency_channel > 128) {
                eph.glonass_frequency_channel -= 256;
            }
            eph.glonass_age = static_cast<int>(parseD(lines[3], c3, 19));
            eph.valid = true;
            return true;
        }

        if (lines.size() < 8) {
            return false;  // Need 8 lines for Kepler broadcast ephemeris
        }

        // Line 0: PRN, Epoch, af0, af1, af2
        if (is_v3) {
            // RINEX 3: time string at pos 3-22 (20 chars: " YYYY MM DD HH MM SS")
            eph.toc = parseTime(first_line.substr(3, 20), header_.version);
        } else {
            eph.toc = parseTime(first_line.substr(3, 19), header_.version);
        }
        if (eph.satellite.system == GNSSSystem::BeiDou) {
            eph.toc = bdtToGpst(eph.toc);
        }
        eph.af0 = parseD(first_line, af0_col, 19);  // Clock bias
        eph.af1 = parseD(first_line, af1_col, 19);  // Clock drift
        eph.af2 = parseD(first_line, af2_col, 19);  // Clock drift rate

        // Line 1: IODE, Crs, delta_n, M0
        double iode = parseD(lines[1], c0, 19);
        eph.crs = parseD(lines[1], c1, 19);
        eph.delta_n = parseD(lines[1], c2, 19);
        eph.m0 = parseD(lines[1], c3, 19);

        // Line 2: Cuc, e, Cus, sqrt(A)
        eph.cuc = parseD(lines[2], c0, 19);
        eph.e = parseD(lines[2], c1, 19);
        eph.cus = parseD(lines[2], c2, 19);
        eph.sqrt_a = parseD(lines[2], c3, 19);

        // Line 3: Toe, Cic, OMEGA0, Cis
        double toe_seconds = parseD(lines[3], c0, 19);
        eph.toes = toe_seconds;

        eph.cic = parseD(lines[3], c1, 19);
        eph.omega0 = parseD(lines[3], c2, 19);
        eph.cis = parseD(lines[3], c3, 19);

        // Line 4: i0, Crc, omega, OMEGA_DOT
        eph.i0 = parseD(lines[4], c0, 19);
        eph.crc = parseD(lines[4], c1, 19);
        eph.omega = parseD(lines[4], c2, 19);
        eph.omega_dot = parseD(lines[4], c3, 19);

        // Line 5: IDOT, system-specific codes, week, system-specific flag
        eph.i_dot = parseD(lines[5], c0, 19);
        eph.idot = eph.i_dot;
        double system_codes = parseD(lines[5], c1, 19);
        double system_week = parseD(lines[5], c2, 19);
        double system_flag = parseD(lines[5], c3, 19);

        // Line 6: SV accuracy, SV health, TGD/BGD1, system-specific value
        double sv_accuracy = parseD(lines[6], c0, 19);
        double sv_health = parseD(lines[6], c1, 19);
        double delay_1 = parseD(lines[6], c2, 19);
        double delay_2_or_iodc = parseD(lines[6], c3, 19);

        // Line 7: Transmission time, fit interval/AODC
        double transmission_time = parseD(lines[7], c0, 19);
        double fit_interval_or_aodc = parseD(lines[7], c1, 19);

        // Suppress unused variable warnings
        (void)system_codes;
        (void)system_flag;
        (void)transmission_time;

        // Set times
        int week = static_cast<int>(system_week);

        eph.sv_accuracy = sv_accuracy;
        eph.sv_health = sv_health;
        eph.health = static_cast<uint8_t>(std::max(0.0, sv_health));
        eph.tgd = delay_1;
        eph.tgd_secondary = 0.0;
        eph.valid = true;
        eph.iode = static_cast<int>(iode);

        switch (eph.satellite.system) {
            case GNSSSystem::BeiDou:
                eph.toe = bdtWeekTowToGpst(week, toe_seconds);
                eph.week = static_cast<uint16_t>(week);
                eph.tgd_secondary = delay_2_or_iodc;
                eph.iodc = static_cast<int>(fit_interval_or_aodc);
                break;
            case GNSSSystem::Galileo:
                eph.toe = GNSSTime(week, toe_seconds);
                eph.week = static_cast<uint16_t>(week);
                eph.tgd_secondary = delay_2_or_iodc;
                eph.iodc = static_cast<int>(iode);
                break;
            case GNSSSystem::QZSS:
            case GNSSSystem::GPS:
            default:
                eph.toe = GNSSTime(week, toe_seconds);
                eph.week = static_cast<uint16_t>(week);
                eph.iodc = static_cast<int>(delay_2_or_iodc);
                break;
        }

    } catch (const std::exception& e) {
        std::cerr << "Error parsing ephemeris: " << e.what() << std::endl;
        return false;
    }

    return true;
}

GNSSTime RINEXReader::parseTime(const std::string& time_str, double version) {
    (void)version;

    std::istringstream iss(time_str);
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    double second = 0.0;
    iss >> year >> month >> day >> hour >> minute >> second;

    if (year < 80) {
        year += 2000;
    } else if (year < 100) {
        year += 1900;
    }

    auto days_from_civil = [](int y, unsigned m, unsigned d) -> int {
        y -= m <= 2;
        const int era = (y >= 0 ? y : y - 399) / 400;
        const unsigned yoe = static_cast<unsigned>(y - era * 400);
        const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
        const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
        return era * 146097 + static_cast<int>(doe) - 719468;
    };

    const int gps_epoch_days = days_from_civil(1980, 1, 6);
    const int current_days = days_from_civil(year, static_cast<unsigned>(month), static_cast<unsigned>(day));
    const int days_since_gps = current_days - gps_epoch_days;
    const int week = days_since_gps / 7;
    const int day_of_week = days_since_gps % 7;
    const double tow = day_of_week * 86400.0 + hour * 3600.0 + minute * 60.0 + second;

    return GNSSTime(week, tow);
}

SatelliteId RINEXReader::parseSatelliteId(const std::string& sat_str, double version) {
    (void)version;
    if (sat_str.empty()) {
        return SatelliteId(GNSSSystem::GPS, 1);
    }

    size_t prn_start = 0;
    GNSSSystem system = GNSSSystem::GPS;
    if (std::isalpha(sat_str[0])) {
        system = systemFromRinexChar(sat_str[0]);
        prn_start = 1;
    }

    if (sat_str.length() >= prn_start + 2) {
        std::string prn_str = sat_str.substr(prn_start, 2);
        prn_str.erase(0, prn_str.find_first_not_of(' '));
        if (!prn_str.empty()) {
            return SatelliteId(system, std::stoi(prn_str));
        }
    }
    return SatelliteId(system, 1);
}

bool RINEXReader::readLine(std::string& line) {
    if (std::getline(file_, line)) {
        current_line_++;
        return true;
    }
    return false;
}

// RINEXWriter implementation
bool RINEXWriter::createObservationFile(const std::string& filename, const RINEXReader::RINEXHeader& header) {
    file_.open(filename);
    if (!file_.is_open()) {
        return false;
    }
    
    header_ = header;
    return writeHeader(header);
}

bool RINEXWriter::writeHeader(const RINEXReader::RINEXHeader& header) {
    if (!file_.is_open()) {
        return false;
    }
    
    // Write RINEX header
    file_ << std::fixed << std::setprecision(2) << header.version;
    if (header.file_type == RINEXReader::FileType::NAVIGATION) {
        file_ << "           NAVIGATION DATA     ";
    } else {
        file_ << "           OBSERVATION DATA    ";
    }
    file_ << header.satellite_system << "                   RINEX VERSION / TYPE\n";
    
    file_ << "LibGNSS++           User                ";
    file_ << "20240101 000000 UTC PGM / RUN BY / DATE\n";
    
    file_ << "                                                            END OF HEADER\n";
    
    return true;
}

bool RINEXWriter::writeObservationEpoch(const ObservationData& obs_data) {
    if (!file_.is_open()) {
        return false;
    }
    
    // Simplified epoch writing
    file_ << "> " << obs_data.time.week << " " << obs_data.time.tow;
    file_ << "  0  " << obs_data.observations.size() << "\n";
    
    for (const auto& obs : obs_data.observations) {
        file_ << obs.satellite.toString() << " ";
        file_ << std::fixed << std::setprecision(3) << obs.pseudorange << "\n";
    }
    
    return true;
}

bool RINEXWriter::createNavigationFile(const std::string& filename, const RINEXReader::RINEXHeader& header) {
    file_.open(filename);
    if (!file_.is_open()) {
        return false;
    }

    header_ = header;
    return writeHeader(header);
}

std::string RINEXWriter::formatTime(const GNSSTime& time, double version) {
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    double second = 0.0;
    gpstToCalendar(time, year, month, day, hour, minute, second);

    std::ostringstream oss;
    if (version >= 3.0) {
        oss << ' '
            << std::setw(4) << year
            << std::setw(3) << month
            << std::setw(3) << day
            << std::setw(3) << hour
            << std::setw(3) << minute
            << std::setw(3) << static_cast<int>(std::floor(second + 0.5));
    } else {
        oss << ' '
            << std::setw(2) << (year % 100)
            << std::setw(3) << month
            << std::setw(3) << day
            << std::setw(3) << hour
            << std::setw(3) << minute
            << std::setw(5) << std::fixed << std::setprecision(1) << second;
    }
    return oss.str();
}

std::string RINEXWriter::formatSatelliteId(const SatelliteId& sat, double version) {
    std::ostringstream oss;
    if (version >= 3.0) {
        oss << rinexCharForSystem(sat.system)
            << std::setw(2) << std::setfill('0') << static_cast<int>(sat.prn);
        return oss.str();
    }
    oss << std::setw(2) << std::setfill(' ') << static_cast<int>(sat.prn);
    return oss.str();
}

bool RINEXWriter::writeNavigationMessage(const Ephemeris& eph) {
    if (!file_.is_open()) {
        return false;
    }

    if (eph.satellite.system == GNSSSystem::GLONASS) {
        int year = 0;
        int month = 0;
        int day = 0;
        int hour = 0;
        int minute = 0;
        double second = 0.0;
        gpstToUtcCalendar(eph.toc, year, month, day, hour, minute, second);

        const double tof_seconds = std::fmod((eph.tof - static_cast<double>(
            leapSecondsForDate(year, month, day))).tow, 86400.0);

        file_ << formatSatelliteId(eph.satellite, header_.version)
              << ' '
              << std::setw(4) << year
              << std::setw(3) << month
              << std::setw(3) << day
              << std::setw(3) << hour
              << std::setw(3) << minute
              << std::setw(3) << static_cast<int>(std::floor(second + 0.5))
              << formatRinexFloat(-eph.glonass_taun)
              << formatRinexFloat(eph.glonass_gamn)
              << formatRinexFloat(tof_seconds)
              << "\n";

        file_ << "    "
              << formatRinexFloat(eph.glonass_position.x() * 1e-3)
              << formatRinexFloat(eph.glonass_velocity.x() * 1e-3)
              << formatRinexFloat(eph.glonass_acceleration.x() * 1e-3)
              << formatRinexFloat(eph.health)
              << "\n";

        file_ << "    "
              << formatRinexFloat(eph.glonass_position.y() * 1e-3)
              << formatRinexFloat(eph.glonass_velocity.y() * 1e-3)
              << formatRinexFloat(eph.glonass_acceleration.y() * 1e-3)
              << formatRinexFloat(eph.glonass_frequency_channel)
              << "\n";

        file_ << "    "
              << formatRinexFloat(eph.glonass_position.z() * 1e-3)
              << formatRinexFloat(eph.glonass_velocity.z() * 1e-3)
              << formatRinexFloat(eph.glonass_acceleration.z() * 1e-3)
              << formatRinexFloat(eph.glonass_age)
              << "\n";
        return true;
    }

    const bool is_beidou = eph.satellite.system == GNSSSystem::BeiDou;
    const bool is_galileo = eph.satellite.system == GNSSSystem::Galileo;
    const GNSSTime toc_time = is_beidou ? gpstToBdt(eph.toc) : eph.toc;
    const double week_field = static_cast<double>(eph.week);
    const double line6_col4 =
        (is_beidou || is_galileo) ? eph.tgd_secondary : static_cast<double>(eph.iodc);
    const double line7_col1 = is_beidou ? gpstToBdt(eph.tof).tow : eph.tof.tow;
    const double line7_col2 = is_beidou ? static_cast<double>(eph.iodc) : 0.0;

    file_ << formatSatelliteId(eph.satellite, header_.version)
          << formatTime(toc_time, header_.version)
          << formatRinexFloat(eph.af0)
          << formatRinexFloat(eph.af1)
          << formatRinexFloat(eph.af2)
          << "\n";

    file_ << "    "
          << formatRinexFloat(eph.iode)
          << formatRinexFloat(eph.crs)
          << formatRinexFloat(eph.delta_n)
          << formatRinexFloat(eph.m0)
          << "\n";

    file_ << "    "
          << formatRinexFloat(eph.cuc)
          << formatRinexFloat(eph.e)
          << formatRinexFloat(eph.cus)
          << formatRinexFloat(eph.sqrt_a)
          << "\n";

    file_ << "    "
          << formatRinexFloat(eph.toes)
          << formatRinexFloat(eph.cic)
          << formatRinexFloat(eph.omega0)
          << formatRinexFloat(eph.cis)
          << "\n";

    file_ << "    "
          << formatRinexFloat(eph.i0)
          << formatRinexFloat(eph.crc)
          << formatRinexFloat(eph.omega)
          << formatRinexFloat(eph.omega_dot)
          << "\n";

    file_ << "    "
          << formatRinexFloat(eph.idot)
          << formatRinexFloat(0.0)
          << formatRinexFloat(week_field)
          << formatRinexFloat(0.0)
          << "\n";

    file_ << "    "
          << formatRinexFloat(eph.sv_accuracy)
          << formatRinexFloat(eph.sv_health)
          << formatRinexFloat(eph.tgd)
          << formatRinexFloat(line6_col4)
          << "\n";

    file_ << "    "
          << formatRinexFloat(line7_col1)
          << formatRinexFloat(line7_col2)
          << formatRinexFloat(0.0)
          << formatRinexFloat(0.0)
          << "\n";

    return true;
}

void RINEXWriter::close() {
    if (file_.is_open()) {
        file_.close();
    }
}

} // namespace io
} // namespace libgnss
