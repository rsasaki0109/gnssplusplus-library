#include <libgnss++/io/rinex.hpp>
#include <libgnss++/io/rinex4.hpp>
#include <libgnss++/algorithms/ppp_env_overrides.hpp>
#include <libgnss++/core/signal_policy.hpp>
#include <libgnss++/algorithms/source_tracking_selection.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <system_error>
#include <tuple>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <cctype>
#include <limits>
#include <set>
#include <utility>

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

// ---------------------------------------------------------------------------
// RINEX 2.x observation record helpers
// ---------------------------------------------------------------------------

bool isDigitOrSpace(char c) {
    return c == ' ' || std::isdigit(static_cast<unsigned char>(c)) != 0;
}

bool hasDigit(const std::string& text, size_t pos, size_t len) {
    for (size_t i = pos; i < pos + len && i < text.size(); ++i) {
        if (std::isdigit(static_cast<unsigned char>(text[i])) != 0) {
            return true;
        }
    }
    return false;
}

// RINEX 2.x epoch record: (1X,I2.2,4(1X,I2),F11.7,2X,I1,I3,12(A1,I2)).
// Used to resynchronise on the next epoch, so it checks the fixed blank
// separators and that the date fields / flag / satellite count are numeric.
// Observation rows, satellite-list continuation rows and blank rows all fail.
bool looksLikeRinex2EpochLine(const std::string& line) {
    if (line.size() < 29) {
        return false;
    }
    for (const size_t pos : {3U, 6U, 9U, 12U}) {
        if (line[pos] != ' ') {
            return false;
        }
    }
    for (const size_t pos : {1U, 2U, 4U, 5U, 7U, 8U, 10U, 11U, 13U, 14U}) {
        if (!isDigitOrSpace(line[pos])) {
            return false;
        }
    }
    // Year, month and day must carry digits; hour/minute may be blank-padded.
    if (!hasDigit(line, 1, 2) || !hasDigit(line, 4, 2) || !hasDigit(line, 7, 2)) {
        return false;
    }
    if (!isDigitOrSpace(line[28])) {
        return false;
    }
    for (size_t pos = 29; pos < 32 && pos < line.size(); ++pos) {
        if (!isDigitOrSpace(line[pos])) {
            return false;
        }
    }
    return true;
}

// Map a RINEX 2.11/2.12 satellite system letter to a system.  Blank means
// GPS (original 2.x convention); letters that RINEX 2.x does not define are
// UNKNOWN so the caller can skip the satellite without ever mislabelling it
// as GPS.
GNSSSystem rinex2SystemFromChar(char sys_char) {
    switch (sys_char) {
        case ' ':
        case 'G': return GNSSSystem::GPS;
        case 'R': return GNSSSystem::GLONASS;
        case 'E': return GNSSSystem::Galileo;
        case 'S': return GNSSSystem::SBAS;
        case 'J': return GNSSSystem::QZSS;
        case 'C': return GNSSSystem::BeiDou;
        default: return GNSSSystem::UNKNOWN;
    }
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

void annotateGlonassFrequencyChannel(Observation& observation,
                                     const std::map<SatelliteId, int>& channels) {
    if (observation.satellite.system != GNSSSystem::GLONASS) {
        return;
    }
    const auto it = channels.find(observation.satellite);
    if (it == channels.end()) {
        return;
    }
    observation.has_glonass_frequency_channel = true;
    observation.glonass_frequency_channel = it->second;
}

// RINEX loss-of-lock indicator: only '0'..'7' are defined.  Anything else
// (blank, a stray '\r', garbage) means "no indicator" instead of a value
// derived from `c - '0'` that could set the cycle-slip bit.
inline int rinexLliFromChar(char c) {
    return (c >= '0' && c <= '7') ? c - '0' : 0;
}

// RINEX signal-strength indicator: only '1'..'9' are defined; '0' / blank
// mean "not set" (0).
inline int rinexSsiFromChar(char c) {
    return (c >= '1' && c <= '9') ? c - '0' : 0;
}

void assignObservationField(Observation& obs,
                            const std::string& obs_type,
                            double value,
                            int lli,
                            int signal_strength);

char rinexTrackingCode(const std::string& obs_type) {
    return obs_type.size() >= 3 ? obs_type[2] : '\0';
}

bool sameTrackingCode(char selected, char candidate) {
    return selected == '\0' || candidate == '\0' || selected == candidate;
}

int qzssL5TrackingPriority(char tracking_code) {
    switch (tracking_code) {
        case 'Q': return 0;
        case 'X': return 1;
        case 'I': return 2;
        default: return 100;
    }
}

struct ObservationSelection {
    Observation observation;
    bool has_data = false;
    int priority = 100;
    int band = -1;
    char tracking_code = '\0';
    int tracking_rank = signal_policy::kUnlistedTrackingRank;
};

// True when `candidate` should replace `current` among two different tracking
// codes of one band.  Fixed attribute priority (signal_policy::
// trackingAttributePriority), then the attribute letter itself, so the result
// never depends on the order the observation types are declared in.
bool trackingCodeBeats(int candidate_rank, char candidate_code,
                       int current_rank, char current_code) {
    if (candidate_rank != current_rank) {
        return candidate_rank < current_rank;
    }
    return candidate_code < current_code;
}

void maybeAssignSelectedObservation(ObservationSelection& selection,
                                    const SatelliteId& sat,
                                    const std::string& obs_type,
                                    double value,
                                    int lli,
                                    int signal_strength,
                                    bool prefer_qzss_l1l,
                                    bool prefer_qzss_l5_secondary,
                                    bool primary) {
    if (value == 0.0) {
        return;
    }

    const int band = rinexBand(obs_type);
    int candidate_priority = bandPriority(sat.system, band, primary);
    if (candidate_priority >= 100) {
        return;
    }

    const char tracking_code = rinexTrackingCode(obs_type);
    // Native MADOCA-PPP sets GNSS_PPP_QZSS_PREFER_L1L so QZSS L1 tracks the
    // L1L/L1X correction chain used by MADOCALIB. RINEX files often list
    // C1C/L1C before C1L/L1L; prefer L only when that mode is enabled.
    if (prefer_qzss_l1l && sat.system == GNSSSystem::QZSS && primary && band == 1) {
        if (tracking_code == 'L') {
            candidate_priority -= 2;
        } else if (tracking_code != 'C') {
            candidate_priority += 1;
        }
    }
    if (prefer_qzss_l5_secondary && sat.system == GNSSSystem::QZSS && !primary &&
        band == 5) {
        const int tracking_priority = qzssL5TrackingPriority(tracking_code);
        if (tracking_priority < 100) {
            candidate_priority -= 4;
            candidate_priority += tracking_priority;
        }
    }
    const int tracking_rank =
        signal_policy::trackingAttributeRank(sat.system, band, tracking_code);
    const bool same_band_level =
        candidate_priority == selection.priority && band == selection.band;
    const bool continues_current_track =
        same_band_level &&
        sameTrackingCode(selection.tracking_code, tracking_code);
    const bool starts_better_track =
        candidate_priority < selection.priority ||
        (same_band_level && !continues_current_track &&
         trackingCodeBeats(tracking_rank, tracking_code,
                           selection.tracking_rank, selection.tracking_code));
    if (!starts_better_track && !continues_current_track) {
        return;
    }

    if (starts_better_track) {
        selection.observation = Observation();
        selection.observation.satellite = sat;
        selection.observation.signal =
            signalForObservationType(sat.system, obs_type, primary);
        selection.observation.valid = true;
        selection.has_data = false;
        selection.priority = candidate_priority;
        selection.band = band;
        selection.tracking_code = tracking_code;
        selection.tracking_rank = tracking_rank;
    }

    assignObservationField(selection.observation,
                           obs_type,
                           value,
                           lli,
                           signal_strength);
    selection.has_data = true;
}

void appendSelectedObservations(
    const ObservationSelection& primary,
    const ObservationSelection& secondary,
    const std::map<int, ObservationSelection>& by_band,
    bool preserve_additional_bands,
    const std::map<SatelliteId, int>& glonass_frequency_channels,
    ObservationData& observations) {
    std::set<int> emitted_bands;
    const auto append = [&](const ObservationSelection& selection) {
        if (!selection.has_data || emitted_bands.count(selection.band) != 0) {
            return;
        }
        Observation observation = selection.observation;
        annotateGlonassFrequencyChannel(observation, glonass_frequency_channels);
        observations.addObservation(observation);
        emitted_bands.insert(selection.band);
    };

    append(primary);
    append(secondary);
    if (!preserve_additional_bands) {
        return;
    }
    for (const auto& [band, selection] : by_band) {
        (void)band;
        append(selection);
    }
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
        obs.pseudorange = value;
        obs.has_pseudorange = true;
        obs.pseudorange_observation_type = obs_type;
    } else if (isCarrierObservationType(obs_type)) {
        obs.carrier_phase = value;
        obs.has_carrier_phase = true;
        obs.carrier_phase_observation_type = obs_type;
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

}  // namespace

RINEXReader::RINEXReader()
    : qzss_prefer_l1l_(PPPEnvOverrides::fromEnvironment().qzss_prefer_l1l) {}

bool RINEXReader::open(const std::string& filename) {
    rinex4_system_data_.clear();
    if (rinex4::isCompactRinexPath(filename)) {
        std::cerr << "CompactRINEX input is not supported natively (.crx/.crx.gz): "
                  << filename << std::endl;
        return false;
    }
    file_.open(filename);
    current_line_ = 0;
    header_read_ = false;
    last_rinex4_epoch_was_event_ = false;
    last_rinex2_epoch_was_event_ = false;
    obs_type_sys_ = ' ';
    obs_type_expected_ = 0;
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

    // The caller may reuse a header object.  Reset only the Phase128 ledger
    // fields here; all historical header fields retain their existing parse
    // semantics below.
    header.glonass_frequency_channel_header_status =
        GlonassFrequencyChannelHeaderStatus::Absent;
    header.glonass_frequency_channel_header_label_lines = 0U;
    header.glonass_frequency_channels.clear();
    header.glonass_frequency_channel_entries.clear();
    header.glonass_frequency_channel_malformed_entries = 0U;
    
    std::string line;
    while (readLine(line)) {
        if (line.find("END OF HEADER") != std::string::npos) {
            break;
        }
        
        if (!parseHeaderLine(line, header)) {
            continue; // Skip invalid lines
        }
    }

    if (header.glonass_frequency_channel_header_label_lines == 0U) {
        header.glonass_frequency_channel_header_status =
            GlonassFrequencyChannelHeaderStatus::Absent;
    } else if (header.glonass_frequency_channel_malformed_entries != 0U) {
        header.glonass_frequency_channel_header_status =
            GlonassFrequencyChannelHeaderStatus::Malformed;
    } else if (header.glonass_frequency_channel_entries.empty()) {
        header.glonass_frequency_channel_header_status =
            GlonassFrequencyChannelHeaderStatus::ValidEmpty;
    } else {
        header.glonass_frequency_channel_header_status =
            GlonassFrequencyChannelHeaderStatus::Entries;
    }
    
    header_ = header;
    header_read_ = true;
    return true;
}

bool RINEXReader::readObservationEpoch(ObservationData& obs_data) {
    if (source_header_tracking_filter_ &&
        (header_.version < 3.0 || header_.version >= 4.0)) {
        return false;
    }
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

        // RINEX 2.x: only fixed-width epoch records start an epoch.  Short or
        // blank rows (all-blank observation fields) and any other stray row
        // are skipped; they are never a reason to stop reading.
        if (header_.version < 3.0) {
            while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
                line.pop_back();
            }
            if (!looksLikeRinex2EpochLine(line)) {
                continue;
            }
            last_rinex2_epoch_was_event_ = false;
            try {
                if (!parseObservationEpochV2(line, obs_data)) {
                    obs_data.clear();
                    continue;
                }
            } catch (const std::exception&) {
                // If parsing fails, continue looking for next epoch
                obs_data.clear();
                continue;
            }
            if (last_rinex2_epoch_was_event_) {
                // Event flags 2-5 (special records) and 6 (cycle slip
                // records) are consumed by the parser but are not
                // observation epochs.
                obs_data.clear();
                continue;
            }
            return true;
        } else if (isRinex4()) {
            if (!line.empty() && line[0] == '>') {
                if (!parseObservationEpochV4(line, obs_data)) {
                    return false;
                }
                // Event and cycle-slip records are consumed by the RINEX 4
                // parser but are not ordinary ObservationData epochs.  Keep
                // scanning so readAllObservations() reaches the next epoch.
                if (last_rinex4_epoch_was_event_) {
                    continue;
                }
                return true;
            }
        } else if (header_.version >= 3.0 && header_.version < 4.0) {
            // RINEX 3.x format starts with '>'
            if (!line.empty() && line[0] == '>') {
                return parseObservationEpochV3(line, obs_data);
            }
        } else {
            std::cerr << "Unsupported RINEX version for observation data: "
                      << header_.version << std::endl;
            return false;
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
    bool header_found = header_read_;
    if (!header_read_) {
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
        header_read_ = true;
    }

    if (!header_found) {
        // Header already processed, rewind might not work, so just continue
    }

    if (isRinex4()) {
        return readRinex4NavigationData(nav_data);
    }

    if (header_.version >= 5.0) {
        std::cerr << "Unsupported RINEX version for navigation data: "
                  << header_.version << std::endl;
        return false;
    }

    std::vector<std::string> eph_lines;
    int eph_detected = 0;
    int eph_parsed = 0;
    int eph_skipped_non_gps = 0;
    bool skipping_non_gps = false;
    int skip_lines_remaining = 0;

    // Detect RINEX version for navigation file format differences
    bool is_rinex3 = (header_.version >= 3.0 && header_.version < 4.0);

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

bool RINEXReader::readRinex4NavigationData(NavigationData& nav_data) {
    rinex4_system_data_.clear();

    // RINEX 4 makes the navigation record boundary explicit.  Keep the
    // complete body between two '>' headers so unsupported STO/EOP/ION (or
    // unsupported EPH message types) can be skipped without feeding their
    // fields to the legacy ephemeris parser.
    std::string line;
    rinex4::NavigationRecordHeader active_header;
    std::vector<std::string> body;
    bool have_active_record = false;
    bool active_record_supported = false;

    const auto finish_record = [&]() {
        if (!have_active_record || !active_record_supported) {
            body.clear();
            return;
        }

        if (active_header.record_type == "STO") {
            rinex4::SystemTimeOffsetRecord record;
            if (!rinex4::parseSystemTimeOffsetRecord(active_header, body, record)) {
                std::cerr << "Skipping malformed RINEX 4 STO body for "
                          << active_header.source << ' '
                          << active_header.message_type << std::endl;
            } else {
                rinex4_system_data_.system_time_offsets.push_back(std::move(record));
            }
            body.clear();
            return;
        }
        if (active_header.record_type == "EOP") {
            rinex4::EarthOrientationRecord record;
            if (!rinex4::parseEarthOrientationRecord(active_header, body, record)) {
                std::cerr << "Skipping malformed RINEX 4 EOP body for "
                          << active_header.source << ' '
                          << active_header.message_type << std::endl;
            } else {
                rinex4_system_data_.earth_orientation_parameters.push_back(
                    std::move(record));
            }
            body.clear();
            return;
        }
        if (active_header.record_type == "ION") {
            rinex4::IonosphereRecord record;
            if (!rinex4::parseIonosphereRecord(active_header, body, record)) {
                std::cerr << "Skipping malformed or unsupported RINEX 4 ION body for "
                          << active_header.source << ' '
                          << active_header.message_type;
                if (!active_header.subtype.empty()) {
                    std::cerr << ' ' << active_header.subtype;
                }
                std::cerr << std::endl;
            } else {
                rinex4_system_data_.ionosphere_records.push_back(std::move(record));
            }
            body.clear();
            return;
        }

        if (active_header.system == 'R' &&
            (active_header.message_type == "L1OC" ||
             active_header.message_type == "L3OC")) {
            rinex4::GlonassCdmaEphemerisRecord record;
            if (!rinex4::parseGlonassCdmaEphemerisRecord(
                    active_header, body, record)) {
                std::cerr << "Skipping malformed RINEX 4 GLONASS "
                          << active_header.message_type << " body for "
                          << active_header.source << std::endl;
                body.clear();
                return;
            }

            Ephemeris eph;
            std::ostringstream epoch_text;
            epoch_text << record.toc.year << ' ' << record.toc.month << ' '
                       << record.toc.day << ' ' << record.toc.hour << ' '
                       << record.toc.minute << ' ' << record.toc.second;
            const GNSSTime toc_utc = parseTime(epoch_text.str(), header_.version);
            const GNSSTime toc_gpst = utcToGpst(
                toc_utc, record.toc.year, record.toc.month, record.toc.day);

            GNSSTime tof_utc = normalizeWeekTow(
                toc_utc.week, record.transmission_time_utc_week);
            const double week_delta = tof_utc.tow - toc_utc.tow;
            if (week_delta < -302400.0) {
                ++tof_utc.week;
            } else if (week_delta > 302400.0) {
                --tof_utc.week;
            }
            const GNSSTime tof_gpst = utcToGpst(
                tof_utc, record.toc.year, record.toc.month, record.toc.day);

            eph.satellite = SatelliteId(GNSSSystem::GLONASS, active_header.prn);
            eph.toc = toc_gpst;
            eph.toe = toc_gpst;
            eph.tof = tof_gpst;
            eph.toes = toc_gpst.tow;
            eph.week = static_cast<uint16_t>(toc_gpst.week);
            // A16/A17 transmit -TauN; Ephemeris stores the actual TauN and
            // computeGlonassState() applies the broadcast leading minus sign.
            eph.glonass_taun = -record.minus_tau_n;
            eph.glonass_gamn = record.gamma_n;
            eph.glonass_position = Vector3d(
                record.position_km[0] * 1e3,
                record.position_km[1] * 1e3,
                record.position_km[2] * 1e3);
            eph.glonass_velocity = Vector3d(
                record.velocity_km_per_s[0] * 1e3,
                record.velocity_km_per_s[1] * 1e3,
                record.velocity_km_per_s[2] * 1e3);
            eph.glonass_acceleration = Vector3d(
                record.acceleration_km_per_s2[0] * 1e3,
                record.acceleration_km_per_s2[1] * 1e3,
                record.acceleration_km_per_s2[2] * 1e3);
            eph.health = static_cast<uint8_t>(record.signal_health);
            eph.valid = record.signal_health == 0 && record.data_validity == 0;
            eph.navigation_message_type = active_header.navigation_message_type;

            GlonassCdmaNavigationData cdma;
            cdma.beta = record.beta;
            cdma.data_validity = record.data_validity;
            cdma.satellite_type = record.satellite_type;
            cdma.source_flags = record.source_flags;
            cdma.aode = record.aode;
            cdma.aodc = record.aodc;
            cdma.attitude_flag = record.attitude_flag;
            cdma.sign_flag = record.sign_flag;
            cdma.urai_orbit = record.urai_orbit;
            cdma.urai_clock = record.urai_clock;
            cdma.tin = record.tin;
            cdma.tau1 = record.tau1;
            cdma.tau2 = record.tau2;
            cdma.yaw_angle = record.yaw_angle;
            cdma.angular_rate = record.angular_rate;
            cdma.angular_acceleration = record.angular_acceleration;
            cdma.max_angular_rate = record.max_angular_rate;
            cdma.pc_x = record.phase_center_m[0];
            cdma.pc_y = record.phase_center_m[1];
            cdma.pc_z = record.phase_center_m[2];
            cdma.transmission_time_utc_week = record.transmission_time_utc_week;
            cdma.tgd_l2ocp = record.tgd_l2ocp;
            cdma.isc_l3ocp = record.isc_l3ocp;
            eph.glonass_cdma_data = std::move(cdma);

            nav_data.addEphemeris(eph);
            body.clear();
            return;
        }

        if (active_header.system == 'S' &&
            active_header.message_type == "SBAS") {
            rinex4::SbasEphemerisRecord record;
            if (!rinex4::parseSbasEphemerisRecord(
                    active_header, body, record)) {
                std::cerr << "Skipping malformed RINEX 4 SBAS body for "
                          << active_header.source << std::endl;
                body.clear();
                return;
            }

            Ephemeris eph;
            std::ostringstream epoch_text;
            epoch_text << record.toc.year << ' ' << record.toc.month << ' '
                       << record.toc.day << ' ' << record.toc.hour << ' '
                       << record.toc.minute << ' ' << record.toc.second;
            const GNSSTime toc_utc = parseTime(epoch_text.str(), header_.version);
            const GNSSTime toc_gpst = utcToGpst(
                toc_utc, record.toc.year, record.toc.month, record.toc.day);

            // RINEX writes SBAS GEO satellites with the on-file PRN offset by
            // 100 from the broadcast SBAS PRN (S21 -> PRN 121), matching
            // RTKLIB satid2no()'s case 'S' (+100).
            const uint8_t sbas_prn =
                static_cast<uint8_t>(active_header.prn + 100);
            eph.satellite = SatelliteId(GNSSSystem::SBAS, sbas_prn);
            eph.toc = toc_gpst;
            eph.toe = toc_gpst;
            eph.tof = toc_gpst;
            eph.toes = toc_gpst.tow;
            eph.week = static_cast<uint16_t>(toc_gpst.week);
            eph.glonass_position = Vector3d(
                record.x_position_km * 1e3,
                record.y_position_km * 1e3,
                record.z_position_km * 1e3);
            eph.glonass_velocity = Vector3d(
                record.x_velocity_m_per_s,
                record.y_velocity_m_per_s,
                record.z_velocity_m_per_s);
            eph.glonass_acceleration = Vector3d(
                record.x_acceleration_m_per_s2,
                record.y_acceleration_m_per_s2,
                record.z_acceleration_m_per_s2);
            eph.af0 = record.clock_bias_s;
            eph.af1 = record.clock_drift_s_per_s;
            eph.sv_health = static_cast<double>(record.health);
            eph.health = static_cast<uint8_t>(record.health);
            eph.sv_accuracy = static_cast<double>(record.accuracy_index);
            eph.valid = record.health == 0;
            eph.navigation_message_type = active_header.navigation_message_type;

            nav_data.addEphemeris(eph);
            body.clear();
            return;
        }

        Ephemeris eph;
        if (!parseNavigationMessage(body, eph)) {
            std::cerr << "Skipping RINEX 4 EPH " << active_header.source << ' '
                      << active_header.message_type
                      << ": incompatible or malformed navigation body"
                      << std::endl;
            body.clear();
            return;
        }

        const SatelliteId expected_satellite(
            systemFromRinexChar(active_header.system), active_header.prn);
        if (eph.satellite != expected_satellite) {
            std::cerr << "Skipping RINEX 4 EPH " << active_header.source
                      << ": body satellite does not match record header"
                      << std::endl;
            body.clear();
            return;
        }

        if (active_header.system == 'R' &&
            !rinex4::isPlausibleGlonassFdmaState(eph.glonass_position,
                                                 eph.glonass_velocity)) {
            // A record assembled from immediate-data strings of two frames
            // (e.g. stale Z/Vz/Az or TauN after a tb change) is not on any
            // GLONASS orbit; using it puts the satellite thousands of km off.
            const std::string epoch =
                body.front().size() >= 23 ? body.front().substr(4, 19) : std::string();
            std::cerr << "Skipping RINEX 4 GLONASS EPH " << active_header.source << ' '
                      << active_header.message_type << ' ' << epoch
                      << ": broadcast state vector is not on a GLONASS orbit (|r|="
                      << eph.glonass_position.norm() * 1e-3 << " km)" << std::endl;
            body.clear();
            return;
        }

        if (active_header.system == 'E') {
            const bool inav_e1b_source = (eph.data_source_code & (1 << 0)) != 0;
            const bool fnav_e5a_source = (eph.data_source_code & (1 << 1)) != 0;
            const bool inav_e5b_source = (eph.data_source_code & (1 << 2)) != 0;
            const bool fnav_clock_source = (eph.data_source_code & (1 << 8)) != 0;
            const bool inav_clock_source = (eph.data_source_code & (1 << 9)) != 0;
            const bool header_is_fnav =
                active_header.navigation_message_type == NavigationMessageType::FNAV;
            const bool header_is_inav =
                active_header.navigation_message_type == NavigationMessageType::INAV;
            const bool source_matches_header =
                (header_is_fnav && fnav_e5a_source && fnav_clock_source &&
                 !inav_e1b_source && !inav_e5b_source && !inav_clock_source) ||
                (header_is_inav && (inav_e1b_source || inav_e5b_source) &&
                 inav_clock_source && !fnav_e5a_source && !fnav_clock_source);
            if (!source_matches_header) {
                std::cerr << "Skipping RINEX 4 Galileo EPH " << active_header.source
                          << ' ' << active_header.message_type
                          << ": body data-source clock bit contradicts header"
                          << std::endl;
                body.clear();
                return;
            }
        }

        eph.navigation_message_type = active_header.navigation_message_type;

        nav_data.addEphemeris(eph);
        body.clear();
    };

    while (readLine(line)) {
        const size_t first_non_space = line.find_first_not_of(" \t");
        const bool is_record_header =
            first_non_space != std::string::npos && line[first_non_space] == '>';
        if (is_record_header) {
            finish_record();
            have_active_record = false;
            active_record_supported = false;
            active_header = rinex4::NavigationRecordHeader();

            if (!rinex4::parseNavigationRecordHeader(line, active_header)) {
                std::cerr << "Skipping malformed RINEX 4 navigation record header: "
                          << line << std::endl;
                have_active_record = true;
                continue;
            }

            have_active_record = true;
            if (active_header.record_type != "EPH") {
                if (active_header.record_type == "STO" ||
                    active_header.record_type == "EOP" ||
                    active_header.record_type == "ION") {
                    active_record_supported = true;
                } else {
                    std::cerr << "Skipping unsupported RINEX 4 navigation record type: "
                              << active_header.record_type << std::endl;
                }
                continue;
            }

            active_record_supported = rinex4::supportsEphemerisMessage(
                active_header.system, active_header.navigation_message_type);
            if (!active_record_supported) {
                std::cerr << "Skipping unsupported RINEX 4 EPH "
                          << active_header.source << ' '
                          << active_header.message_type << std::endl;
            }
            continue;
        }

        if (!have_active_record) {
            if (line.find_first_not_of(" \t\r\n") != std::string::npos) {
                std::cerr << "Skipping RINEX 4 navigation data before a record header"
                          << std::endl;
            }
            continue;
        }

        if (active_record_supported && !line.empty()) {
            body.push_back(line);
        }
    }

    finish_record();
    return !nav_data.isEmpty() || !rinex4_system_data_.empty();
}

bool RINEXReader::parseHeaderLine(const std::string& line, RINEXHeader& header) {
    if (line.length() < 60) {
        // A shortened line carrying the fixed GLONASS label is malformed,
        // not an absent header.  Keep the distinction for Phase128 while
        // retaining the historical ignore-short-line behavior otherwise.
        if (line.find("GLONASS SLOT / FRQ #") != std::string::npos) {
            ++header.glonass_frequency_channel_header_label_lines;
            ++header.glonass_frequency_channel_malformed_entries;
        }
        return false;
    }
    
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
        header.has_approximate_position = header.approximate_position.allFinite();
    }
    else if (label.find("ANTENNA: DELTA H/E/N") != std::string::npos) {
        const double height = std::stod(line.substr(0, 14));
        const double east = std::stod(line.substr(14, 14));
        const double north = std::stod(line.substr(28, 14));
        header.antenna_delta = Vector3d(east, north, height);
        header.has_antenna_delta = header.antenna_delta.allFinite();
    }
    else if (label.find("TIME OF FIRST OBS") != std::string::npos) {
        try {
            header.first_obs = parseTime(line.substr(0, 43), header.version);
        } catch (...) {
            // Leave first_obs at its default if the optional header record is malformed.
        }
    }
    else if (label.find("TIME OF LAST OBS") != std::string::npos) {
        try {
            header.last_obs = parseTime(line.substr(0, 43), header.version);
        } catch (...) {
            // Optional record; leave last_obs at its default if malformed.
        }
    }
    else if (label.find("# / TYPES OF OBSERV") != std::string::npos) {
        // RINEX 2: "I6,9(4X,A2)" - the count is on the first record only;
        // lists with more than 9 types continue on following records whose
        // count field is blank (e.g. 2.11 files with D/S/L5 types).
        const std::string count_field = trimCopy(line.substr(0, 6));
        if (!count_field.empty()) {
            try {
                obs_type_expected_ = std::stoi(count_field);
            } catch (const std::exception&) {
                obs_type_expected_ = 0;
            }
            header.observation_types.clear();
        }

        for (int i = 0; i < 9 &&
                        static_cast<int>(header.observation_types.size()) <
                            obs_type_expected_;
             ++i) {
            const size_t pos = 10 + static_cast<size_t>(i) * 6;
            if (pos + 2 > line.length()) {
                break;
            }
            const std::string obs_type = trimCopy(line.substr(pos, 2));
            if (!obs_type.empty()) {
                header.observation_types.push_back(obs_type);
            }
        }
    }
    else if (label.find("SYS / # / OBS TYPES") != std::string::npos) {
        // RINEX 3/4: Per-system observation types
        // Format: "G   22 C1C L1C C2X L2X ..." (system char at pos 0, count at
        // pos 3-6, types at pos 7+, up to 13 types per line). When a system has
        // more than 13 types, the remainder continue on following lines whose
        // system column is blank.
        char sys_char = line[0];
        if (sys_char != ' ') {
            // First line for this system.
            obs_type_sys_ = sys_char;
            obs_type_expected_ = std::stoi(line.substr(3, 3));
            header.system_obs_types[sys_char] = {};
        }
        // Append types from this line (first or continuation) to the system
        // currently being parsed. Up to 13 types per line, 4 chars each at pos 7.
        if (obs_type_sys_ != ' ') {
            std::vector<std::string>& types = header.system_obs_types[obs_type_sys_];
            for (int i = 0; i < 13 && static_cast<int>(types.size()) < obs_type_expected_; ++i) {
                size_t pos = 7 + i * 4;
                if (pos + 3 <= line.length()) {
                    std::string obs_type = line.substr(pos, 3);
                    obs_type.erase(0, obs_type.find_first_not_of(' '));
                    obs_type.erase(obs_type.find_last_not_of(' ') + 1);
                    if (!obs_type.empty()) {
                        types.push_back(obs_type);
                    }
                }
            }

            // Mirror the GPS type list into the generic observation_types for
            // backward compatibility, and clear the in-progress marker once the
            // expected count has been reached.
            if (obs_type_sys_ == 'G') {
                header.observation_types = types;
            }
            if (static_cast<int>(types.size()) >= obs_type_expected_) {
                obs_type_sys_ = ' ';
                obs_type_expected_ = 0;
            }
        }
    }
    else if (label.find("GLONASS SLOT / FRQ #") != std::string::npos) {
        ++header.glonass_frequency_channel_header_label_lines;
        for (int i = 0; i < 8; ++i) {
            const size_t pos = 4 + static_cast<size_t>(i) * 7;
            if (pos + 6 > line.size()) {
                if (pos < line.size() &&
                    line.find_first_not_of(" \t\r\n", pos) !=
                        std::string::npos) {
                    ++header.glonass_frequency_channel_malformed_entries;
                }
                break;
            }
            if (line[pos] != 'R') {
                // A non-empty slot with another system/designator is not a
                // valid empty slot.  Do not silently classify it as absent.
                if (line.find_first_not_of(" \t", pos) != std::string::npos &&
                    line.find_first_not_of(" \t", pos) < pos + 7U) {
                    ++header.glonass_frequency_channel_malformed_entries;
                }
                continue;
            }
            const std::string prn_text = trimCopy(line.substr(pos + 1, 2));
            const std::string channel_text = trimCopy(line.substr(pos + 4, 3));
            if (prn_text.empty() || channel_text.empty()) {
                ++header.glonass_frequency_channel_malformed_entries;
                continue;
            }
            try {
                std::size_t prn_consumed = 0U;
                std::size_t channel_consumed = 0U;
                const int prn = std::stoi(prn_text, &prn_consumed);
                const int channel = std::stoi(channel_text, &channel_consumed);
                // Keep the historical map assignment for valid integer
                // prefixes, but preserve strict lexical validity in the
                // opt-in ledger.  Thus selector-off parsing remains
                // compatible while selector-on cannot accept "-4x" or a
                // non-integer FCN as if it were a real channel.
                const SatelliteId satellite(GNSSSystem::GLONASS,
                                            static_cast<uint8_t>(prn));
                header.glonass_frequency_channels[satellite] = channel;
                if (prn_consumed != prn_text.size() ||
                    channel_consumed != channel_text.size() || prn < 1 ||
                    prn > 27) {
                    ++header.glonass_frequency_channel_malformed_entries;
                    continue;
                }
                header.glonass_frequency_channel_entries.emplace_back(
                    satellite, channel);
            } catch (...) {
                ++header.glonass_frequency_channel_malformed_entries;
            }
        }
    }

    return true;
}

bool RINEXReader::readLineStripCr(std::string& line) {
    if (!readLine(line)) {
        return false;
    }
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
        line.pop_back();
    }
    return true;
}

bool RINEXReader::parseObservationEpochV2(const std::string& epoch_line, ObservationData& obs_data) {
    // RINEX 2.x epoch record: (1X,I2.2,4(1X,I2),F11.7,2X,I1,I3,12(A1,I2))
    // followed, per satellite, by ceil(n_types / 5) observation rows of
    // 5 x (F14.3,I1,I1).
    last_rinex2_epoch_was_event_ = false;

    // Rows may be right-trimmed by some writers; pad so field access is safe.
    std::string line = epoch_line;
    if (line.length() < 32) line.resize(32, ' ');

    // Epoch flag (col 29) and satellite count / special-record count
    // (cols 30-32) come first: event records carry no usable epoch time.
    int epoch_flag = 0;
    if (line[28] != ' ') {
        epoch_flag = line[28] - '0';
    }
    const std::string num_sats_field = trimCopy(line.substr(29, 3));
    const int num_sats = num_sats_field.empty() ? 0 : std::stoi(num_sats_field);

    if (epoch_flag >= 2 && epoch_flag <= 5) {
        // 2 = start moving antenna, 3 = new site occupation, 4 = header
        // information follows, 5 = external event.  The count field is the
        // number of special-record lines that follow; consume them so they
        // are not misread as epochs.  Flags 3/4 can carry header records:
        // an updated "# / TYPES OF OBSERV" list is applied (it changes the
        // layout of all following observation rows); every other record
        // (marker name, antenna, comments, ...) is skipped.
        last_rinex2_epoch_was_event_ = true;
        for (int i = 0; i < num_sats; ++i) {
            std::string special;
            if (!readLineStripCr(special)) break;
            if ((epoch_flag == 3 || epoch_flag == 4) && special.length() >= 60 &&
                special.find("# / TYPES OF OBSERV", 60) != std::string::npos) {
                try {
                    parseHeaderLine(special, header_);
                } catch (const std::exception&) {
                    // Leave the current observation type list unchanged.
                }
            }
        }
        return true;
    }

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

    // Satellite list: 12 three-character ids per record in cols 33-68; further
    // ids continue on following records (cols 33-68 again).  The continuation
    // is read whenever the 12 slots of a record are used up - independent of
    // how the first record is padded or whether a receiver clock offset
    // occupies cols 69-80.
    struct Rinex2Satellite {
        SatelliteId id;
        bool valid = false;
    };
    std::vector<Rinex2Satellite> satellites;
    satellites.reserve(static_cast<size_t>(num_sats));
    std::string current_line = line;

    for (int i = 0; i < num_sats; ++i) {
        if (i > 0 && i % 12 == 0) {
            if (!readLineStripCr(current_line)) break;
        }
        const size_t prn_pos = 32 + static_cast<size_t>(i % 12) * 3;
        std::string id_text = prn_pos < current_line.length()
                                  ? current_line.substr(prn_pos, 3)
                                  : std::string();
        id_text.resize(3, ' ');

        // Every listed satellite owns observation rows, so an id that cannot
        // be mapped stays in the list as an invalid placeholder (its rows are
        // consumed and dropped) instead of shifting the later satellites.
        Rinex2Satellite entry;
        const GNSSSystem system = rinex2SystemFromChar(id_text[0]);
        const std::string prn_str = trimCopy(id_text.substr(1, 2));
        int prn = 0;
        if (system != GNSSSystem::UNKNOWN && !prn_str.empty() &&
            std::isdigit(static_cast<unsigned char>(prn_str.front())) != 0) {
            try {
                prn = std::stoi(prn_str);
            } catch (const std::exception&) {
                prn = 0;
            }
        }
        if (prn >= 1 && prn <= 99) {
            // PRN is stored as printed (S20 -> SBAS 20, J02 -> QZSS 2),
            // matching the RINEX 3 path and SatelliteId::toString().
            entry.id = SatelliteId(system, static_cast<uint8_t>(prn));
            entry.valid = true;
        }
        satellites.push_back(entry);
    }

    // Read observation data for each satellite
    int num_obs_types = header_.observation_types.size();
    if (num_obs_types == 0) num_obs_types = 4;  // Default: L1, C1, L2, P2

    for (size_t sat_idx = 0; sat_idx < satellites.size(); ++sat_idx) {
        const SatelliteId sat = satellites[sat_idx].id;
        const bool sat_valid = satellites[sat_idx].valid;

        // Calculate number of lines needed for this satellite (5 obs per line)
        int lines_per_sat = (num_obs_types + 4) / 5;

        std::vector<double> obs_values(num_obs_types, 0.0);
        std::vector<int> lli_flags(num_obs_types, 0);
        std::vector<int> signal_strength(num_obs_types, 0);

        // Read observation lines for this satellite
        for (int line_idx = 0; line_idx < lines_per_sat; ++line_idx) {
            std::string obs_line;
            if (!readLineStripCr(obs_line)) break;
            // Blank or right-trimmed rows are valid: missing columns are
            // all-blank observation fields.
            if (obs_line.length() < 80) obs_line.resize(80, ' ');

            // Each observation occupies 16 characters
            for (int obs_in_line = 0; obs_in_line < 5; ++obs_in_line) {
                int obs_idx = line_idx * 5 + obs_in_line;
                if (obs_idx >= num_obs_types) break;

                size_t col_start = obs_in_line * 16;

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
                    lli_flags[obs_idx] = rinexLliFromChar(obs_line[col_start + 14]);
                }

                // Parse signal strength (position 15-16)
                if (col_start + 15 < obs_line.length() && obs_line[col_start + 15] != ' ') {
                    signal_strength[obs_idx] = rinexSsiFromChar(obs_line[col_start + 15]);
                }
            }
        }

        // Rows of satellites with an unmappable id (e.g. a system letter that
        // RINEX 2.x does not define) were consumed above; drop their data.
        if (!sat_valid) {
            continue;
        }

        // C1/P1 and C2/P2 selection must not depend on header order.  RINEX
        // 2.x has no tracking-mode letter, so when both pseudoranges of a
        // band are present the choice follows RTKLIB's code priority for
        // 2.x files (rinex.c convcode + rtkcmn.c codepris), and the other
        // one is ignored; if the preferred one is missing the other is used:
        //   L1 (all systems): C1 (C/A) before P1 (P(Y))   [GPS "CPYW...", GLO "CPAB.."]
        //   L2 GLONASS      : C2 (C/A) before P2          [GLO "CPAB.."]
        //   L2 other        : P2 (P(Y)) before C2 (L2C)   [GPS "CPYW...DLSX": W before X]
        std::vector<char> ignored_type(obs_values.size(), 0);
        {
            const auto find_type = [&](const char* name) -> int {
                for (size_t k = 0; k < header_.observation_types.size() &&
                                   k < obs_values.size(); ++k) {
                    if (header_.observation_types[k] == name) {
                        return static_cast<int>(k);
                    }
                }
                return -1;
            };
            const auto prefer = [&](const char* preferred, const char* other) {
                const int pi = find_type(preferred);
                const int oi = find_type(other);
                if (pi >= 0 && oi >= 0 && obs_values[pi] != 0.0) {
                    ignored_type[oi] = 1;
                }
            };
            prefer("C1", "P1");
            if (sat.system == GNSSSystem::GLONASS) {
                prefer("C2", "P2");
            } else {
                prefer("P2", "C2");
            }
        }

        ObservationSelection primary_selection;
        ObservationSelection secondary_selection;
        std::map<int, ObservationSelection> band_selections;

        for (size_t i = 0; i < header_.observation_types.size() && i < obs_values.size(); ++i) {
            if (ignored_type[i]) {
                continue;
            }
            const std::string& obs_type = header_.observation_types[i];
            maybeAssignSelectedObservation(primary_selection,
                                           sat,
                                           obs_type,
                                           obs_values[i],
                                           lli_flags[i],
                                           signal_strength[i],
                                           qzss_prefer_l1l_,
                                           qzss_prefer_l5_secondary_,
                                           true);
            maybeAssignSelectedObservation(secondary_selection,
                                           sat,
                                           obs_type,
                                           obs_values[i],
                                           lli_flags[i],
                                           signal_strength[i],
                                           qzss_prefer_l1l_,
                                           qzss_prefer_l5_secondary_,
                                           false);
            if (preserve_additional_frequency_bands_) {
                const int band = rinexBand(obs_type);
                const bool primary_band = isPrimaryBand(sat.system, band);
                if (primary_band || isSecondaryBand(sat.system, band)) {
                    maybeAssignSelectedObservation(
                        band_selections[band], sat, obs_type, obs_values[i],
                        lli_flags[i], signal_strength[i], qzss_prefer_l1l_,
                        qzss_prefer_l5_secondary_, primary_band);
                }
            }
        }

        appendSelectedObservations(
            primary_selection, secondary_selection, band_selections,
            preserve_additional_frequency_bands_,
            header_.glonass_frequency_channels, obs_data);
    }

    // Epoch flag 6: the rows above are cycle-slip records (observation
    // layout, slip information in LLI), not measurements.  They have been
    // consumed; do not report them as an epoch (RTKLIB likewise ignores them).
    if (epoch_flag == 6) {
        obs_data.clear();
        last_rinex2_epoch_was_event_ = true;
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

        // Epoch record is (A1,1X,I4,4(1X,I2.2),F11.7,2X,I1,I3): the satellite
        // count is the I3 at columns 32-34, so counts of 100 or more parse.
        std::string num_sats_str = epoch_line.substr(32, 3);
        num_sats_str.erase(0, num_sats_str.find_first_not_of(' '));
        int num_sats = num_sats_str.empty() ? 0 : std::stoi(num_sats_str);

        if (!parseObservationRows(num_sats, obs_data, false)) {
            return false;
        }

    } catch (const std::exception& e) {
        std::cerr << "RINEX 3 observation epoch parsing error: " << e.what() << std::endl;
        return false;
    }

    return true;
}

bool RINEXReader::parseObservationSatelliteRecord(
    const std::string& sat_line,
    ObservationData& obs_data,
    bool strict) {
    if (sat_line.size() < 3) {
        return !strict;
    }

    const char sys_char = sat_line[0];
    const bool exact_prn = std::isdigit(static_cast<unsigned char>(sat_line[1])) != 0 &&
                           std::isdigit(static_cast<unsigned char>(sat_line[2])) != 0;
    if (strict && !exact_prn) {
        return false;
    }
    std::string prn_str = sat_line.substr(1, 2);
    if (!strict) {
        prn_str.erase(0, prn_str.find_first_not_of(' '));
    }
    if (prn_str.empty() || (strict && !exact_prn)) {
        return !strict;
    }

    int prn = 0;
    try {
        prn = std::stoi(prn_str);
    } catch (...) {
        return !strict;
    }
    if (strict && prn <= 0) {
        return false;
    }
    const GNSSSystem system = systemFromRinexChar(sys_char);
    if (system == GNSSSystem::UNKNOWN) {
        return !strict;
    }

    auto sys_it = header_.system_obs_types.find(sys_char);
    const std::vector<std::string>& header_obs_types =
        (sys_it != header_.system_obs_types.end()) ? sys_it->second : header_.observation_types;
    // RINEX 3.00-3.02 BeiDou "C1I/L1I/D1I/S1I" is B1I (band 2 in 3.03+).
    std::vector<std::string> version_normalized_obs_types;
    if (signal_policy::needsRinexVersionObsTypeRemap(system, header_.version)) {
        version_normalized_obs_types.reserve(header_obs_types.size());
        for (const auto& type : header_obs_types) {
            version_normalized_obs_types.push_back(
                signal_policy::normalizeObservationTypeForRinexVersion(
                    system, type, header_.version));
        }
    }
    const std::vector<std::string>& obs_types =
        version_normalized_obs_types.empty() ? header_obs_types
                                             : version_normalized_obs_types;
    int num_obs_types = static_cast<int>(obs_types.size());
    if (num_obs_types == 0) {
        num_obs_types = 4;
    }

    SatelliteId sat(system, prn);
    std::vector<double> obs_values(num_obs_types, 0.0);
    std::vector<int> lli_flags(num_obs_types, 0);
    std::vector<int> signal_strength(num_obs_types, 0);

    for (int i = 0; i < num_obs_types; ++i) {
        const size_t col_start = 3 + static_cast<size_t>(i) * 16;
        const std::string& row = sat_line;
        if (col_start + 14 > row.length()) {
            if (strict) {
                return false;
            }
            continue;
        }
        if (strict && col_start + 16 > row.length()) {
            return false;
        }

        std::string obs_str = row.substr(col_start, 14);
        obs_str.erase(0, obs_str.find_first_not_of(' '));
        if (!obs_str.empty()) {
            obs_str.erase(obs_str.find_last_not_of(' ') + 1);
            try {
                size_t consumed = 0;
                const double value = std::stod(obs_str, &consumed);
                if (strict && (consumed != obs_str.size() || !std::isfinite(value))) {
                    return false;
                }
                obs_values[i] = value;
            } catch (...) {
                if (strict) {
                    return false;
                }
                obs_values[i] = 0.0;
            }
        }

        const size_t lli_pos = col_start + 14;
        const size_t strength_pos = col_start + 15;
        if (lli_pos < row.length() && row[lli_pos] != ' ') {
            if (strict && !std::isdigit(static_cast<unsigned char>(row[lli_pos]))) {
                return false;
            }
            lli_flags[i] = rinexLliFromChar(row[lli_pos]);
        }
        if (strength_pos < row.length() && row[strength_pos] != ' ') {
            if (strict && !std::isdigit(static_cast<unsigned char>(row[strength_pos]))) {
                return false;
            }
            signal_strength[i] = rinexSsiFromChar(row[strength_pos]);
        }
    }

    ObservationSelection primary_selection;
    ObservationSelection secondary_selection;
    std::map<std::string, Observation> tracking_observations;
    std::map<int, ObservationSelection> band_selections;

    // RTKLIB fixes its normal-frequency slots from the RINEX header, not
    // from whichever values happen to be present in this epoch.
    if (sat.system == GNSSSystem::GPS) {
        obs_data.setRinexFrequencySlot(GNSSSystem::GPS, 0, "1C");
        static constexpr char kGpsL2Priority[] = "PYWCMNDLXS";
        for (const char* priority = kGpsL2Priority;
             *priority != '\0'; ++priority) {
            const std::string candidate = std::string("2") + *priority;
            const bool declared = std::any_of(
                obs_types.begin(), obs_types.end(),
                [&candidate](const std::string& type) {
                    return type.size() >= 3 && type.substr(1) == candidate;
                });
            if (declared) {
                obs_data.setRinexFrequencySlot(GNSSSystem::GPS, 1, candidate);
                break;
            }
        }
    }

    std::map<source_transmission_clock::Slot, std::string> source_codes;
    if (source_header_tracking_filter_) {
        std::vector<std::string> codes;
        for (const auto& type : obs_types) {
            if (type.size() == 3) codes.push_back(type.substr(1));
        }
        source_codes = source_transmission_clock::selectHeaderTrackingCodes(system, codes);
    }
    for (size_t i = 0; i < obs_types.size() && i < obs_values.size(); ++i) {
        const std::string& obs_type = obs_types[i];
        if (source_header_tracking_filter_) {
            const auto slot = source_transmission_clock::slotForRinexBand(system, rinexBand(obs_type));
            if (!slot || !source_codes.count(*slot) || obs_type.size() != 3 ||
                source_codes.at(*slot) != obs_type.substr(1)) continue;
        }
        if (obs_values[i] != 0.0 && obs_type.size() >= 3) {
            const std::string tracking_code = obs_type.substr(1);
            auto [it, inserted] = tracking_observations.try_emplace(tracking_code);
            Observation& exact = it->second;
            if (inserted) {
                exact.satellite = sat;
                exact.signal = signalForObservationType(
                    sat.system, obs_type,
                    isPrimaryBand(sat.system, rinexBand(obs_type)));
                exact.valid = true;
            }
            assignObservationField(exact, obs_type, obs_values[i],
                                   lli_flags[i], signal_strength[i]);
        }
        maybeAssignSelectedObservation(primary_selection, sat, obs_type,
                                       obs_values[i], lli_flags[i],
                                       signal_strength[i], qzss_prefer_l1l_,
                                       qzss_prefer_l5_secondary_, true);
        maybeAssignSelectedObservation(secondary_selection, sat, obs_type,
                                       obs_values[i], lli_flags[i],
                                       signal_strength[i], qzss_prefer_l1l_,
                                       qzss_prefer_l5_secondary_, false);
        if (preserve_additional_frequency_bands_) {
            const int band = rinexBand(obs_type);
            const bool primary_band = isPrimaryBand(sat.system, band);
            if (primary_band || isSecondaryBand(sat.system, band)) {
                maybeAssignSelectedObservation(
                    band_selections[band], sat, obs_type, obs_values[i],
                    lli_flags[i], signal_strength[i], qzss_prefer_l1l_,
                    qzss_prefer_l5_secondary_, primary_band);
            }
        }
    }

    appendSelectedObservations(primary_selection, secondary_selection,
                               band_selections,
                               preserve_additional_frequency_bands_,
                               header_.glonass_frequency_channels, obs_data);
    for (auto& [tracking_code, exact] : tracking_observations) {
        annotateGlonassFrequencyChannel(exact, header_.glonass_frequency_channels);
        obs_data.addRinexTrackingObservation(tracking_code, exact);
    }
    return true;
}

bool RINEXReader::parseObservationRows(int num_sats,
                                       ObservationData& obs_data,
                                       bool strict) {
    for (int s = 0; s < num_sats; ++s) {
        std::string sat_line;
        if (!readLine(sat_line)) {
            return !strict;
        }
        if (sat_line.size() < 3) {
            if (strict) {
                return false;
            }
            continue;
        }

        if (!parseObservationSatelliteRecord(sat_line, obs_data, strict)) {
            return false;
        }
    }
    return true;
}

bool RINEXReader::parseObservationEpochV4(const std::string& epoch_line,
                                          ObservationData& obs_data) {
    last_rinex4_epoch_was_event_ = false;
    obs_data.clear();
    ObservationData parsed;
    rinex4::ObservationEpochHeader epoch;
    if (!rinex4::parseObservationEpochHeader(epoch_line, epoch)) {
        std::cerr << "RINEX 4 observation epoch header is malformed" << std::endl;
        return false;
    }

    if (epoch.has_date) {
        std::ostringstream time_text;
        time_text << epoch.year << ' ' << epoch.month << ' ' << epoch.day << ' '
                  << epoch.hour << ' ' << epoch.minute << ' '
                  << std::setprecision(17) << epoch.second;
        parsed.time = parseTime(time_text.str(), header_.version);
    } else if (epoch.flag <= 1) {
        std::cerr << "RINEX 4 normal epoch is missing date/time fields" << std::endl;
        return false;
    }

    if (epoch.flag >= 2) {
        last_rinex4_epoch_was_event_ = true;
        for (int record = 0; record < epoch.record_count; ++record) {
            std::string special_record;
            if (!readLine(special_record)) {
                std::cerr << "RINEX 4 event record is truncated" << std::endl;
                return false;
            }
            if (epoch.flag == 4) {
                // Header event records can change observation types for later
                // epochs.  Ignore ordinary comments, but let valid labels
                // update the same header state used during initial parsing.
                parseHeaderLine(special_record, header_);
            }
        }
        return true;
    }

    if (!parseObservationRows(epoch.record_count, parsed, true)) {
        std::cerr << "RINEX 4 observation epoch has truncated or malformed satellite records"
                  << std::endl;
        return false;
    }
    parsed.receiver_clock_bias = epoch.has_receiver_clock_offset
                                     ? epoch.receiver_clock_offset
                                     : 0.0;
    obs_data = std::move(parsed);
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

        // Phase128 keeps the historical permissive parseD() values for the
        // default path, but also records a strict source-positioned view of a
        // GLONASS data[0..14] record.  Empty/malformed fields become NaN in
        // this sidecar only; they are not shifted or replaced with zero.
        auto parseDStrict = [](const std::string& line, size_t start,
                               size_t len) -> double {
            if (start + len > line.length()) {
                return std::numeric_limits<double>::quiet_NaN();
            }
            std::string value = line.substr(start, len);
            const auto first = value.find_first_not_of(" \t");
            if (first == std::string::npos) {
                return std::numeric_limits<double>::quiet_NaN();
            }
            value.erase(0, first);
            const auto last = value.find_last_not_of(" \t\r\n");
            if (last == std::string::npos) {
                return std::numeric_limits<double>::quiet_NaN();
            }
            value.erase(last + 1U);
            std::replace(value.begin(), value.end(), 'D', 'E');
            std::replace(value.begin(), value.end(), 'd', 'E');
            try {
                std::size_t consumed = 0U;
                const double parsed = std::stod(value, &consumed);
                if (consumed != value.size() || !std::isfinite(parsed)) {
                    return std::numeric_limits<double>::quiet_NaN();
                }
                return parsed;
            } catch (...) {
                return std::numeric_limits<double>::quiet_NaN();
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
            // Preserve whether the broadcast FCN field was actually present.
            // Channel zero is valid, so the legacy numeric default cannot be
            // used as a presence sentinel by strict provenance consumers.
            std::string channel_text;
            if (c3 < lines[2].size()) {
                channel_text = lines[2].substr(c3, 19);
            }
            channel_text.erase(0, channel_text.find_first_not_of(" \t"));
            if (channel_text.find_last_not_of(" \t") != std::string::npos) {
                channel_text.erase(channel_text.find_last_not_of(" \t") + 1);
            }
            std::replace(channel_text.begin(), channel_text.end(), 'D', 'E');
            std::replace(channel_text.begin(), channel_text.end(), 'd', 'E');
            try {
                std::size_t consumed = 0U;
                const double raw_channel =
                    std::stod(channel_text, &consumed);
                const double normalized_channel =
                    raw_channel > 128.0 ? raw_channel - 256.0 : raw_channel;
                eph.glonass_frequency_channel_present =
                    consumed == channel_text.size() &&
                    std::isfinite(raw_channel) &&
                    std::isfinite(normalized_channel) &&
                    std::floor(normalized_channel) == normalized_channel &&
                    normalized_channel >=
                        static_cast<double>(std::numeric_limits<int>::min()) &&
                    normalized_channel <=
                        static_cast<double>(std::numeric_limits<int>::max());
                if (eph.glonass_frequency_channel_present) {
                    eph.glonass_frequency_channel =
                        static_cast<int>(normalized_channel);
                }
            } catch (...) {
                eph.glonass_frequency_channel_present = false;
            }
            eph.glonass_age = static_cast<int>(parseD(lines[3], c3, 19));
            const std::array<double, 15> canonical_data = {
                parseDStrict(first_line, af0_col, 19),
                parseDStrict(first_line, af1_col, 19),
                parseDStrict(first_line, af2_col, 19),
                parseDStrict(lines[1], c0, 19),
                parseDStrict(lines[1], c1, 19),
                parseDStrict(lines[1], c2, 19),
                parseDStrict(lines[1], c3, 19),
                parseDStrict(lines[2], c0, 19),
                parseDStrict(lines[2], c1, 19),
                parseDStrict(lines[2], c2, 19),
                parseDStrict(lines[2], c3, 19),
                parseDStrict(lines[3], c0, 19),
                parseDStrict(lines[3], c1, 19),
                parseDStrict(lines[3], c2, 19),
                parseDStrict(lines[3], c3, 19),
            };
            const auto canonical_result =
                decodeCanonicalGlonassGeph(canonical_data);
            eph.glonass_canonical_geph_data_valid = canonical_result.accepted;
            eph.glonass_canonical_geph_reject_reason =
                static_cast<int>(canonical_result.reject_reason);
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

        // Galileo broadcasts both I/NAV (E1/E5b) and F/NAV (E5a) ephemerides
        // for the same satellite, distinguished by this "data sources" word.
        // Retain it so the selector can prefer I/NAV (RTKLIB seleph default),
        // otherwise native picks the age-closest record and lands on F/NAV ~half
        // the time, giving a ~0.5 m along/cross-track orbit error vs the bridge.
        eph.data_source_code = static_cast<int>(system_codes);

        // Suppress unused variable warnings
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
        // CRLF files: drop the carriage return so header, navigation and
        // observation rows decode exactly like their LF counterparts.
        while (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        return true;
    }
    return false;
}

// RINEXWriter implementation
namespace {

// ---------------------------------------------------------------------------
// RINEX 3.04 observation writer helpers
// ---------------------------------------------------------------------------

constexpr double kWriterRinexVersion = 3.04;
constexpr std::int64_t kTicksPerSecond = 10000000;  // F11.7 epoch resolution
constexpr std::int64_t kTicksPerMinute = 60 * kTicksPerSecond;
constexpr std::int64_t kTicksPerDay = 86400 * kTicksPerSecond;
constexpr std::int64_t kTicksPerWeek = 7 * kTicksPerDay;

enum ObsKind : int { kKindCode = 0, kKindPhase = 1, kKindDoppler = 2, kKindSnr = 3 };
constexpr std::array<char, 4> kKindChar = {'C', 'L', 'D', 'S'};

// One tracking code ("1C") of one satellite within one epoch.
struct WriterChannel {
    std::string code;
    std::array<bool, 4> has{};
    std::array<double, 4> value{};
    int lli = 0;
};

// One value contributed by an Observation, before it is merged into a channel.
struct WriterContribution {
    std::string code;
    int kind = kKindCode;
    double value = 0.0;
    int lli = 0;
};

using WriterSatChannels = std::map<SatelliteId, std::vector<WriterChannel>>;

std::int64_t floorDiv(std::int64_t a, std::int64_t b) {
    std::int64_t q = a / b;
    if ((a % b != 0) && ((a < 0) != (b < 0))) {
        --q;
    }
    return q;
}

std::int64_t writerEpochTicks(const GNSSTime& time) {
    return static_cast<std::int64_t>(time.week) * kTicksPerWeek +
           static_cast<std::int64_t>(std::llround(time.tow * 1e7));
}

struct WriterCalendar {
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    int second = 0;
    int fraction_ticks = 0;  // 1e-7 s
};

WriterCalendar writerCalendarFromTicks(std::int64_t ticks) {
    const std::int64_t gps_days = floorDiv(ticks, kTicksPerDay);
    const std::int64_t tod = ticks - gps_days * kTicksPerDay;

    // Civil date from days since the GPS epoch (1980-01-06 = day 3657 after
    // 1970-01-01, then the usual days-from-civil inverse).
    const std::int64_t z = gps_days + 3657 + 719468;
    const std::int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    const std::int64_t doe = z - era * 146097;
    const std::int64_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    const std::int64_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const std::int64_t mp = (5 * doy + 2) / 153;
    const std::int64_t d = doy - (153 * mp + 2) / 5 + 1;
    const std::int64_t m = mp + (mp < 10 ? 3 : -9);

    WriterCalendar cal;
    cal.year = static_cast<int>(yoe + era * 400 + (m <= 2 ? 1 : 0));
    cal.month = static_cast<int>(m);
    cal.day = static_cast<int>(d);
    cal.hour = static_cast<int>(tod / (3600 * kTicksPerSecond));
    cal.minute = static_cast<int>((tod / kTicksPerMinute) % 60);
    const std::int64_t in_minute = tod % kTicksPerMinute;
    cal.second = static_cast<int>(in_minute / kTicksPerSecond);
    cal.fraction_ticks = static_cast<int>(in_minute % kTicksPerSecond);
    return cal;
}

template <typename... Args>
std::string formatString(const char* format, Args... args) {
    const int length = std::snprintf(nullptr, 0, format, args...);
    if (length <= 0) {
        return "";
    }
    std::string text(static_cast<std::size_t>(length) + 1U, '\0');
    std::snprintf(&text[0], text.size(), format, args...);
    text.resize(static_cast<std::size_t>(length));
    return text;
}

// Seconds as F<width>.7 (11 in epoch records, 13 in TIME OF FIRST OBS).
std::string writerSecondsField(const WriterCalendar& cal, int width) {
    const std::string seconds = formatString("%d.%07d", cal.second, cal.fraction_ticks);
    return formatString("%*s", width, seconds.c_str());
}

// Left-justified, space-padded, truncated A<width> field.
std::string headerField(const std::string& text, std::size_t width) {
    std::string field = trimCopy(text);
    field.resize(width, ' ');
    return field;
}

std::string headerLine(std::string body, const char* label) {
    body.resize(60, ' ');
    std::string text = label;
    text.resize(20, ' ');  // label field is A20
    return body + text + "\n";
}

bool writerSystemChar(GNSSSystem system, char& sys_char) {
    switch (system) {
        case GNSSSystem::GPS: sys_char = 'G'; return true;
        case GNSSSystem::GLONASS: sys_char = 'R'; return true;
        case GNSSSystem::Galileo: sys_char = 'E'; return true;
        case GNSSSystem::BeiDou: sys_char = 'C'; return true;
        case GNSSSystem::QZSS: sys_char = 'J'; return true;
        case GNSSSystem::SBAS: sys_char = 'S'; return true;
        case GNSSSystem::NavIC: sys_char = 'I'; return true;
        default: return false;
    }
}

// RINEX satellite number: SBAS PRN 120..158 -> S20..S58, QZSS PRN 193..202
// -> J01..J10.
bool writerSatelliteNumber(const SatelliteId& sat, int& number) {
    number = sat.prn;
    if (sat.system == GNSSSystem::SBAS && number >= 100) {
        number -= 100;
    } else if (sat.system == GNSSSystem::QZSS && number >= 193) {
        number -= 192;
    }
    return number >= 1 && number <= 99;
}

// A usable "<type><band><attribute>" RINEX 3 observation code, e.g. "C1C".
// Returns the two-character tracking code ("1C") or an empty string.
std::string trackingCodeFromObservationType(const std::string& obs_type) {
    if (obs_type.size() != 3) {
        return "";
    }
    const char type = obs_type[0];
    if (type != 'C' && type != 'L' && type != 'D' && type != 'S') {
        return "";
    }
    if (std::isdigit(static_cast<unsigned char>(obs_type[1])) == 0 ||
        std::isalpha(static_cast<unsigned char>(obs_type[2])) == 0) {
        return "";
    }
    return obs_type.substr(1);
}

// Everything the observation contributes to the file, keyed by tracking code.
// `forced_code` (from ObservationData::rinex_tracking_observations) overrides
// the observation's own code resolution.
void collectContributions(const Observation& obs,
                          const std::string* forced_code,
                          std::vector<WriterContribution>& out) {
    if (!obs.valid) {
        return;
    }
    const std::string fallback =
        forced_code != nullptr
            ? *forced_code
            : defaultRinexTrackingCode(obs.satellite.system, obs.signal);
    std::string code_of_range = fallback;
    std::string code_of_phase = fallback;
    if (forced_code == nullptr) {
        const std::string pr_code =
            trackingCodeFromObservationType(obs.pseudorange_observation_type);
        const std::string cp_code =
            trackingCodeFromObservationType(obs.carrier_phase_observation_type);
        code_of_range = !pr_code.empty() ? pr_code : (!cp_code.empty() ? cp_code : fallback);
        code_of_phase = !cp_code.empty() ? cp_code : (!pr_code.empty() ? pr_code : fallback);
    }
    // Doppler and C/N0 belong to the tracking channel that produced the phase.
    const std::string& code_of_aux = obs.has_carrier_phase ? code_of_phase : code_of_range;

    const auto push = [&out](const std::string& code, int kind, double value, int lli) {
        if (code.empty() || !std::isfinite(value)) {
            return;
        }
        WriterContribution c;
        c.code = code;
        c.kind = kind;
        c.value = value;
        c.lli = lli;
        out.push_back(std::move(c));
    };
    if (obs.has_pseudorange) {
        push(code_of_range, kKindCode, obs.pseudorange, 0);
    }
    if (obs.has_carrier_phase) {
        push(code_of_phase, kKindPhase, obs.carrier_phase,
             (obs.lli & 0x07) | (obs.loss_of_lock ? 0x01 : 0));
    }
    if (obs.has_doppler) {
        push(code_of_aux, kKindDoppler, obs.doppler, 0);
    }
    if (obs.snr > 0.0) {
        push(code_of_aux, kKindSnr, obs.snr, 0);
    }
}

// Observation column order within a system: band, then the library's
// tracking-attribute priority, then the attribute letter.
bool trackingCodeLess(GNSSSystem system, const std::string& a, const std::string& b) {
    if (a[0] != b[0]) {
        return a[0] < b[0];
    }
    const int band = a[0] - '0';
    const int rank_a = signal_policy::trackingAttributeRank(system, band, a[1]);
    const int rank_b = signal_policy::trackingAttributeRank(system, band, b[1]);
    if (rank_a != rank_b) {
        return rank_a < rank_b;
    }
    return a[1] < b[1];
}

// RINEX 3.04 table A23 reference phase signals: no phase shift correction is
// ever needed, so SYS / PHASE SHIFT leaves the value blank (as RTKLIB does).
bool isReferencePhaseCode(GNSSSystem system, const std::string& code) {
    static const std::map<GNSSSystem, std::set<std::string>> kReference = {
        {GNSSSystem::GPS, {"1C", "2P", "5I"}},
        {GNSSSystem::GLONASS, {"1C", "4A", "2C", "6A", "3I"}},
        {GNSSSystem::Galileo, {"1B", "5I", "7I", "8I", "6B"}},
        {GNSSSystem::QZSS, {"1C", "2S", "5I", "5D", "6S"}},
        {GNSSSystem::SBAS, {"1C", "5I"}},
        {GNSSSystem::BeiDou, {"2I", "1D", "5D", "7I", "7D", "8D", "6I"}},
        {GNSSSystem::NavIC, {"5A", "9A"}},
    };
    const auto it = kReference.find(system);
    return it != kReference.end() && it->second.count(code) != 0;
}

std::string observationField(const WriterChannel* channel, int kind) {
    std::string field(16, ' ');
    if (channel == nullptr || !channel->has[kind]) {
        return field;
    }
    char value[48];
    std::snprintf(value, sizeof(value), "%14.3f", channel->value[kind]);
    if (std::strlen(value) != 14) {
        return field;  // not representable in F14.3
    }
    std::memcpy(&field[0], value, 14);
    if (kind == kKindPhase && channel->lli >= 1 && channel->lli <= 7) {
        field[14] = static_cast<char>('0' + channel->lli);
    }
    // The signal-strength indicator (last column) stays blank on purpose: the
    // S columns carry the exact C/N0.  Writing a 1-9 digit there would also
    // be read as a measurement std-dev by RTKLIB demo5.
    return field;
}

}  // namespace

std::string defaultRinexTrackingCode(GNSSSystem system, SignalType signal) {
    switch (system) {
        case GNSSSystem::GPS:
            switch (signal) {
                case SignalType::GPS_L1CA: return "1C";
                case SignalType::GPS_L1P: return "1W";
                case SignalType::GPS_L2P: return "2W";
                case SignalType::GPS_L2C: return "2X";
                case SignalType::GPS_L5: return "5X";
                default: return "";
            }
        case GNSSSystem::GLONASS:
            switch (signal) {
                case SignalType::GLO_L1CA: return "1C";
                case SignalType::GLO_L1P: return "1P";
                case SignalType::GLO_L2CA: return "2C";
                case SignalType::GLO_L2P: return "2P";
                default: return "";
            }
        case GNSSSystem::Galileo:
            switch (signal) {
                case SignalType::GAL_E1: return "1X";
                case SignalType::GAL_E5A: return "5X";
                case SignalType::GAL_E5B: return "7X";
                case SignalType::GAL_E6: return "6X";
                default: return "";
            }
        case GNSSSystem::BeiDou:
            switch (signal) {
                case SignalType::BDS_B1I: return "2I";
                case SignalType::BDS_B2I: return "7I";
                case SignalType::BDS_B3I: return "6I";
                case SignalType::BDS_B1C: return "1X";
                case SignalType::BDS_B2A: return "5X";
                default: return "";
            }
        case GNSSSystem::QZSS:
            switch (signal) {
                case SignalType::QZS_L1CA: return "1C";
                case SignalType::QZS_L2C: return "2X";
                case SignalType::QZS_L5: return "5X";
                default: return "";
            }
        case GNSSSystem::SBAS:
            switch (signal) {
                case SignalType::GPS_L1CA: return "1C";
                case SignalType::GPS_L5: return "5X";
                default: return "";
            }
        case GNSSSystem::NavIC:
            return signal == SignalType::GPS_L5 ? "5A" : "";
        default:
            return "";
    }
}

// Compact copy of everything the observation file needs, so the header can be
// written after the last epoch (observation types, first/last epoch).
struct RINEXWriter::ObservationBuffer {
    // key: epoch in 1e-7 s ticks since the GPS epoch (also merges/sorts epochs)
    std::map<std::int64_t, WriterSatChannels> epochs;
    // system -> tracking code -> which of C/L/D/S occur anywhere in the file
    std::map<GNSSSystem, std::map<std::string, std::array<bool, 4>>> used;
    std::map<SatelliteId, int> glonass_channels;
    Vector3d first_position = Vector3d::Zero();
    bool has_first_position = false;

    // Where the file goes, and when the last on-disk snapshot was taken.
    std::string path;
    double checkpoint_interval_s = 5.0;
    std::chrono::steady_clock::time_point last_checkpoint = std::chrono::steady_clock::now();
    double last_checkpoint_cost_s = 0.0;
    bool dirty = false;

    bool writeFile(std::ostream& file, const RINEXReader::RINEXHeader& header) const;
    // Write the complete file to <path>.tmp and move it over <path>, so the
    // file on disk is always a complete, valid RINEX file.
    bool writeSnapshot(const RINEXReader::RINEXHeader& header);
    // Snapshot when data is pending and the interval (backed off to at most
    // ~5% of the time spent writing) has elapsed.
    void checkpointIfDue(const RINEXReader::RINEXHeader& header);
};

bool RINEXWriter::ObservationBuffer::writeFile(
    std::ostream& file, const RINEXReader::RINEXHeader& header) const {
    std::string out;

    // RINEX VERSION / TYPE
    out += headerLine(formatString("%9.2f%11s%-20s%-20s", kWriterRinexVersion, "",
                                   "OBSERVATION DATA", "M: Mixed"),
                      "RINEX VERSION / TYPE");

    // PGM / RUN BY / DATE
    std::string date = trimCopy(header.date);
    if (date.empty()) {
        const std::time_t now = std::time(nullptr);
        std::tm utc{};
#ifdef _WIN32
        gmtime_s(&utc, &now);
#else
        gmtime_r(&now, &utc);
#endif
        date = formatString("%04d%02d%02d %02d%02d%02d UTC", utc.tm_year + 1900,
                            utc.tm_mon + 1, utc.tm_mday, utc.tm_hour, utc.tm_min,
                            utc.tm_sec);
    }
    const std::string program = trimCopy(header.program);
    out += headerLine(headerField(program.empty() ? "libgnss++" : program, 20) +
                          headerField(header.run_by, 20) + headerField(date, 20),
                      "PGM / RUN BY / DATE");

    const std::string marker = trimCopy(header.marker_name);
    out += headerLine(marker.empty() ? "UNKNOWN" : marker, "MARKER NAME");
    if (!trimCopy(header.marker_number).empty()) {
        out += headerLine(headerField(header.marker_number, 20), "MARKER NUMBER");
    }
    out += headerLine("", "MARKER TYPE");
    out += headerLine(headerField(header.observer, 20) + headerField(header.agency, 40),
                      "OBSERVER / AGENCY");
    out += headerLine(headerField(header.receiver_number, 20) +
                          headerField(header.receiver_type, 20) +
                          headerField(header.receiver_version, 20),
                      "REC # / TYPE / VERS");
    out += headerLine(headerField(header.antenna_number, 20) +
                          headerField(header.antenna_type, 20),
                      "ANT # / TYPE");

    Vector3d position = Vector3d::Zero();
    if (header.approximate_position.allFinite() &&
        (header.has_approximate_position || header.approximate_position.norm() > 0.0)) {
        position = header.approximate_position;
    } else if (has_first_position) {
        position = first_position;
    }
    out += headerLine(formatString("%14.4f%14.4f%14.4f", position.x(), position.y(),
                                   position.z()),
                      "APPROX POSITION XYZ");
    // RINEX order is H/E/N; RINEXHeader::antenna_delta is (east, north, height).
    const Vector3d delta =
        header.antenna_delta.allFinite() ? header.antenna_delta : Vector3d::Zero();
    out += headerLine(formatString("%14.4f%14.4f%14.4f", delta.z(), delta.x(), delta.y()),
                      "ANTENNA: DELTA H/E/N");

    // Observation types, in a fixed column order per system.
    std::map<GNSSSystem, std::vector<std::string>> columns;  // "C1C", "L1C", ...
    std::map<GNSSSystem, std::vector<std::string>> phase_codes;
    for (const auto& [system, codes] : used) {
        char sys_char = 'G';
        if (!writerSystemChar(system, sys_char)) {
            continue;
        }
        std::vector<std::string> sorted_codes;
        for (const auto& entry : codes) {
            sorted_codes.push_back(entry.first);
        }
        std::sort(sorted_codes.begin(), sorted_codes.end(),
                  [system](const std::string& a, const std::string& b) {
                      return trackingCodeLess(system, a, b);
                  });
        auto& types = columns[system];
        for (const auto& code : sorted_codes) {
            const auto& kinds = codes.at(code);
            for (int kind = 0; kind < 4; ++kind) {
                if (kinds[kind]) {
                    types.push_back(std::string(1, kKindChar[kind]) + code);
                }
            }
            if (kinds[kKindPhase]) {
                phase_codes[system].push_back(code);
            }
        }
        std::string line;
        for (std::size_t i = 0; i < types.size(); ++i) {
            if (i % 13 == 0) {
                if (i != 0) {
                    out += headerLine(line, "SYS / # / OBS TYPES");
                }
                line = i == 0 ? formatString("%c  %3d", sys_char,
                                             static_cast<int>(types.size()))
                              : std::string(6, ' ');
            }
            line += " " + types[i];
        }
        if (!types.empty()) {
            out += headerLine(line, "SYS / # / OBS TYPES");
        }
    }

    if (header.interval > 0.0) {
        out += headerLine(formatString("%10.3f", header.interval), "INTERVAL");
    }
    if (!epochs.empty()) {
        const auto time_line = [](std::int64_t ticks, const char* label) {
            const WriterCalendar cal = writerCalendarFromTicks(ticks);
            return headerLine(
                formatString("%6d%6d%6d%6d%6d%s     %-3s", cal.year, cal.month, cal.day,
                             cal.hour, cal.minute, writerSecondsField(cal, 13).c_str(),
                             "GPS"),
                label);
        };
        out += time_line(epochs.begin()->first, "TIME OF FIRST OBS");
        out += time_line(epochs.rbegin()->first, "TIME OF LAST OBS");
    }

    // SYS / PHASE SHIFT: no correction applied by this writer.
    for (const auto& [system, codes] : phase_codes) {
        char sys_char = 'G';
        writerSystemChar(system, sys_char);
        for (const auto& code : codes) {
            const std::string obs_code = "L" + code;
            out += headerLine(
                isReferencePhaseCode(system, code)
                    ? formatString("%c %3s %8s", sys_char, obs_code.c_str(), "")
                    : formatString("%c %3s %8.5f", sys_char, obs_code.c_str(), 0.0),
                "SYS / PHASE SHIFT");
        }
    }

    // GLONASS SLOT / FRQ #: header entries first, observation-carried
    // channels override them.
    std::map<SatelliteId, int> fcn;
    for (const auto& [sat, channel] : header.glonass_frequency_channels) {
        if (sat.system == GNSSSystem::GLONASS && channel >= -7 && channel <= 6 &&
            sat.prn >= 1 && sat.prn <= 99) {
            fcn[sat] = channel;
        }
    }
    for (const auto& [sat, channel] : glonass_channels) {
        fcn[sat] = channel;
    }
    if (!fcn.empty()) {
        std::string line;
        std::size_t index = 0;
        for (const auto& [sat, channel] : fcn) {
            if (index % 8 == 0) {
                if (index != 0) {
                    out += headerLine(line, "GLONASS SLOT / FRQ #");
                }
                line = index == 0 ? formatString("%3d ", static_cast<int>(fcn.size()))
                                  : std::string(4, ' ');
            }
            line += formatString("R%02d %2d ", static_cast<int>(sat.prn), channel);
            ++index;
        }
        out += headerLine(line, "GLONASS SLOT / FRQ #");
    }
    out += headerLine("", "END OF HEADER");

    // Epochs.
    for (const auto& [ticks, sats] : epochs) {
        std::vector<std::pair<SatelliteId, const std::vector<WriterChannel>*>> rows;
        for (const auto& [sat, channels] : sats) {
            char sys_char = 'G';
            int number = 0;
            if (writerSystemChar(sat.system, sys_char) &&
                writerSatelliteNumber(sat, number) && columns.count(sat.system) != 0) {
                rows.emplace_back(sat, &channels);
            }
        }
        if (rows.empty()) {
            continue;
        }
        const WriterCalendar cal = writerCalendarFromTicks(ticks);
        out += formatString("> %04d %02d %02d %02d %02d%s  %d%3d\n", cal.year, cal.month,
                            cal.day, cal.hour, cal.minute,
                            writerSecondsField(cal, 11).c_str(), 0,
                            static_cast<int>(rows.size()));
        for (const auto& [sat, channels] : rows) {
            char sys_char = 'G';
            int number = 0;
            writerSystemChar(sat.system, sys_char);
            writerSatelliteNumber(sat, number);
            out += formatString("%c%02d", sys_char, number);
            for (const auto& type : columns.at(sat.system)) {
                const int kind = type[0] == 'C'   ? kKindCode
                                 : type[0] == 'L' ? kKindPhase
                                 : type[0] == 'D' ? kKindDoppler
                                                  : kKindSnr;
                const std::string code = type.substr(1);
                const WriterChannel* channel = nullptr;
                for (const auto& candidate : *channels) {
                    if (candidate.code == code) {
                        channel = &candidate;
                        break;
                    }
                }
                out += observationField(channel, kind);
            }
            out += "\n";
        }
        if (out.size() > (1U << 20)) {
            file.write(out.data(), static_cast<std::streamsize>(out.size()));
            out.clear();
        }
    }
    file.write(out.data(), static_cast<std::streamsize>(out.size()));
    return file.good();
}

bool RINEXWriter::ObservationBuffer::writeSnapshot(const RINEXReader::RINEXHeader& header) {
    const std::string temporary = path + ".tmp";
    {
        std::ofstream file(temporary, std::ios::binary | std::ios::trunc);
        if (!file.is_open() || !writeFile(file, header)) {
            return false;
        }
        file.flush();
        if (!file.good()) {
            return false;
        }
    }
    std::error_code error;
    std::filesystem::rename(temporary, path, error);
    if (error) {
        std::filesystem::remove(path, error);
        std::filesystem::rename(temporary, path, error);
    }
    dirty = !!error;
    return !error;
}

void RINEXWriter::ObservationBuffer::checkpointIfDue(const RINEXReader::RINEXHeader& header) {
    if (!dirty || checkpoint_interval_s < 0.0) {
        return;
    }
    const auto start = std::chrono::steady_clock::now();
    const double since_last = std::chrono::duration<double>(start - last_checkpoint).count();
    if (since_last < std::max(checkpoint_interval_s, 20.0 * last_checkpoint_cost_s)) {
        return;
    }
    writeSnapshot(header);  // failures resurface in close()
    const auto end = std::chrono::steady_clock::now();
    last_checkpoint_cost_s = std::chrono::duration<double>(end - start).count();
    last_checkpoint = end;
}

RINEXWriter::RINEXWriter() = default;

RINEXWriter::~RINEXWriter() {
    close();
}

bool RINEXWriter::createObservationFile(const std::string& filename, const RINEXReader::RINEXHeader& header) {
    close();
    {
        // Fail early when the path is not writable; this also truncates any
        // previous file, as opening for output always did.
        std::ofstream probe(filename, std::ios::binary | std::ios::trunc);
        if (!probe.is_open()) {
            return false;
        }
    }

    header_ = header;
    header_.file_type = RINEXReader::FileType::OBSERVATION;
    observation_buffer_ = std::make_unique<ObservationBuffer>();
    observation_buffer_->path = filename;
    observation_buffer_->checkpoint_interval_s = checkpoint_interval_s_;
    // The file is completed by close() (and snapshotted periodically), once
    // the observation types are known.
    return true;
}

void RINEXWriter::setCheckpointInterval(double seconds) {
    checkpoint_interval_s_ = seconds;
    if (observation_buffer_) {
        observation_buffer_->checkpoint_interval_s = seconds;
    }
}

bool RINEXWriter::writeObservationEpoch(const ObservationData& obs_data) {
    if (!observation_buffer_ || !std::isfinite(obs_data.time.tow)) {
        return false;
    }
    ObservationBuffer& buffer = *observation_buffer_;

    std::map<SatelliteId, std::vector<WriterContribution>> contributions;
    std::set<std::tuple<SatelliteId, std::string, int>> present;
    char sys_char = 'G';
    for (const auto& obs : obs_data.observations) {
        if (!writerSystemChar(obs.satellite.system, sys_char)) {
            continue;
        }
        auto& list = contributions[obs.satellite];
        const std::size_t before = list.size();
        collectContributions(obs, nullptr, list);
        for (std::size_t i = before; i < list.size(); ++i) {
            present.emplace(obs.satellite, list[i].code, list[i].kind);
        }
        if (obs.satellite.system == GNSSSystem::GLONASS &&
            obs.has_glonass_frequency_channel &&
            obs.glonass_frequency_channel >= -7 && obs.glonass_frequency_channel <= 6) {
            buffer.glonass_channels[obs.satellite] = obs.glonass_frequency_channel;
        }
    }
    // Further tracking codes of satellites that are in the epoch (the reader
    // and the RTCM decoder keep every tracking code of a band there).  The
    // policy-selected observations above win on conflicts.
    for (const auto& [key, obs] : obs_data.rinex_tracking_observations) {
        const auto sat_it = contributions.find(key.first);
        if (sat_it == contributions.end() ||
            trackingCodeFromObservationType("C" + key.second).empty()) {
            continue;
        }
        std::vector<WriterContribution> extra;
        collectContributions(obs, &key.second, extra);
        for (auto& c : extra) {
            if (present.emplace(key.first, c.code, c.kind).second) {
                sat_it->second.push_back(std::move(c));
            }
        }
    }

    const bool any = std::any_of(contributions.begin(), contributions.end(),
                                 [](const auto& entry) { return !entry.second.empty(); });
    if (!any) {
        return true;
    }
    if (!buffer.has_first_position && obs_data.receiver_position.allFinite() &&
        obs_data.receiver_position.norm() > 0.0) {
        buffer.first_position = obs_data.receiver_position;
        buffer.has_first_position = true;
    }

    WriterSatChannels& sats = buffer.epochs[writerEpochTicks(obs_data.time)];
    for (const auto& [sat, list] : contributions) {
        if (list.empty()) {
            continue;
        }
        std::vector<WriterChannel>& channels = sats[sat];
        for (const auto& c : list) {
            auto it = std::find_if(channels.begin(), channels.end(),
                                   [&c](const WriterChannel& ch) { return ch.code == c.code; });
            if (it == channels.end()) {
                channels.emplace_back();
                it = std::prev(channels.end());
                it->code = c.code;
            }
            it->has[c.kind] = true;
            it->value[c.kind] = c.value;
            if (c.kind == kKindPhase) {
                it->lli = c.lli;
            }
            buffer.used[sat.system][c.code][c.kind] = true;
        }
    }
    buffer.dirty = true;
    buffer.checkpointIfDue(header_);
    return true;
}

bool RINEXWriter::close() {
    bool ok = true;
    if (observation_buffer_) {
        ok = observation_buffer_->writeSnapshot(header_);
        observation_buffer_.reset();
    }
    if (file_.is_open()) {
        file_.flush();
        ok = ok && file_.good();
        file_.close();
    }
    return ok;
}

bool RINEXWriter::writeHeader(const RINEXReader::RINEXHeader& header) {
    if (!file_.is_open()) {
        return false;
    }
    
    // RINEX VERSION / TYPE: F9.2, 11X, A20 file type, A20 satellite system
    // (columns 1-60), so readers find the version and the label in place.
    file_ << std::fixed << std::setprecision(2) << std::setw(9) << header.version
          << std::string(11, ' ');
    if (header.file_type == RINEXReader::FileType::NAVIGATION) {
        file_ << "NAVIGATION DATA     ";
    } else {
        file_ << "OBSERVATION DATA    ";
    }
    std::string satellite_system = header.satellite_system.substr(0, 20);
    satellite_system.resize(20, ' ');
    file_ << satellite_system << "RINEX VERSION / TYPE\n";
    
    file_ << "LibGNSS++           User                ";
    file_ << "20240101 000000 UTC PGM / RUN BY / DATE\n";
    
    file_ << "                                                            END OF HEADER\n";
    
    return true;
}

bool RINEXWriter::createNavigationFile(const std::string& filename, const RINEXReader::RINEXHeader& header) {
    close();
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

namespace {

// Galileo "data sources" word of a RINEX navigation record. Decoders record
// the pages the ephemeris came from in Ephemeris::data_source_code; an
// ephemeris that only knows its message type gets that type's canonical word.
int galileoRinexDataSources(const Ephemeris& eph) {
    if (eph.data_source_code != 0) {
        return eph.data_source_code;
    }
    switch (eph.navigation_message_type) {
        case NavigationMessageType::INAV:
            return galileo_data_source::kInavE1B | galileo_data_source::kClockE5bE1;
        case NavigationMessageType::FNAV:
            return galileo_data_source::kFnavE5aI | galileo_data_source::kClockE5aE1;
        default:
            return 0;
    }
}

}  // namespace

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
    // Galileo: "data sources" word (RINEX 3.0x Table A8 / 4.0x Table A10);
    // the SV accuracy field holds SISA in metres (Ephemeris::sv_accuracy).
    const double line5_col2 =
        is_galileo ? static_cast<double>(galileoRinexDataSources(eph)) : 0.0;

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
          << formatRinexFloat(line5_col2)
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

} // namespace io
} // namespace libgnss
