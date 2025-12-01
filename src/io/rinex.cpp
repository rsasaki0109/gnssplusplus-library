#include <libgnss++/io/rinex.hpp>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <iostream>

namespace libgnss {
namespace io {

bool RINEXReader::open(const std::string& filename) {
    file_.open(filename);
    current_line_ = 0;
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

    // Skip header if not already done
    std::string line;
    bool header_found = false;
    while (readLine(line)) {
        if (line.find("END OF HEADER") != std::string::npos) {
            header_found = true;
            break;
        }
    }

    if (!header_found) {
        // Header already processed, rewind might not work, so just continue
    }

    std::vector<std::string> eph_lines;
    int eph_detected = 0;
    int eph_parsed = 0;

    try {
        while (readLine(line)) {
            // Skip very short lines but allow shorter navigation data lines (last line of ephemeris can be short)
            if (line.length() < 19) continue;

            // Check if this is start of new ephemeris
            // Ephemeris records start with "PRN YY MM DD..." where:
            // - PRN is 1-2 digits (" 1", "11")
            // - YY is 2-digit year at position 3-4
            // Data lines start with spaces and numbers but not this pattern
            bool is_new_ephemeris = false;
            if (line.length() >= 5) {
                // Check if positions 3-4 look like a year (digits)
                bool has_year = std::isdigit(line[3]) && std::isdigit(line[4]);
                // Check if starts with PRN format
                bool has_prn = (line[0] == ' ' && std::isdigit(line[1])) ||  // Single digit
                               (std::isdigit(line[0]) && std::isdigit(line[1]));  // Double digit
                is_new_ephemeris = has_prn && has_year;
            }

            if (is_new_ephemeris) {
                eph_detected++;
                // Process previous ephemeris if exists
                if (!eph_lines.empty()) {
                    Ephemeris eph;
                    if (parseNavigationMessage(eph_lines, eph)) {
                        nav_data.addEphemeris(eph);
                        eph_parsed++;
                    }
                    eph_lines.clear();
                }
            }

            eph_lines.push_back(line);
        }

        // Process last ephemeris
        if (!eph_lines.empty()) {
            Ephemeris eph;
            if (parseNavigationMessage(eph_lines, eph)) {
                nav_data.addEphemeris(eph);
                eph_parsed++;
            }
        }

        std::cerr << "DEBUG: Detected " << eph_detected << " ephemeris records, parsed " << eph_parsed << std::endl;
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
    else if (label.find("APPROX POSITION XYZ") != std::string::npos) {
        header.approximate_position(0) = std::stod(line.substr(0, 14));
        header.approximate_position(1) = std::stod(line.substr(14, 14));
        header.approximate_position(2) = std::stod(line.substr(28, 14));
    }
    else if (label.find("# / TYPES OF OBSERV") != std::string::npos) {
        // Parse number of observation types
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

        // Create combined observations for each signal
        // Group L1/C1 together and L2/P2 together
        Observation obs_l1;
        obs_l1.satellite = sat;
        obs_l1.signal = SignalType::GPS_L1CA;
        obs_l1.valid = true;
        bool has_l1_data = false;

        Observation obs_l2;
        obs_l2.satellite = sat;
        obs_l2.signal = SignalType::GPS_L2C;
        obs_l2.valid = true;
        bool has_l2_data = false;

        for (size_t i = 0; i < header_.observation_types.size() && i < obs_values.size(); ++i) {
            if (obs_values[i] == 0.0) continue;

            const std::string& obs_type = header_.observation_types[i];

            if (obs_type == "L1") {
                obs_l1.carrier_phase = obs_values[i];
                obs_l1.has_carrier_phase = true;
                obs_l1.lli = lli_flags[i];
                obs_l1.loss_of_lock = (lli_flags[i] & 0x01) != 0;
                has_l1_data = true;
            } else if (obs_type == "C1" || obs_type == "P1") {
                obs_l1.pseudorange = obs_values[i];
                obs_l1.has_pseudorange = true;
                obs_l1.signal_strength = signal_strength[i];
                has_l1_data = true;
            } else if (obs_type == "L2") {
                obs_l2.carrier_phase = obs_values[i];
                obs_l2.has_carrier_phase = true;
                obs_l2.lli = lli_flags[i];
                obs_l2.loss_of_lock = (lli_flags[i] & 0x01) != 0;
                has_l2_data = true;
            } else if (obs_type == "P2" || obs_type == "C2") {
                obs_l2.pseudorange = obs_values[i];
                obs_l2.has_pseudorange = true;
                obs_l2.signal_strength = signal_strength[i];
                has_l2_data = true;
            }
        }

        // Add observations if they have data
        if (has_l1_data) {
            obs_data.addObservation(obs_l1);
        }
        if (has_l2_data) {
            obs_data.addObservation(obs_l2);
        }
    }

    return true;
}

bool RINEXReader::parseObservationEpochV3(const std::string& line, ObservationData& obs_data) {
    // Simplified RINEX 3.x parsing
    return parseObservationEpochV2(line, obs_data);
}

bool RINEXReader::parseNavigationMessage(const std::vector<std::string>& lines, Ephemeris& eph) {
    if (lines.size() < 8) {
        static int debug_count = 0;
        if (debug_count++ < 2) {
            std::cerr << "DEBUG: Skipping ephemeris with " << lines.size() << " lines (need 8)" << std::endl;
        }
        return false;  // Need 8 lines for GPS ephemeris
    }

    const std::string& first_line = lines[0];
    if (first_line.length() < 3) return false;

    try {
        // Parse satellite ID - RINEX 2.x navigation format: "sPRN YY MM DD ..."
        std::string sat_id_str = first_line.substr(0, 3);
        sat_id_str.erase(0, sat_id_str.find_first_not_of(' '));
        size_t space_pos = sat_id_str.find(' ');
        if (space_pos != std::string::npos) {
            sat_id_str = sat_id_str.substr(0, space_pos);
        }
        if (sat_id_str.empty()) return false;
        eph.satellite = SatelliteId(GNSSSystem::GPS, std::stoi(sat_id_str));

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

        // Line 0: PRN, Epoch, af0, af1, af2
        eph.af0 = parseD(first_line, 22, 19);  // Clock bias
        eph.af1 = parseD(first_line, 41, 19);  // Clock drift
        eph.af2 = parseD(first_line, 60, 19);  // Clock drift rate

        // Line 1: IODE, Crs, delta_n, M0
        double iode = parseD(lines[1], 3, 19);
        eph.crs = parseD(lines[1], 22, 19);
        eph.delta_n = parseD(lines[1], 41, 19);
        eph.m0 = parseD(lines[1], 60, 19);

        // Line 2: Cuc, e, Cus, sqrt(A)
        eph.cuc = parseD(lines[2], 3, 19);
        eph.e = parseD(lines[2], 22, 19);
        eph.cus = parseD(lines[2], 41, 19);
        eph.sqrt_a = parseD(lines[2], 60, 19);

        // Line 3: Toe, Cic, OMEGA0, Cis
        double toe_seconds = parseD(lines[3], 3, 19);

        // Debug line 3 parsing
        static int line3_debug = 0;
        if (line3_debug < 2) {
            std::cerr << "DEBUG Line3: '" << lines[3] << "'" << std::endl;
            std::cerr << "DEBUG Line3: toe_seconds=" << toe_seconds << std::endl;
            line3_debug++;
        }

        eph.cic = parseD(lines[3], 22, 19);
        eph.omega0 = parseD(lines[3], 41, 19);
        eph.cis = parseD(lines[3], 60, 19);

        // Line 4: i0, Crc, omega, OMEGA_DOT
        eph.i0 = parseD(lines[4], 3, 19);
        eph.crc = parseD(lines[4], 22, 19);
        eph.omega = parseD(lines[4], 41, 19);
        eph.omega_dot = parseD(lines[4], 60, 19);

        // Line 5: IDOT, L2 codes, GPS week, L2 P flag
        eph.i_dot = parseD(lines[5], 3, 19);
        double l2_codes = parseD(lines[5], 22, 19);
        double gps_week = parseD(lines[5], 41, 19);
        double l2_flag = parseD(lines[5], 60, 19);

        // Line 6: SV accuracy, SV health, TGD, IODC
        eph.sv_accuracy = parseD(lines[6], 3, 19);
        eph.sv_health = parseD(lines[6], 22, 19);
        eph.tgd = parseD(lines[6], 41, 19);
        double iodc = parseD(lines[6], 60, 19);

        // Line 7: Transmission time, fit interval
        double transmission_time = parseD(lines[7], 3, 19);

        // Set times
        int week = static_cast<int>(gps_week);

        // Debug: print first few ephemeris TOE
        static int eph_count = 0;
        if (eph_count < 3) {
            std::cerr << "DEBUG: Ephemeris " << eph_count << " - PRN=" << eph.satellite.prn
                      << ", week=" << week << ", toe=" << toe_seconds << std::endl;
            eph_count++;
        }

        eph.toe = GNSSTime(week, toe_seconds);
        eph.toc = eph.toe;  // Simplified: use toe as toc

        eph.valid = true;
        eph.iode = static_cast<int>(iode);
        eph.iodc = static_cast<int>(iodc);

    } catch (const std::exception& e) {
        std::cerr << "Error parsing ephemeris: " << e.what() << std::endl;
        return false;
    }

    return true;
}

GNSSTime RINEXReader::parseTime(const std::string& time_str, double version) {
    // Simplified time parsing
    (void)time_str;
    (void)version;
    return GNSSTime(2000, 345600.0);
}

SatelliteId RINEXReader::parseSatelliteId(const std::string& sat_str, double version) {
    // Simplified satellite ID parsing
    (void)version;
    if (sat_str.length() >= 2) {
        std::string prn_str = sat_str.substr(0, 2);
        prn_str.erase(0, prn_str.find_first_not_of(' '));
        if (!prn_str.empty()) {
            int prn = std::stoi(prn_str);
            return SatelliteId(GNSSSystem::GPS, prn);
        }
    }
    return SatelliteId(GNSSSystem::GPS, 1);
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
    file_ << "           OBSERVATION DATA    ";
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

void RINEXWriter::close() {
    if (file_.is_open()) {
        file_.close();
    }
}

} // namespace io
} // namespace libgnss
