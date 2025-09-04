#include <libgnss++/io/rinex.hpp>
#include <fstream>
#include <sstream>
#include <iomanip>

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
    if (!readLine(line)) {
        return false;
    }
    
    // Parse epoch header based on RINEX version
    if (header_.version >= 3.0) {
        return parseObservationEpochV3(line, obs_data);
    } else {
        return parseObservationEpochV2(line, obs_data);
    }
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
    
    std::string line;
    std::vector<std::string> eph_lines;
    
    while (readLine(line)) {
        if (line.length() < 60) continue;
        
        // Check if this is start of new ephemeris
        if (line[0] != ' ') {
            // Process previous ephemeris if exists
            if (!eph_lines.empty()) {
                Ephemeris eph;
                if (parseNavigationMessage(eph_lines, eph)) {
                    nav_data.addEphemeris(eph);
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
        }
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
    
    return true;
}

bool RINEXReader::parseObservationEpochV2(const std::string& line, ObservationData& obs_data) {
    // Simplified RINEX 2.x parsing
    if (line.length() < 26) return false;
    
    // Parse time
    // int year = std::stoi(line.substr(1, 2));
    // int month = std::stoi(line.substr(4, 2));
    // int day = std::stoi(line.substr(7, 2));
    int hour = std::stoi(line.substr(10, 2));
    int minute = std::stoi(line.substr(13, 2));
    double second = std::stod(line.substr(15, 11));
    
    // Convert to GPS time (simplified)
    obs_data.time = GNSSTime(2000, hour * 3600 + minute * 60 + second);
    
    // Parse number of satellites
    int num_sats = std::stoi(line.substr(29, 3));
    
    // Read satellite observations (simplified)
    for (int i = 0; i < num_sats; ++i) {
        Observation obs;
        obs.satellite = SatelliteId(GNSSSystem::GPS, i + 1);
        obs.signal = SignalType::GPS_L1CA;
        obs.pseudorange = 20000000.0 + i * 1000.0; // Dummy data
        obs.valid = true;
        obs_data.addObservation(obs);
    }
    
    return true;
}

bool RINEXReader::parseObservationEpochV3(const std::string& line, ObservationData& obs_data) {
    // Simplified RINEX 3.x parsing
    return parseObservationEpochV2(line, obs_data);
}

bool RINEXReader::parseNavigationMessage(const std::vector<std::string>& lines, Ephemeris& eph) {
    if (lines.empty()) return false;
    
    // Simplified ephemeris parsing
    const std::string& first_line = lines[0];
    if (first_line.length() < 3) return false;
    
    // Parse satellite ID
    eph.satellite = SatelliteId(GNSSSystem::GPS, std::stoi(first_line.substr(0, 2)));
    
    // Set dummy ephemeris data
    eph.toe = GNSSTime(2000, 345600.0);
    eph.toc = GNSSTime(2000, 345600.0);
    eph.sqrt_a = 5153.0;
    eph.e = 0.01;
    eph.i0 = 0.97;
    eph.omega0 = 0.0;
    eph.omega = 0.0;
    eph.m0 = 0.0;
    eph.af0 = 1e-6;
    eph.af1 = 0.0;
    eph.af2 = 0.0;
    eph.valid = true;
    
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
        int prn = std::stoi(sat_str.substr(0, 2));
        return SatelliteId(GNSSSystem::GPS, prn);
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
