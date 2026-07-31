#include <libgnss++/core/solution.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <utility>

namespace libgnss {
namespace {

constexpr std::size_t kMandatorySolutionColumns = 11;

Matrix3d ecefToEnuRotation(double latitude_rad, double longitude_rad) {
    Matrix3d rotation;
    rotation.col(0) = ecef2enu(Vector3d::UnitX(), latitude_rad, longitude_rad);
    rotation.col(1) = ecef2enu(Vector3d::UnitY(), latitude_rad, longitude_rad);
    rotation.col(2) = ecef2enu(Vector3d::UnitZ(), latitude_rad, longitude_rad);
    return rotation;
}

std::vector<std::string> parseSolutionColumnsHeader(const std::string& line) {
    const std::string columns_marker = "% Columns:";
    std::string column_text;

    if (line.rfind(columns_marker, 0) == 0) {
        column_text = line.substr(columns_marker.size());
    } else if (!line.empty() && line[0] == '%') {
        column_text = line.substr(1);
        std::istringstream probe(column_text);
        std::string first_column;
        probe >> first_column;
        if (first_column != "GPS_Week") {
            return {};
        }
    } else {
        return {};
    }

    std::istringstream column_stream(column_text);
    std::vector<std::string> columns;
    std::string column;
    while (column_stream >> column) {
        columns.push_back(column);
    }
    return columns;
}

int optionalColumnToInt(double value) {
    return static_cast<int>(std::lround(value));
}

void applyOptionalSolutionColumns(PositionSolution& sol,
                                  const std::vector<std::string>& columns,
                                  const std::vector<double>& values) {
    if (values.empty()) {
        return;
    }

    if (columns.empty()) {
        sol.ratio = values[0];
        if (values.size() > 1) {
            sol.num_fixed_ambiguities = optionalColumnToInt(values[1]);
        }
        if (values.size() > 2) {
            sol.iterations = optionalColumnToInt(values[2]);
        }
        return;
    }

    for (std::size_t offset = 0; offset < values.size(); ++offset) {
        const std::size_t column_index = kMandatorySolutionColumns + offset;
        if (column_index >= columns.size()) {
            continue;
        }

        const std::string& name = columns[column_index];
        const double value = values[offset];
        if (name == "Ratio") {
            sol.ratio = value;
        } else if (name == "FixedAmbiguities") {
            sol.num_fixed_ambiguities = optionalColumnToInt(value);
        } else if (name == "Iterations" || name == "RTKIter" || name == "RtkIter") {
            sol.iterations = optionalColumnToInt(value);
        } else if (name == "Baseline(m)") {
            sol.baseline_length = value;
        } else if (name == "RTKObs" || name == "RtkObs") {
            sol.rtk_update_observations = optionalColumnToInt(value);
        } else if (name == "RTKPhaseObs" || name == "RtkPhaseObs") {
            sol.rtk_update_phase_observations = optionalColumnToInt(value);
        } else if (name == "RTKCodeObs" || name == "RtkCodeObs") {
            sol.rtk_update_code_observations = optionalColumnToInt(value);
        } else if (name == "RTKOutliers" || name == "RtkOutliers") {
            sol.rtk_update_suppressed_outliers = optionalColumnToInt(value);
        } else if (name == "RTKPrefitRMS(m)") {
            sol.rtk_update_prefit_residual_rms_m = value;
        } else if (name == "RTKPrefitMax(m)") {
            sol.rtk_update_prefit_residual_max_m = value;
        } else if (name == "RTKPostSuppressRMS(m)") {
            sol.rtk_update_post_suppression_residual_rms_m = value;
        } else if (name == "RTKPostSuppressMax(m)") {
            sol.rtk_update_post_suppression_residual_max_m = value;
        } else if (name == "RTKUpdateNIS") {
            sol.rtk_update_normalized_innovation_squared = value;
        } else if (name == "RTKUpdateNISPerObs") {
            sol.rtk_update_normalized_innovation_squared_per_observation = value;
        } else if (name == "RTKUpdateNISRejected") {
            sol.rtk_update_rejected_by_innovation_gate = optionalColumnToInt(value);
        }
    }
}

bool isLeapYear(int year) {
    return (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
}

GNSSTime gpstCalendarToTime(int year, int month, int day, int hour, int minute,
                            double second) {
    if (year < 1980 || month < 1 || month > 12 || day < 1 || hour < 0 ||
        hour > 23 || minute < 0 || minute > 59 || !std::isfinite(second) ||
        second < 0.0 || second >= 61.0) {
        return {};
    }

    int days_since_gps_epoch = 0;
    for (int current_year = 1980; current_year < year; ++current_year) {
        days_since_gps_epoch += isLeapYear(current_year) ? 366 : 365;
    }
    const int days_in_month[] = {31, 28, 31, 30, 31, 30,
                                 31, 31, 30, 31, 30, 31};
    for (int current_month = 1; current_month < month; ++current_month) {
        days_since_gps_epoch +=
            current_month == 2 && isLeapYear(year)
                ? 29
                : days_in_month[current_month - 1];
    }
    days_since_gps_epoch += day - 6;  // 1980-01-06 is GPS day zero.

    const int gps_week = days_since_gps_epoch / 7;
    const int day_of_week = days_since_gps_epoch % 7;
    return GNSSTime(gps_week,
                    static_cast<double>(day_of_week) * 86400.0 +
                        static_cast<double>(hour) * 3600.0 +
                        static_cast<double>(minute) * 60.0 + second);
}

bool parseDateToken(const std::string& token, int& year, int& month, int& day) {
    char slash1 = '\0';
    char slash2 = '\0';
    std::istringstream input(token);
    return static_cast<bool>(input >> year >> slash1 >> month >> slash2 >> day) &&
           slash1 == '/' && slash2 == '/' && input.peek() == EOF;
}

bool parseTimeToken(const std::string& token, int& hour, int& minute, double& second) {
    char colon1 = '\0';
    char colon2 = '\0';
    std::istringstream input(token);
    return static_cast<bool>(input >> hour >> colon1 >> minute >> colon2 >> second) &&
           colon1 == ':' && colon2 == ':' && input.peek() == EOF;
}

double signedSquare(double value) {
    return std::copysign(value * value, value);
}

SolutionStatus rtklibQualityToStatus(int quality) {
    switch (quality) {
        case 1: return SolutionStatus::FIXED;
        case 2: return SolutionStatus::FLOAT;
        case 4: return SolutionStatus::DGPS;
        case 5: return SolutionStatus::SPP;
        case 6: return SolutionStatus::PPP_FLOAT;
        default: return SolutionStatus::NONE;
    }
}

bool parseRtklibSolutionLine(const std::string& line, PositionSolution& sol) {
    std::istringstream input(line);
    std::string date_token;
    std::string time_token;
    if (!(input >> date_token >> time_token)) {
        return false;
    }

    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    double second = 0.0;
    if (!parseDateToken(date_token, year, month, day) ||
        !parseTimeToken(time_token, hour, minute, second)) {
        return false;
    }

    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double height_m = 0.0;
    double quality_value = 0.0;
    double satellite_value = 0.0;
    double sdn = 0.0;
    double sde = 0.0;
    double sdu = 0.0;
    double sdne = 0.0;
    double sdeu = 0.0;
    double sdun = 0.0;
    double age = 0.0;
    double ratio = 0.0;
    if (!(input >> latitude_deg >> longitude_deg >> height_m >> quality_value >>
          satellite_value >> sdn >> sde >> sdu >> sdne >> sdeu >> sdun >> age >>
          ratio)) {
        return false;
    }

    constexpr double kDegreesToRadians = M_PI / 180.0;
    const double latitude_rad = latitude_deg * kDegreesToRadians;
    const double longitude_rad = longitude_deg * kDegreesToRadians;
    sol.time = gpstCalendarToTime(year, month, day, hour, minute, second);
    sol.position_geodetic =
        GeodeticCoord(latitude_rad, longitude_rad, height_m);
    sol.position_ecef = geodetic2ecef(latitude_rad, longitude_rad, height_m);
    sol.status = rtklibQualityToStatus(static_cast<int>(std::lround(quality_value)));
    sol.num_satellites = static_cast<int>(std::lround(satellite_value));
    sol.ratio = ratio;

    // RTKLIB reports N/E/U standard deviations and signed square roots of
    // covariance. Convert the full local covariance to ECEF, matching the
    // PositionSolution contract consumed by LooseCouplingProcessor.
    Matrix3d covariance_neu;
    covariance_neu << sdn * sdn, signedSquare(sdne), signedSquare(sdun),
        signedSquare(sdne), sde * sde, signedSquare(sdeu),
        signedSquare(sdun), signedSquare(sdeu), sdu * sdu;
    Matrix3d covariance_enu;
    const Eigen::PermutationMatrix<3> neu_to_enu(Eigen::Vector3i(1, 0, 2));
    covariance_enu = neu_to_enu * covariance_neu * neu_to_enu.transpose();
    const Matrix3d ecef_to_enu = ecefToEnuRotation(latitude_rad, longitude_rad);
    sol.position_covariance =
        ecef_to_enu.transpose() * covariance_enu * ecef_to_enu;

    double vn = 0.0;
    double ve = 0.0;
    double vu = 0.0;
    double sdvn = 0.0;
    double sdve = 0.0;
    double sdvu = 0.0;
    double sdvne = 0.0;
    double sdveu = 0.0;
    double sdvun = 0.0;
    if (input >> vn >> ve >> vu >> sdvn >> sdve >> sdvu >> sdvne >> sdveu >>
        sdvun) {
        sol.velocity_ned = Vector3d(vn, ve, -vu);
        const Vector3d velocity_enu(ve, vn, vu);
        sol.velocity_ecef = ecef_to_enu.transpose() * velocity_enu;

        Matrix3d velocity_covariance_neu;
        velocity_covariance_neu
            << sdvn * sdvn, signedSquare(sdvne), signedSquare(sdvun),
            signedSquare(sdvne), sdve * sdve, signedSquare(sdveu),
            signedSquare(sdvun), signedSquare(sdveu), sdvu * sdvu;
        const Matrix3d velocity_covariance_enu =
            neu_to_enu * velocity_covariance_neu * neu_to_enu.transpose();
        sol.velocity_covariance =
            ecef_to_enu.transpose() * velocity_covariance_enu * ecef_to_enu;
        sol.has_velocity = true;
    }
    return sol.position_ecef.allFinite() && sol.position_ecef.norm() > 1e6;
}

} // namespace

double PositionSolution::getHorizontalAccuracy() const {
    // 95% confidence (2-sigma)
    double sigma_h = std::sqrt(position_covariance(0,0) + position_covariance(1,1));
    return 2.0 * sigma_h;
}

double PositionSolution::getVerticalAccuracy() const {
    // 95% confidence (2-sigma)
    double sigma_v = std::sqrt(position_covariance(2,2));
    return 2.0 * sigma_v;
}

double PositionSolution::get3DAccuracy() const {
    // 95% confidence
    double sigma_3d = std::sqrt(position_covariance(0,0) + position_covariance(1,1) + position_covariance(2,2));
    return 2.0 * sigma_3d;
}

std::string PositionSolution::toString() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << "Time: " << time.week << " " << time.tow << "\n";
    oss << "Position (ECEF): " << position_ecef.transpose() << "\n";
    oss << "Position (Geodetic): " << position_geodetic.latitude * 180.0/M_PI 
        << " " << position_geodetic.longitude * 180.0/M_PI 
        << " " << position_geodetic.height << "\n";
    oss << "Status: " << static_cast<int>(status) << "\n";
    oss << "Satellites: " << num_satellites << "\n";
    oss << "PDOP: " << pdop << "\n";
    return oss.str();
}

std::string PositionSolution::toNMEA() const {
    // Simple GGA message implementation
    std::ostringstream oss;
    oss << "$GPGGA,";
    // Time would be formatted here
    oss << "000000.00,";
    // Latitude/longitude formatting would be here
    oss << "0000.0000,N,00000.0000,E,";
    oss << (isValid() ? "1" : "0") << ",";
    oss << std::setfill('0') << std::setw(2) << num_satellites << ",";
    oss << std::fixed << std::setprecision(1) << hdop << ",";
    oss << std::fixed << std::setprecision(1) << position_geodetic.height << ",M,0.0,M,,";
    oss << "*00\r\n"; // Checksum would be calculated
    return oss.str();
}

Solution::SolutionStatistics Solution::calculateStatistics(const Vector3d& reference_position) const {
    SolutionStatistics stats;
    
    if (solutions.empty()) {
        return stats;
    }
    
    stats.total_epochs = solutions.size();
    
    double sum_hdop = 0.0, sum_vdop = 0.0, sum_pdop = 0.0;
    double sum_processing_time = 0.0, sum_satellites = 0.0;
    
    for (const auto& sol : solutions) {
        if (sol.isValid()) {
            stats.valid_solutions++;
            if (sol.isFixed()) {
                stats.fixed_solutions++;
            } else if (sol.status == SolutionStatus::FLOAT) {
                stats.float_solutions++;
            }
            
            sum_hdop += sol.hdop;
            sum_vdop += sol.vdop;
            sum_pdop += sol.pdop;
            sum_processing_time += sol.processing_time_ms;
            sum_satellites += sol.num_satellites;
        }
    }
    
    if (stats.valid_solutions > 0) {
        stats.availability_rate = static_cast<double>(stats.valid_solutions) / stats.total_epochs;
        stats.fix_rate = static_cast<double>(stats.fixed_solutions) / stats.valid_solutions;
        
        stats.mean_hdop = sum_hdop / stats.valid_solutions;
        stats.mean_vdop = sum_vdop / stats.valid_solutions;
        stats.mean_pdop = sum_pdop / stats.valid_solutions;
        stats.mean_processing_time = sum_processing_time / stats.valid_solutions;
        stats.mean_satellites = sum_satellites / stats.valid_solutions;
        
        // Calculate RMS errors if reference position is provided
        if (reference_position.norm() > 0) {
            double sum_h_error_sq = 0.0, sum_v_error_sq = 0.0, sum_3d_error_sq = 0.0;
            
            for (const auto& sol : solutions) {
                if (sol.isValid()) {
                    Vector3d error = sol.position_ecef - reference_position;
                    sum_h_error_sq += error(0)*error(0) + error(1)*error(1);
                    sum_v_error_sq += error(2)*error(2);
                    sum_3d_error_sq += error.squaredNorm();
                }
            }
            
            stats.rms_horizontal = std::sqrt(sum_h_error_sq / stats.valid_solutions);
            stats.rms_vertical = std::sqrt(sum_v_error_sq / stats.valid_solutions);
            stats.rms_3d = std::sqrt(sum_3d_error_sq / stats.valid_solutions);
        }
    }
    
    return stats;
}

bool Solution::writeToFile(const std::string& filename, const std::string& format) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    // Write header
    file << "% LibGNSS++ Position Solution\n";
    file << "% Format: " << format << "\n";
    file << "% Columns: GPS_Week GPS_TOW X(m) Y(m) Z(m) Lat(deg) Lon(deg) Height(m) Status Satellites PDOP Ratio FixedAmbiguities Iterations\n";
    
    for (const auto& sol : solutions) {
        file << std::fixed << std::setprecision(6);
        file << sol.time.week << " " << sol.time.tow << " ";
        file << sol.position_ecef(0) << " " << sol.position_ecef(1) << " " << sol.position_ecef(2) << " ";
        file << sol.position_geodetic.latitude * 180.0/M_PI << " ";
        file << sol.position_geodetic.longitude * 180.0/M_PI << " ";
        file << sol.position_geodetic.height << " ";
        file << static_cast<int>(sol.status) << " ";
        file << sol.num_satellites << " ";
        file << sol.pdop << " ";
        file << sol.ratio << " ";
        file << sol.num_fixed_ambiguities << " ";
        file << sol.iterations << "\n";
    }
    
    return true;
}

bool Solution::writeKML(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    file << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n";
    file << "<Document>\n";
    file << "<name>LibGNSS++ Solution</name>\n";
    file << "<Placemark>\n";
    file << "<name>GNSS Track</name>\n";
    file << "<LineString>\n";
    file << "<coordinates>\n";
    
    for (const auto& sol : solutions) {
        if (sol.isValid()) {
            file << std::fixed << std::setprecision(8);
            file << sol.position_geodetic.longitude * 180.0/M_PI << ",";
            file << sol.position_geodetic.latitude * 180.0/M_PI << ",";
            file << sol.position_geodetic.height << "\n";
        }
    }
    
    file << "</coordinates>\n";
    file << "</LineString>\n";
    file << "</Placemark>\n";
    file << "</Document>\n";
    file << "</kml>\n";
    
    return true;
}

bool Solution::writeNMEA(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    for (const auto& sol : solutions) {
        file << sol.toNMEA();
    }
    
    return true;
}

bool Solution::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        return false;
    }

    solutions.clear();

    std::string line;
    std::vector<std::string> columns;
    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }

        if (line[0] == '%') {
            std::vector<std::string> parsed_columns = parseSolutionColumnsHeader(line);
            if (!parsed_columns.empty()) {
                columns = std::move(parsed_columns);
            }
            continue;
        }

        if (line.find('/') != std::string::npos) {
            PositionSolution rtklib_solution;
            if (parseRtklibSolutionLine(line, rtklib_solution)) {
                solutions.push_back(rtklib_solution);
            }
            continue;
        }

        std::istringstream iss(line);
        PositionSolution sol;
        double lat_deg = 0.0;
        double lon_deg = 0.0;
        int status = 0;

        if (!(iss >> sol.time.week >> sol.time.tow
                  >> sol.position_ecef(0) >> sol.position_ecef(1) >> sol.position_ecef(2)
                  >> lat_deg >> lon_deg >> sol.position_geodetic.height
                  >> status >> sol.num_satellites >> sol.pdop)) {
            continue;
        }

        std::vector<double> optional_values;
        double optional_value = 0.0;
        while (iss >> optional_value) {
            optional_values.push_back(optional_value);
        }
        applyOptionalSolutionColumns(sol, columns, optional_values);

        sol.position_geodetic.latitude = lat_deg * M_PI / 180.0;
        sol.position_geodetic.longitude = lon_deg * M_PI / 180.0;
        sol.status = static_cast<SolutionStatus>(status);
        solutions.push_back(sol);
    }

    return !solutions.empty();
}

const PositionSolution* Solution::getSolution(const GNSSTime& time) const {
    for (const auto& sol : solutions) {
        if (sol.time == time) {
            return &sol;
        }
    }
    return nullptr;
}

std::vector<PositionSolution> Solution::filterByStatus(SolutionStatus status) const {
    std::vector<PositionSolution> filtered;
    filtered.reserve(solutions.size());
    for (const auto& sol : solutions) {
        if (sol.status == status) {
            filtered.push_back(sol);
        }
    }
    return filtered;
}

void Solution::sortByTime() {
    std::sort(solutions.begin(), solutions.end(), 
              [](const PositionSolution& a, const PositionSolution& b) {
                  return a.time < b.time;
              });
}

std::pair<GNSSTime, GNSSTime> Solution::getTimeSpan() const {
    if (solutions.empty()) {
        return {GNSSTime(), GNSSTime()};
    }
    
    auto minmax = std::minmax_element(solutions.begin(), solutions.end(),
                                     [](const PositionSolution& a, const PositionSolution& b) {
                                         return a.time < b.time;
                                     });
    
    return {minmax.first->time, minmax.second->time};
}

} // namespace libgnss
